//! `/dev/audio0` — RK3588 I2S/TDM microphone capture node (PIO drain).
//!
//! This is the thin StarryOS device-node adapter over the portable
//! [`audio_capture`] runtime. It FDT-discovers the `rockchip,rk3588-i2s-tdm`
//! controller, maps its register block, and drives capture in PIO mode: the RX
//! FIFO-ready interrupt wakes the handler, which drains the FIFO through
//! [`I2sTdmController::read_fifo`] into [`AudioCaptureStream::ingest`] and wakes
//! any poll waiter. The read path drains published mono samples through
//! [`AudioCaptureStream::read`]. It mirrors the `/dev/input/event*` device: an
//! IRQ-safe lock around the hardware/runtime state plus a [`PollSet`] woken from
//! IRQ context.
//!
//! DMA capture is deliberately not used here; the FIFO is drained by the CPU.
//! On a platform without the controller in its FDT (e.g. QEMU) [`AudioCaptureDev::probe`]
//! returns `None` and the node is simply absent.

use alloc::sync::Arc;
use core::{any::Any, task::Context};

use audio_capture::{AudioCaptureStream, MonoSource, OutputFormat, ReadOutcome};
use ax_errno::AxError;
use ax_memory_addr::PhysAddr;
use ax_runtime::hal::irq::{self, AutoEnable, IrqHandle, IrqId, IrqRequest, IrqReturn, ShareMode};
use ax_sync::spin::SpinNoIrq as Mutex;
use axfs_ng_vfs::{DeviceId, NodeFlags, VfsResult};
use axpoll::{IoEvents, PollSet, Pollable};
use mmio_api::{MmioAddr, MmioRaw};
use rockchip_i2s_tdm::{
    CaptureFormat, ClockDividers, I2sTdmController, IrqEvent, MmioRegisters,
    RK3588_I2S_TDM_REGISTER_SIZE,
};

use crate::pseudofs::DeviceOps;

/// `/dev/audio0` device number. 241 is in the local/experimental major range
/// (240 is already taken by the KPU node), minor 0 for the single controller.
pub const AUDIO_DEVICE_ID: DeviceId = DeviceId::new(241, 0);

/// Published PCM ring capacity in mono samples (~0.34 s at 48 kHz). A 512 Ki
/// bump (1 MiB inline `[i16; N]` in the device struct) HUNG the board at
/// pseudofs init on 2026-08-30 -- the allocation happens inside device
/// registration and does not complete at that size on the current heap.
/// Before re-bumping for --live (an ASR pass idles the reader for 5-6 s),
/// move the ring to an explicit heap allocation and debug the boot in QEMU.
const AUDIO_RING_SAMPLES: usize = 16 * 1024;

/// Scratch depth for a single FIFO drain. The RX FIFO level field is 6 bits
/// (`RXFIFOLR & 0x3f`, max 63), so a 64-sample buffer empties it in one pass.
const FIFO_DRAIN_SAMPLES: usize = 64;

/// The controller always captures interleaved stereo S16 at 48 kHz.
const HARDWARE_FORMAT: CaptureFormat = CaptureFormat::STEREO_S16_48K;

/// `/dev/audio0` publishes logical mono taken from the left slot; mono
/// extraction lives in the runtime, not the register core.
const OUTPUT_FORMAT: OutputFormat = OutputFormat::Mono(MonoSource::Left);

/// PIO capture clock ratios, reproducing the exact I2S clocking a working Linux
/// capture leaves on this board.
///
/// Clock chain CONFIRMED on the board (2026-08-26, Orange Pi 5 Plus, Linux
/// 6.1.43 active 48 kHz S16_LE stereo capture on `hw:3,0`):
///   - RK3588 is I2S master, ES8388 is slave (ES8388 MASTERMODE reg 0x08 = 0x00;
///     `i2s0_8ch_mclkout` enabled → SoC generates BCLK/LRCK/MCLK).
///   - `rockchip,mclk-fs = 256`; MCLK = 256 × 48000 = 12.288 MHz, sourced from
///     pll_aupll 786.432 MHz → /2 → frac ÷32 → mclk_i2s0_8ch_rx 12.288 MHz;
///     hclk_i2s0_8ch = 198 MHz.
///   - `bclk_div = 4` → BCLK = MCLK / 4 = 3.072 MHz (CLKDIV.{TXM,RXM}).
///   - `lrck_div = 64` → LRCK = BCLK / 64 = 48 kHz, two 32-bit slots
///     (CKR.{TSD,RSD}).
///
/// The core composes these into the Linux ground-truth words the board runs
/// under an active `arecord`: CKR = `0x10003f3f` (TRCM_TX + master + RSD/TSD 64),
/// CLKDIV = `0x0303`, TXCR = `0x7200000f`, RXCR = `0x01c8000f`. TX is programmed
/// and started alongside RX because the shared BCLK/LRCK is generated on the TX
/// timing engine in the board's `rockchip,trcm-sync-tx-only` mode. This path only
/// runs on the real board; QEMU has no controller node so it is never reached.
const CAPTURE_CLOCK: ClockDividers = ClockDividers {
    bclk_div: 4,
    lrck_div: 64,
};

/// Hardware controller plus the runtime it feeds. Both are mutated from the read
/// path and the IRQ path, so they share one IRQ-safe lock.
struct Inner {
    controller: I2sTdmController<MmioRegisters>,
    stream: AudioCaptureStream<AUDIO_RING_SAMPLES>,
}

/// `/dev/audio0` capture device: RK3588 I2S/TDM in PIO mode behind the portable
/// capture runtime. Modelled on the `/dev/input/event*` `EventDev`.
pub struct AudioCaptureDev {
    inner: Mutex<Inner>,
    waiters: PollSet,
    irq: Option<IrqId>,
    irq_handle: spin::Once<IrqHandle>,
}

impl AudioCaptureDev {
    /// FDT-discover the controller, map its register block, and build the
    /// stopped capture device. Returns `None` when the controller is absent or
    /// unmappable so a platform without it (QEMU) still boots.
    pub fn probe() -> Option<Self> {
        let resource = AudioResource::from_fdt()?;
        let base_vaddr =
            match axklib::mem::iomap(PhysAddr::from(resource.base_paddr), resource.size) {
                Ok(base) => base.as_usize(),
                Err(err) => {
                    warn!(
                        "rk3588-audio: failed to map I2S/TDM MMIO at {:#x}+{:#x}: {err:?}",
                        resource.base_paddr, resource.size
                    );
                    return None;
                }
            };
        // SAFETY: `iomap` returned a valid, aligned mapping of `resource.size`
        // bytes covering the whole RK3588 I2S/TDM register file. The controller
        // owns this mapping for its lifetime (the node is never dropped), and
        // the only accessor is serialized behind `inner`.
        let raw = unsafe {
            MmioRaw::new(
                MmioAddr::from(resource.base_paddr),
                core::ptr::NonNull::new(base_vaddr as *mut u8)
                    .expect("rk3588-audio: iomap returned a null base"),
                resource.size,
            )
        };
        let controller =
            match I2sTdmController::from_mmio(raw, HARDWARE_FORMAT) {
                Ok(controller) => controller,
                Err(err) => {
                    warn!("rk3588-audio: unsupported capture format: {err:?}");
                    return None;
                }
            };
        info!(
            "rk3588-audio: /dev/audio0 base={:#x} size={:#x} irq={:?} publishing {:?}",
            resource.base_paddr, resource.size, resource.irq, OUTPUT_FORMAT
        );
        Some(Self {
            inner: Mutex::new(Inner {
                controller,
                stream: AudioCaptureStream::new(HARDWARE_FORMAT, OUTPUT_FORMAT),
            }),
            waiters: PollSet::new(),
            irq: resource.irq,
            irq_handle: spin::Once::new(),
        })
    }

    /// Register and enable the RX interrupt handler at the GIC. The controller's
    /// own interrupt sources are enabled later, in [`open`](DeviceOps::open),
    /// so no interrupt fires until capture actually starts.
    pub fn register_irq(self: &Arc<Self>) {
        let Some(irq) = self.irq else {
            warn!("rk3588-audio: no IRQ resolved; /dev/audio0 cannot deliver capture");
            return;
        };
        let dev = Arc::clone(self);
        let request = IrqRequest::new(move |_| dev.handle_irq())
            .share_mode(ShareMode::Shared)
            .auto_enable(AutoEnable::No);
        match irq::request_irq(irq, request) {
            Ok(handle) => {
                self.irq_handle.call_once(|| handle);
                if let Some(handle) = self.irq_handle.get().copied()
                    && let Err(err) = irq::enable_irq(handle)
                {
                    warn!("rk3588-audio: failed to enable IRQ {irq:?}: {err:?}");
                }
            }
            Err(err) => warn!("rk3588-audio: failed to register IRQ {irq:?}: {err:?}"),
        }
    }

    /// RX interrupt: on a FIFO-ready event drain the FIFO into the runtime and
    /// wake poll waiters; on overflow drop the stale FIFO contents (the core has
    /// already acknowledged the overflow bit).
    fn handle_irq(&self) -> IrqReturn {
        // `lock()` (not `try_lock`) so a shared, level-triggered line is always
        // serviced: `SpinNoIrq` disables local IRQs for the holder, so this can
        // only contend across CPUs.
        let mut inner = self.inner.lock();
        match inner.controller.handle_irq() {
            IrqEvent::RxReady => {
                let mut scratch = [0i16; FIFO_DRAIN_SAMPLES];
                // Drain until the FIFO is below one scratch buffer, clearing the
                // level interrupt before we return.
                loop {
                    let count = inner.controller.read_fifo(&mut scratch);
                    if count == 0 {
                        break;
                    }
                    inner.stream.ingest(&scratch[..count]);
                    if count < scratch.len() {
                        break;
                    }
                }
                drop(inner);
                self.waiters.wake_from_irq(IoEvents::IN);
                IrqReturn::Wake
            }
            IrqEvent::RxOverflow => {
                let mut scratch = [0i16; FIFO_DRAIN_SAMPLES];
                while inner.controller.read_fifo(&mut scratch) == scratch.len() {}
                IrqReturn::Handled
            }
            IrqEvent::None => IrqReturn::Unhandled,
        }
    }
}

impl DeviceOps for AudioCaptureDev {
    fn read_at(&self, buf: &mut [u8], _offset: u64) -> VfsResult<usize> {
        if buf.is_empty() {
            return Ok(0);
        }
        match self.inner.lock().stream.read(buf) {
            ReadOutcome::Filled(read) => Ok(read),
            ReadOutcome::WouldBlock => Err(AxError::WouldBlock),
            ReadOutcome::Inactive => Err(AxError::BadState),
        }
    }

    fn write_at(&self, _buf: &[u8], _offset: u64) -> VfsResult<usize> {
        // Capture is read-only.
        Err(AxError::InvalidInput)
    }

    fn as_any(&self) -> &dyn Any {
        self
    }

    fn as_pollable(&self) -> Option<&dyn Pollable> {
        Some(self)
    }

    fn flags(&self) -> NodeFlags {
        NodeFlags::NON_CACHEABLE | NodeFlags::STREAM
    }

    /// Program the PIO RX path and start capture. `configure_capture_pio` enables
    /// the FIFO-ready/overflow interrupt sources (the GIC handler was wired at
    /// boot); `start` clears stale samples and `start_capture` begins the RX
    /// transfer. Reprogramming on each open is safe because `close` stops the
    /// transfer.
    fn open(&self, _exclusive: bool) -> VfsResult<()> {
        let mut inner = self.inner.lock();
        if let Err(err) = inner.controller.configure_capture_pio(CAPTURE_CLOCK) {
            warn!("rk3588-audio: capture configuration rejected: {err:?}");
            return Err(AxError::InvalidInput);
        }
        inner.stream.start();
        // Start the RX transfer with the vendor RK3588 clock-discontinuity: gate
        // `mclk_i2s0_8ch_tx` off, write `XFER = TXS|RXS` while gated, gate back on.
        // In `trcm-sync-tx-only` the shared BCLK/LRCK generator only kicks on that
        // edge — arming XFER against a free-running mclk leaves rx_fifo_level=0.
        {
            let controller = &mut inner.controller;
            super::audio_clk::start_i2s0_capture_gate_cycle(|| controller.start_capture());
        }
        Ok(())
    }

    /// Stop the RX transfer and the runtime when the last reader closes.
    fn close(&self, _exclusive: bool) {
        let mut inner = self.inner.lock();
        inner.controller.stop_capture();
        inner.stream.stop();
    }
}

impl Pollable for AudioCaptureDev {
    fn poll(&self) -> IoEvents {
        let mut events = IoEvents::empty();
        events.set(IoEvents::IN, self.inner.lock().stream.poll().readable);
        events
    }

    fn register(&self, context: &mut Context<'_>, events: IoEvents) {
        if !events.contains(IoEvents::IN) {
            return;
        }
        // SAFETY: mirrors `EventDev` — register the waker for `IN` readiness on
        // the shared `PollSet`; the RX IRQ path wakes it via `wake_from_irq`.
        unsafe { self.waiters.register(context.waker(), IoEvents::IN) };
        if self.inner.lock().stream.poll().readable {
            context.waker().wake_by_ref();
        }
    }
}

/// Physical base of the RK3588 I2S/TDM controller wired to the ES8388 codec on
/// the Orange Pi 5 Plus. The board `rockchip,multicodecs-card` / ES8388 sound
/// card points its `rockchip,cpu` DAI at `i2s@fe470000`. Several other
/// `rockchip,rk3588-i2s-tdm` nodes (fddf0000/fddf4000/fddf8000) are also
/// `status = "okay"` in the FDT but are NOT on the microphone capture path;
/// without this filter the probe binds the first enabled node (fddf0000) and
/// capture never receives data. Board ground truth confirmed on the running
/// image (2026-08-27): binding fddf0000 → `open` succeeds but `read` blocks
/// forever because no RX FIFO data arrives.
///
/// `pub(super)` so the sibling `audio_codec` pinmux glue can filter the same
/// controller node when applying its `pinctrl-0` audio pads.
pub(super) const ES8388_I2S_BASE: usize = 0xfe47_0000;

/// FDT-resolved location of the RK3588 I2S/TDM controller.
struct AudioResource {
    base_paddr: usize,
    size: usize,
    irq: Option<IrqId>,
}

impl AudioResource {
    fn from_fdt() -> Option<Self> {
        rdrive::with_fdt(|fdt| {
            fdt.find_compatible(&["rockchip,rk3588-i2s-tdm"])
                .into_iter()
                .find_map(Self::from_fdt_node)
        })
        .flatten()
    }

    fn from_fdt_node(node: rdrive::probe::fdt::NodeType<'_>) -> Option<Self> {
        if matches!(
            node.as_node().status(),
            Some(rdrive::probe::fdt::Status::Disabled)
        ) {
            return None;
        }
        let reg = node.regs().into_iter().next()?;
        // Bind only the ES8388-connected controller (fe470000); skip the other
        // enabled i2s-tdm nodes so `find_map` continues past them. See
        // [`ES8388_I2S_BASE`].
        if reg.address as usize != ES8388_I2S_BASE {
            return None;
        }
        let irq = match decode_fdt_irq(&node.interrupts()) {
            Ok(irq) => irq,
            Err(err) => {
                warn!("rk3588-audio: failed to resolve I2S/TDM IRQ: {err:?}");
                return None;
            }
        };
        Some(Self {
            base_paddr: reg.address as usize,
            size: reg
                .size
                .map(|size| size as usize)
                .unwrap_or(RK3588_I2S_TDM_REGISTER_SIZE),
            irq,
        })
    }
}

/// Resolve the first FDT interrupt of `node` to a kernel [`IrqId`] via its
/// interrupt controller. Mirrors the KPU node's decoder.
fn decode_fdt_irq(
    interrupts: &[rdrive::probe::fdt::InterruptRef],
) -> Result<Option<IrqId>, irq::IrqError> {
    let Some(interrupt) = interrupts.first() else {
        return Ok(None);
    };
    let controller = rdrive::fdt_phandle_to_device_id(interrupt.interrupt_parent)
        .ok_or(irq::IrqError::Unsupported)?;
    ax_runtime::irq::resolve_binding_irq(ax_driver::BindingIrq::fdt_interrupt_with_controller(
        controller,
        interrupt.specifier.clone(),
    ))
    .map(Some)
}
