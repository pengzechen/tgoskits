#![no_std]

//! Portable hardware core for the RK3588 I2S/TDM controller used by the
//! Orange Pi 5 Plus ES8388 capture path.
//!
//! MMIO mapping, clock/reset/pinctrl setup, DMA allocation, IRQ registration,
//! and StarryOS VFS integration intentionally remain outside this crate.

use mmio_api::MmioRaw;
use thiserror::Error;

pub const RK3588_I2S_TDM_BASE: usize = 0xfe47_0000;
pub const RK3588_I2S_TDM_IRQ: u32 = 0xb4;
pub const RK3588_I2S_TDM_REGISTER_SIZE: usize = 0x1000;

const TXCR: usize = 0x000;
const RXCR: usize = 0x004;
const CKR: usize = 0x008;
const DMACR: usize = 0x010;
const INTCR: usize = 0x014;
const INTSR: usize = 0x018;
const XFER: usize = 0x01c;
const CLR: usize = 0x020;
const RXDR: usize = 0x028;
const RXFIFOLR: usize = 0x02c;
const TDM_TXCR: usize = 0x030;
const TDM_RXCR: usize = 0x034;
const CLKDIV: usize = 0x038;

// XFER transfer-start bits. In the board's TX-common-clock mode (see
// `CKR_TRCM_TX`) the shared BCLK/LRCK is generated on the TX timing engine, so
// RX only clocks in data when TX is started too — capture must set BOTH.
const XFER_TXS_START: u32 = 1 << 0;
const XFER_RXS_START: u32 = 1 << 1;

/// RX FIFO level field in `RXFIFOLR` (samples currently buffered on the lane).
/// The exact width is confirmed against the board TRM during gate 4; a 6-bit
/// field covers the RK3588 32-entry RX FIFO.
const RXFIFOLR_LEVEL_MASK: u32 = 0x3f;

// TXCR/RXCR valid-data-width field (`VDW`, encoded `bits - 1`) and the RXCR
// channel-select (`CSR`) for two interleaved slots (stereo). From Linux
// `rockchip_i2s_tdm.h`.
const RXCR_VDW_16_BIT: u32 = 15;
const RXCR_VDW_24_BIT: u32 = 23;
const RXCR_CSR_TWO_SLOTS: u32 = 0 << 15;

// TXCR/RXCR high bits: the reset-default lane→path routing (path0..3 = 0,1,2,3)
// plus I2S mode. Written explicitly on every configure so a different framing
// left by a prior (Linux) boot cannot leak into StarryOS capture. These match
// the Linux `reg_default` high bits (TXCR 0x72000000, RXCR 0x01c80000); OR in
// the format's VDW/CSR to get the full word (16-bit → 0x7200000f / 0x01c8000f).
const TXCR_PATH_DEFAULT: u32 = 0x7200_0000;
const RXCR_PATH_DEFAULT: u32 = 0x01c8_0000;

// TDM control registers are unused in plain I2S mode; write their reset default
// (Linux `reg_default` 0x00003eff) so they are deterministic under capture.
const TDM_CTRL_DEFAULT: u32 = 0x0000_3eff;

// CKR (clock) fields, from Linux `rockchip_i2s_tdm.h`.
// TRCM = TX common mode (bit 28): RX shares the frame/bit clock generated on the
// TX engine. The Orange Pi 5 Plus sound card declares `rockchip,trcm-sync-tx-only`,
// so the SoC runs in this mode. `MSS_MASTER` (0) = RK3588 generates BCLK/LRCK.
// `RSD`/`TSD` are the SCLK-per-frame (BCLK→LRCK) dividers, encoded `n - 1`.
const CKR_TRCM_TX: u32 = 1 << 28;
const CKR_MSS_MASTER: u32 = 0 << 27;
const CKR_RSD_SHIFT: u32 = 8;
const CKR_TSD_SHIFT: u32 = 0;

// CLKDIV (MCLK→BCLK) TX/RX divider fields, encoded `n - 1`.
const CLKDIV_TXM_SHIFT: u32 = 0;
const CLKDIV_RXM_SHIFT: u32 = 8;

const INT_RX_READY: u32 = 1 << 16;
const INT_RX_OVERFLOW: u32 = 1 << 17;
const INT_RX_OVERFLOW_CLEAR: u32 = 1 << 18;
const DMACR_RX_ENABLE: u32 = 1 << 24;
const DMACR_RX_LEVEL: u32 = 16 << 16;
const INT_RX_THRESHOLD: u32 = 16 << 20;
const INT_RX_ENABLE: u32 = 1 << 16;
const INT_RX_OVERFLOW_ENABLE: u32 = 1 << 17;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Error)]
pub enum AudioConfigError {
    #[error("the RK3588 capture path only supports interleaved stereo")]
    UnsupportedChannels,
    #[error("only 16-bit and 24-bit PCM capture is supported")]
    UnsupportedSampleWidth,
    #[error("only 16 kHz and 48 kHz capture are supported")]
    UnsupportedSampleRate,
    #[error("I2S clock dividers must be non-zero")]
    InvalidClockDividers,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CaptureFormat {
    pub sample_rate_hz: u32,
    pub channels: u8,
    pub sample_width_bits: u8,
}

/// I2S bit/frame clock ratios from MCLK. The board runs MCLK = 12.288 MHz
/// (mclk-fs = 256 @ 48 kHz). `bclk_div` is MCLK→BCLK (4 → 3.072 MHz SCLK);
/// `lrck_div` is BCLK→LRCK, i.e. SCLK-per-frame (64 = two 32-bit slots → 48 kHz).
///
/// These map onto two distinct RK3588 registers: `bclk_div` drives
/// `CLKDIV.{TXM,RXM}` (MCLK divider) and `lrck_div` drives `CKR.{TSD,RSD}`
/// (SCLK-per-frame). Both hardware fields are encoded `n - 1`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ClockDividers {
    pub bclk_div: u8,
    pub lrck_div: u16,
}

impl ClockDividers {
    pub const fn validate(self) -> bool {
        self.bclk_div != 0 && self.lrck_div != 0
    }

    /// `CKR` word: TX-common-mode (shared clock on the TX engine, matching the
    /// board's `rockchip,trcm-sync-tx-only`), RK3588 as clock master, and the
    /// SCLK-per-frame divider replicated into both TSD and RSD. For the 48 kHz
    /// stereo format this yields the Linux ground-truth `0x10003f3f`.
    const fn ckr_value(self) -> u32 {
        let sd = (self.lrck_div - 1) as u32;
        CKR_TRCM_TX | CKR_MSS_MASTER | (sd << CKR_RSD_SHIFT) | (sd << CKR_TSD_SHIFT)
    }

    /// `CLKDIV` word: the MCLK→BCLK divider replicated into TXM and RXM. For the
    /// 48 kHz stereo format (bclk_div = 4) this yields the Linux ground-truth
    /// `0x0303`.
    const fn clkdiv_value(self) -> u32 {
        let m = (self.bclk_div - 1) as u32;
        (m << CLKDIV_TXM_SHIFT) | (m << CLKDIV_RXM_SHIFT)
    }
}

impl CaptureFormat {
    /// The board's DAI forces interleaved stereo; this is the native RX shape and
    /// the format the runtime downmixes to logical mono for consumers.
    pub const STEREO_S16_48K: Self = Self {
        sample_rate_hz: 48_000,
        channels: 2,
        sample_width_bits: 16,
    };

    /// The wider capture width the hardware accepts, still interleaved stereo.
    pub const STEREO_S24_48K: Self = Self {
        sample_rate_hz: 48_000,
        channels: 2,
        sample_width_bits: 24,
    };

    pub const fn validate(self) -> Result<Self, AudioConfigError> {
        if self.channels != 2 {
            return Err(AudioConfigError::UnsupportedChannels);
        }
        if self.sample_width_bits != 16 && self.sample_width_bits != 24 {
            return Err(AudioConfigError::UnsupportedSampleWidth);
        }
        if self.sample_rate_hz != 16_000 && self.sample_rate_hz != 48_000 {
            return Err(AudioConfigError::UnsupportedSampleRate);
        }
        Ok(self)
    }

    /// Valid-data-width field for this format. The controller encodes it as
    /// `width - 1` (16-bit → 15, 24-bit → 23); only the two widths accepted by
    /// [`validate`](Self::validate) are reachable here. Shared by TXCR and RXCR.
    const fn valid_data_width(self) -> u32 {
        match self.sample_width_bits {
            24 => RXCR_VDW_24_BIT,
            _ => RXCR_VDW_16_BIT,
        }
    }

    /// `TXCR` word: the reset-default lane routing / I2S mode plus this format's
    /// valid-data width. TX is configured (not just RX) because the shared clock
    /// runs on the TX engine in TX-common mode. 16-bit → `0x7200000f`.
    const fn txcr_value(self) -> u32 {
        TXCR_PATH_DEFAULT | self.valid_data_width()
    }

    /// `RXCR` word: reset-default routing / I2S mode, two interleaved slots
    /// (stereo), plus this format's valid-data width. 16-bit → `0x01c8000f`.
    const fn rxcr_value(self) -> u32 {
        RXCR_PATH_DEFAULT | RXCR_CSR_TWO_SLOTS | self.valid_data_width()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IrqEvent {
    None,
    RxReady,
    RxOverflow,
}

/// Access to the controller register file. Hardware uses [`MmioRegisters`];
/// tests supply a recording bank so the register sequence and interrupt
/// acknowledgement can be verified without a board.
pub trait RegisterBank {
    fn read(&self, offset: usize) -> u32;
    fn write(&mut self, offset: usize, value: u32);
}

/// Volatile access to a caller-mapped register file. OS glue constructs the
/// [`MmioRaw`] over its `iomap` result; offsets are byte addresses of whole
/// `u32` register words within the RK3588 register file.
pub struct MmioRegisters {
    raw: MmioRaw,
}

impl RegisterBank for MmioRegisters {
    fn read(&self, offset: usize) -> u32 {
        self.raw.read::<u32>(offset)
    }

    fn write(&mut self, offset: usize, value: u32) {
        self.raw.write::<u32>(offset, value)
    }
}

/// Register-level controller. The register bank owns the MMIO mapping for its
/// whole lifetime and task-context access must be serialized against IRQ code.
pub struct I2sTdmController<R: RegisterBank = MmioRegisters> {
    regs: R,
    format: CaptureFormat,
}

impl I2sTdmController<MmioRegisters> {
    /// Build a controller over a glue-mapped register file. The mapping must
    /// cover [`RK3588_I2S_TDM_REGISTER_SIZE`] bytes and remain valid for this
    /// controller's lifetime; constructing [`MmioRaw`] over the `iomap` result
    /// is OS glue's unsafe edge, not this crate's.
    pub fn from_mmio(raw: MmioRaw, format: CaptureFormat) -> Result<Self, AudioConfigError> {
        Ok(Self {
            regs: MmioRegisters { raw },
            format: format.validate()?,
        })
    }
}

impl<R: RegisterBank> I2sTdmController<R> {
    /// Build a controller over an arbitrary register bank. The bank must map the
    /// RK3588 register file; this is the seam mock-MMIO tests use.
    pub fn with_registers(regs: R, format: CaptureFormat) -> Result<Self, AudioConfigError> {
        Ok(Self {
            regs,
            format: format.validate()?,
        })
    }

    pub const fn format(&self) -> CaptureFormat {
        self.format
    }

    /// DMA capture: the RX FIFO is drained by an external DMA controller. The
    /// FIFO-ready threshold routes to DMA (`DMACR.RDE`) while the FIFO-ready and
    /// overflow interrupts stay enabled for status.
    pub fn configure_capture(&mut self, clock: ClockDividers) -> Result<(), AudioConfigError> {
        self.configure_rx_common(clock)?;
        self.write(DMACR, DMACR_RX_ENABLE | DMACR_RX_LEVEL);
        self.write(
            INTCR,
            INT_RX_THRESHOLD | INT_RX_ENABLE | INT_RX_OVERFLOW_ENABLE,
        );
        self.write(CLR, 1 << 1);
        Ok(())
    }

    /// PIO capture: no DMA controller is involved. `DMACR` is explicitly cleared
    /// so a stale `RDE` left by a prior (Linux) boot cannot route the FIFO to a
    /// DMA engine StarryOS does not drive and starve the CPU drain. The RX
    /// FIFO-ready interrupt signals task context to drain the FIFO through
    /// [`read_fifo`](Self::read_fifo); overflow is reported the same way as the
    /// DMA path.
    pub fn configure_capture_pio(&mut self, clock: ClockDividers) -> Result<(), AudioConfigError> {
        self.configure_rx_common(clock)?;
        self.write(DMACR, 0);
        self.write(
            INTCR,
            INT_RX_THRESHOLD | INT_RX_ENABLE | INT_RX_OVERFLOW_ENABLE,
        );
        self.write(CLR, 1 << 1);
        Ok(())
    }

    /// Register writes shared by the DMA and PIO capture paths. Programs the full
    /// ground-truth I2S RX framing so the RX FIFO advances: TX and RX lane/format
    /// (TX is configured because the shared clock runs on the TX engine in
    /// TX-common mode), the CKR clock word (TRCM_TX + master + SCLK-per-frame),
    /// the TDM control registers (reset default — unused in plain I2S), and the
    /// MCLK→BCLK divider. The data-path-specific `DMACR`/`INTCR`/`CLR` writes
    /// follow in the caller.
    fn configure_rx_common(&mut self, clock: ClockDividers) -> Result<(), AudioConfigError> {
        if !clock.validate() {
            return Err(AudioConfigError::InvalidClockDividers);
        }
        self.write(TXCR, self.format.txcr_value());
        self.write(RXCR, self.format.rxcr_value());
        self.write(CKR, clock.ckr_value());
        self.write(TDM_TXCR, TDM_CTRL_DEFAULT);
        self.write(TDM_RXCR, TDM_CTRL_DEFAULT);
        self.write(CLKDIV, clock.clkdiv_value());
        Ok(())
    }

    /// PIO drain: pop up to `out.len()` samples from the RX FIFO into `out`,
    /// returning how many were read. Task context calls this after an
    /// [`IrqEvent::RxReady`]; draining the FIFO clears the level interrupt. The
    /// current FIFO depth is read from `RXFIFOLR` and that many samples (bounded
    /// by `out`) are popped from the RX data register, each read yielding one
    /// 16-bit sample in the low half of the 32-bit register. The exact
    /// stereo-S16 FIFO packing is confirmed on the board in gate 4.
    pub fn read_fifo(&mut self, out: &mut [i16]) -> usize {
        let level = (self.read(RXFIFOLR) & RXFIFOLR_LEVEL_MASK) as usize;
        let count = level.min(out.len());
        for slot in out.iter_mut().take(count) {
            *slot = self.read(RXDR) as i16;
        }
        count
    }

    /// Start capture. In TX-common mode the shared BCLK/LRCK is generated on the
    /// TX timing engine, so RX only receives data when TX is started too — both
    /// `TXS` and `RXS` are set (XFER = 0x3).
    pub fn start_capture(&mut self) {
        self.write(XFER, XFER_TXS_START | XFER_RXS_START);
    }

    pub fn stop_capture(&mut self) {
        self.write(XFER, 0);
    }

    pub fn handle_irq(&mut self) -> IrqEvent {
        let status = self.read(INTSR);
        if status & INT_RX_OVERFLOW != 0 {
            // Acknowledge the overflow with the write-1-to-clear bit while
            // preserving the RX interrupt enables already programmed in INTCR.
            let intcr = self.read(INTCR);
            self.write(INTCR, intcr | INT_RX_OVERFLOW_CLEAR);
            IrqEvent::RxOverflow
        } else if status & INT_RX_READY != 0 {
            // Level interrupt: cleared by draining the RX FIFO in task context.
            IrqEvent::RxReady
        } else {
            IrqEvent::None
        }
    }

    fn read(&self, offset: usize) -> u32 {
        self.regs.read(offset)
    }

    fn write(&mut self, offset: usize, value: u32) {
        self.regs.write(offset, value)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RingError {
    Full,
    Empty,
}

/// Single-producer (DMA/IRQ) and single-consumer (read) PCM ring.
pub struct PcmRing<const CAPACITY: usize> {
    samples: [i16; CAPACITY],
    read: usize,
    write: usize,
    len: usize,
    overruns: usize,
}

impl<const CAPACITY: usize> PcmRing<CAPACITY> {
    pub const fn new() -> Self {
        Self {
            samples: [0; CAPACITY],
            read: 0,
            write: 0,
            len: 0,
            overruns: 0,
        }
    }

    pub const fn capacity(&self) -> usize {
        CAPACITY
    }

    pub const fn available(&self) -> usize {
        self.len
    }

    pub const fn overruns(&self) -> usize {
        self.overruns
    }

    pub fn push(&mut self, sample: i16) -> Result<(), RingError> {
        if CAPACITY == 0 || self.len == CAPACITY {
            self.overruns = self.overruns.saturating_add(1);
            return Err(RingError::Full);
        }
        self.samples[self.write] = sample;
        self.write = (self.write + 1) % CAPACITY;
        self.len += 1;
        Ok(())
    }

    /// Ingest a freshly DMA'd, cache-invalidated block of samples in one call.
    /// Returns how many were accepted; samples that do not fit are dropped and
    /// counted as overruns, matching [`push`](Self::push). This is the producer
    /// path board glue calls on an RX period after invalidating the DMA buffer.
    pub fn push_slice(&mut self, samples: &[i16]) -> usize {
        let mut accepted = 0;
        for &sample in samples {
            if self.push(sample).is_ok() {
                accepted += 1;
            }
        }
        accepted
    }

    pub fn pop(&mut self) -> Result<i16, RingError> {
        if self.len == 0 {
            return Err(RingError::Empty);
        }
        let sample = self.samples[self.read];
        self.read = (self.read + 1) % CAPACITY;
        self.len -= 1;
        Ok(sample)
    }

    pub fn pop_slice(&mut self, output: &mut [i16]) -> usize {
        let mut count = 0;
        while count < output.len() {
            let Ok(sample) = self.pop() else { break };
            output[count] = sample;
            count += 1;
        }
        count
    }

    /// Drop every buffered sample without disturbing the lifetime overrun count.
    /// The capture runtime calls this when a stream (re)starts so stale samples
    /// captured before the reader was ready are not delivered.
    pub fn clear(&mut self) {
        self.read = 0;
        self.write = 0;
        self.len = 0;
    }
}

impl<const CAPACITY: usize> Default for PcmRing<CAPACITY> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The hardware `MmioRegisters` bank over a mock mapping: exercises the
    /// glue-side construction path (`MmioRaw` over a fake register file) that
    /// the real board takes through `iomap`.
    #[test]
    fn mmio_registers_roundtrip_over_raw_mapping() {
        const WORDS: usize = RK3588_I2S_TDM_REGISTER_SIZE / 4;
        let mut store = [0u32; WORDS];
        // SAFETY: `store` outlives the bank in this test and is a valid,
        // aligned mapping of `size_of_val(&store)` bytes by construction.
        let raw = unsafe {
            MmioRaw::new(
                0usize.into(),
                core::ptr::NonNull::new(store.as_mut_ptr().cast())
                    .expect("test register storage is non-null"),
                core::mem::size_of_val(&store),
            )
        };
        let mut regs = MmioRegisters { raw };

        regs.write(0x10, 0xC0C0_C0C0);
        assert_eq!(regs.read(0x10), 0xC0C0_C0C0);
        assert_eq!(store[0x10 / 4], 0xC0C0_C0C0);
    }

    /// Recording register bank: writes update the backing store and append to an
    /// ordered log; reads return the store, so tests can preset hardware status
    /// registers (`INTSR`) and then assert what the controller wrote and when.
    struct FakeRegs {
        store: [u32; 0x40],
        log: [(usize, u32); 32],
        entries: usize,
    }

    impl FakeRegs {
        const fn new() -> Self {
            Self {
                store: [0; 0x40],
                log: [(0, 0); 32],
                entries: 0,
            }
        }

        /// Preset a value the hardware would present, e.g. an `INTSR` status.
        fn preset(&mut self, offset: usize, value: u32) {
            self.store[offset / 4] = value;
        }
    }

    impl RegisterBank for FakeRegs {
        fn read(&self, offset: usize) -> u32 {
            self.store[offset / 4]
        }

        fn write(&mut self, offset: usize, value: u32) {
            self.store[offset / 4] = value;
            if self.entries < self.log.len() {
                self.log[self.entries] = (offset, value);
                self.entries += 1;
            }
        }
    }

    const TEST_CLOCK: ClockDividers = ClockDividers {
        bclk_div: 4,
        lrck_div: 64,
    };

    fn configured() -> I2sTdmController<FakeRegs> {
        let mut controller =
            I2sTdmController::with_registers(FakeRegs::new(), CaptureFormat::STEREO_S16_48K)
                .unwrap();
        controller.configure_capture(TEST_CLOCK).unwrap();
        controller
    }

    #[test]
    fn hardware_capture_format_is_stereo_s16_or_s24() {
        // The board forces interleaved stereo; S16 @ 48 kHz is the native shape.
        let stereo_s16 = CaptureFormat {
            sample_rate_hz: 48_000,
            channels: 2,
            sample_width_bits: 16,
        };
        assert_eq!(stereo_s16.validate(), Ok(stereo_s16));
        // 24-bit stereo is the other width the hardware capture path accepts.
        let stereo_s24 = CaptureFormat {
            sample_rate_hz: 48_000,
            channels: 2,
            sample_width_bits: 24,
        };
        assert_eq!(stereo_s24.validate(), Ok(stereo_s24));
        // Mono is not a hardware capture format: the runtime extracts it from the
        // stereo stream, so the register core rejects a mono request.
        let mono = CaptureFormat {
            sample_rate_hz: 48_000,
            channels: 1,
            sample_width_bits: 16,
        };
        assert_eq!(mono.validate(), Err(AudioConfigError::UnsupportedChannels));
        // 32-bit and off-list rates stay rejected.
        assert_eq!(
            CaptureFormat {
                sample_width_bits: 32,
                ..stereo_s16
            }
            .validate(),
            Err(AudioConfigError::UnsupportedSampleWidth)
        );
        assert_eq!(
            CaptureFormat {
                sample_rate_hz: 44_100,
                ..stereo_s16
            }
            .validate(),
            Err(AudioConfigError::UnsupportedSampleRate)
        );
    }

    #[test]
    fn ring_preserves_order_and_reports_overrun() {
        let mut ring = PcmRing::<2>::new();
        assert_eq!(ring.push(10), Ok(()));
        assert_eq!(ring.push(20), Ok(()));
        assert_eq!(ring.push(30), Err(RingError::Full));
        assert_eq!(ring.overruns(), 1);
        assert_eq!(ring.pop(), Ok(10));
        assert_eq!(ring.pop(), Ok(20));
        assert_eq!(ring.pop(), Err(RingError::Empty));
    }

    #[test]
    fn push_slice_ingests_block_and_counts_dropped_samples() {
        let mut ring = PcmRing::<4>::new();

        // A block that fits is ingested whole with no overruns.
        assert_eq!(ring.push_slice(&[1, 2, 3]), 3);
        assert_eq!(ring.available(), 3);
        assert_eq!(ring.overruns(), 0);

        // A block that overflows the remaining space accepts what fits and
        // counts every dropped sample as an overrun.
        assert_eq!(ring.push_slice(&[4, 5, 6]), 1);
        assert_eq!(ring.available(), 4);
        assert_eq!(ring.overruns(), 2);

        // FIFO order is preserved across the bulk ingest.
        let mut out = [0i16; 4];
        assert_eq!(ring.pop_slice(&mut out), 4);
        assert_eq!(out, [1, 2, 3, 4]);
    }

    #[test]
    fn clear_drops_buffered_samples_but_keeps_overrun_count() {
        let mut ring = PcmRing::<2>::new();
        ring.push(1).unwrap();
        ring.push(2).unwrap();
        assert_eq!(ring.push(3), Err(RingError::Full)); // records one overrun

        ring.clear();
        assert_eq!(ring.available(), 0);
        assert_eq!(ring.pop(), Err(RingError::Empty));
        assert_eq!(ring.overruns(), 1);

        // The ring is usable again after clearing.
        assert_eq!(ring.push(9), Ok(()));
        assert_eq!(ring.pop(), Ok(9));
    }

    #[test]
    fn configure_capture_writes_expected_register_sequence() {
        let controller = configured();

        let expected = [
            TXCR, RXCR, CKR, TDM_TXCR, TDM_RXCR, CLKDIV, DMACR, INTCR, CLR,
        ];
        assert_eq!(controller.regs.entries, expected.len());
        for (entry, &offset) in controller.regs.log[..controller.regs.entries]
            .iter()
            .zip(expected.iter())
        {
            assert_eq!(entry.0, offset);
        }

        // Ground-truth framing words (Linux `arecord hw:3,0` register dump).
        assert_eq!(controller.regs.store[TXCR / 4], 0x7200_000f);
        assert_eq!(controller.regs.store[RXCR / 4], 0x01c8_000f);
        assert_eq!(controller.regs.store[CKR / 4], 0x1000_3f3f);
        assert_eq!(controller.regs.store[CKR / 4], TEST_CLOCK.ckr_value());
        assert_eq!(controller.regs.store[CLKDIV / 4], 0x0303);
        assert_eq!(controller.regs.store[CLKDIV / 4], TEST_CLOCK.clkdiv_value());
        assert_eq!(controller.regs.store[TDM_TXCR / 4], TDM_CTRL_DEFAULT);
        assert_eq!(controller.regs.store[TDM_RXCR / 4], TDM_CTRL_DEFAULT);
        assert_eq!(
            controller.regs.store[DMACR / 4],
            DMACR_RX_ENABLE | DMACR_RX_LEVEL
        );
        assert_eq!(
            controller.regs.store[INTCR / 4],
            INT_RX_THRESHOLD | INT_RX_ENABLE | INT_RX_OVERFLOW_ENABLE
        );
    }

    #[test]
    fn configure_capture_encodes_valid_data_width_from_format() {
        // S16 stereo programs the 16-bit VDW field in both TXCR and RXCR...
        let s16 = configured();
        assert_eq!(
            s16.regs.store[TXCR / 4],
            TXCR_PATH_DEFAULT | RXCR_VDW_16_BIT
        );
        assert_eq!(
            s16.regs.store[RXCR / 4],
            RXCR_PATH_DEFAULT | RXCR_CSR_TWO_SLOTS | RXCR_VDW_16_BIT
        );

        // ...and a 24-bit stereo format widens the VDW field, still two slots,
        // without touching the rest of the sequence.
        let mut s24 =
            I2sTdmController::with_registers(FakeRegs::new(), CaptureFormat::STEREO_S24_48K)
                .unwrap();
        s24.configure_capture(TEST_CLOCK).unwrap();
        assert_eq!(
            s24.regs.store[TXCR / 4],
            TXCR_PATH_DEFAULT | RXCR_VDW_24_BIT
        );
        assert_eq!(
            s24.regs.store[RXCR / 4],
            RXCR_PATH_DEFAULT | RXCR_CSR_TWO_SLOTS | RXCR_VDW_24_BIT
        );
    }

    #[test]
    fn configure_capture_pio_clears_dma_and_enables_fifo_irq() {
        let mut controller =
            I2sTdmController::with_registers(FakeRegs::new(), CaptureFormat::STEREO_S16_48K)
                .unwrap();
        controller.configure_capture_pio(TEST_CLOCK).unwrap();

        // Same register sequence as the DMA path so the ordering guarantees hold,
        // but DMACR is programmed to zero rather than enabling RDE.
        let expected = [
            TXCR, RXCR, CKR, TDM_TXCR, TDM_RXCR, CLKDIV, DMACR, INTCR, CLR,
        ];
        assert_eq!(controller.regs.entries, expected.len());
        for (entry, &offset) in controller.regs.log[..controller.regs.entries]
            .iter()
            .zip(expected.iter())
        {
            assert_eq!(entry.0, offset);
        }

        // No DMA routing: a stale RDE left by a prior boot must be cleared.
        assert_eq!(controller.regs.store[DMACR / 4], 0);
        // The RX format is still programmed like the DMA path.
        assert_eq!(
            controller.regs.store[RXCR / 4],
            RXCR_PATH_DEFAULT | RXCR_CSR_TWO_SLOTS | RXCR_VDW_16_BIT
        );
        // FIFO-ready and overflow interrupts drive the CPU drain.
        assert_ne!(controller.regs.store[INTCR / 4] & INT_RX_ENABLE, 0);
        assert_ne!(controller.regs.store[INTCR / 4] & INT_RX_OVERFLOW_ENABLE, 0);
    }

    #[test]
    fn read_fifo_pops_reported_level_bounded_by_output() {
        let mut controller =
            I2sTdmController::with_registers(FakeRegs::new(), CaptureFormat::STEREO_S16_48K)
                .unwrap();

        // The FIFO reports three buffered samples; the data register presents the
        // sample value. read_fifo pops exactly the reported level.
        controller.regs.preset(RXFIFOLR, 3);
        controller.regs.preset(RXDR, 0x1234);
        let mut out = [0i16; 8];
        assert_eq!(controller.read_fifo(&mut out), 3);
        assert_eq!(&out[..3], &[0x1234, 0x1234, 0x1234]);

        // A shallow output buffer bounds the drain below the reported level.
        controller.regs.preset(RXFIFOLR, 5);
        let mut small = [0i16; 2];
        assert_eq!(controller.read_fifo(&mut small), 2);

        // A masked-off high bit in RXFIFOLR does not inflate the level.
        controller.regs.preset(RXFIFOLR, RXFIFOLR_LEVEL_MASK + 1);
        let mut none = [0i16; 4];
        assert_eq!(controller.read_fifo(&mut none), 0);
    }

    #[test]
    fn start_and_stop_toggle_rx_transfer() {
        let mut controller = configured();
        controller.start_capture();
        // TX-common mode: both TXS and RXS start so the shared clock runs.
        assert_eq!(
            controller.regs.store[XFER / 4],
            XFER_TXS_START | XFER_RXS_START
        );
        controller.stop_capture();
        assert_eq!(controller.regs.store[XFER / 4], 0);
    }

    #[test]
    fn irq_overflow_is_acknowledged_via_intcr_clear_bit() {
        let mut controller = configured();
        controller.regs.preset(INTSR, INT_RX_OVERFLOW);
        let before = controller.regs.entries;

        assert_eq!(controller.handle_irq(), IrqEvent::RxOverflow);

        // Overflow is acknowledged by the write-1-to-clear bit in INTCR, and the
        // RX interrupt enables must survive the acknowledgement.
        assert_ne!(controller.regs.store[INTCR / 4] & INT_RX_OVERFLOW_CLEAR, 0);
        assert_ne!(controller.regs.store[INTCR / 4] & INT_RX_ENABLE, 0);
        // The transfer-state CLR register is not the overflow acknowledgement path.
        for entry in &controller.regs.log[before..controller.regs.entries] {
            assert_ne!(entry.0, CLR);
        }
    }

    #[test]
    fn irq_ready_is_a_level_event_with_no_clear_write() {
        let mut controller = configured();
        controller.regs.preset(INTSR, INT_RX_READY);
        let before = controller.regs.entries;

        assert_eq!(controller.handle_irq(), IrqEvent::RxReady);

        // FIFO-ready is cleared by draining the RX FIFO in task context, so the
        // handler must not touch INTCR here.
        for entry in &controller.regs.log[before..controller.regs.entries] {
            assert_ne!(entry.0, INTCR);
        }
    }

    #[test]
    fn irq_idle_reports_none_without_writes() {
        let mut controller = configured();
        controller.regs.preset(INTSR, 0);
        let before = controller.regs.entries;

        assert_eq!(controller.handle_irq(), IrqEvent::None);
        assert_eq!(controller.regs.entries, before);
    }
}
