# RK3588 Orange Pi 5 Plus Capture Design

Status: host-side core plus the StarryOS `/dev/audio0` node are implemented and
compile-verified (fmt, the full `starry-kernel` clippy matrix with
`rk3588-audio`, and a linked Orange Pi 5 Plus board image). The capture data path
is **PIO-first**: a FIFO-threshold RX interrupt drains the RX FIFO in the handler
and feeds `AudioCaptureStream::ingest`; DMA is deferred as a later throughput
optimization. The clock/codec register values below are not yet a claim that the
board has been exercised end to end.

## Hardware evidence

The Orange Pi 5 Plus Linux device tree in `drivers/npu/rockchip-npu/fireware/orangepi5plus.dts` identifies the active board codec path as:

| Function | Evidence |
| --- | --- |
| CPU DAI | `i2s@fe470000`, compatible `rockchip,rk3588-i2s-tdm` |
| MMIO | `0xfe470000`, size `0x1000` |
| IRQ | GIC SPI `0xb4` |
| DMA | controller/channel pairs `0x7a/0x00` and `0x7a/0x01` |
| Codec | `es8388@11`, I2C address `0x11` |
| Link | `rockchip,multicodecs-card`, I2S format, MCLK ratio `256` |
| Capture routes | `LINPUT1/LINPUT2 -> Main Mic`, `RINPUT1/RINPUT2 -> Headset Mic` |

The separate `i2s@fddfc000` capture-only node is not used as the first target because the board sound card points at `fe470000`.

## Linux register evidence

The register offsets and bit fields are taken from Linux v6.1 `sound/soc/rockchip/rockchip_i2s_tdm.h` and the setup order from `rockchip_i2s_tdm.c`:

- `TXCR 0x000`, `RXCR 0x004`, `CKR 0x008`, `DMACR 0x010`, `INTCR 0x014`, `INTSR 0x018`, `XFER 0x01c`, `CLR 0x020`, `RXFIFOLR 0x02c`.
- RX DMA enable is `DMACR.RDE` bit 24; RX DMA level is bits 16-20.
- RX FIFO threshold enable is `INTCR.RXFIE` bit 16; RX overflow interrupt enable is bit 17; RX overflow clear is bit 18.
- RX FIFO and overflow status are reported at `INTSR` bits 16 and 17.
- RX transfer starts with `XFER.RXS_START` bit 1 and is cleared with `CLR_RXC` bit 1.

The driver core now uses these offsets and fields. RX overflow is acknowledged by setting the write-1-to-clear bit (`INTCR` bit 18) while preserving the RX interrupt enables; the FIFO-ready interrupt is a level event cleared by draining the RX FIFO in task context, not by an INTCR write. Register access flows through a small `RegisterBank` seam so the sequence and the acknowledgement path are exercised by host mock-MMIO tests without a board. Clock parent selection, exact divider values, I2S master/slave role, slot count, and the ES8388 register sequence remain board-glue responsibilities until the Linux boot logs and schematic are checked on the actual board.

## Codec bring-up (ES8388)

The `es8388` crate models the codec capture path as an ordered list of `(register, value)` writes (`CaptureSetup::register_writes`), taken from the Linux `sound/soc/codecs/es8328.h` map (the ES8388 shares the ES8328 registers). Board-specific analog choices — PGA gain, analog input select, and the MCLK/LRCK ratio field — are caller-supplied, not guessed, and must be confirmed against the schematic and Linux boot logs.

Sequence execution crosses an `I2cBus` capability boundary: `write_register` routes one 8-bit register/value pair to the codec at `ES8388_I2C_ADDR`, and `CaptureSetup::apply` validates first, then issues every write in order so an invalid configuration never leaves the codec half-programmed. Board glue implements `I2cBus` over the platform I2C controller; host tests supply a recording bus, so the whole bring-up is exercised without hardware. The bus seam mirrors the I2S core's `RegisterBank` seam, and `Es8388BringUpError` names the register whose write failed so glue can tell how far the sequence progressed.

## Layer boundary

```text
StarryOS /dev/audio0 (DeviceOps + Pollable glue in kernel devfs)
    -> audio-capture runtime: start/stop lifecycle, SPSC read path, poll readiness
    -> RK3588 board glue: FDT, clocks, reset, pinctrl, I2C, IRQ, DMA API
    -> rockchip-i2s-tdm core: register sequence, IRQ event, PCM ring
    -> RK3588 I2S/TDM + ES8388
```

The `audio-capture` crate is the portable, host-tested runtime between the ring
core and the OS device node. `AudioCaptureStream` owns the `PcmRing` consumer
side and the running/stopped lifecycle: `start` clears stale pre-reader samples,
`ingest` is the producer entry the RX-period glue calls, `read` drains whole
signed-16-bit little-endian samples (`ReadOutcome::Filled`/`WouldBlock`/
`Inactive`), and `poll` reports reader readiness. It stays `#![no_std]` with no
OS dependency so the read/poll/lifecycle state machine is exercised on the host.

The StarryOS node itself is a thin `DeviceOps` (and `Pollable`) adapter in kernel
devfs (`os/StarryOS/kernel/src/pseudofs/dev/audio.rs`, gated behind the
`rk3588-audio` feature), mirroring the existing `/dev/input/event*` (`EventDev`)
char device: `read_at` maps `ReadOutcome::Filled(n) -> Ok(n)`,
`WouldBlock -> AxError::WouldBlock`, `Inactive -> AxError::BadState`; `poll` maps
`Readiness.readable` to `IoEvents::IN`; and the RX FIFO-threshold IRQ drains the
FIFO into `AudioCaptureStream::ingest` then wakes poll waiters allocation-free via
`PollSet::wake_from_irq`. `write_at` is rejected (capture is read-only). The node
owns the `I2sTdmController<MmioRegisters>` directly — it FDT-discovers the
controller, `iomap`s the register block, and resolves the IRQ through
`resolve_binding_irq` (the `EventDev`/`KpuDevice` "Arch-1" pattern), so no
`ax-driver` model registration or `rdif-*` dependency is needed. On a platform
whose FDT lacks `rockchip,rk3588-i2s-tdm` (e.g. QEMU) `probe` returns `None` and
the node is simply absent. Boot maps + registers the GIC handler but writes no
hardware; `open` programs the PIO RX path and starts capture; `close` stops it.

The capture hardware is fixed to interleaved stereo: `hw:3,0` accepts only S16_LE/S24_LE and forces two channels, so the register core (`CaptureFormat::validate`) accepts stereo at 16- or 24-bit and rejects a mono request rather than pretending the controller can drop a slot. Turning that stereo stream into what `/dev/audio0` publishes is the runtime's job: `AudioCaptureStream::new` takes the hardware `CaptureFormat` plus an `OutputFormat` — `Mono(MonoSource::{Left,Right,Downmix})` to extract one logical channel, or `Stereo` to pass frames through — and `published_format` reports the resulting channel count at signed 16-bit. Mono extraction lives in the runtime, intentionally not hidden in the register core.

RX data flow is PIO-first. On an RX FIFO-threshold interrupt the handler reads the
FIFO level (`RXFIFOLR & 0x3f`, max 63) and pops samples from `RXDR` into a small
stack scratch buffer, then hands the CPU-readable interleaved stereo samples to
`AudioCaptureStream::ingest`, which extracts the configured `OutputFormat` per
frame into the `PcmRing` and counts any that overflow the ring. The portable core
owns only the single-producer/single-consumer `PcmRing`; the FIFO drain lives in
the OS node. This avoids `dma-api` in the core, which would need a platform
`DmaOp` and `alloc` and break the host-only ring/register tests. RX DMA (a
coherent `dma-api` buffer with `invalidate` on each period, feeding the same
`ingest` entry) remains a later throughput optimization, not a gate-4/5
requirement.

## Validation gates

1. Host unit tests for format validation, ring wrap/overrun, and the `audio-capture` read/poll/lifecycle state machine. (done)
2. Mock-MMIO tests for write order and interrupt acknowledgement. (done)
3. Linux board probe to confirm clocks, codec and actual `dmesg`/`/proc/asound` topology. (done) The clock chain is now confirmed from board ground truth (2026-08-26, Orange Pi 5 Plus, Linux 6.1.43, active 48 kHz S16_LE stereo capture on `hw:3,0`): RK3588 is I2S master and ES8388 is slave (ES8388 MASTERMODE reg `0x08 = 0x00`; `i2s0_8ch_mclkout` enabled); `rockchip,mclk-fs = 256`; MCLK = 12.288 MHz from pll_aupll 786.432 MHz → /2 → frac ÷32 → `mclk_i2s0_8ch_rx` 12.288 MHz; `hclk_i2s0_8ch` = 198 MHz. So `CAPTURE_CLOCK` `mclk_div: 1` and BCLK = MCLK / 8 = 1.536 MHz are the acoustically correct values. **Register-semantics gap discovered (feeds gate 4):** the raw registers Linux leaves running are `CKR = 0x10003f3f`, `RXCR = 0x01c8000f`, `TXCR = 0x7200000f`, `DMACR = 0x010f0010`, `INTCR = 0x01f20000`, `XFER = 0x00000003`. The current `ClockDividers::register_value` model (mclk@[16]/rx-bclk@[8]/tx-bclk@[0]) cannot reproduce `CKR = 0x10003f3f` — bit 28 (TRCM) is set and RSD/TSD = 0x3f read as an SCLK-per-frame ratio, not an MCLK/BCLK divider — and the core omits the RXCR framing bits `0x01c80000`. **This gap is now closed (2026-08-29):** the CKR/RXCR field layout was retaken from Linux `rockchip_i2s_tdm.h` and the core composition rewritten — `CKR_TRCM_TX` (bit 28), `CKR_MSS_MASTER`, `CKR_{RSD,TSD}_SHIFT` = 8/0 carrying the SCLK-per-frame ratio (`lrck_div - 1` = `0x3f`), plus `RXCR_PATH_DEFAULT = 0x01c80000` / `TXCR_PATH_DEFAULT = 0x72000000` for lane→path routing. `drivers/audio/rockchip-i2s-tdm/src/lib.rs:631-633` now asserts the three board words exactly (`TXCR = 0x7200000f`, `RXCR = 0x01c8000f`, `CKR = 0x10003f3f`) against the mock MMIO; 12/12 host tests pass. Treat the code, not this paragraph, as the authority on register composition.
4. StarryOS `/dev/audio0` `DeviceOps`/`Pollable` node in PIO mode: FDT probe, `iomap`, GIC IRQ registration, FIFO-threshold drain into `AudioCaptureStream::ingest`, poll wake. Compile/lint/board-link verified (fmt, full `starry-kernel` clippy matrix with `rk3588-audio`, linked Orange Pi 5 Plus image); on-board runtime bring-up (open → FIFO/IRQ/read pipeline) pending. On-board capture additionally requires SoC-side CRU clock/pinctrl/reset setup and ES8388 I2C bring-up; **both are now wired in (2026-08-29)** — `pseudofs/dev/audio_clk.rs` (`bring_up_i2s0_rx_clocks`, called from `pseudofs/dev/mod.rs:607`) brings up the I2S0 RX clock tree with bare-MMIO writes to the main CRU at `0xfd7c0000`, `pseudofs/dev/audio_codec.rs` (`bring_up_es8388_capture`, `mod.rs:611`) powers i2c7 and programs the ES8388 ADC over a polled rk3x master, and `audio.rs:248` wraps `start_capture` in `start_i2s0_capture_gate_cycle`. `probe` itself still does only `iomap` + IRQ registration by design — bring-up is sequenced at device-registration time, not in `probe`. GIC SPI `0xb4` (LEVEL_HIGH) and RX DMA request line 1 (controller phandle `0x7a`) are confirmed from the board device tree.
5. Userspace WAV capture over `/dev/audio0` and a 10-minute stability run on the board.

DMA RX (coherent `dma-api` buffer + per-period `invalidate` feeding `ingest`) is a deferred throughput optimization beyond gate 5, not part of the PIO-first bring-up.
