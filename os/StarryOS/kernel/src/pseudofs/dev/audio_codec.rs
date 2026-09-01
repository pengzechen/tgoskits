//! ES8388 codec I2C (i2c7) bring-up for `/dev/audio0`.
//!
//! Phase 1 goal: power the i2c7 bus (clock ungate + reset de-assert + pinmux)
//! and prove register read/write to the ES8388 over a polled rk3x I2C master.
//! Without the codec's ADC brought up over I2C the RK3588 I2S RX FIFO never
//! fills, so this is the first missing glue block behind `read()` blocking.
//!
//! The rk3x transaction core mirrors the proven PMIC master in ax-driver
//! (`soc/rockchip/pmic_i2c.rs`), retargeted from i2c0 (PMU CRU) to i2c7 (main
//! CRU) with the ES8388 bus wiring. It is re-implemented here rather than shared
//! because that master is private to ax-driver and gated behind
//! `rk3588-cpufreq`; the node owns its bring-up directly, matching the
//! self-contained `/dev/audio0` (`audio.rs`) probe pattern.
//!
//! Addresses confirmed from the Orange Pi 5 Plus FDT: i2c7 @ `0xfec90000`
//! (es8388@11), main CRU @ `0xfd7c0000`; gate PCLK_I2C7 = CLKGATE_CON10 bit14,
//! CLK_I2C7 = CLKGATE_CON11 bit6; reset SRST_P_I2C7 = SOFTRST_CON10 bit14,
//! SRST_I2C7 = SOFTRST_CON11 bit6.

// Phase 1 proved bus power-up + register read-back; Phase 3 now also programs the
// ES8388 ADC capture sequence over this master. Some rk3x fields stay unused
// (single-byte SMBus only), so keep the module-wide dead-code allowance.
#![allow(dead_code)]

use core::time::Duration;

use ax_memory_addr::PhysAddr;
use es8388::{ES8388_I2C_ADDR, I2cBus};
use rdif_pinctrl::PinctrlDevice;

use super::audio::ES8388_I2S_BASE;

/// i2c7 controller MMIO (Orange Pi 5 Plus; es8388@11 hangs off this bus).
const I2C7_BASE: usize = 0xfec9_0000;
const I2C7_SIZE: usize = 0x1000;

/// RK3588 main CRU (not the PMU CRU that i2c0 lives in).
/// `pub(super)` so the sibling `audio_clk` I2S clock-tree module can reuse the
/// same window base/size without a second copy of these primitives.
pub(super) const MAIN_CRU_BASE: usize = 0xfd7c_0000;
pub(super) const MAIN_CRU_SIZE: usize = 0x1000;

// Main CRU register layout: CLKGATE_CON(x) = 0x800 + x*4, SOFTRST_CON(x) = 0xa00 + x*4.
const CLKGATE_CON10: usize = 0x800 + 10 * 4; // 0x828 — PCLK_I2C7 bit 14
const CLKGATE_CON11: usize = 0x800 + 11 * 4; // 0x82c — CLK_I2C7 bit 6
const SOFTRST_CON10: usize = 0xa00 + 10 * 4; // 0xa28 — SRST_P_I2C7 bit 14
const SOFTRST_CON11: usize = 0xa00 + 11 * 4; // 0xa2c — SRST_I2C7 bit 6
const PCLK_I2C7_BIT: u32 = 1 << 14;
const CLK_I2C7_BIT: u32 = 1 << 6;

// rk3x I2C master register file (identical layout to ax-driver's pmic_i2c.rs).
const REG_CON: usize = 0x00;
const REG_CLKDIV: usize = 0x04;
const REG_MRXADDR: usize = 0x08;
const REG_MRXRADDR: usize = 0x0c;
const REG_MTXCNT: usize = 0x10;
const REG_MRXCNT: usize = 0x14;
const REG_IEN: usize = 0x18;
const REG_IPD: usize = 0x1c;
const TXDATA_BASE: usize = 0x100;
const RXDATA_BASE: usize = 0x200;

const CON_EN: u32 = 1 << 0;
const CON_START: u32 = 1 << 3;
const CON_STOP: u32 = 1 << 4;
const CON_LASTACK: u32 = 1 << 5;
const MODE_TX: u32 = 0;
const MODE_TRX: u32 = 1;
const fn con_mod(mode: u32) -> u32 {
    mode << 1
}
const MRXADDR_VALID0: u32 = 1 << 24;
const INT_MBTF: u32 = 1 << 2;
const INT_MBRF: u32 = 1 << 3;
const INT_START: u32 = 1 << 4;
const INT_STOP: u32 = 1 << 5;
const INT_NAKRCV: u32 = 1 << 6;
const INT_ALL: u32 = 0x7f;
const I2C_POLL_MAX: u32 = 100_000;

/// Minimal volatile MMIO window (avoids pulling `mmio-api` into the kernel as a
/// non-dev dependency; matches the bare-pointer style already used elsewhere).
/// `pub(super)` + `new()` so `audio_clk` can build a window over the same CRU
/// without duplicating this glue.
#[derive(Clone, Copy)]
pub(super) struct Mmio {
    base: *mut u8,
}

impl Mmio {
    pub(super) fn new(base: *mut u8) -> Self {
        Self { base }
    }

    pub(super) fn r(&self, off: usize) -> u32 {
        // SAFETY: `off` is a fixed in-range register offset and `base` maps the
        // whole `I2C7_SIZE`/`MAIN_CRU_SIZE` block returned by `iomap`.
        unsafe { self.base.add(off).cast::<u32>().read_volatile() }
    }

    pub(super) fn w(&self, off: usize, val: u32) {
        // SAFETY: see `r`.
        unsafe { self.base.add(off).cast::<u32>().write_volatile(val) }
    }
}

/// Map a physical MMIO window, returning its base pointer or `None` on failure.
pub(super) fn iomap(paddr: usize, size: usize) -> Option<*mut u8> {
    match axklib::mem::iomap(PhysAddr::from(paddr), size) {
        Ok(base) => Some(base.as_usize() as *mut u8),
        Err(err) => {
            warn!("rk3588-audio: failed to map MMIO at {paddr:#x}+{size:#x}: {err:?}");
            None
        }
    }
}

pub(super) fn busy_wait(dur: Duration) {
    axklib::time::busy_wait(dur);
}

/// Why a polled transaction ended without the expected interrupt-pending bit.
#[derive(Debug, Clone, Copy)]
enum I2cError {
    /// Slave did not ACK (NAKRCV latched) — usually wrong address or dead bus.
    Nak,
    /// The expected completion bit never latched within `I2C_POLL_MAX`.
    Timeout,
}

/// Polled rk3x I2C master over one mapped controller window. Single-byte SMBus
/// transactions only; the logic is copied verbatim from ax-driver's proven
/// `pmic_i2c.rs::Rk3xI2c` (i2c0 PMIC) and retargeted to i2c7.
struct Rk3xI2c {
    mmio: Mmio,
}

impl Rk3xI2c {
    fn r(&self, off: usize) -> u32 {
        self.mmio.r(off)
    }

    fn w(&self, off: usize, val: u32) {
        self.mmio.w(off, val);
    }

    /// Bring the master to a known-idle state: seed the clock divider if U-Boot
    /// left it unprogrammed (i2c7 is the codec bus, not touched by U-Boot),
    /// disable the controller, mask all IRQ sources, clear pending.
    fn init_controller(&self) {
        // CLKDIV halves: (124+1)+(124+1) = 250 core clocks/bit → ~100 kHz off a
        // 200 MHz i2c function clock. ES8388 is fine at standard mode.
        if self.r(REG_CLKDIV) == 0 {
            self.w(REG_CLKDIV, (124 << 16) | 124);
        }
        self.w(REG_CON, 0);
        self.w(REG_IEN, 0);
        self.w(REG_IPD, INT_ALL);
    }

    /// Poll IPD until `mask` latches. NAKRCV → `Nak`; exhausting the budget →
    /// `Timeout`. The matched (and any NAK) bit is written back to clear it.
    fn wait_ipd(&self, mask: u32) -> Result<(), I2cError> {
        for _ in 0..I2C_POLL_MAX {
            let ipd = self.r(REG_IPD);
            if ipd & INT_NAKRCV != 0 {
                self.w(REG_IPD, INT_NAKRCV);
                return Err(I2cError::Nak);
            }
            if ipd & mask != 0 {
                self.w(REG_IPD, mask);
                return Ok(());
            }
            busy_wait(Duration::from_micros(1));
        }
        Err(I2cError::Timeout)
    }

    fn send_start(&self) -> Result<(), I2cError> {
        self.w(REG_IPD, INT_ALL);
        self.w(REG_CON, CON_EN | CON_START);
        self.w(REG_IEN, INT_START);
        self.wait_ipd(INT_START)
    }

    fn send_stop(&self) -> Result<(), I2cError> {
        self.w(REG_IPD, INT_ALL);
        self.w(REG_CON, CON_EN | CON_STOP);
        self.w(REG_IEN, INT_STOP);
        self.wait_ipd(INT_STOP)
    }

    fn disable(&self) {
        self.w(REG_CON, 0);
    }

    /// Write one 8-bit register: START, TX {addr, reg, val}, STOP. Returns
    /// whether the byte was ACKed end-to-end. (Phase 3 codec programming path.)
    fn write_reg(&self, chip: u8, reg: u8, val: u8) -> bool {
        if self.send_start().is_err() {
            self.disable();
            return false;
        }
        let word0 = ((chip as u32) << 1) | ((reg as u32) << 8) | ((val as u32) << 16);
        self.w(TXDATA_BASE, word0);
        self.w(REG_CON, CON_EN | con_mod(MODE_TX));
        self.w(REG_MTXCNT, 3);
        self.w(REG_IEN, INT_MBTF | INT_NAKRCV);
        let ok = self.wait_ipd(INT_MBTF).is_ok();
        let _ = self.send_stop();
        self.disable();
        ok
    }

    /// Read one 8-bit register: START, TRX {addr|rd, reg}, receive 1 byte with
    /// LASTACK, STOP. `None` on NAK/timeout. Phase 1 read-back verification.
    fn read_reg(&self, chip: u8, reg: u8) -> Option<u8> {
        if self.send_start().is_err() {
            self.disable();
            return None;
        }
        self.w(REG_MRXADDR, (((chip as u32) << 1) | 1) | MRXADDR_VALID0);
        self.w(REG_MRXRADDR, (reg as u32) | MRXADDR_VALID0);
        self.w(REG_CON, CON_EN | CON_LASTACK | con_mod(MODE_TRX));
        self.w(REG_MRXCNT, 1);
        self.w(REG_IEN, INT_MBRF | INT_NAKRCV);
        let val = match self.wait_ipd(INT_MBRF) {
            Ok(()) => Some((self.r(RXDATA_BASE) & 0xff) as u8),
            Err(_) => None,
        };
        let _ = self.send_stop();
        self.disable();
        val
    }
}

/// A failed ES8388 register write over i2c7 (NAK or controller timeout inside
/// [`Rk3xI2c::write_reg`]). Named so [`es8388::Es8388BringUpError::Bus`] can
/// report which codec register did not take.
#[derive(Debug, Clone, Copy)]
pub(super) struct CodecWriteError;

impl I2cBus for Rk3xI2c {
    type Error = CodecWriteError;

    /// Route one codec register write through the proven single-byte rk3x TX
    /// path (device address fixed to the ES8388). `write_reg` reports whether the
    /// byte was ACKed end to end; anything else is a bus failure.
    fn write_register(&mut self, reg: u8, value: u8) -> Result<(), Self::Error> {
        if self.write_reg(ES8388_I2C_ADDR, reg, value) {
            Ok(())
        } else {
            Err(CodecWriteError)
        }
    }
}

/// Ungate the i2c7 clocks and release the controller from soft-reset in the
/// main CRU. Unlike i2c0 (both bits in one PMU CON), i2c7's PCLK/CLK gate and
/// P/functional reset bits each straddle two CON registers, so four write-masked
/// writes are needed. Write-masked: high 16 bits select which bits change, a `0`
/// in the low half means enabled/released, so re-clearing is idempotent.
/// Best-effort: a mapping failure is logged and the later transaction times out
/// into a safe no-op.
fn power_i2c7_bus() {
    let Some(base) = iomap(MAIN_CRU_BASE, MAIN_CRU_SIZE) else {
        return;
    };
    let cru = Mmio { base };
    // Ungate: clear PCLK_I2C7 (CON10 bit14) and CLK_I2C7 (CON11 bit6).
    cru.w(CLKGATE_CON10, PCLK_I2C7_BIT << 16);
    cru.w(CLKGATE_CON11, CLK_I2C7_BIT << 16);
    busy_wait(Duration::from_micros(5));
    // De-assert: clear SRST_P_I2C7 (CON10 bit14) and SRST_I2C7 (CON11 bit6).
    cru.w(SOFTRST_CON10, PCLK_I2C7_BIT << 16);
    cru.w(SOFTRST_CON11, CLK_I2C7_BIT << 16);
    busy_wait(Duration::from_micros(10));
    // Read the bits back (unmasked) for board diagnostics: 0 == enabled/released.
    let gate10 = cru.r(CLKGATE_CON10) & PCLK_I2C7_BIT;
    let gate11 = cru.r(CLKGATE_CON11) & CLK_I2C7_BIT;
    let rst10 = cru.r(SOFTRST_CON10) & PCLK_I2C7_BIT;
    let rst11 = cru.r(SOFTRST_CON11) & CLK_I2C7_BIT;
    info!(
        "rk3588-audio: i2c7 clocks/reset via main CRU {MAIN_CRU_BASE:#x} -- gate PCLK={gate10:#x} \
         CLK={gate11:#x}, reset P={rst10:#x} FUNC={rst11:#x} (0 == enabled/released)"
    );
}

/// Mux the i2c7 SCL/SDA pads (GPIO1_D0/D1 func9, i2c7m0) to the i2c7 function
/// through the registered Rockchip pinctrl driver, applying the controller
/// node's `pinctrl-0` default state. Without this the rk3x master never sees a
/// free bus and never generates START. Best-effort (mirrors i2c0 in pmic_i2c).
fn set_i2c7_pinmux() {
    let Some(pinctrl) = rdrive::get_one::<PinctrlDevice>() else {
        warn!("rk3588-audio: PinctrlDevice not registered; cannot mux i2c7 pins (START will fail)");
        return;
    };
    let mut pinctrl = match pinctrl.lock() {
        Ok(guard) => guard,
        Err(err) => {
            warn!("rk3588-audio: failed to lock PinctrlDevice: {err}; cannot mux i2c7 pins");
            return;
        }
    };
    let Some(fdt) = rdrive::with_fdt(Clone::clone) else {
        warn!("rk3588-audio: live FDT not found; cannot mux i2c7 pins");
        return;
    };
    // Several i2c controllers share the rk3399-i2c compatible; match on reg base.
    let node = fdt
        .find_compatible(&["rockchip,rk3588-i2c", "rockchip,rk3399-i2c"])
        .into_iter()
        .find(|n| {
            n.regs()
                .into_iter()
                .next()
                .is_some_and(|r| r.address as usize == I2C7_BASE)
        });
    let Some(node) = node else {
        warn!("rk3588-audio: i2c7 node @ {I2C7_BASE:#x} not found in FDT; cannot mux pins");
        return;
    };
    match pinctrl.apply_fdt_default_state(&fdt, node.as_node()) {
        Ok(()) => info!("rk3588-audio: applied i2c7 pinctrl-0 (GPIO1_D0/D1 -> func9) via pinctrl"),
        Err(err) => warn!("rk3588-audio: failed to apply i2c7 pinctrl-0: {err:?}"),
    }
}

/// Mux the I2S0 audio pads to their I2S functions: the controller's data/clock
/// pins (`i2s@fe470000` pinctrl-0 = i2s0-lrck/sclk/sdi0/sdo0) and the ES8388's
/// master-clock pin (`es8388@11` pinctrl-0 = i2s0-mclk). Both sit at their GPIO
/// reset default on a cold StarryOS boot, so without this the SoC samples a dead
/// SDI0 pad (every captured word is exactly `0`) and the codec ADC has no MCLK —
/// two independent causes of all-zero PCM even after the FIFO is advancing. Both
/// states are read straight from the live FDT and applied through the same
/// registered Rockchip pinctrl driver / [`PinctrlDevice::apply_fdt_default_state`]
/// path as [`set_i2c7_pinmux`], exactly what Linux does at probe. Best-effort: a
/// missing driver/node is logged and leaves the pad as-is (capture stays silent,
/// matching the pre-fix behaviour).
///
/// Called from [`bring_up_es8388_capture`] before the codec register replay so
/// MCLK is already live when the ADC powers on (its last replayed write).
fn apply_i2s0_audio_pinmux() {
    let Some(pinctrl) = rdrive::get_one::<PinctrlDevice>() else {
        warn!("rk3588-audio: PinctrlDevice not registered; cannot mux I2S0 audio pins (PCM stays 0)");
        return;
    };
    let mut pinctrl = match pinctrl.lock() {
        Ok(guard) => guard,
        Err(err) => {
            warn!("rk3588-audio: failed to lock PinctrlDevice for I2S0 pins: {err}");
            return;
        }
    };
    let Some(fdt) = rdrive::with_fdt(Clone::clone) else {
        warn!("rk3588-audio: live FDT not found; cannot mux I2S0 audio pins");
        return;
    };
    // I2S0 controller data/clock pads: i2s@fe470000 pinctrl-0 (lrck/sclk/sdi0/sdo0).
    // Several rk3588-i2s-tdm nodes are enabled; match the ES8388-connected one by
    // reg base, mirroring the probe binding filter in `audio.rs`.
    let i2s_node = fdt
        .find_compatible(&["rockchip,rk3588-i2s-tdm"])
        .into_iter()
        .find(|n| {
            n.regs()
                .into_iter()
                .next()
                .is_some_and(|r| r.address as usize == ES8388_I2S_BASE)
        });
    match i2s_node {
        Some(node) => match pinctrl.apply_fdt_default_state(&fdt, node.as_node()) {
            Ok(()) => {
                info!("rk3588-audio: applied i2s@{ES8388_I2S_BASE:#x} pinctrl-0 (lrck/sclk/sdi0/sdo0)")
            }
            Err(err) => warn!("rk3588-audio: failed to apply i2s0 pinctrl-0: {err:?}"),
        },
        None => warn!(
            "rk3588-audio: i2s node @ {ES8388_I2S_BASE:#x} not found in FDT; SDI0 pad unmuxed (PCM stays 0)"
        ),
    }
    // ES8388 master-clock pad: es8388@11 pinctrl-0 (i2s0-mclk). The codec node owns
    // the MCLKOUT pad mux; without it the ADC has no master clock and stays silent.
    let codec_node = fdt
        .find_compatible(&["everest,es8388", "everest,es8323"])
        .into_iter()
        .next();
    match codec_node {
        Some(node) => match pinctrl.apply_fdt_default_state(&fdt, node.as_node()) {
            Ok(()) => info!("rk3588-audio: applied es8388 pinctrl-0 (i2s0-mclk)"),
            Err(err) => warn!("rk3588-audio: failed to apply es8388 mclk pinctrl-0: {err:?}"),
        },
        None => warn!(
            "rk3588-audio: es8388 node not found in FDT; MCLK pad unmuxed (codec ADC has no clock)"
        ),
    }
}

/// ES8388 capture register replay — the exact register state a **working**
/// `arecord hw:3,0` leaves on this board, read register-by-register over i2c-7
/// (0x11) during an active Linux capture (2026-08-27, Orange Pi 5 Plus, kernel
/// 6.1.43). We replay these known-good words wholesale rather than compute them
/// from a generic codec config: the crate's `CaptureSetup` diverged from the
/// board's real capture state in several registers (CHIPPOWER 0x55 vs 0x00,
/// ADCPOWER 0x50 vs 0x09, ADCCONTROL4 0x0c vs 0x4c, ADCCONTROL5 0x00 vs 0x02)
/// and never wrote the analog reference registers CONTROL1/CONTROL2 at all —
/// so on a cold StarryOS boot the ES8388 stayed at its power-down reset defaults
/// (0x02=0xf3, 0x03=0xf9), the ADC front-end had no bias/reference, and `read()`
/// returned all-zero PCM (silence, not the analog noise floor). This mirrors the
/// board-specific CRU replay in `audio_clk.rs` — the board glue replays measured
/// Linux words; the generic crate is not "hardened" for this one board.
///
/// Ordering is the vendor power-up order: bring the reference/analog up first
/// (CONTROL1 ENREF + CONTROL2 vref/bias/analog), take all blocks out of reset
/// (CHIPPOWER=0x00), program the ADC data path (format/rate/HPF/volume/ALC),
/// unmute (ADCCONTROL7), and power the ADC on LAST (ADCPOWER=0x09). Values and
/// bit meanings cross-checked against Linux `sound/soc/codecs/es8328.h`:
///   0x00 CONTROL1=0x36  VMIDSEL=50k, ENREF on, SAMEFS, DACMCLK
///   0x01 CONTROL2=0x60  vref-buf/ibias/analog all ON (only VCM/overcurrent bits set)
///   0x08 MASTERMODE=0x00 codec is I2S clock slave (RK3588 is master)
///   0x02 CHIPPOWER=0x00 all DAC/ADC vref/DLL/STM/dig powered & out of reset
///   0x0b ADCCONTROL3=0x02
///   0x09 ADCCONTROL1=0x00 PGA 0 dB (analog gain comes from the ALC below)
///   0x0a ADCCONTROL2=0x00 LINPUT1/RINPUT1 selected
///   0x0c ADCCONTROL4=0x4c 16-bit (WL=011) I2S format (+ board bit6)
///   0x0d ADCCONTROL5=0x02 MCLK/LRCK ratio = 256fs (12.288 MHz / 48 kHz)
///   0x0e ADCCONTROL6=0x30 ADC HPF / polarity
///   0x10 ADCCONTROL8=0x00 left ADC digital volume 0 dB
///   0x11 ADCCONTROL9=0x00 right ADC digital volume 0 dB
///   0x12 ADCCONTROL10=0xea ALC stereo, target/max gain
///   0x13 ADCCONTROL11=0xc0 ALC hold/attack
///   0x14 ADCCONTROL12=0x05 ALC decay/attack
///   0x15 ADCCONTROL13=0x06 ALC mode
///   0x16 ADCCONTROL14=0x53 ALC noise gate
///   0x0f ADCCONTROL7=0x20 soft-ramp on, ADC unmuted
///   0x03 ADCPOWER=0x09 ADC L+R on, AIN on, INT1 low-power (LAST; mic-bias
///   bit unresolved -- see the note on the entry)
const ES8388_CAPTURE_REPLAY: &[(u8, u8)] = &[
    (0x00, 0x36),
    (0x01, 0x60),
    (0x08, 0x00),
    (0x02, 0x00),
    (0x0b, 0x02),
    // 2026-08-30 evening: align the analog front end with the vendor's
    // official onboard-mic configuration (test_record.sh main on the OPi
    // 5 Plus official image, cross-checked against the vendor es8323.c
    // enum encodings):
    //   'Left/Right PGA Mux' = 1  -> ADCCONTROL2 sel=1 "Line 2" both channels
    //   'Differential Mux'   = 1  -> ADCCONTROL3 bit7 set ("Line 2" source)
    //   'Left/Right Channel Capture Volume' = 4 -> ADCCONTROL1 nibbles 0x4
    //   'Capture Digital Volume' = 192 -> ADCCONTROL8/9 = 0xC0 (0 dB; our
    //     replay had 0x00, which on this register map is near-mute)
    // The arecord dump we replayed captured a state that never recorded the
    // onboard mic successfully (08-26: noise only), so replaying it exactly
    // reproduced the breakage.
    (0x09, 0x44),
    (0x0a, 0x50),
    (0x0b, 0x82),
    (0x0c, 0x4c),
    (0x0d, 0x02),
    (0x0e, 0x30),
    (0x10, 0xC0),
    (0x11, 0xC0),
    // ADCCONTROL10 = 0xE2: ALC stereo on (bits[7:6] kept from the dump) with
    // the vendor main-mic clamps 'ALC Capture Max PGA'=4 / 'Min PGA'=2 ->
    // bits[5:3]=100, bits[2:0]=010. Without the clamp the ALC pumped the
    // noise floor up to a constant RMS ~700 (2026-08-30 spectrograms).
    (0x12, 0xE2),
    (0x13, 0xc0),
    (0x14, 0x05),
    (0x15, 0x06),
    (0x16, 0x53),
    (0x0f, 0x20),
    // ADCPOWER=0x01: ADC L+R on, AIN on, MICBIAS ON (bit3 clear -- vendor
    // es8323.c: SND_SOC_DAPM_MICBIAS("Mic Bias", ES8323_ADCPOWER, 3, 1), so
    // enabling the bias CLEARS bit3; the Linux dump's 0x09 had it OFF). The
    // vendor amixer script never touches this register, so the official DAPM
    // graph presumably clears bit3 when the Main Mic route is live; we set
    // it statically.
    (0x03, 0x01),
];

/// Bring the ES8388 up for capture: power the i2c7 bus (clocks + reset + pinmux),
/// map the controller, prove the codec still ACKs (Phase 1 read-back), then push
/// the ordered capture register sequence (Phase 3). Without the ADC powered and
/// unmuted over I2C the RK3588 I2S RX FIFO stays empty and `read()` blocks, so
/// this is the last missing glue block. Called once from the `rk3588-audio`
/// probe, after the I2S RX clock tree is up so the codec sees a stable MCLK.
/// Idempotent and best-effort: a dead bus is logged and leaves the codec as-is.
pub fn bring_up_es8388_capture() {
    power_i2c7_bus();
    set_i2c7_pinmux();
    // Mux the I2S0 data/clock pads and the codec MCLK pad before touching the
    // codec: the ADC powers on at the tail of the replay below and needs a live
    // MCLK, and the SoC needs SDI0 muxed to sample anything but a dead `0` pad.
    apply_i2s0_audio_pinmux();
    let Some(base) = iomap(I2C7_BASE, I2C7_SIZE) else {
        warn!("rk3588-audio: i2c7 controller unmappable; ES8388 bring-up skipped");
        return;
    };
    let mut i2c = Rk3xI2c { mmio: Mmio { base } };
    i2c.init_controller();
    // Phase 1 read-back: REG_CONTROL1(0x00)/CHIP_POWER(0x02)/ADC_POWER(0x03) are
    // readable and not 0xff on a live codec. Any non-0xff proves the link before
    // we start writing. On a cold boot the codec is at its power-down reset state
    // (0x02≈0xf3, 0x03≈0xf9).
    for reg in [0x00u8, 0x02, 0x03] {
        match i2c.read_reg(ES8388_I2C_ADDR, reg) {
            Some(val) => info!("rk3588-audio: ES8388 reg {reg:#04x} = {val:#04x}"),
            None => warn!("rk3588-audio: ES8388 reg {reg:#04x} read failed (NAK/timeout)"),
        }
    }
    // Phase 3: replay the Linux ground-truth capture register set (see
    // [`ES8388_CAPTURE_REPLAY`]), in vendor power-up order, through the crate's
    // single-byte write path. Stop at the first NAK/timeout so a dead bus is
    // reported against the exact register that did not take.
    let mut applied = 0usize;
    for &(reg, val) in ES8388_CAPTURE_REPLAY {
        if i2c.write_register(reg, val).is_err() {
            warn!("rk3588-audio: ES8388 replay reg {reg:#04x}={val:#04x} failed (NAK/timeout)");
            break;
        }
        applied += 1;
    }
    if applied == ES8388_CAPTURE_REPLAY.len() {
        info!(
            "rk3588-audio: ES8388 capture replay applied ({applied} regs, Linux arecord ground \
             truth: slave, LINPUT1, mono-left, 256fs, ADC powered)"
        );
    }
    // Confirm the key writes stuck: CHIP_POWER(0x02) should now read 0x00 (all
    // blocks on), ADC_POWER(0x03) 0x09 (ADC L+R on), ADCCONTROL5(0x0d) 0x02
    // (256fs) — a change away from the 0xf3/0xf9 reset state.
    for reg in [0x02u8, 0x03, 0x0d] {
        if let Some(val) = i2c.read_reg(ES8388_I2C_ADDR, reg) {
            info!("rk3588-audio: ES8388 reg {reg:#04x} post-config = {val:#04x}");
        }
    }
}
