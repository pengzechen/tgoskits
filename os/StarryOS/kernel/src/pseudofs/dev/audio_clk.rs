//! RK3588 I2S0 (fe470000) RX clock-tree bring-up for `/dev/audio0` (Phase 2).
//!
//! Without the SoC-side I2S RX clock tree the I2S0 RX FIFO never advances, so
//! `read()` on `/dev/audio0` blocks forever even though probe / open / GIC IRQ
//! all succeed (that exact symptom was observed on-board once the fe470000
//! binding was fixed). Under Linux the kernel CRU driver programs this tree;
//! when StarryOS takes over fe470000 it must do the same.
//!
//! The repo CRU crate (`rockchip-soc`) has no I2S clock support beyond the
//! AUPLL primitive, and per the project scope ("先跑通链路，不做通用补强") we
//! bring the tree up with self-contained bare-MMIO writes to the main CRU
//! (`0xfd7c0000`), mirroring the verified `audio_codec.rs` i2c7 style rather
//! than extending the CRU crate. The low-level MMIO/iomap/busy_wait primitives
//! and the CRU window base are reused from `audio_codec` (no duplication).
//!
//! Target clock: `mclk_i2s0_8ch_rx = 12.288 MHz` (AUPLL 786.432 MHz ÷2 →
//! fractional ÷32), `hclk_i2s0_8ch` gated on, I2S0 RX path out of reset.
//! `mclk-fs = 256` ⇒ 48 kHz sample rate, matching the hardware capture format.
//!
//! Register map (offsets from `0xfd7c0000`; every CRU register here is
//! hiword-masked — high 16 bits select which bits change, low 16 bits are the
//! value — EXCEPT the fractional divider `CLKSEL_CON27`, which is a plain
//! 32-bit `num[31:16]/den[15:0]` word and must be written raw):
//!   AUPLL:        CON0 `0x180` (M), CON1 `0x184` (P/S/PWRDOWN), CON2 `0x188`
//!                 (K), CON6 `0x198` (LOCK); mode `0x280` bits[7:6].
//!   CLKSEL_CON24  `0x360`  hclk_audio_root mux → clk_200m_src (~198 MHz)
//!   CLKSEL_CON26  `0x368`  clk_i2s0_8ch_rx_src mux(AUPLL)+div(÷2)
//!   CLKSEL_CON27  `0x36c`  rx fractional divider (RAW 32-bit num=1/den=32)
//!   CLKSEL_CON28  `0x370`  rx final mux → fractional path
//!   CLKGATE_CON7  `0x81c`  audio-root / i2s0_8ch / rx_src / rx_frac / mclk gates
//!   SOFTRST_CON7  `0xa1c`  audio-biu / i2s0_8ch / i2s0_8ch_rx resets
//!
//! Values cross-verified against Linux mainline `clk-rk3588.c` and the repo's
//! own tested `cru/pll.rs` AUPLL programming sequence.

#![allow(dead_code)]

use core::ptr::NonNull;
use core::time::Duration;

use rockchip_pm::{RkBoard, RockchipPM};

use super::audio_codec::{MAIN_CRU_BASE, MAIN_CRU_SIZE, Mmio, busy_wait, iomap};

// --- RK3588 PMU power-controller (audio power domain) ------------------------
// fe470000 (I2S0) sits in the `audio` power domain, which is POWERED OFF at
// StarryOS boot (Linux runtime-PM only resumes it while a capture holds it).
// Any read of fe470000 with the domain off takes a synchronous external abort
// (DataAbort @ VA fe470000, ESR=0x96000010). The CRU gates were never the
// blocker — they are already on at idle. We reuse the tested in-repo
// `rockchip-pm` crate to power the AUDIO(38) domain on before touching the CRU
// or the fe470000 controller. PMU is always powered, so mapping/writing it is
// safe. Base/size from DT `power-management@fd8d8000`.
const PMU_BASE: usize = 0xfd8d_8000;
const PMU_SIZE: usize = 0x400;

// --- AUPLL (audio PLL) register file, offsets from main CRU base -------------
const AUPLL_CON0: usize = 0x180; // M[9:0]
const AUPLL_CON1: usize = 0x184; // P[5:0], S[8:6], PWRDOWN bit13
const AUPLL_CON2: usize = 0x188; // K[15:0] (fractional)
const AUPLL_CON6: usize = 0x198; // LOCK_STATUS bit15
const AUPLL_MODE: usize = 0x280; // mode field at bits[7:6]
const AUPLL_MODE_SHIFT: u32 = 6;

const PLL_MODE_MASK: u32 = 0x3;
const PLL_MODE_SLOW: u32 = 0;
const PLL_MODE_NORMAL: u32 = 1;

const PLLCON0_M_MASK: u32 = 0x3ff;
const PLLCON1_P_MASK: u32 = 0x3f;
const PLLCON1_S_SHIFT: u32 = 6;
const PLLCON1_S_MASK: u32 = 0x7 << PLLCON1_S_SHIFT;
const PLLCON1_PWRDOWN: u32 = 1 << 13;
const PLLCON2_K_MASK: u32 = 0xffff;
const PLLCON6_LOCK: u32 = 1 << 15;

const OSC_HZ: u64 = 24_000_000;

/// AUPLL 786.432 MHz params (p=2, m=262, s=2, k=9437) — the fractional entry
/// from `PLL_RATE_TABLE`; yields 786_431_991 Hz by integer math.
const AUPLL_TARGET_HZ: u64 = 786_432_000;
const AUPLL_P: u32 = 2;
const AUPLL_M: u32 = 262;
const AUPLL_S: u32 = 2;
const AUPLL_K: u32 = 9437;

// --- I2S0 clock-tree registers -----------------------------------------------
// The board is `rockchip,trcm-sync-tx-only`: the shared BCLK/LRCK is generated
// on the TX timing engine, and `I2S0_8CH_MCLKOUT` (the pin that clocks ES8388)
// is sourced from `mclk_i2s0_8ch_tx` (CON28[3:2]=0). So BOTH the TX and the RX
// MCLK trees must be brought up — an RX-only bring-up leaves the TX engine
// without a valid MCLK and the RX FIFO never advances (observed on-board: the
// full ground-truth I2S framing landed but `rx_fifo_level` stayed 0).
//
// Register/bit map verified against Linux mainline `clk-rk3588.c`:
//   CON24[9]  TX_SRC mux (0=gpll,1=aupll), [8:4] TX_SRC div (n+1)
//   CON25     TX fractional divider (RAW num[31:16]/den[15:0])
//   CON26[1:0] TX final mux (0=src,1=frac,2=mclkin,3=xin12m);
//             [7] RX_SRC mux (aupll), [6:2] RX_SRC div  -- CON26 is SHARED
//   CON27     RX fractional divider (RAW)
//   CON28[1:0] RX final mux; [3:2] MCLKOUT select (0=mclk_tx)
//   CLKGATE_CON7: bit4 hclk, 5 tx_src, 6 tx_frac, 7 mclk_tx, 8 rx_src,
//             9 rx_frac, 10 mclk_rx
//   SOFTRST_CON7: bit4 hclk, 7 mclk_tx, 10 mclk_rx
const CLKSEL_CON24: usize = 0x360;
const CLKSEL_CON25: usize = 0x364; // RAW 32-bit TX fractional divider
const CLKSEL_CON26: usize = 0x368;
const CLKSEL_CON27: usize = 0x36c; // RAW 32-bit RX fractional divider
const CLKSEL_CON28: usize = 0x370;
const CLKGATE_CON7: usize = 0x81c;
const SOFTRST_CON7: usize = 0xa1c;

// These words REPRODUCE the exact main-CRU state a working Linux capture leaves
// on this board — read on-board via /dev/mem at 0xfd7c0000 during an active
// `arecord hw:3,0` (2026-08-27). We write Linux's known-good CLKSEL/CLKGATE
// words wholesale (mask 0xffff) rather than only the RX sub-fields: the earlier
// partial gate (0x0711 → bits {0,4,8,9,10}) left `hclk_i2s0_8ch` — the fe470000
// register-block bus clock, one of the {2,3,5,6,7} bits we skipped — GATED, so
// every write to the controller was silently dropped and the first *read* took
// a synchronous external abort (DataAbort @ VA fe470000, ESR=0x96000010). The
// idle-vs-active CRU dump was identical, so the RX gates stay on in Linux and
// matching that state is what un-blocks fe470000 access.
// CLKSEL_CON25/CON27 stay plain 32-bit fractional words (num[31:16]/den[15:0]).
const CON24_TX_SRC: u32 = 0xffff_0210; // TX_SRC mux=AUPLL, div=÷2 → 393.216 MHz
const CON25_TX_FRAC_RAW: u32 = 0x0001_0020; // TX frac num1/den32 → 12.288 MHz
const CON26_MUX: u32 = 0xffff_0085; // TX final mux[1:0]=frac + rx_src AUPLL ÷2
const CON27_RX_FRAC_RAW: u32 = 0x0001_0020; // RX frac num1/den32 → 12.288 MHz
const CON28_RX_MUX: u32 = 0xffff_0011; // rx final = frac path; MCLKOUT = mclk_tx
const GATE7_OPEN: u32 = 0xffff_f802; // Linux CLKGATE_CON7 = 0xf802 (hclk + tx & rx paths on)
const SRST7_ASSERT: u32 = 0x0490_0490; // pulse-assert i2s0 hclk/tx/rx resets (bits 4,7,10)
const SRST7_DEASSERT: u32 = 0x07ff_0000; // release audio-root/i2s0 resets (Linux SOFTRST_CON7 = 0)

/// Power the RK3588 `audio` power domain (index 38) on via the tested in-repo
/// `rockchip-pm` crate. This MUST run before anything reads fe470000 (the CRU
/// bring-up below and the controller writes in `audio.rs`): with the domain off
/// the first fe470000 access aborts. The crate performs the pwr-bit clear plus
/// a repair-status poll; it does not do the bus-idle de-request handshake, so
/// if fe470000 still aborts after this the idle path must be added on top.
/// Best-effort: on any failure we log and continue (the abort, if it still
/// happens, is the diagnostic).
fn power_on_audio_domain() {
    let Some(base) = iomap(PMU_BASE, PMU_SIZE) else {
        warn!("rk3588-audio: PMU {PMU_BASE:#x} unmappable; AUDIO power-domain on skipped");
        return;
    };
    let Some(nn) = NonNull::new(base) else {
        warn!("rk3588-audio: PMU iomap returned null; AUDIO power-domain on skipped");
        return;
    };
    let mut pm = RockchipPM::new(nn, RkBoard::Rk3588);
    match pm.get_power_dowain_by_name("audio") {
        Some(domain) => match pm.power_domain_on(domain) {
            Ok(()) => info!("rk3588-audio: AUDIO power-domain (38) powered on"),
            Err(e) => warn!("rk3588-audio: AUDIO power-domain on failed: {e:?}"),
        },
        None => warn!("rk3588-audio: 'audio' power-domain not found in rockchip-pm"),
    }
}

// --- Rockchip hiword-masked write helpers ------------------------------------
// Every write-masked CRU register takes `(mask << 16) | value`: the high half
// enables exactly the bits named in `mask`, the low half supplies their value.

/// Clear the bits in `clr` and set the bits in `set` (a `0` in a gate/reset
/// field means "enabled/released").
fn clrsetreg(cru: &Mmio, off: usize, clr: u32, set: u32) {
    cru.w(off, ((clr | set) << 16) | set);
}

/// Set the given bits to 1.
fn setbits(cru: &Mmio, off: usize, set: u32) {
    cru.w(off, (set << 16) | set);
}

/// Clear the given bits to 0.
fn clrbits(cru: &Mmio, off: usize, clr: u32) {
    cru.w(off, clr << 16);
}

/// Read AUPLL's current output rate the same way the tested `cru/pll.rs`
/// `pll_get_rate` does: SLOW→OSC, DEEP→32768, NORMAL→compute from M/P/S/K.
fn aupll_read_rate(cru: &Mmio) -> u64 {
    let mode = (cru.r(AUPLL_MODE) >> AUPLL_MODE_SHIFT) & PLL_MODE_MASK;
    match mode {
        PLL_MODE_SLOW => return OSC_HZ,
        PLL_MODE_NORMAL => {}
        2 => return 32768, // DEEP
        _ => return 0,
    }
    let m = cru.r(AUPLL_CON0) & PLLCON0_M_MASK;
    let con1 = cru.r(AUPLL_CON1);
    let p = con1 & PLLCON1_P_MASK;
    let s = (con1 & PLLCON1_S_MASK) >> PLLCON1_S_SHIFT;
    let k = cru.r(AUPLL_CON2) & PLLCON2_K_MASK;
    if p == 0 {
        return OSC_HZ;
    }
    let mut rate = (OSC_HZ / p as u64) * m as u64;
    if k != 0 {
        rate += (OSC_HZ * k as u64) / (p as u64 * 65536);
    }
    rate >> s
}

/// Program AUPLL to 786.432 MHz only if it isn't already there — U-Boot may or
/// may not set it on a cold StarryOS boot, so guard on the read-back rate to
/// keep this idempotent and avoid disturbing a working PLL. The programming
/// sequence mirrors `cru/pll.rs::pll_set_rate`: SLOW → power-down → write
/// M/P/S/K → power-up → wait lock → NORMAL.
fn ensure_aupll_786m(cru: &Mmio) {
    let before = aupll_read_rate(cru);
    // 0.1% tolerance (integer math lands on 786_431_991 Hz).
    if before.abs_diff(AUPLL_TARGET_HZ) <= AUPLL_TARGET_HZ / 1000 {
        info!("rk3588-audio: AUPLL already at {before} Hz (~786.432 MHz); skip reprogram");
        return;
    }
    info!("rk3588-audio: AUPLL at {before} Hz, programming to 786.432 MHz (p=2 m=262 s=2 k=9437)");

    // SLOW mode, then power down before touching dividers.
    clrsetreg(
        cru,
        AUPLL_MODE,
        PLL_MODE_MASK << AUPLL_MODE_SHIFT,
        PLL_MODE_SLOW << AUPLL_MODE_SHIFT,
    );
    setbits(cru, AUPLL_CON1, PLLCON1_PWRDOWN);

    // M, then P|S, then K.
    clrsetreg(cru, AUPLL_CON0, PLLCON0_M_MASK, AUPLL_M & PLLCON0_M_MASK);
    clrsetreg(
        cru,
        AUPLL_CON1,
        PLLCON1_P_MASK | PLLCON1_S_MASK,
        (AUPLL_P & PLLCON1_P_MASK) | ((AUPLL_S << PLLCON1_S_SHIFT) & PLLCON1_S_MASK),
    );
    clrsetreg(cru, AUPLL_CON2, PLLCON2_K_MASK, AUPLL_K & PLLCON2_K_MASK);

    // Power up and wait for lock (≤1 ms, matching pll.rs).
    clrbits(cru, AUPLL_CON1, PLLCON1_PWRDOWN);
    let mut timeout = 1000u32;
    while cru.r(AUPLL_CON6) & PLLCON6_LOCK == 0 {
        if timeout == 0 {
            warn!("rk3588-audio: AUPLL lock timeout; continuing (rx clock may be wrong)");
            break;
        }
        busy_wait(Duration::from_micros(1));
        timeout -= 1;
    }

    // Back to NORMAL mode so the programmed rate drives the tree.
    clrsetreg(
        cru,
        AUPLL_MODE,
        PLL_MODE_MASK << AUPLL_MODE_SHIFT,
        PLL_MODE_NORMAL << AUPLL_MODE_SHIFT,
    );
    let after = aupll_read_rate(cru);
    info!("rk3588-audio: AUPLL now {after} Hz (~786.432 MHz target)");
}

/// Phase 2 entry: bring up the I2S0 RX clock tree on the main CRU so the RX
/// FIFO can advance. Called once from the `rk3588-audio` device probe, before
/// the I2S controller register writes in `audio.rs`. Best-effort and
/// idempotent (all steps below are deassert/mux writes; only the AUPLL step is
/// guarded because a PLL reprogram is not idempotent).
pub fn bring_up_i2s0_rx_clocks() {
    // 0. Power on the `audio` power domain FIRST — fe470000 lives in it and is
    //    off at boot; every step below (and audio.rs) reads/writes that block.
    power_on_audio_domain();

    let Some(base) = iomap(MAIN_CRU_BASE, MAIN_CRU_SIZE) else {
        warn!("rk3588-audio: main CRU unmappable; I2S0 RX clock bring-up skipped");
        return;
    };
    let cru = Mmio::new(base);

    // 1. AUPLL → 786.432 MHz (guarded).
    ensure_aupll_786m(&cru);

    // 2. clk_i2s0_8ch_tx_src: source AUPLL, pre-divide ÷2 → 393.216 MHz. This is
    //    the shared-clock (BCLK/LRCK) engine's MCLK source in trcm-sync-tx-only.
    cru.w(CLKSEL_CON24, CON24_TX_SRC);
    // 3. tx fractional divider 1/32 → mclk_i2s0_8ch_tx = 12.288 MHz (RAW 32-bit).
    //    Without this the TX engine has no valid MCLK, no BCLK/LRCK is generated,
    //    and the RX FIFO never advances (observed on-board: rx_fifo_level=0).
    cru.w(CLKSEL_CON25, CON25_TX_FRAC_RAW);
    // 4. CON26 (SHARED): tx final mux[1:0]=frac AND rx_src mux=AUPLL / div ÷2.
    cru.w(CLKSEL_CON26, CON26_MUX);
    // 5. rx fractional divider 1/32 → mclk_i2s0_8ch_rx = 12.288 MHz (RAW 32-bit).
    cru.w(CLKSEL_CON27, CON27_RX_FRAC_RAW);
    // 6. rx final mux → fractional path; MCLKOUT (codec pin) ← mclk_i2s0_8ch_tx.
    cru.w(CLKSEL_CON28, CON28_RX_MUX);
    // 7. Ungate audio-root / i2s0_8ch / tx & rx src / frac / mclk.
    cru.w(CLKGATE_CON7, GATE7_OPEN);
    // 8. Pulse-reset the I2S0 hclk/tx/rx clock paths clean: assert, hold, deassert.
    cru.w(SOFTRST_CON7, SRST7_ASSERT);
    busy_wait(Duration::from_micros(10));
    cru.w(SOFTRST_CON7, SRST7_DEASSERT);
    busy_wait(Duration::from_micros(10));

    // Read-back diagnostics (0 in a gate/reset bit == enabled/released).
    let sel24 = cru.r(CLKSEL_CON24);
    let sel25 = cru.r(CLKSEL_CON25);
    let sel26 = cru.r(CLKSEL_CON26);
    let sel27 = cru.r(CLKSEL_CON27);
    let sel28 = cru.r(CLKSEL_CON28);
    let gate7 = cru.r(CLKGATE_CON7);
    let srst7 = cru.r(SOFTRST_CON7);
    info!(
        "rk3588-audio: I2S0 TX+RX clocks via main CRU {MAIN_CRU_BASE:#x} -- \
         sel24={sel24:#010x} sel25={sel25:#010x} sel26={sel26:#010x} sel27={sel27:#010x} \
         sel28={sel28:#010x} gate7={gate7:#010x} srst7={srst7:#010x} (mclk tx&rx target 12.288 MHz)"
    );
}

/// `mclk_i2s0_8ch_tx` gate bit in `CLKGATE_CON7` (1 = gated OFF).
const GATE7_MCLK_TX_BIT: u32 = 1 << 7;

/// Start-time clock discontinuity for I2S0 capture (`rockchip,trcm-sync-tx-only`).
///
/// Arming `XFER = TXS|RXS` while `mclk_i2s0_8ch_tx` is already free-running does
/// NOT reliably kick the shared BCLK/LRCK generator on the TX timing engine, so
/// RX never sees a frame clock and the RX FIFO stays empty — observed on-board
/// (`rx_fifo_level=0`) with every controller/CRU register value already matching
/// a working Linux capture. The vendor RK3588 driver
/// (`rockchip_i2s_tdm_xfer_with_gate`) gates `mclk_i2s0_8ch_tx` OFF, writes the
/// combined `XFER` start bits while gated, then gates it back ON; that
/// gate→ungate edge re-aligns the TX and RX engines on the same clock edge so
/// the generator actually starts. This is the exact choreography the board's own
/// `arecord hw:3,0` runs. `start_xfer` performs the single `XFER = TXS|RXS`
/// write (`I2sTdmController::start_capture`). Best-effort: if the CRU cannot be
/// mapped we still issue the XFER write so the failure mode is unchanged.
pub fn start_i2s0_capture_gate_cycle(start_xfer: impl FnOnce()) {
    let Some(base) = iomap(MAIN_CRU_BASE, MAIN_CRU_SIZE) else {
        warn!("rk3588-audio: main CRU unmappable at capture start; XFER without gate cycle");
        start_xfer();
        return;
    };
    let cru = Mmio::new(base);
    // Gate mclk_i2s0_8ch_tx OFF (CLKGATE_CON7 bit7 = 1), only that bit.
    setbits(&cru, CLKGATE_CON7, GATE7_MCLK_TX_BIT);
    busy_wait(Duration::from_micros(10));
    // Arm TX+RX transfer in one XFER write while the mclk is gated.
    start_xfer();
    busy_wait(Duration::from_micros(10));
    // Gate mclk_i2s0_8ch_tx back ON (bit7 = 0) — the edge kicks the LRCK/BCLK gen.
    clrbits(&cru, CLKGATE_CON7, GATE7_MCLK_TX_BIT);
    info!("rk3588-audio: I2S0 capture started via mclk_tx gate cycle (trcm-sync-tx-only)");
}
