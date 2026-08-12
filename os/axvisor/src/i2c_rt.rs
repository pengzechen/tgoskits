// Copyright 2025 The Axvisor Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//! Reserved-core (core 7) I2C control for an LU9685 servo controller on the
//! OrangePi-5-Plus 40-pin header.
//!
//! Target bus: **I2C5 (`0xfead_0000`)**, muxed to function group **m3** on
//! `GPIO1_B6`/`GPIO1_B7` — the OrangePi-5-Plus header pins 28 (SCL) and 27
//! (SDA). This bus is `disabled` in the board DTB, so unlike the on-board buses
//! it carries no committed peripheral and is safe to bring up for a future wired
//! device. It is a header bus, not a PMIC rail: it is never I2C0/I2C1, so it
//! cannot brown-out the board.
//!
//! Because the bus is disabled in the DTB, neither U-Boot nor the (absent) host
//! pinctrl driver clocks or muxes it. This module therefore performs the three
//! bring-up steps a pinctrl/clk driver would normally do, once on a host CPU in
//! [`setup_host_side`]:
//!
//! 1. **Ungate** `PCLK_I2C5` and `CLK_I2C5` in the MAIN CRU, and make sure the
//!    shared `clk_200m_src`/`clk_100m_src` that feed `CLK_I2C5` are running
//!    (an enabled bus gets these for free — they are `CLK_IS_CRITICAL` — but a
//!    `disabled` bus that u-boot never used may not).
//! 2. **Deassert** the `SRST_P_I2C5` and `SRST_I2C5` soft-resets.
//! 3. **Mux** `GPIO1_B6`/`GPIO1_B7` to I2C5 function 9 in the BUS_IOC, and
//!    enable their internal pull-ups (the header has no board pull-ups, so the
//!    open-drain lines would otherwise float).
//!
//! The rk3x polling master is inlined from `ax-driver`'s `pmic_i2c` (same IP
//! block, `rockchip,rk3588-i2c`), retargeted to I2C5. The RT task writes the
//! LU9685 I2C protocol used by the ESP32 reference program: two data bytes,
//! `channel` and `angle`, to 7-bit address `0x00`.
//!
//! # Address-space handshake
//!
//! The reserved RT core shares the host kernel page table, so the controller
//! MMIO window is mapped **once on a host CPU** ([`setup_host_side`]) and its
//! virtual base is published to the RT task through [`I2C5_VIRT`]. The RT task
//! never calls `ioremap` (that locks `kernel_aspace()` and edits page tables — a
//! host-only operation); it rebuilds a [`MmioRaw`] view from the published base
//! and drives the controller with polling only.

use core::{
    ptr::NonNull,
    sync::atomic::{AtomicUsize, Ordering},
    time::Duration,
};

use ax_rt::{rt_output_write, rt_sleep};
use mmio_api::{MapError, MmioRaw};

/// RK3588 I2C5 controller MMIO window (`i2c@fead0000`, `rockchip,rk3588-i2c`).
const I2C5_BASE: usize = 0xfead_0000;
const I2C5_SIZE: usize = 0x1000;

// ---------------------------------------------------------------------------
// RK3588 clock/reset bring-up (MAIN CRU) and pin mux (BUS_IOC)
//
// i2c1..i2c8 live in the MAIN CRU at 0xfd7c_0000 (i2c0 is in the PMU CRU, which
// we never touch). Rockchip gate/soft-reset registers are write-masked: the
// upper 16 bits are a per-bit write-enable mask and the lower 16 bits are the
// value. Writing mask=1, value=0 to a bit therefore ENABLES a clock / DEASSERTS
// a reset. Register maps: clkgate_con(n) = 0x800 + n*4, softrst_con(n) = 0xa00
// + n*4. The (con, bit) slots below match both the repo RK3588 gate table and
// the board DTB `clocks`/`resets` phandles for i2c5.
// ---------------------------------------------------------------------------

/// RK3588 MAIN CRU window (clock gates + soft-resets for i2c1..i2c8).
const CRU_BASE: usize = 0xfd7c_0000;
const CRU_SIZE: usize = 0x1000;

/// `(clkgate/softrst register offset, bit)` slots for I2C5. Gate and reset share
/// the same bit layout on RK3588, so each pair mirrors the other's `(con, bit)`.
const GATE_PCLK_I2C5: (usize, u32) = (0x828, 12); // clkgate_con(10) bit12
const GATE_CLK_I2C5: (usize, u32) = (0x82c, 4); // clkgate_con(11) bit4
const SRST_P_I2C5: (usize, u32) = (0xa2c, 4); // softrst_con(11) bit4  (DTB id 180)
const SRST_I2C5: (usize, u32) = (0xa28, 12); // softrst_con(10) bit12 (DTB id 172)

/// The `CLK_I2C5` function clock is a `COMPOSITE_NODIV` off `mux_200m_100m_p`
/// (`{clk_200m_src, clk_100m_src}`), parent-selected by `CLKSEL_CON(38)` bit10
/// (0 → 200 MHz, 1 → 100 MHz). Both parents are `CLK_IS_CRITICAL` in Linux, so
/// an *enabled* i2c bus never touches them and the leaf-gate-only bring-up the
/// repo CRU driver does is sufficient. I2C5 is `disabled`, though, so u-boot may
/// never have run any MAIN-domain i2c and could have left these source clocks
/// gated. Ungate both and pin the 200 MHz parent as part of bring-up. Gate bits
/// are from `clk-rk3588.c`: `clk_100m_src` = `CLKGATE_CON(0)` bit1, `clk_200m_src`
/// = `CLKGATE_CON(0)` bit3. These are single-bit **masked clears** (enable-only):
/// clearing a gate bit can only start a clock, never stop one another peripheral
/// depends on, so touching the shared `CLKGATE_CON(0)` here is safe.
const GATE_CLK_100M_SRC: (usize, u32) = (0x800, 1); // clkgate_con(0) bit1
const GATE_CLK_200M_SRC: (usize, u32) = (0x800, 3); // clkgate_con(0) bit3
const CLKSEL_I2C5_SRC_OFF: usize = 0x398; // clksel_con(38)
const CLKSEL_I2C5_SRC_BIT: u32 = 10; // 0 = clk_200m_src, 1 = clk_100m_src

/// `PCLK_GPIO1` gate, `CLKGATE_CON(16)` bit14. Enabled (masked clear) before the
/// pad probe so GPIO1's `ext_port` reads reflect the live pad level instead of a
/// stale value read across a gated APB clock.
const GATE_PCLK_GPIO1: (usize, u32) = (0x840, 14); // clkgate_con(16) bit14

/// RK3588 IOC block window. The IOC is a set of sub-blocks in one 64 KiB region
/// at `0xfd5f_0000`: PMU1_IOC (+0x0000), PMU2_IOC (+0x4000), BUS_IOC (+0x8000),
/// and the VCCIO*_IOC blocks (+0x9000..). Pin *mux* for the GPIO1..GPIO4 "bus"
/// banks lives in BUS_IOC, but the pin *pull* config for those same pins lives
/// in a VCCIO_IOC block — a different page. Map the whole block once so a single
/// window reaches both the mux and pull registers.
const IOC_BASE: usize = 0xfd5f_0000;
const IOC_SIZE: usize = 0x1_0000;

/// GPIO1_B6 (pin 14) and GPIO1_B7 (pin 15) both mux in the bank-1 "B high"
/// register at BUS_IOC + 0x2c (IOC + 0x802c). RK3588 IOMUX uses one register per
/// 4-pin group: bank base = bank*0x20, the B group adds +0x08, and the high
/// nibble (pins 4..7 of the 8-pin group, i.e. B4..B7) adds a further +0x04 —
/// so GPIO1_B4..B7 land at 0x20 + 0x08 + 0x04 = 0x2c inside BUS_IOC. Within that
/// register the nibbles are bits [11:8] (B6) and [15:12] (B7); function 9 selects
/// I2C5_m3 (SCL on B6, SDA on B7). One write-masked write muxes both: mask 0xff00
/// (both nibbles), value 0x9900. (BUS_IOC + 0x24 is the *A* group, GPIO1_A4..A7 —
/// writing the mux there leaves B6/B7 at their reset default of GPIO, so the
/// controller's SCL/SDA never reach the header pins and every START times out.)
const IOC_I2C5_MUX_OFF: usize = 0x802c;
const IOC_I2C5_MUX_VAL: u32 = 0xff00_9900;

/// GPIO1_B6/B7 pull config, in VCCIO1-4_IOC's GPIO1_B pull register at
/// IOC + 0x9114. Each pin owns 2 bits; the value `3` selects pull-up. B6 =
/// bits [13:12], B7 = bits [15:14]. This header bus is `disabled` in the DTB and
/// has no board pull-ups, so I2C's open-drain SCL/SDA cannot rise without help;
/// enabling the SoC's internal pull-ups lets a START actually drive the lines.
/// One write-masked write pulls both up: mask 0xf000, value 0xf000.
const IOC_I2C5_PULL_OFF: usize = 0x9114;
const IOC_I2C5_PULL_VAL: u32 = 0xf000_f000;
/// Same register, both pins pulled *down* (value 1): mask 0xf000, value 0x5000.
/// Used only by [`diagnose_pad_levels`] as the low reference for the pad probe.
const IOC_I2C5_PULL_DOWN_VAL: u32 = 0xf000_5000;
/// Same register, both pins with pull *disabled* (high-Z, value 0): mask 0xf000,
/// value 0. Used by [`diagnose_pad_levels`] to sense any *external* pull-up.
const IOC_I2C5_PULL_NONE_VAL: u32 = 0xf000_0000;
/// Same register, both pins muxed back to GPIO func0: mask 0xff00, value 0.
const IOC_I2C5_MUX_GPIO_VAL: u32 = 0xff00_0000;

/// RK3588 GPIO1 controller window (`gpio@fec20000`, GPIO "v2" register layout).
/// Used read-only by the pad probe to sample the B6/B7 input level.
const GPIO1_BASE: usize = 0xfec2_0000;
const GPIO1_SIZE: usize = 0x1000;
/// GPIO v2 data-direction (low half, pins 0..15); write-masked. Bit=0 → input.
const GPIO_SWPORT_DDR_L: usize = 0x08;
/// GPIO v2 external port register: reads the live pad input level.
const GPIO_EXT_PORT: usize = 0x70;
/// B6 = bit 14, B7 = bit 15 within GPIO1's 32-bit port words.
const GPIO1_B6_BIT: u32 = 14;

/// Published I2C5 virtual base, or `0` until the host has mapped the window.
///
/// Written once with `Release` from [`setup_host_side`] on a host CPU and read
/// with `Acquire` from the RT core; the zero sentinel closes the boot-order race
/// (the RT task may start before the host finishes mapping).
static I2C5_VIRT: AtomicUsize = AtomicUsize::new(0);

const I2C5_BUS: I2cBusConfig = I2cBusConfig {
    name: "I2C5",
    base: I2C5_BASE,
    size: I2C5_SIZE,
    virt: &I2C5_VIRT,
};

// ---------------------------------------------------------------------------
// rk3x_i2c register map (offsets from the controller base)
// ---------------------------------------------------------------------------

const REG_CON: usize = 0x00; // control
const REG_CLKDIV: usize = 0x04; // clock divider
const REG_MTXCNT: usize = 0x10; // master-tx byte count
const REG_IEN: usize = 0x18; // interrupt enable
const REG_IPD: usize = 0x1c; // interrupt pending (raw status we poll)
const TXDATA_BASE: usize = 0x100; // tx FIFO word 0

// REG_CON bits
const CON_EN: u32 = 1 << 0;
const CON_START: u32 = 1 << 3;
const CON_STOP: u32 = 1 << 4;

// REG_CON transfer modes (shifted into bits [2:1] by `con_mod`)
const MODE_TX: u32 = 0; // write: address + data from tx FIFO

const fn con_mod(mode: u32) -> u32 {
    mode << 1
}

// REG_IPD/REG_IEN interrupt bits
const INT_MBTF: u32 = 1 << 2; // master byte-transmit finished
const INT_START: u32 = 1 << 4; // START generated
const INT_STOP: u32 = 1 << 5; // STOP generated
const INT_NAKRCV: u32 = 1 << 6; // NACK received
const INT_ALL: u32 = 0x7f; // all pending bits (write-1-to-clear)

/// Poll budget for a single IPD event, at 1 µs granularity — ~100 ms, matching
/// u-boot's `I2C_TIMEOUT_MS`. A register access completes in tens of µs; this
/// only bounds a wedged bus.
const I2C_POLL_MAX: u32 = 100_000;
/// RT sleep between smooth servo steps.
const SERVO_STEP_PERIOD_NANOS: u64 = 50_000_000;
/// RT retry backoff while waiting for the host to publish the MMIO base.
const MAP_WAIT_NANOS: u64 = 100_000_000;

const SERVO_PHYSICAL_RANGE_DEGREES: u16 = 270;
const SERVO_MIN_DEGREES: u16 = 60;
const SERVO_MAX_DEGREES: u16 = 210;
const SERVO_STEP_DEGREES: u16 = 2;

const LU9685_I2C_DEVICES: [Lu9685I2cConfig; 1] = [Lu9685I2cConfig {
    name: "LU9685@I2C5",
    address: 0x00,
    channels: &[0, 2],
}];

#[derive(Clone, Copy)]
struct I2cBusConfig {
    name: &'static str,
    base: usize,
    size: usize,
    virt: &'static AtomicUsize,
}

#[derive(Clone, Copy)]
struct Lu9685I2cConfig {
    name: &'static str,
    address: u8,
    channels: &'static [u8],
}

/// Outcome of waiting on an I2C completion event.
enum I2cErr {
    Nak,
    Timeout,
}

/// Write a Rockchip write-masked gate/reset bit to its "clear" state, i.e.
/// enable the clock or deassert the reset (`mask=1, value=0`).
fn cru_clear_bit(cru: &MmioRaw, off: usize, bit: u32) {
    cru.write::<u32>(off, 1u32 << (bit + 16));
}

/// Minimal rk3x polling master over a pre-mapped MMIO window.
///
/// All methods here run on the reserved RT core, so none of them take a host
/// console/log lock: diagnostics are emitted through [`rt_output_write`], the
/// RT-safe output ring. Only [`Rk3xI2c::init_controller`] is meant to run on a
/// host CPU (from [`setup_host_side`]).
struct Rk3xI2c {
    mmio: MmioRaw,
}

impl Rk3xI2c {
    #[inline]
    fn r(&self, off: usize) -> u32 {
        self.mmio.read::<u32>(off)
    }

    #[inline]
    fn w(&self, off: usize, val: u32) {
        self.mmio.write::<u32>(off, val);
    }

    /// Prepare the controller for polled use (host CPU only). I2C5 is disabled
    /// in the DTB, so firmware never configured `CLKDIV` — its reset default is
    /// junk (a ~1.8 MHz divider far too fast for the SoC's weak ~50 kΩ internal
    /// pull-ups to charge the line between edges). Force a conservative ~100 kHz
    /// divider (assuming RK3588's 200 MHz i2c function clock), then park the
    /// controller disabled with interrupts off and pending status cleared.
    /// Uses host logging; must not be called from the RT core.
    fn init_controller(&self) {
        // period = 8·(divh + divl + 2) i2c-clocks = 8·250 = 2000 clocks
        //        = 10 µs @ 200 MHz → 100 kHz SCL.
        let (divl, divh) = (124u32, 124u32);
        self.w(REG_CLKDIV, (divh << 16) | divl);
        self.w(REG_CON, 0);
        self.w(REG_IEN, 0);
        self.w(REG_IPD, INT_ALL);
        info!(
            "i2c_rt: rk3x_i2c@{I2C5_BASE:#x} ready (clkdiv={:#x})",
            self.r(REG_CLKDIV)
        );
    }

    /// Host-CPU START self-test (host CPU only). Issue one START + STOP on this
    /// bus from the boot CPU, right after [`init_controller`], and log whether
    /// the transfer FSM completes.
    ///
    /// This partitions the persistent core-7 START timeout, which — by every
    /// static measure (clock/reset/gate bits, mux read-back, idle-high pads) —
    /// matches the proven `pmic_i2c` path that drives I2C0 successfully. The one
    /// difference left is *where* the transfer runs: `pmic_i2c` does it on a host
    /// CPU, this module does it on the reserved core. Reading the outcome here:
    ///
    /// * **PASS** (with the RT task still timing out): the controller, its clock
    ///   source, and the pin mux are all live when driven from a host CPU, so the
    ///   remaining fault is specific to the RT core's execution/MMIO context.
    /// * **TIMEOUT** (same `IPD=0`): START fails regardless of which CPU drives
    ///   it, so the fault is fundamental to this controller instance (function
    ///   clock not reaching it, mux not truly connecting, or a held reset) and
    ///   the RT core is not the variable.
    ///
    /// Non-destructive: nothing is wired to the header, so START/STOP only toggle
    /// the idle-high lines; the controller is returned to the clean disabled
    /// state `init_controller` leaves. Host logging only; never call from the RT
    /// core.
    fn start_selftest_host(&self) {
        self.w(REG_IPD, INT_ALL);
        self.w(REG_CON, CON_EN | CON_START);
        self.w(REG_IEN, INT_START);
        let mut fired = false;
        for _ in 0..I2C_POLL_MAX {
            if self.r(REG_IPD) & INT_START != 0 {
                self.w(REG_IPD, INT_START);
                fired = true;
                break;
            }
            axklib::time::busy_wait(Duration::from_micros(1));
        }
        if fired {
            info!(
                "i2c_rt: HOST START self-test PASS (CON={:#010x} IPD={:#010x}); controller+clock+mux \
                 live on a host CPU — a remaining RT-core timeout is core-7-specific",
                self.r(REG_CON),
                self.r(REG_IPD),
            );
            // Emit a matching STOP so the bus is left idle, not mid-transaction.
            self.w(REG_IPD, INT_ALL);
            self.w(REG_CON, CON_EN | CON_STOP);
            self.w(REG_IEN, INT_STOP);
            for _ in 0..I2C_POLL_MAX {
                if self.r(REG_IPD) & INT_STOP != 0 {
                    self.w(REG_IPD, INT_STOP);
                    break;
                }
                axklib::time::busy_wait(Duration::from_micros(1));
            }
        } else {
            warn!(
                "i2c_rt: HOST START self-test TIMEOUT CON={:#010x} IPD={:#010x} CLKDIV={:#010x} \
                 IEN={:#010x}; START fails even on a host CPU — fault is fundamental to this \
                 controller (clock source / mux / reset), not RT-core-specific",
                self.r(REG_CON),
                self.r(REG_IPD),
                self.r(REG_CLKDIV),
                self.r(REG_IEN),
            );
        }
        // Return the controller to the clean, disabled state the RT task expects.
        self.w(REG_CON, 0);
        self.w(REG_IEN, 0);
        self.w(REG_IPD, INT_ALL);
    }

    /// Poll IPD until any bit in `mask` is set. A received NACK aborts early.
    /// The 1 µs delay uses `axklib::time::busy_wait`, which reads the physical
    /// counter and is safe on the isolated RT core.
    fn wait_ipd(&self, mask: u32) -> Result<(), I2cErr> {
        for _ in 0..I2C_POLL_MAX {
            let ipd = self.r(REG_IPD);
            if ipd & mask != 0 {
                self.w(REG_IPD, mask);
                return Ok(());
            }
            if ipd & INT_NAKRCV != 0 {
                self.w(REG_IPD, INT_NAKRCV);
                return Err(I2cErr::Nak);
            }
            axklib::time::busy_wait(Duration::from_micros(1));
        }
        Err(I2cErr::Timeout)
    }

    fn send_start(&self) -> Result<(), I2cErr> {
        self.w(REG_IPD, INT_ALL);
        self.w(REG_CON, CON_EN | CON_START);
        self.w(REG_IEN, INT_START);
        self.wait_ipd(INT_START)
    }

    fn send_stop(&self) -> Result<(), I2cErr> {
        self.w(REG_IPD, INT_ALL);
        self.w(REG_CON, CON_EN | CON_STOP);
        self.w(REG_IEN, INT_STOP);
        self.wait_ipd(INT_STOP)
    }

    #[inline]
    fn disable(&self) {
        self.w(REG_CON, 0);
    }

    /// RT-side MMIO sanity check. `REG_IEN` is safe to toggle while the
    /// controller is disabled, and readback tells us whether CPU7 stores reach
    /// the controller window at all.
    fn rt_mmio_write_readback_probe(&self) {
        self.w(REG_CON, 0);
        self.w(REG_IEN, 0);
        self.w(REG_IEN, INT_START);
        let ien_after_set = self.r(REG_IEN);
        self.w(REG_IEN, 0);
        let ien_after_clear = self.r(REG_IEN);

        rt_output_write(b"RT i2c5 mmio probe: CON=");
        rt_write_hex32(self.r(REG_CON));
        rt_output_write(b" CLKDIV=");
        rt_write_hex32(self.r(REG_CLKDIV));
        rt_output_write(b" IEN set=");
        rt_write_hex32(ien_after_set);
        rt_output_write(b" clear=");
        rt_write_hex32(ien_after_clear);
        rt_output_write(b"\n");
    }

    /// LU9685 I2C write: `[START][chip.W][channel][angle][STOP]`.
    fn write_lu9685_angle(&self, chip: u8, channel: u8, raw_angle: u8) -> Result<(), I2cErr> {
        self.send_start()?;
        let word0 = ((chip as u32) << 1) | ((channel as u32) << 8) | ((raw_angle as u32) << 16);
        self.w(TXDATA_BASE, word0);
        self.w(REG_CON, CON_EN | con_mod(MODE_TX));
        self.w(REG_MTXCNT, 3);
        self.w(REG_IEN, INT_MBTF | INT_NAKRCV);
        let res = self.wait_ipd(INT_MBTF);
        let _ = self.send_stop();
        self.disable();
        res
    }
}

/// Write `v` as `0x` + 8 lowercase hex digits to the RT output ring.
fn rt_write_hex32(v: u32) {
    let mut buf = [b'0'; 10];
    buf[1] = b'x';
    for (i, slot) in buf[2..].iter_mut().enumerate() {
        let nib = ((v >> ((7 - i) * 4)) & 0xf) as u8;
        *slot = if nib < 10 {
            b'0' + nib
        } else {
            b'a' + (nib - 10)
        };
    }
    rt_output_write(&buf);
}

/// Write `v` as two lowercase hex digits (no `0x` prefix) to the RT output ring.
fn rt_write_hex8(v: u8) {
    let hex = |nib: u8| {
        if nib < 10 {
            b'0' + nib
        } else {
            b'a' + nib - 10
        }
    };
    rt_output_write(&[hex(v >> 4), hex(v & 0xf)]);
}

/// Decisive electrical probe (host CPU only): is the dead bus a *wiring/domain*
/// problem or an *internal* one? Temporarily switches GPIO1_B6/B7 to GPIO input
/// and samples the pad level first under an internal pull-up, then under a
/// pull-down, restoring the I2C5 mux + pull-up before returning.
///
/// rk3x can only generate a START when SCL and SDA are both idle-high, so a
/// line stuck low makes START time out with `IPD=0` — exactly the observed
/// failure. Interpreting the log line:
///
/// - `pull-up->0b11 pull-down->0b00`: the IO domain is powered and both internal
///   pulls work, so the lines *can* idle high — the dead bus is internal (clock
///   source / controller), not electrical.
/// - `pull-up->0b00` (stays low): the lines are held low — an unpowered IO
///   voltage domain or an external short — and no START can ever form.
///
/// Bit 0 of each value is B6 (SCL), bit 1 is B7 (SDA).
fn diagnose_pad_levels() -> Result<(), MapError> {
    let ioc = axklib::mmio::ioremap_raw((IOC_BASE as u64).into(), IOC_SIZE)?;
    let gpio = axklib::mmio::ioremap_raw((GPIO1_BASE as u64).into(), GPIO1_SIZE)?;

    // B6/B7 -> GPIO func0, input direction (write-masked: mask bits 14/15).
    ioc.write::<u32>(IOC_I2C5_MUX_OFF, IOC_I2C5_MUX_GPIO_VAL);
    gpio.write::<u32>(GPIO_SWPORT_DDR_L, 0x3 << (GPIO1_B6_BIT + 16));

    // Pull-up, let the (weak) pull settle against pad capacitance, then sample.
    ioc.write::<u32>(IOC_I2C5_PULL_OFF, IOC_I2C5_PULL_VAL);
    axklib::time::busy_wait(Duration::from_micros(50));
    let up = (gpio.read::<u32>(GPIO_EXT_PORT) >> GPIO1_B6_BIT) & 0x3;

    // Pull-down and re-sample as the low reference.
    ioc.write::<u32>(IOC_I2C5_PULL_OFF, IOC_I2C5_PULL_DOWN_VAL);
    axklib::time::busy_wait(Duration::from_micros(50));
    let down = (gpio.read::<u32>(GPIO_EXT_PORT) >> GPIO1_B6_BIT) & 0x3;

    // Pull disabled (high-Z): with no internal pull, the level reflects whatever
    // is on the wire. High here means an *external* pull-up is present (the
    // header would then not need the internal one); low/indeterminate means the
    // pin only rises via the internal pull-up sampled above.
    ioc.write::<u32>(IOC_I2C5_PULL_OFF, IOC_I2C5_PULL_NONE_VAL);
    axklib::time::busy_wait(Duration::from_micros(50));
    let float = (gpio.read::<u32>(GPIO_EXT_PORT) >> GPIO1_B6_BIT) & 0x3;

    info!(
        "i2c_rt: pad probe B6/B7 ext_port: pull-up->{up:#04b} pull-down->{down:#04b} \
         float->{float:#04b} (bit0=B6/SCL, bit1=B7/SDA; healthy = up 0b11 / down 0b00; \
         float 0b11 = external pull-ups present, 0b00 = none)"
    );

    // Restore the final I2C5 pin state: pull-up + I2C5_m3 mux.
    ioc.write::<u32>(IOC_I2C5_PULL_OFF, IOC_I2C5_PULL_VAL);
    ioc.write::<u32>(IOC_I2C5_MUX_OFF, IOC_I2C5_MUX_VAL);
    Ok(())
}

/// Apply the bring-up steps I2C5 needs on this board, once on a host CPU:
/// ungate its clocks, deassert its soft-resets, and mux + pull-up GPIO1_B6/B7.
///
/// Runs on a host CPU (it calls `ioremap`, which edits the shared kernel page
/// table). Each window is mapped only for the duration of these register
/// writes; the RT core only ever touches the controller window.
fn setup_pinmux_and_clocks() -> Result<(), MapError> {
    // Steps 1 & 2: ungate clocks and deassert soft-resets in the MAIN CRU.
    let cru = axklib::mmio::ioremap_raw((CRU_BASE as u64).into(), CRU_SIZE)?;
    cru_clear_bit(&cru, GATE_PCLK_I2C5.0, GATE_PCLK_I2C5.1);
    cru_clear_bit(&cru, GATE_CLK_I2C5.0, GATE_CLK_I2C5.1);
    cru_clear_bit(&cru, SRST_P_I2C5.0, SRST_P_I2C5.1);
    cru_clear_bit(&cru, SRST_I2C5.0, SRST_I2C5.1);

    // Step 2b: make sure the function-clock *source* is actually running. Log
    // the source registers as-found first (pure reads, no risk), then ungate
    // clk_200m_src / clk_100m_src and pin I2C5 to the 200 MHz parent. All three
    // writes are enable-only / single-bit masked: none can gate a clock another
    // peripheral relies on. This is the one bring-up step the repo CRU driver
    // skips (it assumes these CLK_IS_CRITICAL sources are already on), and the
    // most likely thing u-boot never did for this `disabled` bus.
    let gate0_before = cru.read::<u32>(GATE_CLK_200M_SRC.0);
    let sel0_before = cru.read::<u32>(0x300); // clksel_con(0): clk_100m_src sel/div
    let sel1_before = cru.read::<u32>(0x304); // clksel_con(1): clk_200m_src sel/div
    let sel38_before = cru.read::<u32>(CLKSEL_I2C5_SRC_OFF);
    cru_clear_bit(&cru, GATE_CLK_100M_SRC.0, GATE_CLK_100M_SRC.1);
    cru_clear_bit(&cru, GATE_CLK_200M_SRC.0, GATE_CLK_200M_SRC.1);
    // Select clk_200m_src (bit10 = 0) via a write-masked single-bit clear.
    cru.write::<u32>(CLKSEL_I2C5_SRC_OFF, 1u32 << (CLKSEL_I2C5_SRC_BIT + 16));
    // Step 2c: ungate GPIO1's APB clock so the pad probe's ext_port reads are live.
    cru_clear_bit(&cru, GATE_PCLK_GPIO1.0, GATE_PCLK_GPIO1.1);

    // Step 3: mux GPIO1_B6 (SCL) / GPIO1_B7 (SDA) to I2C5_m3 and enable their
    // internal pull-ups. Both registers live in the one IOC block window.
    let ioc = axklib::mmio::ioremap_raw((IOC_BASE as u64).into(), IOC_SIZE)?;
    ioc.write::<u32>(IOC_I2C5_MUX_OFF, IOC_I2C5_MUX_VAL);
    ioc.write::<u32>(IOC_I2C5_PULL_OFF, IOC_I2C5_PULL_VAL);

    // Read the (write-masked) registers back so the boot log proves each step
    // actually took. Expect: gate/reset bits read 0 (clock enabled / reset
    // deasserted); mux nibbles read 0x9 (both B6 and B7 → func9); pull nibbles
    // read 0x3 (both pulled up). A bit that stays set points at the failed step.
    info!(
        "i2c_rt: CRU gate10={:#010x} gate11={:#010x} srst10={:#010x} srst11={:#010x}",
        cru.read::<u32>(GATE_PCLK_I2C5.0),
        cru.read::<u32>(GATE_CLK_I2C5.0),
        cru.read::<u32>(SRST_I2C5.0),
        cru.read::<u32>(SRST_P_I2C5.0),
    );
    // Source-clock state: `gate0` bit1 (clk_100m_src) and bit3 (clk_200m_src)
    // should read 0 (running) after the ungate; `sel38` bit10 should read 0
    // (I2C5 sourced from 200 MHz). If gate0 read 0 *before*, u-boot already had
    // the sources on and the dead bus is not a source-gate problem.
    info!(
        "i2c_rt: CRU src as-found gate0={gate0_before:#010x} sel0={sel0_before:#010x} \
         sel1={sel1_before:#010x} sel38={sel38_before:#010x}; now gate0={:#010x} sel38={:#010x}",
        cru.read::<u32>(GATE_CLK_200M_SRC.0),
        cru.read::<u32>(CLKSEL_I2C5_SRC_OFF),
    );
    info!(
        "i2c_rt: IOC mux[{IOC_I2C5_MUX_OFF:#x}]={:#010x} pull[{IOC_I2C5_PULL_OFF:#x}]={:#010x}",
        ioc.read::<u32>(IOC_I2C5_MUX_OFF),
        ioc.read::<u32>(IOC_I2C5_PULL_OFF),
    );
    info!("i2c_rt: I2C5 clocks ungated, resets deasserted, GPIO1_B6/B7 muxed to func9 + pulled up");
    Ok(())
}

/// Bring up I2C5 (clocks + resets + pin mux + controller init) and map its MMIO
/// window once on a host CPU, publishing the virtual base to the RT task. Call
/// after `ax_realtime::setup_host_side()`.
pub fn setup_host_side() {
    if let Err(err) = setup_pinmux_and_clocks() {
        // Mapping the CRU/IOC failed; the controller will very likely time out
        // its first START. Still map it so the RT task can surface the failure.
        warn!(
            "i2c_rt: I2C5 clock/pinmux bring-up failed: {err:?}; RT servo task may report bus-dead"
        );
    }
    // One-shot electrical probe: distinguish an unpowered IO domain / stuck line
    // from an internal (clock/controller) fault. Non-fatal; log and continue.
    if let Err(err) = diagnose_pad_levels() {
        warn!("i2c_rt: pad-level probe skipped (map failed: {err:?})");
    }
    match axklib::mmio::ioremap_raw((I2C5_BUS.base as u64).into(), I2C5_BUS.size) {
        Ok(mmio) => {
            let controller = Rk3xI2c { mmio };
            controller.init_controller();
            // Decisive partition: drive one START from this host CPU before
            // handing the window to the RT core. PASS here + RT timeout ⇒ the
            // fault is core-7-specific; TIMEOUT here ⇒ it is fundamental to the
            // controller. See [`Rk3xI2c::start_selftest_host`].
            controller.start_selftest_host();
            // Publish only the virtual base; the RT core rebuilds its own MmioRaw
            // view. Release pairs with the Acquire load in `i2c_servo_task`.
            let virt = controller.mmio.as_nonnull_ptr().as_ptr() as usize;
            I2C5_BUS.virt.store(virt, Ordering::Release);
            info!(
                "i2c_rt: {} mapped for RT core (virt={virt:#x})",
                I2C5_BUS.name
            );
        }
        Err(err) => {
            warn!(
                "i2c_rt: ioremap {}@{:#x} failed: {err:?}; RT servo task will idle",
                I2C5_BUS.name, I2C5_BUS.base
            );
        }
    }
}

/// RT task body: once the host has published the I2C5 base, smoothly sweep the
/// LU9685 channels 0 and 2 through a safe mid-range window.
///
/// Runs on the reserved RT core and uses only RT-safe APIs (`rt_output_write`,
/// `rt_sleep`, and the polling `Rk3xI2c`); it never takes a host lock or maps
/// memory itself.
pub fn i2c_servo_task() -> ! {
    let mut probed_mmio = false;
    let mut motion = ServoMotion::new(SERVO_MIN_DEGREES, SERVO_MAX_DEGREES, SERVO_STEP_DEGREES);
    let mut report_countdown = 0u8;
    loop {
        let virt = I2C5_BUS.virt.load(Ordering::Acquire);
        if virt == 0 {
            // Host has not finished mapping I2C5 yet; back off and retry.
            rt_sleep(MAP_WAIT_NANOS);
            continue;
        }

        // SAFETY: `virt` was produced by `setup_host_side` on a host CPU via
        // `ioremap_raw` of `[I2C5_BASE, I2C5_BASE + I2C5_SIZE)` into the shared
        // kernel page table, as DEVICE memory. The RT core shares that page
        // table, so the mapping is valid here. The Axvisor host configures no
        // I2C driver and I2C5 is disabled in the DTB, so the reserved core is
        // the sole user of this window — no aliasing or concurrent access.
        let mmio = unsafe {
            MmioRaw::new(
                (I2C5_BUS.base as u64).into(),
                NonNull::new_unchecked(virt as *mut u8),
                I2C5_BUS.size,
            )
        };
        let controller = Rk3xI2c { mmio };
        if !probed_mmio {
            controller.rt_mmio_write_readback_probe();
            probed_mmio = true;
        }

        let physical_degrees = motion.next_position();
        let raw_angle = physical_to_lu9685_angle(physical_degrees);
        match send_servo_positions(&controller, raw_angle) {
            Ok(()) => {
                if report_countdown == 0
                    || physical_degrees == SERVO_MIN_DEGREES
                    || physical_degrees == SERVO_MAX_DEGREES
                {
                    report_servo_position(physical_degrees, raw_angle);
                    report_countdown = 20;
                } else {
                    report_countdown -= 1;
                }
            }
            Err(err) => report_servo_failure(&controller, err, physical_degrees, raw_angle),
        }
        rt_sleep(SERVO_STEP_PERIOD_NANOS);
    }
}

struct ServoMotion {
    position: u16,
    min: u16,
    max: u16,
    step: u16,
    ascending: bool,
}

impl ServoMotion {
    const fn new(min: u16, max: u16, step: u16) -> Self {
        Self {
            position: min,
            min,
            max,
            step,
            ascending: true,
        }
    }

    fn next_position(&mut self) -> u16 {
        let current = self.position;
        self.advance();
        current
    }

    fn advance(&mut self) {
        if self.ascending {
            self.position = self.position.saturating_add(self.step);
            if self.position >= self.max {
                self.position = self.max;
                self.ascending = false;
            }
        } else {
            self.position = self.position.saturating_sub(self.step);
            if self.position <= self.min {
                self.position = self.min;
                self.ascending = true;
            }
        }
    }
}

fn send_servo_positions(controller: &Rk3xI2c, raw_angle: u8) -> Result<(), I2cErr> {
    for device in LU9685_I2C_DEVICES {
        for &channel in device.channels {
            controller.write_lu9685_angle(device.address, channel, raw_angle)?;
        }
    }
    Ok(())
}

fn physical_to_lu9685_angle(physical_degrees: u16) -> u8 {
    ((physical_degrees * 180 + SERVO_PHYSICAL_RANGE_DEGREES / 2) / SERVO_PHYSICAL_RANGE_DEGREES)
        as u8
}

fn report_servo_position(physical_degrees: u16, raw_angle: u8) {
    rt_output_write(b"RT i2c5");
    for device in LU9685_I2C_DEVICES {
        rt_output_write(b" ");
        rt_output_write(device.name.as_bytes());
    }
    rt_output_write(b" set physical=");
    ax_rt::rt_output_write_decimal(physical_degrees as u64);
    rt_output_write(b" raw=");
    ax_rt::rt_output_write_decimal(raw_angle as u64);
    rt_output_write(b" addr=0x");
    rt_write_hex8(LU9685_I2C_DEVICES[0].address);
    rt_output_write(b"\n");
}

fn report_servo_failure(controller: &Rk3xI2c, err: I2cErr, physical_degrees: u16, raw_angle: u8) {
    rt_output_write(b"RT i2c5 LU9685 write FAIL ");
    match err {
        I2cErr::Nak => rt_output_write(b"NAK"),
        I2cErr::Timeout => rt_output_write(b"timeout"),
    }
    rt_output_write(b" physical=");
    ax_rt::rt_output_write_decimal(physical_degrees as u64);
    rt_output_write(b" raw=");
    ax_rt::rt_output_write_decimal(raw_angle as u64);
    rt_output_write(b" CON=");
    rt_write_hex32(controller.r(REG_CON));
    rt_output_write(b" IPD=");
    rt_write_hex32(controller.r(REG_IPD));
    rt_output_write(b" CLKDIV=");
    rt_write_hex32(controller.r(REG_CLKDIV));
    rt_output_write(b" IEN=");
    rt_write_hex32(controller.r(REG_IEN));
    rt_output_write(b"\n");
}
