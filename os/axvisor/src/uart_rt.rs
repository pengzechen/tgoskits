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

//! Reserved-core UART control for devices on the OrangePi-5-Plus 40-pin header.
//!
//! Three RK3588 UARTs are brought up here, each on an independent pin group:
//!
//! | UART | Device (feature) | TX / RX pins | Mux |
//! | --- | --- | --- | --- |
//! | UART7 | LU9685 servo (`rt-uart`) | GPIO1_B5 (pin 26) / GPIO1_B4 (pin 24) | `uart7m2-xfer` |
//! | UART6 | right motor (`rt-motor`) | GPIO1_A1 (pin 8) / GPIO1_A0 (pin 10) | `uart6m1-xfer` |
//! | UART3 | left motor (`rt-motor`) | GPIO3_B5 (pin 16) / GPIO3_B6 (pin 18) | `uart3m1-xfer` |
//!
//! The servo is driven by the `FA address channel angle FE` protocol. The two
//! motors use the Lingkong V2.35 single-motor protocol and are wired as plain
//! TTL UARTs (TX to motor RX, motor TX to RX), so this is full-duplex with no
//! RS485 direction control; commands are sent and responses read back the same
//! way as the Python reference scripts.

use core::{
    ptr::NonNull,
    sync::atomic::{AtomicUsize, Ordering},
};

use ax_rt::{rt_output_write, rt_sleep};
use mmio_api::{MapError, MmioRaw};

const CRU_BASE: usize = 0xfd7c_0000;
const CRU_SIZE: usize = 0x1000;
const IOC_BASE: usize = 0xfd5f_0000;
const IOC_SIZE: usize = 0x10_000;

// DW APB UART register offsets, identical for every RK3588 UART.
const THR: usize = 0x00; // also RBR on read
const DLL: usize = 0x00; // with DLAB set, same offset as THR/RBR
const IER: usize = 0x04;
const DLH: usize = 0x04; // with DLAB set, same offset as IER
const FCR: usize = 0x08;
const LCR: usize = 0x0c;
const MCR: usize = 0x10;
const LSR: usize = 0x14;
const SRR: usize = 0x88;
const LCR_DLAB: u32 = 0x80;
const LCR_WLEN8: u32 = 0x03;
#[cfg(feature = "rt-motor")]
const LSR_RDR: u32 = 0x01;
const LSR_THRE: u32 = 0x20;
const UART_TX_POLL_MAX: u32 = 100_000;

// 24 MHz oscillator parent: divisor = 24 MHz / (16 * baud).
#[cfg(feature = "rt-uart")]
const DIV_9600: u32 = 156; // 24e6 / (16 * 156) ~= 9615 baud
#[cfg(feature = "rt-motor")]
const DIV_115200: u32 = 13; // 24e6 / (16 * 13) ~= 115385 baud

#[cfg(feature = "rt-uart")]
const SERVO_STEP_PERIOD_NANOS: u64 = 50_000_000;

#[cfg(feature = "rt-uart")]
const SERVO_PHYSICAL_RANGE_DEGREES: u16 = 270;
#[cfg(feature = "rt-uart")]
const SERVO_MIN_DEGREES: u16 = 60;
#[cfg(feature = "rt-uart")]
const SERVO_MAX_DEGREES: u16 = 210;
#[cfg(feature = "rt-uart")]
const SERVO_STEP_DEGREES: u16 = 2;

/// A reserved-core UART port: base/size for ioremap, a virt slot the host-side
/// setup fills in, and the clock/pinmux/baud config needed to bring it up.
#[derive(Clone, Copy)]
struct UartPortConfig {
    name: &'static str,
    base: usize,
    size: usize,
    virt: &'static AtomicUsize,
    gate_pclk: (usize, u32),
    gate_sclk: (usize, u32),
    /// `clksel_con` offset whose low 2 bits select the UART clock parent.
    clksel: usize,
    /// `GPIOx_IOMUX_SEL*` register and the write value (upper 16 bits are the
    /// write-enable mask, lower 16 the mux functions).
    mux_off: usize,
    mux_val: u32,
    /// Offset of the bank pull register inside the BUS_IOC ioremap, if it lives
    /// there. Only read for diagnostics; pull is never modified.
    pull_off: Option<usize>,
    baud_divisor: u32,
}

#[cfg(feature = "rt-uart")]
static UART7_VIRT: AtomicUsize = AtomicUsize::new(0);

#[cfg(feature = "rt-uart")]
const UART7_PORT: UartPortConfig = UartPortConfig {
    name: "UART7",
    base: 0xfeba_0000,
    size: 0x100,
    virt: &UART7_VIRT,
    gate_pclk: (0x830, 8),  // clkgate_con(12) bit8
    gate_sclk: (0x834, 15), // clkgate_con(13) bit15
    clksel: 0x3dc,          // clksel_con(55)
    mux_off: 0x802c,        // GPIO1_IOMUX_SEL1: B4/B5 -> uart7m2-xfer
    mux_val: 0x00ff_00aa,
    pull_off: Some(0x9114), // GPIO1 pull, B group
    baud_divisor: DIV_9600,
};

#[cfg(feature = "rt-motor")]
static UART3_VIRT: AtomicUsize = AtomicUsize::new(0);
#[cfg(feature = "rt-motor")]
static UART6_VIRT: AtomicUsize = AtomicUsize::new(0);

#[cfg(feature = "rt-motor")]
const UART3_PORT: UartPortConfig = UartPortConfig {
    name: "UART3",
    base: 0xfeb6_0000,
    size: 0x100,
    virt: &UART3_VIRT,
    gate_pclk: (0x830, 4), // clkgate_con(12) bit4
    gate_sclk: (0x834, 3), // clkgate_con(13) bit3
    clksel: 0x3bc,         // clksel_con(47)
    mux_off: 0x806c,       // GPIO3_IOMUX_SEL1: B5/B6 -> uart3m1-xfer
    mux_val: 0x0fff_0aa0,
    pull_off: None, // GPIO3 pull lives in the VCCIO3-5_IOC block
    baud_divisor: DIV_115200,
};

#[cfg(feature = "rt-motor")]
const UART6_PORT: UartPortConfig = UartPortConfig {
    name: "UART6",
    base: 0xfeb9_0000,
    size: 0x100,
    virt: &UART6_VIRT,
    gate_pclk: (0x830, 7),  // clkgate_con(12) bit7
    gate_sclk: (0x834, 12), // clkgate_con(13) bit12
    clksel: 0x3d4,          // clksel_con(53)
    mux_off: 0x8020,        // GPIO1_IOMUX_SEL_L: A0/A1 -> uart6m1-xfer
    mux_val: 0x00ff_00aa,
    pull_off: Some(0x9110), // GPIO1 pull, A group
    baud_divisor: DIV_115200,
};

struct Uart {
    mmio: MmioRaw,
}

impl Uart {
    fn r(&self, off: usize) -> u32 {
        self.mmio.read::<u32>(off)
    }

    fn w(&self, off: usize, value: u32) {
        self.mmio.write::<u32>(off, value);
    }

    fn init_8n1(&self, divisor: u32) {
        self.w(IER, 0);
        self.w(SRR, 0x07);
        self.w(MCR, 0);
        self.w(LCR, LCR_DLAB | LCR_WLEN8);
        // UART clock is selected to 24 MHz; divisor = 24 MHz / (16 * baud).
        self.w(DLL, divisor);
        self.w(DLH, 0);
        self.w(LCR, LCR_WLEN8);
        self.w(FCR, 0x07);
    }

    fn try_write_byte(&self, byte: u8) -> bool {
        for _ in 0..UART_TX_POLL_MAX {
            if self.r(LSR) & LSR_THRE != 0 {
                self.w(THR, byte as u32);
                return true;
            }
            core::hint::spin_loop();
        }
        false
    }

    fn try_write_bytes(&self, bytes: &[u8]) -> bool {
        for &byte in bytes {
            if !self.try_write_byte(byte) {
                return false;
            }
        }
        true
    }

    #[cfg(feature = "rt-motor")]
    fn try_read_byte(&self) -> Option<u8> {
        if self.r(LSR) & LSR_RDR != 0 {
            Some(self.r(THR) as u8)
        } else {
            None
        }
    }
}

fn cru_clear_bit(cru: &MmioRaw, off: usize, bit: u32) {
    cru.write::<u32>(off, 1u32 << (bit + 16));
}

/// Ungates the UART clocks, forces the 24 MHz oscillator parent, and muxes the
/// TX/RX pins. Reads the old mux (and pull, when in the same IOC block) for
/// diagnostics but never modifies pull.
fn setup_port(p: &UartPortConfig) -> Result<(), MapError> {
    let ioc = axklib::mmio::ioremap_raw((IOC_BASE as u64).into(), IOC_SIZE)?;
    let cru = axklib::mmio::ioremap_raw((CRU_BASE as u64).into(), CRU_SIZE)?;
    cru_clear_bit(&cru, p.gate_pclk.0, p.gate_pclk.1);
    cru_clear_bit(&cru, p.gate_sclk.0, p.gate_sclk.1);
    // Select the 24 MHz oscillator parent for a simple integer baud divisor.
    cru.write::<u32>(p.clksel, 0x0003_0002);

    let old_mux = ioc.read::<u32>(p.mux_off);
    ioc.write::<u32>(p.mux_off, p.mux_val);

    match p.pull_off {
        Some(off) => {
            let pull = ioc.read::<u32>(off);
            info!(
                "uart_rt: {} clocks ungated, clk_sel={:#010x}; mux[{:#x}] {old_mux:#010x}->{:#010x}, pull[{off:#x}]={pull:#010x}",
                p.name,
                cru.read::<u32>(p.clksel),
                p.mux_off,
                ioc.read::<u32>(p.mux_off),
            );
        }
        None => info!(
            "uart_rt: {} clocks ungated, clk_sel={:#010x}; mux[{:#x}] {old_mux:#010x}->{:#010x}",
            p.name,
            cru.read::<u32>(p.clksel),
            p.mux_off,
            ioc.read::<u32>(p.mux_off),
        ),
    }
    Ok(())
}

/// Host-side bring-up for every UART port enabled by the active features. Runs
/// before the reserved CPU starts its RT executor.
pub fn setup_host_side() {
    #[cfg(feature = "rt-uart")]
    setup_one(&UART7_PORT);
    #[cfg(feature = "rt-motor")]
    {
        setup_one(&UART3_PORT);
        setup_one(&UART6_PORT);
    }
}

fn setup_one(p: &UartPortConfig) {
    if let Err(err) = setup_port(p) {
        warn!("uart_rt: {} clock/pinmux bring-up failed: {err:?}", p.name);
        return;
    }
    match axklib::mmio::ioremap_raw((p.base as u64).into(), p.size) {
        Ok(mmio) => {
            let uart = Uart { mmio };
            uart.init_8n1(p.baud_divisor);
            let virt = uart.mmio.as_nonnull_ptr().as_ptr() as usize;
            p.virt.store(virt, Ordering::Release);
            info!(
                "uart_rt: {} mapped for RT core (virt={virt:#x}, LSR={:#010x}, LCR={:#010x})",
                p.name,
                uart.r(LSR),
                uart.r(LCR)
            );
        }
        Err(err) => warn!("uart_rt: ioremap {}@{:#x} failed: {err:?}", p.name, p.base),
    }
}

/// Rebuilds a [`Uart`] handle from the host-set virt slot. `None` until the
/// host-side bring-up for this port has completed.
fn uart_for(p: &UartPortConfig) -> Option<Uart> {
    let virt = p.virt.load(Ordering::Acquire);
    if virt == 0 {
        return None;
    }
    let mmio = unsafe {
        MmioRaw::new(
            (p.base as u64).into(),
            NonNull::new_unchecked(virt as *mut u8),
            p.size,
        )
    };
    Some(Uart { mmio })
}

/// Re-asserts a port's UART clocks from the RT side. The host may have its
/// clock state re-gate these clocks after `main()`'s bring-up, so device tasks
/// re-clear the gates (and the 24 MHz clksel) before touching the UART. The CRU
/// block is identity-mapped like the UART registers, so RT tasks can write it.
fn rt_assert_uart_clocks(p: &UartPortConfig) {
    // SAFETY: CRU is identity-mapped and only written by the calling RT task.
    let cru = unsafe {
        MmioRaw::new(
            (CRU_BASE as u64).into(),
            NonNull::new_unchecked(CRU_BASE as *mut u8),
            CRU_SIZE,
        )
    };
    cru_clear_bit(&cru, p.gate_pclk.0, p.gate_pclk.1);
    cru_clear_bit(&cru, p.gate_sclk.0, p.gate_sclk.1);
    cru.write::<u32>(p.clksel, 0x0003_0002);
}

/// Re-asserts the clocks and re-runs the controller init, for when a port's
/// LSR reads dead (clock re-gated after host bring-up).
fn rt_recover_port(p: &UartPortConfig) {
    rt_assert_uart_clocks(p);
    if let Some(uart) = uart_for(p) {
        uart.init_8n1(p.baud_divisor);
    }
}

/// Returns true when the port's UART controller looks alive (THR ready). Used
/// by device tasks to detect a re-gated clock and trigger recovery.
#[cfg(feature = "rt-motor")]
fn uart_alive(p: &UartPortConfig) -> bool {
    match uart_for(p) {
        Some(uart) => uart.r(LSR) & LSR_THRE != 0,
        None => false,
    }
}

fn hex_digit(nib: u8) -> u8 {
    if nib < 10 {
        b'0' + nib
    } else {
        b'a' + (nib - 10)
    }
}

#[cfg(feature = "rt-uart")]
fn rt_write_hex32(value: u32) {
    let mut buf = [b'0'; 10];
    buf[1] = b'x';
    for (i, slot) in buf[2..].iter_mut().enumerate() {
        let nib = ((value >> ((7 - i) * 4)) & 0xf) as u8;
        *slot = hex_digit(nib);
    }
    rt_output_write(&buf);
}

#[cfg(feature = "rt-motor")]
fn rt_write_hex_bytes(data: &[u8]) {
    for &byte in data {
        rt_output_write(&[hex_digit(byte >> 4), hex_digit(byte & 0xf)]);
    }
}

// ========================================================================
// LU9685 servo on UART7 (feature rt-uart)
// ========================================================================

#[cfg(feature = "rt-uart")]
const LU9685_UART_DEVICES: [Lu9685UartConfig; 1] = [Lu9685UartConfig {
    name: "LU9685@UART7",
    address: 0x00,
    channels: &[1, 2],
}];

#[cfg(feature = "rt-uart")]
#[derive(Clone, Copy)]
struct Lu9685UartConfig {
    name: &'static str,
    address: u8,
    channels: &'static [u8],
}

#[cfg(feature = "rt-uart")]
pub fn uart_task() -> ! {
    // Wait for the host's UART bring-up (the RT core starts before `main()`
    // finishes setup). One-time boot wait; short poll so it stays out of the way
    // of future higher-rate tasks.
    while !crate::realtime::rt_devices_ready() {
        rt_sleep(1_000_000);
    }
    let mut motion = ServoMotion::new(SERVO_MIN_DEGREES, SERVO_MAX_DEGREES, SERVO_STEP_DEGREES);
    loop {
        let Some(uart) = uart_for(&UART7_PORT) else {
            rt_sleep(100_000_000);
            continue;
        };

        // If the runtime re-gated UART7's clocks after host bring-up, LSR reads
        // dead; re-assert the clocks, re-init, and retry next iteration.
        if uart.r(LSR) & LSR_THRE == 0 {
            rt_recover_port(&UART7_PORT);
            continue;
        }

        let physical_degrees = motion.next_position();
        for device in LU9685_UART_DEVICES {
            send_servo_positions(&uart, &device, physical_degrees);
        }

        rt_sleep(SERVO_STEP_PERIOD_NANOS);
    }
}

#[cfg(feature = "rt-uart")]
struct ServoMotion {
    position: u16,
    min: u16,
    max: u16,
    step: u16,
    ascending: bool,
}

#[cfg(feature = "rt-uart")]
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

#[cfg(feature = "rt-uart")]
fn send_servo_positions(uart: &Uart, device: &Lu9685UartConfig, physical_degrees: u16) {
    let raw_angle = physical_to_lu9685_angle(physical_degrees);
    for &channel in device.channels {
        if !send_lu9685_angle(uart, device.address, channel, raw_angle) {
            rt_output_write(b"RT UART7 LU9685 TX not ready LSR=");
            rt_write_hex32(uart.r(LSR));
            rt_output_write(b"\n");
            return;
        }
    }

    rt_output_write(b"RT UART7 ");
    rt_output_write(device.name.as_bytes());
    rt_output_write(b" set physical=");
    ax_rt::rt_output_write_decimal(physical_degrees as u64);
    rt_output_write(b" raw=");
    ax_rt::rt_output_write_decimal(raw_angle as u64);
    rt_output_write(b"\n");
}

#[cfg(feature = "rt-uart")]
fn send_lu9685_angle(uart: &Uart, address: u8, channel: u8, raw_angle: u8) -> bool {
    uart.try_write_bytes(&[0xfa, address, channel, raw_angle, 0xfe])
}

#[cfg(feature = "rt-uart")]
fn physical_to_lu9685_angle(physical_degrees: u16) -> u8 {
    ((physical_degrees * 180 + SERVO_PHYSICAL_RANGE_DEGREES / 2) / SERVO_PHYSICAL_RANGE_DEGREES)
        as u8
}

// ========================================================================
// Lingkong V2.35 motors on UART3 / UART6 (feature rt-motor)
// ========================================================================

#[cfg(feature = "rt-motor")]
const MOTOR_HEADER: u8 = 0x3E;
#[cfg(feature = "rt-motor")]
const CMD_MOTOR_OFF: u8 = 0x80;
#[cfg(feature = "rt-motor")]
const CMD_STOP: u8 = 0x81;
#[cfg(feature = "rt-motor")]
const CMD_MOTOR_ON: u8 = 0x88;
#[cfg(feature = "rt-motor")]
const CMD_STATUS_1: u8 = 0x9A;
#[cfg(feature = "rt-motor")]
const CMD_SPEED_CLOSED_LOOP: u8 = 0xA2;

#[cfg(feature = "rt-motor")]
const MOTOR_LEFT_ID: u8 = 1;
#[cfg(feature = "rt-motor")]
const MOTOR_RIGHT_ID: u8 = 2;
#[cfg(feature = "rt-motor")]
const MOTOR_RUN_SPEED_DPS: i32 = 90;
#[cfg(feature = "rt-motor")]
const MOTOR_RUN_DURATION_NANOS: u64 = 2_000_000_000;
#[cfg(feature = "rt-motor")]
const MOTOR_PAUSE_NANOS: u64 = 500_000_000;
#[cfg(feature = "rt-motor")]
const MOTOR_OFF_NANOS: u64 = 1_000_000_000;
#[cfg(feature = "rt-motor")]
const MOTOR_RX_READ_TRIES: u32 = 30;
#[cfg(feature = "rt-motor")]
const MOTOR_RX_POLL_INTERVAL_NANOS: u64 = 1_000_000;
/// Max bytes drained per poll round. Bounds how long a noisy RX line can keep
/// this task busy before it yields back to the RT core.
#[cfg(feature = "rt-motor")]
const MOTOR_RX_DRAIN_MAX: usize = 16;

#[cfg(feature = "rt-motor")]
#[derive(Clone, Copy)]
struct MotorConfig {
    port: &'static UartPortConfig,
    side: &'static str,
    motor_id: u8,
}

#[cfg(feature = "rt-motor")]
fn motor_checksum(data: &[u8]) -> u8 {
    data.iter().fold(0u8, |sum, &b| sum.wrapping_add(b))
}

/// Writes the V2.35 request frame `[0x3E, cmd, id, len, csum]` plus `payload +
/// csum(payload)` into `out` and returns the frame length.
#[cfg(feature = "rt-motor")]
fn motor_frame(cmd: u8, motor_id: u8, payload: &[u8], out: &mut [u8; 16]) -> usize {
    out[0] = MOTOR_HEADER;
    out[1] = cmd;
    out[2] = motor_id;
    out[3] = payload.len() as u8;
    out[4] = motor_checksum(&out[..4]);
    let mut len = 5;
    if !payload.is_empty() {
        out[len..len + payload.len()].copy_from_slice(payload);
        len += payload.len();
        out[len] = motor_checksum(payload);
        len += 1;
    }
    len
}

#[cfg(feature = "rt-motor")]
fn speed_payload(speed_dps: i32, out: &mut [u8; 4]) {
    // Protocol 0xA2 uses int32 little-endian with 0.01 dps/LSB.
    let raw = speed_dps.wrapping_mul(100);
    out[0] = raw as u8;
    out[1] = (raw >> 8) as u8;
    out[2] = (raw >> 16) as u8;
    out[3] = (raw >> 24) as u8;
}

/// Discards stale RX bytes before a new command, mirroring the Python scripts'
/// `reset_input_buffer()`.
#[cfg(feature = "rt-motor")]
fn drain_rx(uart: &Uart) {
    for _ in 0..64 {
        if uart.try_read_byte().is_none() {
            break;
        }
    }
}

/// Reads one V2.35 response frame for `expected_cmd`/`expected_id`. Yields
/// between polls so the RT scheduler keeps running other tasks; bytes
/// accumulate in the 16-byte UART FIFO while the task sleeps.
#[cfg(feature = "rt-motor")]
fn read_motor_response(
    uart: &Uart,
    expected_cmd: u8,
    expected_id: u8,
    buf: &mut [u8; 16],
) -> Result<usize, ()> {
    let mut len = 0usize;
    for _ in 0..MOTOR_RX_READ_TRIES {
        // Bound the bytes drained per round so a noisy/floating RX line can
        // never keep this task busy without yielding (cooperative scheduler).
        let mut drained = 0usize;
        while drained < MOTOR_RX_DRAIN_MAX {
            let Some(byte) = uart.try_read_byte() else {
                break;
            };
            drained += 1;
            if len == 0 && byte != MOTOR_HEADER {
                continue; // discard bus noise until the header
            }
            if len == buf.len() {
                return Err(()); // response overflows the buffer
            }
            buf[len] = byte;
            len += 1;
            if len < 4 {
                continue;
            }
            let data_len = buf[3] as usize;
            let frame_len = 5 + data_len + usize::from(data_len > 0);
            if len < frame_len {
                continue;
            }
            if motor_checksum(&buf[..4]) != buf[4]
                || buf[1] != expected_cmd
                || buf[2] != expected_id
            {
                return Err(());
            }
            if data_len > 0 && motor_checksum(&buf[5..5 + data_len]) != buf[frame_len - 1] {
                return Err(());
            }
            return Ok(data_len);
        }
        rt_sleep(MOTOR_RX_POLL_INTERVAL_NANOS);
    }
    Err(())
}

#[cfg(feature = "rt-motor")]
fn log_motor_line(m: &MotorConfig, label: &[u8], tail: &[u8]) {
    rt_output_write(b"RT motor ");
    rt_output_write(m.side.as_bytes());
    rt_output_write(b" ");
    rt_output_write(m.port.name.as_bytes());
    rt_output_write(b" id=");
    ax_rt::rt_output_write_decimal(m.motor_id as u64);
    rt_output_write(b" ");
    rt_output_write(label);
    rt_output_write(tail);
}

#[cfg(feature = "rt-motor")]
fn rt_write_i8(raw: u8) {
    let value = raw as i8;
    if value < 0 {
        rt_output_write(b"-");
        ax_rt::rt_output_write_decimal((value as i16).unsigned_abs() as u64);
    } else {
        ax_rt::rt_output_write_decimal(value as u64);
    }
}

#[cfg(feature = "rt-motor")]
fn rt_write_i16_le(raw: &[u8]) {
    let value = i16::from_le_bytes([raw[0], raw[1]]);
    if value < 0 {
        rt_output_write(b"-");
        ax_rt::rt_output_write_decimal((value as i32).unsigned_abs() as u64);
    } else {
        ax_rt::rt_output_write_decimal(value as u64);
    }
}

/// Sends a command and reads its reply, logging a short status line.
#[cfg(feature = "rt-motor")]
fn motor_transaction(m: &MotorConfig, cmd: u8, payload: &[u8], label: &[u8]) {
    let Some(uart) = uart_for(m.port) else {
        log_motor_line(m, b"port", b" not-ready\n");
        return;
    };
    let mut frame = [0u8; 16];
    let len = motor_frame(cmd, m.motor_id, payload, &mut frame);
    drain_rx(&uart);
    if !uart.try_write_bytes(&frame[..len]) {
        log_motor_line(m, label, b" tx-timeout\n");
        return;
    }
    let mut resp = [0u8; 16];
    match read_motor_response(&uart, cmd, m.motor_id, &mut resp) {
        Ok(0) => log_motor_line(m, label, b" ack\n"),
        Ok(n) => {
            log_motor_line(m, label, b" reply ");
            rt_write_hex_bytes(&resp[..n]);
            rt_output_write(b"\n");
        }
        Err(()) => log_motor_line(m, label, b" no-reply\n"),
    }
}

/// Reads status 1 (0x9A) and logs temperature / voltage / current raw values
/// (divide by 100 for volts / amps, matching the reference scripts).
#[cfg(feature = "rt-motor")]
fn motor_status(m: &MotorConfig) {
    let Some(uart) = uart_for(m.port) else {
        log_motor_line(m, b"port", b" not-ready\n");
        return;
    };
    let mut frame = [0u8; 16];
    let len = motor_frame(CMD_STATUS_1, m.motor_id, &[], &mut frame);
    drain_rx(&uart);
    if !uart.try_write_bytes(&frame[..len]) {
        log_motor_line(m, b"status", b" tx-timeout\n");
        return;
    }
    let mut resp = [0u8; 16];
    match read_motor_response(&uart, CMD_STATUS_1, m.motor_id, &mut resp) {
        Ok(7) => {
            // temp i8, voltage i16le, current i16le, motor_state, error_state
            log_motor_line(m, b"status", b" temp=");
            rt_write_i8(resp[0]);
            rt_output_write(b"C v100=");
            rt_write_i16_le(&resp[1..3]);
            rt_output_write(b" c100=");
            rt_write_i16_le(&resp[3..5]);
            rt_output_write(b" st=0x");
            rt_write_hex_bytes(&resp[5..6]);
            rt_output_write(b" err=0x");
            rt_write_hex_bytes(&resp[6..7]);
            rt_output_write(b"\n");
        }
        Ok(n) => {
            log_motor_line(m, b"status", b" reply ");
            rt_write_hex_bytes(&resp[..n]);
            rt_output_write(b"\n");
        }
        Err(()) => log_motor_line(m, b"status", b" no-reply\n"),
    }
}

/// Commands closed-loop speed and logs the reply's iq / measured speed /
/// encoder (status 2).
#[cfg(feature = "rt-motor")]
fn motor_set_speed(m: &MotorConfig, speed_dps: i32) {
    let Some(uart) = uart_for(m.port) else {
        log_motor_line(m, b"port", b" not-ready\n");
        return;
    };
    let mut payload = [0u8; 4];
    speed_payload(speed_dps, &mut payload);
    let mut frame = [0u8; 16];
    let len = motor_frame(CMD_SPEED_CLOSED_LOOP, m.motor_id, &payload, &mut frame);
    drain_rx(&uart);
    if !uart.try_write_bytes(&frame[..len]) {
        log_motor_line(m, b"speed", b" tx-timeout\n");
        return;
    }
    let mut resp = [0u8; 16];
    match read_motor_response(&uart, CMD_SPEED_CLOSED_LOOP, m.motor_id, &mut resp) {
        Ok(7) => {
            // temp i8, iq i16le, speed i16le (dps), encoder u16le
            log_motor_line(m, b"speed", b" ok iq=");
            rt_write_i16_le(&resp[1..3]);
            rt_output_write(b" dps=");
            rt_write_i16_le(&resp[3..5]);
            rt_output_write(b" enc=");
            let enc = u16::from_le_bytes([resp[5], resp[6]]);
            ax_rt::rt_output_write_decimal(enc as u64);
            rt_output_write(b"\n");
        }
        Ok(n) => {
            log_motor_line(m, b"speed", b" reply ");
            rt_write_hex_bytes(&resp[..n]);
            rt_output_write(b"\n");
        }
        Err(()) => log_motor_line(m, b"speed", b" no-reply\n"),
    }
}

/// Repeating safe test sequence: read status, enable both motors, run both at a
/// low speed, stop, power off, then pause and repeat.
#[cfg(feature = "rt-motor")]
pub fn motor_task() -> ! {
    const LEFT: MotorConfig = MotorConfig {
        port: &UART3_PORT,
        side: "LEFT",
        motor_id: MOTOR_LEFT_ID,
    };
    const RIGHT: MotorConfig = MotorConfig {
        port: &UART6_PORT,
        side: "RIGHT",
        motor_id: MOTOR_RIGHT_ID,
    };
    // Wait for the host's UART bring-up before touching either motor UART (the
    // RT core starts before `main()` finishes setup). One-time boot wait.
    while !crate::realtime::rt_devices_ready() {
        rt_sleep(1_000_000);
    }
    loop {
        // Re-assert any motor UART clocks the runtime re-gated after host
        // bring-up before running the cycle.
        if !uart_alive(&UART3_PORT) {
            rt_recover_port(&UART3_PORT);
        }
        if !uart_alive(&UART6_PORT) {
            rt_recover_port(&UART6_PORT);
        }
        motor_status(&LEFT);
        motor_status(&RIGHT);
        motor_transaction(&LEFT, CMD_MOTOR_ON, &[], b"enable");
        motor_transaction(&RIGHT, CMD_MOTOR_ON, &[], b"enable");
        motor_set_speed(&LEFT, MOTOR_RUN_SPEED_DPS);
        motor_set_speed(&RIGHT, MOTOR_RUN_SPEED_DPS);
        rt_sleep(MOTOR_RUN_DURATION_NANOS);
        motor_transaction(&LEFT, CMD_STOP, &[], b"stop");
        motor_transaction(&RIGHT, CMD_STOP, &[], b"stop");
        rt_sleep(MOTOR_PAUSE_NANOS);
        motor_transaction(&LEFT, CMD_MOTOR_OFF, &[], b"off");
        motor_transaction(&RIGHT, CMD_MOTOR_OFF, &[], b"off");
        rt_sleep(MOTOR_OFF_NANOS);
    }
}
