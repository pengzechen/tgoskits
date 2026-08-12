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

//! Reserved-core UART1 control for an LU9685 servo controller on the
//! OrangePi-5-Plus 40-pin header.
//!
//! GPIO1_B6/B7 are muxed as `uart1m1-xfer` in the RK3588 pinctrl data: B7 is TX
//! and B6 is RX. Connect B7 to the LU9685 UART RX pin and share ground between
//! the board, LU9685, and servo power supply.

use core::{
    ptr::NonNull,
    sync::atomic::{AtomicUsize, Ordering},
};

use ax_rt::{rt_output_write, rt_sleep};
use mmio_api::{MapError, MmioRaw};

const UART1_BASE: usize = 0xfeb4_0000;
const UART1_SIZE: usize = 0x100;

const CRU_BASE: usize = 0xfd7c_0000;
const CRU_SIZE: usize = 0x1000;
const GATE_PCLK_UART1: (usize, u32) = (0x830, 2); // clkgate_con(12) bit2
const GATE_SCLK_UART1: (usize, u32) = (0x830, 13); // clkgate_con(12) bit13
const CLKSEL_UART1_OFF: usize = 0x3ac; // clksel_con(43): UART1 clock select

const IOC_UART1_MUX_OFF: usize = 0x802c;
const IOC_UART1_PULL_OFF: usize = 0x9114;
const IOC_UART1_MUX_VAL: u32 = 0xff00_aa00;

const THR: usize = 0x00;
const DLL: usize = 0x00;
const IER: usize = 0x04;
const DLH: usize = 0x04;
const FCR: usize = 0x08;
const LCR: usize = 0x0c;
const MCR: usize = 0x10;
const LSR: usize = 0x14;
const SRR: usize = 0x88;

const LCR_DLAB: u32 = 0x80;
const LCR_WLEN8: u32 = 0x03;
const LSR_THRE: u32 = 0x20;
const UART_TX_POLL_MAX: u32 = 100_000;

const SERVO_STEP_PERIOD_NANOS: u64 = 50_000_000;

const SERVO_PHYSICAL_RANGE_DEGREES: u16 = 270;
const SERVO_MIN_DEGREES: u16 = 60;
const SERVO_MAX_DEGREES: u16 = 210;
const SERVO_STEP_DEGREES: u16 = 2;

static UART1_VIRT: AtomicUsize = AtomicUsize::new(0);

const UART1_PORT: UartPortConfig = UartPortConfig {
    name: "UART1",
    base: UART1_BASE,
    size: UART1_SIZE,
    virt: &UART1_VIRT,
};

const LU9685_UART_DEVICES: [Lu9685UartConfig; 1] = [Lu9685UartConfig {
    name: "LU9685@UART1",
    address: 0x00,
    channels: &[0, 2],
}];

#[derive(Clone, Copy)]
struct UartPortConfig {
    name: &'static str,
    base: usize,
    size: usize,
    virt: &'static AtomicUsize,
}

#[derive(Clone, Copy)]
struct Lu9685UartConfig {
    name: &'static str,
    address: u8,
    channels: &'static [u8],
}

struct Uart1 {
    mmio: MmioRaw,
}

impl Uart1 {
    fn r(&self, off: usize) -> u32 {
        self.mmio.read::<u32>(off)
    }

    fn w(&self, off: usize, value: u32) {
        self.mmio.write::<u32>(off, value);
    }

    fn init_9600_8n1(&self) {
        self.w(IER, 0);
        self.w(SRR, 0x07);
        self.w(MCR, 0);
        self.w(LCR, LCR_DLAB | LCR_WLEN8);
        // UART clock is selected to 24 MHz; 24 MHz / (16 * 156) ~= 9615 baud.
        self.w(DLL, 156);
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
}

fn cru_clear_bit(cru: &MmioRaw, off: usize, bit: u32) {
    cru.write::<u32>(off, 1u32 << (bit + 16));
}

fn setup_pinmux_and_clocks() -> Result<(), MapError> {
    let ioc = axklib::mmio::ioremap_raw((0xfd5f_0000u64).into(), 0x10_000)?;
    let cru = axklib::mmio::ioremap_raw((CRU_BASE as u64).into(), CRU_SIZE)?;
    cru_clear_bit(&cru, GATE_PCLK_UART1.0, GATE_PCLK_UART1.1);
    cru_clear_bit(&cru, GATE_SCLK_UART1.0, GATE_SCLK_UART1.1);
    // Select the 24 MHz oscillator parent for a simple integer baud divisor.
    cru.write::<u32>(CLKSEL_UART1_OFF, 0x0003_0002);

    let old_mux = ioc.read::<u32>(IOC_UART1_MUX_OFF);
    let old_pull = ioc.read::<u32>(IOC_UART1_PULL_OFF);
    ioc.write::<u32>(IOC_UART1_MUX_OFF, IOC_UART1_MUX_VAL);

    info!(
        "uart_rt: UART1 clocks ungated, clk_sel={:#010x}; mux[{IOC_UART1_MUX_OFF:#x}] {old_mux:#010x}->{:#010x}, pull[{IOC_UART1_PULL_OFF:#x}]={old_pull:#010x}",
        cru.read::<u32>(CLKSEL_UART1_OFF),
        ioc.read::<u32>(IOC_UART1_MUX_OFF),
    );
    Ok(())
}

pub fn setup_host_side() {
    if let Err(err) = setup_pinmux_and_clocks() {
        warn!("uart_rt: UART1 clock/pinmux bring-up failed: {err:?}");
    }
    match axklib::mmio::ioremap_raw((UART1_PORT.base as u64).into(), UART1_PORT.size) {
        Ok(mmio) => {
            let uart = Uart1 { mmio };
            uart.init_9600_8n1();
            let virt = uart.mmio.as_nonnull_ptr().as_ptr() as usize;
            UART1_PORT.virt.store(virt, Ordering::Release);
            info!(
                "uart_rt: {} mapped for RT core (virt={virt:#x}, LSR={:#010x}, LCR={:#010x})",
                UART1_PORT.name,
                uart.r(LSR),
                uart.r(LCR)
            );
        }
        Err(err) => warn!(
            "uart_rt: ioremap {}@{:#x} failed: {err:?}",
            UART1_PORT.name, UART1_PORT.base
        ),
    }
}

pub fn uart_task() -> ! {
    let mut motion = ServoMotion::new(SERVO_MIN_DEGREES, SERVO_MAX_DEGREES, SERVO_STEP_DEGREES);
    loop {
        let virt = UART1_PORT.virt.load(Ordering::Acquire);
        if virt == 0 {
            rt_sleep(100_000_000);
            continue;
        }

        let mmio = unsafe {
            MmioRaw::new(
                (UART1_PORT.base as u64).into(),
                NonNull::new_unchecked(virt as *mut u8),
                UART1_PORT.size,
            )
        };
        let uart = Uart1 { mmio };

        let physical_degrees = motion.next_position();
        for device in LU9685_UART_DEVICES {
            send_servo_positions(&uart, &device, physical_degrees);
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

fn send_servo_positions(uart: &Uart1, device: &Lu9685UartConfig, physical_degrees: u16) {
    let raw_angle = physical_to_lu9685_angle(physical_degrees);
    for &channel in device.channels {
        if !send_lu9685_angle(uart, device.address, channel, raw_angle) {
            rt_output_write(b"RT UART1 LU9685 TX not ready LSR=");
            rt_write_hex32(uart.r(LSR));
            rt_output_write(b"\n");
            return;
        }
    }

    rt_output_write(b"RT UART1 ");
    rt_output_write(device.name.as_bytes());
    rt_output_write(b" set physical=");
    ax_rt::rt_output_write_decimal(physical_degrees as u64);
    rt_output_write(b" raw=");
    ax_rt::rt_output_write_decimal(raw_angle as u64);
    rt_output_write(b"\n");
}

fn send_lu9685_angle(uart: &Uart1, address: u8, channel: u8, raw_angle: u8) -> bool {
    uart.try_write_bytes(&[0xfa, address, channel, raw_angle, 0xfe])
}

fn physical_to_lu9685_angle(physical_degrees: u16) -> u8 {
    ((physical_degrees * 180 + SERVO_PHYSICAL_RANGE_DEGREES / 2) / SERVO_PHYSICAL_RANGE_DEGREES)
        as u8
}

fn rt_write_hex32(value: u32) {
    let mut buf = [b'0'; 10];
    buf[1] = b'x';
    for (i, slot) in buf[2..].iter_mut().enumerate() {
        let nib = ((value >> ((7 - i) * 4)) & 0xf) as u8;
        *slot = if nib < 10 {
            b'0' + nib
        } else {
            b'a' + (nib - 10)
        };
    }
    rt_output_write(&buf);
}
