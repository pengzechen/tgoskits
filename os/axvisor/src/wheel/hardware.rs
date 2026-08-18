//! Deadline-bounded RT hardware adapters for the wheel controller.

use core::{
    ptr::NonNull,
    sync::atomic::{AtomicUsize, Ordering},
};

use ax_rt::{rt_delay_until, rt_monotonic_nanos, rt_output_write};
use mmio_api::{MapError, MmioRaw};

use super::{
    WheelController, WheelMeasurements, build_torque_control_frame, convert_mpu6050_sample,
    hip_servo_channels,
    imu::RawMpu6050Sample,
    model::RobotModel,
    motor::{MotorProtocolError, MotorState2, TORQUE_RESPONSE_COMMAND},
    params::HEIGHT_MAX,
    parse_state2_response,
    servo::{HipServoAngles, hip_angles_to_servo_degrees},
};

const CRU_BASE: usize = 0xfd7c_0000;
const CRU_SIZE: usize = 0x1000;
const IOC_BASE: usize = 0xfd5f_0000;
const IOC_SIZE: usize = 0x1_0000;

const I2C5_BASE: usize = 0xfead_0000;
const I2C5_SIZE: usize = 0x1000;
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
const MRXADDR_VALID0: u32 = 1 << 24;
const INT_MBTF: u32 = 1 << 2;
const INT_MBRF: u32 = 1 << 3;
const INT_START: u32 = 1 << 4;
const INT_STOP: u32 = 1 << 5;
const INT_NAKRCV: u32 = 1 << 6;
const INT_ALL: u32 = 0x7f;
const MPU6050_ADDR: u8 = 0x68;
const MPU6050_REG_ACCEL_XOUT_H: u8 = 0x3b;
const MPU6050_REG_PWR_MGMT_1: u8 = 0x6b;
const MPU6050_REG_SMPLRT_DIV: u8 = 0x19;
const MPU6050_REG_CONFIG: u8 = 0x1a;
const MPU6050_REG_GYRO_CONFIG: u8 = 0x1b;
const MPU6050_REG_ACCEL_CONFIG: u8 = 0x1c;

const CMD_MOTOR_ON: u8 = 0x88;

const I2C_EVENT_TIMEOUT_NANOS: u64 = 2_000_000;
const UART_RESPONSE_TIMEOUT_NANOS: u64 = 3_000_000;
const UART_TX_POLL_MAX: u32 = 100_000;
const UART_RX_DRAIN_MAX: usize = 16;
const DIAG_EVERY_FAILURES: u32 = 100;
const MOTOR_ENABLE_RETRY_NANOS: u64 = 10_000_000;
const HIP_SERVO_SETTLE_NANOS: u64 = 15_000_000_000;

static I2C5_VIRT: AtomicUsize = AtomicUsize::new(0);
static UART7_VIRT: AtomicUsize = AtomicUsize::new(0);
static UART3_VIRT: AtomicUsize = AtomicUsize::new(0);
static UART6_VIRT: AtomicUsize = AtomicUsize::new(0);

pub fn setup_host_side() {
    setup_i2c5();
    setup_uart(&UART7_PORT);
    setup_uart(&UART3_PORT);
    setup_uart(&UART6_PORT);
}

pub fn wheel_task() -> ! {
    while !crate::realtime::rt_devices_ready() {
        rt_delay_until(rt_monotonic_nanos().saturating_add(1_000_000));
    }

    rt_output_write(b"wheel-control: RT task started, positioning hip servos\n");
    wait_for_hip_servos_positioned();
    rt_output_write(b"wheel-control: waiting for hip servo settle ns=");
    ax_rt::rt_output_write_decimal(HIP_SERVO_SETTLE_NANOS);
    rt_output_write(b"\n");
    rt_delay_until(rt_monotonic_nanos().saturating_add(HIP_SERVO_SETTLE_NANOS));
    rt_output_write(b"wheel-control: hip servos positioned, enabling motors\n");
    wait_for_motors_enabled();

    let mut controller = WheelController::default();
    let mut right_state = MotorState2::default();
    let mut left_state = MotorState2::default();
    let mut initialized = false;
    let mut failure_count = 0u32;
    let mut deadline_miss_count = 0u32;
    let mut motor_failure_count = 0u32;
    let mut cycle_count = 0u32;
    let mut right_motor_response_count = 0u32;
    let mut left_motor_response_count = 0u32;
    let mut next_deadline = rt_monotonic_nanos();
    loop {
        next_deadline = next_deadline.saturating_add(WheelController::PERIOD_NANOS);
        let mut timing = CycleTiming::default();
        let read_start = rt_monotonic_nanos();
        let cycle = read_wheel_measurements(right_state, left_state)
            .inspect(|measurements| {
                right_state = measurements.right_motor;
                left_state = measurements.left_motor;
            })
            .and_then(|measurements| {
                timing.imu_nanos = rt_monotonic_nanos().saturating_sub(read_start);
                let controller_start = rt_monotonic_nanos();
                controller
                    .step(measurements)
                    .map_err(|_| WheelFailure::Controller)
                    .inspect(|_| {
                        timing.controller_nanos =
                            rt_monotonic_nanos().saturating_sub(controller_start);
                    })
            });

        match cycle {
            Ok(output) => {
                failure_count = 0;
                cycle_count = cycle_count.saturating_add(1);
                let right_motor_start = rt_monotonic_nanos();
                let right_motor_result = send_motor_torque(
                    &UART6_PORT,
                    2,
                    output.motor_command.right_iq,
                    &mut right_state,
                );
                timing.right_motor_nanos = rt_monotonic_nanos().saturating_sub(right_motor_start);
                if matches!(right_motor_result, MotorTransactionResult::Ok) {
                    right_motor_response_count = right_motor_response_count.saturating_add(1);
                }
                let left_motor_start = rt_monotonic_nanos();
                let left_motor_result = send_motor_torque(
                    &UART3_PORT,
                    1,
                    output.motor_command.left_iq,
                    &mut left_state,
                );
                timing.left_motor_nanos = rt_monotonic_nanos().saturating_sub(left_motor_start);
                if matches!(left_motor_result, MotorTransactionResult::Ok) {
                    left_motor_response_count = left_motor_response_count.saturating_add(1);
                }
                if let Some(reason) = right_motor_result.failure_reason() {
                    motor_failure_count = motor_failure_count.saturating_add(1);
                    report_motor_failure(&UART6_PORT, reason, motor_failure_count);
                }
                if let Some(reason) = left_motor_result.failure_reason() {
                    motor_failure_count = motor_failure_count.saturating_add(1);
                    report_motor_failure(&UART3_PORT, reason, motor_failure_count);
                }
                if right_motor_result.needs_reenable() || left_motor_result.needs_reenable() {
                    rt_output_write(
                        b"wheel-control: motor UART transport failed, re-enabling motors\n",
                    );
                    wait_for_motors_enabled();
                    controller.reset();
                    right_state = MotorState2::default();
                    left_state = MotorState2::default();
                }
                if !initialized {
                    rt_output_write(b"wheel-control: running 8ms closed-loop task\n");
                    initialized = true;
                }
                report_balance_state(
                    cycle_count,
                    output,
                    right_state,
                    left_state,
                    right_motor_response_count,
                    left_motor_response_count,
                );
            }
            Err(reason) => {
                failure_count = failure_count.saturating_add(1);
                report_failure(reason, failure_count);
                let _ = send_motor_torque(&UART6_PORT, 2, 0, &mut right_state);
                let _ = send_motor_torque(&UART3_PORT, 1, 0, &mut left_state);
            }
        }

        let now = rt_monotonic_nanos();
        if next_deadline > now {
            rt_delay_until(next_deadline);
        } else {
            deadline_miss_count = deadline_miss_count.saturating_add(1);
            report_deadline_miss(
                deadline_miss_count,
                now.saturating_sub(next_deadline),
                timing,
            );
            next_deadline = now;
            ax_rt::rt_yield_now();
        }
    }
}

#[derive(Clone, Copy)]
enum WheelFailure {
    Mpu6050,
    Controller,
}

#[derive(Clone, Copy, Default)]
struct CycleTiming {
    imu_nanos: u64,
    controller_nanos: u64,
    servo_nanos: u64,
    right_motor_nanos: u64,
    left_motor_nanos: u64,
}

#[derive(Clone, Copy)]
enum MotorTransactionResult {
    Ok,
    NoResponse,
    BadResponse(MotorProtocolError),
    TransportFailed,
}

impl MotorTransactionResult {
    fn needs_reenable(self) -> bool {
        matches!(self, Self::TransportFailed)
    }

    fn failure_reason(self) -> Option<MotorFailureReason> {
        match self {
            Self::Ok => None,
            Self::NoResponse => Some(MotorFailureReason::NoResponse),
            Self::BadResponse(err) => Some(MotorFailureReason::BadResponse(err)),
            Self::TransportFailed => Some(MotorFailureReason::TransportFailed),
        }
    }
}

#[derive(Clone, Copy)]
enum MotorFailureReason {
    NoResponse,
    BadResponse(MotorProtocolError),
    TransportFailed,
}

fn read_wheel_measurements(
    right_state: MotorState2,
    left_state: MotorState2,
) -> Result<WheelMeasurements, WheelFailure> {
    let raw = read_mpu6050_sample().map_err(|_| WheelFailure::Mpu6050)?;
    Ok(WheelMeasurements {
        imu: convert_mpu6050_sample(raw),
        right_motor: right_state,
        left_motor: left_state,
    })
}

fn wait_for_motors_enabled() {
    let mut retry_count = 0u32;
    loop {
        let right_enabled = send_motor_enable(&UART6_PORT, 2);
        let left_enabled = send_motor_enable(&UART3_PORT, 1);
        if right_enabled && left_enabled {
            if retry_count != 0 {
                rt_output_write(b"wheel-control: motors enabled after retries count=");
                ax_rt::rt_output_write_decimal(retry_count as u64);
                rt_output_write(b"\n");
            }
            return;
        }
        retry_count = retry_count.saturating_add(1);
        if retry_count == 1 || retry_count % DIAG_EVERY_FAILURES == 0 {
            rt_output_write(b"wheel-control: waiting for motor enable retry=");
            ax_rt::rt_output_write_decimal(retry_count as u64);
            rt_output_write(b"\n");
        }
        rt_recover_uart(&UART6_PORT);
        rt_recover_uart(&UART3_PORT);
        rt_delay_until(rt_monotonic_nanos().saturating_add(MOTOR_ENABLE_RETRY_NANOS));
    }
}

fn wait_for_hip_servos_positioned() {
    let angles = initial_hip_servo_angles();
    let mut retry_count = 0u32;
    loop {
        if send_hip_servos(angles) {
            if retry_count != 0 {
                rt_output_write(b"wheel-control: hip servos positioned after retries count=");
                ax_rt::rt_output_write_decimal(retry_count as u64);
                rt_output_write(b"\n");
            }
            return;
        }
        retry_count = retry_count.saturating_add(1);
        if retry_count == 1 || retry_count % DIAG_EVERY_FAILURES == 0 {
            rt_output_write(b"wheel-control: waiting for hip servo positioning retry=");
            ax_rt::rt_output_write_decimal(retry_count as u64);
            rt_output_write(b"\n");
        }
        rt_recover_uart(&UART7_PORT);
        rt_delay_until(rt_monotonic_nanos().saturating_add(MOTOR_ENABLE_RETRY_NANOS));
    }
}

fn initial_hip_servo_angles() -> HipServoAngles {
    let mut model = RobotModel::default();
    let _ = model.update_height_roll(HEIGHT_MAX, 0.0);
    let [right_hip, left_hip] = model.hip_angles();
    hip_angles_to_servo_degrees(right_hip, left_hip)
}

struct I2c {
    mmio: MmioRaw,
}

impl I2c {
    fn r(&self, off: usize) -> u32 {
        self.mmio.read::<u32>(off)
    }

    fn w(&self, off: usize, val: u32) {
        self.mmio.write::<u32>(off, val);
    }

    fn init(&self) {
        self.w(REG_CLKDIV, (124 << 16) | 124);
        self.w(REG_CON, 0);
        self.w(REG_IEN, 0);
        self.w(REG_IPD, INT_ALL);
    }

    fn wait_ipd(&self, mask: u32) -> Result<(), ()> {
        let deadline = rt_monotonic_nanos().saturating_add(I2C_EVENT_TIMEOUT_NANOS);
        while rt_monotonic_nanos() <= deadline {
            let ipd = self.r(REG_IPD);
            if ipd & mask != 0 {
                self.w(REG_IPD, mask);
                return Ok(());
            }
            if ipd & INT_NAKRCV != 0 {
                self.w(REG_IPD, INT_NAKRCV);
                return Err(());
            }
            core::hint::spin_loop();
        }
        Err(())
    }

    fn write_reg(&self, chip: u8, reg: u8, value: u8) -> Result<(), ()> {
        self.w(REG_IPD, INT_ALL);
        self.w(REG_CON, CON_EN | CON_START);
        self.w(REG_IEN, INT_START);
        self.wait_ipd(INT_START)?;
        self.w(
            TXDATA_BASE,
            ((chip as u32) << 1) | ((reg as u32) << 8) | ((value as u32) << 16),
        );
        self.w(REG_CON, CON_EN | (MODE_TX << 1));
        self.w(REG_MTXCNT, 3);
        self.w(REG_IEN, INT_MBTF | INT_NAKRCV);
        let result = self.wait_ipd(INT_MBTF);
        let _ = self.stop();
        self.w(REG_CON, 0);
        result
    }

    fn read_regs(&self, chip: u8, reg: u8, out: &mut [u8]) -> Result<(), ()> {
        self.w(REG_IPD, INT_ALL);
        self.w(REG_CON, CON_EN | CON_START);
        self.w(REG_IEN, INT_START);
        self.wait_ipd(INT_START)?;
        self.w(REG_MRXADDR, (((chip as u32) << 1) | 1) | MRXADDR_VALID0);
        self.w(REG_MRXRADDR, (reg as u32) | MRXADDR_VALID0);
        self.w(REG_CON, CON_EN | CON_LASTACK | (MODE_TRX << 1));
        self.w(REG_MRXCNT, out.len() as u32);
        self.w(REG_IEN, INT_MBRF | INT_NAKRCV);
        let result = self.wait_ipd(INT_MBRF);
        if result.is_ok() {
            for (index, byte) in out.iter_mut().enumerate() {
                let word = self.r(RXDATA_BASE + (index / 4) * 4);
                *byte = ((word >> ((index % 4) * 8)) & 0xff) as u8;
            }
        }
        let _ = self.stop();
        self.w(REG_CON, 0);
        result
    }

    fn stop(&self) -> Result<(), ()> {
        self.w(REG_IPD, INT_ALL);
        self.w(REG_CON, CON_EN | CON_STOP);
        self.w(REG_IEN, INT_STOP);
        self.wait_ipd(INT_STOP)
    }
}

fn read_mpu6050_sample() -> Result<RawMpu6050Sample, ()> {
    let i2c = i2c5()?;
    let mut data = [0u8; 14];
    i2c.read_regs(MPU6050_ADDR, MPU6050_REG_ACCEL_XOUT_H, &mut data)?;
    Ok(RawMpu6050Sample {
        accel: [
            i16::from_be_bytes([data[0], data[1]]),
            i16::from_be_bytes([data[2], data[3]]),
            i16::from_be_bytes([data[4], data[5]]),
        ],
        temperature_raw: i16::from_be_bytes([data[6], data[7]]),
        gyro: [
            i16::from_be_bytes([data[8], data[9]]),
            i16::from_be_bytes([data[10], data[11]]),
            i16::from_be_bytes([data[12], data[13]]),
        ],
    })
}

fn i2c5() -> Result<I2c, ()> {
    let virt = I2C5_VIRT.load(Ordering::Acquire);
    if virt == 0 {
        return Err(());
    }
    let mmio = unsafe {
        MmioRaw::new(
            (I2C5_BASE as u64).into(),
            NonNull::new_unchecked(virt as *mut u8),
            I2C5_SIZE,
        )
    };
    Ok(I2c { mmio })
}

fn setup_i2c5() {
    if let Err(err) = setup_i2c5_pinmux() {
        warn!("wheel: I2C5 pinmux setup failed: {err:?}");
    }
    match axklib::mmio::ioremap_raw((I2C5_BASE as u64).into(), I2C5_SIZE) {
        Ok(mmio) => {
            let i2c = I2c { mmio };
            i2c.init();
            let _ = i2c.write_reg(MPU6050_ADDR, MPU6050_REG_PWR_MGMT_1, 0x01);
            let _ = i2c.write_reg(MPU6050_ADDR, MPU6050_REG_SMPLRT_DIV, 9);
            let _ = i2c.write_reg(MPU6050_ADDR, MPU6050_REG_CONFIG, 0x03);
            let _ = i2c.write_reg(MPU6050_ADDR, MPU6050_REG_GYRO_CONFIG, 0x00);
            let _ = i2c.write_reg(MPU6050_ADDR, MPU6050_REG_ACCEL_CONFIG, 0x00);
            I2C5_VIRT.store(
                i2c.mmio.as_nonnull_ptr().as_ptr() as usize,
                Ordering::Release,
            );
            info!("wheel: I2C5 MPU6050 mapped for RT wheel task");
        }
        Err(err) => warn!("wheel: ioremap I2C5 failed: {err:?}"),
    }
}

fn setup_i2c5_pinmux() -> Result<(), MapError> {
    let cru = axklib::mmio::ioremap_raw((CRU_BASE as u64).into(), CRU_SIZE)?;
    let ioc = axklib::mmio::ioremap_raw((IOC_BASE as u64).into(), IOC_SIZE)?;
    cru_clear_bit(&cru, 0x828, 12);
    cru_clear_bit(&cru, 0x82c, 4);
    cru_clear_bit(&cru, 0xa2c, 4);
    cru_clear_bit(&cru, 0xa28, 12);
    cru_clear_bit(&cru, 0x800, 1);
    cru_clear_bit(&cru, 0x800, 3);
    cru.write::<u32>(0x398, 1 << (10 + 16));
    ioc.write::<u32>(0x802c, 0xff00_9900);
    ioc.write::<u32>(0x9114, 0xf000_f000);
    Ok(())
}

#[derive(Clone, Copy)]
struct UartPort {
    name: &'static str,
    base: usize,
    virt: &'static AtomicUsize,
    gate_pclk: (usize, u32),
    gate_sclk: (usize, u32),
    clksel: usize,
    mux_off: usize,
    mux_val: u32,
    divisor: u32,
}

const UART7_PORT: UartPort = UartPort {
    name: "UART7",
    base: 0xfeba_0000,
    virt: &UART7_VIRT,
    gate_pclk: (0x830, 8),
    gate_sclk: (0x834, 15),
    clksel: 0x3dc,
    mux_off: 0x802c,
    mux_val: 0x00ff_00aa,
    divisor: 156,
};
const UART3_PORT: UartPort = UartPort {
    name: "UART3",
    base: 0xfeb6_0000,
    virt: &UART3_VIRT,
    gate_pclk: (0x830, 4),
    gate_sclk: (0x834, 3),
    clksel: 0x3bc,
    mux_off: 0x806c,
    mux_val: 0x0fff_0aa0,
    divisor: 13,
};
const UART6_PORT: UartPort = UartPort {
    name: "UART6",
    base: 0xfeb9_0000,
    virt: &UART6_VIRT,
    gate_pclk: (0x830, 7),
    gate_sclk: (0x834, 12),
    clksel: 0x3d4,
    mux_off: 0x8020,
    mux_val: 0x00ff_00aa,
    divisor: 13,
};

const UART_SIZE: usize = 0x100;
const THR: usize = 0x00;
const IER: usize = 0x04;
const FCR: usize = 0x08;
const LCR: usize = 0x0c;
const MCR: usize = 0x10;
const LSR: usize = 0x14;
const SRR: usize = 0x88;
const LCR_DLAB: u32 = 0x80;
const LCR_WLEN8: u32 = 0x03;
const LSR_RDR: u32 = 0x01;
const LSR_THRE: u32 = 0x20;

struct Uart {
    mmio: MmioRaw,
}

#[derive(Clone, Copy)]
struct UartTxError {
    byte_index: usize,
    lsr: u32,
}

impl Uart {
    fn r(&self, off: usize) -> u32 {
        self.mmio.read::<u32>(off)
    }

    fn w(&self, off: usize, value: u32) {
        self.mmio.write::<u32>(off, value);
    }

    fn init(&self, divisor: u32) {
        self.w(IER, 0);
        self.w(SRR, 0x07);
        self.w(MCR, 0);
        self.w(LCR, LCR_DLAB | LCR_WLEN8);
        self.w(THR, divisor);
        self.w(IER, 0);
        self.w(LCR, LCR_WLEN8);
        self.w(FCR, 0x07);
    }

    fn write_byte(&self, byte: u8) -> Result<(), u32> {
        for _ in 0..UART_TX_POLL_MAX {
            if self.r(LSR) & LSR_THRE != 0 {
                self.w(THR, byte as u32);
                return Ok(());
            }
            core::hint::spin_loop();
        }
        Err(self.r(LSR))
    }

    fn write_bytes(&self, bytes: &[u8]) -> Result<(), UartTxError> {
        for (byte_index, &byte) in bytes.iter().enumerate() {
            self.write_byte(byte)
                .map_err(|lsr| UartTxError { byte_index, lsr })?;
        }
        Ok(())
    }

    fn read_byte(&self) -> Option<u8> {
        if self.r(LSR) & LSR_RDR != 0 {
            Some(self.r(THR) as u8)
        } else {
            None
        }
    }
}

fn send_hip_servos(angles: HipServoAngles) -> bool {
    let Some(uart) = uart_for(&UART7_PORT) else {
        return false;
    };
    if uart.r(LSR) & LSR_THRE == 0 {
        rt_recover_uart(&UART7_PORT);
        return false;
    }
    let [left, right] = hip_servo_channels();
    if uart
        .write_bytes(&[0xfa, 0x00, left, angles.left_degrees, 0xfe])
        .is_err()
    {
        rt_recover_uart(&UART7_PORT);
        return false;
    }
    if uart
        .write_bytes(&[0xfa, 0x00, right, angles.right_degrees, 0xfe])
        .is_err()
    {
        rt_recover_uart(&UART7_PORT);
        return false;
    }
    true
}

fn send_motor_torque(
    port: &UartPort,
    motor_id: u8,
    iq: i16,
    state: &mut MotorState2,
) -> MotorTransactionResult {
    let Some(uart) = uart_for(port) else {
        return MotorTransactionResult::TransportFailed;
    };
    if !ensure_uart_alive(port, &uart) {
        return MotorTransactionResult::TransportFailed;
    }
    drain_rx(&uart);
    if uart
        .write_bytes(&build_torque_control_frame(motor_id, iq))
        .is_err()
    {
        rt_recover_uart(port);
        return MotorTransactionResult::TransportFailed;
    }
    let mut response = [0u8; 13];
    if read_response(&uart, &mut response).is_err() {
        return MotorTransactionResult::NoResponse;
    }
    match parse_state2_response(motor_id, TORQUE_RESPONSE_COMMAND, &response) {
        Ok(next_state) => {
            *state = next_state;
            MotorTransactionResult::Ok
        }
        Err(err) => MotorTransactionResult::BadResponse(err),
    }
}

fn send_motor_enable(port: &UartPort, motor_id: u8) -> bool {
    let Some(uart) = uart_for(port) else {
        rt_output_write(b"wheel-control: motor UART not ready during enable port=");
        rt_output_write(port.name.as_bytes());
        rt_output_write(b"\n");
        return false;
    };
    if !ensure_uart_alive(port, &uart) {
        rt_output_write(b"wheel-control: motor enable UART dead port=");
        rt_output_write(port.name.as_bytes());
        rt_output_write(b"\n");
        return false;
    }
    drain_rx(&uart);
    if let Err(err) = uart.write_bytes(&build_empty_motor_frame(CMD_MOTOR_ON, motor_id)) {
        rt_output_write(b"wheel-control: motor enable TX failed port=");
        rt_output_write(port.name.as_bytes());
        rt_output_write(b" byte=");
        ax_rt::rt_output_write_decimal(err.byte_index as u64);
        rt_output_write(b" lsr=");
        ax_rt::rt_output_write_decimal(err.lsr as u64);
        rt_output_write(b"\n");
        rt_recover_uart(port);
        return false;
    }
    true
}

fn read_response(uart: &Uart, response: &mut [u8]) -> Result<(), ()> {
    let mut len = 0;
    let deadline = rt_monotonic_nanos().saturating_add(UART_RESPONSE_TIMEOUT_NANOS);
    while rt_monotonic_nanos() <= deadline {
        let mut drained = 0;
        while drained < UART_RX_DRAIN_MAX {
            let Some(byte) = uart.read_byte() else {
                break;
            };
            drained += 1;
            if len == 0 && byte != 0x3e {
                continue;
            }
            if len == response.len() {
                return Err(());
            }
            response[len] = byte;
            len += 1;
            if len == response.len() {
                return Ok(());
            }
        }
        core::hint::spin_loop();
    }
    Err(())
}

fn build_empty_motor_frame(command: u8, motor_id: u8) -> [u8; 5] {
    [
        0x3e,
        command,
        motor_id,
        0,
        0x3e_u8.wrapping_add(command).wrapping_add(motor_id),
    ]
}

fn report_failure(reason: WheelFailure, count: u32) {
    if count != 1 && count % DIAG_EVERY_FAILURES != 0 {
        return;
    }
    rt_output_write(b"wheel-control: cycle failed reason=");
    match reason {
        WheelFailure::Mpu6050 => rt_output_write(b"mpu6050"),
        WheelFailure::Controller => rt_output_write(b"controller"),
    }
    rt_output_write(b" count=");
    ax_rt::rt_output_write_decimal(count as u64);
    rt_output_write(b"\n");
}

fn report_motor_failure(port: &UartPort, reason: MotorFailureReason, count: u32) {
    if count != 1 && count % DIAG_EVERY_FAILURES != 0 {
        return;
    }
    rt_output_write(b"wheel-control: motor transaction failed port=");
    rt_output_write(port.name.as_bytes());
    rt_output_write(b" reason=");
    match reason {
        MotorFailureReason::NoResponse => rt_output_write(b"no-response"),
        MotorFailureReason::TransportFailed => rt_output_write(b"transport"),
        MotorFailureReason::BadResponse(err) => {
            rt_output_write(b"bad-response-");
            report_motor_protocol_error(err);
        }
    }
    rt_output_write(b" count=");
    ax_rt::rt_output_write_decimal(count as u64);
    rt_output_write(b"\n");
}

fn report_motor_protocol_error(err: MotorProtocolError) {
    match err {
        MotorProtocolError::BadFrame => rt_output_write(b"bad-frame"),
        MotorProtocolError::HeaderChecksum => rt_output_write(b"header-checksum"),
        MotorProtocolError::DataChecksum => rt_output_write(b"data-checksum"),
        MotorProtocolError::WrongCommand => rt_output_write(b"wrong-command"),
        MotorProtocolError::WrongMotorId => rt_output_write(b"wrong-id"),
        MotorProtocolError::UnexpectedLength => rt_output_write(b"unexpected-length"),
    }
}

fn report_balance_state(
    cycle_count: u32,
    output: super::WheelControlOutput,
    right_state: MotorState2,
    left_state: MotorState2,
    right_motor_response_count: u32,
    left_motor_response_count: u32,
) {
    if cycle_count != 1 && cycle_count % DIAG_EVERY_FAILURES != 0 {
        return;
    }
    rt_output_write(b"wheel-control: state cycle=");
    ax_rt::rt_output_write_decimal(cycle_count as u64);
    rt_output_write(b" theta_mrad=");
    rt_write_i32((output.state.theta * 1000.0) as i32);
    rt_output_write(b" velocity_mmps=");
    rt_write_i32((output.state.velocity * 1000.0) as i32);
    rt_output_write(b" right_speed_raw=");
    rt_write_i16(right_state.speed_raw);
    rt_output_write(b" left_speed_raw=");
    rt_write_i16(left_state.speed_raw);
    rt_output_write(b" right_enc=");
    ax_rt::rt_output_write_decimal(right_state.encoder as u64);
    rt_output_write(b" left_enc=");
    ax_rt::rt_output_write_decimal(left_state.encoder as u64);
    rt_output_write(b" right_iq=");
    rt_write_i16(output.motor_command.right_iq);
    rt_output_write(b" left_iq=");
    rt_write_i16(output.motor_command.left_iq);
    rt_output_write(b" right_resp=");
    ax_rt::rt_output_write_decimal(right_motor_response_count as u64);
    rt_output_write(b" left_resp=");
    ax_rt::rt_output_write_decimal(left_motor_response_count as u64);
    rt_output_write(b"\n");
}

fn rt_write_i16(value: i16) {
    rt_write_i32(value as i32);
}

fn rt_write_i32(value: i32) {
    if value < 0 {
        rt_output_write(b"-");
        ax_rt::rt_output_write_decimal(value.unsigned_abs() as u64);
    } else {
        ax_rt::rt_output_write_decimal(value as u64);
    }
}

fn report_deadline_miss(count: u32, overrun_nanos: u64, timing: CycleTiming) {
    if count != 1 && count % DIAG_EVERY_FAILURES != 0 {
        return;
    }
    rt_output_write(b"wheel-control: deadline missed count=");
    ax_rt::rt_output_write_decimal(count as u64);
    rt_output_write(b" overrun_ns=");
    ax_rt::rt_output_write_decimal(overrun_nanos);
    rt_output_write(b" imu_ns=");
    ax_rt::rt_output_write_decimal(timing.imu_nanos);
    rt_output_write(b" ctrl_ns=");
    ax_rt::rt_output_write_decimal(timing.controller_nanos);
    rt_output_write(b" servo_ns=");
    ax_rt::rt_output_write_decimal(timing.servo_nanos);
    rt_output_write(b" right_motor_ns=");
    ax_rt::rt_output_write_decimal(timing.right_motor_nanos);
    rt_output_write(b" left_motor_ns=");
    ax_rt::rt_output_write_decimal(timing.left_motor_nanos);
    rt_output_write(b"\n");
}

fn drain_rx(uart: &Uart) {
    for _ in 0..16 {
        if uart.read_byte().is_none() {
            break;
        }
    }
}

fn setup_uart(port: &UartPort) {
    if let Err(err) = setup_uart_pinmux(port) {
        warn!("wheel: {} pinmux setup failed: {err:?}", port.name);
    }
    match axklib::mmio::ioremap_raw((port.base as u64).into(), UART_SIZE) {
        Ok(mmio) => {
            let uart = Uart { mmio };
            uart.init(port.divisor);
            port.virt.store(
                uart.mmio.as_nonnull_ptr().as_ptr() as usize,
                Ordering::Release,
            );
            info!("wheel: {} mapped for RT wheel task", port.name);
        }
        Err(err) => warn!("wheel: ioremap {} failed: {err:?}", port.name),
    }
}

fn setup_uart_pinmux(port: &UartPort) -> Result<(), MapError> {
    let cru = axklib::mmio::ioremap_raw((CRU_BASE as u64).into(), CRU_SIZE)?;
    let ioc = axklib::mmio::ioremap_raw((IOC_BASE as u64).into(), IOC_SIZE)?;
    cru_clear_bit(&cru, port.gate_pclk.0, port.gate_pclk.1);
    cru_clear_bit(&cru, port.gate_sclk.0, port.gate_sclk.1);
    cru.write::<u32>(port.clksel, 0x0003_0002);
    ioc.write::<u32>(port.mux_off, port.mux_val);
    Ok(())
}

fn ensure_uart_alive(port: &UartPort, uart: &Uart) -> bool {
    if uart.r(LSR) & LSR_THRE != 0 {
        return true;
    }
    rt_recover_uart(port);
    uart.r(LSR) & LSR_THRE != 0
}

fn rt_recover_uart(port: &UartPort) {
    let cru = unsafe {
        MmioRaw::new(
            (CRU_BASE as u64).into(),
            NonNull::new_unchecked(CRU_BASE as *mut u8),
            CRU_SIZE,
        )
    };
    cru_clear_bit(&cru, port.gate_pclk.0, port.gate_pclk.1);
    cru_clear_bit(&cru, port.gate_sclk.0, port.gate_sclk.1);
    cru.write::<u32>(port.clksel, 0x0003_0002);
    if let Some(uart) = uart_for(port) {
        uart.init(port.divisor);
    }
}

fn uart_for(port: &UartPort) -> Option<Uart> {
    let virt = port.virt.load(Ordering::Acquire);
    if virt == 0 {
        return None;
    }
    let mmio = unsafe {
        MmioRaw::new(
            (port.base as u64).into(),
            NonNull::new_unchecked(virt as *mut u8),
            UART_SIZE,
        )
    };
    Some(Uart { mmio })
}

fn cru_clear_bit(cru: &MmioRaw, off: usize, bit: u32) {
    cru.write::<u32>(off, 1u32 << (bit + 16));
}
