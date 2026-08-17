//! MPU6050 measurement conversion for the wheel robot body frame.

const GRAVITY: f32 = 9.80665;
const DEG_TO_RAD: f32 = core::f32::consts::PI / 180.0;

/// Calibration copied from `IMU.h` and `command.md`.
const GYRO_BIAS_RAD_PER_SEC: [f32; 3] = [-0.012_987_159, 0.024_731_049, 0.000_443_216];
const ACCEL_OFFSET_RAW: [f32; 3] = [497.735, 77.190, 813.265];
const ACCEL_SENSITIVITY_RAW_PER_G: [f32; 3] = [16_362.115, 16_382.780, 16_732.935];

#[derive(Clone, Copy, Debug, Default)]
pub struct RawMpu6050Sample {
    pub accel: [i16; 3],
    pub gyro: [i16; 3],
    pub temperature_raw: i16,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct ImuMeasurement {
    pub acceleration_mps2: [f32; 3],
    pub gyro_rad_per_sec: [f32; 3],
    pub temperature_celsius: f32,
}

/// Converts MPU6050 raw values into the robot body frame used by the ESP32 EKF.
pub fn convert_mpu6050_sample(raw: RawMpu6050Sample) -> ImuMeasurement {
    let mut acceleration = [0.0; 3];
    let mut gyro = [0.0; 3];
    for axis in 0..3 {
        acceleration[axis] = ((raw.accel[axis] as f32 - ACCEL_OFFSET_RAW[axis])
            / ACCEL_SENSITIVITY_RAW_PER_G[axis])
            * GRAVITY;
        gyro[axis] = raw.gyro[axis] as f32 / 131.0 * DEG_TO_RAD - GYRO_BIAS_RAD_PER_SEC[axis];
    }

    // ESP32 mounting rule: robot +X = sensor +X, robot +Y = sensor -Y, robot +Z
    // = sensor -Z. Apply the same rotation to acceleration and angular velocity.
    acceleration[1] = -acceleration[1];
    acceleration[2] = -acceleration[2];
    gyro[1] = -gyro[1];
    gyro[2] = -gyro[2];

    ImuMeasurement {
        acceleration_mps2: acceleration,
        gyro_rad_per_sec: gyro,
        temperature_celsius: raw.temperature_raw as f32 / 340.0 + 36.53,
    }
}
