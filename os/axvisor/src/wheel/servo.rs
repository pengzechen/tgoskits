//! Hip-servo angle mapping copied from the ESP32 `HRController`.

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct HipServoAngles {
    pub right_degrees: u8,
    pub left_degrees: u8,
}

const LEFT_CHANNEL: u8 = 1;
const RIGHT_CHANNEL: u8 = 2;
const LEFT_MIN_DEGREES: i32 = 88;
const LEFT_MAX_DEGREES: i32 = 171;
const LEFT_CENTER_DEGREES: i32 = 101;
const RIGHT_MIN_DEGREES: i32 = 15;
const RIGHT_MAX_DEGREES: i32 = 98;
const RIGHT_CENTER_DEGREES: i32 = 85;

pub fn hip_servo_channels() -> [u8; 2] {
    [LEFT_CHANNEL, RIGHT_CHANNEL]
}

pub fn hip_angles_to_servo_degrees(right_hip_rad: f32, left_hip_rad: f32) -> HipServoAngles {
    let rad_to_deg = 180.0 / core::f32::consts::PI;
    HipServoAngles {
        right_degrees: clamp_degrees(
            RIGHT_CENTER_DEGREES as f32 - right_hip_rad * rad_to_deg,
            RIGHT_MIN_DEGREES,
            RIGHT_MAX_DEGREES,
        ),
        left_degrees: clamp_degrees(
            LEFT_CENTER_DEGREES as f32 - left_hip_rad * rad_to_deg,
            LEFT_MIN_DEGREES,
            LEFT_MAX_DEGREES,
        ),
    }
}

fn clamp_degrees(value: f32, min: i32, max: i32) -> u8 {
    value.clamp(min as f32, max as f32).round() as u8
}
