//! Deterministic LQR control step for the wheeled biped robot.

use super::params::{
    HEIGHT_MAX, HEIGHT_MIN, LEFT_WHEEL_TORQUE_FACTOR, LQR_GAINS, MAX_TORQUE,
    RIGHT_WHEEL_TORQUE_FACTOR, VEL_MAX, YAW_MAX,
};

/// Estimated state `[theta, theta_dot, forward_velocity, yaw_rate]`.
#[derive(Clone, Copy, Debug, Default)]
pub struct BalanceState {
    pub theta: f32,
    pub theta_dot: f32,
    pub velocity: f32,
    pub yaw_rate: f32,
}

/// Desired height/velocity/yaw command.
#[derive(Clone, Copy, Debug)]
pub struct BalanceTarget {
    pub height: f32,
    pub theta: f32,
    pub velocity: f32,
    pub yaw_rate: f32,
}

impl Default for BalanceTarget {
    fn default() -> Self {
        Self {
            height: HEIGHT_MAX,
            theta: 0.0,
            velocity: 0.0,
            yaw_rate: 0.0,
        }
    }
}

/// Saturated wheel torque output in Nm.
#[derive(Clone, Copy, Debug, Default)]
pub struct WheelTorque {
    pub right: f32,
    pub left: f32,
    pub right_saturated: bool,
    pub left_saturated: bool,
}

/// Raw motor current command values for the Lingkong `0xA1` torque command.
#[derive(Clone, Copy, Debug, Default)]
pub struct MotorTorqueCommand {
    pub right_iq: i16,
    pub left_iq: i16,
}

/// Runs the ESP32 LQR table lookup and torque saturation.
pub fn compute_wheel_torque(target: BalanceTarget, state: BalanceState) -> WheelTorque {
    let target = clamp_target(target);
    let gain = interpolated_gain(target.height);
    let error = [
        target.theta - state.theta,
        -state.theta_dot,
        target.velocity - state.velocity,
        target.yaw_rate - state.yaw_rate,
    ];
    let right = row_dot(gain[0], error);
    let left = row_dot(gain[1], error);
    saturate_torque(right, left)
}

/// Converts saturated torque values to the original motor `iqControl` units.
pub fn torque_to_motor_command(torque: WheelTorque) -> MotorTorqueCommand {
    MotorTorqueCommand {
        right_iq: clamp_i16(torque.right / RIGHT_WHEEL_TORQUE_FACTOR),
        left_iq: clamp_i16(torque.left / LEFT_WHEEL_TORQUE_FACTOR),
    }
}

fn clamp_target(target: BalanceTarget) -> BalanceTarget {
    BalanceTarget {
        height: target.height.clamp(HEIGHT_MIN, HEIGHT_MAX),
        theta: target.theta,
        velocity: target.velocity.clamp(-VEL_MAX, VEL_MAX),
        yaw_rate: target.yaw_rate.clamp(-YAW_MAX, YAW_MAX),
    }
}

fn interpolated_gain(height: f32) -> [[f32; 4]; 2] {
    let position = ((height - HEIGHT_MIN) / 0.01).clamp(0.0, (LQR_GAINS.len() - 1) as f32);
    let lower = position.floor() as usize;
    let upper = (lower + 1).min(LQR_GAINS.len() - 1);
    let ratio = position - lower as f32;
    let mut gain = [[0.0; 4]; 2];
    for row in 0..2 {
        for col in 0..4 {
            gain[row][col] =
                LQR_GAINS[lower][row][col] * (1.0 - ratio) + LQR_GAINS[upper][row][col] * ratio;
        }
    }
    gain
}

fn row_dot(row: [f32; 4], vector: [f32; 4]) -> f32 {
    row[0] * vector[0] + row[1] * vector[1] + row[2] * vector[2] + row[3] * vector[3]
}

fn saturate_torque(right: f32, left: f32) -> WheelTorque {
    let saturated_right = right.clamp(-MAX_TORQUE, MAX_TORQUE);
    let saturated_left = left.clamp(-MAX_TORQUE, MAX_TORQUE);
    WheelTorque {
        right: saturated_right,
        left: saturated_left,
        right_saturated: saturated_right != right,
        left_saturated: saturated_left != left,
    }
}

fn clamp_i16(value: f32) -> i16 {
    value.clamp(i16::MIN as f32, i16::MAX as f32).round() as i16
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zero_error_produces_zero_torque() {
        let torque = compute_wheel_torque(BalanceTarget::default(), BalanceState::default());

        assert_eq!(torque.right, 0.0);
        assert_eq!(torque.left, 0.0);
    }

    #[test]
    fn torque_command_uses_original_esp32_factors() {
        let command = torque_to_motor_command(WheelTorque {
            right: RIGHT_WHEEL_TORQUE_FACTOR * 100.0,
            left: LEFT_WHEEL_TORQUE_FACTOR * -100.0,
            right_saturated: false,
            left_saturated: false,
        });

        assert_eq!(command.right_iq, 100);
        assert_eq!(command.left_iq, -100);
    }
}
