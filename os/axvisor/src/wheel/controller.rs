//! 8 ms WBR control orchestration without Wi-Fi, serial parser, or logging.

use super::{
    control::{
        BalanceState, BalanceTarget, MotorTorqueCommand, WheelTorque, compute_wheel_torque,
        torque_to_motor_command,
    },
    ekf::{EkfError, WheelEkf},
    imu::ImuMeasurement,
    math::Vec8,
    model::{RobotModel, torque_command_to_nm},
    motor::MotorState2,
    params::ESP32_CONTROL_PERIOD_NANOS,
    servo::{HipServoAngles, hip_angles_to_servo_degrees},
};

#[derive(Clone, Copy, Debug)]
pub struct WheelMeasurements {
    pub imu: ImuMeasurement,
    pub right_motor: MotorState2,
    pub left_motor: MotorState2,
}

#[derive(Clone, Copy, Debug)]
pub struct WheelControlOutput {
    pub state: BalanceState,
    pub target: BalanceTarget,
    pub torque: WheelTorque,
    pub motor_command: MotorTorqueCommand,
    pub hip_servos: HipServoAngles,
}

#[derive(Clone, Copy, Debug)]
pub struct WheelController {
    model: RobotModel,
    ekf: WheelEkf,
    state: BalanceState,
    target: BalanceTarget,
    previous_motor_command: MotorTorqueCommand,
}

impl Default for WheelController {
    fn default() -> Self {
        Self {
            model: RobotModel::default(),
            ekf: WheelEkf::default(),
            state: BalanceState::default(),
            target: BalanceTarget::default(),
            previous_motor_command: MotorTorqueCommand::default(),
        }
    }
}

impl WheelController {
    pub const PERIOD_NANOS: u64 = ESP32_CONTROL_PERIOD_NANOS;

    pub fn set_target(&mut self, target: BalanceTarget) {
        self.target = target;
    }

    pub fn reset(&mut self) {
        self.ekf.reset();
        self.state = BalanceState::default();
        self.previous_motor_command = MotorTorqueCommand::default();
    }

    pub fn step(
        &mut self,
        measurements: WheelMeasurements,
    ) -> Result<WheelControlOutput, EkfError> {
        let theta_equilibrium = self
            .model
            .update_height_roll(self.target.height, 0.0)
            .map_err(EkfError::Model)?;
        let mut target = self.target;
        target.theta = theta_equilibrium;
        let previous_torque = torque_command_to_nm(
            self.previous_motor_command.right_iq,
            self.previous_motor_command.left_iq,
        );
        self.state = self.ekf.estimate(
            &self.model,
            self.state,
            previous_torque,
            measurement_vector(measurements),
        )?;
        let torque = compute_wheel_torque(target, self.state);
        let motor_command = torque_to_motor_command(torque);
        self.previous_motor_command = motor_command;
        let hips = self.model.hip_angles();
        Ok(WheelControlOutput {
            state: self.state,
            target,
            torque,
            motor_command,
            hip_servos: hip_angles_to_servo_degrees(hips[0], hips[1]),
        })
    }
}

fn measurement_vector(measurements: WheelMeasurements) -> Vec8 {
    [
        measurements.imu.acceleration_mps2[0],
        measurements.imu.acceleration_mps2[1],
        measurements.imu.acceleration_mps2[2],
        measurements.imu.gyro_rad_per_sec[0],
        measurements.imu.gyro_rad_per_sec[1],
        measurements.imu.gyro_rad_per_sec[2],
        measurements.right_motor.speed_radians_per_second(),
        measurements.left_motor.speed_radians_per_second(),
    ]
}
