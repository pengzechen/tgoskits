//! EKF state estimator ported from `EKF.h`.

use super::{
    control::BalanceState,
    math::{Mat4, Mat8, Mat8x4, Vec8, invert_8},
    model::{ModelError, RobotModel},
};

const PROCESS_NOISE: [f32; 4] = [0.0, 1.0, 1.0, 1.0];
const MEASUREMENT_NOISE: [f32; 8] = [0.1, 4.0, 0.1, 1.0, 4.126_42e-6, 1.0, 0.0, 0.0];

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum EkfError {
    Model(ModelError),
    SingularInnovation,
}

#[derive(Clone, Copy, Debug)]
pub struct WheelEkf {
    covariance: Mat4,
}

impl Default for WheelEkf {
    fn default() -> Self {
        Self {
            covariance: identity4(),
        }
    }
}

impl WheelEkf {
    pub fn reset(&mut self) {
        self.covariance = identity4();
    }

    pub fn estimate(
        &mut self,
        model: &RobotModel,
        state: BalanceState,
        previous_torque: [f32; 2],
        measurement: Vec8,
    ) -> Result<BalanceState, EkfError> {
        let prediction = model
            .predict(state, previous_torque)
            .map_err(EkfError::Model)?;
        let predicted_covariance = add_process_noise(mat4_mul(
            mat4_mul(prediction.state_jacobian, self.covariance),
            mat4_transpose(prediction.state_jacobian),
        ));
        let h = model.observation_jacobian(prediction.predicted_state);
        let expected = model.observation(prediction.predicted_state);
        let innovation = subtract8(measurement, expected);
        let innovation_covariance = add_measurement_noise(mat8x4_mul_4x8(
            h,
            mat4_mul_4x8(predicted_covariance, transpose_8x4(h)),
        ));
        let Some(innovation_inverse) = invert_8(innovation_covariance) else {
            return Err(EkfError::SingularInnovation);
        };
        let kalman_gain = mat4x8_mul_8x8(
            mat4_mul_4x8(predicted_covariance, transpose_8x4(h)),
            innovation_inverse,
        );
        let correction = mat4x8_vec8(kalman_gain, innovation);
        let predicted = [
            prediction.predicted_state.theta,
            prediction.predicted_state.theta_dot,
            prediction.predicted_state.velocity,
            prediction.predicted_state.yaw_rate,
        ];
        let corrected = [
            predicted[0] + correction[0],
            predicted[1] + correction[1],
            predicted[2] + correction[2],
            predicted[3] + correction[3],
        ];
        self.covariance = mat4_mul(
            sub4(identity4(), mat4x8_mul_8x4(kalman_gain, h)),
            predicted_covariance,
        );
        Ok(BalanceState {
            theta: corrected[0],
            theta_dot: corrected[1],
            velocity: corrected[2],
            yaw_rate: corrected[3],
        })
    }
}

fn identity4() -> Mat4 {
    [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
}

fn mat4_transpose(m: Mat4) -> Mat4 {
    [
        [m[0][0], m[1][0], m[2][0], m[3][0]],
        [m[0][1], m[1][1], m[2][1], m[3][1]],
        [m[0][2], m[1][2], m[2][2], m[3][2]],
        [m[0][3], m[1][3], m[2][3], m[3][3]],
    ]
}

fn mat4_mul(a: Mat4, b: Mat4) -> Mat4 {
    let mut out = [[0.0; 4]; 4];
    for i in 0..4 {
        for j in 0..4 {
            for (k, row) in b.iter().enumerate() {
                out[i][j] += a[i][k] * row[j];
            }
        }
    }
    out
}

fn add_process_noise(mut m: Mat4) -> Mat4 {
    for i in 0..4 {
        m[i][i] += PROCESS_NOISE[i];
    }
    m
}

fn add_measurement_noise(mut m: Mat8) -> Mat8 {
    for i in 0..8 {
        m[i][i] += MEASUREMENT_NOISE[i];
    }
    m
}

fn subtract8(a: Vec8, b: Vec8) -> Vec8 {
    let mut out = [0.0; 8];
    for i in 0..8 {
        out[i] = a[i] - b[i];
    }
    out
}

fn sub4(a: Mat4, b: Mat4) -> Mat4 {
    let mut out = [[0.0; 4]; 4];
    for i in 0..4 {
        for j in 0..4 {
            out[i][j] = a[i][j] - b[i][j];
        }
    }
    out
}

fn transpose_8x4(m: Mat8x4) -> [[f32; 8]; 4] {
    let mut out = [[0.0; 8]; 4];
    for (i, row) in m.iter().enumerate() {
        for j in 0..4 {
            out[j][i] = row[j];
        }
    }
    out
}

fn mat4_mul_4x8(a: Mat4, b: [[f32; 8]; 4]) -> [[f32; 8]; 4] {
    let mut out = [[0.0; 8]; 4];
    for i in 0..4 {
        for j in 0..8 {
            for (k, row) in b.iter().enumerate() {
                out[i][j] += a[i][k] * row[j];
            }
        }
    }
    out
}

fn mat8x4_mul_4x8(a: Mat8x4, b: [[f32; 8]; 4]) -> Mat8 {
    let mut out = [[0.0; 8]; 8];
    for i in 0..8 {
        for j in 0..8 {
            for (k, row) in b.iter().enumerate() {
                out[i][j] += a[i][k] * row[j];
            }
        }
    }
    out
}

fn mat4x8_mul_8x8(a: [[f32; 8]; 4], b: Mat8) -> [[f32; 8]; 4] {
    let mut out = [[0.0; 8]; 4];
    for i in 0..4 {
        for j in 0..8 {
            for (k, row) in b.iter().enumerate() {
                out[i][j] += a[i][k] * row[j];
            }
        }
    }
    out
}

fn mat4x8_mul_8x4(a: [[f32; 8]; 4], b: Mat8x4) -> Mat4 {
    let mut out = [[0.0; 4]; 4];
    for i in 0..4 {
        for j in 0..4 {
            for (k, row) in b.iter().enumerate() {
                out[i][j] += a[i][k] * row[j];
            }
        }
    }
    out
}

fn mat4x8_vec8(a: [[f32; 8]; 4], b: Vec8) -> [f32; 4] {
    let mut out = [0.0; 4];
    for i in 0..4 {
        for (k, value) in b.iter().enumerate() {
            out[i] += a[i][k] * value;
        }
    }
    out
}
