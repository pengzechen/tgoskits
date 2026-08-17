//! WBR pendulum-on-wheel dynamics and hip kinematics ported from `POL.h`.

use super::{
    control::BalanceState,
    math::{
        Mat3, Mat4, Vec2, Vec3, Vec4, add3, dot3, mat3_add, mat3_inverse, mat3_mul, mat3_transpose,
        mat3_vec, rotation_y, scale3, sub3,
    },
    params::{
        BODY_COM, BODY_INERTIA, BODY_MASS, GRAVITY, HEIGHT_MAX, HEIGHT_MIN,
        LEFT_WHEEL_TORQUE_FACTOR, LINK_A, LINK_B, LINK_L1, LINK_L2, LINK_L3, LINK_L4, LINK_L5,
        RIGHT_WHEEL_TORQUE_FACTOR, WHEEL_BASE_HALF, WHEEL_INERTIA_LEFT, WHEEL_INERTIA_RIGHT,
        WHEEL_MASS_LEFT, WHEEL_MASS_RIGHT, WHEEL_RADIUS,
    },
};

const DT_SECONDS: f32 = 0.008;
const JACOBIAN_EPSILON: f32 = 1.0e-4;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ModelError {
    HipHeightOutOfRange,
    SingularMassMatrix,
    NonFiniteState,
}

#[derive(Clone, Copy, Debug)]
pub struct ModelOutput {
    pub predicted_state: BalanceState,
    pub state_jacobian: Mat4,
}

#[derive(Clone, Copy, Debug)]
pub struct RobotModel {
    height: f32,
    roll_degrees: f32,
    body_com: Vec3,
    body_inertia: Mat3,
    hip_angles: Vec2,
}

impl Default for RobotModel {
    fn default() -> Self {
        Self {
            height: HEIGHT_MAX,
            roll_degrees: 0.0,
            body_com: [0.0; 3],
            body_inertia: [[0.0; 3]; 3],
            hip_angles: [0.0; 2],
        }
    }
}

impl RobotModel {
    pub fn update_height_roll(
        &mut self,
        height: f32,
        roll_degrees: f32,
    ) -> Result<f32, ModelError> {
        self.height = height.clamp(HEIGHT_MIN, HEIGHT_MAX);
        self.roll_degrees = roll_degrees;
        self.calculate_com_and_inertia()?;
        Ok(self.equilibrium_theta())
    }

    pub fn hip_angles(&self) -> Vec2 {
        self.hip_angles
    }

    pub fn predict(&self, state: BalanceState, torque: Vec2) -> Result<ModelOutput, ModelError> {
        let predicted_state = self.predict_state_only(state, torque)?;
        let state_jacobian = self.finite_difference_state_jacobian(state, torque)?;
        Ok(ModelOutput {
            predicted_state,
            state_jacobian,
        })
    }

    pub fn observation(&self, state: BalanceState) -> [f32; 8] {
        let theta = state.theta;
        let theta_dot = state.theta_dot;
        let velocity = state.velocity;
        let yaw_rate = state.yaw_rate;
        let cos_theta = libm::cosf(theta);
        let sin_theta = libm::sinf(theta);
        let cos_theta_2 = cos_theta * cos_theta;
        let sin_cos_theta = sin_theta * cos_theta;
        [
            -GRAVITY * sin_theta - self.height * yaw_rate * yaw_rate * sin_cos_theta,
            yaw_rate * velocity + 2.0 * self.height * yaw_rate * theta_dot * cos_theta,
            -self.height * theta_dot * theta_dot + GRAVITY * cos_theta
                - yaw_rate * yaw_rate * (self.height - self.height * cos_theta_2),
            -yaw_rate * sin_theta,
            theta_dot,
            yaw_rate * cos_theta,
            theta_dot - velocity / WHEEL_RADIUS - WHEEL_BASE_HALF * yaw_rate / WHEEL_RADIUS,
            -theta_dot + velocity / WHEEL_RADIUS - WHEEL_BASE_HALF * yaw_rate / WHEEL_RADIUS,
        ]
    }

    pub fn observation_jacobian(&self, state: BalanceState) -> [[f32; 4]; 8] {
        let theta = state.theta;
        let theta_dot = state.theta_dot;
        let velocity = state.velocity;
        let yaw_rate = state.yaw_rate;
        let cos_theta = libm::cosf(theta);
        let sin_theta = libm::sinf(theta);
        let cos_theta_2 = cos_theta * cos_theta;
        let sin_theta_2 = sin_theta * sin_theta;
        let sin_cos_theta = sin_theta * cos_theta;
        let mut h = [[0.0; 4]; 8];
        h[0][0] = yaw_rate * yaw_rate * (self.height * sin_theta_2 - self.height * cos_theta_2)
            - GRAVITY * cos_theta;
        h[1][0] = -2.0 * self.height * yaw_rate * theta_dot * sin_theta;
        h[2][0] = -GRAVITY * sin_theta - 2.0 * self.height * yaw_rate * yaw_rate * sin_cos_theta;
        h[3][0] = -yaw_rate * cos_theta;
        h[5][0] = -yaw_rate * sin_theta;
        h[1][1] = 2.0 * self.height * yaw_rate * cos_theta;
        h[2][1] = -2.0 * self.height * theta_dot;
        h[4][1] = 1.0;
        h[6][1] = 1.0;
        h[7][1] = -1.0;
        h[1][2] = yaw_rate;
        h[6][2] = -1.0 / WHEEL_RADIUS;
        h[7][2] = 1.0 / WHEEL_RADIUS;
        h[0][3] = -2.0 * self.height * yaw_rate * sin_cos_theta;
        h[1][3] = velocity + 2.0 * self.height * theta_dot * cos_theta;
        h[2][3] = -2.0 * yaw_rate * (self.height - self.height * cos_theta_2);
        h[3][3] = -sin_theta;
        h[5][3] = cos_theta;
        h[6][3] = -WHEEL_BASE_HALF / WHEEL_RADIUS;
        h[7][3] = -WHEEL_BASE_HALF / WHEEL_RADIUS;
        h
    }

    fn predict_state_only(
        &self,
        state: BalanceState,
        torque: Vec2,
    ) -> Result<BalanceState, ModelError> {
        let acceleration = self.acceleration(state, torque)?;
        let next = BalanceState {
            theta: state.theta + state.theta_dot * DT_SECONDS,
            theta_dot: state.theta_dot + acceleration[0] * DT_SECONDS,
            velocity: state.velocity + acceleration[1] * DT_SECONDS,
            yaw_rate: state.yaw_rate + acceleration[2] * DT_SECONDS,
        };
        if [next.theta, next.theta_dot, next.velocity, next.yaw_rate]
            .iter()
            .all(|value| value.is_finite())
        {
            Ok(next)
        } else {
            Err(ModelError::NonFiniteState)
        }
    }

    fn finite_difference_state_jacobian(
        &self,
        state: BalanceState,
        torque: Vec2,
    ) -> Result<Mat4, ModelError> {
        let base = balance_to_vec4(self.predict_state_only(state, torque)?);
        let mut jacobian = [[0.0; 4]; 4];
        let mut source = balance_to_vec4(state);
        for col in 0..4 {
            source[col] += JACOBIAN_EPSILON;
            let shifted = balance_from_vec4(source);
            let predicted = balance_to_vec4(self.predict_state_only(shifted, torque)?);
            for row in 0..4 {
                jacobian[row][col] = (predicted[row] - base[row]) / JACOBIAN_EPSILON;
            }
            source[col] -= JACOBIAN_EPSILON;
        }
        Ok(jacobian)
    }

    fn acceleration(&self, state: BalanceState, torque: Vec2) -> Result<Vec3, ModelError> {
        let m = self.mass_matrix(state);
        let Some(m_inv) = mat3_inverse(m) else {
            return Err(ModelError::SingularMassMatrix);
        };
        let nle = self.nonlinear_effects(state);
        let generalized_input = [
            torque[0] - torque[1],
            -torque[0] / WHEEL_RADIUS + torque[1] / WHEEL_RADIUS,
            -WHEEL_BASE_HALF * (torque[0] + torque[1]) / WHEEL_RADIUS,
        ];
        let acceleration = mat3_vec(m_inv, sub3(generalized_input, nle));
        if acceleration.iter().all(|value| value.is_finite()) {
            Ok(acceleration)
        } else {
            Err(ModelError::NonFiniteState)
        }
    }

    fn mass_matrix(&self, state: BalanceState) -> Mat3 {
        let theta = state.theta;
        let cos_theta = libm::cosf(theta);
        let sin_theta = libm::sinf(theta);
        let cos_theta_2 = cos_theta * cos_theta;
        let sin_cos_theta = sin_theta * cos_theta;
        let h_2 = self.height * self.height;
        let p = self.body_com;
        let ib = self.body_inertia;
        let il = WHEEL_INERTIA_LEFT;
        let ir = WHEEL_INERTIA_RIGHT;
        let m_body = BODY_MASS.iter().sum::<f32>();
        let mut m = [[0.0; 3]; 3];
        m[0][0] = ib[1][1]
            + m_body * (h_2 + p[0] * p[0] + p[2] * p[2])
            + 2.0 * self.height * m_body * p[2];
        m[1][0] = m_body * (-sin_theta * p[0] + cos_theta * (self.height + p[2]));
        m[0][1] = m[1][0];
        m[2][0] = cos_theta * (ib[2][1] - m_body * p[1] * (self.height + p[2]))
            - sin_theta * (ib[1][0] - m_body * p[0] * p[1]);
        m[0][2] = m[2][0];
        m[1][1] = m_body
            + WHEEL_MASS_LEFT
            + WHEEL_MASS_RIGHT
            + (il[1][1] + ir[1][1]) / (WHEEL_RADIUS * WHEEL_RADIUS);
        m[2][1] = -m_body * p[1] + (cos_theta * (il[2][1] + ir[2][1])) / WHEEL_RADIUS
            - (sin_theta * (il[1][0] + ir[1][0])) / WHEEL_RADIUS;
        m[1][2] = m[2][1];
        m[2][2] = ib[0][0]
            + il[0][0]
            + ir[0][0]
            + WHEEL_BASE_HALF * WHEEL_BASE_HALF * (il[1][1] + ir[1][1])
                / (WHEEL_RADIUS * WHEEL_RADIUS)
            + WHEEL_BASE_HALF * WHEEL_BASE_HALF * (WHEEL_MASS_LEFT + WHEEL_MASS_RIGHT)
            + m_body * (h_2 + p[1] * p[1] + p[2] * p[2])
            - cos_theta
                * ((il[2][1] * WHEEL_BASE_HALF * 2.0) / WHEEL_RADIUS
                    - (ir[2][1] * WHEEL_BASE_HALF * 2.0) / WHEEL_RADIUS)
            + sin_theta
                * ((il[1][0] * WHEEL_BASE_HALF * 2.0) / WHEEL_RADIUS
                    - (ir[1][0] * WHEEL_BASE_HALF * 2.0) / WHEEL_RADIUS)
            - cos_theta_2
                * (ib[0][0] - ib[2][2] + il[0][0] + ir[0][0] - il[2][2] - ir[2][2]
                    + m_body * (self.height * p[2] * 2.0 + h_2 - p[0] * p[0] + p[2] * p[2]))
            - sin_cos_theta
                * (ib[2][0] * 2.0 + il[2][0] * 2.0 + ir[2][0] * 2.0
                    - m_body * p[0] * (self.height + p[2]) * 2.0)
            + self.height * m_body * p[2] * 2.0;
        m
    }

    fn nonlinear_effects(&self, state: BalanceState) -> Vec3 {
        let theta = state.theta;
        let cos_theta = libm::cosf(theta);
        let sin_theta = libm::sinf(theta);
        let cos_theta_2 = cos_theta * cos_theta;
        let sin_cos_theta = sin_theta * cos_theta;
        let theta_dot_2 = state.theta_dot * state.theta_dot;
        let psi_dot_2 = state.yaw_rate * state.yaw_rate;
        let p = self.body_com;
        let ib = self.body_inertia;
        let il = WHEEL_INERTIA_LEFT;
        let ir = WHEEL_INERTIA_RIGHT;
        let m_body = BODY_MASS.iter().sum::<f32>();
        let h_2 = self.height * self.height;

        let first = -psi_dot_2
            * (ib[2][2] + il[2][2] + ir[2][2]
                - m_body
                    * (self.height * p[0] + p[0] * p[2]
                        - cos_theta_2 * (self.height * p[0] * 2.0 + p[0] * p[2] * 2.0)
                        - sin_cos_theta
                            * (self.height * p[2] * 2.0 + h_2 - p[0] * p[0] + p[2] * p[2]))
                - cos_theta_2 * (ib[2][2] * 2.0 + il[2][2] * 2.0 + ir[2][2] * 2.0)
                + sin_cos_theta
                    * (ib[0][0] - ib[2][2] + il[0][0] + ir[0][0] - il[2][2] - ir[2][2]))
            - GRAVITY * m_body * (sin_theta * (self.height + p[2]) + p[0] * cos_theta);
        let second = -m_body * theta_dot_2 * (sin_theta * (self.height + p[2]) + p[0] * cos_theta);
        let third = -theta_dot_2
            * (-m_body * (sin_theta * (p[1] * (self.height + p[2])) + p[0] * p[1] * cos_theta)
                + ib[1][1] * cos_theta
                + ib[2][2] * sin_theta);
        [first, second, third]
    }

    fn calculate_com_and_inertia(&mut self) -> Result<(), ModelError> {
        self.solve_inverse_kinematics()?;
        let rotations = self.solve_forward_kinematics()?;
        let mut body_points = [[0.0; 3]; 7];
        let mut body_com = [0.0; 3];
        let body_mass = BODY_MASS.iter().sum::<f32>();
        let mut p_vecs = [[0.0; 3]; 7];
        p_vecs[1] = [-0.064_951_904, -0.086, 0.0375];
        p_vecs[2] = [-0.064_951_904, 0.086, 0.0375];
        p_vecs[3] = [0.0, -0.081, 0.0];
        p_vecs[4] = [0.0, 0.081, 0.0];
        let th_b0 = -self.hip_angles[0];
        let th_b1 = self.hip_angles[1];
        p_vecs[5] = [
            -LINK_A - LINK_L2 * libm::cosf(th_b0),
            -0.102,
            LINK_B + LINK_L2 * libm::sinf(th_b0),
        ];
        p_vecs[6] = [
            -LINK_A - LINK_L2 * libm::cosf(th_b1),
            0.102,
            LINK_B + LINK_L2 * libm::sinf(th_b1),
        ];

        for i in 0..7 {
            body_points[i] = add3(p_vecs[i], mat3_vec(rotations[i], BODY_COM[i]));
            body_com = add3(body_com, scale3(body_points[i], BODY_MASS[i]));
        }
        body_com = scale3(body_com, 1.0 / body_mass);

        let mut inertia = [[0.0; 3]; 3];
        for i in 0..7 {
            let r = sub3(body_points[i], body_com);
            let nn = dot3(r, r);
            let rotated = mat3_mul(
                mat3_mul(rotations[i], BODY_INERTIA[i]),
                mat3_transpose(rotations[i]),
            );
            let parallel_axis = [
                [nn - r[0] * r[0], -r[0] * r[1], -r[0] * r[2]],
                [-r[1] * r[0], nn - r[1] * r[1], -r[1] * r[2]],
                [-r[2] * r[0], -r[2] * r[1], nn - r[2] * r[2]],
            ];
            let mut contribution = rotated;
            for row in 0..3 {
                for col in 0..3 {
                    contribution[row][col] += BODY_MASS[i] * parallel_axis[row][col];
                }
            }
            inertia = mat3_add(inertia, contribution);
        }
        self.body_com = body_com;
        self.body_inertia = inertia;
        Ok(())
    }

    fn solve_inverse_kinematics(&mut self) -> Result<(), ModelError> {
        let roll = self.roll_degrees * core::f32::consts::PI / 180.0;
        let phi_max = libm::atanf((self.height - HEIGHT_MIN) / WHEEL_BASE_HALF)
            .min(libm::atanf((HEIGHT_MAX - self.height) / WHEEL_BASE_HALF));
        let roll = roll.clamp(-phi_max, phi_max);
        let heights = [
            self.height - WHEEL_BASE_HALF * libm::tanf(roll),
            self.height + WHEEL_BASE_HALF * libm::tanf(roll),
        ];
        let angle_edf = libm::atanf(LINK_L5 / LINK_L4);
        let ab = libm::sqrtf(LINK_A * LINK_A + LINK_B * LINK_B);
        for (i, h_val) in heights.into_iter().enumerate() {
            let ade_num = LINK_L1 * LINK_L1 + LINK_L4 * LINK_L4 + LINK_L5 * LINK_L5 - h_val * h_val;
            let ade_den = 2.0 * LINK_L1 * libm::sqrtf(LINK_L4 * LINK_L4 + LINK_L5 * LINK_L5);
            let angle_ade = checked_acos(ade_num / ade_den)?;
            let angle_adc = core::f32::consts::PI - (angle_ade + angle_edf);
            let ac_squared = LINK_L1 * LINK_L1 + LINK_L3 * LINK_L3
                - 2.0 * LINK_L1 * LINK_L3 * libm::cosf(angle_adc);
            let ac = libm::sqrtf(ac_squared);
            let angle_abc =
                checked_acos((ab * ab + LINK_L2 * LINK_L2 - ac * ac) / (2.0 * ab * LINK_L2))?;
            self.hip_angles[i] = 5.0 * core::f32::consts::PI / 6.0 - angle_abc;
        }
        self.hip_angles[1] = -self.hip_angles[1];
        Ok(())
    }

    fn solve_forward_kinematics(&self) -> Result<[Mat3; 7], ModelError> {
        let theta_bs = [-self.hip_angles[0], self.hip_angles[1]];
        let mut rotations = [[[0.0; 3]; 3]; 7];
        rotations[0] = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        for i in 0..2 {
            let c = libm::cosf(theta_bs[i]);
            let s = libm::sinf(theta_bs[i]);
            let ax = LINK_A + LINK_L2 * c;
            let ay = LINK_B + LINK_L2 * s;
            let dist = libm::sqrtf(ax * ax + ay * ay);
            let num = LINK_A * LINK_A + LINK_B * LINK_B + LINK_L1 * LINK_L1 + LINK_L2 * LINK_L2
                - LINK_L3 * LINK_L3
                + 2.0 * LINK_L2 * (LINK_A * c + LINK_B * s);
            let theta_a = -checked_acos((num / (2.0 * LINK_L1)) / dist)? + libm::atan2f(ay, ax);
            let nx = LINK_A + LINK_L2 * c - LINK_L1 * libm::cosf(theta_a);
            let ny = LINK_B + LINK_L2 * s - LINK_L1 * libm::sinf(theta_a);
            rotations[1 + i] = rotation_y(theta_bs[i]);
            rotations[3 + i] = rotation_y(theta_a);
            rotations[5 + i] = rotation_y(libm::atan2f(ny, nx));
        }
        Ok(rotations)
    }

    fn equilibrium_theta(&self) -> f32 {
        libm::atanf(-self.body_com[0] / (self.height + self.body_com[2]))
    }
}

pub fn torque_command_to_nm(right_iq: i16, left_iq: i16) -> Vec2 {
    [
        right_iq as f32 * RIGHT_WHEEL_TORQUE_FACTOR,
        left_iq as f32 * LEFT_WHEEL_TORQUE_FACTOR,
    ]
}

fn checked_acos(value: f32) -> Result<f32, ModelError> {
    if value.abs() > 1.0 + 1.0e-3 || !value.is_finite() {
        return Err(ModelError::HipHeightOutOfRange);
    }
    Ok(libm::acosf(value.clamp(-1.0, 1.0)))
}

fn balance_to_vec4(state: BalanceState) -> Vec4 {
    [state.theta, state.theta_dot, state.velocity, state.yaw_rate]
}

fn balance_from_vec4(v: Vec4) -> BalanceState {
    BalanceState {
        theta: v[0],
        theta_dot: v[1],
        velocity: v[2],
        yaw_rate: v[3],
    }
}
