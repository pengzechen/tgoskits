pub type Vec2 = [f32; 2];
pub type Vec3 = [f32; 3];
pub type Vec4 = [f32; 4];
pub type Vec8 = [f32; 8];
pub type Mat3 = [[f32; 3]; 3];
pub type Mat4 = [[f32; 4]; 4];
pub type Mat8 = [[f32; 8]; 8];
pub type Mat8x4 = [[f32; 4]; 8];

pub fn dot3(a: Vec3, b: Vec3) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

pub fn add3(a: Vec3, b: Vec3) -> Vec3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

pub fn sub3(a: Vec3, b: Vec3) -> Vec3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

pub fn scale3(a: Vec3, k: f32) -> Vec3 {
    [a[0] * k, a[1] * k, a[2] * k]
}

pub fn mat3_vec(m: Mat3, v: Vec3) -> Vec3 {
    [dot3(m[0], v), dot3(m[1], v), dot3(m[2], v)]
}

pub fn mat3_mul(a: Mat3, b: Mat3) -> Mat3 {
    let mut out = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            out[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j];
        }
    }
    out
}

pub fn mat3_add(a: Mat3, b: Mat3) -> Mat3 {
    let mut out = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            out[i][j] = a[i][j] + b[i][j];
        }
    }
    out
}

pub fn mat3_transpose(m: Mat3) -> Mat3 {
    [
        [m[0][0], m[1][0], m[2][0]],
        [m[0][1], m[1][1], m[2][1]],
        [m[0][2], m[1][2], m[2][2]],
    ]
}

pub fn mat3_inverse(m: Mat3) -> Option<Mat3> {
    let det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    if !det.is_finite() || det.abs() < 1.0e-9 {
        return None;
    }
    let inv_det = 1.0 / det;
    Some([
        [
            (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * inv_det,
            (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * inv_det,
            (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * inv_det,
        ],
        [
            (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * inv_det,
            (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * inv_det,
            (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * inv_det,
        ],
        [
            (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * inv_det,
            (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * inv_det,
            (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * inv_det,
        ],
    ])
}

pub fn rotation_y(theta: f32) -> Mat3 {
    let c = libm::cosf(theta);
    let s = libm::sinf(theta);
    [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]]
}

pub fn invert_8(mut m: Mat8) -> Option<Mat8> {
    let mut inv = [[0.0; 8]; 8];
    for (i, row) in inv.iter_mut().enumerate() {
        row[i] = 1.0;
    }
    for col in 0..8 {
        let mut pivot = col;
        for row in col + 1..8 {
            if m[row][col].abs() > m[pivot][col].abs() {
                pivot = row;
            }
        }
        if !m[pivot][col].is_finite() || m[pivot][col].abs() < 1.0e-9 {
            return None;
        }
        if pivot != col {
            m.swap(col, pivot);
            inv.swap(col, pivot);
        }
        let scale = m[col][col];
        for j in 0..8 {
            m[col][j] /= scale;
            inv[col][j] /= scale;
        }
        for row in 0..8 {
            if row == col {
                continue;
            }
            let factor = m[row][col];
            for j in 0..8 {
                m[row][j] -= factor * m[col][j];
                inv[row][j] -= factor * inv[col][j];
            }
        }
    }
    Some(inv)
}
