//! Wheeled bipedal robot control port from the ESP32 WBR Control project.
//!
//! This module is intentionally split from the existing RT device demos. The
//! ESP32 control loop was a single Arduino loop that mixed command parsing,
//! blocking I/O, EKF/LQR computation, servo writes, motor reads, and logging.
//! The Axvisor RT port keeps only deterministic control math and protocol
//! helpers here; concrete I2C/UART drivers must enter through deadline-bounded
//! adapters before this can run as a 5 ms balance loop.

mod control;
mod controller;
mod ekf;
mod hardware;
mod imu;
mod math;
mod model;
mod motor;
mod params;
mod servo;

pub use control::*;
pub use controller::*;
pub use ekf::*;
pub use hardware::*;
pub use imu::*;
pub use model::*;
pub use motor::*;
pub use params::*;
pub use servo::*;
