//! Console → RT wheel motion command protocol.
//!
//! Voice recognition inside the guest emits short `@@RT <token>` console lines
//! (see [`super::console`]). The host decodes each line into a [`WheelCommand`],
//! forwards it across the RT mailbox as a single-byte payload under
//! [`TAG_WHEEL_COMMAND`], and the RT wheel task maps it back to a
//! [`BalanceTarget`]. Keeping the code → setpoint mapping on the RT side means
//! the untrusted guest only ever picks from a fixed, magnitude-bounded command
//! set; it can never inject a raw velocity or yaw rate.

use ax_rt::RtMessage;

use super::control::BalanceTarget;

/// Mailbox tag carrying a one-byte wheel motion command (host → RT).
///
/// Distinct from the self-test echo tag `0x01` and its `| 0x80` reply
/// convention, so voice commands never collide with mailbox diagnostics.
pub const TAG_WHEEL_COMMAND: u32 = 0x20;

/// Forward/return speed applied by a directional command, in m/s. Well inside
/// [`VEL_MAX`](super::params::VEL_MAX); the controller clamps the setpoint again
/// regardless.
const COMMAND_SPEED_MPS: f32 = 0.3;

/// Turn rate applied by a left/right command, in rad/s. Well inside
/// [`YAW_MAX`](super::params::YAW_MAX).
const COMMAND_YAW_RAD_PER_SEC: f32 = 0.6;

/// A discrete motion command the wheel controller understands.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum WheelCommand {
    /// Hold position: zero forward velocity and yaw rate.
    Stop,
    /// Drive forward at [`COMMAND_SPEED_MPS`].
    Forward,
    /// Drive backward at [`COMMAND_SPEED_MPS`].
    Backward,
    /// Turn left (positive yaw rate).
    Left,
    /// Turn right (negative yaw rate).
    Right,
}

impl WheelCommand {
    /// Decodes a console keyword (the text after the `@@RT ` prefix) into a
    /// command. Returns `None` for any unrecognised token.
    pub fn from_console_token(token: &[u8]) -> Option<Self> {
        match token {
            b"forward" | b"fwd" | b"go" => Some(Self::Forward),
            b"back" | b"backward" | b"reverse" => Some(Self::Backward),
            b"left" => Some(Self::Left),
            b"right" => Some(Self::Right),
            b"stop" | b"halt" => Some(Self::Stop),
            _ => None,
        }
    }

    /// Wire encoding for the single-byte mailbox payload.
    pub fn code(self) -> u8 {
        match self {
            Self::Stop => 0,
            Self::Forward => 1,
            Self::Backward => 2,
            Self::Left => 3,
            Self::Right => 4,
        }
    }

    /// Inverse of [`code`](Self::code); rejects unknown codes.
    pub fn from_code(code: u8) -> Option<Self> {
        match code {
            0 => Some(Self::Stop),
            1 => Some(Self::Forward),
            2 => Some(Self::Backward),
            3 => Some(Self::Left),
            4 => Some(Self::Right),
            _ => None,
        }
    }

    /// Maps the command to a balance setpoint. Height stays at its default so a
    /// motion command never changes ride height; only forward velocity and yaw
    /// rate move.
    pub fn target(self) -> BalanceTarget {
        let mut target = BalanceTarget::default();
        match self {
            Self::Stop => {}
            Self::Forward => target.velocity = COMMAND_SPEED_MPS,
            Self::Backward => target.velocity = -COMMAND_SPEED_MPS,
            Self::Left => target.yaw_rate = COMMAND_YAW_RAD_PER_SEC,
            Self::Right => target.yaw_rate = -COMMAND_YAW_RAD_PER_SEC,
        }
        target
    }
}

/// Decodes a wheel command carried by a mailbox message, or `None` if the tag
/// or payload does not match the command protocol.
pub fn decode_wheel_command(message: &RtMessage) -> Option<WheelCommand> {
    if message.tag() != TAG_WHEEL_COMMAND {
        return None;
    }
    match message.payload() {
        [code] => WheelCommand::from_code(*code),
        _ => None,
    }
}

/// Watchdog that latches a motion command for a bounded hold window.
///
/// Voice commands arrive sparsely (inference takes hundreds of ms) while the
/// control loop runs every 8 ms. A directional command is held for
/// [`HOLD_NANOS`](Self::HOLD_NANOS); once it elapses the RT task forces a stop,
/// so a lost follow-up command or a silent guest can never leave the robot
/// driving indefinitely. `Stop` is never latched.
#[derive(Clone, Copy, Debug, Default)]
pub struct CommandGate {
    hold_until_nanos: u64,
    latched: bool,
}

impl CommandGate {
    /// How long a directional command stays active before the watchdog stops.
    pub const HOLD_NANOS: u64 = 2_200_000_000;

    /// Latches `command` starting at `now`. A `Stop` disarms the watchdog.
    pub fn arm(&mut self, command: WheelCommand, now: u64) {
        self.latched = !matches!(command, WheelCommand::Stop);
        self.hold_until_nanos = now.saturating_add(Self::HOLD_NANOS);
    }

    /// Whether the hold window has elapsed and a stop should be forced.
    pub fn expired(&self, now: u64) -> bool {
        self.latched && now >= self.hold_until_nanos
    }

    /// Clears the latch after a forced stop has been applied.
    pub fn disarm(&mut self) {
        self.latched = false;
    }
}

#[cfg(test)]
mod tests {
    use super::super::params::{VEL_MAX, YAW_MAX};
    use super::*;

    const EVERY_COMMAND: [WheelCommand; 5] = [
        WheelCommand::Stop,
        WheelCommand::Forward,
        WheelCommand::Backward,
        WheelCommand::Left,
        WheelCommand::Right,
    ];

    #[test]
    fn code_round_trips_every_command() {
        for command in EVERY_COMMAND {
            assert_eq!(WheelCommand::from_code(command.code()), Some(command));
        }
    }

    #[test]
    fn unknown_code_is_rejected() {
        assert_eq!(WheelCommand::from_code(5), None);
    }

    #[test]
    fn console_tokens_map_to_commands() {
        assert_eq!(
            WheelCommand::from_console_token(b"forward"),
            Some(WheelCommand::Forward)
        );
        assert_eq!(
            WheelCommand::from_console_token(b"stop"),
            Some(WheelCommand::Stop)
        );
        assert_eq!(WheelCommand::from_console_token(b"spin"), None);
    }

    #[test]
    fn directional_targets_stay_within_limits() {
        for command in [
            WheelCommand::Forward,
            WheelCommand::Backward,
            WheelCommand::Left,
            WheelCommand::Right,
        ] {
            let target = command.target();
            assert!(target.velocity.abs() <= VEL_MAX);
            assert!(target.yaw_rate.abs() <= YAW_MAX);
        }
    }

    #[test]
    fn stop_target_is_motionless() {
        let target = WheelCommand::Stop.target();
        assert_eq!(target.velocity, 0.0);
        assert_eq!(target.yaw_rate, 0.0);
    }

    #[test]
    fn directional_command_expires_after_hold_window() {
        let mut gate = CommandGate::default();
        gate.arm(WheelCommand::Forward, 1_000);
        assert!(!gate.expired(1_000));
        assert!(!gate.expired(1_000 + CommandGate::HOLD_NANOS - 1));
        assert!(gate.expired(1_000 + CommandGate::HOLD_NANOS));
    }

    #[test]
    fn stop_command_never_latches() {
        let mut gate = CommandGate::default();
        gate.arm(WheelCommand::Stop, 1_000);
        assert!(!gate.expired(1_000 + CommandGate::HOLD_NANOS * 10));
    }

    #[test]
    fn decode_rejects_foreign_tag_and_bad_payload() {
        let good = RtMessage::new(TAG_WHEEL_COMMAND, &[WheelCommand::Left.code()]).unwrap();
        assert_eq!(decode_wheel_command(&good), Some(WheelCommand::Left));
        let wrong_tag = RtMessage::new(TAG_WHEEL_COMMAND + 1, &[1]).unwrap();
        assert_eq!(decode_wheel_command(&wrong_tag), None);
        let empty = RtMessage::new(TAG_WHEEL_COMMAND, &[]).unwrap();
        assert_eq!(decode_wheel_command(&empty), None);
    }
}
