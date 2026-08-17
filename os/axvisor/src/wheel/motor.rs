//! Lingkong motor protocol helpers used by the hardware-proven ESP32 control.

const FRAME_HEAD: u8 = 0x3e;
const TORQUE_CONTROL_COMMAND: u8 = 0xa1;
const READ_MOTOR_STATE2_COMMAND: u8 = 0x9c;
const READ_MOTOR_STATE2_LENGTH: usize = 7;

pub const TORQUE_RESPONSE_COMMAND: u8 = TORQUE_CONTROL_COMMAND;
pub const STATE2_RESPONSE_COMMAND: u8 = READ_MOTOR_STATE2_COMMAND;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MotorProtocolError {
    BadFrame,
    HeaderChecksum,
    DataChecksum,
    WrongCommand,
    WrongMotorId,
    UnexpectedLength,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct MotorState2 {
    pub temperature_celsius: i8,
    pub iq_raw: i16,
    pub speed_raw: i16,
    pub encoder: u16,
}

impl MotorState2 {
    pub fn current_ampere(self) -> f32 {
        self.iq_raw as f32 * 3.3 / 2048.0
    }

    pub fn speed_degrees_per_second(self) -> f32 {
        self.speed_raw as f32 / 10.0
    }

    pub fn speed_radians_per_second(self) -> f32 {
        self.speed_degrees_per_second() * core::f32::consts::PI / 180.0
    }
}

pub fn build_torque_control_frame(motor_id: u8, iq_control: i16) -> [u8; 8] {
    let payload = iq_control.to_le_bytes();
    let mut frame = [0; 8];
    prepare_header(
        &mut frame[..5],
        TORQUE_CONTROL_COMMAND,
        motor_id,
        payload.len() as u8,
    );
    frame[5..7].copy_from_slice(&payload);
    frame[7] = checksum(&payload);
    frame
}

pub fn build_read_state2_frame(motor_id: u8) -> [u8; 5] {
    let mut frame = [0; 5];
    prepare_header(&mut frame, READ_MOTOR_STATE2_COMMAND, motor_id, 0);
    frame
}

pub fn parse_state2_response(
    motor_id: u8,
    command: u8,
    response: &[u8],
) -> Result<MotorState2, MotorProtocolError> {
    if response.len() != 5 + READ_MOTOR_STATE2_LENGTH + 1 || response[0] != FRAME_HEAD {
        return Err(MotorProtocolError::BadFrame);
    }
    if checksum(&response[..4]) != response[4] {
        return Err(MotorProtocolError::HeaderChecksum);
    }
    if response[1] != command {
        return Err(MotorProtocolError::WrongCommand);
    }
    if response[2] != motor_id {
        return Err(MotorProtocolError::WrongMotorId);
    }
    if response[3] as usize != READ_MOTOR_STATE2_LENGTH {
        return Err(MotorProtocolError::UnexpectedLength);
    }
    let payload = &response[5..12];
    if checksum(payload) != response[12] {
        return Err(MotorProtocolError::DataChecksum);
    }

    Ok(MotorState2 {
        temperature_celsius: payload[0] as i8,
        iq_raw: i16::from_le_bytes([payload[1], payload[2]]),
        speed_raw: i16::from_le_bytes([payload[3], payload[4]]),
        encoder: u16::from_le_bytes([payload[5], payload[6]]),
    })
}

fn prepare_header(frame: &mut [u8], command: u8, motor_id: u8, payload_len: u8) {
    frame[0] = FRAME_HEAD;
    frame[1] = command;
    frame[2] = motor_id;
    frame[3] = payload_len;
    frame[4] = checksum(&frame[..4]);
}

fn checksum(data: &[u8]) -> u8 {
    data.iter().fold(0u8, |sum, byte| sum.wrapping_add(*byte))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn torque_frame_matches_lingkong_layout() {
        assert_eq!(
            build_torque_control_frame(2, -100),
            [0x3e, 0xa1, 0x02, 0x02, 0xe3, 0x9c, 0xff, 0x9b]
        );
    }
}
