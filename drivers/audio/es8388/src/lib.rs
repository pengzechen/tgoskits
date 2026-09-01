#![no_std]

//! Portable ES8388 (a.k.a. ES8323) capture-path register model for the
//! Orange Pi 5 Plus ES8388 codec.
//!
//! I2C transport, MCLK/reset/GPIO handling, and the RK3588 I2S link stay in
//! board glue. This crate only turns explicit capture choices into the ordered
//! `(register, value)` writes a codec init routine must issue.
//!
//! Register addresses and the documented bit fields come from the Linux
//! `sound/soc/codecs/es8328.h` header (the ES8388 shares the ES8328 register
//! map). Values that depend on the board's analog wiring — PGA gain, the input
//! source, and the MCLK/LRCK rate field — are caller-supplied, not guessed
//! here; they must be confirmed against the schematic and Linux boot logs. See
//! `docs/audio/rk3588-es8388-capture-design.md`.

use thiserror::Error;

/// Codec I2C address on the Orange Pi 5 Plus (`es8388@11` in the device tree).
pub const ES8388_I2C_ADDR: u8 = 0x11;

// --- Register file (Linux es8328.h; ES8388 shares the ES8328 map) ---
pub const REG_CONTROL1: u8 = 0x00;
pub const REG_CONTROL2: u8 = 0x01;
pub const REG_CHIP_POWER: u8 = 0x02;
pub const REG_ADC_POWER: u8 = 0x03;
pub const REG_DAC_POWER: u8 = 0x04;
pub const REG_MASTER_MODE: u8 = 0x08;
pub const REG_ADC_CONTROL1: u8 = 0x09; // L/R PGA gain
pub const REG_ADC_CONTROL2: u8 = 0x0a; // analog input select
pub const REG_ADC_CONTROL4: u8 = 0x0c; // data format + word length
pub const REG_ADC_CONTROL5: u8 = 0x0d; // MCLK/LRCK ratio
pub const REG_ADC_CONTROL7: u8 = 0x0f; // mute / ramp
pub const REG_ADC_CONTROL8: u8 = 0x10; // ADC left digital volume
pub const REG_ADC_CONTROL9: u8 = 0x11; // ADC right digital volume

// MASTERMODE (0x08): 1 = codec drives BCLK/LRCK.
const MASTER_MODE_MSC: u8 = 1 << 7;

// CHIPPOWER (0x02) / ADCPOWER (0x03): a set bit powers the block *down*.
const CHIP_POWER_DAC_VREF_OFF: u8 = 1 << 0;
const CHIP_POWER_DAC_DLL_OFF: u8 = 1 << 2;
const CHIP_POWER_DAC_STM_RESET: u8 = 1 << 4;
const CHIP_POWER_DAC_DIG_OFF: u8 = 1 << 6;
const ADC_POWER_ADCR_OFF: u8 = 1 << 4;
const ADC_POWER_AINR_OFF: u8 = 1 << 6;

// ADCCONTROL4 (0x0c): data format in bits[1:0], word length in bits[4:2].
const ADC_FORMAT_I2S: u8 = 0;
const ADC_WORD_LENGTH_SHIFT: u8 = 2;
// ES8328 word-length encoding: 24b=000, 20b=001, 18b=010, 16b=011, 32b=100.
const ADC_WORD_LENGTH_16BIT: u8 = 0b011;

// ADCCONTROL5 (0x0d): 5-bit MCLK/LRCK ratio field.
const ADC_RATE_MASK: u8 = 0x1f;

// ADCCONTROL7 (0x0f): writing 0 leaves the ADC digital output unmuted (bit 2).

/// Largest 3 dB PGA step the analog front-end accepts (0 dB..=24 dB).
pub const MAX_PGA_GAIN_STEP: u8 = 8;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Error)]
pub enum Es8388ConfigError {
    #[error("PGA gain step must be 0..=8 (3 dB per step)")]
    PgaGainOutOfRange,
    #[error("MCLK/LRCK ratio field must fit the 5-bit ADCCONTROL5 rate field")]
    RateFieldOutOfRange,
}

/// Who drives the I2S bit and frame clocks. The Orange Pi 5 Plus sound card
/// leaves `bitclock-master`/`frame-master` unset for the ES8388 link, so the
/// codec is a clock slave; [`I2sRole::Master`] is provided for other boards.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum I2sRole {
    Master,
    Slave,
}

/// Analog input routed into the ADC. Encodings are the ADCCONTROL2 select
/// fields from the ES8328 datasheet; the board wiring that a given mic sits on
/// must be confirmed from the schematic (Main Mic is on LINPUT, Headset Mic on
/// RINPUT on this board).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AdcInput {
    Linput1,
    Linput2,
}

impl AdcInput {
    /// ADCCONTROL2 value driving the left select field; the right select mirrors
    /// it so a mono capture that later switches channel keeps a defined input.
    const fn control2(self) -> u8 {
        let sel = match self {
            AdcInput::Linput1 => 0b00,
            AdcInput::Linput2 => 0b01,
        };
        (sel << 6) | (sel << 4)
    }
}

/// Which ADC channel(s) the capture path keeps powered.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AdcChannel {
    /// Mono from the left ADC (Main Mic on LINPUT); the right ADC/AIN is powered
    /// down to save power.
    MonoLeft,
    /// Both ADC channels powered.
    Stereo,
}

impl AdcChannel {
    const fn adc_power(self) -> u8 {
        match self {
            AdcChannel::MonoLeft => ADC_POWER_ADCR_OFF | ADC_POWER_AINR_OFF,
            AdcChannel::Stereo => 0,
        }
    }
}

/// One `(register, value)` write an I2C init routine issues to the codec.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RegWrite {
    pub reg: u8,
    pub value: u8,
}

/// Number of register writes in a capture bring-up sequence.
pub const CAPTURE_WRITE_COUNT: usize = 10;

/// Byte-level I2C register write to the ES8388 at [`ES8388_I2C_ADDR`]. Board
/// glue implements this over the platform I2C controller; host tests supply a
/// recording bus so the whole bring-up sequence is exercised without hardware.
/// The seam is deliberately narrow: the device address is fixed to the codec,
/// so implementations only route the 8-bit register/value pair.
pub trait I2cBus {
    /// Transport error surfaced by the underlying I2C controller.
    type Error;

    /// Write one 8-bit `value` to the 8-bit codec register `reg`.
    fn write_register(&mut self, reg: u8, value: u8) -> Result<(), Self::Error>;
}

/// Failure applying a capture bring-up sequence over an [`I2cBus`].
#[derive(Debug, PartialEq, Eq, Error)]
pub enum Es8388BringUpError<E> {
    /// The requested capture configuration was invalid; no register was written
    /// to the bus.
    #[error(transparent)]
    Config(Es8388ConfigError),
    /// An I2C write failed. `reg` names the codec register whose write did not
    /// complete, so the caller can tell how far the sequence progressed.
    #[error("I2C write to codec register {reg:#04x} failed")]
    Bus { reg: u8, source: E },
}

/// Explicit capture configuration. The data format is fixed to the first
/// version's signed 16-bit I2S; everything analog is caller-supplied so the
/// core never bakes in board-specific magic.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CaptureSetup {
    pub role: I2sRole,
    pub input: AdcInput,
    pub channel: AdcChannel,
    /// PGA gain step, 3 dB each, `0..=MAX_PGA_GAIN_STEP`.
    pub pga_gain_step: u8,
    /// Raw 5-bit ADCCONTROL5 MCLK/LRCK ratio field (board/rate dependent).
    pub mclk_lrck_ratio_field: u8,
    /// ADCCONTROL8/9 digital volume byte (`0x00` is 0 dB in the datasheet).
    pub adc_digital_volume: u8,
}

impl CaptureSetup {
    fn validate(&self) -> Result<(), Es8388ConfigError> {
        if self.pga_gain_step > MAX_PGA_GAIN_STEP {
            return Err(Es8388ConfigError::PgaGainOutOfRange);
        }
        if self.mclk_lrck_ratio_field & !ADC_RATE_MASK != 0 {
            return Err(Es8388ConfigError::RateFieldOutOfRange);
        }
        Ok(())
    }

    /// Build the ordered capture bring-up writes: clock role, data format and
    /// word length, rate, input select, PGA and digital volume, ADC power, ADC
    /// unmute, then release the ADC out of reset while leaving the DAC off.
    pub fn register_writes(&self) -> Result<[RegWrite; CAPTURE_WRITE_COUNT], Es8388ConfigError> {
        self.validate()?;

        let master_mode = match self.role {
            I2sRole::Master => MASTER_MODE_MSC,
            I2sRole::Slave => 0,
        };
        let adc_format = ADC_FORMAT_I2S | (ADC_WORD_LENGTH_16BIT << ADC_WORD_LENGTH_SHIFT);
        let pga = (self.pga_gain_step << 4) | self.pga_gain_step;
        // Capture-only power: keep the DAC blocks off, bring the ADC up.
        let chip_power = CHIP_POWER_DAC_VREF_OFF
            | CHIP_POWER_DAC_DLL_OFF
            | CHIP_POWER_DAC_STM_RESET
            | CHIP_POWER_DAC_DIG_OFF;

        Ok([
            RegWrite {
                reg: REG_MASTER_MODE,
                value: master_mode,
            },
            RegWrite {
                reg: REG_ADC_CONTROL4,
                value: adc_format,
            },
            RegWrite {
                reg: REG_ADC_CONTROL5,
                value: self.mclk_lrck_ratio_field,
            },
            RegWrite {
                reg: REG_ADC_CONTROL2,
                value: self.input.control2(),
            },
            RegWrite {
                reg: REG_ADC_CONTROL1,
                value: pga,
            },
            RegWrite {
                reg: REG_ADC_CONTROL8,
                value: self.adc_digital_volume,
            },
            RegWrite {
                reg: REG_ADC_CONTROL9,
                value: self.adc_digital_volume,
            },
            RegWrite {
                reg: REG_ADC_POWER,
                value: self.channel.adc_power(),
            },
            RegWrite {
                reg: REG_ADC_CONTROL7,
                value: 0,
            },
            RegWrite {
                reg: REG_CHIP_POWER,
                value: chip_power,
            },
        ])
    }

    /// Drive the capture bring-up over an I2C bus: validate, then issue every
    /// register write in order. Validation runs before any bus traffic, so an
    /// invalid configuration never leaves the codec half-programmed.
    pub fn apply<B: I2cBus>(&self, bus: &mut B) -> Result<(), Es8388BringUpError<B::Error>> {
        let writes = self.register_writes().map_err(Es8388BringUpError::Config)?;
        for write in &writes {
            bus.write_register(write.reg, write.value)
                .map_err(|source| Es8388BringUpError::Bus {
                    reg: write.reg,
                    source,
                })?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn main_mic_mono() -> CaptureSetup {
        CaptureSetup {
            role: I2sRole::Slave,
            input: AdcInput::Linput1,
            channel: AdcChannel::MonoLeft,
            pga_gain_step: 4,
            mclk_lrck_ratio_field: 0x02,
            adc_digital_volume: 0x00,
        }
    }

    fn value_at(writes: &[RegWrite; CAPTURE_WRITE_COUNT], reg: u8) -> u8 {
        writes.iter().find(|w| w.reg == reg).unwrap().value
    }

    #[test]
    fn capture_sequence_encodes_slave_i2s_mono_left() {
        let writes = main_mic_mono().register_writes().unwrap();

        let order = [
            REG_MASTER_MODE,
            REG_ADC_CONTROL4,
            REG_ADC_CONTROL5,
            REG_ADC_CONTROL2,
            REG_ADC_CONTROL1,
            REG_ADC_CONTROL8,
            REG_ADC_CONTROL9,
            REG_ADC_POWER,
            REG_ADC_CONTROL7,
            REG_CHIP_POWER,
        ];
        for (write, &reg) in writes.iter().zip(order.iter()) {
            assert_eq!(write.reg, reg);
        }

        assert_eq!(value_at(&writes, REG_MASTER_MODE), 0x00); // slave
        assert_eq!(value_at(&writes, REG_ADC_CONTROL4), 0x0c); // 16-bit I2S
        assert_eq!(value_at(&writes, REG_ADC_CONTROL5), 0x02);
        assert_eq!(value_at(&writes, REG_ADC_CONTROL2), 0x00); // LINPUT1 on both selects
        assert_eq!(value_at(&writes, REG_ADC_CONTROL1), 0x44); // 12 dB on L and R PGA
        assert_eq!(value_at(&writes, REG_ADC_POWER), 0x50); // right ADC/AIN powered down
        assert_eq!(value_at(&writes, REG_ADC_CONTROL7), 0x00); // unmuted
        assert_eq!(value_at(&writes, REG_CHIP_POWER), 0x55); // DAC off, ADC on
    }

    #[test]
    fn master_role_sets_msc_bit() {
        let writes = CaptureSetup {
            role: I2sRole::Master,
            ..main_mic_mono()
        }
        .register_writes()
        .unwrap();
        assert_eq!(value_at(&writes, REG_MASTER_MODE), MASTER_MODE_MSC);
    }

    #[test]
    fn stereo_keeps_both_adc_channels_powered() {
        let writes = CaptureSetup {
            channel: AdcChannel::Stereo,
            ..main_mic_mono()
        }
        .register_writes()
        .unwrap();
        assert_eq!(value_at(&writes, REG_ADC_POWER), 0x00);
    }

    #[test]
    fn linput2_selects_second_analog_input() {
        let writes = CaptureSetup {
            input: AdcInput::Linput2,
            ..main_mic_mono()
        }
        .register_writes()
        .unwrap();
        assert_eq!(
            value_at(&writes, REG_ADC_CONTROL2),
            (0b01 << 6) | (0b01 << 4)
        );
    }

    #[test]
    fn out_of_range_inputs_are_rejected() {
        assert_eq!(
            CaptureSetup {
                pga_gain_step: MAX_PGA_GAIN_STEP + 1,
                ..main_mic_mono()
            }
            .register_writes(),
            Err(Es8388ConfigError::PgaGainOutOfRange)
        );
        assert_eq!(
            CaptureSetup {
                mclk_lrck_ratio_field: 0x20,
                ..main_mic_mono()
            }
            .register_writes(),
            Err(Es8388ConfigError::RateFieldOutOfRange)
        );
    }

    /// Recording I2C bus: appends each `(reg, value)` write in order, and can be
    /// primed to fail at a chosen step so error propagation is exercised without
    /// hardware.
    #[derive(Debug, PartialEq, Eq, Error)]
    #[error("mock I2C bus failure")]
    struct MockBusError;

    struct RecordingBus {
        writes: [(u8, u8); CAPTURE_WRITE_COUNT],
        count: usize,
        fail_at: Option<usize>,
    }

    impl RecordingBus {
        const fn new() -> Self {
            Self {
                writes: [(0, 0); CAPTURE_WRITE_COUNT],
                count: 0,
                fail_at: None,
            }
        }

        const fn failing_at(index: usize) -> Self {
            Self {
                writes: [(0, 0); CAPTURE_WRITE_COUNT],
                count: 0,
                fail_at: Some(index),
            }
        }
    }

    impl I2cBus for RecordingBus {
        type Error = MockBusError;

        fn write_register(&mut self, reg: u8, value: u8) -> Result<(), Self::Error> {
            if self.fail_at == Some(self.count) {
                return Err(MockBusError);
            }
            self.writes[self.count] = (reg, value);
            self.count += 1;
            Ok(())
        }
    }

    #[test]
    fn apply_drives_full_sequence_in_order() {
        let setup = main_mic_mono();
        let mut bus = RecordingBus::new();
        assert_eq!(setup.apply(&mut bus), Ok(()));

        let expected = setup.register_writes().unwrap();
        assert_eq!(bus.count, CAPTURE_WRITE_COUNT);
        for (recorded, write) in bus.writes.iter().zip(expected.iter()) {
            assert_eq!(*recorded, (write.reg, write.value));
        }
    }

    #[test]
    fn apply_reports_the_register_whose_bus_write_failed() {
        let mut bus = RecordingBus::failing_at(3);
        let expected = main_mic_mono().register_writes().unwrap();

        assert_eq!(
            main_mic_mono().apply(&mut bus),
            Err(Es8388BringUpError::Bus {
                reg: expected[3].reg,
                source: MockBusError,
            })
        );
        // The bus stops at the failed write: three writes recorded before it.
        assert_eq!(bus.count, 3);
    }

    #[test]
    fn apply_rejects_invalid_config_before_touching_the_bus() {
        let setup = CaptureSetup {
            pga_gain_step: MAX_PGA_GAIN_STEP + 1,
            ..main_mic_mono()
        };
        let mut bus = RecordingBus::new();
        assert_eq!(
            setup.apply(&mut bus),
            Err(Es8388BringUpError::Config(
                Es8388ConfigError::PgaGainOutOfRange
            ))
        );
        assert_eq!(bus.count, 0);
    }
}
