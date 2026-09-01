#![no_std]

//! Portable capture-stream runtime for the RK3588/ES8388 `/dev/audio0` node.
//!
//! This crate owns the OS-independent behaviour behind the character device:
//! the single-consumer read path, poll readiness, and the start/stop lifecycle
//! over a [`PcmRing`]. DMA/IRQ production (the single producer) and the StarryOS
//! `DeviceOps`/devfs adapter stay in board glue; both are thin wrappers around
//! the state machine modelled and host-tested here.

use rockchip_i2s_tdm::{CaptureFormat, PcmRing};

/// Sample count the default `/dev/audio0` ring buffers (~0.34 s at 48 kHz mono,
/// signed 16-bit). Board glue may pick another capacity via the const
/// parameter on [`AudioCaptureStream`].
pub const DEFAULT_RING_SAMPLES: usize = 16 * 1024;

/// Bytes per PCM sample in the first public format (signed 16-bit).
const BYTES_PER_SAMPLE: usize = 2;

/// Poll readiness for a would-be reader, mirroring the `readable` half of a VFS
/// poll result. Board glue maps this onto the kernel poll flags.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Readiness {
    /// A blocking reader can make progress without sleeping.
    pub readable: bool,
}

/// Outcome of a non-blocking read against the capture stream. The VFS glue
/// turns [`ReadOutcome::WouldBlock`] into either a task sleep (blocking `read`)
/// or `EAGAIN` (non-blocking `read`), and [`ReadOutcome::Inactive`] into the
/// error a read on a stopped stream should return.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReadOutcome {
    /// `n` bytes were written to the caller buffer. `n` is a whole number of
    /// samples and is 0 only when the buffer cannot hold one sample.
    Filled(usize),
    /// The stream is running but no samples are buffered yet.
    WouldBlock,
    /// The stream has not been started, so there is nothing to read.
    Inactive,
}

/// Which hardware slot(s) feed the logical mono stream the device publishes.
///
/// The RK3588 capture path always delivers interleaved stereo frames, so a mono
/// `/dev/audio0` must select or combine the two slots in software.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MonoSource {
    /// Take the left slot of each stereo frame.
    Left,
    /// Take the right slot of each stereo frame.
    Right,
    /// Average both slots (arithmetic mean, rounded toward zero).
    Downmix,
}

/// What the read path publishes given the hardware-fixed interleaved stereo
/// capture. Mono extraction happens here, in the runtime, not in the register
/// core.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OutputFormat {
    /// Extract a single logical mono channel from each stereo frame.
    Mono(MonoSource),
    /// Pass interleaved stereo frames through unchanged.
    Stereo,
}

/// OS-independent `/dev/audio0` capture runtime over a fixed-capacity ring.
///
/// The ring is single-producer/single-consumer: board glue pushes freshly
/// DMA'd, interleaved stereo samples through [`ingest`](Self::ingest) from
/// IRQ/task context, and the device read path drains the published samples
/// through [`read`](Self::read). The runtime turns hardware stereo into the
/// [`OutputFormat`] consumers see. Serialising the two contexts (an IRQ-safe
/// lock) is the glue's responsibility; this type models the behaviour, not the
/// synchronisation.
pub struct AudioCaptureStream<const CAPACITY: usize = DEFAULT_RING_SAMPLES> {
    ring: PcmRing<CAPACITY>,
    hardware_format: CaptureFormat,
    output: OutputFormat,
    running: bool,
}

impl<const CAPACITY: usize> AudioCaptureStream<CAPACITY> {
    /// Create a stopped stream capturing `hardware_format` (interleaved stereo)
    /// and publishing `output` to readers.
    pub const fn new(hardware_format: CaptureFormat, output: OutputFormat) -> Self {
        Self {
            ring: PcmRing::new(),
            hardware_format,
            output,
            running: false,
        }
    }

    /// The interleaved stereo format the hardware captures into the DMA buffer.
    pub const fn hardware_format(&self) -> CaptureFormat {
        self.hardware_format
    }

    /// How the runtime maps hardware stereo onto the published stream.
    pub const fn output_format(&self) -> OutputFormat {
        self.output
    }

    /// The PCM format the read path emits: the hardware sample rate and 16-bit
    /// width, with one channel for mono extraction or two for stereo passthrough.
    pub const fn published_format(&self) -> CaptureFormat {
        let channels = match self.output {
            OutputFormat::Mono(_) => 1,
            OutputFormat::Stereo => 2,
        };
        CaptureFormat {
            sample_rate_hz: self.hardware_format.sample_rate_hz,
            channels,
            sample_width_bits: 16,
        }
    }

    /// Whether capture is currently running.
    pub const fn is_running(&self) -> bool {
        self.running
    }

    /// Buffered bytes a reader could take right now.
    pub const fn buffered_bytes(&self) -> usize {
        self.ring.available() * BYTES_PER_SAMPLE
    }

    /// Ring overruns since construction (samples the DMA producer dropped
    /// because the reader fell behind).
    pub const fn overruns(&self) -> usize {
        self.ring.overruns()
    }

    /// Start capture. Samples buffered before the reader was ready are dropped
    /// so the first read returns fresh audio.
    pub fn start(&mut self) {
        self.ring.clear();
        self.running = true;
    }

    /// Stop capture and drop any buffered samples. Reads then report
    /// [`ReadOutcome::Inactive`] until the stream is started again.
    pub fn stop(&mut self) {
        self.running = false;
        self.ring.clear();
    }

    /// Producer path: ingest a freshly DMA'd, cache-invalidated block of
    /// interleaved stereo samples (`[l, r, l, r, ...]`) and return how many
    /// output samples were accepted into the ring. Samples arriving while the
    /// stream is stopped are dropped (return 0). A trailing lone sample that does
    /// not complete a stereo frame is ignored — the hardware only delivers whole
    /// frames.
    pub fn ingest(&mut self, samples: &[i16]) -> usize {
        if !self.running {
            return 0;
        }
        let (frames, _partial) = samples.as_chunks::<2>();
        let mut accepted = 0;
        match self.output {
            OutputFormat::Stereo => {
                for &[left, right] in frames {
                    if self.ring.push(left).is_ok() {
                        accepted += 1;
                    }
                    if self.ring.push(right).is_ok() {
                        accepted += 1;
                    }
                }
            }
            OutputFormat::Mono(source) => {
                for &[left, right] in frames {
                    let sample = match source {
                        MonoSource::Left => left,
                        MonoSource::Right => right,
                        // Widen before summing so full-scale slots do not wrap;
                        // the mean is always back in range.
                        MonoSource::Downmix => ((left as i32 + right as i32) / 2) as i16,
                    };
                    if self.ring.push(sample).is_ok() {
                        accepted += 1;
                    }
                }
            }
        }
        accepted
    }

    /// Poll readiness for a would-be reader.
    pub const fn poll(&self) -> Readiness {
        Readiness {
            readable: self.running && self.ring.available() > 0,
        }
    }

    /// Consumer path: drain buffered samples into `out` as little-endian signed
    /// 16-bit PCM. Whole samples only — at most `out.len() / 2` samples are
    /// written, so glue should offer buffers in multiples of two bytes.
    pub fn read(&mut self, out: &mut [u8]) -> ReadOutcome {
        if !self.running {
            return ReadOutcome::Inactive;
        }
        if out.len() < BYTES_PER_SAMPLE {
            return ReadOutcome::Filled(0);
        }
        if self.ring.available() == 0 {
            return ReadOutcome::WouldBlock;
        }
        let mut bytes = 0;
        let (chunks, _tail) = out.as_chunks_mut::<BYTES_PER_SAMPLE>();
        for chunk in chunks {
            let Ok(sample) = self.ring.pop() else { break };
            *chunk = sample.to_le_bytes();
            bytes += BYTES_PER_SAMPLE;
        }
        ReadOutcome::Filled(bytes)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A stream that extracts the left slot, the common `/dev/audio0` shape.
    fn mono_stream() -> AudioCaptureStream<4> {
        AudioCaptureStream::new(
            CaptureFormat::STEREO_S16_48K,
            OutputFormat::Mono(MonoSource::Left),
        )
    }

    #[test]
    fn published_format_reflects_output_over_hardware_stereo() {
        let mono = mono_stream();
        assert_eq!(mono.hardware_format(), CaptureFormat::STEREO_S16_48K);
        assert_eq!(mono.output_format(), OutputFormat::Mono(MonoSource::Left));
        assert_eq!(
            mono.published_format(),
            CaptureFormat {
                sample_rate_hz: 48_000,
                channels: 1,
                sample_width_bits: 16,
            }
        );

        let stereo =
            AudioCaptureStream::<4>::new(CaptureFormat::STEREO_S16_48K, OutputFormat::Stereo);
        assert_eq!(stereo.published_format().channels, 2);
        // The hardware format is preserved regardless of what is published.
        assert_eq!(stereo.hardware_format(), CaptureFormat::STEREO_S16_48K);
    }

    #[test]
    fn stopped_stream_reads_inactive_and_drops_ingest() {
        let mut s = mono_stream();
        assert!(!s.is_running());
        let mut buf = [0u8; 8];
        assert_eq!(s.read(&mut buf), ReadOutcome::Inactive);
        assert_eq!(s.poll(), Readiness { readable: false });
        // A stereo frame arriving while stopped is dropped, not buffered.
        assert_eq!(s.ingest(&[1, 2]), 0);
        assert_eq!(s.buffered_bytes(), 0);
    }

    #[test]
    fn running_but_empty_stream_would_block() {
        let mut s = mono_stream();
        s.start();
        let mut buf = [0u8; 8];
        assert_eq!(s.read(&mut buf), ReadOutcome::WouldBlock);
        assert_eq!(s.poll(), Readiness { readable: false });
    }

    #[test]
    fn mono_left_extraction_keeps_only_the_left_slot() {
        let mut s = mono_stream();
        s.start();
        // Two stereo frames: (0x0102, -1), (0x0304, -2). Left slots survive.
        assert_eq!(s.ingest(&[0x0102, -1, 0x0304, -2]), 2);
        assert_eq!(s.buffered_bytes(), 4);
        assert_eq!(s.poll(), Readiness { readable: true });

        let mut buf = [0u8; 8];
        assert_eq!(s.read(&mut buf), ReadOutcome::Filled(4));
        assert_eq!(&buf[..4], &[0x02, 0x01, 0x04, 0x03]);
    }

    #[test]
    fn mono_right_extraction_keeps_only_the_right_slot() {
        let mut s = AudioCaptureStream::<4>::new(
            CaptureFormat::STEREO_S16_48K,
            OutputFormat::Mono(MonoSource::Right),
        );
        s.start();
        assert_eq!(s.ingest(&[10, 20, 30, 40]), 2);
        let mut buf = [0u8; 8];
        assert_eq!(s.read(&mut buf), ReadOutcome::Filled(4));
        assert_eq!(&buf[..2], &20i16.to_le_bytes());
        assert_eq!(&buf[2..4], &40i16.to_le_bytes());
    }

    #[test]
    fn mono_downmix_averages_both_slots_without_overflow() {
        let mut s = AudioCaptureStream::<4>::new(
            CaptureFormat::STEREO_S16_48K,
            OutputFormat::Mono(MonoSource::Downmix),
        );
        s.start();
        // Full-scale slots must average to full scale, not wrap through i16.
        assert_eq!(s.ingest(&[i16::MAX, i16::MAX, 100, -50]), 2);
        let mut buf = [0u8; 8];
        assert_eq!(s.read(&mut buf), ReadOutcome::Filled(4));
        assert_eq!(&buf[..2], &i16::MAX.to_le_bytes());
        assert_eq!(&buf[2..4], &25i16.to_le_bytes()); // (100 + -50) / 2
    }

    #[test]
    fn stereo_passthrough_keeps_both_interleaved_slots() {
        let mut s =
            AudioCaptureStream::<4>::new(CaptureFormat::STEREO_S16_48K, OutputFormat::Stereo);
        s.start();
        assert_eq!(s.ingest(&[1, 2, 3, 4]), 4);
        assert_eq!(s.buffered_bytes(), 8);
        let mut buf = [0u8; 8];
        assert_eq!(s.read(&mut buf), ReadOutcome::Filled(8));
        assert_eq!(
            &buf,
            &[
                1, 0, // 1
                2, 0, // 2
                3, 0, // 3
                4, 0, // 4
            ]
        );
    }

    #[test]
    fn ingest_ignores_a_trailing_incomplete_frame() {
        let mut s = mono_stream();
        s.start();
        // Three samples = one whole frame plus a lone trailing sample (dropped).
        assert_eq!(s.ingest(&[7, 8, 9]), 1);
        assert_eq!(s.buffered_bytes(), 2);
        let mut buf = [0u8; 8];
        assert_eq!(s.read(&mut buf), ReadOutcome::Filled(2));
        assert_eq!(&buf[..2], &7i16.to_le_bytes());
    }

    #[test]
    fn read_is_bounded_by_whole_samples_in_the_caller_buffer() {
        let mut s = mono_stream();
        s.start();
        // Four stereo frames -> four mono (left) samples 10, 30, 50, 70.
        assert_eq!(s.ingest(&[10, 20, 30, 40, 50, 60, 70, 80]), 4);

        // A 3-byte buffer holds exactly one whole sample.
        let mut small = [0u8; 3];
        assert_eq!(s.read(&mut small), ReadOutcome::Filled(2));
        assert_eq!(&small[..2], &10i16.to_le_bytes());
        assert_eq!(s.buffered_bytes(), 6); // three samples left

        // A 1-byte buffer cannot hold a sample.
        let mut tiny = [0u8; 1];
        assert_eq!(s.read(&mut tiny), ReadOutcome::Filled(0));
        assert_eq!(s.buffered_bytes(), 6);
    }

    #[test]
    fn start_drops_samples_captured_before_the_reader_was_ready() {
        let mut s = mono_stream();
        s.start();
        s.ingest(&[1, 2]);
        // Re-start discards stale pre-start samples.
        s.start();
        assert_eq!(s.buffered_bytes(), 0);
        let mut buf = [0u8; 8];
        assert_eq!(s.read(&mut buf), ReadOutcome::WouldBlock);
    }

    #[test]
    fn stop_discards_buffer_and_makes_reads_inactive() {
        let mut s = mono_stream();
        s.start();
        s.ingest(&[1, 2, 3, 4]);
        s.stop();
        assert!(!s.is_running());
        assert_eq!(s.buffered_bytes(), 0);
        let mut buf = [0u8; 8];
        assert_eq!(s.read(&mut buf), ReadOutcome::Inactive);
    }

    #[test]
    fn overruns_surface_from_the_ring() {
        let mut s = mono_stream(); // capacity 4 mono samples
        s.start();
        // Six stereo frames -> six mono samples into a four-sample ring: two dropped.
        assert_eq!(s.ingest(&[1, 0, 2, 0, 3, 0, 4, 0, 5, 0, 6, 0]), 4);
        assert_eq!(s.overruns(), 2);
        assert_eq!(s.buffered_bytes(), 8);
    }
}
