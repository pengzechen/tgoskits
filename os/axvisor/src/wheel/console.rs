//! Host-side console sniff for guest voice commands (Channel A PoC).
//!
//! The SenseVoice program in the Starry guest emits recognised motion commands
//! as ordinary console lines of the form `@@RT <token>\n` (for example
//! `print("@@RT forward", flush=True)`). Every guest serial write already flows
//! through the console mux; this module observes that stream, reassembles whole
//! lines per VM, and forwards any recognised [`WheelCommand`] to the RT core
//! over the mailbox. It never consumes or rewrites the byte stream, so normal
//! console output is unaffected.
//!
//! This is a proof-of-concept transport: piggybacking on the console stream
//! avoids adding a virtio channel, at the cost of a global lock on the guest
//! output path and a fixed `@@RT ` marker. Command delivery is best-effort and
//! bounded — a full mailbox ring drops the command, matching the mailbox's own
//! back-pressure semantics.

use alloc::{collections::BTreeMap, vec::Vec};
use std::sync::{LazyLock, Mutex};

use ax_rt::{RtMessage, host_mailbox_send, rt_output_write};
use axvm::VMId;

use super::command::{TAG_WHEEL_COMMAND, WheelCommand};

/// Marker a guest writes into a console line before a command token.
const COMMAND_PREFIX: &[u8] = b"@@RT ";

/// Longest line retained while scanning for a terminator. A real command line
/// is under 20 bytes; anything longer is unrelated console output, so the
/// accumulator stops growing and simply waits for the next newline.
const MAX_LINE_LEN: usize = 96;

/// Per-VM line accumulators. Keyed by [`VMId`] so interleaved output from
/// multiple guests cannot corrupt each other's command lines.
static LINE_BUFFERS: LazyLock<Mutex<BTreeMap<VMId, Vec<u8>>>> =
    LazyLock::new(|| Mutex::new(BTreeMap::new()));

/// Observes one chunk of guest serial output, dispatching any complete `@@RT`
/// command lines it completes. Safe to call on every guest write; it never
/// blocks on the RT core and never alters the console stream.
pub fn observe_guest_output(vm_id: VMId, bytes: &[u8]) {
    let mut buffers = LINE_BUFFERS
        .lock()
        .expect("wheel console sniff mutex poisoned");
    let buffer = buffers.entry(vm_id).or_default();
    scan_commands(buffer, bytes, forward_command);
}

/// Reassembles lines from `bytes` into `buffer`, invoking `on_command` for each
/// complete line that decodes to a wheel command. Any line longer than
/// [`MAX_LINE_LEN`] stops accumulating and waits for the next terminator, so an
/// unrelated long console line cannot grow the buffer without bound.
fn scan_commands(buffer: &mut Vec<u8>, bytes: &[u8], mut on_command: impl FnMut(WheelCommand)) {
    for &byte in bytes {
        if byte == b'\n' || byte == b'\r' {
            if let Some(command) = parse_command_line(buffer) {
                on_command(command);
            }
            buffer.clear();
        } else if buffer.len() < MAX_LINE_LEN {
            buffer.push(byte);
        }
    }
}

/// Parses one reassembled line into a command, or `None` if it lacks the marker
/// or carries an unrecognised token.
fn parse_command_line(line: &[u8]) -> Option<WheelCommand> {
    let offset = line
        .windows(COMMAND_PREFIX.len())
        .enumerate()
        .find_map(|(offset, window)| {
            (window == COMMAND_PREFIX && !marker_is_inside_quoted_echo(line, offset))
                .then_some(offset)
        })?;
    let token_start = offset + COMMAND_PREFIX.len();
    let token_end = line[token_start..]
        .iter()
        .position(|byte| !byte.is_ascii_alphabetic())
        .map(|len| token_start + len)
        .unwrap_or(line.len());
    let token = &line[token_start..token_end];
    WheelCommand::from_console_token(token.trim_ascii())
}

fn marker_is_inside_quoted_echo(line: &[u8], offset: usize) -> bool {
    offset > 0 && matches!(line[offset - 1], b'\'' | b'"')
}

/// Encodes a command and pushes it to the RT mailbox, dropping it on a full
/// ring rather than blocking the guest output path.
fn forward_command(command: WheelCommand) {
    let Ok(message) = RtMessage::new(TAG_WHEEL_COMMAND, &[command.code()]) else {
        return;
    };
    if host_mailbox_send(&message).is_ok() {
        rt_output_write(b"wheel-control: host forwarded voice command\n");
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn collect_commands(chunks: &[&[u8]]) -> Vec<WheelCommand> {
        let mut buffer = Vec::new();
        let mut commands = Vec::new();
        for chunk in chunks {
            scan_commands(&mut buffer, chunk, |command| commands.push(command));
        }
        commands
    }

    #[test]
    fn plain_command_line_is_recognised() {
        assert_eq!(
            parse_command_line(b"@@RT forward"),
            Some(WheelCommand::Forward)
        );
    }

    #[test]
    fn trailing_whitespace_is_trimmed() {
        assert_eq!(
            parse_command_line(b"@@RT stop \t"),
            Some(WheelCommand::Stop)
        );
    }

    #[test]
    fn lines_without_marker_are_ignored() {
        assert_eq!(parse_command_line(b"hello forward"), None);
    }

    #[test]
    fn command_after_shell_prompt_is_recognised() {
        assert_eq!(
            parse_command_line(b"root@starry:/opt # @@RT forward"),
            Some(WheelCommand::Forward)
        );
    }

    #[test]
    fn command_after_log_fragment_is_recognised() {
        assert_eq!(
            parse_command_line(b"ip=0xf7014@@RT forward"),
            Some(WheelCommand::Forward)
        );
    }

    #[test]
    fn command_inside_python_invocation_is_ignored() {
        assert_eq!(
            parse_command_line(br#"python3 -c 'print("@@RT forward", flush=True)'"#),
            None
        );
    }

    #[test]
    fn marker_with_unknown_token_is_ignored() {
        assert_eq!(parse_command_line(b"@@RT dance"), None);
    }

    #[test]
    fn newline_terminated_command_dispatches_once() {
        assert_eq!(collect_commands(&[b"@@RT left\n"]), [WheelCommand::Left]);
    }

    #[test]
    fn command_split_across_chunks_reassembles() {
        assert_eq!(
            collect_commands(&[b"@@RT fo", b"rward\n"]),
            [WheelCommand::Forward]
        );
    }

    #[test]
    fn crlf_terminator_does_not_double_dispatch() {
        assert_eq!(collect_commands(&[b"@@RT stop\r\n"]), [WheelCommand::Stop]);
    }

    #[test]
    fn interleaved_console_noise_is_skipped() {
        assert_eq!(
            collect_commands(&[b"booting...\n@@RT right\nready\n"]),
            [WheelCommand::Right]
        );
    }

    #[test]
    fn overlong_line_without_terminator_stays_bounded() {
        let mut buffer = Vec::new();
        let flood = [b'x'; MAX_LINE_LEN * 4];
        scan_commands(&mut buffer, &flood, |_| panic!("no command expected"));
        assert_eq!(buffer.len(), MAX_LINE_LEN);
    }
}
