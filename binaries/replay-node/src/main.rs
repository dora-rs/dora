use std::{fs::File, thread, time::Duration};

use dora_message::{common::Timestamped, daemon_to_daemon::InterDaemonEvent};
use dora_node_api::{
    DoraNode,
    arrow::array::{NullArray, make_array},
    arrow_utils::decode_arrow_ipc,
};
use dora_recording::RecordingReader;
use eyre::Context;

/// Nanoseconds to sleep before emitting an entry, given the previous entry's
/// recording offset, this entry's offset, and the replay `speed`.
///
/// `prev_offset` starts at 0 (recording start), so the very first entry sleeps
/// for its own `timestamp_offset_nanos` — the delay from recording-start to the
/// node's first output. Dropping that initial gap (by starting the baseline at
/// the first entry's own offset) would emit every node's first message at ~t=0
/// and destroy cross-node alignment on replay (dora-rs/dora#2602).
fn pacing_sleep_nanos(prev_offset: u64, entry_offset: u64, speed: f64) -> u64 {
    if speed <= 0.0 {
        return 0;
    }
    let delta_nanos = entry_offset.saturating_sub(prev_offset);
    (delta_nanos as f64 / speed) as u64
}

fn main() -> eyre::Result<()> {
    let replay_file =
        std::env::var("DORA_REPLAY_FILE").wrap_err("DORA_REPLAY_FILE env var not set")?;
    let replay_node =
        std::env::var("DORA_REPLAY_NODE").wrap_err("DORA_REPLAY_NODE env var not set")?;
    let speed: f64 = std::env::var("DORA_REPLAY_SPEED")
        .unwrap_or_else(|_| "1.0".to_string())
        .parse()
        .wrap_err("invalid DORA_REPLAY_SPEED")?;
    let do_loop = std::env::var("DORA_REPLAY_LOOP")
        .map(|v| v == "true" || v == "1")
        .unwrap_or(false);

    let (mut node, _events) = DoraNode::init_from_env()?;

    loop {
        let file =
            File::open(&replay_file).wrap_err_with(|| format!("failed to open {replay_file}"))?;
        let mut reader = RecordingReader::open(file).wrap_err("failed to read recording")?;

        // Baseline for inter-message pacing. Seeded to 0 (recording start)
        // rather than the first entry's own offset so the initial gap — each
        // node's `timestamp_offset_nanos` from recording-start to its first
        // output — is honored, preserving cross-node alignment on replay.
        let mut prev_offset: u64 = 0;
        let mut replayed = 0u64;

        while let Some(entry) = reader.next_entry()? {
            if entry.node_id != replay_node {
                continue;
            }

            // Sleep to maintain timing
            let sleep_nanos = pacing_sleep_nanos(prev_offset, entry.timestamp_offset_nanos, speed);
            if sleep_nanos > 0 {
                thread::sleep(Duration::from_nanos(sleep_nanos));
            }
            prev_offset = entry.timestamp_offset_nanos;

            // Deserialize the InterDaemonEvent from raw bincode
            let timestamped: Timestamped<InterDaemonEvent> =
                match bincode::deserialize(&entry.event_bytes) {
                    Ok(event) => event,
                    Err(e) => {
                        eprintln!(
                            "warning: failed to deserialize event for {}/{}: {e}",
                            entry.node_id, entry.output_id
                        );
                        continue;
                    }
                };

            match timestamped.inner {
                InterDaemonEvent::Output {
                    output_id,
                    metadata,
                    data,
                    ..
                } => {
                    // The recorded payload is a self-describing Arrow IPC
                    // stream (or absent for metadata-only messages). Decode it
                    // back to an array and re-send; `send_output` re-encodes it
                    // into a fresh IPC stream on the wire.
                    let array = match &data {
                        Some(bytes) => decode_arrow_ipc(bytes)
                            .wrap_err("failed to decode recorded Arrow IPC payload")?,
                        None => NullArray::new(0).into(),
                    };
                    node.send_output(output_id, metadata.parameters, make_array(array))
                        .wrap_err("failed to send replay output")?;
                    replayed += 1;
                }
                InterDaemonEvent::OutputClosed { .. } => {
                    // Skip close events during replay
                }
            }
        }

        eprintln!("dora-replay-node[{replay_node}]: replayed {replayed} messages");

        if !do_loop {
            break;
        }
        eprintln!("dora-replay-node[{replay_node}]: looping...");
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::pacing_sleep_nanos;

    #[test]
    fn first_entry_honors_its_initial_offset() {
        // A node whose first output was recorded 1s after recording-start must
        // sleep ~1s before emitting it, not fire immediately (dora-rs/dora#2602).
        let one_sec = 1_000_000_000;
        assert_eq!(pacing_sleep_nanos(0, one_sec, 1.0), one_sec);
    }

    #[test]
    fn subsequent_entries_sleep_the_inter_message_delta() {
        // From offset 1s to offset 1.25s the pacing sleeps only the 250ms gap.
        assert_eq!(
            pacing_sleep_nanos(1_000_000_000, 1_250_000_000, 1.0),
            250_000_000
        );
    }

    #[test]
    fn speed_scales_the_sleep() {
        // 2x speed halves the sleep; a non-positive speed disables pacing.
        assert_eq!(pacing_sleep_nanos(0, 1_000_000_000, 2.0), 500_000_000);
        assert_eq!(pacing_sleep_nanos(0, 1_000_000_000, 0.0), 0);
    }

    #[test]
    fn non_monotonic_offset_saturates_to_zero() {
        // A later entry with a smaller offset must not underflow into a huge sleep.
        assert_eq!(pacing_sleep_nanos(1_000_000_000, 500_000_000, 1.0), 0);
    }
}
