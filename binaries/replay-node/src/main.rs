use std::{fs::File, thread, time::Duration};

use dora_message::{common::Timestamped, daemon_to_daemon::InterDaemonEvent};
use dora_node_api::{
    DoraNode,
    arrow::array::{ArrayData, NullArray, make_array},
    arrow_utils::decode_arrow_ipc,
};
use dora_recording::RecordingReader;
use eyre::Context;

/// Decode a recorded output payload back into Arrow [`ArrayData`].
///
/// The recorded payload is a self-describing Arrow IPC stream, or `None` for
/// a metadata-only message (which replays as an empty null array). A decode
/// failure is returned as an `Err` so the caller can skip that single record
/// rather than aborting the whole replay.
fn decode_recorded_payload(data: Option<&[u8]>) -> eyre::Result<ArrayData> {
    match data {
        Some(bytes) => {
            decode_arrow_ipc(bytes).wrap_err("failed to decode recorded Arrow IPC payload")
        }
        None => Ok(NullArray::new(0).into()),
    }
}

/// Whether a replay pass emitted nothing usable: every matching record was
/// skipped as undecodable. This is a hard failure (a systematically corrupt or
/// format-drifted recording), distinct from a pass that legitimately matched no
/// records at all (`replayed == 0 && skipped == 0` -- the node simply had no
/// recorded output), which is not a failure.
fn replay_emitted_nothing_usable(replayed: u64, skipped: u64) -> bool {
    replayed == 0 && skipped > 0
}

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
        let mut skipped = 0u64;

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
                        skipped += 1;
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
                    let array = match decode_recorded_payload(data.as_deref()) {
                        Ok(array) => array,
                        Err(e) => {
                            // A single corrupt-but-complete payload must not
                            // abort the whole replay. Skip it, matching the
                            // bincode branch above and the recording layer's
                            // torn-record resilience (dropping a bad record
                            // rather than failing the run).
                            eprintln!(
                                "warning: skipping undecodable payload for {}/{output_id}: {e:#}",
                                entry.node_id
                            );
                            skipped += 1;
                            continue;
                        }
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

        eprintln!(
            "dora-replay-node[{replay_node}]: replayed {replayed} messages ({skipped} skipped)"
        );

        // A pass that matched records but could decode none of them emitted
        // nothing. Skipping individual corrupt records keeps replay resilient,
        // but a *systematically* undecodable recording (format drift like
        // dora-rs/dora#2366, or a truncated file) would otherwise print
        // per-record warnings, report `replayed 0`, and exit 0 -- turning a
        // data-integrity failure into a silent wrong answer. Fail instead.
        if replay_emitted_nothing_usable(replayed, skipped) {
            eyre::bail!(
                "replay of {replay_file} for node `{replay_node}` emitted nothing: \
                 all {skipped} matching record(s) were undecodable \
                 (corrupt or format-drifted recording)"
            );
        }

        if !do_loop {
            break;
        }
        eprintln!("dora-replay-node[{replay_node}]: looping...");
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::{decode_recorded_payload, pacing_sleep_nanos, replay_emitted_nothing_usable};
    use dora_node_api::arrow::array::{Array, Int32Array};
    use dora_node_api::arrow_utils::encode_arrow_ipc;

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

    #[test]
    fn metadata_only_payload_decodes_to_empty_null_array() {
        // A `None` payload (metadata-only message) replays as an empty array,
        // never an error.
        let array = decode_recorded_payload(None).expect("None must decode");
        assert_eq!(array.len(), 0);
    }

    #[test]
    fn valid_ipc_payload_round_trips() {
        // A well-formed recorded IPC stream decodes back to its data.
        let original = Int32Array::from(vec![1, 2, 3]);
        let bytes = encode_arrow_ipc(&original.to_data()).expect("encode");
        let decoded = decode_recorded_payload(Some(&bytes)).expect("valid IPC must decode");
        assert_eq!(decoded.len(), 3);
        assert_eq!(&decoded, &original.to_data());
    }

    #[test]
    fn total_failure_only_when_records_matched_but_none_decoded() {
        // Emitted something -> not a failure, regardless of skips.
        assert!(!replay_emitted_nothing_usable(5, 0));
        assert!(!replay_emitted_nothing_usable(5, 3));
        // Matched nothing at all (node had no recorded output) -> not a failure.
        assert!(!replay_emitted_nothing_usable(0, 0));
        // Matched records but decoded none of them -> hard failure.
        assert!(replay_emitted_nothing_usable(0, 1));
        assert!(replay_emitted_nothing_usable(0, 42));
    }

    #[test]
    fn corrupt_payload_is_an_error_not_a_panic() {
        // A corrupt-but-present payload returns `Err` so the caller can skip
        // that single record instead of aborting the whole replay. This is the
        // behavior the `main` loop relies on to stay resilient.
        let garbage = [0xde, 0xad, 0xbe, 0xef, 0x00, 0x01, 0x02, 0x03];
        assert!(decode_recorded_payload(Some(&garbage)).is_err());
    }
}
