use std::{
    fs::File,
    time::{Duration, Instant},
};

use dora_message::{common::Timestamped, daemon_to_daemon::InterDaemonEvent};
use dora_node_api::{
    DoraArray, DoraNode, Event, EventStream, IntoArrow, TryRecvError,
    arrow_utils::decode_arrow_ipc, arrow_v59::array::NullArray,
};
use dora_recording::RecordingReader;
use eyre::Context;

/// Decode a recorded output payload back into a dora payload.
///
/// The recorded payload is a self-describing Arrow IPC stream, or `None` for
/// a metadata-only message (which replays as an empty null array). A decode
/// failure is returned as an `Err` so the caller can skip that single record
/// rather than aborting the whole replay.
fn decode_recorded_payload(data: Option<&[u8]>) -> eyre::Result<DoraArray> {
    match data {
        Some(bytes) => {
            decode_arrow_ipc(bytes).wrap_err("failed to decode recorded Arrow IPC payload")
        }
        None => Ok(NullArray::new(0).into_arrow()),
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

/// How long to wait before emitting an entry, given the time `elapsed` since
/// the current replay pass started, the entry's recording offset, and the
/// replay `speed`.
///
/// Pacing is against an absolute schedule: the entry is due `entry_offset /
/// speed` after the pass started. Waiting only the gap to the *previous*
/// entry would add every entry's decode/send time and every timer overshoot to
/// the schedule, so a long replay would drift later and later and different
/// replay nodes (which drift at different rates) would lose their relative
/// alignment. An entry that is already late is emitted immediately, letting
/// the replay catch up.
///
/// The schedule starts at 0 (recording start), so the very first entry waits
/// for its own `timestamp_offset_nanos` — the delay from recording-start to the
/// node's first output. Dropping that initial gap would emit every node's first
/// message at ~t=0 and destroy cross-node alignment on replay
/// (dora-rs/dora#2602).
fn pacing_gap(elapsed: Duration, entry_offset: u64, speed: f64) -> Duration {
    if speed <= 0.0 {
        return Duration::ZERO;
    }
    // `as u64` saturates, so a tiny `speed` cannot overflow.
    let due = Duration::from_nanos((entry_offset as f64 / speed) as u64);
    due.saturating_sub(elapsed)
}

/// Whether the replay loop should keep going or wind down.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Replay {
    /// Nothing from the daemon that concerns us; keep replaying.
    Continue,
    /// The daemon asked us to stop, or the event stream closed.
    Stop,
}

/// Classify one poll of the daemon event stream.
///
/// `None` means the stream closed, which only happens once the daemon is gone —
/// there is nobody left to replay into, so it ends the replay just like an
/// explicit `Stop` does. A timeout surfaces as [`Event::Error`] rather than
/// `None` (see [`EventStream::recv_timeout`]), so it correctly reads as
/// `Continue` here and must not be mistaken for a closed stream.
fn classify_event(event: Option<&Event>) -> Replay {
    match event {
        Some(Event::Stop(_)) => Replay::Stop,
        None => Replay::Stop,
        Some(_) => Replay::Continue,
    }
}

/// Poll the daemon for a `Stop` without blocking.
///
/// Used on the paths that have no pacing gap to wait out: `--speed 0` replays
/// as fast as possible, and the gap between two `--loop` passes is zero.
fn poll_stop(events: &mut EventStream) -> Replay {
    match events.try_recv() {
        Ok(event) => classify_event(Some(&event)),
        Err(TryRecvError::Empty) => Replay::Continue,
        Err(TryRecvError::Closed) => Replay::Stop,
    }
}

/// Wait out the pacing `gap` before the next entry, returning early if the
/// daemon sends `Stop`.
///
/// This replaces a plain `thread::sleep`: the node used to never read its event
/// stream at all, so it could not see `Stop` and kept replaying into live nodes
/// until the daemon's force-kill grace deadline. Waiting *on* the event stream
/// paces and stays responsive in one call, with no added latency — a timeout
/// costs the same as the sleep it replaces.
///
/// Events other than `Stop` (an `InputClosed`, a timeout `Error`) don't end the
/// replay, so we keep waiting out whatever is left of the gap rather than
/// returning early and pacing too fast.
fn wait_out_gap(events: &mut EventStream, gap: Duration) -> Replay {
    if gap.is_zero() {
        return poll_stop(events);
    }
    // Track elapsed rather than an absolute `Instant::now() + gap` deadline: a
    // pathologically small `--speed` can make `gap` large enough to overflow
    // that addition and panic, where the `thread::sleep` this replaced would
    // merely have slept for it.
    let start = Instant::now();
    loop {
        let Some(remaining) = gap.checked_sub(start.elapsed()) else {
            return Replay::Continue;
        };
        if remaining.is_zero() {
            return Replay::Continue;
        }
        if classify_event(events.recv_timeout(remaining).as_ref()) == Replay::Stop {
            return Replay::Stop;
        }
    }
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

    let (mut node, mut events) = DoraNode::init_from_env()?;

    loop {
        let file =
            File::open(&replay_file).wrap_err_with(|| format!("failed to open {replay_file}"))?;
        let mut reader = RecordingReader::open(file).wrap_err("failed to read recording")?;

        // Start of this pass's pacing schedule; see `pacing_gap`.
        let pass_start = Instant::now();
        let mut replayed = 0u64;
        let mut skipped = 0u64;
        let mut stopped = false;

        while let Some(entry) = reader.next_entry_for_node(&replay_node)? {
            // Wait out the pacing gap, watching for `Stop` while we do.
            let gap = pacing_gap(pass_start.elapsed(), entry.timestamp_offset_nanos, speed);
            if wait_out_gap(&mut events, gap) == Replay::Stop {
                stopped = true;
                break;
            }

            let timestamped: Timestamped<InterDaemonEvent> =
                match Timestamped::deserialize_inter_daemon_event(&entry.event_bytes) {
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
                            // event-decode branch above and the recording layer's
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
                    node.send_output(output_id, metadata.parameters, array)
                        .wrap_err("failed to send replay output")?;
                    replayed += 1;
                }
                InterDaemonEvent::OutputClosed { .. } => {
                    // Skip close events during replay
                }
                // `InterDaemonEvent` is `#[non_exhaustive]`: a recording made by a
                // newer dora may carry events this replay node predates. Skip them
                // rather than aborting an otherwise-replayable recording.
                _ => {
                    skipped += 1;
                }
            }
        }

        eprintln!(
            "dora-replay-node[{replay_node}]: replayed {replayed} messages ({skipped} skipped)"
        );

        // A stop cuts the pass short, so its counters describe a partial pass.
        // Exit `Ok` and skip the completeness check below: the pass was
        // interrupted, not defective, and failing here would turn every
        // ordinary `dora stop` into a replay error.
        if stopped {
            eprintln!("dora-replay-node[{replay_node}]: stop requested, exiting");
            break;
        }

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

        // A pass that matched no entries never reaches the poll inside the loop
        // above, so check here too — otherwise `--loop` on a recording holding
        // nothing for this node spins reopening the file with no way to stop.
        if poll_stop(&mut events) == Replay::Stop {
            eprintln!("dora-replay-node[{replay_node}]: stop requested, exiting");
            break;
        }
        eprintln!("dora-replay-node[{replay_node}]: looping...");
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::{
        Replay, classify_event, decode_recorded_payload, pacing_gap, replay_emitted_nothing_usable,
    };
    use dora_node_api::DoraArray;
    use dora_node_api::arrow_utils::encode_arrow_ipc;
    use dora_node_api::arrow_v59::array::Int32Array;
    use dora_node_api::{Event, StopCause};
    use std::time::Duration;

    #[test]
    fn stop_event_ends_the_replay() {
        // Both stop causes end the replay. `Manual` is the one that matters in
        // practice: its docs say a node must exit as soon as possible or dora
        // kills it, which is exactly what this node used to fail to do.
        assert_eq!(
            classify_event(Some(&Event::Stop(StopCause::Manual))),
            Replay::Stop
        );
        assert_eq!(
            classify_event(Some(&Event::Stop(StopCause::AllInputsClosed))),
            Replay::Stop
        );
    }

    #[test]
    fn closed_stream_ends_the_replay() {
        // `None` only comes from a closed stream — the daemon is gone, so there
        // is nobody left to replay into.
        assert_eq!(classify_event(None), Replay::Stop);
    }

    #[test]
    fn unrelated_events_do_not_end_the_replay() {
        // A pacing wait that times out surfaces as `Event::Error`, not as a
        // closed stream. Treating it as a stop would end every replay at its
        // first pacing gap, so this is the case that keeps replay working at
        // all.
        assert_eq!(
            classify_event(Some(&Event::Error("receiver timed out".to_string()))),
            Replay::Continue
        );
        assert_eq!(
            classify_event(Some(&Event::InputClosed {
                id: "some_input".into()
            })),
            Replay::Continue
        );
    }

    const MS: u64 = 1_000_000;
    const SEC: u64 = 1_000_000_000;

    #[test]
    fn first_entry_honors_its_initial_offset() {
        // A node whose first output was recorded 1s after recording-start must
        // wait ~1s before emitting it, not fire immediately (dora-rs/dora#2602).
        assert_eq!(
            pacing_gap(Duration::ZERO, SEC, 1.0),
            Duration::from_nanos(SEC)
        );
    }

    #[test]
    fn time_spent_since_the_previous_entry_is_not_waited_again() {
        // Entry due at 1.25s; 1.2s have already passed (the 1s entry was
        // emitted, and decoding/sending it plus timer overshoot took 200ms).
        // Only the remaining 50ms is waited, so that overhead does not
        // accumulate into drift over a long replay.
        assert_eq!(
            pacing_gap(Duration::from_millis(1200), 1250 * MS, 1.0),
            Duration::from_millis(50)
        );
    }

    #[test]
    fn late_entry_is_emitted_immediately() {
        assert_eq!(pacing_gap(Duration::from_secs(2), SEC, 1.0), Duration::ZERO);
        // A non-monotonic (earlier) offset is simply late: no underflow.
        assert_eq!(
            pacing_gap(Duration::from_secs(1), 500 * MS, 1.0),
            Duration::ZERO
        );
    }

    #[test]
    fn speed_scales_the_schedule() {
        // 2x speed halves the due time; a non-positive speed disables pacing.
        assert_eq!(
            pacing_gap(Duration::ZERO, SEC, 2.0),
            Duration::from_millis(500)
        );
        assert_eq!(
            pacing_gap(Duration::from_millis(400), SEC, 2.0),
            Duration::from_millis(100)
        );
        assert_eq!(pacing_gap(Duration::ZERO, SEC, 0.0), Duration::ZERO);
    }

    #[test]
    fn tiny_speed_saturates_instead_of_overflowing() {
        assert_eq!(
            pacing_gap(Duration::ZERO, u64::MAX, f64::MIN_POSITIVE),
            Duration::from_nanos(u64::MAX)
        );
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
        let original = DoraArray::from_array(Int32Array::from(vec![1, 2, 3]));
        let bytes = encode_arrow_ipc(&original).expect("encode");
        let decoded = decode_recorded_payload(Some(&bytes)).expect("valid IPC must decode");
        assert_eq!(decoded.len(), 3);
        assert_eq!(decoded.as_array(), original.as_array());
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
