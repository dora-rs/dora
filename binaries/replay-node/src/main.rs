use std::{fs::File, thread, time::Duration};

use dora_message::{common::Timestamped, daemon_to_daemon::InterDaemonEvent};
use dora_node_api::{
    DoraNode,
    arrow::array::{NullArray, make_array},
    arrow_utils::decode_arrow_ipc,
};
use dora_recording::RecordingReader;
use eyre::Context;

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

        let mut prev_offset: Option<u64> = None;
        let mut replayed = 0u64;

        while let Some(entry) = reader.next_entry()? {
            if entry.node_id != replay_node {
                continue;
            }

            // Sleep to maintain timing
            if speed > 0.0
                && let Some(prev) = prev_offset
            {
                let delta_nanos = entry.timestamp_offset_nanos.saturating_sub(prev);
                let sleep_nanos = (delta_nanos as f64 / speed) as u64;
                if sleep_nanos > 0 {
                    thread::sleep(Duration::from_nanos(sleep_nanos));
                }
            }
            prev_offset = Some(entry.timestamp_offset_nanos);

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
