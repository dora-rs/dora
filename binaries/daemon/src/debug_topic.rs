//! Topic debug streams: relaying observed outputs to coordinator-side
//! watchers (`dora topic echo`/`hz`). The `@schema` cache that lets a late
//! watcher decode Arrow IPC frames is in `dora_core::debug_topic`.

use crate::{Daemon, InterDaemonEvent, OutputId};
use aligned_vec::AVec;
use dora_message::{DataflowId, node_to_daemon::Timestamped};
use eyre::Context;

impl Daemon {
    /// Forward a debug-observed local output to coordinator debug watchers.
    ///
    /// Reconstructs the same `InterDaemonEvent::Output` frame that `send_out`
    /// produced before #1787 moved data routing off the daemon, so the CLI's
    /// `dora topic` inspection sees an unchanged wire format. Does **not**
    /// deliver to local receivers — the publishing node already did that over
    /// Zenoh.
    pub(crate) async fn handle_debug_topic_data(
        &self,
        dataflow_id: DataflowId,
        output_id: OutputId,
        metadata: dora_message::metadata::Metadata,
        data: Option<Vec<u8>>,
    ) -> eyre::Result<()> {
        // Skip (re)serialization when no CLI is currently watching this topic.
        let has_watchers = self
            .running
            .get(&dataflow_id)
            .map(|df| df.debug_topic_watchers.contains_key(&output_id))
            .unwrap_or(false);
        if !has_watchers {
            return Ok(());
        }

        let event = InterDaemonEvent::Output {
            dataflow_id,
            node_id: output_id.0.clone(),
            output_id: output_id.1.clone(),
            metadata,
            data: data.map(|d| AVec::from_slice(128, &d)),
        };
        let serialized_event = Timestamped {
            inner: event,
            timestamp: self.clock.new_timestamp(),
        }
        .serialize()
        .wrap_err("failed to serialize debug topic event")?;

        self.send_topic_debug_frames(dataflow_id, &output_id, serialized_event)
            .await
    }
}
