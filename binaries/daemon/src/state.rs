use std::{collections::BTreeMap, path::PathBuf, sync::Arc, time::Instant};

use dashmap::DashMap;
use dora_core::{
    build::{BuildInfo, GitManager},
    uhlc::HLC,
};
use dora_message::{
    BuildId, DataflowId, SessionId,
    common::{DaemonId, NodeError},
    daemon_to_coordinator::{DaemonToCoordinatorControlClient, DataflowDaemonResult},
    id::NodeId,
    node_to_daemon::Timestamped,
    tarpc,
};
use tokio::sync::{Mutex, mpsc};
use uuid::Uuid;

use crate::{Event, InterDaemonEvent, RunningDataflow};

/// Shared daemon state accessible from both the event loop and the RPC server.
///
/// Modelled after `CoordinatorState` — fields use `DashMap` for concurrent
/// access so the tarpc RPC server can read state without going through the
/// event loop's mpsc channel.
pub(crate) struct DaemonState {
    pub(crate) clock: Arc<HLC>,
    pub(crate) daemon_id: DaemonId,
    pub(crate) events_tx: mpsc::Sender<Timestamped<Event>>,

    pub(crate) running: DashMap<DataflowId, RunningDataflow>,
    pub(crate) working_dir: DashMap<DataflowId, PathBuf>,
    pub(crate) dataflow_node_results: DashMap<Uuid, BTreeMap<NodeId, Result<(), NodeError>>>,
    pub(crate) sessions: DashMap<SessionId, BuildId>,
    pub(crate) builds: DashMap<BuildId, BuildInfo>,

    /// tarpc client for daemon→coordinator RPC (replaces raw TCP `coordinator_connection`).
    pub(crate) coordinator_client: Option<DaemonToCoordinatorControlClient>,
    /// Last time we received a heartbeat from the coordinator.
    pub(crate) last_coordinator_heartbeat: Mutex<Instant>,
    /// Git clone management for builds.
    pub(crate) git_manager: Mutex<GitManager>,
    /// Zenoh session for inter-daemon communication.
    pub(crate) zenoh_session: Option<zenoh::Session>,
    /// Channel to send remote daemon events into the event loop.
    pub(crate) remote_daemon_events_tx:
        Option<flume::Sender<eyre::Result<Timestamped<InterDaemonEvent>>>>,
}

impl DaemonState {
    /// Finish a dataflow: report to coordinator and clean up state.
    ///
    /// Used by the RPC server's `stop_dataflow` handler (which doesn't have
    /// access to the logger). For the event-loop path, `Daemon::finish_dataflow`
    /// provides more detailed logging.
    pub(crate) async fn finish_dataflow(&self, dataflow_id: DataflowId) -> eyre::Result<()> {
        let result = DataflowDaemonResult {
            timestamp: self.clock.new_timestamp(),
            node_results: self
                .dataflow_node_results
                .get(&dataflow_id)
                .map(|entry| entry.value().clone())
                .unwrap_or_default(),
        };

        {
            let mut git_manager = self.git_manager.lock().await;
            git_manager
                .clones_in_use
                .values_mut()
                .for_each(|dataflows| {
                    dataflows.remove(&dataflow_id);
                });
        }

        if let Some(ref client) = self.coordinator_client {
            let ctx = tarpc::context::current();
            let _ = client.all_nodes_finished(ctx, dataflow_id, result).await;
        }
        self.running.remove(&dataflow_id);

        Ok(())
    }
}
