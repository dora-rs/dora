use std::{collections::BTreeMap, path::PathBuf, sync::Arc, time::Instant};

use dashmap::DashMap;
use dora_core::{
    build::{BuildInfo, GitManager},
    uhlc::HLC,
};
use dora_message::{
    BuildId, DataflowId, SessionId,
    common::{DaemonId, NodeError},
    daemon_to_coordinator::{
        CoordinatorNotifyClient, DataflowDaemonResult, StateGetRequest, StateSetRequest,
    },
    id::NodeId,
    node_to_daemon::Timestamped,
    tarpc,
};
use tokio::sync::{Mutex, mpsc};

use crate::{Event, InterDaemonEvent, RunningDataflow};

/// Shared daemon state accessible from both the event loop and the RPC server.
///
/// Modelled after `CoordinatorState` — fields use `DashMap` for concurrent
/// access so the tarpc RPC server can read state without going through the
/// event loop's mpsc channel.
pub(crate) struct DaemonState {
    pub(crate) clock: Arc<HLC>,
    /// Set once during registration via [`set_daemon_id`].
    daemon_id: std::sync::OnceLock<DaemonId>,
    pub(crate) events_tx: mpsc::Sender<Timestamped<Event>>,

    pub(crate) running: DashMap<DataflowId, RunningDataflow>,
    pub(crate) working_dir: DashMap<DataflowId, PathBuf>,
    pub(crate) dataflow_node_results: DashMap<DataflowId, BTreeMap<NodeId, Result<(), NodeError>>>,
    pub(crate) sessions: DashMap<SessionId, BuildId>,
    pub(crate) builds: DashMap<BuildId, BuildInfo>,

    /// tarpc client for daemon→coordinator RPC (replaces raw TCP `coordinator_connection`).
    /// Set once during registration via [`set_coordinator_client`].
    coordinator_client: std::sync::OnceLock<CoordinatorNotifyClient>,
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
    pub(crate) fn new(
        clock: Arc<HLC>,
        events_tx: mpsc::Sender<Timestamped<Event>>,
        zenoh_session: Option<zenoh::Session>,
        remote_daemon_events_tx: Option<flume::Sender<eyre::Result<Timestamped<InterDaemonEvent>>>>,
    ) -> Self {
        Self {
            clock,
            daemon_id: std::sync::OnceLock::new(),
            events_tx,
            running: Default::default(),
            working_dir: Default::default(),
            dataflow_node_results: Default::default(),
            sessions: Default::default(),
            builds: Default::default(),
            coordinator_client: std::sync::OnceLock::new(),
            last_coordinator_heartbeat: Mutex::new(Instant::now()),
            git_manager: Mutex::new(Default::default()),
            zenoh_session,
            remote_daemon_events_tx,
        }
    }

    /// Create state for standalone mode (no coordinator).
    pub(crate) fn new_standalone(
        clock: Arc<HLC>,
        daemon_id: DaemonId,
        events_tx: mpsc::Sender<Timestamped<Event>>,
        zenoh_session: zenoh::Session,
        builds: BTreeMap<BuildId, BuildInfo>,
    ) -> Self {
        let state = Self {
            clock,
            daemon_id: std::sync::OnceLock::new(),
            events_tx,
            running: Default::default(),
            working_dir: Default::default(),
            dataflow_node_results: Default::default(),
            sessions: Default::default(),
            builds: {
                let map = DashMap::new();
                for (k, v) in builds {
                    map.insert(k, v);
                }
                map
            },
            coordinator_client: std::sync::OnceLock::new(),
            last_coordinator_heartbeat: Mutex::new(Instant::now()),
            git_manager: Mutex::new(Default::default()),
            zenoh_session: Some(zenoh_session),
            remote_daemon_events_tx: None,
        };
        let _ = state.daemon_id.set(daemon_id);
        state
    }

    /// Set the daemon ID after registration. Can only be called once.
    pub(crate) fn set_daemon_id(&self, id: DaemonId) {
        let _ = self.daemon_id.set(id);
    }

    /// Get the daemon ID. Panics if called before registration.
    pub(crate) fn daemon_id(&self) -> &DaemonId {
        self.daemon_id
            .get()
            .expect("daemon_id accessed before registration")
    }

    /// Set the coordinator client after registration. Can only be called once.
    pub(crate) fn set_coordinator_client(&self, client: CoordinatorNotifyClient) {
        let _ = self.coordinator_client.set(client);
    }

    /// Get the coordinator client, if set.
    pub(crate) fn coordinator_client(&self) -> Option<&CoordinatorNotifyClient> {
        self.coordinator_client.get()
    }

    /// Best-effort shared-state read through the coordinator RPC channel.
    ///
    /// Current prototype behavior:
    /// - returns `Ok(None)` if no coordinator is configured.
    /// - backend may return `Ok(None)` when no shared-state store is enabled.
    #[allow(dead_code)]
    pub(crate) async fn state_get(
        &self,
        namespace: impl Into<String>,
        key: impl Into<String>,
    ) -> Result<Option<Vec<u8>>, String> {
        let Some(client) = self.coordinator_client() else {
            return Ok(None);
        };
        client
            .state_get(
                tarpc::context::current(),
                StateGetRequest {
                    namespace: namespace.into(),
                    key: key.into(),
                },
            )
            .await
            .map_err(|err| format!("state_get RPC failed: {err}"))?
    }

    /// Best-effort shared-state write through the coordinator RPC channel.
    ///
    /// Current prototype behavior:
    /// - returns `Ok(())` if no coordinator is configured.
    /// - backend may acknowledge without persistence.
    #[allow(dead_code)]
    pub(crate) async fn state_set(
        &self,
        namespace: impl Into<String>,
        key: impl Into<String>,
        value: Vec<u8>,
    ) -> Result<(), String> {
        let Some(client) = self.coordinator_client() else {
            return Ok(());
        };
        client
            .state_set(
                tarpc::context::current(),
                StateSetRequest {
                    namespace: namespace.into(),
                    key: key.into(),
                    value,
                },
            )
            .await
            .map_err(|err| format!("state_set RPC failed: {err}"))?
    }

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

        if let Some(client) = self.coordinator_client.get() {
            let client = client.clone();
            tokio::spawn(async move {
                let _ = client
                    .all_nodes_finished(tarpc::context::current(), dataflow_id, result)
                    .await;
            });
        }
        self.running.remove(&dataflow_id);

        Ok(())
    }
}
