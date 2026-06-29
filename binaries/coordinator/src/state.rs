use crate::{log_subscriber::LogSubscriber, topic_subscriber::TopicSubscriber};
use dora_coordinator_store::{CoordinatorStore, DataflowRecord, DataflowStatus};
use dora_core::config::NodeId;
use dora_message::{
    common::DaemonId,
    coordinator_to_cli::{ControlRequestReply, LogMessage},
    coordinator_to_daemon::{StateCatchUpEntry, StateCatchUpOperation},
    daemon_to_coordinator::{FaultToleranceSnapshot, NodeMetrics},
    descriptor::{Descriptor, ResolvedNode},
};
use eyre::eyre;
use std::collections::{BTreeMap, BTreeSet, HashMap};
use std::sync::Arc;
use std::time::Instant;
use tokio::sync::{Mutex, mpsc, oneshot};
use uuid::Uuid;

/// A deferred restart: the coordinator has sent `StopDataflow` and is waiting
/// for all daemons to confirm the dataflow has actually finished before starting
/// the replacement. Avoids the race where new nodes spawn before old nodes'
/// Zenoh subscribers/declarations are torn down (dora-rs/dora#2082).
pub(crate) struct PendingRestart {
    pub descriptor: Descriptor,
    pub name: Option<String>,
    pub uv: bool,
    pub reply_sender: oneshot::Sender<eyre::Result<ControlRequestReply>>,
}

#[derive(Default)]
pub(crate) struct DaemonConnections {
    daemons: BTreeMap<DaemonId, DaemonConnection>,
}

impl DaemonConnections {
    pub(crate) fn add(&mut self, daemon_id: DaemonId, connection: DaemonConnection) {
        let previous = self.daemons.insert(daemon_id.clone(), connection);
        if previous.is_some() {
            tracing::info!("closing previous connection `{daemon_id}` on new register");
        }
    }

    pub(crate) fn get_mut(&mut self, id: &DaemonId) -> Option<&mut DaemonConnection> {
        self.daemons.get_mut(id)
    }

    pub(crate) fn get_matching_daemon_id(&self, machine_id: &str) -> Option<&DaemonId> {
        self.daemons
            .keys()
            .find(|id| id.matches_machine_id(machine_id))
    }

    /// Whether the given daemon advertised support for hub-sourced git nodes
    /// (`subdir` / `hub` provenance, P2.10). Unknown daemon → `false`.
    pub(crate) fn supports_hub_sources(&self, id: &DaemonId) -> bool {
        self.daemons.get(id).is_some_and(|c| c.supports_hub_sources)
    }

    pub(crate) fn drain(&mut self) -> impl Iterator<Item = (DaemonId, DaemonConnection)> {
        std::mem::take(&mut self.daemons).into_iter()
    }

    pub(crate) fn is_empty(&self) -> bool {
        self.daemons.is_empty()
    }

    pub(crate) fn iter_mut(&mut self) -> impl Iterator<Item = (&DaemonId, &mut DaemonConnection)> {
        self.daemons.iter_mut()
    }

    pub(crate) fn remove(&mut self, daemon_id: &DaemonId) -> Option<DaemonConnection> {
        self.daemons.remove(daemon_id)
    }

    /// Returns the `connection_id` of the currently-registered connection for
    /// `daemon_id`, or `None` if the daemon is not connected.
    pub(crate) fn connection_id_of(&self, daemon_id: &DaemonId) -> Option<Uuid> {
        self.daemons.get(daemon_id).map(|c| c.connection_id)
    }

    pub(crate) fn unnamed(&self) -> impl Iterator<Item = &DaemonId> {
        self.daemons.keys().filter(|id| id.machine_id().is_none())
    }

    /// Find the first daemon whose labels are a superset of `required`.
    pub(crate) fn get_matching_daemon_by_labels(
        &self,
        required: &BTreeMap<String, String>,
    ) -> Option<&DaemonId> {
        self.daemons.iter().find_map(|(id, conn)| {
            let all_match = required.iter().all(|(k, v)| conn.labels.get(k) == Some(v));
            if all_match { Some(id) } else { None }
        })
    }
}

#[derive(Clone)]
pub(crate) struct DaemonConnection {
    pub(crate) sender: mpsc::Sender<String>,
    /// Shared with the ws_daemon handler task to resolve correlation-based replies.
    pub(crate) pending_replies: Arc<Mutex<HashMap<Uuid, oneshot::Sender<String>>>>,
    pub(crate) last_heartbeat: Instant,
    pub(crate) labels: BTreeMap<String, String>,
    /// Latest fault tolerance stats from this daemon (updated on each heartbeat).
    pub(crate) ft_stats: Option<FaultToleranceSnapshot>,
    /// Whether this daemon can build/spawn hub-sourced git nodes (`subdir` /
    /// `hub` provenance, P2.10). Reported at registration; defaults `false`
    /// for a daemon too old to advertise it, so the coordinator can refuse to
    /// route a hub node to it with a clear error.
    pub(crate) supports_hub_sources: bool,
    /// Unique ID for this connection instance. Used to distinguish a stale
    /// connection's `DaemonExit` from a freshly re-registered connection
    /// that reused the same `DaemonId` (daemon reconnect race, #2392).
    pub(crate) connection_id: Uuid,
}

impl DaemonConnection {
    pub(crate) fn new(
        sender: mpsc::Sender<String>,
        pending_replies: Arc<Mutex<HashMap<Uuid, oneshot::Sender<String>>>>,
        labels: BTreeMap<String, String>,
    ) -> Self {
        Self {
            sender,
            pending_replies,
            last_heartbeat: Instant::now(),
            labels,
            ft_stats: None,
            // set from the register request at the real registration site;
            // conservative default keeps non-registration constructors safe
            supports_hub_sources: false,
            connection_id: Uuid::new_v4(),
        }
    }

    /// Send a message to the daemon and wait for a reply.
    ///
    /// Embeds raw JSON bytes directly to preserve u128 fidelity
    /// for uhlc::ID inside timestamps.
    /// Maximum number of in-flight requests per `DaemonConnection` clone.
    ///
    /// `DaemonConnection` is cloneable and each clone keeps an independent
    /// pending-reply map, so this cap is enforced per clone rather than
    /// globally per WebSocket connection.
    const MAX_PENDING_REPLIES: usize = 256;

    pub(crate) async fn send_and_receive(&self, message: &[u8]) -> eyre::Result<Vec<u8>> {
        let id = Uuid::new_v4();
        let params_str =
            std::str::from_utf8(message).map_err(|e| eyre!("outgoing message not UTF-8: {e}"))?;
        let json = format!(r#"{{"id":"{id}","method":"daemon_command","params":{params_str}}}"#);

        let (reply_tx, reply_rx) = oneshot::channel();
        {
            let mut pending = self.pending_replies.lock().await;
            if pending.len() >= Self::MAX_PENDING_REPLIES {
                return Err(eyre!(
                    "too many pending daemon replies ({}), refusing new request",
                    pending.len()
                ));
            }
            pending.insert(id, reply_tx);
        }

        self.sender
            .send(json)
            .await
            .map_err(|_| eyre!("WS daemon send channel closed"))?;

        let response_json =
            match tokio::time::timeout(dora_message::TCP_READ_TIMEOUT, reply_rx).await {
                Ok(result) => result.map_err(|_| eyre!("daemon WS reply channel dropped"))?,
                Err(_) => {
                    self.pending_replies.lock().await.remove(&id);
                    return Err(eyre!("timeout waiting for daemon WS reply"));
                }
            };

        Ok(response_json.into_bytes())
    }

    /// Send a message to the daemon without waiting for a reply (fire-and-forget).
    ///
    /// Embeds raw JSON bytes directly to preserve u128 fidelity
    /// for uhlc::ID inside timestamps.
    pub(crate) async fn send(&self, message: &[u8]) -> eyre::Result<()> {
        let params_str =
            std::str::from_utf8(message).map_err(|e| eyre!("outgoing message not UTF-8: {e}"))?;
        let id = Uuid::new_v4();
        let json = format!(r#"{{"id":"{id}","method":"daemon_event","params":{params_str}}}"#);
        self.sender
            .send(json)
            .await
            .map_err(|_| eyre!("WS daemon send channel closed"))
    }
}

pub(crate) struct RunningBuild {
    pub(crate) errors: Vec<String>,
    pub(crate) build_result: CachedResult,

    /// Buffer for log messages that were sent before there were any subscribers.
    pub(crate) buffered_log_messages: Vec<LogMessage>,
    pub(crate) log_subscribers: Vec<LogSubscriber>,

    pub(crate) pending_build_results: BTreeSet<DaemonId>,

    /// Coordinator-local timestamp captured when this `RunningBuild` was
    /// constructed (i.e. just after the per-daemon `TriggerBuild` dispatch).
    /// Used by the heartbeat-driven watchdog to detect distributed builds
    /// that have been pending past `BUILD_RESULT_TIMEOUT` so `wait_for_build`
    /// waiters can be released with a clear error instead of hanging on the
    /// client-side RPC deadline. Mirrors `RunningDataflow::spawn_started_at`
    /// (#1465).
    pub(crate) build_started_at: Instant,
}

pub(crate) struct RunningDataflow {
    pub(crate) name: Option<String>,
    pub(crate) uuid: Uuid,
    pub(crate) descriptor: Descriptor,
    /// The IDs of the daemons that the dataflow is running on.
    pub(crate) daemons: BTreeSet<DaemonId>,
    /// IDs of daemons that are waiting until all nodes are started.
    pub(crate) pending_daemons: BTreeSet<DaemonId>,
    pub(crate) exited_before_subscribe: Vec<NodeId>,
    pub(crate) nodes: BTreeMap<NodeId, ResolvedNode>,
    /// Maps each node to the daemon it's running on
    pub(crate) node_to_daemon: BTreeMap<NodeId, DaemonId>,
    /// Latest metrics for each node (from daemons)
    pub(crate) node_metrics: BTreeMap<NodeId, NodeMetrics>,
    /// Nodes the daemon has reported as finalized via `DaemonEvent::NodeStopped`.
    /// Authoritative: any subsequent `NodeMetrics` push from the daemon for
    /// a node in this set is dropped on the floor (stale pre-exit snapshot).
    /// Covers BOTH `NodeStatus::Stopped` (clean) and `NodeStatus::Failed`
    /// (crash / final-failure) so a delayed metrics task cannot resurrect
    /// either kind of finalized row back to "Running" indefinitely.
    /// Cleared by the grace-period sweep (for Stopped) or by `AddNode`
    /// (when the same node id is added back to the dataflow).
    pub(crate) node_finalized: BTreeSet<NodeId>,
    /// When a node entered `NodeStatus::Stopped` (clean teardown only).
    /// Drives the auto-expire sweep: after the grace window the row is
    /// dropped from `node_metrics`/`node_finalized` so `dora node list`
    /// stops showing zombies. `Failed` rows are NOT inserted here — they
    /// stay visible until the dataflow itself is stopped so `dora doctor`
    /// keeps reporting the crash.
    pub(crate) node_stopped_at: BTreeMap<NodeId, Instant>,
    pub(crate) network_metrics: Option<dora_message::daemon_to_coordinator::NetworkMetrics>,

    pub(crate) spawn_result: CachedResult,
    pub(crate) stop_reply_senders: Vec<oneshot::Sender<eyre::Result<ControlRequestReply>>>,

    /// Buffer for log messages that were sent before there were any subscribers.
    pub(crate) buffered_log_messages: Vec<LogMessage>,
    pub(crate) log_subscribers: Vec<LogSubscriber>,
    pub(crate) topic_subscribers: BTreeMap<Uuid, TopicSubscriber>,

    pub(crate) pending_spawn_results: BTreeSet<DaemonId>,
    /// Coordinator-local timestamp captured when this `RunningDataflow` was
    /// constructed (i.e. just before the per-daemon spawn dispatch). Used by
    /// the heartbeat-driven watchdog to detect spawn flows that have been
    /// pending past `SPAWN_RESULT_TIMEOUT` so waiters can be released with a
    /// clear error instead of hanging on the client-side RPC deadline.
    /// Rescue of [#1593](https://github.com/dora-rs/dora/pull/1593).
    pub(crate) spawn_started_at: Instant,

    /// Timestamp (unix millis) when this dataflow was first persisted.
    pub(crate) created_at: u64,
    /// Monotonically increasing version; bumped on every persist.
    pub(crate) store_generation: u64,

    /// Per-daemon timestamp of last auto-recovery attempt (for backoff).
    pub(crate) last_recovery_attempt: BTreeMap<DaemonId, Instant>,
    /// Per-daemon timestamp of last full fallback param replay attempt
    /// (for state catch-up backoff on pruned logs).
    pub(crate) last_replay_attempt: BTreeMap<DaemonId, Instant>,

    /// Whether UV was used for this dataflow (needed for restart).
    pub(crate) uv: bool,

    // --- State catch-up replication log ---
    /// Monotonically increasing sequence counter for state mutations.
    pub(crate) state_log_sequence: u64,
    /// In-memory replication log of state mutations for incremental catch-up.
    pub(crate) state_log: Vec<StateCatchUpEntry>,
    /// Last state_log sequence acknowledged by each daemon.
    pub(crate) daemon_ack_sequence: BTreeMap<DaemonId, u64>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) enum ParamTarget {
    Running { daemon_id: DaemonId },
    // Dataflow exists in persisted store and contains the node, but is not
    // currently running. Mutations remain allowed for pre-provisioning.
    PersistedOnly,
}

pub(crate) fn resolve_param_target(
    running_dataflows: &HashMap<dora_message::DataflowId, RunningDataflow>,
    store: &dyn CoordinatorStore,
    dataflow_id: &dora_message::DataflowId,
    node_id: &NodeId,
) -> eyre::Result<ParamTarget> {
    if let Some(dataflow) = running_dataflows.get(dataflow_id) {
        let daemon_id = dataflow
            .node_to_daemon
            .get(node_id)
            .cloned()
            .ok_or_else(|| eyre::eyre!("node `{node_id}` not found in dataflow `{dataflow_id}`"))?;
        return Ok(ParamTarget::Running { daemon_id });
    }

    let Some(record) = store.get_dataflow(dataflow_id)? else {
        eyre::bail!(
            "dataflow `{dataflow_id}` not found for node `{node_id}`; \
             cannot operate params on unknown dataflow"
        );
    };

    let descriptor: dora_message::descriptor::Descriptor =
        serde_json::from_str(&record.descriptor_json).map_err(|e| {
            eyre::eyre!("failed to parse persisted descriptor for `{dataflow_id}`: {e}")
        })?;
    let node_exists = descriptor.nodes.iter().any(|n| n.id == *node_id);
    if !node_exists {
        eyre::bail!("node `{node_id}` not found in dataflow `{dataflow_id}`");
    }

    Ok(ParamTarget::PersistedOnly)
}

pub(crate) fn now_millis() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis() as u64
}

/// Maximum bytes for error strings persisted in `DataflowStatus::Failed`.
const MAX_ERROR_BYTES: usize = 4096;

/// Truncate `s` to at most `max_bytes` bytes on a UTF-8 char boundary.
pub(crate) fn truncate_str(s: &str, max_bytes: usize) -> &str {
    if s.len() <= max_bytes {
        return s;
    }
    let mut end = max_bytes;
    while end > 0 && !s.is_char_boundary(end) {
        end -= 1;
    }
    &s[..end]
}

/// Maximum number of entries in the replication log before pruning.
const MAX_STATE_LOG_ENTRIES: usize = 10_000;

impl RunningDataflow {
    /// Append a state mutation to the replication log.
    pub(crate) fn append_state_log(&mut self, operation: StateCatchUpOperation) {
        self.state_log_sequence += 1;
        self.state_log.push(StateCatchUpEntry {
            sequence: self.state_log_sequence,
            operation,
        });
        // Hard-cap: drop oldest entries if the log is too large.
        if self.state_log.len() > MAX_STATE_LOG_ENTRIES {
            let drain_count = self.state_log.len() - MAX_STATE_LOG_ENTRIES;
            tracing::warn!(
                "state replication log exceeded {MAX_STATE_LOG_ENTRIES} entries, \
                 dropping {drain_count} oldest (disconnected daemons will need full replay)"
            );
            self.state_log.drain(..drain_count);
        }
    }

    /// Prune log entries that all daemons have acknowledged.
    pub(crate) fn prune_state_log(&mut self) {
        let min_ack = self
            .daemon_ack_sequence
            .values()
            .copied()
            .min()
            .unwrap_or(0);
        self.state_log.retain(|entry| entry.sequence > min_ack);
    }

    /// Return the entries a daemon has missed since `last_ack`.
    /// Returns `None` if the log has been pruned past `last_ack` (caller
    /// should fall back to a full param replay).
    pub(crate) fn state_log_delta(&self, last_ack: u64) -> Option<Vec<StateCatchUpEntry>> {
        if self.state_log.is_empty() {
            return Some(Vec::new());
        }
        let oldest = self.state_log[0].sequence;
        if last_ack.saturating_add(1) < oldest {
            // There's a gap: entries between last_ack and oldest were pruned.
            return None;
        }
        Some(
            self.state_log
                .iter()
                .filter(|e| e.sequence > last_ack)
                .cloned()
                .collect(),
        )
    }

    /// Reconstruct the live `RunningDataflow` for a dataflow that survived a
    /// coordinator-connection drop, from its persisted record + a reconnecting
    /// daemon's status report (dora-rs/dora#2029 P1).
    ///
    /// Spawn already succeeded, so `spawn_result` is cached `Ok`; per-node
    /// metrics / log-subscriber / topic-subscriber state starts empty and the
    /// daemon re-reports/-subscribes. `node_to_daemon` is seeded from the
    /// reporting daemon's nodes; in a multi-daemon dataflow each daemon's
    /// reconnect relinks its own share (see the caller's relink branch).
    pub(crate) fn recovered(
        record: &DataflowRecord,
        descriptor: Descriptor,
        nodes: BTreeMap<NodeId, ResolvedNode>,
        daemon_id: DaemonId,
        reported_nodes: &[NodeId],
    ) -> Self {
        let daemons: BTreeSet<DaemonId> = std::iter::once(daemon_id.clone()).collect();
        let node_to_daemon = reported_nodes
            .iter()
            .map(|n| (n.clone(), daemon_id.clone()))
            .collect();
        RunningDataflow {
            uuid: record.uuid,
            name: record.name.clone(),
            descriptor,
            daemons: daemons.clone(),
            pending_daemons: BTreeSet::new(),
            exited_before_subscribe: Vec::new(),
            nodes,
            node_to_daemon,
            node_metrics: BTreeMap::new(),
            node_finalized: BTreeSet::new(),
            node_stopped_at: BTreeMap::new(),
            network_metrics: None,
            spawn_result: CachedResult::Cached {
                result: Ok(ControlRequestReply::DataflowSpawned { uuid: record.uuid }),
            },
            stop_reply_senders: Vec::new(),
            buffered_log_messages: Vec::new(),
            log_subscribers: Vec::new(),
            topic_subscribers: BTreeMap::new(),
            pending_spawn_results: BTreeSet::new(),
            spawn_started_at: Instant::now(),
            created_at: record.created_at,
            store_generation: record.generation,
            last_recovery_attempt: BTreeMap::new(),
            last_replay_attempt: BTreeMap::new(),
            uv: record.uv,
            state_log_sequence: 0,
            state_log: Vec::new(),
            daemon_ack_sequence: daemons.iter().map(|d| (d.clone(), 0)).collect(),
        }
    }

    /// Create a persistable [`DataflowRecord`] snapshot of this dataflow.
    /// Increments `store_generation` on each call.
    pub(crate) fn make_record(&mut self, status: DataflowStatus) -> eyre::Result<DataflowRecord> {
        self.store_generation += 1;
        let descriptor_json = serde_json::to_string(&self.descriptor)
            .map_err(|e| eyre::eyre!("failed to serialize descriptor: {e}"))?;
        let status = match status {
            DataflowStatus::Failed { error, terminal } => DataflowStatus::Failed {
                error: truncate_str(&error, MAX_ERROR_BYTES).to_owned(),
                terminal,
            },
            other => other,
        };
        Ok(DataflowRecord {
            uuid: self.uuid,
            name: self.name.clone(),
            descriptor_json,
            status,
            daemon_ids: self.daemons.iter().cloned().collect(),
            node_to_daemon: self
                .node_to_daemon
                .iter()
                .map(|(k, v)| (k.to_string(), v.to_string()))
                .collect(),
            uv: self.uv,
            generation: self.store_generation,
            created_at: self.created_at,
            updated_at: now_millis(),
        })
    }
}

pub(crate) enum CachedResult {
    Pending {
        result_senders: Vec<oneshot::Sender<eyre::Result<ControlRequestReply>>>,
    },
    Cached {
        result: eyre::Result<ControlRequestReply>,
    },
}

impl Default for CachedResult {
    fn default() -> Self {
        Self::Pending {
            result_senders: Vec::new(),
        }
    }
}

impl CachedResult {
    pub(crate) fn register(
        &mut self,
        reply_sender: oneshot::Sender<eyre::Result<ControlRequestReply>>,
    ) {
        match self {
            CachedResult::Pending { result_senders } => result_senders.push(reply_sender),
            CachedResult::Cached { result } => {
                Self::send_result_to(result, reply_sender);
            }
        }
    }

    pub(crate) fn set_result(&mut self, result: eyre::Result<ControlRequestReply>) {
        match self {
            CachedResult::Pending { result_senders } => {
                for sender in result_senders.drain(..) {
                    Self::send_result_to(&result, sender);
                }
                *self = CachedResult::Cached { result };
            }
            CachedResult::Cached { .. } => {}
        }
    }

    /// Returns `true` if no result has been recorded yet. Used by the
    /// heartbeat-driven spawn-timeout watchdog to decide whether to fire a
    /// rollback-with-error path on a stuck pending spawn.
    pub(crate) fn is_pending(&self) -> bool {
        matches!(self, CachedResult::Pending { .. })
    }

    /// Returns `true` if a terminally-failed result has already been cached.
    /// Used by other handlers (auto-recovery, late `DataflowSpawnResult`
    /// arms) to skip operations that would resurrect or muddle a dataflow
    /// the spawn-timeout watchdog (or any other terminal-failure path)
    /// has already marked as failed.
    pub(crate) fn is_terminal_error(&self) -> bool {
        matches!(self, CachedResult::Cached { result: Err(_) })
    }

    /// Returns `true` if a successful result has been cached, i.e. the dataflow
    /// finished spawning successfully. Used by the daemon-disconnect cleanup to
    /// gate ready-barrier release and orphaned-dataflow teardown to dataflows
    /// that are past spawn — spawn-pending ones remain the spawn-timeout
    /// watchdog's domain. See #2028.
    pub(crate) fn is_cached_ok(&self) -> bool {
        matches!(self, CachedResult::Cached { result: Ok(_) })
    }

    fn send_result_to(
        result: &eyre::Result<ControlRequestReply>,
        sender: oneshot::Sender<eyre::Result<ControlRequestReply>>,
    ) {
        let result = match result {
            Ok(r) => Ok(r.clone()),
            Err(err) => Err(eyre!("{err:?}")),
        };
        let _ = sender.send(result);
    }
}

pub(crate) struct ArchivedDataflow {
    pub(crate) name: Option<String>,
    pub(crate) nodes: BTreeMap<NodeId, ResolvedNode>,
}

impl From<&RunningDataflow> for ArchivedDataflow {
    fn from(dataflow: &RunningDataflow) -> ArchivedDataflow {
        ArchivedDataflow {
            name: dataflow.name.clone(),
            nodes: dataflow.nodes.clone(),
        }
    }
}

impl PartialEq for RunningDataflow {
    fn eq(&self, other: &Self) -> bool {
        self.name == other.name && self.uuid == other.uuid && self.daemons == other.daemons
    }
}

impl Eq for RunningDataflow {}

#[cfg(test)]
mod hub_capability_tests {
    use super::*;

    fn conn(supports_hub: bool) -> DaemonConnection {
        let (tx, _rx) = mpsc::channel(1);
        let mut c =
            DaemonConnection::new(tx, Arc::new(Mutex::new(HashMap::new())), BTreeMap::new());
        c.supports_hub_sources = supports_hub;
        c
    }

    #[test]
    fn supports_hub_sources_reflects_registration_and_unknown_is_false() {
        let mut conns = DaemonConnections::default();
        let hub_aware = DaemonId::new(Some("new".into()));
        let legacy = DaemonId::new(Some("old".into()));
        conns.add(hub_aware.clone(), conn(true));
        conns.add(legacy.clone(), conn(false));

        assert!(conns.supports_hub_sources(&hub_aware));
        assert!(!conns.supports_hub_sources(&legacy));
        // unknown daemon → false (fail-closed: refuse to route a hub node)
        assert!(!conns.supports_hub_sources(&DaemonId::new(Some("ghost".into()))));
    }
}
