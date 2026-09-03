use std::collections::BTreeMap;

pub use crate::common::{
    DataMessage, LogLevel, LogMessage, NodeError, NodeErrorCause, NodeExitStatus, Timestamped,
};
use crate::{
    BuildId, DataflowId, common::DaemonId, current_crate_version, id::NodeId, metadata::Metadata,
    versions_compatible,
};

/// Per-dataflow status reported by a daemon after (re-)registration.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct DataflowStatusEntry {
    pub dataflow_id: uuid::Uuid,
    pub running_nodes: Vec<NodeId>,
}

#[allow(clippy::large_enum_variant)]
#[derive(Debug, serde::Serialize, serde::Deserialize)]
#[non_exhaustive]
pub enum CoordinatorRequest {
    Register(DaemonRegisterRequest),
    Event {
        daemon_id: DaemonId,
        event: DaemonEvent,
    },
    /// Resolve a machine id to a registered daemon (cross-machine pools).
    ResolveMachine {
        machine_id: String,
    },
}

#[derive(Debug, serde::Serialize, serde::Deserialize)]
pub struct DaemonRegisterRequest {
    dora_version: semver::Version,
    pub machine_id: Option<String>,
    #[serde(default)]
    pub labels: BTreeMap<String, String>,
    /// Whether this daemon understands hub-sourced git nodes — the `subdir`
    /// and `hub` provenance fields on a `GitSource` (spec §10.2, P2.10).
    ///
    /// `#[serde(default)]` makes this `false` for a daemon built before the
    /// field existed, so the coordinator can refuse to route a hub node to it
    /// with a clear error. This is the capability signal the `dora_version`
    /// gate cannot provide: during the `1.0.0-rc` window a pre-hub daemon and a
    /// hub-aware coordinator report the *same* version, so version alone can't
    /// distinguish them — an explicit flag can.
    #[serde(default)]
    supports_hub_sources: bool,

    /// Layout version of [`Metadata`], which every `InterDaemonEvent::Output`
    /// carries between daemons over zenoh.
    ///
    /// The coordinator is the only chokepoint that sees every daemon: there is
    /// no daemon-to-daemon connection to handshake on, because that path is
    /// zenoh pub/sub. Gating registration therefore gates the daemon-to-daemon
    /// wire transitively — any two daemons routing a dataflow have both passed
    /// this check, so they agree on the `Metadata` layout.
    ///
    /// `dora_version` alone does not cover this: #2366 dropped a `Metadata`
    /// field without changing the version, and the resulting mid-stream desync
    /// was #2742. The node-to-daemon path already gates this
    /// ([`crate::node_to_daemon::NodeRegisterRequest::check_version`]); this is
    /// the same gate one hop out.
    ///
    /// `#[serde(default)]` makes a pre-field daemon report 0 and fail the check
    /// with a legible message. That works because this frame is JSON over the
    /// coordinator WebSocket — a non-self-describing encoding would fail while
    /// decoding the frame that carries the field, never reaching the check.
    #[serde(default)]
    metadata_version: u16,

    /// The zenoh endpoint this daemon is about to bind, so the coordinator can
    /// hand it to daemons that register after this one.
    ///
    /// Carried *in the registration* rather than reported once the session is
    /// open, and that timing is the whole point. The coordinator handles
    /// registrations one at a time on its event loop, so an endpoint that
    /// arrives with the registration is already on record before the next
    /// daemon's reply is built — two daemons starting simultaneously are
    /// ordered by that loop and the later one always learns about the earlier.
    /// Reporting after the session opened instead left a window (register →
    /// listener bound) in which both daemons could register, each be handed a
    /// list without the other, and stay partitioned: zenoh reads
    /// `connect/endpoints` once at session open, so neither could act on the
    /// other's later report.
    ///
    /// The port is reserved before registering, so this is the endpoint the
    /// daemon *will* bind rather than one it has bound. A bind that then fails
    /// verification is withdrawn with
    /// [`DaemonEvent::ZenohListenEndpoint`]`(None)`, so the coordinator never
    /// keeps handing out an endpoint with nothing behind it for long.
    ///
    /// `None` for a daemon with no dialable listener — a single-machine
    /// deployment (loopback, which would point a remote peer at its own host)
    /// or a failed reservation.
    #[serde(default)]
    pub zenoh_listen_endpoint: Option<String>,
}

impl DaemonRegisterRequest {
    pub fn new(machine_id: Option<String>, labels: BTreeMap<String, String>) -> Self {
        Self::with_zenoh_endpoint(machine_id, labels, None)
    }

    /// [`Self::new`] plus the zenoh endpoint this daemon will bind; see
    /// [`Self::zenoh_listen_endpoint`].
    pub fn with_zenoh_endpoint(
        machine_id: Option<String>,
        labels: BTreeMap<String, String>,
        zenoh_listen_endpoint: Option<String>,
    ) -> Self {
        Self {
            dora_version: current_crate_version(),
            machine_id,
            labels,
            // a daemon built from this crate understands hub git sources
            supports_hub_sources: true,
            metadata_version: Metadata::CURRENT_VERSION,
            zenoh_listen_endpoint,
        }
    }

    /// Whether the registering daemon can build/spawn hub-sourced git nodes
    /// (carrying `subdir` / `hub` provenance).
    pub fn supports_hub_sources(&self) -> bool {
        self.supports_hub_sources
    }

    pub fn check_version(&self) -> Result<(), String> {
        let crate_version = current_crate_version();
        let specified_version = &self.dora_version;

        if versions_compatible(&crate_version, specified_version)? {
            // Even when semver matches, the payload layout can differ within a
            // release series (#2366). Reject here so the failure is a legible
            // registration error rather than a mid-stream desync between
            // daemons routing the same dataflow (#2742).
            if self.metadata_version != Metadata::CURRENT_VERSION {
                return Err(format!(
                    "message wire-format mismatch: this daemon speaks metadata format v{} \
                     but the coordinator speaks v{}. The daemon and coordinator were built \
                     from dora revisions with incompatible message formats; rebuild both \
                     from the same revision.",
                    self.metadata_version,
                    Metadata::CURRENT_VERSION
                ));
            }
            Ok(())
        } else {
            // Direction-aware remediation: `versions_compatible` rejects both
            // older and newer daemons, so the fix differs. Upgrade whichever
            // side is older.
            let remedy = if *specified_version < crate_version {
                format!(
                    "upgrade the daemon to match the coordinator (e.g. \
                     `cargo install dora-cli --version {crate_version}`) — an older daemon \
                     also lacks newer wire features such as hub `subdir`/`hub:` node sources"
                )
            } else {
                format!(
                    "upgrade the coordinator to dora v{specified_version} (or run an older \
                     daemon) so both sides match"
                )
            };
            Err(format!(
                "version mismatch: this daemon runs dora v{specified_version} but the \
                 coordinator expects v{crate_version} — these dora versions are incompatible. \
                 {remedy}.",
            ))
        }
    }
}

#[cfg(test)]
mod register_version_tests {
    use super::*;

    #[test]
    fn current_version_is_compatible() {
        assert!(
            DaemonRegisterRequest::new(None, Default::default())
                .check_version()
                .is_ok()
        );
    }

    fn request_with_version(dora_version: semver::Version) -> DaemonRegisterRequest {
        DaemonRegisterRequest {
            dora_version,
            machine_id: None,
            labels: Default::default(),
            supports_hub_sources: true,
            metadata_version: Metadata::CURRENT_VERSION,
            zenoh_listen_endpoint: None,
        }
    }

    #[test]
    fn same_version_daemon_with_a_different_metadata_layout_is_rejected() {
        // The gap this closes: semver alone let #2366 through, where a
        // `Metadata` field was dropped without a version bump. Two daemons
        // routing one dataflow exchange `InterDaemonEvent::Output` over zenoh,
        // which carries `Metadata` — and zenoh pub/sub has no connection to
        // handshake on, so the coordinator is the only place this can be
        // caught. A same-version daemon with a stale layout must be rejected
        // here rather than desyncing mid-stream (#2742).
        let mut req = DaemonRegisterRequest::new(None, Default::default());
        req.metadata_version = Metadata::CURRENT_VERSION.wrapping_add(1);

        let err = req
            .check_version()
            .expect_err("a metadata layout mismatch must be rejected");
        assert!(err.contains("wire-format mismatch"), "{err}");
        assert!(
            err.contains("rebuild both"),
            "the error should say how to fix it: {err}"
        );
    }

    #[test]
    fn a_daemon_predating_the_field_is_rejected_with_a_legible_error() {
        // The frame is JSON over the coordinator WebSocket, so a daemon built
        // before `metadata_version` existed simply omits it and `serde(default)`
        // yields 0. That must fail the gate with a real message rather than
        // being silently treated as compatible.
        let json = serde_json::to_string(&DaemonRegisterRequest::new(None, Default::default()))
            .expect("serialize");
        let stripped: serde_json::Value = {
            let mut v: serde_json::Value = serde_json::from_str(&json).unwrap();
            v.as_object_mut().unwrap().remove("metadata_version");
            v
        };
        let old: DaemonRegisterRequest =
            serde_json::from_value(stripped).expect("a pre-field daemon must still deserialize");
        assert_eq!(old.metadata_version, 0);
        assert!(
            old.check_version().is_err(),
            "a pre-field daemon must not be treated as compatible"
        );
    }

    #[test]
    fn incompatible_daemon_gets_direction_aware_upgrade_advice() {
        // `versions_compatible` rejects both older and newer daemons, so the
        // remediation must name the right side. (Cross-version only — a
        // *same-version* pre-hub daemon passes this gate, which is why hub
        // capability is signalled explicitly via `supports_hub_sources`.)
        let current = current_crate_version();

        // A NEWER daemon than the coordinator → upgrade the *coordinator*.
        let err = request_with_version(semver::Version::new(current.major + 1, 0, 0))
            .check_version()
            .expect_err("newer-major daemon must be rejected");
        assert!(err.contains("version mismatch"), "{err}");
        assert!(
            err.contains("upgrade the coordinator"),
            "newer daemon should advise upgrading the coordinator: {err}"
        );

        // An OLDER daemon than the coordinator → upgrade the *daemon*.
        let err = request_with_version(semver::Version::new(0, 1, 0))
            .check_version()
            .expect_err("older daemon must be rejected");
        assert!(
            err.contains("upgrade the daemon"),
            "older daemon should advise upgrading the daemon: {err}"
        );
    }

    #[test]
    fn hub_capability_is_advertised_by_current_daemons_and_defaults_off() {
        // A daemon built from this crate advertises hub support.
        assert!(DaemonRegisterRequest::new(None, Default::default()).supports_hub_sources());

        // A daemon built before the field existed sends a request without it;
        // `#[serde(default)]` must decode that as "no hub support" so the
        // coordinator refuses to route hub nodes to it. This is the same-version
        // gap the version check cannot catch.
        let legacy = r#"{"dora_version":"1.0.0-rc1","machine_id":null,"labels":{}}"#;
        let decoded: DaemonRegisterRequest = serde_json::from_str(legacy).unwrap();
        assert!(!decoded.supports_hub_sources());
    }
}

#[derive(Debug, serde::Serialize, serde::Deserialize)]
#[non_exhaustive]
pub enum DaemonEvent {
    BuildResult {
        build_id: BuildId,
        result: Result<(), String>,
    },
    SpawnResult {
        dataflow_id: DataflowId,
        result: Result<(), String>,
    },
    AllNodesReady {
        dataflow_id: DataflowId,
        exited_before_subscribe: Vec<NodeId>,
    },
    AllNodesFinished {
        dataflow_id: DataflowId,
        result: DataflowDaemonResult,
    },
    Heartbeat {
        #[serde(default)]
        ft_stats: Option<FaultToleranceSnapshot>,
    },
    /// The zenoh endpoint this daemon actually bound and is reachable at,
    /// sent once its session is open.
    ///
    /// The coordinator records it and hands it to daemons that register later
    /// (see `RegisterResult::Ok::peer_zenoh_endpoints`), which is what lets a
    /// multi-machine deployment wire itself without every daemon being told
    /// every other daemon's address.
    ///
    /// Confirms or withdraws the endpoint this daemon advertised in its
    /// registration, once its zenoh session is open and the listener has been
    /// verified against `info().locators()`.
    ///
    /// `Some(endpoint)` confirms (and would correct a differing one);
    /// `None` withdraws, which is what a daemon whose listener did not bind
    /// must do so the coordinator stops handing out a dead endpoint.
    ///
    /// The registration carries the endpoint in the first place — see
    /// [`DaemonRegisterRequest::zenoh_listen_endpoint`] for why it cannot wait
    /// until here. This is the correction, not the announcement.
    ZenohListenEndpoint {
        endpoint: Option<String>,
    },
    /// Sent by the daemon after registration to report its current state.
    /// Enables coordinator-daemon reconciliation on reconnect.
    StatusReport {
        running_dataflows: Vec<DataflowStatusEntry>,
    },
    Log(LogMessage),
    Exit,
    NodeMetrics {
        dataflow_id: DataflowId,
        metrics: BTreeMap<NodeId, NodeMetrics>,
        #[serde(default)]
        network: Option<NetworkMetrics>,
    },
    /// Topic debug payload destined for one or more active CLI subscriptions.
    ///
    /// Daemon and coordinator are co-deployed from the same build, so this
    /// multi-subscriber shape is safe to evolve within the repository.
    TopicDebugData {
        dataflow_id: DataflowId,
        subscription_ids: Vec<uuid::Uuid>,
        payload: Vec<u8>,
    },
    /// Daemon acknowledges state catch-up through a given sequence number.
    StateCatchUpAck {
        dataflow_id: DataflowId,
        ack_sequence: u64,
    },
    /// Sent by the daemon when a node has exited and the daemon will NOT
    /// restart it (e.g. `dora node stop`, a node exiting under
    /// `restart_policy: Never`, or a final-failure cascade). The
    /// coordinator uses this to invalidate its cached `node_metrics`
    /// entry so `dora node list` reflects the actual state instead of
    /// the last-reported "Running" snapshot. Without this signal the
    /// daemon's metrics-snapshot loop simply stops including the dead
    /// node and the coordinator's cache is frozen at the last
    /// pre-exit values forever.
    NodeStopped {
        dataflow_id: DataflowId,
        node_id: NodeId,
        /// `true` if the daemon called `disable_restart()` before the
        /// exit (i.e. the `stop_single_node` / `restart_single_node`
        /// path triggered by `dora node stop`/`restart`). `false` for
        /// a final-failure exit under `restart_policy: Never` or a
        /// `max_restarts` exhaustion. The coordinator uses this to
        /// pick `NodeStatus::Stopped` vs `NodeStatus::Failed`, so a
        /// crash is not silently reported as a clean teardown (which
        /// would hide it from `dora doctor`).
        #[serde(default)]
        clean_stop: bool,
    },
}

/// Health status of a node
#[derive(Debug, Clone, Default, serde::Serialize, serde::Deserialize, PartialEq, Eq)]
#[serde(rename_all = "kebab-case")]
pub enum NodeStatus {
    #[default]
    Running,
    Restarting,
    /// One or more inputs have timed out (circuit breaker open)
    Degraded,
    Failed,
    /// Node was cleanly stopped (e.g. via `dora node stop`) and the
    /// process has exited. Distinguishes a deliberate teardown from a
    /// crash failure. Coordinator-side entries with this status are
    /// removed after `NODE_STOPPED_GRACE_PERIOD` so `dora node list`
    /// eventually stops showing zombies.
    Stopped,
}

impl std::fmt::Display for NodeStatus {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            NodeStatus::Running => write!(f, "Running"),
            NodeStatus::Restarting => write!(f, "Restarting"),
            NodeStatus::Degraded => write!(f, "Degraded"),
            NodeStatus::Failed => write!(f, "Failed"),
            NodeStatus::Stopped => write!(f, "Stopped"),
        }
    }
}

/// Snapshot of daemon-level fault tolerance counters
#[derive(Debug, Clone, Default, serde::Serialize, serde::Deserialize)]
pub struct FaultToleranceSnapshot {
    pub restarts: u64,
    pub health_check_kills: u64,
    pub input_timeouts: u64,
    pub circuit_breaker_recoveries: u64,
    #[serde(default)]
    pub startup_timeout_kills: u64,
}

/// Resource metrics for a node process
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct NodeMetrics {
    /// Process ID
    pub pid: u32,
    /// CPU usage percentage (0-100 per core)
    pub cpu_usage: f32,
    /// Memory usage in bytes
    pub memory_bytes: u64,
    /// Disk read bytes per second (if available)
    pub disk_read_bytes: Option<u64>,
    /// Disk write bytes per second (if available)
    pub disk_write_bytes: Option<u64>,
    /// Number of times this node has been restarted
    #[serde(default)]
    pub restart_count: u32,
    /// Input IDs that have timed out (circuit breaker open)
    #[serde(default)]
    pub broken_inputs: Vec<String>,
    /// Current health status
    #[serde(default)]
    pub status: NodeStatus,
    /// Number of pending messages in the node's input queue
    #[serde(default)]
    pub pending_messages: u64,
}

/// Per-dataflow network I/O counters for cross-daemon Zenoh traffic.
#[derive(Debug, Clone, Default, serde::Serialize, serde::Deserialize)]
pub struct NetworkMetrics {
    pub bytes_sent: u64,
    pub bytes_received: u64,
    pub messages_sent: u64,
    pub messages_received: u64,
    #[serde(default)]
    pub publish_failures: u64,
}

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub struct DataflowDaemonResult {
    pub timestamp: uhlc::Timestamp,
    pub node_results: BTreeMap<NodeId, Result<(), NodeError>>,
}

impl DataflowDaemonResult {
    pub fn is_ok(&self) -> bool {
        self.node_results.values().all(|r| r.is_ok())
    }
}

#[derive(Debug, serde::Deserialize, serde::Serialize)]
pub enum DaemonCoordinatorReply {
    TriggerBuildResult(Result<(), String>),
    TriggerSpawnResult(Result<(), String>),
    ReloadResult(Result<(), String>),
    StopResult(Result<(), String>),
    DestroyResult {
        result: Result<(), String>,
        #[serde(skip)]
        notify: Option<tokio::sync::oneshot::Sender<()>>,
    },
    Logs(Result<Vec<u8>, String>),
    /// Reply for `DaemonCoordinatorEvent::AddNode`. Previously the daemon
    /// returned `None` and the coordinator accepted any successful TCP
    /// response as proof that AddNode applied, even a `SetParamResult` or
    /// other unrelated reply — committing state for a node the daemon
    /// may have rejected (#1682). This variant lets the coordinator
    /// pattern-match a specific reply and forward daemon errors to the
    /// CLI instead of corrupting the dataflow state. Rescue of #1757.
    AddNodeResult(Result<(), String>),
    RestartNodeResult(Result<(), String>),
    StopNodeResult(Result<(), String>),
    RemoveNodeResult(Result<(), String>),
    /// Reply for `DaemonCoordinatorEvent::ReplaceNode`. Same
    /// specific-reply contract as `AddNodeResult` (#1682): the
    /// coordinator only commits its descriptor update after matching
    /// this exact variant.
    ReplaceNodeResult(Result<(), String>),
    /// Reply for `DaemonCoordinatorEvent::AddMapping`. Previously the daemon
    /// returned `None`, which the coordinator's WS layer skipped instead
    /// of forwarding as a reply, causing `send_and_receive` to time out
    /// after 30s with `daemon dispatch failed: timeout waiting for daemon
    /// WS reply`. Same bug class as #1682's AddNode silent-reply hole;
    /// applied to mappings here.
    AddMappingResult(Result<(), String>),
    /// Reply for `DaemonCoordinatorEvent::RemoveMapping`. See
    /// `AddMappingResult` doc for the silent-reply bug class.
    RemoveMappingResult(Result<(), String>),
    SetParamResult(Result<(), String>),
    DeleteParamResult(Result<(), String>),
    StartTopicDebugStreamResult(Result<(), String>),
    StopTopicDebugStreamResult(Result<(), String>),
}
