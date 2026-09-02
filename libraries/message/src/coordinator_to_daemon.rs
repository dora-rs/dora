use std::{
    collections::{BTreeMap, BTreeSet},
    path::PathBuf,
    time::Duration,
};

use crate::{
    BuildId, DataflowId, SessionId,
    common::{DaemonId, GitSource},
    descriptor::{Descriptor, ResolvedNode},
    id::{DataId, NodeId, OperatorId},
};

// ---------------------------------------------------------------------------
// State catch-up types (incremental replay for reconnecting daemons)
// ---------------------------------------------------------------------------

/// A single state mutation that a reconnecting daemon may have missed.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct StateCatchUpEntry {
    pub sequence: u64,
    pub operation: StateCatchUpOperation,
}

/// The kind of state mutation recorded in the replication log.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub enum StateCatchUpOperation {
    SetParam {
        node_id: NodeId,
        key: String,
        value: serde_json::Value,
    },
    DeleteParam {
        node_id: NodeId,
        key: String,
    },
}

pub use crate::common::Timestamped;

#[derive(Debug, serde::Serialize, serde::Deserialize)]
#[non_exhaustive]
pub enum RegisterResult {
    /// Constructed through [`RegisterResult::ok`], not by literal: the variant
    /// is `#[non_exhaustive]` so that the *next* field added here is a minor
    /// change rather than a 2.0. Adding `peer_zenoh_endpoints` to an exhaustive
    /// variant was itself a major break (`enum_struct_variant_field_added`);
    /// paying it once, with the attribute, is what keeps it from recurring.
    #[non_exhaustive]
    Ok {
        /// unique ID assigned by the coordinator
        daemon_id: DaemonId,
        /// Zenoh listen endpoints of the daemons that were already registered
        /// when this one joined, for it to dial.
        ///
        /// The coordinator is the only component every daemon already talks
        /// to, which makes it the one place a daemon can learn where its peers
        /// are without anyone configuring an address twice. Without this a
        /// multi-machine deployment has to name every daemon's endpoint on
        /// every other daemon's command line (`--zenoh-connect`), or rely on
        /// multicast — which a mesh VPN does not carry.
        ///
        /// Only endpoints a daemon *verified as bound* appear here (see the
        /// `info().locators()` check in `open_zenoh_session_with_listen`), so a
        /// dial planned from this list has a listener behind it.
        ///
        /// Deliberately only the *earlier* daemons: zenoh reads
        /// `connect/endpoints` once at session open and never re-reads it, so a
        /// daemon cannot act on an endpoint that arrives later. It does not
        /// need to — a zenoh transport is bidirectional, so the joining
        /// daemon's dial carries traffic in both directions. Each daemon
        /// dialing everyone who came before it therefore builds the full
        /// clique, with no daemon ever needing to learn about a later one.
        ///
        /// Daemons may start simultaneously: each advertises its endpoint in
        /// its own registration (see
        /// [`crate::daemon_to_coordinator::DaemonRegisterRequest::zenoh_listen_endpoint`]),
        /// and the coordinator handles registrations one at a time, so the one
        /// that registers second always sees the first. That ordering is what
        /// removes the need for a daemon to ever act on a *later* report —
        /// which it could not do anyway, zenoh having no runtime equivalent of
        /// `connect/endpoints`.
        ///
        /// `#[serde(default)]` keeps a daemon built before this field existed
        /// decodable: it sees no peers and falls back to the multicast/explicit
        /// wiring it already had.
        #[serde(default)]
        peer_zenoh_endpoints: Vec<String>,
    },
    Err(String),
}

impl RegisterResult {
    /// A successful registration: the assigned id and the peers to dial.
    pub fn ok(daemon_id: DaemonId, peer_zenoh_endpoints: Vec<String>) -> Self {
        Self::Ok {
            daemon_id,
            peer_zenoh_endpoints,
        }
    }

    /// The assigned id alone, for callers that do not wire zenoh.
    pub fn to_result(self) -> eyre::Result<DaemonId> {
        self.into_parts().map(|(daemon_id, _)| daemon_id)
    }

    /// The assigned id plus the peer endpoints to dial; see
    /// [`RegisterResult::Ok::peer_zenoh_endpoints`].
    pub fn into_parts(self) -> eyre::Result<(DaemonId, Vec<String>)> {
        match self {
            RegisterResult::Ok {
                daemon_id,
                peer_zenoh_endpoints,
            } => Ok((daemon_id, peer_zenoh_endpoints)),
            RegisterResult::Err(err) => Err(eyre::eyre!(err)),
        }
    }
}

/// Reply to `CoordinatorRequest::ResolveMachine` — sent by the coordinator
/// to the requesting daemon over the same request/response channel used for
/// `RegisterResult`.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub enum ResolveMachineReply {
    /// Reply to `CoordinatorRequest::ResolveMachine`.
    ResolveMachineResult {
        found: bool,
        /// The target daemon's WS peer address as seen by the coordinator
        /// (set at registration). Used by the memory-pool direct-TCP data
        /// plane to reach the mirror daemon's data listener.
        address: Option<std::net::SocketAddr>,
    },
}

#[allow(clippy::large_enum_variant)]
#[derive(Debug, serde::Deserialize, serde::Serialize)]
pub enum DaemonCoordinatorEvent {
    Build(BuildDataflowNodes),
    Spawn(SpawnDataflowNodes),
    AllNodesReady {
        dataflow_id: DataflowId,
        exited_before_subscribe: Vec<NodeId>,
    },
    StopDataflow {
        dataflow_id: DataflowId,
        grace_duration: Option<Duration>,
        #[serde(default)]
        force: bool,
    },
    ReloadDataflow {
        dataflow_id: DataflowId,
        node_id: NodeId,
        operator_id: Option<OperatorId>,
    },
    Logs {
        dataflow_id: DataflowId,
        node_id: NodeId,
        tail: Option<usize>,
    },
    RestartNode {
        dataflow_id: DataflowId,
        node_id: NodeId,
        grace_duration: Option<Duration>,
    },
    StopNode {
        dataflow_id: DataflowId,
        node_id: NodeId,
        grace_duration: Option<Duration>,
    },
    SetParam {
        dataflow_id: DataflowId,
        node_id: NodeId,
        key: String,
        value: serde_json::Value,
    },
    DeleteParam {
        dataflow_id: DataflowId,
        node_id: NodeId,
        key: String,
    },
    Destroy,
    Heartbeat,
    PeerDaemonDisconnected {
        daemon_id: DaemonId,
    },
    // --- Dynamic Topology ---
    /// Add a node to a running dataflow on this daemon.
    AddNode {
        dataflow_id: DataflowId,
        node: crate::descriptor::ResolvedNode,
        uv: bool,
    },
    /// Remove a node from a running dataflow on this daemon.
    RemoveNode {
        dataflow_id: DataflowId,
        node_id: NodeId,
        grace_duration: Option<Duration>,
    },
    /// Atomically replace a running node on this daemon with a new
    /// definition under the same id (dora-rs/dora#2927): spawn the
    /// replacement first (a failure leaves the current incarnation
    /// untouched), then swap the entry and stop the outgoing incarnation.
    ///
    /// `unresolved_node` is the original YAML-shape [`crate::descriptor::Node`]
    /// (as `dora node replace` received it); the daemon assigns it wholesale
    /// onto the stored descriptor entry so the child's `DORA_NODE_CONFIG` /
    /// `DoraNode::dataflow_descriptor()` reflect the replacement's definition
    /// end-to-end. The field is separate from `node` because the two shapes
    /// differ (`Node` is the flat YAML surface; `ResolvedNode` is the nested
    /// resolved form used for spawning), and a hand-written back-conversion
    /// would silently drift as fields are added to either type
    /// (dora-rs/dora#2988 review, finding 2).
    ReplaceNode {
        dataflow_id: DataflowId,
        node: crate::descriptor::ResolvedNode,
        unresolved_node: crate::descriptor::Node,
        uv: bool,
        grace_duration: Option<Duration>,
    },
    /// Add a mapping (connection) in a running dataflow.
    AddMapping {
        dataflow_id: DataflowId,
        source_node: NodeId,
        source_output: DataId,
        target_node: NodeId,
        target_input: DataId,
    },
    /// Remove a mapping (connection) in a running dataflow.
    RemoveMapping {
        dataflow_id: DataflowId,
        source_node: NodeId,
        source_output: DataId,
        target_node: NodeId,
        target_input: DataId,
    },
    /// Start forwarding matching output frames back to the coordinator over
    /// the daemon control channel for CLI topic inspection.
    StartTopicDebugStream {
        dataflow_id: DataflowId,
        outputs: Vec<(NodeId, DataId)>,
        subscription_id: uuid::Uuid,
    },
    /// Stop forwarding output frames for a previously registered CLI topic
    /// inspection subscription.
    StopTopicDebugStream {
        dataflow_id: DataflowId,
        subscription_id: uuid::Uuid,
    },
    /// Incremental state catch-up: replays missed state mutations to a
    /// reconnecting daemon.
    StateCatchUp {
        dataflow_id: DataflowId,
        /// The entries the daemon missed, ordered by sequence number.
        entries: Vec<StateCatchUpEntry>,
    },
}

#[derive(Debug, serde::Deserialize, serde::Serialize)]
pub struct BuildDataflowNodes {
    pub build_id: BuildId,
    pub session_id: SessionId,
    /// Allows overwriting the base working dir when CLI and daemon are
    /// running on the same machine.
    ///
    /// Must not be used for multi-machine dataflows.
    ///
    /// Note that nodes with git sources still use a subdirectory of
    /// the base working dir.
    pub local_working_dir: Option<PathBuf>,
    pub git_sources: BTreeMap<NodeId, GitSource>,
    pub prev_git_sources: BTreeMap<NodeId, GitSource>,
    pub dataflow_descriptor: Descriptor,
    pub nodes_on_machine: BTreeSet<NodeId>,
    pub uv: bool,
}

#[derive(Debug, serde::Deserialize, serde::Serialize)]
pub struct SpawnDataflowNodes {
    pub build_id: Option<BuildId>,
    pub session_id: SessionId,
    pub dataflow_id: DataflowId,
    /// Allows overwriting the base working dir when CLI and daemon are
    /// running on the same machine.
    ///
    /// Must not be used for multi-machine dataflows.
    ///
    /// Note that nodes with git sources still use a subdirectory of
    /// the base working dir.
    pub local_working_dir: Option<PathBuf>,
    pub nodes: BTreeMap<NodeId, ResolvedNode>,
    pub dataflow_descriptor: Descriptor,
    pub spawn_nodes: BTreeSet<NodeId>,
    pub uv: bool,
    pub write_events_to: Option<PathBuf>,
    /// Base URL for downloading artifacts from the coordinator (HTTP distribution mode).
    /// When set, daemons can pull binaries from `{artifact_base_url}/{build_id}/{node_id}`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub artifact_base_url: Option<String>,
}

#[cfg(test)]
mod register_result_tests {
    use super::*;

    /// A daemon built before `peer_zenoh_endpoints` existed sends a reply
    /// without the field. It must still decode — into "no peers to dial" —
    /// rather than failing registration outright, which would take the whole
    /// daemon down over a field it does not need.
    #[test]
    fn a_reply_without_peer_endpoints_decodes_as_no_peers() {
        let legacy = r#"{"Ok":{"daemon_id":{"machine_id":"A","uuid":"00000000-0000-0000-0000-000000000001"}}}"#;
        let decoded: RegisterResult =
            serde_json::from_str(legacy).expect("legacy register reply must stay decodable");
        let (_, peers) = decoded.into_parts().expect("legacy reply is Ok");
        assert!(peers.is_empty());
    }

    #[test]
    fn peer_endpoints_round_trip() {
        let peers = vec!["tcp/10.0.2.100:5456".to_string()];
        let encoded = serde_json::to_string(&RegisterResult::Ok {
            daemon_id: DaemonId::new(Some("A".to_string())),
            peer_zenoh_endpoints: peers.clone(),
        })
        .expect("serialize");
        let decoded: RegisterResult = serde_json::from_str(&encoded).expect("deserialize");
        assert_eq!(decoded.into_parts().expect("ok").1, peers);
    }

    /// `to_result` is the id-only convenience over `into_parts`; an `Err` reply
    /// must stay an error through both.
    #[test]
    fn an_error_reply_is_an_error_through_both_accessors() {
        assert!(RegisterResult::Err("nope".into()).to_result().is_err());
        assert!(RegisterResult::Err("nope".into()).into_parts().is_err());
    }
}
