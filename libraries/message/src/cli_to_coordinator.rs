use std::{collections::BTreeMap, path::PathBuf, time::Duration};

use uuid::Uuid;

use crate::{
    BuildId, SessionId,
    common::GitSource,
    descriptor::Descriptor,
    id::{DataId, NodeId, OperatorId},
};

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub enum ControlRequest {
    Build {
        session_id: SessionId,
        dataflow: Descriptor,
        git_sources: BTreeMap<NodeId, GitSource>,
        prev_git_sources: BTreeMap<NodeId, GitSource>,
        /// Allows overwriting the base working dir when CLI and daemon are
        /// running on the same machine.
        ///
        /// Must not be used for multi-machine dataflows.
        ///
        /// Note that nodes with git sources still use a subdirectory of
        /// the base working dir.
        local_working_dir: Option<PathBuf>,
        uv: bool,
    },
    WaitForBuild {
        build_id: BuildId,
    },
    Start {
        build_id: Option<BuildId>,
        session_id: SessionId,
        dataflow: Descriptor,
        name: Option<String>,
        /// Allows overwriting the base working dir when CLI and daemon are
        /// running on the same machine.
        ///
        /// Must not be used for multi-machine dataflows.
        ///
        /// Note that nodes with git sources still use a subdirectory of
        /// the base working dir.
        local_working_dir: Option<PathBuf>,
        uv: bool,
        write_events_to: Option<PathBuf>,
    },
    WaitForSpawn {
        dataflow_id: Uuid,
    },
    Reload {
        dataflow_id: Uuid,
        node_id: NodeId,
        operator_id: Option<OperatorId>,
    },
    Check {
        dataflow_uuid: Uuid,
    },
    Stop {
        dataflow_uuid: Uuid,
        grace_duration: Option<Duration>,
        #[serde(default)]
        force: bool,
    },
    StopByName {
        name: String,
        grace_duration: Option<Duration>,
        #[serde(default)]
        force: bool,
    },
    Restart {
        dataflow_uuid: Uuid,
        grace_duration: Option<Duration>,
        #[serde(default)]
        force: bool,
    },
    RestartByName {
        name: String,
        grace_duration: Option<Duration>,
        #[serde(default)]
        force: bool,
    },
    Logs {
        uuid: Option<Uuid>,
        name: Option<String>,
        node: String,
        tail: Option<usize>,
    },
    Destroy,
    List,
    Info {
        dataflow_uuid: Uuid,
    },
    DaemonConnected,
    ConnectedMachines,
    LogSubscribe {
        dataflow_id: Uuid,
        level: log::LevelFilter,
    },
    BuildLogSubscribe {
        build_id: BuildId,
        level: log::LevelFilter,
    },
    CliAndDefaultDaemonOnSameMachine,
    GetNodeInfo,
    TopicSubscribe {
        dataflow_id: Uuid,
        topics: Vec<(NodeId, DataId)>,
    },
    TopicUnsubscribe {
        subscription_id: Uuid,
    },
    GetTraces,
    GetTraceSpans {
        trace_id: String,
    },
    /// Restart a specific node without stopping the entire dataflow.
    RestartNode {
        dataflow_id: Uuid,
        node_id: NodeId,
        grace_duration: Option<Duration>,
    },
    /// Stop a specific node without stopping the entire dataflow.
    StopNode {
        dataflow_id: Uuid,
        node_id: NodeId,
        grace_duration: Option<Duration>,
    },
    /// Publish a message to a topic (for debugging/testing).
    ///
    /// The coordinator serializes the JSON data into Arrow format and
    /// publishes it to Zenoh on the appropriate topic key.
    TopicPublish {
        dataflow_id: Uuid,
        node_id: NodeId,
        output_id: DataId,
        /// JSON data to publish (will be converted to Arrow UInt8 array)
        data_json: String,
    },
    /// List runtime parameters for a node.
    GetParams {
        dataflow_id: Uuid,
        node_id: NodeId,
    },
    /// Get a single runtime parameter value.
    GetParam {
        dataflow_id: Uuid,
        node_id: NodeId,
        key: String,
    },
    /// Set a runtime parameter on a node.
    SetParam {
        dataflow_id: Uuid,
        node_id: NodeId,
        key: String,
        value: serde_json::Value,
    },
    /// Delete a runtime parameter from a node.
    DeleteParam {
        dataflow_id: Uuid,
        node_id: NodeId,
        key: String,
    },
    // --- Dynamic Topology ---
    /// Add a node to a running dataflow.
    AddNode {
        dataflow_id: Uuid,
        node: crate::descriptor::Node,
    },
    /// Remove a node from a running dataflow.
    RemoveNode {
        dataflow_id: Uuid,
        node_id: NodeId,
        grace_duration: Option<std::time::Duration>,
    },
    /// Add a mapping (connection) between two nodes in a running dataflow.
    AddMapping {
        dataflow_id: Uuid,
        source_node: NodeId,
        source_output: DataId,
        target_node: NodeId,
        target_input: DataId,
    },
    /// Remove a mapping (connection) between two nodes in a running dataflow.
    RemoveMapping {
        dataflow_id: Uuid,
        source_node: NodeId,
        source_output: DataId,
        target_node: NodeId,
        target_input: DataId,
    },
}
