use std::{collections::BTreeMap, path::PathBuf, time::Duration};

use uuid::Uuid;

use crate::{
    BuildId, SessionId,
    common::GitSource,
    descriptor::Descriptor,
    id::{DataId, NodeId, OperatorId},
};

#[allow(clippy::large_enum_variant)]
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
    /// Protocol version handshake. Sent by the CLI as its first request
    /// after connecting so the coordinator can reject version-mismatched
    /// clients before they exchange incompatible messages
    /// (dora-rs/adora#151).
    ///
    /// The coordinator replies with either
    /// [`ControlRequestReply::HelloOk`] carrying its own crate version,
    /// or [`ControlRequestReply::Error`] with a human-readable mismatch
    /// message.
    Hello {
        adora_version: semver::Version,
    },
}

impl ControlRequest {
    /// Build a Hello request stamped with the current crate version of
    /// `adora-message` (the wire-protocol version).
    pub fn hello() -> Self {
        Self::Hello {
            adora_version: crate::current_crate_version(),
        }
    }
}

/// Check whether a CLI-reported adora version is compatible with this
/// coordinator's crate version. Returns `Ok(())` on success or a
/// human-readable error describing the mismatch.
pub fn check_cli_version(cli_version: &semver::Version) -> Result<(), String> {
    let crate_version = crate::current_crate_version();
    if crate::versions_compatible(&crate_version, cli_version)? {
        Ok(())
    } else {
        Err(format!(
            "adora version mismatch: CLI v{cli_version} is not compatible \
             with coordinator v{crate_version}. Upgrade the component that \
             is behind (usually the CLI) so both sides share a semver-compatible \
             version."
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ---- dora-rs/adora#151: CLI ↔ coordinator protocol version handshake ----

    #[test]
    fn hello_stamps_current_crate_version() {
        let req = ControlRequest::hello();
        match req {
            ControlRequest::Hello { adora_version } => {
                assert_eq!(adora_version, crate::current_crate_version());
            }
            other => panic!("expected Hello, got {other:?}"),
        }
    }

    #[test]
    fn check_cli_version_accepts_matching_version() {
        let same = crate::current_crate_version();
        assert!(check_cli_version(&same).is_ok());
    }

    #[test]
    fn check_cli_version_rejects_incompatible_major_bump() {
        // Semver allows same-major patch bumps but not major jumps.
        let current = crate::current_crate_version();
        let incompatible = semver::Version::new(current.major + 1, 0, 0);
        let err = check_cli_version(&incompatible).expect_err("major bump should reject");
        assert!(
            err.contains("version mismatch"),
            "error must mention mismatch: {err}"
        );
        assert!(err.contains("CLI v"), "error must include CLI version");
    }

    #[test]
    fn check_cli_version_accepts_compatible_patch_bump() {
        // A patch bump on the same major is always semver-compatible.
        let current = crate::current_crate_version();
        let patched = semver::Version::new(current.major, current.minor, current.patch + 1);
        assert!(check_cli_version(&patched).is_ok());
    }

    #[test]
    fn hello_roundtrips_through_json() {
        // The handshake is sent as JSON over the WS control channel,
        // so the enum variant must survive a roundtrip with preserved
        // version fidelity.
        let req = ControlRequest::hello();
        let json = serde_json::to_string(&req).expect("serialize");
        let decoded: ControlRequest = serde_json::from_str(&json).expect("deserialize");
        match decoded {
            ControlRequest::Hello { adora_version } => {
                assert_eq!(adora_version, crate::current_crate_version());
            }
            other => panic!("expected Hello, got {other:?}"),
        }
    }
}
