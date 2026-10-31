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
        #[serde(serialize_with = "crate::descriptor::serialize_without_path_fallback")]
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
        #[serde(serialize_with = "crate::descriptor::serialize_without_path_fallback")]
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
    /// Remove fully-completed dataflows from the coordinator's state.
    ///
    /// A dataflow is considered fully completed when no daemon is still
    /// running it (i.e. it's no longer in `running_dataflows`). Multi-daemon
    /// dataflows that are still finishing — where some daemons have reported
    /// results but others haven't — are intentionally skipped so their final
    /// status is computed correctly when the last daemon completes.
    ///
    /// Candidates are enumerated from BOTH the in-memory
    /// `dataflow_results` map AND the persisted store
    /// (`Succeeded` / `Failed` records). This lets a restarted
    /// coordinator still reap historical rows that exist only on
    /// disk — the recovery loop intentionally does not reload them
    /// into memory, so without the persisted-store pass they would
    /// otherwise sit in redb forever and never become reachable for
    /// `dora clean`. If the persisted-store enumeration itself
    /// errors, the entire request fails with
    /// [`ControlRequestReply::Error`](crate::coordinator_to_cli::ControlRequestReply::Error) and no in-memory state is
    /// mutated — degrading silently to in-memory-only would let the
    /// CLI claim "nothing to clean" while historical rows are still
    /// sitting on disk.
    ///
    /// For each cleaned dataflow the coordinator removes its persisted
    /// record first and only then mutates in-memory state, so the reply
    /// reflects what was actually persisted. The response is
    /// [`ControlRequestReply::CleanResult`](crate::coordinator_to_cli::ControlRequestReply::CleanResult) carrying two separate
    /// lists: `cleaned` for dataflows whose redb row (and every
    /// `dora param` row owned by it — the persisted-store delete
    /// cascades) is gone, and `failed` for dataflows whose
    /// persisted-store delete errored. In-memory entries for failed
    /// candidates are preserved so a later `dora clean` can retry;
    /// they show up in `failed`, not `cleaned`, so the CLI can tell
    /// "nothing eligible" apart from "all candidates failed to
    /// clean". Logs and archived descriptors for successfully cleaned
    /// dataflows are no longer available afterward. Cached build
    /// results (`finished_builds`) are intentionally not touched —
    /// clearing them would break concurrent `dora build` calls with
    /// "unknown build id" errors.
    Clean,
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
    /// Constructed through [`ControlRequest::topic_subscribe`], not by
    /// literal: the variant is `#[non_exhaustive]` so that the *next* field
    /// added here is a minor change rather than a 2.0 (see
    /// [`RegisterResult::Ok`](crate::coordinator_to_daemon::RegisterResult::Ok)).
    #[non_exhaustive]
    TopicSubscribe {
        dataflow_id: Uuid,
        topics: Vec<(NodeId, DataId)>,
        /// Binary-frame encoding the client speaks, see
        /// [`TOPIC_DATA_PROTOCOL_VERSION`](crate::TOPIC_DATA_PROTOCOL_VERSION).
        ///
        /// `None` means the client predates the handshake and therefore speaks
        /// the bincode encoding; the coordinator rejects it rather than let it
        /// misparse postcard frames.
        #[serde(default)]
        protocol_version: Option<u16>,
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
    /// Atomically replace a running node with a new definition under the
    /// same id (dora-rs/dora#2927). The replacement must keep the node's
    /// edges (same input mappings, outputs covering every mapped output);
    /// a spawn failure leaves the current incarnation running.
    ReplaceNode {
        dataflow_id: Uuid,
        node: crate::descriptor::Node,
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
    /// [`ControlRequestReply::HelloOk`](crate::coordinator_to_cli::ControlRequestReply::HelloOk) carrying its own crate version,
    /// or [`ControlRequestReply::Error`](crate::coordinator_to_cli::ControlRequestReply::Error) with a human-readable mismatch
    /// message.
    Hello {
        dora_version: semver::Version,
    },
}

impl ControlRequest {
    /// Build a Hello request stamped with the current crate version of
    /// `dora-message` (the wire-protocol version).
    pub fn hello() -> Self {
        Self::Hello {
            dora_version: crate::current_crate_version(),
        }
    }

    /// Subscribe to `topics` of a running dataflow, stamped with the
    /// binary-frame encoding this crate speaks
    /// ([`TOPIC_DATA_PROTOCOL_VERSION`](crate::TOPIC_DATA_PROTOCOL_VERSION)).
    pub fn topic_subscribe(dataflow_id: Uuid, topics: Vec<(NodeId, DataId)>) -> Self {
        Self::TopicSubscribe {
            dataflow_id,
            topics,
            protocol_version: Some(crate::TOPIC_DATA_PROTOCOL_VERSION),
        }
    }
}

/// Check whether a CLI-reported dora version is compatible with this
/// coordinator's crate version. Returns `Ok(())` on success or a
/// human-readable error describing the mismatch.
pub fn check_cli_version(cli_version: &semver::Version) -> Result<(), String> {
    let crate_version = crate::current_crate_version();
    if crate::versions_compatible(&crate_version, cli_version)? {
        Ok(())
    } else {
        Err(format!(
            "dora version mismatch: CLI v{cli_version} is not compatible \
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
            ControlRequest::Hello { dora_version } => {
                assert_eq!(dora_version, crate::current_crate_version());
            }
            other => panic!("expected Hello, got {other:?}"),
        }
    }

    #[test]
    fn topic_subscribe_stamps_current_protocol_version() {
        assert!(matches!(
            ControlRequest::topic_subscribe(Uuid::nil(), vec![]),
            ControlRequest::TopicSubscribe {
                protocol_version: Some(crate::TOPIC_DATA_PROTOCOL_VERSION),
                ..
            }
        ));
    }

    /// The wire property the `#[non_exhaustive]` on `TopicSubscribe` is there
    /// to let us rely on: a request from a *newer* CLI carrying a field this
    /// coordinator does not know still decodes, and one from an *older* CLI
    /// missing a `#[serde(default)]` field decodes too. Together they are what
    /// make appending such a field a minor change.
    #[test]
    fn topic_subscribe_tolerates_unknown_and_missing_fields() {
        let newer = r#"{"TopicSubscribe":{"dataflow_id":"00000000-0000-0000-0000-000000000000","topics":[],"protocol_version":2,"mode":"metadata-only"}}"#;
        let older = r#"{"TopicSubscribe":{"dataflow_id":"00000000-0000-0000-0000-000000000000","topics":[]}}"#;
        for (label, json, expected_version) in [("newer", newer, Some(2)), ("older", older, None)] {
            let req: ControlRequest = serde_json::from_str(json)
                .unwrap_or_else(|e| panic!("{label} peer's request must decode: {e}"));
            match req {
                ControlRequest::TopicSubscribe {
                    protocol_version, ..
                } => assert_eq!(protocol_version, expected_version, "{label}"),
                other => panic!("expected TopicSubscribe, got {other:?}"),
            }
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
            ControlRequest::Hello { dora_version } => {
                assert_eq!(dora_version, crate::current_crate_version());
            }
            other => panic!("expected Hello, got {other:?}"),
        }
    }
}

impl ControlRequest {
    /// Encode a JSON control request, keeping fallback commands beside the
    /// strict descriptor. Use this at the WebSocket boundary to preserve the
    /// stable Rust enum and its ordinary serde representation.
    pub fn to_wire_value(&self) -> Result<serde_json::Value, serde_json::Error> {
        let mut value = serde_json::to_value(self)?;
        let (kind, dataflow) = match self {
            Self::Build { dataflow, .. } => ("Build", dataflow),
            Self::Start { dataflow, .. } => ("Start", dataflow),
            _ => return Ok(value),
        };
        let fallbacks = dataflow.path_fallbacks();
        if !fallbacks.is_empty() {
            let payload = value
                .get_mut(kind)
                .and_then(serde_json::Value::as_object_mut)
                .ok_or_else(|| {
                    <serde_json::Error as serde::ser::Error>::custom(
                        "invalid control request shape",
                    )
                })?;
            payload.insert("path_fallbacks".into(), serde_json::to_value(fallbacks)?);
        }
        Ok(value)
    }

    /// Decode a JSON control request and restore its optional fallback map.
    /// Unknown node ids, malformed maps and conflicting values are request
    /// errors; the coordinator's normal descriptor validation checks the names
    /// and node kinds before building or starting anything.
    pub fn from_wire_value(mut value: serde_json::Value) -> Result<Self, serde_json::Error> {
        let metadata = value
            .as_object_mut()
            .and_then(|map| match map.iter_mut().next() {
                Some((kind, payload)) if kind == "Build" || kind == "Start" => payload
                    .as_object_mut()
                    .and_then(|payload| payload.remove("path_fallbacks")),
                _ => None,
            });
        let fallbacks: BTreeMap<NodeId, String> = match metadata {
            Some(metadata) => serde_json::from_value(metadata)?,
            None => BTreeMap::new(),
        };
        let mut request: Self = serde_json::from_value(value)?;
        if let Self::Build { dataflow, .. } | Self::Start { dataflow, .. } = &mut request {
            for (id, fallback) in fallbacks {
                let node = dataflow
                    .nodes
                    .iter_mut()
                    .find(|node| node.id == id)
                    .ok_or_else(|| {
                        <serde_json::Error as serde::de::Error>::custom(format!(
                            "path fallback refers to unknown node `{id}`"
                        ))
                    })?;
                if node
                    .path_fallback
                    .as_ref()
                    .is_some_and(|existing| existing != &fallback)
                {
                    return Err(<serde_json::Error as serde::de::Error>::custom(format!(
                        "conflicting path fallbacks for node `{id}`"
                    )));
                }
                node.path_fallback = Some(fallback);
            }
        }
        Ok(request)
    }
}

#[cfg(test)]
mod path_fallback_tests {
    use super::*;
    use crate::descriptor::Node;

    fn requests() -> [ControlRequest; 2] {
        let mut node = Node::new("module.worker".parse().unwrap());
        node.path = Some("modules/worker".into());
        node.path_fallback = Some("worker".into());
        let dataflow = Descriptor::new(vec![node]);
        [
            ControlRequest::Build {
                session_id: SessionId::generate(),
                dataflow: dataflow.clone(),
                git_sources: BTreeMap::new(),
                prev_git_sources: BTreeMap::new(),
                local_working_dir: None,
                uv: false,
            },
            ControlRequest::Start {
                build_id: None,
                session_id: SessionId::generate(),
                dataflow,
                name: None,
                local_working_dir: None,
                uv: false,
                write_events_to: None,
            },
        ]
    }

    #[test]
    fn module_fallback_is_outside_the_control_request_descriptor() {
        for request in requests() {
            let value = request.to_wire_value().unwrap();
            let payload = value.as_object().unwrap().values().next().unwrap();
            let dataflow = match &request {
                ControlRequest::Build { dataflow, .. } | ControlRequest::Start { dataflow, .. } => {
                    dataflow
                }
                _ => unreachable!(),
            };
            let mut legacy = dataflow.clone();
            legacy.nodes[0].path_fallback = None;
            assert_eq!(payload["dataflow"], serde_json::to_value(&legacy).unwrap());
            assert_eq!(payload["path_fallbacks"]["module.worker"], "worker");
            let decoded: ControlRequest = ControlRequest::from_wire_value(value.clone()).unwrap();
            let restored = match decoded {
                ControlRequest::Build { dataflow, .. } | ControlRequest::Start { dataflow, .. } => {
                    dataflow
                }
                _ => unreachable!(),
            };
            assert_eq!(
                serde_json::to_value(&restored).unwrap(),
                serde_json::to_value(dataflow).unwrap()
            );
            let mut old_request = value;
            old_request
                .as_object_mut()
                .unwrap()
                .values_mut()
                .next()
                .unwrap()
                .as_object_mut()
                .unwrap()
                .remove("path_fallbacks");
            let old_request: ControlRequest = ControlRequest::from_wire_value(old_request).unwrap();
            let old_request = serde_json::to_value(old_request).unwrap();
            assert!(
                old_request
                    .as_object()
                    .unwrap()
                    .values()
                    .next()
                    .unwrap()
                    .get("path_fallbacks")
                    .is_none()
            );
        }
    }

    #[test]
    fn requests_without_fallback_keep_the_existing_json_shape() {
        for mut request in requests().into_iter().chain([ControlRequest::List]) {
            if let ControlRequest::Build { dataflow, .. } | ControlRequest::Start { dataflow, .. } =
                &mut request
            {
                dataflow.nodes[0].path_fallback = None;
            }
            let old = serde_json::to_value(&request).unwrap();
            assert_eq!(request.to_wire_value().unwrap(), old);
            let restored = ControlRequest::from_wire_value(old.clone()).unwrap();
            assert_eq!(serde_json::to_value(restored).unwrap(), old);
        }
    }

    #[test]
    fn malformed_path_fallback_metadata_is_a_decode_error() {
        for request in requests() {
            let original = request.to_wire_value().unwrap();
            for metadata in [
                serde_json::json!({"missing":"worker"}),
                serde_json::json!({"module.worker":42}),
                serde_json::Value::Null,
            ] {
                let mut value = original.clone();
                value.as_object_mut().unwrap().values_mut().next().unwrap()["path_fallbacks"] =
                    metadata;
                assert!(ControlRequest::from_wire_value(value).is_err());
            }
            let mut conflicting = original;
            conflicting
                .as_object_mut()
                .unwrap()
                .values_mut()
                .next()
                .unwrap()["dataflow"]["nodes"][0]["path_fallback"] = "other".into();
            assert!(ControlRequest::from_wire_value(conflicting).is_err());
        }
    }
}
