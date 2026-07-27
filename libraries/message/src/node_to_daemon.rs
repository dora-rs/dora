pub use crate::common::{DataMessage, LogLevel, LogMessage, SharedMemoryId, Timestamped};
use crate::{
    DataflowId, current_crate_version,
    id::{DataId, NodeId},
    metadata::Metadata,
    versions_compatible,
};

#[allow(clippy::large_enum_variant)]
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub enum DaemonRequest {
    Register(NodeRegisterRequest),
    Subscribe,
    SendMessage {
        output_id: DataId,
        metadata: Metadata,
        data: Option<DataMessage>,
    },
    OutputSent {
        output_id: DataId,
        metadata: Metadata,
    },
    CloseOutputs(Vec<DataId>),
    /// Signals that the node is finished sending outputs.
    OutputsDone,
    NextEvent,
    EventStreamDropped,
    NodeConfig {
        node_id: NodeId,
    },
    RegisterPinnedMemory {
        shared_memory_id: String,
        metadata: Metadata,
    },
    ReadPinnedMemory {
        shared_memory_id: String,
        free: bool,
    },
    FreePinnedMemory {
        shared_memory_id: String,
    },
}

impl DaemonRequest {
    pub fn expects_tcp_bincode_reply(&self) -> bool {
        #[allow(clippy::match_like_matches_macro)]
        match self {
            DaemonRequest::SendMessage { .. }
            | DaemonRequest::OutputSent { .. }
            | DaemonRequest::NodeConfig { .. } => false,
            DaemonRequest::Register(NodeRegisterRequest { .. })
            | DaemonRequest::Subscribe
            | DaemonRequest::CloseOutputs(_)
            | DaemonRequest::OutputsDone
            | DaemonRequest::NextEvent
            | DaemonRequest::EventStreamDropped
            | DaemonRequest::RegisterPinnedMemory { .. }
            | DaemonRequest::ReadPinnedMemory { .. }
            | DaemonRequest::FreePinnedMemory { .. } => true,
        }
    }

    pub fn expects_tcp_json_reply(&self) -> bool {
        #[allow(clippy::match_like_matches_macro)]
        match self {
            DaemonRequest::NodeConfig { .. } => true,
            DaemonRequest::Register(NodeRegisterRequest { .. })
            | DaemonRequest::Subscribe
            | DaemonRequest::CloseOutputs(_)
            | DaemonRequest::OutputsDone
            | DaemonRequest::NextEvent
            | DaemonRequest::SendMessage { .. }
            | DaemonRequest::OutputSent { .. }
            | DaemonRequest::EventStreamDropped
            | DaemonRequest::RegisterPinnedMemory { .. }
            | DaemonRequest::ReadPinnedMemory { .. }
            | DaemonRequest::FreePinnedMemory { .. } => false,
        }
    }
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct NodeRegisterRequest {
    pub dataflow_id: DataflowId,
    pub node_id: NodeId,
    dora_version: semver::Version,
    /// The metadata wire-format version this node was built against
    /// ([`Metadata::CURRENT_VERSION`]).
    ///
    /// The `dora_version` semver check above is too coarse to catch a
    /// message-format break within a release series: two builds that both
    /// report `1.0.0-rc` are "compatible" by semver even when the payload
    /// layout differs (as it did across #2366). Carrying the format version
    /// explicitly lets the daemon reject an incompatible peer at register with
    /// a clear message, instead of the node desyncing mid-stream into a
    /// cryptic `tag for enum is not valid` bincode error (#2742).
    metadata_version: u16,
}

impl NodeRegisterRequest {
    pub fn new(dataflow_id: DataflowId, node_id: NodeId) -> Self {
        Self {
            dataflow_id,
            node_id,
            dora_version: semver::Version::parse(env!("CARGO_PKG_VERSION")).unwrap(),
            metadata_version: Metadata::CURRENT_VERSION,
        }
    }

    pub fn check_version(&self) -> Result<(), String> {
        let crate_version = current_crate_version();
        let specified_version = &self.dora_version;

        if !versions_compatible(&crate_version, specified_version)? {
            return Err(format!(
                "version mismatch: message format v{} is not compatible \
                with expected message format v{crate_version}",
                self.dora_version
            ));
        }

        // Even when the semver check passes, the payload wire format can still
        // differ within a release series (#2366 dropped a `Metadata` field
        // without changing the version). Reject that here so the failure is a
        // legible register-time error rather than a mid-stream desync (#2742).
        if self.metadata_version != Metadata::CURRENT_VERSION {
            return Err(format!(
                "message wire-format mismatch: node speaks metadata format v{} \
                but this daemon speaks v{}. The node and daemon were built from \
                dora revisions with incompatible message formats; rebuild both \
                from the same revision.",
                self.metadata_version,
                Metadata::CURRENT_VERSION
            ));
        }

        Ok(())
    }
}

#[derive(Debug, serde::Deserialize, serde::Serialize)]
pub enum DynamicNodeEvent {
    NodeConfig { node_id: NodeId },
}

#[cfg(test)]
mod register_version_tests {
    use super::*;

    fn request() -> NodeRegisterRequest {
        NodeRegisterRequest::new(uuid::Uuid::nil(), NodeId::from("test-node".to_string()))
    }

    #[test]
    fn new_stamps_current_metadata_version() {
        assert_eq!(request().metadata_version, Metadata::CURRENT_VERSION);
    }

    #[test]
    fn check_version_accepts_a_matching_request() {
        // A request built by this same crate version passes both the semver
        // and the wire-format gate.
        request().check_version().unwrap();
    }

    #[test]
    fn check_version_rejects_a_wire_format_mismatch() {
        // A peer built from a dora revision with a different metadata wire
        // format — same semver, incompatible bytes. This is the #2366 / #2742
        // shape: it must be rejected at register with a legible message rather
        // than desyncing mid-stream into a cryptic bincode error.
        let mut req = request();
        req.metadata_version = Metadata::CURRENT_VERSION.wrapping_add(1);
        let err = req
            .check_version()
            .expect_err("mismatched metadata wire version must be rejected");
        assert!(
            err.contains("wire-format") && err.contains(&Metadata::CURRENT_VERSION.to_string()),
            "error should name the wire-format mismatch and the expected version, got: {err}"
        );
    }
}
