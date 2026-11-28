pub use crate::common::{DataMessage, LogLevel, LogMessage, SharedMemoryId, Timestamped};
use crate::{
    DataflowId, current_crate_version,
    id::{DataId, NodeId},
    metadata::Metadata,
    versions_compatible,
};

/// A request from a node to its daemon.
///
/// # Adding a variant
///
/// Every new variant must expect a binary [`DaemonReply`](crate::daemon_to_node::DaemonReply),
/// i.e. [`expects_tcp_binary_reply`](Self::expects_tcp_binary_reply) must
/// return `true` for it. A daemon cannot decode a variant it predates, so it
/// cannot tell whether the sender waits for an answer. With this rule, an
/// undecodable request always waits for one, and the daemon can answer it
/// with an error while keeping the connection open. Only `SendMessage`,
/// `OutputSent` and `NodeConfig` get no binary reply; they are part of the
/// frozen 1.x wire format and must always decode. A change to the metadata
/// wire format they carry must bump [`Metadata::CURRENT_VERSION`].
/// [`frame_expects_reply`](Self::frame_expects_reply) is the daemon's fallback
/// if one still fails to decode. See "Stability scope at 1.0" in
/// `docs/api-rust.md`.
#[allow(clippy::large_enum_variant)]
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
#[non_exhaustive]
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
    /// Store an opaque value in the daemon's dataflow-scoped extension table.
    ///
    /// dora attaches no meaning to `namespace`, `key` or `value`: this is the
    /// out-of-band channel for transports that live outside the tree (see
    /// `docs/extensions.md`). The daemon tracks who stored a key and who has
    /// read it, so it can notify readers on removal and reclaim the entry when
    /// the dataflow ends.
    ExtensionStore {
        namespace: String,
        key: String,
        value: Vec<u8>,
    },
    /// Read an opaque value back. `remove: true` drops it in the same round
    /// trip, so a consume-once handoff needs one request rather than two.
    ExtensionLoad {
        namespace: String,
        key: String,
        remove: bool,
    },
    /// Drop an opaque value. Every node that stored or loaded the key is sent
    /// [`NodeEvent::ExtensionDropped`](crate::daemon_to_node::NodeEvent::ExtensionDropped) so it can release whatever the value
    /// referred to.
    ExtensionDrop {
        namespace: String,
        key: String,
    },
    /// An opaque request from a node to the extension registered under
    /// `namespace` on its daemon. dora never interprets either field.
    ///
    /// Complements the extension table (`ExtensionStore`/`ExtensionLoad`/
    /// `ExtensionDrop`), which brokers descriptor *lifetime*. This one
    /// carries a call the extension's daemon half must service — the
    /// tensor-pool uses it for cross-machine registration and pool writes.
    ExtensionRequest {
        namespace: String,
        #[serde(with = "crate::bulk_bytes::vec")]
        payload: Vec<u8>,
    },
}

impl DaemonRequest {
    /// Bulk bytes this request will contribute to its encoding, for
    /// [`crate::encode_presized`].
    ///
    /// Matched exhaustively on purpose: a new payload-carrying variant that
    /// forgets to report its size would silently fall back to growing the
    /// buffer from empty, which is the cost `encode_presized` exists to avoid
    /// and which no test would catch.
    pub fn encode_size_hint(&self) -> usize {
        match self {
            DaemonRequest::SendMessage { data, .. } => data.as_ref().map_or(0, DataMessage::len),
            DaemonRequest::Register(_)
            | DaemonRequest::Subscribe
            | DaemonRequest::OutputSent { .. }
            | DaemonRequest::CloseOutputs(_)
            | DaemonRequest::OutputsDone
            | DaemonRequest::NextEvent
            | DaemonRequest::EventStreamDropped
            | DaemonRequest::NodeConfig { .. } => 0,
            // The stored value dominates; the namespace and key are short.
            DaemonRequest::ExtensionStore { value, .. } => value.len(),
            DaemonRequest::ExtensionLoad { .. } | DaemonRequest::ExtensionDrop { .. } => 0,
            DaemonRequest::ExtensionRequest { payload, .. } => payload.len(),
        }
    }

    pub fn expects_tcp_binary_reply(&self) -> bool {
        #[allow(clippy::match_like_matches_macro)]
        match self {
            // Frozen set: don't add variants here. A new request must expect a
            // binary reply (see the `DaemonRequest` docs).
            DaemonRequest::SendMessage { .. }
            | DaemonRequest::OutputSent { .. }
            | DaemonRequest::NodeConfig { .. } => false,
            DaemonRequest::Register(NodeRegisterRequest { .. })
            | DaemonRequest::Subscribe
            | DaemonRequest::CloseOutputs(_)
            | DaemonRequest::OutputsDone
            | DaemonRequest::NextEvent
            | DaemonRequest::EventStreamDropped
            | DaemonRequest::ExtensionRequest { .. }
            | DaemonRequest::ExtensionStore { .. }
            | DaemonRequest::ExtensionLoad { .. }
            | DaemonRequest::ExtensionDrop { .. } => true,
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
            | DaemonRequest::ExtensionRequest { .. }
            | DaemonRequest::ExtensionStore { .. }
            | DaemonRequest::ExtensionLoad { .. }
            | DaemonRequest::ExtensionDrop { .. } => false,
        }
    }

    /// Whether the sender of an encoded `Timestamped<DaemonRequest>` frame
    /// waits for a reply, judged from the frame's leading variant tag alone.
    ///
    /// For a daemon that failed to [`decode`](crate::decode) the frame: it must
    /// answer a request whose sender blocks on a reply, and must not answer one
    /// whose sender reads nothing, or the unsolicited frame would shift every
    /// later request/reply pair on the connection by one.
    ///
    /// This is defence in depth. The fire-and-forget requests are
    /// expected to always decode: a payload change that breaks that, such as a
    /// new [`Parameter`](crate::metadata::Parameter) kind, must bump
    /// [`Metadata::CURRENT_VERSION`], which rejects the node at register time.
    ///
    /// Returns `false` only for `SendMessage` and `OutputSent` (neither a
    /// binary nor a JSON reply). Every other tag, including one this crate does
    /// not know and a frame too short to hold a tag, counts as expecting a
    /// reply, per the rule on [`DaemonRequest`].
    pub fn frame_expects_reply(frame: &[u8]) -> bool {
        // `Timestamped` encodes `inner` first, and postcard writes an enum
        // variant index as a varint `u32`. The indices are pinned by
        // `reply_policy_tests::frame_tag_matches_the_reply_policy`.
        const SEND_MESSAGE: u32 = 2;
        const OUTPUT_SENT: u32 = 3;
        !matches!(
            postcard::take_from_bytes::<u32>(frame),
            Ok((SEND_MESSAGE | OUTPUT_SENT, _))
        )
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
    /// cryptic bad-enum-discriminant deserialization error (#2742).
    ///
    /// This covers *layout* drift within one encoding. It cannot cover a change
    /// of the encoding itself: the register frame is encoded the same way as
    /// everything else, so a peer from before the bincode→postcard move fails
    /// while decoding the frame that carries this field, never reaching the
    /// check. Such changes are gated by the release notes instead.
    metadata_version: u16,
}

impl NodeRegisterRequest {
    pub fn new(dataflow_id: DataflowId, node_id: NodeId) -> Self {
        Self {
            dataflow_id,
            node_id,
            dora_version: current_crate_version(),
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
        // than desyncing mid-stream into a cryptic deserialization error.
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

#[cfg(test)]
mod reply_policy_tests {
    use super::*;

    /// The frozen set of requests that get no binary reply.
    ///
    /// Matched exhaustively, so a new `DaemonRequest` variant fails to compile
    /// here until it is classified. It belongs in the `false` arm: a new
    /// request must expect a binary reply (see the `DaemonRequest` docs).
    fn is_frozen_no_binary_reply(request: &DaemonRequest) -> bool {
        match request {
            DaemonRequest::SendMessage { .. }
            | DaemonRequest::OutputSent { .. }
            | DaemonRequest::NodeConfig { .. } => true,
            DaemonRequest::Register(_)
            | DaemonRequest::Subscribe
            | DaemonRequest::CloseOutputs(_)
            | DaemonRequest::OutputsDone
            | DaemonRequest::NextEvent
            | DaemonRequest::EventStreamDropped
            | DaemonRequest::ExtensionStore { .. }
            | DaemonRequest::ExtensionLoad { .. }
            | DaemonRequest::ExtensionDrop { .. }
            | DaemonRequest::ExtensionRequest { .. } => false,
        }
    }

    fn one_of_each() -> Vec<DaemonRequest> {
        let metadata = || Metadata::new(uhlc::HLC::default().new_timestamp());
        let text = || "x".to_string();
        vec![
            DaemonRequest::Register(NodeRegisterRequest::new(
                uuid::Uuid::nil(),
                NodeId::from(text()),
            )),
            DaemonRequest::Subscribe,
            DaemonRequest::SendMessage {
                output_id: DataId::from(text()),
                metadata: metadata(),
                data: None,
            },
            DaemonRequest::OutputSent {
                output_id: DataId::from(text()),
                metadata: metadata(),
            },
            DaemonRequest::CloseOutputs(Vec::new()),
            DaemonRequest::OutputsDone,
            DaemonRequest::NextEvent,
            DaemonRequest::EventStreamDropped,
            DaemonRequest::NodeConfig {
                node_id: NodeId::from(text()),
            },
            DaemonRequest::ExtensionStore {
                namespace: text(),
                key: text(),
                value: Vec::new(),
            },
            DaemonRequest::ExtensionLoad {
                namespace: text(),
                key: text(),
                remove: false,
            },
            DaemonRequest::ExtensionDrop {
                namespace: text(),
                key: text(),
            },
            DaemonRequest::ExtensionRequest {
                namespace: text(),
                payload: Vec::new(),
            },
        ]
    }

    #[test]
    fn only_the_frozen_requests_skip_the_binary_reply() {
        for request in one_of_each() {
            assert_eq!(
                request.expects_tcp_binary_reply(),
                !is_frozen_no_binary_reply(&request),
                "a request other than SendMessage, OutputSent and NodeConfig must \
                 expect a binary reply: {request:?}"
            );
        }
    }
    #[test]
    fn frame_tag_matches_the_reply_policy() {
        for request in one_of_each() {
            let expects_reply =
                request.expects_tcp_binary_reply() || request.expects_tcp_json_reply();
            let frame = crate::encode(&crate::common::Timestamped {
                inner: request.clone(),
                timestamp: uhlc::HLC::default().new_timestamp(),
            })
            .unwrap();
            assert_eq!(
                DaemonRequest::frame_expects_reply(&frame),
                expects_reply,
                "the frame tag must classify {request:?} like the reply policy"
            );
            // A payload this crate cannot decode keeps the tag's verdict.
            let mut truncated = frame.clone();
            truncated.truncate(1);
            assert_eq!(
                DaemonRequest::frame_expects_reply(&truncated),
                expects_reply
            );
        }
    }

    #[test]
    fn unknown_or_missing_tag_expects_a_reply() {
        // A variant this crate predates (and a multi-byte varint index).
        assert!(DaemonRequest::frame_expects_reply(&[200, 1]));
        assert!(DaemonRequest::frame_expects_reply(&[0x7F]));
        assert!(DaemonRequest::frame_expects_reply(&[]));
    }
}
