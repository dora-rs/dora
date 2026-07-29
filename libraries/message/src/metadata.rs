use std::collections::BTreeMap;

use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};

/// Additional data that is sent as part of output messages.
///
/// Includes a timestamp and additional user-provided parameters. The payload is
/// a self-describing Arrow IPC stream, so the message carries no separate type
/// descriptor.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Metadata {
    metadata_version: u16,
    timestamp: uhlc::Timestamp,
    pub parameters: MetadataParameters,
}

impl Metadata {
    /// Current metadata wire-format version, stamped on every outgoing message.
    ///
    /// Bumped from 0 to 1 when the `ArrowTypeInfo` sidecar was dropped and the
    /// wire format became Arrow-IPC-only. A receiver can compare
    /// [`metadata_version`](Self::metadata_version) against this to detect a peer
    /// speaking an incompatible format and report it clearly instead of failing
    /// with a cryptic positional-deserialization error.
    pub const CURRENT_VERSION: u16 = 1;

    pub fn new(timestamp: uhlc::Timestamp) -> Self {
        Self::from_parameters(timestamp, Default::default())
    }

    /// Metadata for a startup route-probe marker (see [`STARTUP_MARKER_PARAM`]).
    pub fn startup_marker(timestamp: uhlc::Timestamp) -> Self {
        Self::from_parameters(
            timestamp,
            BTreeMap::from([(STARTUP_MARKER_PARAM.to_owned(), Parameter::Bool(true))]),
        )
    }

    /// Whether this message is a startup route-probe marker rather than node
    /// data. Markers are consumed by the receiving node's startup barrier and
    /// must never be decoded or surfaced to user code — see
    /// [`STARTUP_MARKER_PARAM`].
    pub fn is_startup_marker(&self) -> bool {
        get_bool_param(&self.parameters, STARTUP_MARKER_PARAM).unwrap_or(false)
    }

    /// Metadata for a startup route-probe **ack**: the consumer-side reply to a
    /// startup marker, identifying which consumer input received it (see
    /// [`STARTUP_ACK_PARAM`]).
    pub fn startup_ack(timestamp: uhlc::Timestamp, consumer_node: &str, input_id: &str) -> Self {
        Self::from_parameters(
            timestamp,
            BTreeMap::from([
                (STARTUP_ACK_PARAM.to_owned(), Parameter::Bool(true)),
                (
                    STARTUP_ACK_CONSUMER_PARAM.to_owned(),
                    Parameter::String(consumer_node.to_owned()),
                ),
                (
                    STARTUP_ACK_INPUT_PARAM.to_owned(),
                    Parameter::String(input_id.to_owned()),
                ),
            ]),
        )
    }

    /// `Some((consumer_node, input_id))` iff this message is a well-formed
    /// startup route-probe ack — see [`STARTUP_ACK_PARAM`]. Malformed acks
    /// (missing or wrongly-typed identity parameters) return `None` and are
    /// ignored by producers, which keeps the affected output on the reliable
    /// daemon path instead of switching on bad evidence.
    pub fn startup_ack_identity(&self) -> Option<(&str, &str)> {
        if !get_bool_param(&self.parameters, STARTUP_ACK_PARAM).unwrap_or(false) {
            return None;
        }
        let consumer = get_string_param(&self.parameters, STARTUP_ACK_CONSUMER_PARAM)?;
        let input = get_string_param(&self.parameters, STARTUP_ACK_INPUT_PARAM)?;
        Some((consumer, input))
    }

    pub fn from_parameters(timestamp: uhlc::Timestamp, parameters: MetadataParameters) -> Self {
        Self {
            metadata_version: Self::CURRENT_VERSION,
            timestamp,
            parameters,
        }
    }

    /// The wire-format version stamped on this metadata. Compare against
    /// [`CURRENT_VERSION`](Self::CURRENT_VERSION) on receive to reject peers
    /// using an incompatible format.
    pub fn metadata_version(&self) -> u16 {
        self.metadata_version
    }

    pub fn timestamp(&self) -> uhlc::Timestamp {
        self.timestamp
    }

    pub fn open_telemetry_context(&self) -> String {
        get_string_param(&self.parameters, "open_telemetry_context")
            .unwrap_or("")
            .to_string()
    }
}

/// Reserved [`MetadataParameters`] key marking a message as a **startup
/// route-probe marker** rather than node data.
///
/// The zenoh data plane is direct node-to-node pub/sub, so a producer that
/// publishes before a consumer's subscription has propagated would drop those
/// early samples. Rather than infer route-readiness from zenoh declarations,
/// each producer publishes markers on its real output topic while an output is
/// still on the reliable daemon path, and each consumer answers every received
/// marker with an ack (see [`STARTUP_ACK_PARAM`]): an ack arriving back at the
/// producer is end-to-end proof that the route pair carries data. The producer
/// stops marking an output — and switches it to the direct zenoh path — once
/// all its required consumers have acked; a route that never proves itself
/// just keeps the output on the daemon path.
///
/// The `__dora_` prefix is reserved; user parameters must not use it. Receivers
/// filter markers before decoding the payload, so they never reach user code.
pub const STARTUP_MARKER_PARAM: &str = "__dora_startup_marker";

/// Reserved [`MetadataParameters`] key marking a message as a **startup
/// route-probe ack**: the consumer-side half of the startup handshake.
///
/// When a consumer's data subscriber receives a startup marker (see
/// [`STARTUP_MARKER_PARAM`]) for an input, it replies with an ack on the
/// output's dedicated `@ack` topic. An ack arriving back at the producer is
/// end-to-end proof that the route works in *both* directions; once every
/// required consumer of an output has acked, the producer switches that output
/// from the reliable daemon path to the direct node-to-node zenoh path. A
/// missing ack never fails anything — the output simply stays on the daemon
/// path.
///
/// Acks travel as an empty-payload message whose attachment carries this flag
/// plus the acking consumer's identity under [`STARTUP_ACK_CONSUMER_PARAM`] and
/// [`STARTUP_ACK_INPUT_PARAM`] — the identity rides in the attachment rather
/// than the zenoh key so consumer/input ids never need key escaping.
pub const STARTUP_ACK_PARAM: &str = "__dora_startup_ack";

/// Reserved key carrying the acking consumer's node id as a
/// [`Parameter::String`] — see [`STARTUP_ACK_PARAM`].
pub const STARTUP_ACK_CONSUMER_PARAM: &str = "__dora_startup_ack_consumer";

/// Reserved key carrying the acking consumer's input id as a
/// [`Parameter::String`] — see [`STARTUP_ACK_PARAM`].
pub const STARTUP_ACK_INPUT_PARAM: &str = "__dora_startup_ack_input";

/// Additional metadata that can be sent as part of output messages.
pub type MetadataParameters = BTreeMap<String, Parameter>;

/// A metadata parameter that can be sent as part of output messages.
#[derive(Debug, PartialEq, Clone, Serialize, Deserialize)]
pub enum Parameter {
    Bool(bool),
    Integer(i64),
    String(String),
    ListInt(Vec<i64>),
    Float(f64),
    ListFloat(Vec<f64>),
    ListString(Vec<String>),
    Timestamp(DateTime<Utc>),
}

/// Extract a string parameter from metadata, returning `None` if missing or
/// not a `Parameter::String`.
pub fn get_string_param<'a>(params: &'a MetadataParameters, key: &str) -> Option<&'a str> {
    params.get(key).and_then(|p| match p {
        Parameter::String(s) => Some(s.as_str()),
        _ => None,
    })
}

/// Extract an integer parameter from metadata, returning `None` if missing or
/// not a `Parameter::Integer`.
pub fn get_integer_param(params: &MetadataParameters, key: &str) -> Option<i64> {
    params.get(key).and_then(|p| match p {
        Parameter::Integer(n) => Some(*n),
        _ => None,
    })
}

/// Extract a bool parameter from metadata, returning `None` if missing or
/// not a `Parameter::Bool`.
pub fn get_bool_param(params: &MetadataParameters, key: &str) -> Option<bool> {
    params.get(key).and_then(|p| match p {
        Parameter::Bool(b) => Some(*b),
        _ => None,
    })
}

// ---------------------------------------------------------------------------
// Well-known metadata parameter keys for service and action patterns
// ---------------------------------------------------------------------------

/// Metadata key for correlating a service request with its response.
pub const REQUEST_ID: &str = "request_id";

/// Metadata key for identifying an action goal across feedback/result messages.
pub const GOAL_ID: &str = "goal_id";

/// Metadata key for the completion status of an action goal.
pub const GOAL_STATUS: &str = "goal_status";

/// Goal completed successfully.
pub const GOAL_STATUS_SUCCEEDED: &str = "succeeded";

/// Goal was aborted by the server.
pub const GOAL_STATUS_ABORTED: &str = "aborted";

/// Goal was canceled by the client.
pub const GOAL_STATUS_CANCELED: &str = "canceled";

// ---------------------------------------------------------------------------
// Well-known metadata parameter keys for the streaming pattern
// ---------------------------------------------------------------------------

/// Metadata key identifying the conversation/session.
pub const SESSION_ID: &str = "session_id";

/// Metadata key for the logical segment within a session (e.g. one utterance).
pub const SEGMENT_ID: &str = "segment_id";

/// Metadata key for chunk sequence number within a segment.
pub const SEQ: &str = "seq";

/// Metadata key marking the last chunk of a segment (`true` on final chunk).
pub const FIN: &str = "fin";

/// Metadata key to discard older queued messages on this input (`true` to flush).
pub const FLUSH: &str = "flush";

/// Metadata key indicating the wire framing of the data payload.
/// When set to `"arrow-ipc"`, the payload is an Arrow IPC stream.
pub const FRAMING: &str = "_framing";

/// Value for [`FRAMING`] indicating Arrow IPC stream framing.
pub const FRAMING_ARROW_IPC: &str = "arrow-ipc";

/// Metadata key carrying the FNV-1a hash (as an `i64`) of the Arrow IPC schema
/// for a zenoh data message. Present on schema-once messages so a receiver can
/// tell which primed decoder a schema-less batch belongs to and detect schema
/// changes. Absent on messages a receiver should decode as a standalone full
/// stream (large/SHM and daemon-path payloads).
pub const SCHEMA_HASH: &str = "_schema_hash";

/// Metadata key carrying the true on-wire byte size (as an `i64`) of a
/// `dora topic` debug frame's data sample.
///
/// The daemon rebuilds a self-describing Arrow IPC stream for inspection by
/// prepending the retained schema block to each schema-once batch, so the
/// rebuilt stream the CLI receives is larger than what actually travelled on
/// the wire (the schema-less batch — the schema is published only once on the
/// `@schema` subtopic). To keep `dora topic info`'s bandwidth accounting
/// accurate, the daemon stamps the original data-sample length here; the CLI
/// measures this instead of the rebuilt stream length (dora-rs/dora#2584).
///
/// Debug/inspection path only — never set on real node→node outputs.
pub const WIRE_SIZE: &str = "_wire_size";

/// Byte size to charge for a `dora topic` debug frame when accounting bandwidth.
///
/// Prefers the daemon-stamped [`WIRE_SIZE`] (the real on-wire data-sample
/// length): the `data` the CLI receives is a rebuilt self-describing stream
/// whose schema was re-prepended for inspection, so for a schema-once output
/// `data.len()` over-reports what actually travelled. Falls back to the buffer
/// length when the key is absent — an older daemon, or a non-debug frame that
/// never carried it (dora-rs/dora#2584). A present-but-out-of-range stamp (a
/// negative `i64` that can't be a byte count) also falls back rather than
/// silently counting zero. Keep this the single reader of [`WIRE_SIZE`] so the
/// daemon stamp and the CLI accounting can never disagree on the fallback rule.
pub fn debug_frame_wire_size(params: &MetadataParameters, data: Option<&[u8]>) -> usize {
    get_integer_param(params, WIRE_SIZE)
        .and_then(|n| usize::try_from(n).ok())
        .or_else(|| data.map(|d| d.len()))
        .unwrap_or(0)
}

/// Returns `true` if the given parameters carry any pattern-correlation key
/// ([`REQUEST_ID`], [`GOAL_ID`], or [`GOAL_STATUS`]).
///
/// Messages marked with these keys belong to a service or action pattern where
/// multiple Arrow schemas can legitimately flow through a single output/input,
/// distinguished by metadata rather than a fixed Arrow type. Runtime type
/// checks skip such messages (dora-rs/adora#150), and the schema-once zenoh
/// optimization excludes them — on the send side (they always travel as full
/// self-describing streams), on the node receive side (their schemas must not
/// churn the per-input decoder), and on the daemon's `dora topic` debug path
/// (same, for its schema cache). Keep this the single definition so those
/// layers can never disagree on what "pattern-correlated" means.
pub fn carries_pattern_correlation(params: &MetadataParameters) -> bool {
    params.contains_key(REQUEST_ID)
        || params.contains_key(GOAL_ID)
        || params.contains_key(GOAL_STATUS)
}

/// Remove internal wire-protocol keys ([`SCHEMA_HASH`], [`FRAMING`]) from a
/// parameter map. Call this at every wire→user boundary: the keys are
/// meaningless after decode, and a stale [`SCHEMA_HASH`] forwarded from an
/// input's metadata into `send_output` parameters (a standard pattern, e.g.
/// replay) would ride onto outputs that don't overwrite it, making receivers
/// hash-mismatch and silently drop them (dora-rs/dora#2366 review).
pub fn strip_internal_parameters(params: &mut MetadataParameters) {
    params.remove(SCHEMA_HASH);
    params.remove(FRAMING);
}

/// FNV-1a-64 hash with a fixed seed (cross-process deterministic). Used to
/// fingerprint an Arrow IPC schema block so a schema-less batch can be matched
/// to the schema it was encoded against (see [`SCHEMA_HASH`]). The producer
/// node, the consumer node, and the daemon's `dora topic` debug path all hash
/// the same schema-block bytes with this function, so the value must stay
/// identical across crates — keep this the single source of truth.
pub fn fnv1a(bytes: &[u8]) -> u64 {
    let mut hash: u64 = 0xcbf29ce484222325;
    for b in bytes {
        hash ^= *b as u64;
        hash = hash.wrapping_mul(0x100000001b3);
    }
    hash
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fnv1a_matches_standard_vectors() {
        // Canonical FNV-1a-64 vectors — pin the algorithm so the producer and
        // consumers (across crates/processes) never disagree on a schema hash.
        assert_eq!(fnv1a(b""), 0xcbf29ce484222325);
        assert_eq!(fnv1a(b"a"), 0xaf63dc4c8601ec8c);
    }

    fn test_timestamp() -> uhlc::Timestamp {
        uhlc::HLC::default().new_timestamp()
    }

    #[test]
    fn startup_marker_is_detected_and_survives_the_wire() {
        // A marker is recognized only via the reserved parameter, and the flag
        // must survive bincode round-tripping — it travels as the zenoh
        // attachment, and a receiver that failed to recognize it would decode
        // the marker as node data and surface it to user code.
        let marker = Metadata::startup_marker(test_timestamp());
        assert!(marker.is_startup_marker());

        let bytes = bincode::serialize(&marker).expect("serialize");
        let decoded: Metadata = bincode::deserialize(&bytes).expect("deserialize");
        assert!(decoded.is_startup_marker());
    }

    #[test]
    fn ordinary_metadata_is_not_a_startup_marker() {
        // Guard the other direction: real data must never be mistaken for a
        // marker (it would be silently dropped instead of delivered).
        assert!(!Metadata::new(test_timestamp()).is_startup_marker());

        // Wrong type under the reserved key must not count as a marker.
        let wrong_type = Metadata::from_parameters(
            test_timestamp(),
            BTreeMap::from([(
                STARTUP_MARKER_PARAM.to_owned(),
                Parameter::String("true".into()),
            )]),
        );
        assert!(!wrong_type.is_startup_marker());

        // Explicit `false` is not a marker either.
        let explicit_false = Metadata::from_parameters(
            test_timestamp(),
            BTreeMap::from([(STARTUP_MARKER_PARAM.to_owned(), Parameter::Bool(false))]),
        );
        assert!(!explicit_false.is_startup_marker());
    }

    #[test]
    fn startup_marker_key_is_reserved_and_distinct() {
        // The `__dora_` prefix keeps it out of the user parameter namespace, and
        // it must not collide with any well-known protocol key.
        assert_eq!(STARTUP_MARKER_PARAM, "__dora_startup_marker");
        assert!(STARTUP_MARKER_PARAM.starts_with("__dora_"));
        for key in [
            REQUEST_ID,
            GOAL_ID,
            GOAL_STATUS,
            SESSION_ID,
            SEGMENT_ID,
            SEQ,
            FIN,
            FLUSH,
        ] {
            assert_ne!(STARTUP_MARKER_PARAM, key);
        }
    }

    #[test]
    fn startup_ack_round_trips_and_extracts_identity() {
        // The ack travels as a bincode attachment on the `@ack` topic; the
        // producer must recover exactly the (consumer, input) identity it needs
        // to tick off a required acker.
        let ack = Metadata::startup_ack(test_timestamp(), "camera-consumer", "image/depth");
        assert_eq!(
            ack.startup_ack_identity(),
            Some(("camera-consumer", "image/depth"))
        );
        // An ack is not a marker (and vice versa, checked below): the two
        // travel on different topics but share the filtering code path.
        assert!(!ack.is_startup_marker());

        let bytes = bincode::serialize(&ack).expect("serialize");
        let decoded: Metadata = bincode::deserialize(&bytes).expect("deserialize");
        assert_eq!(
            decoded.startup_ack_identity(),
            Some(("camera-consumer", "image/depth"))
        );
    }

    #[test]
    fn malformed_startup_acks_are_rejected() {
        // Ordinary metadata and markers are not acks.
        assert_eq!(Metadata::new(test_timestamp()).startup_ack_identity(), None);
        assert_eq!(
            Metadata::startup_marker(test_timestamp()).startup_ack_identity(),
            None
        );

        // Flag present but identity missing → not a valid ack: a producer must
        // never count an acker it cannot identify.
        let flag_only = Metadata::from_parameters(
            test_timestamp(),
            BTreeMap::from([(STARTUP_ACK_PARAM.to_owned(), Parameter::Bool(true))]),
        );
        assert_eq!(flag_only.startup_ack_identity(), None);

        // Wrongly-typed identity parameters are rejected too.
        let wrong_types = Metadata::from_parameters(
            test_timestamp(),
            BTreeMap::from([
                (STARTUP_ACK_PARAM.to_owned(), Parameter::Bool(true)),
                (STARTUP_ACK_CONSUMER_PARAM.to_owned(), Parameter::Integer(1)),
                (STARTUP_ACK_INPUT_PARAM.to_owned(), Parameter::Integer(2)),
            ]),
        );
        assert_eq!(wrong_types.startup_ack_identity(), None);

        // `Bool(false)` under the flag key is not an ack.
        let explicit_false = Metadata::from_parameters(
            test_timestamp(),
            BTreeMap::from([
                (STARTUP_ACK_PARAM.to_owned(), Parameter::Bool(false)),
                (
                    STARTUP_ACK_CONSUMER_PARAM.to_owned(),
                    Parameter::String("c".into()),
                ),
                (
                    STARTUP_ACK_INPUT_PARAM.to_owned(),
                    Parameter::String("i".into()),
                ),
            ]),
        );
        assert_eq!(explicit_false.startup_ack_identity(), None);
    }

    #[test]
    fn startup_ack_keys_are_reserved_and_distinct() {
        let ack_keys = [
            STARTUP_ACK_PARAM,
            STARTUP_ACK_CONSUMER_PARAM,
            STARTUP_ACK_INPUT_PARAM,
        ];
        for key in ack_keys {
            assert!(key.starts_with("__dora_"));
            assert_ne!(key, STARTUP_MARKER_PARAM);
        }
        for (i, a) in ack_keys.iter().enumerate() {
            for b in &ack_keys[i + 1..] {
                assert_ne!(a, b);
            }
        }
    }

    #[test]
    fn well_known_keys_have_stable_values() {
        // These string values are part of the cross-language protocol
        // (Python nodes use the same literal strings). Do not change.
        assert_eq!(REQUEST_ID, "request_id");
        assert_eq!(GOAL_ID, "goal_id");
        assert_eq!(GOAL_STATUS, "goal_status");
        assert_eq!(GOAL_STATUS_SUCCEEDED, "succeeded");
        assert_eq!(GOAL_STATUS_ABORTED, "aborted");
        assert_eq!(GOAL_STATUS_CANCELED, "canceled");
        assert_eq!(SESSION_ID, "session_id");
        assert_eq!(SEGMENT_ID, "segment_id");
        assert_eq!(SEQ, "seq");
        assert_eq!(FIN, "fin");
        assert_eq!(FLUSH, "flush");
    }

    #[test]
    fn well_known_keys_are_distinct() {
        let keys = [
            REQUEST_ID,
            GOAL_ID,
            GOAL_STATUS,
            SESSION_ID,
            SEGMENT_ID,
            SEQ,
            FIN,
            FLUSH,
        ];
        for (i, a) in keys.iter().enumerate() {
            for b in &keys[i + 1..] {
                assert_ne!(a, b);
            }
        }
    }

    #[test]
    fn goal_status_values_are_distinct() {
        let vals = [
            GOAL_STATUS_SUCCEEDED,
            GOAL_STATUS_ABORTED,
            GOAL_STATUS_CANCELED,
        ];
        for (i, a) in vals.iter().enumerate() {
            for b in &vals[i + 1..] {
                assert_ne!(a, b);
            }
        }
    }

    #[test]
    fn outgoing_metadata_is_stamped_with_current_version() {
        // The wire format is positional (bincode) and carries no separate type
        // descriptor, so `metadata_version` is the only in-band signal of an
        // incompatible layout. Every constructor must stamp `CURRENT_VERSION`.
        assert_eq!(Metadata::CURRENT_VERSION, 1);
        let ts = uhlc::HLC::default().new_timestamp();
        assert_eq!(
            Metadata::new(ts).metadata_version(),
            Metadata::CURRENT_VERSION
        );
        assert_eq!(
            Metadata::from_parameters(ts, Default::default()).metadata_version(),
            Metadata::CURRENT_VERSION
        );
    }

    #[test]
    fn get_string_param_extracts_string() {
        let mut params = MetadataParameters::default();
        params.insert("key".to_string(), Parameter::String("value".to_string()));
        assert_eq!(get_string_param(&params, "key"), Some("value"));
        assert_eq!(get_string_param(&params, "missing"), None);
    }

    #[test]
    fn get_string_param_returns_none_for_non_string() {
        let mut params = MetadataParameters::default();
        params.insert("num".to_string(), Parameter::Integer(42));
        assert_eq!(get_string_param(&params, "num"), None);
    }

    #[test]
    fn get_integer_param_extracts_integer() {
        let mut params = MetadataParameters::default();
        params.insert("key".to_string(), Parameter::Integer(42));
        assert_eq!(get_integer_param(&params, "key"), Some(42));
        assert_eq!(get_integer_param(&params, "missing"), None);
    }

    #[test]
    fn get_integer_param_returns_none_for_non_integer() {
        let mut params = MetadataParameters::default();
        params.insert("s".to_string(), Parameter::String("hello".to_string()));
        assert_eq!(get_integer_param(&params, "s"), None);
    }

    #[test]
    fn get_bool_param_extracts_bool() {
        let mut params = MetadataParameters::default();
        params.insert("key".to_string(), Parameter::Bool(true));
        assert_eq!(get_bool_param(&params, "key"), Some(true));
        assert_eq!(get_bool_param(&params, "missing"), None);
    }

    #[test]
    fn get_bool_param_returns_none_for_non_bool() {
        let mut params = MetadataParameters::default();
        params.insert("n".to_string(), Parameter::Integer(1));
        assert_eq!(get_bool_param(&params, "n"), None);
    }

    #[test]
    fn debug_frame_wire_size_prefers_stamped_value() {
        // The stamped size wins over the (larger, schema-inflated) buffer: a
        // schema-once frame's rebuilt `data` is bigger than what travelled.
        let mut params = MetadataParameters::default();
        params.insert(WIRE_SIZE.to_string(), Parameter::Integer(17));
        assert_eq!(debug_frame_wire_size(&params, Some(&[0u8; 42])), 17);
    }

    #[test]
    fn debug_frame_wire_size_falls_back_to_buffer_len() {
        // No stamp (older daemon / non-debug frame) ⇒ use the buffer length.
        let params = MetadataParameters::default();
        assert_eq!(debug_frame_wire_size(&params, Some(&[0u8; 42])), 42);
    }

    #[test]
    fn debug_frame_wire_size_falls_back_on_out_of_range_stamp() {
        // A negative i64 can't be a byte count; fall back rather than count 0.
        let mut params = MetadataParameters::default();
        params.insert(WIRE_SIZE.to_string(), Parameter::Integer(-1));
        assert_eq!(debug_frame_wire_size(&params, Some(&[0u8; 42])), 42);
    }

    #[test]
    fn debug_frame_wire_size_zero_without_stamp_or_buffer() {
        assert_eq!(
            debug_frame_wire_size(&MetadataParameters::default(), None),
            0
        );
    }
}
