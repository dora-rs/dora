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
}
