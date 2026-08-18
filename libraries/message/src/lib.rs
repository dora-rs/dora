//! Message types and wire protocol shared by all dora components.
//!
//! Binary encoding is [postcard](https://docs.rs/postcard): a compact,
//! non-self-describing serde format with a stable, documented wire spec. The
//! encoding is positional, so field order and enum variant order are part of
//! the protocol — see [`metadata::Metadata::CURRENT_VERSION`] for how an
//! incompatible peer is detected.
//!

/// The version of the dora-message crate
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

pub use uhlc;

/// Per-message slack when pre-sizing an encode buffer, covering the envelope
/// around the bulk payload: enum tags, ids, [`metadata::Metadata`] and varint
/// length prefixes. Overshooting is free (one slightly larger allocation);
/// undershooting costs a realloc plus a memcpy of the payload.
const ENVELOPE_SIZE_HINT: usize = 512;

/// Encode `value` in dora's binary wire format.
///
/// This and [`encode_presized`] are the only encode entry points; call sites do
/// not name the underlying codec, so it stays swappable from one file.
pub fn encode<T: serde::Serialize>(value: &T) -> postcard::Result<Vec<u8>> {
    encode_presized(value, 0)
}

/// [`encode`], for a message carrying `bulk_bytes` bytes of bulk payload.
///
/// The encoder writes into a `Vec` that would otherwise start empty and
/// reallocate as it grows — 10–80% of the encode cost on dora's messages, worst
/// on the small control messages that dominate the daemon↔node TCP path. Pass
/// `encode_size_hint()`, which the message types that carry a payload provide;
/// the envelope slack is added here so that policy lives in one place.
pub fn encode_presized<T: serde::Serialize>(
    value: &T,
    bulk_bytes: usize,
) -> postcard::Result<Vec<u8>> {
    postcard::to_extend(
        value,
        Vec::with_capacity(bulk_bytes.saturating_add(ENVELOPE_SIZE_HINT)),
    )
}

/// Decode `bytes` in dora's binary wire format, requiring the value to consume
/// the **entire** slice.
///
/// This is the only decode entry point, and the trailing-byte check is why it
/// must stay that way: postcard's own `from_bytes` ignores trailing bytes where
/// bincode (this protocol's previous codec) rejected them, and two call sites
/// depend on the strict behaviour.
///
/// - The zenoh attachment filters read a decode failure as "not a dora message,
///   ignore it". A foreign publisher whose attachment merely *starts* with
///   something shaped like a [`metadata::Metadata`] would otherwise be accepted
///   as genuine.
/// - The length-prefixed daemon↔node frames read a decode failure as a desynced
///   or incompatible peer. A frame longer than the value it carries is evidence
///   of exactly that, and must not be silently truncated.
pub fn decode<'a, T: serde::Deserialize<'a>>(bytes: &'a [u8]) -> eyre::Result<T> {
    let (value, rest) = postcard::take_from_bytes(bytes)?;
    if !rest.is_empty() {
        eyre::bail!(
            "trailing bytes after decoded value ({} of {} unconsumed) — \
             likely a desynced or incompatible peer",
            rest.len(),
            bytes.len()
        );
    }
    Ok(value)
}

/// Maximum allowed message size over TCP (64 MiB).
///
/// Large payloads should use the shared-memory transport instead,
/// which bypasses this limit via zero-copy IPC.
pub const MAX_MESSAGE_BYTES: usize = 64 * 1024 * 1024;

/// Read timeout for TCP/socket connections (30 seconds).
pub const TCP_READ_TIMEOUT: std::time::Duration = std::time::Duration::from_secs(30);

/// Encoding version of the WebSocket topic data channel's **binary** frames
/// (see `docs/websocket-topic-data-channel.md`).
///
/// The subscription handshake is JSON and self-describing, but the frames that
/// follow it are `subscription_id ++ <encoded Timestamped<InterDaemonEvent>>`
/// with no per-frame envelope. Both the old (bincode) and current (postcard)
/// encodings are positional, so a peer speaking the wrong one does not fail to
/// decode — it *misparses*, yielding plausible-looking garbage.
///
/// Version 1 is the bincode encoding; version 2 is postcard. Peers exchange
/// this in the `TopicSubscribe`/`TopicSubscribed` handshake and refuse the
/// subscription on mismatch, so the failure is a clear error at subscribe time
/// rather than corrupt data later.
///
/// A peer that predates versioning sends no value at all; that absence is
/// treated as "incompatible" for the same reason.
///
/// # Why the `Hello` handshake is not enough
///
/// [`cli_to_coordinator::check_cli_version`] already rejects a CLI whose dora
/// version is incompatible with the coordinator's, which covers the
/// bincode-to-postcard break at the 1.0 boundary. It does not make this
/// redundant, for two reasons:
///
/// - **Third-party subscribers never send `Hello`.** The channel is documented
///   for external consumers, who reach the binary frames without any version
///   exchange at all.
/// - **`versions_compatible` is semver-caret.** A 1.0 peer and a 1.5 peer are
///   "compatible", so if this encoding ever changes again inside the 1.x
///   series, `Hello` waves it through and only this version catches it.
pub const TOPIC_DATA_PROTOCOL_VERSION: u16 = 2;

/// Rejection message for a topic-data peer whose binary-frame encoding is not
/// [`TOPIC_DATA_PROTOCOL_VERSION`].
///
/// Shared by both sides of the handshake so the wording, and the explanation of
/// why a mismatch cannot simply be tolerated, live next to the constant they
/// describe. `peer` names the other side ("client" / "coordinator") so the
/// message reads correctly in whichever direction it is produced.
pub fn topic_protocol_mismatch_message(peer: &str, peer_version: Option<u16>) -> String {
    let ours = TOPIC_DATA_PROTOCOL_VERSION;
    let peer_state = match peer_version {
        Some(version) => format!("{peer} speaks version {version}"),
        None => format!("{peer} predates the topic data protocol handshake (bincode-era frames)"),
    };
    format!(
        "topic data protocol mismatch: {peer_state}, this side speaks {ours}. \
         Binary frames are positionally encoded, so subscribing would silently \
         misparse rather than fail. Upgrade whichever side is older."
    )
}

pub mod auth;
pub(crate) mod bulk_bytes;
pub mod common;
pub mod config;
/// Dataflow descriptor types for YAML-based dataflow specifications.
pub mod descriptor;
pub mod id;
pub mod metadata;

pub mod coordinator_to_daemon;
pub mod daemon_to_coordinator;

pub mod daemon_to_daemon;

pub mod daemon_to_node;
pub mod node_to_daemon;

pub mod cli_to_coordinator;
pub mod coordinator_to_cli;

pub mod ws_protocol;

pub mod integration_testing_format;

pub use aligned_vec;
pub use arrow_data;
pub use arrow_schema;
use uuid::{Timestamp, Uuid};

/// Unique identifier for a dataflow instance.
///
/// Dora assigns each dataflow instance a unique ID on start.
pub type DataflowId = uuid::Uuid;

#[derive(
    Debug, Clone, Copy, serde::Serialize, serde::Deserialize, PartialEq, Eq, PartialOrd, Ord, Hash,
)]
pub struct SessionId(uuid::Uuid);

impl SessionId {
    pub fn generate() -> Self {
        Self(Uuid::new_v7(Timestamp::now(uuid::NoContext)))
    }

    pub fn uuid(&self) -> uuid::Uuid {
        self.0
    }
}

#[derive(
    Debug, Clone, Copy, serde::Serialize, serde::Deserialize, PartialEq, Eq, PartialOrd, Ord, Hash,
)]
pub struct BuildId(uuid::Uuid);

impl BuildId {
    pub fn generate() -> Self {
        Self(Uuid::new_v7(Timestamp::now(uuid::NoContext)))
    }

    /// Parse a `BuildId` from its [`Display`](std::fmt::Display) form,
    /// `BuildId(<uuid>)`, so a value logged with `%build_id` round-trips.
    ///
    /// A bare UUID is also accepted for backward compatibility.
    ///
    /// ```
    /// use dora_message::BuildId;
    ///
    /// let id = BuildId::generate();
    /// // The `Display` form recovers the original id...
    /// assert_eq!(BuildId::from_display_str(&id.to_string()), Some(id));
    /// // ...as does a bare UUID.
    /// assert_eq!(BuildId::from_display_str(&id.uuid().to_string()), Some(id));
    /// // Garbage does not parse to a bogus id.
    /// assert_eq!(BuildId::from_display_str("not-a-build-id"), None);
    /// ```
    pub fn from_display_str(s: &str) -> Option<Self> {
        let inner = s
            .strip_prefix("BuildId(")
            .and_then(|rest| rest.strip_suffix(')'))
            .unwrap_or(s);
        Uuid::parse_str(inner).ok().map(BuildId)
    }

    /// The underlying UUID.
    pub fn uuid(&self) -> uuid::Uuid {
        self.0
    }
}

impl std::fmt::Display for BuildId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "BuildId({})", self.0)
    }
}

pub fn current_crate_version() -> semver::Version {
    let crate_version_raw = env!("CARGO_PKG_VERSION");

    semver::Version::parse(crate_version_raw).unwrap()
}

pub(crate) fn versions_compatible(
    crate_version: &semver::Version,
    specified_version: &semver::Version,
) -> Result<bool, String> {
    let req = semver::VersionReq::parse(&crate_version.to_string()).map_err(|error| {
        format!("failed to parse crate version `{crate_version}` as `VersionReq`: {error}")
    })?;
    let specified_dora_req = semver::VersionReq::parse(&specified_version.to_string())
        .map_err(|error| {
            format!(
                "failed to parse specified dora version `{specified_version}` as `VersionReq`: {error}",
            )
        })?;
    let matches = req.matches(specified_version) || specified_dora_req.matches(crate_version);
    Ok(matches)
}

#[cfg(test)]
mod encoding_tests {
    use crate::metadata::Metadata;

    /// The property the zenoh attachment filters rely on: a buffer that merely
    /// *starts* with a valid value is not a valid message.
    #[test]
    fn decode_rejects_trailing_bytes() {
        let mut bytes =
            crate::encode(&Metadata::new(uhlc::HLC::default().new_timestamp())).expect("serialize");
        bytes.extend_from_slice(b"foreign publisher trailer");

        let err = crate::decode::<Metadata>(&bytes).expect_err("trailing bytes must be rejected");
        assert!(
            format!("{err:#}").contains("trailing bytes"),
            "error should name the cause, got: {err:#}"
        );
    }
}

#[cfg(test)]
mod topic_protocol_tests {
    use super::*;

    /// The message is the only thing an operator sees when a subscription is
    /// refused, so it has to name the peer's version (or say it has none), name
    /// ours, and explain why a mismatch cannot simply be tolerated.
    #[test]
    fn mismatch_message_names_both_sides_and_the_reason() {
        let msg = topic_protocol_mismatch_message("client", Some(1));
        assert!(msg.contains("client speaks version 1"), "got: {msg}");
        assert!(
            msg.contains(&TOPIC_DATA_PROTOCOL_VERSION.to_string()),
            "should name our own version, got: {msg}"
        );
        assert!(
            msg.contains("misparse"),
            "should say why a mismatch is fatal rather than merely different, got: {msg}"
        );
    }

    /// An absent version is a peer from before the handshake existed, which is a
    /// different diagnosis from "speaks a number we don't like".
    #[test]
    fn mismatch_message_distinguishes_a_pre_handshake_peer() {
        let msg = topic_protocol_mismatch_message("coordinator", None);
        assert!(msg.contains("predates"), "got: {msg}");
        assert!(
            !msg.contains("version None"),
            "should not leak a Debug-formatted Option, got: {msg}"
        );
    }

    /// Both directions of the handshake share one message, so the peer label has
    /// to be what varies — not two divergent copies of the wording.
    #[test]
    fn peer_label_is_what_varies_between_directions() {
        let from_coordinator = topic_protocol_mismatch_message("client", Some(1));
        let from_cli = topic_protocol_mismatch_message("coordinator", Some(1));
        assert_ne!(from_coordinator, from_cli);
        assert!(from_coordinator.contains("client speaks"));
        assert!(from_cli.contains("coordinator speaks"));
    }
}
