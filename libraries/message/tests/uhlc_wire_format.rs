//! Wire-format contract between `dora-message` and its `uhlc` dependency.
//!
//! `uhlc::Timestamp` is embedded in every bincode-encoded daemon↔node,
//! daemon↔daemon and record-file message (`Metadata`, `Timestamped`), and in
//! every JSON-encoded CLI↔coordinator and coordinator↔daemon WebSocket frame.
//! Its encoding is therefore part of dora's on-the-wire protocol even though the
//! type is owned by an upstream crate — a silent change to it would corrupt HLC
//! clock identities across a mixed-version deployment rather than failing
//! loudly.
//!
//! The golden vectors below pin that encoding so a future `uhlc` bump cannot
//! move it unnoticed. They were captured under `uhlc 0.5.2` and still hold
//! under `uhlc 0.9.0` (dora-rs/dora#2446): 0.9 changed the in-memory
//! representation of `ID` from `NonZeroU128` to `[u8; 16]`, but both encode to
//! the same 16 little-endian bytes under bincode, so the binary plane is
//! untouched. The JSON plane *did* change shape (see
//! `timestamp_json_encoding_is_pinned`) — that link is version-gated and now
//! survives `serde_json::Value`, which the `NonZeroU128` form could not.

use dora_message::{
    common::Timestamped,
    metadata::Metadata,
    uhlc::{ID, NTP64, Timestamp},
};

/// The pinned bincode encoding of [`golden_timestamp`]: the NTP64 as a
/// little-endian `u64`, then the HLC id as its 16 little-endian bytes.
///
/// This exact vector is what a uhlc-0.5 dora produced and what a uhlc-0.9 dora
/// produces. If a bump changes it, every daemon↔node message, every inter-daemon
/// zenoh sample and every `dora record` file written by a different dora version
/// becomes unreadable — so a test failing against it means a protocol migration
/// is genuinely required, not that the vector needs updating.
const TIMESTAMP_HEX: &str = "88776655443322110102030405060708090a0b0c0d0e0f10";

/// A fixed timestamp: an arbitrary but non-round NTP64 and a 16-byte HLC id
/// whose bytes are all distinct, so any reordering (the failure mode #2446
/// feared) shows up immediately rather than being masked by symmetry.
fn golden_timestamp() -> Timestamp {
    let id_bytes: [u8; 16] = [
        0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
        0x10,
    ];
    Timestamp::new(
        NTP64(0x1122_3344_5566_7788),
        ID::try_from(&id_bytes).expect("non-zero id"),
    )
}

fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{b:02x}")).collect()
}

#[test]
fn timestamp_bincode_encoding_is_pinned() {
    let encoded = bincode::serialize(&golden_timestamp()).expect("serialize timestamp");
    assert_eq!(
        hex(&encoded),
        TIMESTAMP_HEX,
        "uhlc::Timestamp bincode encoding changed — this is a protocol break"
    );
}

#[test]
fn timestamp_bincode_round_trips() {
    let original = golden_timestamp();
    let encoded = bincode::serialize(&original).expect("serialize");
    let decoded: Timestamp = bincode::deserialize(&encoded).expect("deserialize");

    assert_eq!(decoded, original);
    // Compare the id through its byte form too: `PartialEq` alone would still
    // hold if both sides reordered consistently.
    assert_eq!(
        hex(&decoded.get_id().to_le_bytes()),
        "0102030405060708090a0b0c0d0e0f10"
    );
    assert_eq!(decoded.get_time().as_u64(), 0x1122_3344_5566_7788);
}

#[test]
fn metadata_bincode_encoding_is_pinned() {
    // `Metadata` is the envelope that actually rides on every output message,
    // so pin the composite too: `metadata_version` as a little-endian u16, the
    // 24-byte timestamp, then the parameter map length as a little-endian u64.
    let encoded = bincode::serialize(&Metadata::new(golden_timestamp())).expect("serialize");
    assert_eq!(
        hex(&encoded),
        format!(
            "{version}{TIMESTAMP_HEX}{empty_parameter_map}",
            version = "0100",
            empty_parameter_map = "0000000000000000",
        ),
        "Metadata bincode encoding changed — bump Metadata::CURRENT_VERSION"
    );
}

#[test]
fn timestamped_bincode_prefix_is_the_inner_value() {
    // `Timestamped<T>` puts `inner` first and the timestamp last, so the
    // timestamp's 24 bytes are the tail of every daemon↔coordinator frame.
    let timestamped = Timestamped {
        inner: 0xABu8,
        timestamp: golden_timestamp(),
    };
    let encoded = bincode::serialize(&timestamped).expect("serialize");
    assert_eq!(hex(&encoded), format!("ab{TIMESTAMP_HEX}"));
}

#[test]
fn timestamp_json_encoding_is_pinned() {
    // The CLI↔coordinator and coordinator↔daemon WebSocket links carry
    // `Timestamped<_>` as JSON text. Under uhlc 0.5 the id serialized as a bare
    // `u128` number; under 0.9 it is a 16-element little-endian byte array. The
    // byte *values* are the same and in the same order — only the JSON shape
    // changed. Both links are gated by the `dora_version` semver handshake, and
    // a version mismatch now fails at parse time with a type error rather than
    // being silently misread.
    let json = serde_json::to_string(&golden_timestamp()).expect("serialize");
    assert_eq!(
        json,
        r#"{"time":1234605616436508552,"id":[1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16]}"#
    );

    let decoded: Timestamp = serde_json::from_str(&json).expect("deserialize");
    assert_eq!(decoded, golden_timestamp());
}

#[test]
fn timestamp_survives_a_serde_json_value_round_trip() {
    // The property the WS layers could not rely on under uhlc 0.5: an HLC id
    // large enough to need all 16 bytes exceeds `u64`, and `serde_json::Value`
    // has no `u128` variant, so `to_value` failed outright with "number out of
    // range" for *every* real timestamp. That is why the coordinator, daemon
    // and CLI all embed pre-serialized JSON fragments instead of composing
    // `Value`s. With the `[u8; 16]` form the round trip is lossless, so that
    // constraint no longer applies.
    let original = Timestamped {
        inner: "payload".to_string(),
        timestamp: golden_timestamp(),
    };

    let value = serde_json::to_value(&original).expect("Timestamped must survive to_value");
    let decoded: Timestamped<String> = serde_json::from_value(value).expect("from_value");

    assert_eq!(decoded.timestamp, original.timestamp);
    assert_eq!(decoded.inner, original.inner);
}

#[test]
fn a_freshly_generated_hlc_id_uses_all_sixteen_bytes() {
    // Guards the premise of the test above: if `HLC::default()` ever produced a
    // short id, the `serde_json::Value` round trip would pass for reasons that
    // do not generalize to production timestamps.
    let timestamp = dora_message::uhlc::HLC::default().new_timestamp();
    assert_eq!(timestamp.get_id().size(), 16);

    let value = serde_json::to_value(Timestamped {
        inner: (),
        timestamp,
    })
    .expect("real HLC timestamp must survive to_value");
    let decoded: Timestamped<()> = serde_json::from_value(value).expect("from_value");
    assert_eq!(decoded.timestamp, timestamp);
}
