//! Wire-format contract between `dora-message` and its `uhlc` dependency.
//!
//! `uhlc::Timestamp` is embedded in every postcard-encoded daemon↔node,
//! daemon↔daemon and record-file message (`Metadata`, `Timestamped`), and in
//! every JSON-encoded CLI↔coordinator and coordinator↔daemon WebSocket frame.
//! Its encoding is therefore part of dora's on-the-wire protocol even though the
//! type is owned by an upstream crate — a silent change to it would corrupt HLC
//! clock identities across a mixed-version deployment rather than failing
//! loudly.
//!
//! The golden vectors below pin that encoding so a future `uhlc` bump cannot
//! move it unnoticed. The binary vectors were re-captured when the binary plane
//! moved from bincode to postcard (`Metadata::CURRENT_VERSION` 1 → 2): postcard
//! varint-encodes the `NTP64` instead of writing a fixed little-endian `u64`,
//! while the HLC id stays a fixed 16-byte array. That was the last deliberate
//! change to these bytes — a test failing against them now means a protocol
//! migration is genuinely required, not that the vector needs updating.
//!
//! The JSON plane is unaffected by the postcard move. It last changed shape at
//! `uhlc 0.9.0` (dora-rs/dora#2446), which switched `ID`'s in-memory form from
//! `NonZeroU128` to `[u8; 16]` (see `timestamp_json_encoding_is_pinned`) — that
//! link is version-gated and now survives `serde_json::Value`, which the
//! `NonZeroU128` form could not.

use dora_message::{
    common::Timestamped,
    metadata::Metadata,
    uhlc::{ID, NTP64, Timestamp},
};

/// The pinned postcard encoding of [`golden_timestamp`]: the NTP64 as a
/// 9-byte varint, then the HLC id as its 16 little-endian bytes (a fixed-size
/// array, so postcard writes no length prefix for it).
///
/// If a uhlc bump changes this, every daemon↔node message, every inter-daemon
/// zenoh sample and every `dora record` file written by a different dora version
/// becomes unreadable — so a test failing against it means a protocol migration
/// is genuinely required, not that the vector needs updating.
const TIMESTAMP_HEX: &str = "88ef99abc5e88c91110102030405060708090a0b0c0d0e0f10";

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
fn timestamp_postcard_encoding_is_pinned() {
    let encoded = postcard::to_stdvec(&golden_timestamp()).expect("serialize timestamp");
    assert_eq!(
        hex(&encoded),
        TIMESTAMP_HEX,
        "uhlc::Timestamp postcard encoding changed — this is a protocol break"
    );
}

#[test]
fn timestamp_postcard_round_trips() {
    let original = golden_timestamp();
    let encoded = postcard::to_stdvec(&original).expect("serialize");
    let decoded: Timestamp = postcard::from_bytes(&encoded).expect("deserialize");

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
fn metadata_postcard_encoding_is_pinned() {
    // `Metadata` is the envelope that actually rides on every output message,
    // so pin the composite too: `metadata_version` as a varint u16, the 25-byte
    // timestamp, then the parameter map length as a varint.
    let encoded = postcard::to_stdvec(&Metadata::new(golden_timestamp())).expect("serialize");
    assert_eq!(
        hex(&encoded),
        format!(
            "{version}{TIMESTAMP_HEX}{empty_parameter_map}",
            version = "02",
            empty_parameter_map = "00",
        ),
        "Metadata postcard encoding changed — bump Metadata::CURRENT_VERSION"
    );
}

#[test]
fn timestamped_postcard_prefix_is_the_inner_value() {
    // `Timestamped<T>` puts `inner` first and the timestamp last, so the
    // timestamp's 25 bytes are the tail of every daemon↔coordinator frame.
    let timestamped = Timestamped {
        inner: 0xABu8,
        timestamp: golden_timestamp(),
    };
    let encoded = postcard::to_stdvec(&timestamped).expect("serialize");
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
fn a_freshly_generated_hlc_timestamp_survives_a_serde_json_value_round_trip() {
    // Companion to `timestamp_survives_a_serde_json_value_round_trip`, which pins
    // the property on the deterministic full-width `golden_timestamp` (top byte
    // `0x10`, so it exceeds `u64`): confirm a *real* `HLC::default()` timestamp
    // also survives `serde_json::Value`.
    //
    // We deliberately do NOT assert that the id uses all sixteen bytes. Under
    // uhlc 0.9 `HLC::default()` seeds its id with a uniformly random 128-bit
    // value (`ID::rand()`), whose most-significant byte is zero ~1/256 of the
    // time, so `size() == 16` is probabilistic and would flake ~1 run in 256
    // (dora-rs/dora#3104). The round trip below is lossless for an id of any
    // width — that, not the byte count of a random id, is the property that
    // matters here.
    let timestamp = dora_message::uhlc::HLC::default().new_timestamp();

    let value = serde_json::to_value(Timestamped {
        inner: (),
        timestamp,
    })
    .expect("real HLC timestamp must survive to_value");
    let decoded: Timestamped<()> = serde_json::from_value(value).expect("from_value");
    assert_eq!(decoded.timestamp, timestamp);
}
