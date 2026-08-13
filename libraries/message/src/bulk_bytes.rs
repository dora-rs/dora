//! Bulk-payload serde for `AVec<u8, ConstAlign<128>>`.
//!
//! serde's default impl for a byte container is a *sequence* of `u8`, so the
//! encoder writes the payload one element at a time. Routing it through
//! `serialize_bytes`/`deserialize_bytes` instead lets the codec move the whole
//! slice at once, which is 50–65x faster on payload-sized buffers.
//!
//! **The encoding is unchanged.** postcard writes a varint length followed by
//! the raw bytes either way, so this is a pure speedup — no wire-format version
//! bump. `encoding_is_unchanged_from_the_seq_form` pins that, including at the
//! varint length-prefix boundaries. In self-describing formats the two forms
//! also agree: serde_json renders both as an array of numbers, and the visitor
//! below accepts that shape back via [`Visitor::visit_seq`].
//!
//! Deserialization preserves the 128-byte alignment the Arrow zero-copy decode
//! path depends on — see `daemon_path_ipc_roundtrip_preserves_payload_and_alignment`.

use std::fmt;

use aligned_vec::{AVec, ConstAlign};
use serde::{
    Deserializer, Serializer,
    de::{self, Visitor},
};

/// The alignment required by the Arrow zero-copy decode path.
const ALIGN: usize = 128;

type Payload = AVec<u8, ConstAlign<ALIGN>>;

pub fn serialize<S: Serializer>(value: &Payload, serializer: S) -> Result<S::Ok, S::Error> {
    serializer.serialize_bytes(value)
}

pub fn deserialize<'de, D: Deserializer<'de>>(deserializer: D) -> Result<Payload, D::Error> {
    deserializer.deserialize_bytes(PayloadVisitor)
}

struct PayloadVisitor;

impl<'de> Visitor<'de> for PayloadVisitor {
    type Value = Payload;

    fn expecting(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("a byte array")
    }

    fn visit_bytes<E: de::Error>(self, v: &[u8]) -> Result<Self::Value, E> {
        Ok(AVec::from_slice(ALIGN, v))
    }

    fn visit_borrowed_bytes<E: de::Error>(self, v: &'de [u8]) -> Result<Self::Value, E> {
        Ok(AVec::from_slice(ALIGN, v))
    }

    fn visit_byte_buf<E: de::Error>(self, v: Vec<u8>) -> Result<Self::Value, E> {
        Ok(AVec::from_slice(ALIGN, &v))
    }

    /// Self-describing formats (notably JSON) render bytes as a sequence of
    /// numbers and hand that back here rather than to `visit_bytes`.
    fn visit_seq<A: de::SeqAccess<'de>>(self, mut seq: A) -> Result<Self::Value, A::Error> {
        let mut out = AVec::with_capacity(ALIGN, seq.size_hint().unwrap_or(0));
        while let Some(byte) = seq.next_element::<u8>()? {
            out.push(byte);
        }
        Ok(out)
    }
}

/// The same treatment for an `Option<AVec<..>>` field.
pub mod option {
    use super::{Payload, PayloadVisitor};
    use serde::{
        Deserializer, Serializer,
        de::{self, Visitor},
    };
    use std::fmt;

    pub fn serialize<S: Serializer>(
        value: &Option<Payload>,
        serializer: S,
    ) -> Result<S::Ok, S::Error> {
        match value {
            Some(v) => serializer.serialize_some(&Wrapper(v)),
            None => serializer.serialize_none(),
        }
    }

    /// Applies [`super::serialize`] to the inner value; `serialize_some` needs
    /// something implementing `Serialize`, and `#[serde(with)]` gives us a
    /// free function rather than an impl.
    struct Wrapper<'a>(&'a Payload);

    impl serde::Serialize for Wrapper<'_> {
        fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
            super::serialize(self.0, serializer)
        }
    }

    pub fn deserialize<'de, D: Deserializer<'de>>(
        deserializer: D,
    ) -> Result<Option<Payload>, D::Error> {
        deserializer.deserialize_option(OptionVisitor)
    }

    struct OptionVisitor;

    impl<'de> Visitor<'de> for OptionVisitor {
        type Value = Option<Payload>;

        fn expecting(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
            f.write_str("an optional byte array")
        }

        fn visit_none<E: de::Error>(self) -> Result<Self::Value, E> {
            Ok(None)
        }

        fn visit_unit<E: de::Error>(self) -> Result<Self::Value, E> {
            Ok(None)
        }

        fn visit_some<D: Deserializer<'de>>(self, d: D) -> Result<Self::Value, D::Error> {
            d.deserialize_bytes(PayloadVisitor).map(Some)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde::{Deserialize, Serialize};

    /// Mirrors `DataMessage`'s shape but keeps serde's default (sequence)
    /// encoding, so the two can be compared byte for byte.
    #[derive(Serialize, Deserialize)]
    struct SeqForm(Payload);

    #[derive(Serialize, Deserialize)]
    struct BytesForm(#[serde(with = "super")] Payload);

    fn payload(len: usize) -> Payload {
        let data: Vec<u8> = (0..len).map(|i| (i % 251) as u8).collect();
        AVec::from_slice(ALIGN, &data)
    }

    /// The whole premise of this module: swapping the element loop for a bulk
    /// copy must not move a single byte on the wire. Lengths straddle postcard's
    /// varint length-prefix boundary at 127/128, where a naive change would
    /// shift the prefix width.
    #[test]
    fn encoding_is_unchanged_from_the_seq_form() {
        for len in [0, 1, 127, 128, 129, 300, 4096, 70_000] {
            let value = payload(len);
            assert_eq!(
                postcard::to_stdvec(&SeqForm(value.clone())).expect("seq"),
                postcard::to_stdvec(&BytesForm(value)).expect("bytes"),
                "len {len}: bulk encoding differs from the sequence encoding — \
                 this would be an unversioned wire break"
            );
        }
    }

    #[test]
    fn round_trips_and_preserves_alignment() {
        for len in [0, 1, 128, 4096] {
            let value = payload(len);
            let bytes = postcard::to_stdvec(&BytesForm(value.clone())).expect("serialize");
            let back: BytesForm = postcard::from_bytes(&bytes).expect("deserialize");

            assert_eq!(&back.0[..], &value[..], "len {len}: payload changed");
            assert_eq!(
                back.0.as_ptr() as usize % ALIGN,
                0,
                "len {len}: decoded payload must stay {ALIGN}-byte aligned for \
                 the Arrow zero-copy path"
            );
        }
    }

    /// JSON hands bytes back as a sequence, so the visitor must accept that
    /// shape too — `NodeConfig` and the WS planes travel as JSON.
    #[test]
    fn survives_a_json_round_trip() {
        let value = payload(300);
        let json = serde_json::to_string(&BytesForm(value.clone())).expect("to json");
        let back: BytesForm = serde_json::from_str(&json).expect("from json");
        assert_eq!(&back.0[..], &value[..]);

        // And the JSON text itself is unchanged from the sequence form.
        assert_eq!(
            json,
            serde_json::to_string(&SeqForm(value)).expect("seq to json")
        );
    }

    #[test]
    fn option_round_trips_in_both_states() {
        #[derive(Serialize, Deserialize)]
        struct OptForm(#[serde(with = "super::option")] Option<Payload>);
        #[derive(Serialize, Deserialize)]
        struct OptSeqForm(Option<Payload>);

        for value in [None, Some(payload(0)), Some(payload(300))] {
            let bytes = postcard::to_stdvec(&OptForm(value.clone())).expect("serialize");
            assert_eq!(
                bytes,
                postcard::to_stdvec(&OptSeqForm(value.clone())).expect("seq"),
                "option encoding differs from the sequence encoding"
            );

            let back: OptForm = postcard::from_bytes(&bytes).expect("deserialize");
            match (&back.0, &value) {
                (None, None) => {}
                (Some(a), Some(b)) => assert_eq!(&a[..], &b[..]),
                _ => panic!("option state changed across the round trip"),
            }
        }
    }
}
