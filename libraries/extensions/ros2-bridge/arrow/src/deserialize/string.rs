use arrow::array::{ArrayData, StringBuilder};
use core::fmt;

pub struct StringDeserializer;

impl<'de> serde::de::DeserializeSeed<'de> for StringDeserializer {
    type Value = ArrayData;

    fn deserialize<D>(self, deserializer: D) -> Result<Self::Value, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        deserializer.deserialize_str(StringVisitor)
    }
}

/// Based on https://docs.rs/serde_yaml/0.9.22/src/serde_yaml/value/de.rs.html#14-121
struct StringVisitor;

impl serde::de::Visitor<'_> for StringVisitor {
    type Value = ArrayData;

    fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
        formatter.write_str("a string value")
    }

    fn visit_str<E>(self, s: &str) -> Result<Self::Value, E>
    where
        E: serde::de::Error,
    {
        let mut array = StringBuilder::new();
        array.append_value(s);
        Ok(array.finish().into())
    }

    fn visit_string<E>(self, s: String) -> Result<Self::Value, E>
    where
        E: serde::de::Error,
    {
        let mut array = StringBuilder::new();
        array.append_value(s);
        Ok(array.finish().into())
    }
}

/// Deserializer for ROS2 WString (UTF-16) types.
///
/// WStrings are serialized as `Vec<u16>` in CDR. We deserialize the u16
/// sequence, convert to UTF-8, and store as a regular Arrow StringArray.
pub struct WStringDeserializer;

impl<'de> serde::de::DeserializeSeed<'de> for WStringDeserializer {
    type Value = ArrayData;

    fn deserialize<D>(self, deserializer: D) -> Result<Self::Value, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        deserializer.deserialize_seq(WStringVisitor)
    }
}

struct WStringVisitor;

impl<'de> serde::de::Visitor<'de> for WStringVisitor {
    type Value = ArrayData;

    fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
        formatter.write_str("a sequence of u16 values (UTF-16 wstring)")
    }

    fn visit_seq<A>(self, mut seq: A) -> Result<Self::Value, A::Error>
    where
        A: serde::de::SeqAccess<'de>,
    {
        let mut u16_buf: Vec<u16> = Vec::new();
        while let Some(value) = seq.next_element::<u16>()? {
            // Bound the element count so a malformed CDR length prefix cannot
            // drive an unbounded allocation. Mirrors every sequence path in
            // `sequence.rs`, which the single-`wstring` field path previously
            // omitted.
            if u16_buf.len() >= super::sequence::MAX_SEQUENCE_ELEMENTS {
                return Err(serde::de::Error::custom(format!(
                    "wstring exceeds maximum of {} code units",
                    super::sequence::MAX_SEQUENCE_ELEMENTS
                )));
            }
            u16_buf.push(value);
        }
        let utf8 = String::from_utf16(&u16_buf).map_err(serde::de::Error::custom)?;
        let mut array = StringBuilder::new();
        array.append_value(utf8);
        Ok(array.finish().into())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use arrow::array::{Array, StringArray};
    use serde::de::value::{Error as ValueError, U16Deserializer};
    use serde::de::{DeserializeSeed, SeqAccess, Visitor};

    /// A minimal, lazy `SeqAccess` yielding `u16` values from an iterator, so
    /// the oversized case doesn't need to materialize a huge buffer up front.
    struct U16Seq<I>(I);

    impl<'de, I: Iterator<Item = u16>> SeqAccess<'de> for U16Seq<I> {
        type Error = ValueError;

        fn next_element_seed<T>(&mut self, seed: T) -> Result<Option<T::Value>, Self::Error>
        where
            T: DeserializeSeed<'de>,
        {
            match self.0.next() {
                Some(v) => seed.deserialize(U16Deserializer::new(v)).map(Some),
                None => Ok(None),
            }
        }
    }

    #[test]
    fn wstring_roundtrips() {
        // UTF-16 code units for "hi".
        let seq = U16Seq([104u16, 105].into_iter());
        let data = WStringVisitor.visit_seq(seq).expect("valid wstring");
        let array = StringArray::from(data);
        assert_eq!(array.len(), 1);
        assert_eq!(array.value(0), "hi");
    }

    #[test]
    fn wstring_rejects_oversized() {
        use super::super::sequence::MAX_SEQUENCE_ELEMENTS;
        // One code unit past the cap; the iterator is lazy so nothing is
        // allocated beyond the cap before the guard fires.
        let seq = U16Seq(std::iter::repeat_n(b'a' as u16, MAX_SEQUENCE_ELEMENTS + 1));
        let err = WStringVisitor
            .visit_seq(seq)
            .expect_err("wstring past the element cap must be rejected");
        assert!(
            err.to_string().contains("maximum"),
            "unexpected error: {err}"
        );
    }
}
