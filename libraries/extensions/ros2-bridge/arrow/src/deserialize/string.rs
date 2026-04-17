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
            u16_buf.push(value);
        }
        let utf8 = String::from_utf16(&u16_buf).map_err(serde::de::Error::custom)?;
        let mut array = StringBuilder::new();
        array.append_value(utf8);
        Ok(array.finish().into())
    }
}
