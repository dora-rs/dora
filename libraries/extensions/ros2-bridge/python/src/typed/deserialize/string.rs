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

impl<'de> serde::de::Visitor<'de> for StringVisitor {
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
