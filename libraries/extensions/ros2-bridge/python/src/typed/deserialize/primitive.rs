use arrow::array::{
    ArrayData, BooleanBuilder, Float32Builder, Float64Builder, Int16Builder, Int32Builder,
    Int64Builder, Int8Builder, NullArray, UInt16Builder, UInt32Builder, UInt64Builder,
    UInt8Builder,
};
use core::fmt;
use dora_ros2_bridge_msg_gen::types::primitives::BasicType;

pub struct PrimitiveDeserializer<'a>(pub &'a BasicType);

impl<'de> serde::de::DeserializeSeed<'de> for PrimitiveDeserializer<'_> {
    type Value = ArrayData;

    fn deserialize<D>(self, deserializer: D) -> Result<Self::Value, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        match self.0 {
            BasicType::I8 => deserializer.deserialize_i8(PrimitiveValueVisitor),
            BasicType::I16 => deserializer.deserialize_i16(PrimitiveValueVisitor),
            BasicType::I32 => deserializer.deserialize_i32(PrimitiveValueVisitor),
            BasicType::I64 => deserializer.deserialize_i64(PrimitiveValueVisitor),
            BasicType::U8 | BasicType::Char | BasicType::Byte => {
                deserializer.deserialize_u8(PrimitiveValueVisitor)
            }
            BasicType::U16 => deserializer.deserialize_u16(PrimitiveValueVisitor),
            BasicType::U32 => deserializer.deserialize_u32(PrimitiveValueVisitor),
            BasicType::U64 => deserializer.deserialize_u64(PrimitiveValueVisitor),
            BasicType::F32 => deserializer.deserialize_f32(PrimitiveValueVisitor),
            BasicType::F64 => deserializer.deserialize_f64(PrimitiveValueVisitor),
            BasicType::Bool => deserializer.deserialize_bool(PrimitiveValueVisitor),
        }
    }
}

/// Based on https://docs.rs/serde_yaml/0.9.22/src/serde_yaml/value/de.rs.html#14-121
struct PrimitiveValueVisitor;

impl<'de> serde::de::Visitor<'de> for PrimitiveValueVisitor {
    type Value = ArrayData;

    fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
        formatter.write_str("a primitive value")
    }

    fn visit_bool<E>(self, b: bool) -> Result<Self::Value, E>
    where
        E: serde::de::Error,
    {
        let mut array = BooleanBuilder::new();
        array.append_value(b);
        Ok(array.finish().into())
    }

    fn visit_i8<E>(self, u: i8) -> Result<Self::Value, E>
    where
        E: serde::de::Error,
    {
        let mut array = Int8Builder::new();
        array.append_value(u);
        Ok(array.finish().into())
    }

    fn visit_i16<E>(self, u: i16) -> Result<Self::Value, E>
    where
        E: serde::de::Error,
    {
        let mut array = Int16Builder::new();
        array.append_value(u);
        Ok(array.finish().into())
    }
    fn visit_i32<E>(self, u: i32) -> Result<Self::Value, E>
    where
        E: serde::de::Error,
    {
        let mut array = Int32Builder::new();
        array.append_value(u);
        Ok(array.finish().into())
    }
    fn visit_i64<E>(self, i: i64) -> Result<Self::Value, E>
    where
        E: serde::de::Error,
    {
        let mut array = Int64Builder::new();
        array.append_value(i);
        Ok(array.finish().into())
    }

    fn visit_u8<E>(self, u: u8) -> Result<Self::Value, E>
    where
        E: serde::de::Error,
    {
        let mut array = UInt8Builder::new();
        array.append_value(u);
        Ok(array.finish().into())
    }
    fn visit_u16<E>(self, u: u16) -> Result<Self::Value, E>
    where
        E: serde::de::Error,
    {
        let mut array = UInt16Builder::new();
        array.append_value(u);
        Ok(array.finish().into())
    }
    fn visit_u32<E>(self, u: u32) -> Result<Self::Value, E>
    where
        E: serde::de::Error,
    {
        let mut array = UInt32Builder::new();
        array.append_value(u);
        Ok(array.finish().into())
    }
    fn visit_u64<E>(self, u: u64) -> Result<Self::Value, E>
    where
        E: serde::de::Error,
    {
        let mut array = UInt64Builder::new();
        array.append_value(u);
        Ok(array.finish().into())
    }

    fn visit_f32<E>(self, f: f32) -> Result<Self::Value, E>
    where
        E: serde::de::Error,
    {
        let mut array = Float32Builder::new();
        array.append_value(f);
        Ok(array.finish().into())
    }

    fn visit_f64<E>(self, f: f64) -> Result<Self::Value, E>
    where
        E: serde::de::Error,
    {
        let mut array = Float64Builder::new();
        array.append_value(f);
        Ok(array.finish().into())
    }

    fn visit_unit<E>(self) -> Result<Self::Value, E>
    where
        E: serde::de::Error,
    {
        let array = NullArray::new(0);
        Ok(array.into())
    }

    fn visit_none<E>(self) -> Result<Self::Value, E>
    where
        E: serde::de::Error,
    {
        let array = NullArray::new(0);
        Ok(array.into())
    }
}
