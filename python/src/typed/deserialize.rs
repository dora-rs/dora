use super::{StructField, TypeInfo};
use core::fmt;
use std::ops::Deref;

pub struct Ros2Value(serde_yaml::Value);

impl Deref for Ros2Value {
    type Target = serde_yaml::Value;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct TypedDeserializer {
    type_info: TypeInfo,
}

impl TypedDeserializer {
    pub fn new(type_info: TypeInfo) -> Self {
        Self { type_info }
    }
}

impl<'de> serde::de::DeserializeSeed<'de> for TypedDeserializer {
    type Value = Ros2Value;

    fn deserialize<D>(self, deserializer: D) -> Result<Self::Value, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        let value = match self.type_info {
            TypeInfo::Struct { name: _, fields } => {
                /// Serde requires that struct and field names are known at
                /// compile time with a `'static` lifetime, which is not
                /// possible in this case. Thus, we need to use dummy names
                /// instead.
                ///
                /// The actual names do not really matter because
                /// the CDR format of ROS2 does not encode struct or field
                /// names.
                const DUMMY_STRUCT_NAME: &str = "struct";
                const DUMMY_FIELDS: &[&str] = &[""; 100];

                deserializer.deserialize_struct(
                    DUMMY_STRUCT_NAME,
                    &DUMMY_FIELDS[..fields.len()],
                    StructVisitor { fields },
                )
            }
            TypeInfo::I32 => deserializer.deserialize_i32(PrimitiveValueVisitor),
            TypeInfo::F32 => deserializer.deserialize_f32(PrimitiveValueVisitor),
            TypeInfo::F64 => deserializer.deserialize_f64(PrimitiveValueVisitor),
        }?;
        Ok(Ros2Value(value))
    }
}

/// Based on https://docs.rs/serde_yaml/0.9.22/src/serde_yaml/value/de.rs.html#14-121
struct PrimitiveValueVisitor;

impl<'de> serde::de::Visitor<'de> for PrimitiveValueVisitor {
    type Value = serde_yaml::Value;

    fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
        formatter.write_str("a primitive value")
    }

    fn visit_bool<E>(self, b: bool) -> Result<Self::Value, E>
    where
        E: serde::de::Error,
    {
        Ok(serde_yaml::Value::Bool(b))
    }

    fn visit_i64<E>(self, i: i64) -> Result<Self::Value, E>
    where
        E: serde::de::Error,
    {
        Ok(serde_yaml::Value::Number(i.into()))
    }

    fn visit_u64<E>(self, u: u64) -> Result<Self::Value, E>
    where
        E: serde::de::Error,
    {
        Ok(serde_yaml::Value::Number(u.into()))
    }

    fn visit_f64<E>(self, f: f64) -> Result<Self::Value, E>
    where
        E: serde::de::Error,
    {
        Ok(serde_yaml::Value::Number(f.into()))
    }

    fn visit_str<E>(self, s: &str) -> Result<Self::Value, E>
    where
        E: serde::de::Error,
    {
        Ok(serde_yaml::Value::String(s.to_owned()))
    }

    fn visit_string<E>(self, s: String) -> Result<Self::Value, E>
    where
        E: serde::de::Error,
    {
        Ok(serde_yaml::Value::String(s))
    }

    fn visit_unit<E>(self) -> Result<Self::Value, E>
    where
        E: serde::de::Error,
    {
        Ok(serde_yaml::Value::Null)
    }

    fn visit_none<E>(self) -> Result<Self::Value, E>
    where
        E: serde::de::Error,
    {
        Ok(serde_yaml::Value::Null)
    }
}

struct StructVisitor {
    fields: Vec<StructField>,
}

impl<'de> serde::de::Visitor<'de> for StructVisitor {
    type Value = serde_yaml::Value;

    fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
        formatter.write_str("a struct encoded as sequence")
    }

    fn visit_seq<A>(self, mut data: A) -> Result<Self::Value, A::Error>
    where
        A: serde::de::SeqAccess<'de>,
    {
        let mut mapping = serde_yaml::Mapping::new();
        for field in self.fields {
            let value = match data.next_element_seed(TypedDeserializer {
                type_info: field.ty,
            })? {
                Some(value) => value.0,
                None => match field.default {
                    Some(value) => value,
                    None => {
                        return Err(serde::de::Error::custom(format!(
                            "missing field {}",
                            &field.name
                        )))
                    }
                },
            };
            mapping.insert(serde_yaml::Value::String(field.name), value);
        }

        Ok(serde_yaml::Value::Mapping(mapping))
    }
}
