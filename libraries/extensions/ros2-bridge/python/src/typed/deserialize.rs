use super::TypeInfo;
use arrow::{
    array::{
        make_array, Array, ArrayData, BooleanBuilder, Float32Builder, Float64Builder, Int16Builder,
        Int32Builder, Int64Builder, Int8Builder, ListArray, NullArray, StringBuilder, StructArray,
        UInt16Builder, UInt32Builder, UInt64Builder, UInt8Builder,
    },
    buffer::OffsetBuffer,
    compute::concat,
    datatypes::{DataType, Field, Fields},
};
use core::fmt;
use std::sync::Arc;
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
    type Value = ArrayData;

    fn deserialize<D>(self, deserializer: D) -> Result<Self::Value, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        let data_type = self.type_info.data_type;
        let value = match data_type.clone() {
            DataType::Struct(fields) => {
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
                    StructVisitor {
                        fields,
                        defaults: self.type_info.defaults,
                    },
                )
            }
            DataType::List(field) => deserializer.deserialize_seq(ListVisitor {
                field,
                defaults: self.type_info.defaults,
            }),
            DataType::UInt8 => deserializer.deserialize_u8(PrimitiveValueVisitor),
            DataType::UInt16 => deserializer.deserialize_u16(PrimitiveValueVisitor),
            DataType::UInt32 => deserializer.deserialize_u32(PrimitiveValueVisitor),
            DataType::UInt64 => deserializer.deserialize_u64(PrimitiveValueVisitor),
            DataType::Int8 => deserializer.deserialize_i8(PrimitiveValueVisitor),
            DataType::Int16 => deserializer.deserialize_i16(PrimitiveValueVisitor),
            DataType::Int32 => deserializer.deserialize_i32(PrimitiveValueVisitor),
            DataType::Int64 => deserializer.deserialize_i64(PrimitiveValueVisitor),
            DataType::Float32 => deserializer.deserialize_f32(PrimitiveValueVisitor),
            DataType::Float64 => deserializer.deserialize_f64(PrimitiveValueVisitor),
            DataType::Utf8 => deserializer.deserialize_str(PrimitiveValueVisitor),
            _ => todo!(),
        }?;

        debug_assert!(
        value.data_type() == &data_type,
        "Datatype does not correspond to default data type.\n Expected: {:#?} \n but got: {:#?}, with value: {:#?}", data_type, value.data_type(), value
        );

        Ok(value)
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

struct StructVisitor {
    fields: Fields,
    defaults: ArrayData,
}

impl<'de> serde::de::Visitor<'de> for StructVisitor {
    type Value = ArrayData;

    fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
        formatter.write_str("a struct encoded as sequence")
    }

    fn visit_seq<A>(self, mut data: A) -> Result<Self::Value, A::Error>
    where
        A: serde::de::SeqAccess<'de>,
    {
        let mut fields = vec![];
        let defaults: StructArray = self.defaults.clone().into();
        for field in self.fields.iter() {
            let default = match defaults.column_by_name(field.name()) {
                Some(value) => value.clone(),
                None => {
                    return Err(serde::de::Error::custom(format!(
                        "missing field {} for deserialization",
                        &field.name()
                    )))
                }
            };
            let value = match data.next_element_seed(TypedDeserializer {
                type_info: TypeInfo {
                    data_type: field.data_type().clone(),
                    defaults: default.to_data(),
                },
            })? {
                Some(value) => make_array(value),
                None => default,
            };
            fields.push((
                // Recreate a new field as List(UInt8) can be converted to UInt8
                Arc::new(Field::new(field.name(), value.data_type().clone(), true)),
                value,
            ));
        }

        let struct_array: StructArray = fields.into();

        Ok(struct_array.into())
    }
}

struct ListVisitor {
    field: Arc<Field>,
    defaults: ArrayData,
}

impl<'de> serde::de::Visitor<'de> for ListVisitor {
    type Value = ArrayData;

    fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
        formatter.write_str("an array encoded as sequence")
    }

    fn visit_seq<A>(self, mut data: A) -> Result<Self::Value, A::Error>
    where
        A: serde::de::SeqAccess<'de>,
    {
        let data = match self.field.data_type().clone() {
            DataType::UInt8 => {
                let mut array = UInt8Builder::new();
                while let Some(value) = data.next_element::<u8>()? {
                    array.append_value(value);
                }
                Ok(array.finish().into())
            }
            DataType::UInt16 => {
                let mut array = UInt16Builder::new();
                while let Some(value) = data.next_element::<u16>()? {
                    array.append_value(value);
                }
                Ok(array.finish().into())
            }
            DataType::UInt32 => {
                let mut array = UInt32Builder::new();
                while let Some(value) = data.next_element::<u32>()? {
                    array.append_value(value);
                }
                Ok(array.finish().into())
            }
            DataType::UInt64 => {
                let mut array = UInt64Builder::new();
                while let Some(value) = data.next_element::<u64>()? {
                    array.append_value(value);
                }
                Ok(array.finish().into())
            }
            DataType::Int8 => {
                let mut array = Int8Builder::new();
                while let Some(value) = data.next_element::<i8>()? {
                    array.append_value(value);
                }
                Ok(array.finish().into())
            }
            DataType::Int16 => {
                let mut array = Int16Builder::new();
                while let Some(value) = data.next_element::<i16>()? {
                    array.append_value(value);
                }
                Ok(array.finish().into())
            }
            DataType::Int32 => {
                let mut array = Int32Builder::new();
                while let Some(value) = data.next_element::<i32>()? {
                    array.append_value(value);
                }
                Ok(array.finish().into())
            }
            DataType::Int64 => {
                let mut array = Int64Builder::new();
                while let Some(value) = data.next_element::<i64>()? {
                    array.append_value(value);
                }
                Ok(array.finish().into())
            }
            DataType::Float32 => {
                let mut array = Float32Builder::new();
                while let Some(value) = data.next_element::<f32>()? {
                    array.append_value(value);
                }
                Ok(array.finish().into())
            }
            DataType::Float64 => {
                let mut array = Float64Builder::new();
                while let Some(value) = data.next_element::<f64>()? {
                    array.append_value(value);
                }
                Ok(array.finish().into())
            }
            DataType::Utf8 => {
                let mut array = StringBuilder::new();
                while let Some(value) = data.next_element::<String>()? {
                    array.append_value(value);
                }
                Ok(array.finish().into())
            }
            _ => {
                let mut buffer = vec![];
                while let Some(value) = data.next_element_seed(TypedDeserializer {
                    type_info: TypeInfo {
                        data_type: self.field.data_type().clone(),
                        defaults: self.defaults.clone(),
                    },
                })? {
                    let element = make_array(value);
                    buffer.push(element);
                }

                concat(
                    buffer
                        .iter()
                        .map(|data| data.as_ref())
                        .collect::<Vec<_>>()
                        .as_slice(),
                )
                .map(|op| op.to_data())
            }
        };

        if let Ok(values) = data {
            let offsets = OffsetBuffer::new(vec![0, values.len() as i32].into());

            let array = ListArray::new(self.field, offsets, make_array(values), None).to_data();
            Ok(array)
        } else {
            Ok(self.defaults) // TODO: Better handle deserialization error
        }
    }
}
