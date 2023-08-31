use arrow::array::ArrayData;
use arrow::array::Float32Array;
use arrow::array::Float64Array;
use arrow::array::Int16Array;
use arrow::array::Int32Array;
use arrow::array::Int64Array;
use arrow::array::Int8Array;
use arrow::array::ListArray;
use arrow::array::StringArray;
use arrow::array::StructArray;
use arrow::array::UInt16Array;
use arrow::array::UInt32Array;
use arrow::array::UInt64Array;
use arrow::array::UInt8Array;
use arrow::datatypes::DataType;
use serde::ser::SerializeSeq;
use serde::ser::SerializeStruct;

use super::TypeInfo;

#[derive(Debug, Clone, PartialEq)]
pub struct TypedValue<'a> {
    pub value: &'a ArrayData,
    pub type_info: &'a TypeInfo,
}

impl serde::Serialize for TypedValue<'_> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        match &self.type_info.data_type {
            DataType::UInt8 => {
                let uint_array: UInt8Array = self.value.clone().into();
                let number = uint_array.value(0);
                serializer.serialize_u8(number)
            }
            DataType::UInt16 => {
                let uint_array: UInt16Array = self.value.clone().into();
                let number = uint_array.value(0);
                serializer.serialize_u16(number)
            }
            DataType::UInt32 => {
                let uint_array: UInt32Array = self.value.clone().into();
                let number = uint_array.value(0);
                serializer.serialize_u32(number)
            }
            DataType::UInt64 => {
                let uint_array: UInt64Array = self.value.clone().into();
                let number = uint_array.value(0);
                serializer.serialize_u64(number)
            }
            DataType::Int8 => {
                let int_array: Int8Array = self.value.clone().into();
                let number = int_array.value(0);
                serializer.serialize_i8(number)
            }
            DataType::Int16 => {
                let int_array: Int16Array = self.value.clone().into();
                let number = int_array.value(0);
                serializer.serialize_i16(number)
            }
            DataType::Int32 => {
                let int_array: Int32Array = self.value.clone().into();
                let number = int_array.value(0);
                serializer.serialize_i32(number)
            }
            DataType::Int64 => {
                let int_array: Int64Array = self.value.clone().into();
                let number = int_array.value(0);
                serializer.serialize_i64(number)
            }
            DataType::Float32 => {
                let int_array: Float32Array = self.value.clone().into();
                let number = int_array.value(0);
                serializer.serialize_f32(number)
            }
            DataType::Float64 => {
                let int_array: Float64Array = self.value.clone().into();
                let number = int_array.value(0);
                serializer.serialize_f64(number)
            }
            DataType::Utf8 => {
                let int_array: StringArray = self.value.clone().into();
                let string = int_array.value(0);
                serializer.serialize_str(string)
            }
            DataType::List(field) => {
                let list_array: ListArray = self.value.clone().into();
                let values = list_array.values();
                let mut s = serializer.serialize_seq(Some(values.len()))?;
                for value in list_array.iter() {
                    let value = match value {
                        Some(value) => value.to_data(),
                        None => {
                            return Err(serde::ser::Error::custom(
                                "Value in ListArray is null and not yet supported".to_string(),
                            ))
                        }
                    };

                    s.serialize_element(&TypedValue {
                        value: &value,
                        type_info: &TypeInfo {
                            data_type: field.data_type().clone(),
                            defaults: self.type_info.defaults.clone(),
                        },
                    })?;
                }
                s.end()
            }
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
                const DUMMY_FIELD_NAME: &str = "field";

                let struct_array: StructArray = self.value.clone().into();
                let mut s = serializer.serialize_struct(DUMMY_STRUCT_NAME, fields.len())?;
                let defaults: StructArray = self.type_info.defaults.clone().into();
                for field in fields.iter() {
                    let default = match defaults.column_by_name(field.name()) {
                        Some(value) => value.to_data(),
                        None => {
                            return Err(serde::ser::Error::custom(format!(
                                "missing field {} for serialization",
                                &field.name()
                            )))
                        }
                    };
                    let field_value = match struct_array.column_by_name(field.name()) {
                        Some(value) => value.to_data(),
                        None => default.clone(),
                    };

                    s.serialize_field(
                        DUMMY_FIELD_NAME,
                        &TypedValue {
                            value: &field_value,
                            type_info: &TypeInfo {
                                data_type: field.data_type().clone(),
                                defaults: default,
                            },
                        },
                    )?;
                }
                s.end()
            }
            _ => todo!(),
        }
    }
}
