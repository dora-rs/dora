use arrow::array::make_array;
use arrow::array::Array;
use arrow::array::ArrayRef;
use arrow::array::AsArray;
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
    pub value: &'a ArrayRef,
    pub type_info: &'a TypeInfo,
}

impl serde::Serialize for TypedValue<'_> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        match &self.type_info.data_type {
            DataType::UInt8 => {
                let uint_array: &UInt8Array = self.value.as_primitive();
                if uint_array.len() == 1 {
                    let number = uint_array.value(0);
                    serializer.serialize_u8(number)
                } else {
                    let mut s = serializer.serialize_seq(Some(uint_array.len()))?;
                    for value in uint_array.iter() {
                        s.serialize_element(&value)?;
                    }
                    s.end()
                }
            }
            DataType::UInt16 => {
                let uint_array: &UInt16Array = self.value.as_primitive();
                if uint_array.len() == 1 {
                    let number = uint_array.value(0);
                    serializer.serialize_u16(number)
                } else {
                    let mut s = serializer.serialize_seq(Some(uint_array.len()))?;
                    for value in uint_array.iter() {
                        s.serialize_element(&value)?;
                    }
                    s.end()
                }
            }
            DataType::UInt32 => {
                let array: &UInt32Array = self.value.as_primitive();
                if array.len() == 1 {
                    let number = array.value(0);
                    serializer.serialize_u32(number)
                } else {
                    let mut s = serializer.serialize_seq(Some(array.len()))?;
                    for value in array.iter() {
                        s.serialize_element(&value)?;
                    }
                    s.end()
                }
            }
            DataType::UInt64 => {
                let array: &UInt64Array = self.value.as_primitive();
                if array.len() == 1 {
                    let number = array.value(0);
                    serializer.serialize_u64(number)
                } else {
                    let mut s = serializer.serialize_seq(Some(array.len()))?;
                    for value in array.iter() {
                        s.serialize_element(&value)?;
                    }
                    s.end()
                }
            }
            DataType::Int8 => {
                let array: &Int8Array = self.value.as_primitive();
                if array.len() == 1 {
                    let number = array.value(0);
                    serializer.serialize_i8(number)
                } else {
                    let mut s = serializer.serialize_seq(Some(array.len()))?;
                    for value in array.iter() {
                        s.serialize_element(&value)?;
                    }
                    s.end()
                }
            }
            DataType::Int16 => {
                let array: &Int16Array = self.value.as_primitive();
                if array.len() == 1 {
                    let number = array.value(0);
                    serializer.serialize_i16(number)
                } else {
                    let mut s = serializer.serialize_seq(Some(array.len()))?;
                    for value in array.iter() {
                        s.serialize_element(&value)?;
                    }
                    s.end()
                }
            }
            DataType::Int32 => {
                let array: &Int32Array = self.value.as_primitive();
                if array.len() == 1 {
                    let number = array.value(0);
                    serializer.serialize_i32(number)
                } else {
                    let mut s = serializer.serialize_seq(Some(array.len()))?;
                    for value in array.iter() {
                        s.serialize_element(&value)?;
                    }
                    s.end()
                }
            }
            DataType::Int64 => {
                let array: &Int64Array = self.value.as_primitive();
                if array.len() == 1 {
                    let number = array.value(0);
                    serializer.serialize_i64(number)
                } else {
                    let mut s = serializer.serialize_seq(Some(array.len()))?;
                    for value in array.iter() {
                        s.serialize_element(&value)?;
                    }
                    s.end()
                }
            }
            DataType::Float32 => {
                let array: &Float32Array = self.value.as_primitive();
                if array.len() == 1 {
                    let number = array.value(0);
                    serializer.serialize_f32(number)
                } else {
                    let mut s = serializer.serialize_seq(Some(array.len()))?;
                    for value in array.iter() {
                        s.serialize_element(&value)?;
                    }
                    s.end()
                }
            }
            DataType::Float64 => {
                let array: &Float64Array = self.value.as_primitive();
                if array.len() == 1 {
                    let number = array.value(0);
                    serializer.serialize_f64(number)
                } else {
                    let mut s = serializer.serialize_seq(Some(array.len()))?;
                    for value in array.iter() {
                        s.serialize_element(&value)?;
                    }
                    s.end()
                }
            }
            DataType::Utf8 => {
                let int_array: &StringArray = self.value.as_string();
                let string = int_array.value(0);
                serializer.serialize_str(string)
            }
            DataType::List(field) => {
                let list_array: &ListArray = self.value.as_list();
                // let values: &UInt16Array = list_array.values().as_primitive();
                let mut s = serializer.serialize_seq(Some(list_array.len()))?;
                for value in list_array.iter() {
                    if let Some(value) = value {
                        s.serialize_element(&TypedValue {
                            value: &value,
                            type_info: &TypeInfo {
                                data_type: field.data_type().clone(),
                                defaults: self.type_info.defaults.clone(),
                            },
                        })?;
                    }
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

                let struct_array: &StructArray = self.value.as_struct();
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
                    let value = make_array(default.clone());
                    let field_value = match struct_array.column_by_name(field.name()) {
                        Some(value) => value,
                        None => &value,
                    };

                    s.serialize_field(
                        DUMMY_FIELD_NAME,
                        &TypedValue {
                            value: &field_value.clone(),
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
