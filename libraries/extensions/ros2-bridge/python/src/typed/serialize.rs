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
                let array: &UInt8Array = self.value.as_primitive();
                debug_assert!(array.len() == 1, "array length was: {}", array.len());
                let number = array.value(0);
                serializer.serialize_u8(number)
            }
            DataType::UInt16 => {
                let array: &UInt16Array = self.value.as_primitive();
                debug_assert!(array.len() == 1);
                let number = array.value(0);
                serializer.serialize_u16(number)
            }
            DataType::UInt32 => {
                let array: &UInt32Array = self.value.as_primitive();
                debug_assert!(array.len() == 1);
                let number = array.value(0);
                serializer.serialize_u32(number)
            }
            DataType::UInt64 => {
                let array: &UInt64Array = self.value.as_primitive();
                debug_assert!(array.len() == 1);
                let number = array.value(0);
                serializer.serialize_u64(number)
            }
            DataType::Int8 => {
                let array: &Int8Array = self.value.as_primitive();
                debug_assert!(array.len() == 1);
                let number = array.value(0);
                serializer.serialize_i8(number)
            }
            DataType::Int16 => {
                let array: &Int16Array = self.value.as_primitive();
                debug_assert!(array.len() == 1);
                let number = array.value(0);
                serializer.serialize_i16(number)
            }
            DataType::Int32 => {
                let array: &Int32Array = self.value.as_primitive();
                debug_assert!(array.len() == 1);
                let number = array.value(0);
                serializer.serialize_i32(number)
            }
            DataType::Int64 => {
                let array: &Int64Array = self.value.as_primitive();
                debug_assert!(array.len() == 1, "array was: {:#?}", array);
                let number = array.value(0);
                serializer.serialize_i64(number)
            }
            DataType::Float32 => {
                let array: &Float32Array = self.value.as_primitive();
                debug_assert!(array.len() == 1);
                let number = array.value(0);
                serializer.serialize_f32(number)
            }
            DataType::Float64 => {
                let array: &Float64Array = self.value.as_primitive();
                debug_assert!(array.len() == 1);
                let number = array.value(0);
                serializer.serialize_f64(number)
            }
            DataType::Utf8 => {
                let int_array: &StringArray = self.value.as_string();
                let string = int_array.value(0);
                serializer.serialize_str(string)
            }
            DataType::List(_field) => {
                let list_array: &ListArray = self.value.as_list();
                let mut s = serializer.serialize_seq(Some(list_array.len()))?;
                for root in list_array.iter() {
                    if let Some(values) = root {
                        match values.data_type() {
                            DataType::UInt8 => {
                                let values: &UInt8Array = values.as_primitive();
                                for value in values.iter() {
                                    if let Some(value) = value {
                                        s.serialize_element(&value)?;
                                    } else {
                                        todo!("Implement null management");
                                    }
                                }
                            }
                            DataType::UInt16 => {
                                let values: &UInt16Array = values.as_primitive();
                                for value in values.iter() {
                                    if let Some(value) = value {
                                        s.serialize_element(&value)?;
                                    } else {
                                        todo!("Implement null management");
                                    }
                                }
                            }
                            DataType::UInt32 => {
                                let values: &UInt32Array = values.as_primitive();
                                for value in values.iter() {
                                    if let Some(value) = value {
                                        s.serialize_element(&value)?;
                                    } else {
                                        todo!("Implement null management");
                                    }
                                }
                            }
                            DataType::UInt64 => {
                                let values: &UInt64Array = values.as_primitive();
                                for value in values.iter() {
                                    if let Some(value) = value {
                                        s.serialize_element(&value)?;
                                    } else {
                                        todo!("Implement null management");
                                    }
                                }
                            }
                            DataType::Int8 => {
                                let values: &Int8Array = values.as_primitive();
                                for value in values.iter() {
                                    if let Some(value) = value {
                                        s.serialize_element(&value)?;
                                    } else {
                                        todo!("Implement null management");
                                    }
                                }
                            }
                            DataType::Int16 => {
                                let values: &Int16Array = values.as_primitive();
                                for value in values.iter() {
                                    if let Some(value) = value {
                                        s.serialize_element(&value)?;
                                    } else {
                                        todo!("Implement null management");
                                    }
                                }
                            }
                            DataType::Int32 => {
                                let values: &Int32Array = values.as_primitive();
                                for value in values.iter() {
                                    if let Some(value) = value {
                                        s.serialize_element(&value)?;
                                    } else {
                                        todo!("Implement null management");
                                    }
                                }
                            }
                            DataType::Int64 => {
                                let values: &Int64Array = values.as_primitive();
                                for value in values.iter() {
                                    if let Some(value) = value {
                                        s.serialize_element(&value)?;
                                    } else {
                                        todo!("Implement null management");
                                    }
                                }
                            }
                            DataType::Float32 => {
                                let values: &Float32Array = values.as_primitive();
                                for value in values.iter() {
                                    if let Some(value) = value {
                                        s.serialize_element(&value)?;
                                    } else {
                                        todo!("Implement null management");
                                    }
                                }
                            }
                            DataType::Float64 => {
                                let values: &Float64Array = values.as_primitive();
                                for value in values.iter() {
                                    if let Some(value) = value {
                                        s.serialize_element(&value)?;
                                    } else {
                                        todo!("Implement null management");
                                    }
                                }
                            }
                            DataType::Utf8 => {
                                let values: &StringArray = values.as_string();
                                for value in values.iter() {
                                    if let Some(value) = value {
                                        s.serialize_element(&value)?;
                                    } else {
                                        todo!("Implement null management");
                                    }
                                }
                            }
                            DataType::Struct(_fields) => {
                                let list_array: ListArray = self.type_info.defaults.clone().into();
                                s.serialize_element(&TypedValue {
                                    value: &values,
                                    type_info: &TypeInfo {
                                        data_type: values.data_type().clone(),
                                        defaults: list_array.value(0).to_data(),
                                    },
                                })?;
                            }
                            op => todo!("Implement additional type: {:?}", op),
                        }
                    } else {
                        todo!("Implement null management");
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
