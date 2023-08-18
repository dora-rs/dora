use arrow::array::ArrayData;
use arrow::array::Float32Array;
use arrow::array::Float64Array;
use arrow::array::Int32Array;
use arrow::array::StringArray;
use arrow::array::StructArray;
use arrow::datatypes::DataType;
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
        match &self.value.data_type() {
            DataType::Int32 => {
                let int_array: Int32Array = self.value.clone().into();
                let number = int_array.value(0);
                serializer.serialize_i32(number)
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
            DataType::List(_field_ref) => todo!(),
            DataType::Struct(_fields) => {
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
                if let DataType::Struct(fields) = self.type_info.fields.clone() {
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
                                    fields: field.data_type().clone(),
                                    defaults: default,
                                },
                            },
                        )?;
                    }
                    s.end()
                } else {
                    return Err(serde::ser::Error::custom(format!("Wrong fields type",)));
                }
            }
            _ => todo!(),
        }
    }
}
