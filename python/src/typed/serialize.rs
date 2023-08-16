use super::TypeInfo;
use arrow::array::ArrayData;
use arrow::array::Float32Array;
use arrow::array::Float64Array;
use arrow::array::Int32Array;
use arrow::array::StringArray;
use arrow::array::StructArray;
use serde::ser::SerializeStruct;

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
        match &self.type_info {
            TypeInfo::I32 => {
                let int_array: Int32Array = self.value.clone().into();
                let number = int_array.value(0);
                serializer.serialize_i32(number)
            }
            TypeInfo::F32 => {
                let int_array: Float32Array = self.value.clone().into();
                let number = int_array.value(0);
                serializer.serialize_f32(number)
            }
            TypeInfo::F64 => {
                let int_array: Float64Array = self.value.clone().into();
                let number = int_array.value(0);
                serializer.serialize_f64(number)
            }
            TypeInfo::String => {
                let int_array: StringArray = self.value.clone().into();
                let string = int_array.value(0);
                serializer.serialize_str(string)
            }
            TypeInfo::Array(basic_type) => todo!(),
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
                const DUMMY_FIELD_NAME: &str = "field";

                let mut s = serializer.serialize_struct(DUMMY_STRUCT_NAME, fields.len())?;
                for field in fields {
                    let field_value = match self.value.get(&field.name) {
                        Some(value) => value,
                        None => match &field.default {
                            Some(default) => default,
                            None => {
                                return Err(serde::ser::Error::custom(eyre::eyre!(
                                    "value has no field `{}`",
                                    &field.name
                                )))
                            }
                        },
                    };
                    s.serialize_field(
                        DUMMY_FIELD_NAME,
                        &TypedValue {
                            value: field_value,
                            type_info: &field.ty,
                        },
                    )?;
                }
                s.end()
            }
        }
    }
}
