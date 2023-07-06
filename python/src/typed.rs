use eyre::{Context, ContextCompat};
use serde::ser::SerializeStruct;

#[derive(Debug, Clone, PartialEq)]
pub struct TypedValue<'a> {
    pub value: &'a serde_yaml::Value,
    pub type_info: &'a TypeInfo,
}

#[derive(Debug, Clone, PartialEq)]
pub enum TypeInfo {
    Struct {
        name: String,
        fields: Vec<(String, TypeInfo)>,
    },
    I32,
}

impl serde::Serialize for TypedValue<'_> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        match &self.type_info {
            TypeInfo::I32 => {
                let number = self
                    .value
                    .as_i64()
                    .context("expected i32 value")
                    .and_then(|n| i32::try_from(n).context("value too large"))
                    .map_err(serde::ser::Error::custom)?;
                serializer.serialize_i32(number)
            }
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
                for (field_name, field_type) in fields {
                    let field_value = self
                        .value
                        .get(field_name)
                        .with_context(|| format!("value has no field `{field_name}`"))
                        .map_err(serde::ser::Error::custom)?;
                    s.serialize_field(
                        DUMMY_FIELD_NAME,
                        &TypedValue {
                            value: field_value,
                            type_info: field_type,
                        },
                    )?;
                }
                s.end()
            }
        }
    }
}
