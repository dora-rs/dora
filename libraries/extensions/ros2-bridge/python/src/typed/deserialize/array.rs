use arrow::array::ArrayData;
use dora_ros2_bridge_msg_gen::types::sequences;

use crate::typed::TypeInfo;

use super::sequence::SequenceVisitor;

pub struct ArrayDeserializer<'a> {
    pub array_type: &'a sequences::Array,
    pub type_info: &'a TypeInfo<'a>,
}

impl<'de> serde::de::DeserializeSeed<'de> for ArrayDeserializer<'_> {
    type Value = ArrayData;

    fn deserialize<D>(self, deserializer: D) -> Result<Self::Value, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        deserializer.deserialize_tuple(
            self.array_type.size,
            SequenceVisitor {
                item_type: &self.array_type.value_type,
                type_info: self.type_info,
            },
        )
    }
}
