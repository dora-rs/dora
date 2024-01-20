use arrow::array::ArrayData;
use dora_ros2_bridge_msg_gen::types::sequences;

use super::sequence::SequenceVisitor;

pub struct ArrayDeserializer<'a>(pub &'a sequences::Array);

impl<'de> serde::de::DeserializeSeed<'de> for ArrayDeserializer<'_> {
    type Value = ArrayData;

    fn deserialize<D>(self, deserializer: D) -> Result<Self::Value, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        deserializer.deserialize_tuple(self.0.size, SequenceVisitor(&self.0.value_type))
    }
}
