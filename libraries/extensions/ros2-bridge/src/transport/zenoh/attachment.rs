use thiserror::Error;
use zenoh::bytes::ZBytes;
use zenoh_ext::{ZDeserializer, ZSerializer};

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Attachment {
    pub sequence_number: i64,
    pub source_timestamp_ns: i64,
    pub gid: [u8; 16],
}

#[derive(Debug, Error)]
pub enum AttachmentError {
    #[error("malformed rmw_zenoh attachment")]
    Malformed,
    #[error("rmw_zenoh attachment has trailing bytes")]
    TrailingBytes,
}

impl Attachment {
    pub fn encode(&self) -> Result<Vec<u8>, AttachmentError> {
        let mut serializer = ZSerializer::new();
        serializer.serialize(self.sequence_number);
        serializer.serialize(self.source_timestamp_ns);
        serializer.serialize(self.gid);
        Ok(serializer.finish().to_bytes().into_owned())
    }

    pub fn decode(bytes: &[u8]) -> Result<Self, AttachmentError> {
        let bytes = ZBytes::from(bytes.to_vec());
        let mut deserializer = ZDeserializer::new(&bytes);
        let value = Self {
            sequence_number: deserializer
                .deserialize()
                .map_err(|_| AttachmentError::Malformed)?,
            source_timestamp_ns: deserializer
                .deserialize()
                .map_err(|_| AttachmentError::Malformed)?,
            gid: deserializer
                .deserialize()
                .map_err(|_| AttachmentError::Malformed)?,
        };
        if !deserializer.done() {
            return Err(AttachmentError::TrailingBytes);
        }
        Ok(value)
    }
}
