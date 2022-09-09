//! Enable serialisation and deserialisation of capnproto messages
//!

use std::borrow::Cow;
pub mod message_capnp {
    include!(concat!(env!("OUT_DIR"), "/message_capnp.rs"));
}

/// Helper function to serialize message
pub fn serialize_message(message: &Message) -> Result<Vec<u8>, capnp::Error> {
    let Message {
        data,
        metadata:
            Metadata {
                metadata_version,
                watermark,
                deadline,
                open_telemetry_context: otel_context,
            },
    } = message;

    // Build the metadata
    let mut meta_builder = capnp::message::Builder::new_default();
    let mut metadata = meta_builder.init_root::<message_capnp::metadata::Builder>();
    metadata.set_metadata_version(*metadata_version);
    metadata.set_watermark(*watermark);
    metadata.set_deadline(*deadline);
    metadata.set_otel_context(otel_context);

    // Build the data of the message
    let mut builder = capnp::message::Builder::new_default();
    let mut message = builder.init_root::<message_capnp::message::Builder>();
    message.set_data(data);
    message.set_metadata(metadata.into_reader())?;

    let mut buffer = Vec::new();
    capnp::serialize::write_message(&mut buffer, &builder)?;
    Ok(buffer)
}

pub fn deserialize_message(mut raw: &[u8]) -> Result<Message, capnp::Error> {
    let deserialized =
        capnp::serialize::read_message_from_flat_slice(&mut raw, Default::default())?
            .into_typed::<message_capnp::message::Owned>();

    let reader = deserialized.get()?;
    let metadata_reader = reader.get_metadata()?;

    let message = Message {
        data: reader.get_data()?.into(),
        metadata: Metadata {
            metadata_version: metadata_reader.get_metadata_version(),
            watermark: metadata_reader.get_watermark(),
            deadline: metadata_reader.get_deadline(),
            open_telemetry_context: metadata_reader.get_otel_context()?.into(),
        },
    };

    // TODO: avoid copying and make this zero copy
    Ok(message.into_owned())
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Message<'a> {
    pub data: Cow<'a, [u8]>,
    pub metadata: Metadata<'a>,
}

impl<'a> From<&'a [u8]> for Message<'a> {
    fn from(data: &'a [u8]) -> Self {
        Self {
            data: data.into(),
            metadata: Default::default(),
        }
    }
}

impl From<Vec<u8>> for Message<'static> {
    fn from(data: Vec<u8>) -> Self {
        Self {
            data: data.into(),
            metadata: Default::default(),
        }
    }
}

impl Message<'_> {
    pub fn into_owned(self) -> Message<'static> {
        Message {
            data: self.data.into_owned().into(),
            metadata: self.metadata.into_owned(),
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Metadata<'a> {
    pub metadata_version: u16,
    pub watermark: u64,
    pub deadline: u64,
    pub open_telemetry_context: Cow<'a, str>,
}

impl Metadata<'_> {
    fn into_owned(self) -> Metadata<'static> {
        Metadata {
            open_telemetry_context: self.open_telemetry_context.into_owned().into(),
            ..self
        }
    }
}
