//! Enable serialisation and deserialisation of capnproto messages
//!

use std::borrow::Cow;
pub mod message_capnp {
    include!(concat!(env!("OUT_DIR"), "/message_capnp.rs"));
}

impl Metadata<'_> {
    pub fn serialize(&self) -> Result<Vec<u8>, capnp::Error> {
        let Metadata {
            metadata_version,
            watermark,
            deadline,
            open_telemetry_context: otel_context,
        } = self;

        let mut meta_builder = capnp::message::Builder::new_default();
        let mut metadata = meta_builder.init_root::<message_capnp::metadata::Builder>();
        metadata.set_metadata_version(*metadata_version);
        metadata.set_watermark(*watermark);
        metadata.set_deadline(*deadline);
        metadata.set_otel_context(otel_context);

        let mut buffer = Vec::new();
        capnp::serialize::write_message(&mut buffer, &meta_builder)?;
        Ok(buffer)
    }

    pub fn deserialize(raw: &mut &[u8]) -> Result<Self, capnp::Error> {
        let deserialized = capnp::serialize::read_message_from_flat_slice(raw, Default::default())?
            .into_typed::<message_capnp::metadata::Owned>();

        let metadata_reader = deserialized.get()?;

        let metadata = Metadata {
            metadata_version: metadata_reader.get_metadata_version(),
            watermark: metadata_reader.get_watermark(),
            deadline: metadata_reader.get_deadline(),
            open_telemetry_context: metadata_reader.get_otel_context()?.into(),
        };

        Ok(metadata.into_owned())
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
