//! Enable serialisation and deserialisation of capnproto messages
//!

use std::borrow::Cow;
pub mod message_capnp {
    include!("message_capnp.rs");
}
pub use uhlc;

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Metadata<'a> {
    metadata_version: u16,
    timestamp: uhlc::Timestamp,
    pub parameters: MetadataParameters<'a>,
}

#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct MetadataParameters<'a> {
    pub watermark: u64,
    pub deadline: u64,
    pub open_telemetry_context: Cow<'a, str>,
}

impl MetadataParameters<'_> {
    fn into_owned(self) -> MetadataParameters<'static> {
        MetadataParameters {
            open_telemetry_context: self.open_telemetry_context.into_owned().into(),
            ..self
        }
    }
}

impl<'a> Metadata<'a> {
    pub fn new(timestamp: uhlc::Timestamp) -> Self {
        Self::from_parameters(timestamp, Default::default())
    }

    pub fn from_parameters(timestamp: uhlc::Timestamp, parameters: MetadataParameters<'a>) -> Self {
        Self {
            metadata_version: 0,
            timestamp,
            parameters,
        }
    }

    pub fn serialize(&self) -> Result<Vec<u8>, capnp::Error> {
        let Metadata {
            metadata_version,
            timestamp,
            parameters:
                MetadataParameters {
                    watermark,
                    deadline,
                    open_telemetry_context: otel_context,
                },
        } = self;

        let mut meta_builder = capnp::message::Builder::new_default();
        let mut metadata = meta_builder.init_root::<message_capnp::metadata::Builder>();
        metadata.set_metadata_version(*metadata_version);
        metadata.set_watermark(*watermark);
        metadata.set_deadline(*deadline);
        metadata.set_otel_context(otel_context);
        metadata.set_timestamp(&timestamp.to_string());

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
            timestamp: metadata_reader
                .get_timestamp()?
                .parse()
                .map_err(|_| capnp::Error::failed("failed to parse timestamp".into()))?,
            parameters: MetadataParameters {
                watermark: metadata_reader.get_watermark(),
                deadline: metadata_reader.get_deadline(),
                open_telemetry_context: metadata_reader.get_otel_context()?.into(),
            },
        };

        Ok(metadata.into_owned())
    }

    fn into_owned(self) -> Metadata<'static> {
        Metadata {
            parameters: self.parameters.into_owned(),
            ..self
        }
    }

    pub fn timestamp(&self) -> uhlc::Timestamp {
        self.timestamp
    }
}
