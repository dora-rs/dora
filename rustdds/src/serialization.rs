pub(crate) mod speedy_pl_cdr_helpers;

pub(crate) mod cdr_deserializer;
pub(crate) mod cdr_serializer;
pub mod error;
pub mod representation_identifier;

pub(crate) mod pl_cdr_adapters;

// public exports
pub use cdr_serializer::{to_writer_endian, CDRSerializerAdapter, CdrSerializer};
pub use cdr_deserializer::{deserialize_from_cdr, CDRDeserializerAdapter, CdrDeserializer};
pub use byteorder::{BigEndian, LittleEndian};
pub use error::{Error, Result};

pub use crate::dds::adapters::{no_key, with_key};
