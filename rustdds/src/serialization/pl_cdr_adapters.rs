use std::marker::PhantomData;

use bytes::Bytes;
use byteorder::{ByteOrder, LittleEndian};

use crate::{
  dds::adapters::{no_key, with_key},
  structure::parameter_id::ParameterId,
  Keyed, RepresentationIdentifier,
};

// This is to be implemented by all Discovery message types.
// .. likely it is not useful for others.
pub trait PlCdrSerialize {
  // encoding must be either PL_CDR_LE or PL_CDR_BE
  fn to_pl_cdr_bytes(
    &self,
    encoding: RepresentationIdentifier,
  ) -> Result<Bytes, PlCdrSerializeError>;
}

#[derive(Debug, thiserror::Error)]
pub enum PlCdrSerializeError {
  #[error("Serializer does not support this operation: {0}")]
  NotSupported(String),

  #[error("Speedy serializer error: {0}")]
  Speedy(#[from] speedy::Error),
}

pub struct PlCdrSerializerAdapter<D, BO = LittleEndian>
where
  BO: ByteOrder,
{
  phantom: PhantomData<D>,
  ghost: PhantomData<BO>,
}

impl<D, BO> no_key::SerializerAdapter<D> for PlCdrSerializerAdapter<D, BO>
where
  D: PlCdrSerialize,
  BO: ByteOrder,
{
  type Error = PlCdrSerializeError;

  fn output_encoding() -> RepresentationIdentifier {
    // TODO: This works only for BO=LittleEndian
    RepresentationIdentifier::PL_CDR_LE
  }

  fn to_bytes(value: &D) -> Result<Bytes, Self::Error> {
    // TODO: This works only for BO=LittleEndian
    value.to_pl_cdr_bytes(RepresentationIdentifier::PL_CDR_LE)
  }
}

impl<D, BO> with_key::SerializerAdapter<D> for PlCdrSerializerAdapter<D, BO>
where
  D: Keyed + PlCdrSerialize,
  <D as Keyed>::K: PlCdrSerialize,
  BO: ByteOrder,
{
  fn key_to_bytes(value: &D::K) -> Result<Bytes, Self::Error> {
    // TODO: This works only for BO=LittleEndian
    value.to_pl_cdr_bytes(RepresentationIdentifier::PL_CDR_LE)
  }
}

// ----------------------------------
// ----------------------------------
// ----------------------------------

// This is to be implemented by all Discovery message types.
// .. likely it is not useful for others.
pub trait PlCdrDeserialize: Sized {
  // encoding must be either PL_CDR_LE or PL_CDR_BE
  fn from_pl_cdr_bytes(
    input_bytes: &[u8],
    encoding: RepresentationIdentifier,
  ) -> Result<Self, PlCdrDeserializeError>;
}

#[derive(Debug, thiserror::Error)]
pub enum PlCdrDeserializeError {
  #[error("Deserializer does not support this operation: {0}")]
  NotSupported(String),

  #[error("Speedy deserializer error: {0}")]
  Speedy(#[from] speedy::Error),

  #[error("Parameter List missing {0:?} , expected for field {1}")]
  MissingField(ParameterId, String),
}

pub struct PlCdrDeserializerAdapter<D> {
  phantom: PhantomData<D>,
}

const REPR_IDS: [RepresentationIdentifier; 2] = [
  // PL_CDR_* are expected
  RepresentationIdentifier::PL_CDR_BE,
  RepresentationIdentifier::PL_CDR_LE,
];

impl<D> no_key::DeserializerAdapter<D> for PlCdrDeserializerAdapter<D>
where
  D: PlCdrDeserialize,
{
  type Error = PlCdrDeserializeError;

  fn supported_encodings() -> &'static [RepresentationIdentifier] {
    &REPR_IDS
  }

  fn from_bytes(input_bytes: &[u8], encoding: RepresentationIdentifier) -> Result<D, Self::Error> {
    match encoding {
      RepresentationIdentifier::PL_CDR_LE | RepresentationIdentifier::PL_CDR_BE => {
        D::from_pl_cdr_bytes(input_bytes, encoding)
      }
      repr_id => Err(PlCdrDeserializeError::NotSupported(format!(
        "Unknown representation identifier {:?}",
        repr_id
      ))),
    }
  }
}

impl<D> with_key::DeserializerAdapter<D> for PlCdrDeserializerAdapter<D>
where
  D: Keyed + PlCdrDeserialize,
  <D as Keyed>::K: PlCdrDeserialize,
{
  fn key_from_bytes(
    input_bytes: &[u8],
    encoding: RepresentationIdentifier,
  ) -> Result<D::K, Self::Error> {
    match encoding {
      RepresentationIdentifier::PL_CDR_LE | RepresentationIdentifier::PL_CDR_BE => {
        <D::K>::from_pl_cdr_bytes(input_bytes, encoding)
      }
      repr_id => Err(PlCdrDeserializeError::NotSupported(format!(
        "Unknown representation identifier {:?}",
        repr_id
      ))),
    }
  }
}
