/// DeserializerAdapter is used to fit serde Deserializer implementations and
/// DataReader together.
///
/// DataReader cannot assume a specific serialization
/// format, so it needs to be given as a parameter.
///
/// for WITH_KEY topics, we need to be able to (de)serailize the key in addition
/// to data.
pub mod no_key {
  use bytes::Bytes;

  use crate::{serialization::error::Result, RepresentationIdentifier};

  /// trait for connecting a Deserializer implementation and DataReader
  /// together - no_key version.
  pub trait DeserializerAdapter<D> {
    /// Which data representations can the DeserializerAdapter read?
    /// See RTPS specification Section 10 and Table 10.3
    fn supported_encodings() -> &'static [RepresentationIdentifier];

    /// Deserialize data from bytes to an object.
    /// `encoding` must be something given by `supported_encodings()`, or
    /// implementation may fail with Err or `panic!()`.
    fn from_bytes(input_bytes: &[u8], encoding: RepresentationIdentifier) -> Result<D>;

    /// This method has a default implementation, but the default will make a
    /// copy of all the input data in memory and then call from_bytes() .
    // In order to avoid the copy, implement also this method.
    fn from_vec_bytes(input_vec_bytes: &[Bytes], encoding: RepresentationIdentifier) -> Result<D> {
      let total_len = input_vec_bytes.iter().map(Bytes::len).sum();
      let mut total_payload = Vec::with_capacity(total_len);
      for iv in input_vec_bytes {
        total_payload.extend(iv);
      }
      Self::from_bytes(&total_payload, encoding)
    }
  }

  /// trait for connecting a Serializer implementation and DataWriter
  /// together - no_key version.
  pub trait SerializerAdapter<D> {
    // what encoding do we produce?
    fn output_encoding() -> RepresentationIdentifier;

    fn to_bytes(value: &D) -> Result<Bytes>;
  }
}

pub mod with_key {
  use bytes::Bytes;

  use crate::{serialization::Result, Keyed, RepresentationIdentifier};
  use super::no_key;

  /// trait for connecting a Desrializer implementation and DataReader
  /// together - with_key version.
  pub trait DeserializerAdapter<D>: no_key::DeserializerAdapter<D>
  where
    D: Keyed,
  {
    /// Deserialize a key `D::K` from bytes.
    fn key_from_bytes(input_bytes: &[u8], encoding: RepresentationIdentifier) -> Result<D::K>;
  }

  /// trait for connecting a Serializer implementation and DataWriter
  /// together - with_key version.
  pub trait SerializerAdapter<D>: no_key::SerializerAdapter<D>
  where
    D: Keyed,
  {
    /// serialize a key `D::K` to Bytes.
    fn key_to_bytes(value: &D::K) -> Result<Bytes>;
  }
}
