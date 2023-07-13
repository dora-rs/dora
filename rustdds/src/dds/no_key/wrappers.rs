use std::{marker::PhantomData, ops::Deref};

use bytes::Bytes;

use crate::{
  dds::adapters::*, messages::submessages::submessages::RepresentationIdentifier, Keyed,
};

// This wrapper is used to convert NO_KEY types to WITH_KEY
// * inside the wrapper there is a NO_KEY type
// * the wrapper is good for WITH_KEY
// The wrapper introduces a dummy key of type (), which of course has an always
// known value ()
pub(crate) struct NoKeyWrapper<D> {
  pub(crate) d: D,
}

impl<D> From<D> for NoKeyWrapper<D> {
  fn from(d: D) -> Self {
    Self { d }
  }
}

// implement Deref so that &NoKeyWrapper<D> is coercible to &D
impl<D> Deref for NoKeyWrapper<D> {
  type Target = D;
  fn deref(&self) -> &Self::Target {
    &self.d
  }
}

impl<D> Keyed for NoKeyWrapper<D> {
  type K = ();
  fn key(&self) {}
}

// wrapper for SerializerAdapter
// * inside is NO_KEY
// * outside of wrapper is WITH_KEY
pub struct SAWrapper<SA> {
  no_key: PhantomData<SA>,
}

// have to implement base trait first, just trivial passthrough
impl<D, SA> no_key::SerializerAdapter<NoKeyWrapper<D>> for SAWrapper<SA>
where
  SA: no_key::SerializerAdapter<D>,
{
  type Error = SA::Error;

  fn output_encoding() -> RepresentationIdentifier {
    SA::output_encoding()
  }

  fn to_bytes(value: &NoKeyWrapper<D>) -> Result<Bytes, SA::Error> {
    SA::to_bytes(&value.d)
  }
}

// This is the point of wrapping. Implement dummy key serialization
// Of course, this is never supposed to be actually called.
impl<D, SA> with_key::SerializerAdapter<NoKeyWrapper<D>> for SAWrapper<SA>
where
  SA: no_key::SerializerAdapter<D>,
{
  fn key_to_bytes(_value: &()) -> Result<Bytes, SA::Error> {
    Ok(Bytes::new())
  }
}

// wrapper for DeserializerAdapter
// * inside is NO_KEY
// * outside of wrapper is WITH_KEY
pub struct DAWrapper<DA> {
  no_key: PhantomData<DA>,
}

// first, implement no_key DA
impl<D, DA> no_key::DeserializerAdapter<NoKeyWrapper<D>> for DAWrapper<DA>
where
  DA: no_key::DeserializerAdapter<D>,
{
  type Error = DA::Error;

  fn supported_encodings() -> &'static [RepresentationIdentifier] {
    DA::supported_encodings()
  }

  fn from_bytes(
    input_bytes: &[u8],
    encoding: RepresentationIdentifier,
  ) -> Result<NoKeyWrapper<D>, DA::Error> {
    DA::from_bytes(input_bytes, encoding).map(|d| NoKeyWrapper::<D> { d })
  }
}

// then, implement with_key DA
impl<D, DA> with_key::DeserializerAdapter<NoKeyWrapper<D>> for DAWrapper<DA>
where
  DA: no_key::DeserializerAdapter<D>,
{
  fn key_from_bytes(
    _input_bytes: &[u8],
    _encoding: RepresentationIdentifier,
  ) -> Result<<NoKeyWrapper<D> as Keyed>::K, DA::Error> {
    // also unreachable!() should work here, as this is not supposed to be used
    Ok(())
  }
}
