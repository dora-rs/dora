use std::collections::BTreeMap;

#[allow(unused_imports)]
use log::{debug, error, info, trace, warn};
use speedy::{Context, Readable, Reader, Writable, Writer};

use crate::{
  messages::submessages::elements::parameter::Parameter,
  serialization::pl_cdr_adapters::{PlCdrDeserializeError, PlCdrSerializeError},
  structure::parameter_id::ParameterId,
  RepresentationIdentifier,
};

pub fn pl_cdr_rep_id_to_speedy(
  encoding: RepresentationIdentifier,
) -> Result<speedy::Endianness, PlCdrSerializeError> {
  match encoding {
    RepresentationIdentifier::PL_CDR_LE => Ok(speedy::Endianness::LittleEndian),
    RepresentationIdentifier::PL_CDR_BE => Ok(speedy::Endianness::BigEndian),
    _ => Err(PlCdrSerializeError::NotSupported(
      "Unknown encoding, expected PL_CDR".to_string(),
    )),
  }
}

pub fn pl_cdr_rep_id_to_speedy_d(
  encoding: RepresentationIdentifier,
) -> Result<speedy::Endianness, PlCdrDeserializeError> {
  match encoding {
    RepresentationIdentifier::PL_CDR_LE => Ok(speedy::Endianness::LittleEndian),
    RepresentationIdentifier::PL_CDR_BE => Ok(speedy::Endianness::BigEndian),
    _ => Err(PlCdrDeserializeError::NotSupported(
      "Unknown encoding, expected PL_CDR".to_string(),
    )),
  }
}

// This is a helper type for serialization.
// CDR (and therefore PL_CDR) mandates that strings are nul-terminated.
// Our CDR serializer does that, but Speedy Readable and Writable need this
// wrapper.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StringWithNul {
  string: String,
}

impl StringWithNul {
  // length including null terminator
  pub fn len(&self) -> usize {
    self.string.len() + 1
  }
}

impl From<String> for StringWithNul {
  fn from(string: String) -> Self {
    StringWithNul { string }
  }
}

impl From<&String> for StringWithNul {
  fn from(string: &String) -> Self {
    StringWithNul {
      string: string.clone(),
    }
  }
}

impl From<StringWithNul> for String {
  fn from(value: StringWithNul) -> String {
    value.string
  }
}

impl<C: Context> Writable<C> for StringWithNul {
  #[inline]
  fn write_to<T: ?Sized + Writer<C>>(&self, writer: &mut T) -> std::result::Result<(), C::Error> {
    // TODO: How should we fail if someone tries to serialize string longer than 4
    // GBytes? RTPS does not support that.

    // TODO: Should align to 4 before writing
    writer.write_u32((self.string.len() + 1).try_into().unwrap())?; // +1 for NUL character
    writer.write_slice(self.string.as_bytes())?;
    writer.write_u8(0)?; // NUL character
    Ok(())
  }
}

impl<'a, C: Context> Readable<'a, C> for StringWithNul {
  #[inline]
  fn read_from<R: speedy::Reader<'a, C>>(reader: &mut R) -> std::result::Result<Self, C::Error> {
    // TODO: Should align to 4 before reading string length
    let mut raw_str: String = reader.read_value()?;
    let assumed_nul = raw_str.pop(); //
    match assumed_nul {
      Some('\0') => { /* fine */ }
      Some(other) => error!("StringWithNul deserialize: Expected NUL character, decoded {other:?}"),
      None => {
        error!("StringWithNul deserialize: Expected NUL character, but end of input reached.");
      }
    }
    Ok(StringWithNul { string: raw_str })
  }
}

// Helpers for Readable/Writable padding

// These are for PL_CDR (de)serialization
pub(crate) fn read_pad<'a, C: Context, R: Reader<'a, C>>(
  reader: &mut R,
  read_length: usize,
  align: usize,
) -> std::result::Result<(), C::Error> {
  let m = read_length % align;
  if m > 0 {
    reader.skip_bytes(align - m)?;
  }
  Ok(())
}

pub(crate) fn write_pad<C: Context, T: ?Sized + Writer<C>>(
  writer: &mut T,
  previous_length: usize,
  align: usize,
) -> std::result::Result<(), C::Error> {
  let m = previous_length % align;
  if m > 0 {
    for _ in 0..(align - m) {
      writer.write_u8(0)?;
    }
  }
  Ok(())
}

// Helper functions for ParameterList deserialization:
//
// Get and deserialize first occurrence of ParameterId in map
pub(crate) fn get_first_from_pl_map<'a, C, D>(
  pl_map: &'a BTreeMap<ParameterId, Vec<&Parameter>>,
  ctx: C,
  pid: ParameterId,
  name: &str,
) -> Result<D, PlCdrDeserializeError>
where
  C: speedy::Context,
  D: Readable<'a, C>,
  PlCdrDeserializeError: From<<C as speedy::Context>::Error>,
{
  pl_map
    .get(&pid)
    .and_then(|v| v.first())
    .ok_or(PlCdrDeserializeError::MissingField(pid, name.to_string()))
    .and_then(|p| {
      D::read_from_buffer_with_ctx(ctx, &p.value).map_err(|e| {
        error!("PL_CDR Deserializing {name}");
        e.into()
      })
    })
}

// same, but gets all occurrences
#[allow(clippy::needless_pass_by_value)]
pub(crate) fn get_all_from_pl_map<'a, C, D>(
  pl_map: &'a BTreeMap<ParameterId, Vec<&Parameter>>,
  ctx: C,
  pid: ParameterId,
  name: &str,
) -> Result<Vec<D>, PlCdrDeserializeError>
where
  C: speedy::Context + Clone,
  D: Readable<'a, C>,
  PlCdrDeserializeError: From<<C as speedy::Context>::Error>,
{
  pl_map
    .get(&pid)
    .unwrap_or(&Vec::new())
    .iter()
    .map(|p| {
      D::read_from_buffer_with_ctx(ctx.clone(), &p.value).map_err(|e| {
        error!("PL_CDR Deserializing {name}");
        e.into()
      })
    })
    .collect()
}

// same, but either gets the occurrence or not. Getting nothing is not an Error.
pub(crate) fn get_option_from_pl_map<'a, C, D>(
  pl_map: &'a BTreeMap<ParameterId, Vec<&Parameter>>,
  ctx: C,
  pid: ParameterId,
  name: &str,
) -> Result<Option<D>, PlCdrDeserializeError>
where
  C: speedy::Context + Clone,
  D: Readable<'a, C>,
  PlCdrDeserializeError: From<<C as speedy::Context>::Error>,
{
  pl_map
    .get(&pid)
    .and_then(|v| v.first()) // Option<Parameter> here
    .map(|p| {
      D::read_from_buffer_with_ctx(ctx, &p.value).map_err(|e| {
        error!("PL_CDR Deserializing {name}");
        info!("Parameter payload was {:x?}", p.value);
        e.into()
      })
    })
    .transpose()
}
