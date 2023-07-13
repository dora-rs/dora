use std::{cmp::min, io};

use bytes::{Bytes, BytesMut};
use speedy::{Context, Writable, Writer};
use byteorder::ReadBytesExt;
use log::warn;

use crate::RepresentationIdentifier;

/// A SerializedPayload submessage element contains the serialized
/// representation of either value of an application-defined data-object or
/// the value of the key that uniquely identifies the data-object
/// See RTPS spec v2.3 section 10.
/// Standard representation identifier values are defined in sections 10.2 -
/// 10.5 representation_options "shall be interpreted in the context of the
/// RepresentationIdentifier, such that each RepresentationIdentifier may define
/// the representation_options that it requires." and "The [2.3] version of the
/// protocol does not use the representation_options: The sender shall set the
/// representation_options to zero. The receiver shall ignore the value of the
/// representation_options."
#[derive(Debug, PartialEq, Eq, Clone)]
pub struct SerializedPayload {
  pub representation_identifier: RepresentationIdentifier,
  pub representation_options: [u8; 2], // Not used. Send as zero, ignore on receive.
  pub value: Bytes,
}

// header length
// 2 bytes for representation identifier
// + 2 bytes for representation options
const H_LEN: usize = 2 + 2;

impl SerializedPayload {
  #[cfg(test)]
  pub fn new(rep_id: RepresentationIdentifier, payload: Vec<u8>) -> Self {
    Self {
      representation_identifier: rep_id,
      representation_options: [0, 0],
      value: Bytes::from(payload),
    }
  }

  pub fn new_from_bytes(rep_id: RepresentationIdentifier, payload: Bytes) -> Self {
    Self {
      representation_identifier: rep_id,
      representation_options: [0, 0],
      value: payload,
    }
  }

  /// serialized size in bytes
  pub fn len_serialized(&self) -> usize {
    H_LEN + self.value.len()
  }

  // a slice of serialized data
  // This has a lot of H_LEN offsets, because the data to be sliced
  // is header + value
  pub fn bytes_slice(&self, from: usize, to_before: usize) -> Bytes {
    // sanitize inputs. These are unsigned values, so always at least zero.
    let to_before = min(to_before, self.value.len() + H_LEN);
    let from = min(from, to_before);

    if from >= H_LEN {
      // no need to copy, can return a slice
      self.value.slice(from - H_LEN..to_before - H_LEN)
    } else {
      // We need to copy the payload on order to prefix with header
      let mut b = BytesMut::with_capacity(to_before);
      b.extend_from_slice(&self.representation_identifier.bytes);
      b.extend_from_slice(&self.representation_options);
      assert_eq!(b.len(), H_LEN);
      if to_before > H_LEN {
        b.extend_from_slice(&self.value.slice(..to_before - H_LEN));
      }
      b.freeze().slice(from..to_before)
    }
  }

  // Implement deserialization here, because Speedy just makes it difficult.
  pub fn from_bytes(bytes: &Bytes) -> io::Result<Self> {
    let mut reader = io::Cursor::new(&bytes);
    let representation_identifier = RepresentationIdentifier {
      bytes: [reader.read_u8()?, reader.read_u8()?],
    };
    let representation_options = [reader.read_u8()?, reader.read_u8()?];
    let value = if bytes.len() >= H_LEN {
      bytes.slice(H_LEN..)
    } else {
      warn!(
        "DATA submessage was smaller than submessage header: {:?}",
        bytes
      );
      return Err(io::Error::new(
        io::ErrorKind::Other,
        "Too short DATA submessage.",
      ));
    };

    Ok(Self {
      representation_identifier,
      representation_options,
      value,
    })
  }
}

impl<C: Context> Writable<C> for SerializedPayload {
  fn write_to<T: ?Sized + Writer<C>>(&self, writer: &mut T) -> Result<(), C::Error> {
    writer.write_u8(self.representation_identifier.bytes[0])?;
    writer.write_u8(self.representation_identifier.bytes[1])?;
    writer.write_u8(self.representation_options[0])?;
    writer.write_u8(self.representation_options[1])?;
    writer.write_bytes(&self.value)?;
    Ok(())
  }
}
