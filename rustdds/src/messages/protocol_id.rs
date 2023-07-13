use speedy::{Context, Readable, Reader, Writable, Writer};

use crate::messages::validity_trait::Validity;

#[derive(Debug, Copy, Clone, PartialOrd, PartialEq, Ord, Eq)]
pub struct ProtocolId {
  protocol_id: [char; 4],
}

impl ProtocolId {
  pub const PROTOCOL_RTPS: Self = Self {
    protocol_id: ['R', 'T', 'P', 'S'],
  };
}

impl Default for ProtocolId {
  fn default() -> Self {
    Self::PROTOCOL_RTPS
  }
}

impl Validity for ProtocolId {
  fn valid(&self) -> bool {
    *self == Self::PROTOCOL_RTPS
  }
}

impl<'a, C: Context> Readable<'a, C> for ProtocolId {
  #[inline]
  fn read_from<R: Reader<'a, C>>(reader: &mut R) -> Result<Self, C::Error> {
    let mut protocol_id = Self::default();
    for i in 0..protocol_id.protocol_id.len() {
      protocol_id.protocol_id[i] = reader.read_u8()? as char;
    }
    Ok(protocol_id)
  }

  #[inline]
  fn minimum_bytes_needed() -> usize {
    4
  }
}

impl<C: Context> Writable<C> for ProtocolId {
  #[inline]
  fn write_to<T: ?Sized + Writer<C>>(&self, writer: &mut T) -> Result<(), C::Error> {
    for elem in &self.protocol_id {
      writer.write_u8(*elem as u8)?;
    }
    Ok(())
  }
}

#[cfg(test)]
mod tests {
  use speedy::Endianness;

  use super::*;

  #[test]
  fn validity() {
    let protocol_id = ProtocolId::PROTOCOL_RTPS;
    assert!(protocol_id.valid());
    let protocol_id = ProtocolId {
      protocol_id: ['S', 'P', 'T', 'R'],
    };
    assert!(!protocol_id.valid());
  }

  #[test]
  fn minimum_bytes_needed() {
    assert_eq!(
      4,
      <ProtocolId as Readable<Endianness>>::minimum_bytes_needed()
    );
  }

  serialization_test!( type = ProtocolId,
  {
      protocol_rtps,
      ProtocolId::PROTOCOL_RTPS,
      le = [0x52, 0x54, 0x50, 0x53],
      be = [0x52, 0x54, 0x50, 0x53]
  });
}
