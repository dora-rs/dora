use speedy::{Context, Endianness, Readable, Reader, Writable, Writer};

use crate::messages::submessages::{submessage_flag::*, submessage_kind::SubmessageKind};

#[derive(Debug, PartialEq, Eq, Clone, Copy)] // This is only 32 bits, so better Copy
pub struct SubmessageHeader {
  pub kind: SubmessageKind,
  pub flags: u8, // This must be able to contain anything combination of any flags.
  pub content_length: u16, // Note that 0 is a special value, see spec 9.4.5.1.3
}

impl<'a, C: Context> Readable<'a, C> for SubmessageHeader {
  #[inline]
  fn read_from<R: Reader<'a, C>>(reader: &mut R) -> Result<Self, C::Error> {
    let kind: SubmessageKind = reader.read_value()?;
    let flags: u8 = reader.read_value()?;
    let content_length = match endianness_flag(flags) {
      // Speedy does not make this too easy. There seems to be no convenient way to
      // read u16 when endianness is decided at run-time.
      Endianness::LittleEndian => u16::from_le_bytes([reader.read_u8()?, reader.read_u8()?]),
      Endianness::BigEndian => u16::from_be_bytes([reader.read_u8()?, reader.read_u8()?]),
    };

    Ok(Self {
      kind,
      flags,
      content_length,
    })
  }

  #[inline]
  fn minimum_bytes_needed() -> usize {
    std::mem::size_of::<Self>()
  }
}

impl<C: Context> Writable<C> for SubmessageHeader {
  #[inline]
  fn write_to<T: ?Sized + Writer<C>>(&self, writer: &mut T) -> Result<(), C::Error> {
    writer.write_value(&self.kind)?;
    writer.write_value(&self.flags)?;

    match endianness_flag(self.flags) {
      // matching via writer.context().endianness() panics
      speedy::Endianness::LittleEndian => {
        writer.write_u8(self.content_length as u8)?;
        writer.write_u8((self.content_length >> 8) as u8)?;
      }
      speedy::Endianness::BigEndian => {
        writer.write_u8((self.content_length >> 8) as u8)?;
        writer.write_u8(self.content_length as u8)?;
      }
    };

    Ok(())
  }
}

#[cfg(test)]
mod tests {
  use enumflags2::BitFlags;

  use super::*;

  serialization_test!( type = SubmessageHeader,
  {
      submessage_header_big_endian_flag,
      SubmessageHeader {
          kind: SubmessageKind::ACKNACK,
          flags: BitFlags::<ACKNACK_Flags>::from_endianness(Endianness::BigEndian).bits(),
          content_length: 42,
      },
      le = [0x06, 0x00, 0x00, 0x2A],
      be = [0x06, 0x00, 0x00, 0x2A]
  },
  {
      submessage_header_little_endian_flag,
      SubmessageHeader {
          kind: SubmessageKind::ACKNACK,
          flags: BitFlags::<ACKNACK_Flags>::from_endianness(Endianness::LittleEndian).bits(),
          content_length: 42,
      },
      le = [0x06, 0x01, 0x2A, 0x00],
      be = [0x06, 0x01, 0x2A, 0x00]
  },
  {
      submessage_header_big_endian_2_bytes_length,
      SubmessageHeader {
          kind: SubmessageKind::ACKNACK,
          flags: BitFlags::<ACKNACK_Flags>::from_endianness(Endianness::BigEndian).bits(),
          content_length: 258,
      },
      le = [0x06, 0x00, 0x01, 0x02],
      be = [0x06, 0x00, 0x01, 0x02]
  },
  {
      submessage_header_little_endian_2_bytes_length,
      SubmessageHeader {
          kind: SubmessageKind::ACKNACK,
          flags: BitFlags::<ACKNACK_Flags>::from_endianness(Endianness::LittleEndian).bits(),
          content_length: 258,
      },
      le = [0x06, 0x01, 0x02, 0x01],
      be = [0x06, 0x01, 0x02, 0x01]
  },
  {
      submessage_header_wireshark,
      SubmessageHeader {
          kind: SubmessageKind::INFO_TS,
          flags: BitFlags::<INFOTIMESTAMP_Flags>::from_endianness(Endianness::LittleEndian).bits(),
          content_length: 8,
      },
      le = [0x09, 0x01, 0x08, 0x00],
      be = [0x09, 0x01, 0x08, 0x00]
  },
  {
      submessage_header_gap,
      SubmessageHeader {
          kind: SubmessageKind::GAP,
          flags: BitFlags::<GAP_Flags>::from_endianness(Endianness::LittleEndian).bits(),
          content_length: 7,
      },
      le = [0x08, 0x01, 0x07, 0x00],
      be = [0x08, 0x01, 0x07, 0x00]
      //TODO: Where is the flags value 0x03 from? RTPS 2.3 spec 9.4.5.5 shows only Endianness bit is legal.
  });
}
