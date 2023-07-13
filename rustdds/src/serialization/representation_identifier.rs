use std::io;

use speedy::{Readable, Writable};
use byteorder::ReadBytesExt;

/// Used to identify serialization format of payload data over RTPS.
#[derive(Debug, PartialEq, Eq, Clone, Copy, Readable, Writable)]
pub struct RepresentationIdentifier {
  pub(crate) bytes: [u8; 2], // semi-public for serialization elsewhere
}

impl RepresentationIdentifier {
  // Numeric values are from RTPS spec v2.3 Section 10.5 , Table 10.3
  pub const CDR_BE: Self = Self {
    bytes: [0x00, 0x00],
  };
  pub const CDR_LE: Self = Self {
    bytes: [0x00, 0x01],
  };

  pub const PL_CDR_BE: Self = Self {
    bytes: [0x00, 0x02],
  };
  pub const PL_CDR_LE: Self = Self {
    bytes: [0x00, 0x03],
  };

  pub const CDR2_BE: Self = Self {
    bytes: [0x00, 0x10],
  };
  pub const CDR2_LE: Self = Self {
    bytes: [0x00, 0x11],
  };

  pub const PL_CDR2_BE: Self = Self {
    bytes: [0x00, 0x12],
  };
  pub const PL_CDR2_LE: Self = Self {
    bytes: [0x00, 0x13],
  };

  pub const D_CDR_BE: Self = Self {
    bytes: [0x00, 0x14],
  };
  pub const D_CDR_LE: Self = Self {
    bytes: [0x00, 0x15],
  };

  pub const XML: Self = Self {
    bytes: [0x00, 0x04],
  };

  // Reads two bytes to form a `RepresentationIdentifier`
  pub fn from_bytes(bytes: &[u8]) -> io::Result<Self> {
    let mut reader = io::Cursor::new(bytes);
    Ok(Self {
      bytes: [reader.read_u8()?, reader.read_u8()?],
    })
  }

  pub fn to_bytes(self) -> [u8; 2] {
    self.bytes
  }
}
