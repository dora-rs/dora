#![allow(non_camel_case_types)]

use speedy::{Endianness, Readable};
use enumflags2::{bitflags, BitFlags};

use crate::RepresentationIdentifier;

pub trait FromEndianness {
  fn from_endianness(end: speedy::Endianness) -> Self;
}

macro_rules! submessageflag_impls {
  ($t:ident) => {
    impl FromEndianness for BitFlags<$t> {
      fn from_endianness(end: speedy::Endianness) -> Self {
        if end == Endianness::LittleEndian {
          $t::Endianness.into()
        } else {
          Self::empty()
        }
      }
    }

    impl $t {
      // This returns representation identifier, assuming it is ordinary CDR
      #[allow(dead_code)] // allowing dead code, as it is auto-generated for each flag type.
      pub fn cdr_representation_identifier(bfs: BitFlags<$t>) -> RepresentationIdentifier {
        if bfs.contains($t::Endianness) {
          RepresentationIdentifier::CDR_LE
        } else {
          RepresentationIdentifier::CDR_BE
        }
      }
    }
  };
}

pub fn endianness_flag(flags: u8) -> speedy::Endianness {
  if (flags & 0x01) == 0 {
    Endianness::BigEndian
  } else {
    Endianness::LittleEndian
  }
}

/// Identifies the endianness used to encapsulate the Submessage, the
/// presence of optional elements with in the Submessage, and possibly
/// modifies the interpretation of the Submessage. There are
/// 8 possible flags. The first flag (index 0) identifies the
/// endianness used to encapsulate the Submessage. The remaining
/// flags are interpreted differently depending on the kind
/// of Submessage and are described separately for each Submessage.

#[derive(Debug, PartialOrd, PartialEq, Ord, Eq, Readable, Clone, Copy)]
#[bitflags]
#[repr(u8)]
pub enum ACKNACK_Flags {
  Endianness = 0b01,
  Final = 0b10,
}
submessageflag_impls!(ACKNACK_Flags);

#[derive(Debug, PartialOrd, PartialEq, Ord, Eq, Readable, Clone, Copy)]
#[repr(u8)]
#[bitflags]
pub enum DATA_Flags {
  Endianness = 0b00001,
  InlineQos = 0b00010,
  Data = 0b00100,
  Key = 0b01000,
  NonStandardPayload = 0b10000,
}
submessageflag_impls!(DATA_Flags);

#[derive(Debug, PartialOrd, PartialEq, Ord, Eq, Readable, Clone, Copy)]
#[repr(u8)]
#[bitflags]
pub enum DATAFRAG_Flags {
  Endianness = 0b00001,
  InlineQos = 0b00010,
  Key = 0b00100,
  NonStandardPayload = 0b01000,
}
submessageflag_impls!(DATAFRAG_Flags);

impl DATAFRAG_Flags {
  #[allow(dead_code)] // TODO: remove annotation when DATA_FRAG is supported
  pub fn to_data_flags(dff: BitFlags<Self>) -> BitFlags<DATA_Flags> {
    let mut df: BitFlags<DATA_Flags> = if dff.contains(Self::Key) {
      DATA_Flags::Key.into()
    } else {
      DATA_Flags::Data.into()
    };
    if dff.contains(Self::Endianness) {
      df.insert(DATA_Flags::Endianness);
    }
    if dff.contains(Self::InlineQos) {
      df.insert(DATA_Flags::InlineQos);
    }
    if dff.contains(Self::NonStandardPayload) {
      df.insert(DATA_Flags::NonStandardPayload);
    }
    df
  }
}

#[derive(Debug, PartialOrd, PartialEq, Ord, Eq, Readable, Clone, Copy)]
#[repr(u8)]
#[bitflags]
pub enum GAP_Flags {
  Endianness = 0b00001,
}
submessageflag_impls!(GAP_Flags);

#[derive(Debug, PartialOrd, PartialEq, Ord, Eq, Readable, Clone, Copy)]
#[repr(u8)]
#[bitflags]
pub enum HEARTBEAT_Flags {
  Endianness = 0b00001,
  Final = 0b00010,
  Liveliness = 0b00100,
}
submessageflag_impls!(HEARTBEAT_Flags);

#[derive(Debug, PartialOrd, PartialEq, Ord, Eq, Readable, Clone, Copy)]
#[repr(u8)]
#[bitflags]
pub enum HEARTBEATFRAG_Flags {
  Endianness = 0b00001,
}
submessageflag_impls!(HEARTBEATFRAG_Flags);

#[derive(Debug, PartialOrd, PartialEq, Ord, Eq, Readable, Clone, Copy)]
#[repr(u8)]
#[bitflags]
pub enum INFODESTINATION_Flags {
  Endianness = 0b00001,
}
submessageflag_impls!(INFODESTINATION_Flags);

#[derive(Debug, PartialOrd, PartialEq, Ord, Eq, Readable, Clone, Copy)]
#[repr(u8)]
#[bitflags]
pub enum INFOREPLY_Flags {
  Endianness = 0b01,
  Multicast = 0b10,
}
submessageflag_impls!(INFOREPLY_Flags);

#[derive(Debug, PartialOrd, PartialEq, Ord, Eq, Readable, Clone, Copy)]
#[repr(u8)]
#[bitflags]
pub enum INFOSOURCE_Flags {
  Endianness = 0b00001,
}
submessageflag_impls!(INFOSOURCE_Flags);

#[derive(Debug, PartialOrd, PartialEq, Ord, Eq, Readable, Clone, Copy)]
#[repr(u8)]
#[bitflags]
pub enum INFOTIMESTAMP_Flags {
  Endianness = 0b01,
  Invalidate = 0b10,
}
submessageflag_impls!(INFOTIMESTAMP_Flags);

#[derive(Debug, PartialOrd, PartialEq, Ord, Eq, Readable, Clone, Copy)]
#[repr(u8)]
#[bitflags]
pub enum PAD_Flags {
  Endianness = 0b00001,
}
submessageflag_impls!(PAD_Flags);

#[derive(Debug, PartialOrd, PartialEq, Ord, Eq, Readable, Clone, Copy)]
#[repr(u8)]
#[bitflags]
pub enum NACKFRAG_Flags {
  Endianness = 0b00001,
}
submessageflag_impls!(NACKFRAG_Flags);

#[derive(Debug, PartialOrd, PartialEq, Ord, Eq, Readable, Clone, Copy)]
#[repr(u8)]
#[bitflags]
pub enum INFOREPLYIP4_Flags {
  Endianness = 0b01,
  Multicast = 0b10,
}
submessageflag_impls!(INFOREPLYIP4_Flags);

/// Section 7.3.7.5.3 of the Security specification (v. 1.1)
#[derive(Debug, PartialOrd, PartialEq, Ord, Eq, Readable, Clone, Copy)]
#[repr(u8)]
#[bitflags]
pub enum SECUREBODY_Flags {
  Endianness = 0b1,
}
submessageflag_impls!(SECUREBODY_Flags);

/// Section 7.3.7.6.3 of the Security specification (v. 1.1)
#[derive(Debug, PartialOrd, PartialEq, Ord, Eq, Readable, Clone, Copy)]
#[repr(u8)]
#[bitflags]
pub enum SECUREPREFIX_Flags {
  Endianness = 0b1,
}
submessageflag_impls!(SECUREPREFIX_Flags);

/// Section 7.3.7.7.3 of the Security specification (v. 1.1)
#[derive(Debug, PartialOrd, PartialEq, Ord, Eq, Readable, Clone, Copy)]
#[repr(u8)]
#[bitflags]
pub enum SECUREPOSTFIX_Flags {
  Endianness = 0b1,
}
submessageflag_impls!(SECUREPOSTFIX_Flags);

/// Section 7.3.7.8.3 of the Security specification (v. 1.1)
#[derive(Debug, PartialOrd, PartialEq, Ord, Eq, Readable, Clone, Copy)]
#[repr(u8)]
#[bitflags]
pub enum SECURERTPSPREFIX_Flags {
  Endianness = 0b1,
}
submessageflag_impls!(SECURERTPSPREFIX_Flags);

/// Section 7.3.7.9.3 of the Security specification (v. 1.1)
#[derive(Debug, PartialOrd, PartialEq, Ord, Eq, Readable, Clone, Copy)]
#[repr(u8)]
#[bitflags]
pub enum SECURERTPSPOSTFIX_Flags {
  Endianness = 0b1,
}
submessageflag_impls!(SECURERTPSPOSTFIX_Flags);

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn endianness_flag_test() {
    assert_eq!(Endianness::BigEndian, endianness_flag(0x00));
    assert_eq!(Endianness::LittleEndian, endianness_flag(0x01));
  }
}
