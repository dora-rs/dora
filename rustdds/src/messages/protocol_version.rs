use speedy::{Readable, Writable};
#[allow(unused_imports)]
use log::{debug, error, info, trace};

#[derive(Debug, PartialOrd, PartialEq, Ord, Eq, Readable, Writable, Clone, Copy)]
pub struct ProtocolVersion {
  pub major: u8,
  pub minor: u8,
}

impl ProtocolVersion {
  pub const THIS_IMPLEMENTATION: Self = Self::PROTOCOLVERSION_2_4;

  #[allow(dead_code)] // Specification defines this, but not necessarily used.
  pub const PROTOCOLVERSION_1_0: Self = Self { major: 1, minor: 0 };
  #[allow(dead_code)] // Specification defines this, but not necessarily used.
  pub const PROTOCOLVERSION_1_1: Self = Self { major: 1, minor: 1 };
  #[allow(dead_code)] // Specification defines this, but not necessarily used.
  pub const PROTOCOLVERSION_2_0: Self = Self { major: 2, minor: 0 };
  #[allow(dead_code)] // Specification defines this, but not necessarily used.
  pub const PROTOCOLVERSION_2_1: Self = Self { major: 2, minor: 1 };
  #[allow(dead_code)] // Specification defines this, but not necessarily used.
  pub const PROTOCOLVERSION_2_2: Self = Self { major: 2, minor: 2 };
  #[allow(dead_code)] // Specification defines this, but not necessarily used.
  pub const PROTOCOLVERSION_2_3: Self = Self { major: 2, minor: 3 };
  pub const PROTOCOLVERSION_2_4: Self = Self { major: 2, minor: 4 };
}

impl Default for ProtocolVersion {
  fn default() -> Self {
    Self::THIS_IMPLEMENTATION
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  serialization_test!( type = ProtocolVersion,
  {
      protocol_version,
      ProtocolVersion::THIS_IMPLEMENTATION,
      le = [0x02, 0x04],
      be = [0x02, 0x04]
  },
  {
      protocol_version_default,
      ProtocolVersion::default(),
      le = [0x02, 0x04],
      be = [0x02, 0x04]
  },
  {
      protocol_version_1_0,
      ProtocolVersion::PROTOCOLVERSION_1_0,
      le = [0x01, 0x00],
      be = [0x01, 0x00]
  },
  {
      protocol_version_1_1,
      ProtocolVersion::PROTOCOLVERSION_1_1,
      le = [0x01, 0x01],
      be = [0x01, 0x01]
  },
  {
      protocol_version_2_0,
      ProtocolVersion::PROTOCOLVERSION_2_0,
      le = [0x02, 0x00],
      be = [0x02, 0x00]
  },
  {
      protocol_version_2_1,
      ProtocolVersion::PROTOCOLVERSION_2_1,
      le = [0x02, 0x01],
      be = [0x02, 0x01]
  },
  {
      protocol_version_2_2,
      ProtocolVersion::PROTOCOLVERSION_2_2,
      le = [0x02, 0x02],
      be = [0x02, 0x02]
  },
  {
      protocol_version_2_4,
      ProtocolVersion::PROTOCOLVERSION_2_4,
      le = [0x02, 0x04],
      be = [0x02, 0x04]
  });
}
