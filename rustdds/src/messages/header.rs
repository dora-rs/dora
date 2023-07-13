use speedy::{Readable, Writable};

use crate::{
  messages::{
    protocol_id::ProtocolId, protocol_version::ProtocolVersion, validity_trait::Validity,
    vendor_id::VendorId,
  },
  structure::guid::GuidPrefix,
};

#[derive(Debug, Clone, Readable, Writable, PartialEq, Eq)]
pub struct Header {
  pub protocol_id: ProtocolId,
  pub protocol_version: ProtocolVersion,
  pub vendor_id: VendorId,
  pub guid_prefix: GuidPrefix,
}

impl Header {
  pub fn new(guid: GuidPrefix) -> Self {
    Self {
      protocol_id: ProtocolId::PROTOCOL_RTPS,
      protocol_version: ProtocolVersion::THIS_IMPLEMENTATION,
      vendor_id: VendorId::THIS_IMPLEMENTATION,
      guid_prefix: guid,
    }
  }
}

impl Validity for Header {
  fn valid(&self) -> bool {
    // Three validity rules from RTPS 2.3 spec section 8.3.6.3
    // (1) We cannot reach this point if the message has too few bytes to contain a
    // full header.
    self.protocol_id == ProtocolId::PROTOCOL_RTPS // (2)
    && self.protocol_version.major <= ProtocolVersion::THIS_IMPLEMENTATION.major
    // (3)
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn header_protocol_version_major() {
    let mut header = Header::new(GuidPrefix::UNKNOWN);

    header.protocol_version = ProtocolVersion::PROTOCOLVERSION_1_0;
    assert!(header.valid());

    header.protocol_version = ProtocolVersion::THIS_IMPLEMENTATION;
    assert!(header.valid());

    header.protocol_version.major += 1;
    assert!(!header.valid());
  }

  #[test]
  fn header_protocol_id_same_as_rtps() {
    let mut header = Header::new(GuidPrefix::UNKNOWN);

    header.protocol_id = ProtocolId::PROTOCOL_RTPS;
    assert!(header.valid());
  }

  serialization_test!( type = Header,
  {
      header_with_unknown_guid_prefix,
      Header::new(GuidPrefix::UNKNOWN),
      le = [0x52, 0x54, 0x50, 0x53, // protocol_id
            0x02, 0x04,             // protocol_version
            0x01, 0x12,             // vendor_id
            0x00, 0x00, 0x00, 0x00, // guid_prefix
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00],
      be = [0x52, 0x54, 0x50, 0x53,
            0x02, 0x04,
            0x01, 0x12,
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00]
  });
}
