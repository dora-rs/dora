use speedy::{Readable, Writable};

use crate::{
  messages::{protocol_version::ProtocolVersion, vendor_id::VendorId},
  structure::guid::GuidPrefix,
};

/// This message modifies the logical source of the Submessages
/// that follow.
#[derive(Debug, PartialEq, Eq, Clone, Readable, Writable)]
pub struct InfoSource {
  /// Indicates the protocol used to encapsulate subsequent Submessages
  pub protocol_version: ProtocolVersion,

  /// Indicates the VendorId of the vendor that
  /// encapsulated subsequent Submessage
  pub vendor_id: VendorId,

  /// Identifies the Participant that is the container of the RTPS Writer
  /// entities that are the source of the Submessages that follow
  pub guid_prefix: GuidPrefix,
}

#[cfg(test)]
mod tests {
  use super::*;

  serialization_test!( type = InfoSource,
  {
      info_source,
      InfoSource {
          protocol_version: ProtocolVersion::PROTOCOLVERSION_2_2,
          vendor_id: VendorId {
              vendor_id: [0xFF, 0xAA]
          },
          guid_prefix: GuidPrefix {
              bytes: [0x01, 0x02, 0x6D, 0x3F,
                          0x7E, 0x07, 0x00, 0x00,
                          0x01, 0x00, 0x00, 0x00]
          }
      },
      le = [0x02, 0x02, 0xFF, 0xAA,
            0x01, 0x02, 0x6D, 0x3F,
            0x7E, 0x07, 0x00, 0x00,
            0x01, 0x00, 0x00, 0x00],
      be = [0x02, 0x02, 0xFF, 0xAA,
            0x01, 0x02, 0x6D, 0x3F,
            0x7E, 0x07, 0x00, 0x00,
            0x01, 0x00, 0x00, 0x00]
  });
}
