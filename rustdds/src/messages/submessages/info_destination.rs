use enumflags2::BitFlags;
use speedy::{Readable, Writable};
#[allow(unused_imports)]
use log::error;

use crate::{
  messages::submessages::submessages::SubmessageHeader,
  rtps::{Submessage, SubmessageBody},
  structure::guid::GuidPrefix,
};
use super::{
  submessage::InterpreterSubmessage, submessage_flag::INFODESTINATION_Flags,
  submessage_kind::SubmessageKind,
};

/// This message is sent from an RTPS Writer to an RTPS Reader
/// to modify the GuidPrefix used to interpret the Reader entity_ids
/// appearing in the Submessages that follow it.
#[derive(Debug, PartialEq, Eq, Clone, Readable, Writable)]
pub struct InfoDestination {
  /// Provides the GuidPrefix that should be used to reconstruct the GUIDs
  /// of all the RTPS Reader entities whose EntityIds appears
  /// in the Submessages that follow.
  pub guid_prefix: GuidPrefix,
}

impl InfoDestination {
  pub fn len_serialized(&self) -> usize {
    std::mem::size_of::<GuidPrefix>()
  }

  pub fn create_submessage(self, flags: BitFlags<INFODESTINATION_Flags>) -> Submessage {
    Submessage {
      header: SubmessageHeader {
        kind: SubmessageKind::INFO_DST,
        flags: flags.bits(),
        content_length: self.len_serialized() as u16,
      },
      body: SubmessageBody::Interpreter(InterpreterSubmessage::InfoDestination(self, flags)),
    }
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  serialization_test!( type = InfoDestination,
  {
      info_destination,
      InfoDestination {
          guid_prefix: GuidPrefix {
              bytes: [0x01, 0x02, 0x6D, 0x3F,
                          0x7E, 0x07, 0x00, 0x00,
                          0x01, 0x00, 0x00, 0x00]
          }
      },
      le = [0x01, 0x02, 0x6D, 0x3F,
            0x7E, 0x07, 0x00, 0x00,
            0x01, 0x00, 0x00, 0x00],
      be = [0x01, 0x02, 0x6D, 0x3F,
            0x7E, 0x07, 0x00, 0x00,
            0x01, 0x00, 0x00, 0x00]
  });
}
