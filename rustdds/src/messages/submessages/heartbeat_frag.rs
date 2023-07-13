use speedy::{Readable, Writable};

use crate::structure::{
  guid::EntityId,
  sequence_number::{FragmentNumber, SequenceNumber},
};

/// When fragmenting data and until all fragments are available, the
/// HeartbeatFrag Submessage is sent from an RTPS Writer to an RTPS Reader to
/// communicate which fragments the Writer has available. This enables reliable
/// communication at the fragment level.
///
/// Once all fragments are available, a regular Heartbeat message is used.
#[derive(Debug, PartialEq, Eq, Clone, Readable, Writable)]
pub struct HeartbeatFrag {
  /// Identifies the Reader Entity that is being informed of the availability
  /// of fragments. Can be set to UNKNOWN to indicate all readers for
  /// the writer that sent the message.
  pub reader_id: EntityId,

  /// Identifies the Writer Entity that sent the Submessage.
  pub writer_id: EntityId,

  /// Identifies the sequence number of the data change for which fragments
  /// are available.
  pub writer_sn: SequenceNumber,

  /// All fragments up to and including this last (highest) fragment are
  /// available on the Writer for the change identified by writerSN.
  pub last_fragment_num: FragmentNumber,

  /// A counter that is incremented each time a new HeartbeatFrag message is
  /// sent. Provides the means for a Reader to detect duplicate HeartbeatFrag
  /// messages that can result from the presence of redundant communication
  /// paths.
  pub count: i32,
}

#[cfg(test)]
mod tests {
  use super::*;

  serialization_test!( type = HeartbeatFrag,
  {
      heartbeat_frag,
      HeartbeatFrag {
          reader_id: EntityId::SEDP_BUILTIN_PUBLICATIONS_READER,
          writer_id: EntityId::SEDP_BUILTIN_PUBLICATIONS_WRITER,
          writer_sn: SequenceNumber::from(42),
          last_fragment_num: FragmentNumber::from(99_u32),
          count: 6,
      },
      le = [0x00, 0x00, 0x03, 0xC7,
            0x00, 0x00, 0x03, 0xC2,
            0x00, 0x00, 0x00, 0x00,
            0x2A, 0x00, 0x00, 0x00,
            0x63, 0x00, 0x00, 0x00,
            0x06, 0x00, 0x00, 0x00],
      be = [0x00, 0x00, 0x03, 0xC7,
            0x00, 0x00, 0x03, 0xC2,
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x2A,
            0x00, 0x00, 0x00, 0x63,
            0x00, 0x00, 0x00, 0x06]
  });
}
