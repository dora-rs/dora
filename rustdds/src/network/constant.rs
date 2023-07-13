use mio_06::Token;
use mio_extras::channel as mio_channel;

use crate::{
  discovery::sedp_messages::{DiscoveredReaderData, DiscoveredWriterData},
  structure::guid::{EntityKind, GuidPrefix, GUID},
};

const PTB: usize = EntityKind::POLL_TOKEN_BASE;

pub const STOP_POLL_TOKEN: Token = Token(PTB);

// pub const DISCOVERY_SENDER_TOKEN: Token = Token(1 + PTB);
// pub const USER_TRAFFIC_SENDER_TOKEN: Token = Token(2 + PTB);

// pub const DATA_SEND_TOKEN: Token = Token(5 + PTB);

pub const DISCOVERY_LISTENER_TOKEN: Token = Token(6 + PTB);
pub const DISCOVERY_MUL_LISTENER_TOKEN: Token = Token(7 + PTB);
pub const USER_TRAFFIC_LISTENER_TOKEN: Token = Token(8 + PTB);
pub const USER_TRAFFIC_MUL_LISTENER_TOKEN: Token = Token(9 + PTB);

pub const ADD_READER_TOKEN: Token = Token(10 + PTB);
pub const REMOVE_READER_TOKEN: Token = Token(11 + PTB);

// pub const READER_CHANGE_TOKEN: Token = Token(12 + PTB);
// pub const DATAREADER_CHANGE_TOKEN: Token = Token(13 + PTB);

// pub const ADD_DATAREADER_TOKEN: Token = Token(14 + PTB);
// pub const REMOVE_DATAREADER_TOKEN: Token = Token(15 + PTB);

pub const ADD_WRITER_TOKEN: Token = Token(16 + PTB);
pub const REMOVE_WRITER_TOKEN: Token = Token(17 + PTB);

// pub const ADD_DATAWRITER_TOKEN: Token = Token(18 + PTB);
// pub const REMOVE_DATAWRITER_TOKEN: Token = Token(19 + PTB);

pub const ACKNACK_MESSAGE_TO_LOCAL_WRITER_TOKEN: Token = Token(20 + PTB);

pub const DISCOVERY_UPDATE_NOTIFICATION_TOKEN: Token = Token(21 + PTB);
pub const DISCOVERY_COMMAND_TOKEN: Token = Token(22 + PTB);
pub const SPDP_LIVENESS_TOKEN: Token = Token(23 + PTB);

pub const DISCOVERY_PARTICIPANT_DATA_TOKEN: Token = Token(30 + PTB);
pub const DISCOVERY_PARTICIPANT_CLEANUP_TOKEN: Token = Token(31 + PTB);
pub const DISCOVERY_SEND_PARTICIPANT_INFO_TOKEN: Token = Token(32 + PTB);
pub const DISCOVERY_READER_DATA_TOKEN: Token = Token(33 + PTB);
pub const DISCOVERY_SEND_READERS_INFO_TOKEN: Token = Token(34 + PTB);
pub const DISCOVERY_WRITER_DATA_TOKEN: Token = Token(35 + PTB);
pub const DISCOVERY_SEND_WRITERS_INFO_TOKEN: Token = Token(36 + PTB);
pub const DISCOVERY_TOPIC_DATA_TOKEN: Token = Token(37 + PTB);
pub const DISCOVERY_TOPIC_CLEANUP_TOKEN: Token = Token(38 + PTB);
pub const DISCOVERY_SEND_TOPIC_INFO_TOKEN: Token = Token(39 + PTB);
pub const DISCOVERY_PARTICIPANT_MESSAGE_TOKEN: Token = Token(40 + PTB);
pub const DISCOVERY_PARTICIPANT_MESSAGE_TIMER_TOKEN: Token = Token(41 + PTB);

pub const DPEV_ACKNACK_TIMER_TOKEN: Token = Token(50 + PTB);

pub struct TokenReceiverPair<T> {
  pub token: Token,
  pub receiver: mio_channel::Receiver<T>,
}

// These constants are from RTPS spec Section 9.6.2.3 Default Port Numbers
const PB: u16 = 7400;
const DG: u16 = 250;
const PG: u16 = 2;

const D0: u16 = 0;
const D1: u16 = 10;
const D2: u16 = 1;
const D3: u16 = 11;

pub fn spdp_well_known_multicast_port(domain_id: u16) -> u16 {
  PB + DG * domain_id + D0
}

pub fn spdp_well_known_unicast_port(domain_id: u16, participant_id: u16) -> u16 {
  PB + DG * domain_id + D1 + PG * participant_id
}

pub fn user_traffic_multicast_port(domain_id: u16) -> u16 {
  PB + DG * domain_id + D2
}

pub fn user_traffic_unicast_port(domain_id: u16, participant_id: u16) -> u16 {
  PB + DG * domain_id + D3 + PG * participant_id
}

#[derive(Debug)]
#[allow(clippy::large_enum_variant)]
pub(crate) enum DiscoveryNotificationType {
  ReaderUpdated {
    discovered_reader_data: DiscoveredReaderData,
  },
  ReaderLost {
    reader_guid: GUID,
  },
  WriterUpdated {
    discovered_writer_data: DiscoveredWriterData,
  },
  WriterLost {
    writer_guid: GUID,
  },
  ParticipantUpdated {
    guid_prefix: GuidPrefix,
  },
  ParticipantLost {
    guid_prefix: GuidPrefix,
  },
  AssertTopicLiveliness {
    writer_guid: GUID,
    manual_assertion: bool,
  },
}
