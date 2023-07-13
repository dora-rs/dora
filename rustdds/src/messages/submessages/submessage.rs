use speedy::{Context, Writable, Writer};
use enumflags2::BitFlags;

use crate::{
  messages::submessages::{
    ack_nack::AckNack, data::Data, data_frag::DataFrag, gap::Gap, heartbeat::Heartbeat,
    heartbeat_frag::HeartbeatFrag, info_destination::InfoDestination, info_reply::InfoReply,
    info_source::InfoSource, info_timestamp::InfoTimestamp, nack_frag::NackFrag,
    submessage_flag::*,
  },
  structure::guid::EntityId,
};
use super::{
  secure_body::SecureBody, secure_postfix::SecurePostfix, secure_prefix::SecurePrefix,
  secure_rtps_postfix::SecureRTPSPostfix, secure_rtps_prefix::SecureRTPSPrefix,
};

// TODO: These messages are structured a bit oddly. Why is flags separate from
// the submessage proper?

#[derive(Debug, PartialEq, Eq, Clone)]
pub enum WriterSubmessage {
  Data(Data, BitFlags<DATA_Flags>),
  DataFrag(DataFrag, BitFlags<DATAFRAG_Flags>),
  Gap(Gap, BitFlags<GAP_Flags>),
  Heartbeat(Heartbeat, BitFlags<HEARTBEAT_Flags>),
  #[allow(dead_code)] // Functionality not yet implemented
  HeartbeatFrag(HeartbeatFrag, BitFlags<HEARTBEATFRAG_Flags>),
}

// we must write this manually, because
// 1) we cannot implement Writable for *Flags defined using enumflags2, as they
// are foreign types (coherence rules) 2) Writer should not use any enum variant
// tag in this type, as we have SubmessageHeader already.
impl<C: Context> Writable<C> for WriterSubmessage {
  fn write_to<T: ?Sized + Writer<C>>(&self, writer: &mut T) -> Result<(), C::Error> {
    match self {
      WriterSubmessage::Data(s, _f) => writer.write_value(s),
      WriterSubmessage::DataFrag(s, _f) => writer.write_value(s),
      WriterSubmessage::Gap(s, _f) => writer.write_value(s),
      WriterSubmessage::Heartbeat(s, _f) => writer.write_value(s),
      WriterSubmessage::HeartbeatFrag(s, _f) => writer.write_value(s),
    }
  }
}

#[derive(Debug, PartialEq, Eq, Clone)]
pub enum ReaderSubmessage {
  AckNack(AckNack, BitFlags<ACKNACK_Flags>),
  NackFrag(NackFrag, BitFlags<NACKFRAG_Flags>),
}

// we must write this manually, because
// 1) we cannot implement Writable for *Flags defined using enumflags2, as they
// are foreign types (coherence rules) 2) Writer should not use any enum variant
// tag in this type, as we have SubmessageHeader already.
impl<C: Context> Writable<C> for ReaderSubmessage {
  fn write_to<T: ?Sized + Writer<C>>(&self, writer: &mut T) -> Result<(), C::Error> {
    match self {
      ReaderSubmessage::AckNack(s, _f) => writer.write_value(s),
      ReaderSubmessage::NackFrag(s, _f) => writer.write_value(s),
    }
  }
}

/// New submessage types: section 7.3.6 of the Security specification (v. 1.1)
#[derive(Debug, PartialEq, Eq, Clone)]
#[allow(clippy::enum_variant_names)] // We are using variant names from the spec
pub enum SecuritySubmessage {
  SecureBody(SecureBody, BitFlags<SECUREBODY_Flags>),
  SecurePrefix(SecurePrefix, BitFlags<SECUREPREFIX_Flags>),
  SecurePostfix(SecurePostfix, BitFlags<SECUREPOSTFIX_Flags>),
  SecureRTPSPrefix(SecureRTPSPrefix, BitFlags<SECURERTPSPREFIX_Flags>),
  SecureRTPSPostfix(SecureRTPSPostfix, BitFlags<SECURERTPSPOSTFIX_Flags>),
}

// we must write this manually, because
// 1) we cannot implement Writable for *Flags defined using enumflags2, as they
// are foreign types (coherence rules) 2) Writer should not use any enum variant
// tag in this type, as we have SubmessageHeader already.
impl<C: Context> Writable<C> for SecuritySubmessage {
  fn write_to<T: ?Sized + Writer<C>>(&self, writer: &mut T) -> Result<(), C::Error> {
    match self {
      SecuritySubmessage::SecureBody(s, _f) => writer.write_value(s),
      SecuritySubmessage::SecurePrefix(s, _f) => writer.write_value(s),
      SecuritySubmessage::SecurePostfix(s, _f) => writer.write_value(s),
      SecuritySubmessage::SecureRTPSPrefix(s, _f) => writer.write_value(s),
      SecuritySubmessage::SecureRTPSPostfix(s, _f) => writer.write_value(s),
    }
  }
}

#[derive(Debug, PartialEq, Eq, Clone)]
#[allow(clippy::enum_variant_names)]
pub enum InterpreterSubmessage {
  InfoSource(InfoSource, BitFlags<INFOSOURCE_Flags>),
  InfoDestination(InfoDestination, BitFlags<INFODESTINATION_Flags>),
  InfoReply(InfoReply, BitFlags<INFOREPLY_Flags>),
  InfoTimestamp(InfoTimestamp, BitFlags<INFOTIMESTAMP_Flags>),
  // Pad(Pad), // Pad message does not need to be processed above serialization layer
}

// See notes on impl Writer for EntitySubmessage
impl<C: Context> Writable<C> for InterpreterSubmessage {
  fn write_to<T: ?Sized + Writer<C>>(&self, writer: &mut T) -> Result<(), C::Error> {
    match self {
      InterpreterSubmessage::InfoSource(s, _f) => writer.write_value(s),
      InterpreterSubmessage::InfoDestination(s, _f) => writer.write_value(s),
      InterpreterSubmessage::InfoReply(s, _f) => writer.write_value(s),
      InterpreterSubmessage::InfoTimestamp(s, _f) => match s {
        InfoTimestamp { timestamp: None } => Ok(()), // serialization is empty string
        InfoTimestamp {
          timestamp: Some(ts),
        } => writer.write_value(ts),
      },
    }
  }
}

#[derive(Debug)]
pub enum AckSubmessage {
  AckNack(AckNack),
  #[allow(dead_code)] // Functionality not yet implemented
  NackFrag(NackFrag),
}

impl AckSubmessage {
  pub fn writer_id(&self) -> EntityId {
    match self {
      AckSubmessage::AckNack(a) => a.writer_id,
      AckSubmessage::NackFrag(a) => a.writer_id,
    }
  }
}
