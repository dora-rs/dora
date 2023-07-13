use speedy::{Context, Writable, Writer};

use crate::messages::submessages::{
  submessage::{ReaderSubmessage, SecuritySubmessage, WriterSubmessage},
  submessage_header::SubmessageHeader,
  submessages::InterpreterSubmessage,
};

#[derive(Debug, PartialEq, Eq, Clone)]
pub struct Submessage {
  pub header: SubmessageHeader,
  pub body: SubmessageBody,
}

/// See section 7.3.1 of the Security specification (v. 1.1)
// TODO: Submessages should implement some Length trait that returns the length
// of Submessage in bytes. This is needed for Submessage construction.
#[derive(Debug, PartialEq, Eq, Clone)]
pub enum SubmessageBody {
  Writer(WriterSubmessage),
  Reader(ReaderSubmessage),
  Security(SecuritySubmessage),
  Interpreter(InterpreterSubmessage),
}

impl<C: Context> Writable<C> for Submessage {
  fn write_to<T: ?Sized + Writer<C>>(&self, writer: &mut T) -> Result<(), C::Error> {
    writer.write_value(&self.header)?;
    match &self.body {
      SubmessageBody::Writer(m) => writer.write_value(&m),
      SubmessageBody::Reader(m) => writer.write_value(&m),
      SubmessageBody::Interpreter(m) => writer.write_value(&m),
      SubmessageBody::Security(m) => writer.write_value(&m),
    }
  }
}

#[cfg(test)]
mod tests {
  use bytes::Bytes;
  use enumflags2::BitFlags;
  use log::info;
  use speedy::{Readable, Writable};

  use super::Submessage;
  use crate::{messages::submessages::submessages::*, rtps::submessage::*};

  #[test]
  fn submessage_data_submessage_deserialization() {
    // this is wireshark captured shapesdemo data_submessage
    let serialized_data_submessage = Bytes::from_static(&[
      0x15, 0x05, 0x2c, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x01,
      0x02, 0x00, 0x00, 0x00, 0x00, 0x5b, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x04, 0x00,
      0x00, 0x00, 0x52, 0x45, 0x44, 0x00, 0x69, 0x00, 0x00, 0x00, 0x17, 0x00, 0x00, 0x00, 0x1e,
      0x00, 0x00, 0x00,
    ]);

    let header = SubmessageHeader::read_from_buffer(&serialized_data_submessage[0..4])
      .expect("could not create submessage header");
    let flags = BitFlags::<DATA_Flags>::from_bits_truncate(header.flags);
    let suba = Data::deserialize_data(&serialized_data_submessage.slice(4..), flags)
      .expect("DATA deserialization failed.");
    let sub = Submessage {
      header,
      body: SubmessageBody::Writer(WriterSubmessage::Data(suba, flags)),
    };
    info!("{:?}", sub);

    let message_buffer = sub.write_to_vec().expect("DATA serialization failed");

    assert_eq!(serialized_data_submessage, message_buffer);
  }

  #[test]
  fn submessage_hearbeat_deserialization() {
    let serialized_heartbeat_message: Vec<u8> = vec![
      0x07, 0x01, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x01, 0x02, 0x00, 0x00, 0x00,
      0x00, 0x5b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5b, 0x00, 0x00, 0x00, 0x1f, 0x00,
      0x00, 0x00,
    ];

    let header = SubmessageHeader::read_from_buffer(&serialized_heartbeat_message[0..4])
      .expect("could not create submessage header");
    let flags = BitFlags::<HEARTBEAT_Flags>::from_bits_truncate(header.flags);
    let e = endianness_flag(header.flags);
    let suba = Heartbeat::read_from_buffer_with_ctx(e, &serialized_heartbeat_message[4..])
      .expect("deserialization failed.");
    let sub = Submessage {
      header,
      body: SubmessageBody::Writer(WriterSubmessage::Heartbeat(suba, flags)),
    };
    info!("{:?}", sub);

    let message_buffer = sub.write_to_vec().expect("serialization failed");

    assert_eq!(serialized_heartbeat_message, message_buffer);
  }

  #[test]
  fn submessage_info_dst_deserialization() {
    let serialized_info_dst_message: Vec<u8> = vec![
      0x0e, 0x01, 0x0c, 0x00, 0x01, 0x03, 0x00, 0x0c, 0x29, 0x2d, 0x31, 0xa2, 0x28, 0x20, 0x02,
      0x08,
    ];

    let header = SubmessageHeader::read_from_buffer(&serialized_info_dst_message[0..4])
      .expect("could not create submessage header");
    let flags = BitFlags::<INFODESTINATION_Flags>::from_bits_truncate(header.flags);
    let e = endianness_flag(header.flags);
    let suba = InfoDestination::read_from_buffer_with_ctx(e, &serialized_info_dst_message[4..])
      .expect("deserialization failed.");
    let sub = Submessage {
      header,
      body: SubmessageBody::Interpreter(InterpreterSubmessage::InfoDestination(suba, flags)),
    };
    info!("{:?}", sub);

    let message_buffer = sub.write_to_vec().expect("serialization failed");

    assert_eq!(serialized_info_dst_message, message_buffer);
  }

  // #[test]
  // fn submessage_info_ts_deserialization() {
  //   let serializedInfoTSMessage: Vec<u8> = vec![
  //     0x09, 0x01, 0x08, 0x00, 0x1a, 0x15, 0xf3, 0x5e, 0x00, 0xcc, 0xfb, 0x13,
  //   ];
  //   let header =
  // SubmessageHeader::read_from_buffer(&serializedInfoTSMessage[0..4])
  //     .expect("could not create submessage header");
  //   let flags =
  // BitFlags::<INFOTIMESTAMP_Flags>::from_bits_truncate(header.flags);
  //   let e = endianness_flag(header.flags);
  //   let suba = InfoTimestamp::read_from_buffer_with_ctx(e,
  // &serializedInfoTSMessage[4..])     .expect("deserialization failed.");
  //   let sub = SubMessage {
  //     header,
  //     body: SubmessageBody::Interpreter(InterpreterSubmessage::
  // InfoTimestamp(suba, flags)),   };
  //   info!("{:?}", sub);

  //   let message_buffer = sub.write_to_vec().expect("serialization failed");

  //   assert_eq!(serializedInfoTSMessage, message_buffer);
  // }
}
