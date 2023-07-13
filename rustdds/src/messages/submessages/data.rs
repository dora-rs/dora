use std::io;

use bytes::Bytes;
use speedy::{Context, Error, Readable, Writable, Writer};
use enumflags2::BitFlags;

use crate::{
  messages::submessages::{
    elements::{parameter_list::ParameterList, serialized_payload::SerializedPayload},
    submessages::*,
  },
  structure::{guid::EntityId, sequence_number::SequenceNumber},
};
//use log::debug;

/// This Submessage is sent from an RTPS Writer (NO_KEY or WITH_KEY)
/// to an RTPS Reader (NO_KEY or WITH_KEY)
///
/// The Submessage notifies the RTPS Reader of a change to
/// a data-object belonging to the RTPS Writer. The possible changes
/// include both changes in value as well as changes to the lifecycle
/// of the data-object.
#[derive(Debug, PartialEq, Eq, Clone)]
pub struct Data {
  /// Identifies the RTPS Reader entity that is being informed of the change
  /// to the data-object.
  pub reader_id: EntityId,

  /// Identifies the RTPS Writer entity that made the change to the
  /// data-object.
  pub writer_id: EntityId,

  /// Uniquely identifies the change and the relative order for all changes
  /// made by the RTPS Writer identified by the writerGuid. Each change
  /// gets a consecutive sequence number. Each RTPS Writer maintains is
  /// own sequence number.
  pub writer_sn: SequenceNumber,

  /// Contains QoS that may affect the interpretation of the message.
  /// Present only if the InlineQosFlag is set in the header.
  pub inline_qos: Option<ParameterList>,

  /// If the DataFlag is set, then it contains the encapsulation of
  /// the new value of the data-object after the change.
  /// If the KeyFlag is set, then it contains the encapsulation of
  /// the key of the data-object the message refers to.
  pub serialized_payload: Option<SerializedPayload>,
}

impl Data {
  /// DATA submessage cannot be speedy Readable because deserializing this
  /// requires info from submessage header. Required information is  expect_qos
  /// and expect_payload, which are told on submessage headerflags.

  pub fn deserialize_data(buffer: &Bytes, flags: BitFlags<DATA_Flags>) -> io::Result<Self> {
    let mut cursor = io::Cursor::new(&buffer);
    let endianness = endianness_flag(flags.bits());
    let map_speedy_err = |p: Error| io::Error::new(io::ErrorKind::Other, p);

    let _extra_flags =
      u16::read_from_stream_unbuffered_with_ctx(endianness, &mut cursor).map_err(map_speedy_err)?;
    let octets_to_inline_qos =
      u16::read_from_stream_unbuffered_with_ctx(endianness, &mut cursor).map_err(map_speedy_err)?;
    let reader_id = EntityId::read_from_stream_unbuffered_with_ctx(endianness, &mut cursor)
      .map_err(map_speedy_err)?;
    let writer_id = EntityId::read_from_stream_unbuffered_with_ctx(endianness, &mut cursor)
      .map_err(map_speedy_err)?;
    let sequence_number =
      SequenceNumber::read_from_stream_unbuffered_with_ctx(endianness, &mut cursor)
        .map_err(map_speedy_err)?;

    let expect_qos = flags.contains(DATA_Flags::InlineQos);
    let expect_data = flags.contains(DATA_Flags::Data) || flags.contains(DATA_Flags::Key);

    // size of DATA-specific header above is
    // extraFlags (2) + octetsToInlineQos (2) + readerId (4) + writerId (4) +
    // writerSN (8) = 20 bytes
    // of which 16 bytes is after octetsToInlineQos field.
    let rtps_v23_data_header_size: u16 = 16;
    // ... and octets_to_inline_qos must be at least this much, or otherwise inline
    // Qos (or in case it is absent, the following SerializedPayload) would
    // overlap with the rtps_v23_data_header fields (readerId, writerId, and
    // writerSN).
    if octets_to_inline_qos < rtps_v23_data_header_size {
      return Err(io::Error::new(
        io::ErrorKind::InvalidData,
        format!("DATA submessage has invalid octets_to_inline_qos={octets_to_inline_qos}."),
      ));
    }

    // We need to check to avoid subtract overflow
    // https://github.com/jhelovuo/RustDDS/issues/277
    if octets_to_inline_qos > rtps_v23_data_header_size {
      let extra_octets = octets_to_inline_qos - rtps_v23_data_header_size;
      // There may be some extra data between writerSN and inlineQos, if the header is
      // extended in future versions. But as of RTPS v2.3 , extra_octets should be
      // always zero.

      // Nevertheless, skip over that extra data, if we are told such exists.
      cursor.set_position(cursor.position() + u64::from(extra_octets));

      if cursor.position() > buffer.len().try_into().unwrap() {
        // octets_to_inline_qos told us to skip past the end of the message.
        // This is a malformed message.
        return Err(io::Error::new(
          io::ErrorKind::InvalidData,
          format!(
            "DATA submessage octets_to_inline_qos points to byte {}, but message len={}.",
            cursor.position(),
            buffer.len()
          ),
        ));
      }
    }

    // read the inline Qos
    let parameter_list = if expect_qos {
      Some(
        ParameterList::read_from_stream_unbuffered_with_ctx(endianness, &mut cursor)
          .map_err(map_speedy_err)?,
      )
    } else {
      None
    };

    let payload = if expect_data {
      let p = SerializedPayload::from_bytes(&buffer.clone().split_off(cursor.position() as usize))?;
      Some(p)
    } else {
      None
    };

    Ok(Self {
      reader_id,
      writer_id,
      writer_sn: sequence_number,
      inline_qos: parameter_list,
      serialized_payload: payload,
    })
  }

  // Serialized length of Data submessage without submessage header.
  // This is compatible with the definition of the definition of
  // "octetsToNextHeader" field in RTPS spec v2.5 Section "9.4.5.1 Submessage
  // Header".
  pub fn len_serialized(&self) -> usize {
    2 + // extraFlags
    2 + // octetsToInlineSos
    4 + // readerId
    4 + // writerId
    8 + // writerSN
    self.inline_qos.as_ref().map(|q| q.len_serialized() ).unwrap_or(0) + // QoS ParamterList
    self.serialized_payload.as_ref().map(|q| q.len_serialized()).unwrap_or(0)
  }
}

impl<C: Context> Writable<C> for Data {
  fn write_to<T: ?Sized + Writer<C>>(&self, writer: &mut T) -> Result<(), C::Error> {
    //This version of the protocol (2.3) should set all the bits in the extraFlags
    // to zero
    writer.write_u16(0)?;
    // The octetsToInlineQos field contains the number of octets starting from the
    // first octet immediately following this field until the first octet of the
    // inlineQos SubmessageElement. If the inlineQos SubmessageElement is not
    // present (i.e., the InlineQosFlag is not set), then octetsToInlineQos contains
    // the offset to the next field after the inlineQos.
    writer.write_u16(16)?;

    writer.write_value(&self.reader_id)?;
    writer.write_value(&self.writer_id)?;
    writer.write_value(&self.writer_sn)?;
    if let Some(inline_qos) = self.inline_qos.as_ref() {
      writer.write_value(inline_qos)?;
    }

    if let Some(serialized_payload) = self.serialized_payload.as_ref() {
      writer.write_value(serialized_payload)?;
    }

    Ok(())
  }
}
