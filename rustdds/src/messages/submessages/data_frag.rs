use std::io;

#[allow(unused_imports)]
use log::{debug, error, info, trace, warn};
use speedy::{Context, Error, Readable, Writable, Writer};
use enumflags2::BitFlags;
use bytes::Bytes;

use crate::{
  messages::submessages::{elements::parameter_list::ParameterList, submessages::*},
  structure::{
    guid::EntityId,
    sequence_number::{FragmentNumber, SequenceNumber},
  },
};

/// The DataFrag Submessage extends the Data Submessage by enabling the
/// serializedData to be fragmented and sent as multiple DataFrag Submessages.
/// The fragments contained in the DataFrag Submessages are then re-assembled by
/// the RTPS Reader.
#[derive(Debug, PartialEq, Eq, Clone)]
#[cfg_attr(test, derive(Default))]
pub struct DataFrag {
  /// Identifies the RTPS Reader entity that is being informed of the change
  /// to the data-object.
  pub reader_id: EntityId,

  /// Identifies the RTPS Writer entity that made the change to the
  /// data-object.
  pub writer_id: EntityId,

  /// Uniquely identifies the change and the relative order for all changes
  /// made by the RTPS Writer identified by the writerGuid.
  /// Each change gets a consecutive sequence number.
  /// Each RTPS Writer maintains is own sequence number.
  pub writer_sn: SequenceNumber,

  /// Indicates the starting fragment for the series of fragments in
  /// serialized_data. Fragment numbering starts with number 1.
  pub fragment_starting_num: FragmentNumber,

  /// The number of consecutive fragments contained in this Submessage,
  /// starting at fragment_starting_num.
  pub fragments_in_submessage: u16,

  /// The total size in bytes of the original data before fragmentation.
  pub data_size: u32,

  /// The size of an individual fragment in bytes. The maximum fragment size
  /// equals 64K.
  pub fragment_size: u16,

  /// Contains QoS that may affect the interpretation of the message.
  /// Present only if the InlineQosFlag is set in the header.
  pub inline_qos: Option<ParameterList>,

  /// Encapsulation of a consecutive series of fragments, starting at
  /// fragment_starting_num for a total of fragments_in_submessage.
  /// Represents part of the new value of the data-object
  /// after the change. Present only if either the DataFlag or the KeyFlag are
  /// set in the header. Present only if DataFlag is set in the header.
  ///
  /// Note: RTPS spec says the serialized_payload is of type SerializedPayload,
  /// but that is technically incorrect. It is a fragment of
  /// SerializedPayload. The headers at the beginning of SerializedPayload
  /// appear only at the first fragment. The fragmentation mechanism here
  /// should treat serialized_payload as an opaque stream of bytes.
  pub serialized_payload: Bytes,
}

impl DataFrag {
  // Serialized length of DataFrag submessage without submessage header.
  // This is compatible with the definition of the definition of
  // "octetsToNextHeader" field in RTPS spec v2.5 Section "9.4.5.1 Submessage
  // Header".
  pub fn len_serialized(&self) -> usize {
    2 + // extraFlags (unused in RTPS v2.5)
    2 + // octetsToInlineSos
    4 + // readerId
    4 + // writerId
    8 + // writerSN
    4 + // fragmentStartingNum
    2 + // fragmentsInSubmessage
    2 + // fragmentSize
    4 + // sampleSize
    self.inline_qos.as_ref().map(|q| q.len_serialized() ).unwrap_or(0) + // QoS ParamterList
    self.serialized_payload.len()
  }

  /// Spec talks about (expected) total number of fragments.
  /// This is technically the last (Expected) fragment number, which should be
  /// the same value.
  pub fn total_number_of_fragments(&self) -> FragmentNumber {
    // RTPS spec v2.5 Section "8.3.8.3.5 Logical Interpretation" defines this as
    // follows "The total number of fragments to expect equals:
    // (dataSize / fragmentSize) + ((dataSize % fragmentSize) ? 1 : 0) "
    //
    // Note: The above formula is a bit suspect, since fragmentSize == 0 seems to
    // be allowed by the spec.

    // This is a integer division with rounding up.
    let frag_size = self.fragment_size as u32;
    if frag_size < 1 {
      FragmentNumber::INVALID
    } else {
      FragmentNumber::new((self.data_size / frag_size) + u32::from(self.data_size % frag_size > 0))
      // TODO: Use self.data_size.div_ceil(frag_size) from core::num
      // instead of the above when it is available in stable Rust.
      //
      // Note: Alternative solution
      // (data_size + frag_size - 1) / frag_size
      // will overflow for large values of data_size and frag_size, unless
      // promoted to u64 arithmetic
    }
  }

  pub fn deserialize(buffer: &Bytes, flags: BitFlags<DATAFRAG_Flags>) -> io::Result<Self> {
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
    let writer_sn = SequenceNumber::read_from_stream_unbuffered_with_ctx(endianness, &mut cursor)
      .map_err(map_speedy_err)?;
    let fragment_starting_num =
      FragmentNumber::read_from_stream_unbuffered_with_ctx(endianness, &mut cursor)
        .map_err(map_speedy_err)?;
    let fragments_in_submessage =
      u16::read_from_stream_unbuffered_with_ctx(endianness, &mut cursor).map_err(map_speedy_err)?;
    let fragment_size =
      u16::read_from_stream_unbuffered_with_ctx(endianness, &mut cursor).map_err(map_speedy_err)?;
    let data_size =
      u32::read_from_stream_unbuffered_with_ctx(endianness, &mut cursor).map_err(map_speedy_err)?;

    let expect_qos = flags.contains(DATAFRAG_Flags::InlineQos);
    //let expect_key = flags.contains(DATAFRAG_Flags::Key);

    // Size of header after "octets_to_inline_qos" field:
    // reader_id: 4
    // writer_id: 4
    // writer_sn: 8
    // fragment_starting_num: 4
    // fragments_in_submessage: 2
    // fragment_size: 2
    // data_size: 4
    //
    // Total: 28

    // Skip any possible fields we do not know about.
    let rtps_v25_header_size: u16 = 28;
    if octets_to_inline_qos < rtps_v25_header_size {
      return Err(io::Error::new(
        io::ErrorKind::Other,
        format!(
          "DataFrag has too low octetsToInlineQos = {}",
          octets_to_inline_qos
        ),
      ));
    }
    // condition to avoid subtract overflow
    if octets_to_inline_qos > rtps_v25_header_size {
      let extra_octets = octets_to_inline_qos - rtps_v25_header_size;
      cursor.set_position(cursor.position() + u64::from(extra_octets));

      if cursor.position() > buffer.len().try_into().unwrap() {
        // octets_to_inline_qos told us to skip past the end of the message.
        // This is a malformed message.
        return Err(io::Error::new(
          io::ErrorKind::InvalidData,
          format!(
            "DATAFRAG submessage octets_to_inline_qos points to byte {}, but message len={}.",
            cursor.position(),
            buffer.len()
          ),
        ));
      }
    }

    let inline_qos = if expect_qos {
      Some(
        ParameterList::read_from_stream_unbuffered_with_ctx(endianness, &mut cursor)
          .map_err(map_speedy_err)?,
      )
    } else {
      None
    };

    // Validity checks from RTPS spec v2.5 Section 8.3.8.3.3 "Validity"

    // writer_sn strictly positive
    if writer_sn < SequenceNumber::new(1) {
      return Err(io::Error::new(
        io::ErrorKind::Other,
        "DataFrag SequenceNumber < 1. Discarding as invalid.",
      ));
    }

    // Fragment size == 0 is not strictly forbidden in the RTPS spec, but
    // it smells so fishy that we just drop it to avoid confusion later.
    // TODO:
    // Is there any valid use case for sending zero-sized fragments?
    // Sending zero-sized data may be ok, but it could be sent as zero fragments of
    // some positive size, or preferably as non-fragmented DATA submessage.
    if fragment_size < 1 || (fragment_size as u32) > data_size {
      return Err(io::Error::new(
        io::ErrorKind::Other,
        format!("Invalid DataFrag. fragment_size={} data_size={}  Expected 1 <= fragment_size <= data_size.",
          fragment_size, data_size),
      ));
    }

    // Payload should be always present, be it data or key fragments.
    let serialized_payload = buffer.clone().split_off(cursor.position() as usize);

    let datafrag = Self {
      reader_id,
      writer_id,
      writer_sn,
      fragment_starting_num,
      fragments_in_submessage,
      data_size,
      fragment_size,
      inline_qos,
      serialized_payload,
    };

    // fragment_starting_num strictly positive and must not exceed total number of
    // fragments

    let expected_total = datafrag.total_number_of_fragments();
    if fragment_starting_num < FragmentNumber::new(1) || fragment_starting_num > expected_total {
      return Err(io::Error::new(
        io::ErrorKind::Other,
        format!("DataFrag fragmentStartingNum={:?} expected_total={:?}.  Expected 1 <= fragmentStartingNum <= expeceted_total.  Discarding as invalid.",
          fragment_starting_num,expected_total)
      ));
    }

    if datafrag.serialized_payload.len()
      > (fragments_in_submessage as usize) * (fragment_size as usize)
    {
      return Err(io::Error::new(
        io::ErrorKind::Other,
        format!("Invalid DataFrag. serializedData length={} should be less than or equal to (fragments_in_submessage={}) x (fragment_size={})",
          datafrag.serialized_payload.len(), fragments_in_submessage, fragment_size)
      ));
    }

    Ok(datafrag)
  }
}

impl<C: Context> Writable<C> for DataFrag {
  fn write_to<T: ?Sized + Writer<C>>(&self, writer: &mut T) -> Result<(), C::Error> {
    writer.write_u16(0)?; // extraflags
    writer.write_u16(28)?; // See calculation of this value in deserialization above.
                           // We always write constant 28 here, because this implementation does not (yet)
                           // write any fields between sampleSize ( = data_size)
                           // and inline QoS. If some future protocol version adds fields there, then this
                           // must be changed.
    writer.write_value(&self.reader_id)?;
    writer.write_value(&self.writer_id)?;
    writer.write_value(&self.writer_sn)?;
    writer.write_value(&self.fragment_starting_num)?;
    writer.write_value(&self.fragments_in_submessage)?;
    writer.write_value(&self.fragment_size)?;
    writer.write_value(&self.data_size)?;
    if self.inline_qos.is_some() && !self.inline_qos.as_ref().unwrap().parameters.is_empty() {
      writer.write_value(&self.inline_qos)?;
    }
    writer.write_bytes(&self.serialized_payload)?;
    Ok(())
  }
}
