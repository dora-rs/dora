use speedy::{Context, Readable, Reader, Writable, Writer};
use bit_vec::BitVec;

use crate::structure::parameter_id::ParameterId;

#[derive(Debug, PartialEq, Eq, Clone)]
pub struct Parameter {
  /// Uniquely identifies the type of parameter
  pub parameter_id: ParameterId,
  /// Contains the CDR encapsulation of the Parameter type
  /// that corresponds to the specified parameterId
  pub value: Vec<u8>, /* TODO: bad field naming. E.g. "bytes" or something else that described
                       * serialized blob */
}

impl Parameter {
  pub fn new(parameter_id: ParameterId, serialized_parameter: Vec<u8>) -> Self {
    Parameter {
      parameter_id,
      value: serialized_parameter,
    }
  }

  /// Creates new parameter of type PID_STATUS_INFO.
  /// Sets flag bits to parameter : is_disposed=1 indicates that the DDS
  /// DataWriter has disposed the instance of the data-object whose Key appears
  /// in the submessage                               is_unregistered=1
  /// indicates that the DDS DataWriter has unregistered the instance of the
  /// data-object whose Key appears in the submessage                         
  /// is_filtered=1 indicates that the DDS DataWriter has written as sample for
  /// the instance of the data-object whose Key appears in the submessage but
  /// the sample did not pass the content filter specified by the DDS
  /// DataReader. The status info parameter may appear in the Data or in the
  /// DataFrag submessages for additional info look '<https://www.omg.org/spec/DDSI-RTPS/2.3/PDF>' -> 9.6.3.9 StatusInfo_t (PID_STATUS_INFO)
  pub fn create_pid_status_info_parameter(
    is_disposed: bool,
    is_unregistered: bool,
    is_filtered: bool,
  ) -> Self {
    //0...2..........8...............16..............24..............32
    //+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    //|X|X|X|X|X|X|X|X|X|X|X|X|X|X|X|X|X|X|X|X|X|X|X|X|X|X|X|X|X|F|U|D|
    //+--------------+---------------+---------------+----------------+
    // The current version of the protocol (2.3) defines the DisposedFlag, the
    // UnregisteredFlag, the FilteredFlag. DisposedFlag is represented with the
    // literal ‘D.’ UnregisteredFlag is represented with the literal ‘U.’
    // FilteredFlag is represented with the literal ‘F.’

    let mut bit_vec = BitVec::from_bytes(&[0b0000_0000]);
    bit_vec.set(7, is_disposed);
    bit_vec.set(6, is_unregistered);
    bit_vec.set(5, is_filtered);
    let bytes = bit_vec.to_bytes();
    let last_byte = bytes[0];
    Self {
      parameter_id: ParameterId::PID_STATUS_INFO,
      value: vec![0, 0, 0, last_byte],
    }
  }

  pub fn len_serialized(&self) -> usize {
    // Serialization aligns parameters to 4-byte boundaries
    // by padding at the end if necessary.
    // RTPS spec v2.5 section "9.4.2.11 ParameterList"
    let unaligned_length = self.value.len();
    let pad = if unaligned_length % 4 != 0 {
      4 - (unaligned_length % 4)
    } else {
      0
    };

    2 + // parameter_id 
    2 + // length field
    unaligned_length + // payload
    pad
  }
}

impl<'a, C: Context> Readable<'a, C> for Parameter {
  #[inline]
  fn read_from<R: Reader<'a, C>>(reader: &mut R) -> Result<Self, C::Error> {
    let parameter_id: ParameterId = reader.read_value()?;
    let length = reader.read_u16()?;

    let mut value = vec![0; length as usize];
    reader.read_bytes(&mut value)?;

    Ok(Self {
      parameter_id,
      value,
    })
  }

  #[inline]
  fn minimum_bytes_needed() -> usize {
    8
  }
}

impl<C: Context> Writable<C> for Parameter {
  #[inline]
  fn write_to<T: ?Sized + Writer<C>>(&self, writer: &mut T) -> Result<(), C::Error> {
    let length = self.value.len();
    let pad = if length % 4 != 0 { 4 - (length % 4) } else { 0 };

    writer.write_value(&self.parameter_id)?;
    writer.write_u16((length + pad) as u16)?;
    writer.write_bytes(&self.value)?;

    for _ in 0..pad {
      writer.write_u8(0x00)?;
    }

    Ok(())
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  serialization_test!( type = Parameter,
  {
      pid_protocol_version,
      Parameter {
          parameter_id: ParameterId::PID_PROTOCOL_VERSION,
          value: vec![0x02, 0x01, 0x00, 0x00],
      },
      le = [0x15, 0x00, 0x04, 0x00,
            0x02, 0x01, 0x00, 0x00],
      be = [0x00, 0x15, 0x00, 0x04,
            0x02, 0x01, 0x00, 0x00]
  },
  {
      pid_vendor_id,
      Parameter {
          parameter_id: ParameterId::PID_VENDOR_ID,
          value: vec![0x01, 0x02, 0x03, 0x04],
      },
      le = [0x16, 0x00, 0x04, 0x00,
            0x01, 0x02, 0x03, 0x04],
      be = [0x00, 0x16, 0x00, 0x04,
            0x01, 0x02, 0x03, 0x04]
  },
  {
      pid_participant_guid,
      Parameter {
          parameter_id: ParameterId::PID_PARTICIPANT_GUID,
          value: vec![0x01, 0x0F, 0xBB, 0x1D,
                      0xDF, 0x2B, 0x00, 0x00,
                      0x00, 0x00, 0x00, 0x00,
                      0x00, 0x00, 0x01, 0xC1],
      },
      le = [0x50, 0x00, 0x10, 0x00,
            0x01, 0x0F, 0xBB, 0x1D,
            0xDF, 0x2B, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x01, 0xC1],
      be = [0x00, 0x50, 0x00, 0x10,
            0x01, 0x0F, 0xBB, 0x1D,
            0xDF, 0x2B, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x01, 0xC1]
  },
  {
      pid_participant_lease_duration,
      Parameter {
          parameter_id: ParameterId::PID_PARTICIPANT_LEASE_DURATION,
          value: vec![0xFF, 0xFF, 0xFF, 0x7F,
                      0xFF, 0xFF, 0xFF, 0xFF],
      },
      le = [0x02, 0x00, 0x08, 0x00,
            0xFF, 0xFF, 0xFF, 0x7F,
            0xFF, 0xFF, 0xFF, 0xFF],
      be = [0x00, 0x02, 0x00, 0x08,
            0xFF, 0xFF, 0xFF, 0x7F,
            0xFF, 0xFF, 0xFF, 0xFF]
  });
}
