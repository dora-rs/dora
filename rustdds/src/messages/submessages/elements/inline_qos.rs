use enumflags2::{bitflags, BitFlags};
#[cfg(test)]
use byteorder::ByteOrder;
use speedy::{Context, Endianness, Readable, Writable, Writer};
use serde::{Deserialize, Serialize};
#[allow(unused_imports)]
use log::{debug, error, info, trace, warn};

use crate::{
  dds::key::KeyHash,
  messages::submessages::elements::{parameter_list::ParameterList, RepresentationIdentifier},
  serialization::{pl_cdr_adapters::PlCdrDeserializeError, speedy_pl_cdr_helpers::*},
  structure::{cache_change::ChangeKind, parameter_id::ParameterId, rpc::SampleIdentity},
};
#[cfg(test)]
use crate::{
  dds::adapters::no_key::*, serialization, serialization::cdr_serializer::to_bytes,
  serialization::CDRDeserializerAdapter,
};

// Utility for parsing RTPS inlineQoS parameters
// TODO: This does not need to be a struct, since is has no contents.
// Maybe someone has had an overdose of object-orientation?
// Some standalone functions should suffice.
pub(crate) struct InlineQos {}

impl InlineQos {
  pub fn status_info(
    params: &ParameterList,
    rep_id: RepresentationIdentifier,
  ) -> std::result::Result<StatusInfo, PlCdrDeserializeError> {
    let status_info = params
      .parameters
      .iter()
      .find(|p| p.parameter_id == ParameterId::PID_STATUS_INFO);
    let ctx = pl_cdr_rep_id_to_speedy_d(rep_id)?;

    let status_info = match status_info {
      Some(p) => StatusInfo::read_from_buffer_with_ctx(ctx, &p.value)?,
      None => StatusInfo::empty(),
    };

    Ok(status_info)
  }

  pub fn key_hash(params: &ParameterList) -> Result<Option<KeyHash>, PlCdrDeserializeError> {
    let key_hash = params
      .parameters
      .iter()
      .find(|p| p.parameter_id == ParameterId::PID_KEY_HASH);
    Ok(match key_hash {
      Some(p) => Some(KeyHash::from_pl_cdr_bytes(p.value.clone())?),
      None => None,
    })
  }

  pub fn related_sample_identity(
    params: &ParameterList,
    representation_id: RepresentationIdentifier,
  ) -> Result<Option<SampleIdentity>, PlCdrDeserializeError> {
    let rsi = params
      .parameters
      .iter()
      .find(|p| p.parameter_id == ParameterId::PID_RELATED_SAMPLE_IDENTITY);

    let endianness = match representation_id {
      RepresentationIdentifier::PL_CDR_LE => Endianness::LittleEndian,
      RepresentationIdentifier::CDR_LE => Endianness::LittleEndian,
      RepresentationIdentifier::PL_CDR_BE => Endianness::BigEndian,
      RepresentationIdentifier::CDR_BE => Endianness::BigEndian,
      _ => Err(PlCdrDeserializeError::NotSupported(
        "Unknown encoding, expected PL_CDR".to_string(),
      ))?,
    };

    Ok(match rsi {
      Some(p) => Some(
        SampleIdentity::read_from_buffer_with_ctx(endianness, &p.value)?,
        //.map_err(|e| io::Error::new(io::ErrorKind::Other, e))?,
      ),
      None => None,
    })
  }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[repr(u8)]
#[bitflags]
pub enum StatusInfoEnum {
  Disposed = 0b0001,
  Unregistered = 0b0010,
  Filtered = 0b0100,
}

/// [`StatusInfo`] is a 4 octet array
/// RTPS spec v2.3, Section 9.6.3.9 StatusInfo_t
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct StatusInfo {
  em: [u8; 3],
  si: BitFlags<StatusInfoEnum>, /* This is now a bit set of StatusInfoEnum
                                 * Interpretation:
                                 * empty set => Sample is ALIVE and must be present in the
                                 * message not Unregistered and
                                 * not Disposed => ALIVE (but may be absent)
                                 * Disposed => DataWriter disposed instance. NOT_ALIVE
                                 * Unregistered => DataWriter unregistered message. Note that
                                 * DataWriter is not required
                                 *  notify about any unregister operations. This does not make
                                 * the instance NOT_ALIVE, but informs
                                 *  that the DataWriter is not going to update that instance
                                 * anymore. Filtered =>
                                 * DataWriter wrote a sample, but it was filtered away by the
                                 * current QoS settings and thus
                                 * data is not present.
                                 *
                                 * There may be several flags set at the same time.
                                 *
                                 * Disposed & Unregistered:
                                 *
                                 * Meanings of some combinations are unknown:
                                 * Disposed & Filtered : ???
                                 * Unregistered & Filtered: ???
                                 * Disposed & Unregistered & Filtered: ??? */
}

impl<C: Context> Writable<C> for StatusInfo {
  #[inline]
  fn write_to<T: ?Sized + Writer<C>>(&self, writer: &mut T) -> std::result::Result<(), C::Error> {
    writer.write_u8(0x00)?;
    writer.write_u8(0x00)?;
    writer.write_u8(0x00)?;
    writer.write_u8(self.si.bits())?;
    Ok(())
  }
}

impl<'a, C: Context> Readable<'a, C> for StatusInfo {
  #[inline]
  fn read_from<R: speedy::Reader<'a, C>>(reader: &mut R) -> std::result::Result<Self, C::Error> {
    reader.read_u8()?;
    reader.read_u8()?;
    reader.read_u8()?;
    let si = BitFlags::<StatusInfoEnum>::from_bits_truncate(reader.read_u8()?);
    Ok(Self { em: [0; 3], si })
  }
}

impl StatusInfo {
  pub fn empty() -> Self {
    Self {
      em: [0; 3],
      si: BitFlags::empty(),
    }
  }

  pub fn contains(&self, sie: StatusInfoEnum) -> bool {
    self.si.contains(sie)
  }

  pub fn change_kind(&self) -> ChangeKind {
    if self.contains(StatusInfoEnum::Disposed) {
      // DISPOSED is strongest
      ChangeKind::NotAliveDisposed
    } else if self.contains(StatusInfoEnum::Unregistered) {
      // Checking unregistered second
      ChangeKind::NotAliveUnregistered
    } else {
      // Even if filtered is set it is still alive
      ChangeKind::Alive
    }
  }

  #[cfg(test)]
  pub fn into_cdr_bytes<BO: ByteOrder>(
    self,
  ) -> Result<Vec<u8>, serialization::cdr_serializer::Error> {
    to_bytes::<Self, BO>(&self)
  }

  #[cfg(test)]
  pub fn from_cdr_bytes(
    bytes: &[u8],
    representation_id: RepresentationIdentifier,
  ) -> Result<Self, serialization::cdr_deserializer::Error> {
    CDRDeserializerAdapter::from_bytes(bytes, representation_id)
  }
}

#[cfg(test)]
mod tests {
  use byteorder::{BigEndian, LittleEndian};

  use super::*;

  #[test]
  fn inline_qos_status_info() {
    // Little endian
    let si_bytes = StatusInfo {
      em: [0; 3],
      si: StatusInfoEnum::Disposed | StatusInfoEnum::Unregistered,
    }
    .into_cdr_bytes::<LittleEndian>()
    .unwrap();

    let bytes: Vec<u8> = vec![0x00, 0x00, 0x00, 0x03];
    assert_eq!(si_bytes, bytes);

    let status_info = StatusInfo::from_cdr_bytes(&bytes, RepresentationIdentifier::CDR_LE).unwrap();
    assert_eq!(
      status_info,
      StatusInfo {
        em: [0; 3],
        si: StatusInfoEnum::Disposed | StatusInfoEnum::Unregistered
      }
    );

    // Big endian
    let si_bytes = StatusInfo {
      em: [0; 3],
      si: StatusInfoEnum::Disposed | StatusInfoEnum::Unregistered,
    }
    .into_cdr_bytes::<BigEndian>()
    .unwrap();

    let bytes: Vec<u8> = vec![0x00, 0x00, 0x00, 0x03];
    assert_eq!(si_bytes, bytes);

    let status_info = StatusInfo::from_cdr_bytes(&bytes, RepresentationIdentifier::CDR_BE).unwrap();
    assert_eq!(
      status_info,
      StatusInfo {
        em: [0; 3],
        si: StatusInfoEnum::Disposed | StatusInfoEnum::Unregistered
      }
    );
  }
}
