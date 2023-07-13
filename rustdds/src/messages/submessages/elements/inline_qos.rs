use std::io;

use enumflags2::{bitflags, BitFlags};
use serde::{Deserialize, Serialize};
#[cfg(test)]
use byteorder::ByteOrder;
use speedy::{Endianness, Readable};
#[allow(unused_imports)]
use log::{debug, error, info, trace, warn};

use crate::{
  dds::{adapters::no_key::*, key::KeyHash},
  messages::submessages::elements::{parameter_list::ParameterList, RepresentationIdentifier},
  serialization,
  serialization::CDRDeserializerAdapter,
  structure::{cache_change::ChangeKind, parameter_id::ParameterId, rpc::SampleIdentity},
};
#[cfg(test)]
use crate::serialization::cdr_serializer::to_bytes;

// Utility for parsing RTPS inlineQoS parameters
// TODO: This does not need to be a struct, since is has no contents.
// Maybe someone has had an overdose of object-orientation?
// Some standalone functions should suffice.
pub(crate) struct InlineQos {}

impl InlineQos {
  pub fn status_info(
    params: &ParameterList,
    rep_id: RepresentationIdentifier,
  ) -> std::result::Result<StatusInfo, serialization::Error> {
    let status_info = params
      .parameters
      .iter()
      .find(|p| p.parameter_id == ParameterId::PID_STATUS_INFO);
    let status_info = match status_info {
      Some(p) => StatusInfo::from_cdr_bytes(&p.value, rep_id)?,
      None => StatusInfo::empty(),
    };

    Ok(status_info)
  }

  pub fn key_hash(params: &ParameterList) -> Result<Option<KeyHash>, serialization::Error> {
    let key_hash = params
      .parameters
      .iter()
      .find(|p| p.parameter_id == ParameterId::PID_KEY_HASH);
    Ok(match key_hash {
      Some(p) => Some(KeyHash::from_cdr_bytes(p.value.clone())?),
      None => None,
    })
  }

  pub fn related_sample_identity(
    params: &ParameterList,
    representation_id: RepresentationIdentifier,
  ) -> Result<Option<SampleIdentity>, serialization::Error> {
    let rsi = params
      .parameters
      .iter()
      .find(|p| p.parameter_id == ParameterId::PID_RELATED_SAMPLE_IDENTITY);

    let endianness = if representation_id == RepresentationIdentifier::CDR_BE
      || representation_id == RepresentationIdentifier::PL_CDR_BE
    {
      Endianness::BigEndian
    } else {
      Endianness::LittleEndian
    };

    Ok(match rsi {
      Some(p) => Some(
        SampleIdentity::read_from_buffer_with_ctx(endianness, &p.value)
          .map_err(|e| io::Error::new(io::ErrorKind::Other, e))?,
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
                                 * not Disposed => ALIVE (but may be asbent)
                                 * Disposed => DataWriter disposed instance. NOT_ALIVE
                                 * Unregistered => DataWriter unregistered message. Note that
                                 * DataWriter is not required
                                 *  notify about any unregister operations. This does not make
                                 * the instance NOT_ALIVE, but infoms
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
                                 * Meanings of some combinations are uknown:
                                 * Disposed & Filtered : ???
                                 * Unregistered & Filtered: ???
                                 * Disposed & Unregistered & Filtered: ??? */
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
  pub fn into_cdr_bytes<BO: ByteOrder>(self) -> Result<Vec<u8>, serialization::Error> {
    to_bytes::<Self, BO>(&self)
  }

  pub fn from_cdr_bytes(
    bytes: &[u8],
    representation_id: RepresentationIdentifier,
  ) -> Result<Self, serialization::Error> {
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
