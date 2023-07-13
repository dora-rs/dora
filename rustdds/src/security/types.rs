use bytes::Bytes;
use enumflags2::bitflags;
use speedy::{Context, Readable, Reader, Writable, Writer};

use crate::serialization::speedy_pl_cdr_helpers::*;

// Property_t type from section 7.2.1 of the Security specification (v. 1.1)
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Property {
  name: String,
  value: String,
  propagate: bool, // NOT SERIALIZED
}

impl<'a, C: Context> Readable<'a, C> for Property {
  fn read_from<R: Reader<'a, C>>(reader: &mut R) -> Result<Self, C::Error> {
    let name: StringWithNul = reader.read_value()?;

    read_pad(reader, name.len(), 4)?; // pad according to previous read
    let value: StringWithNul = reader.read_value()?;

    Ok(Property {
      name: name.into(),
      value: value.into(),
      propagate: true, // since we read this from thw wire, it was propagated
    })
  }
}

// Writing several strings is a bit complicated, because
// we have to keep track of alignment.
// Again, alignment comes BEFORE string length, or vector item count, not after
// string.
impl<C: Context> Writable<C> for Property {
  fn write_to<T: ?Sized + Writer<C>>(&self, writer: &mut T) -> Result<(), C::Error> {
    let name = StringWithNul::from(self.name.clone());
    // nothing yet to pad
    writer.write_value(&name)?;

    write_pad(writer, name.len(), 4)?;
    let value = StringWithNul::from(self.value.clone());
    writer.write_value(&value)?;

    Ok(())
  }
}

impl Property {
  pub fn serialized_len(&self) -> usize {
    let first = 4 + self.name.len() + 1;
    let misalign = first % 4;
    let align = if misalign > 0 { 4 - misalign } else { 0 };
    let second = 4 + self.value.len() + 1;
    first + align + second
  }
}

// BinaryProperty_t type from section 7.2.2 of the Security specification (v.
// 1.1)
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BinaryProperty {
  pub(crate) name: String, // public because of serialization
  pub(crate) value: Bytes,
  pub(crate) propagate: bool, // propagate field is not serialized
}

impl BinaryProperty {
  pub fn serialized_len(&self) -> usize {
    let first = 4 + self.name.len() + 1;
    let misalign = first % 4;
    let align = if misalign > 0 { 4 - misalign } else { 0 };
    let second = 4 + self.value.len(); // no nul terminator byte here
    first + align + second
  }
}

impl<'a, C: Context> Readable<'a, C> for BinaryProperty {
  fn read_from<R: Reader<'a, C>>(reader: &mut R) -> Result<Self, C::Error> {
    let name: StringWithNul = reader.read_value()?;

    read_pad(reader, name.len(), 4)?; // pad according to previous read
    let value: Vec<u8> = reader.read_value()?;

    Ok(BinaryProperty {
      name: name.into(),
      value: value.into(),
      propagate: true, // since we read this from thw wire, it was propagated
    })
  }
}

// Writing several strings is a bit complicated, because
// we have to keep track of alignment.
// Again, alignment comes BEFORE string length, or vector item count, not after
// string.
impl<C: Context> Writable<C> for BinaryProperty {
  fn write_to<T: ?Sized + Writer<C>>(&self, writer: &mut T) -> Result<(), C::Error> {
    let name = StringWithNul::from(self.name.clone());
    writer.write_value(&name)?;

    write_pad(writer, name.len(), 4)?;
    writer.write_value(&<Vec<u8>>::from(self.value.clone()))?;

    Ok(())
  }
}

// DataHolder type from section 7.2.3 of the Security specification (v. 1.1)
// fields need to be public to make (de)serializable
pub struct DataHolder {
  pub(crate) class_id: String,
  pub(crate) properties: Vec<Property>,
  pub(crate) binary_properties: Vec<BinaryProperty>,
}

// Token type from section 7.2.4 of the Security specification (v. 1.1)
pub type Token = DataHolder;

// ParticipantBuiltinTopicDataSecure from section 7.4.1.6 of the Security
// specification
pub struct ParticipantBuiltinTopicDataSecure {}

// PublicationBuiltinTopicDataSecure from section 7.4.1.7 of the Security
// specification
pub struct PublicationBuiltinTopicDataSecure {}

// SubscriptionBuiltinTopicDataSecure from section 7.4.1.8 of the Security
// specification
pub struct SubscriptionBuiltinTopicDataSecure {}

// Result type with generic OK type. Error type is SecurityError.
pub type SecurityResult<T> = std::result::Result<T, SecurityError>;

// Something like the SecurityException of the specification
#[derive(Debug, thiserror::Error)]
#[error("Security exception: {msg}")]
pub struct SecurityError {
  pub(crate) msg: String,
}

// DDS Security spec v1.1 Section 7.2.7 ParticipantSecurityInfo
// This is communicated over Discovery

#[derive(Debug, Clone, PartialEq, Eq, Readable, Writable)]
pub struct ParticipantSecurityInfo {
  participant_security_attributes: ParticipantSecurityAttributesMask,
  plugin_participant_security_attributes: PluginParticipantSecurityAttributesMask,
}

#[derive(Debug, PartialOrd, PartialEq, Ord, Eq, Clone, Copy, Readable, Writable)]
#[bitflags]
#[repr(u32)]
#[allow(clippy::enum_variant_names)]
// Clippy complains, because all variant names have the same prefix "Is",
// but we blame the DDS Security spec for naming.
pub enum ParticipantSecurityAttributesMask {
  IsValid = 0x8000_0000, // (0x1 << 31) -- only this bit is understood outside security plugins

  // DDS Security specification v1.1
  // Section 8.4.2.5 Definition of the ParticipantSecurityAttributesMask
  // Table 28
  IsRTPSProtected = 0b000_0001,
  IsDiscoveryProtected = 0b000_0010,
  IsLivelinessProtected = 0b000_0100,
}

#[derive(Debug, PartialOrd, PartialEq, Ord, Eq, Clone, Copy, Readable, Writable)]
#[bitflags]
#[repr(u32)]
#[allow(clippy::enum_variant_names)]
// Clippy complains, because all variant names have the same prefix.
pub enum PluginParticipantSecurityAttributesMask {
  IsValid = 0x8000_0000, // (0x1 << 31)

  // DDS Security specification v1.1
  // Section 9.4.2.4 Definition of the PluginParticipantSecurityAttributesMask
  // Table 60
  IsRTPSEncrypted = 0b0000_0001,
  IsDiscoveryEncrypted = 0b0000_0010,
  IsLivelinessEncrypted = 0b0000_0100,
  IsRTPSOriginAuthenticated = 0b0000_1000,
  IsDiscoveryOriginAuthenticated = 0b0001_0000,
  IsLivelinessOriginAuthenticated = 0b0010_0000,
}

// // serialization helper struct
// #[derive(Serialize, Deserialize)]
// pub(crate) struct ParticipantSecurityInfoData {
//   parameter_id: ParameterId,
//   parameter_length: u16,
//   security_info: ParticipantSecurityInfo,
// }

// impl ParticipantSecurityInfoData {
//   pub fn new(security_info: ParticipantSecurityInfo) -> Self {
//     Self {
//       parameter_id: ParameterId::PID_PARTICIPANT_SECURITY_INFO,
//       parameter_length: 8, // 2x u32
//       security_info,
//     }
//   }
// }

// DDS Security spec v1.1 Section 7.2.8 EndpointSecurityInfo
// This is communicated over Discovery

#[derive(Debug, Clone, PartialEq, Eq, Readable, Writable)]
pub struct EndpointSecurityInfo {
  endpoint_security_attributes: EndpointSecurityAttributesMask,
  plugin_endpoint_security_attributes: PluginEndpointSecurityAttributesMask,
}

#[derive(Debug, PartialOrd, PartialEq, Ord, Eq, Clone, Copy, Readable, Writable)]
#[bitflags]
#[repr(u32)]
#[allow(clippy::enum_variant_names)]
// Clippy complains, because all variant names have the same prefix "Is",
// but we blame the DDS Security spec for naming.
pub enum EndpointSecurityAttributesMask {
  IsValid = 0x8000_0000, // (0x1 << 31) -- only this bit is understood outside security plugins

  // DDS Security specification v1.1
  // Section 8.4.2.8 Definition of the EndpointSecurityAttributesMask
  // Table 31
  IsReadProtected = 0b0000_0001,
  IsWriteProtected = 0b0000_0010,
  IsDiscoveryProtected = 0b0000_0100,
  IsSubmessageProtected = 0b0000_1000,
  IsPayloadProtected = 0b0001_0000,
  IsKeyProtected = 0b0010_0000,
  IsLivelinessProtected = 0b0100_0000,
}

#[derive(Debug, PartialOrd, PartialEq, Ord, Eq, Clone, Copy, Readable, Writable)]
#[bitflags]
#[repr(u32)]
#[allow(clippy::enum_variant_names)]
// Clippy complains, because all variant names have the same prefix.
pub enum PluginEndpointSecurityAttributesMask {
  IsValid = 0x8000_0000, // (0x1 << 31)

  // DDS Security specification v1.1
  // Section 9.4.2.6 Definition of the PluginEndpointSecurityAttributesMask
  // Table 62
  IsSubmessageEncrypted = 0b0000_0001,
  IsPayloadEncrypted = 0b0000_0010,
  IsSubmessageOriginAuthenticated = 0b0000_0100,
}

// // serialization helper struct
// #[derive(Serialize, Deserialize)]
// pub(crate) struct EndpointSecurityInfoData {
//   parameter_id: ParameterId,
//   parameter_length: u16,
//   security_info: EndpointSecurityInfo,
// }

// impl EndpointSecurityInfoData {
//   pub fn new(security_info: EndpointSecurityInfo) -> Self {
//     Self {
//       parameter_id: ParameterId::PID_ENDPOINT_SECURITY_INFO,
//       parameter_length: 8, // 2x u32
//       security_info,
//     }
//   }
// }
