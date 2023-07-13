use std::{fmt, hash::Hash, ops::RangeBounds};

use speedy::{Context, Readable, Reader, Writable, Writer};
use serde::{Deserialize, Serialize};
use cdr_encoding_size::*;
use mio_06::Token;
use log::warn;
use static_assertions as sa;

use crate::dds::key::Key;

/// DDS/RTPS Participant GuidPrefix
#[derive(
  Copy, Clone, PartialOrd, PartialEq, Ord, Eq, Hash, Serialize, Deserialize, CdrEncodingSize,
)]
pub struct GuidPrefix {
  pub(crate) bytes: [u8; 12],
}

impl GuidPrefix {
  pub const UNKNOWN: Self = Self { bytes: [0x00; 12] };

  pub fn new(prefix: &[u8]) -> Self {
    let mut bytes: [u8; 12] = [0; 12];
    for (ix, data) in prefix.iter().enumerate() {
      if ix >= 12 {
        break;
      }
      bytes[ix] = *data;
    }
    Self { bytes }
  }

  pub fn random_for_this_participant() -> Self {
    let mut bytes: [u8; 12] = rand::random(); // start with random data

    // The prefix is arbitrary, but let's place our vendor id at the head
    // for easy recognition. It seems some other RTPS implementations are doing the
    // same.
    let my_vendor_id_bytes = crate::messages::vendor_id::VendorId::THIS_IMPLEMENTATION.as_bytes();
    bytes[0] = my_vendor_id_bytes[0];
    bytes[1] = my_vendor_id_bytes[1];

    // TODO:
    // We could add some other identifying stuff here also, like one of
    // our IP addresses (but which one?)

    Self { bytes }
  }

  pub fn range(&self) -> impl RangeBounds<GUID> {
    GUID::new(*self, EntityId::MIN)..=GUID::new(*self, EntityId::MAX)
  }
}

impl AsRef<[u8]> for GuidPrefix {
  fn as_ref(&self) -> &[u8] {
    &self.bytes
  }
}

impl fmt::Debug for GuidPrefix {
  // This is so common that we skip all the introductions and just print the data.
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    self.bytes.fmt(f)
  }
}

impl Default for GuidPrefix {
  fn default() -> Self {
    Self::UNKNOWN
  }
}

impl<'a, C: Context> Readable<'a, C> for GuidPrefix {
  #[inline]
  fn read_from<R: Reader<'a, C>>(reader: &mut R) -> Result<Self, C::Error> {
    let mut guid_prefix = Self::default();
    for i in 0..guid_prefix.bytes.len() {
      guid_prefix.bytes[i] = reader.read_u8()?;
    }
    Ok(guid_prefix)
  }

  #[inline]
  fn minimum_bytes_needed() -> usize {
    std::mem::size_of::<Self>()
  }
}

impl<C: Context> Writable<C> for GuidPrefix {
  #[inline]
  fn write_to<T: ?Sized + Writer<C>>(&self, writer: &mut T) -> Result<(), C::Error> {
    for elem in &self.bytes {
      writer.write_u8(*elem)?;
    }
    Ok(())
  }
}

#[derive(
  Copy, Clone, PartialOrd, PartialEq, Ord, Eq, Hash, Serialize, Deserialize, CdrEncodingSize,
)]
pub struct EntityKind(u8);

impl EntityKind {
  // constants from RTPS spec Table 9.1
  pub const UNKNOWN_USER_DEFINED: Self = Self(0x00);
  // pub const PARTICIPANT_USER_DEFINED : Self = Self(0x01);
  // User-defined participants do not exist by definition.
  pub const WRITER_WITH_KEY_USER_DEFINED: Self = Self(0x02);
  pub const WRITER_NO_KEY_USER_DEFINED: Self = Self(0x03);
  pub const READER_NO_KEY_USER_DEFINED: Self = Self(0x04);
  pub const READER_WITH_KEY_USER_DEFINED: Self = Self(0x07);
  pub const WRITER_GROUP_USER_DEFINED: Self = Self(0x08);
  pub const READER_GROUP_USER_DEFINED: Self = Self(0x09);

  pub const UNKNOWN_BUILT_IN: Self = Self(0xC0);
  pub const PARTICIPANT_BUILT_IN: Self = Self(0xC1);
  pub const WRITER_WITH_KEY_BUILT_IN: Self = Self(0xC2);
  pub const WRITER_NO_KEY_BUILT_IN: Self = Self(0xC3);
  pub const READER_NO_KEY_BUILT_IN: Self = Self(0xC4);
  pub const READER_WITH_KEY_BUILT_IN: Self = Self(0xC7);
  pub const WRITER_GROUP_BUILT_IN: Self = Self(0xC8);
  pub const READER_GROUP_BUILT_IN: Self = Self(0xC9);

  pub const MIN: Self = Self(0x00);
  pub const MAX: Self = Self(0xFF);

  // We will encode polling tokens as EntityId, containing an EntityKind
  // The upper nibble of EntityKind will distinguish between different
  // poll tokens:
  // 0 = user-defined entity
  // 1
  // 2 = user-defined alt token (timers etc)
  // 3
  // 4 = fixed poll tokens (not entity-specific)
  pub const POLL_TOKEN_BASE: usize = 0x40;
  // 5 = fixed poll tokens continued
  // 6 = fixed poll tokens continued
  // 7 = fixed poll tokens continued
  // 8
  // 9
  // A
  // B
  // C = built-in entity
  // D
  // E = built-in alt token
  // F

  pub fn is_reader(&self) -> bool {
    let e = self.0 & 0x0F;
    e == 0x04 || e == 0x07 || e == 0x09
  }

  pub fn is_writer(&self) -> bool {
    let e = self.0 & 0x0F;
    e == 0x02 || e == 0x03 || e == 0x08
  }

  pub fn is_built_in(&self) -> bool {
    (self.0 & 0xF0) == 0xC0
  }

  pub fn is_user_defined(&self) -> bool {
    (self.0 & 0xF0) == 0x00
  }
}

impl From<u8> for EntityKind {
  fn from(b: u8) -> Self {
    Self(b)
  }
}

impl From<EntityKind> for u8 {
  fn from(ek: EntityKind) -> Self {
    ek.0
  }
}
impl fmt::Debug for EntityKind {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    match *self {
      Self::UNKNOWN_USER_DEFINED => f.write_str("EntityKind::UNKNOWN_USER_DEFINED"),
      Self::WRITER_WITH_KEY_USER_DEFINED => f.write_str("EntityKind::WRITER_WITH_KEY_USER_DEFINED"),
      Self::WRITER_NO_KEY_USER_DEFINED => f.write_str("EntityKind::WRITER_NO_KEY_USER_DEFINED"),
      Self::READER_NO_KEY_USER_DEFINED => f.write_str("EntityKind::READER_NO_KEY_USER_DEFINED"),
      Self::READER_WITH_KEY_USER_DEFINED => f.write_str("EntityKind::READER_WITH_KEY_USER_DEFINED"),
      Self::WRITER_GROUP_USER_DEFINED => f.write_str("EntityKind::WRITER_GROUP_USER_DEFINED"),
      Self::READER_GROUP_USER_DEFINED => f.write_str("EntityKind::READER_GROUP_USER_DEFINED"),

      Self::UNKNOWN_BUILT_IN => f.write_str("EntityKind::UNKNOWN_BUILT_IN"),
      Self::PARTICIPANT_BUILT_IN => f.write_str("EntityKind::PARTICIPANT_BUILT_IN"),
      Self::WRITER_WITH_KEY_BUILT_IN => f.write_str("EntityKind::WRITER_WITH_KEY_BUILT_IN"),
      Self::WRITER_NO_KEY_BUILT_IN => f.write_str("EntityKind::WRITER_NO_KEY_BUILT_IN"),
      Self::READER_NO_KEY_BUILT_IN => f.write_str("EntityKind::READER_NO_KEY_BUILT_IN"),
      Self::READER_WITH_KEY_BUILT_IN => f.write_str("EntityKind::READER_WITH_KEY_BUILT_IN"),
      Self::WRITER_GROUP_BUILT_IN => f.write_str("EntityKind::WRITER_GROUP_BUILT_IN"),
      Self::READER_GROUP_BUILT_IN => f.write_str("EntityKind::READER_GROUP_BUILT_IN"),
      _ => f.write_fmt(format_args!("EntityKind({:x?})", self.0)),
    }
  }
}

/// RTPS EntityId
/// See RTPS spec section 8.2.4 , 8.3.5.1 and 9.3.1.2
#[derive(
  Copy, Clone, PartialOrd, PartialEq, Ord, Eq, Hash, Serialize, Deserialize, CdrEncodingSize,
)]
pub struct EntityId {
  pub entity_key: [u8; 3],
  pub entity_kind: EntityKind,
}

// We are going to pack 32 bits of payload into an usize, or ultimately
// into a mio::Token, so we need it to be large enough.
sa::const_assert!(std::mem::size_of::<usize>() >= std::mem::size_of::<u32>());

#[derive(Copy, Clone, Debug, PartialOrd, PartialEq, Ord, Eq)]
pub enum TokenDecode {
  Entity(EntityId),
  AltEntity(EntityId),
  FixedToken(Token),
}

impl EntityId {
  pub const UNKNOWN: Self = Self {
    entity_key: [0x00; 3],
    entity_kind: EntityKind::UNKNOWN_USER_DEFINED,
  };
  pub const PARTICIPANT: Self = Self {
    entity_key: [0x00, 0x00, 0x01],
    entity_kind: EntityKind::PARTICIPANT_BUILT_IN,
  };
  pub const SEDP_BUILTIN_TOPIC_WRITER: Self = Self {
    entity_key: [0x00, 0x00, 0x02],
    entity_kind: EntityKind::WRITER_WITH_KEY_BUILT_IN,
  };
  pub const SEDP_BUILTIN_TOPIC_READER: Self = Self {
    entity_key: [0x00, 0x00, 0x02],
    entity_kind: EntityKind::READER_WITH_KEY_BUILT_IN,
  };
  pub const SEDP_BUILTIN_PUBLICATIONS_WRITER: Self = Self {
    entity_key: [0x00, 0x00, 0x03],
    entity_kind: EntityKind::WRITER_WITH_KEY_BUILT_IN,
  };
  pub const SEDP_BUILTIN_PUBLICATIONS_READER: Self = Self {
    entity_key: [0x00, 0x00, 0x03],
    entity_kind: EntityKind::READER_WITH_KEY_BUILT_IN,
  };
  pub const SEDP_BUILTIN_SUBSCRIPTIONS_WRITER: Self = Self {
    entity_key: [0x00, 0x00, 0x04],
    entity_kind: EntityKind::WRITER_WITH_KEY_BUILT_IN,
  };
  pub const SEDP_BUILTIN_SUBSCRIPTIONS_READER: Self = Self {
    entity_key: [0x00, 0x00, 0x04],
    entity_kind: EntityKind::READER_WITH_KEY_BUILT_IN,
  };
  pub const SPDP_BUILTIN_PARTICIPANT_WRITER: Self = Self {
    entity_key: [0x00, 0x01, 0x00],
    entity_kind: EntityKind::WRITER_WITH_KEY_BUILT_IN,
  };
  pub const SPDP_BUILTIN_PARTICIPANT_READER: Self = Self {
    entity_key: [0x00, 0x01, 0x00],
    entity_kind: EntityKind::READER_WITH_KEY_BUILT_IN,
  };
  pub const P2P_BUILTIN_PARTICIPANT_MESSAGE_WRITER: Self = Self {
    entity_key: [0x00, 0x02, 0x00],
    entity_kind: EntityKind::WRITER_WITH_KEY_BUILT_IN,
  };
  pub const P2P_BUILTIN_PARTICIPANT_MESSAGE_READER: Self = Self {
    entity_key: [0x00, 0x02, 0x00],
    entity_kind: EntityKind::READER_WITH_KEY_BUILT_IN,
  };

  pub const MIN: Self = Self {
    entity_key: [0x00; 3],
    entity_kind: EntityKind::MIN,
  };
  pub const MAX: Self = Self {
    entity_key: [0xFF, 0xFF, 0xFF],
    entity_kind: EntityKind::MAX,
  };

  pub fn new(entity_key: [u8; 3], entity_kind: EntityKind) -> Self {
    Self {
      entity_key,
      entity_kind,
    }
  }

  #[cfg(test)]
  pub(crate) fn create_custom_entity_id(entity_key: [u8; 3], entity_kind: EntityKind) -> Self {
    Self::new(entity_key, entity_kind)
  }

  fn as_usize(self) -> usize {
    // usize is generated like this because there needs to be
    // a way to tell entity kind from the result
    let u1 = u32::from(self.entity_key[0]);
    let u2 = u32::from(self.entity_key[1]);
    let u3 = u32::from(self.entity_key[2]);
    let u4 = u32::from(self.entity_kind.0);

    // This is essentially big-endian encoding
    // The type coercion will always succeed, because we have
    // above a static assert that usize is at least 32-bit
    ((u1 << 24) | (u2 << 16) | (u3 << 8) | u4) as usize
  }

  /// Use this only with usize generated with EntityID::as_usize function.!!!
  fn from_usize(number: usize) -> Self {
    let u4 = (number & 0xFF) as u8;
    let u3 = ((number >> 8) & 0xFF) as u8;
    let u2 = ((number >> 16) & 0xFF) as u8;
    let u1 = ((number >> 24) & 0xFF) as u8;

    let result = Self {
      entity_key: [u1, u2, u3],
      entity_kind: EntityKind::from(u4),
    };

    // check sanity, as the result should be
    let kind_kind = u4 & (0xC0 | 0x10);
    if kind_kind == 0xC0 || kind_kind == 0x00 {
      // this is ok, all normal
    } else {
      warn!("EntityId::from_usize tried to decode 0x{:x?}", number);
    }

    result
  }

  pub fn as_token(self) -> Token {
    let u = self.as_usize();
    assert_eq!(u & !0x20, u); // check bit 5 is zero
    Token(u)
  }

  pub fn as_alt_token(self) -> Token {
    Token(self.as_usize() | 0x20) // set bit 5
  }

  pub fn from_token(t: Token) -> TokenDecode {
    match (t.0 & 0xF0) as u8 {
      0x00 | 0xC0 => TokenDecode::Entity(Self::from_usize(t.0)),
      0x20 | 0xE0 => TokenDecode::AltEntity(Self::from_usize(t.0 & !0x20)),
      0x40 | 0x50 | 0x60 | 0x70 => TokenDecode::FixedToken(t),
      _other => {
        warn!("EntityId::from_token tried to decode 0x{:x?}", t.0);
        TokenDecode::FixedToken(t)
      }
    }
  }

  pub fn kind(self) -> EntityKind {
    self.entity_kind
  }

  pub fn set_kind(&mut self, entity_kind: EntityKind) {
    self.entity_kind = entity_kind;
  }

  pub fn to_slice(self) -> [u8; 4] {
    let mut slice = [0; 4];
    slice[0] = self.entity_key[0];
    slice[1] = self.entity_key[1];
    slice[2] = self.entity_key[2];
    slice[3] = self.entity_kind.0;

    slice
  }

  pub fn from_slice(bytes: [u8; 4]) -> Self {
    Self {
      entity_key: [bytes[0], bytes[1], bytes[2]],
      entity_kind: EntityKind::from(bytes[3]),
    }
  }
}

impl Default for EntityId {
  fn default() -> Self {
    Self::UNKNOWN
  }
}

impl fmt::Debug for EntityId {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    match *self {
      Self::UNKNOWN => f.write_str("EntityId::UNKNOWN"),
      Self::PARTICIPANT => f.write_str("EntityId::PARTICIPANT"),
      Self::SEDP_BUILTIN_TOPIC_WRITER => f.write_str("EntityId::SEDP_BUILTIN_TOPIC_WRITER"),
      Self::SEDP_BUILTIN_TOPIC_READER => f.write_str("EntityId::SEDP_BUILTIN_TOPIC_READER"),
      Self::SEDP_BUILTIN_PUBLICATIONS_WRITER => {
        f.write_str("EntityId::SEDP_BUILTIN_PUBLICATIONS_WRITER")
      }
      Self::SEDP_BUILTIN_PUBLICATIONS_READER => {
        f.write_str("EntityId::SEDP_BUILTIN_PUBLICATIONS_READER")
      }
      Self::SEDP_BUILTIN_SUBSCRIPTIONS_WRITER => {
        f.write_str("EntityId::SEDP_BUILTIN_SUBSCRIPTIONS_WRITER")
      }
      Self::SEDP_BUILTIN_SUBSCRIPTIONS_READER => {
        f.write_str("EntityId::SEDP_BUILTIN_SUBSCRIPTIONS_READER")
      }
      Self::SPDP_BUILTIN_PARTICIPANT_WRITER => {
        f.write_str("EntityId::SPDP_BUILTIN_PARTICIPANT_WRITER")
      }
      Self::SPDP_BUILTIN_PARTICIPANT_READER => {
        f.write_str("EntityId::SPDP_BUILTIN_PARTICIPANT_READER")
      }
      Self::P2P_BUILTIN_PARTICIPANT_MESSAGE_WRITER => {
        f.write_str("EntityId::P2P_BUILTIN_PARTICIPANT_MESSAGE_WRITER")
      }
      Self::P2P_BUILTIN_PARTICIPANT_MESSAGE_READER => {
        f.write_str("EntityId::P2P_BUILTIN_PARTICIPANT_MESSAGE_READER")
      }
      _ => {
        f.write_str("EntityId {")?;
        self.entity_key.fmt(f)?;
        f.write_str(" ")?;
        self.entity_kind.fmt(f)?;
        f.write_str("}")
      }
    }
  }
}

impl<'a, C: Context> Readable<'a, C> for EntityId {
  #[inline]
  fn read_from<R: Reader<'a, C>>(reader: &mut R) -> Result<Self, C::Error> {
    let entity_key = [reader.read_u8()?, reader.read_u8()?, reader.read_u8()?];
    let entity_kind = EntityKind(reader.read_u8()?);
    Ok(Self {
      entity_key,
      entity_kind,
    })
  }
}

impl<C: Context> Writable<C> for EntityId {
  #[inline]
  fn write_to<T: ?Sized + Writer<C>>(&self, writer: &mut T) -> Result<(), C::Error> {
    for elem in &self.entity_key {
      writer.write_u8(*elem)?;
    }
    writer.write_u8(self.entity_kind.0)
  }
}

/// DDS/RTPS GUID
///
/// Spec 2.5, Section 8.2.4.1 Identifying RTPS entities: the GUID
///
/// The GUID (Globally Unique Identifier) is an attribute of all RTPS Entities
/// and uniquely identifies the Entity within a DDS Domain.
///
/// The GUID is built as a tuple <prefix, entityId> combining a GuidPrefix_t
/// prefix and an EntityId_t entityId
/// ...
/// The implementation is free to choose the prefix, as long as every
/// Participant in the Domain has a unique GUID.
/// ...
/// The GUIDs of all the Endpoints within a Participant have the same prefix.

#[derive(
  Copy,
  Clone,
  Default,
  PartialOrd,
  PartialEq,
  Ord,
  Eq,
  Readable,
  Writable,
  Hash,
  Serialize,
  Deserialize,
  CdrEncodingSize,
)]
pub struct GUID {
  // Note: It is important to have guid_prefix first, so that derive'd Ord trait
  // will produce ordering, where GUIDs with same GuidPrefix are grouped
  // together.
  pub prefix: GuidPrefix,
  pub entity_id: EntityId,
}

impl GUID {
  pub const GUID_UNKNOWN: Self = Self {
    prefix: GuidPrefix::UNKNOWN,
    entity_id: EntityId::UNKNOWN,
  };

  /// basic constructor from components
  pub fn new(prefix: GuidPrefix, entity_id: EntityId) -> Self {
    Self::new_with_prefix_and_id(prefix, entity_id)
  }

  pub fn from_bytes(bytes: [u8; 16]) -> Self {
    let mut prefix = GuidPrefix { bytes: [0; 12] };
    prefix.bytes.as_mut_slice().copy_from_slice(&bytes[0..12]);

    let mut eid_bytes = [0; 4];
    eid_bytes.as_mut_slice().copy_from_slice(&bytes[12..16]);

    Self {
      prefix,
      entity_id: EntityId::from_slice(eid_bytes),
    }
  }

  /// Generates new GUID for Participant when `guid_prefix` is random
  pub fn new_participant_guid() -> Self {
    Self {
      prefix: GuidPrefix::random_for_this_participant(),
      entity_id: EntityId::PARTICIPANT,
    }
  }

  pub fn dummy_test_guid(entity_kind: EntityKind) -> Self {
    Self {
      prefix: GuidPrefix::new(b"FakeTestGUID"),
      entity_id: EntityId {
        entity_key: [1, 2, 3],
        entity_kind,
      },
    }
  }

  /// Generates GUID for specific entity_id from current prefix
  #[must_use]
  pub fn from_prefix(self, entity_id: EntityId) -> Self {
    Self {
      prefix: self.prefix,
      entity_id,
    }
  }

  /// alias for basic constructor from components
  pub fn new_with_prefix_and_id(prefix: GuidPrefix, entity_id: EntityId) -> Self {
    Self { prefix, entity_id }
  }

  pub fn as_usize(&self) -> usize {
    self.entity_id.as_usize()
  }

  pub fn to_bytes(&self) -> [u8; 16] {
    let mut bytes = [0; 16];
    bytes.as_mut_slice()[0..12].copy_from_slice(self.prefix.as_ref());
    bytes.as_mut_slice()[12..16].copy_from_slice(&self.entity_id.to_slice());
    bytes
  }
}

impl Key for GUID {}

impl fmt::Debug for GUID {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    f.write_fmt(format_args!(
      "GUID {{{:?} {:?}}}",
      self.prefix, self.entity_id
    ))
  }
}

#[cfg(test)]
mod tests {
  use speedy::Endianness;
  use mio_06::Token;
  use log::info;
  use byteorder::BigEndian;

  use super::*;

  #[test]
  fn serde_test() {
    use crate::serialization::{
      cdr_deserializer::deserialize_from_big_endian, cdr_serializer::to_bytes,
    };

    let test_bytes = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16];
    let test_guid = GUID::from_bytes(test_bytes);
    let ser = to_bytes::<GUID, BigEndian>(&test_guid).unwrap();
    assert_eq!(test_bytes.to_vec(), ser);

    let and_back = deserialize_from_big_endian::<GUID>(&ser).unwrap();
    assert_eq!(test_guid, and_back);
  }

  // #[test]
  // fn serde_pl_cdr_test() {
  //   use crate::{
  //     serialization::{pl_cdr_deserializer::PlCdrDeserializer,
  // cdr_serializer::to_bytes},   };

  //   let test_bytes = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16];
  //   let test_guid = GUID::from_bytes(test_bytes);
  //   let ser = to_bytes::<GUID, BigEndian>(&test_guid).unwrap();
  //   assert_eq!(test_bytes.to_vec(), ser);

  //   let and_back =
  // PlCdrDeserializer::from_big_endian_bytes::<GUID>(&ser).unwrap();
  //   assert_eq!(test_guid, and_back);
  // }

  #[test]
  fn keyhash_test() {
    let test_bytes = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16];
    let test_guid = GUID::from_bytes(test_bytes);
    let key_hash = test_guid.hash_key(); // from trait Key
    assert_eq!(key_hash.to_vec(), test_bytes.to_vec());
  }

  #[test]
  fn convert_entity_id_to_token_and_back() {
    let e = EntityId::SPDP_BUILTIN_PARTICIPANT_WRITER;
    let _t = Token(e.as_usize());
    info!("{:?}", e.as_usize());
    let entity = EntityId::from_usize(e.as_usize());
    assert_eq!(e, entity);

    let e2 = EntityId::P2P_BUILTIN_PARTICIPANT_MESSAGE_READER;
    let entity2 = EntityId::from_usize(e2.as_usize());
    assert_eq!(e2, entity2);

    let e3 = EntityId::SEDP_BUILTIN_TOPIC_WRITER;
    let entity3 = EntityId::from_usize(e3.as_usize());
    assert_eq!(e3, entity3);

    let e4 = EntityId::SEDP_BUILTIN_TOPIC_WRITER;
    let entity4 = EntityId::from_usize(e4.as_usize());
    assert_eq!(e4, entity4);

    let e5 = EntityId::UNKNOWN;
    let entity5 = EntityId::from_usize(e5.as_usize());
    assert_eq!(e5, entity5);

    let e6 = EntityId::create_custom_entity_id([12u8, 255u8, 0u8], EntityKind(254u8));
    let entity6 = EntityId::from_usize(e6.as_usize());
    assert_eq!(e6, entity6);
  }

  #[test]
  fn minimum_bytes_needed() {
    assert_eq!(
      12,
      <GuidPrefix as Readable<Endianness>>::minimum_bytes_needed()
    );
  }

  serialization_test!( type = GuidPrefix,
  {
      guid_prefix_unknown,
      GuidPrefix::UNKNOWN,
      le = [0x00; 12],
      be = [0x00; 12]
  },
  {
      guid_prefix_default,
      GuidPrefix::default(),
      le = [0x00; 12],
      be = [0x00; 12]
  },
  {
      guid_prefix_endianness_insensitive,
      GuidPrefix {
          bytes: [0x00, 0x11, 0x22, 0x33, 0x44, 0x55,
                      0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB]
      },
      le = [0x00, 0x11, 0x22, 0x33, 0x44, 0x55,
            0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB],
      be = [0x00, 0x11, 0x22, 0x33, 0x44, 0x55,
            0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB]
  });

  serialization_test!( type = EntityId,
    {
        entity_unknown,
        EntityId::UNKNOWN,
        le = [0x00, 0x00, 0x00, 0x00],
        be = [0x00, 0x00, 0x00, 0x00]
    },
    {
        entity_default,
        EntityId::default(),
        le = [0x00, 0x00, 0x00, 0x00],
        be = [0x00, 0x00, 0x00, 0x00]
    },
    {
        entity_participant,
        EntityId::PARTICIPANT,
        le = [0x00, 0x00, 0x01, 0xC1],
        be = [0x00, 0x00, 0x01, 0xC1]
    },
    {
        entity_sedp_builtin_topic_writer,
        EntityId::SEDP_BUILTIN_TOPIC_WRITER,
        le = [0x00, 0x00, 0x02, 0xC2],
        be = [0x00, 0x00, 0x02, 0xC2]
    },
    {
        entity_sedp_builtin_topic_reader,
        EntityId::SEDP_BUILTIN_TOPIC_READER,
        le = [0x00, 0x00, 0x02, 0xC7],
        be = [0x00, 0x00, 0x02, 0xC7]
    },
    {
        entity_sedp_builtin_publications_writer,
        EntityId::SEDP_BUILTIN_PUBLICATIONS_WRITER,
        le = [0x00, 0x00, 0x03, 0xC2],
        be = [0x00, 0x00, 0x03, 0xC2]
    },
    {
        entity_sedp_builtin_publications_reader,
        EntityId::SEDP_BUILTIN_PUBLICATIONS_READER,
        le = [0x00, 0x00, 0x03, 0xC7],
        be = [0x00, 0x00, 0x03, 0xC7]
    },
    {
        entity_sedp_builtin_subscriptions_writer,
        EntityId::SEDP_BUILTIN_SUBSCRIPTIONS_WRITER,
        le = [0x00, 0x00, 0x04, 0xC2],
        be = [0x00, 0x00, 0x04, 0xC2]
    },
    {
        entity_sedp_builtin_subscriptions_reader,
        EntityId::SEDP_BUILTIN_SUBSCRIPTIONS_READER,
        le = [0x00, 0x00, 0x04, 0xC7],
        be = [0x00, 0x00, 0x04, 0xC7]
    },
    {
        entity_spdp_builtin_participant_writer,
        EntityId::SPDP_BUILTIN_PARTICIPANT_WRITER,
        le = [0x00, 0x01, 0x00, 0xC2],
        be = [0x00, 0x01, 0x00, 0xC2]
    },
    {
        entity_spdp_builtin_participant_reader,
        EntityId::SPDP_BUILTIN_PARTICIPANT_READER,
        le = [0x00, 0x01, 0x00, 0xC7],
        be = [0x00, 0x01, 0x00, 0xC7]
    },
    {
        entity_p2p_builtin_participant_message_writer,
        EntityId::P2P_BUILTIN_PARTICIPANT_MESSAGE_WRITER,
        le = [0x00, 0x02, 0x00, 0xC2],
        be = [0x00, 0x02, 0x00, 0xC2]
    },
    {
        entity_p2p_builtin_participant_message_reader,
        EntityId::P2P_BUILTIN_PARTICIPANT_MESSAGE_READER,
        le = [0x00, 0x02, 0x00, 0xC7],
        be = [0x00, 0x02, 0x00, 0xC7]
    }
  );

  #[test]
  fn guid_unknown_is_a_combination_of_unknown_members() {
    assert_eq!(
      GUID {
        entity_id: EntityId::UNKNOWN,
        prefix: GuidPrefix::UNKNOWN
      },
      GUID::GUID_UNKNOWN
    );
  }

  serialization_test!( type = GUID,
      {
          guid_unknown,
          GUID::GUID_UNKNOWN,
          le = [0x00; 16],
          be = [0x00; 16]
      },
      {
          guid_default,
          GUID::default(),
          le = [0x00; 16],
          be = [0x00; 16]
      },
      {
          guid_entity_id_on_the_last_position,
          GUID {
              entity_id: EntityId::PARTICIPANT,
              ..GUID::default()
          },
          le = [0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x01, 0xC1],
          be = [0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x01, 0xC1]
      }
  );
}
