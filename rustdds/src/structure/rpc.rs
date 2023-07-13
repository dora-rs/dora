//! Structures for Remote Procedure Call over DDS, v1.0
//!
//! See the OMG Specification
//
use serde::{Deserialize, Serialize};
use speedy::{Readable, Writable};

use crate::structure::{guid::*, sequence_number::*};

// Spec Section 7.5.1.1.1 Common Types
#[derive(
  Copy,
  Clone,
  Default,
  Debug,
  PartialOrd,
  PartialEq,
  Ord,
  Eq,
  Readable,
  Writable,
  Hash,
  Serialize,
  Deserialize,
)]
// We also derive Copy, although this is a bit large: 32 bytes
// But on 64-bit computers that is only 4 machine words.
pub struct SampleIdentity {
  pub writer_guid: GUID,
  pub sequence_number: SequenceNumber,
}

#[derive(
  Clone,
  Copy,
  Debug,
  PartialOrd,
  PartialEq,
  Ord,
  Eq,
  Readable,
  Writable,
  Hash,
  Serialize,
  Deserialize,
)]
// TODO: Where are the binary serialization values for these specified? Nowhere?
pub enum RemoteExceptionCode {
  Ok,
  Unsupported,
  InvalidArgument,
  OutOfResources,
  UnknownOperation,
  UnknownException,
}

impl Default for RemoteExceptionCode {
  fn default() -> Self {
    Self::UnknownException
  }
}

#[derive(
  Clone, Default, PartialOrd, PartialEq, Ord, Eq, Readable, Writable, Hash, Serialize, Deserialize,
)]
pub struct RequestHeader {
  pub request_id: SampleIdentity,
  pub instance_name: String, // limit to 255 characters
}

#[derive(
  Clone, Default, PartialOrd, PartialEq, Ord, Eq, Readable, Writable, Hash, Serialize, Deserialize,
)]
pub struct ReplyHeader {
  pub related_request_id: SampleIdentity,
  pub remote_ex: RemoteExceptionCode,
}
