use speedy::{Readable, Writable};

#[derive(Debug, PartialEq, Eq, Copy, Clone, Readable, Writable)]
pub struct BuiltinEndpointSet {
  value: u32,
}

impl BuiltinEndpointSet {
  // Note: Names are abbreviated. The common prefix is dropped, because
  // the data type BuiltinEndpointSet provides an enclosing namespace.
  pub const PARTICIPANT_ANNOUNCER: u32 = 0x00000001;
  pub const PARTICIPANT_DETECTOR: u32 = 0x000000002;
  pub const PUBLICATIONS_ANNOUNCER: u32 = 0x00000004;
  pub const PUBLICATIONS_DETECTOR: u32 = 0x00000008;
  pub const SUBSCRIPTIONS_ANNOUNCER: u32 = 0x00000010;
  pub const SUBSCRIPTIONS_DETECTOR: u32 = 0x00000020;

  pub const PARTICIPANT_MESSAGE_DATA_WRITER: u32 = 0x00000400;
  pub const PARTICIPANT_MESSAGE_DATA_READER: u32 = 0x00000800;

  // DDS Security spec v1.1
  // Section 7.4.1.4 Extension to RTPS Standard DCPSParticipants Builtin Topic
  // Table 11
  // TODO: Remove dead code attributes when these are used.
  #[allow(dead_code)]
  pub const PUBLICATIONS_SECURE_WRITER: u32 = 1 << 16;
  #[allow(dead_code)]
  pub const PUBLICATIONS_SECURE_READER: u32 = 1 << 17;

  #[allow(dead_code)]
  pub const SUBSCRIPTIONS_SECURE_WRITER: u32 = 1 << 18;
  #[allow(dead_code)]
  pub const SUBSCRIPTIONS_SECURE_READER: u32 = 1 << 19;

  #[allow(dead_code)]
  pub const PARTICIPANT_MESSAGE_SECURE_WRITER: u32 = 1 << 20;
  #[allow(dead_code)]
  pub const PARTICIPANT_MESSAGE_SECURE_READER: u32 = 1 << 21;

  #[allow(dead_code)]
  pub const PARTICIPANT_STATELESS_MESSAGE_WRITER: u32 = 1 << 22;
  #[allow(dead_code)]
  pub const PARTICIPANT_STATELESS_MESSAGE_READER: u32 = 1 << 23;

  #[allow(dead_code)]
  pub const PARTICIPANT_VOLATILE_MESSAGE_SECURE_WRITER: u32 = 1 << 24;
  #[allow(dead_code)]
  pub const PARTICIPANT_VOLATILE_MESSAGE_SECURE_READER: u32 = 1 << 25;

  #[allow(dead_code)]
  pub const PARTICIPANT_SECURE_WRITER: u32 = 1 << 26;
  #[allow(dead_code)]
  pub const PARTICIPANT_SECURE_READER: u32 = 1 << 27;

  // non-security again
  pub const TOPICS_ANNOUNCER: u32 = 0x08000000;
  pub const TOPICS_DETECTOR: u32 = 0x10000000;

  pub fn from_u32(val: u32) -> Self {
    Self { value: val }
  }

  pub fn contains(&self, other: u32) -> bool {
    (self.value & other) == other
  }
}

#[derive(Debug, PartialEq, Eq, Copy, Clone, Readable, Writable)]
pub struct BuiltinEndpointQos {
  value: u32,
}

impl BuiltinEndpointQos {
  pub const BEST_EFFORT_PARTICIPANT_MESSAGE_DATA_READER: u32 = 0x00000001;

  pub fn is_best_effort(&self) -> bool {
    self.value == Self::BEST_EFFORT_PARTICIPANT_MESSAGE_DATA_READER
  }
}
