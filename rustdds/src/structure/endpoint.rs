use speedy::{Readable, Writable};

// TODO: Why is this module here? Can we move this to QoS?

#[derive(Debug, Clone, Copy, PartialEq, Eq, Readable, Writable)]
#[repr(u32)]
pub enum ReliabilityKind {
  BestEffort = 1,
  Reliable = 2,
}

// impl ReliabilityKind {
//   pub const BEST_EFFORT: Self = Self(1);
//   pub const RELIABLE: Self = Self(2);
// }

#[cfg(test)]
mod tests {
  use super::*;

  serialization_test!( type = ReliabilityKind,
  {
      reliability_kind_best_effort,
      ReliabilityKind::BestEffort,
      le = [0x01, 0x00, 0x00, 0x00],
      be = [0x00, 0x00, 0x00, 0x01]
  },
  {
      reliability_kind_reliable,
      ReliabilityKind::Reliable,
      le = [0x02, 0x00, 0x00, 0x00],
      be = [0x00, 0x00, 0x00, 0x02]
  });
}
