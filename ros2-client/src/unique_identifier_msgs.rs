use std::fmt;

use serde::{Deserialize, Serialize};
use uuid::Uuid;

use crate::message::Message;

// deriving also Copy here is a bit on the expensive side, but makes life easier
#[derive(Clone, Copy, Serialize, Deserialize, Eq, PartialEq, Ord, PartialOrd)]
pub struct UUID {
  #[serde(with = "uuid::serde::compact")] // straightforward binary serialization, not text
  pub uuid: Uuid,
}
impl Message for UUID {}

impl fmt::Debug for UUID {
  fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
    fmt::Display::fmt(self.uuid.as_simple(), f)
  }
}

impl UUID {
  pub const ZERO: UUID = UUID { uuid: Uuid::nil() };

  pub fn new_random() -> Self {
    UUID {
      uuid: Uuid::new_v4(),
    }
  }
}

// #[cfg(test)]
// mod tests {

//   #[test]
//   fn test_serialize() {
//     let
//   }

// }
