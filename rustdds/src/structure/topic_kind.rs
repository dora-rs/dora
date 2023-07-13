use speedy::{Readable, Writable};

/// Type for DDS Topic (With Key or No key)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Readable, Writable)]
#[repr(u32)]
pub enum TopicKind {
  NoKey = 1,
  WithKey = 2,
}

#[cfg(test)]
mod tests {
  use super::*;

  serialization_test!(type = TopicKind,
  {
      topic_kind_no_key,
      TopicKind::NoKey,
      le = [0x01, 0x00, 0x00, 0x00],
      be = [0x00, 0x00, 0x00, 0x01]
  },
  {
      topic_kind_with_key,
      TopicKind::WithKey,
      le = [0x02, 0x00, 0x00, 0x00],
      be = [0x00, 0x00, 0x00, 0x02]
  });
}
