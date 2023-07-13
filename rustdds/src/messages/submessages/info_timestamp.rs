use crate::structure::time::Timestamp;

/// This message modifies the logical source of the Submessages
/// that follow.
#[derive(Debug, PartialEq, Eq, Clone)]
// We cannot use the Speedy-derived Writable/Readable impls, because
// The content is either serialized or not, depending on flags.
pub struct InfoTimestamp {
  /// Contains the timestamp that should be used to interpret the
  /// subsequent Submessages
  ///
  /// Present only if the InvalidateFlag is not set in the header.
  pub timestamp: Option<Timestamp>,
}
