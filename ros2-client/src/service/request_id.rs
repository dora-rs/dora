use serde::{Deserialize, Serialize};
use rustdds::{rpc::*, *};

// This replicates SequenceNumber from DDS
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct SequenceNumber {
  number: i64,
}

impl SequenceNumber {
  pub fn new(number: i64) -> SequenceNumber {
    SequenceNumber { number }
  }

  pub fn zero() -> SequenceNumber {
    SequenceNumber::new(0)
  }

  pub fn from_high_low(high: i32, low: u32) -> SequenceNumber {
    SequenceNumber {
      number: ((high as i64) << 32) + (low as i64),
    }
  }

  pub fn high(&self) -> i32 {
    (self.number >> 32) as i32
  }

  pub fn low(&self) -> u32 {
    (self.number & 0xFFFF_FFFF) as u32
  }

  pub fn next(&self) -> SequenceNumber {
    SequenceNumber {
      number: self.number + 1,
    }
  }
}

impl Default for SequenceNumber {
  fn default() -> SequenceNumber {
    SequenceNumber::new(1) // This is consistent with RustDDS SequenceNumber
                           // default value
  }
}

impl From<SequenceNumber> for i64 {
  fn from(sn: SequenceNumber) -> i64 {
    sn.number
  }
}

impl From<i64> for SequenceNumber {
  fn from(i: i64) -> SequenceNumber {
    SequenceNumber::new(i)
  }
}

impl From<rustdds::SequenceNumber> for SequenceNumber {
  fn from(sn: rustdds::SequenceNumber) -> SequenceNumber {
    SequenceNumber::new(i64::from(sn))
  }
}

/// [Original](https://docs.ros2.org/foxy/api/rmw/structrmw__request__id__t.html)
/// This structure seems to be identical in structure and function to
/// SampleIdentity defined by the RPC over DDS Spec.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct RmwRequestId {
  pub writer_guid: GUID,
  pub sequence_number: SequenceNumber,
}

impl From<RmwRequestId> for SampleIdentity {
  fn from(si: RmwRequestId) -> SampleIdentity {
    SampleIdentity {
      writer_guid: si.writer_guid,
      sequence_number: rustdds::SequenceNumber::from(i64::from(si.sequence_number)),
    }
  }
}

impl From<SampleIdentity> for RmwRequestId {
  fn from(si: SampleIdentity) -> RmwRequestId {
    RmwRequestId {
      writer_guid: si.writer_guid,
      sequence_number: SequenceNumber::from(si.sequence_number),
    }
  }
}

// [original](https://docs.ros2.org/foxy/api/rmw/structrmw__service__info__t.html)
// But where is this used?
//
// pub struct RmwServiceInfo {
//   pub source_timestamp: RmwTimePointValue,
//   pub received_timestamp: RmwTimePointValue,
//   pub request_id: RmwRequestId,
// }
