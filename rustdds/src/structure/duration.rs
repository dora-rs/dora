use std::{convert::TryFrom, ops::Div};

use speedy::{Readable, Writable};
use serde::{Deserialize, Serialize};

#[derive(
  Debug,
  PartialEq,
  Eq,
  Hash,
  PartialOrd,
  Ord,
  Readable,
  Writable,
  Serialize,
  Deserialize,
  Copy,
  Clone,
)]
/// Duration is the DDS/RTPS representation for lengths of time, such as
/// timeouts. It is very similar to [`std::time::Duration`]. See also
/// [`Timestamp`](crate::Timestamp).
///
/// The resolution of `Duration` is 2^32 ticks per second.
/// Specified (as `Duration_t`) in RTPS spec Section 9.3.2
pub struct Duration {
  seconds: i32,
  fraction: u32, // unit is sec/2^32
}

impl Duration {
  pub const DURATION_ZERO: Self = Self {
    seconds: 0,
    fraction: 0,
  };

  pub const fn from_secs(secs: i32) -> Self {
    Self {
      seconds: secs, // loss of range here
      fraction: 0,
    }
  }

  pub fn from_frac_seconds(secs: f64) -> Self {
    Self {
      seconds: secs.trunc() as i32,
      fraction: (secs.fract().abs() * 32.0_f64.exp2()) as u32,
    }
  }

  pub const fn from_millis(millis: i64) -> Self {
    let fraction = (((millis % 1000) << 32) / 1000) as u32;

    Self {
      seconds: (millis / 1000) as i32,
      fraction,
    }
  }

  pub const fn from_nanos(nanos: i64) -> Self {
    let fraction = (((nanos % 1_000_000_000) << 32) / 1_000_000_000) as u32;

    Self {
      seconds: (nanos / 1_000_000_000) as i32,
      fraction,
    }
  }

  pub(crate) fn to_ticks(self) -> i64 {
    (i64::from(self.seconds) << 32) + i64::from(self.fraction)
  }

  pub(crate) fn from_ticks(ticks: i64) -> Self {
    Self {
      seconds: (ticks >> 32) as i32,
      fraction: ticks as u32,
    }
  }

  /* DURATION_INVALID is not part of the spec. And it is also dangerous, as it is plausible someone could
  legitimately measure such an interval, and others would interpret it as "invalid".
  pub const DURATION_INVALID: Self = Self {
    seconds: -1,
    fraction: 0xFFFFFFFF,
  };*/

  pub const DURATION_INFINITE: Self = Self {
    seconds: 0x7FFFFFFF,
    fraction: 0xFFFFFFFF,
  };

  pub fn to_nanoseconds(&self) -> i64 {
    ((i128::from(self.to_ticks()) * 1_000_000_000) >> 32) as i64
  }

  pub fn from_std(duration: std::time::Duration) -> Self {
    Self::from(duration)
  }

  pub fn to_std(&self) -> std::time::Duration {
    std::time::Duration::from(*self)
  }
}

impl From<Duration> for chrono::Duration {
  fn from(d: Duration) -> Self {
    Self::nanoseconds(d.to_nanoseconds())
  }
}

impl From<std::time::Duration> for Duration {
  fn from(duration: std::time::Duration) -> Self {
    Self {
      seconds: duration.as_secs() as i32,
      fraction: ((u64::from(duration.subsec_nanos()) << 32) / 1_000_000_000) as u32,
    }
  }
}

impl From<Duration> for std::time::Duration {
  fn from(d: Duration) -> Self {
    Self::from_nanos(
      u64::try_from(d.to_nanoseconds()).unwrap_or(0), /* saturate to zero, because
                                                       * std::time::Duration is unsigned */
    )
  }
}

impl Div<i64> for Duration {
  type Output = Self;

  fn div(self, rhs: i64) -> Self {
    Self::from_ticks(self.to_ticks() / rhs)
  }
}

impl std::ops::Add for Duration {
  type Output = Self;
  fn add(self, other: Self) -> Self {
    Self::from_ticks(self.to_ticks() + other.to_ticks())
  }
}

impl std::ops::Mul<Duration> for f64 {
  type Output = Duration;
  fn mul(self, rhs: Duration) -> Duration {
    Duration::from_ticks((self * (rhs.to_ticks() as Self)) as i64)
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  serialization_test!( type = Duration,
  {
      duration_zero,
      Duration::DURATION_ZERO,
      le = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
      be = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
  },
  {
      duration_infinite,
      Duration::DURATION_INFINITE,
      le = [0xFF, 0xFF, 0xFF, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF],
      be = [0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]
  },
  {
      duration_current_empty_fraction,
      Duration { seconds: 1_537_045_491, fraction: 0 },
      le = [0xF3, 0x73, 0x9D, 0x5B, 0x00, 0x00, 0x00, 0x00],
      be = [0x5B, 0x9D, 0x73, 0xF3, 0x00, 0x00, 0x00, 0x00]
  },
  {
      duration_from_wireshark,
      Duration { seconds: 1_519_152_760, fraction: 1_328_210_046 },
      le = [0x78, 0x6E, 0x8C, 0x5A, 0x7E, 0xE0, 0x2A, 0x4F],
      be = [0x5A, 0x8C, 0x6E, 0x78, 0x4F, 0x2A, 0xE0, 0x7E]
  });

  const NANOS_PER_SEC: u64 = 1_000_000_000;

  #[test]
  fn convert_from_duration() {
    let duration = std::time::Duration::from_nanos(1_519_152_761 * NANOS_PER_SEC + 328_210_046);
    let duration: Duration = duration.into();

    assert_eq!(
      duration,
      Duration {
        seconds: 1_519_152_761,
        fraction: 1_409_651_413,
      }
    );
  }

  #[test]
  fn convert_to_duration() {
    let duration = Duration {
      seconds: 1_519_152_760,
      fraction: 1_328_210_046,
    };
    let duration: std::time::Duration = duration.into();

    assert_eq!(
      duration,
      std::time::Duration::from_nanos(1_519_152_760 * NANOS_PER_SEC + 309_247_999)
    );
  }
}
