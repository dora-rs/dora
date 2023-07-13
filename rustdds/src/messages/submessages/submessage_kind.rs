use std::{fmt, fmt::Debug};

use speedy::{Readable, Writable};

#[derive(PartialEq, Eq, Readable, Writable, Clone, Copy)]
pub struct SubmessageKind {
  value: u8,
}

impl From<SubmessageKind> for u8 {
  fn from(s: SubmessageKind) -> Self {
    s.value
  }
}

impl SubmessageKind {
  pub const PAD: Self = Self { value: 0x01 };
  pub const ACKNACK: Self = Self { value: 0x06 };
  pub const HEARTBEAT: Self = Self { value: 0x07 };
  pub const GAP: Self = Self { value: 0x08 };
  pub const INFO_TS: Self = Self { value: 0x09 };
  pub const INFO_SRC: Self = Self { value: 0x0c };
  pub const INFO_REPLY_IP4: Self = Self { value: 0x0d };
  pub const INFO_DST: Self = Self { value: 0x0e };
  pub const INFO_REPLY: Self = Self { value: 0x0f };
  pub const NACK_FRAG: Self = Self { value: 0x12 };
  pub const HEARTBEAT_FRAG: Self = Self { value: 0x13 };
  pub const DATA: Self = Self { value: 0x15 };
  pub const DATA_FRAG: Self = Self { value: 0x16 };
  pub const SEC_BODY: Self = Self { value: 0x30 }; // Section 7.3.7.5.2 of the Security specification (v. 1.1)
  pub const SEC_PREFIX: Self = Self { value: 0x31 }; // Section 7.3.7.6.2 of the Security specification (v. 1.1)
  pub const SEC_POSTFIX: Self = Self { value: 0x32 }; // Section 7.3.7.7.2 of the Security specification (v. 1.1)
  pub const SRTPS_PREFIX: Self = Self { value: 0x33 }; // Section 7.3.7.8.2 of the Security specification (v. 1.1)
  pub const SRTPS_POSTFIX: Self = Self { value: 0x34 }; // Section 7.3.7.9.2 of the Security specification (v. 1.1)
}

impl Debug for SubmessageKind {
  fn fmt(&self, fmt: &mut fmt::Formatter<'_>) -> fmt::Result {
    match *self {
      Self::PAD => fmt.write_str("PAD"),
      Self::ACKNACK => fmt.write_str("ACKNACK"),
      Self::HEARTBEAT => fmt.write_str("HEARTBEAT"),
      Self::GAP => fmt.write_str("GAP"),
      Self::INFO_TS => fmt.write_str("INFO_TS"),
      Self::INFO_SRC => fmt.write_str("INFO_SRC"),
      Self::INFO_REPLY_IP4 => fmt.write_str("INFO_REPLY_IP4"),
      Self::INFO_DST => fmt.write_str("INFO_DST"),
      Self::INFO_REPLY => fmt.write_str("INFO_REPLY"),
      Self::NACK_FRAG => fmt.write_str("NACK_FRAG"),
      Self::HEARTBEAT_FRAG => fmt.write_str("HEARTBEAT_FRAG"),
      Self::DATA => fmt.write_str("DATA"),
      Self::DATA_FRAG => fmt.write_str("DATA_FRAG"),
      Self::SEC_BODY => fmt.write_str("SEC_BODY"),
      Self::SEC_PREFIX => fmt.write_str("SEC_PREFIX"),
      Self::SEC_POSTFIX => fmt.write_str("SEC_POSTFIX"),
      Self::SRTPS_PREFIX => fmt.write_str("SRTPS_PREFIX"),
      Self::SRTPS_POSTFIX => fmt.write_str("SRTPS_POSTFIX"),
      Self { value: other } => fmt.write_fmt(format_args!("SubmessageKind {} (UNKNOWN!)", other)),
    }
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  serialization_test!( type = SubmessageKind,
  {
      submessage_kind_pad,
      SubmessageKind::PAD,
      le = [0x01],
      be = [0x01]
  },
  {
      submessage_kind_acknack,
      SubmessageKind::ACKNACK,
      le = [0x06],
      be = [0x06]
  },
  {
      submessage_kind_heartbeat,
      SubmessageKind::HEARTBEAT,
      le = [0x07],
      be = [0x07]
  },
  {
      submessage_kind_gap,
      SubmessageKind::GAP,
      le = [0x08],
      be = [0x08]
  },
  {
      submessage_kind_info_ts,
      SubmessageKind::INFO_TS,
      le = [0x09],
      be = [0x09]
  },
  {
      submessage_kind_info_src,
      SubmessageKind::INFO_SRC,
      le = [0x0c],
      be = [0x0c]
  },
  {
      submessage_kind_info_replay_ip4,
      SubmessageKind::INFO_REPLY_IP4,
      le = [0x0d],
      be = [0x0d]
  },
  {
      submessage_kind_info_dst,
      SubmessageKind::INFO_DST,
      le = [0x0e],
      be = [0x0e]
  },
  {
      submessage_kind_info_replay,
      SubmessageKind::INFO_REPLY,
      le = [0x0f],
      be = [0x0f]
  },
  {
      submessage_kind_nack_frag,
      SubmessageKind::NACK_FRAG,
      le = [0x12],
      be = [0x12]
  },
  {
      submessage_kind_heartbeat_frag,
      SubmessageKind::HEARTBEAT_FRAG,
      le = [0x13],
      be = [0x13]
  },
  {
      submessage_kind_data,
      SubmessageKind::DATA,
      le = [0x15],
      be = [0x15]
  },
  {
      submessage_kind_data_frag,
      SubmessageKind::DATA_FRAG,
      le = [0x16],
      be = [0x16]
  });
}
