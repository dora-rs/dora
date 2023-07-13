use speedy::{Readable, Writable};

use super::elements::crypto_header::CryptoHeader;

/// SecurePrefixSubMsg: section 7.3.6.4 of the Security specification (v. 1.1)
/// See sections 7.3.7.3 and 7.3.7.6.1
#[derive(Debug, PartialEq, Eq, Clone, Readable, Writable)]
pub struct SecurePrefix {
  submessage_length: u16, // ushort

  crypto_header: CryptoHeader,
}
