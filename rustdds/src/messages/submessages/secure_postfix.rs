use speedy::{Readable, Writable};

use super::elements::crypto_footer::CryptoFooter;

/// SecurePostfixSubMsg: section 7.3.6.5 of the Security specification (v. 1.1)
/// See section 7.3.7.7
#[derive(Debug, PartialEq, Eq, Clone, Readable, Writable)]
pub struct SecurePostfix {
  submessage_length: u16, // ushort

  crypto_footer: CryptoFooter,
}
