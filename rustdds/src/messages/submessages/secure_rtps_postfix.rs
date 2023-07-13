use speedy::{Readable, Writable};

use super::elements::crypto_footer::CryptoFooter;

/// SecureRTPSPostfixSubMsg: section 7.3.6.7 of the Security specification (v.
/// 1.1) See section 7.3.7.9
#[derive(Debug, PartialEq, Eq, Clone, Readable, Writable)]
pub struct SecureRTPSPostfix {
  submessage_length: u16, // ushort

  crypto_footer: CryptoFooter,
}
