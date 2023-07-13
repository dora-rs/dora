use speedy::{Readable, Writable};

use super::elements::crypto_content::CryptoContent;

/// SecureBodySubMsg: section 7.3.6.3 of the Security specification (v. 1.1)
#[derive(Debug, PartialEq, Eq, Clone, Readable, Writable)]
pub struct SecureBody {
  submessage_length: u16, // ushort

  crypto_content: CryptoContent,
}
