use speedy::{Readable, Writable};

use crate::security::cryptographic::types::CryptoTransformIdentifier;

/// CryptoHeader: section 7.3.6.2.3 of the Security specification (v.
/// 1.1)
/// See section 7.3.7.3
#[derive(Debug, PartialEq, Eq, Clone, Readable, Writable)]
pub struct CryptoHeader {
  pub transformation_id: CryptoTransformIdentifier,
  pub plugin_crypto_header_extra: PluginCryptoHeaderExtra,
}

/// Should be interpreted by the plugin based on `transformation_id`
#[derive(Debug, PartialEq, Eq, Clone, Readable, Writable)]
pub struct PluginCryptoHeaderExtra {
  pub data: Vec<u8>,
}
impl From<Vec<u8>> for PluginCryptoHeaderExtra {
  fn from(data: Vec<u8>) -> Self {
    Self { data }
  }
}
