use speedy::{Readable, Writable};

/// CryptoContent: section 7.3.6.2.2 of the Security specification (v.
/// 1.1)
/// Should be interpreted by the plugin
#[derive(Debug, PartialEq, Eq, Clone, Readable, Writable)]
pub struct CryptoContent {
  pub data: Vec<u8>,
}
impl From<Vec<u8>> for CryptoContent {
  fn from(data: Vec<u8>) -> Self {
    Self { data }
  }
}
impl From<CryptoContent> for Vec<u8> {
  fn from(CryptoContent { data }: CryptoContent) -> Self {
    data
  }
}
