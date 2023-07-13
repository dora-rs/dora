use std::hash::Hash;

use serde::{Deserialize, Serialize};

#[allow(unused_imports)] // since this is testing code only
use crate::{serialization::cdr_serializer::to_bytes, Key, Keyed};

#[derive(Debug, PartialOrd, PartialEq, Eq, Ord, Clone, Hash)]
pub struct RandomKey {
  val: i64,
}

impl RandomKey {
  #[allow(dead_code)] // just testing
  pub fn new(val: i64) -> Self {
    Self { val }
  }
}

// impl Key for RandomKey {
//}

#[derive(Serialize, Debug, Clone, PartialEq, Eq, Deserialize)]
pub struct RandomData {
  pub a: i64,
  pub b: String,
}

impl Keyed for RandomData {
  type K = i64;
  fn key(&self) -> i64 {
    self.a
  }
}
