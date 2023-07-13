use std::{self, fmt::Display};

use serde::{de, ser};

pub type Result<T> = std::result::Result<T, Error>;

// This is a bare-bones implementation. A real library would provide additional
// information in its error type, for example the line and column at which the
// error occurred, the byte offset into the input, or the current key being
// processed.
#[derive(Debug, thiserror::Error)]
pub enum Error {
  // One or more variants that can be created by data structures through the
  // `ser::Error` and `de::Error` traits. For example the Serialize impl for
  // Mutex<T> might return an error because the mutex is poisoned, or the
  // Deserialize impl for a struct may return an error because a required
  // field is missing.
  #[error("{0}")]
  Message(String),

  #[error("io::Error: {0}")]
  Io(#[from] std::io::Error),

  #[error("CDR serialization requires sequence length to be specified at the start.")]
  SequenceLengthUnknown,

  // Zero or more variants that can be created directly by the Serializer and
  // Deserializer without going through `ser::Error` and `de::Error`.
  #[error("unexpected end of input")]
  Eof,

  #[error("Expected 0 or 1 as Boolean, got: {0}")]
  BadBoolean(u8),

  // was not valid UTF-8
  #[error("UTF-8 error: {0}")]
  BadString(std::str::Utf8Error),

  #[error("Bad Unicode character code: {0}")]
  BadChar(u32), // invalid Unicode codepoint

  #[error("Option value must have discriminant 0 or 1, read: {0}")]
  BadOption(u32), // Option variant tag (discriminant) is not 0 or 1

  #[error("Trailing garbage, {:?} bytes", .0.len())]
  TrailingCharacters(Vec<u8>),

  #[error("speedy::Error: {0}")]
  Speedy(#[from] speedy::Error),
}

impl ser::Error for Error {
  fn custom<T: Display>(msg: T) -> Self {
    Self::Message(msg.to_string())
  }
}

impl de::Error for Error {
  fn custom<T: Display>(msg: T) -> Self {
    Self::Message(msg.to_string())
  }
}
