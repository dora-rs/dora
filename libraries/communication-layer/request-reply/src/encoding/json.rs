use crate::encoding::{Decoder, Encoder};

use serde::{de::DeserializeOwned, Serialize};
use std::io;

pub struct JsonEncoding;

impl<T: Serialize> Encoder<T> for JsonEncoding {
    fn encode(&self, item: &T, dest: &mut Vec<u8>) -> Result<(), io::Error> {
        serde_json::to_writer(dest, item)
            .map_err(|e| io::Error::other(format!("JSON serialization error: {e}")))
    }
}

impl<T: DeserializeOwned> Decoder<T> for JsonEncoding {
    fn decode(&self, buf: &[u8]) -> Result<T, io::Error> {
        serde_json::from_slice(buf)
            .map_err(|e| io::Error::other(format!("JSON deserialization error: {e}")))
    }
}

