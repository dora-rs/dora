use crate::encoding::{Decoder, Encoder};

use serde::{Serialize, de::DeserializeOwned};
use std::io;

#[derive(Clone, Debug, Default)]
pub struct YamlEncoding;

impl<T: Serialize> Encoder<T> for YamlEncoding {
    fn encode(&self, item: &T, dest: &mut Vec<u8>) -> Result<(), io::Error> {
        // serde_yaml writes to any Write implementor
        serde_yaml::to_writer(dest, item)
            .map_err(|e| io::Error::other(format!("YAML serialization error: {e}")))
    }
}

impl<T: DeserializeOwned> Decoder<T> for YamlEncoding {
    fn decode(&self, buf: &[u8]) -> Result<T, io::Error> {
        serde_yaml::from_slice(buf)
            .map_err(|e| io::Error::other(format!("YAML deserialization error: {e}")))
    }
}
