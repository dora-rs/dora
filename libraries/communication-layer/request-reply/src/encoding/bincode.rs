use crate::encoding::{Decoder, Encoder};

use serde::{Serialize, de::DeserializeOwned};
use std::io;

pub struct BincodeEncoding;

impl<T: Serialize> Encoder<T> for BincodeEncoding {
    fn encode(&self, item: &T, dest: &mut Vec<u8>) -> Result<(), io::Error> {
        bincode::serialize_into(dest, item)
            .map_err(|e| io::Error::other(format!("Bincode serialization error: {e}")))
    }
}

impl<T: DeserializeOwned> Decoder<T> for BincodeEncoding {
    fn decode(&self, buf: &[u8]) -> Result<T, io::Error> {
        bincode::deserialize(buf)
            .map_err(|e| io::Error::other(format!("Bincode deserialization error: {e}")))
    }
}
