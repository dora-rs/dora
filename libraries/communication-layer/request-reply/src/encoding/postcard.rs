use crate::encoding::{Decoder, Encoder};

use serde::{Serialize, de::DeserializeOwned};
use std::{io, mem};

#[derive(Clone, Debug, Default)]
pub struct PostcardEncoding;

impl<T: Serialize> Encoder<T> for PostcardEncoding {
    fn encode(&self, item: &T, dest: &mut Vec<u8>) -> Result<(), io::Error> {
        let temp = mem::take(dest);
        *dest = postcard::to_extend(item, temp)
            .map_err(|e| io::Error::other(format!("Postcard serialization error: {e}")))?;
        Ok(())
    }
}

impl<T: DeserializeOwned> Decoder<T> for PostcardEncoding {
    fn decode(&self, buf: &[u8]) -> Result<T, io::Error> {
        postcard::from_bytes::<T>(buf)
            .map_err(|e| io::Error::other(format!("Postcard deserialization error: {e}")))
    }
}
