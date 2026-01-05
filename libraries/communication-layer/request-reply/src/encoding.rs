use std::io;
use std::marker::PhantomData;

#[cfg(feature = "bincode")]
pub use bincode::BincodeEncoding;
#[cfg(feature = "postcard")]
pub use postcard::PostcardEncoding;
#[cfg(feature = "json")]
pub use json::JsonEncoding;
#[cfg(feature = "yaml")]
pub use yaml::YamlEncoding;

use crate::Transport;

#[cfg(feature = "bincode")]
mod bincode;
#[cfg(feature = "postcard")]
mod postcard;
#[cfg(feature = "json")]
mod json;
#[cfg(feature = "yaml")]
mod yaml;

pub trait Encoder<T: ?Sized> {
    fn encode(&self, item: &T, dest: &mut Vec<u8>) -> Result<(), io::Error>;
}
pub trait Decoder<T> {
    fn decode(&self, buf: &[u8]) -> Result<T, io::Error>;
}

pub struct EncodedTransport<Inner, Encoding, Req: ?Sized, Resp> {
    inner: Inner,
    buf: Vec<u8>,
    encoding: Encoding,
    _phantom1: PhantomData<Req>,
    _phantom2: PhantomData<Resp>,
}

impl<T, Encoding, Req: ?Sized, Resp> EncodedTransport<T, Encoding, Req, Resp> {
    pub fn new(inner: T, encoding: Encoding) -> Self {
        Self {
            inner,
            buf: Vec::new(),
            encoding,
            _phantom1: PhantomData,
            _phantom2: PhantomData,
        }
    }

    pub fn inner_mut(&mut self) -> &mut T {
        &mut self.inner
    }
}

impl<T, Encoding, Req: ?Sized, Resp> Transport<Req, Resp>
    for EncodedTransport<T, Encoding, Req, Resp>
where
    T: Transport<[u8], Vec<u8>>,
    Encoding: Encoder<Req> + Decoder<Resp>,
{
    fn send(&mut self, item: &Req) -> io::Result<()> {
        self.buf.clear();
        self.encoding.encode(item, &mut self.buf)?;
        self.inner.send(&self.buf)?;
        Ok(())
    }

    fn receive(&mut self) -> io::Result<Option<Resp>> {
        self.inner
            .receive()?
            .map(|data| self.encoding.decode(&data))
            .transpose()
    }
}
