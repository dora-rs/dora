use std::io;

use crate::EncodedTransport;

use async_trait::async_trait;
pub use framed::FramedTransport;
pub use shmem::ShmemTransport;

mod framed;
mod shmem;

pub trait Transport<Req, Resp>
where
    Req: ?Sized,
{
    fn send(&mut self, item: &Req) -> io::Result<()>;
    fn receive(&mut self) -> io::Result<Option<Resp>>;

    fn with_encoding<Encoding, Req2: ?Sized, Resp2>(
        self,
        encoding: Encoding,
    ) -> EncodedTransport<Self, Encoding, Req2, Resp2>
    where
        Self: Sized,
    {
        EncodedTransport::new(self, encoding)
    }
}

#[async_trait]
pub trait AsyncTransport<Req, Resp>
where
    Req: ?Sized,
{
    async fn send(&mut self, item: &Req) -> io::Result<()>;
    async fn receive(&mut self) -> io::Result<Option<Resp>>;

    fn with_encoding<Encoding, Req2: ?Sized, Resp2>(
        self,
        encoding: Encoding,
    ) -> EncodedTransport<Self, Encoding, Req2, Resp2>
    where
        Self: Sized,
    {
        EncodedTransport::new(self, encoding)
    }
}
