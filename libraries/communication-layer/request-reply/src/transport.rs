use std::io;

use crate::{EncodedTransport, Protocol};

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
    /// Receives an item. If the underlying transport is closed (finished), returns Ok(None).
    /// Otherwise, returns Ok(Some(item)) or an error.
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

    fn into_server<P>(self) -> EncodedTransport<Self, P::Encoding, P::Response, P::Request>
    where
        Self: Sized,
        P: Protocol,
        P::Encoding: Default,
    {
        EncodedTransport::new(self, P::Encoding::default())
    }
    fn into_client<P>(self) -> EncodedTransport<Self, P::Encoding, P::Request, P::Response>
    where
        Self: Sized,
        P: Protocol,
        P::Encoding: Default,
    {
        EncodedTransport::new(self, P::Encoding::default())
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

    fn into_server<P>(self) -> EncodedTransport<Self, P::Encoding, P::Response, P::Request>
    where
        Self: Sized,
        P: Protocol,
        P::Encoding: Default,
    {
        EncodedTransport::new(self, P::Encoding::default())
    }
    fn into_client<P>(self) -> EncodedTransport<Self, P::Encoding, P::Request, P::Response>
    where
        Self: Sized,
        P: Protocol,
        P::Encoding: Default,
    {
        EncodedTransport::new(self, P::Encoding::default())
    }
}

pub type ServerTransport<IO, P> = EncodedTransport<
    FramedTransport<IO>,
    <P as Protocol>::Encoding,
    <P as Protocol>::Response,
    <P as Protocol>::Request,
>;
pub type ClientTransport<IO, P> = EncodedTransport<
    FramedTransport<IO>,
    <P as Protocol>::Encoding,
    <P as Protocol>::Request,
    <P as Protocol>::Response,
>;
