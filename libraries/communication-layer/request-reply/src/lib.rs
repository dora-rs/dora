#![cfg_attr(docsrs, feature(doc_auto_cfg))]

//! Abstraction of various request/reply communication backends.
//!
//! Provides a [`RequestReplyLayer`] trait as an abstraction for different request/reply
//! systems. The following set of backends are currently supported:
//!
//! TODO

pub use encoding::EncodedTransport;
pub use tcp::*;
pub use transport::{AsyncTransport, Transport};

use crate::{
    encoding::{Decoder, Encoder},
    transport::{ClientTransport, FramedTransport, ServerTransport},
};

pub mod encoding;
mod tcp;
pub mod transport;

pub type TcpRequestReplyConnection =
    dyn RequestReplyConnection<RequestData = Vec<u8>, ReplyData = Vec<u8>, Error = std::io::Error>;

/// Abstraction trait for different publisher/subscriber implementations.
pub trait RequestReplyLayer: Send + Sync {
    type Address;
    type RequestData;
    type ReplyData;
    type Error;

    #[allow(clippy::type_complexity)]
    fn listen(
        &mut self,
        addr: Self::Address,
    ) -> Result<
        Box<
            dyn Iterator<
                Item = Result<
                    Box<
                        dyn ListenConnection<
                                RequestData = Self::RequestData,
                                ReplyData = Self::ReplyData,
                                Error = Self::Error,
                            >,
                    >,
                    Self::Error,
                >,
            >,
        >,
        Self::Error,
    >;

    #[allow(clippy::type_complexity)]
    fn connect(
        &mut self,
        addr: Self::Address,
    ) -> Result<
        Box<
            dyn RequestReplyConnection<
                    RequestData = Self::RequestData,
                    ReplyData = Self::ReplyData,
                    Error = Self::Error,
                >,
        >,
        Self::Error,
    >;
}

pub trait ListenConnection: Send + Sync {
    type RequestData;
    type ReplyData;
    type Error;

    #[allow(clippy::type_complexity)]
    fn handle_next(
        &mut self,
        handler: Box<dyn FnOnce(Self::RequestData) -> Result<Self::ReplyData, Self::Error>>,
    ) -> Result<(), Self::Error>;
}

pub trait RequestReplyConnection: Send + Sync {
    type RequestData;
    type ReplyData;
    type Error;

    fn request(&mut self, request: &Self::RequestData) -> Result<Self::ReplyData, Self::Error>;
}

impl<T: RequestReplyConnection + ?Sized> RequestReplyConnection for &mut T {
    type RequestData = T::RequestData;
    type ReplyData = T::ReplyData;
    type Error = T::Error;
    fn request(&mut self, request: &Self::RequestData) -> Result<Self::ReplyData, Self::Error> {
        (**self).request(request)
    }
}

pub trait Protocol {
    type Encoding: Encoder<Self::Request>
        + Decoder<Self::Request>
        + Encoder<Self::Response>
        + Decoder<Self::Response>
        + Default;
    type Request;
    type Response;

    fn bind<IO>(io: IO) -> ServerTransport<IO, Self>
    where
        IO: std::io::Read + std::io::Write,
    {
        EncodedTransport::new(FramedTransport::new(io), Self::Encoding::default())
    }
    fn bind_async<IO>(io: IO) -> ServerTransport<IO, Self>
    where
        IO: tokio::io::AsyncRead + tokio::io::AsyncWrite + Unpin + Send,
    {
        EncodedTransport::new(FramedTransport::new(io), Self::Encoding::default())
    }

    fn connect<IO>(io: IO) -> ClientTransport<IO, Self>
    where
        IO: std::io::Read + std::io::Write,
    {
        EncodedTransport::new(FramedTransport::new(io), Self::Encoding::default())
    }
    fn connect_async<IO>(io: IO) -> ClientTransport<IO, Self>
    where
        IO: tokio::io::AsyncRead + tokio::io::AsyncWrite + Unpin + Send,
    {
        EncodedTransport::new(FramedTransport::new(io), Self::Encoding::default())
    }
}
