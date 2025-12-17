#![cfg_attr(docsrs, feature(doc_auto_cfg))]

//! Abstraction of various request/reply communication backends.
//!
//! Provides a [`RequestReplyLayer`] trait as an abstraction for different request/reply
//! systems. The following set of backends are currently supported:
//!
//! TODO

pub use tcp::*;

mod tcp;

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
