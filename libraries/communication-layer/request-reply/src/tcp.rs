use std::{
    io::{Read, Write},
    net::{SocketAddr, TcpListener, TcpStream},
};

use crate::{ListenConnection, RequestReplyConnection, RequestReplyLayer};

pub type TcpRequestReplyConnection =
    dyn RequestReplyConnection<RequestData = Vec<u8>, ReplyData = Vec<u8>, Error = std::io::Error>;

pub struct TcpLayer {}

impl TcpLayer {
    pub fn new() -> Self {
        Self {}
    }
}

impl RequestReplyLayer for TcpLayer {
    type Address = SocketAddr;
    type RequestData = Vec<u8>;
    type ReplyData = Vec<u8>;
    type Error = std::io::Error;

    fn listen(
        &mut self,
        addr: Self::Address,
    ) -> Result<
        Box<
            dyn Iterator<
                Item = Result<
                    Box<
                        dyn crate::ListenConnection<
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
    > {
        let incoming: Box<dyn Iterator<Item = Result<_, _>>> = Box::new(
            IntoIncoming {
                listener: TcpListener::bind(addr)?,
            }
            .map(|r| {
                r.map(|stream| {
                    let connection: Box<
                        dyn ListenConnection<
                            RequestData = Self::RequestData,
                            ReplyData = Self::ReplyData,
                            Error = Self::Error,
                        >,
                    > = Box::new(TcpConnection { stream });
                    connection
                })
            }),
        );
        Ok(incoming)
    }

    fn connect(
        &mut self,
        addr: Self::Address,
    ) -> Result<
        Box<
            dyn crate::RequestReplyConnection<
                RequestData = Self::RequestData,
                ReplyData = Self::ReplyData,
                Error = Self::Error,
            >,
        >,
        Self::Error,
    > {
        TcpStream::connect(addr).map(|s| {
            let connection: Box<
                dyn RequestReplyConnection<
                    RequestData = Self::RequestData,
                    ReplyData = Self::ReplyData,
                    Error = Self::Error,
                >,
            > = Box::new(TcpConnection { stream: s });
            connection
        })
    }
}

struct TcpConnection {
    stream: TcpStream,
}

impl ListenConnection for TcpConnection {
    type RequestData = Vec<u8>;
    type ReplyData = Vec<u8>;
    type Error = std::io::Error;

    fn handle_next(
        &mut self,
        handler: Box<dyn FnOnce(Self::RequestData) -> Result<Self::ReplyData, Self::Error>>,
    ) -> Result<(), Self::Error> {
        let request = self.receive()?;
        let reply = handler(request)?;
        self.send(&reply)?;
        Ok(())
    }
}

impl RequestReplyConnection for TcpConnection {
    type RequestData = Vec<u8>;
    type ReplyData = Vec<u8>;
    type Error = std::io::Error;

    fn request(&mut self, request: &Self::RequestData) -> Result<Self::ReplyData, Self::Error> {
        self.send(request)?;
        let reply = self.receive()?;

        Ok(reply)
    }
}

impl TcpConnection {
    fn send(&mut self, request: &[u8]) -> std::io::Result<()> {
        let len_raw = (request.len() as u64).to_le_bytes();
        self.stream.write_all(&len_raw)?;
        self.stream.write_all(&request)?;
        Ok(())
    }

    fn receive(&mut self) -> std::io::Result<Vec<u8>> {
        let reply_len = {
            let mut raw = [0; 8];
            self.stream.read_exact(&mut raw)?;
            u64::from_le_bytes(raw) as usize
        };
        let mut reply = vec![0; reply_len];
        self.stream.read_exact(&mut reply)?;
        Ok(reply)
    }
}

// taken from `std::net::tcp` module (still unstable)
pub struct IntoIncoming {
    listener: TcpListener,
}

impl Iterator for IntoIncoming {
    type Item = std::io::Result<TcpStream>;
    fn next(&mut self) -> Option<std::io::Result<TcpStream>> {
        Some(self.listener.accept().map(|p| p.0))
    }
}

impl std::iter::FusedIterator for IntoIncoming {}
