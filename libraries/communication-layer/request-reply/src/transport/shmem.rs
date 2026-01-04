use std::{io, time::Duration};

use shared_memory_server::ShmemChannel;

use crate::Transport;

pub struct ShmemTransport {
    inner: ShmemChannel,
    timeout: Option<Duration>,
}
impl ShmemTransport {
    pub fn new(inner: ShmemChannel, timeout: Option<Duration>) -> Self {
        Self { inner, timeout }
    }

    pub fn inner_mut(&mut self) -> &mut ShmemChannel {
        &mut self.inner
    }

    pub fn into_inner(self) -> ShmemChannel {
        self.inner
    }
}

impl Transport<[u8], Vec<u8>> for ShmemTransport {
    fn send(&mut self, item: &[u8]) -> io::Result<()> {
        self.inner.send(item).map_err(io::Error::other)
    }

    fn receive(&mut self) -> io::Result<Option<Vec<u8>>> {
        self.inner.receive(self.timeout).map_err(io::Error::other)
    }
}
