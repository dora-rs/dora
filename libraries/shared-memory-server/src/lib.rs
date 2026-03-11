#![allow(clippy::missing_safety_doc)]

use self::channel::ShmemChannel;
use eyre::{Context, eyre};
use serde::{Deserialize, Serialize};
pub use shared_memory_extended::{Shmem, ShmemConf};
use std::marker::PhantomData;
use std::time::Duration;

mod channel;

pub struct ShmemServer<T, U> {
    channel: ShmemChannel,
    reply_expected: bool,
    phantom: PhantomData<(T, U)>,
}

impl<T, U> ShmemServer<T, U> {
    pub unsafe fn new(memory: Shmem) -> eyre::Result<Self> {
        Ok(Self {
            channel: unsafe { ShmemChannel::new_server(memory)? },
            reply_expected: false,
            phantom: PhantomData,
        })
    }

    /// Poll interval used by [`listen`] to avoid blocking indefinitely inside
    /// `pthread_cond_wait` when a node crashes without signalling disconnect.
    const LISTEN_POLL_INTERVAL: Duration = Duration::from_millis(50);

    /// Wait for the next client request, returning `Ok(None)` on clean client
    /// disconnect.  Returns `Err` after [`LISTEN_POLL_INTERVAL`] with no
    /// activity so the caller can check whether it should stop.
    pub fn listen(&mut self) -> eyre::Result<Option<T>>
    where
        T: for<'a> Deserialize<'a> + std::fmt::Debug,
    {
        assert!(!self.reply_expected);
        let result = self.channel.receive(Some(Self::LISTEN_POLL_INTERVAL));
        if matches!(result, Ok(Some(_))) {
            self.reply_expected = true;
        }

        result
    }

    pub fn send_reply(&mut self, value: &U) -> eyre::Result<()>
    where
        U: Serialize + std::fmt::Debug,
    {
        assert!(self.reply_expected);
        self.channel.send(value)?;
        self.reply_expected = false;
        Ok(())
    }
}

pub struct ShmemClient<T, U> {
    channel: ShmemChannel,
    timeout: Option<Duration>,
    phantom: PhantomData<(T, U)>,
}

impl<T, U> ShmemClient<T, U> {
    pub unsafe fn new(memory: Shmem, timeout: Option<Duration>) -> eyre::Result<Self> {
        Ok(Self {
            channel: unsafe { ShmemChannel::new_client(memory)? },
            timeout,
            phantom: PhantomData,
        })
    }

    pub fn request(&mut self, value: &T) -> eyre::Result<U>
    where
        T: Serialize + std::fmt::Debug,
        U: for<'a> Deserialize<'a> + std::fmt::Debug,
    {
        self.channel
            .send(value)
            .wrap_err("failed to send request")?;
        self.channel
            .receive(self.timeout)
            .wrap_err("failed to receive reply")?
            .ok_or_else(|| eyre!("server disconnected unexpectedly"))
    }
}
