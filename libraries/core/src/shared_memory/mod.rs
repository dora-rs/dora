use std::time::Duration;

use self::channel::ShmemChannel;
use eyre::{eyre, Context};
use serde::{Deserialize, Serialize};
use shared_memory::Shmem;

mod channel;

pub struct ShmemServer {
    channel: ShmemChannel,
    reply_expected: bool,
}

impl ShmemServer {
    pub unsafe fn new(memory: Shmem) -> eyre::Result<Self> {
        Ok(Self {
            channel: ShmemChannel::new_server(memory)?,
            reply_expected: false,
        })
    }

    pub fn listen<T>(&mut self) -> eyre::Result<Option<T>>
    where
        T: for<'a> Deserialize<'a> + std::fmt::Debug,
    {
        assert!(!self.reply_expected);
        let result = self.channel.receive(None);
        if matches!(result, Ok(Some(_))) {
            self.reply_expected = true;
        }

        result
    }

    pub fn send_reply<T>(&mut self, value: &T) -> eyre::Result<()>
    where
        T: Serialize + std::fmt::Debug,
    {
        assert!(self.reply_expected);
        self.channel.send(value)?;
        self.reply_expected = false;
        Ok(())
    }
}

pub struct ShmemClient {
    channel: ShmemChannel,
    timeout: Option<Duration>,
}

impl ShmemClient {
    pub unsafe fn new(memory: Shmem, timeout: Option<Duration>) -> eyre::Result<Self> {
        Ok(Self {
            channel: ShmemChannel::new_client(memory)?,
            timeout,
        })
    }

    pub fn request<T, U>(&mut self, value: &T) -> eyre::Result<U>
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
