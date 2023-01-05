use self::channel::ShmemChannel;
use eyre::eyre;
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
        let result = self.channel.receive();
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
}

impl ShmemClient {
    pub unsafe fn new(memory: Shmem) -> eyre::Result<Self> {
        Ok(Self {
            channel: ShmemChannel::new_client(memory)?,
        })
    }

    pub fn request<T, U>(&mut self, value: &T) -> eyre::Result<U>
    where
        T: Serialize + std::fmt::Debug,
        U: for<'a> Deserialize<'a> + std::fmt::Debug,
    {
        self.channel.send(value)?;
        self.channel
            .receive()?
            .ok_or_else(|| eyre!("server disconnected unexpectedly"))
    }
}
