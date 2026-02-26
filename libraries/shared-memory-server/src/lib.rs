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
    /// Create a new shared-memory server from an already-created `Shmem` region.
    ///
    /// # Safety
    ///
    /// The caller must ensure that:
    /// - `memory` points to a valid shared-memory region with sufficient size for
    ///   the channel header (events, disconnect flag, length, and data area).
    /// - No other `ShmemServer` is using the same `memory` region concurrently.
    /// - The memory region remains valid for the lifetime of this server.
    pub unsafe fn new(memory: Shmem) -> eyre::Result<Self> {
        Ok(Self {
            channel: unsafe { ShmemChannel::new_server(memory)? },
            reply_expected: false,
            phantom: PhantomData,
        })
    }

    pub fn listen(&mut self) -> eyre::Result<Option<T>>
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
    /// Create a new shared-memory client from an already-created `Shmem` region.
    ///
    /// # Safety
    ///
    /// The caller must ensure that:
    /// - `memory` points to a valid shared-memory region that was previously
    ///   initialized by a `ShmemServer`.
    /// - No other `ShmemClient` is using the same `memory` region concurrently.
    /// - The memory region remains valid for the lifetime of this client.
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

#[cfg(test)]
mod tests {
    use super::*;

    const SHMEM_SIZE: usize = 4096;

    fn create_shmem_pair() -> (Shmem, Shmem) {
        let server_mem = ShmemConf::new().size(SHMEM_SIZE).create().unwrap();
        let os_id = server_mem.get_os_id().to_string();
        let client_mem = ShmemConf::new().os_id(&os_id).open().unwrap();
        (server_mem, client_mem)
    }

    #[test]
    fn round_trip_request_reply() {
        let (server_mem, client_mem) = create_shmem_pair();
        let mut server: ShmemServer<String, String> =
            unsafe { ShmemServer::new(server_mem) }.unwrap();
        let mut client: ShmemClient<String, String> =
            unsafe { ShmemClient::new(client_mem, Some(Duration::from_secs(2))) }.unwrap();

        // Client runs in a thread with delay so server enters listen() first.
        // On Unix, shared memory events are edge-triggered, so the server must
        // be waiting before the client signals.
        let handle = std::thread::spawn(move || {
            std::thread::sleep(Duration::from_millis(100));
            client.request(&"hello".to_string()).unwrap()
        });

        let msg = server.listen().unwrap().unwrap();
        assert_eq!(msg, "hello");
        server.send_reply(&"world".to_string()).unwrap();

        let reply = handle.join().unwrap();
        assert_eq!(reply, "world");
    }

    #[test]
    fn multiple_request_replies() {
        let (server_mem, client_mem) = create_shmem_pair();
        let mut server: ShmemServer<u32, u32> = unsafe { ShmemServer::new(server_mem) }.unwrap();
        let mut client: ShmemClient<u32, u32> =
            unsafe { ShmemClient::new(client_mem, Some(Duration::from_secs(2))) }.unwrap();

        let handle = std::thread::spawn(move || {
            std::thread::sleep(Duration::from_millis(100));
            let mut replies = Vec::new();
            for i in 1..=3u32 {
                replies.push(client.request(&i).unwrap());
            }
            replies
        });

        for _ in 0..3 {
            let val = server.listen().unwrap().unwrap();
            server.send_reply(&(val * 2)).unwrap();
        }

        let replies = handle.join().unwrap();
        assert_eq!(replies, vec![2, 4, 6]);
    }

    #[test]
    fn client_disconnect_detected_by_server() {
        let (server_mem, client_mem) = create_shmem_pair();
        let mut server: ShmemServer<String, String> =
            unsafe { ShmemServer::new(server_mem) }.unwrap();
        let client: ShmemClient<String, String> =
            unsafe { ShmemClient::new(client_mem, Some(Duration::from_secs(1))) }.unwrap();

        // Drop client to trigger disconnect
        drop(client);

        // Server should detect disconnect (returns None)
        let result = server.listen().unwrap();
        assert!(result.is_none());
    }

    #[test]
    fn oversized_message_error() {
        // OS rounds up shmem to page boundary, so use a small region and a
        // message guaranteed to exceed it (1 MB > any page-rounded 256 bytes).
        let server_mem = ShmemConf::new().size(256).create().unwrap();
        let os_id = server_mem.get_os_id().to_string();
        let client_mem = ShmemConf::new().os_id(&os_id).open().unwrap();
        let _server: ShmemServer<Vec<u8>, Vec<u8>> =
            unsafe { ShmemServer::new(server_mem) }.unwrap();
        let mut client: ShmemClient<Vec<u8>, Vec<u8>> =
            unsafe { ShmemClient::new(client_mem, Some(Duration::from_secs(1))) }.unwrap();

        let large_msg = vec![0u8; 1024 * 1024];
        let result = client.request(&large_msg);
        assert!(result.is_err());
        let err_msg = format!("{:?}", result.unwrap_err());
        assert!(err_msg.contains("too large"), "unexpected error: {err_msg}");
    }
}
