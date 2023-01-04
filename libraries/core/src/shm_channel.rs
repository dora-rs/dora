use std::{mem, slice, sync::atomic::AtomicU64, time::Duration};

use eyre::{eyre, Context};
use raw_sync::events::{Event, EventImpl, EventInit, EventState};
use serde::{Deserialize, Serialize};
use shared_memory::Shmem;

/// Size of the encoded length in bytes.
const LEN_LEN: usize = mem::size_of::<AtomicU64>();

pub struct ShmemChannel {
    memory: Shmem,
    server_event: Box<dyn EventImpl>,
    client_event: Box<dyn EventImpl>,
    buffer_start_offset: usize,
    server: bool,
}

#[allow(clippy::missing_safety_doc)]
impl ShmemChannel {
    pub unsafe fn new_server(memory: Shmem) -> eyre::Result<Self> {
        let (server_event, server_event_len) = unsafe { Event::new(memory.as_ptr(), true) }
            .map_err(|err| eyre!("failed to open raw server event: {err}"))?;
        let (client_event, client_event_len) =
            unsafe { Event::new(memory.as_ptr().wrapping_add(server_event_len), true) }
                .map_err(|err| eyre!("failed to open raw client event: {err}"))?;
        let buffer_start_offset = server_event_len + client_event_len;

        server_event
            .set(EventState::Clear)
            .map_err(|err| eyre!("failed to init server_event: {err}"))?;
        client_event
            .set(EventState::Clear)
            .map_err(|err| eyre!("failed to init client_event: {err}"))?;
        unsafe {
            memory
                .as_ptr()
                .wrapping_add(buffer_start_offset)
                .cast::<AtomicU64>()
                .write(AtomicU64::new(0));
        }

        Ok(Self {
            memory,
            server_event,
            client_event,
            buffer_start_offset,
            server: true,
        })
    }

    pub unsafe fn new_client(memory: Shmem) -> eyre::Result<Self> {
        let (server_event, offset) = unsafe { Event::from_existing(memory.as_ptr()) }
            .map_err(|err| eyre!("failed to open raw server event: {err}"))?;
        let (client_event, buffer_start_offset) =
            unsafe { Event::from_existing(memory.as_ptr().wrapping_add(offset)) }
                .map_err(|err| eyre!("failed to open raw client event: {err}"))?;

        Ok(Self {
            memory,
            server_event,
            client_event,
            buffer_start_offset,
            server: false,
        })
    }

    pub fn send<T>(&mut self, value: &T) -> eyre::Result<()>
    where
        T: Serialize + std::fmt::Debug,
    {
        let msg = bincode::serialize(value).wrap_err("failed to serialize value")?;
        tracing::debug!("sending message with length {}: {value:?}", msg.len());

        let total_len = LEN_LEN + msg.len();
        assert!(total_len <= self.memory.len() - self.buffer_start_offset);

        // write data first
        unsafe {
            self.data_mut()
                .copy_from_nonoverlapping(msg.as_ptr(), msg.len());
        }

        // write len second for synchronization
        unsafe {
            (*self.data_len()).store(msg.len() as u64, std::sync::atomic::Ordering::Release);
        }

        // signal event
        let event = if self.server {
            &self.client_event
        } else {
            &self.server_event
        };
        event
            .set(EventState::Signaled)
            .map_err(|err| eyre!("failed to send message over ShmemChannel: {err}"))?;

        Ok(())
    }

    pub fn receive<T>(&mut self) -> eyre::Result<T>
    where
        T: for<'a> Deserialize<'a> + std::fmt::Debug,
    {
        // wait for event
        let (event, timeout) = if self.server {
            (&self.server_event, raw_sync::Timeout::Infinite)
        } else {
            (
                &self.client_event,
                raw_sync::Timeout::Val(Duration::from_secs(5)),
            )
        };

        event
            .wait(timeout)
            .map_err(|err| eyre!("failed to wait for reply from ShmemChannel: {err}"))?;

        // read len first for synchronization
        let msg_len =
            unsafe { &*self.data_len() }.load(std::sync::atomic::Ordering::Acquire) as usize;
        assert!(msg_len < self.memory.len() - self.buffer_start_offset - LEN_LEN);

        let value_raw = unsafe { slice::from_raw_parts(self.data(), msg_len) };
        let msg = bincode::deserialize(value_raw).wrap_err("failed to deserialize value");
        tracing::debug!("received message with length {msg_len}: {msg:?}");
        msg
    }

    fn data_len(&self) -> *const AtomicU64 {
        self.data_len_ptr().cast()
    }

    fn data_len_ptr(&self) -> *mut u8 {
        self.memory.as_ptr().wrapping_add(self.buffer_start_offset)
    }

    fn data(&self) -> *const u8 {
        self.data_len_ptr().wrapping_add(LEN_LEN)
    }

    fn data_mut(&mut self) -> *mut u8 {
        self.data_len_ptr().wrapping_add(LEN_LEN)
    }
}

unsafe impl Send for ShmemChannel {}
