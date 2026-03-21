use eyre::{Context, eyre};
use raw_sync_2::events::{Event, EventImpl, EventInit, EventState};
use serde::{Deserialize, Serialize};
use shared_memory_extended::Shmem;
use std::{
    mem::{self, align_of},
    slice,
    sync::atomic::{AtomicBool, AtomicU64},
    time::Duration,
};

pub struct ShmemChannel {
    memory: Shmem,
    server_event: Box<dyn EventImpl>,
    client_event: Box<dyn EventImpl>,
    disconnect_offset: usize,
    len_offset: usize,
    data_offset: usize,
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
        let (disconnect_offset, len_offset, data_offset) =
            offsets(memory.as_ptr(), server_event_len, client_event_len, memory.len())?;

        server_event
            .set(EventState::Clear)
            .map_err(|err| eyre!("failed to init server_event: {err}"))?;
        client_event
            .set(EventState::Clear)
            .map_err(|err| eyre!("failed to init client_event: {err}"))?;
        unsafe {
            memory
                .as_ptr()
                .wrapping_add(disconnect_offset)
                .cast::<AtomicBool>()
                .write(AtomicBool::new(false));
        }
        unsafe {
            memory
                .as_ptr()
                .wrapping_add(len_offset)
                .cast::<AtomicU64>()
                .write(AtomicU64::new(0));
        }

        Ok(Self {
            memory,
            server_event,
            client_event,
            disconnect_offset,
            len_offset,
            data_offset,
            server: true,
        })
    }

    pub unsafe fn new_client(memory: Shmem) -> eyre::Result<Self> {
        let (server_event, server_event_len) = unsafe { Event::from_existing(memory.as_ptr()) }
            .map_err(|err| eyre!("failed to open raw server event: {err}"))?;
        let (client_event, client_event_len) =
            unsafe { Event::from_existing(memory.as_ptr().wrapping_add(server_event_len)) }
                .map_err(|err| eyre!("failed to open raw client event: {err}"))?;
        let (disconnect_offset, len_offset, data_offset) =
            offsets(memory.as_ptr(), server_event_len, client_event_len, memory.len())?;

        Ok(Self {
            memory,
            server_event,
            client_event,
            disconnect_offset,
            len_offset,
            data_offset,
            server: false,
        })
    }

    pub fn send<T>(&mut self, value: &T) -> eyre::Result<()>
    where
        T: Serialize + std::fmt::Debug,
    {
        let msg = bincode::serialize(value).wrap_err("failed to serialize value")?;

        self.send_raw(&msg)
    }

    fn send_raw(&mut self, msg: &[u8]) -> Result<(), eyre::ErrReport> {
        assert!(msg.len() <= self.memory.len() - self.data_offset);
        // write data first
        unsafe {
            self.data_mut()
                .copy_from_nonoverlapping(msg.as_ptr(), msg.len());
        }
        // write len second for synchronization
        self.data_len()
            .store(msg.len() as u64, std::sync::atomic::Ordering::Release);

        // signal event
        let event = if self.server {
            &self.client_event
        } else {
            &self.server_event
        };
        event
            .set(EventState::Signaled)
            .map_err(|err| eyre!("failed to send message over ShmemChannel: {err}"))?;

        let disconnected = self.disconnect().load(std::sync::atomic::Ordering::Acquire);
        if disconnected {
            eyre::bail!("server closed the connection");
        }

        Ok(())
    }

    pub fn receive<T>(&mut self, timeout: Option<Duration>) -> eyre::Result<Option<T>>
    where
        T: for<'a> Deserialize<'a> + std::fmt::Debug,
    {
        // wait for event
        let event = if self.server {
            &self.server_event
        } else {
            &self.client_event
        };
        let timeout = timeout
            .map(raw_sync_2::Timeout::Val)
            .unwrap_or(raw_sync_2::Timeout::Infinite);
        event
            .wait(timeout)
            .map_err(|err| eyre!("failed to receive from ShmemChannel: {err}"))?;

        // check for disconnect first
        if self.disconnect().load(std::sync::atomic::Ordering::Acquire) {
            if self.server {
                tracing::trace!("shm client disconnected");
            } else {
                tracing::error!("shm server disconnected");
            }
            return Ok(None);
        }

        // then read len for synchronization
        let msg_len = self.data_len().load(std::sync::atomic::Ordering::Acquire) as usize;
        assert_ne!(msg_len, 0);
        assert!(msg_len < self.memory.len() - self.data_offset);

        // finally read the data
        let value_raw = unsafe { slice::from_raw_parts(self.data(), msg_len) };

        bincode::deserialize(value_raw)
            .wrap_err("failed to deserialize value")
            .map(|v| Some(v))
    }

    fn disconnect(&self) -> &AtomicBool {
        unsafe {
            &*self
                .memory
                .as_ptr()
                .wrapping_add(self.disconnect_offset)
                .cast::<AtomicBool>()
        }
    }

    fn data_len(&self) -> &AtomicU64 {
        unsafe {
            &*self
                .memory
                .as_ptr()
                .wrapping_add(self.len_offset)
                .cast::<AtomicU64>()
        }
    }

    fn data(&self) -> *const u8 {
        self.memory.as_ptr().wrapping_add(self.data_offset)
    }

    fn data_mut(&mut self) -> *mut u8 {
        self.memory.as_ptr().wrapping_add(self.data_offset)
    }
}

fn offsets(
    base_ptr: *mut u8,
    server_event_len: usize,
    client_event_len: usize,
    memory_len: usize,
) -> eyre::Result<(usize, usize, usize)> {
    let total_event_len = server_event_len
        .checked_add(client_event_len)
        .ok_or_else(|| eyre!("event length overflow"))?;

    if total_event_len > memory_len {
        eyre::bail!("event sizes ({total_event_len}) exceed shared memory length ({memory_len})");
    }

    let next_free = base_ptr.wrapping_add(total_event_len);
    let (disconnect, len, data) = offset_ptrs(next_free);

    let base = base_ptr as usize;
    let disconnect_offset = disconnect as usize - base;
    let len_offset = len as usize - base;
    let data_offset = data as usize - base;

    if data_offset > memory_len {
        eyre::bail!("shared memory too small for layout (required offset: {data_offset}, available: {memory_len})");
    }

    Ok((disconnect_offset, len_offset, data_offset))
}

fn offset_ptrs(next_free: *mut u8) -> (*mut AtomicBool, *mut AtomicU64, *mut u8) {
    let disconnect_ptr = next_free.wrapping_add(next_free.align_offset(align_of::<AtomicBool>()));
    let len_ptr_unaligned = disconnect_ptr.wrapping_add(mem::size_of::<AtomicBool>());
    let len_ptr =
        len_ptr_unaligned.wrapping_add(len_ptr_unaligned.align_offset(align_of::<AtomicU64>()));
    let data_ptr = len_ptr.wrapping_add(mem::size_of::<AtomicU64>());
    (disconnect_ptr.cast(), len_ptr.cast(), data_ptr)
}

unsafe impl Send for ShmemChannel {}
unsafe impl Sync for ShmemChannel {}

impl Drop for ShmemChannel {
    fn drop(&mut self) {
        if self.server {
            // server must only exit after client is disconnected
            let disconnected = self.disconnect().load(std::sync::atomic::Ordering::Acquire);
            if disconnected {
                tracing::debug!("closing ShmemServer after client disconnect");
            } else {
                tracing::error!("ShmemServer closed before client disconnect");

                self.disconnect()
                    .store(true, std::sync::atomic::Ordering::Release);
            }
        } else {
            tracing::debug!("disconnecting client");

            self.disconnect()
                .store(true, std::sync::atomic::Ordering::Release);

            // wake up server
            if let Err(err) = self.server_event.set(EventState::Signaled) {
                tracing::warn!("failed to signal ShmemChannel disconnect: {err}");
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use shared_memory_extended::ShmemConf;

    #[test]
    fn test_small_memory_server_fails() {
        let shmem = ShmemConf::new()
            .size(10)
            .create()
            .unwrap();
        
        let result = unsafe { ShmemChannel::new_server(shmem) };
        assert!(result.is_err());
        let err = result.err().unwrap();
        assert!(err.to_string().contains("shared memory too small") || err.to_string().contains("event sizes exceed"));
    }
}
