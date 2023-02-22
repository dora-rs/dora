use core::fmt;
use std::{
    collections::{HashMap, HashSet},
    sync::Arc,
};

use dora_core::{
    config::{DataId, NodeId},
    daemon_messages::{DaemonReply, DataflowId, DropEvent, DropToken},
};
use eyre::{eyre, Context};
use flume::{Receiver, Sender};
use futures::StreamExt;
use futures_concurrency::stream::Merge;
use shared_memory_server::{Shmem, ShmemConf};
use tokio::sync::oneshot;
use uuid::Uuid;

use crate::MessageId;

pub struct SharedMemHandler {
    events_tx: Sender<crate::ShmemHandlerEvent>,
    prepared_messages: HashMap<String, PreparedMessage>,
    sent_out_shared_memory: HashMap<DropToken, Arc<ShmemHandle>>,
    dropped: HashSet<DropToken>,
}

impl SharedMemHandler {
    pub fn new(events_tx: Sender<crate::ShmemHandlerEvent>) -> Self {
        Self {
            events_tx,
            prepared_messages: HashMap::new(),
            sent_out_shared_memory: HashMap::new(),
            dropped: HashSet::new(),
        }
    }

    pub async fn run(
        &mut self,
        node_events: Receiver<NodeEvent>,
        daemon_events: Receiver<DaemonEvent>,
    ) {
        if let Err(err) = self.run_inner(node_events, daemon_events).await {
            if let Err(send_err) = self
                .events_tx
                .send_async(crate::ShmemHandlerEvent::HandlerError(err))
                .await
            {
                tracing::error!("{send_err:?}");
            }
        }
    }

    pub async fn run_inner(
        &mut self,
        node_events: Receiver<NodeEvent>,
        daemon_events: Receiver<DaemonEvent>,
    ) -> eyre::Result<()> {
        let mut events = (
            node_events.stream().map(Event::Node),
            daemon_events.stream().map(Event::Daemon),
        )
            .merge();
        while let Some(event) = events.next().await {
            match event {
                Event::Node(event) => self.handle_node_event(event).await?,
                Event::Daemon(event) => self.handle_daemon_event(event).await?,
            }
        }
        Ok(())
    }

    async fn handle_node_event(&mut self, event: NodeEvent) -> eyre::Result<()> {
        match event {
            NodeEvent::Drop(DropEvent { tokens }) => {
                for token in tokens {
                    match self.sent_out_shared_memory.remove(&token) {
                        Some(arc) => {
                            if let Ok(shmem) = Arc::try_unwrap(arc) {
                                tokio::task::spawn_blocking(move || {
                                    tracing::trace!(
                                        "freeing shared memory after receiving last drop token"
                                    );
                                    std::mem::drop(shmem);
                                });
                            }
                        }
                        None => {
                            self.dropped.insert(token);
                        }
                    }
                }
            }
            NodeEvent::PrepareOutputMessage {
                dataflow_id,
                node_id,
                output_id,
                metadata,
                data_len,
                reply_sender,
            } => {
                tracing::trace!(
                    "Time between construct and prepare: {:?}",
                    metadata
                        .timestamp()
                        .get_time()
                        .to_system_time()
                        .elapsed()
                        .unwrap()
                );

                let memory = if data_len > 0 {
                    Some(ShmemHandle(
                        ShmemConf::new()
                            .size(data_len)
                            .create()
                            .wrap_err("failed to allocate shared memory")?,
                    ))
                } else {
                    None
                };
                let id = memory
                    .as_ref()
                    .map(|m| m.0.get_os_id().to_owned())
                    .unwrap_or_else(|| Uuid::new_v4().to_string());
                let message = PreparedMessage {
                    dataflow_id,
                    node_id,
                    output_id,
                    metadata,
                    data: memory.map(|m| (m, data_len)),
                };
                self.prepared_messages.insert(id.clone(), message);

                let reply = DaemonReply::PreparedMessage {
                    shared_memory_id: id.clone(),
                };
                if reply_sender.send(reply).is_err() {
                    // free shared memory slice again
                    self.prepared_messages.remove(&id);
                }
            }
            NodeEvent::SendPreparedMessage { id, reply_sender } => {
                let message = self
                    .prepared_messages
                    .remove(&id)
                    .ok_or_else(|| eyre!("invalid shared memory id"))?;
                let PreparedMessage {
                    dataflow_id,
                    node_id,
                    output_id,
                    metadata,
                    data,
                } = message;
                let data = data.map(|(m, len)| {
                    SharedMemSample {
                        shared_memory: m,
                        len,
                    }
                    .into()
                });

                let send_result = self
                    .events_tx
                    .send_async(crate::ShmemHandlerEvent::SendOut {
                        dataflow_id,
                        node_id,
                        output_id,
                        metadata,
                        data,
                    })
                    .await;
                let _ = reply_sender.send(DaemonReply::Result(
                    send_result.map_err(|_| "daemon is no longer running".into()),
                ));
            }
        }
        Ok(())
    }

    async fn handle_daemon_event(&mut self, event: DaemonEvent) -> eyre::Result<()> {
        match event {
            DaemonEvent::SentOut { data, drop_tokens } => {
                // keep shared memory alive until we received all drop tokens
                let memory = Arc::new(data.shared_memory);
                for drop_token in drop_tokens {
                    if self.dropped.remove(&drop_token) {
                        // this token was already dropped -> ignore
                    } else {
                        self.sent_out_shared_memory
                            .insert(drop_token, memory.clone());
                    }
                }
            }
        }
        Ok(())
    }
}

pub struct SharedMemSample {
    shared_memory: ShmemHandle,
    len: usize,
}

impl SharedMemSample {
    pub fn as_raw_slice(&self) -> *const [u8] {
        std::ptr::slice_from_raw_parts(self.shared_memory.0.as_ptr(), self.len)
    }

    pub unsafe fn as_slice(&self) -> &[u8] {
        unsafe { &*self.as_raw_slice() }
    }

    pub fn get_os_id(&self) -> &str {
        self.shared_memory.0.get_os_id()
    }

    pub fn len(&self) -> usize {
        self.len
    }
}

#[derive(Debug)]
enum Event {
    Node(NodeEvent),
    Daemon(DaemonEvent),
}

#[derive(Debug)]
pub enum NodeEvent {
    PrepareOutputMessage {
        dataflow_id: DataflowId,
        node_id: NodeId,
        output_id: DataId,
        metadata: dora_message::Metadata<'static>,
        data_len: usize,
        reply_sender: oneshot::Sender<DaemonReply>,
    },
    SendPreparedMessage {
        id: MessageId,
        reply_sender: oneshot::Sender<DaemonReply>,
    },
    Drop(DropEvent),
}

pub enum DaemonEvent {
    SentOut {
        data: SharedMemSample,
        drop_tokens: Vec<DropToken>,
    },
}
impl fmt::Debug for DaemonEvent {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::SentOut {
                data: _,
                drop_tokens,
            } => f
                .debug_struct("SentOut")
                .field("data", &"[..]")
                .field("drop_tokens", drop_tokens)
                .finish(),
        }
    }
}

struct PreparedMessage {
    dataflow_id: DataflowId,
    node_id: NodeId,
    output_id: DataId,
    metadata: dora_message::Metadata<'static>,
    data: Option<(ShmemHandle, usize)>,
}

struct ShmemHandle(Shmem);

unsafe impl Send for ShmemHandle {}
unsafe impl Sync for ShmemHandle {}
