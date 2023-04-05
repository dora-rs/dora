use std::{
    collections::{HashMap, VecDeque},
    ops::{Deref, DerefMut},
    time::Duration,
};

use dora_core::{
    config::{DataId, NodeId, NodeRunConfig},
    daemon_messages::{Data, DropToken, NodeConfig},
    message::{uhlc, Metadata, MetadataParameters},
};
use eyre::{bail, WrapErr};
use shared_memory::{Shmem, ShmemConf};

use crate::{
    daemon_connection::{ControlChannel, DaemonConnection},
    EventStream,
};

#[cfg(feature = "tracing")]
use dora_tracing::set_up_tracing;

const ZERO_COPY_THRESHOLD: usize = 4096;

pub struct DoraNode {
    id: NodeId,
    node_config: NodeRunConfig,
    control_channel: ControlChannel,
    hlc: uhlc::HLC,

    sent_out_shared_memory: HashMap<DropToken, ShmemHandle>,
    finished_drop_tokens: flume::Receiver<DropToken>,
    cache: VecDeque<ShmemHandle>,
}

impl DoraNode {
    pub fn init_from_env() -> eyre::Result<(Self, EventStream)> {
        let node_config: NodeConfig = {
            let raw = std::env::var("DORA_NODE_CONFIG")
                .wrap_err("env variable DORA_NODE_CONFIG must be set")?;
            serde_yaml::from_str(&raw).context("failed to deserialize operator config")?
        };
        #[cfg(feature = "tracing")]
        set_up_tracing(&node_config.node_id.to_string())
            .context("failed to set up tracing subscriber")?;
        Self::init(node_config)
    }

    pub fn init(node_config: NodeConfig) -> eyre::Result<(Self, EventStream)> {
        let NodeConfig {
            dataflow_id,
            node_id,
            run_config,
            daemon_communication,
        } = node_config;

        let DaemonConnection {
            control_channel,
            event_stream,
            finished_drop_tokens,
        } = DaemonConnection::init(dataflow_id, &node_id, &daemon_communication)
            .wrap_err("failed to connect to dora-daemon")?;

        let node = Self {
            id: node_id,
            node_config: run_config,
            control_channel,
            hlc: uhlc::HLC::default(),
            sent_out_shared_memory: HashMap::new(),
            finished_drop_tokens,
            cache: VecDeque::new(),
        };
        Ok((node, event_stream))
    }

    pub fn send_output<F>(
        &mut self,
        output_id: DataId,
        parameters: MetadataParameters,
        data_len: usize,
        data: F,
    ) -> eyre::Result<()>
    where
        F: FnOnce(&mut [u8]),
    {
        self.handle_finished_drop_tokens()?;

        if !self.node_config.outputs.contains(&output_id) {
            eyre::bail!("unknown output");
        }
        let metadata = Metadata::from_parameters(self.hlc.new_timestamp(), parameters.into_owned());

        let (data, shmem) = if data_len >= ZERO_COPY_THRESHOLD {
            // create shared memory region
            let mut shared_memory = self.allocate_shared_memory(data_len)?;

            // fill in the data
            let raw = unsafe { shared_memory.as_slice_mut() };
            data(&mut raw[..data_len]);

            let drop_token = DropToken::generate();
            let data = Data::SharedMemory {
                shared_memory_id: shared_memory.get_os_id().to_owned(),
                len: data_len,
                drop_token,
            };
            (Some(data), Some((shared_memory, drop_token)))
        } else if data_len == 0 {
            data(&mut []);
            (None, None)
        } else {
            let mut buffer = vec![0; data_len];
            data(&mut buffer);
            (Some(Data::Vec(buffer)), None)
        };

        self.control_channel
            .send_message(output_id.clone(), metadata, data)
            .wrap_err_with(|| format!("failed to send output {output_id}"))?;

        if let Some((shared_memory, drop_token)) = shmem {
            self.sent_out_shared_memory
                .insert(drop_token, shared_memory);
        }

        Ok(())
    }

    pub fn close_outputs(&mut self, outputs: Vec<DataId>) -> eyre::Result<()> {
        for output_id in &outputs {
            if !self.node_config.outputs.remove(output_id) {
                eyre::bail!("unknown output {output_id}");
            }
        }

        self.control_channel
            .report_closed_outputs(outputs)
            .wrap_err("failed to report closed outputs to daemon")?;

        Ok(())
    }

    pub fn id(&self) -> &NodeId {
        &self.id
    }

    pub fn node_config(&self) -> &NodeRunConfig {
        &self.node_config
    }

    fn allocate_shared_memory(&mut self, data_len: usize) -> eyre::Result<ShmemHandle> {
        let cache_index = self
            .cache
            .iter()
            .enumerate()
            .rev()
            .filter(|(_, s)| s.len() >= data_len)
            .min_by_key(|(_, s)| s.len())
            .map(|(i, _)| i);
        let memory = match cache_index {
            Some(i) => {
                // we know that this index exists, so we can safely unwrap here
                self.cache.remove(i).unwrap()
            }
            None => ShmemHandle(Box::new(
                ShmemConf::new()
                    .size(data_len)
                    .create()
                    .wrap_err("failed to allocate shared memory")?,
            )),
        };
        assert!(memory.len() >= data_len);

        Ok(memory)
    }

    fn handle_finished_drop_tokens(&mut self) -> eyre::Result<()> {
        loop {
            match self.finished_drop_tokens.try_recv() {
                Ok(token) => match self.sent_out_shared_memory.remove(&token) {
                    Some(region) => self.add_to_cache(region),
                    None => tracing::warn!("received unknown finished drop token `{token:?}`"),
                },
                Err(flume::TryRecvError::Empty) => break,
                Err(flume::TryRecvError::Disconnected) => {
                    bail!("event stream was closed before sending all expected drop tokens")
                }
            }
        }
        Ok(())
    }

    fn add_to_cache(&mut self, memory: ShmemHandle) {
        const MAX_CACHE_SIZE: usize = 20;

        self.cache.push_back(memory);
        while self.cache.len() > MAX_CACHE_SIZE {
            self.cache.pop_front();
        }
    }
}

impl Drop for DoraNode {
    #[tracing::instrument(skip(self), fields(self.id = %self.id), level = "trace")]
    fn drop(&mut self) {
        // close all outputs first to notify subscribers as early as possible
        if let Err(err) = self
            .control_channel
            .report_closed_outputs(
                std::mem::take(&mut self.node_config.outputs)
                    .into_iter()
                    .collect(),
            )
            .context("failed to close outputs on drop")
        {
            tracing::warn!("{err:?}")
        }

        if !self.sent_out_shared_memory.is_empty() {
            tracing::debug!(
                "waiting for {} remaining drop tokens",
                self.sent_out_shared_memory.len()
            );
        }
        while !self.sent_out_shared_memory.is_empty() {
            match self
                .finished_drop_tokens
                .recv_timeout(Duration::from_secs(10))
            {
                Ok(token) => {
                    self.sent_out_shared_memory.remove(&token);
                }
                Err(flume::RecvTimeoutError::Disconnected) => {
                    tracing::warn!(
                        "finished_drop_tokens channel closed while still waiting for drop tokens; \
                        closing {} shared memory regions that might still be used",
                        self.sent_out_shared_memory.len()
                    );
                    break;
                }
                Err(flume::RecvTimeoutError::Timeout) => {
                    tracing::warn!(
                        "timeout while waiting for drop tokens; \
                        closing {} shared memory regions that might still be used",
                        self.sent_out_shared_memory.len()
                    );
                    break;
                }
            }
        }

        // close `finished_drop_tokens` to signal event stream thread that no
        // more drop tokens are expected
        self.finished_drop_tokens = flume::bounded(0).1;

        tracing::debug!("reporting node stop for node `{}`", self.id);
        if let Err(err) = self.control_channel.report_stop() {
            tracing::warn!("{err:?}")
        }
    }
}

struct ShmemHandle(Box<Shmem>);

impl Deref for ShmemHandle {
    type Target = Shmem;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for ShmemHandle {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

unsafe impl Send for ShmemHandle {}
unsafe impl Sync for ShmemHandle {}
