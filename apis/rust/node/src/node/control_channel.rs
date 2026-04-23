use std::sync::Arc;

use crate::{DaemonCommunicationWrapper, daemon_connection::DaemonChannel};
use dora_core::{
    config::{DataId, NodeId},
    uhlc::HLC,
};
use dora_message::{
    DataflowId,
    daemon_to_node::{DaemonCommunication, DaemonReply},
    metadata::Metadata,
    node_to_daemon::{DaemonRequest, DataMessage, Timestamped},
};
use eyre::{Context, bail, eyre};

pub(crate) struct ControlChannel {
    channel: DaemonChannel,
    clock: Arc<HLC>,
}

impl ControlChannel {
    #[tracing::instrument(level = "trace", skip(clock))]
    pub(crate) fn init(
        dataflow_id: DataflowId,
        node_id: &NodeId,
        daemon_communication: &DaemonCommunicationWrapper,
        clock: Arc<HLC>,
    ) -> eyre::Result<Self> {
        let channel = match daemon_communication {
            DaemonCommunicationWrapper::Standard(daemon_communication) => {
                match daemon_communication {
                    DaemonCommunication::Tcp { socket_addr } => {
                        DaemonChannel::new_tcp(*socket_addr)
                            .wrap_err("failed to connect control channel")?
                    }
                    DaemonCommunication::Interactive => {
                        DaemonChannel::Interactive(Default::default())
                    }
                }
            }
            DaemonCommunicationWrapper::Testing { channel } => {
                DaemonChannel::IntegrationTestChannel(channel.clone())
            }
        };

        Self::init_on_channel(dataflow_id, node_id, channel, clock)
    }

    #[tracing::instrument(skip(channel, clock), level = "trace")]
    pub fn init_on_channel(
        dataflow_id: DataflowId,
        node_id: &NodeId,
        mut channel: DaemonChannel,
        clock: Arc<HLC>,
    ) -> eyre::Result<Self> {
        channel.register(dataflow_id, node_id.clone(), clock.new_timestamp())?;

        Ok(Self { channel, clock })
    }

    pub fn report_outputs_done(&mut self) -> eyre::Result<()> {
        let reply = self
            .channel
            .request(&Timestamped {
                inner: DaemonRequest::OutputsDone,
                timestamp: self.clock.new_timestamp(),
            })
            .wrap_err("failed to report outputs done to dora-daemon")?;
        match reply {
            DaemonReply::Result(result) => result
                .map_err(|e| eyre!(e))
                .wrap_err("failed to report outputs done event to dora-daemon")?,
            other => bail!("unexpected outputs done reply: {other:?}"),
        }
        Ok(())
    }

    pub fn report_closed_outputs(&mut self, outputs: Vec<DataId>) -> eyre::Result<()> {
        let reply = self
            .channel
            .request(&Timestamped {
                inner: DaemonRequest::CloseOutputs(outputs),
                timestamp: self.clock.new_timestamp(),
            })
            .wrap_err("failed to report closed outputs to dora-daemon")?;
        match reply {
            DaemonReply::Result(result) => result
                .map_err(|e| eyre!(e))
                .wrap_err("failed to receive closed outputs reply from dora-daemon")?,
            other => bail!("unexpected closed outputs reply: {other:?}"),
        }
        Ok(())
    }

    pub fn send_message(
        &mut self,
        output_id: DataId,
        metadata: Metadata,
        data: Option<DataMessage>,
    ) -> eyre::Result<()> {
        let request = DaemonRequest::SendMessage {
            output_id,
            metadata,
            data,
        };
        let reply = self
            .channel
            .request(&Timestamped {
                inner: request,
                timestamp: self.clock.new_timestamp(),
            })
            .wrap_err("failed to send SendMessage request to dora-daemon")?;
        match reply {
            DaemonReply::Empty => Ok(()),
            other => bail!("unexpected SendMessage reply: {other:?}"),
        }
    }

    pub fn register_pinned_memory(
        &mut self,
        shared_memory_id: String,
        metadata: Metadata,
    ) -> eyre::Result<()> {
        let request = DaemonRequest::RegisterPinnedMemory {
            shared_memory_id,
            metadata,
        };
        let reply = self
            .channel
            .request(&Timestamped {
                inner: request,
                timestamp: self.clock.new_timestamp(),
            })
            .wrap_err("failed to send RegisterPinnedMemory request to dora-daemon")?;
        match reply {
            DaemonReply::Result(Ok(())) | DaemonReply::Empty => Ok(()),
            DaemonReply::Result(Err(e)) => bail!("daemon error: {}", e),
            other => bail!("unexpected RegisterPinnedMemory reply: {other:?}"),
        }
    }

    pub fn read_pinned_memory(
        &mut self,
        shared_memory_id: String,
        free: bool,
    ) -> eyre::Result<Metadata> {
        let request = DaemonRequest::ReadPinnedMemory {
            shared_memory_id,
            free,
        };
        let reply = self
            .channel
            .request(&Timestamped {
                inner: request,
                timestamp: self.clock.new_timestamp(),
            })
            .wrap_err("failed to send ReadPinnedMemory request to dora-daemon")?;
        match reply {
            DaemonReply::PinnedMemoryMetadata { metadata } => Ok(metadata),
            other => bail!("unexpected ReadPinnedMemory reply: {other:?}"),
        }
    }

    pub fn free_pinned_memory(&mut self, shared_memory_id: String) -> eyre::Result<()> {
        let request = DaemonRequest::FreePinnedMemory { shared_memory_id };
        let reply = self
            .channel
            .request(&Timestamped {
                inner: request,
                timestamp: self.clock.new_timestamp(),
            })
            .wrap_err("failed to send FreePinnedMemory request to dora-daemon")?;
        match reply {
            DaemonReply::Result(Ok(())) | DaemonReply::Empty => Ok(()),
            DaemonReply::Result(Err(e)) => bail!("daemon error: {}", e),
            other => bail!("unexpected FreePinnedMemory reply: {other:?}"),
        }
    }
}
