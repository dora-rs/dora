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

    pub fn report_output_sent(
        &mut self,
        output_id: DataId,
        metadata: Metadata,
    ) -> eyre::Result<()> {
        let request = DaemonRequest::OutputSent {
            output_id,
            metadata,
        };
        let reply = self
            .channel
            .request(&Timestamped {
                inner: request,
                timestamp: self.clock.new_timestamp(),
            })
            .wrap_err("failed to send OutputSent request to dora-daemon")?;
        match reply {
            DaemonReply::Empty => Ok(()),
            other => bail!("unexpected OutputSent reply: {other:?}"),
        }
    }

    pub fn extension_store(
        &mut self,
        namespace: String,
        key: String,
        value: Vec<u8>,
    ) -> eyre::Result<()> {
        let request = DaemonRequest::ExtensionStore {
            namespace,
            key,
            value,
        };
        let reply = self
            .channel
            .request(&Timestamped {
                inner: request,
                timestamp: self.clock.new_timestamp(),
            })
            .wrap_err("failed to send ExtensionStore request to dora-daemon")?;
        match reply {
            DaemonReply::Result(Ok(())) => Ok(()),
            DaemonReply::Result(Err(e)) => bail!("{e}"),
            other => bail!("unexpected ExtensionStore reply: {other:?}"),
        }
    }

    pub fn extension_load(
        &mut self,
        namespace: String,
        key: String,
        remove: bool,
    ) -> eyre::Result<Option<Vec<u8>>> {
        let request = DaemonRequest::ExtensionLoad {
            namespace,
            key,
            remove,
        };
        let reply = self
            .channel
            .request(&Timestamped {
                inner: request,
                timestamp: self.clock.new_timestamp(),
            })
            .wrap_err("failed to send ExtensionLoad request to dora-daemon")?;
        match reply {
            DaemonReply::ExtensionValue { value } => Ok(value),
            DaemonReply::Result(Err(e)) => bail!("{e}"),
            other => bail!("unexpected ExtensionLoad reply: {other:?}"),
        }
    }

    pub fn extension_drop(&mut self, namespace: String, key: String) -> eyre::Result<()> {
        let request = DaemonRequest::ExtensionDrop { namespace, key };
        let reply = self
            .channel
            .request(&Timestamped {
                inner: request,
                timestamp: self.clock.new_timestamp(),
            })
            .wrap_err("failed to send ExtensionDrop request to dora-daemon")?;
        match reply {
            DaemonReply::Result(Ok(())) => Ok(()),
            DaemonReply::Result(Err(e)) => bail!("{e}"),
            other => bail!("unexpected ExtensionDrop reply: {other:?}"),
        }
    }

    /// Read a memory pool's metadata from the daemon. The cross-machine
    /// data plane itself never needs this (mirror segments are derived by
    /// name), but the legacy node-side path keeps it as a fallback for
    /// pools registered with an explicit `name=`. On a daemon that no
    /// longer serves the pool table this returns an error; callers treat
    /// it as a soft miss.
    pub fn read_pinned_memory(
        &mut self,
        shared_memory_id: String,
        _free: bool,
    ) -> eyre::Result<Metadata> {
        let request = DaemonRequest::ReadPinnedMemory { shared_memory_id };
        let reply = self
            .channel
            .request(&Timestamped {
                inner: request,
                timestamp: self.clock.new_timestamp(),
            })
            .wrap_err("failed to send ReadPinnedMemory request to dora-daemon")?;
        match reply {
            DaemonReply::PinnedMemoryMetadata { metadata } => Ok(metadata),
            DaemonReply::Result(Err(e)) => bail!("{e}"),
            other => bail!("unexpected ReadPinnedMemory reply: {other:?}"),
        }
    }

    pub fn write_pinned_memory(
        &mut self,
        shared_memory_id: String,
        tensor_data: Vec<u8>,
        size: usize,
    ) -> eyre::Result<()> {
        let request = DaemonRequest::WritePinnedMemory {
            shared_memory_id,
            tensor_data,
            size,
        };
        let reply = self
            .channel
            .request(&Timestamped {
                inner: request,
                timestamp: self.clock.new_timestamp(),
            })
            .wrap_err("failed to send WritePinnedMemory request to dora-daemon")?;
        match reply {
            DaemonReply::Result(Ok(())) => Ok(()),
            DaemonReply::Result(Err(e)) => bail!("{e}"),
            other => bail!("unexpected WritePinnedMemory reply: {other:?}"),
        }
    }

    /// Register a pool on a remote machine via the daemon (the daemon
    /// resolves the machine through the coordinator and mirrors the
    /// pool there with a synchronous confirmation).
    #[allow(clippy::too_many_arguments)]
    pub fn register_cross_machine_pool(
        &mut self,
        shared_memory_id: String,
        shmem_name: String,
        size: usize,
        dtype: String,
        shape: Vec<i64>,
        device: String,
        machine_id: String,
    ) -> eyre::Result<(Result<(), String>, bool)> {
        let request = DaemonRequest::RegisterCrossMachinePool {
            shared_memory_id,
            shmem_name,
            size,
            dtype,
            shape,
            device,
            machine_id,
        };
        let reply = self
            .channel
            .request(&Timestamped {
                inner: request,
                timestamp: self.clock.new_timestamp(),
            })
            .wrap_err("failed to send RegisterCrossMachinePool request to dora-daemon")?;
        match reply {
            DaemonReply::CrossMachinePoolRegistered { result, direct } => Ok((result, direct)),
            other => bail!("unexpected RegisterCrossMachinePool reply: {other:?}"),
        }
    }

    /// Release a memory pool through the daemon. Cross-machine pools get
    /// their tracking entry removed, the peer is told (targeted `FreePool`
    /// publish), and the local mirror is unlinked; local-only pools are a
    /// plain acknowledgement (the pool's own cleanup is node-side).
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
            DaemonReply::Result(Ok(())) => Ok(()),
            DaemonReply::Result(Err(e)) => bail!("{e}"),
            other => bail!("unexpected FreePinnedMemory reply: {other:?}"),
        }
    }
}
