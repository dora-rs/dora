use std::sync::Arc;

use crate::{DaemonCommunicationWrapper, daemon_connection::DaemonChannel};
use dora_core::{
    config::{DataId, NodeId},
    uhlc::HLC,
};
use dora_message::{
    DataflowId,
    daemon_to_node::{DaemonCommunication, DaemonReply, StateWriteResult},
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
                    DaemonCommunication::Shmem {
                        daemon_control_region_id,
                        ..
                    } => unsafe { DaemonChannel::new_shmem(daemon_control_region_id) }
                        .wrap_err("failed to create shmem control channel")?,
                    DaemonCommunication::Tcp { socket_addr } => {
                        DaemonChannel::new_tcp(*socket_addr)
                            .wrap_err("failed to connect control channel")?
                    }
                    #[cfg(unix)]
                    DaemonCommunication::UnixDomain { socket_file } => {
                        DaemonChannel::new_unix_socket(socket_file)
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

    pub fn state_get(&mut self, key: String) -> eyre::Result<(Option<Vec<u8>>, u64)> {
        let reply = self
            .channel
            .request(&Timestamped {
                inner: DaemonRequest::StateGet { key },
                timestamp: self.clock.new_timestamp(),
            })
            .wrap_err("failed to send StateGet request to dora-daemon")?;
        match reply {
            DaemonReply::StateGet { value, revision } => Ok((value, revision)),
            DaemonReply::Result(Err(err)) => bail!("state get failed: {err}"),
            other => bail!("unexpected StateGet reply: {other:?}"),
        }
    }

    pub fn state_set(&mut self, key: String, value: Vec<u8>) -> eyre::Result<StateWriteResult> {
        let reply = self
            .channel
            .request(&Timestamped {
                inner: DaemonRequest::StateSet { key, value },
                timestamp: self.clock.new_timestamp(),
            })
            .wrap_err("failed to send StateSet request to dora-daemon")?;
        match reply {
            DaemonReply::StateWrite(result) => Ok(result),
            DaemonReply::Result(Err(err)) => bail!("state set failed: {err}"),
            other => bail!("unexpected StateSet reply: {other:?}"),
        }
    }

    pub fn state_compare_and_set(
        &mut self,
        key: String,
        expected_revision: u64,
        value: Option<Vec<u8>>,
    ) -> eyre::Result<StateWriteResult> {
        let reply = self
            .channel
            .request(&Timestamped {
                inner: DaemonRequest::StateCompareAndSet {
                    key,
                    expected_revision,
                    value,
                },
                timestamp: self.clock.new_timestamp(),
            })
            .wrap_err("failed to send StateCompareAndSet request to dora-daemon")?;
        match reply {
            DaemonReply::StateWrite(result) => Ok(result),
            DaemonReply::Result(Err(err)) => bail!("state compare_and_set failed: {err}"),
            other => bail!("unexpected StateCompareAndSet reply: {other:?}"),
        }
    }
}
