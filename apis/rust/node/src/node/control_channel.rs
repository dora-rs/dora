use crate::daemon_connection::DaemonChannel;
use dora_core::{
    config::{DataId, NodeId},
    daemon_messages::{DaemonCommunication, DaemonRequest, Data, DataflowId},
    message::Metadata,
};
use eyre::{bail, eyre, Context};

pub(crate) struct ControlChannel {
    channel: DaemonChannel,
}

impl ControlChannel {
    #[tracing::instrument(level = "trace")]
    pub(crate) fn init(
        dataflow_id: DataflowId,
        node_id: &NodeId,
        daemon_communication: &DaemonCommunication,
    ) -> eyre::Result<Self> {
        let channel = match daemon_communication {
            DaemonCommunication::Shmem {
                daemon_control_region_id,
                ..
            } => unsafe { DaemonChannel::new_shmem(daemon_control_region_id) }
                .wrap_err("failed to create shmem control channel")?,
            DaemonCommunication::Tcp { socket_addr } => DaemonChannel::new_tcp(*socket_addr)
                .wrap_err("failed to connect control channel")?,
        };

        Self::init_on_channel(dataflow_id, node_id, channel)
    }

    #[tracing::instrument(skip(channel), level = "trace")]
    pub fn init_on_channel(
        dataflow_id: DataflowId,
        node_id: &NodeId,
        mut channel: DaemonChannel,
    ) -> eyre::Result<Self> {
        channel.register(dataflow_id, node_id.clone())?;

        Ok(Self { channel })
    }

    pub fn report_outputs_done(&mut self) -> eyre::Result<()> {
        let reply = self
            .channel
            .request(&DaemonRequest::OutputsDone)
            .wrap_err("failed to report outputs done to dora-daemon")?;
        match reply {
            dora_core::daemon_messages::DaemonReply::Result(result) => result
                .map_err(|e| eyre!(e))
                .wrap_err("failed to report outputs done event to dora-daemon")?,
            other => bail!("unexpected outputs done reply: {other:?}"),
        }
        Ok(())
    }

    pub fn report_closed_outputs(&mut self, outputs: Vec<DataId>) -> eyre::Result<()> {
        let reply = self
            .channel
            .request(&DaemonRequest::CloseOutputs(outputs))
            .wrap_err("failed to report closed outputs to dora-daemon")?;
        match reply {
            dora_core::daemon_messages::DaemonReply::Result(result) => result
                .map_err(|e| eyre!(e))
                .wrap_err("failed to receive closed outputs reply from dora-daemon")?,
            other => bail!("unexpected closed outputs reply: {other:?}"),
        }
        Ok(())
    }

    pub fn send_message(
        &mut self,
        output_id: DataId,
        metadata: Metadata<'static>,
        data: Option<Data>,
    ) -> eyre::Result<()> {
        let request = DaemonRequest::SendMessage {
            output_id,
            metadata,
            data,
        };
        let reply = self
            .channel
            .request(&request)
            .wrap_err("failed to send SendMessage request to dora-daemon")?;
        match reply {
            dora_core::daemon_messages::DaemonReply::Empty => Ok(()),
            other => bail!("unexpected SendMessage reply: {other:?}"),
        }
    }
}
