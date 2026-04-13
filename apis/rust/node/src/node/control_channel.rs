use std::sync::Arc;

use crate::{DaemonCommunicationWrapper, daemon_connection::DaemonChannel};
use dora_core::{
    config::{DataId, NodeId},
    uhlc::HLC,
};
use dora_message::{
    DataflowId, daemon_to_node::DaemonCommunication, metadata::Metadata,
    node_to_daemon::DataMessage,
};
use eyre::Context;

pub(crate) struct ControlChannel {
    channel: DaemonChannel,
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
                        DaemonChannel::new_tcp(*socket_addr, clock.clone())
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

        Ok(Self { channel })
    }

    pub fn report_outputs_done(&mut self) -> eyre::Result<()> {
        self.channel
            .outputs_done()
            .wrap_err("failed to report outputs done to dora-daemon")
    }

    pub fn report_closed_outputs(&mut self, outputs: Vec<DataId>) -> eyre::Result<()> {
        self.channel
            .close_outputs(outputs)
            .wrap_err("failed to report closed outputs to dora-daemon")
    }

    pub fn send_message(
        &mut self,
        output_id: DataId,
        metadata: Metadata,
        data: Option<DataMessage>,
    ) -> eyre::Result<()> {
        self.channel
            .send_message(output_id, metadata, data)
            .wrap_err("failed to send SendMessage request to dora-daemon")
    }
}
