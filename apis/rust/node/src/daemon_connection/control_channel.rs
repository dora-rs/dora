use super::{communication::DaemonChannel, EventStreamThreadHandle, MessageSample};
use dora_core::{
    config::{DataId, NodeId},
    daemon_messages::{DaemonRequest, DataflowId},
    message::Metadata,
};
use eyre::{bail, eyre, Context};
use std::sync::Arc;

pub(crate) struct ControlChannel {
    channel: DaemonChannel,
    _event_stream_thread_handle: Option<Arc<EventStreamThreadHandle>>,
}

impl ControlChannel {
    #[tracing::instrument(skip(channel, event_stream_thread_handle))]
    pub(crate) fn init(
        dataflow_id: DataflowId,
        node_id: &NodeId,
        mut channel: DaemonChannel,
        event_stream_thread_handle: Arc<EventStreamThreadHandle>,
    ) -> eyre::Result<Self> {
        channel.register(dataflow_id, node_id.clone())?;

        Ok(Self {
            channel,
            _event_stream_thread_handle: Some(event_stream_thread_handle),
        })
    }

    pub fn report_stop(&mut self) -> eyre::Result<()> {
        let reply = self
            .channel
            .request(&DaemonRequest::Stopped)
            .wrap_err("failed to report stopped to dora-daemon")?;
        match reply {
            dora_core::daemon_messages::DaemonReply::Result(result) => result
                .map_err(|e| eyre!(e))
                .wrap_err("failed to report stop event to dora-daemon")?,
            other => bail!("unexpected stopped reply: {other:?}"),
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

    pub fn prepare_message(
        &mut self,
        output_id: DataId,
        metadata: Metadata<'static>,
        data_len: usize,
    ) -> eyre::Result<MessageSample> {
        let reply = self
            .channel
            .request(&DaemonRequest::PrepareOutputMessage {
                output_id,
                metadata,
                data_len,
            })
            .wrap_err("failed to send PrepareOutputMessage request to dora-daemon")?;
        match reply {
            dora_core::daemon_messages::DaemonReply::PreparedMessage {
                shared_memory_id: id,
            } => Ok(MessageSample { id }),
            dora_core::daemon_messages::DaemonReply::Result(Err(err)) => {
                Err(eyre!(err).wrap_err("failed to report stop event to dora-daemon"))
            }
            other => bail!("unexpected PrepareOutputMessage reply: {other:?}"),
        }
    }

    pub fn send_prepared_message(&mut self, sample: MessageSample) -> eyre::Result<()> {
        let reply = self
            .channel
            .request(&DaemonRequest::SendPreparedMessage { id: sample.id })
            .wrap_err("failed to send SendOutMessage request to dora-daemon")?;
        match reply {
            dora_core::daemon_messages::DaemonReply::Result(result) => {
                result.map_err(|err| eyre!(err))
            }
            other => bail!("unexpected SendOutMessage reply: {other:?}"),
        }
    }

    pub fn send_message(
        &mut self,
        output_id: DataId,
        metadata: Metadata<'static>,
        data: Vec<u8>,
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
