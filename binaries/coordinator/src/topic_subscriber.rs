use std::collections::BTreeMap;

pub struct TopicFrame {
    pub subscription_id: uuid::Uuid,
    pub payload: std::sync::Arc<[u8]>,
}

pub(crate) struct TopicSubscriber {
    outputs_by_daemon: BTreeMap<
        dora_message::common::DaemonId,
        Vec<(dora_message::id::NodeId, dora_message::id::DataId)>,
    >,
    sender: Option<tokio::sync::mpsc::Sender<TopicFrame>>,
    consecutive_timeouts: usize,
}

impl TopicSubscriber {
    pub(crate) fn new(
        outputs_by_daemon: BTreeMap<
            dora_message::common::DaemonId,
            Vec<(dora_message::id::NodeId, dora_message::id::DataId)>,
        >,
        sender: tokio::sync::mpsc::Sender<TopicFrame>,
    ) -> Self {
        Self {
            outputs_by_daemon,
            sender: Some(sender),
            consecutive_timeouts: 0,
        }
    }

    pub(crate) fn outputs_by_daemon(
        &self,
    ) -> &BTreeMap<
        dora_message::common::DaemonId,
        Vec<(dora_message::id::NodeId, dora_message::id::DataId)>,
    > {
        &self.outputs_by_daemon
    }

    pub(crate) async fn send_frame(&mut self, frame: TopicFrame) -> eyre::Result<()> {
        let sender = self
            .sender
            .as_ref()
            .ok_or_else(|| eyre::eyre!("subscriber is closed"))?;
        sender
            .send(frame)
            .await
            .map_err(|_| eyre::eyre!("WS topic subscriber channel closed"))?;
        Ok(())
    }

    pub(crate) fn reset_timeout_streak(&mut self) {
        self.consecutive_timeouts = 0;
    }

    pub(crate) fn record_timeout(&mut self) -> usize {
        self.consecutive_timeouts += 1;
        self.consecutive_timeouts
    }

    pub(crate) fn is_closed(&self) -> bool {
        match &self.sender {
            None => true,
            Some(sender) => sender.is_closed(),
        }
    }

    pub(crate) fn close(&mut self) {
        self.sender = None;
    }
}
