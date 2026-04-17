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

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeMap;

    fn new_subscriber(
        capacity: usize,
    ) -> (TopicSubscriber, tokio::sync::mpsc::Receiver<TopicFrame>) {
        let (tx, rx) = tokio::sync::mpsc::channel(capacity);
        (TopicSubscriber::new(BTreeMap::new(), tx), rx)
    }

    #[tokio::test]
    async fn close_signals_eof_to_receiver() {
        // Regression test for #236 fix (PR #238): when a dataflow finishes,
        // the coordinator calls close() on each subscriber and the CLI must
        // see EOF on data_rx rather than hanging.
        let (mut sub, mut rx) = new_subscriber(4);
        assert!(!sub.is_closed());
        sub.close();
        assert!(sub.is_closed());
        assert!(rx.recv().await.is_none());
    }

    #[tokio::test]
    async fn send_frame_fails_after_close() {
        let (mut sub, _rx) = new_subscriber(4);
        sub.close();
        let frame = TopicFrame {
            subscription_id: uuid::Uuid::new_v4(),
            payload: std::sync::Arc::from(vec![].into_boxed_slice()),
        };
        assert!(sub.send_frame(frame).await.is_err());
    }

    #[test]
    fn record_timeout_is_monotonic() {
        let (mut sub, _rx) = new_subscriber(1);
        assert_eq!(sub.record_timeout(), 1);
        assert_eq!(sub.record_timeout(), 2);
        assert_eq!(sub.record_timeout(), 3);
        sub.reset_timeout_streak();
        assert_eq!(sub.record_timeout(), 1);
    }
}
