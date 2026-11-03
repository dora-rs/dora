use std::collections::BTreeMap;

use tokio::{
    sync::mpsc::error::TrySendError,
    time::{Duration, Instant},
};

pub struct TopicFrame {
    pub subscription_id: uuid::Uuid,
    pub payload: std::sync::Arc<Vec<u8>>,
}

/// How long a subscriber's channel may refuse every frame offered to it before
/// [`crate::handlers::send_topic_frames`] closes the subscriber: a CLI that has
/// stopped reading. About the bound the former 100 consecutive 100 ms send
/// timeouts gave, without the event loop spending that time waiting.
pub(crate) const MAX_TOPIC_SUBSCRIBER_STALL: Duration = Duration::from_secs(10);

/// Shortest gap between warnings about one subscriber's dropped frames.
const DROP_WARNING_INTERVAL: Duration = Duration::from_secs(5);

/// What became of a frame offered to a subscriber.
#[derive(Debug, PartialEq, Eq)]
pub(crate) enum FrameSent {
    /// Queued for the CLI connection.
    Queued,
    /// Dropped because the subscriber's channel was full.
    Dropped,
}

pub(crate) struct TopicSubscriber {
    outputs_by_daemon: BTreeMap<
        dora_message::common::DaemonId,
        Vec<(dora_message::id::NodeId, dora_message::id::DataId)>,
    >,
    sender: Option<tokio::sync::mpsc::Sender<TopicFrame>>,
    /// Since when the channel has refused every frame offered to it, if it
    /// does now.
    stalled_since: Option<Instant>,
    /// Frames dropped since the last warning about them.
    dropped: u64,
    last_drop_warning: Option<Instant>,
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
            stalled_since: None,
            dropped: 0,
            last_drop_warning: None,
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

    /// Offer `frame` to the subscriber without waiting, as of `now`.
    ///
    /// This runs on the coordinator's event loop, so waiting for room would
    /// hold up the whole coordinator behind one slow CLI. A full channel drops
    /// the frame instead, with a rate-limited warning; topic debug frames are
    /// droppable.
    pub(crate) fn send_frame(
        &mut self,
        frame: TopicFrame,
        now: Instant,
    ) -> eyre::Result<FrameSent> {
        let sender = self
            .sender
            .as_ref()
            .ok_or_else(|| eyre::eyre!("subscriber is closed"))?;
        let subscription_id = frame.subscription_id;
        match sender.try_send(frame) {
            Ok(()) => {
                self.stalled_since = None;
                Ok(FrameSent::Queued)
            }
            Err(TrySendError::Full(_)) => {
                self.stalled_since.get_or_insert(now);
                self.dropped += 1;
                if self
                    .last_drop_warning
                    .is_none_or(|last| now - last >= DROP_WARNING_INTERVAL)
                {
                    tracing::warn!(
                        %subscription_id,
                        "dropped {} topic debug frame(s): the CLI subscriber is not keeping up",
                        self.dropped
                    );
                    self.dropped = 0;
                    self.last_drop_warning = Some(now);
                }
                Ok(FrameSent::Dropped)
            }
            Err(TrySendError::Closed(_)) => Err(eyre::eyre!("WS topic subscriber channel closed")),
        }
    }

    /// How long, as of `now`, the channel has refused every frame offered to
    /// it, if it does.
    pub(crate) fn stalled_for(&self, now: Instant) -> Option<Duration> {
        self.stalled_since.map(|since| now.duration_since(since))
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

    fn frame() -> TopicFrame {
        TopicFrame {
            subscription_id: uuid::Uuid::new_v4(),
            payload: std::sync::Arc::new(Vec::new()),
        }
    }

    #[tokio::test]
    async fn send_frame_fails_after_close() {
        let (mut sub, _rx) = new_subscriber(4);
        sub.close();
        assert!(sub.send_frame(frame(), Instant::now()).is_err());
    }

    /// A full channel drops the frame rather than waiting, and the stall is
    /// timed from the first refused frame until one is queued again.
    #[tokio::test(start_paused = true)]
    async fn a_full_channel_drops_and_times_the_stall() {
        let (mut sub, mut rx) = new_subscriber(1);
        let start = Instant::now();
        assert_eq!(sub.send_frame(frame(), start).unwrap(), FrameSent::Queued);
        assert_eq!(sub.stalled_for(start), None);

        assert_eq!(sub.send_frame(frame(), start).unwrap(), FrameSent::Dropped);
        let later = start + Duration::from_secs(3);
        assert_eq!(sub.send_frame(frame(), later).unwrap(), FrameSent::Dropped);
        assert_eq!(sub.stalled_for(later), Some(Duration::from_secs(3)));

        rx.try_recv().expect("the queued frame");
        assert_eq!(sub.send_frame(frame(), later).unwrap(), FrameSent::Queued);
        assert_eq!(
            sub.stalled_for(later),
            None,
            "a queued frame ends the stall"
        );
    }
}
