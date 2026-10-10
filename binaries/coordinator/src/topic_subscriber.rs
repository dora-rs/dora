use dora_message::daemon_to_coordinator::MAX_TOPIC_DEBUG_PAYLOAD_BYTES;
use std::{collections::BTreeMap, sync::Arc};
use tokio::sync::{OwnedSemaphorePermit, Semaphore, TryAcquireError, mpsc};

pub struct TopicFrame {
    pub subscription_id: uuid::Uuid,
    pub payload: std::sync::Arc<[u8]>,
    /// This frame's share of its connection's
    /// [`TOPIC_FRAME_BYTES_PER_CONNECTION`], given back when the frame is
    /// dropped — once the connection has written it to the CLI. Taken by
    /// [`TopicSubscriber::send_frame`].
    pub budget: Option<OwnedSemaphorePermit>,
}

/// Capacity, in frames, of one CLI connection's topic frame channel.
pub(crate) const TOPIC_FRAME_CHANNEL_CAPACITY: usize = 64;

/// Payload bytes the topic frames queued for one CLI connection may hold in
/// total, across all of its subscriptions.
///
/// The channel is bounded by its frame count, and a payload can be up to
/// [`MAX_TOPIC_DEBUG_PAYLOAD_BYTES`], so the count alone would let one `dora
/// topic` connection on a slow link pin about 1 GiB (dora-rs/dora#3636
/// review). Two of the largest payloads: one being written, one waiting.
/// Small frames never come near it; they are bounded by the capacity.
///
/// A frame that does not fit is dropped for that subscriber, not waited for:
/// see [`TopicSubscriber::send_frame`].
pub(crate) const TOPIC_FRAME_BYTES_PER_CONNECTION: usize = 2 * MAX_TOPIC_DEBUG_PAYLOAD_BYTES;

// A payload larger than the whole budget could never be admitted.
const _: () = assert!(MAX_TOPIC_DEBUG_PAYLOAD_BYTES <= TOPIC_FRAME_BYTES_PER_CONNECTION);

/// The sending side of one CLI connection's topic frame channel: the channel
/// plus its [`TOPIC_FRAME_BYTES_PER_CONNECTION`] budget, shared by every
/// subscription on that connection.
#[derive(Clone, Debug)]
pub struct TopicFrameSender {
    tx: mpsc::Sender<TopicFrame>,
    budget: Arc<Semaphore>,
}

/// The channel one CLI connection receives its topic frames on.
pub(crate) fn topic_frame_channel() -> (TopicFrameSender, mpsc::Receiver<TopicFrame>) {
    topic_frame_channel_sized(
        TOPIC_FRAME_CHANNEL_CAPACITY,
        TOPIC_FRAME_BYTES_PER_CONNECTION,
    )
}

pub(crate) fn topic_frame_channel_sized(
    capacity: usize,
    bytes: usize,
) -> (TopicFrameSender, mpsc::Receiver<TopicFrame>) {
    let (tx, rx) = mpsc::channel(capacity);
    let budget = Arc::new(Semaphore::new(bytes));
    (TopicFrameSender { tx, budget }, rx)
}

/// For tests that build a subscriber on a bare channel: a budget of its own.
#[cfg(test)]
impl From<mpsc::Sender<TopicFrame>> for TopicFrameSender {
    fn from(tx: mpsc::Sender<TopicFrame>) -> Self {
        Self {
            tx,
            budget: Arc::new(Semaphore::new(TOPIC_FRAME_BYTES_PER_CONNECTION)),
        }
    }
}

/// What [`TopicSubscriber::send_frame`] did with a frame.
#[derive(Debug, PartialEq, Eq)]
pub(crate) enum FrameSent {
    Queued,
    /// The connection's byte budget was spent, so the frame was dropped for
    /// this subscriber.
    Dropped,
}

pub(crate) struct TopicSubscriber {
    outputs_by_daemon: BTreeMap<
        dora_message::common::DaemonId,
        Vec<(dora_message::id::NodeId, dora_message::id::DataId)>,
    >,
    sender: Option<TopicFrameSender>,
    timeouts: crate::timeout_streak::TimeoutStreak,
    dropped: DroppedTopicFrames,
}

impl TopicSubscriber {
    pub(crate) fn new(
        outputs_by_daemon: BTreeMap<
            dora_message::common::DaemonId,
            Vec<(dora_message::id::NodeId, dora_message::id::DataId)>,
        >,
        sender: impl Into<TopicFrameSender>,
    ) -> Self {
        Self {
            outputs_by_daemon,
            sender: Some(sender.into()),
            timeouts: crate::timeout_streak::TimeoutStreak::default(),
            dropped: DroppedTopicFrames::default(),
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

    /// Queue `frame` for the CLI, or drop it if the connection's byte budget
    /// is spent. Only room in the channel is waited for, and callers bound
    /// that wait with a timeout.
    ///
    /// The budget is never waited for. This runs on the coordinator's event
    /// loop, and a CLI reading more slowly than the stream keeps the budget
    /// spent almost all the time. Waiting for it would hold the loop for most
    /// of every frame, while the CLI drains one now and then — often enough to
    /// reset the timeout streak, so it would never be evicted either
    /// (dora-rs/dora#3636 review). Dropping costs only that CLI its frames.
    pub(crate) async fn send_frame(&mut self, mut frame: TopicFrame) -> eyre::Result<FrameSent> {
        let sender = self
            .sender
            .as_ref()
            .ok_or_else(|| eyre::eyre!("subscriber is closed"))?;
        let bytes = u32::try_from(frame.payload.len())
            .map_err(|_| eyre::eyre!("topic frame payload too large"))?;
        match Arc::clone(&sender.budget).try_acquire_many_owned(bytes) {
            Ok(budget) => frame.budget = Some(budget),
            Err(TryAcquireError::NoPermits) => {
                self.dropped.record(frame.subscription_id);
                return Ok(FrameSent::Dropped);
            }
            Err(TryAcquireError::Closed) => eyre::bail!("topic frame budget closed"),
        }
        sender
            .tx
            .send(frame)
            .await
            .map_err(|_| eyre::eyre!("WS topic subscriber channel closed"))?;
        Ok(FrameSent::Queued)
    }

    pub(crate) fn reset_timeout_streak(&mut self) {
        self.timeouts.reset();
    }

    pub(crate) fn record_timeout(&mut self) -> usize {
        self.timeouts.record()
    }

    pub(crate) fn is_closed(&self) -> bool {
        match &self.sender {
            None => true,
            Some(sender) => sender.tx.is_closed(),
        }
    }

    pub(crate) fn close(&mut self) {
        self.sender = None;
    }
}

/// Rate-limited accounting of the frames one subscriber lost to a spent
/// connection budget: a CLI that cannot keep up loses them at the stream's
/// rate, so it gets at most one warning per interval.
#[derive(Default)]
struct DroppedTopicFrames {
    count: u64,
    last_log: Option<std::time::Instant>,
}

impl DroppedTopicFrames {
    fn record(&mut self, subscription_id: uuid::Uuid) {
        self.count += 1;
        let now = std::time::Instant::now();
        if self.last_log.is_none_or(|last| {
            now.duration_since(last) >= crate::ws_daemon::TOPIC_DEBUG_DROP_LOG_INTERVAL
        }) {
            tracing::warn!(
                %subscription_id,
                "dropped {} topic frame(s) for a CLI that is reading too slowly",
                self.count,
            );
            self.count = 0;
            self.last_log = Some(now);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use futures::FutureExt;
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
            budget: None,
        };
        assert!(sub.send_frame(frame).await.is_err());
    }

    fn frame(payload_len: usize) -> TopicFrame {
        TopicFrame {
            subscription_id: uuid::Uuid::new_v4(),
            payload: std::sync::Arc::from(vec![0; payload_len].into_boxed_slice()),
            budget: None,
        }
    }

    /// A connection's queued frames are bounded in bytes across all of its
    /// subscriptions, not just in count, and a frame's bytes come back only
    /// once the connection has written it — dropped it after sending
    /// (dora-rs/dora#3636 review). A frame that does not fit is dropped at
    /// once rather than waited for, since the wait would be on the event loop.
    #[tokio::test]
    async fn a_frame_past_the_connection_byte_budget_is_dropped_without_waiting() {
        let (sender, mut rx) = topic_frame_channel_sized(8, 100);
        let mut first = TopicSubscriber::new(BTreeMap::new(), sender.clone());
        let mut second = TopicSubscriber::new(BTreeMap::new(), sender);

        assert_eq!(
            first.send_frame(frame(60)).now_or_never().unwrap().unwrap(),
            FrameSent::Queued
        );
        assert_eq!(
            second
                .send_frame(frame(60))
                .now_or_never()
                .expect("a spent budget must not be waited for")
                .unwrap(),
            FrameSent::Dropped,
            "the connection's budget is spent, whichever subscription sends next"
        );
        assert!(
            !second.is_closed(),
            "dropping a frame keeps the subscription"
        );

        // Taking the frame off the channel is not enough: it holds its bytes
        // until it has been written and dropped.
        let written = rx.try_recv().expect("the first frame");
        assert_eq!(
            second
                .send_frame(frame(60))
                .now_or_never()
                .unwrap()
                .unwrap(),
            FrameSent::Dropped
        );
        assert!(rx.try_recv().is_err(), "dropped frames are not queued");

        drop(written);
        assert_eq!(
            second
                .send_frame(frame(60))
                .now_or_never()
                .unwrap()
                .unwrap(),
            FrameSent::Queued,
            "the budget frees up once the frame has been written"
        );
        assert!(rx.try_recv().is_ok());
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
