//! Applies `queue_size` to zenoh inputs from the moment a message arrives.
//!
//! A zenoh payload points into its producer's shared memory, so every message
//! waiting in a node that reads more slowly than its producer holds on to some
//! of that memory, and enough of them use it all up. Each input therefore keeps
//! a list of its waiting messages, in the ingress channel and then in the
//! scheduler's queue. Once the list is longer than `queue_size`, the oldest
//! message's payload is released. The message itself stays where it is, so the
//! order of everything else doesn't change, and the node skips it when it gets
//! to it ([`EventItem::settle`](super::thread::EventItem::settle)). A payload
//! is counted as a drop by whichever side releases it: the list here or the
//! scheduler's eviction.

use std::{
    collections::VecDeque,
    sync::{
        Arc, Mutex, MutexGuard, Weak,
        atomic::{AtomicU64, Ordering},
    },
};

use arrow::array::ArrayData;
use dora_core::config::DataId;
use dora_message::{
    config::{Input, QueuePolicy},
    metadata::{Metadata, carries_pattern_correlation},
};

use super::should_warn_drop;

type Slot = Mutex<Option<ArrayData>>;

fn lock<T>(mutex: &Mutex<T>) -> MutexGuard<'_, T> {
    mutex
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner())
}

/// The payload of a zenoh-delivered input.
#[derive(Debug)]
pub(crate) enum ZenohPayload {
    /// Waiting to be delivered. Its input's [`ArrivalBound`] may release it.
    Waiting(Arc<Slot>),
    /// Taken for delivery, or never under a bound. Nothing can release it.
    Settled(ArrayData),
}

impl ZenohPayload {
    /// Takes the payload for delivery, so the bound can no longer release it.
    /// Returns `None` if the bound already did.
    pub(crate) fn settle(self) -> Option<Self> {
        match self {
            Self::Waiting(slot) => lock(&slot).take().map(Self::Settled),
            settled => Some(settled),
        }
    }

    /// The data, or `None` if the bound released it.
    pub(crate) fn data(&self) -> Option<ArrayData> {
        match self {
            Self::Waiting(slot) => lock(slot).clone(),
            Self::Settled(data) => Some(data.clone()),
        }
    }

    /// The data of a [settled](Self::settle) payload.
    pub(crate) fn into_settled_data(self) -> ArrayData {
        match self {
            Self::Settled(data) => data,
            Self::Waiting(_) => panic!("a zenoh payload is settled before its data is read"),
        }
    }
}

/// Lets an [`ArrivalBound`] release a payload after its message is sent.
pub(crate) struct Waiting(Weak<Slot>);

/// Keeps at most `queue_size` payloads of one `drop_oldest` input waiting in
/// the node.
pub(crate) struct ArrivalBound {
    input_id: DataId,
    cap: usize,
    /// Payloads of this input's waiting messages, oldest first.
    waiting: Mutex<VecDeque<Weak<Slot>>>,
    /// The input's ingress drop count, which `drain_drop_counts` reports.
    dropped: Arc<AtomicU64>,
}

impl ArrivalBound {
    /// The bound for `input`, or `None` for a `backpressure` input, which never
    /// drops.
    pub(crate) fn for_input(
        input_id: DataId,
        input: &Input,
        dropped: Arc<AtomicU64>,
    ) -> Option<Self> {
        let policy = input.queue_policy.unwrap_or_default();
        (policy == QueuePolicy::DropOldest).then(|| Self {
            input_id,
            cap: policy.effective_cap(
                input
                    .queue_size
                    .unwrap_or(dora_message::config::DEFAULT_QUEUE_SIZE),
            ),
            waiting: Mutex::default(),
            dropped,
        })
    }

    /// Wraps `data` to send. Service and action messages are never released;
    /// the scheduler's eviction spares them too.
    pub(crate) fn wrap(
        &self,
        data: ArrayData,
        metadata: &Metadata,
    ) -> (ZenohPayload, Option<Waiting>) {
        if carries_pattern_correlation(&metadata.parameters) {
            return (ZenohPayload::Settled(data), None);
        }
        let slot = Arc::new(Mutex::new(Some(data)));
        let waiting = Waiting(Arc::downgrade(&slot));
        (ZenohPayload::Waiting(slot), Some(waiting))
    }

    /// Adds a message that was just sent to the list and, if that makes the
    /// list longer than `queue_size`, releases the oldest payload. It runs after
    /// the send so that a message the full channel turned away doesn't push
    /// out one that is already waiting.
    pub(crate) fn admit(&self, sent: Waiting) {
        let released = {
            let mut waiting = lock(&self.waiting);
            // Forget messages that have since been delivered or discarded.
            waiting.retain(|slot| slot.strong_count() > 0);
            waiting.push_back(sent.0);
            // The list was within the cap before this message, so at most one
            // has to go.
            waiting.len() > self.cap
                && waiting
                    .pop_front()
                    .and_then(|slot| slot.upgrade())
                    .is_some_and(|slot| lock(&slot).take().is_some())
        };
        if released {
            let drops = self.dropped.fetch_add(1, Ordering::Relaxed) + 1;
            if should_warn_drop(drops) {
                tracing::warn!(
                    drops,
                    "Discarding event for input `{}` due to queue size limit",
                    self.input_id
                );
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use dora_message::metadata::{MetadataParameters, Parameter};

    fn input(queue_size: usize) -> Input {
        serde_yaml::from_str(&format!("source: a/out\nqueue_size: {queue_size}\n")).unwrap()
    }

    fn metadata(params: MetadataParameters) -> Metadata {
        Metadata::from_parameters(dora_core::uhlc::HLC::default().new_timestamp(), params)
    }

    fn sample(value: i32) -> ArrayData {
        use arrow::array::Array;
        arrow::array::Int32Array::from(vec![value]).into_data()
    }

    /// Wraps and admits `value`, as delivering it does when the send succeeds.
    fn send(bound: &ArrivalBound, value: i32, params: MetadataParameters) -> ZenohPayload {
        let (payload, waiting) = bound.wrap(sample(value), &metadata(params));
        if let Some(waiting) = waiting {
            bound.admit(waiting);
        }
        payload
    }

    fn value(payload: ZenohPayload) -> Option<i32> {
        payload
            .settle()
            .map(|p| arrow::array::Int32Array::from(p.into_settled_data()).value(0))
    }

    /// The oldest waiting payloads are released first and each is counted
    /// once. A payload already taken for delivery doesn't count toward the cap
    /// and is never released.
    #[test]
    fn releases_the_oldest_waiting_payloads_beyond_queue_size() {
        let dropped = Arc::new(AtomicU64::new(0));
        let bound = ArrivalBound::for_input("cam".into(), &input(2), dropped.clone()).unwrap();

        let settled = send(&bound, 1, MetadataParameters::new()).settle().unwrap();
        let waiting: Vec<_> = (2..=5)
            .map(|v| send(&bound, v, MetadataParameters::new()))
            .collect();

        assert_eq!(value(settled), Some(1));
        assert_eq!(
            waiting.into_iter().map(value).collect::<Vec<_>>(),
            [None, None, Some(4), Some(5)]
        );
        assert_eq!(dropped.load(Ordering::Relaxed), 2);
    }

    /// A service or action message is never released and doesn't count toward
    /// the cap.
    #[test]
    fn keeps_correlated_messages() {
        let dropped = Arc::new(AtomicU64::new(0));
        let bound = ArrivalBound::for_input("srv".into(), &input(1), dropped.clone()).unwrap();
        let mut params = MetadataParameters::new();
        params.insert(
            dora_message::metadata::REQUEST_ID.into(),
            Parameter::String("req-1".into()),
        );

        let response = send(&bound, 1, params);
        let plain: Vec<_> = (2..=3)
            .map(|v| send(&bound, v, MetadataParameters::new()))
            .collect();

        assert_eq!(value(response), Some(1));
        assert_eq!(
            plain.into_iter().map(value).collect::<Vec<_>>(),
            [None, Some(3)]
        );
    }

    #[test]
    fn backpressure_inputs_have_no_bound() {
        let input: Input =
            serde_yaml::from_str("source: a/out\nqueue_size: 1\nqueue_policy: backpressure\n")
                .unwrap();
        assert!(ArrivalBound::for_input("cmd".into(), &input, Arc::default()).is_none());
    }
}
