use crate::timeout_streak::TimeoutStreak;

/// The send-channel + timeout-streak lifecycle shared by the coordinator's
/// bounded-channel WS subscribers.
///
/// Both `LogSubscriber` and `TopicSubscriber` wrap a bounded mpsc sender that
/// is dropped (set to `None`) on close, alongside a [`TimeoutStreak`] for
/// eviction. The close / `is_closed` / timeout bookkeeping is identical for
/// both — only the payload type `T` and the serialization done in their
/// `send_*` methods differ — so it lives here in one definition instead of
/// being copied into each subscriber.
pub(crate) struct SubscriberChannel<T> {
    sender: Option<tokio::sync::mpsc::Sender<T>>,
    timeouts: TimeoutStreak,
}

impl<T> SubscriberChannel<T> {
    pub(crate) fn new(sender: tokio::sync::mpsc::Sender<T>) -> Self {
        Self {
            sender: Some(sender),
            timeouts: TimeoutStreak::default(),
        }
    }

    /// The live sender, or an error if the subscriber has been closed.
    pub(crate) fn sender(&self) -> eyre::Result<&tokio::sync::mpsc::Sender<T>> {
        self.sender
            .as_ref()
            .ok_or_else(|| eyre::eyre!("subscriber is closed"))
    }

    /// Reset the consecutive-timeout streak after a successful send.
    pub(crate) fn reset_timeout_streak(&mut self) {
        self.timeouts.reset();
    }

    /// Record a send timeout and return the new consecutive-timeout count.
    pub(crate) fn record_timeout(&mut self) -> usize {
        self.timeouts.record()
    }

    /// Whether the subscriber has been closed, or its receiver dropped.
    pub(crate) fn is_closed(&self) -> bool {
        match &self.sender {
            None => true,
            Some(sender) => sender.is_closed(),
        }
    }

    /// Close the subscriber, dropping the sender so the receiver sees EOF.
    pub(crate) fn close(&mut self) {
        self.sender = None;
    }
}

#[cfg(test)]
mod tests {
    use super::SubscriberChannel;

    #[test]
    fn timeouts_are_monotonic_and_reset() {
        let (tx, _rx) = tokio::sync::mpsc::channel::<u8>(1);
        let mut channel = SubscriberChannel::new(tx);
        assert_eq!(channel.record_timeout(), 1);
        assert_eq!(channel.record_timeout(), 2);
        channel.reset_timeout_streak();
        assert_eq!(channel.record_timeout(), 1);
    }

    #[test]
    fn close_marks_closed_and_drops_sender() {
        let (tx, _rx) = tokio::sync::mpsc::channel::<u8>(1);
        let mut channel = SubscriberChannel::new(tx);
        assert!(!channel.is_closed());
        assert!(channel.sender().is_ok());
        channel.close();
        assert!(channel.is_closed());
        assert!(channel.sender().is_err());
    }

    #[test]
    fn is_closed_when_receiver_dropped() {
        let (tx, rx) = tokio::sync::mpsc::channel::<u8>(1);
        let channel = SubscriberChannel::new(tx);
        assert!(!channel.is_closed());
        drop(rx);
        assert!(channel.is_closed());
    }
}
