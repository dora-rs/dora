use crate::timeout_streak::TimeoutStreak;
use dora_message::coordinator_to_cli::LogMessage;
use eyre::{Context, ContextCompat};

pub struct LogSubscriber {
    pub level: log::LevelFilter,
    sender: Option<tokio::sync::mpsc::Sender<String>>,
    timeouts: TimeoutStreak,
}

impl LogSubscriber {
    pub fn new(level: log::LevelFilter, sender: tokio::sync::mpsc::Sender<String>) -> Self {
        Self {
            level,
            sender: Some(sender),
            timeouts: TimeoutStreak::default(),
        }
    }

    /// Deliver `message` to this subscriber.
    ///
    /// Returns `Ok(true)` when the message was actually enqueued and
    /// `Ok(false)` when it was dropped by the level filter (not a delivery
    /// attempt). The caller uses this to keep the timeout streak meaningful:
    /// a filtered message must not reset the streak, or a stuck subscriber
    /// that keeps seeing below-filter traffic would never be evicted.
    pub async fn send_message(&mut self, message: &LogMessage) -> eyre::Result<bool> {
        match message.level {
            dora_core::build::LogLevelOrStdout::LogLevel(level) => {
                if level > self.level {
                    return Ok(false);
                }
            }
            dora_core::build::LogLevelOrStdout::Stdout => {}
        }

        let sender = self.sender.as_ref().context("subscriber is closed")?;
        let json = serde_json::to_string(&dora_message::ws_protocol::WsEvent {
            event: "log".to_string(),
            payload: serde_json::to_value(message)?,
        })
        .context("failed to serialize log WsEvent")?;
        sender
            .send(json)
            .await
            .map_err(|_e| eyre::eyre!("WS log subscriber channel closed"))?;
        Ok(true)
    }

    /// Reset the consecutive-timeout streak after a successful send.
    pub fn reset_timeout_streak(&mut self) {
        self.timeouts.reset();
    }

    /// Record a send timeout and return the new consecutive-timeout count.
    pub fn record_timeout(&mut self) -> usize {
        self.timeouts.record()
    }

    pub fn is_closed(&self) -> bool {
        match &self.sender {
            None => true,
            Some(sender) => sender.is_closed(),
        }
    }

    pub fn close(&mut self) {
        self.sender = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn new_subscriber(capacity: usize) -> (LogSubscriber, tokio::sync::mpsc::Receiver<String>) {
        let (tx, rx) = tokio::sync::mpsc::channel(capacity);
        (LogSubscriber::new(log::LevelFilter::Info, tx), rx)
    }

    #[test]
    fn record_timeout_is_monotonic_and_resets() {
        let (mut sub, _rx) = new_subscriber(1);
        assert_eq!(sub.record_timeout(), 1);
        assert_eq!(sub.record_timeout(), 2);
        assert_eq!(sub.record_timeout(), 3);
        sub.reset_timeout_streak();
        assert_eq!(sub.record_timeout(), 1);
    }

    #[test]
    fn close_marks_subscriber_closed() {
        let (mut sub, _rx) = new_subscriber(1);
        assert!(!sub.is_closed());
        sub.close();
        assert!(sub.is_closed());
    }
}
