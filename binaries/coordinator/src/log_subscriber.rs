use crate::timeout_streak::TimeoutStreak;
use dora_message::coordinator_to_cli::LogMessage;
use eyre::{Context, ContextCompat};
use tokio::sync::mpsc::error::TrySendError;

/// Outcome of [`LogSubscriber::try_send_message`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Delivery {
    /// Enqueued for the reader.
    Sent,
    /// Below the subscriber's level filter; not a delivery attempt.
    Filtered,
    /// The reader's channel was full, so the line was dropped. `stalled` is
    /// how long the channel has been full without anything going out.
    Full { stalled: std::time::Duration },
}

pub struct LogSubscriber {
    pub level: log::LevelFilter,
    sender: Option<tokio::sync::mpsc::Sender<String>>,
    timeouts: TimeoutStreak,
    /// Lines dropped on a full channel since the last one that went out.
    dropped: u64,
    /// When the channel was first found full since the last successful send.
    full_since: Option<tokio::time::Instant>,
}

fn encode(message: &LogMessage) -> eyre::Result<String> {
    serde_json::to_string(&dora_message::ws_protocol::WsEvent {
        event: "log".to_string(),
        payload: serde_json::to_value(message)?,
    })
    .context("failed to serialize log WsEvent")
}

/// The line a reader gets in place of the `dropped` lines it fell behind on.
fn dropped_lines_notice(next: &LogMessage, dropped: u64) -> LogMessage {
    LogMessage {
        build_id: next.build_id,
        dataflow_id: next.dataflow_id,
        node_id: None,
        daemon_id: None,
        level: dora_message::common::LogLevel::Warn.into(),
        target: Some("coordinator".into()),
        module_path: None,
        file: None,
        line: None,
        message: format!("dropped {dropped} log line(s) because this reader fell behind"),
        timestamp: next.timestamp,
        fields: None,
    }
}

impl LogSubscriber {
    pub fn new(level: log::LevelFilter, sender: tokio::sync::mpsc::Sender<String>) -> Self {
        Self {
            level,
            sender: Some(sender),
            timeouts: TimeoutStreak::default(),
            dropped: 0,
            full_since: None,
        }
    }

    /// Deliver `message` to this subscriber, waiting for channel space.
    ///
    /// Only for the one-time replay of buffered logs when the subscriber
    /// attaches (bounded by the caller's timeout); live logs go through the
    /// non-blocking [`Self::try_send_message`].
    ///
    /// Returns `Ok(true)` when the message was actually enqueued and
    /// `Ok(false)` when it was dropped by the level filter (not a delivery
    /// attempt). The caller uses this to keep the timeout streak meaningful:
    /// a filtered message must not reset the streak, or a stuck subscriber
    /// that keeps seeing below-filter traffic would never be evicted.
    pub async fn send_message(&mut self, message: &LogMessage) -> eyre::Result<bool> {
        if !self.passes_filter(message) {
            return Ok(false);
        }
        let sender = self.sender.as_ref().context("subscriber is closed")?;
        sender
            .send(encode(message)?)
            .await
            .map_err(|_e| eyre::eyre!("WS log subscriber channel closed"))?;
        Ok(true)
    }

    /// Deliver `message` without waiting: the coordinator's event loop calls
    /// this for every live log line, so a slow `dora logs -f` reader must not
    /// pace it (#3794).
    ///
    /// When the channel is full the line is dropped and counted. Once space
    /// frees, the reader first gets one line saying how many were dropped.
    /// `now` dates the start of a stall: anything enqueued ends it.
    pub fn try_send_message(
        &mut self,
        message: &LogMessage,
        now: tokio::time::Instant,
    ) -> eyre::Result<Delivery> {
        if !self.passes_filter(message) {
            return Ok(Delivery::Filtered);
        }
        if self.dropped > 0 {
            let notice = dropped_lines_notice(message, self.dropped);
            if !self.try_enqueue(&notice)? {
                return Ok(self.drop_line(now));
            }
            self.dropped = 0;
        }
        if self.try_enqueue(message)? {
            Ok(Delivery::Sent)
        } else {
            Ok(self.drop_line(now))
        }
    }

    /// Enqueue `message` if the channel has room; `Ok(false)` if it is full.
    fn try_enqueue(&mut self, message: &LogMessage) -> eyre::Result<bool> {
        let sender = self.sender.as_ref().context("subscriber is closed")?;
        match sender.try_send(encode(message)?) {
            Ok(()) => {
                self.full_since = None;
                Ok(true)
            }
            Err(TrySendError::Full(_)) => Ok(false),
            Err(TrySendError::Closed(_)) => eyre::bail!("WS log subscriber channel closed"),
        }
    }

    fn drop_line(&mut self, now: tokio::time::Instant) -> Delivery {
        self.dropped += 1;
        Delivery::Full {
            stalled: now - *self.full_since.get_or_insert(now),
        }
    }

    fn passes_filter(&self, message: &LogMessage) -> bool {
        match message.level {
            dora_core::build::LogLevelOrStdout::LogLevel(level) => level <= self.level,
            dora_core::build::LogLevelOrStdout::Stdout => true,
        }
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
