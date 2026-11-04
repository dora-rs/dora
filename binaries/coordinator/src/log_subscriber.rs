use dora_message::{common::DROPPED_LOG_LINES_FIELD, coordinator_to_cli::LogMessage};
use eyre::{Context, ContextCompat};
use std::sync::{
    Arc,
    atomic::{AtomicUsize, Ordering},
};
use tokio::sync::mpsc;

/// How many bytes of encoded log lines one WS connection may have queued
/// before live lines to it are dropped.
///
/// The coordinator's event loop never waits on a log reader (#3794), so this
/// queue is what absorbs a reader that falls behind. It is bounded by bytes,
/// not by entries, because a single line can be up to 1 MiB.
pub(crate) const MAX_QUEUED_LOG_BYTES: usize = 16 * 1024 * 1024;

/// Create the log queue of one WS connection: the coordinator pushes into
/// the [`LogSink`] and the connection's task drains the [`LogQueue`].
pub(crate) fn log_queue() -> (LogSink, LogQueue) {
    let (tx, rx) = mpsc::unbounded_channel();
    let queued = Arc::new(AtomicUsize::new(0));
    (
        LogSink {
            tx,
            queued: queued.clone(),
        },
        LogQueue { rx, queued },
    )
}

/// The sending half of a connection's log queue.
#[derive(Debug, Clone)]
pub struct LogSink {
    tx: mpsc::UnboundedSender<String>,
    queued: Arc<AtomicUsize>,
}

impl LogSink {
    fn has_room_for(&self, line: &str) -> bool {
        self.queued.load(Ordering::Relaxed) + line.len() <= MAX_QUEUED_LOG_BYTES
    }

    pub(crate) fn push(&self, line: String) -> eyre::Result<()> {
        let len = line.len();
        self.queued.fetch_add(len, Ordering::Relaxed);
        self.tx.send(line).map_err(|_| {
            self.queued.fetch_sub(len, Ordering::Relaxed);
            eyre::eyre!("WS log subscriber channel closed")
        })
    }

    fn is_closed(&self) -> bool {
        self.tx.is_closed()
    }
}

/// The receiving half of a connection's log queue.
pub(crate) struct LogQueue {
    rx: mpsc::UnboundedReceiver<String>,
    queued: Arc<AtomicUsize>,
}

impl LogQueue {
    pub(crate) async fn recv(&mut self) -> Option<String> {
        let line = self.rx.recv().await?;
        self.queued.fetch_sub(line.len(), Ordering::Relaxed);
        Some(line)
    }

    pub(crate) fn try_recv(&mut self) -> Option<String> {
        let line = self.rx.try_recv().ok()?;
        self.queued.fetch_sub(line.len(), Ordering::Relaxed);
        Some(line)
    }

    pub(crate) fn len(&self) -> usize {
        self.rx.len()
    }
}

/// Whether a line may be dropped when the connection's queue is over
/// [`MAX_QUEUED_LOG_BYTES`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Admission {
    /// A live line: dropped (and counted) when the queue is over its cap.
    Capped,
    /// Always queued: the backlog replayed on attach, which is already held
    /// in memory, and the last line a subscriber gets before it is dropped,
    /// which also carries any pending dropped-lines notice.
    Always,
}

pub struct LogSubscriber {
    pub level: log::LevelFilter,
    sink: Option<LogSink>,
    /// Live lines dropped over the byte cap since the last one that went out.
    dropped: u64,
}

fn encode(message: &LogMessage) -> eyre::Result<String> {
    serde_json::to_string(&dora_message::ws_protocol::WsEvent {
        event: "log".to_string(),
        payload: serde_json::to_value(message)?,
    })
    .context("failed to serialize log WsEvent")
}

/// The line a reader gets in place of the `dropped` lines it fell behind on.
///
/// It carries [`DROPPED_LOG_LINES_FIELD`] so the CLI shows it even when it
/// filters the stream by node or pattern.
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
        fields: Some([(DROPPED_LOG_LINES_FIELD.to_owned(), dropped.to_string())].into()),
    }
}

impl LogSubscriber {
    pub(crate) fn new(level: log::LevelFilter, sink: LogSink) -> Self {
        Self {
            level,
            sink: Some(sink),
            dropped: 0,
        }
    }

    /// Queue `message` for this subscriber without waiting.
    ///
    /// A [`Admission::Capped`] line is dropped and counted when the
    /// connection's queue is over [`MAX_QUEUED_LOG_BYTES`]. The next line
    /// that is queued goes out right after a notice of how many were
    /// dropped, so a reader that drains at all keeps getting real lines.
    pub(crate) fn send(&mut self, message: &LogMessage, admission: Admission) -> eyre::Result<()> {
        let sink = self.sink.as_ref().context("subscriber is closed")?;
        if !self.passes_filter(message) {
            // A last line below the filter still flushes the notice, since
            // no later line will.
            if admission == Admission::Always && self.dropped > 0 {
                sink.push(encode(&dropped_lines_notice(message, self.dropped))?)?;
                self.dropped = 0;
            }
            return Ok(());
        }
        let line = encode(message)?;
        if admission == Admission::Capped && !sink.has_room_for(&line) {
            self.dropped += 1;
            return Ok(());
        }
        if self.dropped > 0 {
            sink.push(encode(&dropped_lines_notice(message, self.dropped))?)?;
            self.dropped = 0;
        }
        sink.push(line)
    }

    fn passes_filter(&self, message: &LogMessage) -> bool {
        match message.level {
            dora_core::build::LogLevelOrStdout::LogLevel(level) => level <= self.level,
            dora_core::build::LogLevelOrStdout::Stdout => true,
        }
    }

    pub fn is_closed(&self) -> bool {
        match &self.sink {
            None => true,
            Some(sink) => sink.is_closed(),
        }
    }

    pub fn close(&mut self) {
        self.sink = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn close_marks_subscriber_closed() {
        let (sink, _queue) = log_queue();
        let mut sub = LogSubscriber::new(log::LevelFilter::Info, sink);
        assert!(!sub.is_closed());
        sub.close();
        assert!(sub.is_closed());
    }

    #[test]
    fn dropping_the_queue_closes_the_subscriber() {
        let (sink, queue) = log_queue();
        let sub = LogSubscriber::new(log::LevelFilter::Info, sink);
        drop(queue);
        assert!(sub.is_closed());
    }

    #[test]
    fn queued_bytes_track_what_the_connection_has_not_drained() {
        let (sink, mut queue) = log_queue();
        sink.push("abc".into()).unwrap();
        sink.push("de".into()).unwrap();
        assert_eq!(sink.queued.load(Ordering::Relaxed), 5);
        assert_eq!(queue.try_recv().as_deref(), Some("abc"));
        assert_eq!(sink.queued.load(Ordering::Relaxed), 2);
        assert_eq!(queue.try_recv().as_deref(), Some("de"));
        assert_eq!(sink.queued.load(Ordering::Relaxed), 0);
        assert_eq!(queue.try_recv(), None);
    }
}
