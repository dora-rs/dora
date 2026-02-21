use adora_message::common::{LogLevel, LogLevelOrStdout, LogMessage};
use chrono::{DateTime, Utc};
use eyre::{Context, Result, bail};

/// Maximum size of a single log JSON string (64 KB).
const MAX_LOG_JSON_BYTES: usize = 64 * 1024;

/// Parse a [`LogMessage`] from a JSON string.
///
/// Log entries routed via `send_logs_as` arrive as JSON-encoded strings.
/// This function deserializes them back into a [`LogMessage`].
/// Rejects inputs larger than 64 KB to prevent unbounded allocation.
pub fn parse_log(json: &str) -> Result<LogMessage> {
    if json.len() > MAX_LOG_JSON_BYTES {
        bail!(
            "log JSON exceeds maximum size ({} bytes, limit {})",
            json.len(),
            MAX_LOG_JSON_BYTES
        );
    }
    serde_json::from_str(json).context("failed to parse log JSON")
}

/// Parse a [`LogMessage`] from Arrow input data.
///
/// Convenience wrapper for node event handlers. The daemon sends one log
/// entry per Arrow message, so this extracts the first string element and
/// parses it as JSON. Additional elements (if any) are ignored.
pub fn parse_log_from_arrow(data: &adora_arrow_convert::ArrowData) -> Result<LogMessage> {
    let json: &str = data.try_into().context("expected string arrow data")?;
    parse_log(json)
}

/// Merge multiple log vectors into a single timeline sorted by timestamp.
pub fn merge_by_timestamp(streams: Vec<Vec<LogMessage>>) -> Vec<LogMessage> {
    let mut merged: Vec<LogMessage> = streams.into_iter().flatten().collect();
    merged.sort_by_key(|m| m.timestamp);
    merged
}

/// Filter logs that pass the given minimum level threshold.
///
/// Uses the same filtering logic as the daemon: a message passes if its
/// level is at least as severe as `min_level`.
pub fn filter_by_level<'a>(
    logs: &'a [LogMessage],
    min_level: &LogLevelOrStdout,
) -> Vec<&'a LogMessage> {
    logs.iter().filter(|m| m.level.passes(min_level)).collect()
}

/// Format a log entry as a JSON string (one line, no trailing newline).
///
/// Callers writing JSONL should append `"\n"` after each call.
pub fn format_json(log: &LogMessage) -> String {
    serde_json::to_string(log).expect("LogMessage serialization is infallible")
}

/// Format a log entry as a compact single-line string.
///
/// Format: `<timestamp> <node> <LEVEL>: <message>`
pub fn format_compact(log: &LogMessage) -> String {
    let ts = format_timestamp(&log.timestamp);
    let node = log
        .node_id
        .as_ref()
        .map(|n| n.to_string())
        .unwrap_or_default();
    let level = format_level(&log.level);
    format!("{ts} {node} {level}: {}", log.message)
}

/// Format a log entry as a pretty human-readable string.
///
/// Format: `[<timestamp>][<LEVEL>][<node>] <message>`
pub fn format_pretty(log: &LogMessage) -> String {
    let ts = format_timestamp(&log.timestamp);
    let node = log
        .node_id
        .as_ref()
        .map(|n| n.to_string())
        .unwrap_or_default();
    let level = format_level(&log.level);
    format!("[{ts}][{level:6}][{node}] {}", log.message)
}

fn format_timestamp(ts: &DateTime<Utc>) -> String {
    ts.format("%Y-%m-%dT%H:%M:%S%.3fZ").to_string()
}

fn format_level(level: &LogLevelOrStdout) -> &'static str {
    match level {
        LogLevelOrStdout::Stdout => "STDOUT",
        LogLevelOrStdout::LogLevel(l) => match *l {
            LogLevel::Error => "ERROR",
            LogLevel::Warn => "WARN",
            LogLevel::Info => "INFO",
            LogLevel::Debug => "DEBUG",
            LogLevel::Trace => "TRACE",
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use chrono::Utc;

    fn make_log(level: LogLevelOrStdout, msg: &str, node: &str) -> LogMessage {
        LogMessage {
            build_id: None,
            dataflow_id: None,
            node_id: Some(node.to_string().into()),
            daemon_id: None,
            level,
            target: None,
            module_path: None,
            file: None,
            line: None,
            message: msg.to_string(),
            timestamp: Utc::now(),
            fields: None,
        }
    }

    #[test]
    fn parse_roundtrip() {
        let log = make_log(
            LogLevelOrStdout::LogLevel(LogLevel::Info),
            "hello",
            "sensor",
        );
        let json = serde_json::to_string(&log).unwrap();
        let parsed = parse_log(&json).unwrap();
        assert_eq!(parsed.message, "hello");
        assert_eq!(parsed.node_id, log.node_id);
    }

    #[test]
    fn parse_invalid_json() {
        assert!(parse_log("not json").is_err());
    }

    #[test]
    fn merge_sorts_by_timestamp() {
        let now = Utc::now();
        let early = now - chrono::Duration::seconds(10);
        let late = now + chrono::Duration::seconds(10);

        let mut log1 = make_log(LogLevelOrStdout::LogLevel(LogLevel::Info), "late", "a");
        log1.timestamp = late;

        let mut log2 = make_log(LogLevelOrStdout::LogLevel(LogLevel::Info), "early", "b");
        log2.timestamp = early;

        let merged = merge_by_timestamp(vec![vec![log1], vec![log2]]);
        assert_eq!(merged[0].message, "early");
        assert_eq!(merged[1].message, "late");
    }

    #[test]
    fn filter_by_level_filters_correctly() {
        let error = make_log(LogLevelOrStdout::LogLevel(LogLevel::Error), "err", "a");
        let info = make_log(LogLevelOrStdout::LogLevel(LogLevel::Info), "info", "b");
        let debug = make_log(LogLevelOrStdout::LogLevel(LogLevel::Debug), "dbg", "c");

        let logs = vec![error, info, debug];
        let min = LogLevelOrStdout::LogLevel(LogLevel::Info);
        let filtered = filter_by_level(&logs, &min);
        assert_eq!(filtered.len(), 2);
        assert_eq!(filtered[0].message, "err");
        assert_eq!(filtered[1].message, "info");
    }

    #[test]
    fn format_compact_output() {
        let log = make_log(
            LogLevelOrStdout::LogLevel(LogLevel::Warn),
            "overheating",
            "motor",
        );
        let out = format_compact(&log);
        assert!(out.contains("motor"));
        assert!(out.contains("WARN"));
        assert!(out.contains("overheating"));
    }

    #[test]
    fn format_pretty_output() {
        let log = make_log(
            LogLevelOrStdout::LogLevel(LogLevel::Error),
            "failure",
            "sensor",
        );
        let out = format_pretty(&log);
        assert!(out.contains("[ERROR ]"));
        assert!(out.contains("[sensor]"));
        assert!(out.contains("failure"));
    }

    #[test]
    fn format_pretty_stdout_alignment() {
        let log = make_log(LogLevelOrStdout::Stdout, "raw line", "cam");
        let out = format_pretty(&log);
        assert!(out.contains("[STDOUT]"));
        assert!(out.contains("[cam]"));
    }

    #[test]
    fn format_json_roundtrip() {
        let log = make_log(LogLevelOrStdout::LogLevel(LogLevel::Info), "test", "node1");
        let json = format_json(&log);
        let parsed: LogMessage = serde_json::from_str(&json).unwrap();
        assert_eq!(parsed.message, "test");
    }

    #[test]
    fn merge_empty_streams() {
        let merged = merge_by_timestamp(vec![]);
        assert!(merged.is_empty());
    }

    #[test]
    fn merge_single_stream() {
        let now = Utc::now();
        let mut a = make_log(LogLevelOrStdout::LogLevel(LogLevel::Info), "a", "n");
        a.timestamp = now;
        let mut b = make_log(LogLevelOrStdout::LogLevel(LogLevel::Info), "b", "n");
        b.timestamp = now + chrono::Duration::seconds(1);
        let merged = merge_by_timestamp(vec![vec![b, a]]);
        assert_eq!(merged[0].message, "a");
        assert_eq!(merged[1].message, "b");
    }

    #[test]
    fn parse_rejects_oversized_json() {
        let huge = "x".repeat(65 * 1024);
        let err = parse_log(&huge);
        assert!(err.is_err());
        let msg = format!("{}", err.unwrap_err());
        assert!(msg.contains("exceeds maximum size"));
    }
}
