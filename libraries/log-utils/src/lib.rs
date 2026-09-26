//! **Internal to dora — not a public API.**
//!
//! This crate is published to crates.io only because cargo requires every
//! dependency of a published crate to be published; `dora-node-api` and
//! `dora-cli` depend on it. It is not covered by dora's 1.0 stability
//! guarantee and may change in any release, including a patch.
//!
//! Depend on it directly at your own risk. See the "Stability scope at 1.0"
//! section of `docs/api-rust.md`.
//!
use dora_message::common::LogMessage;
use eyre::{Context, Result, bail};

/// Maximum size of a single log JSON string (8 MiB).
///
/// This must stay above the largest entry the daemon forwards, or a sink
/// silently loses lines the daemon deliberately kept. The daemon truncates a
/// node's log line at 1 MiB (`MAX_LOG_LINE_BYTES` in `dora-daemon`) and then
/// JSON-encodes the resulting `LogMessage`, which can grow the message up to
/// 6x (`\u00XX` escapes for control characters) plus the envelope fields.
/// 8 MiB covers that worst case while still bounding what a sink will parse.
const MAX_LOG_JSON_BYTES: usize = 8 * 1024 * 1024;

/// Parse a [`LogMessage`] from a JSON string.
///
/// Log entries routed via `send_logs_as` arrive as JSON-encoded strings.
/// This function deserializes them back into a [`LogMessage`].
/// Rejects inputs larger than 8 MiB to prevent unbounded allocation.
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
/// Convenience wrapper for node event handlers: the daemon sends one log
/// entry per Arrow message as a single-element string array, and this
/// extracts that element and parses it as JSON via [`parse_log`].
///
/// The input must be a string array of **exactly one** non-null element —
/// the underlying `&str` conversion rejects empty, multi-element, and
/// null arrays — so a batch of several strings is an error, not a case
/// where the extra elements are silently ignored.
///
/// ```
/// use dora_arrow_convert::IntoArrow;
/// use dora_log_utils::parse_log_from_arrow;
///
/// // More than one element is rejected: exactly one is required.
/// let batched = vec!["{}".to_string(), "{}".to_string()].into_arrow();
/// assert!(parse_log_from_arrow(&batched).is_err());
/// ```
pub fn parse_log_from_arrow(data: &dora_arrow_convert::DoraArray) -> Result<LogMessage> {
    let json: &str = data.try_into().context("expected string arrow data")?;
    parse_log(json)
}

/// Format a log entry as a JSON string (one line, no trailing newline).
///
/// Callers writing JSONL should append `"\n"` after each call.
pub fn format_json(log: &LogMessage) -> String {
    serde_json::to_string(log).expect("LogMessage serialization is infallible")
}

#[cfg(test)]
mod tests {
    use super::*;
    use chrono::Utc;
    use dora_message::common::{LogLevel, LogLevelOrStdout};

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
    fn format_json_roundtrip() {
        let log = make_log(LogLevelOrStdout::LogLevel(LogLevel::Info), "test", "node1");
        let json = format_json(&log);
        let parsed: LogMessage = serde_json::from_str(&json).unwrap();
        assert_eq!(parsed.message, "test");
    }

    /// The daemon forwards log lines up to 1 MiB (then marks them
    /// truncated); a sink must be able to parse every such entry, including
    /// one whose message JSON-escapes to several times its raw size.
    #[test]
    fn parse_accepts_largest_daemon_forwarded_line() {
        let mut message = "x".repeat(1024 * 1024);
        message.push_str("... [truncated]");
        let log = make_log(LogLevelOrStdout::Stdout, &message, "sensor");
        let parsed = parse_log(&format_json(&log)).unwrap();
        assert_eq!(parsed.message, message);

        let control = "\u{1}".repeat(1024 * 1024);
        let log = make_log(LogLevelOrStdout::Stdout, &control, "sensor");
        let json = format_json(&log);
        assert!(json.len() > 6 * 1024 * 1024);
        assert_eq!(parse_log(&json).unwrap().message, control);
    }

    #[test]
    fn parse_rejects_oversized_json() {
        let huge = "x".repeat(MAX_LOG_JSON_BYTES + 1);
        let err = parse_log(&huge);
        assert!(err.is_err());
        let msg = format!("{}", err.unwrap_err());
        assert!(msg.contains("exceeds maximum size"));
    }
}
