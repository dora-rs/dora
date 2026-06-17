use dora_message::common::LogMessage;
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
pub fn parse_log_from_arrow(data: &dora_arrow_convert::ArrowData) -> Result<LogMessage> {
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

    #[test]
    fn parse_rejects_oversized_json() {
        let huge = "x".repeat(65 * 1024);
        let err = parse_log(&huge);
        assert!(err.is_err());
        let msg = format!("{}", err.unwrap_err());
        assert!(msg.contains("exceeds maximum size"));
    }
}
