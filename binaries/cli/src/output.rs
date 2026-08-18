use std::collections::HashMap;
use std::hash::{DefaultHasher, Hash, Hasher};

use chrono::Local;
use colored::{Color, Colorize};
use dora_core::build::LogLevelOrStdout;
use dora_message::common::LogMessage;

#[derive(Debug, Clone, Copy, Default, clap::ValueEnum)]
pub enum LogFormat {
    #[default]
    Pretty,
    Json,
    Compact,
}

#[derive(Debug, Clone)]
pub struct LogOutputConfig {
    pub min_level: LogLevelOrStdout,
    pub format: LogFormat,
    pub node_filters: HashMap<String, LogLevelOrStdout>,
    pub print_dataflow_id: bool,
    pub print_daemon_name: bool,
}

impl Default for LogOutputConfig {
    fn default() -> Self {
        Self {
            min_level: LogLevelOrStdout::Stdout,
            format: LogFormat::Pretty,
            node_filters: HashMap::new(),
            print_dataflow_id: false,
            print_daemon_name: false,
        }
    }
}

fn should_display(
    msg_level: &LogLevelOrStdout,
    msg_node: Option<&str>,
    config: &LogOutputConfig,
) -> bool {
    let effective_level = msg_node
        .and_then(|n| config.node_filters.get(n))
        .unwrap_or(&config.min_level);
    msg_level.passes(effective_level)
}

/// Returns whether `msg` passes the configured minimum-level / per-node level
/// filters. Exposed so the `logs` command can drop filtered-out messages
/// *before* applying `--tail`, so tail counts only lines that will be shown.
pub(crate) fn message_passes_level_filter(msg: &LogMessage, config: &LogOutputConfig) -> bool {
    let node_id_str = msg.node_id.as_ref().map(|n| n.to_string());
    should_display(&msg.level, node_id_str.as_deref(), config)
}

/// Returns whether the configured level filters can drop any message. When
/// this is true, `--tail` must be applied client-side (after filtering) rather
/// than pre-tailed at the coordinator, otherwise older matching lines are
/// trimmed away before the level filter ever sees them.
pub(crate) fn level_filter_is_active(config: &LogOutputConfig) -> bool {
    !matches!(config.min_level, LogLevelOrStdout::Stdout) || !config.node_filters.is_empty()
}

pub fn print_log_message(log_message: LogMessage, config: &LogOutputConfig) {
    let node_id_str = log_message.node_id.as_ref().map(|n| n.to_string());
    if !should_display(&log_message.level, node_id_str.as_deref(), config) {
        return;
    }

    match config.format {
        LogFormat::Pretty => print_pretty(log_message, config),
        LogFormat::Json => print_json(&log_message),
        LogFormat::Compact => print_compact(&log_message),
    }
}

fn print_pretty(log_message: LogMessage, config: &LogOutputConfig) {
    let is_system = log_message.node_id.is_none();
    let is_lifecycle = is_system && is_lifecycle_message(&log_message.message);
    let line = format_pretty_line(&log_message, config);

    if is_lifecycle {
        println!();
    }
    println!("{line}");
    if is_lifecycle {
        println!();
    }
}

/// Render a single log message as a colored one-line string for the `pretty`
/// format. Kept separate from [`print_pretty`] so the exact column spacing is
/// unit-testable without capturing stdout.
fn format_pretty_line(log_message: &LogMessage, config: &LogOutputConfig) -> String {
    let LogMessage {
        build_id: _,
        dataflow_id,
        node_id,
        daemon_id,
        level,
        target,
        module_path: _,
        file: _,
        line: _,
        message,
        timestamp,
        fields: _,
    } = log_message;

    let level_str = match level {
        LogLevelOrStdout::LogLevel(level) => match level {
            log::Level::Error => "ERROR ".red(),
            log::Level::Warn => "WARN  ".yellow(),
            log::Level::Info => "INFO  ".green(),
            log::Level::Debug => "DEBUG ".bright_blue(),
            log::Level::Trace => "TRACE ".dimmed(),
        },
        LogLevelOrStdout::Stdout => "stdout".bright_blue().italic().dimmed(),
    };

    let dataflow = match dataflow_id {
        Some(dataflow_id) if config.print_dataflow_id => {
            format!("dataflow `{dataflow_id}` ").cyan()
        }
        _ => String::new().cyan(),
    };
    let daemon = match daemon_id {
        Some(id) if config.print_daemon_name => match id.machine_id() {
            Some(machine_id) => format!("on daemon `{machine_id}`"),
            None => "on default daemon ".to_string(),
        },
        None if config.print_daemon_name => "on default daemon".to_string(),
        _ => String::new(),
    }
    .bright_black();
    let time = format!("{}", timestamp.with_timezone(&Local).format("%H:%M:%S"));
    let colon = ":".bright_black().bold();
    let node = match node_id {
        Some(node_id) => {
            let colored_id = node_id
                .to_string()
                .bold()
                .color(word_to_color(node_id.as_ref()));
            let padding = if daemon.is_empty() { "" } else { " " };
            format!("{colored_id}{padding}{daemon}{colon} ")
        }
        None => {
            let prefix = "[dora]".dimmed();
            if daemon.is_empty() {
                format!("{prefix}{colon} ")
            } else {
                format!("{prefix} {daemon}{colon} ")
            }
        }
    };
    let target = match target {
        Some(target) => format!("{target} ").dimmed(),
        None => "".normal(),
    };

    // `dataflow` and `target` already carry their own trailing space when
    // present and are empty strings otherwise, so they must abut the next
    // field directly — inserting a literal space around them here produced a
    // spurious double space on every line where they were empty (the default,
    // i.e. almost all output).
    format!("{time} {level_str} {dataflow}{node}{target}{message}")
}

fn is_lifecycle_message(message: &str) -> bool {
    message.contains("spawning")
        || message.contains("node finished")
        || message.contains("stopping")
}

fn print_json(log_message: &LogMessage) {
    if let Ok(json) = serde_json::to_string(log_message) {
        println!("{json}");
    }
}

fn print_compact(log_message: &LogMessage) {
    let time = log_message
        .timestamp
        .with_timezone(&Local)
        .format("%H:%M:%S");
    let level = match &log_message.level {
        LogLevelOrStdout::LogLevel(l) => match l {
            log::Level::Error => "ERROR",
            log::Level::Warn => "WARN",
            log::Level::Info => "INFO",
            log::Level::Debug => "DEBUG",
            log::Level::Trace => "TRACE",
        },
        LogLevelOrStdout::Stdout => "STDOUT",
    };
    let node = log_message
        .node_id
        .as_ref()
        .map(|n| n.to_string())
        .unwrap_or_else(|| "dora".to_string());
    println!("{time} {level} {node}: {}", log_message.message);
}

/// Parse a JSONL log line into a LogMessage.
/// Handles both the daemon's compact format (ts/level/node/msg) and full LogMessage.
pub fn parse_jsonl_line(line: &str) -> Option<LogMessage> {
    // Try full LogMessage format first
    if let Ok(msg) = serde_json::from_str::<LogMessage>(line) {
        return Some(msg);
    }
    // Try daemon compact JSONL format
    let v: serde_json::Value = serde_json::from_str(line).ok()?;
    let ts = v.get("ts")?.as_str()?;
    let timestamp = chrono::DateTime::parse_from_rfc3339(ts).ok()?.to_utc();
    // A missing `level` key, a non-string value, or an unrecognized level all
    // default to stdout, so that externally-produced JSONL log lines are still
    // shown rather than silently dropped. Reuse `parse_log_level_str` (the
    // single source of truth for level names) so a capitalized level such as
    // "INFO" is classified correctly instead of falling through to stdout.
    let level = v
        .get("level")
        .and_then(|l| l.as_str())
        .and_then(|s| parse_log_level_str(s).ok())
        .unwrap_or(LogLevelOrStdout::Stdout);
    // Parse fallibly: `NodeId::from` panics on any id that fails validation,
    // and this input is untrusted -- it comes from a log file that may have
    // been written by an older dora version (whose id rules were laxer) or by
    // an external producer. A malformed id degrades to `None` (rendered as the
    // default node label) instead of aborting the whole `dora logs` command.
    let node_id = v
        .get("node")
        .and_then(|n| n.as_str())
        .and_then(|s| s.parse::<dora_message::id::NodeId>().ok());
    let message = v
        .get("msg")
        .and_then(|m| m.as_str())
        .unwrap_or("")
        .to_string();
    let target = v
        .get("target")
        .and_then(|t| t.as_str())
        .map(|s| s.to_string());

    Some(LogMessage {
        build_id: None,
        dataflow_id: None,
        node_id,
        daemon_id: None,
        level,
        target,
        module_path: None,
        file: None,
        line: None,
        message,
        timestamp,
        fields: None,
    })
}

/// Parse a log filter string like "sensor=debug,processor=warn".
pub fn parse_log_filter(s: &str) -> Result<HashMap<String, LogLevelOrStdout>, String> {
    let mut map = HashMap::new();
    for pair in s.split(',') {
        let pair = pair.trim();
        if pair.is_empty() {
            continue;
        }
        let (node, level) = pair
            .split_once('=')
            .ok_or_else(|| format!("invalid filter: '{pair}', expected 'node=level'"))?;
        let level = parse_log_level_str(level.trim())?;
        map.insert(node.trim().to_string(), level);
    }
    Ok(map)
}

pub fn parse_log_level_str(s: &str) -> Result<LogLevelOrStdout, String> {
    match s.to_lowercase().as_str() {
        "error" => Ok(LogLevelOrStdout::LogLevel(log::Level::Error)),
        "warn" => Ok(LogLevelOrStdout::LogLevel(log::Level::Warn)),
        "info" => Ok(LogLevelOrStdout::LogLevel(log::Level::Info)),
        "debug" => Ok(LogLevelOrStdout::LogLevel(log::Level::Debug)),
        "trace" => Ok(LogLevelOrStdout::LogLevel(log::Level::Trace)),
        "stdout" => Ok(LogLevelOrStdout::Stdout),
        _ => Err(format!(
            "invalid log level: '{s}', expected one of: error, warn, info, debug, trace, stdout"
        )),
    }
}

/// Generate a color for a word based on its semantic features
/// Optimized for technical abbreviations (stt, tts, llm, vlm, etc.)
pub fn word_to_color(word: &str) -> Color {
    let word_lower = word.to_lowercase();

    // Create a simple hash for the word
    let mut hasher = DefaultHasher::new();
    word_lower.hash(&mut hasher);
    let hash = hasher.finish();

    // Extract features from the word for similarity
    let length_factor = (word_lower.len() as f32 / 5.0).min(1.0);

    // Count repeated characters (stt has 2 t's, tts has 2 t's)
    let repeat_ratio = calculate_repeat_ratio(&word_lower);

    // Character diversity - unique chars / total chars
    let diversity = calculate_char_diversity(&word_lower);

    // Sum of character positions in alphabet (normalized)
    let char_sum = calculate_char_sum(&word_lower);

    // Blend hash-based color with feature-based adjustments
    let base_r = ((hash >> 16) & 0xFF) as u8;
    let base_g = ((hash >> 8) & 0xFF) as u8;
    let base_b = (hash & 0xFF) as u8;

    // Adjust colors based on word features for similarity
    // Similar abbreviations will have similar features
    let r = (base_r as f32 * 0.5 + repeat_ratio * 255.0 * 0.2 + char_sum * 255.0 * 0.3) as u8;
    let g = (base_g as f32 * 0.5 + diversity * 255.0 * 0.25 + length_factor * 255.0 * 0.25) as u8;
    let b = (base_b as f32 * 0.5
        + (1.0 - repeat_ratio) * 255.0 * 0.3
        + (1.0 - char_sum) * 255.0 * 0.2) as u8;

    Color::TrueColor { r, g, b }
}

/// Calculate ratio of repeated characters
fn calculate_repeat_ratio(s: &str) -> f32 {
    if s.is_empty() {
        return 0.0;
    }

    let mut char_counts = std::collections::HashMap::new();
    for c in s.chars() {
        *char_counts.entry(c).or_insert(0) += 1;
    }

    let repeated = char_counts.values().filter(|&&count| count > 1).count();
    repeated as f32 / char_counts.len().max(1) as f32
}

/// Calculate character diversity (unique chars / total chars)
fn calculate_char_diversity(s: &str) -> f32 {
    if s.is_empty() {
        return 0.0;
    }

    let unique: std::collections::HashSet<_> = s.chars().collect();
    unique.len() as f32 / s.len() as f32
}

/// Exposed for testing. Returns true if a log message should be displayed
/// given the output config.
#[cfg(test)]
pub(crate) fn should_display_test(
    msg_level: &LogLevelOrStdout,
    msg_node: Option<&str>,
    config: &LogOutputConfig,
) -> bool {
    should_display(msg_level, msg_node, config)
}

/// Calculate normalized sum of character positions (a=1, z=26)
fn calculate_char_sum(s: &str) -> f32 {
    if s.is_empty() {
        return 0.0;
    }

    let sum: u32 = s
        .chars()
        .filter(|c| c.is_ascii_alphabetic())
        .map(|c| c.to_ascii_lowercase() as u32 - 'a' as u32 + 1)
        .sum();

    // Normalize by max possible sum for this length
    let max_sum = s.len() as u32 * 26;
    (sum as f32 / max_sum as f32).min(1.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    // --- format_pretty_line spacing ---

    fn pretty_message(node: Option<&str>, target: Option<&str>, message: &str) -> LogMessage {
        LogMessage {
            build_id: None,
            dataflow_id: None,
            node_id: node.map(|n| dora_message::id::NodeId::from(n.to_string())),
            daemon_id: None,
            level: LogLevelOrStdout::LogLevel(log::Level::Info),
            target: target.map(|t| t.to_string()),
            module_path: None,
            file: None,
            line: None,
            message: message.to_string(),
            timestamp: chrono::Utc::now(),
            fields: None,
        }
    }

    /// The rendered line is `"<time> <rest>"`; the time has no spaces, so this
    /// returns everything after the first space for a timezone-independent
    /// assertion.
    fn rendered_rest(msg: LogMessage, config: &LogOutputConfig) -> String {
        colored::control::set_override(false);
        let line = format_pretty_line(&msg, config);
        line.splitn(2, ' ').nth(1).unwrap().to_string()
    }

    #[test]
    fn pretty_line_default_has_no_double_space() {
        // The common case: no dataflow id, no daemon name, no target. The
        // `dataflow`/`target` fields are empty here, so they must not each
        // contribute a stray separator space (regression: `node:  message`).
        let config = LogOutputConfig::default();
        let rest = rendered_rest(pretty_message(Some("sensor"), None, "hello"), &config);
        assert_eq!(rest, "INFO   sensor: hello");
        assert!(
            !rest.contains("sensor:  hello"),
            "double space before message: {rest:?}"
        );
    }

    #[test]
    fn pretty_line_with_target_single_space() {
        let config = LogOutputConfig::default();
        let rest = rendered_rest(
            pretty_message(Some("sensor"), Some("mymod"), "hello"),
            &config,
        );
        assert_eq!(rest, "INFO   sensor: mymod hello");
        assert!(
            !rest.contains("mymod  hello"),
            "double space before message: {rest:?}"
        );
    }

    #[test]
    fn pretty_line_system_message_no_double_space() {
        let config = LogOutputConfig::default();
        let rest = rendered_rest(pretty_message(None, None, "coordinator ready"), &config);
        assert_eq!(rest, "INFO   [dora]: coordinator ready");
    }

    // --- parse_log_level_str ---

    #[test]
    fn parse_level_valid_strings() {
        assert!(matches!(
            parse_log_level_str("error"),
            Ok(LogLevelOrStdout::LogLevel(log::Level::Error))
        ));
        assert!(matches!(
            parse_log_level_str("warn"),
            Ok(LogLevelOrStdout::LogLevel(log::Level::Warn))
        ));
        assert!(matches!(
            parse_log_level_str("info"),
            Ok(LogLevelOrStdout::LogLevel(log::Level::Info))
        ));
        assert!(matches!(
            parse_log_level_str("debug"),
            Ok(LogLevelOrStdout::LogLevel(log::Level::Debug))
        ));
        assert!(matches!(
            parse_log_level_str("trace"),
            Ok(LogLevelOrStdout::LogLevel(log::Level::Trace))
        ));
        assert!(matches!(
            parse_log_level_str("stdout"),
            Ok(LogLevelOrStdout::Stdout)
        ));
    }

    #[test]
    fn parse_level_case_insensitive() {
        assert!(parse_log_level_str("INFO").is_ok());
        assert!(parse_log_level_str("Info").is_ok());
        assert!(parse_log_level_str("info").is_ok());
    }

    #[test]
    fn parse_level_invalid() {
        assert!(parse_log_level_str("invalid").is_err());
        assert!(parse_log_level_str("").is_err());
    }

    // --- parse_log_filter ---

    #[test]
    fn parse_filter_single_pair() {
        let map = parse_log_filter("sensor=debug").unwrap();
        assert_eq!(map.len(), 1);
        assert!(matches!(
            map.get("sensor"),
            Some(LogLevelOrStdout::LogLevel(log::Level::Debug))
        ));
    }

    #[test]
    fn parse_filter_multiple_pairs() {
        let map = parse_log_filter("sensor=debug,planner=warn").unwrap();
        assert_eq!(map.len(), 2);
        assert!(matches!(
            map.get("planner"),
            Some(LogLevelOrStdout::LogLevel(log::Level::Warn))
        ));
    }

    #[test]
    fn parse_filter_empty_string() {
        let map = parse_log_filter("").unwrap();
        assert!(map.is_empty());
    }

    #[test]
    fn parse_filter_trailing_comma() {
        let map = parse_log_filter("sensor=debug,").unwrap();
        assert_eq!(map.len(), 1);
    }

    #[test]
    fn parse_filter_invalid_no_equals() {
        assert!(parse_log_filter("sensorDEBUG").is_err());
    }

    #[test]
    fn parse_filter_whitespace_trimming() {
        let map = parse_log_filter("sensor = debug , planner = warn").unwrap();
        assert_eq!(map.len(), 2);
        assert!(map.contains_key("sensor"));
        assert!(map.contains_key("planner"));
    }

    // --- parse_jsonl_line ---

    #[test]
    fn parse_jsonl_daemon_compact() {
        let line = r#"{"ts":"2025-01-01T00:00:00Z","level":"info","node":"sensor","msg":"hello"}"#;
        let msg = parse_jsonl_line(line).unwrap();
        assert_eq!(msg.message, "hello");
        assert!(matches!(
            msg.level,
            LogLevelOrStdout::LogLevel(log::Level::Info)
        ));
        assert_eq!(msg.node_id.unwrap().to_string(), "sensor");
    }

    #[test]
    fn parse_jsonl_missing_level_defaults_to_stdout() {
        // A line without a `level` key must still be parsed (defaulting to
        // stdout), not silently dropped.
        let line = r#"{"ts":"2025-01-01T00:00:00Z","node":"sensor","msg":"hello"}"#;
        let msg = parse_jsonl_line(line).expect("line without level should still parse");
        assert_eq!(msg.message, "hello");
        assert!(matches!(msg.level, LogLevelOrStdout::Stdout));
        assert_eq!(msg.node_id.unwrap().to_string(), "sensor");
    }

    #[test]
    fn parse_jsonl_non_string_level_defaults_to_stdout() {
        let line = r#"{"ts":"2025-01-01T00:00:00Z","level":42,"msg":"hi"}"#;
        let msg = parse_jsonl_line(line).expect("line with non-string level should still parse");
        assert!(matches!(msg.level, LogLevelOrStdout::Stdout));
    }

    #[test]
    fn parse_jsonl_capitalized_level_is_classified() {
        // An external tool emitting a capitalized level must be classified,
        // not silently treated as stdout.
        let line = r#"{"ts":"2025-01-01T00:00:00Z","level":"INFO","msg":"hi"}"#;
        let msg = parse_jsonl_line(line).unwrap();
        assert!(matches!(
            msg.level,
            LogLevelOrStdout::LogLevel(log::Level::Info)
        ));
    }

    #[test]
    fn parse_jsonl_unknown_level_defaults_to_stdout() {
        let line = r#"{"ts":"2025-01-01T00:00:00Z","level":"bogus","msg":"hi"}"#;
        let msg = parse_jsonl_line(line).unwrap();
        assert!(matches!(msg.level, LogLevelOrStdout::Stdout));
    }

    #[test]
    fn parse_jsonl_invalid_node_id_does_not_panic() {
        // Log files written by an older dora (or by an external producer) can
        // carry a node id that today's `validate_node_id` rejects -- notably
        // the now-reserved `dora`. Parsing must degrade to `node_id: None`
        // rather than panicking, which would abort the whole `dora logs` run
        // on a single bad line.
        for node in ["dora", "bad id", "node/out", ".hidden", ""] {
            let line = format!(
                r#"{{"ts":"2025-01-01T00:00:00Z","level":"info","node":"{node}","msg":"hello"}}"#
            );
            let msg = parse_jsonl_line(&line)
                .unwrap_or_else(|| panic!("line with node `{node}` should still parse"));
            assert_eq!(msg.message, "hello");
            assert!(
                msg.node_id.is_none(),
                "invalid node id `{node}` must not yield a NodeId"
            );
        }
    }

    #[test]
    fn parse_jsonl_invalid_json() {
        assert!(parse_jsonl_line("not json at all").is_none());
    }

    #[test]
    fn parse_jsonl_empty_string() {
        assert!(parse_jsonl_line("").is_none());
    }

    // --- should_display ---

    #[test]
    fn should_display_passes_global_min_level() {
        let config = LogOutputConfig {
            min_level: LogLevelOrStdout::LogLevel(log::Level::Info),
            ..Default::default()
        };
        // Error is more severe than Info -> passes
        assert!(should_display_test(
            &LogLevelOrStdout::LogLevel(log::Level::Error),
            None,
            &config,
        ));
    }

    #[test]
    fn should_display_blocked_by_global_min_level() {
        let config = LogOutputConfig {
            min_level: LogLevelOrStdout::LogLevel(log::Level::Info),
            ..Default::default()
        };
        // Debug is more verbose than Info -> blocked
        assert!(!should_display_test(
            &LogLevelOrStdout::LogLevel(log::Level::Debug),
            None,
            &config,
        ));
    }

    #[test]
    fn should_display_per_node_override() {
        let mut node_filters = HashMap::new();
        node_filters.insert(
            "sensor".to_string(),
            LogLevelOrStdout::LogLevel(log::Level::Debug),
        );
        let config = LogOutputConfig {
            min_level: LogLevelOrStdout::LogLevel(log::Level::Error),
            node_filters,
            ..Default::default()
        };
        // Global says Error-only, but sensor override allows Debug
        assert!(should_display_test(
            &LogLevelOrStdout::LogLevel(log::Level::Debug),
            Some("sensor"),
            &config,
        ));
        // Other nodes still use global Error filter
        assert!(!should_display_test(
            &LogLevelOrStdout::LogLevel(log::Level::Debug),
            Some("other"),
            &config,
        ));
    }

    // --- word_to_color ---

    #[test]
    fn word_to_color_returns_true_color() {
        // Smoke test: function does not panic and returns TrueColor
        assert!(matches!(word_to_color("stt"), Color::TrueColor { .. }));
        assert!(matches!(word_to_color("vlm"), Color::TrueColor { .. }));
    }

    #[test]
    fn word_to_color_char_sum_affects_output() {
        // "aaa" has char_sum ≈ 1/26 (low); "zzz" has char_sum = 1.0 (high).
        // After the fix (char_sum * 255.0 * weight), the two words must
        // produce different colors. Before the fix both terms rounded to 0
        // and the colors could be identical despite a 26x difference in
        // char_sum.
        let low = word_to_color("aaa");
        let high = word_to_color("zzz");
        assert_ne!(
            low, high,
            "char_sum should influence color: 'aaa' and 'zzz' must differ"
        );
    }
}
