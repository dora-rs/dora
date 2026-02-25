use std::collections::HashMap;
use std::hash::{DefaultHasher, Hash, Hasher};

use adora_core::build::LogLevelOrStdout;
use adora_message::common::LogMessage;
use chrono::Local;
use colored::{Color, Colorize};

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

    let is_system = node_id.is_none();

    let level_str = match &level {
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
        Some(ref node_id) => {
            let colored_id = node_id
                .to_string()
                .bold()
                .color(word_to_color(node_id.as_ref()));
            let padding = if daemon.is_empty() { "" } else { " " };
            format!("{colored_id}{padding}{daemon}{colon} ")
        }
        None => {
            let prefix = "[adora]".dimmed();
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

    if is_system && is_lifecycle_message(&message) {
        println!();
    }
    println!("{time} {level_str} {dataflow} {node}{target} {message}");
    if is_system && is_lifecycle_message(&message) {
        println!();
    }
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
        .unwrap_or_else(|| "adora".to_string());
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
    let level_str = v.get("level")?.as_str().unwrap_or("stdout");
    let level = match level_str {
        "error" => LogLevelOrStdout::LogLevel(log::Level::Error),
        "warn" => LogLevelOrStdout::LogLevel(log::Level::Warn),
        "info" => LogLevelOrStdout::LogLevel(log::Level::Info),
        "debug" => LogLevelOrStdout::LogLevel(log::Level::Debug),
        "trace" => LogLevelOrStdout::LogLevel(log::Level::Trace),
        _ => LogLevelOrStdout::Stdout,
    };
    let node_id = v
        .get("node")
        .and_then(|n| n.as_str())
        .map(|s| adora_message::id::NodeId::from(s.to_string()));
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
    let r = (base_r as f32 * 0.5 + repeat_ratio * 255.0 * 0.2 + char_sum * 0.3) as u8;
    let g = (base_g as f32 * 0.5 + diversity * 255.0 * 0.25 + length_factor * 255.0 * 0.25) as u8;
    let b =
        (base_b as f32 * 0.5 + (1.0 - repeat_ratio) * 255.0 * 0.3 + (1.0 - char_sum) * 0.2) as u8;

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
}
