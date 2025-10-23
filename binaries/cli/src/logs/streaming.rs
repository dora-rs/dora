// Log Streaming Implementation for Issue #20

use super::types::*;
use chrono::Utc;
use eyre::Result;
use std::collections::VecDeque;

/// Mock log stream for demonstration
pub struct LogStream {
    buffer: VecDeque<LogEntry>,
    follow_mode: bool,
    current_index: usize,
}

impl LogStream {
    pub fn new(follow_mode: bool) -> Self {
        Self {
            buffer: VecDeque::new(),
            follow_mode,
            current_index: 0,
        }
    }

    /// Get next log entry from stream
    pub async fn next_log(&mut self) -> Result<Option<LogEntry>> {
        // Mock implementation - would connect to real log source
        if self.buffer.is_empty() {
            self.generate_mock_logs();
        }

        Ok(self.buffer.pop_front())
    }

    fn generate_mock_logs(&mut self) {
        // Generate some mock log entries for demonstration
        let log_levels = vec![
            LogLevel::Info,
            LogLevel::Info,
            LogLevel::Debug,
            LogLevel::Warn,
            LogLevel::Error,
        ];

        for (i, level) in log_levels.iter().enumerate() {
            let entry = LogEntry::new(
                *level,
                format!("Mock log message {}", self.current_index + i),
            )
            .with_source("mock-source".to_string());

            self.buffer.push_back(entry);
        }

        self.current_index += log_levels.len();
    }

    pub fn is_active(&self) -> bool {
        self.follow_mode || !self.buffer.is_empty()
    }
}

/// Log buffer for maintaining recent logs
pub struct LogBuffer {
    logs: VecDeque<LogEntry>,
    capacity: usize,
}

impl LogBuffer {
    pub fn new(capacity: usize) -> Self {
        Self {
            logs: VecDeque::with_capacity(capacity),
            capacity,
        }
    }

    pub fn add(&mut self, log: LogEntry) {
        if self.logs.len() >= self.capacity {
            self.logs.pop_front();
        }
        self.logs.push_back(log);
    }

    pub fn get_all_logs(&self) -> Vec<LogEntry> {
        self.logs.iter().cloned().collect()
    }

    pub fn len(&self) -> usize {
        self.logs.len()
    }

    pub fn is_empty(&self) -> bool {
        self.logs.is_empty()
    }
}

/// Rate limiter for log streaming
pub struct RateLimiter {
    max_rate: u32,
    current_count: u32,
    last_reset: std::time::Instant,
}

impl RateLimiter {
    pub fn new(max_rate: u32) -> Self {
        Self {
            max_rate,
            current_count: 0,
            last_reset: std::time::Instant::now(),
        }
    }

    pub fn should_allow(&mut self, _log: &LogEntry) -> bool {
        // Reset counter if 1 second has passed
        if self.last_reset.elapsed().as_secs() >= 1 {
            self.current_count = 0;
            self.last_reset = std::time::Instant::now();
        }

        if self.current_count < self.max_rate {
            self.current_count += 1;
            true
        } else {
            false
        }
    }
}

/// Smart filter for intelligent log filtering
pub struct SmartLogFilter {
    filter_config: LogFilterConfig,
}

impl SmartLogFilter {
    pub fn new(filter_config: LogFilterConfig) -> Self {
        Self { filter_config }
    }

    pub fn should_display(&self, log: &LogEntry) -> bool {
        // Check level filter
        if !self.filter_config.levels.is_empty() && !log.matches_level(&self.filter_config.levels) {
            return false;
        }

        // Check errors_only filter
        if self.filter_config.errors_only
            && !matches!(log.level, LogLevel::Error | LogLevel::Fatal)
        {
            return false;
        }

        // Check include patterns
        if !self.filter_config.include_patterns.is_empty() {
            let matches_any = self
                .filter_config
                .include_patterns
                .iter()
                .any(|pattern| log.matches_pattern(pattern));
            if !matches_any {
                return false;
            }
        }

        // Check exclude patterns
        if !self.filter_config.exclude_patterns.is_empty() {
            let matches_any = self
                .filter_config
                .exclude_patterns
                .iter()
                .any(|pattern| log.matches_pattern(pattern));
            if matches_any {
                return false;
            }
        }

        // Check search patterns
        if !self.filter_config.search_patterns.is_empty() {
            let matches_any = self
                .filter_config
                .search_patterns
                .iter()
                .any(|pattern| log.matches_pattern(pattern));
            if !matches_any {
                return false;
            }
        }

        true
    }

    pub fn apply_smart_filtering(&self, logs: Vec<LogEntry>) -> Vec<LogEntry> {
        if !self.filter_config.smart_filtering {
            return logs;
        }

        // Smart filtering - reduce noise while preserving important logs
        let mut filtered = Vec::new();
        let mut seen_messages = std::collections::HashSet::new();

        for log in logs {
            // Always include errors and warnings
            if matches!(
                log.level,
                LogLevel::Error | LogLevel::Fatal | LogLevel::Warn
            ) {
                filtered.push(log);
                continue;
            }

            // For info/debug, deduplicate similar messages
            let message_key = self.extract_message_key(&log.message);
            if !seen_messages.contains(&message_key) {
                seen_messages.insert(message_key);
                filtered.push(log);
            }
        }

        filtered
    }

    fn extract_message_key(&self, message: &str) -> String {
        // Extract message pattern without variable parts
        message
            .split_whitespace()
            .take(5)
            .collect::<Vec<_>>()
            .join(" ")
    }
}

/// Log renderer for CLI output
pub struct LogRenderer {
    supports_color: bool,
    show_timestamps: bool,
    format: LogFormat,
}

impl LogRenderer {
    pub fn new(supports_color: bool, show_timestamps: bool, format: LogFormat) -> Self {
        Self {
            supports_color,
            show_timestamps,
            format,
        }
    }

    pub fn render(&self, log: &LogEntry) -> String {
        let timestamp_str = if self.show_timestamps {
            format!("{} ", log.timestamp.format("%H:%M:%S%.3f"))
        } else {
            String::new()
        };

        let level_str = if self.supports_color {
            self.colorize_log_level(&log.level)
        } else {
            format!("[{}]", log.level)
        };

        let source_str = if let Some(source) = &log.source {
            format!(" {}", source)
        } else {
            String::new()
        };

        match self.format {
            LogFormat::Text | LogFormat::Colored => {
                format!("{}{}{}: {}", timestamp_str, level_str, source_str, log.message)
            }
            LogFormat::Json => {
                serde_json::json!({
                    "timestamp": log.timestamp,
                    "level": log.level,
                    "source": log.source,
                    "message": log.message,
                    "fields": log.fields,
                })
                .to_string()
            }
            LogFormat::Compact => {
                format!("{}{} {}", level_str, source_str, log.message)
            }
            LogFormat::Detailed => {
                let mut lines = vec![
                    format!("┌─ {} {} {}", timestamp_str, level_str, source_str),
                    format!("│  {}", log.message),
                ];

                for (key, value) in &log.fields {
                    lines.push(format!("│  {}: {}", key, value));
                }

                lines.push("└─".to_string());
                lines.join("\n")
            }
        }
    }

    fn colorize_log_level(&self, level: &LogLevel) -> String {
        match level {
            LogLevel::Trace => "\x1b[90m[TRACE]\x1b[0m".to_string(),
            LogLevel::Debug => "\x1b[36m[DEBUG]\x1b[0m".to_string(),
            LogLevel::Info => "\x1b[32m[INFO]\x1b[0m".to_string(),
            LogLevel::Warn => "\x1b[33m[WARN]\x1b[0m".to_string(),
            LogLevel::Error => "\x1b[31m[ERROR]\x1b[0m".to_string(),
            LogLevel::Fatal => "\x1b[35m[FATAL]\x1b[0m".to_string(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_log_buffer() {
        let mut buffer = LogBuffer::new(3);

        buffer.add(LogEntry::new(LogLevel::Info, "msg1".to_string()));
        buffer.add(LogEntry::new(LogLevel::Info, "msg2".to_string()));
        buffer.add(LogEntry::new(LogLevel::Info, "msg3".to_string()));

        assert_eq!(buffer.len(), 3);

        // Adding another should remove the oldest
        buffer.add(LogEntry::new(LogLevel::Info, "msg4".to_string()));
        assert_eq!(buffer.len(), 3);

        let all_logs = buffer.get_all_logs();
        assert_eq!(all_logs[0].message, "msg2");
        assert_eq!(all_logs[2].message, "msg4");
    }

    #[test]
    fn test_rate_limiter() {
        let mut limiter = RateLimiter::new(2);
        let log = LogEntry::new(LogLevel::Info, "test".to_string());

        assert!(limiter.should_allow(&log));
        assert!(limiter.should_allow(&log));
        assert!(!limiter.should_allow(&log)); // Should be rate limited
    }

    #[test]
    fn test_smart_filter() {
        let config = LogFilterConfig {
            levels: vec![LogLevel::Error, LogLevel::Warn],
            include_patterns: vec![],
            exclude_patterns: vec![],
            smart_filtering: false,
            errors_only: false,
            search_patterns: vec![],
        };

        let filter = SmartLogFilter::new(config);

        let error_log = LogEntry::new(LogLevel::Error, "error".to_string());
        let info_log = LogEntry::new(LogLevel::Info, "info".to_string());

        assert!(filter.should_display(&error_log));
        assert!(!filter.should_display(&info_log));
    }
}
