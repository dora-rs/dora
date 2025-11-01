/// Types and state management for Interactive Log Viewer (Issue #28 - Phase 1)
use std::collections::VecDeque;
use std::time::Instant;

/// Log level enumeration
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum LogLevel {
    Error,
    Warn,
    Info,
    Debug,
    Trace,
}

impl LogLevel {
    /// Get all log levels in priority order
    pub fn all() -> Vec<LogLevel> {
        vec![
            LogLevel::Error,
            LogLevel::Warn,
            LogLevel::Info,
            LogLevel::Debug,
            LogLevel::Trace,
        ]
    }

    /// Get the display name for this log level
    pub fn name(&self) -> &str {
        match self {
            LogLevel::Error => "ERROR",
            LogLevel::Warn => "WARN",
            LogLevel::Info => "INFO",
            LogLevel::Debug => "DEBUG",
            LogLevel::Trace => "TRACE",
        }
    }

    /// Get the short name for compact display
    pub fn short_name(&self) -> &str {
        match self {
            LogLevel::Error => "ERR",
            LogLevel::Warn => "WRN",
            LogLevel::Info => "INF",
            LogLevel::Debug => "DBG",
            LogLevel::Trace => "TRC",
        }
    }

    /// Get color indicator for this log level
    pub fn color_code(&self) -> &str {
        match self {
            LogLevel::Error => "red",
            LogLevel::Warn => "yellow",
            LogLevel::Info => "blue",
            LogLevel::Debug => "gray",
            LogLevel::Trace => "dark_gray",
        }
    }
}

/// Individual log entry
#[derive(Debug, Clone, PartialEq)]
pub struct LogEntry {
    pub id: usize,
    pub timestamp: Instant,
    pub level: LogLevel,
    pub source: String,
    pub message: String,
}

impl LogEntry {
    /// Create a new log entry
    pub fn new(id: usize, level: LogLevel, source: String, message: String) -> Self {
        Self {
            id,
            timestamp: Instant::now(),
            level,
            source,
            message,
        }
    }

    /// Get formatted timestamp string (relative time in seconds)
    pub fn timestamp_str(&self) -> String {
        let elapsed = self.timestamp.elapsed();
        let secs = elapsed.as_secs();
        let millis = elapsed.subsec_millis();
        format!("{secs}.{millis:03}s ago")
    }

    /// Check if this entry matches the search query (case-insensitive)
    pub fn matches_search(&self, query: &str) -> bool {
        if query.is_empty() {
            return true;
        }
        let query_lower = query.to_lowercase();
        self.message.to_lowercase().contains(&query_lower)
            || self.source.to_lowercase().contains(&query_lower)
    }

    /// Format as a single-line string for display
    pub fn format_line(&self) -> String {
        format!(
            "[{}] {:5} {} - {}",
            self.timestamp_str(),
            self.level.short_name(),
            self.source,
            self.message
        )
    }
}

/// Log filtering configuration
#[derive(Debug, Clone, PartialEq)]
pub struct LogFilter {
    pub enabled_levels: Vec<LogLevel>,
    pub search_query: String,
}

impl LogFilter {
    /// Create a new filter with all levels enabled
    pub fn new() -> Self {
        Self {
            enabled_levels: LogLevel::all(),
            search_query: String::new(),
        }
    }

    /// Check if a log level is enabled
    pub fn is_level_enabled(&self, level: LogLevel) -> bool {
        self.enabled_levels.contains(&level)
    }

    /// Toggle a log level on/off
    pub fn toggle_level(&mut self, level: LogLevel) {
        if let Some(pos) = self.enabled_levels.iter().position(|&l| l == level) {
            self.enabled_levels.remove(pos);
        } else {
            self.enabled_levels.push(level);
            self.enabled_levels.sort();
        }
    }

    /// Enable all log levels
    pub fn enable_all(&mut self) {
        self.enabled_levels = LogLevel::all();
    }

    /// Disable all log levels
    pub fn disable_all(&mut self) {
        self.enabled_levels.clear();
    }

    /// Set search query
    pub fn set_search(&mut self, query: String) {
        self.search_query = query;
    }

    /// Clear search query
    pub fn clear_search(&mut self) {
        self.search_query.clear();
    }

    /// Check if an entry passes this filter
    pub fn passes(&self, entry: &LogEntry) -> bool {
        // Check level filter
        if !self.is_level_enabled(entry.level) {
            return false;
        }

        // Check search filter
        entry.matches_search(&self.search_query)
    }

    /// Get count of enabled levels
    pub fn enabled_count(&self) -> usize {
        self.enabled_levels.len()
    }
}

impl Default for LogFilter {
    fn default() -> Self {
        Self::new()
    }
}

/// Log viewer state management
#[derive(Debug)]
pub struct LogViewerState {
    pub log_buffer: VecDeque<LogEntry>,
    pub max_buffer_size: usize,
    pub selected_index: usize,
    pub scroll_offset: usize,
    pub paused: bool,
    pub filter: LogFilter,
    pub total_received: usize,
    pub last_refresh: Instant,
}

impl LogViewerState {
    /// Create a new log viewer state
    pub fn new() -> Self {
        Self {
            log_buffer: VecDeque::new(),
            max_buffer_size: 1000,
            selected_index: 0,
            scroll_offset: 0,
            paused: false,
            filter: LogFilter::new(),
            total_received: 0,
            last_refresh: Instant::now(),
        }
    }

    /// Add a log entry to the buffer
    pub fn add_log(&mut self, entry: LogEntry) {
        self.log_buffer.push_back(entry);
        self.total_received += 1;

        // Trim buffer if needed
        while self.log_buffer.len() > self.max_buffer_size {
            self.log_buffer.pop_front();
            // Adjust selected index if needed
            if self.selected_index > 0 {
                self.selected_index = self.selected_index.saturating_sub(1);
            }
            if self.scroll_offset > 0 {
                self.scroll_offset = self.scroll_offset.saturating_sub(1);
            }
        }

        // Auto-scroll to bottom if not paused
        if !self.paused {
            self.scroll_to_bottom();
        }
    }

    /// Get filtered logs (those that pass the current filter)
    pub fn get_filtered_logs(&self) -> Vec<&LogEntry> {
        self.log_buffer
            .iter()
            .filter(|entry| self.filter.passes(entry))
            .collect()
    }

    /// Get the currently selected log entry (from filtered view)
    pub fn get_selected_log(&self) -> Option<&LogEntry> {
        let filtered = self.get_filtered_logs();
        filtered.get(self.selected_index).copied()
    }

    /// Move selection up
    pub fn move_up(&mut self) {
        if self.selected_index > 0 {
            self.selected_index -= 1;
            self.paused = true;
        }
    }

    /// Move selection down
    pub fn move_down(&mut self) {
        let filtered_count = self.get_filtered_logs().len();
        if self.selected_index + 1 < filtered_count {
            self.selected_index += 1;
        }
    }

    /// Page up (move 10 entries up)
    pub fn page_up(&mut self) {
        self.selected_index = self.selected_index.saturating_sub(10);
        self.paused = true;
    }

    /// Page down (move 10 entries down)
    pub fn page_down(&mut self) {
        let filtered_count = self.get_filtered_logs().len();
        self.selected_index = (self.selected_index + 10).min(filtered_count.saturating_sub(1));
    }

    /// Jump to start
    pub fn jump_to_start(&mut self) {
        self.selected_index = 0;
        self.scroll_offset = 0;
        self.paused = true;
    }

    /// Jump to end
    pub fn jump_to_end(&mut self) {
        let filtered_count = self.get_filtered_logs().len();
        self.selected_index = filtered_count.saturating_sub(1);
        self.paused = false; // Resume auto-scroll when jumping to end
    }

    /// Scroll to bottom (for auto-scroll)
    pub fn scroll_to_bottom(&mut self) {
        let filtered_count = self.get_filtered_logs().len();
        self.selected_index = filtered_count.saturating_sub(1);
    }

    /// Toggle pause state
    pub fn toggle_pause(&mut self) {
        self.paused = !self.paused;
        if !self.paused {
            self.scroll_to_bottom();
        }
    }

    /// Clear all logs
    pub fn clear(&mut self) {
        self.log_buffer.clear();
        self.selected_index = 0;
        self.scroll_offset = 0;
    }

    /// Mark as refreshed
    pub fn mark_refreshed(&mut self) {
        self.last_refresh = Instant::now();
    }

    /// Get count of filtered logs
    pub fn filtered_count(&self) -> usize {
        self.get_filtered_logs().len()
    }

    /// Get total logs in buffer
    pub fn buffer_count(&self) -> usize {
        self.log_buffer.len()
    }

    /// Calculate statistics
    pub fn stats(&self) -> LogStats {
        let mut error_count = 0;
        let mut warn_count = 0;
        let mut info_count = 0;
        let mut debug_count = 0;
        let mut trace_count = 0;

        for entry in &self.log_buffer {
            match entry.level {
                LogLevel::Error => error_count += 1,
                LogLevel::Warn => warn_count += 1,
                LogLevel::Info => info_count += 1,
                LogLevel::Debug => debug_count += 1,
                LogLevel::Trace => trace_count += 1,
            }
        }

        LogStats {
            total: self.log_buffer.len(),
            filtered: self.filtered_count(),
            error_count,
            warn_count,
            info_count,
            debug_count,
            trace_count,
        }
    }
}

impl Default for LogViewerState {
    fn default() -> Self {
        Self::new()
    }
}

/// Statistics about logs
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LogStats {
    pub total: usize,
    pub filtered: usize,
    pub error_count: usize,
    pub warn_count: usize,
    pub info_count: usize,
    pub debug_count: usize,
    pub trace_count: usize,
}

impl LogStats {
    /// Get percentage of errors
    pub fn error_percentage(&self) -> f64 {
        if self.total == 0 {
            0.0
        } else {
            (self.error_count as f64 / self.total as f64) * 100.0
        }
    }

    /// Get percentage of warnings
    pub fn warn_percentage(&self) -> f64 {
        if self.total == 0 {
            0.0
        } else {
            (self.warn_count as f64 / self.total as f64) * 100.0
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_log_level_all() {
        let levels = LogLevel::all();
        assert_eq!(levels.len(), 5);
        assert_eq!(levels[0], LogLevel::Error);
        assert_eq!(levels[4], LogLevel::Trace);
    }

    #[test]
    fn test_log_level_names() {
        assert_eq!(LogLevel::Error.name(), "ERROR");
        assert_eq!(LogLevel::Warn.name(), "WARN");
        assert_eq!(LogLevel::Info.name(), "INFO");
        assert_eq!(LogLevel::Debug.name(), "DEBUG");
        assert_eq!(LogLevel::Trace.name(), "TRACE");
    }

    #[test]
    fn test_log_level_short_names() {
        assert_eq!(LogLevel::Error.short_name(), "ERR");
        assert_eq!(LogLevel::Warn.short_name(), "WRN");
        assert_eq!(LogLevel::Info.short_name(), "INF");
    }

    #[test]
    fn test_log_entry_creation() {
        let entry = LogEntry::new(
            1,
            LogLevel::Info,
            "test-node".to_string(),
            "Test message".to_string(),
        );
        assert_eq!(entry.id, 1);
        assert_eq!(entry.level, LogLevel::Info);
        assert_eq!(entry.source, "test-node");
        assert_eq!(entry.message, "Test message");
    }

    #[test]
    fn test_log_entry_search() {
        let entry = LogEntry::new(
            1,
            LogLevel::Info,
            "test-node".to_string(),
            "Connection established".to_string(),
        );

        assert!(entry.matches_search("connection"));
        assert!(entry.matches_search("CONNECTION")); // case-insensitive
        assert!(entry.matches_search("test-node"));
        assert!(!entry.matches_search("error"));
        assert!(entry.matches_search("")); // empty query matches all
    }

    #[test]
    fn test_log_filter_new() {
        let filter = LogFilter::new();
        assert_eq!(filter.enabled_levels.len(), 5);
        assert!(filter.search_query.is_empty());
    }

    #[test]
    fn test_log_filter_toggle_level() {
        let mut filter = LogFilter::new();

        // Initially all enabled
        assert!(filter.is_level_enabled(LogLevel::Error));

        // Toggle off
        filter.toggle_level(LogLevel::Error);
        assert!(!filter.is_level_enabled(LogLevel::Error));
        assert_eq!(filter.enabled_count(), 4);

        // Toggle back on
        filter.toggle_level(LogLevel::Error);
        assert!(filter.is_level_enabled(LogLevel::Error));
        assert_eq!(filter.enabled_count(), 5);
    }

    #[test]
    fn test_log_filter_enable_disable_all() {
        let mut filter = LogFilter::new();

        filter.disable_all();
        assert_eq!(filter.enabled_count(), 0);

        filter.enable_all();
        assert_eq!(filter.enabled_count(), 5);
    }

    #[test]
    fn test_log_filter_passes() {
        let mut filter = LogFilter::new();
        let entry = LogEntry::new(
            1,
            LogLevel::Error,
            "test".to_string(),
            "Error occurred".to_string(),
        );

        // Should pass with all levels enabled
        assert!(filter.passes(&entry));

        // Disable error level
        filter.toggle_level(LogLevel::Error);
        assert!(!filter.passes(&entry));

        // Enable back, but set search that doesn't match
        filter.toggle_level(LogLevel::Error);
        filter.set_search("connection".to_string());
        assert!(!filter.passes(&entry));

        // Set search that matches
        filter.set_search("error".to_string());
        assert!(filter.passes(&entry));
    }

    #[test]
    fn test_log_viewer_state_new() {
        let state = LogViewerState::new();
        assert_eq!(state.buffer_count(), 0);
        assert_eq!(state.selected_index, 0);
        assert!(!state.paused);
        assert_eq!(state.total_received, 0);
    }

    #[test]
    fn test_log_viewer_state_add_log() {
        let mut state = LogViewerState::new();
        let entry = LogEntry::new(1, LogLevel::Info, "test".to_string(), "Message".to_string());

        state.add_log(entry);
        assert_eq!(state.buffer_count(), 1);
        assert_eq!(state.total_received, 1);
    }

    #[test]
    fn test_log_viewer_state_buffer_limit() {
        let mut state = LogViewerState::new();
        state.max_buffer_size = 10;

        // Add 15 logs
        for i in 0..15 {
            let entry = LogEntry::new(
                i,
                LogLevel::Info,
                "test".to_string(),
                format!("Message {}", i),
            );
            state.add_log(entry);
        }

        // Should only keep last 10
        assert_eq!(state.buffer_count(), 10);
        assert_eq!(state.total_received, 15);
    }

    #[test]
    fn test_log_viewer_state_navigation() {
        let mut state = LogViewerState::new();

        // Add some logs
        for i in 0..5 {
            let entry = LogEntry::new(
                i,
                LogLevel::Info,
                "test".to_string(),
                format!("Message {}", i),
            );
            state.add_log(entry);
        }

        state.jump_to_start();
        assert_eq!(state.selected_index, 0);
        assert!(state.paused);

        state.move_down();
        assert_eq!(state.selected_index, 1);

        state.move_up();
        assert_eq!(state.selected_index, 0);

        state.jump_to_end();
        assert_eq!(state.selected_index, 4);
        assert!(!state.paused); // Should resume
    }

    #[test]
    fn test_log_viewer_state_pause_toggle() {
        let mut state = LogViewerState::new();
        assert!(!state.paused);

        state.toggle_pause();
        assert!(state.paused);

        state.toggle_pause();
        assert!(!state.paused);
    }

    #[test]
    fn test_log_viewer_state_clear() {
        let mut state = LogViewerState::new();

        for i in 0..5 {
            let entry = LogEntry::new(i, LogLevel::Info, "test".to_string(), "Message".to_string());
            state.add_log(entry);
        }

        state.selected_index = 2;
        state.clear();

        assert_eq!(state.buffer_count(), 0);
        assert_eq!(state.selected_index, 0);
    }

    #[test]
    fn test_log_viewer_state_filtering() {
        let mut state = LogViewerState::new();

        // Add different level logs
        state.add_log(LogEntry::new(
            0,
            LogLevel::Error,
            "test".to_string(),
            "Error".to_string(),
        ));
        state.add_log(LogEntry::new(
            1,
            LogLevel::Warn,
            "test".to_string(),
            "Warning".to_string(),
        ));
        state.add_log(LogEntry::new(
            2,
            LogLevel::Info,
            "test".to_string(),
            "Info".to_string(),
        ));

        assert_eq!(state.filtered_count(), 3);

        // Disable warnings
        state.filter.toggle_level(LogLevel::Warn);
        assert_eq!(state.filtered_count(), 2);

        // Set search
        state.filter.set_search("Error".to_string());
        assert_eq!(state.filtered_count(), 1);
    }

    #[test]
    fn test_log_stats() {
        let mut state = LogViewerState::new();

        state.add_log(LogEntry::new(
            0,
            LogLevel::Error,
            "test".to_string(),
            "E1".to_string(),
        ));
        state.add_log(LogEntry::new(
            1,
            LogLevel::Error,
            "test".to_string(),
            "E2".to_string(),
        ));
        state.add_log(LogEntry::new(
            2,
            LogLevel::Warn,
            "test".to_string(),
            "W1".to_string(),
        ));
        state.add_log(LogEntry::new(
            3,
            LogLevel::Info,
            "test".to_string(),
            "I1".to_string(),
        ));

        let stats = state.stats();
        assert_eq!(stats.total, 4);
        assert_eq!(stats.error_count, 2);
        assert_eq!(stats.warn_count, 1);
        assert_eq!(stats.info_count, 1);
        assert_eq!(stats.error_percentage(), 50.0);
        assert_eq!(stats.warn_percentage(), 25.0);
    }
}
