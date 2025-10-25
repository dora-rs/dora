/// Tests for Interactive Log Viewer (Issue #28)
#[cfg(test)]
mod log_viewer_tests {
    use dora_cli::tui::views::{
        LogLevel, LogEntry, LogFilter, LogViewerState, LogViewerView, View,
    };
    use dora_cli::tui::theme::ThemeConfig;

    // LogLevel tests
    #[test]
    fn test_log_level_all() {
        let levels = LogLevel::all();
        assert_eq!(levels.len(), 5);
        assert_eq!(levels[0], LogLevel::Error);
        assert_eq!(levels[1], LogLevel::Warn);
        assert_eq!(levels[2], LogLevel::Info);
        assert_eq!(levels[3], LogLevel::Debug);
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
        assert_eq!(LogLevel::Debug.short_name(), "DBG");
        assert_eq!(LogLevel::Trace.short_name(), "TRC");
    }

    #[test]
    fn test_log_level_ordering() {
        assert!(LogLevel::Error < LogLevel::Warn);
        assert!(LogLevel::Warn < LogLevel::Info);
        assert!(LogLevel::Info < LogLevel::Debug);
        assert!(LogLevel::Debug < LogLevel::Trace);
    }

    // LogEntry tests
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
    fn test_log_entry_matches_search() {
        let entry = LogEntry::new(
            1,
            LogLevel::Info,
            "camera-node".to_string(),
            "Processing frame 42".to_string(),
        );

        // Should match case-insensitive
        assert!(entry.matches_search("processing"));
        assert!(entry.matches_search("PROCESSING"));
        assert!(entry.matches_search("frame"));
        assert!(entry.matches_search("camera"));

        // Should not match non-existent strings
        assert!(!entry.matches_search("error"));
        assert!(!entry.matches_search("xyz"));

        // Empty query matches all
        assert!(entry.matches_search(""));
    }

    #[test]
    fn test_log_entry_format_line() {
        let entry = LogEntry::new(
            1,
            LogLevel::Error,
            "test-node".to_string(),
            "Test error message".to_string(),
        );

        let formatted = entry.format_line();
        assert!(formatted.contains("ERR"));
        assert!(formatted.contains("test-node"));
        assert!(formatted.contains("Test error message"));
    }

    // LogFilter tests
    #[test]
    fn test_log_filter_new() {
        let filter = LogFilter::new();

        assert_eq!(filter.enabled_count(), 5);
        assert!(filter.search_query.is_empty());
        assert!(filter.is_level_enabled(LogLevel::Error));
        assert!(filter.is_level_enabled(LogLevel::Info));
    }

    #[test]
    fn test_log_filter_toggle_level() {
        let mut filter = LogFilter::new();

        // Initially all enabled
        assert!(filter.is_level_enabled(LogLevel::Error));
        assert_eq!(filter.enabled_count(), 5);

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
        assert!(!filter.is_level_enabled(LogLevel::Error));

        filter.enable_all();
        assert_eq!(filter.enabled_count(), 5);
        assert!(filter.is_level_enabled(LogLevel::Error));
    }

    #[test]
    fn test_log_filter_search() {
        let mut filter = LogFilter::new();

        filter.set_search("test query".to_string());
        assert_eq!(filter.search_query, "test query");

        filter.clear_search();
        assert!(filter.search_query.is_empty());
    }

    #[test]
    fn test_log_filter_passes() {
        let mut filter = LogFilter::new();
        let entry = LogEntry::new(
            1,
            LogLevel::Error,
            "test-node".to_string(),
            "Connection failed".to_string(),
        );

        // Should pass with all levels enabled and no search
        assert!(filter.passes(&entry));

        // Should not pass when level is disabled
        filter.toggle_level(LogLevel::Error);
        assert!(!filter.passes(&entry));

        // Re-enable level
        filter.toggle_level(LogLevel::Error);
        assert!(filter.passes(&entry));

        // Should not pass when search doesn't match
        filter.set_search("success".to_string());
        assert!(!filter.passes(&entry));

        // Should pass when search matches
        filter.set_search("connection".to_string());
        assert!(filter.passes(&entry));
    }

    // LogViewerState tests
    #[test]
    fn test_log_viewer_state_new() {
        let state = LogViewerState::new();

        assert_eq!(state.buffer_count(), 0);
        assert_eq!(state.selected_index, 0);
        assert!(!state.paused);
        assert_eq!(state.total_received, 0);
        assert_eq!(state.max_buffer_size, 1000);
    }

    #[test]
    fn test_log_viewer_state_add_log() {
        let mut state = LogViewerState::new();
        let entry = LogEntry::new(
            1,
            LogLevel::Info,
            "test".to_string(),
            "Message".to_string(),
        );

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
    fn test_log_viewer_state_navigation_up_down() {
        let mut state = LogViewerState::new();

        // Add some logs
        for i in 0..5 {
            let entry = LogEntry::new(i, LogLevel::Info, "test".to_string(), "Message".to_string());
            state.add_log(entry);
        }

        state.jump_to_start();
        assert_eq!(state.selected_index, 0);

        state.move_down();
        assert_eq!(state.selected_index, 1);

        state.move_down();
        assert_eq!(state.selected_index, 2);

        state.move_up();
        assert_eq!(state.selected_index, 1);

        state.move_up();
        assert_eq!(state.selected_index, 0);

        // Should not go below 0
        state.move_up();
        assert_eq!(state.selected_index, 0);
    }

    #[test]
    fn test_log_viewer_state_page_navigation() {
        let mut state = LogViewerState::new();

        // Add 30 logs
        for i in 0..30 {
            let entry = LogEntry::new(i, LogLevel::Info, "test".to_string(), "Message".to_string());
            state.add_log(entry);
        }

        state.jump_to_start();
        assert_eq!(state.selected_index, 0);

        state.page_down();
        assert_eq!(state.selected_index, 10);

        state.page_down();
        assert_eq!(state.selected_index, 20);

        state.page_up();
        assert_eq!(state.selected_index, 10);

        state.page_up();
        assert_eq!(state.selected_index, 0);
    }

    #[test]
    fn test_log_viewer_state_jump_to_start_end() {
        let mut state = LogViewerState::new();

        // Add 10 logs
        for i in 0..10 {
            let entry = LogEntry::new(i, LogLevel::Info, "test".to_string(), "Message".to_string());
            state.add_log(entry);
        }

        state.jump_to_end();
        assert_eq!(state.selected_index, 9);
        assert!(!state.paused); // Should resume when jumping to end

        state.jump_to_start();
        assert_eq!(state.selected_index, 0);
        assert!(state.paused); // Should pause when jumping to start
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
        state.add_log(LogEntry::new(0, LogLevel::Error, "test".to_string(), "Error".to_string()));
        state.add_log(LogEntry::new(1, LogLevel::Warn, "test".to_string(), "Warning".to_string()));
        state.add_log(LogEntry::new(2, LogLevel::Info, "test".to_string(), "Info".to_string()));
        state.add_log(LogEntry::new(3, LogLevel::Debug, "test".to_string(), "Debug".to_string()));

        assert_eq!(state.filtered_count(), 4);

        // Disable warnings
        state.filter.toggle_level(LogLevel::Warn);
        assert_eq!(state.filtered_count(), 3);

        // Disable debug
        state.filter.toggle_level(LogLevel::Debug);
        assert_eq!(state.filtered_count(), 2);

        // Set search that matches only error
        state.filter.set_search("Error".to_string());
        assert_eq!(state.filtered_count(), 1);
    }

    #[test]
    fn test_log_viewer_state_get_selected_log() {
        let mut state = LogViewerState::new();

        state.add_log(LogEntry::new(0, LogLevel::Error, "test".to_string(), "E1".to_string()));
        state.add_log(LogEntry::new(1, LogLevel::Warn, "test".to_string(), "W1".to_string()));
        state.add_log(LogEntry::new(2, LogLevel::Info, "test".to_string(), "I1".to_string()));

        state.selected_index = 1;
        let selected = state.get_selected_log().unwrap();
        assert_eq!(selected.level, LogLevel::Warn);
        assert_eq!(selected.message, "W1");
    }

    #[test]
    fn test_log_stats_calculation() {
        let mut state = LogViewerState::new();

        state.add_log(LogEntry::new(0, LogLevel::Error, "test".to_string(), "E1".to_string()));
        state.add_log(LogEntry::new(1, LogLevel::Error, "test".to_string(), "E2".to_string()));
        state.add_log(LogEntry::new(2, LogLevel::Warn, "test".to_string(), "W1".to_string()));
        state.add_log(LogEntry::new(3, LogLevel::Info, "test".to_string(), "I1".to_string()));
        state.add_log(LogEntry::new(4, LogLevel::Debug, "test".to_string(), "D1".to_string()));
        state.add_log(LogEntry::new(5, LogLevel::Trace, "test".to_string(), "T1".to_string()));

        let stats = state.stats();

        assert_eq!(stats.total, 6);
        assert_eq!(stats.error_count, 2);
        assert_eq!(stats.warn_count, 1);
        assert_eq!(stats.info_count, 1);
        assert_eq!(stats.debug_count, 1);
        assert_eq!(stats.trace_count, 1);
    }

    #[test]
    fn test_log_stats_percentages() {
        let mut state = LogViewerState::new();

        // Add 4 errors out of 10 total (40%)
        for i in 0..4 {
            state.add_log(LogEntry::new(i, LogLevel::Error, "test".to_string(), format!("E{}", i)));
        }
        // Add 2 warnings out of 10 total (20%)
        for i in 4..6 {
            state.add_log(LogEntry::new(i, LogLevel::Warn, "test".to_string(), format!("W{}", i)));
        }
        // Add 4 info logs
        for i in 6..10 {
            state.add_log(LogEntry::new(i, LogLevel::Info, "test".to_string(), format!("I{}", i)));
        }

        let stats = state.stats();

        assert_eq!(stats.error_percentage(), 40.0);
        assert_eq!(stats.warn_percentage(), 20.0);
    }

    #[test]
    fn test_log_stats_empty() {
        let state = LogViewerState::new();
        let stats = state.stats();

        assert_eq!(stats.total, 0);
        assert_eq!(stats.error_count, 0);
        assert_eq!(stats.error_percentage(), 0.0);
    }

    // LogViewerView tests
    #[test]
    fn test_log_viewer_view_creation() {
        let theme = ThemeConfig::default();
        let viewer = LogViewerView::new("", &theme);

        assert_eq!(viewer.state.buffer_count(), 0);
    }

    #[test]
    fn test_log_viewer_view_title() {
        let theme = ThemeConfig::default();
        let viewer = LogViewerView::new("test-source", &theme);

        assert_eq!(viewer.title(), "Logs: test-source");
    }

    #[test]
    fn test_auto_scroll_behavior() {
        let mut state = LogViewerState::new();

        // Add logs while not paused - should auto-scroll to bottom
        for i in 0..5 {
            state.add_log(LogEntry::new(i, LogLevel::Info, "test".to_string(), "Message".to_string()));
        }

        assert_eq!(state.selected_index, 4); // Should be at last log

        // Pause and navigate
        state.paused = true;
        state.selected_index = 2;

        // Add more logs while paused - should not auto-scroll
        state.add_log(LogEntry::new(5, LogLevel::Info, "test".to_string(), "Message".to_string()));

        assert_eq!(state.selected_index, 2); // Should stay at index 2
    }

    #[test]
    fn test_complex_filtering_scenario() {
        let mut state = LogViewerState::new();

        // Add diverse logs
        state.add_log(LogEntry::new(0, LogLevel::Error, "camera".to_string(), "Failed to connect".to_string()));
        state.add_log(LogEntry::new(1, LogLevel::Warn, "camera".to_string(), "Low framerate".to_string()));
        state.add_log(LogEntry::new(2, LogLevel::Info, "camera".to_string(), "Connected successfully".to_string()));
        state.add_log(LogEntry::new(3, LogLevel::Error, "sensor".to_string(), "Timeout error".to_string()));
        state.add_log(LogEntry::new(4, LogLevel::Info, "sensor".to_string(), "Reading data".to_string()));

        // Filter to only errors
        state.filter.disable_all();
        state.filter.toggle_level(LogLevel::Error);
        assert_eq!(state.filtered_count(), 2);

        // Further filter by source containing "camera"
        state.filter.set_search("camera".to_string());
        assert_eq!(state.filtered_count(), 1);

        let filtered_logs = state.get_filtered_logs();
        assert_eq!(filtered_logs[0].message, "Failed to connect");
    }
}
