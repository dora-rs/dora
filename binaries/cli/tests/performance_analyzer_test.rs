/// Tests for PerformanceAnalyzerView (Issue #26)
#[cfg(test)]
mod performance_analyzer_tests {
    use dora_cli::tui::theme::ThemeConfig;
    use dora_cli::tui::views::{
        AlertSeverity, AnalyzerTab, MetricType, PerformanceAlert, PerformanceAnalyzerState,
        PerformanceAnalyzerView, TimeRange,
    };

    #[test]
    fn test_analyzer_tab_all_returns_three_tabs() {
        let tabs = AnalyzerTab::all();
        assert_eq!(tabs.len(), 3);
        assert_eq!(tabs[0], AnalyzerTab::Overview);
        assert_eq!(tabs[1], AnalyzerTab::Metrics);
        assert_eq!(tabs[2], AnalyzerTab::Alerts);
    }

    #[test]
    fn test_analyzer_tab_names() {
        assert_eq!(AnalyzerTab::Overview.name(), "Overview");
        assert_eq!(AnalyzerTab::Metrics.name(), "Metrics");
        assert_eq!(AnalyzerTab::Alerts.name(), "Alerts");
    }

    #[test]
    fn test_analyzer_tab_shortcuts() {
        assert_eq!(AnalyzerTab::Overview.shortcut(), "1");
        assert_eq!(AnalyzerTab::Metrics.shortcut(), "2");
        assert_eq!(AnalyzerTab::Alerts.shortcut(), "3");
    }

    #[test]
    fn test_analyzer_tab_navigation() {
        let mut tab = AnalyzerTab::Overview;

        tab = tab.next();
        assert_eq!(tab, AnalyzerTab::Metrics);

        tab = tab.next();
        assert_eq!(tab, AnalyzerTab::Alerts);

        // Test wrapping
        tab = tab.next();
        assert_eq!(tab, AnalyzerTab::Overview);
    }

    #[test]
    fn test_analyzer_tab_prev_navigation() {
        let mut tab = AnalyzerTab::Overview;

        tab = tab.prev();
        assert_eq!(tab, AnalyzerTab::Alerts);

        tab = tab.prev();
        assert_eq!(tab, AnalyzerTab::Metrics);

        tab = tab.prev();
        assert_eq!(tab, AnalyzerTab::Overview);
    }

    #[test]
    fn test_time_range_all() {
        let ranges = TimeRange::all();
        assert_eq!(ranges.len(), 3);
        assert_eq!(ranges[0], TimeRange::LastHour);
        assert_eq!(ranges[1], TimeRange::Last6Hours);
        assert_eq!(ranges[2], TimeRange::Last24Hours);
    }

    #[test]
    fn test_time_range_names() {
        assert_eq!(TimeRange::LastHour.name(), "Last Hour");
        assert_eq!(TimeRange::Last6Hours.name(), "Last 6 Hours");
        assert_eq!(TimeRange::Last24Hours.name(), "Last 24 Hours");
    }

    #[test]
    fn test_time_range_durations() {
        assert_eq!(TimeRange::LastHour.duration_secs(), 3600);
        assert_eq!(TimeRange::Last6Hours.duration_secs(), 21600);
        assert_eq!(TimeRange::Last24Hours.duration_secs(), 86400);
    }

    #[test]
    fn test_time_range_data_points() {
        assert_eq!(TimeRange::LastHour.data_points(), 60);
        assert_eq!(TimeRange::Last6Hours.data_points(), 72);
        assert_eq!(TimeRange::Last24Hours.data_points(), 96);
    }

    #[test]
    fn test_time_range_cycling() {
        let mut range = TimeRange::LastHour;

        range = range.next();
        assert_eq!(range, TimeRange::Last6Hours);

        range = range.next();
        assert_eq!(range, TimeRange::Last24Hours);

        range = range.next();
        assert_eq!(range, TimeRange::LastHour);
    }

    #[test]
    fn test_metric_type_all() {
        let metrics = MetricType::all();
        assert_eq!(metrics.len(), 4);
        assert_eq!(metrics[0], MetricType::Cpu);
        assert_eq!(metrics[1], MetricType::Memory);
        assert_eq!(metrics[2], MetricType::MessageRate);
        assert_eq!(metrics[3], MetricType::Latency);
    }

    #[test]
    fn test_metric_type_names() {
        assert_eq!(MetricType::Cpu.name(), "CPU");
        assert_eq!(MetricType::Memory.name(), "Memory");
        assert_eq!(MetricType::MessageRate.name(), "Message Rate");
        assert_eq!(MetricType::Latency.name(), "Latency");
    }

    #[test]
    fn test_metric_type_units() {
        assert_eq!(MetricType::Cpu.unit(), "%");
        assert_eq!(MetricType::Memory.unit(), "MB");
        assert_eq!(MetricType::MessageRate.unit(), "msg/s");
        assert_eq!(MetricType::Latency.unit(), "ms");
    }

    #[test]
    fn test_metric_type_thresholds() {
        // Warning thresholds
        assert_eq!(MetricType::Cpu.warning_threshold(), 70.0);
        assert_eq!(MetricType::Memory.warning_threshold(), 80.0);
        assert_eq!(MetricType::MessageRate.warning_threshold(), 1000.0);
        assert_eq!(MetricType::Latency.warning_threshold(), 100.0);

        // Critical thresholds
        assert_eq!(MetricType::Cpu.critical_threshold(), 90.0);
        assert_eq!(MetricType::Memory.critical_threshold(), 95.0);
        assert_eq!(MetricType::MessageRate.critical_threshold(), 5000.0);
        assert_eq!(MetricType::Latency.critical_threshold(), 500.0);

        // Verify critical is higher than warning
        for metric in MetricType::all() {
            assert!(metric.critical_threshold() > metric.warning_threshold());
        }
    }

    #[test]
    fn test_alert_severity_names() {
        assert_eq!(AlertSeverity::Warning.name(), "Warning");
        assert_eq!(AlertSeverity::Critical.name(), "Critical");
    }

    #[test]
    fn test_alert_severity_symbols() {
        assert_eq!(AlertSeverity::Warning.symbol(), "⚠");
        assert_eq!(AlertSeverity::Critical.symbol(), "🔴");
    }

    #[test]
    fn test_performance_alert_creation() {
        let alert = PerformanceAlert::new(MetricType::Cpu, AlertSeverity::Warning, 75.0, 70.0);

        assert_eq!(alert.metric_type, MetricType::Cpu);
        assert_eq!(alert.severity, AlertSeverity::Warning);
        assert_eq!(alert.current_value, 75.0);
        assert_eq!(alert.threshold, 70.0);
        assert!(alert.message.contains("CPU"));
        assert!(alert.message.contains("75.0"));
        assert!(alert.message.contains("70.0"));
    }

    #[test]
    fn test_performance_analyzer_state_new() {
        let state = PerformanceAnalyzerState::new();

        assert_eq!(state.active_tab, AnalyzerTab::Overview);
        assert_eq!(state.time_range, TimeRange::LastHour);
        assert_eq!(state.selected_metric, MetricType::Cpu);
        assert!(state.alerts.is_empty());
        assert_eq!(state.selected_alert, 0);
    }

    #[test]
    fn test_performance_analyzer_state_default() {
        let state = PerformanceAnalyzerState::default();

        assert_eq!(state.active_tab, AnalyzerTab::Overview);
        assert_eq!(state.time_range, TimeRange::LastHour);
    }

    #[test]
    fn test_performance_analyzer_state_tab_navigation() {
        let mut state = PerformanceAnalyzerState::new();

        assert_eq!(state.active_tab, AnalyzerTab::Overview);

        state.next_tab();
        assert_eq!(state.active_tab, AnalyzerTab::Metrics);

        state.next_tab();
        assert_eq!(state.active_tab, AnalyzerTab::Alerts);

        state.prev_tab();
        assert_eq!(state.active_tab, AnalyzerTab::Metrics);

        state.prev_tab();
        assert_eq!(state.active_tab, AnalyzerTab::Overview);
    }

    #[test]
    fn test_performance_analyzer_state_switch_tab() {
        let mut state = PerformanceAnalyzerState::new();

        state.switch_tab(AnalyzerTab::Metrics);
        assert_eq!(state.active_tab, AnalyzerTab::Metrics);

        state.switch_tab(AnalyzerTab::Alerts);
        assert_eq!(state.active_tab, AnalyzerTab::Alerts);
    }

    #[test]
    fn test_performance_analyzer_state_time_range_cycling() {
        let mut state = PerformanceAnalyzerState::new();

        assert_eq!(state.time_range, TimeRange::LastHour);

        state.next_time_range();
        assert_eq!(state.time_range, TimeRange::Last6Hours);

        state.next_time_range();
        assert_eq!(state.time_range, TimeRange::Last24Hours);

        state.next_time_range();
        assert_eq!(state.time_range, TimeRange::LastHour);
    }

    #[test]
    fn test_performance_analyzer_state_metric_selection() {
        let mut state = PerformanceAnalyzerState::new();

        assert_eq!(state.selected_metric, MetricType::Cpu);

        state.next_metric();
        assert_eq!(state.selected_metric, MetricType::Memory);

        state.next_metric();
        assert_eq!(state.selected_metric, MetricType::MessageRate);

        state.next_metric();
        assert_eq!(state.selected_metric, MetricType::Latency);

        // Should wrap around
        state.next_metric();
        assert_eq!(state.selected_metric, MetricType::Cpu);

        // Test previous
        state.prev_metric();
        assert_eq!(state.selected_metric, MetricType::Latency);
    }

    #[test]
    fn test_performance_analyzer_state_alert_navigation() {
        let mut state = PerformanceAnalyzerState::new();

        // Add some alerts
        let alert1 = PerformanceAlert::new(MetricType::Cpu, AlertSeverity::Warning, 75.0, 70.0);
        let alert2 = PerformanceAlert::new(MetricType::Memory, AlertSeverity::Critical, 96.0, 95.0);
        let alert3 =
            PerformanceAlert::new(MetricType::Latency, AlertSeverity::Warning, 150.0, 100.0);

        state.add_alert(alert1);
        state.add_alert(alert2);
        state.add_alert(alert3);

        assert_eq!(state.alerts.len(), 3);
        assert_eq!(state.selected_alert, 0);

        state.next_alert();
        assert_eq!(state.selected_alert, 1);

        state.next_alert();
        assert_eq!(state.selected_alert, 2);

        // Should wrap around
        state.next_alert();
        assert_eq!(state.selected_alert, 0);

        // Test previous
        state.prev_alert();
        assert_eq!(state.selected_alert, 2);
    }

    #[test]
    fn test_performance_analyzer_state_add_alert() {
        let mut state = PerformanceAnalyzerState::new();

        assert!(state.alerts.is_empty());

        let alert = PerformanceAlert::new(MetricType::Cpu, AlertSeverity::Warning, 75.0, 70.0);
        state.add_alert(alert);

        assert_eq!(state.alerts.len(), 1);
    }

    #[test]
    fn test_performance_analyzer_state_clear_alerts() {
        let mut state = PerformanceAnalyzerState::new();

        let alert1 = PerformanceAlert::new(MetricType::Cpu, AlertSeverity::Warning, 75.0, 70.0);
        let alert2 = PerformanceAlert::new(MetricType::Memory, AlertSeverity::Critical, 96.0, 95.0);

        state.add_alert(alert1);
        state.add_alert(alert2);

        assert_eq!(state.alerts.len(), 2);

        state.next_alert();
        assert_eq!(state.selected_alert, 1);

        state.clear_alerts();

        assert!(state.alerts.is_empty());
        assert_eq!(state.selected_alert, 0);
    }

    #[test]
    fn test_performance_analyzer_state_mark_refreshed() {
        let mut state = PerformanceAnalyzerState::new();

        let initial_time = state.last_refresh;

        std::thread::sleep(std::time::Duration::from_millis(10));

        state.mark_refreshed();

        assert!(state.last_refresh > initial_time);
    }

    #[test]
    fn test_performance_analyzer_view_creation() {
        let theme = ThemeConfig::default();
        let view = PerformanceAnalyzerView::new(&theme);

        assert_eq!(view.state.active_tab, AnalyzerTab::Overview);
        assert_eq!(view.state.time_range, TimeRange::LastHour);
        assert_eq!(view.state.selected_metric, MetricType::Cpu);
    }

    #[test]
    fn test_performance_analyzer_state_complex_workflow() {
        let mut state = PerformanceAnalyzerState::new();

        // Simulate a user workflow
        state.switch_tab(AnalyzerTab::Metrics);
        state.next_metric();
        state.next_metric();
        state.next_time_range();

        // Add alerts
        let alert1 = PerformanceAlert::new(MetricType::Cpu, AlertSeverity::Warning, 75.0, 70.0);
        let alert2 = PerformanceAlert::new(MetricType::Memory, AlertSeverity::Critical, 96.0, 95.0);
        state.add_alert(alert1);
        state.add_alert(alert2);

        // Switch to alerts tab
        state.switch_tab(AnalyzerTab::Alerts);
        state.next_alert();

        // Verify state
        assert_eq!(state.active_tab, AnalyzerTab::Alerts);
        assert_eq!(state.selected_metric, MetricType::MessageRate);
        assert_eq!(state.time_range, TimeRange::Last6Hours);
        assert_eq!(state.alerts.len(), 2);
        assert_eq!(state.selected_alert, 1);

        // Clear alerts
        state.clear_alerts();
        assert!(state.alerts.is_empty());
    }

    #[test]
    fn test_empty_alert_navigation() {
        let mut state = PerformanceAnalyzerState::new();

        // No alerts, navigation should not panic
        state.next_alert();
        assert_eq!(state.selected_alert, 0);

        state.prev_alert();
        assert_eq!(state.selected_alert, 0);
    }

    #[test]
    fn test_metric_type_hash_equality() {
        use std::collections::HashSet;

        let mut set = HashSet::new();
        set.insert(MetricType::Cpu);
        set.insert(MetricType::Memory);

        assert!(set.contains(&MetricType::Cpu));
        assert!(set.contains(&MetricType::Memory));
        assert!(!set.contains(&MetricType::Latency));
    }
}
