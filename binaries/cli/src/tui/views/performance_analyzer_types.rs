/// Type definitions for Performance Analyzer View (Issue #26)
use std::time::Instant;

/// Tab variants for Performance Analyzer
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AnalyzerTab {
    Overview,
    Metrics,
    Alerts,
}

impl AnalyzerTab {
    /// Get all available tabs
    pub fn all() -> Vec<AnalyzerTab> {
        vec![
            AnalyzerTab::Overview,
            AnalyzerTab::Metrics,
            AnalyzerTab::Alerts,
        ]
    }

    /// Get the display name for this tab
    pub fn name(&self) -> &str {
        match self {
            AnalyzerTab::Overview => "Overview",
            AnalyzerTab::Metrics => "Metrics",
            AnalyzerTab::Alerts => "Alerts",
        }
    }

    /// Get the keyboard shortcut for this tab
    pub fn shortcut(&self) -> &str {
        match self {
            AnalyzerTab::Overview => "1",
            AnalyzerTab::Metrics => "2",
            AnalyzerTab::Alerts => "3",
        }
    }

    /// Get the next tab in the sequence
    pub fn next(&self) -> AnalyzerTab {
        match self {
            AnalyzerTab::Overview => AnalyzerTab::Metrics,
            AnalyzerTab::Metrics => AnalyzerTab::Alerts,
            AnalyzerTab::Alerts => AnalyzerTab::Overview,
        }
    }

    /// Get the previous tab in the sequence
    pub fn prev(&self) -> AnalyzerTab {
        match self {
            AnalyzerTab::Overview => AnalyzerTab::Alerts,
            AnalyzerTab::Metrics => AnalyzerTab::Overview,
            AnalyzerTab::Alerts => AnalyzerTab::Metrics,
        }
    }
}

/// Time range for performance data
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TimeRange {
    LastHour,
    Last6Hours,
    Last24Hours,
}

impl TimeRange {
    /// Get all available time ranges
    pub fn all() -> Vec<TimeRange> {
        vec![
            TimeRange::LastHour,
            TimeRange::Last6Hours,
            TimeRange::Last24Hours,
        ]
    }

    /// Get the display name for this time range
    pub fn name(&self) -> &str {
        match self {
            TimeRange::LastHour => "Last Hour",
            TimeRange::Last6Hours => "Last 6 Hours",
            TimeRange::Last24Hours => "Last 24 Hours",
        }
    }

    /// Get the next time range in the sequence
    pub fn next(&self) -> TimeRange {
        match self {
            TimeRange::LastHour => TimeRange::Last6Hours,
            TimeRange::Last6Hours => TimeRange::Last24Hours,
            TimeRange::Last24Hours => TimeRange::LastHour,
        }
    }

    /// Get the duration in seconds
    pub fn duration_secs(&self) -> u64 {
        match self {
            TimeRange::LastHour => 3600,
            TimeRange::Last6Hours => 21600,
            TimeRange::Last24Hours => 86400,
        }
    }

    /// Get the number of data points for this range
    pub fn data_points(&self) -> usize {
        match self {
            TimeRange::LastHour => 60,    // 1 point per minute
            TimeRange::Last6Hours => 72,  // 1 point per 5 minutes
            TimeRange::Last24Hours => 96, // 1 point per 15 minutes
        }
    }
}

impl Default for TimeRange {
    fn default() -> Self {
        TimeRange::LastHour
    }
}

/// Type of performance metric
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum MetricType {
    Cpu,
    Memory,
    MessageRate,
    Latency,
}

impl MetricType {
    /// Get all available metric types
    pub fn all() -> Vec<MetricType> {
        vec![
            MetricType::Cpu,
            MetricType::Memory,
            MetricType::MessageRate,
            MetricType::Latency,
        ]
    }

    /// Get the display name for this metric
    pub fn name(&self) -> &str {
        match self {
            MetricType::Cpu => "CPU",
            MetricType::Memory => "Memory",
            MetricType::MessageRate => "Message Rate",
            MetricType::Latency => "Latency",
        }
    }

    /// Get the unit for this metric
    pub fn unit(&self) -> &str {
        match self {
            MetricType::Cpu => "%",
            MetricType::Memory => "MB",
            MetricType::MessageRate => "msg/s",
            MetricType::Latency => "ms",
        }
    }

    /// Get the default warning threshold
    pub fn warning_threshold(&self) -> f64 {
        match self {
            MetricType::Cpu => 70.0,
            MetricType::Memory => 80.0,
            MetricType::MessageRate => 1000.0,
            MetricType::Latency => 100.0,
        }
    }

    /// Get the default critical threshold
    pub fn critical_threshold(&self) -> f64 {
        match self {
            MetricType::Cpu => 90.0,
            MetricType::Memory => 95.0,
            MetricType::MessageRate => 5000.0,
            MetricType::Latency => 500.0,
        }
    }
}

/// Performance alert severity
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AlertSeverity {
    Warning,
    Critical,
}

impl AlertSeverity {
    /// Get the display name for this severity
    pub fn name(&self) -> &str {
        match self {
            AlertSeverity::Warning => "Warning",
            AlertSeverity::Critical => "Critical",
        }
    }

    /// Get the symbol for this severity
    pub fn symbol(&self) -> &str {
        match self {
            AlertSeverity::Warning => "⚠",
            AlertSeverity::Critical => "🔴",
        }
    }
}

/// Performance alert
#[derive(Debug, Clone)]
pub struct PerformanceAlert {
    pub metric_type: MetricType,
    pub severity: AlertSeverity,
    pub current_value: f64,
    pub threshold: f64,
    pub message: String,
    pub timestamp: Instant,
}

impl PerformanceAlert {
    /// Create a new performance alert
    pub fn new(
        metric_type: MetricType,
        severity: AlertSeverity,
        current_value: f64,
        threshold: f64,
    ) -> Self {
        let message = format!(
            "{} usage at {:.1}{} exceeds {} threshold of {:.1}{}",
            metric_type.name(),
            current_value,
            metric_type.unit(),
            severity.name().to_lowercase(),
            threshold,
            metric_type.unit()
        );

        Self {
            metric_type,
            severity,
            current_value,
            threshold,
            message,
            timestamp: Instant::now(),
        }
    }
}

/// State for the Performance Analyzer view
#[derive(Debug, Clone)]
pub struct PerformanceAnalyzerState {
    /// Currently active tab
    pub active_tab: AnalyzerTab,

    /// Selected time range
    pub time_range: TimeRange,

    /// Selected metric for detailed view
    pub selected_metric: MetricType,

    /// Current performance alerts
    pub alerts: Vec<PerformanceAlert>,

    /// Last refresh timestamp
    pub last_refresh: Instant,

    /// Selected alert index
    pub selected_alert: usize,
}

impl PerformanceAnalyzerState {
    /// Create a new PerformanceAnalyzerState
    pub fn new() -> Self {
        Self {
            active_tab: AnalyzerTab::Overview,
            time_range: TimeRange::default(),
            selected_metric: MetricType::Cpu,
            alerts: Vec::new(),
            last_refresh: Instant::now(),
            selected_alert: 0,
        }
    }

    /// Switch to a specific tab
    pub fn switch_tab(&mut self, tab: AnalyzerTab) {
        self.active_tab = tab;
    }

    /// Navigate to the next tab
    pub fn next_tab(&mut self) {
        self.active_tab = self.active_tab.next();
    }

    /// Navigate to the previous tab
    pub fn prev_tab(&mut self) {
        self.active_tab = self.active_tab.prev();
    }

    /// Cycle to the next time range
    pub fn next_time_range(&mut self) {
        self.time_range = self.time_range.next();
    }

    /// Select the next metric
    pub fn next_metric(&mut self) {
        let metrics = MetricType::all();
        let current_idx = metrics
            .iter()
            .position(|m| *m == self.selected_metric)
            .unwrap_or(0);
        let next_idx = (current_idx + 1) % metrics.len();
        self.selected_metric = metrics[next_idx];
    }

    /// Select the previous metric
    pub fn prev_metric(&mut self) {
        let metrics = MetricType::all();
        let current_idx = metrics
            .iter()
            .position(|m| *m == self.selected_metric)
            .unwrap_or(0);
        let prev_idx = if current_idx == 0 {
            metrics.len() - 1
        } else {
            current_idx - 1
        };
        self.selected_metric = metrics[prev_idx];
    }

    /// Select the next alert
    pub fn next_alert(&mut self) {
        if !self.alerts.is_empty() {
            self.selected_alert = (self.selected_alert + 1) % self.alerts.len();
        }
    }

    /// Select the previous alert
    pub fn prev_alert(&mut self) {
        if !self.alerts.is_empty() {
            self.selected_alert = if self.selected_alert == 0 {
                self.alerts.len() - 1
            } else {
                self.selected_alert - 1
            };
        }
    }

    /// Add a performance alert
    pub fn add_alert(&mut self, alert: PerformanceAlert) {
        self.alerts.push(alert);
    }

    /// Clear all alerts
    pub fn clear_alerts(&mut self) {
        self.alerts.clear();
        self.selected_alert = 0;
    }

    /// Mark as refreshed
    pub fn mark_refreshed(&mut self) {
        self.last_refresh = Instant::now();
    }
}

impl Default for PerformanceAnalyzerState {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
        assert_eq!(MetricType::Cpu.warning_threshold(), 70.0);
        assert_eq!(MetricType::Cpu.critical_threshold(), 90.0);
        assert!(MetricType::Cpu.warning_threshold() < MetricType::Cpu.critical_threshold());
    }

    #[test]
    fn test_alert_severity_names() {
        assert_eq!(AlertSeverity::Warning.name(), "Warning");
        assert_eq!(AlertSeverity::Critical.name(), "Critical");
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
    fn test_performance_analyzer_state_tab_navigation() {
        let mut state = PerformanceAnalyzerState::new();

        state.next_tab();
        assert_eq!(state.active_tab, AnalyzerTab::Metrics);

        state.next_tab();
        assert_eq!(state.active_tab, AnalyzerTab::Alerts);

        state.prev_tab();
        assert_eq!(state.active_tab, AnalyzerTab::Metrics);
    }

    #[test]
    fn test_performance_analyzer_state_time_range_cycling() {
        let mut state = PerformanceAnalyzerState::new();

        assert_eq!(state.time_range, TimeRange::LastHour);

        state.next_time_range();
        assert_eq!(state.time_range, TimeRange::Last6Hours);

        state.next_time_range();
        assert_eq!(state.time_range, TimeRange::Last24Hours);
    }

    #[test]
    fn test_performance_analyzer_state_metric_selection() {
        let mut state = PerformanceAnalyzerState::new();

        assert_eq!(state.selected_metric, MetricType::Cpu);

        state.next_metric();
        assert_eq!(state.selected_metric, MetricType::Memory);

        state.next_metric();
        assert_eq!(state.selected_metric, MetricType::MessageRate);

        state.prev_metric();
        assert_eq!(state.selected_metric, MetricType::Memory);
    }

    #[test]
    fn test_performance_analyzer_state_alert_management() {
        let mut state = PerformanceAnalyzerState::new();

        assert!(state.alerts.is_empty());

        let alert1 = PerformanceAlert::new(MetricType::Cpu, AlertSeverity::Warning, 75.0, 70.0);
        let alert2 = PerformanceAlert::new(MetricType::Memory, AlertSeverity::Critical, 96.0, 95.0);

        state.add_alert(alert1);
        state.add_alert(alert2);

        assert_eq!(state.alerts.len(), 2);

        state.next_alert();
        assert_eq!(state.selected_alert, 1);

        state.next_alert();
        assert_eq!(state.selected_alert, 0); // Wraps around

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
}
