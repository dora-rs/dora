/// Dashboard-specific data structures for Issue #24
use std::time::{Duration, Instant};

/// Complete dashboard state
#[derive(Debug, Clone, Default)]
pub struct DashboardState {
    pub system_overview: SystemOverview,
    pub dataflow_summary: DataflowSummary,
    pub performance_metrics: PerformanceMetrics,
    pub recent_activity: RecentActivity,
    pub alerts: Vec<Alert>,
    pub quick_stats: QuickStats,
}

/// System overview with comprehensive metrics
#[derive(Debug, Clone)]
pub struct SystemOverview {
    pub status: SystemStatus,
    pub uptime: Duration,
    pub version: String,
    pub cpu_usage: f64,
    pub memory_usage: MemoryUsage,
    pub disk_usage: DiskUsage,
    pub network_activity: NetworkActivity,
    pub active_connections: u32,
}

impl Default for SystemOverview {
    fn default() -> Self {
        Self {
            status: SystemStatus::Disconnected,
            uptime: Duration::from_secs(0),
            version: env!("CARGO_PKG_VERSION").to_string(),
            cpu_usage: 0.0,
            memory_usage: MemoryUsage::default(),
            disk_usage: DiskUsage::default(),
            network_activity: NetworkActivity::default(),
            active_connections: 0,
        }
    }
}

/// System connection status
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SystemStatus {
    Connected,
    Disconnected,
    Connecting,
}

impl std::fmt::Display for SystemStatus {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            SystemStatus::Connected => write!(f, "Connected"),
            SystemStatus::Disconnected => write!(f, "Disconnected"),
            SystemStatus::Connecting => write!(f, "Connecting..."),
        }
    }
}

/// Detailed memory usage information
#[derive(Debug, Clone, Default)]
pub struct MemoryUsage {
    pub total_mb: u64,
    pub used_mb: u64,
    pub usage_percent: f64,
}

/// Disk usage information
#[derive(Debug, Clone, Default)]
pub struct DiskUsage {
    pub total_gb: u64,
    pub used_gb: u64,
    pub usage_percent: f64,
}

/// Network activity metrics
#[derive(Debug, Clone, Default)]
pub struct NetworkActivity {
    pub bytes_received: u64,
    pub bytes_transmitted: u64,
}

/// Dataflow summary with health tracking
#[derive(Debug, Clone, Default)]
pub struct DataflowSummary {
    pub total_dataflows: u32,
    pub running_dataflows: u32,
    pub failed_dataflows: u32,
    pub stopped_dataflows: u32,
    pub total_nodes: u32,
    pub healthy_nodes: u32,
    pub unhealthy_nodes: u32,
    pub recent_deployments: Vec<RecentDeployment>,
}

impl DataflowSummary {
    /// Calculate overall health percentage
    pub fn health_percentage(&self) -> f64 {
        if self.total_dataflows == 0 {
            100.0
        } else {
            (self.running_dataflows as f64 / self.total_dataflows as f64) * 100.0
        }
    }

    /// Calculate node health percentage
    pub fn node_health_percentage(&self) -> f64 {
        if self.total_nodes == 0 {
            100.0
        } else {
            (self.healthy_nodes as f64 / self.total_nodes as f64) * 100.0
        }
    }
}

/// Recent deployment information
#[derive(Debug, Clone)]
pub struct RecentDeployment {
    pub dataflow_name: String,
    pub timestamp: Instant,
    pub status: DeploymentStatus,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DeploymentStatus {
    Success,
    Failed,
    InProgress,
}

/// Performance metrics over time
#[derive(Debug, Clone, Default)]
pub struct PerformanceMetrics {
    pub cpu_history: Vec<MetricPoint>,
    pub memory_history: Vec<MetricPoint>,
    pub throughput_history: Vec<MetricPoint>,
    pub latency_history: Vec<MetricPoint>,
    pub max_history_points: usize,
}

impl PerformanceMetrics {
    pub fn new(max_points: usize) -> Self {
        Self {
            cpu_history: Vec::with_capacity(max_points),
            memory_history: Vec::with_capacity(max_points),
            throughput_history: Vec::with_capacity(max_points),
            latency_history: Vec::with_capacity(max_points),
            max_history_points: max_points,
        }
    }

    /// Add a new metric point and maintain history limit
    pub fn add_cpu_point(&mut self, value: f64) {
        Self::add_point(&mut self.cpu_history, self.max_history_points, value);
    }

    pub fn add_memory_point(&mut self, value: f64) {
        Self::add_point(&mut self.memory_history, self.max_history_points, value);
    }

    pub fn add_throughput_point(&mut self, value: f64) {
        Self::add_point(&mut self.throughput_history, self.max_history_points, value);
    }

    pub fn add_latency_point(&mut self, value: f64) {
        Self::add_point(&mut self.latency_history, self.max_history_points, value);
    }

    fn add_point(history: &mut Vec<MetricPoint>, max_points: usize, value: f64) {
        if history.len() >= max_points {
            history.remove(0);
        }
        history.push(MetricPoint {
            timestamp: Instant::now(),
            value,
        });
    }
}

/// Single metric data point
#[derive(Debug, Clone)]
pub struct MetricPoint {
    pub timestamp: Instant,
    pub value: f64,
}

/// Recent activity tracking
#[derive(Debug, Clone)]
pub struct RecentActivity {
    pub activities: Vec<ActivityItem>,
    pub max_items: usize,
}

impl Default for RecentActivity {
    fn default() -> Self {
        Self {
            activities: Vec::new(),
            max_items: 50,
        }
    }
}

impl RecentActivity {
    pub fn add_activity(&mut self, activity: ActivityItem) {
        if self.activities.len() >= self.max_items {
            self.activities.remove(0);
        }
        self.activities.push(activity);
    }

    /// Get recent activities (newest first)
    pub fn recent(&self, limit: usize) -> Vec<&ActivityItem> {
        self.activities.iter().rev().take(limit).collect()
    }
}

/// Single activity item
#[derive(Debug, Clone)]
pub struct ActivityItem {
    pub timestamp: Instant,
    pub event_type: ActivityType,
    pub description: String,
    pub dataflow_name: Option<String>,
}

/// Types of activities
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ActivityType {
    DataflowStarted,
    DataflowStopped,
    DataflowFailed,
    NodeStarted,
    NodeStopped,
    NodeFailed,
    SystemEvent,
    UserAction,
}

impl ActivityType {
    pub fn icon(&self) -> &'static str {
        match self {
            ActivityType::DataflowStarted => "▶",
            ActivityType::DataflowStopped => "■",
            ActivityType::DataflowFailed => "✗",
            ActivityType::NodeStarted => "→",
            ActivityType::NodeStopped => "◾",
            ActivityType::NodeFailed => "⚠",
            ActivityType::SystemEvent => "ℹ",
            ActivityType::UserAction => "👤",
        }
    }
}

/// Alert information
#[derive(Debug, Clone)]
pub struct Alert {
    pub level: AlertLevel,
    pub title: String,
    pub message: String,
    pub timestamp: Instant,
    pub acknowledged: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum AlertLevel {
    Info,
    Warning,
    Error,
    Critical,
}

impl AlertLevel {
    pub fn color(&self) -> ratatui::style::Color {
        match self {
            AlertLevel::Info => ratatui::style::Color::Blue,
            AlertLevel::Warning => ratatui::style::Color::Yellow,
            AlertLevel::Error => ratatui::style::Color::Red,
            AlertLevel::Critical => ratatui::style::Color::Magenta,
        }
    }

    pub fn icon(&self) -> &'static str {
        match self {
            AlertLevel::Info => "ℹ",
            AlertLevel::Warning => "⚠",
            AlertLevel::Error => "✗",
            AlertLevel::Critical => "🔥",
        }
    }
}

/// Quick statistics summary
#[derive(Debug, Clone, Default)]
pub struct QuickStats {
    pub total_messages_processed: u64,
    pub avg_message_latency_ms: f64,
    pub error_count_last_hour: u32,
    pub active_dataflows_percent: f64,
}

/// Refresh manager for dashboard updates
#[derive(Debug)]
pub struct RefreshManager {
    refresh_interval: Duration,
    last_refresh: Instant,
    is_running: bool,
}

impl RefreshManager {
    pub fn new(refresh_interval: Duration) -> Self {
        Self {
            refresh_interval,
            last_refresh: Instant::now(),
            is_running: false,
        }
    }

    pub fn start(&mut self) {
        self.is_running = true;
        self.last_refresh = Instant::now();
    }

    pub fn stop(&mut self) {
        self.is_running = false;
    }

    pub fn should_refresh(&self) -> bool {
        self.is_running && self.last_refresh.elapsed() >= self.refresh_interval
    }

    pub fn mark_refreshed(&mut self) {
        self.last_refresh = Instant::now();
    }

    pub fn set_interval(&mut self, interval: Duration) {
        self.refresh_interval = interval;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_dataflow_summary_health_percentage() {
        let summary = DataflowSummary {
            total_dataflows: 10,
            running_dataflows: 8,
            ..Default::default()
        };
        assert_eq!(summary.health_percentage(), 80.0);
    }

    #[test]
    fn test_dataflow_summary_empty() {
        let summary = DataflowSummary::default();
        assert_eq!(summary.health_percentage(), 100.0);
    }

    #[test]
    fn test_recent_activity_limit() {
        let mut activity = RecentActivity::default();
        activity.max_items = 3;

        for i in 0..5 {
            activity.add_activity(ActivityItem {
                timestamp: Instant::now(),
                event_type: ActivityType::SystemEvent,
                description: format!("Event {}", i),
                dataflow_name: None,
            });
        }

        assert_eq!(activity.activities.len(), 3);
    }

    #[test]
    fn test_refresh_manager() {
        let mut manager = RefreshManager::new(Duration::from_secs(1));
        assert!(!manager.should_refresh());

        manager.start();
        assert!(!manager.should_refresh()); // Not enough time passed

        std::thread::sleep(Duration::from_millis(1100));
        assert!(manager.should_refresh());

        manager.mark_refreshed();
        assert!(!manager.should_refresh());
    }

    #[test]
    fn test_performance_metrics_history_limit() {
        let mut metrics = PerformanceMetrics::new(3);

        for i in 0..5 {
            metrics.add_cpu_point(i as f64);
        }

        assert_eq!(metrics.cpu_history.len(), 3);
        assert_eq!(metrics.cpu_history[0].value, 2.0); // Oldest is value 2
        assert_eq!(metrics.cpu_history[2].value, 4.0); // Newest is value 4
    }
}
