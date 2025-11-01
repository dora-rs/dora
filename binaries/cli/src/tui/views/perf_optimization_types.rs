// Performance Optimization Types for TUI View
// Issue #36: Performance Optimization Suite

use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Main sections of the performance optimization view
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum PerfSection {
    PerformanceMetrics,
    BottleneckDetection,
    OptimizationSuggestions,
    ResourceAnalysis,
}

impl PerfSection {
    pub fn all() -> Vec<Self> {
        vec![
            Self::PerformanceMetrics,
            Self::BottleneckDetection,
            Self::OptimizationSuggestions,
            Self::ResourceAnalysis,
        ]
    }

    pub fn title(&self) -> &str {
        match self {
            Self::PerformanceMetrics => "Performance Metrics",
            Self::BottleneckDetection => "Bottleneck Detection",
            Self::OptimizationSuggestions => "Optimization Suggestions",
            Self::ResourceAnalysis => "Resource Analysis",
        }
    }

    pub fn description(&self) -> &str {
        match self {
            Self::PerformanceMetrics => "Real-time system and dataflow performance metrics",
            Self::BottleneckDetection => "Identify performance bottlenecks automatically",
            Self::OptimizationSuggestions => {
                "AI-powered recommendations for performance improvements"
            }
            Self::ResourceAnalysis => "Detailed CPU, memory, and network usage analysis",
        }
    }
}

/// Performance metric data point
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PerformanceMetric {
    pub name: String,
    pub metric_type: PerfMetricType,
    pub current_value: f64,
    pub avg_value: f64,
    pub min_value: f64,
    pub max_value: f64,
    pub unit: String,
    pub status: MetricStatus,
    pub updated_at: DateTime<Utc>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum PerfMetricType {
    Throughput,
    Latency,
    CpuUsage,
    MemoryUsage,
    NetworkIO,
    DiskIO,
    MessageRate,
    ErrorRate,
}

impl PerfMetricType {
    pub fn name(&self) -> &str {
        match self {
            Self::Throughput => "Throughput",
            Self::Latency => "Latency",
            Self::CpuUsage => "CPU Usage",
            Self::MemoryUsage => "Memory Usage",
            Self::NetworkIO => "Network I/O",
            Self::DiskIO => "Disk I/O",
            Self::MessageRate => "Message Rate",
            Self::ErrorRate => "Error Rate",
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum MetricStatus {
    Healthy,
    Warning,
    Critical,
    Unknown,
}

impl MetricStatus {
    pub fn name(&self) -> &str {
        match self {
            Self::Healthy => "Healthy",
            Self::Warning => "Warning",
            Self::Critical => "Critical",
            Self::Unknown => "Unknown",
        }
    }
}

/// Performance bottleneck information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Bottleneck {
    pub bottleneck_id: String,
    pub location: BottleneckLocation,
    pub bottleneck_type: BottleneckType,
    pub severity: Severity,
    pub impact: String,
    pub affected_components: Vec<String>,
    pub detected_at: DateTime<Utc>,
    pub metrics: HashMap<String, f64>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum BottleneckLocation {
    Node { node_id: String },
    Link { from: String, to: String },
    System,
    Network,
}

impl BottleneckLocation {
    pub fn display(&self) -> String {
        match self {
            Self::Node { node_id } => format!("Node: {node_id}"),
            Self::Link { from, to } => format!("Link: {from} → {to}"),
            Self::System => "System".to_string(),
            Self::Network => "Network".to_string(),
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum BottleneckType {
    CpuBound,
    MemoryBound,
    IOBound,
    NetworkBound,
    Synchronization,
    QueueBacklog,
}

impl BottleneckType {
    pub fn name(&self) -> &str {
        match self {
            Self::CpuBound => "CPU Bound",
            Self::MemoryBound => "Memory Bound",
            Self::IOBound => "I/O Bound",
            Self::NetworkBound => "Network Bound",
            Self::Synchronization => "Synchronization",
            Self::QueueBacklog => "Queue Backlog",
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Copy)]
pub enum Severity {
    Low,
    Medium,
    High,
    Critical,
}

impl Severity {
    pub fn name(&self) -> &str {
        match self {
            Self::Low => "Low",
            Self::Medium => "Medium",
            Self::High => "High",
            Self::Critical => "Critical",
        }
    }

    pub fn priority(&self) -> i32 {
        match self {
            Self::Low => 1,
            Self::Medium => 2,
            Self::High => 3,
            Self::Critical => 4,
        }
    }
}

/// Optimization suggestion
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OptimizationSuggestion {
    pub suggestion_id: String,
    pub title: String,
    pub category: OptimizationCategory,
    pub priority: Priority,
    pub expected_impact: ImpactLevel,
    pub description: String,
    pub implementation_steps: Vec<String>,
    pub estimated_effort: EffortLevel,
    pub created_at: DateTime<Utc>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum OptimizationCategory {
    Architecture,
    Configuration,
    ResourceAllocation,
    CodeOptimization,
    Caching,
    Parallelization,
}

impl OptimizationCategory {
    pub fn name(&self) -> &str {
        match self {
            Self::Architecture => "Architecture",
            Self::Configuration => "Configuration",
            Self::ResourceAllocation => "Resource Allocation",
            Self::CodeOptimization => "Code Optimization",
            Self::Caching => "Caching",
            Self::Parallelization => "Parallelization",
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Copy)]
pub enum Priority {
    Low,
    Medium,
    High,
    Critical,
}

impl Priority {
    pub fn name(&self) -> &str {
        match self {
            Self::Low => "Low",
            Self::Medium => "Medium",
            Self::High => "High",
            Self::Critical => "Critical",
        }
    }

    pub fn value(&self) -> i32 {
        match self {
            Self::Low => 1,
            Self::Medium => 2,
            Self::High => 3,
            Self::Critical => 4,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum ImpactLevel {
    Minimal,     // <5% improvement
    Moderate,    // 5-20% improvement
    Significant, // 20-50% improvement
    Major,       // >50% improvement
}

impl ImpactLevel {
    pub fn name(&self) -> &str {
        match self {
            Self::Minimal => "Minimal (<5%)",
            Self::Moderate => "Moderate (5-20%)",
            Self::Significant => "Significant (20-50%)",
            Self::Major => "Major (>50%)",
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum EffortLevel {
    Trivial,  // <1 hour
    Low,      // 1-4 hours
    Medium,   // 4-16 hours
    High,     // 16-40 hours
    VeryHigh, // >40 hours
}

impl EffortLevel {
    pub fn name(&self) -> &str {
        match self {
            Self::Trivial => "Trivial (<1h)",
            Self::Low => "Low (1-4h)",
            Self::Medium => "Medium (4-16h)",
            Self::High => "High (16-40h)",
            Self::VeryHigh => "Very High (>40h)",
        }
    }
}

/// Resource usage information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ResourceUsage {
    pub component: String,
    pub cpu_percent: f64,
    pub memory_mb: f64,
    pub network_rx_mbps: f64,
    pub network_tx_mbps: f64,
    pub disk_read_mbps: f64,
    pub disk_write_mbps: f64,
    pub thread_count: usize,
    pub updated_at: DateTime<Utc>,
}

/// Resource trend over time
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ResourceTrend {
    pub resource_type: ResourceType,
    pub component: String,
    pub data_points: Vec<(DateTime<Utc>, f64)>,
    pub trend: TrendDirection,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum ResourceType {
    Cpu,
    Memory,
    Network,
    Disk,
}

impl ResourceType {
    pub fn name(&self) -> &str {
        match self {
            Self::Cpu => "CPU",
            Self::Memory => "Memory",
            Self::Network => "Network",
            Self::Disk => "Disk",
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum TrendDirection {
    Increasing,
    Decreasing,
    Stable,
    Volatile,
}

impl TrendDirection {
    pub fn name(&self) -> &str {
        match self {
            Self::Increasing => "↗ Increasing",
            Self::Decreasing => "↘ Decreasing",
            Self::Stable => "→ Stable",
            Self::Volatile => "⇅ Volatile",
        }
    }
}

/// State for the performance optimization view
#[derive(Debug, Clone)]
pub struct PerfOptimizationState {
    pub current_section: PerfSection,
    pub selected_index: usize,
    pub data: PerfData,
}

impl PerfOptimizationState {
    pub fn new() -> Self {
        Self {
            current_section: PerfSection::PerformanceMetrics,
            selected_index: 0,
            data: PerfData::Metrics(create_mock_metrics()),
        }
    }

    pub fn next_section(&mut self) {
        let sections = PerfSection::all();
        let current_idx = sections
            .iter()
            .position(|s| s == &self.current_section)
            .unwrap_or(0);
        let next_idx = (current_idx + 1) % sections.len();
        self.current_section = sections[next_idx].clone();
        self.selected_index = 0;
        self.update_data();
    }

    pub fn previous_section(&mut self) {
        let sections = PerfSection::all();
        let current_idx = sections
            .iter()
            .position(|s| s == &self.current_section)
            .unwrap_or(0);
        let prev_idx = if current_idx == 0 {
            sections.len() - 1
        } else {
            current_idx - 1
        };
        self.current_section = sections[prev_idx].clone();
        self.selected_index = 0;
        self.update_data();
    }

    pub fn update_data(&mut self) {
        self.data = match &self.current_section {
            PerfSection::PerformanceMetrics => PerfData::Metrics(create_mock_metrics()),
            PerfSection::BottleneckDetection => PerfData::Bottlenecks(create_mock_bottlenecks()),
            PerfSection::OptimizationSuggestions => {
                PerfData::Suggestions(create_mock_suggestions())
            }
            PerfSection::ResourceAnalysis => PerfData::Resources(create_mock_resources()),
        };
    }

    pub fn next_item(&mut self) {
        let max_index = self.get_max_index();
        if max_index > 0 {
            self.selected_index = (self.selected_index + 1).min(max_index - 1);
        }
    }

    pub fn previous_item(&mut self) {
        if self.selected_index > 0 {
            self.selected_index -= 1;
        }
    }

    fn get_max_index(&self) -> usize {
        match &self.data {
            PerfData::Metrics(metrics) => metrics.len(),
            PerfData::Bottlenecks(bottlenecks) => bottlenecks.len(),
            PerfData::Suggestions(suggestions) => suggestions.len(),
            PerfData::Resources(resources) => resources.len(),
        }
    }
}

impl Default for PerfOptimizationState {
    fn default() -> Self {
        Self::new()
    }
}

/// Data container for different sections
#[derive(Debug, Clone)]
pub enum PerfData {
    Metrics(Vec<PerformanceMetric>),
    Bottlenecks(Vec<Bottleneck>),
    Suggestions(Vec<OptimizationSuggestion>),
    Resources(Vec<ResourceUsage>),
}

// Mock data generators for Phase 1 implementation

pub fn create_mock_metrics() -> Vec<PerformanceMetric> {
    vec![
        PerformanceMetric {
            name: "Message Throughput".to_string(),
            metric_type: PerfMetricType::Throughput,
            current_value: 1250.0,
            avg_value: 1180.0,
            min_value: 950.0,
            max_value: 1500.0,
            unit: "msg/s".to_string(),
            status: MetricStatus::Healthy,
            updated_at: Utc::now(),
        },
        PerformanceMetric {
            name: "Average Latency".to_string(),
            metric_type: PerfMetricType::Latency,
            current_value: 12.5,
            avg_value: 15.2,
            min_value: 8.0,
            max_value: 45.0,
            unit: "ms".to_string(),
            status: MetricStatus::Healthy,
            updated_at: Utc::now(),
        },
        PerformanceMetric {
            name: "CPU Usage".to_string(),
            metric_type: PerfMetricType::CpuUsage,
            current_value: 78.5,
            avg_value: 65.0,
            min_value: 35.0,
            max_value: 95.0,
            unit: "%".to_string(),
            status: MetricStatus::Warning,
            updated_at: Utc::now(),
        },
        PerformanceMetric {
            name: "Memory Usage".to_string(),
            metric_type: PerfMetricType::MemoryUsage,
            current_value: 2450.0,
            avg_value: 2100.0,
            min_value: 1800.0,
            max_value: 3200.0,
            unit: "MB".to_string(),
            status: MetricStatus::Healthy,
            updated_at: Utc::now(),
        },
        PerformanceMetric {
            name: "Network I/O".to_string(),
            metric_type: PerfMetricType::NetworkIO,
            current_value: 45.2,
            avg_value: 38.5,
            min_value: 15.0,
            max_value: 85.0,
            unit: "MB/s".to_string(),
            status: MetricStatus::Healthy,
            updated_at: Utc::now(),
        },
        PerformanceMetric {
            name: "Error Rate".to_string(),
            metric_type: PerfMetricType::ErrorRate,
            current_value: 0.02,
            avg_value: 0.05,
            min_value: 0.0,
            max_value: 0.15,
            unit: "%".to_string(),
            status: MetricStatus::Healthy,
            updated_at: Utc::now(),
        },
    ]
}

pub fn create_mock_bottlenecks() -> Vec<Bottleneck> {
    vec![
        Bottleneck {
            bottleneck_id: "btn_001".to_string(),
            location: BottleneckLocation::Node {
                node_id: "image_processor".to_string(),
            },
            bottleneck_type: BottleneckType::CpuBound,
            severity: Severity::High,
            impact: "Processing queue backlog increasing, 200+ messages waiting".to_string(),
            affected_components: vec![
                "image_processor".to_string(),
                "downstream_nodes".to_string(),
            ],
            detected_at: Utc::now(),
            metrics: HashMap::from([
                ("cpu_usage".to_string(), 95.5),
                ("queue_depth".to_string(), 215.0),
            ]),
        },
        Bottleneck {
            bottleneck_id: "btn_002".to_string(),
            location: BottleneckLocation::Link {
                from: "sensor_input".to_string(),
                to: "data_aggregator".to_string(),
            },
            bottleneck_type: BottleneckType::NetworkBound,
            severity: Severity::Medium,
            impact: "Network latency causing 15ms delays in data pipeline".to_string(),
            affected_components: vec!["sensor_input".to_string(), "data_aggregator".to_string()],
            detected_at: Utc::now(),
            metrics: HashMap::from([
                ("latency_ms".to_string(), 15.2),
                ("packet_loss".to_string(), 0.5),
            ]),
        },
        Bottleneck {
            bottleneck_id: "btn_003".to_string(),
            location: BottleneckLocation::Node {
                node_id: "database_writer".to_string(),
            },
            bottleneck_type: BottleneckType::IOBound,
            severity: Severity::Medium,
            impact: "Disk I/O saturation limiting write throughput".to_string(),
            affected_components: vec!["database_writer".to_string()],
            detected_at: Utc::now(),
            metrics: HashMap::from([
                ("disk_util".to_string(), 92.0),
                ("write_latency_ms".to_string(), 45.0),
            ]),
        },
    ]
}

pub fn create_mock_suggestions() -> Vec<OptimizationSuggestion> {
    vec![
        OptimizationSuggestion {
            suggestion_id: "opt_001".to_string(),
            title: "Enable Multi-threading for Image Processor".to_string(),
            category: OptimizationCategory::Parallelization,
            priority: Priority::Critical,
            expected_impact: ImpactLevel::Major,
            description: "Image processor is CPU-bound and single-threaded. Enable parallel processing to utilize all available cores.".to_string(),
            implementation_steps: vec![
                "Update image_processor configuration to enable thread pool".to_string(),
                "Set worker_threads to match CPU core count (8)".to_string(),
                "Add batch processing for similar operations".to_string(),
                "Test with varying load patterns".to_string(),
            ],
            estimated_effort: EffortLevel::Low,
            created_at: Utc::now(),
        },
        OptimizationSuggestion {
            suggestion_id: "opt_002".to_string(),
            title: "Implement Message Caching Layer".to_string(),
            category: OptimizationCategory::Caching,
            priority: Priority::High,
            expected_impact: ImpactLevel::Significant,
            description: "Add caching layer to reduce redundant computations for frequently accessed data.".to_string(),
            implementation_steps: vec![
                "Identify cacheable data patterns (60% of queries are repeated)".to_string(),
                "Implement LRU cache with 1GB limit".to_string(),
                "Add cache hit/miss metrics".to_string(),
                "Monitor cache effectiveness".to_string(),
            ],
            estimated_effort: EffortLevel::Medium,
            created_at: Utc::now(),
        },
        OptimizationSuggestion {
            suggestion_id: "opt_003".to_string(),
            title: "Optimize Database Write Batching".to_string(),
            category: OptimizationCategory::Configuration,
            priority: Priority::High,
            expected_impact: ImpactLevel::Significant,
            description: "Batch database writes to reduce I/O overhead and improve throughput.".to_string(),
            implementation_steps: vec![
                "Configure batch size to 100 records or 100ms timeout".to_string(),
                "Enable write-ahead logging".to_string(),
                "Adjust fsync frequency".to_string(),
            ],
            estimated_effort: EffortLevel::Trivial,
            created_at: Utc::now(),
        },
        OptimizationSuggestion {
            suggestion_id: "opt_004".to_string(),
            title: "Reduce Network Overhead with Compression".to_string(),
            category: OptimizationCategory::Configuration,
            priority: Priority::Medium,
            expected_impact: ImpactLevel::Moderate,
            description: "Enable message compression to reduce network bandwidth usage by ~40%.".to_string(),
            implementation_steps: vec![
                "Enable zstd compression for inter-node communication".to_string(),
                "Set compression level to 3 (balanced)".to_string(),
                "Monitor CPU impact vs network savings".to_string(),
            ],
            estimated_effort: EffortLevel::Trivial,
            created_at: Utc::now(),
        },
        OptimizationSuggestion {
            suggestion_id: "opt_005".to_string(),
            title: "Implement Memory Pool for Large Allocations".to_string(),
            category: OptimizationCategory::ResourceAllocation,
            priority: Priority::Low,
            expected_impact: ImpactLevel::Moderate,
            description: "Pre-allocate memory pool to reduce allocation overhead and fragmentation.".to_string(),
            implementation_steps: vec![
                "Analyze allocation patterns (avg size: 4KB)".to_string(),
                "Implement fixed-size block allocator".to_string(),
                "Add memory pool metrics".to_string(),
            ],
            estimated_effort: EffortLevel::Medium,
            created_at: Utc::now(),
        },
    ]
}

pub fn create_mock_resources() -> Vec<ResourceUsage> {
    vec![
        ResourceUsage {
            component: "image_processor".to_string(),
            cpu_percent: 95.5,
            memory_mb: 850.0,
            network_rx_mbps: 12.5,
            network_tx_mbps: 8.2,
            disk_read_mbps: 2.1,
            disk_write_mbps: 0.5,
            thread_count: 4,
            updated_at: Utc::now(),
        },
        ResourceUsage {
            component: "data_aggregator".to_string(),
            cpu_percent: 45.2,
            memory_mb: 620.0,
            network_rx_mbps: 25.0,
            network_tx_mbps: 18.5,
            disk_read_mbps: 0.8,
            disk_write_mbps: 0.3,
            thread_count: 8,
            updated_at: Utc::now(),
        },
        ResourceUsage {
            component: "database_writer".to_string(),
            cpu_percent: 35.0,
            memory_mb: 420.0,
            network_rx_mbps: 5.5,
            network_tx_mbps: 1.2,
            disk_read_mbps: 1.5,
            disk_write_mbps: 45.0,
            thread_count: 2,
            updated_at: Utc::now(),
        },
        ResourceUsage {
            component: "sensor_input".to_string(),
            cpu_percent: 15.5,
            memory_mb: 180.0,
            network_rx_mbps: 8.0,
            network_tx_mbps: 15.0,
            disk_read_mbps: 0.0,
            disk_write_mbps: 0.0,
            thread_count: 2,
            updated_at: Utc::now(),
        },
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_perf_section_all() {
        let sections = PerfSection::all();
        assert_eq!(sections.len(), 4);
        assert_eq!(sections[0], PerfSection::PerformanceMetrics);
        assert_eq!(sections[1], PerfSection::BottleneckDetection);
        assert_eq!(sections[2], PerfSection::OptimizationSuggestions);
        assert_eq!(sections[3], PerfSection::ResourceAnalysis);
    }

    #[test]
    fn test_perf_section_title() {
        assert_eq!(
            PerfSection::PerformanceMetrics.title(),
            "Performance Metrics"
        );
        assert_eq!(
            PerfSection::BottleneckDetection.title(),
            "Bottleneck Detection"
        );
        assert_eq!(
            PerfSection::OptimizationSuggestions.title(),
            "Optimization Suggestions"
        );
        assert_eq!(PerfSection::ResourceAnalysis.title(), "Resource Analysis");
    }

    #[test]
    fn test_metric_type_name() {
        assert_eq!(PerfMetricType::Throughput.name(), "Throughput");
        assert_eq!(PerfMetricType::Latency.name(), "Latency");
        assert_eq!(PerfMetricType::CpuUsage.name(), "CPU Usage");
    }

    #[test]
    fn test_metric_status_name() {
        assert_eq!(MetricStatus::Healthy.name(), "Healthy");
        assert_eq!(MetricStatus::Warning.name(), "Warning");
        assert_eq!(MetricStatus::Critical.name(), "Critical");
    }

    #[test]
    fn test_bottleneck_type_name() {
        assert_eq!(BottleneckType::CpuBound.name(), "CPU Bound");
        assert_eq!(BottleneckType::MemoryBound.name(), "Memory Bound");
        assert_eq!(BottleneckType::IOBound.name(), "I/O Bound");
    }

    #[test]
    fn test_severity_priority() {
        assert_eq!(Severity::Low.priority(), 1);
        assert_eq!(Severity::Medium.priority(), 2);
        assert_eq!(Severity::High.priority(), 3);
        assert_eq!(Severity::Critical.priority(), 4);
    }

    #[test]
    fn test_priority_value() {
        assert_eq!(Priority::Low.value(), 1);
        assert_eq!(Priority::Medium.value(), 2);
        assert_eq!(Priority::High.value(), 3);
        assert_eq!(Priority::Critical.value(), 4);
    }

    #[test]
    fn test_optimization_category_name() {
        assert_eq!(OptimizationCategory::Architecture.name(), "Architecture");
        assert_eq!(OptimizationCategory::Caching.name(), "Caching");
        assert_eq!(
            OptimizationCategory::Parallelization.name(),
            "Parallelization"
        );
    }

    #[test]
    fn test_impact_level_name() {
        assert_eq!(ImpactLevel::Minimal.name(), "Minimal (<5%)");
        assert_eq!(ImpactLevel::Moderate.name(), "Moderate (5-20%)");
        assert_eq!(ImpactLevel::Significant.name(), "Significant (20-50%)");
        assert_eq!(ImpactLevel::Major.name(), "Major (>50%)");
    }

    #[test]
    fn test_effort_level_name() {
        assert_eq!(EffortLevel::Trivial.name(), "Trivial (<1h)");
        assert_eq!(EffortLevel::Low.name(), "Low (1-4h)");
        assert_eq!(EffortLevel::Medium.name(), "Medium (4-16h)");
    }

    #[test]
    fn test_resource_type_name() {
        assert_eq!(ResourceType::Cpu.name(), "CPU");
        assert_eq!(ResourceType::Memory.name(), "Memory");
        assert_eq!(ResourceType::Network.name(), "Network");
    }

    #[test]
    fn test_trend_direction_name() {
        assert!(TrendDirection::Increasing.name().contains("Increasing"));
        assert!(TrendDirection::Decreasing.name().contains("Decreasing"));
        assert!(TrendDirection::Stable.name().contains("Stable"));
    }

    #[test]
    fn test_perf_optimization_state_new() {
        let state = PerfOptimizationState::new();
        assert_eq!(state.current_section, PerfSection::PerformanceMetrics);
        assert_eq!(state.selected_index, 0);
        assert!(matches!(state.data, PerfData::Metrics(_)));
    }

    #[test]
    fn test_perf_optimization_state_navigation() {
        let mut state = PerfOptimizationState::new();

        state.next_section();
        assert_eq!(state.current_section, PerfSection::BottleneckDetection);
        assert!(matches!(state.data, PerfData::Bottlenecks(_)));

        state.next_section();
        assert_eq!(state.current_section, PerfSection::OptimizationSuggestions);
        assert!(matches!(state.data, PerfData::Suggestions(_)));

        state.previous_section();
        assert_eq!(state.current_section, PerfSection::BottleneckDetection);
    }

    #[test]
    fn test_perf_optimization_state_wrapping() {
        let mut state = PerfOptimizationState::new();

        // Wrap forward
        state.current_section = PerfSection::ResourceAnalysis;
        state.next_section();
        assert_eq!(state.current_section, PerfSection::PerformanceMetrics);

        // Wrap backward
        state.previous_section();
        assert_eq!(state.current_section, PerfSection::ResourceAnalysis);
    }

    #[test]
    fn test_mock_metrics_creation() {
        let metrics = create_mock_metrics();
        assert!(!metrics.is_empty());
        assert!(metrics.len() >= 5);

        for metric in &metrics {
            assert!(!metric.name.is_empty());
            assert!(!metric.unit.is_empty());
            assert!(metric.current_value >= 0.0);
        }
    }

    #[test]
    fn test_mock_bottlenecks_creation() {
        let bottlenecks = create_mock_bottlenecks();
        assert!(!bottlenecks.is_empty());

        for bottleneck in &bottlenecks {
            assert!(!bottleneck.bottleneck_id.is_empty());
            assert!(!bottleneck.impact.is_empty());
            assert!(!bottleneck.affected_components.is_empty());
        }
    }

    #[test]
    fn test_mock_suggestions_creation() {
        let suggestions = create_mock_suggestions();
        assert!(!suggestions.is_empty());

        for suggestion in &suggestions {
            assert!(!suggestion.suggestion_id.is_empty());
            assert!(!suggestion.title.is_empty());
            assert!(!suggestion.description.is_empty());
            assert!(!suggestion.implementation_steps.is_empty());
        }
    }

    #[test]
    fn test_mock_resources_creation() {
        let resources = create_mock_resources();
        assert!(!resources.is_empty());

        for resource in &resources {
            assert!(!resource.component.is_empty());
            assert!(resource.cpu_percent >= 0.0 && resource.cpu_percent <= 100.0);
            assert!(resource.memory_mb >= 0.0);
            assert!(resource.thread_count > 0);
        }
    }

    #[test]
    fn test_bottleneck_location_display() {
        let node_loc = BottleneckLocation::Node {
            node_id: "test_node".to_string(),
        };
        assert!(node_loc.display().contains("test_node"));

        let link_loc = BottleneckLocation::Link {
            from: "a".to_string(),
            to: "b".to_string(),
        };
        assert!(link_loc.display().contains("a"));
        assert!(link_loc.display().contains("b"));
    }

    #[test]
    fn test_severity_ordering() {
        assert!(Severity::Critical.priority() > Severity::High.priority());
        assert!(Severity::High.priority() > Severity::Medium.priority());
        assert!(Severity::Medium.priority() > Severity::Low.priority());
    }

    #[test]
    fn test_priority_ordering() {
        assert!(Priority::Critical.value() > Priority::High.value());
        assert!(Priority::High.value() > Priority::Medium.value());
        assert!(Priority::Medium.value() > Priority::Low.value());
    }

    #[test]
    fn test_metric_value_ranges() {
        let metrics = create_mock_metrics();
        for metric in metrics {
            assert!(metric.min_value <= metric.current_value);
            assert!(metric.current_value <= metric.max_value);
            assert!(metric.min_value <= metric.avg_value);
            assert!(metric.avg_value <= metric.max_value);
        }
    }

    #[test]
    fn test_resource_usage_validity() {
        let resources = create_mock_resources();
        for resource in resources {
            assert!(resource.cpu_percent <= 100.0);
            assert!(resource.memory_mb >= 0.0);
            assert!(resource.network_rx_mbps >= 0.0);
            assert!(resource.network_tx_mbps >= 0.0);
        }
    }
}
