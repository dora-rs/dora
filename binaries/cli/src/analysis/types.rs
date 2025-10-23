// Analysis Types for Issue #19

use chrono::{DateTime, Utc, Duration};
use serde::{Deserialize, Serialize};
use std::path::PathBuf;
use uuid::Uuid;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AnalysisSession {
    pub session_id: Uuid,
    pub start_time: DateTime<Utc>,
    pub analysis_target: AnalysisTarget,
    pub analysis_type: crate::cli::commands::AnalysisType,
    pub depth: AnalysisDepth,
    pub focus_areas: Vec<AnalysisFocus>,
    pub time_window: TimeWindow,
    pub baseline_window: Option<TimeWindow>,
    pub configuration: AnalysisConfiguration,
}

impl Default for AnalysisSession {
    fn default() -> Self {
        Self {
            session_id: Uuid::new_v4(),
            start_time: Utc::now(),
            analysis_target: AnalysisTarget::Auto,
            analysis_type: crate::cli::commands::AnalysisType::Comprehensive,
            depth: AnalysisDepth::Normal,
            focus_areas: Vec::new(),
            time_window: TimeWindow {
                start: Utc::now() - Duration::hours(1),
                end: Utc::now(),
                duration_description: "1h".to_string(),
            },
            baseline_window: None,
            configuration: AnalysisConfiguration::default(),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum AnalysisTarget {
    Auto,
    Dataflow(String),
    Node(String),
    System,
    Component(String),
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum AnalysisDepth {
    Quick,
    Normal,
    Deep,
    Expert,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum AnalysisFocus {
    Performance,
    Reliability,
    Security,
    Resource,
    Dependencies,
    Data,
    Network,
    Errors,
    Patterns,
    Anomalies,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TimeWindow {
    pub start: DateTime<Utc>,
    pub end: DateTime<Utc>,
    pub duration_description: String,
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct AnalysisConfiguration {
    pub compare_baseline: bool,
    pub export_path: Option<PathBuf>,
    pub export_format: ExportFormat,
    pub live_mode: bool,
    pub refresh_interval_secs: u64,
    pub enable_prediction: bool,
    pub prediction_horizon_secs: u64,
    pub verbosity: AnalysisVerbosity,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum ExportFormat {
    Json,
    Yaml,
    Csv,
    Html,
}

impl Default for ExportFormat {
    fn default() -> Self {
        Self::Json
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum AnalysisVerbosity {
    Minimal,
    Normal,
    Detailed,
    Verbose,
}

impl Default for AnalysisVerbosity {
    fn default() -> Self {
        Self::Normal
    }
}

/// Complete analysis results
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AnalysisResults {
    pub session_info: AnalysisSessionInfo,
    pub performance_analysis: Option<PerformanceAnalysisResult>,
    pub health_analysis: Option<HealthAnalysisResult>,
    pub efficiency_analysis: Option<EfficiencyAnalysisResult>,
    pub trend_analysis: Option<TrendAnalysisResult>,
    pub pattern_analysis: Option<PatternAnalysisResult>,
    pub security_analysis: Option<SecurityAnalysisResult>,
    pub predictive_analysis: Option<PredictiveAnalysisResult>,
    pub comparative_analysis: Option<ComparativeAnalysisResult>,
    pub diagnostics: Option<DiagnosticsResult>,
    pub insights: Vec<AnalysisInsight>,
    pub recommendations: Vec<AnalysisRecommendation>,
    pub overall_scores: OverallScores,
}

impl AnalysisResults {
    pub fn new(session: &AnalysisSession) -> Self {
        Self {
            session_info: AnalysisSessionInfo {
                session_id: session.session_id,
                start_time: session.start_time,
                analysis_duration_secs: 0.0,
                total_data_points: 0,
                time_range: session.time_window.clone(),
            },
            performance_analysis: None,
            health_analysis: None,
            efficiency_analysis: None,
            trend_analysis: None,
            pattern_analysis: None,
            security_analysis: None,
            predictive_analysis: None,
            comparative_analysis: None,
            diagnostics: None,
            insights: Vec::new(),
            recommendations: Vec::new(),
            overall_scores: OverallScores::default(),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AnalysisSessionInfo {
    pub session_id: Uuid,
    pub start_time: DateTime<Utc>,
    pub analysis_duration_secs: f32,
    pub total_data_points: usize,
    pub time_range: TimeWindow,
}

/// Performance Analysis Result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PerformanceAnalysisResult {
    pub throughput: ThroughputAnalysis,
    pub latency: LatencyAnalysis,
    pub resource_usage: ResourceUsageAnalysis,
    pub bottlenecks: Vec<Bottleneck>,
    pub performance_scores: PerformanceScores,
    pub issues: Vec<PerformanceIssue>,
    pub insights: Vec<PerformanceInsight>,
    pub analysis_metadata: AnalysisMetadata,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ThroughputAnalysis {
    pub current: f32,
    pub average: f32,
    pub peak: f32,
    pub trend: TrendDirection,
    pub stability: f32,
    pub percentiles: Percentiles,
    pub time_series: Vec<TimeSeriesPoint>,
}

impl ThroughputAnalysis {
    pub fn empty() -> Self {
        Self {
            current: 0.0,
            average: 0.0,
            peak: 0.0,
            trend: TrendDirection::Stable,
            stability: 0.0,
            percentiles: Percentiles::default(),
            time_series: Vec::new(),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LatencyAnalysis {
    pub current: f32,
    pub average: f32,
    pub percentiles: Percentiles,
    pub trend: TrendDirection,
    pub time_series: Vec<TimeSeriesPoint>,
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct Percentiles {
    pub p50: f32,
    pub p75: f32,
    pub p90: f32,
    pub p95: f32,
    pub p99: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ResourceUsageAnalysis {
    pub cpu_usage: f32,
    pub memory_usage: f32,
    pub disk_io: f32,
    pub network_io: f32,
    pub resource_efficiency: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Bottleneck {
    pub location: String,
    pub severity: BottleneckSeverity,
    pub description: String,
    pub impact: f32,
    pub suggested_fixes: Vec<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum BottleneckSeverity {
    Critical,
    High,
    Medium,
    Low,
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct PerformanceScores {
    pub throughput_score: f32,
    pub latency_score: f32,
    pub resource_efficiency_score: f32,
    pub overall_performance_score: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PerformanceIssue {
    pub issue_type: String,
    pub severity: String,
    pub description: String,
    pub metrics: Vec<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PerformanceInsight {
    pub title: String,
    pub description: String,
    pub impact: String,
}

/// Trend Analysis Result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TrendAnalysisResult {
    pub trends: Vec<Trend>,
    pub seasonal_patterns: Vec<SeasonalPattern>,
    pub anomalies: Vec<TrendAnomaly>,
    pub predictions: Vec<TrendPrediction>,
    pub trend_summary: TrendSummary,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Trend {
    pub metric_name: String,
    pub direction: TrendDirection,
    pub strength: f32,
    pub duration_secs: i64,
    pub changes: Vec<TrendChange>,
    pub confidence: f32,
    pub significance: TrendSignificance,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum TrendDirection {
    Increasing,
    Decreasing,
    Stable,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TrendChange {
    pub timestamp: DateTime<Utc>,
    pub from_value: f32,
    pub to_value: f32,
    pub change_type: TrendChangeType,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum TrendChangeType {
    Spike,
    Drop,
    GradualIncrease,
    GradualDecrease,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum TrendSignificance {
    High,
    Medium,
    Low,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SeasonalPattern {
    pub pattern_type: String,
    pub period_secs: i64,
    pub amplitude: f32,
    pub confidence: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TrendAnomaly {
    pub timestamp: DateTime<Utc>,
    pub metric_name: String,
    pub expected_value: f32,
    pub actual_value: f32,
    pub deviation: f32,
    pub severity: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TrendPrediction {
    pub metric_name: String,
    pub predicted_value: f32,
    pub confidence_interval: (f32, f32),
    pub horizon_secs: i64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TrendSummary {
    pub total_trends: usize,
    pub increasing_trends: usize,
    pub decreasing_trends: usize,
    pub stable_trends: usize,
    pub key_findings: Vec<String>,
}

/// Health Analysis Result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HealthAnalysisResult {
    pub overall_health_score: f32,
    pub component_health: Vec<ComponentHealth>,
    pub health_issues: Vec<HealthIssue>,
    pub health_trend: TrendDirection,
    pub recommendations: Vec<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ComponentHealth {
    pub component_name: String,
    pub health_score: f32,
    pub status: HealthStatus,
    pub issues: Vec<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum HealthStatus {
    Healthy,
    Degraded,
    Critical,
    Unknown,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HealthIssue {
    pub severity: String,
    pub component: String,
    pub description: String,
    pub detected_at: DateTime<Utc>,
}

/// Other analysis result types (simplified)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EfficiencyAnalysisResult {
    pub resource_efficiency: f32,
    pub optimization_opportunities: Vec<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PatternAnalysisResult {
    pub patterns: Vec<DetectedPattern>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DetectedPattern {
    pub pattern_type: String,
    pub description: String,
    pub frequency: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SecurityAnalysisResult {
    pub security_score: f32,
    pub vulnerabilities: Vec<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PredictiveAnalysisResult {
    pub predictions: Vec<TrendPrediction>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ComparativeAnalysisResult {
    pub baseline_summary: String,
    pub current_summary: String,
    pub key_differences: Vec<String>,
    pub performance_delta: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DiagnosticsResult {
    pub diagnostic_findings: Vec<String>,
}

/// Insights and Recommendations
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AnalysisInsight {
    pub title: String,
    pub description: String,
    pub priority: InsightPriority,
    pub metric_change: Option<MetricChange>,
    pub affected_components: Vec<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum InsightPriority {
    Critical,
    High,
    Medium,
    Low,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MetricChange {
    pub metric_name: String,
    pub from_value: f32,
    pub to_value: f32,
    pub change_percent: f32,
}

impl MetricChange {
    pub fn is_improvement(&self) -> bool {
        // Simple heuristic: for most metrics, decrease is improvement (latency, errors, etc.)
        // This should be metric-specific in production
        self.change_percent < 0.0
    }

    pub fn format_change(&self) -> String {
        format!("{:.1}%", self.change_percent.abs())
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AnalysisRecommendation {
    pub title: String,
    pub description: String,
    pub priority: RecommendationPriority,
    pub expected_impact: String,
    pub action_items: Vec<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum RecommendationPriority {
    Critical,
    High,
    Medium,
    Low,
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct OverallScores {
    pub performance_score: f32,
    pub health_score: f32,
    pub efficiency_score: f32,
    pub reliability_score: f32,
    pub overall_score: f32,
}

/// Supporting types
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TimeSeriesPoint {
    pub timestamp: DateTime<Utc>,
    pub value: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AnalysisMetadata {
    pub analysis_duration_secs: f32,
    pub data_points_analyzed: usize,
    pub time_range: TimeWindow,
}

/// Data collection types
#[derive(Debug, Clone)]
pub struct DataCollection {
    pub metrics: Vec<MetricPoint>,
    pub time_range: TimeWindow,
}

#[derive(Debug, Clone)]
pub struct MetricPoint {
    pub metric_name: String,
    pub value: f32,
    pub timestamp: DateTime<Utc>,
}

impl MetricPoint {
    pub fn new(metric_name: String, value: f32) -> Self {
        Self {
            metric_name,
            value,
            timestamp: Utc::now(),
        }
    }
}
