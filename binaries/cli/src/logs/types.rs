// Log Streaming Types for Issue #20

use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use uuid::Uuid;

/// Log session for tracking streaming sessions
#[derive(Debug, Clone)]
pub struct LogSession {
    pub session_id: Uuid,
    pub start_time: DateTime<Utc>,
    pub log_target: LogTarget,
    pub time_filter: TimeFilter,
    pub filter_config: LogFilterConfig,
    pub streaming_config: StreamingConfig,
}

/// Log target specification
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum LogTarget {
    System,
    Dataflow(String),
    Node(String),
    Component(String),
}

/// Time-based log filtering
#[derive(Debug, Clone)]
pub struct TimeFilter {
    pub since: Option<DateTime<Utc>>,
    pub until: Option<DateTime<Utc>>,
}

/// Log filtering configuration
#[derive(Debug, Clone)]
pub struct LogFilterConfig {
    pub levels: Vec<LogLevel>,
    pub include_patterns: Vec<String>,
    pub exclude_patterns: Vec<String>,
    pub smart_filtering: bool,
    pub errors_only: bool,
    pub search_patterns: Vec<String>,
}

/// Streaming configuration
#[derive(Debug, Clone)]
pub struct StreamingConfig {
    pub follow: bool,
    pub tail: usize,
    pub rate_limit: Option<u32>,
    pub buffer_size: usize,
}

/// Log level enumeration
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, clap::ValueEnum)]
pub enum LogLevel {
    Trace,
    Debug,
    Info,
    Warn,
    Error,
    Fatal,
}

impl std::fmt::Display for LogLevel {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            LogLevel::Trace => write!(f, "TRACE"),
            LogLevel::Debug => write!(f, "DEBUG"),
            LogLevel::Info => write!(f, "INFO"),
            LogLevel::Warn => write!(f, "WARN"),
            LogLevel::Error => write!(f, "ERROR"),
            LogLevel::Fatal => write!(f, "FATAL"),
        }
    }
}

/// Log entry structure
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LogEntry {
    pub timestamp: DateTime<Utc>,
    pub level: LogLevel,
    pub source: Option<String>,
    pub message: String,
    pub fields: HashMap<String, String>,
}

/// Detected log pattern
#[derive(Debug, Clone)]
pub struct DetectedPattern {
    pub pattern_id: String,
    pub pattern_type: LogPatternType,
    pub severity: PatternSeverity,
    pub confidence: f32,
    pub title: String,
    pub description: String,
    pub sample_logs: Vec<LogEntry>,
    pub frequency: PatternFrequency,
    pub first_seen: DateTime<Utc>,
    pub last_seen: DateTime<Utc>,
    pub suggested_actions: Vec<LogAction>,
    pub escalation_trigger: bool,
}

/// Log pattern types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LogPatternType {
    ErrorSpike,
    RepeatingError,
    PerformanceDegradation,
    SecurityAlert,
    UnusualActivity,
    SystemFailure,
    ResourceExhaustion,
    NetworkIssue,
    DataCorruption,
    ConfigurationError,
}

/// Pattern severity
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PatternSeverity {
    Critical,
    High,
    Medium,
    Low,
    Info,
}

/// Pattern frequency information
#[derive(Debug, Clone)]
pub struct PatternFrequency {
    pub count: u32,
    pub rate_per_minute: f32,
    pub trend: FrequencyTrend,
}

/// Frequency trend
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrequencyTrend {
    Increasing,
    Decreasing,
    Stable,
    Spiking,
}

/// Suggested log action
#[derive(Debug, Clone)]
pub struct LogAction {
    pub action: String,
    pub command: Option<String>,
    pub urgency: ActionUrgency,
}

/// Action urgency level
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ActionUrgency {
    Critical,
    High,
    Medium,
    Low,
}

/// Escalation trigger for TUI launch
#[derive(Debug, Clone)]
pub struct EscalationTrigger {
    pub trigger_type: EscalationTriggerType,
    pub severity: PatternSeverity,
    pub reason: String,
    pub detected_pattern: Option<DetectedPattern>,
    pub suggested_action: String,
}

/// Escalation trigger types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EscalationTriggerType {
    PatternDetected,
    AnomalyDetected,
    VolumeThreshold,
    ErrorRateExceeded,
}

/// Log format options
#[derive(Debug, Clone, Copy, PartialEq, Eq, clap::ValueEnum, Default)]
pub enum LogFormat {
    #[default]
    Text,
    Json,
    Compact,
    Detailed,
    Colored,
}

/// Error group for pattern detection
#[derive(Debug, Clone)]
pub struct ErrorGroup {
    pub signature: String,
    pub count: u32,
    pub similarity_score: f32,
    pub sample_logs: Vec<LogEntry>,
    pub rate_per_minute: f32,
    pub trend: FrequencyTrend,
    pub first_seen: DateTime<Utc>,
    pub last_seen: DateTime<Utc>,
}

/// Time window for analysis
#[derive(Debug, Clone)]
pub struct TimeWindow {
    pub start: DateTime<Utc>,
    pub end: DateTime<Utc>,
    pub duration_secs: i64,
}

/// Log analysis metrics
#[derive(Debug, Clone)]
pub struct LogMetrics {
    pub total_logs: u64,
    pub error_count: u64,
    pub warning_count: u64,
    pub info_count: u64,
    pub logs_per_second: f32,
    pub error_rate: f32,
    pub unique_error_types: usize,
}

/// Streaming statistics
#[derive(Debug, Clone)]
pub struct StreamingStats {
    pub logs_processed: u64,
    pub logs_filtered: u64,
    pub logs_displayed: u64,
    pub patterns_detected: u32,
    pub escalations_triggered: u32,
    pub start_time: DateTime<Utc>,
    pub duration_secs: i64,
}

impl LogSession {
    pub fn new(
        log_target: LogTarget,
        time_filter: TimeFilter,
        filter_config: LogFilterConfig,
        streaming_config: StreamingConfig,
    ) -> Self {
        Self {
            session_id: Uuid::new_v4(),
            start_time: Utc::now(),
            log_target,
            time_filter,
            filter_config,
            streaming_config,
        }
    }
}

impl LogEntry {
    pub fn new(level: LogLevel, message: String) -> Self {
        Self {
            timestamp: Utc::now(),
            level,
            source: None,
            message,
            fields: HashMap::new(),
        }
    }

    pub fn with_source(mut self, source: String) -> Self {
        self.source = Some(source);
        self
    }

    pub fn with_field(mut self, key: String, value: String) -> Self {
        self.fields.insert(key, value);
        self
    }

    pub fn matches_level(&self, levels: &[LogLevel]) -> bool {
        levels.contains(&self.level)
    }

    pub fn matches_pattern(&self, pattern: &str) -> bool {
        self.message.contains(pattern) || self.source.as_ref().is_some_and(|s| s.contains(pattern))
    }
}

impl Default for StreamingConfig {
    fn default() -> Self {
        Self {
            follow: false,
            tail: 100,
            rate_limit: None,
            buffer_size: 1000,
        }
    }
}

impl Default for LogFilterConfig {
    fn default() -> Self {
        Self {
            levels: vec![
                LogLevel::Info,
                LogLevel::Warn,
                LogLevel::Error,
                LogLevel::Fatal,
            ],
            include_patterns: Vec::new(),
            exclude_patterns: Vec::new(),
            smart_filtering: false,
            errors_only: false,
            search_patterns: Vec::new(),
        }
    }
}
