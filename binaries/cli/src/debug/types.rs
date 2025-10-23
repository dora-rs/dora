// Debug Types for Issue #18

use crate::cli::commands::{DebugMode, DebugFocus};
use chrono::{DateTime, Utc};
use serde::{Serialize, Deserialize};
use std::collections::HashMap;
use uuid::Uuid;

/// Debug session containing all debugging state
#[derive(Debug, Clone)]
pub struct DebugSession {
    pub session_id: Uuid,
    pub start_time: DateTime<Utc>,
    pub debug_target: DebugTarget,
    pub mode: DebugMode,
    pub focus: Option<DebugFocus>,
    pub configuration: DebugConfiguration,
}

/// Target being debugged
#[derive(Debug, Clone)]
pub enum DebugTarget {
    System,
    Dataflow(String),
    Node(String),
    Component(String),
    Auto,
}

/// Debug configuration
#[derive(Debug, Clone)]
pub struct DebugConfiguration {
    pub auto_detect: bool,
    pub live_monitoring: bool,
    pub capture_artifacts: bool,
    pub timeout: String,
    pub history_window: String,
    pub output_dir: Option<std::path::PathBuf>,
}

/// Detected issue during debugging
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DetectedIssue {
    pub issue_id: String,
    pub issue_type: IssueType,
    pub severity: IssueSeverity,
    pub confidence: f32,
    pub title: String,
    pub description: String,
    pub affected_components: Vec<String>,
    pub symptoms: Vec<Symptom>,
    pub possible_causes: Vec<PossibleCause>,
    pub suggested_actions: Vec<SuggestedAction>,
    pub debugging_hints: Vec<DebuggingHint>,
    pub first_detected: DateTime<Utc>,
    pub related_issues: Vec<String>,
}

/// Types of issues that can be detected
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum IssueType {
    PerformanceDegradation,
    MemoryLeak,
    NetworkConnectivity,
    DataflowStalled,
    NodeCrashing,
    ConfigurationError,
    DependencyFailure,
    ResourceExhaustion,
    MessageLoss,
    LatencySpike,
    ErrorRateIncrease,
    DeadlockDetected,
}

/// Severity levels for detected issues
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum IssueSeverity {
    Critical,
    High,
    Medium,
    Low,
}

/// Symptom of an issue
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Symptom {
    pub description: String,
    pub metric_value: Option<f32>,
    pub timestamp: DateTime<Utc>,
}

/// Possible cause of an issue
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PossibleCause {
    pub description: String,
    pub likelihood: f32,
    pub investigation_steps: Vec<String>,
}

/// Suggested action to resolve an issue
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SuggestedAction {
    pub action: String,
    pub command: Option<String>,
    pub urgency: ActionUrgency,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum ActionUrgency {
    Critical,
    High,
    Medium,
    Low,
}

/// Debugging hint for interactive exploration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DebuggingHint {
    pub hint: String,
    pub interactive_features: Vec<String>,
}

/// Complexity analysis for debug session
#[derive(Debug, Clone)]
pub struct DebugComplexityAnalysis {
    pub overall_score: f32,
    pub issue_complexity: f32,
    pub system_complexity: f32,
    pub data_complexity: f32,
    pub factors: Vec<ComplexityFactor>,
}

#[derive(Debug, Clone)]
pub struct ComplexityFactor {
    pub factor_type: FactorType,
    pub impact: f32,
    pub description: String,
    pub evidence: Vec<String>,
}

#[derive(Debug, Clone)]
pub enum FactorType {
    IssueComplexity,
    SystemComplexity,
    DataflowComplexity,
    DataComplexity,
    NetworkComplexity,
}

/// System metrics for debugging
#[derive(Debug, Clone)]
pub struct SystemMetrics {
    pub cpu_usage_percent: f32,
    pub memory_usage_mb: f32,
    pub disk_io_mbps: f32,
    pub network_io_mbps: f32,
    pub active_processes: usize,
    pub timestamp: DateTime<Utc>,
}

/// Baseline metrics for comparison
#[derive(Debug, Clone)]
pub struct BaselineMetrics {
    pub average_cpu_usage: f32,
    pub average_memory_usage: f32,
    pub average_throughput: f32,
    pub time_window: String,
}

impl Default for DebugSession {
    fn default() -> Self {
        Self {
            session_id: Uuid::new_v4(),
            start_time: Utc::now(),
            debug_target: DebugTarget::Auto,
            mode: DebugMode::default(),
            focus: None,
            configuration: DebugConfiguration::default(),
        }
    }
}

impl Default for DebugConfiguration {
    fn default() -> Self {
        Self {
            auto_detect: true,
            live_monitoring: false,
            capture_artifacts: false,
            timeout: "30m".to_string(),
            history_window: "1h".to_string(),
            output_dir: None,
        }
    }
}

impl Default for BaselineMetrics {
    fn default() -> Self {
        Self {
            average_cpu_usage: 30.0,
            average_memory_usage: 256.0,
            average_throughput: 100.0,
            time_window: "1h".to_string(),
        }
    }
}
