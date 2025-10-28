// Inspection Types for Issue #17

use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Result of resource inspection
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InspectionResult {
    pub resource: ResolvedResource,
    pub timestamp: DateTime<Utc>,
    pub analysis_depth: u8,
    pub complexity_score: f32,
    pub health_score: HealthScore,
    pub performance_metrics: PerformanceMetrics,
    pub error_summary: ErrorSummary,
    pub recommendations: Vec<Recommendation>,
    pub suggested_actions: Vec<SuggestedAction>,
}

/// Resolved resource with metadata
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ResolvedResource {
    pub resource_type: String,
    pub identifier: String,
    pub status: ResourceStatus,
    pub created_at: Option<DateTime<Utc>>,
    pub metadata: HashMap<String, String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum ResourceStatus {
    Running,
    Stopped,
    Error,
    Unknown,
}

/// Health score for resource
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HealthScore {
    pub overall_score: f32, // 0-100
    pub status: HealthStatus,
    pub component_scores: HashMap<String, f32>,
    pub issues: Vec<HealthIssue>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum HealthStatus {
    Healthy,
    Warning,
    Critical,
    Unknown,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HealthIssue {
    pub component: String,
    pub severity: IssueSeverity,
    pub description: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum IssueSeverity {
    Critical,
    High,
    Medium,
    Low,
}

/// Performance metrics
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PerformanceMetrics {
    pub cpu_usage: Option<f32>,
    pub memory_mb: Option<f32>,
    pub throughput: Option<f32>,
    pub latency_ms: Option<f32>,
    pub error_rate: Option<f32>,
    pub issues: Vec<PerformanceIssue>,
}

impl Default for PerformanceMetrics {
    fn default() -> Self {
        Self {
            cpu_usage: None,
            memory_mb: None,
            throughput: None,
            latency_ms: None,
            error_rate: None,
            issues: Vec::new(),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PerformanceIssue {
    pub severity: IssueSeverity,
    pub description: String,
    pub recommendation: Option<String>,
}

/// Error summary
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ErrorSummary {
    pub total_errors: usize,
    pub recent_errors: Vec<ErrorRecord>,
    pub error_patterns: Vec<ErrorPattern>,
}

impl Default for ErrorSummary {
    fn default() -> Self {
        Self {
            total_errors: 0,
            recent_errors: Vec::new(),
            error_patterns: Vec::new(),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ErrorRecord {
    pub timestamp: DateTime<Utc>,
    pub error_type: String,
    pub message: String,
    pub context: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ErrorPattern {
    pub pattern_type: String,
    pub frequency: usize,
    pub description: String,
}

/// Recommendation for improvements
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Recommendation {
    pub priority: RecommendationPriority,
    pub title: String,
    pub description: String,
    pub suggested_command: Option<String>,
    pub impact: ImpactLevel,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum RecommendationPriority {
    Critical,
    High,
    Medium,
    Low,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum ImpactLevel {
    High,
    Medium,
    Low,
}

/// Suggested action
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SuggestedAction {
    pub action_type: ActionType,
    pub description: String,
    pub command: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum ActionType {
    Restart,
    ScaleUp,
    Investigate,
    Optimize,
    Monitor,
}

impl Default for InspectionResult {
    fn default() -> Self {
        Self {
            resource: ResolvedResource {
                resource_type: "unknown".to_string(),
                identifier: "".to_string(),
                status: ResourceStatus::Unknown,
                created_at: None,
                metadata: HashMap::new(),
            },
            timestamp: Utc::now(),
            analysis_depth: 1,
            complexity_score: 0.0,
            health_score: HealthScore {
                overall_score: 100.0,
                status: HealthStatus::Unknown,
                component_scores: HashMap::new(),
                issues: Vec::new(),
            },
            performance_metrics: PerformanceMetrics {
                cpu_usage: None,
                memory_mb: None,
                throughput: None,
                latency_ms: None,
                error_rate: None,
                issues: Vec::new(),
            },
            error_summary: ErrorSummary {
                total_errors: 0,
                recent_errors: Vec::new(),
                error_patterns: Vec::new(),
            },
            recommendations: Vec::new(),
            suggested_actions: Vec::new(),
        }
    }
}
