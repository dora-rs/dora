// Performance Monitoring Utilities
// Tracks and analyzes CLI command performance

use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::time::{Duration, Instant};

/// Performance metrics for a command execution
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PerformanceMetrics {
    pub command_name: String,
    pub execution_time: Duration,
    pub memory_usage: Option<u64>,
    pub cpu_usage: Option<f32>,
    pub io_operations: u64,
    pub network_requests: u64,
    pub cache_hits: u64,
    pub cache_misses: u64,
    pub status: ExecutionStatus,
}

/// Execution status
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ExecutionStatus {
    Success,
    Failed,
    Timeout,
    Cancelled,
}

/// Performance threshold configuration
#[derive(Debug, Clone)]
pub struct PerformanceThresholds {
    pub max_execution_time: Duration,
    pub max_memory_mb: u64,
    pub max_cpu_percent: f32,
    pub warn_execution_time: Duration,
    pub warn_memory_mb: u64,
}

impl Default for PerformanceThresholds {
    fn default() -> Self {
        Self {
            max_execution_time: Duration::from_secs(30),
            max_memory_mb: 500,
            max_cpu_percent: 80.0,
            warn_execution_time: Duration::from_secs(5),
            warn_memory_mb: 200,
        }
    }
}

/// Performance violation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PerformanceViolation {
    pub metric_type: MetricType,
    pub severity: ViolationSeverity,
    pub actual_value: f64,
    pub threshold_value: f64,
    pub description: String,
    pub recommendation: String,
}

/// Metric types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum MetricType {
    ExecutionTime,
    MemoryUsage,
    CpuUsage,
    IoOperations,
    NetworkLatency,
    CacheEfficiency,
}

/// Violation severity
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ViolationSeverity {
    Critical,
    Warning,
    Info,
}

/// Performance analysis result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PerformanceAnalysis {
    pub metrics: PerformanceMetrics,
    pub violations: Vec<PerformanceViolation>,
    pub score: f32,
    pub grade: PerformanceGrade,
    pub suggestions: Vec<String>,
}

/// Performance grade
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum PerformanceGrade {
    Excellent,
    Good,
    Fair,
    Poor,
    Critical,
}

/// Performance monitor
#[derive(Debug)]
pub struct PerformanceMonitor {
    thresholds: PerformanceThresholds,
    metrics_history: HashMap<String, Vec<PerformanceMetrics>>,
    start_time: Option<Instant>,
}

impl PerformanceMonitor {
    pub fn new() -> Self {
        Self {
            thresholds: PerformanceThresholds::default(),
            metrics_history: HashMap::new(),
            start_time: None,
        }
    }

    pub fn with_thresholds(thresholds: PerformanceThresholds) -> Self {
        Self {
            thresholds,
            metrics_history: HashMap::new(),
            start_time: None,
        }
    }

    /// Start monitoring a command
    pub fn start_monitoring(&mut self) {
        self.start_time = Some(Instant::now());
    }

    /// Stop monitoring and collect metrics
    pub fn stop_monitoring(
        &mut self,
        command_name: &str,
        status: ExecutionStatus,
    ) -> PerformanceMetrics {
        let execution_time = self
            .start_time
            .map(|start| start.elapsed())
            .unwrap_or(Duration::from_secs(0));

        let metrics = PerformanceMetrics {
            command_name: command_name.to_string(),
            execution_time,
            memory_usage: self.get_memory_usage(),
            cpu_usage: self.get_cpu_usage(),
            io_operations: 0, // Would be tracked during execution
            network_requests: 0,
            cache_hits: 0,
            cache_misses: 0,
            status,
        };

        // Store in history
        self.metrics_history
            .entry(command_name.to_string())
            .or_default()
            .push(metrics.clone());

        self.start_time = None;

        metrics
    }

    /// Analyze performance metrics
    pub fn analyze(&self, metrics: &PerformanceMetrics) -> PerformanceAnalysis {
        let mut violations = Vec::new();
        let mut suggestions = Vec::new();

        // Check execution time
        if metrics.execution_time > self.thresholds.max_execution_time {
            violations.push(PerformanceViolation {
                metric_type: MetricType::ExecutionTime,
                severity: ViolationSeverity::Critical,
                actual_value: metrics.execution_time.as_secs_f64(),
                threshold_value: self.thresholds.max_execution_time.as_secs_f64(),
                description: format!(
                    "Execution time {:.2}s exceeds maximum {:.2}s",
                    metrics.execution_time.as_secs_f64(),
                    self.thresholds.max_execution_time.as_secs_f64()
                ),
                recommendation: "Consider optimizing command logic or adding caching".to_string(),
            });
            suggestions.push(
                "Execution time is too high. Consider adding progress indicators.".to_string(),
            );
        } else if metrics.execution_time > self.thresholds.warn_execution_time {
            violations.push(PerformanceViolation {
                metric_type: MetricType::ExecutionTime,
                severity: ViolationSeverity::Warning,
                actual_value: metrics.execution_time.as_secs_f64(),
                threshold_value: self.thresholds.warn_execution_time.as_secs_f64(),
                description: format!(
                    "Execution time {:.2}s approaching threshold",
                    metrics.execution_time.as_secs_f64()
                ),
                recommendation: "Monitor performance and consider optimization".to_string(),
            });
        }

        // Check memory usage
        if let Some(memory_mb) = metrics.memory_usage {
            if memory_mb > self.thresholds.max_memory_mb {
                violations.push(PerformanceViolation {
                    metric_type: MetricType::MemoryUsage,
                    severity: ViolationSeverity::Critical,
                    actual_value: memory_mb as f64,
                    threshold_value: self.thresholds.max_memory_mb as f64,
                    description: format!(
                        "Memory usage {}MB exceeds maximum {}MB",
                        memory_mb, self.thresholds.max_memory_mb
                    ),
                    recommendation: "Reduce memory allocations or implement streaming".to_string(),
                });
                suggestions.push(
                    "High memory usage detected. Consider streaming or pagination.".to_string(),
                );
            }
        }

        // Check CPU usage
        if let Some(cpu) = metrics.cpu_usage {
            if cpu > self.thresholds.max_cpu_percent {
                violations.push(PerformanceViolation {
                    metric_type: MetricType::CpuUsage,
                    severity: ViolationSeverity::Warning,
                    actual_value: cpu as f64,
                    threshold_value: self.thresholds.max_cpu_percent as f64,
                    description: format!("CPU usage {cpu:.1}% is high"),
                    recommendation: "Optimize CPU-intensive operations".to_string(),
                });
            }
        }

        // Check cache efficiency
        let total_cache_ops = metrics.cache_hits + metrics.cache_misses;
        if total_cache_ops > 0 {
            let cache_hit_rate = (metrics.cache_hits as f32 / total_cache_ops as f32) * 100.0;
            if cache_hit_rate < 50.0 {
                violations.push(PerformanceViolation {
                    metric_type: MetricType::CacheEfficiency,
                    severity: ViolationSeverity::Info,
                    actual_value: cache_hit_rate as f64,
                    threshold_value: 50.0,
                    description: format!("Cache hit rate {cache_hit_rate:.1}% is low"),
                    recommendation: "Improve caching strategy".to_string(),
                });
                suggestions.push("Low cache efficiency. Consider pre-warming caches.".to_string());
            }
        }

        let score = self.calculate_score(&violations);
        let grade = self.calculate_grade(score);

        PerformanceAnalysis {
            metrics: metrics.clone(),
            violations,
            score,
            grade,
            suggestions,
        }
    }

    /// Get historical metrics for a command
    pub fn get_history(&self, command_name: &str) -> Option<&Vec<PerformanceMetrics>> {
        self.metrics_history.get(command_name)
    }

    /// Get average execution time for a command
    pub fn get_average_execution_time(&self, command_name: &str) -> Option<Duration> {
        self.metrics_history.get(command_name).and_then(|history| {
            if history.is_empty() {
                return None;
            }
            let total: Duration = history.iter().map(|m| m.execution_time).sum();
            Some(total / history.len() as u32)
        })
    }

    /// Compare current metrics to historical baseline
    pub fn compare_to_baseline(&self, metrics: &PerformanceMetrics) -> Option<f32> {
        let avg = self.get_average_execution_time(&metrics.command_name)?;
        let ratio = metrics.execution_time.as_secs_f64() / avg.as_secs_f64();
        Some((ratio - 1.0) as f32 * 100.0) // Return percentage change
    }

    fn get_memory_usage(&self) -> Option<u64> {
        // In a real implementation, this would use platform-specific APIs
        // For now, return mock data
        Some(150) // MB
    }

    fn get_cpu_usage(&self) -> Option<f32> {
        // In a real implementation, this would use platform-specific APIs
        // For now, return mock data
        Some(25.0) // percent
    }

    fn calculate_score(&self, violations: &[PerformanceViolation]) -> f32 {
        let base_score: f32 = 100.0;
        let mut penalty: f32 = 0.0;

        for violation in violations {
            penalty += match violation.severity {
                ViolationSeverity::Critical => 30.0,
                ViolationSeverity::Warning => 15.0,
                ViolationSeverity::Info => 5.0,
            };
        }

        (base_score - penalty).max(0.0)
    }

    fn calculate_grade(&self, score: f32) -> PerformanceGrade {
        match score {
            s if s >= 90.0 => PerformanceGrade::Excellent,
            s if s >= 75.0 => PerformanceGrade::Good,
            s if s >= 60.0 => PerformanceGrade::Fair,
            s if s >= 40.0 => PerformanceGrade::Poor,
            _ => PerformanceGrade::Critical,
        }
    }

    /// Generate performance report
    pub fn generate_report(&self) -> PerformanceReport {
        let mut command_reports = Vec::new();

        for (command_name, history) in &self.metrics_history {
            if history.is_empty() {
                continue;
            }

            let avg_execution_time = self.get_average_execution_time(command_name).unwrap();
            let total_executions = history.len();
            let successful_executions = history
                .iter()
                .filter(|m| matches!(m.status, ExecutionStatus::Success))
                .count();
            let success_rate = (successful_executions as f32 / total_executions as f32) * 100.0;

            command_reports.push(CommandPerformanceReport {
                command_name: command_name.clone(),
                total_executions,
                successful_executions,
                success_rate,
                average_execution_time: avg_execution_time,
                performance_trend: self.calculate_trend(history),
            });
        }

        PerformanceReport {
            total_commands: self.metrics_history.len(),
            command_reports,
            overall_health: self.calculate_overall_health(),
        }
    }

    fn calculate_trend(&self, history: &[PerformanceMetrics]) -> PerformanceTrend {
        if history.len() < 2 {
            return PerformanceTrend::Stable;
        }

        let recent_count = history.len().min(5);
        let recent = &history[history.len() - recent_count..];
        let older = &history[..history.len() - recent_count];

        let recent_avg: Duration =
            recent.iter().map(|m| m.execution_time).sum::<Duration>() / recent_count as u32;
        let older_avg: Duration =
            older.iter().map(|m| m.execution_time).sum::<Duration>() / older.len() as u32;

        let change_percent = ((recent_avg.as_secs_f64() - older_avg.as_secs_f64())
            / older_avg.as_secs_f64())
            * 100.0;

        match change_percent {
            c if c > 10.0 => PerformanceTrend::Degrading,
            c if c < -10.0 => PerformanceTrend::Improving,
            _ => PerformanceTrend::Stable,
        }
    }

    fn calculate_overall_health(&self) -> HealthStatus {
        let total_metrics: Vec<_> = self
            .metrics_history
            .values()
            .flat_map(|v| v.iter())
            .collect();
        if total_metrics.is_empty() {
            return HealthStatus::Unknown;
        }

        let success_count = total_metrics
            .iter()
            .filter(|m| matches!(m.status, ExecutionStatus::Success))
            .count();
        let success_rate = (success_count as f32 / total_metrics.len() as f32) * 100.0;

        match success_rate {
            r if r >= 95.0 => HealthStatus::Excellent,
            r if r >= 85.0 => HealthStatus::Good,
            r if r >= 70.0 => HealthStatus::Fair,
            _ => HealthStatus::Poor,
        }
    }
}

impl Default for PerformanceMonitor {
    fn default() -> Self {
        Self::new()
    }
}

/// Performance report
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PerformanceReport {
    pub total_commands: usize,
    pub command_reports: Vec<CommandPerformanceReport>,
    pub overall_health: HealthStatus,
}

/// Command-specific performance report
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CommandPerformanceReport {
    pub command_name: String,
    pub total_executions: usize,
    pub successful_executions: usize,
    pub success_rate: f32,
    pub average_execution_time: Duration,
    pub performance_trend: PerformanceTrend,
}

/// Performance trend
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum PerformanceTrend {
    Improving,
    Stable,
    Degrading,
}

/// Health status
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum HealthStatus {
    Excellent,
    Good,
    Fair,
    Poor,
    Unknown,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_performance_monitor_basic() {
        let mut monitor = PerformanceMonitor::new();
        monitor.start_monitoring();

        std::thread::sleep(Duration::from_millis(100));

        let metrics = monitor.stop_monitoring("test_command", ExecutionStatus::Success);
        assert!(metrics.execution_time >= Duration::from_millis(100));
        assert_eq!(metrics.command_name, "test_command");
    }

    #[test]
    fn test_performance_analysis() {
        let monitor = PerformanceMonitor::new();
        let metrics = PerformanceMetrics {
            command_name: "slow_command".to_string(),
            execution_time: Duration::from_secs(35), // Exceeds threshold
            memory_usage: Some(150),
            cpu_usage: Some(25.0),
            io_operations: 0,
            network_requests: 0,
            cache_hits: 0,
            cache_misses: 0,
            status: ExecutionStatus::Success,
        };

        let analysis = monitor.analyze(&metrics);
        assert!(!analysis.violations.is_empty());
        // Score with 1 critical violation (30 penalty) = 70, which is "Fair" grade
        assert!(matches!(
            analysis.grade,
            PerformanceGrade::Fair | PerformanceGrade::Good
        ));
    }

    #[test]
    fn test_performance_grade_calculation() {
        let monitor = PerformanceMonitor::new();

        assert_eq!(monitor.calculate_grade(95.0), PerformanceGrade::Excellent);
        assert_eq!(monitor.calculate_grade(80.0), PerformanceGrade::Good);
        assert_eq!(monitor.calculate_grade(65.0), PerformanceGrade::Fair);
        assert_eq!(monitor.calculate_grade(45.0), PerformanceGrade::Poor);
        assert_eq!(monitor.calculate_grade(20.0), PerformanceGrade::Critical);
    }

    #[test]
    fn test_average_execution_time() {
        let mut monitor = PerformanceMonitor::new();

        // Add some metrics
        monitor.metrics_history.insert(
            "test".to_string(),
            vec![
                PerformanceMetrics {
                    command_name: "test".to_string(),
                    execution_time: Duration::from_secs(1),
                    memory_usage: None,
                    cpu_usage: None,
                    io_operations: 0,
                    network_requests: 0,
                    cache_hits: 0,
                    cache_misses: 0,
                    status: ExecutionStatus::Success,
                },
                PerformanceMetrics {
                    command_name: "test".to_string(),
                    execution_time: Duration::from_secs(3),
                    memory_usage: None,
                    cpu_usage: None,
                    io_operations: 0,
                    network_requests: 0,
                    cache_hits: 0,
                    cache_misses: 0,
                    status: ExecutionStatus::Success,
                },
            ],
        );

        let avg = monitor.get_average_execution_time("test").unwrap();
        assert_eq!(avg, Duration::from_secs(2));
    }
}
