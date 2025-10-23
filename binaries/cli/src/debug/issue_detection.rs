// Automatic Issue Detection for Issue #18

use super::types::*;
use chrono::Utc;
use eyre::Result;

/// Automatic issue detector that runs multiple specialized detectors
#[derive(Debug)]
pub struct AutomaticIssueDetector {
    performance_detector: PerformanceIssueDetector,
    dataflow_detector: DataflowIssueDetector,
    network_detector: NetworkIssueDetector,
}

impl AutomaticIssueDetector {
    pub fn new() -> Self {
        Self {
            performance_detector: PerformanceIssueDetector::new(),
            dataflow_detector: DataflowIssueDetector::new(),
            network_detector: NetworkIssueDetector::new(),
        }
    }

    /// Detect all issues in the debug session
    pub async fn detect_issues(&self, debug_session: &DebugSession) -> Result<Vec<DetectedIssue>> {
        let mut all_issues = Vec::new();

        // Run performance detection
        let perf_issues = self.performance_detector.detect_issues(debug_session).await?;
        all_issues.extend(perf_issues);

        // Run dataflow detection if target is dataflow
        if matches!(debug_session.debug_target, DebugTarget::Dataflow(_)) {
            let dataflow_issues = self.dataflow_detector.detect_issues(debug_session).await?;
            all_issues.extend(dataflow_issues);
        }

        // Run network detection
        let network_issues = self.network_detector.detect_issues(debug_session).await?;
        all_issues.extend(network_issues);

        // Sort by severity (Critical first)
        all_issues.sort_by(|a, b| {
            let a_severity = match a.severity {
                IssueSeverity::Critical => 4,
                IssueSeverity::High => 3,
                IssueSeverity::Medium => 2,
                IssueSeverity::Low => 1,
            };
            let b_severity = match b.severity {
                IssueSeverity::Critical => 4,
                IssueSeverity::High => 3,
                IssueSeverity::Medium => 2,
                IssueSeverity::Low => 1,
            };
            b_severity.cmp(&a_severity)
        });

        Ok(all_issues)
    }
}

impl Default for AutomaticIssueDetector {
    fn default() -> Self {
        Self::new()
    }
}

/// Performance issue detector
#[derive(Debug)]
pub struct PerformanceIssueDetector {
    baseline_metrics: BaselineMetrics,
}

impl PerformanceIssueDetector {
    pub fn new() -> Self {
        Self {
            baseline_metrics: BaselineMetrics::default(),
        }
    }

    pub async fn detect_issues(&self, _debug_session: &DebugSession) -> Result<Vec<DetectedIssue>> {
        let mut issues = Vec::new();

        // Simulate getting current metrics
        let current_metrics = self.get_current_metrics().await?;

        // Check for CPU issues
        if let Some(cpu_issue) = self.analyze_cpu_performance(&current_metrics, &self.baseline_metrics) {
            issues.push(cpu_issue);
        }

        // Check for memory issues
        if let Some(memory_issue) = self.analyze_memory_performance(&current_metrics, &self.baseline_metrics) {
            issues.push(memory_issue);
        }

        Ok(issues)
    }

    async fn get_current_metrics(&self) -> Result<SystemMetrics> {
        // Mock implementation - in production, this would query actual system metrics
        Ok(SystemMetrics {
            cpu_usage_percent: 85.0,  // Simulating high CPU
            memory_usage_mb: 512.0,
            disk_io_mbps: 50.0,
            network_io_mbps: 100.0,
            active_processes: 45,
            timestamp: Utc::now(),
        })
    }

    fn analyze_cpu_performance(
        &self,
        current: &SystemMetrics,
        baseline: &BaselineMetrics,
    ) -> Option<DetectedIssue> {
        let cpu_usage = current.cpu_usage_percent;
        let baseline_cpu = baseline.average_cpu_usage;

        // Detect CPU spike (>50% above baseline and >80% absolute)
        if cpu_usage > baseline_cpu * 1.5 && cpu_usage > 80.0 {
            let severity = if cpu_usage > 95.0 {
                IssueSeverity::Critical
            } else if cpu_usage > 90.0 {
                IssueSeverity::High
            } else {
                IssueSeverity::Medium
            };

            Some(DetectedIssue {
                issue_id: format!("cpu-spike-{}", Utc::now().timestamp()),
                issue_type: IssueType::PerformanceDegradation,
                severity,
                confidence: 0.9,
                title: "High CPU Usage Detected".to_string(),
                description: format!(
                    "CPU usage is {:.1}%, significantly above baseline of {:.1}%",
                    cpu_usage, baseline_cpu
                ),
                affected_components: vec!["CPU".to_string()],
                symptoms: vec![
                    Symptom {
                        description: format!("CPU usage at {:.1}%", cpu_usage),
                        metric_value: Some(cpu_usage),
                        timestamp: Utc::now(),
                    }
                ],
                possible_causes: vec![
                    PossibleCause {
                        description: "Runaway process or infinite loop".to_string(),
                        likelihood: 0.7,
                        investigation_steps: vec![
                            "Check top processes by CPU usage".to_string(),
                            "Review recent code changes".to_string(),
                            "Look for infinite loops in node logic".to_string(),
                        ],
                    },
                    PossibleCause {
                        description: "Insufficient resources for current load".to_string(),
                        likelihood: 0.5,
                        investigation_steps: vec![
                            "Compare current load with historical patterns".to_string(),
                            "Check if system needs scaling".to_string(),
                        ],
                    },
                ],
                suggested_actions: vec![
                    SuggestedAction {
                        action: "Profile CPU usage by process".to_string(),
                        command: Some("dora debug --mode profile --focus performance".to_string()),
                        urgency: ActionUrgency::High,
                    },
                    SuggestedAction {
                        action: "Review system resource allocation".to_string(),
                        command: Some("dora inspect system --focus performance".to_string()),
                        urgency: ActionUrgency::Medium,
                    },
                ],
                debugging_hints: vec![
                    DebuggingHint {
                        hint: "Use interactive debugging to identify the specific process causing high CPU usage".to_string(),
                        interactive_features: vec![
                            "Real-time process monitoring".to_string(),
                            "CPU usage timeline visualization".to_string(),
                            "Process tree analysis".to_string(),
                        ],
                    },
                ],
                first_detected: Utc::now(),
                related_issues: Vec::new(),
            })
        } else {
            None
        }
    }

    fn analyze_memory_performance(
        &self,
        current: &SystemMetrics,
        baseline: &BaselineMetrics,
    ) -> Option<DetectedIssue> {
        let memory_usage = current.memory_usage_mb;
        let baseline_memory = baseline.average_memory_usage;

        // Detect memory spike (>2x baseline and >1GB absolute)
        if memory_usage > baseline_memory * 2.0 && memory_usage > 1024.0 {
            Some(DetectedIssue {
                issue_id: format!("memory-spike-{}", Utc::now().timestamp()),
                issue_type: IssueType::MemoryLeak,
                severity: IssueSeverity::High,
                confidence: 0.85,
                title: "High Memory Usage Detected".to_string(),
                description: format!(
                    "Memory usage is {:.0}MB, significantly above baseline of {:.0}MB",
                    memory_usage, baseline_memory
                ),
                affected_components: vec!["Memory".to_string()],
                symptoms: vec![
                    Symptom {
                        description: format!("Memory usage at {:.0}MB", memory_usage),
                        metric_value: Some(memory_usage),
                        timestamp: Utc::now(),
                    }
                ],
                possible_causes: vec![
                    PossibleCause {
                        description: "Memory leak in application code".to_string(),
                        likelihood: 0.8,
                        investigation_steps: vec![
                            "Check for unclosed resources or circular references".to_string(),
                            "Review memory allocation patterns".to_string(),
                        ],
                    },
                ],
                suggested_actions: vec![
                    SuggestedAction {
                        action: "Profile memory usage".to_string(),
                        command: Some("dora debug --mode profile --focus memory".to_string()),
                        urgency: ActionUrgency::High,
                    },
                ],
                debugging_hints: vec![
                    DebuggingHint {
                        hint: "Interactive debugging can show memory allocation patterns over time".to_string(),
                        interactive_features: vec![
                            "Memory timeline visualization".to_string(),
                            "Heap analysis".to_string(),
                        ],
                    },
                ],
                first_detected: Utc::now(),
                related_issues: Vec::new(),
            })
        } else {
            None
        }
    }
}

impl Default for PerformanceIssueDetector {
    fn default() -> Self {
        Self::new()
    }
}

/// Dataflow issue detector
#[derive(Debug)]
pub struct DataflowIssueDetector;

impl DataflowIssueDetector {
    pub fn new() -> Self {
        Self
    }

    pub async fn detect_issues(&self, debug_session: &DebugSession) -> Result<Vec<DetectedIssue>> {
        let mut issues = Vec::new();

        if let DebugTarget::Dataflow(dataflow_name) = &debug_session.debug_target {
            // Check for stalled dataflow (mock)
            if Self::is_dataflow_stalled(dataflow_name) {
                issues.push(DetectedIssue {
                    issue_id: format!("dataflow-stall-{}", Utc::now().timestamp()),
                    issue_type: IssueType::DataflowStalled,
                    severity: IssueSeverity::Critical,
                    confidence: 0.95,
                    title: "Dataflow Stalled".to_string(),
                    description: format!("Dataflow '{}' appears to be stalled with no message flow", dataflow_name),
                    affected_components: vec![dataflow_name.clone()],
                    symptoms: vec![
                        Symptom {
                            description: "No messages processed in last 30s".to_string(),
                            metric_value: Some(0.0),
                            timestamp: Utc::now(),
                        }
                    ],
                    possible_causes: vec![
                        PossibleCause {
                            description: "Deadlock in node processing".to_string(),
                            likelihood: 0.6,
                            investigation_steps: vec![
                                "Check node states".to_string(),
                                "Review inter-node dependencies".to_string(),
                            ],
                        },
                    ],
                    suggested_actions: vec![
                        SuggestedAction {
                            action: "Inspect dataflow state interactively".to_string(),
                            command: Some(format!("dora debug {} --tui", dataflow_name)),
                            urgency: ActionUrgency::Critical,
                        },
                    ],
                    debugging_hints: vec![
                        DebuggingHint {
                            hint: "Interactive TUI provides real-time dataflow visualization".to_string(),
                            interactive_features: vec![
                                "Dataflow graph with node states".to_string(),
                                "Message flow visualization".to_string(),
                            ],
                        },
                    ],
                    first_detected: Utc::now(),
                    related_issues: Vec::new(),
                });
            }
        }

        Ok(issues)
    }

    fn is_dataflow_stalled(_name: &str) -> bool {
        // Mock implementation - would check actual dataflow state
        false
    }
}

impl Default for DataflowIssueDetector {
    fn default() -> Self {
        Self::new()
    }
}

/// Network issue detector
#[derive(Debug)]
pub struct NetworkIssueDetector;

impl NetworkIssueDetector {
    pub fn new() -> Self {
        Self
    }

    pub async fn detect_issues(&self, _debug_session: &DebugSession) -> Result<Vec<DetectedIssue>> {
        // Mock implementation - no network issues detected for now
        Ok(Vec::new())
    }
}

impl Default for NetworkIssueDetector {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_issue_detection() {
        let detector = AutomaticIssueDetector::new();
        let debug_session = DebugSession::default();

        let issues = detector.detect_issues(&debug_session).await.unwrap();

        // Should detect at least the CPU issue from mock data
        assert!(!issues.is_empty());
        assert!(issues.iter().any(|i| i.issue_type == IssueType::PerformanceDegradation));
    }

    #[tokio::test]
    async fn test_cpu_detection() {
        let detector = PerformanceIssueDetector::new();
        let metrics = SystemMetrics {
            cpu_usage_percent: 95.0,
            memory_usage_mb: 256.0,
            disk_io_mbps: 50.0,
            network_io_mbps: 100.0,
            active_processes: 45,
            timestamp: Utc::now(),
        };
        let baseline = BaselineMetrics::default();

        let issue = detector.analyze_cpu_performance(&metrics, &baseline);

        assert!(issue.is_some());
        let issue = issue.unwrap();
        assert_eq!(issue.issue_type, IssueType::PerformanceDegradation);
        assert!(matches!(issue.severity, IssueSeverity::Critical | IssueSeverity::High));
    }
}
