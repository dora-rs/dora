// Automatic Issue Detection for Issue #18

use super::types::*;
use chrono::Utc;
use dora_core::topics::DORA_COORDINATOR_PORT_CONTROL_DEFAULT;
use dora_message::coordinator_to_cli::{
    DataflowListEntry, DataflowStatus, NodeRuntimeInfo, NodeRuntimeState,
};
use eyre::{Error, Result, WrapErr};
use tokio::time::{Duration, sleep};

use crate::{
    LOCALHOST,
    common::{connect_to_coordinator, query_running_dataflows},
    tui::{app::SystemMetrics as TuiSystemMetrics, metrics::MetricsCollector},
};

const BYTES_PER_MIB: f64 = 1_048_576.0;

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
        let perf_issues = self
            .performance_detector
            .detect_issues(debug_session)
            .await?;
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
        if let Some(cpu_issue) =
            self.analyze_cpu_performance(&current_metrics, &self.baseline_metrics)
        {
            issues.push(cpu_issue);
        }

        // Check for memory issues
        if let Some(memory_issue) =
            self.analyze_memory_performance(&current_metrics, &self.baseline_metrics)
        {
            issues.push(memory_issue);
        }

        Ok(issues)
    }

    async fn get_current_metrics(&self) -> Result<SystemMetrics> {
        let mut collector = MetricsCollector::new();
        // Prime the collector so deltas (network) have a baseline.
        let _ = collector.collect();
        sleep(Duration::from_millis(250)).await;
        let snapshot = collector
            .collect()
            .wrap_err("failed to sample system metrics")?;
        Ok(convert_system_metrics(snapshot))
    }

    fn analyze_cpu_performance(
        &self,
        current: &SystemMetrics,
        baseline: &BaselineMetrics,
    ) -> Option<DetectedIssue> {
        let cpu_usage = current.cpu_usage_percent;
        let baseline_cpu = baseline.average_cpu_usage;

        // Detect CPU spike if significantly above baseline or breaching hard thresholds.
        if cpu_usage > baseline_cpu * 1.5 || cpu_usage >= 85.0 {
            let severity = if cpu_usage >= 95.0 {
                IssueSeverity::Critical
            } else if cpu_usage >= 90.0 {
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
                    "CPU usage is {cpu_usage:.1}%, significantly above baseline of {baseline_cpu:.1}%"
                ),
                affected_components: vec!["CPU".to_string()],
                symptoms: vec![
                    Symptom {
                        description: format!("CPU usage at {cpu_usage:.1}%"),
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
        let memory_percent = current.memory_usage_percent;

        // Detect memory spike (>2x baseline and >1GB absolute) or sustained high utilisation.
        if (memory_usage > baseline_memory * 2.0 && memory_usage > 1024.0) || memory_percent >= 90.0
        {
            let severity = if memory_percent >= 95.0 {
                IssueSeverity::Critical
            } else if memory_percent >= 92.0 {
                IssueSeverity::High
            } else {
                IssueSeverity::Medium
            };

            Some(DetectedIssue {
                issue_id: format!("memory-spike-{}", Utc::now().timestamp()),
                issue_type: IssueType::MemoryLeak,
                severity,
                confidence: 0.85,
                title: "High Memory Usage Detected".to_string(),
                description: format!(
                    "Memory usage is {memory_usage:.0}MB ({memory_percent:.1}%), significantly above baseline of {baseline_memory:.0}MB"
                ),
                affected_components: vec!["Memory".to_string()],
                symptoms: vec![Symptom {
                    description: format!(
                        "Memory usage at {memory_usage:.0}MB ({memory_percent:.1}%)"
                    ),
                    metric_value: Some(memory_usage),
                    timestamp: Utc::now(),
                }],
                possible_causes: vec![PossibleCause {
                    description: "Memory leak in application code".to_string(),
                    likelihood: 0.8,
                    investigation_steps: vec![
                        "Check for unclosed resources or circular references".to_string(),
                        "Review memory allocation patterns".to_string(),
                    ],
                }],
                suggested_actions: vec![SuggestedAction {
                    action: "Profile memory usage".to_string(),
                    command: Some("dora debug --mode profile --focus memory".to_string()),
                    urgency: ActionUrgency::High,
                }],
                debugging_hints: vec![DebuggingHint {
                    hint: "Interactive debugging can show memory allocation patterns over time"
                        .to_string(),
                    interactive_features: vec![
                        "Memory timeline visualization".to_string(),
                        "Heap analysis".to_string(),
                    ],
                }],
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

fn convert_system_metrics(snapshot: TuiSystemMetrics) -> SystemMetrics {
    let memory_usage_mb = snapshot.memory.used_bytes as f64 / BYTES_PER_MIB;
    let network_rx_mib = snapshot.network.received_per_second / BYTES_PER_MIB;
    let network_tx_mib = snapshot.network.transmitted_per_second / BYTES_PER_MIB;

    SystemMetrics {
        cpu_usage_percent: snapshot.cpu_usage,
        memory_usage_mb: memory_usage_mb as f32,
        memory_usage_percent: snapshot.memory_usage,
        // TODO(issue #7): extend MetricsCollector to expose disk throughput deltas.
        disk_io_mbps: 0.0,
        network_io_mbps: (network_rx_mib + network_tx_mib) as f32,
        active_processes: snapshot.process_count,
        timestamp: Utc::now(),
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
            match self.fetch_dataflow_snapshot(dataflow_name) {
                Ok(Some(entry)) => {
                    if entry.status == DataflowStatus::Failed {
                        issues.push(dataflow_failure_issue(dataflow_name, &entry));
                    }

                    if entry.nodes.is_empty() {
                        issues.push(empty_dataflow_issue(dataflow_name));
                    } else {
                        for node in entry.nodes.iter() {
                            if matches!(node.status, NodeRuntimeState::Failed) {
                                issues.push(node_failure_issue(dataflow_name, node));
                            } else if matches!(node.status, NodeRuntimeState::Unknown) {
                                issues.push(node_unknown_state_issue(dataflow_name, node));
                            }
                        }
                    }
                }
                Ok(None) => {
                    issues.push(dataflow_not_found_issue(dataflow_name));
                }
                Err(err) => {
                    issues.push(coordinator_unreachable_issue(err));
                }
            }
        }

        Ok(issues)
    }

    fn fetch_dataflow_snapshot(&self, target: &str) -> Result<Option<DataflowListEntry>> {
        let coordinator_addr = (LOCALHOST, DORA_COORDINATOR_PORT_CONTROL_DEFAULT).into();
        let mut session = connect_to_coordinator(coordinator_addr)
            .wrap_err("failed to connect to coordinator for dataflow inspection")?;
        let list =
            query_running_dataflows(&mut *session).wrap_err("failed to query running dataflows")?;

        Ok(list
            .0
            .into_iter()
            .find(|entry| dataflow_matches(entry, target)))
    }
}

impl Default for DataflowIssueDetector {
    fn default() -> Self {
        Self::new()
    }
}

fn dataflow_matches(entry: &DataflowListEntry, target: &str) -> bool {
    let target_trimmed = target.trim();
    if target_trimmed.is_empty() {
        return false;
    }
    let target_lower = target_trimmed.to_ascii_lowercase();

    if target_lower == "current" && entry.status == DataflowStatus::Running {
        return true;
    }

    if entry
        .id
        .uuid
        .to_string()
        .eq_ignore_ascii_case(target_trimmed)
    {
        return true;
    }

    if let Some(name) = &entry.id.name {
        if name.eq_ignore_ascii_case(target_trimmed) {
            return true;
        }
    }

    false
}

fn dataflow_identifier(entry: &DataflowListEntry) -> String {
    entry
        .id
        .name
        .clone()
        .unwrap_or_else(|| entry.id.uuid.to_string())
}

fn node_identifier(node: &NodeRuntimeInfo) -> String {
    node.node
        .name
        .clone()
        .unwrap_or_else(|| node.node.id.to_string())
}

fn dataflow_failure_issue(dataflow_name: &str, entry: &DataflowListEntry) -> DetectedIssue {
    let identifier = dataflow_identifier(entry);
    DetectedIssue {
        issue_id: format!("dataflow-failed-{}", Utc::now().timestamp()),
        issue_type: IssueType::DataflowStalled,
        severity: IssueSeverity::Critical,
        confidence: 0.9,
        title: format!("Dataflow '{identifier}' reported failure"),
        description: format!(
            "Coordinator reports dataflow '{identifier}' is in a failed state. Requested target: '{dataflow_name}'."
        ),
        affected_components: vec![identifier.clone()],
        symptoms: vec![Symptom {
            description: format!("Dataflow status: {:?}", entry.status),
            metric_value: None,
            timestamp: Utc::now(),
        }],
        possible_causes: vec![PossibleCause {
            description: "One or more nodes exited with an error".to_string(),
            likelihood: 0.7,
            investigation_steps: vec![
                "Inspect node exit codes via `dora logs --tail`".to_string(),
                "Review recent deployment changes".to_string(),
            ],
        }],
        suggested_actions: vec![
            SuggestedAction {
                action: "View failing node logs".to_string(),
                command: Some(format!("dora logs --dataflow {identifier}")),
                urgency: ActionUrgency::High,
            },
            SuggestedAction {
                action: "Restart the dataflow once issues are resolved".to_string(),
                command: Some(format!("dora start {identifier}")),
                urgency: ActionUrgency::Medium,
            },
        ],
        debugging_hints: vec![DebuggingHint {
            hint: "Open the TUI dashboard to inspect node state transitions".to_string(),
            interactive_features: vec![
                "Node status timeline".to_string(),
                "Recent error summaries".to_string(),
            ],
        }],
        first_detected: Utc::now(),
        related_issues: Vec::new(),
    }
}

fn empty_dataflow_issue(dataflow_name: &str) -> DetectedIssue {
    DetectedIssue {
        issue_id: format!("dataflow-empty-{}", Utc::now().timestamp()),
        issue_type: IssueType::ConfigurationError,
        severity: IssueSeverity::Medium,
        confidence: 0.6,
        title: "Dataflow has no active nodes".to_string(),
        description: format!(
            "Coordinator returned a running dataflow for '{dataflow_name}', but no node information was available."
        ),
        affected_components: vec![dataflow_name.to_string()],
        symptoms: vec![Symptom {
            description: "Coordinator response contained zero nodes".to_string(),
            metric_value: None,
            timestamp: Utc::now(),
        }],
        possible_causes: vec![PossibleCause {
            description: "Dataflow definition failed to deploy nodes".to_string(),
            likelihood: 0.5,
            investigation_steps: vec![
                "Verify the dataflow descriptor".to_string(),
                "Check coordinator logs for deployment errors".to_string(),
            ],
        }],
        suggested_actions: vec![SuggestedAction {
            action: "Rebuild and redeploy the dataflow".to_string(),
            command: Some("dora build && dora up".to_string()),
            urgency: ActionUrgency::High,
        }],
        debugging_hints: vec![DebuggingHint {
            hint: "Use the TUI explorer view to ensure nodes are registered correctly".to_string(),
            interactive_features: vec!["Dataflow explorer".to_string()],
        }],
        first_detected: Utc::now(),
        related_issues: Vec::new(),
    }
}

fn node_failure_issue(dataflow_name: &str, node: &NodeRuntimeInfo) -> DetectedIssue {
    let node_id = node_identifier(node);
    DetectedIssue {
        issue_id: format!("node-failed-{}-{}", node_id, Utc::now().timestamp()),
        issue_type: IssueType::NodeCrashing,
        severity: IssueSeverity::High,
        confidence: 0.85,
        title: format!("Node '{node_id}' reported failure"),
        description: format!(
            "Node '{}' within dataflow '{}' exited with status {:?}.",
            node_id, dataflow_name, node.status
        ),
        affected_components: vec![dataflow_name.to_string(), node_id.clone()],
        symptoms: vec![Symptom {
            description: "Node runtime status: Failed".to_string(),
            metric_value: None,
            timestamp: Utc::now(),
        }],
        possible_causes: vec![PossibleCause {
            description: "Runtime error within node processing loop".to_string(),
            likelihood: 0.6,
            investigation_steps: vec![
                "Inspect node logs for stack traces".to_string(),
                "Validate input payloads for unexpected formats".to_string(),
            ],
        }],
        suggested_actions: vec![SuggestedAction {
            action: "Tail node logs".to_string(),
            command: Some(format!("dora logs --node {node_id}")),
            urgency: ActionUrgency::High,
        }],
        debugging_hints: vec![DebuggingHint {
            hint: "Use the node inspector in the TUI to review last telemetry values".to_string(),
            interactive_features: vec!["Node inspector".to_string()],
        }],
        first_detected: Utc::now(),
        related_issues: Vec::new(),
    }
}

fn node_unknown_state_issue(dataflow_name: &str, node: &NodeRuntimeInfo) -> DetectedIssue {
    let node_id = node_identifier(node);
    DetectedIssue {
        issue_id: format!("node-unknown-{}-{}", node_id, Utc::now().timestamp()),
        issue_type: IssueType::MessageLoss,
        severity: IssueSeverity::Medium,
        confidence: 0.5,
        title: format!("Node '{node_id}' state unknown"),
        description: format!(
            "Coordinator returned 'Unknown' for node '{node_id}' in dataflow '{dataflow_name}'."
        ),
        affected_components: vec![dataflow_name.to_string(), node_id.clone()],
        symptoms: vec![Symptom {
            description: "Coordinator did not provide runtime state".to_string(),
            metric_value: None,
            timestamp: Utc::now(),
        }],
        possible_causes: vec![PossibleCause {
            description: "Node is still starting or telemetry not yet reported".to_string(),
            likelihood: 0.5,
            investigation_steps: vec![
                "Wait a few seconds and re-run `dora ps`".to_string(),
                "Ensure node registered correctly with the coordinator".to_string(),
            ],
        }],
        suggested_actions: vec![SuggestedAction {
            action: "Re-check node status after a short delay".to_string(),
            command: Some("dora ps".to_string()),
            urgency: ActionUrgency::Medium,
        }],
        debugging_hints: vec![DebuggingHint {
            hint: "Open the dashboard view to confirm registration events".to_string(),
            interactive_features: vec!["Dashboard status feed".to_string()],
        }],
        first_detected: Utc::now(),
        related_issues: Vec::new(),
    }
}

fn dataflow_not_found_issue(dataflow_name: &str) -> DetectedIssue {
    DetectedIssue {
        issue_id: format!("dataflow-missing-{}", Utc::now().timestamp()),
        issue_type: IssueType::ConfigurationError,
        severity: IssueSeverity::Medium,
        confidence: 0.7,
        title: "Dataflow not found".to_string(),
        description: format!(
            "No running dataflow matching '{dataflow_name}' was reported by the coordinator."
        ),
        affected_components: vec![dataflow_name.to_string()],
        symptoms: vec![Symptom {
            description: "Coordinator list does not include the requested dataflow".to_string(),
            metric_value: None,
            timestamp: Utc::now(),
        }],
        possible_causes: vec![PossibleCause {
            description: "Dataflow has not been started or completed already".to_string(),
            likelihood: 0.6,
            investigation_steps: vec![
                "Start the dataflow with `dora up` or `dora start`".to_string(),
                "Verify the dataflow name or UUID is correct".to_string(),
            ],
        }],
        suggested_actions: vec![SuggestedAction {
            action: "Launch the dataflow".to_string(),
            command: Some(format!("dora start {dataflow_name}")),
            urgency: ActionUrgency::Medium,
        }],
        debugging_hints: vec![DebuggingHint {
            hint: "Use the dashboard to confirm active deployments".to_string(),
            interactive_features: vec!["Dashboard overview".to_string()],
        }],
        first_detected: Utc::now(),
        related_issues: Vec::new(),
    }
}

fn coordinator_unreachable_issue(err: Error) -> DetectedIssue {
    DetectedIssue {
        issue_id: format!("coordinator-unreachable-{}", Utc::now().timestamp()),
        issue_type: IssueType::DependencyFailure,
        severity: IssueSeverity::High,
        confidence: 0.5,
        title: "Coordinator unreachable".to_string(),
        description: format!("Failed to query the coordinator: {err}"),
        affected_components: vec!["coordinator".to_string()],
        symptoms: vec![Symptom {
            description: "No response received from coordinator control port".to_string(),
            metric_value: None,
            timestamp: Utc::now(),
        }],
        possible_causes: vec![PossibleCause {
            description: "Coordinator is not running or network connection blocked".to_string(),
            likelihood: 0.7,
            investigation_steps: vec![
                "Run `dora check` to verify coordinator status".to_string(),
                "Ensure networking allows access to the control port".to_string(),
            ],
        }],
        suggested_actions: vec![SuggestedAction {
            action: "Start the coordinator".to_string(),
            command: Some("dora up".to_string()),
            urgency: ActionUrgency::High,
        }],
        debugging_hints: vec![DebuggingHint {
            hint: "Use `dora tui` to view live coordinator health once it is reachable".to_string(),
            interactive_features: vec!["System monitor".to_string()],
        }],
        first_detected: Utc::now(),
        related_issues: Vec::new(),
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
        let mut collector = MetricsCollector::new();
        let _ = collector.collect();
        sleep(Duration::from_millis(250)).await;
        let snapshot = collector
            .collect()
            .wrap_err("failed to sample system metrics for network analysis")?;

        let total_mib = ((snapshot.network.received_per_second
            + snapshot.network.transmitted_per_second)
            / BYTES_PER_MIB) as f32;

        let mut issues = Vec::new();
        if total_mib >= 80.0 {
            issues.push(network_saturation_issue(total_mib));
        }

        Ok(issues)
    }
}

impl Default for NetworkIssueDetector {
    fn default() -> Self {
        Self::new()
    }
}

fn network_saturation_issue(total_mib: f32) -> DetectedIssue {
    let severity = if total_mib >= 120.0 {
        IssueSeverity::High
    } else {
        IssueSeverity::Medium
    };
    let is_high = matches!(severity, IssueSeverity::High);

    DetectedIssue {
        issue_id: format!("network-saturation-{}", Utc::now().timestamp()),
        issue_type: IssueType::NetworkConnectivity,
        severity,
        confidence: 0.6,
        title: "High network throughput detected".to_string(),
        description: format!(
            "Combined network throughput is {total_mib:.1} MiB/s over the last sample window."
        ),
        affected_components: vec!["network".to_string()],
        symptoms: vec![Symptom {
            description: "Sustained high inbound/outbound traffic".to_string(),
            metric_value: Some(total_mib),
            timestamp: Utc::now(),
        }],
        possible_causes: vec![
            PossibleCause {
                description: "Large dataflow transfers saturating bandwidth".to_string(),
                likelihood: 0.6,
                investigation_steps: vec![
                    "Inspect active dataflows for large payload transfers".to_string(),
                    "Review recent deploys that increased data volume".to_string(),
                ],
            },
            PossibleCause {
                description: "Background system activity (updates, backups)".to_string(),
                likelihood: 0.4,
                investigation_steps: vec![
                    "Check system services for scheduled maintenance tasks".to_string(),
                ],
            },
        ],
        suggested_actions: vec![SuggestedAction {
            action: "Limit dataflow throughput or scale out network resources".to_string(),
            command: Some("dora analyze --analysis-type resources".to_string()),
            urgency: if is_high {
                ActionUrgency::High
            } else {
                ActionUrgency::Medium
            },
        }],
        debugging_hints: vec![DebuggingHint {
            hint: "Use the system monitor view in the TUI to visualise network trends".to_string(),
            interactive_features: vec!["System monitor".to_string()],
        }],
        first_detected: Utc::now(),
        related_issues: Vec::new(),
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

        // Ensure issues (if any) are ordered by severity (critical first).
        let severities: Vec<_> = issues.iter().map(|issue| issue.severity.clone()).collect();
        let mut sorted = severities.clone();
        sorted.sort_by(|a, b| severity_rank(b).cmp(&severity_rank(a)));
        assert_eq!(severities, sorted);
    }

    #[tokio::test]
    async fn test_cpu_detection() {
        let detector = PerformanceIssueDetector::new();
        let metrics = SystemMetrics {
            cpu_usage_percent: 95.0,
            memory_usage_mb: 256.0,
            memory_usage_percent: 96.0,
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
        assert!(matches!(
            issue.severity,
            IssueSeverity::Critical | IssueSeverity::High
        ));
    }

    fn severity_rank(severity: &IssueSeverity) -> u8 {
        match severity {
            IssueSeverity::Critical => 3,
            IssueSeverity::High => 2,
            IssueSeverity::Medium => 1,
            IssueSeverity::Low => 0,
        }
    }
}
