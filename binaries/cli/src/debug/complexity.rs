// Debug Complexity Analysis for Issue #18

use super::types::*;
use crate::cli::commands::DebugMode;
use eyre::Result;

/// Analyzer for debug session complexity
#[derive(Debug)]
pub struct DebugComplexityAnalyzer;

impl DebugComplexityAnalyzer {
    pub fn new() -> Self {
        Self
    }

    /// Analyze the complexity of a debug session
    pub async fn analyze_complexity(
        &self,
        debug_session: &DebugSession,
        detected_issues: &[DetectedIssue],
    ) -> Result<DebugComplexityAnalysis> {
        let mut complexity_score = 0.0;
        let mut complexity_factors = Vec::new();

        // Base complexity from debug mode
        complexity_score += match debug_session.mode {
            DebugMode::Interactive => 6.0,
            DebugMode::Analysis => 3.0,
            DebugMode::Trace => 5.0,
            DebugMode::Profile => 7.0,
            DebugMode::Health => 2.0,
            DebugMode::Network => 4.0,
        };

        // Issue complexity
        let critical_issues = detected_issues.iter()
            .filter(|i| matches!(i.severity, IssueSeverity::Critical))
            .count();
        let high_issues = detected_issues.iter()
            .filter(|i| matches!(i.severity, IssueSeverity::High))
            .count();

        let issue_complexity = (critical_issues as f32 * 3.0) + (high_issues as f32 * 2.0);
        complexity_score += issue_complexity;

        if critical_issues > 0 {
            complexity_factors.push(ComplexityFactor {
                factor_type: FactorType::IssueComplexity,
                impact: 3.0,
                description: format!("{} critical issues detected", critical_issues),
                evidence: detected_issues.iter()
                    .filter(|i| matches!(i.severity, IssueSeverity::Critical))
                    .map(|i| i.title.clone())
                    .collect(),
            });
        }

        // System complexity
        let system_complexity = match &debug_session.debug_target {
            DebugTarget::System => {
                complexity_score += 4.0;
                complexity_factors.push(ComplexityFactor {
                    factor_type: FactorType::SystemComplexity,
                    impact: 4.0,
                    description: "System-wide debugging involves multiple components".to_string(),
                    evidence: vec!["Full system analysis required".to_string()],
                });
                4.0
            },
            DebugTarget::Dataflow(name) => {
                // Get dataflow complexity
                let dataflow_complexity = Self::calculate_dataflow_complexity(name);
                complexity_score += dataflow_complexity;

                if dataflow_complexity > 3.0 {
                    complexity_factors.push(ComplexityFactor {
                        factor_type: FactorType::DataflowComplexity,
                        impact: dataflow_complexity,
                        description: "Complex dataflow with multiple nodes and dependencies".to_string(),
                        evidence: vec![format!("Dataflow: {}", name)],
                    });
                }
                dataflow_complexity
            },
            _ => 1.0,
        };

        // Data volume complexity
        let data_complexity = if debug_session.configuration.live_monitoring {
            complexity_score += 2.0;
            complexity_factors.push(ComplexityFactor {
                factor_type: FactorType::DataComplexity,
                impact: 2.0,
                description: "Live debugging involves real-time data streams".to_string(),
                evidence: vec!["Real-time monitoring enabled".to_string()],
            });
            2.0
        } else {
            0.0
        };

        Ok(DebugComplexityAnalysis {
            overall_score: complexity_score.min(10.0),
            issue_complexity,
            system_complexity,
            data_complexity,
            factors: complexity_factors,
        })
    }

    fn calculate_dataflow_complexity(name: &str) -> f32 {
        // Mock implementation - would analyze actual dataflow structure
        // For now, just use name length as a proxy
        (name.len() as f32 / 10.0).min(5.0)
    }
}

impl Default for DebugComplexityAnalyzer {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_complexity_analysis() {
        let analyzer = DebugComplexityAnalyzer::new();
        let mut debug_session = DebugSession::default();
        debug_session.mode = DebugMode::Interactive;
        debug_session.configuration.live_monitoring = true;

        let critical_issue = DetectedIssue {
            issue_id: "test-1".to_string(),
            issue_type: IssueType::PerformanceDegradation,
            severity: IssueSeverity::Critical,
            confidence: 0.9,
            title: "Test Issue".to_string(),
            description: "Test".to_string(),
            affected_components: vec![],
            symptoms: vec![],
            possible_causes: vec![],
            suggested_actions: vec![],
            debugging_hints: vec![],
            first_detected: chrono::Utc::now(),
            related_issues: vec![],
        };

        let complexity = analyzer.analyze_complexity(&debug_session, &[critical_issue]).await.unwrap();

        assert!(complexity.overall_score > 7.0); // Interactive (6) + Live (2) + Critical issue (3)
        assert!(complexity.issue_complexity > 0.0);
    }

    #[tokio::test]
    async fn test_system_complexity() {
        let analyzer = DebugComplexityAnalyzer::new();
        let mut debug_session = DebugSession::default();
        debug_session.debug_target = DebugTarget::System;

        let complexity = analyzer.analyze_complexity(&debug_session, &[]).await.unwrap();

        assert!(complexity.system_complexity == 4.0);
        assert!(complexity.factors.iter().any(|f| matches!(f.factor_type, FactorType::SystemComplexity)));
    }
}
