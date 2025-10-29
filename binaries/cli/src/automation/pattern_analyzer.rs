//! Automation Pattern Analysis
//!
//! This module analyzes interaction patterns to identify automation contexts
//! and predict automation likelihood based on behavioral indicators.

use super::detector::{AutomationEvidence, InteractionPattern};
use crate::cli::context::ExecutionContext;
use chrono::Timelike;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

#[derive(Debug)]
pub struct AutomationPatternAnalyzer {
    pattern_matchers: Vec<Box<dyn PatternMatcher>>,
    temporal_analyzer: TemporalPatternAnalyzer,
    behavioral_analyzer: BehavioralPatternAnalyzer,
}

pub trait PatternMatcher: Send + Sync + std::fmt::Debug {
    fn analyze_pattern(
        &self,
        context: &ExecutionContext,
        evidence: &[AutomationEvidence],
    ) -> Option<InteractionPattern>;
    fn pattern_name(&self) -> &str;
}

#[derive(Debug)]
struct TemporalPatternAnalyzer {
    execution_history: Vec<ExecutionRecord>,
}

#[derive(Debug)]
struct BehavioralPatternAnalyzer {
    behavior_patterns: HashMap<String, BehaviorSignature>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct ExecutionRecord {
    timestamp: chrono::DateTime<chrono::Utc>,
    context_fingerprint: String,
    automation_indicators: u32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct BehaviorSignature {
    pattern_type: String,
    frequency: f32,
    consistency: f32,
    automation_correlation: f32,
}

impl AutomationPatternAnalyzer {
    pub fn new() -> Self {
        let mut analyzer = Self {
            pattern_matchers: Vec::new(),
            temporal_analyzer: TemporalPatternAnalyzer::new(),
            behavioral_analyzer: BehavioralPatternAnalyzer::new(),
        };

        analyzer.register_default_matchers();
        analyzer
    }

    pub fn analyze_patterns(
        &mut self,
        context: &ExecutionContext,
        evidence: &[AutomationEvidence],
    ) -> Vec<InteractionPattern> {
        let mut patterns = Vec::new();

        // Run pattern matchers
        for matcher in &self.pattern_matchers {
            if let Some(pattern) = matcher.analyze_pattern(context, evidence) {
                patterns.push(pattern);
            }
        }

        // Analyze temporal patterns
        let temporal_patterns = self
            .temporal_analyzer
            .analyze_temporal_patterns(context, evidence);
        patterns.extend(temporal_patterns);

        // Analyze behavioral patterns
        let behavioral_patterns = self
            .behavioral_analyzer
            .analyze_behavioral_patterns(context, evidence);
        patterns.extend(behavioral_patterns);

        // Sort by automation likelihood
        patterns.sort_by(|a, b| {
            b.automation_likelihood
                .partial_cmp(&a.automation_likelihood)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        patterns
    }

    fn register_default_matchers(&mut self) {
        self.pattern_matchers
            .push(Box::new(NonInteractivePatternMatcher));
        self.pattern_matchers.push(Box::new(PipelinePatternMatcher));
        self.pattern_matchers
            .push(Box::new(BatchProcessingPatternMatcher));
        self.pattern_matchers
            .push(Box::new(MonitoringPatternMatcher));
        self.pattern_matchers.push(Box::new(TestingPatternMatcher));
    }
}

impl TemporalPatternAnalyzer {
    fn new() -> Self {
        Self {
            execution_history: Vec::new(),
        }
    }

    fn analyze_temporal_patterns(
        &mut self,
        context: &ExecutionContext,
        evidence: &[AutomationEvidence],
    ) -> Vec<InteractionPattern> {
        let mut patterns = Vec::new();

        // Record current execution
        self.record_execution(context, evidence);

        // Analyze execution frequency
        if let Some(frequency_pattern) = self.analyze_execution_frequency() {
            patterns.push(frequency_pattern);
        }

        // Analyze execution timing
        if let Some(timing_pattern) = self.analyze_execution_timing() {
            patterns.push(timing_pattern);
        }

        patterns
    }

    fn record_execution(&mut self, context: &ExecutionContext, evidence: &[AutomationEvidence]) {
        let context_fingerprint = format!(
            "{}:{}:{}:{}",
            context.is_tty,
            context.is_piped,
            context.is_scripted,
            evidence.len()
        );

        let record = ExecutionRecord {
            timestamp: chrono::Utc::now(),
            context_fingerprint,
            automation_indicators: evidence.len() as u32,
        };

        self.execution_history.push(record);

        // Keep only recent history (last 100 executions)
        if self.execution_history.len() > 100 {
            self.execution_history.remove(0);
        }
    }

    fn analyze_execution_frequency(&self) -> Option<InteractionPattern> {
        if self.execution_history.len() < 5 {
            return None;
        }

        // Check for high-frequency execution (potential scheduled task)
        let recent_executions =
            &self.execution_history[self.execution_history.len().saturating_sub(10)..];
        let time_diffs: Vec<_> = recent_executions
            .windows(2)
            .map(|window| {
                window[1]
                    .timestamp
                    .signed_duration_since(window[0].timestamp)
                    .num_seconds()
            })
            .collect();

        let avg_interval = time_diffs.iter().sum::<i64>() as f64 / time_diffs.len() as f64;

        // If executions are very regular (within 10% variance), likely automated
        let variance = time_diffs
            .iter()
            .map(|&diff| (diff as f64 - avg_interval).powi(2))
            .sum::<f64>()
            / time_diffs.len() as f64;
        let std_dev = variance.sqrt();
        let coefficient_of_variation = std_dev / avg_interval;

        if coefficient_of_variation < 0.1 && avg_interval < 300.0 {
            // Less than 5 minutes
            Some(InteractionPattern {
                pattern_type: "High Frequency Execution".to_string(),
                confidence: 0.8,
                indicators: vec![
                    format!("Regular execution interval: {:.1}s", avg_interval),
                    format!("Low variance: {:.3}", coefficient_of_variation),
                ],
                automation_likelihood: 0.9,
            })
        } else {
            None
        }
    }

    fn analyze_execution_timing(&self) -> Option<InteractionPattern> {
        if self.execution_history.len() < 3 {
            return None;
        }

        // Check for execution outside typical working hours
        let off_hours_count = self
            .execution_history
            .iter()
            .filter(|record| {
                let hour = record.timestamp.hour();
                hour < 6 || hour > 22 // Before 6 AM or after 10 PM
            })
            .count();

        let off_hours_ratio = off_hours_count as f32 / self.execution_history.len() as f32;

        if off_hours_ratio > 0.7 {
            Some(InteractionPattern {
                pattern_type: "Off-Hours Execution".to_string(),
                confidence: 0.7,
                indicators: vec![
                    format!(
                        "{}% executions outside 6 AM - 10 PM",
                        (off_hours_ratio * 100.0) as u32
                    ),
                    "Likely automated/scheduled execution".to_string(),
                ],
                automation_likelihood: 0.8,
            })
        } else {
            None
        }
    }
}

impl BehavioralPatternAnalyzer {
    fn new() -> Self {
        Self {
            behavior_patterns: HashMap::new(),
        }
    }

    fn analyze_behavioral_patterns(
        &mut self,
        context: &ExecutionContext,
        evidence: &[AutomationEvidence],
    ) -> Vec<InteractionPattern> {
        let mut patterns = Vec::new();

        // Analyze consistency in execution context
        if let Some(consistency_pattern) = self.analyze_context_consistency(context) {
            patterns.push(consistency_pattern);
        }

        // Analyze evidence patterns
        if let Some(evidence_pattern) = self.analyze_evidence_patterns(evidence) {
            patterns.push(evidence_pattern);
        }

        patterns
    }

    fn analyze_context_consistency(
        &self,
        context: &ExecutionContext,
    ) -> Option<InteractionPattern> {
        // Check for highly consistent execution context (indicating automation)
        let mut indicators = Vec::new();
        let mut consistency_score = 0.0;

        if !context.is_tty {
            indicators.push("Consistent non-TTY execution".to_string());
            consistency_score += 0.3;
        }

        if context.is_piped {
            indicators.push("Consistent output piping".to_string());
            consistency_score += 0.2;
        }

        if context.environment.is_ci {
            indicators.push("CI environment detected".to_string());
            consistency_score += 0.4;
        }

        if context.environment.is_automation {
            indicators.push("Automation context detected".to_string());
            consistency_score += 0.4;
        }

        if consistency_score > 0.5 {
            Some(InteractionPattern {
                pattern_type: "Consistent Execution Context".to_string(),
                confidence: consistency_score,
                indicators,
                automation_likelihood: consistency_score,
            })
        } else {
            None
        }
    }

    fn analyze_evidence_patterns(
        &mut self,
        evidence: &[AutomationEvidence],
    ) -> Option<InteractionPattern> {
        if evidence.is_empty() {
            return None;
        }

        let mut indicators = Vec::new();
        let mut pattern_strength = 0.0;

        // Count evidence types
        let env_var_count = evidence
            .iter()
            .filter(|e| {
                matches!(
                    e.evidence_type,
                    super::detector::EvidenceType::EnvironmentVariable
                )
            })
            .count();
        let process_count = evidence
            .iter()
            .filter(|e| {
                matches!(
                    e.evidence_type,
                    super::detector::EvidenceType::ProcessHierarchy
                )
            })
            .count();
        let user_agent_count = evidence
            .iter()
            .filter(|e| matches!(e.evidence_type, super::detector::EvidenceType::UserAgent))
            .count();

        if env_var_count > 3 {
            indicators.push(format!(
                "{} automation environment variables detected",
                env_var_count
            ));
            pattern_strength += 0.3;
        }

        if process_count > 1 {
            indicators.push(format!(
                "{} automation process indicators detected",
                process_count
            ));
            pattern_strength += 0.2;
        }

        if user_agent_count > 0 {
            indicators.push("Machine user agent detected".to_string());
            pattern_strength += 0.4;
        }

        // Calculate weighted confidence
        let total_weight: f32 = evidence.iter().map(|e| e.confidence_weight).sum();
        let weighted_confidence = total_weight / evidence.len() as f32;

        if pattern_strength > 0.3 {
            let pattern = InteractionPattern {
                pattern_type: "Strong Automation Evidence".to_string(),
                confidence: weighted_confidence,
                indicators,
                automation_likelihood: pattern_strength,
            };

            self.behavior_patterns.insert(
                pattern.pattern_type.clone(),
                BehaviorSignature {
                    pattern_type: pattern.pattern_type.clone(),
                    frequency: evidence.len() as f32,
                    consistency: weighted_confidence,
                    automation_correlation: pattern_strength,
                },
            );

            Some(pattern)
        } else {
            None
        }
    }
}

// Pattern Matchers

#[derive(Debug)]
struct NonInteractivePatternMatcher;

impl PatternMatcher for NonInteractivePatternMatcher {
    fn analyze_pattern(
        &self,
        context: &ExecutionContext,
        _evidence: &[AutomationEvidence],
    ) -> Option<InteractionPattern> {
        if !context.is_tty && context.is_piped {
            Some(InteractionPattern {
                pattern_type: "Non-Interactive Pipeline".to_string(),
                confidence: 0.8,
                indicators: vec!["No TTY detected".to_string(), "Output is piped".to_string()],
                automation_likelihood: 0.7,
            })
        } else {
            None
        }
    }

    fn pattern_name(&self) -> &str {
        "Non-Interactive"
    }
}

#[derive(Debug)]
struct PipelinePatternMatcher;

impl PatternMatcher for PipelinePatternMatcher {
    fn analyze_pattern(
        &self,
        context: &ExecutionContext,
        evidence: &[AutomationEvidence],
    ) -> Option<InteractionPattern> {
        let ci_evidence = evidence
            .iter()
            .any(|e| e.source.contains("CI") || e.source.contains("Actions"));

        if context.environment.is_ci || ci_evidence {
            Some(InteractionPattern {
                pattern_type: "CI/CD Pipeline".to_string(),
                confidence: 0.9,
                indicators: vec![
                    "CI/CD environment detected".to_string(),
                    "Pipeline execution context".to_string(),
                ],
                automation_likelihood: 0.95,
            })
        } else {
            None
        }
    }

    fn pattern_name(&self) -> &str {
        "Pipeline"
    }
}

#[derive(Debug)]
struct BatchProcessingPatternMatcher;

impl PatternMatcher for BatchProcessingPatternMatcher {
    fn analyze_pattern(
        &self,
        context: &ExecutionContext,
        evidence: &[AutomationEvidence],
    ) -> Option<InteractionPattern> {
        let script_evidence = evidence.iter().any(|e| {
            e.source.contains("Script")
                || e.description.contains("script")
                || e.description.contains("batch")
        });

        if context.is_scripted || script_evidence {
            Some(InteractionPattern {
                pattern_type: "Batch Processing".to_string(),
                confidence: 0.7,
                indicators: vec![
                    "Script execution detected".to_string(),
                    "Batch processing context".to_string(),
                ],
                automation_likelihood: 0.8,
            })
        } else {
            None
        }
    }

    fn pattern_name(&self) -> &str {
        "Batch Processing"
    }
}

#[derive(Debug)]
struct MonitoringPatternMatcher;

impl PatternMatcher for MonitoringPatternMatcher {
    fn analyze_pattern(
        &self,
        _context: &ExecutionContext,
        evidence: &[AutomationEvidence],
    ) -> Option<InteractionPattern> {
        let monitoring_keywords = ["monitor", "health", "check", "status", "probe"];

        let monitoring_evidence = evidence.iter().any(|e| {
            monitoring_keywords
                .iter()
                .any(|keyword| e.description.to_lowercase().contains(keyword))
        });

        if monitoring_evidence {
            Some(InteractionPattern {
                pattern_type: "Monitoring System".to_string(),
                confidence: 0.6,
                indicators: vec!["Monitoring/health check indicators detected".to_string()],
                automation_likelihood: 0.7,
            })
        } else {
            None
        }
    }

    fn pattern_name(&self) -> &str {
        "Monitoring"
    }
}

#[derive(Debug)]
struct TestingPatternMatcher;

impl PatternMatcher for TestingPatternMatcher {
    fn analyze_pattern(
        &self,
        _context: &ExecutionContext,
        evidence: &[AutomationEvidence],
    ) -> Option<InteractionPattern> {
        let testing_keywords = ["test", "pytest", "jest", "junit", "spec"];

        let testing_evidence = evidence.iter().any(|e| {
            testing_keywords.iter().any(|keyword| {
                e.description.to_lowercase().contains(keyword)
                    || e.source.to_lowercase().contains(keyword)
            })
        });

        if testing_evidence {
            Some(InteractionPattern {
                pattern_type: "Testing Framework".to_string(),
                confidence: 0.8,
                indicators: vec!["Testing framework execution detected".to_string()],
                automation_likelihood: 0.85,
            })
        } else {
            None
        }
    }

    fn pattern_name(&self) -> &str {
        "Testing"
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cli::context::{ExecutionContext, ExecutionEnvironment};

    #[test]
    fn test_pattern_analyzer_creation() {
        let analyzer = AutomationPatternAnalyzer::new();
        assert!(!analyzer.pattern_matchers.is_empty());
    }

    #[test]
    fn test_non_interactive_pattern() {
        let context = ExecutionContext {
            is_tty: false,
            is_piped: true,
            is_scripted: false,
            environment: ExecutionEnvironment {
                is_ci: false,
                is_automation: false,
                ci_environment: None,
                shell_type: None,
                relevant_env_vars: std::collections::HashMap::new(),
            },
            ..ExecutionContext::detect_basic()
        };

        let matcher = NonInteractivePatternMatcher;
        let pattern = matcher.analyze_pattern(&context, &[]);

        assert!(pattern.is_some());
        let pattern = pattern.unwrap();
        assert_eq!(pattern.pattern_type, "Non-Interactive Pipeline");
        assert!(pattern.automation_likelihood > 0.5);
    }

    #[test]
    fn test_ci_pipeline_pattern() {
        let context = ExecutionContext {
            environment: ExecutionEnvironment {
                is_ci: true,
                is_automation: true,
                ci_environment: Some(crate::cli::context::CiEnvironment::GitHubActions),
                shell_type: None,
                relevant_env_vars: std::collections::HashMap::new(),
            },
            ..ExecutionContext::detect_basic()
        };

        let matcher = PipelinePatternMatcher;
        let pattern = matcher.analyze_pattern(&context, &[]);

        assert!(pattern.is_some());
        let pattern = pattern.unwrap();
        assert_eq!(pattern.pattern_type, "CI/CD Pipeline");
        assert!(pattern.automation_likelihood > 0.9);
    }
}
