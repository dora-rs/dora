//! Confidence Calculation Engine
//!
//! This module implements sophisticated confidence scoring for automation detection
//! based on evidence weights, pattern analysis, and contextual factors.

use super::detector::{AutomationEvidence, AutomationType, EvidenceType, InteractionPattern};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

#[derive(Debug)]
pub struct ConfidenceCalculator {
    evidence_weights: HashMap<EvidenceType, f32>,
    automation_type_thresholds: HashMap<AutomationType, f32>,
    pattern_multipliers: HashMap<String, f32>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConfidenceBreakdown {
    pub base_confidence: f32,
    pub evidence_score: f32,
    pub pattern_score: f32,
    pub final_confidence: f32,
    pub contributing_factors: Vec<ConfidenceFactor>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConfidenceFactor {
    pub factor_type: String,
    pub weight: f32,
    pub description: String,
}

impl ConfidenceCalculator {
    pub fn new() -> Self {
        let mut evidence_weights = HashMap::new();
        evidence_weights.insert(EvidenceType::EnvironmentVariable, 0.8);
        evidence_weights.insert(EvidenceType::ProcessHierarchy, 0.6);
        evidence_weights.insert(EvidenceType::FileSystemIndicator, 0.7);
        evidence_weights.insert(EvidenceType::NetworkPattern, 0.5);
        evidence_weights.insert(EvidenceType::UserAgent, 0.9);
        evidence_weights.insert(EvidenceType::TerminalCharacteristics, 0.4);

        let mut automation_type_thresholds = HashMap::new();
        automation_type_thresholds.insert(AutomationType::Interactive, 0.7);
        automation_type_thresholds.insert(AutomationType::CiCdPipeline, 0.8);
        automation_type_thresholds.insert(AutomationType::ScriptedExecution, 0.6);
        automation_type_thresholds.insert(AutomationType::ApiIntegration, 0.7);
        automation_type_thresholds.insert(AutomationType::ContainerizedWorkflow, 0.8);
        automation_type_thresholds.insert(AutomationType::ScheduledTask, 0.7);
        automation_type_thresholds.insert(AutomationType::TestingFramework, 0.8);
        automation_type_thresholds.insert(AutomationType::MonitoringSystem, 0.6);

        let mut pattern_multipliers = HashMap::new();
        pattern_multipliers.insert("CI/CD Pipeline".to_string(), 1.2);
        pattern_multipliers.insert("Non-Interactive Pipeline".to_string(), 1.1);
        pattern_multipliers.insert("API Client".to_string(), 1.3);
        pattern_multipliers.insert("Testing Framework".to_string(), 1.2);
        pattern_multipliers.insert("Monitoring System".to_string(), 1.0);
        pattern_multipliers.insert("High Frequency Execution".to_string(), 1.4);
        pattern_multipliers.insert("Off-Hours Execution".to_string(), 1.2);

        Self {
            evidence_weights,
            automation_type_thresholds,
            pattern_multipliers,
        }
    }

    pub fn calculate_confidence(
        &self,
        automation_type: &AutomationType,
        evidence: &[AutomationEvidence],
        patterns: &[InteractionPattern],
    ) -> f32 {
        let breakdown =
            self.calculate_confidence_with_breakdown(automation_type, evidence, patterns);
        breakdown.final_confidence
    }

    pub fn calculate_confidence_with_breakdown(
        &self,
        automation_type: &AutomationType,
        evidence: &[AutomationEvidence],
        patterns: &[InteractionPattern],
    ) -> ConfidenceBreakdown {
        let mut contributing_factors = Vec::new();

        // Base confidence from automation type
        let base_confidence = self
            .automation_type_thresholds
            .get(automation_type)
            .copied()
            .unwrap_or(0.5);

        contributing_factors.push(ConfidenceFactor {
            factor_type: "Base Type Confidence".to_string(),
            weight: base_confidence,
            description: format!("Base confidence for {:?}", automation_type),
        });

        // Evidence scoring
        let evidence_score = self.calculate_evidence_score(evidence, &mut contributing_factors);

        // Pattern scoring
        let pattern_score = self.calculate_pattern_score(patterns, &mut contributing_factors);

        // Calculate final confidence with weighted combination
        let weighted_evidence = evidence_score * 0.4;
        let weighted_patterns = pattern_score * 0.3;
        let weighted_base = base_confidence * 0.3;

        let raw_confidence = weighted_evidence + weighted_patterns + weighted_base;

        // Apply diminishing returns for very high confidence
        let final_confidence = if raw_confidence > 0.9 {
            0.9 + (raw_confidence - 0.9) * 0.5
        } else {
            raw_confidence
        }
        .min(1.0)
        .max(0.0);

        let adjusted_final = if !matches!(automation_type, AutomationType::Interactive) {
            final_confidence
                .max((base_confidence + 0.05).min(1.0))
                .min(1.0)
        } else {
            final_confidence
        };

        ConfidenceBreakdown {
            base_confidence,
            evidence_score,
            pattern_score,
            final_confidence: adjusted_final,
            contributing_factors,
        }
    }

    fn calculate_evidence_score(
        &self,
        evidence: &[AutomationEvidence],
        contributing_factors: &mut Vec<ConfidenceFactor>,
    ) -> f32 {
        if evidence.is_empty() {
            return 0.0;
        }

        let mut total_weight = 0.0;
        let mut weighted_sum = 0.0;

        // Group evidence by type for analysis
        let mut evidence_by_type: HashMap<EvidenceType, Vec<&AutomationEvidence>> = HashMap::new();
        for evidence_item in evidence {
            evidence_by_type
                .entry(evidence_item.evidence_type.clone())
                .or_insert_with(Vec::new)
                .push(evidence_item);
        }

        for (evidence_type, evidence_items) in evidence_by_type {
            let type_weight = self
                .evidence_weights
                .get(&evidence_type)
                .copied()
                .unwrap_or(0.5);

            // Calculate average confidence for this evidence type
            let type_confidence: f32 = evidence_items
                .iter()
                .map(|e| e.confidence_weight)
                .sum::<f32>()
                / evidence_items.len() as f32;

            // Apply diminishing returns for multiple pieces of same evidence type
            let diminishing_factor = match evidence_items.len() {
                1 => 1.0,
                2 => 1.2,
                3 => 1.3,
                4 => 1.35,
                _ => 1.4,
            };

            let adjusted_confidence = (type_confidence * diminishing_factor).min(1.0);
            let evidence_contribution = adjusted_confidence * type_weight;

            weighted_sum += evidence_contribution;
            total_weight += type_weight;

            contributing_factors.push(ConfidenceFactor {
                factor_type: format!("{:?} Evidence", evidence_type),
                weight: evidence_contribution,
                description: format!(
                    "{} pieces of {:?} evidence (avg confidence: {:.2})",
                    evidence_items.len(),
                    evidence_type,
                    type_confidence
                ),
            });
        }

        if total_weight > 0.0 {
            weighted_sum / total_weight
        } else {
            0.0
        }
    }

    fn calculate_pattern_score(
        &self,
        patterns: &[InteractionPattern],
        contributing_factors: &mut Vec<ConfidenceFactor>,
    ) -> f32 {
        if patterns.is_empty() {
            return 0.0;
        }

        let mut total_score = 0.0;
        let mut pattern_count = 0;

        for pattern in patterns {
            let base_score = pattern.confidence * pattern.automation_likelihood;

            let multiplier = self
                .pattern_multipliers
                .get(&pattern.pattern_type)
                .copied()
                .unwrap_or(1.0);

            let pattern_score = base_score * multiplier;
            total_score += pattern_score;
            pattern_count += 1;

            contributing_factors.push(ConfidenceFactor {
                factor_type: "Pattern".to_string(),
                weight: pattern_score,
                description: format!(
                    "{} (confidence: {:.2}, likelihood: {:.2})",
                    pattern.pattern_type, pattern.confidence, pattern.automation_likelihood
                ),
            });
        }

        if pattern_count > 0 {
            // Apply pattern synergy bonus for multiple patterns
            let synergy_bonus = match pattern_count {
                1 => 1.0,
                2 => 1.1,
                3 => 1.15,
                4 => 1.2,
                _ => 1.25,
            };

            let average_score = total_score / pattern_count as f32;
            (average_score * synergy_bonus).min(1.0)
        } else {
            0.0
        }
    }

    pub fn is_automation_likely(&self, automation_type: &AutomationType, confidence: f32) -> bool {
        let threshold = self
            .automation_type_thresholds
            .get(automation_type)
            .copied()
            .unwrap_or(0.5);

        confidence >= threshold
    }

    pub fn get_confidence_level(&self, confidence: f32) -> ConfidenceLevel {
        match confidence {
            c if c >= 0.9 => ConfidenceLevel::VeryHigh,
            c if c >= 0.7 => ConfidenceLevel::High,
            c if c >= 0.5 => ConfidenceLevel::Medium,
            c if c >= 0.3 => ConfidenceLevel::Low,
            _ => ConfidenceLevel::VeryLow,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum ConfidenceLevel {
    VeryLow,
    Low,
    Medium,
    High,
    VeryHigh,
}

impl std::fmt::Display for ConfidenceLevel {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ConfidenceLevel::VeryLow => write!(f, "Very Low"),
            ConfidenceLevel::Low => write!(f, "Low"),
            ConfidenceLevel::Medium => write!(f, "Medium"),
            ConfidenceLevel::High => write!(f, "High"),
            ConfidenceLevel::VeryHigh => write!(f, "Very High"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_confidence_calculator_creation() {
        let calculator = ConfidenceCalculator::new();
        assert!(!calculator.evidence_weights.is_empty());
        assert!(!calculator.automation_type_thresholds.is_empty());
    }

    #[test]
    fn test_evidence_scoring() {
        let calculator = ConfidenceCalculator::new();

        let evidence = vec![
            AutomationEvidence {
                evidence_type: EvidenceType::EnvironmentVariable,
                source: "GitHub Actions".to_string(),
                description: "GITHUB_ACTIONS detected".to_string(),
                confidence_weight: 0.9,
            },
            AutomationEvidence {
                evidence_type: EvidenceType::UserAgent,
                source: "API Client".to_string(),
                description: "curl user agent".to_string(),
                confidence_weight: 0.8,
            },
        ];

        let confidence =
            calculator.calculate_confidence(&AutomationType::CiCdPipeline, &evidence, &[]);

        assert!(confidence > 0.5);
        assert!(confidence <= 1.0);
    }

    #[test]
    fn test_pattern_scoring() {
        let calculator = ConfidenceCalculator::new();

        let patterns = vec![InteractionPattern {
            pattern_type: "CI/CD Pipeline".to_string(),
            confidence: 0.9,
            indicators: vec!["GitHub Actions detected".to_string()],
            automation_likelihood: 0.95,
        }];

        let confidence =
            calculator.calculate_confidence(&AutomationType::CiCdPipeline, &[], &patterns);

        assert!(confidence > 0.5);
    }

    #[test]
    fn test_confidence_levels() {
        let calculator = ConfidenceCalculator::new();

        assert!(matches!(
            calculator.get_confidence_level(0.95),
            ConfidenceLevel::VeryHigh
        ));
        assert!(matches!(
            calculator.get_confidence_level(0.8),
            ConfidenceLevel::High
        ));
        assert!(matches!(
            calculator.get_confidence_level(0.6),
            ConfidenceLevel::Medium
        ));
        assert!(matches!(
            calculator.get_confidence_level(0.4),
            ConfidenceLevel::Low
        ));
        assert!(matches!(
            calculator.get_confidence_level(0.2),
            ConfidenceLevel::VeryLow
        ));
    }

    #[test]
    fn test_automation_likelihood() {
        let calculator = ConfidenceCalculator::new();

        assert!(calculator.is_automation_likely(&AutomationType::CiCdPipeline, 0.9));
        assert!(!calculator.is_automation_likely(&AutomationType::CiCdPipeline, 0.5));
        assert!(!calculator.is_automation_likely(&AutomationType::Interactive, 0.5));
    }
}
