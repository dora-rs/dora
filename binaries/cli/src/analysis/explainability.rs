use crate::analysis::{
    ComponentScores, ComplexityExplanation, ComplexityFactor, FactorType,
    MLOutput, UserExpertiseLevel,
};
use std::collections::HashMap;

/// Explainer for complexity decisions and recommendations
#[derive(Debug)]
pub struct ComplexityExplainer {
    explanation_templates: HashMap<FactorType, ExplanationTemplate>,
    example_generator: ExampleGenerator,
    factor_ranker: FactorRanker,
}

/// Template for generating explanations
#[derive(Debug, Clone)]
pub struct ExplanationTemplate {
    pub summary_template: String,
    pub detail_template: String,
    pub recommendation_template: String,
    pub examples: Vec<ExplanationExample>,
}

/// Example explanation for different scenarios
#[derive(Debug, Clone)]
pub struct ExplanationExample {
    pub scenario: String,
    pub explanation: String,
    pub user_level: UserExpertiseLevel,
}

/// Generates contextual examples
#[derive(Debug)]
pub struct ExampleGenerator {
    scenario_examples: HashMap<String, Vec<ContextualExample>>,
}

/// Contextual example for specific scenarios
#[derive(Debug, Clone)]
pub struct ContextualExample {
    pub context: String,
    pub example: String,
    pub helpful_tip: Option<String>,
}

/// Ranks complexity factors by importance
#[derive(Debug)]
pub struct FactorRanker {
    importance_weights: HashMap<FactorType, f32>,
}

/// Simplified complexity explanation for quick reference
#[derive(Debug, Clone)]
pub struct SimplifiedExplanation {
    pub overall_assessment: String,
    pub primary_reason: String,
    pub recommendation: String,
    pub confidence_level: ConfidenceLevel,
}

/// Confidence level for explanations
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ConfidenceLevel {
    Low,
    Medium,
    High,
    VeryHigh,
}

impl ComplexityExplainer {
    pub fn new() -> Self {
        Self {
            explanation_templates: Self::create_explanation_templates(),
            example_generator: ExampleGenerator::new(),
            factor_ranker: FactorRanker::new(),
        }
    }
    
    /// Generate comprehensive explanation for complexity result
    pub fn explain_complexity(
        &self,
        component_scores: &ComponentScores,
        ml_result: &MLOutput,
        user_expertise: &UserExpertiseLevel,
    ) -> ComplexityExplanation {
        // Create factors from component scores
        let all_factors = self.extract_factors_from_scores(component_scores);
        
        // Rank factors by importance
        let ranked_factors = self.factor_ranker.rank_factors(&all_factors);
        
        // Identify primary, contributing, and mitigating factors
        let primary_factors = self.identify_primary_factors(&ranked_factors);
        let contributing_factors = self.identify_contributing_factors(&ranked_factors);
        let mitigating_factors = self.identify_mitigating_factors(&ranked_factors);
        
        // Generate summary based on user expertise
        let summary = self.generate_summary(
            &primary_factors,
            ml_result.predicted_complexity,
            ml_result.confidence,
            user_expertise,
        );
        
        ComplexityExplanation {
            primary_factors,
            contributing_factors,
            mitigating_factors,
            summary,
        }
    }
    
    /// Generate simplified explanation for quick decisions
    pub fn explain_simplified(
        &self,
        component_scores: &ComponentScores,
        ml_result: &MLOutput,
    ) -> SimplifiedExplanation {
        let overall_assessment = self.assess_overall_complexity(ml_result.predicted_complexity);
        let primary_reason = self.identify_primary_reason(component_scores);
        let recommendation = self.generate_quick_recommendation(ml_result.predicted_complexity);
        let confidence_level = self.assess_confidence_level(ml_result.confidence);
        
        SimplifiedExplanation {
            overall_assessment,
            primary_reason,
            recommendation,
            confidence_level,
        }
    }
    
    /// Extract factors from component scores
    fn extract_factors_from_scores(&self, scores: &ComponentScores) -> Vec<ComplexityFactor> {
        let mut factors = Vec::new();
        
        if scores.command_complexity > 5.0 {
            factors.push(ComplexityFactor {
                factor_type: FactorType::CommandComplexity,
                impact: scores.command_complexity,
                description: "Command operation is complex".to_string(),
                evidence: vec![format!("Command complexity score: {:.1}/10", scores.command_complexity)],
            });
        }
        
        if scores.data_complexity > 4.0 {
            factors.push(ComplexityFactor {
                factor_type: FactorType::DependencyComplexity,
                impact: scores.data_complexity,
                description: "Data processing requirements are significant".to_string(),
                evidence: vec![format!("Data complexity score: {:.1}/10", scores.data_complexity)],
            });
        }
        
        if scores.interaction_complexity > 6.0 {
            factors.push(ComplexityFactor {
                factor_type: FactorType::UserExperienceComplexity,
                impact: scores.interaction_complexity,
                description: "Interactive interface would significantly improve experience".to_string(),
                evidence: vec![format!("Interaction benefit score: {:.1}/10", scores.interaction_complexity)],
            });
        }
        
        if scores.output_complexity > 5.0 {
            factors.push(ComplexityFactor {
                factor_type: FactorType::OutputComplexity,
                impact: scores.output_complexity,
                description: "Output is complex and benefits from interactive navigation".to_string(),
                evidence: vec![format!("Output complexity score: {:.1}/10", scores.output_complexity)],
            });
        }
        
        if scores.error_complexity > 4.0 {
            factors.push(ComplexityFactor {
                factor_type: FactorType::ErrorComplexity,
                impact: scores.error_complexity,
                description: "High potential for errors requiring careful handling".to_string(),
                evidence: vec![format!("Error complexity score: {:.1}/10", scores.error_complexity)],
            });
        }
        
        if scores.performance_complexity > 5.0 {
            factors.push(ComplexityFactor {
                factor_type: FactorType::SystemComplexity,
                impact: scores.performance_complexity,
                description: "System performance considerations are significant".to_string(),
                evidence: vec![format!("Performance complexity score: {:.1}/10", scores.performance_complexity)],
            });
        }
        
        factors
    }
    
    /// Identify primary complexity factors
    fn identify_primary_factors(&self, ranked_factors: &[ComplexityFactor]) -> Vec<ComplexityFactor> {
        ranked_factors.iter()
            .filter(|factor| factor.impact >= 6.0)
            .take(3)
            .cloned()
            .collect()
    }
    
    /// Identify contributing complexity factors
    fn identify_contributing_factors(&self, ranked_factors: &[ComplexityFactor]) -> Vec<ComplexityFactor> {
        ranked_factors.iter()
            .filter(|factor| factor.impact >= 3.0 && factor.impact < 6.0)
            .take(5)
            .cloned()
            .collect()
    }
    
    /// Identify mitigating factors (factors that reduce complexity)
    fn identify_mitigating_factors(&self, ranked_factors: &[ComplexityFactor]) -> Vec<ComplexityFactor> {
        ranked_factors.iter()
            .filter(|factor| factor.impact < 0.0)
            .take(3)
            .cloned()
            .collect()
    }
    
    /// Generate summary explanation based on user expertise
    fn generate_summary(
        &self,
        primary_factors: &[ComplexityFactor],
        overall_score: f32,
        confidence: f32,
        user_expertise: &UserExpertiseLevel,
    ) -> String {
        let complexity_level = self.classify_complexity_level(overall_score);
        let confidence_description = self.describe_confidence(confidence);
        
        match user_expertise {
            UserExpertiseLevel::Beginner => {
                self.generate_beginner_summary(primary_factors, &complexity_level, overall_score)
            },
            UserExpertiseLevel::Intermediate => {
                self.generate_intermediate_summary(
                    primary_factors, 
                    &complexity_level, 
                    overall_score, 
                    &confidence_description
                )
            },
            UserExpertiseLevel::Expert => {
                self.generate_expert_summary(
                    primary_factors, 
                    overall_score, 
                    confidence, 
                    &confidence_description
                )
            },
        }
    }
    
    /// Generate beginner-friendly summary
    fn generate_beginner_summary(
        &self,
        primary_factors: &[ComplexityFactor],
        complexity_level: &str,
        overall_score: f32,
    ) -> String {
        let guidance = self.get_beginner_guidance(overall_score);
        
        if primary_factors.is_empty() {
            return format!(
                "This is a {} operation. {}",
                complexity_level,
                guidance
            );
        }
        
        let main_reason = &primary_factors[0].description;
        
        format!(
            "This is a {} operation, primarily because {}. {}",
            complexity_level,
            main_reason.to_lowercase(),
            guidance
        )
    }
    
    /// Generate intermediate-level summary
    fn generate_intermediate_summary(
        &self,
        primary_factors: &[ComplexityFactor],
        complexity_level: &str,
        overall_score: f32,
        confidence_description: &str,
    ) -> String {
        if primary_factors.is_empty() {
            return format!(
                "Complexity: {:.1}/10 ({}). Confidence: {}.",
                overall_score,
                complexity_level,
                confidence_description
            );
        }
        
        let factor_descriptions: Vec<String> = primary_factors.iter()
            .take(3)
            .map(|f| format!("{} ({:.1})", f.description, f.impact))
            .collect();
        
        format!(
            "Complexity: {:.1}/10 ({}). Key factors: {}. Confidence: {}.",
            overall_score,
            complexity_level,
            factor_descriptions.join(", "),
            confidence_description
        )
    }
    
    /// Generate expert-level summary
    fn generate_expert_summary(
        &self,
        primary_factors: &[ComplexityFactor],
        overall_score: f32,
        confidence: f32,
        confidence_description: &str,
    ) -> String {
        if primary_factors.is_empty() {
            return format!(
                "Complexity: {:.2}/10 (σ={:.3}). No significant factors identified.",
                overall_score,
                1.0 - confidence
            );
        }
        
        let primary_factor = &primary_factors[0];
        let factor_distribution = self.calculate_factor_distribution(primary_factors);
        
        format!(
            "Complexity: {:.2}/10. Primary: {} (impact: {:.2}). Distribution: {}. Confidence: {} (σ={:.3}).",
            overall_score,
            primary_factor.description,
            primary_factor.impact,
            factor_distribution,
            confidence_description,
            1.0 - confidence
        )
    }
    
    /// Classify complexity level
    fn classify_complexity_level(&self, score: f32) -> String {
        match score {
            0.0..=2.0 => "simple".to_string(),
            2.0..=4.0 => "straightforward".to_string(),
            4.0..=6.0 => "moderate".to_string(),
            6.0..=8.0 => "complex".to_string(),
            8.0..=10.0 => "very complex".to_string(),
            _ => "extremely complex".to_string(),
        }
    }
    
    /// Describe confidence level
    fn describe_confidence(&self, confidence: f32) -> String {
        match confidence {
            0.9..=1.0 => "very high",
            0.8..=0.9 => "high",
            0.6..=0.8 => "moderate",
            0.4..=0.6 => "low",
            _ => "very low",
        }.to_string()
    }
    
    /// Get guidance for beginners
    fn get_beginner_guidance(&self, score: f32) -> &str {
        match score {
            0.0..=2.0 => "This should be quick and straightforward.",
            2.0..=4.0 => "This is manageable - you might find the interactive interface helpful.",
            4.0..=6.0 => "Consider using the interactive interface for better guidance and control.",
            6.0..=8.0 => "The interactive interface is recommended for this complex operation.",
            _ => "This is very complex - the interactive interface will provide essential guidance and error prevention.",
        }
    }
    
    /// Calculate factor distribution for expert summary
    fn calculate_factor_distribution(&self, factors: &[ComplexityFactor]) -> String {
        if factors.is_empty() {
            return "uniform".to_string();
        }
        
        let total_impact: f32 = factors.iter().map(|f| f.impact).sum();
        let primary_ratio = factors[0].impact / total_impact;
        
        if primary_ratio > 0.7 {
            "concentrated"
        } else if primary_ratio > 0.5 {
            "primary-focused"
        } else {
            "distributed"
        }.to_string()
    }
    
    /// Assess overall complexity
    fn assess_overall_complexity(&self, score: f32) -> String {
        match score {
            0.0..=3.0 => "Low complexity - suitable for CLI".to_string(),
            3.0..=6.0 => "Moderate complexity - TUI may help".to_string(),
            6.0..=8.0 => "High complexity - TUI recommended".to_string(),
            _ => "Very high complexity - TUI strongly recommended".to_string(),
        }
    }
    
    /// Identify primary reason for complexity
    fn identify_primary_reason(&self, scores: &ComponentScores) -> String {
        let max_score = scores.command_complexity
            .max(scores.data_complexity)
            .max(scores.interaction_complexity)
            .max(scores.output_complexity)
            .max(scores.error_complexity)
            .max(scores.performance_complexity);
        
        if max_score == scores.command_complexity {
            "Complex command operation"
        } else if max_score == scores.interaction_complexity {
            "High interaction benefit"
        } else if max_score == scores.output_complexity {
            "Complex output formatting"
        } else if max_score == scores.data_complexity {
            "Complex data processing"
        } else if max_score == scores.error_complexity {
            "High error potential"
        } else {
            "System performance considerations"
        }.to_string()
    }
    
    /// Generate quick recommendation
    fn generate_quick_recommendation(&self, score: f32) -> String {
        match score {
            0.0..=3.0 => "CLI interface is sufficient".to_string(),
            3.0..=5.0 => "Consider TUI for enhanced experience".to_string(),
            5.0..=7.0 => "TUI interface recommended".to_string(),
            _ => "TUI interface strongly recommended".to_string(),
        }
    }
    
    /// Assess confidence level
    fn assess_confidence_level(&self, confidence: f32) -> ConfidenceLevel {
        match confidence {
            0.9..=1.0 => ConfidenceLevel::VeryHigh,
            0.7..=0.9 => ConfidenceLevel::High,
            0.5..=0.7 => ConfidenceLevel::Medium,
            _ => ConfidenceLevel::Low,
        }
    }
    
    /// Create explanation templates for different factor types
    fn create_explanation_templates() -> HashMap<FactorType, ExplanationTemplate> {
        let mut templates = HashMap::new();
        
        templates.insert(FactorType::CommandComplexity, ExplanationTemplate {
            summary_template: "Command operation complexity: {impact}/10".to_string(),
            detail_template: "This command involves {description} which requires careful execution.".to_string(),
            recommendation_template: "Consider using TUI for better control and feedback.".to_string(),
            examples: vec![
                ExplanationExample {
                    scenario: "debug command".to_string(),
                    explanation: "Debug operations require interactive control and state inspection.".to_string(),
                    user_level: UserExpertiseLevel::Beginner,
                },
            ],
        });
        
        templates.insert(FactorType::OutputComplexity, ExplanationTemplate {
            summary_template: "Output complexity: {impact}/10".to_string(),
            detail_template: "The output format is complex and {description}.".to_string(),
            recommendation_template: "TUI provides better navigation and filtering for complex output.".to_string(),
            examples: vec![
                ExplanationExample {
                    scenario: "large log files".to_string(),
                    explanation: "Large amounts of log data are easier to navigate with interactive filtering.".to_string(),
                    user_level: UserExpertiseLevel::Intermediate,
                },
            ],
        });
        
        templates
    }
}

impl FactorRanker {
    pub fn new() -> Self {
        Self {
            importance_weights: Self::create_importance_weights(),
        }
    }
    
    /// Rank factors by their weighted importance
    pub fn rank_factors(&self, factors: &[ComplexityFactor]) -> Vec<ComplexityFactor> {
        let mut ranked_factors = factors.to_vec();
        
        ranked_factors.sort_by(|a, b| {
            let weight_a = self.importance_weights.get(&a.factor_type).unwrap_or(&1.0);
            let weight_b = self.importance_weights.get(&b.factor_type).unwrap_or(&1.0);
            
            let score_a = a.impact * weight_a;
            let score_b = b.impact * weight_b;
            
            score_b.partial_cmp(&score_a).unwrap_or(std::cmp::Ordering::Equal)
        });
        
        ranked_factors
    }
    
    /// Create importance weights for different factor types
    fn create_importance_weights() -> HashMap<FactorType, f32> {
        let mut weights = HashMap::new();
        
        weights.insert(FactorType::CommandComplexity, 1.0);
        weights.insert(FactorType::UserExperienceComplexity, 0.9);
        weights.insert(FactorType::OutputComplexity, 0.8);
        weights.insert(FactorType::ErrorComplexity, 0.8);
        weights.insert(FactorType::SystemComplexity, 0.7);
        weights.insert(FactorType::DependencyComplexity, 0.6);
        weights.insert(FactorType::ParameterComplexity, 0.5);
        weights.insert(FactorType::ContextComplexity, 0.4);
        
        weights
    }
}

impl ExampleGenerator {
    pub fn new() -> Self {
        Self {
            scenario_examples: Self::create_scenario_examples(),
        }
    }
    
    /// Get contextual examples for a scenario
    pub fn get_examples(&self, scenario: &str) -> Vec<ContextualExample> {
        self.scenario_examples.get(scenario)
            .cloned()
            .unwrap_or_default()
    }
    
    /// Create scenario examples
    fn create_scenario_examples() -> HashMap<String, Vec<ContextualExample>> {
        let mut examples = HashMap::new();
        
        examples.insert("debug_session".to_string(), vec![
            ContextualExample {
                context: "Interactive debugging".to_string(),
                example: "Setting breakpoints, inspecting variables, and stepping through execution".to_string(),
                helpful_tip: Some("Use TUI for visual debugging interface".to_string()),
            },
        ]);
        
        examples.insert("log_analysis".to_string(), vec![
            ContextualExample {
                context: "Large log files".to_string(),
                example: "Filtering by time range, searching for patterns, highlighting errors".to_string(),
                helpful_tip: Some("TUI provides real-time filtering and search".to_string()),
            },
        ]);
        
        examples
    }
}

impl Default for ComplexityExplainer {
    fn default() -> Self {
        Self::new()
    }
}