// Preference Learning Engine - Issue #16

use super::types::*;
use super::context_preferences::ContextAnalysis;
use chrono::{DateTime, Duration, Utc};
use eyre::Result;
use std::collections::HashMap;

/// Main learning engine for preferences
#[derive(Debug)]
pub struct PreferenceLearningEngine {
    learning_models: HashMap<String, BayesianPreferenceLearner>,
    pattern_recognizer: PreferencePatternRecognizer,
    feedback_processor: FeedbackProcessor,
}

/// Learned preference with supporting data
#[derive(Debug, Clone)]
pub struct LearnedPreference {
    pub preference_value: PreferenceValue,
    pub confidence: f32,
    pub supporting_patterns: Vec<BehaviorPattern>,
    pub context_conditions: Vec<super::context_preferences::ContextCondition>,
    pub last_updated: DateTime<Utc>,
    pub usage_count: u32,
}

/// Behavior pattern recognized in user actions
#[derive(Debug, Clone)]
pub struct BehaviorPattern {
    pub pattern_id: String,
    pub pattern_type: PatternType,
    pub confidence: f32,
    pub frequency: u32,
    pub last_seen: DateTime<Utc>,
}

impl BehaviorPattern {
    pub fn generate_key(&self) -> String {
        format!("{}:{}", self.pattern_type.as_str(), self.pattern_id)
    }
}

#[derive(Debug, Clone)]
pub enum PatternType {
    CommandSequence,
    TimeOfDay,
    EnvironmentPreference,
    ErrorRecovery,
}

impl PatternType {
    pub fn as_str(&self) -> &str {
        match self {
            PatternType::CommandSequence => "command_seq",
            PatternType::TimeOfDay => "time",
            PatternType::EnvironmentPreference => "env",
            PatternType::ErrorRecovery => "error",
        }
    }
}

/// Training example for learning
#[derive(Debug, Clone)]
pub struct PreferenceTrainingExample {
    pub query: PreferenceQuery,
    pub chosen_preference: PreferenceValue,
    pub user_satisfaction: UserSatisfaction,
    pub timestamp: DateTime<Utc>,
    pub context_analysis: ContextAnalysis,
}

impl PreferenceTrainingExample {
    pub fn context_key(&self) -> ContextKey {
        ContextKey::from_analysis(&self.context_analysis)
    }
}

/// Bayesian preference learner
#[derive(Debug)]
pub struct BayesianPreferenceLearner {
    preference_probabilities: HashMap<ContextKey, PreferenceProbabilityDistribution>,
    smoothing_factor: f32,
    minimum_observations: u32,
}

/// Context key for grouping similar contexts
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct ContextKey {
    pub environment: String,
    pub time_category: String,
    pub expertise_level: u8,
}

impl ContextKey {
    pub fn from_analysis(analysis: &ContextAnalysis) -> Self {
        Self {
            environment: format!("{:?}", analysis.environment_type),
            time_category: format!("{:?}", analysis.temporal_context.time_of_day),
            expertise_level: (analysis.user_state.expertise_level * 10.0) as u8,
        }
    }

    pub fn from_example(example: &PreferenceTrainingExample) -> Self {
        Self::from_analysis(&example.context_analysis)
    }

    pub fn to_conditions(&self) -> Vec<super::context_preferences::ContextCondition> {
        vec![]  // Simplified for now
    }
}

/// Probability distribution for preferences
#[derive(Debug, Clone)]
pub struct PreferenceProbabilityDistribution {
    pub preferences: HashMap<String, PreferenceObservation>,
    pub total_observations: u32,
    pub last_updated: DateTime<Utc>,
}

#[derive(Debug, Clone)]
pub struct PreferenceObservation {
    pub value: PreferenceValue,
    pub count: u32,
    pub weighted_satisfaction: f32,
}

impl PreferenceProbabilityDistribution {
    pub fn new() -> Self {
        Self {
            preferences: HashMap::new(),
            total_observations: 0,
            last_updated: Utc::now(),
        }
    }

    pub fn add_observation(
        &mut self,
        preference: &PreferenceValue,
        satisfaction: UserSatisfaction,
        timestamp: DateTime<Utc>,
    ) {
        let key = format!("{:?}", preference);
        let entry = self.preferences.entry(key).or_insert(PreferenceObservation {
            value: preference.clone(),
            count: 0,
            weighted_satisfaction: 0.0,
        });

        entry.count += 1;
        entry.weighted_satisfaction += satisfaction.to_score();
        self.total_observations += 1;
        self.last_updated = timestamp;
    }

    pub fn get_most_likely_preference(&self) -> MostLikelyPreference {
        let mut best_preference = None;
        let mut best_score = 0.0;

        for obs in self.preferences.values() {
            // Calculate score as weighted average
            let avg_satisfaction = obs.weighted_satisfaction / obs.count as f32;
            let probability = obs.count as f32 / self.total_observations as f32;
            let score = avg_satisfaction * probability;

            if score > best_score {
                best_score = score;
                best_preference = Some(obs);
            }
        }

        if let Some(pref) = best_preference {
            MostLikelyPreference {
                value: pref.value.clone(),
                probability: best_score,
            }
        } else {
            MostLikelyPreference {
                value: PreferenceValue::None,
                probability: 0.0,
            }
        }
    }

    pub fn get_supporting_patterns(&self) -> Vec<BehaviorPattern> {
        // Simplified pattern extraction
        self.preferences.values()
            .filter(|obs| obs.count > 3)
            .map(|obs| BehaviorPattern {
                pattern_id: format!("{:?}", obs.value),
                pattern_type: PatternType::EnvironmentPreference,
                confidence: obs.count as f32 / self.total_observations as f32,
                frequency: obs.count,
                last_seen: self.last_updated,
            })
            .collect()
    }
}

pub struct MostLikelyPreference {
    pub value: PreferenceValue,
    pub probability: f32,
}

impl BayesianPreferenceLearner {
    pub fn new() -> Self {
        Self {
            preference_probabilities: HashMap::new(),
            smoothing_factor: 0.1,
            minimum_observations: 5,
        }
    }

    pub async fn predict_preference(
        &self,
        query: &PreferenceQuery,
        context_analysis: &ContextAnalysis,
    ) -> Result<Option<LearnedPreference>> {
        let context_key = ContextKey::from_analysis(context_analysis);

        if let Some(distribution) = self.preference_probabilities.get(&context_key) {
            if distribution.total_observations >= self.minimum_observations {
                let most_likely = distribution.get_most_likely_preference();

                if most_likely.probability > self.confidence_threshold() {
                    return Ok(Some(LearnedPreference {
                        preference_value: most_likely.value,
                        confidence: most_likely.probability,
                        supporting_patterns: distribution.get_supporting_patterns(),
                        context_conditions: context_key.to_conditions(),
                        last_updated: distribution.last_updated,
                        usage_count: distribution.total_observations,
                    }));
                }
            }
        }

        Ok(None)
    }

    pub async fn update_model(
        &mut self,
        training_data: &[PreferenceTrainingExample],
    ) -> Result<()> {
        for example in training_data {
            let context_key = example.context_key();

            let distribution = self.preference_probabilities
                .entry(context_key)
                .or_insert_with(PreferenceProbabilityDistribution::new);

            distribution.add_observation(
                &example.chosen_preference,
                example.user_satisfaction,
                example.timestamp,
            );
        }

        // Prune old or low-confidence entries
        self.prune_distributions();

        Ok(())
    }

    pub fn model_type(&self) -> &str {
        "Bayesian Preference Learner"
    }

    pub fn confidence_threshold(&self) -> f32 {
        0.6
    }

    fn prune_distributions(&mut self) {
        let cutoff_date = Utc::now() - Duration::days(30);

        self.preference_probabilities.retain(|_, distribution| {
            distribution.last_updated > cutoff_date &&
            distribution.total_observations >= self.minimum_observations
        });
    }
}

/// Pattern recognizer for user behavior
#[derive(Debug)]
pub struct PreferencePatternRecognizer {
    pattern_database: PatternDatabase,
}

#[derive(Debug)]
pub struct PatternDatabase {
    patterns: Vec<BehaviorPattern>,
}

impl PatternDatabase {
    pub fn new() -> Self {
        Self {
            patterns: Vec::new(),
        }
    }

    pub fn add_pattern(&mut self, pattern: BehaviorPattern) {
        self.patterns.push(pattern);
    }
}

impl PreferencePatternRecognizer {
    pub fn new() -> Self {
        Self {
            pattern_database: PatternDatabase::new(),
        }
    }

    pub fn recognize_patterns(
        &self,
        _user_history: &[UserInteraction],
    ) -> Vec<BehaviorPattern> {
        // Simplified pattern recognition
        // In a real implementation, this would analyze user history
        vec![]
    }

    fn merge_pattern_group(&self, group: Vec<BehaviorPattern>) -> Option<BehaviorPattern> {
        if group.is_empty() {
            return None;
        }

        // Simple merge: take the most recent pattern with aggregated stats
        let mut merged = group[0].clone();
        merged.frequency = group.iter().map(|p| p.frequency).sum();
        merged.confidence = group.iter().map(|p| p.confidence).sum::<f32>() / group.len() as f32;

        Some(merged)
    }
}

/// User interaction record
#[derive(Debug, Clone)]
pub struct UserInteraction {
    pub query: PreferenceQuery,
    pub resolved: ResolvedPreference,
    pub action: UserAction,
    pub satisfaction: UserSatisfaction,
    pub timestamp: DateTime<Utc>,
}

/// Feedback processor
#[derive(Debug)]
pub struct FeedbackProcessor {
    feedback_history: Vec<UserInteraction>,
}

impl FeedbackProcessor {
    pub fn new() -> Self {
        Self {
            feedback_history: Vec::new(),
        }
    }

    pub fn process_feedback(&mut self, interaction: UserInteraction) -> Vec<PreferenceTrainingExample> {
        self.feedback_history.push(interaction.clone());

        // Keep only last 1000 interactions
        if self.feedback_history.len() > 1000 {
            self.feedback_history.drain(0..100);
        }

        // Convert to training examples
        vec![]  // Simplified
    }
}

impl PreferenceLearningEngine {
    pub fn new() -> Self {
        Self {
            learning_models: HashMap::new(),
            pattern_recognizer: PreferencePatternRecognizer::new(),
            feedback_processor: FeedbackProcessor::new(),
        }
    }

    pub async fn get_learned_preference(
        &self,
        query: &PreferenceQuery,
        context_analysis: &ContextAnalysis,
    ) -> Result<Option<LearnedPreference>> {
        // Get or create model for this preference type
        let model_key = format!("{:?}", query.preference_type);

        if let Some(model) = self.learning_models.get(&model_key) {
            model.predict_preference(query, context_analysis).await
        } else {
            Ok(None)
        }
    }

    pub async fn record_feedback(
        &mut self,
        query: &PreferenceQuery,
        resolved: &ResolvedPreference,
        user_action: UserAction,
        satisfaction: UserSatisfaction,
    ) -> Result<()> {
        let interaction = UserInteraction {
            query: query.clone(),
            resolved: resolved.clone(),
            action: user_action,
            satisfaction,
            timestamp: Utc::now(),
        };

        let training_examples = self.feedback_processor.process_feedback(interaction);

        // Update models with new training data
        let model_key = format!("{:?}", query.preference_type);
        let model = self.learning_models
            .entry(model_key)
            .or_insert_with(BayesianPreferenceLearner::new);

        model.update_model(&training_examples).await?;

        Ok(())
    }

    pub async fn clear_learned_patterns(&mut self) -> Result<()> {
        self.learning_models.clear();
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cli::context::ExecutionContext;

    #[tokio::test]
    async fn test_bayesian_learner_basic() {
        let mut learner = BayesianPreferenceLearner::new();

        // Create training data
        let training_data = vec![
            PreferenceTrainingExample {
                query: PreferenceQuery {
                    command: "ps".to_string(),
                    context: ExecutionContext::detect_basic(),
                    user_context: UserContext::current(),
                    preference_type: PreferenceType::InterfaceMode,
                    timestamp: Utc::now(),
                },
                chosen_preference: PreferenceValue::UiMode(crate::cli::UiMode::Tui),
                user_satisfaction: UserSatisfaction::VerySatisfied,
                timestamp: Utc::now(),
                context_analysis: super::super::context_preferences::ContextAnalyzer::new()
                    .analyze(&ExecutionContext::detect_basic(), &UserContext::current()),
            },
        ];

        learner.update_model(&training_data).await.unwrap();

        // Should not have enough observations yet
        let context = ExecutionContext::detect_basic();
        let user_context = UserContext::current();
        let context_analysis = super::super::context_preferences::ContextAnalyzer::new()
            .analyze(&context, &user_context);

        let query = PreferenceQuery {
            command: "ps".to_string(),
            context,
            user_context,
            preference_type: PreferenceType::InterfaceMode,
            timestamp: Utc::now(),
        };

        let prediction = learner.predict_preference(&query, &context_analysis).await.unwrap();
        // With only 1 observation, should not predict (need minimum_observations)
        assert!(prediction.is_none());
    }

    #[tokio::test]
    async fn test_learning_engine() {
        let mut engine = PreferenceLearningEngine::new();

        let context = ExecutionContext::detect_basic();
        let user_context = UserContext::current();
        let context_analysis = super::super::context_preferences::ContextAnalyzer::new()
            .analyze(&context, &user_context);

        let query = PreferenceQuery {
            command: "ps".to_string(),
            context: context.clone(),
            user_context,
            preference_type: PreferenceType::InterfaceMode,
            timestamp: Utc::now(),
        };

        // Should return None initially
        let learned = engine.get_learned_preference(&query, &context_analysis).await.unwrap();
        assert!(learned.is_none());
    }
}
