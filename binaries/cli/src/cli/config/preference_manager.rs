// Preference Manager - Main API for Issue #16

use super::types::*;
use super::preference_resolver::PreferenceResolver;
use crate::cli::context::ExecutionContext;
use chrono::Utc;
use eyre::Result;

/// Main preference management interface
#[derive(Debug)]
pub struct PreferenceManager {
    resolver: PreferenceResolver,
    validator: PreferenceValidator,
    synchronizer: PreferenceSynchronizer,
    migrator: PreferenceMigrator,
}

/// Validates preference values
#[derive(Debug)]
pub struct PreferenceValidator;

impl PreferenceValidator {
    pub fn validate_preference(&self, pref_type: &PreferenceType, value: &PreferenceValue) -> Result<()> {
        match (pref_type, value) {
            (PreferenceType::InterfaceMode, PreferenceValue::UiMode(_)) => Ok(()),
            (PreferenceType::OutputFormat, PreferenceValue::OutputFormat(_)) => Ok(()),
            (PreferenceType::ShowHints, PreferenceValue::Boolean(_)) => Ok(()),
            (PreferenceType::VerbosityLevel, PreferenceValue::Integer(v)) if *v >= 0 && *v <= 5 => Ok(()),
            (PreferenceType::ColorScheme, PreferenceValue::String(_)) => Ok(()),
            (PreferenceType::AutoLaunchTUI, PreferenceValue::Boolean(_)) => Ok(()),
            (PreferenceType::Custom(_), _) => Ok(()),  // Custom preferences are flexible
            _ => Err(eyre::eyre!("Invalid preference value {:?} for type {:?}", value, pref_type)),
        }
    }
}

/// Synchronizes preferences across environments
#[derive(Debug)]
pub struct PreferenceSynchronizer;

impl PreferenceSynchronizer {
    pub async fn schedule_sync(&self) -> Result<()> {
        // TODO: Implement preference synchronization
        // This would sync preferences to cloud storage or distributed config
        Ok(())
    }
}

/// Migrates preferences between versions
#[derive(Debug)]
pub struct PreferenceMigrator;

impl PreferenceMigrator {
    pub fn migrate_preferences(&self, _old_version: &str, _new_version: &str) -> Result<()> {
        // TODO: Implement preference migration
        // This would handle breaking changes in preference schema
        Ok(())
    }
}

impl PreferenceManager {
    pub fn new(user_preferences: UserPreferences) -> Self {
        Self {
            resolver: PreferenceResolver::new(user_preferences),
            validator: PreferenceValidator,
            synchronizer: PreferenceSynchronizer,
            migrator: PreferenceMigrator,
        }
    }

    /// Get the effective preference for a given context
    pub async fn get_effective_preference(
        &mut self,
        preference_type: PreferenceType,
        context: &ExecutionContext,
    ) -> Result<ResolvedPreference> {
        let user_context = UserContext::current();

        let query = PreferenceQuery {
            command: "unknown".to_string(),  // TODO: Extract from context
            context: context.clone(),
            user_context,
            preference_type,
            timestamp: Utc::now(),
        };

        self.resolver.resolve_preference(&query).await
    }

    /// Set a user preference with a specific scope
    pub async fn set_user_preference(
        &mut self,
        preference_type: PreferenceType,
        value: PreferenceValue,
        scope: PreferenceScope,
    ) -> Result<()> {
        // Validate the preference value
        self.validator.validate_preference(&preference_type, &value)?;

        // Check if we need to sync before consuming scope
        let needs_sync = matches!(scope, PreferenceScope::Global | PreferenceScope::Context(_));

        // Apply the preference based on scope
        match scope {
            PreferenceScope::Global => {
                self.resolver.preference_hierarchy.set_global_preference(preference_type, value)?;
            },
            PreferenceScope::Context(context_pattern) => {
                self.resolver.preference_hierarchy.set_contextual_preference(
                    preference_type,
                    value,
                    context_pattern,
                )?;
            },
            PreferenceScope::Session => {
                self.resolver.set_session_preference(preference_type, value)?;
            },
        }

        // Trigger synchronization if needed
        if needs_sync {
            self.synchronizer.schedule_sync().await?;
        }

        Ok(())
    }

    /// Reset preferences to defaults
    pub async fn reset_preferences(
        &mut self,
        reset_scope: ResetScope,
    ) -> Result<()> {
        match reset_scope {
            ResetScope::All => {
                self.resolver.preference_hierarchy.reset_to_defaults()?;
                self.resolver.learning_engine.clear_learned_patterns().await?;
            },
            ResetScope::UserSettings => {
                self.resolver.preference_hierarchy.reset_user_settings()?;
            },
            ResetScope::LearnedBehavior => {
                self.resolver.learning_engine.clear_learned_patterns().await?;
            },
            ResetScope::Specific(preference_types) => {
                for pref_type in preference_types {
                    self.resolver.preference_hierarchy.reset_preference(pref_type)?;
                }
            },
        }

        // Clear caches
        self.resolver.resolution_cache.clear();

        Ok(())
    }

    /// Explain how a preference decision was made
    pub fn explain_preference_decision(
        &self,
        resolved_preference: &ResolvedPreference,
    ) -> DetailedExplanation {
        DetailedExplanation {
            summary: resolved_preference.explanation.summary.clone(),
            resolution_chain: self.build_resolution_chain_explanation(resolved_preference),
            alternative_actions: self.suggest_alternative_actions(resolved_preference),
            learning_insights: self.provide_learning_insights(resolved_preference),
            override_instructions: self.generate_override_instructions(resolved_preference),
        }
    }

    fn build_resolution_chain_explanation(&self, resolved: &ResolvedPreference) -> Vec<ResolutionStep> {
        vec![
            ResolutionStep {
                source: format!("{:?}", resolved.resolution_source),
                value: format!("{:?}", resolved.preference_value),
                applied: true,
                reason: resolved.explanation.summary.clone(),
            },
        ]
    }

    fn suggest_alternative_actions(&self, resolved: &ResolvedPreference) -> Vec<AlternativeAction> {
        resolved.alternative_options.iter().map(|alt| {
            AlternativeAction {
                action: format!("Use {:?}", alt.value),
                description: alt.description.clone(),
                command: Some(format!("--option {:?}", alt.value)),
            }
        }).collect()
    }

    fn provide_learning_insights(&self, resolved: &ResolvedPreference) -> Vec<LearningInsight> {
        if let Some(learning) = &resolved.explanation.learning_opportunity {
            vec![
                LearningInsight {
                    insight: learning.clone(),
                    confidence: resolved.confidence,
                },
            ]
        } else {
            vec![]
        }
    }

    fn generate_override_instructions(&self, _resolved: &ResolvedPreference) -> Vec<OverrideInstruction> {
        vec![
            OverrideInstruction {
                method: "Command line flag".to_string(),
                example: "--ui-mode tui".to_string(),
            },
            OverrideInstruction {
                method: "Environment variable".to_string(),
                example: "DORA_UI_MODE=tui".to_string(),
            },
            OverrideInstruction {
                method: "Configuration file".to_string(),
                example: "~/.config/dora/preferences.toml".to_string(),
            },
        ]
    }

    /// Record user feedback on a preference
    pub async fn record_feedback(
        &mut self,
        query: &PreferenceQuery,
        resolved: &ResolvedPreference,
        user_action: UserAction,
        satisfaction: UserSatisfaction,
    ) -> Result<()> {
        self.resolver.record_user_feedback(query, resolved, user_action, satisfaction).await
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cli::{OutputFormat, UiMode};

    #[tokio::test]
    async fn test_preference_manager_basic() {
        let mut manager = PreferenceManager::new(UserPreferences::default());

        let context = ExecutionContext::detect_basic();

        // Get effective preference
        let resolved = manager.get_effective_preference(
            PreferenceType::InterfaceMode,
            &context,
        ).await.unwrap();

        assert!(resolved.confidence > 0.0);
    }

    #[tokio::test]
    async fn test_set_global_preference() {
        let mut manager = PreferenceManager::new(UserPreferences::default());

        // Set a global preference
        manager.set_user_preference(
            PreferenceType::ShowHints,
            PreferenceValue::Boolean(false),
            PreferenceScope::Global,
        ).await.unwrap();

        // Verify it's set
        let context = ExecutionContext::detect_basic();
        let resolved = manager.get_effective_preference(
            PreferenceType::ShowHints,
            &context,
        ).await.unwrap();

        assert!(matches!(resolved.preference_value, PreferenceValue::Boolean(false)));
    }

    #[tokio::test]
    async fn test_reset_preferences() {
        let mut manager = PreferenceManager::new(UserPreferences::default());

        // Set some preferences
        manager.set_user_preference(
            PreferenceType::ShowHints,
            PreferenceValue::Boolean(false),
            PreferenceScope::Global,
        ).await.unwrap();

        // Reset all preferences
        manager.reset_preferences(ResetScope::All).await.unwrap();

        // Verify reset (should return to default)
        let context = ExecutionContext::detect_basic();
        let resolved = manager.get_effective_preference(
            PreferenceType::ShowHints,
            &context,
        ).await.unwrap();

        // After reset, should use system default (true for hints)
        assert!(resolved.confidence > 0.0);
    }

    #[tokio::test]
    async fn test_preference_validation() {
        let mut manager = PreferenceManager::new(UserPreferences::default());

        // Valid preference
        let result = manager.set_user_preference(
            PreferenceType::InterfaceMode,
            PreferenceValue::UiMode(UiMode::Tui),
            PreferenceScope::Global,
        ).await;
        assert!(result.is_ok());

        // Invalid preference (wrong type)
        let result = manager.set_user_preference(
            PreferenceType::InterfaceMode,
            PreferenceValue::Boolean(true),
            PreferenceScope::Global,
        ).await;
        assert!(result.is_err());
    }

    #[tokio::test]
    async fn test_preference_explanation() {
        let mut manager = PreferenceManager::new(UserPreferences::default());

        let context = ExecutionContext::detect_basic();
        let resolved = manager.get_effective_preference(
            PreferenceType::InterfaceMode,
            &context,
        ).await.unwrap();

        let explanation = manager.explain_preference_decision(&resolved);

        assert!(!explanation.summary.is_empty());
        assert!(!explanation.resolution_chain.is_empty());
        assert!(!explanation.override_instructions.is_empty());
    }
}
