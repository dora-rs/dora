// Preference Resolution Engine - Issue #16

use super::types::*;
use super::context_preferences::ContextAnalyzer;
use super::preference_learning::PreferenceLearningEngine;
use chrono::Utc;
use eyre::Result;
use lru::LruCache;
use std::num::NonZeroUsize;

/// Main preference resolution engine
#[derive(Debug)]
pub struct PreferenceResolver {
    pub preference_hierarchy: PreferenceHierarchy,
    context_analyzer: ContextAnalyzer,
    pub learning_engine: PreferenceLearningEngine,
    pub resolution_cache: LruCache<PreferenceQuery, ResolvedPreference>,
    preference_tracker: PreferenceUsageTracker,
}

/// Tracks preference usage for learning
#[derive(Debug)]
pub struct PreferenceUsageTracker {
    usage_history: Vec<UsageRecord>,
}

#[derive(Debug, Clone)]
struct UsageRecord {
    query: PreferenceQuery,
    resolved: ResolvedPreference,
    timestamp: chrono::DateTime<Utc>,
}

impl PreferenceUsageTracker {
    pub fn new() -> Self {
        Self {
            usage_history: Vec::new(),
        }
    }

    pub fn track_usage(&mut self, query: &PreferenceQuery, resolved: &ResolvedPreference) {
        self.usage_history.push(UsageRecord {
            query: query.clone(),
            resolved: resolved.clone(),
            timestamp: Utc::now(),
        });

        // Keep only last 1000 records
        if self.usage_history.len() > 1000 {
            self.usage_history.drain(0..100);
        }
    }
}

impl PreferenceResolver {
    pub fn new(user_preferences: UserPreferences) -> Self {
        Self {
            preference_hierarchy: PreferenceHierarchy::new(user_preferences),
            context_analyzer: ContextAnalyzer::new(),
            learning_engine: PreferenceLearningEngine::new(),
            resolution_cache: LruCache::new(NonZeroUsize::new(1000).unwrap()),
            preference_tracker: PreferenceUsageTracker::new(),
        }
    }

    /// Resolve a preference query to a concrete value
    pub async fn resolve_preference(
        &mut self,
        query: &PreferenceQuery,
    ) -> Result<ResolvedPreference> {
        // Check cache first
        if let Some(cached) = self.resolution_cache.get(query) {
            if cached.is_still_valid() {
                return Ok(cached.clone());
            }
        }

        // Analyze context for preference resolution
        let context_analysis = self.context_analyzer.analyze(&query.context, &query.user_context);

        // Apply preference hierarchy resolution
        let mut resolution_chain = Vec::new();

        // 1. Check for temporary overrides (highest priority)
        if let Some(temp_override) = self.check_temporary_overrides(query) {
            resolution_chain.push(temp_override);
        }

        // 2. Check environment variable overrides
        if let Some(env_override) = self.check_environment_overrides(query) {
            resolution_chain.push(env_override);
        }

        // 3. Check explicit user settings
        if let Some(explicit_setting) = self.check_explicit_user_settings(query) {
            resolution_chain.push(explicit_setting);
        }

        // 4. Check learned behavior patterns
        if let Some(learned_preference) = self.learning_engine
            .get_learned_preference(query, &context_analysis).await? {
            resolution_chain.push(ResolutionCandidate {
                value: learned_preference.preference_value,
                source: ResolutionSource::LearnedBehavior {
                    pattern: format!("{:?}", learned_preference.supporting_patterns.first()),
                    confidence: learned_preference.confidence,
                },
                priority: 60,
            });
        }

        // 5. Check contextual defaults
        if let Some(contextual_default) = self.get_contextual_default(query, &context_analysis) {
            resolution_chain.push(contextual_default);
        }

        // 6. Fall back to system defaults
        let system_default = self.get_system_default(query);
        resolution_chain.push(system_default);

        // Resolve final preference value
        let resolved = self.resolve_from_chain(&resolution_chain, query)?;

        // Track usage for learning
        self.preference_tracker.track_usage(query, &resolved);

        // Cache the result
        self.resolution_cache.put(query.clone(), resolved.clone());

        Ok(resolved)
    }

    /// Record user feedback on a preference decision
    pub async fn record_user_feedback(
        &mut self,
        query: &PreferenceQuery,
        resolved: &ResolvedPreference,
        user_action: UserAction,
        satisfaction: UserSatisfaction,
    ) -> Result<()> {
        // Update learning engine with feedback
        self.learning_engine.record_feedback(
            query,
            resolved,
            user_action.clone(),
            satisfaction,
        ).await?;

        // Update preference hierarchy if needed
        if let UserAction::OverridePreference { new_value, make_permanent } = user_action {
            if make_permanent {
                self.preference_hierarchy.update_user_preference(
                    &query.preference_type,
                    new_value,
                    Some(query.context.clone()),
                )?;
            }
        }

        Ok(())
    }

    /// Set a session-only preference (temporary)
    pub fn set_session_preference(&mut self, pref_type: PreferenceType, value: PreferenceValue) -> Result<()> {
        self.preference_hierarchy.user_preferences.session.insert(pref_type, value);
        // Clear cache to force re-resolution
        self.resolution_cache.clear();
        Ok(())
    }

    fn check_temporary_overrides(&self, query: &PreferenceQuery) -> Option<ResolutionCandidate> {
        // Check session preferences (highest priority temporary overrides)
        if let Some(value) = self.preference_hierarchy.user_preferences.session.get(&query.preference_type) {
            return Some(ResolutionCandidate {
                value: value.clone(),
                source: ResolutionSource::TemporaryOverride {
                    duration: chrono::Duration::hours(1),
                    reason: "Session preference".to_string(),
                },
                priority: 255,
            });
        }
        None
    }

    fn check_environment_overrides(&self, query: &PreferenceQuery) -> Option<ResolutionCandidate> {
        // Check for environment variable overrides
        let env_var_name = match &query.preference_type {
            PreferenceType::InterfaceMode => "DORA_UI_MODE",
            PreferenceType::OutputFormat => "DORA_OUTPUT_FORMAT",
            PreferenceType::ShowHints => "DORA_HINTS",
            PreferenceType::VerbosityLevel => "DORA_VERBOSE",
            _ => return None,
        };

        if let Ok(env_value) = std::env::var(env_var_name) {
            // Parse environment value based on preference type
            let preference_value = match &query.preference_type {
                PreferenceType::InterfaceMode => {
                    match env_value.to_lowercase().as_str() {
                        "cli" => Some(PreferenceValue::UiMode(crate::cli::UiMode::Cli)),
                        "tui" => Some(PreferenceValue::UiMode(crate::cli::UiMode::Tui)),
                        "minimal" => Some(PreferenceValue::UiMode(crate::cli::UiMode::Minimal)),
                        "auto" => Some(PreferenceValue::UiMode(crate::cli::UiMode::Auto)),
                        _ => None,
                    }
                },
                PreferenceType::ShowHints => {
                    env_value.parse::<bool>().ok().map(PreferenceValue::Boolean)
                },
                _ => None,
            };

            if let Some(value) = preference_value {
                return Some(ResolutionCandidate {
                    value,
                    source: ResolutionSource::EnvironmentOverride {
                        variable: env_var_name.to_string(),
                        value: env_value,
                    },
                    priority: 200,
                });
            }
        }

        None
    }

    fn check_explicit_user_settings(&self, query: &PreferenceQuery) -> Option<ResolutionCandidate> {
        // Check global user preferences first
        if let Some(value) = self.preference_hierarchy.user_preferences.global.get(&query.preference_type) {
            return Some(ResolutionCandidate {
                value: value.clone(),
                source: ResolutionSource::ExplicitUserSetting {
                    config_path: self.preference_hierarchy.config_path.display().to_string(),
                    line_number: None,
                },
                priority: 150,
            });
        }

        // Check contextual preferences
        for ctx_pref in &self.preference_hierarchy.user_preferences.contextual {
            if let Some(value) = ctx_pref.preferences.get(&query.preference_type) {
                // TODO: Check if context pattern matches current context
                return Some(ResolutionCandidate {
                    value: value.clone(),
                    source: ResolutionSource::ExplicitUserSetting {
                        config_path: self.preference_hierarchy.config_path.display().to_string(),
                        line_number: None,
                    },
                    priority: ctx_pref.priority,
                });
            }
        }

        None
    }

    fn get_contextual_default(
        &self,
        query: &PreferenceQuery,
        context_analysis: &super::context_preferences::ContextAnalysis,
    ) -> Option<ResolutionCandidate> {
        // Generate contextual defaults based on context analysis
        match &query.preference_type {
            PreferenceType::InterfaceMode => {
                let value = if context_analysis.complexity_indicators.overall_score > 0.7 {
                    PreferenceValue::UiMode(crate::cli::UiMode::Tui)
                } else if query.context.environment.is_ci {
                    PreferenceValue::UiMode(crate::cli::UiMode::Minimal)
                } else {
                    PreferenceValue::UiMode(crate::cli::UiMode::Auto)
                };

                Some(ResolutionCandidate {
                    value,
                    source: ResolutionSource::ContextualDefault {
                        context_factors: vec![
                            format!("Complexity: {:.2}", context_analysis.complexity_indicators.overall_score),
                            format!("Environment: {:?}", context_analysis.environment_type),
                        ],
                    },
                    priority: 50,
                })
            },
            PreferenceType::ShowHints => {
                let show_hints = !query.context.environment.is_ci &&
                                query.user_context.expertise_level_f32() < 0.7;

                Some(ResolutionCandidate {
                    value: PreferenceValue::Boolean(show_hints),
                    source: ResolutionSource::ContextualDefault {
                        context_factors: vec![
                            format!("Expertise: {:.2}", query.user_context.expertise_level_f32()),
                            format!("CI: {}", query.context.environment.is_ci),
                        ],
                    },
                    priority: 50,
                })
            },
            _ => None,
        }
    }

    fn get_system_default(&self, query: &PreferenceQuery) -> ResolutionCandidate {
        let value = match &query.preference_type {
            PreferenceType::InterfaceMode => {
                PreferenceValue::UiMode(crate::cli::UiMode::Auto)
            },
            PreferenceType::OutputFormat => {
                PreferenceValue::OutputFormat(crate::cli::OutputFormat::Auto)
            },
            PreferenceType::ShowHints => {
                PreferenceValue::Boolean(true)
            },
            PreferenceType::VerbosityLevel => {
                PreferenceValue::Integer(0)
            },
            PreferenceType::ColorScheme => {
                PreferenceValue::String("auto".to_string())
            },
            PreferenceType::AutoLaunchTUI => {
                PreferenceValue::Boolean(false)
            },
            PreferenceType::Custom(_) => {
                PreferenceValue::None
            },
        };

        ResolutionCandidate {
            value,
            source: ResolutionSource::SystemDefault {
                reason: "Default system preference".to_string(),
            },
            priority: 0,
        }
    }

    fn resolve_from_chain(
        &self,
        resolution_chain: &[ResolutionCandidate],
        query: &PreferenceQuery,
    ) -> Result<ResolvedPreference> {
        // Sort by priority (highest first)
        let mut sorted_chain = resolution_chain.to_vec();
        sorted_chain.sort_by_key(|c| std::cmp::Reverse(c.priority));

        // Find the highest priority resolution
        let primary_resolution = sorted_chain.first()
            .ok_or_else(|| eyre::eyre!("No resolution candidates found"))?;

        // Calculate confidence based on source and context
        let confidence = self.calculate_resolution_confidence(primary_resolution, query);

        // Generate explanation
        let explanation = self.generate_preference_explanation(&sorted_chain, query);

        // Find alternative options
        let alternative_options = self.generate_alternative_options(&sorted_chain);

        Ok(ResolvedPreference {
            preference_value: primary_resolution.value.clone(),
            confidence,
            resolution_source: primary_resolution.source.clone(),
            applied_overrides: self.extract_applied_overrides(&sorted_chain),
            alternative_options,
            explanation,
        })
    }

    fn calculate_resolution_confidence(&self, resolution: &ResolutionCandidate, _query: &PreferenceQuery) -> f32 {
        match &resolution.source {
            ResolutionSource::ExplicitUserSetting { .. } => 1.0,
            ResolutionSource::EnvironmentOverride { .. } => 1.0,
            ResolutionSource::TemporaryOverride { .. } => 1.0,
            ResolutionSource::LearnedBehavior { confidence, .. } => *confidence,
            ResolutionSource::ContextualDefault { .. } => 0.7,
            ResolutionSource::SystemDefault { .. } => 0.5,
        }
    }

    fn generate_preference_explanation(
        &self,
        resolution_chain: &[ResolutionCandidate],
        query: &PreferenceQuery,
    ) -> PreferenceExplanation {
        let primary = resolution_chain.first();

        let summary = if let Some(res) = primary {
            match &res.source {
                ResolutionSource::ExplicitUserSetting { .. } => {
                    format!("Using your saved preference for {:?}", query.preference_type)
                },
                ResolutionSource::LearnedBehavior { pattern, .. } => {
                    format!("Based on your usage pattern: {}", pattern)
                },
                ResolutionSource::ContextualDefault { .. } => {
                    format!("Context-appropriate default for {:?}", query.preference_type)
                },
                ResolutionSource::SystemDefault { .. } => {
                    format!("System default for {:?}", query.preference_type)
                },
                ResolutionSource::EnvironmentOverride { variable, .. } => {
                    format!("Overridden by environment variable {}", variable)
                },
                ResolutionSource::TemporaryOverride { .. } => {
                    format!("Temporary session override for {:?}", query.preference_type)
                },
            }
        } else {
            "No preference resolution available".to_string()
        };

        let reasoning_chain = resolution_chain.iter().take(3).map(|res| {
            ReasoningStep {
                step_type: format!("{:?}", res.source).split('{').next().unwrap_or("Unknown").to_string(),
                description: format!("Priority: {}, Source: {:?}", res.priority, res.source),
                confidence: res.priority as f32 / 255.0,
            }
        }).collect();

        PreferenceExplanation {
            summary,
            reasoning_chain,
            override_suggestions: vec![
                "Use --ui-mode flag to override".to_string(),
                "Set DORA_UI_MODE environment variable".to_string(),
                "Update your preferences file".to_string(),
            ],
            learning_opportunity: Some("Your preference will be learned over time".to_string()),
        }
    }

    fn generate_alternative_options(&self, resolution_chain: &[ResolutionCandidate]) -> Vec<AlternativeOption> {
        resolution_chain.iter().skip(1).take(2).map(|res| {
            AlternativeOption {
                value: res.value.clone(),
                description: format!("Alternative from {:?}", res.source),
                confidence: res.priority as f32 / 255.0,
            }
        }).collect()
    }

    fn extract_applied_overrides(&self, resolution_chain: &[ResolutionCandidate]) -> Vec<Override> {
        resolution_chain.iter().filter_map(|res| {
            match &res.source {
                ResolutionSource::EnvironmentOverride { variable, value } => {
                    Some(Override {
                        override_type: "Environment Variable".to_string(),
                        applied_value: res.value.clone(),
                        reason: format!("{}={}", variable, value),
                    })
                },
                ResolutionSource::TemporaryOverride { reason, .. } => {
                    Some(Override {
                        override_type: "Temporary".to_string(),
                        applied_value: res.value.clone(),
                        reason: reason.clone(),
                    })
                },
                _ => None,
            }
        }).collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cli::context::ExecutionContext;

    #[tokio::test]
    async fn test_preference_resolution_hierarchy() {
        let mut resolver = PreferenceResolver::new(UserPreferences::default());
        let query = PreferenceQuery {
            command: "ps".to_string(),
            context: ExecutionContext::detect_basic(),
            user_context: UserContext::current(),
            preference_type: PreferenceType::InterfaceMode,
            timestamp: Utc::now(),
        };

        let resolved = resolver.resolve_preference(&query).await.unwrap();

        assert!(resolved.confidence > 0.0);
        assert!(!resolved.explanation.summary.is_empty());
    }

    #[tokio::test]
    async fn test_environment_override() {
        unsafe {
            std::env::set_var("DORA_UI_MODE", "tui");
        }

        let mut resolver = PreferenceResolver::new(UserPreferences::default());
        let query = PreferenceQuery {
            command: "ps".to_string(),
            context: ExecutionContext::detect_basic(),
            user_context: UserContext::current(),
            preference_type: PreferenceType::InterfaceMode,
            timestamp: Utc::now(),
        };

        let resolved = resolver.resolve_preference(&query).await.unwrap();

        assert!(matches!(resolved.preference_value, PreferenceValue::UiMode(crate::cli::UiMode::Tui)));
        assert_eq!(resolved.confidence, 1.0);

        unsafe {
            std::env::remove_var("DORA_UI_MODE");
        }
    }

    #[tokio::test]
    async fn test_session_preference() {
        let mut resolver = PreferenceResolver::new(UserPreferences::default());
        resolver.set_session_preference(
            PreferenceType::ShowHints,
            PreferenceValue::Boolean(false)
        ).unwrap();

        let query = PreferenceQuery {
            command: "ps".to_string(),
            context: ExecutionContext::detect_basic(),
            user_context: UserContext::current(),
            preference_type: PreferenceType::ShowHints,
            timestamp: Utc::now(),
        };

        let resolved = resolver.resolve_preference(&query).await.unwrap();

        assert!(matches!(resolved.preference_value, PreferenceValue::Boolean(false)));
    }
}
