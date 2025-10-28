use crate::cli::context::ExecutionContext;
use crate::cli::{Command, UiMode};
use crate::config::preferences::UserPreferences;
use chrono::{DateTime, Datelike, Local, Timelike, Utc};
use lru::LruCache;
use std::collections::HashMap;
use std::num::NonZeroUsize;
use std::sync::{Arc, Mutex};

/// Context-aware preference management with caching
#[derive(Debug)]
pub struct ContextAwarePreferences {
    base_preferences: Arc<Mutex<UserPreferences>>,
    context_cache: Arc<Mutex<LruCache<ContextKey, ContextualPreferences>>>,
    adaptation_engine: AdaptationEngine,
}

/// Key for caching contextual preferences
#[derive(Debug, Clone, Hash, PartialEq, Eq)]
pub struct ContextKey {
    pub environment: String,
    pub time_of_day: u8, // Hour of day
    pub day_of_week: u8,
    pub terminal_type: String,
    pub command_category: String,
    pub is_tty: bool,
    pub is_scripted: bool,
}

/// Computed contextual preferences
#[derive(Debug, Clone)]
pub struct ContextualPreferences {
    pub ui_mode_preference: UiMode,
    pub complexity_threshold: u8,
    pub auto_launch_threshold: f32,
    pub confidence: f32,
    pub last_updated: DateTime<Utc>,
    pub reasoning: Vec<PreferenceReason>,
}

/// Reason for a preference decision
#[derive(Debug, Clone)]
pub struct PreferenceReason {
    pub factor: String,
    pub impact: f32,
    pub preference: UiMode,
    pub description: String,
}

/// Engine for adapting preferences based on context
#[derive(Debug)]
pub struct AdaptationEngine {
    time_based_rules: Vec<TimeBasedRule>,
    environment_rules: Vec<EnvironmentRule>,
    complexity_rules: Vec<ComplexityRule>,
}

/// Time-based preference rule
#[derive(Debug, Clone)]
pub struct TimeBasedRule {
    pub start_hour: u8,
    pub end_hour: u8,
    pub days_of_week: Vec<u8>,
    pub preferred_mode: UiMode,
    pub weight: f32,
    pub description: String,
}

/// Environment-based preference rule
#[derive(Debug, Clone)]
pub struct EnvironmentRule {
    pub environment_pattern: String,
    pub preferred_mode: UiMode,
    pub weight: f32,
    pub description: String,
}

/// Complexity-based preference rule
#[derive(Debug, Clone)]
pub struct ComplexityRule {
    pub min_complexity: u8,
    pub max_complexity: u8,
    pub preferred_mode: UiMode,
    pub weight: f32,
    pub description: String,
}

impl ContextAwarePreferences {
    pub fn new(base_preferences: Arc<Mutex<UserPreferences>>) -> Self {
        let cache_size = NonZeroUsize::new(100).unwrap();

        Self {
            base_preferences,
            context_cache: Arc::new(Mutex::new(LruCache::new(cache_size))),
            adaptation_engine: AdaptationEngine::new(),
        }
    }

    /// Get contextual preference for a command and context
    pub fn get_contextual_preference(
        &self,
        command: &Command,
        context: &ExecutionContext,
    ) -> ContextualPreferences {
        let context_key = self.create_context_key(command, context);

        // Check cache first
        {
            let mut cache = self.context_cache.lock().unwrap();
            if let Some(cached_prefs) = cache.get(&context_key) {
                if cached_prefs.last_updated > Utc::now() - chrono::Duration::hours(1) {
                    return cached_prefs.clone();
                }
            }
        }

        // Compute contextual preferences
        let contextual_prefs = self.compute_contextual_preferences(command, context);

        // Cache the result
        {
            let mut cache = self.context_cache.lock().unwrap();
            cache.put(context_key, contextual_prefs.clone());
        }

        contextual_prefs
    }

    /// Clear the preference cache
    pub fn clear_cache(&self) {
        let mut cache = self.context_cache.lock().unwrap();
        cache.clear();
    }

    /// Update base preferences and clear cache
    pub fn update_base_preferences(&self, preferences: UserPreferences) {
        {
            let mut base_prefs = self.base_preferences.lock().unwrap();
            *base_prefs = preferences;
        }
        self.clear_cache();
    }

    /// Create context key for caching
    fn create_context_key(&self, command: &Command, context: &ExecutionContext) -> ContextKey {
        let now = Local::now();

        ContextKey {
            environment: context
                .environment
                .ci_environment
                .as_ref()
                .map(|ci| format!("ci:{:?}", ci))
                .unwrap_or_else(|| "local".to_string()),
            time_of_day: now.hour() as u8,
            day_of_week: now.weekday().num_days_from_sunday() as u8,
            terminal_type: context
                .terminal_capabilities
                .terminal_type
                .clone()
                .unwrap_or_else(|| "unknown".to_string()),
            command_category: self.get_command_category(command),
            is_tty: context.is_tty,
            is_scripted: context.is_scripted,
        }
    }

    /// Compute contextual preferences from scratch
    fn compute_contextual_preferences(
        &self,
        command: &Command,
        context: &ExecutionContext,
    ) -> ContextualPreferences {
        let mut preference_factors = Vec::new();
        let mut reasoning = Vec::new();

        // Get base preferences
        let base_prefs = self.base_preferences.lock().unwrap();
        let default_mode = base_prefs.interface.default_ui_mode.clone();
        let complexity_threshold = base_prefs
            .interface
            .complexity_thresholds
            .suggestion_threshold;
        let auto_launch_threshold = base_prefs.interface.auto_launch.confidence_threshold;
        drop(base_prefs);

        // Apply time-based preferences
        let time_factors = self.adaptation_engine.apply_time_based_rules(context);
        for (mode, weight, description) in time_factors {
            preference_factors.push((mode.clone(), weight));
            reasoning.push(PreferenceReason {
                factor: "time".to_string(),
                impact: weight,
                preference: mode,
                description,
            });
        }

        // Apply environment-based preferences
        let env_factors = self.adaptation_engine.apply_environment_rules(context);
        for (mode, weight, description) in env_factors {
            preference_factors.push((mode.clone(), weight));
            reasoning.push(PreferenceReason {
                factor: "environment".to_string(),
                impact: weight,
                preference: mode,
                description,
            });
        }

        // Apply complexity-based preferences
        let complexity_score = self.calculate_command_complexity(command);
        let complexity_factors = self
            .adaptation_engine
            .apply_complexity_rules(complexity_score);
        for (mode, weight, description) in complexity_factors {
            preference_factors.push((mode.clone(), weight));
            reasoning.push(PreferenceReason {
                factor: "complexity".to_string(),
                impact: weight,
                preference: mode,
                description,
            });
        }

        // Apply context-specific overrides
        if context.is_scripted || context.environment.is_ci {
            preference_factors.push((UiMode::Minimal, 0.9));
            reasoning.push(PreferenceReason {
                factor: "automation".to_string(),
                impact: 0.9,
                preference: UiMode::Minimal,
                description: "Detected automation/CI environment".to_string(),
            });
        }

        if !context.is_tty {
            preference_factors.push((UiMode::Minimal, 0.95));
            reasoning.push(PreferenceReason {
                factor: "non-tty".to_string(),
                impact: 0.95,
                preference: UiMode::Minimal,
                description: "Non-TTY environment detected".to_string(),
            });
        }

        if let Some((width, height)) = context.terminal_size {
            if width < 80 || height < 24 {
                preference_factors.push((UiMode::Cli, 0.8));
                reasoning.push(PreferenceReason {
                    factor: "terminal_size".to_string(),
                    impact: 0.8,
                    preference: UiMode::Cli,
                    description: format!("Small terminal size: {}x{}", width, height),
                });
            }
        }

        // Compute weighted preference
        let (preferred_mode, confidence) =
            self.compute_weighted_preference(&preference_factors, default_mode);

        ContextualPreferences {
            ui_mode_preference: preferred_mode,
            complexity_threshold,
            auto_launch_threshold,
            confidence,
            last_updated: Utc::now(),
            reasoning,
        }
    }

    /// Calculate command complexity score
    fn calculate_command_complexity(&self, command: &Command) -> u8 {
        match command {
            Command::Ps(_) => 2,
            Command::Start(_) => 4,
            Command::Stop(_) => 3,
            Command::Logs(_) => 5,
            Command::Build(_) => 6,
            Command::Up(_) => 5,
            Command::Destroy(_) => 4,
            Command::New(_) => 3,
            Command::Check(_) => 4,
            Command::Graph(_) => 6,
            Command::Inspect(_) => 8,
            Command::Debug(_) => 9,
            Command::Analyze(_) => 9,
            Command::Monitor(_) => 7,
            Command::Tui(_) => 1,
            Command::Dashboard(_) => 1,
            Command::System(_) => 5,
            Command::Config(_) => 4,
            Command::Daemon(_) => 6,
            Command::Runtime(_) => 6,
            Command::Coordinator(_) => 6,
            Command::Self_(_) => 3,
            Command::Help(_) => 2,
        }
    }

    /// Get command category for grouping
    fn get_command_category(&self, command: &Command) -> String {
        match command {
            Command::Ps(_) | Command::Logs(_) | Command::Monitor(_) => "monitoring".to_string(),
            Command::Start(_) | Command::Stop(_) | Command::Up(_) | Command::Destroy(_) => {
                "lifecycle".to_string()
            }
            Command::Build(_) | Command::Check(_) | Command::New(_) => "development".to_string(),
            Command::Inspect(_) | Command::Debug(_) | Command::Analyze(_) => "analysis".to_string(),
            Command::Tui(_) | Command::Dashboard(_) => "interface".to_string(),
            Command::System(_)
            | Command::Config(_)
            | Command::Daemon(_)
            | Command::Runtime(_)
            | Command::Coordinator(_)
            | Command::Self_(_) => "system".to_string(),
            Command::Graph(_) => "visualization".to_string(),
            Command::Help(_) => "help".to_string(),
        }
    }

    /// Compute weighted preference from all factors
    fn compute_weighted_preference(
        &self,
        factors: &[(UiMode, f32)],
        default_mode: UiMode,
    ) -> (UiMode, f32) {
        if factors.is_empty() {
            return (default_mode, 0.5);
        }

        let mut mode_weights: HashMap<UiMode, f32> = HashMap::new();
        let mut total_weight = 0.0;

        for (mode, weight) in factors {
            *mode_weights.entry(mode.clone()).or_insert(0.0) += weight;
            total_weight += weight;
        }

        // Add default mode with base weight
        *mode_weights.entry(default_mode).or_insert(0.0) += 0.3;
        total_weight += 0.3;

        // Find the mode with highest weighted score
        let (preferred_mode, max_weight) = mode_weights
            .into_iter()
            .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
            .unwrap_or((UiMode::Auto, 0.5));

        let confidence = if total_weight > 0.0 {
            (max_weight / total_weight).clamp(0.0, 1.0)
        } else {
            0.5
        };

        (preferred_mode, confidence)
    }
}

impl AdaptationEngine {
    pub fn new() -> Self {
        Self {
            time_based_rules: Self::create_default_time_rules(),
            environment_rules: Self::create_default_environment_rules(),
            complexity_rules: Self::create_default_complexity_rules(),
        }
    }

    /// Apply time-based rules
    pub fn apply_time_based_rules(&self, context: &ExecutionContext) -> Vec<(UiMode, f32, String)> {
        let now = Local::now();
        let current_hour = now.hour() as u8;
        let current_day = now.weekday().num_days_from_sunday() as u8;

        let mut factors = Vec::new();

        for rule in &self.time_based_rules {
            if self.time_matches(current_hour, current_day, rule) {
                factors.push((
                    rule.preferred_mode.clone(),
                    rule.weight,
                    rule.description.clone(),
                ));
            }
        }

        factors
    }

    /// Apply environment-based rules
    pub fn apply_environment_rules(
        &self,
        context: &ExecutionContext,
    ) -> Vec<(UiMode, f32, String)> {
        let mut factors = Vec::new();

        for rule in &self.environment_rules {
            if self.environment_matches(context, rule) {
                factors.push((
                    rule.preferred_mode.clone(),
                    rule.weight,
                    rule.description.clone(),
                ));
            }
        }

        factors
    }

    /// Apply complexity-based rules
    pub fn apply_complexity_rules(&self, complexity: u8) -> Vec<(UiMode, f32, String)> {
        let mut factors = Vec::new();

        for rule in &self.complexity_rules {
            if complexity >= rule.min_complexity && complexity <= rule.max_complexity {
                factors.push((
                    rule.preferred_mode.clone(),
                    rule.weight,
                    rule.description.clone(),
                ));
            }
        }

        factors
    }

    /// Check if time matches rule
    fn time_matches(&self, hour: u8, day: u8, rule: &TimeBasedRule) -> bool {
        let hour_matches = if rule.start_hour <= rule.end_hour {
            hour >= rule.start_hour && hour <= rule.end_hour
        } else {
            // Handle overnight ranges (e.g., 22-6)
            hour >= rule.start_hour || hour <= rule.end_hour
        };

        let day_matches = rule.days_of_week.is_empty() || rule.days_of_week.contains(&day);

        hour_matches && day_matches
    }

    /// Check if environment matches rule
    fn environment_matches(&self, context: &ExecutionContext, rule: &EnvironmentRule) -> bool {
        let env_string = format!(
            "ci:{},tty:{},scripted:{}",
            context.environment.is_ci, context.is_tty, context.is_scripted
        );

        env_string.contains(&rule.environment_pattern)
    }

    /// Create default time-based rules
    fn create_default_time_rules() -> Vec<TimeBasedRule> {
        vec![
            TimeBasedRule {
                start_hour: 6,
                end_hour: 9,
                days_of_week: vec![1, 2, 3, 4, 5], // Weekdays
                preferred_mode: UiMode::Cli,
                weight: 0.6,
                description: "Morning efficiency hours - prefer CLI".to_string(),
            },
            TimeBasedRule {
                start_hour: 10,
                end_hour: 16,
                days_of_week: vec![1, 2, 3, 4, 5], // Weekdays
                preferred_mode: UiMode::Auto,
                weight: 0.7,
                description: "Work hours - balanced approach".to_string(),
            },
            TimeBasedRule {
                start_hour: 17,
                end_hour: 22,
                days_of_week: vec![],
                preferred_mode: UiMode::Tui,
                weight: 0.5,
                description: "Evening exploration - prefer TUI".to_string(),
            },
            TimeBasedRule {
                start_hour: 23,
                end_hour: 5,
                days_of_week: vec![],
                preferred_mode: UiMode::Minimal,
                weight: 0.8,
                description: "Late night - minimal interface".to_string(),
            },
        ]
    }

    /// Create default environment-based rules
    fn create_default_environment_rules() -> Vec<EnvironmentRule> {
        vec![
            EnvironmentRule {
                environment_pattern: "ci:true".to_string(),
                preferred_mode: UiMode::Minimal,
                weight: 0.95,
                description: "CI environment detected".to_string(),
            },
            EnvironmentRule {
                environment_pattern: "tty:false".to_string(),
                preferred_mode: UiMode::Minimal,
                weight: 0.9,
                description: "Non-interactive environment".to_string(),
            },
            EnvironmentRule {
                environment_pattern: "scripted:true".to_string(),
                preferred_mode: UiMode::Minimal,
                weight: 0.85,
                description: "Scripted execution detected".to_string(),
            },
        ]
    }

    /// Create default complexity-based rules
    fn create_default_complexity_rules() -> Vec<ComplexityRule> {
        vec![
            ComplexityRule {
                min_complexity: 0,
                max_complexity: 3,
                preferred_mode: UiMode::Cli,
                weight: 0.6,
                description: "Simple command - CLI sufficient".to_string(),
            },
            ComplexityRule {
                min_complexity: 4,
                max_complexity: 6,
                preferred_mode: UiMode::Auto,
                weight: 0.5,
                description: "Medium complexity - context dependent".to_string(),
            },
            ComplexityRule {
                min_complexity: 7,
                max_complexity: 10,
                preferred_mode: UiMode::Tui,
                weight: 0.8,
                description: "Complex command - TUI recommended".to_string(),
            },
        ]
    }
}

impl ContextualPreferences {
    /// Check if TUI should be auto-launched
    pub fn should_auto_launch_tui(&self) -> bool {
        matches!(self.ui_mode_preference, UiMode::Tui)
            && self.confidence >= self.auto_launch_threshold
    }

    /// Get human-readable reasoning
    pub fn get_reasoning_summary(&self) -> String {
        if self.reasoning.is_empty() {
            return "No specific reasoning available".to_string();
        }

        let top_reasons: Vec<_> = self
            .reasoning
            .iter()
            .filter(|r| r.impact > 0.3)
            .take(3)
            .map(|r| format!("{}: {}", r.factor, r.description))
            .collect();

        if top_reasons.is_empty() {
            "Based on default preferences".to_string()
        } else {
            top_reasons.join("; ")
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cli::context::{ExecutionEnvironment, TerminalCapabilities};
    use std::path::PathBuf;

    fn create_test_context() -> ExecutionContext {
        ExecutionContext {
            working_dir: PathBuf::from("/test"),
            output_format: crate::cli::OutputFormat::Auto,
            ui_mode: None,
            no_hints: false,
            verbose: false,
            quiet: false,
            is_tty: true,
            is_piped: false,
            is_scripted: false,
            terminal_size: Some((120, 40)),
            user_preference: UiMode::Auto,
            terminal_capabilities: TerminalCapabilities {
                colors: true,
                interactive: true,
                width: Some(120),
                height: Some(40),
                tui_capable: true,
                supports_color: true,
                supports_unicode: true,
                supports_mouse: true,
                terminal_type: Some("xterm".to_string()),
            },
            environment: ExecutionEnvironment {
                ci_environment: None,
                shell_type: Some("bash".to_string()),
                relevant_env_vars: HashMap::new(),
                is_ci: false,
                is_automation: false,
            },
            automation_result: None,
        }
    }

    #[test]
    fn test_context_key_creation() {
        let prefs = Arc::new(Mutex::new(UserPreferences::default()));
        let context_prefs = ContextAwarePreferences::new(prefs);
        let context = create_test_context();
        let command = Command::Debug(crate::cli::commands::DebugCommand::default());

        let key = context_prefs.create_context_key(&command, &context);

        assert_eq!(key.environment, "local");
        assert_eq!(key.command_category, "analysis");
        assert!(key.is_tty);
        assert!(!key.is_scripted);
    }

    #[test]
    fn test_contextual_preference_computation() {
        let prefs = Arc::new(Mutex::new(UserPreferences::default()));
        let context_prefs = ContextAwarePreferences::new(prefs);
        let context = create_test_context();
        let command = Command::Debug(crate::cli::commands::DebugCommand::default());

        let contextual = context_prefs.get_contextual_preference(&command, &context);

        assert!(contextual.confidence > 0.0);
        assert!(!contextual.reasoning.is_empty());
    }

    #[test]
    fn test_ci_environment_override() {
        let prefs = Arc::new(Mutex::new(UserPreferences::default()));
        let context_prefs = ContextAwarePreferences::new(prefs);
        let mut context = create_test_context();
        context.environment.is_ci = true;
        context.is_scripted = true;

        let command = Command::Ps(crate::cli::commands::PsCommand::default());
        let contextual = context_prefs.get_contextual_preference(&command, &context);

        assert_eq!(contextual.ui_mode_preference, UiMode::Minimal);
        assert!(contextual.confidence > 0.8);
    }
}
