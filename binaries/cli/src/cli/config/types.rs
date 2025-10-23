// Core types for preference handling system

use crate::cli::{Command, OutputFormat, UiMode};
use crate::cli::context::ExecutionContext;
use chrono::{DateTime, Duration, Utc};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::path::PathBuf;

/// Types of preferences that can be managed
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum PreferenceType {
    /// Interface mode (CLI, TUI, Auto)
    InterfaceMode,
    /// Output format preferences
    OutputFormat,
    /// Hint display preferences
    ShowHints,
    /// Verbosity level
    VerbosityLevel,
    /// Color scheme
    ColorScheme,
    /// Auto-launch TUI for complex operations
    AutoLaunchTUI,
    /// Custom preference
    Custom(String),
}

/// Preference values (enum to support different types)
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum PreferenceValue {
    UiMode(UiMode),
    OutputFormat(OutputFormat),
    Boolean(bool),
    Integer(i64),
    Float(f64),
    String(String),
    None,
}

/// User preference configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UserPreferences {
    /// Global preferences (apply everywhere)
    pub global: HashMap<PreferenceType, PreferenceValue>,
    /// Context-specific preferences
    pub contextual: Vec<ContextualPreference>,
    /// Temporary session preferences
    #[serde(skip)]
    pub session: HashMap<PreferenceType, PreferenceValue>,
}

impl Default for UserPreferences {
    fn default() -> Self {
        Self {
            global: HashMap::new(),
            contextual: Vec::new(),
            session: HashMap::new(),
        }
    }
}

/// Context-specific preference
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ContextualPreference {
    pub pattern: ContextPattern,
    pub preferences: HashMap<PreferenceType, PreferenceValue>,
    pub priority: u8,
}

/// Pattern for matching context
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ContextPattern {
    pub command_pattern: Option<String>,
    pub environment_pattern: Option<String>,
    pub working_dir_pattern: Option<String>,
}

/// Preference query for resolution
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct PreferenceQuery {
    pub command: String,
    pub context: ExecutionContext,
    pub user_context: UserContext,
    pub preference_type: PreferenceType,
    pub timestamp: DateTime<Utc>,
}

/// User context for preference resolution
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct UserContext {
    /// User expertise level (0.0 = beginner, 1.0 = expert)
    pub expertise_level: u8,  // 0-100 for Hash compatibility
    /// Session duration
    pub session_duration: std::time::Duration,
    /// Recent command count
    pub recent_command_count: u32,
    /// Error rate in recent commands (0-100)
    pub recent_error_rate: u8,
}

impl UserContext {
    pub fn current() -> Self {
        Self {
            expertise_level: 50,  // Neutral default
            session_duration: std::time::Duration::from_secs(0),
            recent_command_count: 0,
            recent_error_rate: 0,
        }
    }

    pub fn expertise_level_f32(&self) -> f32 {
        self.expertise_level as f32 / 100.0
    }

    pub fn error_rate_f32(&self) -> f32 {
        self.recent_error_rate as f32 / 100.0
    }
}

/// Resolved preference with metadata
#[derive(Debug, Clone)]
pub struct ResolvedPreference {
    pub preference_value: PreferenceValue,
    pub confidence: f32,
    pub resolution_source: ResolutionSource,
    pub applied_overrides: Vec<Override>,
    pub alternative_options: Vec<AlternativeOption>,
    pub explanation: PreferenceExplanation,
}

impl ResolvedPreference {
    pub fn is_still_valid(&self) -> bool {
        // Cache validity is 5 minutes
        true  // TODO: Add timestamp-based validation
    }
}

/// Source of preference resolution
#[derive(Debug, Clone)]
pub enum ResolutionSource {
    ExplicitUserSetting { config_path: String, line_number: Option<u32> },
    LearnedBehavior { pattern: String, confidence: f32 },
    ContextualDefault { context_factors: Vec<String> },
    SystemDefault { reason: String },
    EnvironmentOverride { variable: String, value: String },
    TemporaryOverride { duration: Duration, reason: String },
}

/// Override applied to preference
#[derive(Debug, Clone)]
pub struct Override {
    pub override_type: String,
    pub applied_value: PreferenceValue,
    pub reason: String,
}

/// Alternative option for preference
#[derive(Debug, Clone)]
pub struct AlternativeOption {
    pub value: PreferenceValue,
    pub description: String,
    pub confidence: f32,
}

/// Explanation for preference decision
#[derive(Debug, Clone)]
pub struct PreferenceExplanation {
    pub summary: String,
    pub reasoning_chain: Vec<ReasoningStep>,
    pub override_suggestions: Vec<String>,
    pub learning_opportunity: Option<String>,
}

/// Reasoning step in preference resolution
#[derive(Debug, Clone)]
pub struct ReasoningStep {
    pub step_type: String,
    pub description: String,
    pub confidence: f32,
}

/// User action in response to preference
#[derive(Debug, Clone)]
pub enum UserAction {
    Accepted,
    OverridePreference { new_value: PreferenceValue, make_permanent: bool },
    Dismissed,
}

/// User satisfaction with preference
#[derive(Debug, Clone, Copy)]
pub enum UserSatisfaction {
    VeryDissatisfied,
    Dissatisfied,
    Neutral,
    Satisfied,
    VerySatisfied,
}

impl UserSatisfaction {
    pub fn to_score(&self) -> f32 {
        match self {
            UserSatisfaction::VeryDissatisfied => 0.0,
            UserSatisfaction::Dissatisfied => 0.25,
            UserSatisfaction::Neutral => 0.5,
            UserSatisfaction::Satisfied => 0.75,
            UserSatisfaction::VerySatisfied => 1.0,
        }
    }
}

/// Scope of preference setting
#[derive(Debug, Clone)]
pub enum PreferenceScope {
    Global,
    Context(ContextPattern),
    Session,
}

/// Scope of preference reset
#[derive(Debug, Clone)]
pub enum ResetScope {
    All,
    UserSettings,
    LearnedBehavior,
    Specific(Vec<PreferenceType>),
}

/// Detailed explanation of preference decision
#[derive(Debug, Clone)]
pub struct DetailedExplanation {
    pub summary: String,
    pub resolution_chain: Vec<ResolutionStep>,
    pub alternative_actions: Vec<AlternativeAction>,
    pub learning_insights: Vec<LearningInsight>,
    pub override_instructions: Vec<OverrideInstruction>,
}

#[derive(Debug, Clone)]
pub struct ResolutionStep {
    pub source: String,
    pub value: String,
    pub applied: bool,
    pub reason: String,
}

#[derive(Debug, Clone)]
pub struct AlternativeAction {
    pub action: String,
    pub description: String,
    pub command: Option<String>,
}

#[derive(Debug, Clone)]
pub struct LearningInsight {
    pub insight: String,
    pub confidence: f32,
}

#[derive(Debug, Clone)]
pub struct OverrideInstruction {
    pub method: String,
    pub example: String,
}

/// Resolution candidate during preference resolution
#[derive(Debug, Clone)]
pub struct ResolutionCandidate {
    pub value: PreferenceValue,
    pub source: ResolutionSource,
    pub priority: u8,
}

/// Preference hierarchy for managing user preferences
#[derive(Debug, Clone)]
pub struct PreferenceHierarchy {
    pub user_preferences: UserPreferences,
    pub config_path: PathBuf,
}

impl PreferenceHierarchy {
    pub fn new(user_preferences: UserPreferences) -> Self {
        let config_path = dirs::config_dir()
            .unwrap_or_else(|| PathBuf::from("."))
            .join("dora")
            .join("preferences.toml");

        Self {
            user_preferences,
            config_path,
        }
    }

    pub fn set_global_preference(&mut self, pref_type: PreferenceType, value: PreferenceValue) -> eyre::Result<()> {
        self.user_preferences.global.insert(pref_type, value);
        Ok(())
    }

    pub fn set_contextual_preference(
        &mut self,
        pref_type: PreferenceType,
        value: PreferenceValue,
        pattern: ContextPattern,
    ) -> eyre::Result<()> {
        // Find existing contextual preference or create new
        let pattern_clone = pattern.clone();
        if let Some(ctx_pref) = self.user_preferences.contextual
            .iter_mut()
            .find(|cp| Self::pattern_matches_static(&cp.pattern, &pattern_clone)) {
            ctx_pref.preferences.insert(pref_type, value);
        } else {
            let mut preferences = HashMap::new();
            preferences.insert(pref_type, value);
            self.user_preferences.contextual.push(ContextualPreference {
                pattern,
                preferences,
                priority: 128,
            });
        }
        Ok(())
    }

    pub fn reset_to_defaults(&mut self) -> eyre::Result<()> {
        self.user_preferences = UserPreferences::default();
        Ok(())
    }

    pub fn reset_user_settings(&mut self) -> eyre::Result<()> {
        self.user_preferences.global.clear();
        self.user_preferences.contextual.clear();
        Ok(())
    }

    pub fn reset_preference(&mut self, pref_type: PreferenceType) -> eyre::Result<()> {
        self.user_preferences.global.remove(&pref_type);
        for ctx_pref in &mut self.user_preferences.contextual {
            ctx_pref.preferences.remove(&pref_type);
        }
        Ok(())
    }

    pub fn update_user_preference(
        &mut self,
        pref_type: &PreferenceType,
        value: PreferenceValue,
        context: Option<ExecutionContext>,
    ) -> eyre::Result<()> {
        if let Some(_context) = context {
            // TODO: Create context pattern from ExecutionContext
            self.set_global_preference(pref_type.clone(), value)?;
        } else {
            self.set_global_preference(pref_type.clone(), value)?;
        }
        Ok(())
    }

    fn pattern_matches(&self, pattern1: &ContextPattern, pattern2: &ContextPattern) -> bool {
        Self::pattern_matches_static(pattern1, pattern2)
    }

    fn pattern_matches_static(pattern1: &ContextPattern, pattern2: &ContextPattern) -> bool {
        pattern1.command_pattern == pattern2.command_pattern &&
        pattern1.environment_pattern == pattern2.environment_pattern &&
        pattern1.working_dir_pattern == pattern2.working_dir_pattern
    }
}

// Implement PartialEq for ExecutionContext so it can be used in PreferenceQuery hash
impl PartialEq for ExecutionContext {
    fn eq(&self, other: &Self) -> bool {
        self.working_dir == other.working_dir &&
        self.is_tty == other.is_tty &&
        self.is_piped == other.is_piped &&
        self.is_scripted == other.is_scripted
    }
}

impl Eq for ExecutionContext {}

impl std::hash::Hash for ExecutionContext {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        self.working_dir.hash(state);
        self.is_tty.hash(state);
        self.is_piped.hash(state);
        self.is_scripted.hash(state);
    }
}
