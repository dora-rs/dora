use crate::cli::context::ExecutionContext;
use crate::cli::{Command, UiMode};
use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use std::collections::{HashMap, VecDeque};
use std::path::PathBuf;
use std::time::Duration;

/// Main user preferences structure
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UserPreferences {
    /// Interface behavior preferences
    pub interface: InterfacePreferences,

    /// Command-specific preferences
    pub commands: CommandPreferences,

    /// Behavioral learning data
    pub behavior: BehaviorPreferences,

    /// Environment-specific overrides
    pub environments: HashMap<String, EnvironmentPreferences>,

    /// Preference metadata
    pub metadata: PreferenceMetadata,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InterfacePreferences {
    /// Global UI mode preference
    pub default_ui_mode: UiMode,

    /// Complexity thresholds for auto-suggestions
    pub complexity_thresholds: ComplexityThresholds,

    /// Auto-launch settings
    pub auto_launch: AutoLaunchPreferences,

    /// Hint and suggestion preferences
    pub hints: HintPreferences,

    /// TUI-specific preferences
    pub tui: TuiPreferences,

    /// CLI-specific preferences
    pub cli: CliPreferences,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CommandPreferences {
    /// Command-specific UI mode overrides
    pub command_ui_modes: HashMap<String, UiMode>,

    /// Command-specific default flags
    pub default_flags: HashMap<String, Vec<String>>,

    /// Command aliases
    pub aliases: HashMap<String, String>,

    /// Auto-completion preferences
    pub completion: CompletionPreferences,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BehaviorPreferences {
    /// User action patterns
    pub action_patterns: HashMap<String, ActionPattern>,

    /// Command usage statistics
    pub command_usage: HashMap<String, UsageStatistics>,

    /// Interface choice history
    pub interface_choices: VecDeque<InterfaceChoice>,

    /// Learning settings
    pub learning_enabled: bool,

    /// Adaptation weights
    pub adaptation_weights: AdaptationWeights,

    /// Maximum history size
    pub max_history_size: usize,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EnvironmentPreferences {
    /// Environment name
    pub name: String,

    /// UI mode override for this environment
    pub ui_mode_override: Option<UiMode>,

    /// Complexity threshold override
    pub complexity_threshold_override: Option<u8>,

    /// Disable learning in this environment
    pub disable_learning: bool,

    /// Environment-specific aliases
    pub aliases: HashMap<String, String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PreferenceMetadata {
    /// Version of the preference format
    pub version: String,

    /// When preferences were created
    pub created_at: DateTime<Utc>,

    /// Last updated timestamp
    pub updated_at: DateTime<Utc>,

    /// User ID for synchronization
    pub user_id: Option<String>,

    /// Schema version for migration
    pub schema_version: u32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ComplexityThresholds {
    /// Threshold for suggesting TUI (0-10)
    pub suggestion_threshold: u8,

    /// Threshold for auto-launching TUI (0-10)
    pub auto_launch_threshold: u8,

    /// Threshold for showing hints (0-10)
    pub hint_threshold: u8,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AutoLaunchPreferences {
    /// Confidence threshold for auto-launch (0.0-1.0)
    pub confidence_threshold: f32,

    /// Commands that should never auto-launch TUI
    pub blacklist: Vec<String>,

    /// Commands that should always auto-launch TUI
    pub whitelist: Vec<String>,

    /// Delay before auto-launch (milliseconds)
    pub delay_ms: u64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HintPreferences {
    /// Whether to show hints
    pub show_hints: bool,

    /// Frequency of hints (never, rare, normal, frequent)
    pub hint_frequency: HintFrequency,

    /// Types of hints to show
    pub hint_types: Vec<HintType>,

    /// Dismissed hints (to avoid repetition)
    pub dismissed_hints: Vec<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TuiPreferences {
    /// Color theme
    pub theme: String,

    /// Auto-refresh interval
    #[serde(with = "duration_serde")]
    pub auto_refresh_interval: Duration,

    /// Mouse support
    pub mouse_support: bool,

    /// Key bindings
    pub key_bindings: HashMap<String, String>,

    /// Default view on TUI launch
    pub default_view: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CliPreferences {
    /// Output width preference
    pub output_width: Option<usize>,

    /// Color preferences
    pub color_mode: ColorMode,

    /// Pagination preferences
    pub pagination: bool,

    /// Command aliases
    pub command_aliases: HashMap<String, String>,

    /// Default output format
    pub default_output_format: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CompletionPreferences {
    /// Enable tab completion
    pub enabled: bool,

    /// Show completion descriptions
    pub show_descriptions: bool,

    /// Maximum number of suggestions
    pub max_suggestions: usize,

    /// Context-aware completion
    pub context_aware: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ActionPattern {
    pub pattern_type: PatternType,
    pub frequency: f32,
    pub last_seen: DateTime<Utc>,
    pub context_factors: Vec<ContextFactor>,
    pub preferred_interface: Option<UiMode>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UsageStatistics {
    pub total_uses: u64,
    pub interface_distribution: HashMap<UiMode, u64>,
    pub success_rate: f32,
    #[serde(with = "duration_serde")]
    pub average_session_length: Duration,
    pub last_used: DateTime<Utc>,
    pub user_rating: Option<f32>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InterfaceChoice {
    pub command: String,
    pub context: ContextSnapshot,
    pub chosen_interface: UiMode,
    pub user_satisfaction: Option<SatisfactionLevel>,
    pub timestamp: DateTime<Utc>,
    pub session_id: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AdaptationWeights {
    /// Weight for behavioral learning (0.0-1.0)
    pub behavioral_weight: f32,

    /// Weight for context-based decisions (0.0-1.0)
    pub context_weight: f32,

    /// Weight for complexity-based decisions (0.0-1.0)
    pub complexity_weight: f32,

    /// Weight for explicit preferences (0.0-1.0)
    pub explicit_weight: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ContextSnapshot {
    pub is_tty: bool,
    pub is_scripted: bool,
    pub terminal_size: Option<(u16, u16)>,
    pub time_of_day: u8,
    pub day_of_week: u8,
    pub environment_type: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum PatternType {
    CommandSequence,
    TimeBasedUsage,
    ContextBasedChoice,
    ErrorRecovery,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum ContextFactor {
    TimeOfDay(u8),
    DayOfWeek(u8),
    TerminalSize(u16, u16),
    Environment(String),
    CommandComplexity(u8),
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum SatisfactionLevel {
    VeryUnsatisfied = 1,
    Unsatisfied = 2,
    Neutral = 3,
    Satisfied = 4,
    VerySatisfied = 5,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum HintFrequency {
    Never,
    Rare,
    Normal,
    Frequent,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum HintType {
    InterfaceSuggestion,
    CommandTip,
    ShortcutHint,
    FeatureDiscovery,
    PerformanceTip,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum ColorMode {
    Auto,
    Always,
    Never,
}

#[derive(Debug)]
pub struct InterfacePreferencePrediction {
    pub predicted_mode: UiMode,
    pub confidence: f32,
    pub factors: Vec<PreferenceFactor>,
}

#[derive(Debug)]
pub struct PreferenceFactor {
    pub factor_type: FactorType,
    pub weight: f32,
    pub preference: UiMode,
    pub confidence: f32,
}

#[derive(Debug)]
pub enum FactorType {
    ExplicitPreference,
    BehavioralLearning,
    ContextBased,
    ComplexityBased,
}

#[derive(Debug)]
pub struct BehavioralPreference {
    pub preference: UiMode,
    pub confidence: f32,
}

impl UserPreferences {
    pub fn load_or_create() -> eyre::Result<Self> {
        let config_path = Self::get_preferences_path()?;

        if config_path.exists() {
            let content = std::fs::read_to_string(&config_path)?;
            let mut preferences: UserPreferences = toml::from_str(&content)?;
            preferences.validate()?;
            preferences.migrate_if_needed()?;
            Ok(preferences)
        } else {
            let default_prefs = Self::default();
            default_prefs.save()?;
            Ok(default_prefs)
        }
    }

    pub fn save(&self) -> eyre::Result<()> {
        let config_path = Self::get_preferences_path()?;
        self.ensure_preferences_dir()?;

        let content = toml::to_string_pretty(self)?;
        std::fs::write(&config_path, content)?;
        Ok(())
    }

    pub fn get_preferences_path() -> eyre::Result<PathBuf> {
        let config_dir = dirs::config_dir()
            .ok_or_else(|| eyre::eyre!("Could not determine config directory"))?;

        Ok(config_dir.join("dora").join("preferences.toml"))
    }

    fn ensure_preferences_dir(&self) -> eyre::Result<()> {
        let config_path = Self::get_preferences_path()?;
        if let Some(parent) = config_path.parent() {
            std::fs::create_dir_all(parent)?;
        }
        Ok(())
    }

    fn validate(&self) -> eyre::Result<()> {
        // Validate thresholds are in range
        if self.interface.complexity_thresholds.suggestion_threshold > 10 {
            return Err(eyre::eyre!("Suggestion threshold must be 0-10"));
        }

        if self.interface.auto_launch.confidence_threshold < 0.0
            || self.interface.auto_launch.confidence_threshold > 1.0
        {
            return Err(eyre::eyre!("Confidence threshold must be 0.0-1.0"));
        }

        // Validate adaptation weights sum to reasonable value
        let total_weight = self.behavior.adaptation_weights.behavioral_weight
            + self.behavior.adaptation_weights.context_weight
            + self.behavior.adaptation_weights.complexity_weight
            + self.behavior.adaptation_weights.explicit_weight;

        if total_weight < 0.5 || total_weight > 4.0 {
            return Err(eyre::eyre!(
                "Adaptation weights should sum to a reasonable value"
            ));
        }

        Ok(())
    }

    fn migrate_if_needed(&mut self) -> eyre::Result<()> {
        const CURRENT_SCHEMA_VERSION: u32 = 1;

        if self.metadata.schema_version < CURRENT_SCHEMA_VERSION {
            // Perform migration based on schema version
            match self.metadata.schema_version {
                0 => {
                    // Migration from version 0 to 1
                    // For now, just update the version
                    self.metadata.schema_version = 1;
                }
                _ => {
                    return Err(eyre::eyre!(
                        "Unknown schema version: {}",
                        self.metadata.schema_version
                    ));
                }
            }

            self.metadata.updated_at = Utc::now();
            self.save()?;
        }

        Ok(())
    }

    pub fn predict_interface_preference(
        &self,
        command: &Command,
        context: &ExecutionContext,
    ) -> InterfacePreferencePrediction {
        let mut prediction = InterfacePreferencePrediction::new();

        // Apply explicit preferences first
        if let Some(explicit_pref) = self.get_explicit_preference(command) {
            prediction.add_factor(PreferenceFactor {
                factor_type: FactorType::ExplicitPreference,
                weight: self.behavior.adaptation_weights.explicit_weight,
                preference: explicit_pref,
                confidence: 1.0,
            });
        }

        // Apply behavioral learning
        if self.behavior.learning_enabled {
            if let Some(behavioral_pref) = self.get_behavioral_preference(command, context) {
                prediction.add_factor(PreferenceFactor {
                    factor_type: FactorType::BehavioralLearning,
                    weight: self.behavior.adaptation_weights.behavioral_weight,
                    preference: behavioral_pref.preference,
                    confidence: behavioral_pref.confidence,
                });
            }
        }

        // Apply context-based preferences
        if let Some(context_pref) = self.get_context_preference(command, context) {
            prediction.add_factor(PreferenceFactor {
                factor_type: FactorType::ContextBased,
                weight: self.behavior.adaptation_weights.context_weight,
                preference: context_pref.preference,
                confidence: context_pref.confidence,
            });
        }

        // Apply complexity-based preferences
        let complexity_pref = self.get_complexity_preference(command, context);
        prediction.add_factor(PreferenceFactor {
            factor_type: FactorType::ComplexityBased,
            weight: self.behavior.adaptation_weights.complexity_weight,
            preference: complexity_pref.preference,
            confidence: complexity_pref.confidence,
        });

        prediction.finalize()
    }

    fn get_explicit_preference(&self, command: &Command) -> Option<UiMode> {
        let command_name = command.name();

        // Check command-specific override
        if let Some(mode) = self.commands.command_ui_modes.get(command_name) {
            return Some(mode.clone());
        }

        // Fall back to default UI mode
        Some(self.interface.default_ui_mode)
    }

    fn get_behavioral_preference(
        &self,
        command: &Command,
        context: &ExecutionContext,
    ) -> Option<BehavioralPreference> {
        let command_name = command.name();

        // Check usage statistics
        if let Some(stats) = self.behavior.command_usage.get(command_name) {
            if stats.total_uses >= 5 {
                // Minimum usage for reliable prediction
                // Find most used interface for this command
                let preferred_interface = stats
                    .interface_distribution
                    .iter()
                    .max_by_key(|(_, count)| *count)
                    .map(|(interface, count)| {
                        let confidence = *count as f32 / stats.total_uses as f32;
                        BehavioralPreference {
                            preference: interface.clone(),
                            confidence,
                        }
                    });

                return preferred_interface;
            }
        }

        // Check for patterns
        let context_key = format!("{}:{}", command_name, self.context_signature(context));
        if let Some(pattern) = self.behavior.action_patterns.get(&context_key) {
            if let Some(preferred) = &pattern.preferred_interface {
                return Some(BehavioralPreference {
                    preference: preferred.clone(),
                    confidence: pattern.frequency,
                });
            }
        }

        None
    }

    fn get_context_preference(
        &self,
        _command: &Command,
        context: &ExecutionContext,
    ) -> Option<BehavioralPreference> {
        // Apply context-based rules

        // CI/scripted environments prefer minimal
        if context.is_scripted || context.environment.is_ci {
            return Some(BehavioralPreference {
                preference: UiMode::Minimal,
                confidence: 0.9,
            });
        }

        // Small terminals prefer CLI
        if let Some((width, height)) = context.terminal_size {
            if width < 80 || height < 24 {
                return Some(BehavioralPreference {
                    preference: UiMode::Cli,
                    confidence: 0.8,
                });
            }
        }

        // Non-TTY environments prefer minimal
        if !context.is_tty {
            return Some(BehavioralPreference {
                preference: UiMode::Minimal,
                confidence: 0.9,
            });
        }

        None
    }

    fn get_complexity_preference(
        &self,
        command: &Command,
        _context: &ExecutionContext,
    ) -> BehavioralPreference {
        let complexity_score = self.calculate_command_complexity(command);

        if complexity_score >= self.interface.complexity_thresholds.suggestion_threshold {
            BehavioralPreference {
                preference: UiMode::Tui,
                confidence: (complexity_score as f32) / 10.0,
            }
        } else if complexity_score <= 3 {
            BehavioralPreference {
                preference: UiMode::Cli,
                confidence: (10 - complexity_score) as f32 / 10.0,
            }
        } else {
            BehavioralPreference {
                preference: UiMode::Auto,
                confidence: 0.5,
            }
        }
    }

    fn calculate_command_complexity(&self, command: &Command) -> u8 {
        match command {
            Command::Ps(_) => 2,
            Command::Start(_) => 4,
            Command::Stop(_) => 3,
            Command::Logs(_) => 5,
            Command::Build(_) => 6,
            Command::Inspect(_) => 8,
            Command::Debug(_) => 9,
            Command::Analyze(_) => 9,
            Command::Monitor(_) => 7,
            Command::Tui(_) => 1,
            Command::Dashboard(_) => 1,
            _ => 5, // Default complexity
        }
    }

    fn context_signature(&self, context: &ExecutionContext) -> String {
        format!(
            "tty:{},scripted:{},size:{:?}",
            context.is_tty, context.is_scripted, context.terminal_size
        )
    }
}

impl InterfacePreferencePrediction {
    pub fn new() -> Self {
        Self {
            predicted_mode: UiMode::Auto,
            confidence: 0.0,
            factors: Vec::new(),
        }
    }

    pub fn add_factor(&mut self, factor: PreferenceFactor) {
        self.factors.push(factor);
    }

    pub fn finalize(mut self) -> Self {
        if self.factors.is_empty() {
            return self;
        }

        // Calculate weighted average
        let mut mode_weights: HashMap<UiMode, f32> = HashMap::new();
        let mut total_weight = 0.0;

        for factor in &self.factors {
            let weighted_confidence = factor.weight * factor.confidence;
            *mode_weights.entry(factor.preference.clone()).or_insert(0.0) += weighted_confidence;
            total_weight += factor.weight;
        }

        // Find the mode with highest weighted confidence
        if let Some((predicted_mode, confidence)) = mode_weights
            .into_iter()
            .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
        {
            self.predicted_mode = predicted_mode;
            self.confidence = if total_weight > 0.0 {
                confidence / total_weight
            } else {
                0.0
            };
        }

        self
    }
}

impl ContextSnapshot {
    pub fn from_context(context: &ExecutionContext) -> Self {
        use chrono::prelude::*;
        let now = Local::now();

        Self {
            is_tty: context.is_tty,
            is_scripted: context.is_scripted,
            terminal_size: context.terminal_size,
            time_of_day: now.hour() as u8,
            day_of_week: now.weekday().num_days_from_sunday() as u8,
            environment_type: if context.environment.is_ci {
                "ci".to_string()
            } else if context.is_tty {
                "interactive".to_string()
            } else {
                "scripted".to_string()
            },
        }
    }
}

impl Command {
    pub fn name(&self) -> &str {
        match self {
            Command::Ps(_) => "ps",
            Command::Start(_) => "start",
            Command::Stop(_) => "stop",
            Command::Logs(_) => "logs",
            Command::Build(_) => "build",
            Command::Up(_) => "up",
            Command::Destroy(_) => "destroy",
            Command::New(_) => "new",
            Command::Check(_) => "check",
            Command::Graph(_) => "graph",
            Command::Inspect(_) => "inspect",
            Command::Debug(_) => "debug",
            Command::Analyze(_) => "analyze",
            Command::Monitor(_) => "monitor",
            Command::Tui(_) => "ui",
            Command::Dashboard(_) => "dashboard",
            Command::System(_) => "system",
            Command::Config(_) => "config",
            Command::Daemon(_) => "daemon",
            Command::Runtime(_) => "runtime",
            Command::Coordinator(_) => "coordinator",
            Command::Self_(_) => "self",
            Command::Help(_) => "help",
        }
    }
}

impl Default for UserPreferences {
    fn default() -> Self {
        Self {
            interface: InterfacePreferences::default(),
            commands: CommandPreferences::default(),
            behavior: BehaviorPreferences::default(),
            environments: HashMap::new(),
            metadata: PreferenceMetadata {
                version: env!("CARGO_PKG_VERSION").to_string(),
                created_at: Utc::now(),
                updated_at: Utc::now(),
                user_id: None,
                schema_version: 1,
            },
        }
    }
}

impl Default for InterfacePreferences {
    fn default() -> Self {
        Self {
            default_ui_mode: UiMode::Auto,
            complexity_thresholds: ComplexityThresholds {
                suggestion_threshold: 6,
                auto_launch_threshold: 8,
                hint_threshold: 4,
            },
            auto_launch: AutoLaunchPreferences {
                confidence_threshold: 0.8,
                blacklist: vec!["ps".to_string(), "stop".to_string()],
                whitelist: vec!["debug".to_string(), "analyze".to_string()],
                delay_ms: 500,
            },
            hints: HintPreferences {
                show_hints: true,
                hint_frequency: HintFrequency::Normal,
                hint_types: vec![
                    HintType::InterfaceSuggestion,
                    HintType::CommandTip,
                    HintType::ShortcutHint,
                ],
                dismissed_hints: Vec::new(),
            },
            tui: TuiPreferences {
                theme: "default".to_string(),
                auto_refresh_interval: Duration::from_secs(5),
                mouse_support: true,
                key_bindings: HashMap::new(),
                default_view: "dashboard".to_string(),
            },
            cli: CliPreferences {
                output_width: None,
                color_mode: ColorMode::Auto,
                pagination: true,
                command_aliases: HashMap::new(),
                default_output_format: "auto".to_string(),
            },
        }
    }
}

impl Default for CommandPreferences {
    fn default() -> Self {
        Self {
            command_ui_modes: HashMap::new(),
            default_flags: HashMap::new(),
            aliases: HashMap::new(),
            completion: CompletionPreferences {
                enabled: true,
                show_descriptions: true,
                max_suggestions: 10,
                context_aware: true,
            },
        }
    }
}

impl Default for BehaviorPreferences {
    fn default() -> Self {
        Self {
            action_patterns: HashMap::new(),
            command_usage: HashMap::new(),
            interface_choices: VecDeque::new(),
            learning_enabled: true,
            adaptation_weights: AdaptationWeights {
                behavioral_weight: 0.4,
                context_weight: 0.3,
                complexity_weight: 0.2,
                explicit_weight: 1.0,
            },
            max_history_size: 1000,
        }
    }
}

impl Default for UsageStatistics {
    fn default() -> Self {
        Self {
            total_uses: 0,
            interface_distribution: HashMap::new(),
            success_rate: 0.0,
            average_session_length: Duration::from_secs(0),
            last_used: Utc::now(),
            user_rating: None,
        }
    }
}

// Custom serialization for Duration
mod duration_serde {
    use serde::{Deserialize, Deserializer, Serialize, Serializer};
    use std::time::Duration;

    pub fn serialize<S>(duration: &Duration, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        duration.as_secs().serialize(serializer)
    }

    pub fn deserialize<'de, D>(deserializer: D) -> Result<Duration, D::Error>
    where
        D: Deserializer<'de>,
    {
        let secs = u64::deserialize(deserializer)?;
        Ok(Duration::from_secs(secs))
    }
}
