// Context Awareness Types for TUI View
// Issue #35: Advanced Context Awareness

use chrono::{DateTime, Timelike, Utc};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Main sections of the context awareness view
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum ContextSection {
    ExecutionContext,
    ContextPreferences,
    BehavioralLearning,
    PreferenceResolution,
}

impl ContextSection {
    pub fn all() -> Vec<Self> {
        vec![
            Self::ExecutionContext,
            Self::ContextPreferences,
            Self::BehavioralLearning,
            Self::PreferenceResolution,
        ]
    }

    pub fn title(&self) -> &str {
        match self {
            Self::ExecutionContext => "Execution Context",
            Self::ContextPreferences => "Context Preferences",
            Self::BehavioralLearning => "Behavioral Learning",
            Self::PreferenceResolution => "Preference Resolution",
        }
    }

    pub fn description(&self) -> &str {
        match self {
            Self::ExecutionContext => "Current environment, terminal capabilities, and automation detection",
            Self::ContextPreferences => "Active preferences and adaptation rules based on context",
            Self::BehavioralLearning => "Usage patterns, command frequency, and learning statistics",
            Self::PreferenceResolution => "Preference resolution hierarchy and active overrides",
        }
    }
}

/// Execution environment information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExecutionEnvironment {
    pub is_tty: bool,
    pub is_piped: bool,
    pub is_scripted: bool,
    pub detected_ci: Option<CiEnvironment>,
    pub terminal_type: Option<String>,
    pub shell: Option<String>,
    pub detected_at: DateTime<Utc>,
}

/// CI/CD environment types
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum CiEnvironment {
    GitHubActions,
    GitLabCI,
    CircleCI,
    TravisCI,
    Jenkins,
    Azure,
    Buildkite,
    TeamCity,
    Other(String),
}

impl CiEnvironment {
    pub fn name(&self) -> &str {
        match self {
            Self::GitHubActions => "GitHub Actions",
            Self::GitLabCI => "GitLab CI",
            Self::CircleCI => "CircleCI",
            Self::TravisCI => "Travis CI",
            Self::Jenkins => "Jenkins",
            Self::Azure => "Azure Pipelines",
            Self::Buildkite => "Buildkite",
            Self::TeamCity => "TeamCity",
            Self::Other(name) => name,
        }
    }
}

/// Terminal capability information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TerminalCapabilities {
    pub supports_color: bool,
    pub color_level: ColorLevel,
    pub supports_unicode: bool,
    pub supports_mouse: bool,
    pub terminal_width: u16,
    pub terminal_height: u16,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum ColorLevel {
    None,
    Basic,      // 16 colors
    Extended,   // 256 colors
    TrueColor,  // 24-bit colors
}

impl ColorLevel {
    pub fn name(&self) -> &str {
        match self {
            Self::None => "None",
            Self::Basic => "Basic (16 colors)",
            Self::Extended => "Extended (256 colors)",
            Self::TrueColor => "TrueColor (24-bit)",
        }
    }
}

/// Context key for preference lookup
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct ContextKey {
    pub command_category: CommandCategory,
    pub time_of_day: TimeOfDay,
    pub terminal_type: TerminalType,
    pub environment_type: EnvironmentType,
}

#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum CommandCategory {
    Monitor,
    Debug,
    Analysis,
    Management,
    Other,
}

impl CommandCategory {
    pub fn name(&self) -> &str {
        match self {
            Self::Monitor => "Monitor",
            Self::Debug => "Debug",
            Self::Analysis => "Analysis",
            Self::Management => "Management",
            Self::Other => "Other",
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum TimeOfDay {
    Morning,
    Afternoon,
    Evening,
    Night,
}

impl TimeOfDay {
    pub fn name(&self) -> &str {
        match self {
            Self::Morning => "Morning (6am-12pm)",
            Self::Afternoon => "Afternoon (12pm-6pm)",
            Self::Evening => "Evening (6pm-10pm)",
            Self::Night => "Night (10pm-6am)",
        }
    }

    pub fn from_hour(hour: u32) -> Self {
        match hour {
            6..=11 => Self::Morning,
            12..=17 => Self::Afternoon,
            18..=21 => Self::Evening,
            _ => Self::Night,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum TerminalType {
    Interactive,
    Piped,
    Scripted,
}

impl TerminalType {
    pub fn name(&self) -> &str {
        match self {
            Self::Interactive => "Interactive",
            Self::Piped => "Piped",
            Self::Scripted => "Scripted",
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum EnvironmentType {
    Development,
    CI,
    Production,
}

impl EnvironmentType {
    pub fn name(&self) -> &str {
        match self {
            Self::Development => "Development",
            Self::CI => "CI/CD",
            Self::Production => "Production",
        }
    }
}

/// Adaptation rule for context-based preference changes
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AdaptationRule {
    pub rule_id: String,
    pub name: String,
    pub rule_type: AdaptationRuleType,
    pub condition: String,
    pub action: String,
    pub priority: i32,
    pub is_active: bool,
    pub trigger_count: usize,
    pub last_triggered: Option<DateTime<Utc>>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum AdaptationRuleType {
    TimeBased,
    EnvironmentBased,
    ComplexityBased,
    UsagePatternBased,
}

impl AdaptationRuleType {
    pub fn name(&self) -> &str {
        match self {
            Self::TimeBased => "Time-Based",
            Self::EnvironmentBased => "Environment-Based",
            Self::ComplexityBased => "Complexity-Based",
            Self::UsagePatternBased => "Usage Pattern-Based",
        }
    }
}

/// Behavioral learning pattern
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LearningPattern {
    pub pattern_id: String,
    pub pattern_type: PatternType,
    pub description: String,
    pub confidence: f64, // 0.0 to 1.0
    pub sample_size: usize,
    pub discovered_at: DateTime<Utc>,
    pub last_observed: DateTime<Utc>,
    pub metadata: HashMap<String, String>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum PatternType {
    CommandSequence,
    TimePreference,
    InterfaceChoice,
    WorkflowPattern,
}

impl PatternType {
    pub fn name(&self) -> &str {
        match self {
            Self::CommandSequence => "Command Sequence",
            Self::TimePreference => "Time Preference",
            Self::InterfaceChoice => "Interface Choice",
            Self::WorkflowPattern => "Workflow Pattern",
        }
    }
}

/// Command usage statistics
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CommandUsage {
    pub command: String,
    pub total_uses: usize,
    pub success_rate: f64,
    pub avg_duration_ms: f64,
    pub last_used: DateTime<Utc>,
    pub common_contexts: Vec<String>,
}

/// Interface preference statistics
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InterfacePreference {
    pub interface_type: InterfaceType,
    pub usage_count: usize,
    pub preference_score: f64, // 0.0 to 1.0
    pub context_affinity: HashMap<String, f64>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum InterfaceType {
    CLI,
    TUI,
    Auto,
}

impl InterfaceType {
    pub fn name(&self) -> &str {
        match self {
            Self::CLI => "CLI Only",
            Self::TUI => "TUI Preferred",
            Self::Auto => "Auto-Select",
        }
    }
}

/// Preference resolution level in hierarchy
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PreferenceLevel {
    pub level: ResolutionLevel,
    pub source: String,
    pub preferences: HashMap<String, String>,
    pub is_active: bool,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Copy)]
pub enum ResolutionLevel {
    TemporaryOverride = 6,
    ContextAdaptation = 5,
    UserPreference = 4,
    WorkspaceConfig = 3,
    ProjectDefault = 2,
    SystemDefault = 1,
}

impl ResolutionLevel {
    pub fn all() -> Vec<Self> {
        vec![
            Self::TemporaryOverride,
            Self::ContextAdaptation,
            Self::UserPreference,
            Self::WorkspaceConfig,
            Self::ProjectDefault,
            Self::SystemDefault,
        ]
    }

    pub fn name(&self) -> &str {
        match self {
            Self::TemporaryOverride => "Temporary Override",
            Self::ContextAdaptation => "Context Adaptation",
            Self::UserPreference => "User Preference",
            Self::WorkspaceConfig => "Workspace Config",
            Self::ProjectDefault => "Project Default",
            Self::SystemDefault => "System Default",
        }
    }

    pub fn description(&self) -> &str {
        match self {
            Self::TemporaryOverride => "Command-line flags and runtime overrides",
            Self::ContextAdaptation => "Automatically adapted based on current context",
            Self::UserPreference => "User's explicitly configured preferences",
            Self::WorkspaceConfig => "Workspace-specific configuration",
            Self::ProjectDefault => "Project-level defaults from config file",
            Self::SystemDefault => "System-wide default values",
        }
    }

    pub fn priority(&self) -> i32 {
        *self as i32
    }
}

/// State for the context awareness view
#[derive(Debug, Clone)]
pub struct ContextAwarenessState {
    pub current_section: ContextSection,
    pub selected_index: usize,
    pub data: ContextData,
}

impl ContextAwarenessState {
    pub fn new() -> Self {
        Self {
            current_section: ContextSection::ExecutionContext,
            selected_index: 0,
            data: ContextData::ExecutionEnvironment(create_mock_execution_env()),
        }
    }

    pub fn next_section(&mut self) {
        let sections = ContextSection::all();
        let current_idx = sections.iter().position(|s| s == &self.current_section).unwrap_or(0);
        let next_idx = (current_idx + 1) % sections.len();
        self.current_section = sections[next_idx].clone();
        self.selected_index = 0;
        self.update_data();
    }

    pub fn previous_section(&mut self) {
        let sections = ContextSection::all();
        let current_idx = sections.iter().position(|s| s == &self.current_section).unwrap_or(0);
        let prev_idx = if current_idx == 0 { sections.len() - 1 } else { current_idx - 1 };
        self.current_section = sections[prev_idx].clone();
        self.selected_index = 0;
        self.update_data();
    }

    pub fn update_data(&mut self) {
        self.data = match &self.current_section {
            ContextSection::ExecutionContext => ContextData::ExecutionEnvironment(create_mock_execution_env()),
            ContextSection::ContextPreferences => ContextData::AdaptationRules(create_mock_adaptation_rules()),
            ContextSection::BehavioralLearning => ContextData::LearningPatterns(create_mock_learning_patterns()),
            ContextSection::PreferenceResolution => ContextData::PreferenceLevels(create_mock_preference_levels()),
        };
    }

    pub fn next_item(&mut self) {
        let max_index = self.get_max_index();
        if max_index > 0 {
            self.selected_index = (self.selected_index + 1).min(max_index - 1);
        }
    }

    pub fn previous_item(&mut self) {
        if self.selected_index > 0 {
            self.selected_index -= 1;
        }
    }

    fn get_max_index(&self) -> usize {
        match &self.data {
            ContextData::ExecutionEnvironment(_) => 1,
            ContextData::AdaptationRules(rules) => rules.len(),
            ContextData::LearningPatterns(patterns) => patterns.len(),
            ContextData::PreferenceLevels(levels) => levels.len(),
        }
    }
}

impl Default for ContextAwarenessState {
    fn default() -> Self {
        Self::new()
    }
}

/// Data container for different sections
#[derive(Debug, Clone)]
pub enum ContextData {
    ExecutionEnvironment(ExecutionEnvironmentData),
    AdaptationRules(Vec<AdaptationRule>),
    LearningPatterns(Vec<LearningPattern>),
    PreferenceLevels(Vec<PreferenceLevel>),
}

/// Combined execution environment data
#[derive(Debug, Clone)]
pub struct ExecutionEnvironmentData {
    pub environment: ExecutionEnvironment,
    pub capabilities: TerminalCapabilities,
    pub current_context: ContextKey,
}

// Mock data generators for Phase 1 implementation

pub fn create_mock_execution_env() -> ExecutionEnvironmentData {
    ExecutionEnvironmentData {
        environment: ExecutionEnvironment {
            is_tty: true,
            is_piped: false,
            is_scripted: false,
            detected_ci: None,
            terminal_type: Some("xterm-256color".to_string()),
            shell: Some("zsh".to_string()),
            detected_at: Utc::now(),
        },
        capabilities: TerminalCapabilities {
            supports_color: true,
            color_level: ColorLevel::TrueColor,
            supports_unicode: true,
            supports_mouse: true,
            terminal_width: 120,
            terminal_height: 40,
        },
        current_context: ContextKey {
            command_category: CommandCategory::Monitor,
            time_of_day: TimeOfDay::from_hour(Utc::now().hour()),
            terminal_type: TerminalType::Interactive,
            environment_type: EnvironmentType::Development,
        },
    }
}

pub fn create_mock_adaptation_rules() -> Vec<AdaptationRule> {
    vec![
        AdaptationRule {
            rule_id: "rule_001".to_string(),
            name: "Auto-TUI for Complex Analysis".to_string(),
            rule_type: AdaptationRuleType::ComplexityBased,
            condition: "complexity_score > 7".to_string(),
            action: "prefer TUI interface".to_string(),
            priority: 10,
            is_active: true,
            trigger_count: 42,
            last_triggered: Some(Utc::now()),
        },
        AdaptationRule {
            rule_id: "rule_002".to_string(),
            name: "CLI for CI/CD Environments".to_string(),
            rule_type: AdaptationRuleType::EnvironmentBased,
            condition: "detected_ci != None".to_string(),
            action: "use CLI-only mode".to_string(),
            priority: 9,
            is_active: true,
            trigger_count: 128,
            last_triggered: Some(Utc::now()),
        },
        AdaptationRule {
            rule_id: "rule_003".to_string(),
            name: "Compact Output at Night".to_string(),
            rule_type: AdaptationRuleType::TimeBased,
            condition: "time_of_day == Night".to_string(),
            action: "use compact output format".to_string(),
            priority: 5,
            is_active: true,
            trigger_count: 24,
            last_triggered: Some(Utc::now()),
        },
        AdaptationRule {
            rule_id: "rule_004".to_string(),
            name: "Verbose Debug in Development".to_string(),
            rule_type: AdaptationRuleType::EnvironmentBased,
            condition: "environment == Development".to_string(),
            action: "enable verbose logging".to_string(),
            priority: 6,
            is_active: true,
            trigger_count: 156,
            last_triggered: Some(Utc::now()),
        },
        AdaptationRule {
            rule_id: "rule_005".to_string(),
            name: "Dashboard After Monitoring".to_string(),
            rule_type: AdaptationRuleType::UsagePatternBased,
            condition: "previous_command == monitor".to_string(),
            action: "suggest dashboard view".to_string(),
            priority: 7,
            is_active: false,
            trigger_count: 18,
            last_triggered: Some(Utc::now()),
        },
    ]
}

pub fn create_mock_learning_patterns() -> Vec<LearningPattern> {
    vec![
        LearningPattern {
            pattern_id: "pattern_001".to_string(),
            pattern_type: PatternType::CommandSequence,
            description: "Often runs 'dora ps' followed by 'dora logs'".to_string(),
            confidence: 0.87,
            sample_size: 45,
            discovered_at: Utc::now(),
            last_observed: Utc::now(),
            metadata: HashMap::from([
                ("sequence".to_string(), "ps -> logs".to_string()),
                ("avg_delay_seconds".to_string(), "3.2".to_string()),
            ]),
        },
        LearningPattern {
            pattern_id: "pattern_002".to_string(),
            pattern_type: PatternType::TimePreference,
            description: "Prefers TUI interface during morning hours".to_string(),
            confidence: 0.92,
            sample_size: 78,
            discovered_at: Utc::now(),
            last_observed: Utc::now(),
            metadata: HashMap::from([
                ("time_range".to_string(), "6am-11am".to_string()),
                ("interface".to_string(), "TUI".to_string()),
            ]),
        },
        LearningPattern {
            pattern_id: "pattern_003".to_string(),
            pattern_type: PatternType::InterfaceChoice,
            description: "Uses CLI for quick status checks, TUI for debugging".to_string(),
            confidence: 0.95,
            sample_size: 120,
            discovered_at: Utc::now(),
            last_observed: Utc::now(),
            metadata: HashMap::from([
                ("quick_commands".to_string(), "ps, status".to_string()),
                ("deep_commands".to_string(), "debug, analyze".to_string()),
            ]),
        },
        LearningPattern {
            pattern_id: "pattern_004".to_string(),
            pattern_type: PatternType::WorkflowPattern,
            description: "Debug workflow: analyze -> inspect nodes -> check logs".to_string(),
            confidence: 0.81,
            sample_size: 32,
            discovered_at: Utc::now(),
            last_observed: Utc::now(),
            metadata: HashMap::from([
                ("workflow".to_string(), "analyze-inspect-logs".to_string()),
                ("success_rate".to_string(), "0.94".to_string()),
            ]),
        },
    ]
}

pub fn create_mock_preference_levels() -> Vec<PreferenceLevel> {
    ResolutionLevel::all()
        .into_iter()
        .map(|level| {
            let (is_active, prefs) = match level {
                ResolutionLevel::TemporaryOverride => (
                    false,
                    HashMap::new(),
                ),
                ResolutionLevel::ContextAdaptation => (
                    true,
                    HashMap::from([
                        ("interface_mode".to_string(), "auto".to_string()),
                        ("output_format".to_string(), "rich".to_string()),
                    ]),
                ),
                ResolutionLevel::UserPreference => (
                    true,
                    HashMap::from([
                        ("theme".to_string(), "dark".to_string()),
                        ("log_level".to_string(), "info".to_string()),
                        ("auto_refresh".to_string(), "enabled".to_string()),
                    ]),
                ),
                ResolutionLevel::WorkspaceConfig => (
                    true,
                    HashMap::from([
                        ("dataflow_path".to_string(), "./dataflows".to_string()),
                        ("default_coordinator".to_string(), "localhost:7010".to_string()),
                    ]),
                ),
                ResolutionLevel::ProjectDefault => (
                    true,
                    HashMap::from([
                        ("build_mode".to_string(), "release".to_string()),
                        ("test_timeout".to_string(), "30s".to_string()),
                    ]),
                ),
                ResolutionLevel::SystemDefault => (
                    true,
                    HashMap::from([
                        ("config_dir".to_string(), "~/.config/dora".to_string()),
                        ("cache_dir".to_string(), "~/.cache/dora".to_string()),
                    ]),
                ),
            };
            PreferenceLevel {
                level,
                source: format!("{:?}", level),
                preferences: prefs,
                is_active,
            }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_context_section_all() {
        let sections = ContextSection::all();
        assert_eq!(sections.len(), 4);
        assert_eq!(sections[0], ContextSection::ExecutionContext);
        assert_eq!(sections[1], ContextSection::ContextPreferences);
        assert_eq!(sections[2], ContextSection::BehavioralLearning);
        assert_eq!(sections[3], ContextSection::PreferenceResolution);
    }

    #[test]
    fn test_context_section_title() {
        assert_eq!(ContextSection::ExecutionContext.title(), "Execution Context");
        assert_eq!(ContextSection::ContextPreferences.title(), "Context Preferences");
        assert_eq!(ContextSection::BehavioralLearning.title(), "Behavioral Learning");
        assert_eq!(ContextSection::PreferenceResolution.title(), "Preference Resolution");
    }

    #[test]
    fn test_ci_environment_name() {
        assert_eq!(CiEnvironment::GitHubActions.name(), "GitHub Actions");
        assert_eq!(CiEnvironment::GitLabCI.name(), "GitLab CI");
        assert_eq!(CiEnvironment::Other("Custom".to_string()).name(), "Custom");
    }

    #[test]
    fn test_color_level_name() {
        assert_eq!(ColorLevel::None.name(), "None");
        assert_eq!(ColorLevel::Basic.name(), "Basic (16 colors)");
        assert_eq!(ColorLevel::Extended.name(), "Extended (256 colors)");
        assert_eq!(ColorLevel::TrueColor.name(), "TrueColor (24-bit)");
    }

    #[test]
    fn test_time_of_day_from_hour() {
        assert_eq!(TimeOfDay::from_hour(8), TimeOfDay::Morning);
        assert_eq!(TimeOfDay::from_hour(14), TimeOfDay::Afternoon);
        assert_eq!(TimeOfDay::from_hour(19), TimeOfDay::Evening);
        assert_eq!(TimeOfDay::from_hour(23), TimeOfDay::Night);
        assert_eq!(TimeOfDay::from_hour(2), TimeOfDay::Night);
    }

    #[test]
    fn test_resolution_level_priority() {
        assert_eq!(ResolutionLevel::TemporaryOverride.priority(), 6);
        assert_eq!(ResolutionLevel::ContextAdaptation.priority(), 5);
        assert_eq!(ResolutionLevel::UserPreference.priority(), 4);
        assert_eq!(ResolutionLevel::SystemDefault.priority(), 1);
    }

    #[test]
    fn test_resolution_level_all() {
        let levels = ResolutionLevel::all();
        assert_eq!(levels.len(), 6);
        assert_eq!(levels[0].priority(), 6); // Highest priority first
        assert_eq!(levels[5].priority(), 1); // Lowest priority last
    }

    #[test]
    fn test_context_awareness_state_new() {
        let state = ContextAwarenessState::new();
        assert_eq!(state.current_section, ContextSection::ExecutionContext);
        assert_eq!(state.selected_index, 0);
        assert!(matches!(state.data, ContextData::ExecutionEnvironment(_)));
    }

    #[test]
    fn test_context_awareness_state_navigation() {
        let mut state = ContextAwarenessState::new();

        state.next_section();
        assert_eq!(state.current_section, ContextSection::ContextPreferences);
        assert!(matches!(state.data, ContextData::AdaptationRules(_)));

        state.next_section();
        assert_eq!(state.current_section, ContextSection::BehavioralLearning);
        assert!(matches!(state.data, ContextData::LearningPatterns(_)));

        state.previous_section();
        assert_eq!(state.current_section, ContextSection::ContextPreferences);
    }

    #[test]
    fn test_context_awareness_state_wrapping() {
        let mut state = ContextAwarenessState::new();

        // Wrap forward
        state.current_section = ContextSection::PreferenceResolution;
        state.next_section();
        assert_eq!(state.current_section, ContextSection::ExecutionContext);

        // Wrap backward
        state.previous_section();
        assert_eq!(state.current_section, ContextSection::PreferenceResolution);
    }

    #[test]
    fn test_mock_execution_env_creation() {
        let env_data = create_mock_execution_env();
        assert_eq!(env_data.environment.is_tty, true);
        assert_eq!(env_data.environment.is_piped, false);
        assert_eq!(env_data.capabilities.supports_color, true);
        assert_eq!(env_data.capabilities.color_level, ColorLevel::TrueColor);
    }

    #[test]
    fn test_mock_adaptation_rules_creation() {
        let rules = create_mock_adaptation_rules();
        assert_eq!(rules.len(), 5);
        assert_eq!(rules[0].rule_id, "rule_001");
        assert_eq!(rules[0].rule_type, AdaptationRuleType::ComplexityBased);
        assert_eq!(rules[0].is_active, true);
        assert_eq!(rules[4].is_active, false);
    }

    #[test]
    fn test_mock_learning_patterns_creation() {
        let patterns = create_mock_learning_patterns();
        assert_eq!(patterns.len(), 4);
        assert_eq!(patterns[0].pattern_type, PatternType::CommandSequence);
        assert!(patterns[0].confidence > 0.8);
        assert!(patterns[0].sample_size > 0);
    }

    #[test]
    fn test_mock_preference_levels_creation() {
        let levels = create_mock_preference_levels();
        assert_eq!(levels.len(), 6);

        // Check priorities are in order
        for i in 0..levels.len()-1 {
            assert!(levels[i].level.priority() > levels[i+1].level.priority());
        }
    }

    #[test]
    fn test_adaptation_rule_type_name() {
        assert_eq!(AdaptationRuleType::TimeBased.name(), "Time-Based");
        assert_eq!(AdaptationRuleType::EnvironmentBased.name(), "Environment-Based");
        assert_eq!(AdaptationRuleType::ComplexityBased.name(), "Complexity-Based");
        assert_eq!(AdaptationRuleType::UsagePatternBased.name(), "Usage Pattern-Based");
    }

    #[test]
    fn test_pattern_type_name() {
        assert_eq!(PatternType::CommandSequence.name(), "Command Sequence");
        assert_eq!(PatternType::TimePreference.name(), "Time Preference");
        assert_eq!(PatternType::InterfaceChoice.name(), "Interface Choice");
        assert_eq!(PatternType::WorkflowPattern.name(), "Workflow Pattern");
    }

    #[test]
    fn test_interface_type_name() {
        assert_eq!(InterfaceType::CLI.name(), "CLI Only");
        assert_eq!(InterfaceType::TUI.name(), "TUI Preferred");
        assert_eq!(InterfaceType::Auto.name(), "Auto-Select");
    }

    #[test]
    fn test_command_category_name() {
        assert_eq!(CommandCategory::Monitor.name(), "Monitor");
        assert_eq!(CommandCategory::Debug.name(), "Debug");
        assert_eq!(CommandCategory::Analysis.name(), "Analysis");
        assert_eq!(CommandCategory::Management.name(), "Management");
        assert_eq!(CommandCategory::Other.name(), "Other");
    }

    #[test]
    fn test_context_key_equality() {
        let key1 = ContextKey {
            command_category: CommandCategory::Monitor,
            time_of_day: TimeOfDay::Morning,
            terminal_type: TerminalType::Interactive,
            environment_type: EnvironmentType::Development,
        };
        let key2 = ContextKey {
            command_category: CommandCategory::Monitor,
            time_of_day: TimeOfDay::Morning,
            terminal_type: TerminalType::Interactive,
            environment_type: EnvironmentType::Development,
        };
        assert_eq!(key1, key2);
    }

    #[test]
    fn test_execution_environment_ci_detection() {
        let env = ExecutionEnvironment {
            is_tty: false,
            is_piped: true,
            is_scripted: true,
            detected_ci: Some(CiEnvironment::GitHubActions),
            terminal_type: None,
            shell: None,
            detected_at: Utc::now(),
        };
        assert!(env.detected_ci.is_some());
        assert_eq!(env.detected_ci.unwrap().name(), "GitHub Actions");
    }

    #[test]
    fn test_terminal_capabilities_structure() {
        let caps = TerminalCapabilities {
            supports_color: true,
            color_level: ColorLevel::Extended,
            supports_unicode: true,
            supports_mouse: false,
            terminal_width: 80,
            terminal_height: 24,
        };
        assert_eq!(caps.terminal_width, 80);
        assert_eq!(caps.terminal_height, 24);
        assert_eq!(caps.supports_mouse, false);
    }

    #[test]
    fn test_learning_pattern_confidence_range() {
        let patterns = create_mock_learning_patterns();
        for pattern in patterns {
            assert!(pattern.confidence >= 0.0 && pattern.confidence <= 1.0);
        }
    }

    #[test]
    fn test_preference_level_descriptions() {
        for level in ResolutionLevel::all() {
            let desc = level.description();
            assert!(!desc.is_empty());
        }
    }

    #[test]
    fn test_adaptation_rule_priority_ordering() {
        let rules = create_mock_adaptation_rules();
        let max_priority = rules.iter().map(|r| r.priority).max().unwrap();
        let min_priority = rules.iter().map(|r| r.priority).min().unwrap();
        assert!(max_priority >= min_priority);
        assert_eq!(max_priority, 10);
        assert_eq!(min_priority, 5);
    }
}
