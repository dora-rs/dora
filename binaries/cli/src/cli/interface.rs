use crate::cli::{Command, UiMode};
use crate::cli::context::ExecutionContext;
use std::collections::{HashMap, HashSet};
use std::num::NonZeroUsize;
use lru::LruCache;
// use serde::{Serialize, Deserialize}; // For future config serialization

/// Smart interface selection engine for hybrid CLI
#[derive(Debug)]
pub struct InterfaceSelector {
    context: ExecutionContext,
    config: UserConfig,
    command_analyzer: CommandAnalyzer,
    decision_cache: LruCache<String, InterfaceDecision>,
}

/// Decision result from interface selection
#[derive(Debug, Clone)]
pub struct InterfaceDecision {
    pub strategy: InterfaceStrategy,
    pub confidence: f32,
    pub reason: String,
    pub fallback: Option<InterfaceStrategy>,
}

/// Interface strategy options
#[derive(Debug, Clone, PartialEq)]
pub enum InterfaceStrategy {
    CliOnly,
    CliWithHint { 
        hint: String, 
        tui_command: String 
    },
    PromptForTui { 
        reason: String, 
        default_yes: bool 
    },
    AutoLaunchTui { 
        reason: String, 
        show_cli_first: bool 
    },
}

/// User configuration for interface preferences
#[derive(Debug, Clone)]
pub struct UserConfig {
    pub global_ui_mode: UiMode,
    pub complexity_threshold: u8,
    pub auto_launch_threshold: f32,
    pub command_preferences: HashMap<String, UiMode>,
    pub hint_preferences: HintPreferences,
}

/// Hint display preferences
#[derive(Debug, Clone)]
pub struct HintPreferences {
    pub show_hints: bool,
    pub hint_frequency: HintFrequency,
    pub dismissed_hints: HashSet<String>,
}

/// How often to show hints
#[derive(Debug, Clone)]
pub enum HintFrequency {
    Always,
    OncePerSession,
    OncePerCommand,
    Never,
}

/// Command complexity analyzer
#[derive(Debug)]
pub struct CommandAnalyzer {
    complexity_rules: ComplexityRules,
    interaction_patterns: InteractionPatterns,
}

/// Analysis result for a command
#[derive(Debug, Clone)]
pub struct CommandAnalysis {
    pub complexity_score: u8,           // 0-10 scale
    pub data_volume_estimate: DataVolume,
    pub interaction_benefit: InteractionBenefit,
    pub automation_suitability: AutomationLevel,
    pub output_characteristics: OutputCharacteristics,
}

/// Estimated data volume for command
#[derive(Debug, Clone, PartialEq)]
pub enum DataVolume {
    Minimal,        // Single values, short lists
    Small,          // Tables under 50 rows
    Medium,         // Tables 50-500 rows, simple graphs
    Large,          // Complex data requiring visualization
    Streaming,      // Real-time data streams
}

/// Benefit of interactive interface
#[derive(Debug, Clone, PartialEq)]
pub enum InteractionBenefit {
    None,           // Static output only
    Low,            // Minor filtering/sorting benefit
    Medium,         // Significant navigation benefit
    High,           // Essential for effective use
    Critical,       // Nearly unusable without interaction
}

/// Automation suitability levels
#[derive(Debug, Clone, PartialEq)]
pub enum AutomationLevel {
    OptimizedForAutomation,
    SuitableForAutomation,
    InteractivePreferred,
    InteractiveRequired,
}

/// Output characteristics
#[derive(Debug, Clone)]
pub struct OutputCharacteristics {
    pub is_real_time: bool,
    pub has_colors: bool,
    pub is_structured: bool,
    pub requires_scrolling: bool,
}

/// Complexity scoring rules
#[derive(Debug)]
pub struct ComplexityRules {
    pub base_scores: HashMap<String, u8>,
    pub modifier_rules: Vec<ComplexityModifier>,
}

/// Interaction pattern analysis
#[derive(Debug)]
pub struct InteractionPatterns {
    pub command_benefits: HashMap<String, InteractionBenefit>,
    pub data_volume_thresholds: HashMap<String, DataVolume>,
}

/// Complexity modifier rule
#[derive(Debug)]
pub struct ComplexityModifier {
    pub condition: String,
    pub modifier: i8,
}

impl InterfaceSelector {
    /// Create new interface selector
    pub fn new(context: ExecutionContext, config: UserConfig) -> Self {
        Self {
            context,
            config,
            command_analyzer: CommandAnalyzer::new(),
            decision_cache: LruCache::new(NonZeroUsize::new(100).unwrap()),
        }
    }
    
    /// Select the most appropriate interface for a command
    pub fn select_interface(&mut self, command: &Command) -> InterfaceDecision {
        // Check cache first for performance
        let cache_key = self.generate_cache_key(command);
        if let Some(cached) = self.decision_cache.get(&cache_key) {
            return cached.clone();
        }
        
        let decision = self.analyze_and_decide(command);
        self.decision_cache.put(cache_key, decision.clone());
        decision
    }
    
    /// Generate cache key for command
    fn generate_cache_key(&self, command: &Command) -> String {
        // Include relevant context and command details
        format!("{}:{}:{}:{}",
            command.name(),
            self.context.is_tty,
            self.context.is_piped,
            self.config.global_ui_mode.as_u8()
        )
    }
    
    /// Analyze command and make interface decision
    fn analyze_and_decide(&self, command: &Command) -> InterfaceDecision {
        let analysis = self.command_analyzer.analyze(command);
        
        // Early exit conditions
        if let Some(decision) = self.check_forced_contexts(&analysis) {
            return decision;
        }
        
        if let Some(decision) = self.check_user_preferences(command) {
            return decision;
        }
        
        // Smart decision based on multiple factors
        self.make_intelligent_decision(command, &analysis)
    }
    
    /// Check for contexts that force certain interfaces
    fn check_forced_contexts(&self, _analysis: &CommandAnalysis) -> Option<InterfaceDecision> {
        // Never TUI in non-interactive contexts
        if !self.context.is_tty || self.context.is_piped || self.context.is_scripted {
            return Some(InterfaceDecision {
                strategy: InterfaceStrategy::CliOnly,
                confidence: 1.0,
                reason: "Non-interactive environment detected".to_string(),
                fallback: None,
            });
        }
        
        // Force CLI if TUI not capable
        if !self.context.terminal_capabilities.tui_capable {
            return Some(InterfaceDecision {
                strategy: InterfaceStrategy::CliOnly,
                confidence: 0.9,
                reason: "Terminal not capable of TUI interface".to_string(),
                fallback: None,
            });
        }
        
        None
    }
    
    /// Check user preference overrides
    fn check_user_preferences(&self, command: &Command) -> Option<InterfaceDecision> {
        // Check command-specific preferences first
        let command_name = command.name();
        if let Some(mode) = self.config.command_preferences.get(&command_name) {
            return Some(self.create_preference_decision(mode, "User command preference"));
        }
        
        // Check global preferences
        match self.config.global_ui_mode {
            UiMode::Cli => Some(self.create_preference_decision(&UiMode::Cli, "User global preference: CLI")),
            UiMode::Tui => Some(self.create_preference_decision(&UiMode::Tui, "User global preference: TUI")),
            UiMode::Minimal => Some(self.create_preference_decision(&UiMode::Minimal, "User global preference: Minimal")),
            UiMode::Auto => None, // Continue with smart detection
        }
    }
    
    /// Create decision based on user preference
    fn create_preference_decision(&self, mode: &UiMode, reason: &str) -> InterfaceDecision {
        let strategy = match mode {
            UiMode::Cli => InterfaceStrategy::CliOnly,
            UiMode::Tui => InterfaceStrategy::AutoLaunchTui {
                reason: "User preference".to_string(),
                show_cli_first: false,
            },
            UiMode::Minimal => InterfaceStrategy::CliOnly,
            UiMode::Auto => unreachable!(),
        };
        
        InterfaceDecision {
            strategy,
            confidence: 1.0,
            reason: reason.to_string(),
            fallback: None,
        }
    }
    
    /// Make intelligent decision based on analysis
    fn make_intelligent_decision(&self, command: &Command, analysis: &CommandAnalysis) -> InterfaceDecision {
        let complexity_weight = 0.4;
        let interaction_weight = 0.3;
        let data_volume_weight = 0.2;
        let user_pattern_weight = 0.1;
        
        let complexity_score = analysis.complexity_score as f32 / 10.0;
        let interaction_score = self.interaction_benefit_score(&analysis.interaction_benefit);
        let data_volume_score = self.data_volume_score(&analysis.data_volume_estimate);
        let user_pattern_score = self.get_user_pattern_score(command);
        
        let weighted_score = 
            complexity_score * complexity_weight +
            interaction_score * interaction_weight +
            data_volume_score * data_volume_weight +
            user_pattern_score * user_pattern_weight;
        
        let confidence = self.calculate_confidence(analysis, weighted_score);
        
        match (weighted_score, confidence) {
            // Low score: CLI only
            (score, _) if score < 0.3 => InterfaceDecision {
                strategy: InterfaceStrategy::CliOnly,
                confidence,
                reason: "Simple operation best suited for CLI".to_string(),
                fallback: None,
            },
            
            // Medium score: CLI with hint
            (score, conf) if score < 0.6 && conf > 0.7 => InterfaceDecision {
                strategy: InterfaceStrategy::CliWithHint {
                    hint: self.generate_helpful_hint(command, analysis),
                    tui_command: self.generate_tui_command(command),
                },
                confidence,
                reason: "TUI available for enhanced experience".to_string(),
                fallback: Some(InterfaceStrategy::CliOnly),
            },
            
            // Medium-high score: Prompt for TUI
            (score, _conf) if score < 0.8 => InterfaceDecision {
                strategy: InterfaceStrategy::PromptForTui {
                    reason: self.generate_prompt_reason(analysis),
                    default_yes: score > 0.65,
                },
                confidence,
                reason: "Interactive interface likely beneficial".to_string(),
                fallback: Some(InterfaceStrategy::CliOnly),
            },
            
            // High score: Auto-launch TUI
            (_, conf) => InterfaceDecision {
                strategy: InterfaceStrategy::AutoLaunchTui {
                    reason: "Complex operation best suited for interactive interface".to_string(),
                    show_cli_first: conf < 0.9,
                },
                confidence,
                reason: "TUI provides optimal experience for this operation".to_string(),
                fallback: Some(InterfaceStrategy::CliOnly),
            },
        }
    }
    
    /// Convert interaction benefit to score
    fn interaction_benefit_score(&self, benefit: &InteractionBenefit) -> f32 {
        match benefit {
            InteractionBenefit::None => 0.0,
            InteractionBenefit::Low => 0.2,
            InteractionBenefit::Medium => 0.5,
            InteractionBenefit::High => 0.8,
            InteractionBenefit::Critical => 1.0,
        }
    }
    
    /// Convert data volume to score
    fn data_volume_score(&self, volume: &DataVolume) -> f32 {
        match volume {
            DataVolume::Minimal => 0.0,
            DataVolume::Small => 0.2,
            DataVolume::Medium => 0.5,
            DataVolume::Large => 0.8,
            DataVolume::Streaming => 1.0,
        }
    }
    
    /// Get user pattern score (placeholder for future ML)
    fn get_user_pattern_score(&self, _command: &Command) -> f32 {
        0.5 // Neutral score - will be enhanced with user behavior tracking
    }
    
    /// Calculate confidence in decision
    fn calculate_confidence(&self, analysis: &CommandAnalysis, weighted_score: f32) -> f32 {
        let mut confidence: f32 = 0.8;
        
        // Increase confidence for clear cases
        if weighted_score < 0.2 || weighted_score > 0.8 {
            confidence += 0.1;
        }
        
        // Decrease confidence for edge cases
        if analysis.complexity_score == 5 { // Middle complexity
            confidence -= 0.1;
        }
        
        // Terminal capabilities affect confidence
        if self.context.terminal_capabilities.tui_capable {
            confidence += 0.05;
        }
        
        confidence.clamp(0.0, 1.0)
    }
    
    /// Generate helpful hint for CLI with hint strategy
    fn generate_helpful_hint(&self, command: &Command, analysis: &CommandAnalysis) -> String {
        match (command, &analysis.interaction_benefit) {
            (Command::Logs(_), InteractionBenefit::High) => {
                "Try 'dora ui logs' for interactive filtering and search".to_string()
            },
            (Command::Inspect(_), InteractionBenefit::Medium) => {
                "Use 'dora ui inspect' for live metrics and interactive exploration".to_string()
            },
            _ => format!(
                "For enhanced experience, try 'dora ui {}'", 
                command.name()
            ),
        }
    }
    
    /// Generate TUI command equivalent
    fn generate_tui_command(&self, command: &Command) -> String {
        match command {
            Command::Ps(_) => "dora ui".to_string(),
            _ => format!("dora ui {}", command.name()),
        }
    }
    
    /// Generate reason for TUI prompt
    fn generate_prompt_reason(&self, analysis: &CommandAnalysis) -> String {
        match analysis.interaction_benefit {
            InteractionBenefit::High => "This operation benefits significantly from interactive features".to_string(),
            InteractionBenefit::Critical => "This operation is much easier with an interactive interface".to_string(),
            _ => match analysis.data_volume_estimate {
                DataVolume::Large => "Large amount of data is easier to navigate interactively".to_string(),
                DataVolume::Streaming => "Real-time data is best viewed in an interactive interface".to_string(),
                _ => "Interactive interface may provide a better experience".to_string(),
            }
        }
    }
}

impl CommandAnalyzer {
    /// Create new command analyzer
    pub fn new() -> Self {
        Self {
            complexity_rules: ComplexityRules::default(),
            interaction_patterns: InteractionPatterns::default(),
        }
    }
    
    /// Analyze command characteristics
    pub fn analyze(&self, command: &Command) -> CommandAnalysis {
        let base_complexity = self.calculate_base_complexity(command);
        let context_modifiers = self.apply_context_modifiers(command);
        let final_complexity = (base_complexity as i8 + context_modifiers).clamp(0, 10) as u8;
        
        CommandAnalysis {
            complexity_score: final_complexity,
            data_volume_estimate: self.estimate_data_volume(command),
            interaction_benefit: self.assess_interaction_benefit(command),
            automation_suitability: self.check_automation_level(command),
            output_characteristics: self.analyze_output_characteristics(command),
        }
    }
    
    /// Calculate base complexity score for command
    fn calculate_base_complexity(&self, command: &Command) -> u8 {
        match command {
            Command::Ps(_) => 1,                    // Simple list
            Command::Start(_) => 2,                 // Moderate complexity
            Command::Stop(_) => 1,                  // Simple operation
            Command::Logs(_) => 3,                  // Can be complex with filtering
            Command::Build(_) => 2,                 // Build complexity varies
            Command::Up(_) => 3,                    // Multiple operations
            Command::Destroy(_) => 2,               // Moderate safety considerations
            Command::New(_) => 2,                   // Template complexity
            Command::Check(_) => 2,                 // Validation complexity
            Command::Graph(_) => 4,                 // Visualization complexity
            Command::Inspect(_) => 6,               // High inspection complexity
            Command::Debug(_) => 8,                 // Always high complexity
            Command::Analyze(_) => 9,               // Always very high complexity
            Command::Monitor(_) => 7,               // Real-time monitoring complex
            Command::Ui(_) => 2,                    // TUI launcher
            Command::Dashboard(_) => 3,             // Dashboard complexity
            Command::System(_) => 4,                // System operations
            Command::Config(_) => 3,                // Configuration complexity
            Command::Daemon(_) => 5,                // Daemon operations complex
            Command::Runtime(_) => 5,               // Runtime operations complex
            Command::Coordinator(_) => 5,           // Coordinator operations complex
            Command::Self_(_) => 3,                 // Self-management
            Command::Preferences(_) => 4,           // Preference management
        }
    }
    
    /// Apply context-based modifiers
    fn apply_context_modifiers(&self, _command: &Command) -> i8 {
        // Future: Apply modifiers based on command parameters
        0
    }
    
    /// Estimate data volume for command
    fn estimate_data_volume(&self, command: &Command) -> DataVolume {
        match command {
            Command::Ps(_) => DataVolume::Small,
            Command::Logs(_) => DataVolume::Medium,
            Command::Inspect(_) => DataVolume::Large,
            Command::Debug(_) => DataVolume::Large,
            Command::Analyze(_) => DataVolume::Large,
            Command::Monitor(_) => DataVolume::Streaming,
            _ => DataVolume::Minimal,
        }
    }
    
    /// Assess interaction benefit
    fn assess_interaction_benefit(&self, command: &Command) -> InteractionBenefit {
        match command {
            Command::Ps(_) => InteractionBenefit::Low,
            Command::Logs(_) => InteractionBenefit::High,
            Command::Inspect(_) => InteractionBenefit::High,
            Command::Debug(_) => InteractionBenefit::Critical,
            Command::Analyze(_) => InteractionBenefit::Critical,
            Command::Monitor(_) => InteractionBenefit::Critical,
            Command::Graph(_) => InteractionBenefit::Medium,
            _ => InteractionBenefit::None,
        }
    }
    
    /// Check automation suitability
    fn check_automation_level(&self, command: &Command) -> AutomationLevel {
        match command {
            Command::Start(_) | Command::Stop(_) | Command::Build(_) | Command::Check(_) => {
                AutomationLevel::OptimizedForAutomation
            },
            Command::Ps(_) | Command::New(_) | Command::Up(_) | Command::Destroy(_) => {
                AutomationLevel::SuitableForAutomation
            },
            Command::Logs(_) | Command::Graph(_) | Command::Inspect(_) => {
                AutomationLevel::InteractivePreferred
            },
            Command::Debug(_) | Command::Analyze(_) | Command::Monitor(_) => {
                AutomationLevel::InteractiveRequired
            },
            _ => AutomationLevel::SuitableForAutomation,
        }
    }
    
    /// Analyze output characteristics
    fn analyze_output_characteristics(&self, command: &Command) -> OutputCharacteristics {
        OutputCharacteristics {
            is_real_time: matches!(command, Command::Monitor(_) | Command::Logs(_)),
            has_colors: true, // Most commands benefit from colors
            is_structured: !matches!(command, Command::Logs(_)),
            requires_scrolling: matches!(command, Command::Inspect(_) | Command::Debug(_) | Command::Analyze(_)),
        }
    }
}

impl Default for UserConfig {
    fn default() -> Self {
        Self {
            global_ui_mode: UiMode::Auto,
            complexity_threshold: 5,
            auto_launch_threshold: 0.8,
            command_preferences: HashMap::new(),
            hint_preferences: HintPreferences::default(),
        }
    }
}

impl UserConfig {
    /// Load user configuration from file or return default
    pub fn load() -> eyre::Result<Self> {
        // For now, return default configuration
        // In full implementation, this would load from config file
        Ok(Self::default())
    }
}

impl Default for HintPreferences {
    fn default() -> Self {
        Self {
            show_hints: true,
            hint_frequency: HintFrequency::OncePerSession,
            dismissed_hints: HashSet::new(),
        }
    }
}

impl Default for ComplexityRules {
    fn default() -> Self {
        Self {
            base_scores: HashMap::new(),
            modifier_rules: Vec::new(),
        }
    }
}

impl Default for InteractionPatterns {
    fn default() -> Self {
        Self {
            command_benefits: HashMap::new(),
            data_volume_thresholds: HashMap::new(),
        }
    }
}

impl Command {
    /// Get command name for caching and analysis
    pub fn name(&self) -> String {
        match self {
            Command::Ps(_) => "ps".to_string(),
            Command::Start(_) => "start".to_string(),
            Command::Stop(_) => "stop".to_string(),
            Command::Logs(_) => "logs".to_string(),
            Command::Build(_) => "build".to_string(),
            Command::Up(_) => "up".to_string(),
            Command::Destroy(_) => "destroy".to_string(),
            Command::New(_) => "new".to_string(),
            Command::Check(_) => "check".to_string(),
            Command::Graph(_) => "graph".to_string(),
            Command::Inspect(_) => "inspect".to_string(),
            Command::Debug(_) => "debug".to_string(),
            Command::Analyze(_) => "analyze".to_string(),
            Command::Monitor(_) => "monitor".to_string(),
            Command::Ui(_) => "ui".to_string(),
            Command::Dashboard(_) => "dashboard".to_string(),
            Command::System(_) => "system".to_string(),
            Command::Config(_) => "config".to_string(),
            Command::Daemon(_) => "daemon".to_string(),
            Command::Runtime(_) => "runtime".to_string(),
            Command::Coordinator(_) => "coordinator".to_string(),
            Command::Self_(_) => "self".to_string(),
            Command::Preferences(_) => "preferences".to_string(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    // use crate::cli::context::TerminalCapabilities;

    fn mock_interactive_context() -> ExecutionContext {
        let mut context = ExecutionContext::detect_basic();
        context.is_tty = true;
        context.is_piped = false;
        context.is_scripted = false;
        context.terminal_capabilities.tui_capable = true;
        context
    }

    fn mock_non_interactive_context() -> ExecutionContext {
        let mut context = ExecutionContext::detect_basic();
        context.is_tty = false;
        context.is_piped = true;
        context.is_scripted = true;
        context.terminal_capabilities.tui_capable = false;
        context
    }

    #[test]
    fn test_simple_command_selection() {
        let context = mock_interactive_context();
        let config = UserConfig::default();
        let mut selector = InterfaceSelector::new(context, config);
        
        // Create a mock Ps command (we'll need to implement this)
        let decision = selector.select_interface(&Command::Ps(super::super::commands::PsCommand::default()));
        
        assert!(matches!(decision.strategy, InterfaceStrategy::CliOnly));
        assert!(decision.confidence > 0.8);
    }

    #[test]
    fn test_non_interactive_context() {
        let context = mock_non_interactive_context();
        let config = UserConfig::default();
        let mut selector = InterfaceSelector::new(context, config);
        
        let decision = selector.select_interface(&Command::Debug(super::super::commands::DebugCommand::default()));
        
        assert!(matches!(decision.strategy, InterfaceStrategy::CliOnly));
        assert_eq!(decision.reason, "Non-interactive environment detected");
    }

    #[test]
    fn test_user_preference_override() {
        let context = mock_interactive_context();
        let mut config = UserConfig::default();
        config.global_ui_mode = UiMode::Cli;
        let mut selector = InterfaceSelector::new(context, config);
        
        let decision = selector.select_interface(&Command::Debug(super::super::commands::DebugCommand::default()));
        
        assert!(matches!(decision.strategy, InterfaceStrategy::CliOnly));
        assert!(decision.reason.contains("User global preference"));
    }

    #[test]
    fn test_decision_caching() {
        let context = mock_interactive_context();
        let config = UserConfig::default();
        let mut selector = InterfaceSelector::new(context, config);
        
        let command = Command::Ps(super::super::commands::PsCommand::default());
        
        // First call
        let decision1 = selector.select_interface(&command);
        
        // Second call (should use cache)
        let decision2 = selector.select_interface(&command);
        
        // Test that decisions are identical (cache working)
        assert_eq!(decision1.strategy, decision2.strategy);
        assert_eq!(decision1.confidence, decision2.confidence);
        assert_eq!(decision1.reason, decision2.reason);
    }

    #[test]
    fn test_complexity_analysis() {
        let analyzer = CommandAnalyzer::new();
        
        // Simple command should have low complexity
        let simple = Command::Ps(super::super::commands::PsCommand::default());
        let analysis = analyzer.analyze(&simple);
        assert!(analysis.complexity_score <= 3);
        
        // Complex command should have high complexity
        let complex = Command::Debug(super::super::commands::DebugCommand::default());
        let analysis = analyzer.analyze(&complex);
        assert!(analysis.complexity_score >= 7);
    }
}