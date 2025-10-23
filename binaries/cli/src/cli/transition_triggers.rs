use std::{
    collections::HashMap,
    path::PathBuf,
    time::Instant,
};

use crate::{
    cli::{
        Command,
        commands::*,
        context::ExecutionContext,
        interface::{InterfaceDecision, InterfaceStrategy},
        transitions::{TransitionTrigger, TuiLaunchContext, TuiLaunchOptions},
    },
    tui::ViewType,
};

/// Analyzes commands and determines appropriate transition triggers
#[derive(Debug)]
pub struct TransitionTriggerAnalyzer {
    command_patterns: HashMap<String, TriggerPattern>,
    context_analyzer: ContextAnalyzer,
    complexity_thresholds: ComplexityThresholds,
}

/// Pattern for triggering transitions based on command characteristics
#[derive(Debug, Clone)]
pub struct TriggerPattern {
    pub min_complexity: u8,
    pub auto_trigger: bool,
    pub suggestion_message: String,
    pub target_view: ViewType,
}

/// Analyzes execution context for transition decisions
#[derive(Debug)]
pub struct ContextAnalyzer {
    interaction_history: Vec<InteractionEvent>,
    user_preferences: UserPreferences,
}

/// Thresholds for complexity-based transitions
#[derive(Debug, Clone)]
pub struct ComplexityThresholds {
    pub auto_launch: u8,      // Auto-launch TUI at this complexity
    pub suggest: u8,          // Suggest TUI at this complexity
    pub prompt: u8,           // Prompt user at this complexity
}

/// User interaction event for pattern analysis
#[derive(Debug, Clone)]
pub struct InteractionEvent {
    pub command: String,
    pub timestamp: Instant,
    pub context: String,
    pub outcome: InteractionOutcome,
}

/// Outcome of user interaction
#[derive(Debug, Clone)]
pub enum InteractionOutcome {
    AcceptedTui,
    DeclinedTui,
    UsedCli,
    SwitchedBack,
}

/// User preferences for transitions
#[derive(Debug, Clone)]
pub struct UserPreferences {
    pub prefer_tui: bool,
    pub auto_launch_enabled: bool,
    pub suggestion_frequency: SuggestionFrequency,
    pub preferred_views: HashMap<String, ViewType>,
}

/// How frequently to show TUI suggestions
#[derive(Debug, Clone)]
pub enum SuggestionFrequency {
    Always,
    Often,
    Sometimes,
    Rarely,
    Never,
}

/// Enhanced launch context with smart defaults
#[derive(Debug, Clone)]
pub struct SmartLaunchContext {
    pub base_context: TuiLaunchContext,
    pub smart_features: SmartFeatures,
    pub performance_hints: PerformanceHints,
}

/// Smart features enabled for this launch
#[derive(Debug, Clone)]
pub struct SmartFeatures {
    pub auto_refresh: bool,
    pub intelligent_filtering: bool,
    pub context_awareness: bool,
    pub predictive_navigation: bool,
}

/// Performance optimization hints
#[derive(Debug, Clone)]
pub struct PerformanceHints {
    pub expected_data_size: DataSize,
    pub update_frequency: UpdateFrequency,
    pub resource_intensity: ResourceIntensity,
}

/// Expected data size for optimization
#[derive(Debug, Clone)]
pub enum DataSize {
    Small,      // < 1MB
    Medium,     // 1-10MB
    Large,      // 10-100MB
    VeryLarge,  // > 100MB
}

/// How frequently data updates
#[derive(Debug, Clone)]
pub enum UpdateFrequency {
    Static,     // No updates
    Slow,       // Every few seconds
    Medium,     // Every second
    Fast,       // Multiple times per second
    RealTime,   // Continuous updates
}

/// Resource intensity of the operation
#[derive(Debug, Clone)]
pub enum ResourceIntensity {
    Low,        // Minimal CPU/memory
    Medium,     // Moderate resource usage
    High,       // Significant resource usage
    Critical,   // Heavy resource usage
}

impl TransitionTriggerAnalyzer {
    pub fn new() -> Self {
        Self {
            command_patterns: Self::initialize_patterns(),
            context_analyzer: ContextAnalyzer::new(),
            complexity_thresholds: ComplexityThresholds::default(),
        }
    }
    
    /// Analyze a command and determine if/how to trigger a transition
    pub fn analyze_command(
        &mut self,
        command: &Command,
        context: &ExecutionContext,
        interface_decision: &InterfaceDecision,
    ) -> Option<TransitionTrigger> {
        // Check if TUI transition is appropriate
        if !self.should_consider_tui(command, context) {
            return None;
        }
        
        let complexity = self.calculate_command_complexity(command, context);
        let command_key = self.get_command_key(command);
        
        // Match against known patterns
        if let Some(pattern) = self.command_patterns.get(&command_key) {
            return self.create_trigger_from_pattern(command, pattern, complexity, context);
        }
        
        // Fallback to interface decision
        self.create_trigger_from_interface_decision(command, interface_decision, complexity)
    }
    
    /// Create a smart launch context with optimizations
    pub fn create_smart_launch_context(
        &self,
        trigger: TransitionTrigger,
        command: &Command,
        context: &ExecutionContext,
    ) -> SmartLaunchContext {
        let base_context = self.create_base_launch_context(&trigger, command);
        let smart_features = self.determine_smart_features(command, context);
        let performance_hints = self.generate_performance_hints(command, context);
        
        SmartLaunchContext {
            base_context,
            smart_features,
            performance_hints,
        }
    }
    
    /// Record user interaction for learning
    pub fn record_interaction(&mut self, event: InteractionEvent) {
        self.context_analyzer.record_interaction(event);
    }
    
    /// Update user preferences based on interaction patterns
    pub fn update_preferences(&mut self) {
        self.context_analyzer.analyze_patterns();
    }
    
    // Private implementation methods
    
    fn initialize_patterns() -> HashMap<String, TriggerPattern> {
        let mut patterns = HashMap::new();
        
        // Inspection commands benefit from interactive visualization
        patterns.insert("inspect".to_string(), TriggerPattern {
            min_complexity: 3,
            auto_trigger: false,
            suggestion_message: "Interactive inspection provides better data exploration".to_string(),
            target_view: ViewType::NodeInspector { node_id: "auto".to_string() },
        });
        
        // Debug commands are complex and benefit from TUI
        patterns.insert("debug".to_string(), TriggerPattern {
            min_complexity: 4,
            auto_trigger: true,
            suggestion_message: "Debug sessions work better with interactive tools".to_string(),
            target_view: ViewType::DebugSession { dataflow_id: "auto".to_string() },
        });
        
        // Log analysis with following enabled
        patterns.insert("logs_follow".to_string(), TriggerPattern {
            min_complexity: 5,
            auto_trigger: false,
            suggestion_message: "Live log analysis benefits from interactive filtering".to_string(),
            target_view: ViewType::LogViewer { target: "auto".to_string() },
        });
        
        // System monitoring
        patterns.insert("monitor".to_string(), TriggerPattern {
            min_complexity: 2,
            auto_trigger: false,
            suggestion_message: "System monitoring provides better insights in interactive mode".to_string(),
            target_view: ViewType::SystemMonitor,
        });
        
        // Analysis commands
        patterns.insert("analyze".to_string(), TriggerPattern {
            min_complexity: 6,
            auto_trigger: true,
            suggestion_message: "Complex analysis benefits from interactive visualization".to_string(),
            target_view: ViewType::Dashboard,
        });
        
        patterns
    }
    
    fn should_consider_tui(&self, command: &Command, context: &ExecutionContext) -> bool {
        // Don't suggest TUI in non-interactive contexts
        if !context.is_tty || context.is_scripted {
            return false;
        }
        
        // Check if command supports TUI transition
        command.can_transition_to_tui()
    }
    
    fn calculate_command_complexity(&self, command: &Command, context: &ExecutionContext) -> u8 {
        let mut complexity = 0;
        
        match command {
            Command::Inspect(cmd) => {
                complexity += 3; // Base complexity for inspection
                if cmd.deep { complexity += 2; }
                if cmd.resource.is_some() { complexity += 1; }
            },
            
            Command::Debug(_) => {
                complexity += 5; // Debug is inherently complex
            },
            
            Command::Logs(cmd) => {
                complexity += 2; // Base complexity for logs
                if cmd.follow { complexity += 3; } // Live logs are more complex
                if cmd.nodes.nodes.as_ref().map_or(false, |n| n.len() > 1) {
                    complexity += 2; // Multiple nodes
                }
            },
            
            Command::Monitor(_) => {
                complexity += 4; // Real-time monitoring is complex
            },
            
            Command::Analyze(cmd) => {
                complexity += 4; // Base analysis complexity
                if let Some(analysis_type) = &cmd.analysis_type {
                    complexity += match analysis_type {
                        AnalysisType::Performance => 3,
                        AnalysisType::Resources => 2,
                        AnalysisType::Trends => 4,
                        AnalysisType::Complexity => 5,
                    };
                }
            },
            
            Command::Start(_) | Command::Build(_) => {
                complexity += 3; // Progress monitoring benefits from TUI
            },
            
            _ => {
                complexity += 1; // Base complexity
            }
        }
        
        // Adjust based on context
        if context.terminal_capabilities.supports_unicode {
            complexity += 1; // Better TUI experience
        }
        
        if context.terminal_size.map_or(false, |(w, h)| w > 100 && h > 30) {
            complexity += 1; // Larger terminal benefits from TUI
        }
        
        complexity.min(10) // Cap at 10
    }
    
    fn get_command_key(&self, command: &Command) -> String {
        match command {
            Command::Inspect(_) => "inspect".to_string(),
            Command::Debug(_) => "debug".to_string(),
            Command::Logs(cmd) if cmd.follow => "logs_follow".to_string(),
            Command::Logs(_) => "logs".to_string(),
            Command::Monitor(_) => "monitor".to_string(),
            Command::Analyze(_) => "analyze".to_string(),
            Command::Start(_) => "start".to_string(),
            Command::Build(_) => "build".to_string(),
            _ => "other".to_string(),
        }
    }
    
    fn create_trigger_from_pattern(
        &self,
        command: &Command,
        pattern: &TriggerPattern,
        complexity: u8,
        context: &ExecutionContext,
    ) -> Option<TransitionTrigger> {
        if complexity < pattern.min_complexity {
            return None;
        }
        
        if pattern.auto_trigger && complexity >= self.complexity_thresholds.auto_launch {
            Some(TransitionTrigger::AutoLaunch {
                original_command: command.clone(),
                complexity_score: complexity,
            })
        } else if complexity >= self.complexity_thresholds.suggest {
            Some(TransitionTrigger::SmartSuggestion {
                original_command: command.clone(),
                reason: pattern.suggestion_message.clone(),
            })
        } else {
            None
        }
    }
    
    fn create_trigger_from_interface_decision(
        &self,
        command: &Command,
        decision: &InterfaceDecision,
        complexity: u8,
    ) -> Option<TransitionTrigger> {
        match &decision.strategy {
            InterfaceStrategy::AutoLaunchTui { reason, .. } => {
                Some(TransitionTrigger::AutoLaunch {
                    original_command: command.clone(),
                    complexity_score: complexity,
                })
            },
            InterfaceStrategy::PromptForTui { reason, default_yes } => {
                Some(TransitionTrigger::UserPrompt {
                    original_command: command.clone(),
                    user_choice: *default_yes,
                })
            },
            InterfaceStrategy::CliWithHint { hint, .. } => {
                Some(TransitionTrigger::SmartSuggestion {
                    original_command: command.clone(),
                    reason: hint.clone(),
                })
            },
            _ => None,
        }
    }
    
    fn create_base_launch_context(
        &self,
        trigger: &TransitionTrigger,
        command: &Command,
    ) -> TuiLaunchContext {
        let initial_view = self.determine_view_for_command(command);
        let focus_target = self.extract_focus_target(command);
        
        TuiLaunchContext {
            initial_view,
            cli_context: crate::cli::transitions::CliState::new(),
            shared_context: crate::cli::transitions::SharedContext::new(),
            transition_trigger: trigger.clone(),
            launch_options: TuiLaunchOptions {
                show_transition_message: !matches!(trigger, TransitionTrigger::ExplicitCommand(_)),
                preserve_cli_output: true,
                auto_refresh: true,
                focus_target,
            },
        }
    }
    
    fn determine_view_for_command(&self, command: &Command) -> ViewType {
        match command {
            Command::Ps(_) => ViewType::Dashboard,
            Command::Inspect(cmd) => {
                if let Some(resource) = &cmd.resource {
                    ViewType::NodeInspector { node_id: resource.clone() }
                } else {
                    ViewType::Dashboard
                }
            },
            Command::Logs(cmd) => {
                let target = cmd.dataflow.dataflow.as_ref()
                    .map(|p| p.to_string_lossy().to_string())
                    .unwrap_or_else(|| "system".to_string());
                ViewType::LogViewer { target }
            },
            Command::Debug(cmd) => {
                let dataflow_id = cmd.dataflow.dataflow.as_ref()
                    .map(|p| p.to_string_lossy().to_string())
                    .unwrap_or_else(|| "current".to_string());
                ViewType::DebugSession { dataflow_id }
            },
            Command::Monitor(_) => ViewType::SystemMonitor,
            Command::Start(_) | Command::Build(_) => ViewType::Dashboard,
            _ => ViewType::Dashboard,
        }
    }
    
    fn extract_focus_target(&self, command: &Command) -> Option<String> {
        match command {
            Command::Inspect(cmd) => cmd.resource.clone(),
            Command::Logs(cmd) => cmd.dataflow.dataflow.as_ref()
                .map(|p| p.to_string_lossy().to_string()),
            Command::Debug(cmd) => cmd.dataflow.dataflow.as_ref()
                .map(|p| p.to_string_lossy().to_string()),
            Command::Start(cmd) => cmd.dataflow.dataflow.as_ref()
                .map(|p| p.to_string_lossy().to_string()),
            _ => None,
        }
    }
    
    fn determine_smart_features(&self, command: &Command, context: &ExecutionContext) -> SmartFeatures {
        SmartFeatures {
            auto_refresh: matches!(command, 
                Command::Monitor(_) | 
                Command::Logs(LogsCommand { follow: true, .. }) |
                Command::Debug(_)
            ),
            intelligent_filtering: matches!(command,
                Command::Logs(_) |
                Command::Inspect(_) |
                Command::Analyze(_)
            ),
            context_awareness: context.is_tty && !context.is_scripted,
            predictive_navigation: self.context_analyzer.user_preferences.prefer_tui,
        }
    }
    
    fn generate_performance_hints(&self, command: &Command, _context: &ExecutionContext) -> PerformanceHints {
        let (data_size, update_frequency, resource_intensity) = match command {
            Command::Monitor(_) => (DataSize::Medium, UpdateFrequency::Fast, ResourceIntensity::Medium),
            Command::Logs(LogsCommand { follow: true, .. }) => (DataSize::Large, UpdateFrequency::RealTime, ResourceIntensity::High),
            Command::Debug(_) => (DataSize::Medium, UpdateFrequency::Medium, ResourceIntensity::High),
            Command::Analyze(_) => (DataSize::Large, UpdateFrequency::Static, ResourceIntensity::Critical),
            Command::Inspect(_) => (DataSize::Small, UpdateFrequency::Slow, ResourceIntensity::Low),
            _ => (DataSize::Small, UpdateFrequency::Static, ResourceIntensity::Low),
        };
        
        PerformanceHints {
            expected_data_size: data_size,
            update_frequency,
            resource_intensity,
        }
    }
}

impl ContextAnalyzer {
    pub fn new() -> Self {
        Self {
            interaction_history: Vec::new(),
            user_preferences: UserPreferences::default(),
        }
    }
    
    pub fn record_interaction(&mut self, event: InteractionEvent) {
        self.interaction_history.push(event);
        
        // Keep history bounded
        const MAX_HISTORY: usize = 1000;
        while self.interaction_history.len() > MAX_HISTORY {
            self.interaction_history.remove(0);
        }
    }
    
    pub fn analyze_patterns(&mut self) {
        // Analyze user behavior patterns
        let recent_interactions = self.interaction_history.iter()
            .rev()
            .take(50)
            .collect::<Vec<_>>();
        
        let tui_acceptance_rate = recent_interactions.iter()
            .filter(|event| matches!(event.outcome, InteractionOutcome::AcceptedTui))
            .count() as f32 / recent_interactions.len() as f32;
        
        // Update preferences based on patterns
        self.user_preferences.prefer_tui = tui_acceptance_rate > 0.7;
        self.user_preferences.suggestion_frequency = match tui_acceptance_rate {
            r if r > 0.8 => SuggestionFrequency::Always,
            r if r > 0.6 => SuggestionFrequency::Often,
            r if r > 0.4 => SuggestionFrequency::Sometimes,
            r if r > 0.2 => SuggestionFrequency::Rarely,
            _ => SuggestionFrequency::Never,
        };
    }
}

impl Default for ComplexityThresholds {
    fn default() -> Self {
        Self {
            auto_launch: 8,  // High complexity triggers auto-launch
            suggest: 5,      // Medium complexity triggers suggestion
            prompt: 3,       // Low complexity triggers prompt
        }
    }
}

impl Default for UserPreferences {
    fn default() -> Self {
        Self {
            prefer_tui: false,
            auto_launch_enabled: true,
            suggestion_frequency: SuggestionFrequency::Sometimes,
            preferred_views: HashMap::new(),
        }
    }
}

impl Default for TransitionTriggerAnalyzer {
    fn default() -> Self {
        Self::new()
    }
}

// Extension trait for Command to support transition analysis
impl Command {
    pub fn can_transition_to_tui(&self) -> bool {
        matches!(self,
            Command::Ps(_) | Command::Inspect(_) | Command::Logs(_) | 
            Command::Debug(_) | Command::Monitor(_) | Command::Analyze(_) |
            Command::Start(_) | Command::Build(_)
        )
    }
    
    pub fn suggest_tui_transition(&self, context: &ExecutionContext) -> Option<String> {
        if !context.is_tty || context.is_scripted {
            return None;
        }
        
        match self {
            Command::Inspect(cmd) if cmd.deep => {
                Some("Deep inspection works better with interactive interface".to_string())
            },
            Command::Debug(_) => {
                Some("Interactive debugging provides visual state exploration".to_string())
            },
            Command::Logs(cmd) if cmd.follow => {
                Some("Live log monitoring benefits from interactive filtering tools".to_string())
            },
            Command::Analyze(_) => {
                Some("Analysis results are easier to explore interactively".to_string())
            },
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_trigger_analyzer_creation() {
        let analyzer = TransitionTriggerAnalyzer::new();
        assert!(!analyzer.command_patterns.is_empty());
    }

    #[test]
    fn test_complexity_calculation() {
        let analyzer = TransitionTriggerAnalyzer::new();
        let context = ExecutionContext::default();
        
        let debug_cmd = Command::Debug(DebugCommand::default());
        let complexity = analyzer.calculate_command_complexity(&debug_cmd, &context);
        
        assert!(complexity >= 5); // Debug commands should be complex
    }

    #[test]
    fn test_command_key_generation() {
        let analyzer = TransitionTriggerAnalyzer::new();
        
        let inspect_cmd = Command::Inspect(InspectCommand::default());
        assert_eq!(analyzer.get_command_key(&inspect_cmd), "inspect");
        
        let logs_cmd = Command::Logs(LogsCommand { follow: true, ..Default::default() });
        assert_eq!(analyzer.get_command_key(&logs_cmd), "logs_follow");
    }

    #[test]
    fn test_view_determination() {
        let analyzer = TransitionTriggerAnalyzer::new();
        
        let inspect_cmd = Command::Inspect(InspectCommand { 
            resource: Some("test_node".to_string()),
            ..Default::default()
        });
        
        let view = analyzer.determine_view_for_command(&inspect_cmd);
        match view {
            ViewType::NodeInspector { node_id } => assert_eq!(node_id, "test_node"),
            _ => panic!("Expected NodeInspector view"),
        }
    }

    #[test]
    fn test_smart_features_determination() {
        let analyzer = TransitionTriggerAnalyzer::new();
        let context = ExecutionContext::default();
        
        let monitor_cmd = Command::Monitor(MonitorCommand::default());
        let features = analyzer.determine_smart_features(&monitor_cmd, &context);
        
        assert!(features.auto_refresh); // Monitor should auto-refresh
    }

    #[test]
    fn test_performance_hints() {
        let analyzer = TransitionTriggerAnalyzer::new();
        let context = ExecutionContext::default();
        
        let logs_cmd = Command::Logs(LogsCommand { follow: true, ..Default::default() });
        let hints = analyzer.generate_performance_hints(&logs_cmd, &context);
        
        assert!(matches!(hints.update_frequency, UpdateFrequency::RealTime));
        assert!(matches!(hints.resource_intensity, ResourceIntensity::High));
    }
}