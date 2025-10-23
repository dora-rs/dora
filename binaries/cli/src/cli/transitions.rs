use std::{
    collections::{HashMap, VecDeque},
    path::PathBuf,
    time::{Duration, Instant},
};

use crate::{
    cli::{
        Command, 
        commands::UiCommand,
        context::ExecutionContext,
        interface::{InterfaceDecision, InterfaceStrategy},
    },
    tui::{ViewType, AppState, DoraApp, CliContext},
};

/// Manages seamless transitions between CLI and TUI modes
#[derive(Debug)]
pub struct TransitionManager {
    cli_state: CliState,
    tui_state: Option<TuiState>,
    shared_context: SharedContext,
    transition_history: VecDeque<Transition>,
}

/// Current CLI execution state
#[derive(Debug, Clone)]
pub struct CliState {
    pub current_command: Option<Command>,
    pub working_directory: PathBuf,
    pub environment: HashMap<String, String>,
    pub last_output: Option<CommandOutput>,
    pub command_history: Vec<String>,
}

/// Current TUI execution state
#[derive(Debug, Clone)]
pub struct TuiState {
    pub current_view: ViewType,
    pub view_stack: Vec<ViewType>,
    pub view_states: HashMap<ViewType, ViewState>,
    pub last_refresh: Instant,
}

/// Shared data context between CLI and TUI modes
#[derive(Debug, Clone)]
pub struct SharedContext {
    pub dataflows: Vec<DataflowInfo>,
    pub system_metrics: SystemMetrics,
    pub user_config: UserConfig,
    pub last_updated: Instant,
    pub active_filters: HashMap<String, FilterState>,
}

/// Record of a transition between modes
#[derive(Debug, Clone)]
pub struct Transition {
    pub transition_type: TransitionType,
    pub timestamp: Instant,
    pub from_context: ContextSnapshot,
    pub to_context: ContextSnapshot,
    pub trigger: TransitionTrigger,
    pub duration: Option<Duration>,
}

/// Type of transition that occurred
#[derive(Debug, Clone)]
pub enum TransitionType {
    CliToTui {
        triggering_command: Command,
        target_view: ViewType,
    },
    TuiToCli {
        source_view: ViewType,
        exit_reason: ExitReason,
    },
    TuiViewSwitch {
        from_view: ViewType,
        to_view: ViewType,
    },
}

/// Snapshot of execution context at transition time
#[derive(Debug, Clone)]
pub enum ContextSnapshot {
    Cli(CliState),
    Tui(TuiState),
}

/// What triggered the transition
#[derive(Debug, Clone)]
pub enum TransitionTrigger {
    ExplicitCommand(Command),
    SmartSuggestion { 
        original_command: Command, 
        reason: String 
    },
    UserPrompt { 
        original_command: Command, 
        user_choice: bool 
    },
    AutoLaunch { 
        original_command: Command, 
        complexity_score: u8 
    },
    UserAction(String),
    ErrorRecovery { 
        failed_command: Command, 
        error: String 
    },
}

/// Context for launching TUI from CLI
#[derive(Debug, Clone)]
pub struct TuiLaunchContext {
    pub initial_view: ViewType,
    pub cli_context: CliState,
    pub shared_context: SharedContext,
    pub transition_trigger: TransitionTrigger,
    pub launch_options: TuiLaunchOptions,
}

/// Context for resuming CLI from TUI
#[derive(Debug, Clone)]
pub struct CliResumeContext {
    pub working_directory: PathBuf,
    pub environment: HashMap<String, String>,
    pub preserve_context: bool,
    pub exit_message: Option<String>,
    pub next_command: Option<String>,
}

/// Options for TUI launch behavior
#[derive(Debug, Clone)]
pub struct TuiLaunchOptions {
    pub show_transition_message: bool,
    pub preserve_cli_output: bool,
    pub auto_refresh: bool,
    pub focus_target: Option<String>,
}

/// Reason for exiting TUI mode
#[derive(Debug, Clone)]
pub enum ExitReason {
    UserExit,
    CommandExit(String),
    ErrorExit(String),
    SwitchToCommand(Command),
}

/// Command output information
#[derive(Debug, Clone)]
pub struct CommandOutput {
    pub success: bool,
    pub stdout: String,
    pub stderr: String,
    pub duration: Duration,
}

/// View-specific state information
#[derive(Debug, Clone)]
pub struct ViewState {
    pub last_update: Instant,
    pub scroll_position: u16,
    pub selection: Option<String>,
    pub filters: HashMap<String, String>,
}

/// Data types (re-use from existing app.rs)
use crate::tui::app::{DataflowInfo, SystemMetrics, UserConfig, StatusMessage};

/// Filter state for data views
#[derive(Debug, Clone)]
pub struct FilterState {
    pub active: bool,
    pub value: String,
    pub last_applied: Instant,
}

impl TransitionManager {
    pub fn new() -> Self {
        Self {
            cli_state: CliState::new(),
            tui_state: None,
            shared_context: SharedContext::new(),
            transition_history: VecDeque::new(),
        }
    }
    
    /// Transition from CLI to TUI mode
    pub async fn transition_to_tui(
        &mut self,
        trigger: TransitionTrigger,
        target_view: ViewType,
    ) -> crate::tui::Result<TuiLaunchContext> {
        let start_time = Instant::now();
        
        // Capture current CLI state
        let cli_snapshot = self.capture_cli_state();
        
        // Update shared context with latest data
        self.refresh_shared_context().await?;
        
        // Create TUI launch context
        let launch_context = TuiLaunchContext {
            initial_view: target_view.clone(),
            cli_context: cli_snapshot.clone(),
            shared_context: self.shared_context.clone(),
            transition_trigger: trigger.clone(),
            launch_options: TuiLaunchOptions::from_trigger(&trigger),
        };
        
        // Record transition
        let mut transition = Transition {
            transition_type: TransitionType::CliToTui {
                triggering_command: trigger.command().cloned().unwrap_or(Command::Ui(UiCommand::default())),
                target_view: target_view.clone(),
            },
            timestamp: start_time,
            from_context: ContextSnapshot::Cli(cli_snapshot),
            to_context: ContextSnapshot::Tui(TuiState {
                current_view: target_view,
                view_stack: Vec::new(),
                view_states: HashMap::new(),
                last_refresh: Instant::now(),
            }),
            trigger,
            duration: None,
        };
        
        transition.duration = Some(start_time.elapsed());
        self.record_transition(transition);
        
        Ok(launch_context)
    }
    
    /// Transition from TUI back to CLI mode
    pub async fn transition_from_tui(
        &mut self,
        tui_state: TuiState,
        exit_reason: ExitReason,
    ) -> crate::tui::Result<CliResumeContext> {
        let start_time = Instant::now();
        
        // Capture TUI state before exit
        let tui_snapshot = tui_state.clone();
        
        // Update shared context with any changes from TUI
        self.sync_context_from_tui(&tui_state).await?;
        
        // Determine appropriate CLI context to resume
        let resume_context = self.determine_cli_resume_context(&tui_state, &exit_reason);
        
        // Record transition
        let mut transition = Transition {
            transition_type: TransitionType::TuiToCli {
                source_view: tui_state.current_view,
                exit_reason: exit_reason.clone(),
            },
            timestamp: start_time,
            from_context: ContextSnapshot::Tui(tui_snapshot),
            to_context: ContextSnapshot::Cli(self.cli_state.clone()),
            trigger: TransitionTrigger::UserAction("exit_tui".to_string()),
            duration: None,
        };
        
        transition.duration = Some(start_time.elapsed());
        self.record_transition(transition);
        
        // Clear TUI state
        self.tui_state = None;
        
        Ok(resume_context)
    }
    
    /// Record a TUI view switch
    pub fn record_view_switch(&mut self, from_view: ViewType, to_view: ViewType) {
        let transition = Transition {
            transition_type: TransitionType::TuiViewSwitch {
                from_view: from_view.clone(),
                to_view: to_view.clone(),
            },
            timestamp: Instant::now(),
            from_context: ContextSnapshot::Tui(TuiState {
                current_view: from_view,
                view_stack: Vec::new(),
                view_states: HashMap::new(),
                last_refresh: Instant::now(),
            }),
            to_context: ContextSnapshot::Tui(TuiState {
                current_view: to_view,
                view_stack: Vec::new(),
                view_states: HashMap::new(),
                last_refresh: Instant::now(),
            }),
            trigger: TransitionTrigger::UserAction("view_switch".to_string()),
            duration: Some(Duration::from_millis(0)), // View switches should be instant
        };
        
        self.record_transition(transition);
    }
    
    /// Get transition history for analysis
    pub fn get_transition_history(&self) -> &VecDeque<Transition> {
        &self.transition_history
    }
    
    /// Get average transition performance
    pub fn get_transition_performance(&self) -> TransitionPerformance {
        let cli_to_tui_times: Vec<Duration> = self.transition_history
            .iter()
            .filter_map(|t| {
                if matches!(t.transition_type, TransitionType::CliToTui { .. }) {
                    t.duration
                } else {
                    None
                }
            })
            .collect();
            
        let tui_to_cli_times: Vec<Duration> = self.transition_history
            .iter()
            .filter_map(|t| {
                if matches!(t.transition_type, TransitionType::TuiToCli { .. }) {
                    t.duration
                } else {
                    None
                }
            })
            .collect();
        
        TransitionPerformance {
            avg_cli_to_tui: average_duration(&cli_to_tui_times),
            avg_tui_to_cli: average_duration(&tui_to_cli_times),
            total_transitions: self.transition_history.len(),
        }
    }
    
    // Private helper methods
    
    fn capture_cli_state(&self) -> CliState {
        self.cli_state.clone()
    }
    
    async fn refresh_shared_context(&mut self) -> crate::tui::Result<()> {
        // Update dataflows, system metrics, etc.
        // This would connect to the actual Dora runtime
        self.shared_context.last_updated = Instant::now();
        Ok(())
    }
    
    async fn sync_context_from_tui(&mut self, _tui_state: &TuiState) -> crate::tui::Result<()> {
        // Sync any TUI-modified state back to CLI context
        // For example, user preferences, filters, etc.
        Ok(())
    }
    
    fn determine_cli_resume_context(&self, _tui_state: &TuiState, exit_reason: &ExitReason) -> CliResumeContext {
        let working_directory = self.cli_state.working_directory.clone();
        let environment = self.cli_state.environment.clone();
        
        let (preserve_context, exit_message, next_command) = match exit_reason {
            ExitReason::UserExit => (true, Some("Exited TUI".to_string()), None),
            ExitReason::CommandExit(cmd) => (true, Some(format!("Executed: {}", cmd)), None),
            ExitReason::ErrorExit(error) => (true, Some(format!("Error: {}", error)), None),
            ExitReason::SwitchToCommand(cmd) => (true, None, Some(format!("{:?}", cmd))),
        };
        
        CliResumeContext {
            working_directory,
            environment,
            preserve_context,
            exit_message,
            next_command,
        }
    }
    
    fn record_transition(&mut self, transition: Transition) {
        const MAX_HISTORY: usize = 100;
        
        self.transition_history.push_back(transition);
        
        // Keep history bounded
        while self.transition_history.len() > MAX_HISTORY {
            self.transition_history.pop_front();
        }
    }
}

/// Performance metrics for transitions
#[derive(Debug, Clone)]
pub struct TransitionPerformance {
    pub avg_cli_to_tui: Option<Duration>,
    pub avg_tui_to_cli: Option<Duration>,
    pub total_transitions: usize,
}

impl Default for TransitionManager {
    fn default() -> Self {
        Self::new()
    }
}

impl CliState {
    pub fn new() -> Self {
        Self {
            current_command: None,
            working_directory: std::env::current_dir().unwrap_or_else(|_| PathBuf::from("/")),
            environment: std::env::vars().collect(),
            last_output: None,
            command_history: Vec::new(),
        }
    }
}

impl SharedContext {
    pub fn new() -> Self {
        Self {
            dataflows: Vec::new(),
            system_metrics: SystemMetrics::default(),
            user_config: UserConfig::default(),
            last_updated: Instant::now(),
            active_filters: HashMap::new(),
        }
    }
}

impl TuiLaunchOptions {
    pub fn from_trigger(trigger: &TransitionTrigger) -> Self {
        match trigger {
            TransitionTrigger::ExplicitCommand(_) => Self {
                show_transition_message: false,
                preserve_cli_output: false,
                auto_refresh: true,
                focus_target: None,
            },
            TransitionTrigger::SmartSuggestion { .. } => Self {
                show_transition_message: true,
                preserve_cli_output: true,
                auto_refresh: true,
                focus_target: None,
            },
            TransitionTrigger::AutoLaunch { .. } => Self {
                show_transition_message: true,
                preserve_cli_output: false,
                auto_refresh: true,
                focus_target: None,
            },
            _ => Self::default(),
        }
    }
}

impl Default for TuiLaunchOptions {
    fn default() -> Self {
        Self {
            show_transition_message: false,
            preserve_cli_output: false,
            auto_refresh: true,
            focus_target: None,
        }
    }
}

impl TransitionTrigger {
    pub fn command(&self) -> Option<&Command> {
        match self {
            TransitionTrigger::ExplicitCommand(cmd) => Some(cmd),
            TransitionTrigger::SmartSuggestion { original_command, .. } => Some(original_command),
            TransitionTrigger::UserPrompt { original_command, .. } => Some(original_command),
            TransitionTrigger::AutoLaunch { original_command, .. } => Some(original_command),
            TransitionTrigger::ErrorRecovery { failed_command, .. } => Some(failed_command),
            TransitionTrigger::UserAction(_) => None,
        }
    }
}

// Helper function for calculating average duration
fn average_duration(durations: &[Duration]) -> Option<Duration> {
    if durations.is_empty() {
        return None;
    }
    
    let total_nanos: u128 = durations.iter().map(|d| d.as_nanos()).sum();
    let avg_nanos = total_nanos / durations.len() as u128;
    
    Some(Duration::from_nanos(avg_nanos as u64))
}

#[cfg(test)]
mod tests;

#[cfg(test)]
mod basic_tests {
    use super::*;

    #[tokio::test]
    async fn test_transition_manager_creation() {
        let manager = TransitionManager::new();
        assert!(manager.transition_history.is_empty());
        assert!(manager.tui_state.is_none());
    }

    #[tokio::test]
    async fn test_cli_to_tui_transition() {
        let mut manager = TransitionManager::new();
        let trigger = TransitionTrigger::ExplicitCommand(Command::Ui(UiCommand::default()));
        
        let context = manager.transition_to_tui(trigger, ViewType::Dashboard).await.unwrap();
        
        assert_eq!(context.initial_view, ViewType::Dashboard);
        assert_eq!(manager.transition_history.len(), 1);
    }

    #[test]
    fn test_transition_trigger_command_extraction() {
        let cmd = Command::Ui(UiCommand::default());
        let trigger = TransitionTrigger::ExplicitCommand(cmd.clone());
        
        assert!(trigger.command().is_some());
    }

    #[test]
    fn test_performance_metrics() {
        let mut manager = TransitionManager::new();
        
        // Add some mock transitions
        manager.record_transition(Transition {
            transition_type: TransitionType::CliToTui {
                triggering_command: Command::Ui(UiCommand::default()),
                target_view: ViewType::Dashboard,
            },
            timestamp: Instant::now(),
            from_context: ContextSnapshot::Cli(CliState::new()),
            to_context: ContextSnapshot::Tui(TuiState {
                current_view: ViewType::Dashboard,
                view_stack: Vec::new(),
                view_states: HashMap::new(),
                last_refresh: Instant::now(),
            }),
            trigger: TransitionTrigger::ExplicitCommand(Command::Ui(UiCommand::default())),
            duration: Some(Duration::from_millis(100)),
        });
        
        let perf = manager.get_transition_performance();
        assert_eq!(perf.total_transitions, 1);
        assert!(perf.avg_cli_to_tui.is_some());
    }
}