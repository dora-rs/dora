# Issue #010: Implement CLI ↔ TUI Transitions

## 📋 Summary
Implement seamless bidirectional transitions between CLI and TUI modes, enabling users to move fluidly between command-line efficiency and interactive visualization as their needs change. This system provides the core user experience that makes the hybrid architecture feel cohesive and natural.

## 🎯 Objectives
- Enable seamless CLI-to-TUI transitions from any command
- Implement TUI-to-CLI transitions with context preservation
- Add CLI command execution within TUI environment
- Provide consistent state synchronization between modes
- Ensure transitions feel instant and natural to users

**Success Metrics:**
- CLI-to-TUI transitions complete in <300ms
- TUI-to-CLI transitions preserve all relevant context
- CLI commands executed in TUI update views appropriately
- State synchronization maintains consistency 100% of the time
- User workflows feel uninterrupted across mode switches

## 🛠️ Technical Requirements

### What to Build

#### 1. Transition Management System
```rust
// src/cli/transitions.rs
#[derive(Debug)]
pub struct TransitionManager {
    cli_state: CliState,
    tui_state: Option<TuiState>,
    shared_context: SharedContext,
    transition_history: VecDeque<Transition>,
}

#[derive(Debug, Clone)]
pub struct CliState {
    pub current_command: Option<Command>,
    pub working_directory: PathBuf,
    pub environment: HashMap<String, String>,
    pub last_output: Option<CommandOutput>,
    pub command_history: Vec<String>,
}

#[derive(Debug, Clone)]
pub struct TuiState {
    pub current_view: ViewType,
    pub view_stack: Vec<ViewType>,
    pub view_states: HashMap<ViewType, ViewState>,
    pub last_refresh: Instant,
}

#[derive(Debug, Clone)]
pub struct SharedContext {
    pub dataflows: Vec<DataflowInfo>,
    pub system_metrics: SystemMetrics,
    pub user_config: UserConfig,
    pub last_updated: Instant,
    pub active_filters: HashMap<String, FilterState>,
}

#[derive(Debug, Clone)]
pub struct Transition {
    pub transition_type: TransitionType,
    pub timestamp: Instant,
    pub from_context: ContextSnapshot,
    pub to_context: ContextSnapshot,
    pub trigger: TransitionTrigger,
}

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

impl TransitionManager {
    pub fn new() -> Self {
        Self {
            cli_state: CliState::new(),
            tui_state: None,
            shared_context: SharedContext::new(),
            transition_history: VecDeque::new(),
        }
    }
    
    pub async fn transition_to_tui(
        &mut self,
        trigger: TransitionTrigger,
        target_view: ViewType,
    ) -> Result<TuiLaunchContext> {
        // Capture current CLI state
        let cli_snapshot = self.capture_cli_state();
        
        // Update shared context with latest data
        self.refresh_shared_context().await?;
        
        // Create TUI launch context
        let launch_context = TuiLaunchContext {
            initial_view: target_view.clone(),
            cli_context: cli_snapshot,
            shared_context: self.shared_context.clone(),
            transition_trigger: trigger.clone(),
        };
        
        // Record transition
        self.record_transition(Transition {
            transition_type: TransitionType::CliToTui {
                triggering_command: trigger.command().cloned().unwrap_or(Command::Ui(UiCommand::default())),
                target_view: target_view.clone(),
            },
            timestamp: Instant::now(),
            from_context: ContextSnapshot::Cli(cli_snapshot),
            to_context: ContextSnapshot::Tui(TuiState {
                current_view: target_view,
                view_stack: Vec::new(),
                view_states: HashMap::new(),
                last_refresh: Instant::now(),
            }),
            trigger,
        });
        
        Ok(launch_context)
    }
    
    pub async fn transition_from_tui(
        &mut self,
        tui_state: TuiState,
        exit_reason: ExitReason,
    ) -> Result<CliResumeContext> {
        // Capture TUI state before exit
        let tui_snapshot = tui_state.clone();
        
        // Update shared context with any changes from TUI
        self.sync_context_from_tui(&tui_state).await?;
        
        // Determine appropriate CLI context to resume
        let resume_context = self.determine_cli_resume_context(&tui_state, &exit_reason);
        
        // Record transition
        self.record_transition(Transition {
            transition_type: TransitionType::TuiToCli {
                source_view: tui_state.current_view,
                exit_reason: exit_reason.clone(),
            },
            timestamp: Instant::now(),
            from_context: ContextSnapshot::Tui(tui_snapshot),
            to_context: ContextSnapshot::Cli(self.cli_state.clone()),
            trigger: TransitionTrigger::UserAction("exit_tui".to_string()),
        });
        
        // Clear TUI state
        self.tui_state = None;
        
        Ok(resume_context)
    }
}
```

#### 2. CLI Command Integration within TUI
```rust
// src/tui/cli_integration.rs
#[derive(Debug)]
pub struct TuiCliExecutor {
    command_history: Vec<String>,
    execution_context: ExecutionContext,
    state_synchronizer: StateSynchronizer,
}

impl TuiCliExecutor {
    pub async fn execute_command(
        &mut self,
        command_str: &str,
        app_state: &mut AppState,
    ) -> Result<CommandResult> {
        // Parse command
        let args = shell_words::split(command_str)?;
        let cli = Cli::try_parse_from(std::iter::once("dora").chain(args.iter()))?;
        
        // Add to history
        self.command_history.push(command_str.to_string());
        
        // Execute command with TUI context
        let result = match &cli.command {
            // State-changing commands that need immediate UI updates
            Command::Start(cmd) => {
                let result = self.execute_start_in_tui(cmd, app_state).await?;
                self.refresh_dataflow_state(app_state).await?;
                result
            },
            
            Command::Stop(cmd) => {
                let result = self.execute_stop_in_tui(cmd, app_state).await?;
                self.refresh_dataflow_state(app_state).await?;
                result
            },
            
            // View navigation commands
            Command::Ps(_) => {
                self.refresh_dataflow_state(app_state).await?;
                CommandResult::ViewSwitch(ViewType::Dashboard)
            },
            
            Command::Inspect(cmd) => {
                self.refresh_node_state(&cmd.target, app_state).await?;
                CommandResult::ViewSwitch(ViewType::NodeInspector { 
                    node_id: cmd.target.clone() 
                })
            },
            
            Command::Logs(cmd) => {
                CommandResult::ViewSwitch(ViewType::LogViewer { 
                    target: cmd.target.clone() 
                })
            },
            
            Command::Ui(ui_cmd) => {
                if let Some(view) = self.parse_ui_command(ui_cmd) {
                    CommandResult::ViewSwitch(view)
                } else {
                    CommandResult::Success("Already in TUI mode".to_string())
                }
            },
            
            // Configuration commands
            Command::Config(config_cmd) => {
                let result = self.execute_config_in_tui(config_cmd, app_state).await?;
                if config_cmd.affects_ui() {
                    self.refresh_ui_config(app_state).await?;
                }
                result
            },
            
            // Exit command
            cmd if command_str.trim() == "exit" || command_str.trim() == "quit" => {
                CommandResult::Exit
            },
            
            // Other commands - execute in background
            _ => {
                let result = self.execute_command_background(&cli).await?;
                CommandResult::Success(format!("Command executed: {}", result.summary))
            }
        };
        
        Ok(result)
    }
    
    async fn execute_start_in_tui(
        &self,
        cmd: &StartCommand,
        app_state: &mut AppState,
    ) -> Result<CommandResult> {
        // Show progress in TUI status area
        app_state.status_messages.push_back(StatusMessage {
            message: format!("Starting dataflow '{}'...", cmd.dataflow_path.display()),
            level: StatusLevel::Info,
            timestamp: Instant::now(),
        });
        
        // Execute start command
        let start_result = cmd.execute_silent().await?;
        
        // Update status based on result
        let status_message = if start_result.success {
            StatusMessage {
                message: format!("✅ Dataflow '{}' started successfully", 
                                start_result.dataflow_name),
                level: StatusLevel::Success,
                timestamp: Instant::now(),
            }
        } else {
            StatusMessage {
                message: format!("❌ Failed to start dataflow: {}", 
                                start_result.error.unwrap_or_default()),
                level: StatusLevel::Error,
                timestamp: Instant::now(),
            }
        };
        
        app_state.status_messages.push_back(status_message);
        
        Ok(CommandResult::Success(format!(
            "Dataflow start {}", 
            if start_result.success { "completed" } else { "failed" }
        )))
    }
}

#[derive(Debug)]
pub enum CommandResult {
    Success(String),
    ViewSwitch(ViewType),
    Exit,
    Error(String),
}
```

#### 3. Context Synchronization
```rust
// src/cli/state_sync.rs
#[derive(Debug)]
pub struct StateSynchronizer {
    last_sync: Instant,
    sync_interval: Duration,
    pending_updates: Vec<StateUpdate>,
}

#[derive(Debug, Clone)]
pub enum StateUpdate {
    DataflowAdded(DataflowInfo),
    DataflowRemoved(String),
    DataflowStatusChanged { name: String, new_status: DataflowStatus },
    NodeStatusChanged { dataflow: String, node: String, status: NodeStatus },
    SystemMetricsUpdated(SystemMetrics),
    ConfigurationChanged(UserConfig),
}

impl StateSynchronizer {
    pub async fn sync_cli_to_tui(&mut self, app_state: &mut AppState) -> Result<()> {
        // Refresh dataflow information
        let current_dataflows = self.fetch_current_dataflows().await?;
        let previous_dataflows = &app_state.dataflows;
        
        // Detect changes
        let updates = self.detect_dataflow_changes(previous_dataflows, &current_dataflows);
        
        // Apply updates to TUI state
        for update in updates {
            self.apply_state_update(update, app_state).await?;
        }
        
        // Update last sync time
        self.last_sync = Instant::now();
        
        Ok(())
    }
    
    pub async fn sync_tui_to_cli(&mut self, tui_state: &TuiState) -> Result<()> {
        // Extract any CLI-relevant state from TUI
        // This includes user preferences, filters, etc.
        
        // Update CLI state with TUI-driven changes
        for update in &self.pending_updates {
            match update {
                StateUpdate::ConfigurationChanged(config) => {
                    self.update_cli_config(config).await?;
                },
                StateUpdate::DataflowStatusChanged { name, new_status } => {
                    self.update_cli_dataflow_cache(name, new_status).await?;
                },
                _ => {} // Other updates don't affect CLI state
            }
        }
        
        self.pending_updates.clear();
        Ok(())
    }
    
    fn detect_dataflow_changes(
        &self,
        previous: &[DataflowInfo],
        current: &[DataflowInfo],
    ) -> Vec<StateUpdate> {
        let mut updates = Vec::new();
        
        // Find additions
        for dataflow in current {
            if !previous.iter().any(|d| d.name == dataflow.name) {
                updates.push(StateUpdate::DataflowAdded(dataflow.clone()));
            }
        }
        
        // Find removals
        for dataflow in previous {
            if !current.iter().any(|d| d.name == dataflow.name) {
                updates.push(StateUpdate::DataflowRemoved(dataflow.name.clone()));
            }
        }
        
        // Find status changes
        for current_df in current {
            if let Some(previous_df) = previous.iter().find(|d| d.name == current_df.name) {
                if current_df.status != previous_df.status {
                    updates.push(StateUpdate::DataflowStatusChanged {
                        name: current_df.name.clone(),
                        new_status: current_df.status.clone(),
                    });
                }
            }
        }
        
        updates
    }
}
```

#### 4. Transition Triggers and Context Preservation
```rust
// src/cli/transition_triggers.rs
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

#[derive(Debug, Clone)]
pub struct TuiLaunchContext {
    pub initial_view: ViewType,
    pub cli_context: CliState,
    pub shared_context: SharedContext,
    pub transition_trigger: TransitionTrigger,
    pub launch_options: TuiLaunchOptions,
}

#[derive(Debug, Clone)]
pub struct TuiLaunchOptions {
    pub show_transition_message: bool,
    pub preserve_cli_output: bool,
    pub auto_refresh: bool,
    pub focus_target: Option<String>,
}

impl TuiLaunchContext {
    pub fn from_command(command: &Command, trigger: TransitionTrigger) -> Self {
        let initial_view = Self::determine_view_for_command(command);
        let focus_target = Self::extract_focus_target(command);
        
        Self {
            initial_view,
            cli_context: CliState::current(),
            shared_context: SharedContext::current(),
            transition_trigger: trigger,
            launch_options: TuiLaunchOptions {
                show_transition_message: true,
                preserve_cli_output: false,
                auto_refresh: true,
                focus_target,
            },
        }
    }
    
    fn determine_view_for_command(command: &Command) -> ViewType {
        match command {
            Command::Ps(_) => ViewType::Dashboard,
            Command::Inspect(cmd) => {
                match cmd.resource_type {
                    Some(ResourceType::Node) => ViewType::NodeInspector { 
                        node_id: cmd.target.clone() 
                    },
                    Some(ResourceType::Dataflow) => ViewType::DataflowManager,
                    _ => ViewType::Dashboard,
                }
            },
            Command::Logs(cmd) => ViewType::LogViewer { 
                target: cmd.target.clone() 
            },
            Command::Debug(cmd) => ViewType::DebugSession { 
                dataflow_id: cmd.dataflow.clone() 
            },
            Command::Monitor(_) => ViewType::SystemMonitor,
            Command::Start(_) | Command::Build(_) => ViewType::Dashboard, // Show progress
            _ => ViewType::Dashboard,
        }
    }
    
    fn extract_focus_target(command: &Command) -> Option<String> {
        match command {
            Command::Inspect(cmd) => Some(cmd.target.clone()),
            Command::Logs(cmd) => Some(cmd.target.clone()),
            Command::Debug(cmd) => Some(cmd.dataflow.clone()),
            Command::Start(cmd) => Some(cmd.dataflow_path.to_string_lossy().to_string()),
            _ => None,
        }
    }
}

// Integration with existing CLI commands
impl Command {
    pub fn can_transition_to_tui(&self) -> bool {
        match self {
            Command::Ps(_) | Command::Inspect(_) | Command::Logs(_) | 
            Command::Debug(_) | Command::Monitor(_) | Command::Analyze(_) => true,
            Command::Start(_) | Command::Build(_) => true, // For progress monitoring
            _ => false,
        }
    }
    
    pub fn suggest_tui_transition(&self, context: &ExecutionContext) -> Option<String> {
        if !context.is_tty || context.is_scripted {
            return None;
        }
        
        match self {
            Command::Inspect(cmd) if cmd.live_mode => {
                Some("Live inspection works better with interactive interface".to_string())
            },
            Command::Debug(_) => {
                Some("Interactive debugging provides visual state exploration".to_string())
            },
            Command::Logs(cmd) if cmd.follow && cmd.analyze_errors => {
                Some("Error analysis benefits from interactive correlation tools".to_string())
            },
            _ => None,
        }
    }
}
```

### Why This Approach

**Seamless User Experience:**
- Natural transitions that preserve user context and intent
- Bidirectional flow allows users to use the best tool for each task
- State synchronization ensures consistency across modes
- Transition history enables better user experience optimization

**Technical Robustness:**
- Clean separation of concerns between CLI and TUI states
- Reliable state synchronization mechanisms
- Comprehensive context preservation
- Error-resilient transition handling

**Performance Optimized:**
- Fast transitions through efficient state management
- Minimal data transfer between modes
- Lazy loading of TUI resources
- Efficient state change detection

### How to Implement

#### Step 1: Transition Management Core (4 hours)
1. **Implement TransitionManager** with state tracking
2. **Create state synchronization** mechanisms
3. **Add transition recording** and history
4. **Build context preservation** system

#### Step 2: CLI-TUI Integration (5 hours)
1. **Implement TuiCliExecutor** for command execution in TUI
2. **Add command result** handling and view updates
3. **Create state update** propagation system
4. **Add background command** execution support

#### Step 3: Context Synchronization (3 hours)
1. **Implement StateSynchronizer** with change detection
2. **Add bidirectional sync** mechanisms
3. **Create efficient state** diffing algorithms
4. **Add conflict resolution** for concurrent updates

#### Step 4: Transition Triggers (2 hours)
1. **Implement trigger system** for different transition types
2. **Add launch context** creation and management
3. **Create focus target** extraction and handling
4. **Add transition options** configuration

#### Step 5: Testing and Polish (2 hours)
1. **Add comprehensive unit tests** for all functionality
2. **Test various transition scenarios**
3. **Validate state consistency** across transitions
4. **Test performance** of transitions and sync

## 🔗 Dependencies
**Depends On:**
- Issue #001 (Hybrid Command Framework) - CLI command structures
- Issue #003 (Interface Selection Engine) - Transition triggers
- Issue #004 (Configuration System) - User preferences
- Issue #009 (TUI Launcher Framework) - TUI application infrastructure

**Blocks:** All subsequent TUI view implementations and CLI-TUI integration features

## 🧪 Testing Requirements

### Unit Tests
```rust
#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_cli_to_tui_transition() {
        let mut manager = TransitionManager::new();
        let trigger = TransitionTrigger::ExplicitCommand(Command::Ps(PsCommand::default()));
        
        let context = manager.transition_to_tui(trigger, ViewType::Dashboard).await.unwrap();
        
        assert_eq!(context.initial_view, ViewType::Dashboard);
        assert!(!manager.transition_history.is_empty());
    }
    
    #[test]
    fn test_state_synchronization() {
        let mut synchronizer = StateSynchronizer::new();
        let mut app_state = AppState::new();
        
        // Simulate dataflow changes
        synchronizer.sync_cli_to_tui(&mut app_state).await.unwrap();
        
        // Verify state was updated
        assert_eq!(app_state.last_updated.elapsed().as_secs(), 0);
    }
    
    #[test]
    fn test_command_execution_in_tui() {
        let mut executor = TuiCliExecutor::new();
        let mut app_state = AppState::new();
        
        let result = executor.execute_command("ps", &mut app_state).await.unwrap();
        
        match result {
            CommandResult::ViewSwitch(ViewType::Dashboard) => {},
            _ => panic!("Expected view switch to dashboard"),
        }
    }
}
```

### Integration Tests
```rust
// tests/transition_integration.rs
#[tokio::test]
async fn test_full_transition_cycle() {
    // Start with CLI
    let cli_context = ExecutionContext::mock_interactive();
    
    // Transition to TUI
    let mut manager = TransitionManager::new();
    let launch_context = manager.transition_to_tui(
        TransitionTrigger::ExplicitCommand(Command::Ps(PsCommand::default())),
        ViewType::Dashboard
    ).await.unwrap();
    
    // Simulate TUI session
    let tui_state = TuiState {
        current_view: ViewType::Dashboard,
        view_stack: vec![ViewType::NodeInspector { node_id: "test".to_string() }],
        view_states: HashMap::new(),
        last_refresh: Instant::now(),
    };
    
    // Transition back to CLI
    let resume_context = manager.transition_from_tui(
        tui_state, 
        ExitReason::UserExit
    ).await.unwrap();
    
    // Verify context preservation
    assert!(resume_context.preserve_context);
}
```

## ✅ Definition of Done
- [ ] TransitionManager implemented with full state tracking
- [ ] CLI commands execute correctly within TUI environment
- [ ] State synchronization maintains consistency between modes
- [ ] Transitions complete within performance targets (<300ms)
- [ ] Context preservation works across all transition types
- [ ] Command history and user preferences persist across modes
- [ ] Error handling provides graceful fallback for failed transitions
- [ ] Background command execution doesn't block TUI interactions
- [ ] Comprehensive unit tests cover all transition scenarios
- [ ] Integration tests validate end-to-end transition workflows
- [ ] Manual testing confirms smooth user experience across modes

This transition system provides the seamless user experience that makes the hybrid CLI architecture feel like a unified tool rather than separate CLI and TUI applications.