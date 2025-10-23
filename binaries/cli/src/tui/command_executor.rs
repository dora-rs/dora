use std::{
    collections::HashMap,
    time::{Duration, Instant},
};

use crate::{
    cli::{
        Command, Cli,
        commands::*,
        context::ExecutionContext,
    },
    tui::{
        ViewType, AppState, CliContext,
        app::{DataflowInfo, NodeInfo, StatusMessage},
    },
};

/// Executes CLI commands within TUI environment with state synchronization
#[derive(Debug)]
pub struct TuiCliExecutor {
    command_history: Vec<String>,
    execution_context: ExecutionContext,
    state_synchronizer: StateSynchronizer,
}

/// Result of executing a CLI command within TUI
#[derive(Debug)]
pub enum CommandResult {
    Success(String),
    ViewSwitch(ViewType),
    Exit,
    Error(String),
    StateUpdate(StateUpdate),
}

/// Types of state updates that can occur
#[derive(Debug, Clone)]
pub enum StateUpdate {
    DataflowAdded(DataflowInfo),
    DataflowRemoved(String),
    DataflowStatusChanged { name: String, new_status: String },
    NodeStatusChanged { dataflow: String, node: String, status: String },
    SystemMetricsUpdated,
    ConfigurationChanged,
    RefreshRequired,
}

/// Manages state synchronization between CLI and TUI
#[derive(Debug)]
pub struct StateSynchronizer {
    last_sync: Instant,
    sync_interval: Duration,
    pending_updates: Vec<StateUpdate>,
}

/// Status levels for messages
#[derive(Debug, Clone)]
pub enum StatusLevel {
    Info,
    Success,
    Warning,
    Error,
}

impl TuiCliExecutor {
    pub fn new(context: ExecutionContext) -> Self {
        Self {
            command_history: Vec::new(),
            execution_context: context,
            state_synchronizer: StateSynchronizer::new(),
        }
    }
    
    /// Execute a CLI command string within TUI context
    pub async fn execute_command(
        &mut self,
        command_str: &str,
        app_state: &mut AppState,
    ) -> crate::tui::Result<CommandResult> {
        // Parse command
        let args = shell_words::split(command_str)
            .map_err(|e| format!("Failed to parse command: {}", e))?;
        
        if args.is_empty() {
            return Ok(CommandResult::Error("Empty command".to_string()));
        }
        
        // Add "dora" prefix if not present
        let full_args = if args[0] == "dora" {
            args
        } else {
            std::iter::once("dora".to_string()).chain(args).collect()
        };
        
        let cli = Cli::try_parse_from(full_args)
            .map_err(|e| format!("Failed to parse CLI args: {}", e))?;
        
        // Add to history
        self.command_history.push(command_str.to_string());
        
        // Execute command with TUI context
        let result = self.execute_parsed_command(&cli.command, app_state).await?;
        
        // Synchronize state if needed
        if matches!(result, CommandResult::StateUpdate(_)) {
            self.state_synchronizer.sync_cli_to_tui(app_state).await?;
        }
        
        Ok(result)
    }
    
    async fn execute_parsed_command(
        &mut self,
        command: &Command,
        app_state: &mut AppState,
    ) -> crate::tui::Result<CommandResult> {
        match command {
            // State-changing commands that need immediate UI updates
            Command::Start(cmd) => {
                self.execute_start_in_tui(cmd, app_state).await
            },
            
            Command::Stop(cmd) => {
                self.execute_stop_in_tui(cmd, app_state).await
            },
            
            Command::Build(cmd) => {
                self.execute_build_in_tui(cmd, app_state).await
            },
            
            // View navigation commands
            Command::Ps(_) => {
                self.state_synchronizer.refresh_dataflow_state(app_state).await?;
                Ok(CommandResult::ViewSwitch(ViewType::Dashboard))
            },
            
            Command::Inspect(cmd) => {
                self.execute_inspect_in_tui(cmd, app_state).await
            },
            
            Command::Logs(cmd) => {
                Ok(CommandResult::ViewSwitch(ViewType::LogViewer { 
                    target: cmd.dataflow.dataflow.as_ref()
                        .map(|df| df.to_string_lossy().to_string())
                        .unwrap_or_else(|| "system".to_string())
                }))
            },
            
            Command::Debug(cmd) => {
                Ok(CommandResult::ViewSwitch(ViewType::DebugSession { 
                    dataflow_id: cmd.dataflow.dataflow.as_ref()
                        .map(|df| df.to_string_lossy().to_string())
                        .unwrap_or_else(|| "current".to_string())
                }))
            },
            
            Command::Monitor(_) => {
                Ok(CommandResult::ViewSwitch(ViewType::SystemMonitor))
            },
            
            Command::Ui(ui_cmd) => {
                if let Some(view) = self.parse_ui_command(ui_cmd) {
                    Ok(CommandResult::ViewSwitch(view))
                } else {
                    Ok(CommandResult::Success("Already in TUI mode".to_string()))
                }
            },
            
            // Configuration commands
            Command::Config(config_cmd) => {
                self.execute_config_in_tui(config_cmd, app_state).await
            },
            
            // System commands
            Command::System(sys_cmd) => {
                self.execute_system_in_tui(sys_cmd, app_state).await
            },
            
            // Exit commands
            _ if self.is_exit_command(command) => {
                Ok(CommandResult::Exit)
            },
            
            // Other commands - execute in background
            _ => {
                self.execute_command_background(command).await
            }
        }
    }
    
    async fn execute_start_in_tui(
        &mut self,
        cmd: &StartCommand,
        app_state: &mut AppState,
    ) -> crate::tui::Result<CommandResult> {
        // Show progress in TUI status area
        app_state.status_messages.push_back(StatusMessage {
            message: format!("Starting dataflow '{}'...", 
                cmd.dataflow.dataflow.as_ref()
                    .map(|p| p.to_string_lossy().to_string())
                    .unwrap_or_else(|| "unknown".to_string())),
            level: crate::tui::app::MessageLevel::Info,
            timestamp: std::time::Instant::now(),
        });
        
        // Execute start command (mock implementation)
        tokio::time::sleep(Duration::from_millis(500)).await; // Simulate work
        
        let success = true; // This would be the actual result
        let dataflow_name = cmd.dataflow.dataflow.as_ref()
            .map(|p| p.file_name().unwrap_or_default().to_string_lossy().to_string())
            .unwrap_or_else(|| "dataflow".to_string());
        
        // Update status based on result
        let status_message = if success {
            StatusMessage {
                message: format!("✅ Dataflow '{}' started successfully", dataflow_name),
                level: crate::tui::app::MessageLevel::Success,
                timestamp: std::time::Instant::now(),
            }
        } else {
            StatusMessage {
                message: format!("❌ Failed to start dataflow '{}'", dataflow_name),
                level: crate::tui::app::MessageLevel::Error,
                timestamp: std::time::Instant::now(),
            }
        };
        
        app_state.status_messages.push_back(status_message);
        
        // Add new dataflow to state
        if success {
            let new_dataflow = DataflowInfo {
                id: format!("df_{}", chrono::Utc::now().timestamp()),
                name: dataflow_name.clone(),
                status: "running".to_string(),
                nodes: vec![
                    NodeInfo {
                        id: "node_1".to_string(),
                        name: "Example Node".to_string(),
                        status: "running".to_string(),
                    }
                ],
            };
            
            app_state.dataflows.push(new_dataflow.clone());
            
            Ok(CommandResult::StateUpdate(StateUpdate::DataflowAdded(new_dataflow)))
        } else {
            Ok(CommandResult::Error(format!("Failed to start dataflow '{}'", dataflow_name)))
        }
    }
    
    async fn execute_stop_in_tui(
        &mut self,
        cmd: &StopCommand,
        app_state: &mut AppState,
    ) -> crate::tui::Result<CommandResult> {
        let dataflow_name = cmd.dataflow.dataflow.as_ref()
            .map(|p| p.to_string_lossy().to_string())
            .unwrap_or_else(|| "unknown".to_string());
        
        app_state.status_messages.push_back(StatusMessage {
            message: format!("Stopping dataflow '{}'...", dataflow_name),
            level: crate::tui::app::MessageLevel::Info,
            timestamp: std::time::Instant::now(),
        });
        
        // Execute stop command (mock implementation)
        tokio::time::sleep(Duration::from_millis(300)).await;
        
        let success = true;
        
        if success {
            // Remove dataflow from state
            app_state.dataflows.retain(|df| df.name != dataflow_name);
            
            app_state.status_messages.push_back(StatusMessage {
                message: format!("✅ Dataflow '{}' stopped", dataflow_name),
                level: crate::tui::app::MessageLevel::Success,
                timestamp: std::time::Instant::now(),
            });
            
            Ok(CommandResult::StateUpdate(StateUpdate::DataflowRemoved(dataflow_name)))
        } else {
            Ok(CommandResult::Error(format!("Failed to stop dataflow '{}'", dataflow_name)))
        }
    }
    
    async fn execute_build_in_tui(
        &mut self,
        cmd: &BuildCommand,
        app_state: &mut AppState,
    ) -> crate::tui::Result<CommandResult> {
        let target = cmd.dataflow.dataflow.as_ref()
            .map(|p| p.to_string_lossy().to_string())
            .unwrap_or_else(|| "current".to_string());
        
        app_state.status_messages.push_back(StatusMessage {
            message: format!("Building '{}'...", target),
            level: crate::tui::app::MessageLevel::Info,
            timestamp: std::time::Instant::now(),
        });
        
        // Execute build command (mock implementation)
        tokio::time::sleep(Duration::from_millis(1000)).await;
        
        app_state.status_messages.push_back(StatusMessage {
            message: format!("✅ Build completed for '{}'", target),
            level: crate::tui::app::MessageLevel::Success,
            timestamp: std::time::Instant::now(),
        });
        
        Ok(CommandResult::Success(format!("Build completed for '{}'", target)))
    }
    
    async fn execute_inspect_in_tui(
        &mut self,
        cmd: &InspectCommand,
        app_state: &mut AppState,
    ) -> crate::tui::Result<CommandResult> {
        let resource = cmd.resource.as_deref().unwrap_or("system");
        
        // Refresh relevant state
        self.state_synchronizer.refresh_node_state(resource, app_state).await?;
        
        // Determine appropriate view based on resource type
        if resource.starts_with("node_") || app_state.dataflows.iter().any(|df| 
            df.nodes.iter().any(|n| n.id == resource || n.name == resource)
        ) {
            Ok(CommandResult::ViewSwitch(ViewType::NodeInspector { 
                node_id: resource.to_string() 
            }))
        } else {
            Ok(CommandResult::ViewSwitch(ViewType::DataflowManager))
        }
    }
    
    async fn execute_config_in_tui(
        &mut self,
        cmd: &ConfigCommand,
        app_state: &mut AppState,
    ) -> crate::tui::Result<CommandResult> {
        match &cmd.subcommand {
            ConfigSubcommand::Get { key } => {
                let value = self.get_config_value(key);
                Ok(CommandResult::Success(format!("{} = {}", key, value)))
            },
            ConfigSubcommand::Set { key, value } => {
                self.set_config_value(key, value);
                app_state.status_messages.push_back(StatusMessage {
                    message: format!("Set {} = {}", key, value),
                    level: crate::tui::app::MessageLevel::Success,
                    timestamp: std::time::Instant::now(),
                });
                Ok(CommandResult::StateUpdate(StateUpdate::ConfigurationChanged))
            },
            ConfigSubcommand::List => {
                Ok(CommandResult::ViewSwitch(ViewType::SettingsManager))
            },
        }
    }
    
    async fn execute_system_in_tui(
        &mut self,
        cmd: &SystemCommand,
        app_state: &mut AppState,
    ) -> crate::tui::Result<CommandResult> {
        match &cmd.subcommand {
            SystemSubcommand::Status => {
                self.state_synchronizer.refresh_system_metrics(app_state).await?;
                Ok(CommandResult::ViewSwitch(ViewType::SystemMonitor))
            },
            SystemSubcommand::Info => {
                Ok(CommandResult::Success("System info displayed".to_string()))
            },
            SystemSubcommand::Clean => {
                app_state.status_messages.push_back(StatusMessage {
                    message: "Cleaning system cache...".to_string(),
                    level: crate::tui::app::MessageLevel::Info,
                    timestamp: std::time::Instant::now(),
                });
                
                // Mock cleanup
                tokio::time::sleep(Duration::from_millis(500)).await;
                
                app_state.status_messages.push_back(StatusMessage {
                    message: "✅ System cache cleaned".to_string(),
                    level: crate::tui::app::MessageLevel::Success,
                    timestamp: std::time::Instant::now(),
                });
                
                Ok(CommandResult::Success("System cleaned".to_string()))
            },
        }
    }
    
    async fn execute_command_background(
        &mut self,
        command: &Command,
    ) -> crate::tui::Result<CommandResult> {
        // Execute command in background without blocking TUI
        let command_str = format!("{:?}", command);
        
        // This would spawn a background task to execute the command
        // For now, just simulate execution
        tokio::time::sleep(Duration::from_millis(100)).await;
        
        Ok(CommandResult::Success(format!("Command executed: {}", command_str)))
    }
    
    fn parse_ui_command(&self, ui_cmd: &UiCommand) -> Option<ViewType> {
        match &ui_cmd.view {
            Some(TuiView::Dashboard) | None => Some(ViewType::Dashboard),
            Some(TuiView::Dataflow) => Some(ViewType::DataflowManager),
            Some(TuiView::Performance) => Some(ViewType::SystemMonitor),
            Some(TuiView::Logs) => Some(ViewType::LogViewer { target: "system".to_string() }),
        }
    }
    
    fn is_exit_command(&self, command: &Command) -> bool {
        // Check if this is an exit-like command
        matches!(command, Command::Self_(SelfCommand { 
            subcommand: SelfSubcommand::Version | SelfSubcommand::Info 
        }))
    }
    
    fn get_config_value(&self, key: &str) -> String {
        // Mock config retrieval
        match key {
            "theme" => "dark".to_string(),
            "refresh_interval" => "1000".to_string(),
            _ => "unknown".to_string(),
        }
    }
    
    fn set_config_value(&mut self, _key: &str, _value: &str) {
        // Mock config setting
        // This would update the actual configuration
    }
    
    pub fn get_command_history(&self) -> &[String] {
        &self.command_history
    }
}

impl StateSynchronizer {
    pub fn new() -> Self {
        Self {
            last_sync: Instant::now(),
            sync_interval: Duration::from_millis(1000),
            pending_updates: Vec::new(),
        }
    }
    
    pub async fn sync_cli_to_tui(&mut self, app_state: &mut AppState) -> crate::tui::Result<()> {
        if self.last_sync.elapsed() < self.sync_interval {
            return Ok(());
        }
        
        // Refresh all state components
        self.refresh_dataflow_state(app_state).await?;
        self.refresh_system_metrics(app_state).await?;
        
        self.last_sync = Instant::now();
        Ok(())
    }
    
    pub async fn refresh_dataflow_state(&mut self, app_state: &mut AppState) -> crate::tui::Result<()> {
        // Mock dataflow state refresh
        // In a real implementation, this would query the Dora runtime
        
        // Simulate some dataflows if none exist
        if app_state.dataflows.is_empty() {
            app_state.dataflows = vec![
                DataflowInfo {
                    id: "df_example".to_string(),
                    name: "example_dataflow".to_string(),
                    status: "running".to_string(),
                    nodes: vec![
                        NodeInfo {
                            id: "node_input".to_string(),
                            name: "Input Node".to_string(),
                            status: "running".to_string(),
                        },
                        NodeInfo {
                            id: "node_process".to_string(),
                            name: "Processing Node".to_string(),
                            status: "running".to_string(),
                        },
                    ],
                },
            ];
        }
        
        Ok(())
    }
    
    pub async fn refresh_node_state(&mut self, _node_id: &str, app_state: &mut AppState) -> crate::tui::Result<()> {
        // Refresh specific node information
        self.refresh_dataflow_state(app_state).await
    }
    
    pub async fn refresh_system_metrics(&mut self, app_state: &mut AppState) -> crate::tui::Result<()> {
        // Mock system metrics refresh
        app_state.system_metrics.cpu_usage = 45.0;
        app_state.system_metrics.memory_usage = 60.0;
        app_state.system_metrics.network_io = (1024, 512);
        app_state.system_metrics.last_update = Some(Instant::now());
        
        Ok(())
    }
}

impl Default for StateSynchronizer {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cli::context::ExecutionContext;

    #[tokio::test]
    async fn test_command_executor_creation() {
        let context = ExecutionContext::default();
        let executor = TuiCliExecutor::new(context);
        assert!(executor.command_history.is_empty());
    }

    #[tokio::test]
    async fn test_ps_command_execution() {
        let context = ExecutionContext::default();
        let mut executor = TuiCliExecutor::new(context);
        let mut app_state = AppState::default();
        
        let result = executor.execute_command("ps", &mut app_state).await.unwrap();
        
        match result {
            CommandResult::ViewSwitch(ViewType::Dashboard) => {},
            _ => panic!("Expected view switch to dashboard"),
        }
    }

    #[tokio::test]
    async fn test_config_command_execution() {
        let context = ExecutionContext::default();
        let mut executor = TuiCliExecutor::new(context);
        let mut app_state = AppState::default();
        
        let result = executor.execute_command("config set theme dark", &mut app_state).await.unwrap();
        
        match result {
            CommandResult::StateUpdate(StateUpdate::ConfigurationChanged) => {},
            _ => panic!("Expected configuration change"),
        }
    }

    #[test]
    fn test_state_synchronizer() {
        let sync = StateSynchronizer::new();
        assert!(sync.pending_updates.is_empty());
    }
}