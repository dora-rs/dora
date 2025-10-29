use std::time::{Duration, Instant};

use clap::Parser;

use crate::{
    cli::{Cli, Command, commands::*, context::ExecutionContext},
    execute_legacy_command,
    tui::{
        AppState, ViewType,
        app::{DataflowInfo, MessageLevel, NodeInfo, StatusMessage},
        metrics::MetricsCollector,
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

#[derive(Debug)]
enum LegacyExecution {
    Success,
    Error(String),
}

/// Types of state updates that can occur
#[derive(Debug, Clone)]
pub enum StateUpdate {
    DataflowAdded(DataflowInfo),
    DataflowRemoved(String),
    DataflowStatusChanged {
        name: String,
        new_status: String,
    },
    NodeStatusChanged {
        dataflow: String,
        node: String,
        status: String,
    },
    SystemMetricsUpdated,
    ConfigurationChanged,
    RefreshRequired,
}

/// Result of executing a command in command mode
#[derive(Debug, Clone)]
pub enum CommandModeExecutionResult {
    Success {
        message: String,
        view_actions: Vec<CommandModeViewAction>,
        state_updates: Vec<StateUpdate>,
    },
    Error(String),
    Exit,
}

/// View actions triggered by command mode execution
#[derive(Debug, Clone)]
pub enum CommandModeViewAction {
    SwitchView(ViewType),
    RefreshCurrentView,
    ShowMessage { message: String, level: StatusLevel },
    UpdateDataflows,
    UpdateNodes,
}

/// Manages state synchronization between CLI and TUI
#[derive(Debug)]
pub struct StateSynchronizer {
    last_sync: Instant,
    sync_interval: Duration,
    pending_updates: Vec<StateUpdate>,
    metrics_collector: MetricsCollector,
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

        let cli = Cli::try_parse_from(full_args.clone())
            .map_err(|e| format!("Failed to parse CLI args: {}", e))?;

        // Add to history
        self.command_history.push(command_str.to_string());

        // Execute command with TUI context
        let command = cli.command.ok_or("No command provided")?;
        let result = self
            .execute_parsed_command(&command, app_state, &full_args)
            .await?;

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
        full_args: &[String],
    ) -> crate::tui::Result<CommandResult> {
        match command {
            Command::Ps(_) => {
                match self
                    .execute_legacy_and_refresh(full_args, app_state)
                    .await?
                {
                    LegacyExecution::Success => {}
                    LegacyExecution::Error(err) => return Ok(CommandResult::Error(err)),
                }
                self.state_synchronizer
                    .refresh_dataflow_state(app_state)
                    .await?;
                Ok(CommandResult::ViewSwitch(ViewType::Dashboard))
            }

            Command::Start(_)
            | Command::Stop(_)
            | Command::Up(_)
            | Command::Destroy(_)
            | Command::Build(_)
            | Command::New(_)
            | Command::Check(_)
            | Command::Graph(_)
            | Command::Daemon(_)
            | Command::Runtime(_)
            | Command::Coordinator(_)
            | Command::Self_(_) => {
                match self
                    .execute_legacy_and_refresh(full_args, app_state)
                    .await?
                {
                    LegacyExecution::Success => {
                        Ok(CommandResult::StateUpdate(StateUpdate::RefreshRequired))
                    }
                    LegacyExecution::Error(err) => Ok(CommandResult::Error(err)),
                }
            }

            Command::Inspect(cmd) => self.execute_inspect_in_tui(cmd, app_state).await,

            Command::Logs(cmd) => {
                match self
                    .execute_legacy_and_refresh(full_args, app_state)
                    .await?
                {
                    LegacyExecution::Success => {
                        Ok(CommandResult::ViewSwitch(ViewType::LogViewer {
                            target: cmd
                                .dataflow
                                .dataflow
                                .as_ref()
                                .map(|df| df.to_string_lossy().to_string())
                                .unwrap_or_else(|| "system".to_string()),
                        }))
                    }
                    LegacyExecution::Error(err) => Ok(CommandResult::Error(err)),
                }
            }

            Command::Debug(cmd) => Ok(CommandResult::ViewSwitch(ViewType::DebugSession {
                dataflow_id: cmd
                    .dataflow
                    .dataflow
                    .as_ref()
                    .map(|df| df.to_string_lossy().to_string())
                    .unwrap_or_else(|| "current".to_string()),
            })),

            Command::Monitor(_) => Ok(CommandResult::ViewSwitch(ViewType::SystemMonitor)),

            Command::Tui(ui_cmd) => {
                if let Some(view) = self.parse_ui_command(ui_cmd) {
                    Ok(CommandResult::ViewSwitch(view))
                } else {
                    Ok(CommandResult::Success("Already in TUI mode".to_string()))
                }
            }

            Command::Config(_) => {
                match self
                    .execute_legacy_and_refresh(full_args, app_state)
                    .await?
                {
                    LegacyExecution::Success => Ok(CommandResult::StateUpdate(
                        StateUpdate::ConfigurationChanged,
                    )),
                    LegacyExecution::Error(err) => Ok(CommandResult::Error(err)),
                }
            }

            Command::System(_) | Command::Analyze(_) | Command::Help(_) => {
                match self
                    .execute_legacy_and_refresh(full_args, app_state)
                    .await?
                {
                    LegacyExecution::Success => Ok(CommandResult::Success(format!(
                        "Command `{}` completed",
                        full_args
                            .iter()
                            .skip(1)
                            .cloned()
                            .collect::<Vec<_>>()
                            .join(" ")
                    ))),
                    LegacyExecution::Error(err) => Ok(CommandResult::Error(err)),
                }
            }

            _ if self.is_exit_command(command) => Ok(CommandResult::Exit),

            // Other commands - execute in background
            _ => match self
                .execute_legacy_and_refresh(full_args, app_state)
                .await?
            {
                LegacyExecution::Success => Ok(CommandResult::Success(format!(
                    "Command `{}` completed",
                    full_args
                        .iter()
                        .skip(1)
                        .cloned()
                        .collect::<Vec<_>>()
                        .join(" ")
                ))),
                LegacyExecution::Error(err) => Ok(CommandResult::Error(err)),
            },
        }
    }

    async fn execute_legacy_and_refresh(
        &mut self,
        full_args: &[String],
        app_state: &mut AppState,
    ) -> crate::tui::Result<LegacyExecution> {
        let display = full_args
            .iter()
            .skip(1)
            .cloned()
            .collect::<Vec<_>>()
            .join(" ");

        #[cfg(test)]
        {
            if std::env::var("DORA_TUI_RUN_REAL_COMMANDS").is_err() {
                self.push_status_message(
                    app_state,
                    format!("Skipping `{display}` execution in test mode"),
                    MessageLevel::Info,
                );
                return Ok(LegacyExecution::Success);
            }
        }

        self.push_status_message(
            app_state,
            format!("Running `{display}`..."),
            MessageLevel::Info,
        );

        let args: Vec<String> = full_args.iter().skip(1).cloned().collect();
        let working_dir = self.execution_context.working_dir.clone();

        let result = tokio::task::spawn_blocking(move || {
            execute_legacy_command(args.iter().map(|s| s.as_str()), Some(working_dir.as_path()))
        })
        .await;

        match result {
            Ok(Ok(())) => {
                self.push_status_message(
                    app_state,
                    format!("✅ `{display}` completed"),
                    MessageLevel::Success,
                );
                Ok(LegacyExecution::Success)
            }
            Ok(Err(err)) => {
                let message = err.to_string();
                self.push_status_message(
                    app_state,
                    format!("❌ `{display}` failed: {}", message),
                    MessageLevel::Error,
                );
                Ok(LegacyExecution::Error(message))
            }
            Err(err) => {
                let message = format!("failed to join command task: {err}");
                self.push_status_message(app_state, format!("❌ {message}"), MessageLevel::Error);
                Ok(LegacyExecution::Error(message))
            }
        }
    }

    fn push_status_message(
        &mut self,
        app_state: &mut AppState,
        message: String,
        level: MessageLevel,
    ) {
        app_state.status_messages.push_back(StatusMessage {
            message,
            level,
            timestamp: Instant::now(),
        });

        while app_state.status_messages.len() > 20 {
            app_state.status_messages.pop_front();
        }
    }

    async fn execute_inspect_in_tui(
        &mut self,
        cmd: &InspectCommand,
        app_state: &mut AppState,
    ) -> crate::tui::Result<CommandResult> {
        let resource = cmd.target.as_deref().unwrap_or("system");

        // Refresh relevant state
        self.state_synchronizer
            .refresh_node_state(resource, app_state)
            .await?;

        // Determine appropriate view based on resource type
        if let Some((dataflow, node)) = find_node_by_identifier(app_state, resource) {
            return Ok(CommandResult::ViewSwitch(ViewType::NodeInspector {
                dataflow_id: dataflow.id.clone(),
                node_id: node.id.clone(),
            }));
        }

        if let Some(dataflow) = app_state
            .dataflows
            .iter()
            .find(|df| df.id == resource || df.name == resource)
        {
            if let Some(node) = preferred_node(&dataflow.nodes) {
                return Ok(CommandResult::ViewSwitch(ViewType::NodeInspector {
                    dataflow_id: dataflow.id.clone(),
                    node_id: node.id.clone(),
                }));
            }
        }

        Ok(CommandResult::ViewSwitch(ViewType::DataflowManager))
    }

    fn parse_ui_command(&self, ui_cmd: &UiCommand) -> Option<ViewType> {
        match &ui_cmd.view {
            Some(TuiView::Dashboard) | None => Some(ViewType::Dashboard),
            Some(TuiView::Dataflow) => Some(ViewType::DataflowManager),
            Some(TuiView::Performance) => Some(ViewType::SystemMonitor),
            Some(TuiView::Logs) => Some(ViewType::LogViewer {
                target: "system".to_string(),
            }),
        }
    }

    fn is_exit_command(&self, command: &Command) -> bool {
        // Check if this is an exit-like command
        matches!(
            command,
            Command::Self_(SelfCommand {
                subcommand: SelfSubcommand::Version | SelfSubcommand::Info
            })
        )
    }

    pub fn get_command_history(&self) -> &[String] {
        &self.command_history
    }

    /// Execute a command in command mode and return structured result
    pub async fn execute_command_mode(
        &mut self,
        command_str: &str,
        app_state: &mut AppState,
    ) -> crate::tui::Result<CommandModeExecutionResult> {
        // Parse and execute command
        let result = self.execute_command(command_str, app_state).await?;

        // Convert CommandResult to CommandModeExecutionResult
        match result {
            CommandResult::Success(msg) => Ok(CommandModeExecutionResult::Success {
                message: msg,
                view_actions: vec![],
                state_updates: vec![],
            }),
            CommandResult::ViewSwitch(view_type) => Ok(CommandModeExecutionResult::Success {
                message: format!("Switched to {} view", format!("{:?}", view_type)),
                view_actions: vec![CommandModeViewAction::SwitchView(view_type)],
                state_updates: vec![],
            }),
            CommandResult::Exit => Ok(CommandModeExecutionResult::Exit),
            CommandResult::Error(err) => Ok(CommandModeExecutionResult::Error(err)),
            CommandResult::StateUpdate(update) => Ok(CommandModeExecutionResult::Success {
                message: "State updated".to_string(),
                view_actions: vec![CommandModeViewAction::RefreshCurrentView],
                state_updates: vec![update],
            }),
        }
    }
}

fn find_node_by_identifier<'a>(
    app_state: &'a AppState,
    identifier: &str,
) -> Option<(&'a DataflowInfo, &'a NodeInfo)> {
    app_state.dataflows.iter().find_map(|df| {
        df.nodes
            .iter()
            .find(|node| node.id == identifier || node.name == identifier)
            .map(|node| (df, node))
    })
}

fn preferred_node<'a>(nodes: &'a [NodeInfo]) -> Option<&'a NodeInfo> {
    nodes
        .iter()
        .find(|node| {
            let status = node.status.to_ascii_lowercase();
            status == "running" || status == "active"
        })
        .or_else(|| nodes.first())
}

impl StateSynchronizer {
    pub fn new() -> Self {
        Self {
            last_sync: Instant::now(),
            sync_interval: Duration::from_millis(1000),
            pending_updates: Vec::new(),
            metrics_collector: MetricsCollector::new(),
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

    pub async fn refresh_dataflow_state(
        &mut self,
        app_state: &mut AppState,
    ) -> crate::tui::Result<()> {
        let result = tokio::task::spawn_blocking(crate::tui::app::fetch_dataflows).await;

        match result {
            Ok(Ok(flows)) => {
                app_state.dataflows = flows;
                app_state.last_error = None;
            }
            Ok(Err(err)) => {
                self.capture_error(app_state, err.to_string());
            }
            Err(err) => {
                self.capture_error(app_state, format!("failed to refresh dataflows: {err}"));
            }
        }

        app_state.dataflow_last_refresh = Some(Instant::now());

        Ok(())
    }

    pub async fn refresh_node_state(
        &mut self,
        _node_id: &str,
        app_state: &mut AppState,
    ) -> crate::tui::Result<()> {
        // Refresh specific node information
        self.refresh_dataflow_state(app_state).await
    }

    pub async fn refresh_system_metrics(
        &mut self,
        app_state: &mut AppState,
    ) -> crate::tui::Result<()> {
        match self.metrics_collector.collect() {
            Ok(metrics) => {
                app_state.system_metrics = metrics;
                app_state.last_error = None;
            }
            Err(err) => self.capture_error(app_state, err.to_string()),
        }

        Ok(())
    }
}

impl StateSynchronizer {
    fn capture_error(&mut self, app_state: &mut AppState, message: String) {
        if app_state.last_error.as_deref() != Some(message.as_str()) {
            app_state.status_messages.push_back(StatusMessage {
                message: format!("❌ {}", message),
                level: MessageLevel::Error,
                timestamp: Instant::now(),
            });
            while app_state.status_messages.len() > 20 {
                app_state.status_messages.pop_front();
            }
        }
        app_state.last_error = Some(message);
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

        let result = executor
            .execute_command("ps", &mut app_state)
            .await
            .unwrap();

        match result {
            CommandResult::ViewSwitch(ViewType::Dashboard) => {}
            _ => panic!("Expected view switch to dashboard"),
        }
    }

    #[tokio::test]
    async fn test_config_command_execution() {
        let context = ExecutionContext::default();
        let mut executor = TuiCliExecutor::new(context);
        let mut app_state = AppState::default();

        let result = executor
            .execute_command("config set theme dark", &mut app_state)
            .await
            .unwrap();

        match result {
            CommandResult::StateUpdate(StateUpdate::ConfigurationChanged) => {}
            _ => panic!("Expected configuration change"),
        }
    }

    #[test]
    fn test_state_synchronizer() {
        let sync = StateSynchronizer::new();
        assert!(sync.pending_updates.is_empty());
    }
}
