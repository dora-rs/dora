use crossterm::{
    event::{
        self, DisableMouseCapture, EnableMouseCapture, Event, KeyCode, KeyEvent, KeyEventKind,
    },
    execute,
    terminal::{EnterAlternateScreen, LeaveAlternateScreen, disable_raw_mode, enable_raw_mode},
};
use ratatui::{
    backend::CrosstermBackend,
    layout::{Alignment, Constraint, Direction, Layout, Rect},
    style::Style,
    terminal::{Frame, Terminal},
    widgets::{Block, Borders, Clear, Paragraph, Wrap},
};
use std::{
    collections::{HashMap, VecDeque},
    io,
    path::PathBuf,
    time::{Duration, Instant},
};

use dora_core::topics::DORA_COORDINATOR_PORT_CONTROL_DEFAULT;
use dora_message::{
    coordinator_to_cli::{DataflowListEntry, DataflowStatus, NodeRuntimeInfo, NodeRuntimeState},
    descriptor::{CoreNodeKind, ResolvedNode},
};
use eyre::WrapErr;

use super::{
    Result,
    cli_integration::{CliContext, CommandHistory, CommandMode},
    command_executor::{CommandModeExecutionResult, CommandModeViewAction, TuiCliExecutor},
    command_mode::{CommandModeAction, CommandModeManager},
    metrics::MetricsCollector,
    theme::ThemeConfig,
};
use crate::{
    LOCALHOST,
    common::{connect_to_coordinator, query_running_dataflows},
    config::preferences::UserPreferences,
};

#[derive(Debug, Clone)]
pub enum ViewType {
    Dashboard,
    DataflowManager,
    DataflowExplorer,
    NodeInspector { node_id: String },
    SystemMonitor,
    LogViewer { target: String },
    RecordingAnalyzer { recording_id: String },
    DebugSession { dataflow_id: String },
    SettingsManager,
    Help,
}

impl Default for ViewType {
    fn default() -> Self {
        ViewType::Dashboard
    }
}

#[derive(Debug, Default)]
pub struct AppState {
    /// Global data cache
    pub data_cache: DataCache,

    /// Current dataflows
    pub dataflows: Vec<DataflowInfo>,

    /// System metrics
    pub system_metrics: SystemMetrics,

    /// User preferences
    pub user_config: UserConfig,

    /// Last error message
    pub last_error: Option<String>,

    /// Status messages
    pub status_messages: VecDeque<StatusMessage>,
}

#[derive(Debug, Default)]
pub struct DataCache {
    pub last_update: Option<Instant>,
    pub cached_data: HashMap<String, serde_json::Value>,
}

#[derive(Debug, Default, Clone)]
pub struct DataflowInfo {
    pub id: String,
    pub name: String,
    pub status: String,
    pub nodes: Vec<NodeInfo>,
}

#[derive(Debug, Default, Clone)]
pub struct NodeInfo {
    pub id: String,
    pub name: String,
    pub status: String,
    pub kind: String,
    pub description: Option<String>,
    pub inputs: Vec<String>,
    pub outputs: Vec<String>,
    pub source: Option<String>,
    pub resolved: Option<ResolvedNode>,
}

#[derive(Debug, Default, Clone)]
pub struct SystemMetrics {
    pub cpu_usage: f32,
    pub memory_usage: f32,
    pub network_io: (u64, u64),
    pub memory: MemoryMetrics,
    pub disk: DiskMetrics,
    pub network: NetworkMetrics,
    pub load_average: Option<LoadAverages>,
    pub uptime: Duration,
    pub process_count: usize,
    pub last_update: Option<Instant>,
}

#[derive(Debug, Default, Clone)]
pub struct MemoryMetrics {
    pub total_bytes: u64,
    pub used_bytes: u64,
    pub free_bytes: u64,
    pub usage_percent: f32,
    pub swap_total_bytes: u64,
    pub swap_used_bytes: u64,
    pub swap_usage_percent: f32,
}

#[derive(Debug, Default, Clone)]
pub struct DiskMetrics {
    pub total_bytes: u64,
    pub used_bytes: u64,
    pub usage_percent: f32,
}

#[derive(Debug, Default, Clone)]
pub struct NetworkMetrics {
    pub total_received: u64,
    pub total_transmitted: u64,
    pub received_per_second: f64,
    pub transmitted_per_second: f64,
}

#[derive(Debug, Default, Clone)]
pub struct LoadAverages {
    pub one: f64,
    pub five: f64,
    pub fifteen: f64,
}

#[derive(Debug, Default, Clone)]
pub struct UserConfig {
    pub theme_name: String,
    pub auto_refresh_interval: Duration,
    pub show_system_info: bool,
}

#[derive(Debug, Clone)]
pub struct StatusMessage {
    pub message: String,
    pub level: MessageLevel,
    pub timestamp: Instant,
}

#[derive(Debug, Clone)]
pub enum MessageLevel {
    Info,
    Success,
    Warning,
    Error,
}

#[derive(Debug)]
pub struct EventHandler {
    last_update: Instant,
}

impl EventHandler {
    pub fn new() -> Self {
        Self {
            last_update: Instant::now(),
        }
    }
}

impl Default for EventHandler {
    fn default() -> Self {
        Self::new()
    }
}

impl From<DataflowListEntry> for DataflowInfo {
    fn from(entry: DataflowListEntry) -> Self {
        let DataflowListEntry { id, status, nodes } = entry;
        let uuid = id.uuid.to_string();
        let name = id.name.unwrap_or_else(|| uuid.clone());
        let status = format_dataflow_status(status);
        let nodes = nodes.into_iter().map(NodeInfo::from).collect();

        Self {
            id: uuid,
            name,
            status,
            nodes,
        }
    }
}

impl From<NodeRuntimeInfo> for NodeInfo {
    fn from(mut runtime: NodeRuntimeInfo) -> Self {
        if runtime.inputs.is_empty() && runtime.outputs.is_empty() {
            let (inputs, outputs) = extract_node_connections(&runtime.node);
            runtime.inputs = inputs;
            runtime.outputs = outputs;
        }

        let status = format_node_status(runtime.status);
        let kind = describe_node_kind(&runtime.node.kind);
        let source = derive_node_source(&runtime.node.kind);
        let description = runtime.node.description.clone();

        Self {
            id: runtime.node.id.to_string(),
            name: runtime
                .node
                .name
                .clone()
                .unwrap_or_else(|| runtime.node.id.to_string()),
            status: status.to_string(),
            kind,
            description,
            inputs: runtime.inputs,
            outputs: runtime.outputs,
            source,
            resolved: Some(runtime.node),
        }
    }
}

pub fn fetch_dataflows() -> eyre::Result<Vec<DataflowInfo>> {
    let coordinator_addr = (LOCALHOST, DORA_COORDINATOR_PORT_CONTROL_DEFAULT).into();
    let mut session =
        connect_to_coordinator(coordinator_addr).wrap_err("failed to connect to coordinator")?;
    let list =
        query_running_dataflows(&mut *session).wrap_err("failed to query running dataflows")?;

    Ok(list.0.into_iter().map(DataflowInfo::from).collect())
}

fn format_dataflow_status(status: DataflowStatus) -> String {
    match status {
        DataflowStatus::Running => "running",
        DataflowStatus::Finished => "finished",
        DataflowStatus::Failed => "failed",
    }
    .to_string()
}

fn format_node_status(status: NodeRuntimeState) -> &'static str {
    match status {
        NodeRuntimeState::Running => "running",
        NodeRuntimeState::Exited => "exited",
        NodeRuntimeState::Completed => "completed",
        NodeRuntimeState::Failed => "failed",
        NodeRuntimeState::Unknown => "unknown",
    }
}

fn describe_node_kind(kind: &CoreNodeKind) -> String {
    match kind {
        CoreNodeKind::Custom(custom) => {
            format!("custom ({})", custom.path)
        }
        CoreNodeKind::Runtime(runtime) => {
            format!("runtime ({} operators)", runtime.operators.len())
        }
    }
}

fn derive_node_source(kind: &CoreNodeKind) -> Option<String> {
    match kind {
        CoreNodeKind::Custom(custom) => Some(custom.path.clone()),
        CoreNodeKind::Runtime(_) => Some("runtime".to_string()),
    }
}

fn extract_node_connections(node: &ResolvedNode) -> (Vec<String>, Vec<String>) {
    match &node.kind {
        CoreNodeKind::Custom(custom) => {
            let inputs = custom
                .run_config
                .inputs
                .iter()
                .map(|(id, input)| format!("{id} <- {}", input.mapping))
                .collect();
            let outputs = custom
                .run_config
                .outputs
                .iter()
                .map(|id| id.to_string())
                .collect();
            (inputs, outputs)
        }
        CoreNodeKind::Runtime(runtime) => {
            let mut inputs = Vec::new();
            let mut outputs = Vec::new();

            for operator in &runtime.operators {
                let label = operator
                    .config
                    .name
                    .clone()
                    .unwrap_or_else(|| operator.id.to_string());
                for (input_id, input) in &operator.config.inputs {
                    inputs.push(format!("{label}.{input_id} <- {}", input.mapping));
                }
                for output_id in &operator.config.outputs {
                    outputs.push(format!("{label}.{output_id}"));
                }
            }

            (inputs, outputs)
        }
    }
}

#[derive(Debug)]
pub struct DoraApp {
    /// Current active view
    current_view: ViewType,

    /// View navigation stack
    view_stack: Vec<ViewType>,

    /// Global application state
    state: AppState,

    /// Event handler
    event_handler: EventHandler,

    /// Theme configuration
    theme: ThemeConfig,

    /// CLI context for command execution
    cli_context: Option<CliContext>,

    /// Command mode state (legacy)
    command_mode: CommandMode,

    /// Command mode manager (new implementation)
    command_mode_manager: CommandModeManager,

    /// CLI command executor for TUI
    command_executor: Option<TuiCliExecutor>,

    /// System metrics collector
    metrics_collector: MetricsCollector,

    /// Should quit flag
    should_quit: bool,
}

impl DoraApp {
    pub fn new(initial_view: ViewType) -> Self {
        let mut app = Self {
            current_view: initial_view,
            view_stack: Vec::new(),
            state: AppState::default(),
            event_handler: EventHandler::new(),
            theme: ThemeConfig::load_user_theme(),
            cli_context: None,
            command_mode: CommandMode::Normal,
            command_mode_manager: CommandModeManager::new(),
            command_executor: None,
            metrics_collector: MetricsCollector::new(),
            should_quit: false,
        };

        app.apply_user_preferences();
        app
    }

    pub fn new_with_context(initial_view: ViewType, cli_context: CliContext) -> Self {
        let mut app = Self::new(initial_view);
        app.cli_context = Some(cli_context.clone());

        // Initialize command executor with a default execution context
        let execution_context = crate::cli::context::ExecutionContext::detect_basic();
        app.command_executor = Some(TuiCliExecutor::new(execution_context));

        app
    }

    fn apply_user_preferences(&mut self) {
        match UserPreferences::load_or_create() {
            Ok(prefs) => {
                self.state.user_config.theme_name = prefs.interface.tui.theme.clone();
                self.state.user_config.auto_refresh_interval =
                    prefs.interface.tui.auto_refresh_interval;
                self.state.user_config.show_system_info = prefs.interface.hints.show_hints;

                self.theme = ThemeConfig::from_name(&self.state.user_config.theme_name);

                if let Some(view) = Self::view_from_name(&prefs.interface.tui.default_view) {
                    self.current_view = view;
                }
            }
            Err(err) => {
                self.show_status_message(
                    format!("❌ failed to load preferences: {err}"),
                    MessageLevel::Error,
                );
                self.state.user_config.auto_refresh_interval = Duration::from_secs(5);
                self.state.user_config.show_system_info = true;
                self.state.user_config.theme_name = "dark".to_string();
                self.theme = ThemeConfig::from_name(&self.state.user_config.theme_name);
            }
        }
    }

    fn view_from_name(name: &str) -> Option<ViewType> {
        match name.to_lowercase().as_str() {
            "dashboard" => Some(ViewType::Dashboard),
            "dataflow" | "dataflow_manager" => Some(ViewType::DataflowManager),
            "explorer" | "dataflow_explorer" => Some(ViewType::DataflowExplorer),
            "system" | "monitor" | "system_monitor" => Some(ViewType::SystemMonitor),
            "logs" | "log" => Some(ViewType::LogViewer {
                target: "system".to_string(),
            }),
            _ => None,
        }
    }

    pub async fn run(&mut self) -> Result<()> {
        // Setup terminal
        enable_raw_mode()?;
        let mut stdout = io::stdout();
        execute!(stdout, EnterAlternateScreen, EnableMouseCapture)?;
        let backend = CrosstermBackend::new(stdout);
        let mut terminal = Terminal::new(backend)?;

        // Initialize application state
        self.initialize().await?;

        // Main application loop
        let result = self.run_event_loop(&mut terminal).await;

        // Cleanup terminal
        disable_raw_mode()?;
        execute!(
            terminal.backend_mut(),
            LeaveAlternateScreen,
            DisableMouseCapture
        )?;
        terminal.show_cursor()?;

        result
    }

    async fn initialize(&mut self) -> Result<()> {
        // Initialize application state
        self.state.user_config.auto_refresh_interval = Duration::from_secs(5);
        self.state.user_config.show_system_info = true;
        self.state.user_config.theme_name = "dark".to_string();

        // Load initial data
        self.refresh_dataflow_list().await?;
        self.update_system_metrics().await?;

        Ok(())
    }

    async fn run_event_loop(
        &mut self,
        terminal: &mut Terminal<CrosstermBackend<std::io::Stdout>>,
    ) -> Result<()> {
        loop {
            // Render current view
            terminal.draw(|f| self.ui(f))?;

            // Handle events with timeout for periodic updates
            if crossterm::event::poll(Duration::from_millis(100))? {
                if let Event::Key(key) = event::read()? {
                    if key.kind == KeyEventKind::Press {
                        self.handle_key_event(key).await?;
                    }
                }
            }

            // Periodic updates
            self.update().await?;

            if self.should_quit {
                break;
            }
        }

        Ok(())
    }

    fn ui(&mut self, f: &mut Frame) {
        let size = f.size();

        // Main layout: header + body + footer
        let chunks = Layout::default()
            .direction(Direction::Vertical)
            .constraints([
                Constraint::Length(3), // Header
                Constraint::Min(0),    // Body
                Constraint::Length(3), // Footer/Status
            ])
            .split(size);

        self.render_header(f, chunks[0]);
        self.render_current_view(f, chunks[1]);
        self.render_footer(f, chunks[2]);

        // Render overlays (command mode, help, etc.)
        self.render_overlays(f, size);
    }

    fn render_header(&mut self, f: &mut Frame, area: Rect) {
        let title = format!("🚀 Dora TUI - {}", self.view_title());
        let header = Paragraph::new(title)
            .style(self.theme.styles.highlight_style)
            .block(
                self.theme
                    .styled_block("Dora CLI/TUI")
                    .borders(Borders::ALL),
            )
            .alignment(Alignment::Center);

        f.render_widget(header, area);
    }

    fn render_current_view(&mut self, f: &mut Frame, area: Rect) {
        use super::views::*;

        match &self.current_view {
            ViewType::Dashboard => {
                let mut view = DashboardView::new(&self.theme);
                view.render(f, area, &self.state);
            }
            ViewType::DataflowManager => {
                let mut view = DataflowManagerView::new(&self.theme);
                view.render(f, area, &self.state);
            }
            ViewType::DataflowExplorer => {
                let mut view = DataflowExplorerView::new(&self.theme);
                view.render(f, area, &self.state);
            }
            ViewType::NodeInspector { node_id } => {
                let mut view = NodeInspectorView::new(&self.theme, node_id.clone());
                view.render(f, area, &self.state);
            }
            ViewType::SystemMonitor => {
                let mut view = SystemMonitorView::new(&self.theme);
                view.render(f, area, &self.state);
            }
            ViewType::LogViewer { target } => {
                let mut view = LogViewerView::new(target, &self.theme);
                view.render(f, area, &self.state);
            }
            ViewType::Help => {
                let mut view = HelpView::new(&self.theme);
                view.render(f, area, &self.state);
            }
            _ => {
                // Fallback for unimplemented views
                let content = match &self.current_view {
                    ViewType::RecordingAnalyzer { recording_id } => format!("🎬 Recording Analyzer\n\nAnalyzing recording: {}\n\nPlayback controls and data analysis tools.", recording_id),
                    ViewType::DebugSession { dataflow_id } => format!("🐛 Debug Session\n\nDebugging dataflow: {}\n\nBreakpoints, step-through debugging, and variable inspection.", dataflow_id),
                    ViewType::SettingsManager => "⚙️ Settings Manager\n\nConfigure Dora CLI/TUI:\n- Theme settings\n- Key bindings\n- Default behaviors\n- Performance tuning".to_string(),
                    _ => "View not implemented yet".to_string(),
                };

                let title = self.view_title();
                let view_widget = Paragraph::new(content)
                    .style(Style::default().fg(self.theme.colors.text))
                    .block(self.theme.styled_block(&title).borders(Borders::ALL))
                    .wrap(Wrap { trim: true });

                f.render_widget(view_widget, area);
            }
        }
    }

    fn render_footer(&mut self, f: &mut Frame, area: Rect) {
        let footer_text = if self.command_mode_manager.is_active() {
            // Command mode footer is rendered by the command mode manager
            "".to_string()
        } else {
            "Press 'q' to quit, ':' for command mode, F1 for help".to_string()
        };

        let footer = Paragraph::new(footer_text)
            .style(self.theme.styles.status_style)
            .block(
                Block::default()
                    .borders(Borders::ALL)
                    .border_type(self.theme.styles.border_style)
                    .border_style(Style::default().fg(self.theme.colors.border)),
            );

        f.render_widget(footer, area);
    }

    fn render_overlays(&mut self, f: &mut Frame, area: Rect) {
        // Render command mode if active
        if self.command_mode_manager.is_active() {
            self.command_mode_manager.render(f, area, &self.theme);
        }

        // Render status messages as overlay (but not over command mode)
        if let Some(status) = self.state.status_messages.back() {
            if !self.command_mode_manager.is_active() {
                let popup_area = centered_rect(60, 20, area);

                let message = Paragraph::new(status.message.as_str())
                    .style(match status.level {
                        MessageLevel::Info => Style::default().fg(self.theme.colors.text),
                        MessageLevel::Success => Style::default().fg(self.theme.colors.success),
                        MessageLevel::Warning => Style::default().fg(self.theme.colors.warning),
                        MessageLevel::Error => Style::default().fg(self.theme.colors.error),
                    })
                    .block(
                        Block::default()
                            .title("Status")
                            .borders(Borders::ALL)
                            .border_type(self.theme.styles.border_style),
                    )
                    .alignment(Alignment::Center);

                f.render_widget(Clear, popup_area);
                f.render_widget(message, popup_area);
            }
        }
    }

    async fn handle_key_event(&mut self, key: KeyEvent) -> Result<()> {
        // Check if we're in new command mode first
        if self.command_mode_manager.is_active() {
            let action = self
                .command_mode_manager
                .handle_key_event(key, &self.state)
                .await?;
            self.handle_command_mode_action(action).await?;
            return Ok(());
        }

        // Global key bindings
        match key.code {
            KeyCode::Char('q') => {
                self.should_quit = true;
                return Ok(());
            }
            KeyCode::Char(':') => {
                self.command_mode_manager.activate();
                return Ok(());
            }
            KeyCode::F(1) => {
                self.switch_view(ViewType::Help);
                return Ok(());
            }
            KeyCode::Esc => {
                if !self.view_stack.is_empty() {
                    self.pop_view();
                }
                return Ok(());
            }
            _ => {}
        }

        // View navigation shortcuts
        match key.code {
            KeyCode::Char('1') => self.switch_view(ViewType::Dashboard),
            KeyCode::Char('2') => self.switch_view(ViewType::DataflowManager),
            KeyCode::Char('3') => self.switch_view(ViewType::SystemMonitor),
            KeyCode::Char('4') => self.switch_view(ViewType::LogViewer {
                target: "system".to_string(),
            }),
            KeyCode::Char('5') => self.switch_view(ViewType::SettingsManager),
            KeyCode::Char('e') => self.switch_view(ViewType::DataflowExplorer),
            _ => {}
        }

        Ok(())
    }

    async fn handle_command_mode_action(&mut self, action: CommandModeAction) -> Result<()> {
        match action {
            CommandModeAction::None => {
                // Nothing to do
            }
            CommandModeAction::UpdateDisplay => {
                // UI will be updated on next render
            }
            CommandModeAction::ExecuteCommand {
                command,
                show_output,
            } => {
                self.execute_command_mode_command(&command, show_output)
                    .await?;
            }
            CommandModeAction::Cancel => {
                // Command mode is already deactivated by the manager
            }
            CommandModeAction::SwitchView(view_type) => {
                self.switch_view(view_type);
            }
        }
        Ok(())
    }

    async fn execute_command_mode_command(
        &mut self,
        command: &str,
        show_output: bool,
    ) -> Result<()> {
        if let Some(executor) = &mut self.command_executor {
            let result = executor
                .execute_command_mode(command, &mut self.state)
                .await?;

            // Handle the result
            match result {
                CommandModeExecutionResult::Success {
                    message,
                    view_actions,
                    state_updates,
                } => {
                    // Handle view actions
                    for view_action in view_actions {
                        match view_action {
                            CommandModeViewAction::SwitchView(view_type) => {
                                self.switch_view(view_type);
                            }
                            CommandModeViewAction::RefreshCurrentView => {
                                // Trigger a refresh of current view data
                                self.refresh_current_view_data().await?;
                            }
                            CommandModeViewAction::ShowMessage { message, level } => {
                                // Convert StatusLevel to MessageLevel
                                use crate::tui::StatusLevel;
                                let msg_level = match level {
                                    StatusLevel::Info => MessageLevel::Info,
                                    StatusLevel::Success => MessageLevel::Success,
                                    StatusLevel::Warning => MessageLevel::Warning,
                                    StatusLevel::Error => MessageLevel::Error,
                                };
                                self.show_status_message(message, msg_level);
                            }
                            CommandModeViewAction::UpdateDataflows => {
                                // Refresh dataflow list
                                self.refresh_current_view_data().await?;
                            }
                            CommandModeViewAction::UpdateNodes => {
                                // Refresh node information
                                self.refresh_current_view_data().await?;
                            }
                        }
                    }

                    // Show execution result if requested
                    if show_output {
                        self.show_status_message(message, MessageLevel::Success);
                    }
                }
                CommandModeExecutionResult::Error(err) => {
                    if show_output {
                        self.show_status_message(err, MessageLevel::Error);
                    }
                }
                CommandModeExecutionResult::Exit => {
                    // Handle exit
                    self.should_quit = true;
                }
            }
        }

        Ok(())
    }

    async fn refresh_current_view_data(&mut self) -> Result<()> {
        // Refresh data based on current view
        match &self.current_view {
            ViewType::Dashboard | ViewType::DataflowManager | ViewType::DataflowExplorer => {
                self.refresh_dataflow_list().await?;
            }
            ViewType::SystemMonitor => {
                self.update_system_metrics().await?;
            }
            ViewType::NodeInspector { .. } => {
                self.refresh_dataflow_list().await?;
            }
            _ => {
                // Other views don't need specific refresh
            }
        }
        Ok(())
    }

    async fn update(&mut self) -> Result<()> {
        let now = Instant::now();

        // Auto-refresh system metrics
        if self.state.system_metrics.last_update.map_or(true, |last| {
            now.duration_since(last) > self.state.user_config.auto_refresh_interval
        }) {
            self.update_system_metrics().await?;
        }

        // Clear old status messages
        self.state
            .status_messages
            .retain(|msg| now.duration_since(msg.timestamp) < Duration::from_secs(5));

        Ok(())
    }

    fn view_title(&self) -> String {
        match &self.current_view {
            ViewType::Dashboard => "Dashboard".to_string(),
            ViewType::DataflowManager => "Dataflow Manager".to_string(),
            ViewType::DataflowExplorer => "Dataflow Explorer".to_string(),
            ViewType::NodeInspector { node_id } => format!("Node Inspector: {}", node_id),
            ViewType::SystemMonitor => "System Monitor".to_string(),
            ViewType::LogViewer { target } => format!("Logs: {}", target),
            ViewType::RecordingAnalyzer { recording_id } => format!("Recording: {}", recording_id),
            ViewType::DebugSession { dataflow_id } => format!("Debug: {}", dataflow_id),
            ViewType::SettingsManager => "Settings".to_string(),
            ViewType::Help => "Help".to_string(),
        }
    }

    pub fn switch_view(&mut self, view_type: ViewType) {
        self.current_view = view_type;
    }

    pub fn push_view(&mut self, view_type: ViewType) {
        self.view_stack
            .push(std::mem::replace(&mut self.current_view, view_type));
    }

    pub fn pop_view(&mut self) {
        if let Some(previous_view) = self.view_stack.pop() {
            self.current_view = previous_view;
        }
    }

    pub fn is_in_command_mode(&self) -> bool {
        self.command_mode_manager.is_active()
    }

    fn enter_command_mode_internal(&mut self) {
        self.command_mode_manager.activate();
        self.command_mode = CommandMode::Command {
            buffer: String::new(),
            cursor: 0,
            history: self.load_command_history(),
            history_index: None,
        };
    }

    fn exit_command_mode_internal(&mut self) {
        self.command_mode = CommandMode::Normal;
        self.command_mode_manager.deactivate();
    }

    async fn handle_command_mode_key(&mut self, key: KeyEvent) -> Result<()> {
        if let CommandMode::Command {
            ref mut buffer,
            ref mut cursor,
            ref history,
            ref mut history_index,
        } = self.command_mode
        {
            match key.code {
                KeyCode::Enter => {
                    let command = buffer.clone();
                    self.exit_command_mode_internal();

                    if !command.is_empty() {
                        self.save_command_to_history(&command);
                        self.execute_cli_command(&command).await?;
                    }
                }

                KeyCode::Esc => {
                    self.exit_command_mode_internal();
                }

                KeyCode::Backspace => {
                    if *cursor > 0 {
                        buffer.remove(*cursor - 1);
                        *cursor -= 1;
                    }
                }

                KeyCode::Delete => {
                    if *cursor < buffer.len() {
                        buffer.remove(*cursor);
                    }
                }

                KeyCode::Left => {
                    if *cursor > 0 {
                        *cursor -= 1;
                    }
                }

                KeyCode::Right => {
                    if *cursor < buffer.len() {
                        *cursor += 1;
                    }
                }

                KeyCode::Up => {
                    if let Some(index) = history_index {
                        if *index > 0 {
                            *index -= 1;
                            *buffer = history[*index].clone();
                            *cursor = buffer.len();
                        }
                    } else if !history.is_empty() {
                        *history_index = Some(history.len() - 1);
                        *buffer = history[history.len() - 1].clone();
                        *cursor = buffer.len();
                    }
                }

                KeyCode::Down => {
                    if let Some(index) = history_index {
                        if *index < history.len() - 1 {
                            *index += 1;
                            *buffer = history[*index].clone();
                            *cursor = buffer.len();
                        } else {
                            *history_index = None;
                            buffer.clear();
                            *cursor = 0;
                        }
                    }
                }

                KeyCode::Char(c) => {
                    buffer.insert(*cursor, c);
                    *cursor += 1;
                }

                _ => {}
            }
        }

        Ok(())
    }

    async fn execute_cli_command(&mut self, command_str: &str) -> Result<()> {
        self.show_status_message(format!("Executing: {}", command_str), MessageLevel::Info);

        // For now, just show a success message
        // In a real implementation, this would parse and execute the command
        tokio::time::sleep(Duration::from_millis(100)).await;
        self.show_status_message(
            format!("Command '{}' executed successfully", command_str),
            MessageLevel::Success,
        );

        Ok(())
    }

    async fn refresh_dataflow_list(&mut self) -> Result<()> {
        let result = tokio::task::spawn_blocking(fetch_dataflows).await;

        match result {
            Ok(Ok(dataflows)) => {
                self.state.dataflows = dataflows;
                self.state.last_error = None;
            }
            Ok(Err(err)) => {
                let message = format!("failed to fetch dataflows: {err}");
                self.show_status_message(format!("❌ {message}"), MessageLevel::Error);
                self.state.last_error = Some(message);
            }
            Err(err) => {
                let message = format!("coordinator query task failed: {err}");
                self.show_status_message(format!("❌ {message}"), MessageLevel::Error);
                self.state.last_error = Some(message);
            }
        }

        Ok(())
    }

    async fn update_system_metrics(&mut self) -> Result<()> {
        if let Err(err) = self
            .metrics_collector
            .collect()
            .map(|metrics| self.state.system_metrics = metrics)
        {
            self.show_status_message(
                format!("❌ failed to collect system metrics: {err}"),
                MessageLevel::Error,
            );
        }

        Ok(())
    }

    fn load_command_history(&self) -> Vec<String> {
        CommandHistory::load_or_default()
            .commands
            .iter()
            .map(|entry| entry.command.clone())
            .collect()
    }

    fn save_command_to_history(&mut self, command: &str) {
        if command.trim().is_empty() {
            return;
        }

        let working_dir = self
            .cli_context
            .as_ref()
            .map(|ctx| ctx.working_directory.clone())
            .unwrap_or_else(|| std::env::current_dir().unwrap_or_else(|_| PathBuf::from(".")));

        if let Err(err) = CommandHistory::record(command.to_string(), true, working_dir) {
            self.show_status_message(
                format!("❌ failed to persist command history: {err}"),
                MessageLevel::Error,
            );
        }
    }

    pub fn show_status_message(&mut self, message: String, level: MessageLevel) {
        self.state.status_messages.push_back(StatusMessage {
            message,
            level,
            timestamp: Instant::now(),
        });

        // Keep only the last 10 messages
        while self.state.status_messages.len() > 10 {
            self.state.status_messages.pop_front();
        }
    }

    pub fn show_error_message(&mut self, message: String) {
        self.show_status_message(message, MessageLevel::Error);
    }

    // Test helper methods
    #[cfg(test)]
    pub fn current_view(&self) -> &ViewType {
        &self.current_view
    }

    #[cfg(test)]
    pub fn should_quit(&self) -> bool {
        self.should_quit
    }

    #[cfg(test)]
    pub fn view_stack_len(&self) -> usize {
        self.view_stack.len()
    }

    #[cfg(test)]
    pub fn has_status_messages(&self) -> bool {
        !self.state.status_messages.is_empty()
    }

    #[cfg(test)]
    pub fn enter_command_mode(&mut self) {
        self.enter_command_mode_internal();
    }

    #[cfg(test)]
    pub fn exit_command_mode(&mut self) {
        self.exit_command_mode_internal();
    }
}

// Helper function to create a centered rectangle
fn centered_rect(percent_x: u16, percent_y: u16, r: Rect) -> Rect {
    let popup_layout = Layout::default()
        .direction(Direction::Vertical)
        .constraints([
            Constraint::Percentage((100 - percent_y) / 2),
            Constraint::Percentage(percent_y),
            Constraint::Percentage((100 - percent_y) / 2),
        ])
        .split(r);

    Layout::default()
        .direction(Direction::Horizontal)
        .constraints([
            Constraint::Percentage((100 - percent_x) / 2),
            Constraint::Percentage(percent_x),
            Constraint::Percentage((100 - percent_x) / 2),
        ])
        .split(popup_layout[1])[1]
}
