# Issue #029: Build Interactive Debug Session View

## 📋 Summary
Implement a comprehensive interactive debug session interface that provides real-time debugging capabilities for dataflows and nodes, including breakpoint management, variable inspection, call stack analysis, and step-by-step execution control. This view serves as the primary debugging environment for developers to diagnose and resolve complex issues in their Dora applications.

## 🎯 Objectives
- Create interactive debugging interface with comprehensive execution control
- Implement breakpoint management and conditional debugging capabilities
- Add real-time variable inspection and memory analysis
- Provide call stack visualization and execution flow tracking
- Enable collaborative debugging with session sharing and remote access

**Success Metrics:**
- Debug session startup completes within 1 second
- Breakpoint operations complete within 100ms
- Variable inspection handles 10,000+ variables without performance degradation
- Step execution maintains sub-50ms response time
- User debugging efficiency improves by 70% compared to log-based debugging

## 🛠️ Technical Requirements

### What to Build

#### 1. Debug Session View Implementation
```rust
// src/tui/views/debug_session.rs
use super::BaseView;
use crate::tui::{
    components::*,
    theme::Theme,
    AppContext, View, ViewEvent, ViewEventResult, ViewType
};
use async_trait::async_trait;
use ratatui::{
    layout::{Constraint, Direction, Layout, Rect},
    style::{Color, Modifier, Style},
    widgets::{Block, Borders, Clear, Tabs},
    Frame,
};
use std::collections::HashMap;
use tokio::time::{Duration, Instant};

pub struct DebugSessionView {
    base: BaseView,
    debug_context: DebugSessionContext,
    debug_state: DebugSessionState,
    session_manager: DebugSessionManager,
    breakpoint_manager: BreakpointManager,
    variable_inspector: VariableInspector,
    execution_controller: ExecutionController,
    call_stack_analyzer: CallStackAnalyzer,
    last_update: Instant,
}

#[derive(Debug, Clone)]
pub struct DebugSessionContext {
    pub session_id: String,
    pub target: DebugTarget,
    pub debug_mode: DebugMode,
    pub session_config: DebugSessionConfig,
}

#[derive(Debug, Clone)]
pub enum DebugTarget {
    Dataflow { name: String },
    Node { dataflow: String, node: String },
    Pipeline { name: String },
    Process { pid: u32 },
}

#[derive(Debug, Clone)]
pub enum DebugMode {
    Interactive,
    StepByStep,
    Conditional,
    Remote,
    Replay,
}

#[derive(Debug)]
pub struct DebugSessionState {
    pub execution_state: ExecutionState,
    pub current_location: ExecutionLocation,
    pub breakpoints: Vec<Breakpoint>,
    pub variables: VariableScope,
    pub call_stack: CallStack,
    pub debug_output: DebugOutput,
    pub session_stats: SessionStats,
}

#[derive(Debug, Clone)]
pub enum ExecutionState {
    Running,
    Paused,
    Stopped,
    StepInto,
    StepOver,
    StepOut,
    WaitingForBreakpoint,
    Error { error: String },
}

impl DebugSessionView {
    pub fn new(session_id: String, target: DebugTarget, debug_mode: DebugMode) -> Self {
        let mut base = BaseView::new();
        
        let debug_context = DebugSessionContext {
            session_id: session_id.clone(),
            target: target.clone(),
            debug_mode: debug_mode.clone(),
            session_config: DebugSessionConfig::default(),
        };
        
        // Configure layout for debug session
        let layout = LayoutConfig::complex_grid(vec![
            LayoutRow {
                height: Constraint::Length(4),
                columns: vec![
                    LayoutColumn {
                        width: Constraint::Percentage(100),
                        component_id: ComponentId("debug_controls".to_string()),
                    },
                ],
            },
            LayoutRow {
                height: Constraint::Min(20),
                columns: vec![
                    LayoutColumn {
                        width: Constraint::Percentage(60),
                        component_id: ComponentId("execution_view".to_string()),
                    },
                    LayoutColumn {
                        width: Constraint::Percentage(40),
                        component_id: ComponentId("debug_panels".to_string()),
                    },
                ],
            },
            LayoutRow {
                height: Constraint::Length(8),
                columns: vec![
                    LayoutColumn {
                        width: Constraint::Percentage(50),
                        component_id: ComponentId("debug_output".to_string()),
                    },
                    LayoutColumn {
                        width: Constraint::Percentage(50),
                        component_id: ComponentId("session_stats".to_string()),
                    },
                ],
            },
        ]);
        
        base.set_layout(layout);
        
        // Add debug session components
        base.add_component(
            ComponentId("debug_controls".to_string()),
            DebugControlsComponent::new(debug_context.clone()),
        );
        
        base.add_component(
            ComponentId("execution_view".to_string()),
            ExecutionViewComponent::new(debug_context.clone()),
        );
        
        base.add_component(
            ComponentId("debug_panels".to_string()),
            DebugPanelsComponent::new(debug_context.clone()),
        );
        
        base.add_component(
            ComponentId("debug_output".to_string()),
            DebugOutputComponent::new(debug_context.clone()),
        );
        
        base.add_component(
            ComponentId("session_stats".to_string()),
            SessionStatsComponent::new(debug_context.clone()),
        );
        
        Self {
            base,
            debug_context,
            debug_state: DebugSessionState::new(),
            session_manager: DebugSessionManager::new(),
            breakpoint_manager: BreakpointManager::new(),
            variable_inspector: VariableInspector::new(),
            execution_controller: ExecutionController::new(),
            call_stack_analyzer: CallStackAnalyzer::new(),
            last_update: Instant::now(),
        }
    }
    
    async fn start_debug_session(&mut self) -> io::Result<()> {
        // Initialize debug session with daemon
        if let Ok(daemon_client) = DaemonClient::connect().await {
            let session_result = daemon_client.start_debug_session(
                &self.debug_context.target,
                &self.debug_context.session_config
            ).await?;
            
            self.debug_state.execution_state = ExecutionState::Paused;
            self.debug_state.current_location = session_result.initial_location;
            
            // Initialize variable inspector
            self.variable_inspector.initialize(&self.debug_context).await?;
            
            // Setup initial breakpoints if any
            for breakpoint in &self.debug_state.breakpoints {
                self.breakpoint_manager.set_breakpoint(breakpoint).await?;
            }
        }
        
        Ok(())
    }
    
    async fn update_debug_state(&mut self) -> io::Result<()> {
        if let Ok(daemon_client) = DaemonClient::connect().await {
            // Get current execution state
            if let Ok(exec_state) = daemon_client.get_execution_state(&self.debug_context.session_id).await {
                self.debug_state.execution_state = exec_state.state;
                self.debug_state.current_location = exec_state.location;
            }
            
            // Update call stack
            if let Ok(call_stack) = daemon_client.get_call_stack(&self.debug_context.session_id).await {
                self.debug_state.call_stack = call_stack;
            }
            
            // Update variables if paused
            if matches!(self.debug_state.execution_state, ExecutionState::Paused) {
                if let Ok(variables) = daemon_client.get_variables(&self.debug_context.session_id).await {
                    self.debug_state.variables = variables;
                }
            }
            
            // Get debug output
            if let Ok(output) = daemon_client.get_debug_output(&self.debug_context.session_id).await {
                self.debug_state.debug_output.append(output);
            }
        }
        
        Ok(())
    }
    
    async fn execute_debug_command(&mut self, command: DebugCommand) -> io::Result<DebugCommandResult> {
        if let Ok(daemon_client) = DaemonClient::connect().await {
            let result = match command {
                DebugCommand::Continue => {
                    self.debug_state.execution_state = ExecutionState::Running;
                    daemon_client.debug_continue(&self.debug_context.session_id).await?
                },
                
                DebugCommand::StepInto => {
                    self.debug_state.execution_state = ExecutionState::StepInto;
                    daemon_client.debug_step_into(&self.debug_context.session_id).await?
                },
                
                DebugCommand::StepOver => {
                    self.debug_state.execution_state = ExecutionState::StepOver;
                    daemon_client.debug_step_over(&self.debug_context.session_id).await?
                },
                
                DebugCommand::StepOut => {
                    self.debug_state.execution_state = ExecutionState::StepOut;
                    daemon_client.debug_step_out(&self.debug_context.session_id).await?
                },
                
                DebugCommand::Pause => {
                    self.debug_state.execution_state = ExecutionState::Paused;
                    daemon_client.debug_pause(&self.debug_context.session_id).await?
                },
                
                DebugCommand::Stop => {
                    self.debug_state.execution_state = ExecutionState::Stopped;
                    daemon_client.debug_stop(&self.debug_context.session_id).await?
                },
                
                DebugCommand::SetBreakpoint { location, condition } => {
                    self.set_breakpoint(location, condition).await?
                },
                
                DebugCommand::RemoveBreakpoint { breakpoint_id } => {
                    self.remove_breakpoint(breakpoint_id).await?
                },
                
                DebugCommand::EvaluateExpression { expression } => {
                    daemon_client.debug_evaluate(&self.debug_context.session_id, &expression).await?
                },
            };
            
            Ok(result)
        } else {
            Err(io::Error::new(io::ErrorKind::NotConnected, "Debug daemon not available"))
        }
    }
    
    async fn set_breakpoint(&mut self, location: BreakpointLocation, condition: Option<String>) -> io::Result<DebugCommandResult> {
        let breakpoint = Breakpoint {
            id: uuid::Uuid::new_v4().to_string(),
            location: location.clone(),
            condition,
            enabled: true,
            hit_count: 0,
            created_at: chrono::Utc::now(),
        };
        
        self.breakpoint_manager.set_breakpoint(&breakpoint).await?;
        self.debug_state.breakpoints.push(breakpoint);
        
        Ok(DebugCommandResult::BreakpointSet { location })
    }
    
    async fn remove_breakpoint(&mut self, breakpoint_id: String) -> io::Result<DebugCommandResult> {
        self.breakpoint_manager.remove_breakpoint(&breakpoint_id).await?;
        self.debug_state.breakpoints.retain(|bp| bp.id != breakpoint_id);
        
        Ok(DebugCommandResult::BreakpointRemoved { breakpoint_id })
    }
}

#[async_trait]
impl View for DebugSessionView {
    async fn initialize(&mut self, context: &AppContext) -> io::Result<()> {
        // Start debug session
        self.start_debug_session().await?;
        
        // Initialize all components
        for component in self.base.component_registry.components_mut() {
            component.update(context).await?;
        }
        
        Ok(())
    }
    
    async fn update(&mut self, context: &mut AppContext) -> io::Result<()> {
        // Update debug state
        self.update_debug_state().await?;
        
        // Update all components
        for component in self.base.component_registry.components_mut() {
            component.update(context).await?;
        }
        
        self.last_update = Instant::now();
        Ok(())
    }
    
    fn render(&self, frame: &mut Frame, area: Rect, theme: &Theme, context: &AppContext) {
        // Render main debug session layout
        self.base.render_components(frame, area, theme, context);
        
        // Render debug session overlays
        if matches!(self.debug_state.execution_state, ExecutionState::Error { .. }) {
            self.render_error_overlay(frame, area, theme);
        }
        
        if self.debug_context.debug_mode == DebugMode::Remote {
            self.render_remote_session_indicator(frame, area, theme);
        }
    }
    
    async fn handle_event(&mut self, event: ViewEvent, context: &mut AppContext) -> io::Result<ViewEventResult> {
        if let ViewEvent::Key(key_event) = &event {
            match key_event.code {
                crossterm::event::KeyCode::F5 => {
                    // Continue execution
                    self.execute_debug_command(DebugCommand::Continue).await?;
                    return Ok(ViewEventResult::None);
                },
                
                crossterm::event::KeyCode::F10 => {
                    // Step over
                    self.execute_debug_command(DebugCommand::StepOver).await?;
                    return Ok(ViewEventResult::None);
                },
                
                crossterm::event::KeyCode::F11 => {
                    // Step into
                    self.execute_debug_command(DebugCommand::StepInto).await?;
                    return Ok(ViewEventResult::None);
                },
                
                crossterm::event::KeyCode::F12 => {
                    // Step out
                    self.execute_debug_command(DebugCommand::StepOut).await?;
                    return Ok(ViewEventResult::None);
                },
                
                crossterm::event::KeyCode::Char('p') => {
                    // Pause execution
                    self.execute_debug_command(DebugCommand::Pause).await?;
                    return Ok(ViewEventResult::None);
                },
                
                crossterm::event::KeyCode::Char('s') => {
                    // Stop debugging
                    self.execute_debug_command(DebugCommand::Stop).await?;
                    return Ok(ViewEventResult::PopView);
                },
                
                crossterm::event::KeyCode::Char('b') => {
                    // Toggle breakpoint at current location
                    let location = self.debug_state.current_location.clone();
                    self.execute_debug_command(DebugCommand::SetBreakpoint {
                        location: BreakpointLocation::from_execution_location(location),
                        condition: None,
                    }).await?;
                    return Ok(ViewEventResult::None);
                },
                
                crossterm::event::KeyCode::Char('B') => {
                    // Set conditional breakpoint
                    return Ok(ViewEventResult::PushView(ViewType::BreakpointDialog {
                        location: self.debug_state.current_location.clone(),
                        session_id: self.debug_context.session_id.clone(),
                    }));
                },
                
                crossterm::event::KeyCode::Char('v') => {
                    // Open variable inspector
                    return Ok(ViewEventResult::PushView(ViewType::VariableInspector {
                        session_id: self.debug_context.session_id.clone(),
                        scope: self.debug_state.variables.current_scope().clone(),
                    }));
                },
                
                crossterm::event::KeyCode::Char('w') => {
                    // Open watch window
                    return Ok(ViewEventResult::PushView(ViewType::WatchWindow {
                        session_id: self.debug_context.session_id.clone(),
                    }));
                },
                
                crossterm::event::KeyCode::Char('c') => {
                    // Open call stack viewer
                    return Ok(ViewEventResult::PushView(ViewType::CallStackViewer {
                        session_id: self.debug_context.session_id.clone(),
                        call_stack: self.debug_state.call_stack.clone(),
                    }));
                },
                
                crossterm::event::KeyCode::Char('e') => {
                    // Evaluate expression
                    return Ok(ViewEventResult::PushView(ViewType::ExpressionEvaluator {
                        session_id: self.debug_context.session_id.clone(),
                        context: self.debug_state.variables.current_scope().clone(),
                    }));
                },
                
                _ => {}
            }
        }
        
        // Forward event to components
        self.base.handle_component_event(event, context).await
    }
    
    async fn handle_resize(&mut self, width: u16, height: u16) -> io::Result<()> {
        self.base.layout_manager.handle_resize(width, height);
        Ok(())
    }
    
    async fn cleanup(&self, context: &AppContext) -> io::Result<()> {
        // Stop debug session
        if let Ok(daemon_client) = DaemonClient::connect().await {
            let _ = daemon_client.stop_debug_session(&self.debug_context.session_id).await;
        }
        
        Ok(())
    }
    
    fn view_type(&self) -> ViewType {
        ViewType::DebugSession {
            session_id: self.debug_context.session_id.clone(),
            debug_mode: self.debug_context.debug_mode.clone(),
        }
    }
    
    fn title(&self) -> String {
        format!("Debug Session: {} [{}]", 
                self.debug_context.target.display_name(),
                self.debug_state.execution_state.display_name())
    }
    
    fn help_text(&self) -> Vec<HelpItem> {
        vec![
            HelpItem::new("F5", "Continue"),
            HelpItem::new("F10", "Step over"),
            HelpItem::new("F11", "Step into"),
            HelpItem::new("F12", "Step out"),
            HelpItem::new("p", "Pause"),
            HelpItem::new("s", "Stop"),
            HelpItem::new("b", "Toggle breakpoint"),
            HelpItem::new("B", "Conditional breakpoint"),
            HelpItem::new("v", "Variables"),
            HelpItem::new("w", "Watch"),
            HelpItem::new("c", "Call stack"),
            HelpItem::new("e", "Evaluate"),
            HelpItem::new("Esc", "Exit debug"),
        ]
    }
}
```

#### 2. Execution View Component
```rust
// src/tui/components/debug/execution_view.rs
pub struct ExecutionViewComponent {
    debug_context: DebugSessionContext,
    focused: bool,
    code_viewer: CodeViewer,
    execution_pointer: ExecutionPointer,
    breakpoint_markers: BreakpointMarkers,
    scroll_state: ScrollState,
}

impl ExecutionViewComponent {
    pub fn new(debug_context: DebugSessionContext) -> Self {
        Self {
            debug_context,
            focused: true,
            code_viewer: CodeViewer::new(),
            execution_pointer: ExecutionPointer::new(),
            breakpoint_markers: BreakpointMarkers::new(),
            scroll_state: ScrollState::new(),
        }
    }
    
    fn render_code_with_debugging_info(&self, frame: &mut Frame, area: Rect, theme: &Theme, context: &AppContext) {
        if let Some(debug_state) = context.get_component_state::<DebugSessionState>("debug_session") {
            // Get source code for current location
            if let Some(source_code) = self.code_viewer.get_source_code(&debug_state.current_location) {
                let lines: Vec<&str> = source_code.lines().collect();
                
                let items: Vec<ListItem> = lines.iter()
                    .enumerate()
                    .map(|(line_number, line_content)| {
                        let line_num = line_number + 1;
                        let is_current_line = line_num == debug_state.current_location.line;
                        let has_breakpoint = debug_state.breakpoints.iter()
                            .any(|bp| bp.location.line == line_num);
                        
                        let mut style = theme.normal_style();
                        let mut prefix = format!("{:4} ", line_num);
                        
                        // Add breakpoint marker
                        if has_breakpoint {
                            prefix = format!("● {:3} ", line_num);
                            style = style.fg(Color::Red);
                        }
                        
                        // Highlight current execution line
                        if is_current_line {
                            prefix = format!("▶{}", &prefix[1..]);
                            style = style.bg(Color::DarkGray).add_modifier(Modifier::BOLD);
                        }
                        
                        let content = format!("{}{}", prefix, line_content);
                        ListItem::new(content).style(style)
                    })
                    .collect();
                
                let list = List::new(items)
                    .highlight_style(theme.highlight_style());
                
                frame.render_stateful_widget(list, area, &mut self.scroll_state.to_list_state());
            }
        }
    }
    
    fn render_execution_context(&self, frame: &mut Frame, area: Rect, theme: &Theme, context: &AppContext) {
        if let Some(debug_state) = context.get_component_state::<DebugSessionState>("debug_session") {
            let context_info = vec![
                format!("File: {}", debug_state.current_location.file),
                format!("Function: {}", debug_state.current_location.function),
                format!("Line: {}", debug_state.current_location.line),
                format!("Column: {}", debug_state.current_location.column),
            ];
            
            let paragraph = Paragraph::new(context_info.join(" | "))
                .style(theme.info_style())
                .wrap(ratatui::widgets::Wrap { trim: true });
            
            frame.render_widget(paragraph, area);
        }
    }
}

#[async_trait]
impl Component for ExecutionViewComponent {
    async fn update(&mut self, context: &AppContext) -> io::Result<()> {
        // Update code viewer with latest execution location
        if let Some(debug_state) = context.get_component_state::<DebugSessionState>("debug_session") {
            self.code_viewer.update_location(&debug_state.current_location).await?;
            
            // Auto-scroll to current execution line
            self.scroll_state.scroll_to_line(debug_state.current_location.line);
        }
        
        Ok(())
    }
    
    fn render(&self, frame: &mut Frame, area: Rect, theme: &Theme, context: &AppContext) {
        let block = Block::default()
            .title("Execution View")
            .borders(Borders::ALL)
            .border_style(if self.focused {
                theme.focused_border_style()
            } else {
                theme.normal_border_style()
            });
        
        let inner_area = block.inner(area);
        frame.render_widget(block, area);
        
        // Split area for context and code
        let chunks = Layout::default()
            .direction(Direction::Vertical)
            .constraints([
                Constraint::Length(2),
                Constraint::Min(10),
            ].as_ref())
            .split(inner_area);
        
        self.render_execution_context(frame, chunks[0], theme, context);
        self.render_code_with_debugging_info(frame, chunks[1], theme, context);
    }
    
    async fn handle_event(&mut self, event: ComponentEvent, context: &AppContext) -> io::Result<ViewEventResult> {
        if !self.focused {
            return Ok(ViewEventResult::None);
        }
        
        match event {
            ComponentEvent::Key(key_event) => {
                match key_event.code {
                    crossterm::event::KeyCode::Enter => {
                        // Set breakpoint at current line
                        if let Some(debug_state) = context.get_component_state::<DebugSessionState>("debug_session") {
                            let current_line = self.scroll_state.selected_line();
                            return Ok(ViewEventResult::Custom(CustomEvent::SetBreakpoint {
                                location: BreakpointLocation {
                                    file: debug_state.current_location.file.clone(),
                                    line: current_line,
                                    function: None,
                                },
                            }));
                        }
                    },
                    
                    crossterm::event::KeyCode::Char('g') => {
                        // Go to line dialog
                        return Ok(ViewEventResult::PushView(ViewType::GoToLineDialog));
                    },
                    
                    _ => {}
                }
            },
            _ => {}
        }
        
        Ok(ViewEventResult::None)
    }
    
    fn component_type(&self) -> ComponentType {
        ComponentType::ExecutionView
    }
    
    fn is_focusable(&self) -> bool {
        true
    }
    
    fn is_focused(&self) -> bool {
        self.focused
    }
    
    fn set_focus(&mut self, focused: bool) {
        self.focused = focused;
    }
}
```

#### 3. Debug Panels Component
```rust
// src/tui/components/debug/debug_panels.rs
pub struct DebugPanelsComponent {
    debug_context: DebugSessionContext,
    focused: bool,
    current_panel: usize,
    panels: DebugPanels,
}

#[derive(Debug)]
pub struct DebugPanels {
    variable_panel: VariablePanel,
    call_stack_panel: CallStackPanel,
    breakpoint_panel: BreakpointPanel,
    watch_panel: WatchPanel,
}

impl DebugPanelsComponent {
    pub fn new(debug_context: DebugSessionContext) -> Self {
        Self {
            debug_context,
            focused: false,
            current_panel: 0,
            panels: DebugPanels {
                variable_panel: VariablePanel::new(),
                call_stack_panel: CallStackPanel::new(),
                breakpoint_panel: BreakpointPanel::new(),
                watch_panel: WatchPanel::new(),
            },
        }
    }
    
    fn render_variable_panel(&self, frame: &mut Frame, area: Rect, theme: &Theme, context: &AppContext) {
        if let Some(debug_state) = context.get_component_state::<DebugSessionState>("debug_session") {
            let variables = &debug_state.variables;
            
            let items: Vec<ListItem> = variables.get_current_scope_variables()
                .iter()
                .map(|var| {
                    let display_text = format!(
                        "{}: {} = {}",
                        var.name,
                        var.var_type,
                        var.value.truncate(50)
                    );
                    
                    let style = match var.value_change {
                        Some(VariableChange::Modified) => theme.warning_style(),
                        Some(VariableChange::New) => theme.success_style(),
                        None => theme.normal_style(),
                    };
                    
                    ListItem::new(display_text).style(style)
                })
                .collect();
            
            let list = List::new(items)
                .block(Block::default().title("Variables").borders(Borders::ALL))
                .highlight_style(theme.highlight_style());
            
            frame.render_widget(list, area);
        }
    }
    
    fn render_call_stack_panel(&self, frame: &mut Frame, area: Rect, theme: &Theme, context: &AppContext) {
        if let Some(debug_state) = context.get_component_state::<DebugSessionState>("debug_session") {
            let call_stack = &debug_state.call_stack;
            
            let items: Vec<ListItem> = call_stack.frames.iter()
                .enumerate()
                .map(|(index, frame)| {
                    let is_current = index == 0;
                    let display_text = format!(
                        "{} {}:{}",
                        frame.function_name,
                        frame.file_name,
                        frame.line_number
                    );
                    
                    let style = if is_current {
                        theme.selected_item_style()
                    } else {
                        theme.normal_style()
                    };
                    
                    ListItem::new(display_text).style(style)
                })
                .collect();
            
            let list = List::new(items)
                .block(Block::default().title("Call Stack").borders(Borders::ALL))
                .highlight_style(theme.highlight_style());
            
            frame.render_widget(list, area);
        }
    }
    
    fn render_breakpoint_panel(&self, frame: &mut Frame, area: Rect, theme: &Theme, context: &AppContext) {
        if let Some(debug_state) = context.get_component_state::<DebugSessionState>("debug_session") {
            let breakpoints = &debug_state.breakpoints;
            
            let items: Vec<ListItem> = breakpoints.iter()
                .map(|bp| {
                    let status_icon = if bp.enabled { "●" } else { "○" };
                    let condition_text = bp.condition.as_ref()
                        .map(|c| format!(" [{}]", c))
                        .unwrap_or_default();
                    
                    let display_text = format!(
                        "{} {}:{} (hits: {}){}",
                        status_icon,
                        bp.location.file,
                        bp.location.line,
                        bp.hit_count,
                        condition_text
                    );
                    
                    let style = if bp.enabled {
                        theme.normal_style()
                    } else {
                        theme.muted_style()
                    };
                    
                    ListItem::new(display_text).style(style)
                })
                .collect();
            
            let list = List::new(items)
                .block(Block::default().title("Breakpoints").borders(Borders::ALL))
                .highlight_style(theme.highlight_style());
            
            frame.render_widget(list, area);
        }
    }
    
    fn render_watch_panel(&self, frame: &mut Frame, area: Rect, theme: &Theme, context: &AppContext) {
        // Render watch expressions
        let watch_items = vec![
            ListItem::new("No watch expressions").style(theme.muted_style()),
        ];
        
        let list = List::new(watch_items)
            .block(Block::default().title("Watch").borders(Borders::ALL))
            .highlight_style(theme.highlight_style());
        
        frame.render_widget(list, area);
    }
}

#[async_trait]
impl Component for DebugPanelsComponent {
    async fn update(&mut self, context: &AppContext) -> io::Result<()> {
        // Panels update their data from the debug session state
        Ok(())
    }
    
    fn render(&self, frame: &mut Frame, area: Rect, theme: &Theme, context: &AppContext) {
        // Create tabs for different panels
        let tab_titles = vec!["Variables", "Call Stack", "Breakpoints", "Watch"];
        let tabs = Tabs::new(tab_titles)
            .block(Block::default().borders(Borders::ALL))
            .style(theme.normal_style())
            .highlight_style(theme.selected_item_style())
            .select(self.current_panel);
        
        let chunks = Layout::default()
            .direction(Direction::Vertical)
            .constraints([
                Constraint::Length(3),
                Constraint::Min(0),
            ].as_ref())
            .split(area);
        
        frame.render_widget(tabs, chunks[0]);
        
        // Render current panel content
        match self.current_panel {
            0 => self.render_variable_panel(frame, chunks[1], theme, context),
            1 => self.render_call_stack_panel(frame, chunks[1], theme, context),
            2 => self.render_breakpoint_panel(frame, chunks[1], theme, context),
            3 => self.render_watch_panel(frame, chunks[1], theme, context),
            _ => {}
        }
    }
    
    async fn handle_event(&mut self, event: ComponentEvent, context: &AppContext) -> io::Result<ViewEventResult> {
        if !self.focused {
            return Ok(ViewEventResult::None);
        }
        
        match event {
            ComponentEvent::Key(key_event) => {
                match key_event.code {
                    crossterm::event::KeyCode::Tab => {
                        self.current_panel = (self.current_panel + 1) % 4;
                    },
                    
                    crossterm::event::KeyCode::BackTab => {
                        self.current_panel = if self.current_panel == 0 { 3 } else { self.current_panel - 1 };
                    },
                    
                    crossterm::event::KeyCode::Delete => {
                        if self.current_panel == 2 { // Breakpoints panel
                            // Delete selected breakpoint
                            return Ok(ViewEventResult::Custom(CustomEvent::DeleteBreakpoint));
                        }
                    },
                    
                    _ => {}
                }
            },
            _ => {}
        }
        
        Ok(ViewEventResult::None)
    }
    
    fn component_type(&self) -> ComponentType {
        ComponentType::DebugPanels
    }
    
    fn is_focusable(&self) -> bool {
        true
    }
    
    fn is_focused(&self) -> bool {
        self.focused
    }
    
    fn set_focus(&mut self, focused: bool) {
        self.focused = focused;
    }
}
```

### Why This Approach

**Comprehensive Debugging Environment:**
- Full-featured debugging interface with all essential tools
- Real-time execution control and state inspection
- Professional IDE-like debugging experience

**Developer-Centric Design:**
- Familiar debugging metaphors and keyboard shortcuts
- Multi-panel layout for efficient information access
- Context-aware debugging information display

**High Performance:**
- Efficient state updates and rendering
- Responsive debugging controls
- Optimized for complex debugging scenarios

### How to Implement

#### Step 1: Debug Session Core (6 hours)
1. **Implement DebugSessionView** with comprehensive session management
2. **Add execution control** and command processing
3. **Create debug state** management and synchronization
4. **Add session lifecycle** management (start, pause, stop)

#### Step 2: Execution View and Code Display (4 hours)
1. **Implement ExecutionViewComponent** with syntax highlighting
2. **Add breakpoint visualization** and management
3. **Create execution pointer** and current line highlighting
4. **Add code navigation** and goto line functionality

#### Step 3: Debug Panels and Information Display (5 hours)
1. **Implement DebugPanelsComponent** with tabbed interface
2. **Add variable inspection** with type and value display
3. **Create call stack visualization** with frame navigation
4. **Add breakpoint management** panel with conditions

#### Step 4: Advanced Debugging Features (4 hours)
1. **Implement variable inspector** with deep object inspection
2. **Add watch expressions** and expression evaluation
3. **Create conditional breakpoints** and hit count tracking
4. **Add memory analysis** and performance profiling integration

#### Step 5: Integration and Testing (3 hours)
1. **Add comprehensive unit tests** for all debug components
2. **Test debugging workflow** with real dataflows
3. **Validate performance** of debug operations
4. **Test session management** and cleanup

## 🔗 Dependencies
**Depends On:**
- Issue #023 (TUI Architecture Foundation) - Base view and component system
- Issue #027 (Node Inspector View) - Node debugging integration
- Phase 2 enhanced debug commands for backend integration

**Enables:**
- Advanced interactive debugging capabilities
- Integration with performance analysis and log correlation

## 🧪 Testing Requirements

### Unit Tests
```rust
#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_debug_session_initialization() {
        let session = DebugSessionView::new(
            "test-session".to_string(),
            DebugTarget::Node {
                dataflow: "test-df".to_string(),
                node: "test-node".to_string(),
            },
            DebugMode::Interactive,
        );
        
        assert_eq!(session.debug_context.session_id, "test-session");
        assert!(matches!(session.debug_state.execution_state, ExecutionState::Stopped));
    }
    
    #[test]
    fn test_breakpoint_management() {
        let mut manager = BreakpointManager::new();
        let breakpoint = Breakpoint::test_breakpoint();
        
        manager.set_breakpoint(&breakpoint).await.unwrap();
        assert!(manager.has_breakpoint(&breakpoint.location));
        
        manager.remove_breakpoint(&breakpoint.id).await.unwrap();
        assert!(!manager.has_breakpoint(&breakpoint.location));
    }
    
    #[test]
    fn test_execution_control_commands() {
        let mut controller = ExecutionController::new();
        
        let result = controller.execute_command(DebugCommand::StepInto).await.unwrap();
        assert!(matches!(result, DebugCommandResult::StepCompleted { .. }));
        
        let result = controller.execute_command(DebugCommand::Continue).await.unwrap();
        assert!(matches!(result, DebugCommandResult::Continued));
    }
}
```

## ✅ Definition of Done
- [ ] DebugSessionView provides comprehensive interactive debugging environment
- [ ] Execution control commands respond within target performance times
- [ ] Breakpoint management supports conditional and hit count breakpoints
- [ ] Variable inspection displays complex data structures clearly
- [ ] Call stack visualization enables frame navigation and inspection
- [ ] Code viewer displays source with debugging annotations
- [ ] Debug panels provide tabbed access to all debugging information
- [ ] Session management handles start, pause, resume, and stop operations
- [ ] Performance targets met for all debugging operations
- [ ] Memory usage stays within limits during extended debug sessions
- [ ] Comprehensive unit tests validate all debug session functionality
- [ ] Integration tests confirm debugging workflow effectiveness
- [ ] Manual testing validates developer debugging experience

This interactive debug session view provides developers with a professional-grade debugging environment, enabling efficient diagnosis and resolution of complex issues in Dora dataflows and applications.