# Issue #009: Create TUI Launcher Framework

## 📋 Summary
Implement the core TUI application framework that provides seamless launching, view management, and CLI integration for the hybrid architecture. This framework serves as the foundation for all TUI interfaces and ensures consistent user experience across different TUI modes.

## 🎯 Objectives
- Create robust TUI application framework with Ratatui
- Implement view management and navigation system
- Add seamless CLI-to-TUI and TUI-to-CLI transitions
- Provide consistent theming and styling across all TUI views
- Enable context-aware TUI launching from any CLI command

**Success Metrics:**
- TUI launches in <500ms from any CLI command
- View transitions are smooth and intuitive
- CLI commands work seamlessly within TUI command mode
- Memory usage stays under 50MB for typical TUI operations
- TUI gracefully handles terminal resize and various terminal types

## 🛠️ Technical Requirements

### What to Build

#### 1. Core TUI Application Framework
```rust
// src/tui/app.rs
use ratatui::{
    backend::{Backend, CrosstermBackend},
    layout::{Constraint, Direction, Layout, Rect},
    style::{Color, Modifier, Style},
    terminal::{Frame, Terminal},
    widgets::*,
};
use crossterm::{
    event::{self, DisableMouseCapture, EnableMouseCapture, Event, KeyCode, KeyEventKind},
    execute,
    terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen},
};

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
    
    /// Command mode state
    command_mode: CommandMode,
    
    /// Should quit flag
    should_quit: bool,
}

#[derive(Debug, Clone)]
pub enum ViewType {
    Dashboard,
    DataflowManager,
    NodeInspector { node_id: String },
    SystemMonitor,
    LogViewer { target: String },
    RecordingAnalyzer { recording_id: String },
    DebugSession { dataflow_id: String },
    SettingsManager,
    Help,
}

#[derive(Debug)]
pub struct AppState {
    /// Global data cache
    data_cache: DataCache,
    
    /// Current dataflows
    dataflows: Vec<DataflowInfo>,
    
    /// System metrics
    system_metrics: SystemMetrics,
    
    /// User preferences
    user_config: UserConfig,
    
    /// Last error message
    last_error: Option<String>,
    
    /// Status messages
    status_messages: VecDeque<StatusMessage>,
}

impl DoraApp {
    pub fn new(initial_view: ViewType) -> Self {
        Self {
            current_view: initial_view,
            view_stack: Vec::new(),
            state: AppState::new(),
            event_handler: EventHandler::new(),
            theme: ThemeConfig::load_user_theme(),
            cli_context: None,
            command_mode: CommandMode::Normal,
            should_quit: false,
        }
    }
    
    pub fn new_with_context(initial_view: ViewType, cli_context: CliContext) -> Self {
        let mut app = Self::new(initial_view);
        app.cli_context = Some(cli_context);
        app
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
    
    async fn run_event_loop<B: Backend>(&mut self, terminal: &mut Terminal<B>) -> Result<()> {
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
    
    fn ui<B: Backend>(&mut self, f: &mut Frame<B>) {
        let size = f.size();
        
        // Main layout: header + body + footer
        let chunks = Layout::default()
            .direction(Direction::Vertical)
            .constraints([
                Constraint::Length(3),  // Header
                Constraint::Min(0),     // Body
                Constraint::Length(3),  // Footer/Status
            ])
            .split(size);
        
        self.render_header(f, chunks[0]);
        self.render_current_view(f, chunks[1]);
        self.render_footer(f, chunks[2]);
        
        // Render overlays (command mode, help, etc.)
        self.render_overlays(f, size);
    }
}
```

#### 2. View Management System
```rust
// src/tui/views/mod.rs
pub trait View {
    /// Render the view content
    fn render<B: Backend>(&mut self, f: &mut Frame<B>, area: Rect, app_state: &AppState);
    
    /// Handle key events
    async fn handle_key(&mut self, key: KeyEvent, app_state: &mut AppState) -> Result<ViewAction>;
    
    /// Update view state (called periodically)
    async fn update(&mut self, app_state: &mut AppState) -> Result<()>;
    
    /// View-specific help text
    fn help_text(&self) -> Vec<(&str, &str)>;
    
    /// Whether this view should auto-refresh
    fn auto_refresh(&self) -> Option<Duration> {
        None
    }
}

#[derive(Debug)]
pub enum ViewAction {
    None,
    SwitchView(ViewType),
    PushView(ViewType),
    PopView,
    Quit,
    ExecuteCommand(String),
    ShowHelp,
    ShowError(String),
    ShowStatus(String),
}

impl DoraApp {
    fn render_current_view<B: Backend>(&mut self, f: &mut Frame<B>, area: Rect) {
        match &mut self.current_view {
            ViewType::Dashboard => {
                let mut view = DashboardView::new(&self.theme);
                view.render(f, area, &self.state);
            },
            ViewType::DataflowManager => {
                let mut view = DataflowManagerView::new(&self.theme);
                view.render(f, area, &self.state);
            },
            ViewType::NodeInspector { node_id } => {
                let mut view = NodeInspectorView::new(node_id, &self.theme);
                view.render(f, area, &self.state);
            },
            ViewType::SystemMonitor => {
                let mut view = SystemMonitorView::new(&self.theme);
                view.render(f, area, &self.state);
            },
            // ... other views
        }
    }
    
    async fn handle_key_event(&mut self, key: KeyEvent) -> Result<()> {
        // Global key bindings
        match key.code {
            KeyCode::Char('q') if !self.is_in_command_mode() => {
                self.should_quit = true;
                return Ok(());
            },
            KeyCode::Char(':') if !self.is_in_command_mode() => {
                self.enter_command_mode();
                return Ok(());
            },
            KeyCode::F(1) => {
                self.switch_view(ViewType::Help);
                return Ok(());
            },
            KeyCode::Esc => {
                if self.is_in_command_mode() {
                    self.exit_command_mode();
                } else if !self.view_stack.is_empty() {
                    self.pop_view();
                }
                return Ok(());
            },
            _ => {}
        }
        
        // Command mode handling
        if self.is_in_command_mode() {
            self.handle_command_mode_key(key).await?;
            return Ok(());
        }
        
        // View-specific key handling
        let action = self.get_current_view_mut().handle_key(key, &mut self.state).await?;
        self.handle_view_action(action).await?;
        
        Ok(())
    }
    
    async fn handle_view_action(&mut self, action: ViewAction) -> Result<()> {
        match action {
            ViewAction::None => {},
            ViewAction::SwitchView(view_type) => {
                self.switch_view(view_type);
            },
            ViewAction::PushView(view_type) => {
                self.push_view(view_type);
            },
            ViewAction::PopView => {
                self.pop_view();
            },
            ViewAction::Quit => {
                self.should_quit = true;
            },
            ViewAction::ExecuteCommand(command) => {
                self.execute_cli_command(&command).await?;
            },
            ViewAction::ShowHelp => {
                self.push_view(ViewType::Help);
            },
            ViewAction::ShowError(message) => {
                self.show_error_message(message);
            },
            ViewAction::ShowStatus(message) => {
                self.show_status_message(message);
            },
        }
        Ok(())
    }
}
```

#### 3. CLI-TUI Integration
```rust
// src/tui/cli_integration.rs
#[derive(Debug, Clone)]
pub struct CliContext {
    /// Original command that launched TUI
    pub original_command: Option<Command>,
    
    /// Working directory
    pub working_directory: PathBuf,
    
    /// Environment variables
    pub environment: HashMap<String, String>,
    
    /// User preferences
    pub user_config: UserConfig,
}

#[derive(Debug)]
pub enum CommandMode {
    Normal,
    Command {
        buffer: String,
        cursor: usize,
        history: Vec<String>,
        history_index: Option<usize>,
    },
}

impl DoraApp {
    async fn execute_cli_command(&mut self, command_str: &str) -> Result<()> {
        let args = shell_words::split(command_str)?;
        if args.is_empty() {
            return Ok(());
        }
        
        // Parse command
        let cli = match Cli::try_parse_from(std::iter::once("dora").chain(args.iter())) {
            Ok(cli) => cli,
            Err(e) => {
                self.show_error_message(format!("Invalid command: {}", e));
                return Ok(());
            }
        };
        
        // Execute command and update TUI state
        match &cli.command {
            Command::Ps(_) => {
                self.refresh_dataflow_list().await?;
                self.show_status_message("Dataflow list refreshed".to_string());
            },
            
            Command::Start(cmd) => {
                self.execute_start_command_in_tui(cmd).await?;
            },
            
            Command::Stop(cmd) => {
                self.execute_stop_command_in_tui(cmd).await?;
            },
            
            Command::Inspect(cmd) => {
                self.switch_to_inspect_view(&cmd.target);
            },
            
            // View switching commands
            Command::Ui(ui_cmd) => {
                if let Some(view_type) = self.parse_ui_command(ui_cmd) {
                    self.switch_view(view_type);
                }
            },
            
            _ => {
                // For other commands, execute in background and show result
                self.execute_command_background(&cli).await?;
            }
        }
        
        Ok(())
    }
    
    fn enter_command_mode(&mut self) {
        self.command_mode = CommandMode::Command {
            buffer: String::new(),
            cursor: 0,
            history: self.load_command_history(),
            history_index: None,
        };
    }
    
    async fn handle_command_mode_key(&mut self, key: KeyEvent) -> Result<()> {
        if let CommandMode::Command { 
            ref mut buffer, 
            ref mut cursor, 
            ref history, 
            ref mut history_index 
        } = self.command_mode {
            match key.code {
                KeyCode::Enter => {
                    let command = buffer.clone();
                    self.exit_command_mode();
                    
                    if !command.is_empty() {
                        self.save_command_to_history(&command);
                        self.execute_cli_command(&command).await?;
                    }
                },
                
                KeyCode::Esc => {
                    self.exit_command_mode();
                },
                
                KeyCode::Backspace => {
                    if *cursor > 0 {
                        buffer.remove(*cursor - 1);
                        *cursor -= 1;
                    }
                },
                
                KeyCode::Delete => {
                    if *cursor < buffer.len() {
                        buffer.remove(*cursor);
                    }
                },
                
                KeyCode::Left => {
                    if *cursor > 0 {
                        *cursor -= 1;
                    }
                },
                
                KeyCode::Right => {
                    if *cursor < buffer.len() {
                        *cursor += 1;
                    }
                },
                
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
                },
                
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
                },
                
                KeyCode::Char(c) => {
                    buffer.insert(*cursor, c);
                    *cursor += 1;
                },
                
                _ => {}
            }
        }
        
        Ok(())
    }
}
```

#### 4. Theme and Styling System
```rust
// src/tui/theme.rs
#[derive(Debug, Clone)]
pub struct ThemeConfig {
    pub name: String,
    pub colors: ColorScheme,
    pub styles: StyleConfig,
}

#[derive(Debug, Clone)]
pub struct ColorScheme {
    pub primary: Color,
    pub secondary: Color,
    pub success: Color,
    pub warning: Color,
    pub error: Color,
    pub background: Color,
    pub text: Color,
    pub muted: Color,
    pub border: Color,
}

#[derive(Debug, Clone)]
pub struct StyleConfig {
    pub border_style: BorderType,
    pub highlight_style: Style,
    pub selection_style: Style,
    pub status_style: Style,
}

impl ThemeConfig {
    pub fn load_user_theme() -> Self {
        // Load from user config or use default
        Self::default_dark()
    }
    
    pub fn default_dark() -> Self {
        Self {
            name: "dark".to_string(),
            colors: ColorScheme {
                primary: Color::Cyan,
                secondary: Color::Blue,
                success: Color::Green,
                warning: Color::Yellow,
                error: Color::Red,
                background: Color::Black,
                text: Color::White,
                muted: Color::Gray,
                border: Color::DarkGray,
            },
            styles: StyleConfig {
                border_style: BorderType::Rounded,
                highlight_style: Style::default().add_modifier(Modifier::BOLD),
                selection_style: Style::default().bg(Color::DarkGray),
                status_style: Style::default().fg(Color::Cyan),
            },
        }
    }
    
    pub fn styled_block(&self, title: &str) -> Block {
        Block::default()
            .title(title)
            .borders(Borders::ALL)
            .border_type(self.styles.border_style)
            .border_style(Style::default().fg(self.colors.border))
            .title_style(Style::default().fg(self.colors.text).add_modifier(Modifier::BOLD))
    }
    
    pub fn status_style(&self, status: &str) -> Style {
        let color = match status.to_lowercase().as_str() {
            "running" | "active" | "healthy" | "ok" => self.colors.success,
            "warning" | "degraded" | "slow" => self.colors.warning,
            "error" | "failed" | "stopped" | "critical" => self.colors.error,
            _ => self.colors.text,
        };
        Style::default().fg(color)
    }
}
```

### Why This Approach

**Modular Architecture:**
- Clean separation between application framework and views
- Pluggable view system for easy extension
- Reusable components across different TUI interfaces

**CLI Integration:**
- Seamless command execution within TUI
- Command history and tab completion
- Context preservation between CLI and TUI

**User Experience:**
- Consistent theming and styling
- Intuitive navigation and keyboard shortcuts
- Responsive design that adapts to terminal size

### How to Implement

#### Step 1: Core Framework (6 hours)
1. **Implement DoraApp structure** with event loop
2. **Add view management system** with stack navigation
3. **Create terminal setup** and cleanup procedures
4. **Implement basic key handling** and event processing

#### Step 2: CLI Integration (4 hours)
1. **Add command mode** with input handling
2. **Implement CLI command execution** within TUI
3. **Create command history** management
4. **Add context preservation** between CLI and TUI

#### Step 3: Theme System (3 hours)
1. **Implement ThemeConfig** with color schemes
2. **Add consistent styling** utilities
3. **Create theme loading** from user configuration
4. **Add responsive design** helpers

#### Step 4: View Infrastructure (3 hours)
1. **Create View trait** for consistent interface
2. **Implement view action** handling system
3. **Add view switching** and navigation
4. **Create helper functions** for common UI patterns

#### Step 5: Testing and Polish (2 hours)
1. **Add unit tests** for core functionality
2. **Test terminal compatibility** across different terminals
3. **Validate memory usage** and performance
4. **Test CLI integration** scenarios

## 🔗 Dependencies
**Depends On:**
- Issue #001 (Hybrid Command Framework) - CLI command structures
- Issue #002 (Execution Context Detection) - Terminal capabilities
- Issue #004 (Configuration System) - Theme and preference loading

**Blocks:** 
- All TUI view implementations (Issues #010-012, #025-040)
- TUI suggestions in CLI commands (Issues #005-008)

## 🧪 Testing Requirements

### Unit Tests
```rust
#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_view_stack_navigation() {
        let mut app = DoraApp::new(ViewType::Dashboard);
        
        app.push_view(ViewType::DataflowManager);
        app.push_view(ViewType::NodeInspector { node_id: "test".to_string() });
        
        assert_eq!(app.view_stack.len(), 2);
        
        app.pop_view();
        assert!(matches!(app.current_view, ViewType::DataflowManager));
        
        app.pop_view();
        assert!(matches!(app.current_view, ViewType::Dashboard));
    }
    
    #[test]
    fn test_command_mode_input() {
        let mut app = DoraApp::new(ViewType::Dashboard);
        app.enter_command_mode();
        
        // Simulate typing "ps"
        app.handle_command_mode_key(KeyEvent::from(KeyCode::Char('p'))).await.unwrap();
        app.handle_command_mode_key(KeyEvent::from(KeyCode::Char('s'))).await.unwrap();
        
        if let CommandMode::Command { buffer, .. } = &app.command_mode {
            assert_eq!(buffer, "ps");
        } else {
            panic!("Not in command mode");
        }
    }
    
    #[test]
    fn test_theme_loading() {
        let theme = ThemeConfig::load_user_theme();
        assert!(!theme.name.is_empty());
        assert_ne!(theme.colors.primary, Color::Reset);
    }
}
```

### Integration Tests
```rust
// tests/tui_integration.rs
#[tokio::test]
async fn test_tui_cli_command_execution() {
    let mut app = DoraApp::new(ViewType::Dashboard);
    
    // Execute a ps command within TUI
    let result = app.execute_cli_command("ps").await;
    assert!(result.is_ok());
    
    // Verify state was updated
    assert!(!app.state.dataflows.is_empty() || app.state.last_error.is_some());
}

#[test]
fn test_tui_terminal_setup() {
    // Test that TUI can be set up and torn down cleanly
    let app = DoraApp::new(ViewType::Dashboard);
    
    // This would test actual terminal setup in a headless environment
    // Implementation depends on testing infrastructure
}
```

### Manual Testing Procedures
1. **Basic TUI Operation**
   ```bash
   # Test TUI launching
   dora ui
   dora ui dataflow
   dora ui monitor
   ```

2. **CLI Integration**
   ```bash
   # Launch TUI, then test command mode
   dora ui
   # Press ':' to enter command mode
   # Type "ps" and press Enter
   # Verify dataflow list updates
   ```

3. **View Navigation**
   ```bash
   # Test view switching and stack navigation
   dora ui
   # Navigate to different views
   # Test back navigation with Esc
   # Test view stack behavior
   ```

## 📚 Resources

### Ratatui Documentation
- [Ratatui Book](https://ratatui.rs/) - Comprehensive guide
- [Ratatui Examples](https://github.com/ratatui-org/ratatui/tree/main/examples)
- [TUI Design Patterns](https://github.com/ratatui-org/ratatui/blob/main/CONTRIBUTING.md)

### Terminal Handling
- [Crossterm Documentation](https://docs.rs/crossterm/latest/crossterm/)
- [Terminal Capabilities Detection](https://invisible-island.net/ncurses/man/terminfo.5.html)

### Code Examples
```rust
// Example of view registration system
impl DoraApp {
    fn register_views(&mut self) {
        self.view_registry.register("dashboard", Box::new(DashboardView::new));
        self.view_registry.register("dataflow", Box::new(DataflowManagerView::new));
        // ... other views
    }
}
```

## ✅ Definition of Done
- [ ] DoraApp framework implemented with complete event loop
- [ ] View management system supports navigation stack and view switching
- [ ] CLI command execution works seamlessly within TUI
- [ ] Command mode provides full CLI functionality with history
- [ ] Theme system provides consistent styling across all views
- [ ] Terminal setup and cleanup work reliably across different terminals
- [ ] Memory usage stays under 50MB for typical operations
- [ ] TUI launches in <500ms from CLI commands
- [ ] Key bindings are intuitive and well-documented
- [ ] Error handling provides clear feedback to users
- [ ] Unit tests cover all core functionality
- [ ] Integration tests validate CLI-TUI integration
- [ ] Manual testing confirms usability across different terminal environments

## 📝 Implementation Notes

### Performance Considerations
- Use efficient data structures for view state management
- Implement lazy loading for expensive view operations
- Optimize rendering to only update changed areas
- Cache commonly accessed data to reduce API calls

### Terminal Compatibility
- Test across major terminal emulators (iTerm2, Terminal.app, gnome-terminal, etc.)
- Handle terminal resize events gracefully
- Support both light and dark terminal themes
- Provide fallback rendering for limited terminal capabilities

### Future Extensions
- Plugin system for custom views
- Scriptable TUI automation
- Remote TUI access over SSH
- Integration with external monitoring tools
- Advanced keyboard shortcut customization

This TUI launcher framework provides the solid foundation needed for all interactive interfaces while maintaining seamless integration with the CLI components of the hybrid architecture.