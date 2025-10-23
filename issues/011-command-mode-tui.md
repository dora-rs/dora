# Issue #011: Add Command Mode in TUI

## 📋 Summary
Implement a powerful command mode within the TUI that allows users to execute any CLI command without leaving the interactive interface. This feature provides vim-like command execution, enabling power users to maintain their workflow efficiency while benefiting from TUI visualization.

## 🎯 Objectives
- Create vim-like command mode accessible via `:` key in TUI
- Enable full CLI command execution with real-time TUI updates
- Add command history, completion, and validation
- Provide seamless integration between command execution and view updates
- Ensure command mode feels natural and responsive

**Success Metrics:**
- Command mode activates instantly (<50ms) on `:` keypress
- All CLI commands work correctly within TUI environment
- Command completion provides helpful suggestions
- TUI views update immediately after command execution
- Command history preserves context across TUI sessions

## 🛠️ Technical Requirements

### What to Build

#### 1. Command Mode Infrastructure
```rust
// src/tui/command_mode.rs
#[derive(Debug, Clone)]
pub enum CommandModeState {
    Inactive,
    Active {
        buffer: String,
        cursor_position: usize,
        completion_state: CompletionState,
        history_navigation: HistoryNavigation,
    },
}

#[derive(Debug, Clone)]
pub struct CompletionState {
    pub suggestions: Vec<CommandSuggestion>,
    pub selected_index: Option<usize>,
    pub completion_prefix: String,
    pub completion_type: CompletionType,
}

#[derive(Debug, Clone)]
pub struct CommandSuggestion {
    pub text: String,
    pub description: String,
    pub suggestion_type: SuggestionType,
    pub priority: u8,
}

#[derive(Debug, Clone)]
pub enum SuggestionType {
    Command,
    Flag,
    Value,
    FilePath,
    DataflowName,
    NodeName,
}

#[derive(Debug)]
pub struct CommandModeManager {
    state: CommandModeState,
    command_executor: TuiCommandExecutor,
    completion_engine: CompletionEngine,
    history_manager: CommandHistoryManager,
    validation_engine: CommandValidationEngine,
}

impl CommandModeManager {
    pub fn new() -> Self {
        Self {
            state: CommandModeState::Inactive,
            command_executor: TuiCommandExecutor::new(),
            completion_engine: CompletionEngine::new(),
            history_manager: CommandHistoryManager::new(),
            validation_engine: CommandValidationEngine::new(),
        }
    }
    
    pub fn activate(&mut self) {
        self.state = CommandModeState::Active {
            buffer: String::new(),
            cursor_position: 0,
            completion_state: CompletionState::new(),
            history_navigation: HistoryNavigation::new(),
        };
    }
    
    pub fn deactivate(&mut self) {
        self.state = CommandModeState::Inactive;
    }
    
    pub async fn handle_key_event(
        &mut self,
        key: KeyEvent,
        app_state: &mut AppState,
    ) -> Result<CommandModeAction> {
        match &mut self.state {
            CommandModeState::Inactive => Ok(CommandModeAction::None),
            CommandModeState::Active { 
                ref mut buffer, 
                ref mut cursor_position, 
                ref mut completion_state,
                ref mut history_navigation,
            } => {
                match key.code {
                    KeyCode::Enter => {
                        let command = buffer.clone();
                        self.deactivate();
                        
                        if !command.is_empty() {
                            self.history_manager.add_command(&command);
                            let result = self.command_executor.execute(&command, app_state).await?;
                            Ok(CommandModeAction::ExecuteCommand(result))
                        } else {
                            Ok(CommandModeAction::Cancel)
                        }
                    },
                    
                    KeyCode::Esc => {
                        self.deactivate();
                        Ok(CommandModeAction::Cancel)
                    },
                    
                    KeyCode::Tab => {
                        self.handle_completion(buffer, cursor_position, completion_state).await?;
                        Ok(CommandModeAction::UpdateDisplay)
                    },
                    
                    KeyCode::Up => {
                        if let Some(prev_command) = history_navigation.previous(&self.history_manager) {
                            *buffer = prev_command;
                            *cursor_position = buffer.len();
                            completion_state.clear();
                        }
                        Ok(CommandModeAction::UpdateDisplay)
                    },
                    
                    KeyCode::Down => {
                        if let Some(next_command) = history_navigation.next(&self.history_manager) {
                            *buffer = next_command;
                            *cursor_position = buffer.len();
                            completion_state.clear();
                        } else {
                            buffer.clear();
                            *cursor_position = 0;
                            completion_state.clear();
                        }
                        Ok(CommandModeAction::UpdateDisplay)
                    },
                    
                    KeyCode::Left => {
                        if *cursor_position > 0 {
                            *cursor_position -= 1;
                        }
                        Ok(CommandModeAction::UpdateDisplay)
                    },
                    
                    KeyCode::Right => {
                        if *cursor_position < buffer.len() {
                            *cursor_position += 1;
                        }
                        Ok(CommandModeAction::UpdateDisplay)
                    },
                    
                    KeyCode::Backspace => {
                        if *cursor_position > 0 {
                            buffer.remove(*cursor_position - 1);
                            *cursor_position -= 1;
                            self.update_completion(buffer, *cursor_position, completion_state).await;
                        }
                        Ok(CommandModeAction::UpdateDisplay)
                    },
                    
                    KeyCode::Delete => {
                        if *cursor_position < buffer.len() {
                            buffer.remove(*cursor_position);
                            self.update_completion(buffer, *cursor_position, completion_state).await;
                        }
                        Ok(CommandModeAction::UpdateDisplay)
                    },
                    
                    KeyCode::Char(c) => {
                        buffer.insert(*cursor_position, c);
                        *cursor_position += 1;
                        self.update_completion(buffer, *cursor_position, completion_state).await;
                        Ok(CommandModeAction::UpdateDisplay)
                    },
                    
                    _ => Ok(CommandModeAction::None),
                }
            }
        }
    }
    
    pub fn render<B: Backend>(
        &self,
        f: &mut Frame<B>,
        area: Rect,
        theme: &ThemeConfig,
    ) {
        if let CommandModeState::Active { 
            ref buffer, 
            cursor_position, 
            ref completion_state,
            .. 
        } = self.state {
            self.render_command_input(f, area, buffer, cursor_position, theme);
            
            if !completion_state.suggestions.is_empty() {
                self.render_completion_popup(f, area, completion_state, theme);
            }
        }
    }
}

#[derive(Debug)]
pub enum CommandModeAction {
    None,
    UpdateDisplay,
    ExecuteCommand(CommandExecutionResult),
    Cancel,
}
```

#### 2. Command Completion Engine
```rust
// src/tui/completion.rs
#[derive(Debug)]
pub struct CompletionEngine {
    command_registry: CommandRegistry,
    context_providers: Vec<Box<dyn ContextProvider>>,
}

#[derive(Debug)]
pub struct CommandRegistry {
    commands: HashMap<String, CommandInfo>,
    aliases: HashMap<String, String>,
}

#[derive(Debug, Clone)]
pub struct CommandInfo {
    pub name: String,
    pub description: String,
    pub subcommands: Vec<String>,
    pub flags: Vec<FlagInfo>,
    pub arguments: Vec<ArgumentInfo>,
}

trait ContextProvider: Send + Sync {
    async fn provide_suggestions(
        &self,
        context: &CompletionContext,
    ) -> Result<Vec<CommandSuggestion>>;
}

impl CompletionEngine {
    pub fn new() -> Self {
        let mut engine = Self {
            command_registry: CommandRegistry::new(),
            context_providers: Vec::new(),
        };
        
        engine.register_built_in_commands();
        engine.register_context_providers();
        engine
    }
    
    pub async fn get_completions(
        &self,
        input: &str,
        cursor_position: usize,
        app_state: &AppState,
    ) -> Result<Vec<CommandSuggestion>> {
        let context = CompletionContext::analyze(input, cursor_position);
        let mut suggestions = Vec::new();
        
        match context.completion_type {
            CompletionType::Command => {
                suggestions.extend(self.complete_command(&context.prefix));
            },
            
            CompletionType::Subcommand => {
                if let Some(parent_cmd) = &context.parent_command {
                    suggestions.extend(self.complete_subcommand(parent_cmd, &context.prefix));
                }
            },
            
            CompletionType::Flag => {
                if let Some(cmd) = &context.current_command {
                    suggestions.extend(self.complete_flag(cmd, &context.prefix));
                }
            },
            
            CompletionType::Value => {
                suggestions.extend(self.complete_value(&context, app_state).await?);
            },
        }
        
        // Sort suggestions by priority and relevance
        suggestions.sort_by(|a, b| {
            b.priority.cmp(&a.priority)
                .then_with(|| a.text.len().cmp(&b.text.len()))
        });
        
        Ok(suggestions)
    }
    
    fn complete_command(&self, prefix: &str) -> Vec<CommandSuggestion> {
        let mut suggestions = Vec::new();
        
        for (name, info) in &self.command_registry.commands {
            if name.starts_with(prefix) {
                suggestions.push(CommandSuggestion {
                    text: name.clone(),
                    description: info.description.clone(),
                    suggestion_type: SuggestionType::Command,
                    priority: if name == prefix { 100 } else { 80 },
                });
            }
        }
        
        // Add aliases
        for (alias, command) in &self.command_registry.aliases {
            if alias.starts_with(prefix) {
                if let Some(info) = self.command_registry.commands.get(command) {
                    suggestions.push(CommandSuggestion {
                        text: alias.clone(),
                        description: format!("Alias for {}: {}", command, info.description),
                        suggestion_type: SuggestionType::Command,
                        priority: 70,
                    });
                }
            }
        }
        
        suggestions
    }
    
    async fn complete_value(
        &self,
        context: &CompletionContext,
        app_state: &AppState,
    ) -> Result<Vec<CommandSuggestion>> {
        let mut suggestions = Vec::new();
        
        // Use context providers for dynamic completion
        for provider in &self.context_providers {
            let provider_suggestions = provider.provide_suggestions(context).await?;
            suggestions.extend(provider_suggestions);
        }
        
        Ok(suggestions)
    }
    
    fn register_context_providers(&mut self) {
        self.context_providers.push(Box::new(DataflowNameProvider));
        self.context_providers.push(Box::new(NodeNameProvider));
        self.context_providers.push(Box::new(FilePathProvider));
        self.context_providers.push(Box::new(ConfigKeyProvider));
    }
}

// Context providers for dynamic completion
struct DataflowNameProvider;

impl ContextProvider for DataflowNameProvider {
    async fn provide_suggestions(
        &self,
        context: &CompletionContext,
    ) -> Result<Vec<CommandSuggestion>> {
        if !context.expects_dataflow_name() {
            return Ok(Vec::new());
        }
        
        let dataflows = DaemonClient::connect().await?.list_dataflows().await?;
        let suggestions = dataflows
            .iter()
            .filter(|df| df.name.starts_with(&context.prefix))
            .map(|df| CommandSuggestion {
                text: df.name.clone(),
                description: format!("Dataflow ({})", df.status),
                suggestion_type: SuggestionType::DataflowName,
                priority: 90,
            })
            .collect();
        
        Ok(suggestions)
    }
}

struct NodeNameProvider;

impl ContextProvider for NodeNameProvider {
    async fn provide_suggestions(
        &self,
        context: &CompletionContext,
    ) -> Result<Vec<CommandSuggestion>> {
        if !context.expects_node_name() {
            return Ok(Vec::new());
        }
        
        let client = DaemonClient::connect().await?;
        let mut suggestions = Vec::new();
        
        // Get nodes from all dataflows
        let dataflows = client.list_dataflows().await?;
        for dataflow in dataflows {
            let nodes = client.get_dataflow_nodes(&dataflow.id).await?;
            for node in nodes {
                if node.name.starts_with(&context.prefix) {
                    suggestions.push(CommandSuggestion {
                        text: node.name.clone(),
                        description: format!("Node in {} ({})", dataflow.name, node.status),
                        suggestion_type: SuggestionType::NodeName,
                        priority: 85,
                    });
                }
            }
        }
        
        Ok(suggestions)
    }
}
```

#### 3. Command Execution Integration
```rust
// src/tui/command_executor.rs
#[derive(Debug)]
pub struct TuiCommandExecutor {
    cli_executor: CliExecutor,
    view_updater: ViewUpdater,
    state_synchronizer: StateSynchronizer,
}

#[derive(Debug)]
pub struct CommandExecutionResult {
    pub success: bool,
    pub output: String,
    pub view_updates: Vec<ViewUpdate>,
    pub status_message: Option<StatusMessage>,
    pub execution_time: Duration,
}

#[derive(Debug)]
pub enum ViewUpdate {
    RefreshCurrentView,
    SwitchToView(ViewType),
    UpdateDataflows,
    UpdateSystemMetrics,
    UpdateSpecificNode(String),
    ShowError(String),
    ShowSuccess(String),
}

impl TuiCommandExecutor {
    pub async fn execute(
        &self,
        command_str: &str,
        app_state: &mut AppState,
    ) -> Result<CommandExecutionResult> {
        let start_time = Instant::now();
        
        // Parse and validate command
        let args = shell_words::split(command_str)?;
        let cli = Cli::try_parse_from(std::iter::once("dora").chain(args.iter()))?;
        
        // Execute command with TUI context
        let execution_result = self.execute_with_context(&cli, app_state).await?;
        
        // Determine view updates needed
        let view_updates = self.determine_view_updates(&cli, &execution_result, app_state);
        
        // Apply view updates
        for update in &view_updates {
            self.apply_view_update(update, app_state).await?;
        }
        
        let execution_time = start_time.elapsed();
        
        Ok(CommandExecutionResult {
            success: execution_result.success,
            output: execution_result.output,
            view_updates,
            status_message: self.create_status_message(&execution_result),
            execution_time,
        })
    }
    
    async fn execute_with_context(
        &self,
        cli: &Cli,
        app_state: &AppState,
    ) -> Result<CliExecutionResult> {
        // Create execution context from TUI state
        let execution_context = ExecutionContext {
            is_tty: true,
            is_piped: false,
            is_scripted: false,
            terminal_size: Some((120, 40)), // TUI size
            user_preference: app_state.user_config.ui.mode.clone(),
            terminal_capabilities: TerminalCapabilities::full(),
            environment: app_state.environment.clone(),
        };
        
        // Execute command
        match &cli.command {
            Command::Ps(cmd) => {
                let result = cmd.execute_silent(&execution_context).await?;
                Ok(CliExecutionResult {
                    success: true,
                    output: "Dataflow list updated".to_string(),
                    data: Some(result),
                    error: None,
                })
            },
            
            Command::Start(cmd) => {
                let result = cmd.execute_silent(&execution_context).await?;
                Ok(CliExecutionResult {
                    success: result.success,
                    output: if result.success {
                        format!("Started dataflow '{}'", result.dataflow_name)
                    } else {
                        format!("Failed to start dataflow: {}", 
                               result.error.unwrap_or_default())
                    },
                    data: Some(result),
                    error: None,
                })
            },
            
            Command::Stop(cmd) => {
                let result = cmd.execute_silent(&execution_context).await?;
                Ok(CliExecutionResult {
                    success: result.success,
                    output: if result.success {
                        format!("Stopped dataflow '{}'", result.dataflow_name)
                    } else {
                        format!("Failed to stop dataflow: {}", 
                               result.error.unwrap_or_default())
                    },
                    data: Some(result),
                    error: None,
                })
            },
            
            Command::Config(cmd) => {
                let result = cmd.execute_silent(&execution_context).await?;
                Ok(CliExecutionResult {
                    success: true,
                    output: result.message,
                    data: None,
                    error: None,
                })
            },
            
            _ => {
                // For other commands, execute in background
                let result = self.cli_executor.execute_background(cli).await?;
                Ok(result)
            }
        }
    }
    
    fn determine_view_updates(
        &self,
        cli: &Cli,
        result: &CliExecutionResult,
        app_state: &AppState,
    ) -> Vec<ViewUpdate> {
        let mut updates = Vec::new();
        
        if !result.success {
            if let Some(error) = &result.error {
                updates.push(ViewUpdate::ShowError(error.clone()));
            }
            return updates;
        }
        
        match &cli.command {
            Command::Ps(_) => {
                updates.push(ViewUpdate::UpdateDataflows);
                if matches!(app_state.current_view, ViewType::Dashboard) {
                    updates.push(ViewUpdate::RefreshCurrentView);
                }
            },
            
            Command::Start(_) | Command::Stop(_) => {
                updates.push(ViewUpdate::UpdateDataflows);
                updates.push(ViewUpdate::RefreshCurrentView);
                updates.push(ViewUpdate::ShowSuccess(result.output.clone()));
            },
            
            Command::Inspect(cmd) => {
                updates.push(ViewUpdate::SwitchToView(ViewType::NodeInspector {
                    node_id: cmd.target.clone(),
                }));
            },
            
            Command::Logs(cmd) => {
                updates.push(ViewUpdate::SwitchToView(ViewType::LogViewer {
                    target: cmd.target.clone(),
                }));
            },
            
            Command::Config(_) => {
                updates.push(ViewUpdate::RefreshCurrentView);
                updates.push(ViewUpdate::ShowSuccess("Configuration updated".to_string()));
            },
            
            _ => {
                updates.push(ViewUpdate::ShowSuccess(result.output.clone()));
            }
        }
        
        updates
    }
}
```

#### 4. Command Mode UI Rendering
```rust
impl CommandModeManager {
    fn render_command_input<B: Backend>(
        &self,
        f: &mut Frame<B>,
        area: Rect,
        buffer: &str,
        cursor_position: usize,
        theme: &ThemeConfig,
    ) {
        let input_area = Rect {
            x: area.x,
            y: area.y + area.height.saturating_sub(1),
            width: area.width,
            height: 1,
        };
        
        // Create input text with cursor
        let mut display_text = format!(":{}", buffer);
        
        // Calculate cursor display position
        let cursor_display_pos = cursor_position + 1; // +1 for the ':'
        
        // Create input widget
        let input = Paragraph::new(display_text.as_str())
            .style(theme.styles.status_style)
            .block(Block::default());
        
        f.render_widget(input, input_area);
        
        // Render cursor
        if cursor_display_pos < display_text.len() {
            f.set_cursor(
                input_area.x + cursor_display_pos as u16,
                input_area.y,
            );
        }
    }
    
    fn render_completion_popup<B: Backend>(
        &self,
        f: &mut Frame<B>,
        area: Rect,
        completion_state: &CompletionState,
        theme: &ThemeConfig,
    ) {
        let suggestions = &completion_state.suggestions;
        if suggestions.is_empty() {
            return;
        }
        
        let popup_height = (suggestions.len() as u16).min(10);
        let popup_width = suggestions
            .iter()
            .map(|s| s.text.len() + s.description.len() + 3)
            .max()
            .unwrap_or(50) as u16;
        
        let popup_area = Rect {
            x: area.x,
            y: area.y + area.height.saturating_sub(popup_height + 2),
            width: popup_width.min(area.width),
            height: popup_height + 2,
        };
        
        // Create list items
        let items: Vec<ListItem> = suggestions
            .iter()
            .enumerate()
            .map(|(i, suggestion)| {
                let style = if Some(i) == completion_state.selected_index {
                    theme.styles.selection_style
                } else {
                    Style::default()
                };
                
                let content = vec![Spans::from(vec![
                    Span::styled(&suggestion.text, style.add_modifier(Modifier::BOLD)),
                    Span::raw(" - "),
                    Span::styled(&suggestion.description, style),
                ])];
                
                ListItem::new(content).style(style)
            })
            .collect();
        
        let list = List::new(items)
            .block(
                Block::default()
                    .title("Completions")
                    .borders(Borders::ALL)
                    .border_style(theme.colors.border)
            )
            .style(Style::default().fg(theme.colors.text));
        
        f.render_widget(Clear, popup_area); // Clear background
        f.render_widget(list, popup_area);
    }
}
```

### Why This Approach

**Vim-like Familiarity:**
- Natural `:` key activation that feels familiar to vim users
- Command history navigation with arrow keys
- Tab completion that enhances discoverability
- Escape key cancellation for quick exit

**Powerful Integration:**
- Full CLI command support within TUI
- Real-time view updates based on command execution
- Context-aware completion using live system state
- Seamless state synchronization between command and visual modes

**Enhanced Productivity:**
- Fast command execution without leaving TUI
- Intelligent completion reduces typing and errors
- Command history preserves workflow efficiency
- Visual feedback shows command results immediately

### How to Implement

#### Step 1: Command Mode Infrastructure (4 hours)
1. **Implement CommandModeManager** with state management
2. **Add key event handling** for command mode activation/deactivation
3. **Create command input** buffer and cursor management
4. **Add basic command** parsing and validation

#### Step 2: Completion Engine (5 hours)
1. **Implement CompletionEngine** with command registry
2. **Add context providers** for dynamic completion
3. **Create completion UI** with popup rendering
4. **Add tab completion** and selection handling

#### Step 3: Command Execution (3 hours)
1. **Implement TuiCommandExecutor** with view integration
2. **Add command execution** with proper error handling
3. **Create view update** system based on command results
4. **Add status message** generation and display

#### Step 4: UI Integration (2 hours)
1. **Integrate command mode** with main TUI event loop
2. **Add command input** rendering with cursor display
3. **Create completion popup** with proper positioning
4. **Add visual feedback** for command execution

#### Step 5: Testing and Polish (2 hours)
1. **Add comprehensive unit tests** for all functionality
2. **Test command completion** accuracy and performance
3. **Validate command execution** and view updates
4. **Test edge cases** and error handling

## 🔗 Dependencies
**Depends On:**
- Issue #001 (Hybrid Command Framework) - CLI command structures
- Issue #009 (TUI Launcher Framework) - TUI infrastructure
- Issue #010 (CLI ↔ TUI Transitions) - Command execution in TUI

**Blocks:** Advanced TUI features that rely on command mode functionality

## 🧪 Testing Requirements

### Unit Tests
```rust
#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_command_mode_activation() {
        let mut manager = CommandModeManager::new();
        manager.activate();
        
        assert!(matches!(manager.state, CommandModeState::Active { .. }));
    }
    
    #[test]
    fn test_command_completion() {
        let engine = CompletionEngine::new();
        let suggestions = engine.get_completions("p", 1, &AppState::new()).await.unwrap();
        
        assert!(suggestions.iter().any(|s| s.text == "ps"));
    }
    
    #[test]
    fn test_command_execution() {
        let executor = TuiCommandExecutor::new();
        let mut app_state = AppState::new();
        
        let result = executor.execute("ps", &mut app_state).await.unwrap();
        assert!(result.success);
        assert!(!result.view_updates.is_empty());
    }
}
```

## ✅ Definition of Done
- [ ] CommandModeManager implemented with full key handling
- [ ] Command completion provides accurate and helpful suggestions
- [ ] All CLI commands execute correctly within TUI environment
- [ ] View updates work seamlessly based on command execution results
- [ ] Command history persists and navigates correctly
- [ ] UI rendering provides clear visual feedback
- [ ] Performance meets targets for completion and execution
- [ ] Error handling provides appropriate user feedback
- [ ] Comprehensive unit tests cover all functionality
- [ ] Integration tests validate command mode workflows
- [ ] Manual testing confirms intuitive user experience

This command mode implementation provides power users with the efficiency of CLI commands while maintaining the benefits of TUI visualization, creating a truly hybrid user experience.