use std::{
    collections::{HashMap, VecDeque},
    time::Instant,
};

use crossterm::event::{KeyCode, KeyEvent};
use ratatui::{
    backend::Backend,
    layout::Rect,
    style::{Modifier, Style},
    terminal::Frame,
    text::{Line, Span},
    widgets::{Block, Borders, Clear, List, ListItem, Paragraph},
};

use crate::tui::{Result, ViewType, app::AppState, theme::ThemeConfig};

/// Command mode state management
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

/// State for command completion
#[derive(Debug, Clone)]
pub struct CompletionState {
    pub suggestions: Vec<CommandSuggestion>,
    pub selected_index: Option<usize>,
    pub completion_prefix: String,
    pub completion_type: CompletionType,
}

impl CompletionState {
    pub fn new() -> Self {
        Self {
            suggestions: Vec::new(),
            selected_index: None,
            completion_prefix: String::new(),
            completion_type: CompletionType::Command,
        }
    }

    pub fn clear(&mut self) {
        self.suggestions.clear();
        self.selected_index = None;
        self.completion_prefix.clear();
    }

    pub fn select_next(&mut self) {
        if self.suggestions.is_empty() {
            return;
        }

        self.selected_index = match self.selected_index {
            None => Some(0),
            Some(i) if i >= self.suggestions.len() - 1 => Some(0),
            Some(i) => Some(i + 1),
        };
    }

    pub fn select_previous(&mut self) {
        if self.suggestions.is_empty() {
            return;
        }

        self.selected_index = match self.selected_index {
            None => Some(self.suggestions.len() - 1),
            Some(0) => Some(self.suggestions.len() - 1),
            Some(i) => Some(i - 1),
        };
    }

    pub fn get_selected_suggestion(&self) -> Option<&CommandSuggestion> {
        self.selected_index.and_then(|i| self.suggestions.get(i))
    }
}

/// Individual command suggestion
#[derive(Debug, Clone)]
pub struct CommandSuggestion {
    pub text: String,
    pub description: String,
    pub suggestion_type: SuggestionType,
    pub priority: u8,
}

/// Type of completion being performed
#[derive(Debug, Clone)]
pub enum CompletionType {
    Command,
    Subcommand,
    Flag,
    Value,
    FilePath,
}

/// Type of suggestion
#[derive(Debug, Clone)]
pub enum SuggestionType {
    Command,
    Flag,
    Value,
    FilePath,
    DataflowName,
    NodeName,
}

/// History navigation state
#[derive(Debug, Clone)]
pub struct HistoryNavigation {
    current_index: Option<usize>,
    original_buffer: String,
}

impl HistoryNavigation {
    pub fn new() -> Self {
        Self {
            current_index: None,
            original_buffer: String::new(),
        }
    }

    pub fn reset(&mut self, buffer: String) {
        self.current_index = None;
        self.original_buffer = buffer;
    }

    pub fn navigate_up(&mut self, history: &[String]) -> Option<String> {
        if history.is_empty() {
            return None;
        }

        match self.current_index {
            None => {
                self.current_index = Some(history.len() - 1);
                Some(history[history.len() - 1].clone())
            }
            Some(0) => None, // Already at oldest
            Some(i) => {
                self.current_index = Some(i - 1);
                Some(history[i - 1].clone())
            }
        }
    }

    pub fn navigate_down(&mut self, history: &[String]) -> Option<String> {
        match self.current_index {
            None => None,
            Some(i) if i >= history.len() - 1 => {
                self.current_index = None;
                Some(self.original_buffer.clone())
            }
            Some(i) => {
                self.current_index = Some(i + 1);
                Some(history[i + 1].clone())
            }
        }
    }
}

/// Main command mode manager
#[derive(Debug)]
pub struct CommandModeManager {
    state: CommandModeState,
    command_history: VecDeque<String>,
    completion_engine: CompletionEngine,
    last_execution_time: Option<Instant>,
}

/// Actions that can result from command mode events
#[derive(Debug)]
pub enum CommandModeAction {
    None,
    UpdateDisplay,
    ExecuteCommand { command: String, show_output: bool },
    Cancel,
    SwitchView(ViewType),
}

impl CommandModeManager {
    pub fn new() -> Self {
        Self {
            state: CommandModeState::Inactive,
            command_history: VecDeque::new(),
            completion_engine: CompletionEngine::new(),
            last_execution_time: None,
        }
    }

    pub fn is_active(&self) -> bool {
        matches!(self.state, CommandModeState::Active { .. })
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
        app_state: &AppState,
    ) -> Result<CommandModeAction> {
        match &mut self.state {
            CommandModeState::Inactive => Ok(CommandModeAction::None),
            CommandModeState::Active {
                buffer,
                cursor_position,
                completion_state,
                history_navigation,
            } => {
                match key.code {
                    KeyCode::Enter => {
                        let command = buffer.clone();
                        self.deactivate();

                        if !command.is_empty() {
                            self.add_to_history(command.clone());
                            self.last_execution_time = Some(Instant::now());

                            // Check for special TUI commands
                            if let Some(view_action) = self.parse_tui_command(&command) {
                                Ok(view_action)
                            } else {
                                Ok(CommandModeAction::ExecuteCommand {
                                    command,
                                    show_output: true,
                                })
                            }
                        } else {
                            Ok(CommandModeAction::Cancel)
                        }
                    }

                    KeyCode::Esc => {
                        self.deactivate();
                        Ok(CommandModeAction::Cancel)
                    }

                    KeyCode::Tab => {
                        // Update completion first
                        let suggestions = self
                            .completion_engine
                            .get_completions(buffer, *cursor_position, app_state)
                            .await?;
                        completion_state.suggestions = suggestions;
                        completion_state.selected_index = if completion_state.suggestions.is_empty()
                        {
                            None
                        } else {
                            Some(0)
                        };

                        // Apply selected completion if available
                        if let Some(suggestion) = completion_state.get_selected_suggestion() {
                            let suggestion_text = suggestion.text.clone();

                            // Apply completion inline to avoid borrow issues
                            let before_cursor = &buffer[..*cursor_position];
                            let after_cursor = &buffer[*cursor_position..];
                            let word_start = before_cursor.rfind(' ').map(|i| i + 1).unwrap_or(0);
                            let new_buffer = format!(
                                "{}{}{}",
                                &buffer[..word_start],
                                suggestion_text,
                                after_cursor
                            );
                            let new_cursor = word_start + suggestion_text.len();

                            *buffer = new_buffer;
                            *cursor_position = new_cursor;
                            completion_state.clear();
                        }

                        Ok(CommandModeAction::UpdateDisplay)
                    }

                    KeyCode::Up => {
                        if !completion_state.suggestions.is_empty() {
                            completion_state.select_previous();
                        } else {
                            // Navigate command history
                            let history: Vec<String> =
                                self.command_history.iter().cloned().collect();
                            if let Some(prev_command) = history_navigation.navigate_up(&history) {
                                *buffer = prev_command;
                                *cursor_position = buffer.len();
                                completion_state.clear();
                            }
                        }
                        Ok(CommandModeAction::UpdateDisplay)
                    }

                    KeyCode::Down => {
                        if !completion_state.suggestions.is_empty() {
                            completion_state.select_next();
                        } else {
                            // Navigate command history
                            let history: Vec<String> =
                                self.command_history.iter().cloned().collect();
                            if let Some(next_command) = history_navigation.navigate_down(&history) {
                                *buffer = next_command;
                                *cursor_position = buffer.len();
                                completion_state.clear();
                            }
                        }
                        Ok(CommandModeAction::UpdateDisplay)
                    }

                    KeyCode::Left => {
                        if *cursor_position > 0 {
                            *cursor_position -= 1;
                        }
                        Ok(CommandModeAction::UpdateDisplay)
                    }

                    KeyCode::Right => {
                        if *cursor_position < buffer.len() {
                            *cursor_position += 1;
                        }
                        Ok(CommandModeAction::UpdateDisplay)
                    }

                    KeyCode::Backspace => {
                        if *cursor_position > 0 {
                            buffer.remove(*cursor_position - 1);
                            *cursor_position -= 1;
                            // Update completion after modifying buffer
                            let suggestions = self
                                .completion_engine
                                .get_completions(buffer, *cursor_position, app_state)
                                .await?;
                            completion_state.suggestions = suggestions;
                            completion_state.selected_index =
                                if completion_state.suggestions.is_empty() {
                                    None
                                } else {
                                    Some(0)
                                };
                        }
                        Ok(CommandModeAction::UpdateDisplay)
                    }

                    KeyCode::Delete => {
                        if *cursor_position < buffer.len() {
                            buffer.remove(*cursor_position);
                            // Update completion after modifying buffer
                            let suggestions = self
                                .completion_engine
                                .get_completions(buffer, *cursor_position, app_state)
                                .await?;
                            completion_state.suggestions = suggestions;
                            completion_state.selected_index =
                                if completion_state.suggestions.is_empty() {
                                    None
                                } else {
                                    Some(0)
                                };
                        }
                        Ok(CommandModeAction::UpdateDisplay)
                    }

                    KeyCode::Char(c) => {
                        buffer.insert(*cursor_position, c);
                        *cursor_position += 1;
                        // Update completion after modifying buffer
                        let suggestions = self
                            .completion_engine
                            .get_completions(buffer, *cursor_position, app_state)
                            .await?;
                        completion_state.suggestions = suggestions;
                        completion_state.selected_index = if completion_state.suggestions.is_empty()
                        {
                            None
                        } else {
                            Some(0)
                        };
                        Ok(CommandModeAction::UpdateDisplay)
                    }

                    _ => Ok(CommandModeAction::None),
                }
            }
        }
    }

    pub fn render(&self, f: &mut Frame, area: Rect, theme: &ThemeConfig) {
        if let CommandModeState::Active {
            ref buffer,
            cursor_position,
            ref completion_state,
            ..
        } = self.state
        {
            self.render_command_input(f, area, buffer, cursor_position, theme);

            if !completion_state.suggestions.is_empty() {
                self.render_completion_popup(f, area, completion_state, theme);
            }
        }
    }

    fn render_command_input(
        &self,
        f: &mut Frame,
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
        let display_text = format!(":{}", buffer);

        // Create input widget
        let input = Paragraph::new(display_text.as_str())
            .style(theme.styles.status_style)
            .block(Block::default());

        f.render_widget(input, input_area);

        // Set cursor position (accounting for the ':' prefix)
        let cursor_display_pos = (cursor_position + 1).min(display_text.len());
        f.set_cursor(input_area.x + cursor_display_pos as u16, input_area.y);
    }

    fn render_completion_popup(
        &self,
        f: &mut Frame,
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
            .map(|s| s.text.len() + s.description.len() + 4)
            .max()
            .unwrap_or(50) as u16;

        let popup_area = Rect {
            x: area.x,
            y: area.y + area.height.saturating_sub(popup_height + 3),
            width: popup_width.min(area.width),
            height: popup_height + 2,
        };

        // Create list items
        let items: Vec<ListItem> = suggestions
            .iter()
            .enumerate()
            .map(|(i, suggestion)| {
                let is_selected = Some(i) == completion_state.selected_index;
                let style = if is_selected {
                    theme.styles.selection_style
                } else {
                    Style::default().fg(theme.colors.text)
                };

                let content = vec![Line::from(vec![
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
                    .border_style(Style::default().fg(theme.colors.border)),
            )
            .style(Style::default().fg(theme.colors.text));

        f.render_widget(Clear, popup_area);
        f.render_widget(list, popup_area);
    }

    fn apply_completion(
        &self,
        buffer: &str,
        cursor_position: usize,
        completion: &str,
    ) -> (String, usize) {
        // Find the word being completed
        let before_cursor = &buffer[..cursor_position];
        let after_cursor = &buffer[cursor_position..];

        // Find the start of the current word
        let word_start = before_cursor.rfind(' ').map(|i| i + 1).unwrap_or(0);
        let word_prefix = &before_cursor[word_start..];

        // Replace the current word with the completion
        let new_buffer = format!(
            "{}{}{}",
            &before_cursor[..word_start],
            completion,
            after_cursor
        );

        let new_cursor_position = word_start + completion.len();

        (new_buffer, new_cursor_position)
    }

    fn parse_tui_command(&self, command: &str) -> Option<CommandModeAction> {
        let trimmed = command.trim();

        match trimmed {
            "dashboard" | "dash" => Some(CommandModeAction::SwitchView(ViewType::Dashboard)),
            "dataflow" | "df" => Some(CommandModeAction::SwitchView(ViewType::DataflowManager)),
            "monitor" | "sys" => Some(CommandModeAction::SwitchView(ViewType::SystemMonitor)),
            "logs" => Some(CommandModeAction::SwitchView(ViewType::LogViewer {
                target: "system".to_string(),
            })),
            "settings" | "config" => Some(CommandModeAction::SwitchView(ViewType::SettingsManager)),
            "help" => Some(CommandModeAction::SwitchView(ViewType::Help)),
            _ => None,
        }
    }

    fn add_to_history(&mut self, command: String) {
        // Avoid duplicate consecutive entries
        if self.command_history.back() != Some(&command) {
            self.command_history.push_back(command);
        }

        // Keep history bounded
        const MAX_HISTORY: usize = 100;
        while self.command_history.len() > MAX_HISTORY {
            self.command_history.pop_front();
        }
    }

    pub fn get_command_history(&self) -> &VecDeque<String> {
        &self.command_history
    }

    pub fn get_last_execution_time(&self) -> Option<Instant> {
        self.last_execution_time
    }
}

/// Simple completion engine for commands
#[derive(Debug)]
pub struct CompletionEngine {
    command_registry: HashMap<String, CommandInfo>,
}

#[derive(Debug, Clone)]
pub struct CommandInfo {
    pub name: String,
    pub description: String,
    pub subcommands: Vec<String>,
    pub common_flags: Vec<String>,
}

impl CompletionEngine {
    pub fn new() -> Self {
        let mut engine = Self {
            command_registry: HashMap::new(),
        };

        engine.register_built_in_commands();
        engine
    }

    pub async fn get_completions(
        &self,
        input: &str,
        cursor_position: usize,
        app_state: &AppState,
    ) -> Result<Vec<CommandSuggestion>> {
        let context = self.analyze_completion_context(input, cursor_position);
        let mut suggestions = Vec::new();

        match context.completion_type {
            CompletionType::Command => {
                suggestions.extend(self.complete_command(&context.prefix));
            }

            CompletionType::Subcommand => {
                if let Some(parent_cmd) = &context.parent_command {
                    suggestions.extend(self.complete_subcommand(parent_cmd, &context.prefix));
                }
            }

            CompletionType::Flag => {
                if let Some(cmd) = &context.current_command {
                    suggestions.extend(self.complete_flag(cmd, &context.prefix));
                }
            }

            CompletionType::Value => {
                suggestions.extend(self.complete_value(&context, app_state).await?);
            }

            CompletionType::FilePath => {
                suggestions.extend(self.complete_file_path(&context.prefix));
            }
        }

        // Sort suggestions by priority and relevance
        suggestions.sort_by(|a, b| {
            b.priority
                .cmp(&a.priority)
                .then_with(|| a.text.len().cmp(&b.text.len()))
        });

        Ok(suggestions)
    }

    fn register_built_in_commands(&mut self) {
        let commands = vec![
            ("ps", "List running dataflows", vec![], vec!["--all", "-a"]),
            (
                "start",
                "Start a dataflow",
                vec![],
                vec!["--debug", "--working-dir"],
            ),
            ("stop", "Stop a dataflow", vec![], vec!["--force", "-f"]),
            ("build", "Build a dataflow", vec![], vec!["--validate"]),
            (
                "logs",
                "View dataflow logs",
                vec![],
                vec!["--follow", "-f", "--tail"],
            ),
            ("inspect", "Inspect a resource", vec![], vec!["--deep"]),
            ("debug", "Debug a dataflow", vec![], vec!["--auto"]),
            (
                "monitor",
                "Monitor system resources",
                vec![],
                vec!["--interval"],
            ),
            (
                "config",
                "Manage configuration",
                vec!["get", "set", "list"],
                vec![],
            ),
            ("ui", "Launch TUI interface", vec![], vec!["--view"]),
            ("help", "Show help information", vec![], vec![]),
        ];

        for (name, desc, subcmds, flags) in commands {
            self.command_registry.insert(
                name.to_string(),
                CommandInfo {
                    name: name.to_string(),
                    description: desc.to_string(),
                    subcommands: subcmds.into_iter().map(String::from).collect(),
                    common_flags: flags.into_iter().map(String::from).collect(),
                },
            );
        }
    }

    fn analyze_completion_context(&self, input: &str, cursor_position: usize) -> CompletionContext {
        let before_cursor = &input[..cursor_position.min(input.len())];
        let parts: Vec<&str> = before_cursor.split_whitespace().collect();
        let is_new_token = before_cursor
            .chars()
            .last()
            .map(|c| c.is_whitespace())
            .unwrap_or(false)
            || before_cursor.is_empty();

        let prefix = if is_new_token {
            String::new()
        } else {
            let suffix: String = before_cursor
                .chars()
                .rev()
                .take_while(|c| !c.is_whitespace())
                .collect();
            suffix.chars().rev().collect()
        };

        if parts.is_empty() || (parts.len() == 1 && !before_cursor.ends_with(' ')) {
            // Completing command name
            CompletionContext {
                completion_type: CompletionType::Command,
                prefix,
                current_command: None,
                parent_command: None,
                position_in_command: 0,
            }
        } else if parts.len() == 1 {
            // After command, might be subcommand
            let cmd = parts[0];
            if let Some(info) = self.command_registry.get(cmd) {
                if !info.subcommands.is_empty() {
                    return CompletionContext {
                        completion_type: CompletionType::Subcommand,
                        prefix,
                        current_command: Some(cmd.to_string()),
                        parent_command: Some(cmd.to_string()),
                        position_in_command: 1,
                    };
                }
            }

            // Could be flag or value
            if prefix.starts_with('-') {
                CompletionContext {
                    completion_type: CompletionType::Flag,
                    prefix,
                    current_command: Some(cmd.to_string()),
                    parent_command: None,
                    position_in_command: 1,
                }
            } else {
                CompletionContext {
                    completion_type: CompletionType::Value,
                    prefix,
                    current_command: Some(cmd.to_string()),
                    parent_command: None,
                    position_in_command: 1,
                }
            }
        } else {
            // In arguments/flags
            let cmd = parts[0];
            if prefix.starts_with('-') {
                CompletionContext {
                    completion_type: CompletionType::Flag,
                    prefix,
                    current_command: Some(cmd.to_string()),
                    parent_command: None,
                    position_in_command: parts.len() - 1,
                }
            } else {
                CompletionContext {
                    completion_type: CompletionType::Value,
                    prefix,
                    current_command: Some(cmd.to_string()),
                    parent_command: None,
                    position_in_command: parts.len() - 1,
                }
            }
        }
    }

    fn complete_command(&self, prefix: &str) -> Vec<CommandSuggestion> {
        self.command_registry
            .values()
            .filter(|info| info.name.starts_with(prefix))
            .map(|info| CommandSuggestion {
                text: info.name.clone(),
                description: info.description.clone(),
                suggestion_type: SuggestionType::Command,
                priority: if info.name == prefix { 100 } else { 80 },
            })
            .collect()
    }

    fn complete_subcommand(&self, parent_cmd: &str, prefix: &str) -> Vec<CommandSuggestion> {
        if let Some(info) = self.command_registry.get(parent_cmd) {
            info.subcommands
                .iter()
                .filter(|subcmd| subcmd.starts_with(prefix))
                .map(|subcmd| CommandSuggestion {
                    text: subcmd.clone(),
                    description: format!("{} subcommand", parent_cmd),
                    suggestion_type: SuggestionType::Command,
                    priority: 85,
                })
                .collect()
        } else {
            Vec::new()
        }
    }

    fn complete_flag(&self, cmd: &str, prefix: &str) -> Vec<CommandSuggestion> {
        if let Some(info) = self.command_registry.get(cmd) {
            info.common_flags
                .iter()
                .filter(|flag| flag.starts_with(prefix))
                .map(|flag| CommandSuggestion {
                    text: flag.clone(),
                    description: format!("{} flag", cmd),
                    suggestion_type: SuggestionType::Flag,
                    priority: 75,
                })
                .collect()
        } else {
            Vec::new()
        }
    }

    async fn complete_value(
        &self,
        context: &CompletionContext,
        app_state: &AppState,
    ) -> Result<Vec<CommandSuggestion>> {
        let mut suggestions = Vec::new();

        if let Some(cmd) = &context.current_command {
            match cmd.as_str() {
                "start" | "stop" | "logs" => {
                    // Complete with dataflow names
                    for dataflow in &app_state.dataflows {
                        if dataflow.name.starts_with(&context.prefix) {
                            suggestions.push(CommandSuggestion {
                                text: dataflow.name.clone(),
                                description: format!("Dataflow ({})", dataflow.status),
                                suggestion_type: SuggestionType::DataflowName,
                                priority: 90,
                            });
                        }
                    }
                }

                "inspect" => {
                    // Complete with dataflow names
                    for dataflow in &app_state.dataflows {
                        if dataflow.name.starts_with(&context.prefix) {
                            suggestions.push(CommandSuggestion {
                                text: dataflow.name.clone(),
                                description: format!("Dataflow ({})", dataflow.status),
                                suggestion_type: SuggestionType::DataflowName,
                                priority: 90,
                            });
                        }
                    }

                    // Also complete with node names
                    for dataflow in &app_state.dataflows {
                        for node in &dataflow.nodes {
                            if node.name.starts_with(&context.prefix) {
                                suggestions.push(CommandSuggestion {
                                    text: node.name.clone(),
                                    description: format!(
                                        "Node in {} ({})",
                                        dataflow.name, node.status
                                    ),
                                    suggestion_type: SuggestionType::NodeName,
                                    priority: 85,
                                });
                            }
                        }
                    }
                }

                _ => {}
            }
        }

        Ok(suggestions)
    }

    fn complete_file_path(&self, prefix: &str) -> Vec<CommandSuggestion> {
        // Simple file path completion - in a real implementation,
        // this would scan the filesystem
        if prefix.ends_with(".yaml") || prefix.ends_with(".yml") {
            vec![CommandSuggestion {
                text: prefix.to_string(),
                description: "YAML file".to_string(),
                suggestion_type: SuggestionType::FilePath,
                priority: 70,
            }]
        } else {
            Vec::new()
        }
    }
}

#[derive(Debug)]
struct CompletionContext {
    completion_type: CompletionType,
    prefix: String,
    current_command: Option<String>,
    parent_command: Option<String>,
    position_in_command: usize,
}

impl Default for CommandModeManager {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod basic_tests {
    use super::*;
    use crate::tui::app::AppState;

    #[test]
    fn test_command_mode_activation() {
        let mut manager = CommandModeManager::new();
        assert!(!manager.is_active());

        manager.activate();
        assert!(manager.is_active());

        manager.deactivate();
        assert!(!manager.is_active());
    }

    #[test]
    fn test_history_navigation() {
        let mut nav = HistoryNavigation::new();
        let history = vec![
            "ps".to_string(),
            "start test.yaml".to_string(),
            "logs test".to_string(),
        ];

        // Navigate up from empty
        assert_eq!(nav.navigate_up(&history), Some("logs test".to_string()));
        assert_eq!(
            nav.navigate_up(&history),
            Some("start test.yaml".to_string())
        );
        assert_eq!(nav.navigate_up(&history), Some("ps".to_string()));
        assert_eq!(nav.navigate_up(&history), None); // At oldest

        // Navigate down
        assert_eq!(
            nav.navigate_down(&history),
            Some("start test.yaml".to_string())
        );
        assert_eq!(nav.navigate_down(&history), Some("logs test".to_string()));
    }

    #[test]
    fn test_completion_state() {
        let mut state = CompletionState::new();

        state.suggestions = vec![
            CommandSuggestion {
                text: "ps".to_string(),
                description: "List dataflows".to_string(),
                suggestion_type: SuggestionType::Command,
                priority: 80,
            },
            CommandSuggestion {
                text: "push".to_string(),
                description: "Push data".to_string(),
                suggestion_type: SuggestionType::Command,
                priority: 70,
            },
        ];

        // Test selection
        state.select_next();
        assert_eq!(state.selected_index, Some(0));

        state.select_next();
        assert_eq!(state.selected_index, Some(1));

        state.select_next(); // Should wrap around
        assert_eq!(state.selected_index, Some(0));

        state.select_previous();
        assert_eq!(state.selected_index, Some(1));
    }

    #[tokio::test]
    async fn test_completion_engine() {
        let engine = CompletionEngine::new();
        let app_state = AppState::default();

        let suggestions = engine.get_completions("p", 1, &app_state).await.unwrap();
        assert!(suggestions.iter().any(|s| s.text == "ps"));

        let suggestions = engine
            .get_completions("config ", 7, &app_state)
            .await
            .unwrap();
        assert!(suggestions.iter().any(|s| s.text == "get"));
        assert!(suggestions.iter().any(|s| s.text == "set"));
    }

    #[test]
    fn test_command_parsing() {
        let manager = CommandModeManager::new();

        if let Some(CommandModeAction::SwitchView(ViewType::Dashboard)) =
            manager.parse_tui_command("dashboard")
        {
            // Expected
        } else {
            panic!("Expected dashboard view switch");
        }

        assert!(manager.parse_tui_command("invalid_command").is_none());
    }
}
