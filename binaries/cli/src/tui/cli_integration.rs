use std::{
    collections::HashMap,
    path::PathBuf,
};

use crate::cli::{Command, Cli};
use super::app::UserConfig;

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
    
    /// Command line arguments that were passed
    pub cli_args: Vec<String>,
}

impl CliContext {
    pub fn new() -> Self {
        Self {
            original_command: None,
            working_directory: std::env::current_dir().unwrap_or_else(|_| PathBuf::from("/")),
            environment: std::env::vars().collect(),
            user_config: UserConfig::default(),
            cli_args: std::env::args().collect(),
        }
    }
    
    pub fn from_cli(cli: &Cli) -> Self {
        let mut context = Self::new();
        context.original_command = Some(cli.command.clone());
        context
    }
    
    pub fn from_command(command: Command) -> Self {
        let mut context = Self::new();
        context.original_command = Some(command);
        context
    }
    
    pub fn get_env_var(&self, key: &str) -> Option<&String> {
        self.environment.get(key)
    }
    
    pub fn is_in_git_repo(&self) -> bool {
        self.working_directory.join(".git").exists()
    }
    
    pub fn get_relative_path(&self, path: &PathBuf) -> PathBuf {
        path.strip_prefix(&self.working_directory)
            .unwrap_or(path)
            .to_path_buf()
    }
}

impl Default for CliContext {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Debug, Clone)]
pub enum CommandMode {
    Normal,
    Command {
        buffer: String,
        cursor: usize,
        history: Vec<String>,
        history_index: Option<usize>,
    },
    Search {
        query: String,
        cursor: usize,
        results: Vec<SearchResult>,
        selected_result: usize,
    },
    Visual {
        start_line: usize,
        end_line: usize,
    },
}

impl Default for CommandMode {
    fn default() -> Self {
        CommandMode::Normal
    }
}

#[derive(Debug, Clone)]
pub struct SearchResult {
    pub text: String,
    pub line_number: usize,
    pub column: usize,
    pub context: String,
}

/// Command execution results for TUI integration
#[derive(Debug, Clone)]
pub struct CommandResult {
    pub success: bool,
    pub output: String,
    pub error: Option<String>,
    pub execution_time: std::time::Duration,
    pub suggested_actions: Vec<SuggestedAction>,
}

#[derive(Debug, Clone)]
pub struct SuggestedAction {
    pub description: String,
    pub command: String,
    pub category: ActionCategory,
}

#[derive(Debug, Clone)]
pub enum ActionCategory {
    Navigation,
    Inspection,
    Management,
    Debugging,
    Configuration,
}

/// Command history management
#[derive(Debug, Clone)]
pub struct CommandHistory {
    pub commands: Vec<HistoryEntry>,
    pub max_entries: usize,
}

#[derive(Debug, Clone)]
pub struct HistoryEntry {
    pub command: String,
    pub timestamp: std::time::SystemTime,
    pub success: bool,
    pub working_directory: PathBuf,
}

impl CommandHistory {
    pub fn new() -> Self {
        Self {
            commands: Vec::new(),
            max_entries: 1000,
        }
    }
    
    pub fn add_entry(&mut self, command: String, success: bool, working_directory: PathBuf) {
        let entry = HistoryEntry {
            command,
            timestamp: std::time::SystemTime::now(),
            success,
            working_directory,
        };
        
        self.commands.push(entry);
        
        // Keep only the last max_entries commands
        if self.commands.len() > self.max_entries {
            self.commands.remove(0);
        }
    }
    
    pub fn search(&self, query: &str) -> Vec<&HistoryEntry> {
        self.commands
            .iter()
            .filter(|entry| entry.command.contains(query))
            .collect()
    }
    
    pub fn get_recent(&self, count: usize) -> Vec<&HistoryEntry> {
        self.commands
            .iter()
            .rev()
            .take(count)
            .collect()
    }
    
    pub fn get_successful_commands(&self) -> Vec<&HistoryEntry> {
        self.commands
            .iter()
            .filter(|entry| entry.success)
            .collect()
    }
}

impl Default for CommandHistory {
    fn default() -> Self {
        Self::new()
    }
}

/// Tab completion for command mode
#[derive(Debug)]
pub struct TabCompletion {
    pub suggestions: Vec<String>,
    pub current_prefix: String,
    pub selected_index: usize,
}

impl TabCompletion {
    pub fn new() -> Self {
        Self {
            suggestions: Vec::new(),
            current_prefix: String::new(),
            selected_index: 0,
        }
    }
    
    pub fn update_suggestions(&mut self, prefix: &str) {
        self.current_prefix = prefix.to_string();
        self.suggestions = self.get_completions_for_prefix(prefix);
        self.selected_index = 0;
    }
    
    fn get_completions_for_prefix(&self, prefix: &str) -> Vec<String> {
        let mut completions = Vec::new();
        
        // Command completions
        let commands = [
            "ps", "start", "stop", "logs", "build", "up", "destroy", "new",
            "check", "graph", "inspect", "debug", "analyze", "monitor",
            "ui", "dashboard", "system", "config", "daemon", "runtime",
            "coordinator", "self",
        ];
        
        for cmd in commands {
            if cmd.starts_with(prefix) {
                completions.push(cmd.to_string());
            }
        }
        
        // File path completions (simplified)
        if prefix.contains('/') || prefix.contains('.') {
            // Add file path completions here
            // This would involve reading the filesystem
        }
        
        // Flag completions
        if prefix.starts_with("--") {
            let flags = [
                "--help", "--version", "--verbose", "--quiet", "--output",
                "--ui-mode", "--no-hints", "--config", "--log-level",
            ];
            
            for flag in flags {
                if flag.starts_with(prefix) {
                    completions.push(flag.to_string());
                }
            }
        }
        
        completions.sort();
        completions
    }
    
    pub fn next_suggestion(&mut self) {
        if !self.suggestions.is_empty() {
            self.selected_index = (self.selected_index + 1) % self.suggestions.len();
        }
    }
    
    pub fn previous_suggestion(&mut self) {
        if !self.suggestions.is_empty() {
            self.selected_index = if self.selected_index == 0 {
                self.suggestions.len() - 1
            } else {
                self.selected_index - 1
            };
        }
    }
    
    pub fn get_current_suggestion(&self) -> Option<&String> {
        self.suggestions.get(self.selected_index)
    }
    
    pub fn complete_current(&self) -> Option<String> {
        self.get_current_suggestion()
            .map(|suggestion| suggestion.clone())
    }
}

impl Default for TabCompletion {
    fn default() -> Self {
        Self::new()
    }
}

/// Keyboard shortcuts and bindings
#[derive(Debug, Clone)]
pub struct KeyBindings {
    pub global: HashMap<String, String>,
    pub view_specific: HashMap<String, HashMap<String, String>>,
}

impl KeyBindings {
    pub fn default_bindings() -> Self {
        let mut global = HashMap::new();
        global.insert("q".to_string(), "quit".to_string());
        global.insert(":".to_string(), "command_mode".to_string());
        global.insert("Esc".to_string(), "back".to_string());
        global.insert("F1".to_string(), "help".to_string());
        global.insert("Tab".to_string(), "next_widget".to_string());
        global.insert("Shift+Tab".to_string(), "previous_widget".to_string());
        global.insert("Ctrl+c".to_string(), "interrupt".to_string());
        global.insert("Ctrl+l".to_string(), "refresh".to_string());
        
        // View navigation
        global.insert("1".to_string(), "switch_view:dashboard".to_string());
        global.insert("2".to_string(), "switch_view:dataflow".to_string());
        global.insert("3".to_string(), "switch_view:monitor".to_string());
        global.insert("4".to_string(), "switch_view:logs".to_string());
        global.insert("5".to_string(), "switch_view:settings".to_string());
        
        let mut view_specific = HashMap::new();
        
        // Dashboard view specific bindings
        let mut dashboard_bindings = HashMap::new();
        dashboard_bindings.insert("r".to_string(), "refresh_dashboard".to_string());
        dashboard_bindings.insert("s".to_string(), "sort_dataflows".to_string());
        dashboard_bindings.insert("f".to_string(), "filter_dataflows".to_string());
        view_specific.insert("dashboard".to_string(), dashboard_bindings);
        
        // Dataflow manager view specific bindings
        let mut dataflow_bindings = HashMap::new();
        dataflow_bindings.insert("n".to_string(), "new_dataflow".to_string());
        dataflow_bindings.insert("d".to_string(), "delete_dataflow".to_string());
        dataflow_bindings.insert("e".to_string(), "edit_dataflow".to_string());
        dataflow_bindings.insert("Space".to_string(), "toggle_dataflow".to_string());
        view_specific.insert("dataflow".to_string(), dataflow_bindings);
        
        // Log viewer specific bindings
        let mut log_bindings = HashMap::new();
        log_bindings.insert("j".to_string(), "scroll_down".to_string());
        log_bindings.insert("k".to_string(), "scroll_up".to_string());
        log_bindings.insert("G".to_string(), "scroll_to_bottom".to_string());
        log_bindings.insert("g".to_string(), "scroll_to_top".to_string());
        log_bindings.insert("/".to_string(), "search".to_string());
        log_bindings.insert("n".to_string(), "next_search_result".to_string());
        log_bindings.insert("N".to_string(), "previous_search_result".to_string());
        view_specific.insert("logs".to_string(), log_bindings);
        
        Self {
            global,
            view_specific,
        }
    }
    
    pub fn get_binding(&self, key: &str, view: Option<&str>) -> Option<&String> {
        // Check view-specific bindings first
        if let Some(view_name) = view {
            if let Some(view_bindings) = self.view_specific.get(view_name) {
                if let Some(action) = view_bindings.get(key) {
                    return Some(action);
                }
            }
        }
        
        // Fall back to global bindings
        self.global.get(key)
    }
    
    pub fn add_binding(&mut self, key: String, action: String, view: Option<String>) {
        if let Some(view_name) = view {
            self.view_specific
                .entry(view_name)
                .or_insert_with(HashMap::new)
                .insert(key, action);
        } else {
            self.global.insert(key, action);
        }
    }
    
    pub fn remove_binding(&mut self, key: &str, view: Option<&str>) {
        if let Some(view_name) = view {
            if let Some(view_bindings) = self.view_specific.get_mut(view_name) {
                view_bindings.remove(key);
            }
        } else {
            self.global.remove(key);
        }
    }
}

impl Default for KeyBindings {
    fn default() -> Self {
        Self::default_bindings()
    }
}

/// Integration utilities for CLI command execution within TUI
pub struct CliIntegration {
    pub context: CliContext,
    pub history: CommandHistory,
    pub completion: TabCompletion,
    pub key_bindings: KeyBindings,
}

impl CliIntegration {
    pub fn new(context: CliContext) -> Self {
        Self {
            context,
            history: CommandHistory::new(),
            completion: TabCompletion::new(),
            key_bindings: KeyBindings::default_bindings(),
        }
    }
    
    pub async fn execute_command(&mut self, command_str: &str) -> CommandResult {
        let start_time = std::time::Instant::now();
        
        // Parse and execute the command
        // This is a simplified implementation
        let success = !command_str.is_empty();
        let output = format!("Executed: {}", command_str);
        let execution_time = start_time.elapsed();
        
        // Add to history
        self.history.add_entry(
            command_str.to_string(),
            success,
            self.context.working_directory.clone(),
        );
        
        // Generate suggested actions based on the command
        let suggested_actions = self.generate_suggested_actions(command_str);
        
        CommandResult {
            success,
            output,
            error: if success { None } else { Some("Command failed".to_string()) },
            execution_time,
            suggested_actions,
        }
    }
    
    fn generate_suggested_actions(&self, command: &str) -> Vec<SuggestedAction> {
        let mut actions = Vec::new();
        
        match command {
            cmd if cmd.starts_with("ps") => {
                actions.push(SuggestedAction {
                    description: "Inspect a specific dataflow".to_string(),
                    command: "inspect <dataflow_id>".to_string(),
                    category: ActionCategory::Inspection,
                });
                actions.push(SuggestedAction {
                    description: "View logs for a dataflow".to_string(),
                    command: "logs <dataflow_id>".to_string(),
                    category: ActionCategory::Inspection,
                });
            },
            cmd if cmd.starts_with("start") => {
                actions.push(SuggestedAction {
                    description: "Monitor the started dataflow".to_string(),
                    command: "monitor".to_string(),
                    category: ActionCategory::Management,
                });
                actions.push(SuggestedAction {
                    description: "View real-time logs".to_string(),
                    command: "logs --follow".to_string(),
                    category: ActionCategory::Inspection,
                });
            },
            cmd if cmd.starts_with("logs") => {
                actions.push(SuggestedAction {
                    description: "Open log viewer in TUI".to_string(),
                    command: "ui logs".to_string(),
                    category: ActionCategory::Navigation,
                });
                actions.push(SuggestedAction {
                    description: "Filter logs by level".to_string(),
                    command: "logs --level error".to_string(),
                    category: ActionCategory::Inspection,
                });
            },
            _ => {
                actions.push(SuggestedAction {
                    description: "View available commands".to_string(),
                    command: "help".to_string(),
                    category: ActionCategory::Navigation,
                });
            }
        }
        
        actions
    }
    
    pub fn get_command_completions(&mut self, prefix: &str) -> &[String] {
        self.completion.update_suggestions(prefix);
        &self.completion.suggestions
    }
    
    pub fn get_command_history(&self, query: Option<&str>) -> Vec<&HistoryEntry> {
        if let Some(q) = query {
            self.history.search(q)
        } else {
            self.history.get_recent(20)
        }
    }
}

impl Default for CliIntegration {
    fn default() -> Self {
        Self::new(CliContext::default())
    }
}