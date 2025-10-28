/// Types and state management for Debug Session View (Issue #29 - Phase 1)
use std::time::Instant;

/// Execution state of the debug session
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ExecutionState {
    Running,
    Paused,
    Stopped,
}

impl ExecutionState {
    /// Get all execution states
    pub fn all() -> Vec<ExecutionState> {
        vec![
            ExecutionState::Running,
            ExecutionState::Paused,
            ExecutionState::Stopped,
        ]
    }

    /// Get the display name for this state
    pub fn name(&self) -> &str {
        match self {
            ExecutionState::Running => "Running",
            ExecutionState::Paused => "Paused",
            ExecutionState::Stopped => "Stopped",
        }
    }

    /// Get color indicator for this state
    pub fn color_code(&self) -> &str {
        match self {
            ExecutionState::Running => "green",
            ExecutionState::Paused => "yellow",
            ExecutionState::Stopped => "red",
        }
    }

    /// Get symbol for this state
    pub fn symbol(&self) -> &str {
        match self {
            ExecutionState::Running => "▶",
            ExecutionState::Paused => "⏸",
            ExecutionState::Stopped => "⏹",
        }
    }
}

/// Debug panel types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DebugPanel {
    Variables,
    CallStack,
    Breakpoints,
}

impl DebugPanel {
    /// Get all debug panels
    pub fn all() -> Vec<DebugPanel> {
        vec![
            DebugPanel::Variables,
            DebugPanel::CallStack,
            DebugPanel::Breakpoints,
        ]
    }

    /// Get the display name for this panel
    pub fn name(&self) -> &str {
        match self {
            DebugPanel::Variables => "Variables",
            DebugPanel::CallStack => "Call Stack",
            DebugPanel::Breakpoints => "Breakpoints",
        }
    }

    /// Get the shortcut key for this panel
    pub fn shortcut(&self) -> &str {
        match self {
            DebugPanel::Variables => "1",
            DebugPanel::CallStack => "2",
            DebugPanel::Breakpoints => "3",
        }
    }

    /// Get the next panel
    pub fn next(&self) -> Self {
        match self {
            DebugPanel::Variables => DebugPanel::CallStack,
            DebugPanel::CallStack => DebugPanel::Breakpoints,
            DebugPanel::Breakpoints => DebugPanel::Variables,
        }
    }

    /// Get the previous panel
    pub fn prev(&self) -> Self {
        match self {
            DebugPanel::Variables => DebugPanel::Breakpoints,
            DebugPanel::CallStack => DebugPanel::Variables,
            DebugPanel::Breakpoints => DebugPanel::CallStack,
        }
    }
}

/// A variable in the debug session
#[derive(Debug, Clone, PartialEq)]
pub struct Variable {
    pub name: String,
    pub value: String,
    pub var_type: String,
    pub scope: String,
}

impl Variable {
    /// Create a new variable
    pub fn new(name: String, value: String, var_type: String, scope: String) -> Self {
        Self {
            name,
            value,
            var_type,
            scope,
        }
    }

    /// Format for display
    pub fn format_display(&self) -> String {
        format!("{}: {} = {}", self.name, self.var_type, self.value)
    }
}

/// A call stack frame
#[derive(Debug, Clone, PartialEq)]
pub struct CallStackFrame {
    pub function_name: String,
    pub file_name: String,
    pub line_number: usize,
}

impl CallStackFrame {
    /// Create a new call stack frame
    pub fn new(function_name: String, file_name: String, line_number: usize) -> Self {
        Self {
            function_name,
            file_name,
            line_number,
        }
    }

    /// Format for display
    pub fn format_display(&self) -> String {
        format!(
            "{} ({}:{})",
            self.function_name, self.file_name, self.line_number
        )
    }
}

/// A breakpoint
#[derive(Debug, Clone, PartialEq)]
pub struct Breakpoint {
    pub id: usize,
    pub file_name: String,
    pub line_number: usize,
    pub enabled: bool,
    pub hit_count: usize,
}

impl Breakpoint {
    /// Create a new breakpoint
    pub fn new(id: usize, file_name: String, line_number: usize) -> Self {
        Self {
            id,
            file_name,
            line_number,
            enabled: true,
            hit_count: 0,
        }
    }

    /// Toggle enabled state
    pub fn toggle_enabled(&mut self) {
        self.enabled = !self.enabled;
    }

    /// Format for display
    pub fn format_display(&self) -> String {
        let status = if self.enabled { "●" } else { "○" };
        format!(
            "{} {}:{} (hits: {})",
            status, self.file_name, self.line_number, self.hit_count
        )
    }
}

/// Code location in the debug session
#[derive(Debug, Clone, PartialEq)]
pub struct CodeLocation {
    pub file_name: String,
    pub line_number: usize,
    pub function_name: String,
}

impl CodeLocation {
    /// Create a new code location
    pub fn new(file_name: String, line_number: usize, function_name: String) -> Self {
        Self {
            file_name,
            line_number,
            function_name,
        }
    }

    /// Format for display
    pub fn format_display(&self) -> String {
        format!(
            "{} - {}:{}",
            self.function_name, self.file_name, self.line_number
        )
    }
}

/// Debug session state
#[derive(Debug)]
pub struct DebugSessionState {
    pub target_name: String,
    pub execution_state: ExecutionState,
    pub current_location: CodeLocation,
    pub active_panel: DebugPanel,
    pub variables: Vec<Variable>,
    pub call_stack: Vec<CallStackFrame>,
    pub breakpoints: Vec<Breakpoint>,
    pub selected_variable: usize,
    pub selected_frame: usize,
    pub selected_breakpoint: usize,
    pub last_refresh: Instant,
}

impl DebugSessionState {
    /// Create a new debug session state
    pub fn new(target_name: String) -> Self {
        Self {
            target_name,
            execution_state: ExecutionState::Stopped,
            current_location: CodeLocation::new("main.rs".to_string(), 1, "main".to_string()),
            active_panel: DebugPanel::Variables,
            variables: Vec::new(),
            call_stack: Vec::new(),
            breakpoints: Vec::new(),
            selected_variable: 0,
            selected_frame: 0,
            selected_breakpoint: 0,
            last_refresh: Instant::now(),
        }
    }

    /// Switch to a specific panel
    pub fn switch_panel(&mut self, panel: DebugPanel) {
        self.active_panel = panel;
    }

    /// Navigate to next panel
    pub fn next_panel(&mut self) {
        self.active_panel = self.active_panel.next();
    }

    /// Navigate to previous panel
    pub fn prev_panel(&mut self) {
        self.active_panel = self.active_panel.prev();
    }

    /// Set execution state
    pub fn set_execution_state(&mut self, state: ExecutionState) {
        self.execution_state = state;
    }

    /// Add a variable
    pub fn add_variable(&mut self, variable: Variable) {
        self.variables.push(variable);
    }

    /// Add a call stack frame
    pub fn add_call_frame(&mut self, frame: CallStackFrame) {
        self.call_stack.push(frame);
    }

    /// Add a breakpoint
    pub fn add_breakpoint(&mut self, breakpoint: Breakpoint) {
        self.breakpoints.push(breakpoint);
    }

    /// Remove a breakpoint by id
    pub fn remove_breakpoint(&mut self, id: usize) {
        self.breakpoints.retain(|bp| bp.id != id);
    }

    /// Toggle breakpoint enabled state
    pub fn toggle_breakpoint(&mut self, id: usize) {
        if let Some(bp) = self.breakpoints.iter_mut().find(|bp| bp.id == id) {
            bp.toggle_enabled();
        }
    }

    /// Navigate selection in current panel
    pub fn navigate_up(&mut self) {
        match self.active_panel {
            DebugPanel::Variables => {
                if self.selected_variable > 0 {
                    self.selected_variable -= 1;
                }
            }
            DebugPanel::CallStack => {
                if self.selected_frame > 0 {
                    self.selected_frame -= 1;
                }
            }
            DebugPanel::Breakpoints => {
                if self.selected_breakpoint > 0 {
                    self.selected_breakpoint -= 1;
                }
            }
        }
    }

    /// Navigate selection down in current panel
    pub fn navigate_down(&mut self) {
        match self.active_panel {
            DebugPanel::Variables => {
                if self.selected_variable + 1 < self.variables.len() {
                    self.selected_variable += 1;
                }
            }
            DebugPanel::CallStack => {
                if self.selected_frame + 1 < self.call_stack.len() {
                    self.selected_frame += 1;
                }
            }
            DebugPanel::Breakpoints => {
                if self.selected_breakpoint + 1 < self.breakpoints.len() {
                    self.selected_breakpoint += 1;
                }
            }
        }
    }

    /// Get selected item in current panel
    pub fn get_selected_item(&self) -> Option<String> {
        match self.active_panel {
            DebugPanel::Variables => self
                .variables
                .get(self.selected_variable)
                .map(|v| v.format_display()),
            DebugPanel::CallStack => self
                .call_stack
                .get(self.selected_frame)
                .map(|f| f.format_display()),
            DebugPanel::Breakpoints => self
                .breakpoints
                .get(self.selected_breakpoint)
                .map(|b| b.format_display()),
        }
    }

    /// Clear all debug data
    pub fn clear(&mut self) {
        self.variables.clear();
        self.call_stack.clear();
        self.breakpoints.clear();
        self.selected_variable = 0;
        self.selected_frame = 0;
        self.selected_breakpoint = 0;
    }

    /// Mark as refreshed
    pub fn mark_refreshed(&mut self) {
        self.last_refresh = Instant::now();
    }

    /// Get count of items in current panel
    pub fn current_panel_count(&self) -> usize {
        match self.active_panel {
            DebugPanel::Variables => self.variables.len(),
            DebugPanel::CallStack => self.call_stack.len(),
            DebugPanel::Breakpoints => self.breakpoints.len(),
        }
    }
}

impl Default for DebugSessionState {
    fn default() -> Self {
        Self::new("default".to_string())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_execution_state_all() {
        let states = ExecutionState::all();
        assert_eq!(states.len(), 3);
        assert_eq!(states[0], ExecutionState::Running);
        assert_eq!(states[1], ExecutionState::Paused);
        assert_eq!(states[2], ExecutionState::Stopped);
    }

    #[test]
    fn test_execution_state_names() {
        assert_eq!(ExecutionState::Running.name(), "Running");
        assert_eq!(ExecutionState::Paused.name(), "Paused");
        assert_eq!(ExecutionState::Stopped.name(), "Stopped");
    }

    #[test]
    fn test_execution_state_symbols() {
        assert_eq!(ExecutionState::Running.symbol(), "▶");
        assert_eq!(ExecutionState::Paused.symbol(), "⏸");
        assert_eq!(ExecutionState::Stopped.symbol(), "⏹");
    }

    #[test]
    fn test_debug_panel_all() {
        let panels = DebugPanel::all();
        assert_eq!(panels.len(), 3);
        assert_eq!(panels[0], DebugPanel::Variables);
        assert_eq!(panels[1], DebugPanel::CallStack);
        assert_eq!(panels[2], DebugPanel::Breakpoints);
    }

    #[test]
    fn test_debug_panel_navigation() {
        let mut panel = DebugPanel::Variables;

        panel = panel.next();
        assert_eq!(panel, DebugPanel::CallStack);

        panel = panel.next();
        assert_eq!(panel, DebugPanel::Breakpoints);

        panel = panel.next();
        assert_eq!(panel, DebugPanel::Variables);

        panel = panel.prev();
        assert_eq!(panel, DebugPanel::Breakpoints);
    }

    #[test]
    fn test_variable_creation() {
        let var = Variable::new(
            "count".to_string(),
            "42".to_string(),
            "i32".to_string(),
            "local".to_string(),
        );

        assert_eq!(var.name, "count");
        assert_eq!(var.value, "42");
        assert_eq!(var.var_type, "i32");
        assert_eq!(var.scope, "local");
    }

    #[test]
    fn test_variable_format_display() {
        let var = Variable::new(
            "count".to_string(),
            "42".to_string(),
            "i32".to_string(),
            "local".to_string(),
        );

        assert_eq!(var.format_display(), "count: i32 = 42");
    }

    #[test]
    fn test_call_stack_frame_creation() {
        let frame = CallStackFrame::new("process_data".to_string(), "processor.rs".to_string(), 42);

        assert_eq!(frame.function_name, "process_data");
        assert_eq!(frame.file_name, "processor.rs");
        assert_eq!(frame.line_number, 42);
    }

    #[test]
    fn test_breakpoint_creation() {
        let bp = Breakpoint::new(1, "main.rs".to_string(), 10);

        assert_eq!(bp.id, 1);
        assert_eq!(bp.file_name, "main.rs");
        assert_eq!(bp.line_number, 10);
        assert!(bp.enabled);
        assert_eq!(bp.hit_count, 0);
    }

    #[test]
    fn test_breakpoint_toggle() {
        let mut bp = Breakpoint::new(1, "main.rs".to_string(), 10);

        assert!(bp.enabled);
        bp.toggle_enabled();
        assert!(!bp.enabled);
        bp.toggle_enabled();
        assert!(bp.enabled);
    }

    #[test]
    fn test_code_location_creation() {
        let loc = CodeLocation::new("main.rs".to_string(), 42, "main".to_string());

        assert_eq!(loc.file_name, "main.rs");
        assert_eq!(loc.line_number, 42);
        assert_eq!(loc.function_name, "main");
    }

    #[test]
    fn test_debug_session_state_new() {
        let state = DebugSessionState::new("test-target".to_string());

        assert_eq!(state.target_name, "test-target");
        assert_eq!(state.execution_state, ExecutionState::Stopped);
        assert_eq!(state.active_panel, DebugPanel::Variables);
        assert_eq!(state.variables.len(), 0);
        assert_eq!(state.call_stack.len(), 0);
        assert_eq!(state.breakpoints.len(), 0);
    }

    #[test]
    fn test_debug_session_panel_navigation() {
        let mut state = DebugSessionState::new("test".to_string());

        assert_eq!(state.active_panel, DebugPanel::Variables);

        state.next_panel();
        assert_eq!(state.active_panel, DebugPanel::CallStack);

        state.next_panel();
        assert_eq!(state.active_panel, DebugPanel::Breakpoints);

        state.prev_panel();
        assert_eq!(state.active_panel, DebugPanel::CallStack);
    }

    #[test]
    fn test_debug_session_add_variable() {
        let mut state = DebugSessionState::new("test".to_string());
        let var = Variable::new(
            "x".to_string(),
            "10".to_string(),
            "i32".to_string(),
            "local".to_string(),
        );

        state.add_variable(var);
        assert_eq!(state.variables.len(), 1);
    }

    #[test]
    fn test_debug_session_add_breakpoint() {
        let mut state = DebugSessionState::new("test".to_string());
        let bp = Breakpoint::new(1, "main.rs".to_string(), 10);

        state.add_breakpoint(bp);
        assert_eq!(state.breakpoints.len(), 1);
    }

    #[test]
    fn test_debug_session_remove_breakpoint() {
        let mut state = DebugSessionState::new("test".to_string());
        state.add_breakpoint(Breakpoint::new(1, "main.rs".to_string(), 10));
        state.add_breakpoint(Breakpoint::new(2, "lib.rs".to_string(), 20));

        assert_eq!(state.breakpoints.len(), 2);

        state.remove_breakpoint(1);
        assert_eq!(state.breakpoints.len(), 1);
        assert_eq!(state.breakpoints[0].id, 2);
    }

    #[test]
    fn test_debug_session_toggle_breakpoint() {
        let mut state = DebugSessionState::new("test".to_string());
        state.add_breakpoint(Breakpoint::new(1, "main.rs".to_string(), 10));

        assert!(state.breakpoints[0].enabled);

        state.toggle_breakpoint(1);
        assert!(!state.breakpoints[0].enabled);

        state.toggle_breakpoint(1);
        assert!(state.breakpoints[0].enabled);
    }

    #[test]
    fn test_debug_session_navigation() {
        let mut state = DebugSessionState::new("test".to_string());

        // Add test data
        state.add_variable(Variable::new(
            "a".to_string(),
            "1".to_string(),
            "i32".to_string(),
            "local".to_string(),
        ));
        state.add_variable(Variable::new(
            "b".to_string(),
            "2".to_string(),
            "i32".to_string(),
            "local".to_string(),
        ));
        state.add_variable(Variable::new(
            "c".to_string(),
            "3".to_string(),
            "i32".to_string(),
            "local".to_string(),
        ));

        assert_eq!(state.selected_variable, 0);

        state.navigate_down();
        assert_eq!(state.selected_variable, 1);

        state.navigate_down();
        assert_eq!(state.selected_variable, 2);

        // Should not go beyond bounds
        state.navigate_down();
        assert_eq!(state.selected_variable, 2);

        state.navigate_up();
        assert_eq!(state.selected_variable, 1);

        state.navigate_up();
        assert_eq!(state.selected_variable, 0);

        // Should not go below 0
        state.navigate_up();
        assert_eq!(state.selected_variable, 0);
    }

    #[test]
    fn test_debug_session_clear() {
        let mut state = DebugSessionState::new("test".to_string());

        state.add_variable(Variable::new(
            "x".to_string(),
            "10".to_string(),
            "i32".to_string(),
            "local".to_string(),
        ));
        state.add_breakpoint(Breakpoint::new(1, "main.rs".to_string(), 10));

        assert_eq!(state.variables.len(), 1);
        assert_eq!(state.breakpoints.len(), 1);

        state.clear();

        assert_eq!(state.variables.len(), 0);
        assert_eq!(state.breakpoints.len(), 0);
        assert_eq!(state.selected_variable, 0);
        assert_eq!(state.selected_breakpoint, 0);
    }

    #[test]
    fn test_debug_session_current_panel_count() {
        let mut state = DebugSessionState::new("test".to_string());

        state.add_variable(Variable::new(
            "x".to_string(),
            "10".to_string(),
            "i32".to_string(),
            "local".to_string(),
        ));
        state.add_variable(Variable::new(
            "y".to_string(),
            "20".to_string(),
            "i32".to_string(),
            "local".to_string(),
        ));

        assert_eq!(state.current_panel_count(), 2);

        state.switch_panel(DebugPanel::Breakpoints);
        assert_eq!(state.current_panel_count(), 0);

        state.add_breakpoint(Breakpoint::new(1, "main.rs".to_string(), 10));
        assert_eq!(state.current_panel_count(), 1);
    }
}
