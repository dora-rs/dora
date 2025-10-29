/// Type definitions for Node Inspector View (Issue #27)
use std::time::Instant;

/// Tab variants for Node Inspector
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InspectorTab {
    Overview,
    Connections,
    Performance,
    Configuration,
    Debug,
}

impl InspectorTab {
    /// Get all available tabs
    pub fn all() -> Vec<InspectorTab> {
        vec![
            InspectorTab::Overview,
            InspectorTab::Connections,
            InspectorTab::Performance,
            InspectorTab::Configuration,
            InspectorTab::Debug,
        ]
    }

    /// Get the display name for this tab
    pub fn name(&self) -> &str {
        match self {
            InspectorTab::Overview => "Overview",
            InspectorTab::Connections => "Connections",
            InspectorTab::Performance => "Performance",
            InspectorTab::Configuration => "Configuration",
            InspectorTab::Debug => "Debug",
        }
    }

    /// Get the keyboard shortcut for this tab
    pub fn shortcut(&self) -> &str {
        match self {
            InspectorTab::Overview => "1",
            InspectorTab::Connections => "2",
            InspectorTab::Performance => "3",
            InspectorTab::Configuration => "4",
            InspectorTab::Debug => "5",
        }
    }

    /// Get the next tab in the sequence
    pub fn next(&self) -> InspectorTab {
        match self {
            InspectorTab::Overview => InspectorTab::Connections,
            InspectorTab::Connections => InspectorTab::Performance,
            InspectorTab::Performance => InspectorTab::Configuration,
            InspectorTab::Configuration => InspectorTab::Debug,
            InspectorTab::Debug => InspectorTab::Overview,
        }
    }

    /// Get the previous tab in the sequence
    pub fn prev(&self) -> InspectorTab {
        match self {
            InspectorTab::Overview => InspectorTab::Debug,
            InspectorTab::Connections => InspectorTab::Overview,
            InspectorTab::Performance => InspectorTab::Connections,
            InspectorTab::Configuration => InspectorTab::Performance,
            InspectorTab::Debug => InspectorTab::Configuration,
        }
    }
}

/// State for the Node Inspector view
#[derive(Debug, Clone)]
pub struct NodeInspectorState {
    /// Currently active tab
    pub active_tab: InspectorTab,

    /// ID of the node being inspected
    pub node_id: String,
    /// ID of the parent dataflow
    pub dataflow_id: String,

    /// Selected input index (for Connections tab)
    pub selected_input: usize,

    /// Selected output index (for Connections tab)
    pub selected_output: usize,

    /// Whether to show detailed metrics
    pub show_detailed_metrics: bool,

    /// Last refresh timestamp
    pub last_refresh: Instant,

    /// Scroll offset for scrollable content
    pub scroll_offset: usize,

    /// Whether the view is in edit mode (for configuration)
    pub edit_mode: bool,
}

impl NodeInspectorState {
    /// Create a new NodeInspectorState for the given node
    pub fn new(dataflow_id: String, node_id: String) -> Self {
        Self {
            active_tab: InspectorTab::Overview,
            node_id,
            dataflow_id,
            selected_input: 0,
            selected_output: 0,
            show_detailed_metrics: false,
            last_refresh: Instant::now(),
            scroll_offset: 0,
            edit_mode: false,
        }
    }

    /// Switch to a specific tab
    pub fn switch_tab(&mut self, tab: InspectorTab) {
        self.active_tab = tab;
        // Reset scroll when switching tabs
        self.scroll_offset = 0;
    }

    /// Navigate to the next tab
    pub fn next_tab(&mut self) {
        self.active_tab = self.active_tab.next();
        self.scroll_offset = 0;
    }

    /// Navigate to the previous tab
    pub fn prev_tab(&mut self) {
        self.active_tab = self.active_tab.prev();
        self.scroll_offset = 0;
    }

    /// Select next input (for Connections tab)
    pub fn select_next_input(&mut self, max_inputs: usize) {
        if max_inputs > 0 {
            self.selected_input = (self.selected_input + 1) % max_inputs;
        }
    }

    /// Select previous input (for Connections tab)
    pub fn select_prev_input(&mut self, max_inputs: usize) {
        if max_inputs > 0 {
            self.selected_input = if self.selected_input == 0 {
                max_inputs - 1
            } else {
                self.selected_input - 1
            };
        }
    }

    /// Select next output (for Connections tab)
    pub fn select_next_output(&mut self, max_outputs: usize) {
        if max_outputs > 0 {
            self.selected_output = (self.selected_output + 1) % max_outputs;
        }
    }

    /// Select previous output (for Connections tab)
    pub fn select_prev_output(&mut self, max_outputs: usize) {
        if max_outputs > 0 {
            self.selected_output = if self.selected_output == 0 {
                max_outputs - 1
            } else {
                self.selected_output - 1
            };
        }
    }

    /// Toggle detailed metrics display
    pub fn toggle_detailed_metrics(&mut self) {
        self.show_detailed_metrics = !self.show_detailed_metrics;
    }

    /// Scroll content down
    pub fn scroll_down(&mut self, max_scroll: usize) {
        if self.scroll_offset < max_scroll {
            self.scroll_offset += 1;
        }
    }

    /// Scroll content up
    pub fn scroll_up(&mut self) {
        if self.scroll_offset > 0 {
            self.scroll_offset -= 1;
        }
    }

    /// Toggle edit mode
    pub fn toggle_edit_mode(&mut self) {
        self.edit_mode = !self.edit_mode;
    }

    /// Mark as refreshed
    pub fn mark_refreshed(&mut self) {
        self.last_refresh = Instant::now();
    }
}

impl Default for NodeInspectorState {
    fn default() -> Self {
        Self::new(String::new(), String::new())
    }
}

/// Connection information for a node
#[derive(Debug, Clone)]
pub struct ConnectionInfo {
    pub name: String,
    pub conn_type: String, // "input" or "output"
    pub data_type: String,
    pub connected_to: Vec<String>,
    pub message_count: u64,
    pub bytes_transferred: u64,
    pub last_message_time: Option<Instant>,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tui::app::NodeMetrics;

    #[test]
    fn test_inspector_tab_all_returns_five_tabs() {
        let tabs = InspectorTab::all();
        assert_eq!(tabs.len(), 5);
        assert_eq!(tabs[0], InspectorTab::Overview);
        assert_eq!(tabs[1], InspectorTab::Connections);
        assert_eq!(tabs[2], InspectorTab::Performance);
        assert_eq!(tabs[3], InspectorTab::Configuration);
        assert_eq!(tabs[4], InspectorTab::Debug);
    }

    #[test]
    fn test_inspector_tab_names() {
        assert_eq!(InspectorTab::Overview.name(), "Overview");
        assert_eq!(InspectorTab::Connections.name(), "Connections");
        assert_eq!(InspectorTab::Performance.name(), "Performance");
        assert_eq!(InspectorTab::Configuration.name(), "Configuration");
        assert_eq!(InspectorTab::Debug.name(), "Debug");
    }

    #[test]
    fn test_inspector_tab_shortcuts() {
        assert_eq!(InspectorTab::Overview.shortcut(), "1");
        assert_eq!(InspectorTab::Connections.shortcut(), "2");
        assert_eq!(InspectorTab::Performance.shortcut(), "3");
        assert_eq!(InspectorTab::Configuration.shortcut(), "4");
        assert_eq!(InspectorTab::Debug.shortcut(), "5");
    }

    #[test]
    fn test_inspector_tab_navigation() {
        let mut tab = InspectorTab::Overview;

        // Test next()
        tab = tab.next();
        assert_eq!(tab, InspectorTab::Connections);

        tab = tab.next();
        assert_eq!(tab, InspectorTab::Performance);

        tab = tab.next();
        assert_eq!(tab, InspectorTab::Configuration);

        tab = tab.next();
        assert_eq!(tab, InspectorTab::Debug);

        // Test wrapping
        tab = tab.next();
        assert_eq!(tab, InspectorTab::Overview);
    }

    #[test]
    fn test_inspector_tab_prev_navigation() {
        let mut tab = InspectorTab::Overview;

        // Test prev() wraps around
        tab = tab.prev();
        assert_eq!(tab, InspectorTab::Debug);

        tab = tab.prev();
        assert_eq!(tab, InspectorTab::Configuration);

        tab = tab.prev();
        assert_eq!(tab, InspectorTab::Performance);
    }

    #[test]
    fn test_node_inspector_state_new() {
        let state = NodeInspectorState::new("df-1".to_string(), "test-node".to_string());

        assert_eq!(state.active_tab, InspectorTab::Overview);
        assert_eq!(state.node_id, "test-node");
        assert_eq!(state.dataflow_id, "df-1");
        assert_eq!(state.selected_input, 0);
        assert_eq!(state.selected_output, 0);
        assert!(!state.show_detailed_metrics);
        assert_eq!(state.scroll_offset, 0);
        assert!(!state.edit_mode);
    }

    #[test]
    fn test_node_inspector_state_switch_tab() {
        let mut state = NodeInspectorState::new("df".to_string(), "test".to_string());

        assert_eq!(state.active_tab, InspectorTab::Overview);

        state.switch_tab(InspectorTab::Connections);
        assert_eq!(state.active_tab, InspectorTab::Connections);

        state.switch_tab(InspectorTab::Performance);
        assert_eq!(state.active_tab, InspectorTab::Performance);
    }

    #[test]
    fn test_node_inspector_state_next_prev_tab() {
        let mut state = NodeInspectorState::new("df".to_string(), "test".to_string());

        state.next_tab();
        assert_eq!(state.active_tab, InspectorTab::Connections);

        state.next_tab();
        assert_eq!(state.active_tab, InspectorTab::Performance);

        state.prev_tab();
        assert_eq!(state.active_tab, InspectorTab::Connections);

        state.prev_tab();
        assert_eq!(state.active_tab, InspectorTab::Overview);
    }

    #[test]
    fn test_node_inspector_state_input_selection() {
        let mut state = NodeInspectorState::new("df".to_string(), "test".to_string());

        // Select next input
        state.select_next_input(3);
        assert_eq!(state.selected_input, 1);

        state.select_next_input(3);
        assert_eq!(state.selected_input, 2);

        // Should wrap
        state.select_next_input(3);
        assert_eq!(state.selected_input, 0);

        // Select previous
        state.select_prev_input(3);
        assert_eq!(state.selected_input, 2);
    }

    #[test]
    fn test_node_inspector_state_output_selection() {
        let mut state = NodeInspectorState::new("df".to_string(), "test".to_string());

        // Select next output
        state.select_next_output(2);
        assert_eq!(state.selected_output, 1);

        // Should wrap
        state.select_next_output(2);
        assert_eq!(state.selected_output, 0);

        // Select previous
        state.select_prev_output(2);
        assert_eq!(state.selected_output, 1);
    }

    #[test]
    fn test_node_inspector_state_toggle_detailed_metrics() {
        let mut state = NodeInspectorState::new("df".to_string(), "test".to_string());

        assert!(!state.show_detailed_metrics);

        state.toggle_detailed_metrics();
        assert!(state.show_detailed_metrics);

        state.toggle_detailed_metrics();
        assert!(!state.show_detailed_metrics);
    }

    #[test]
    fn test_node_inspector_state_scrolling() {
        let mut state = NodeInspectorState::new("df".to_string(), "test".to_string());

        assert_eq!(state.scroll_offset, 0);

        state.scroll_down(10);
        assert_eq!(state.scroll_offset, 1);

        state.scroll_down(10);
        assert_eq!(state.scroll_offset, 2);

        state.scroll_up();
        assert_eq!(state.scroll_offset, 1);

        state.scroll_up();
        assert_eq!(state.scroll_offset, 0);

        // Can't scroll below 0
        state.scroll_up();
        assert_eq!(state.scroll_offset, 0);

        // Can't scroll beyond max
        for _ in 0..20 {
            state.scroll_down(10);
        }
        assert_eq!(state.scroll_offset, 10);
    }

    #[test]
    fn test_node_inspector_state_toggle_edit_mode() {
        let mut state = NodeInspectorState::new("df".to_string(), "test".to_string());

        assert!(!state.edit_mode);

        state.toggle_edit_mode();
        assert!(state.edit_mode);

        state.toggle_edit_mode();
        assert!(!state.edit_mode);
    }

    #[test]
    fn test_node_inspector_state_mark_refreshed() {
        let mut state = NodeInspectorState::new("df".to_string(), "test".to_string());

        let initial_time = state.last_refresh;

        std::thread::sleep(std::time::Duration::from_millis(10));

        state.mark_refreshed();

        assert!(state.last_refresh > initial_time);
    }

    #[test]
    fn test_tab_switching_resets_scroll() {
        let mut state = NodeInspectorState::new("df".to_string(), "test".to_string());

        state.scroll_down(10);
        assert_eq!(state.scroll_offset, 1);

        state.next_tab();
        assert_eq!(state.scroll_offset, 0);

        state.scroll_down(10);
        assert_eq!(state.scroll_offset, 1);

        state.switch_tab(InspectorTab::Overview);
        assert_eq!(state.scroll_offset, 0);
    }

    #[test]
    fn test_node_metrics_default() {
        let metrics = NodeMetrics::default();

        assert_eq!(metrics.cpu_percent, 0.0);
        assert_eq!(metrics.memory_percent, 0.0);
        assert_eq!(metrics.message_rate, 0.0);
        assert_eq!(metrics.processing_latency_ms, 0.0);
        assert_eq!(metrics.uptime_seconds, 0);
        assert_eq!(metrics.error_count, 0);
    }
}
