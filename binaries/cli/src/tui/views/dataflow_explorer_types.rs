/// Dataflow Explorer types for Issue #25
use std::time::Instant;

/// Tab selection in the dataflow explorer
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ExplorerTab {
    Overview,
    Nodes,
    Performance,
    Configuration,
    Logs,
    Debug,
}

impl ExplorerTab {
    pub fn all() -> Vec<ExplorerTab> {
        vec![
            ExplorerTab::Overview,
            ExplorerTab::Nodes,
            ExplorerTab::Performance,
            ExplorerTab::Configuration,
            ExplorerTab::Logs,
            ExplorerTab::Debug,
        ]
    }

    pub fn name(&self) -> &'static str {
        match self {
            ExplorerTab::Overview => "Overview",
            ExplorerTab::Nodes => "Nodes",
            ExplorerTab::Performance => "Performance",
            ExplorerTab::Configuration => "Configuration",
            ExplorerTab::Logs => "Logs",
            ExplorerTab::Debug => "Debug",
        }
    }

    pub fn shortcut(&self) -> &'static str {
        match self {
            ExplorerTab::Overview => "1",
            ExplorerTab::Nodes => "2",
            ExplorerTab::Performance => "3",
            ExplorerTab::Configuration => "4",
            ExplorerTab::Logs => "5",
            ExplorerTab::Debug => "6",
        }
    }

    pub fn next(&self) -> ExplorerTab {
        match self {
            ExplorerTab::Overview => ExplorerTab::Nodes,
            ExplorerTab::Nodes => ExplorerTab::Performance,
            ExplorerTab::Performance => ExplorerTab::Configuration,
            ExplorerTab::Configuration => ExplorerTab::Logs,
            ExplorerTab::Logs => ExplorerTab::Debug,
            ExplorerTab::Debug => ExplorerTab::Overview,
        }
    }

    pub fn prev(&self) -> ExplorerTab {
        match self {
            ExplorerTab::Overview => ExplorerTab::Debug,
            ExplorerTab::Nodes => ExplorerTab::Overview,
            ExplorerTab::Performance => ExplorerTab::Nodes,
            ExplorerTab::Configuration => ExplorerTab::Performance,
            ExplorerTab::Logs => ExplorerTab::Configuration,
            ExplorerTab::Debug => ExplorerTab::Logs,
        }
    }
}

/// View mode for dataflow list display
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ViewMode {
    /// List view with grouping by status
    ListGrouped,
    /// Flat list view
    ListFlat,
    /// ASCII tree showing dependencies
    DependencyTree,
}

impl ViewMode {
    pub fn all() -> Vec<ViewMode> {
        vec![
            ViewMode::ListGrouped,
            ViewMode::ListFlat,
            ViewMode::DependencyTree,
        ]
    }

    pub fn name(&self) -> &'static str {
        match self {
            ViewMode::ListGrouped => "Grouped",
            ViewMode::ListFlat => "Flat",
            ViewMode::DependencyTree => "Tree",
        }
    }

    pub fn next(&self) -> ViewMode {
        match self {
            ViewMode::ListGrouped => ViewMode::ListFlat,
            ViewMode::ListFlat => ViewMode::DependencyTree,
            ViewMode::DependencyTree => ViewMode::ListGrouped,
        }
    }
}

/// Dataflow explorer state
#[derive(Debug, Clone)]
pub struct ExplorerState {
    pub active_tab: ExplorerTab,
    pub view_mode: ViewMode,
    pub selected_dataflow: Option<String>,
    pub selected_node: Option<String>,
    pub filter_text: String,
    pub show_stopped: bool,
    pub last_refresh: Instant,
}

impl Default for ExplorerState {
    fn default() -> Self {
        Self {
            active_tab: ExplorerTab::Overview,
            view_mode: ViewMode::ListGrouped,
            selected_dataflow: None,
            selected_node: None,
            filter_text: String::new(),
            show_stopped: true,
            last_refresh: Instant::now(),
        }
    }
}

impl ExplorerState {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn select_dataflow(&mut self, dataflow_id: String) {
        self.selected_dataflow = Some(dataflow_id);
        self.selected_node = None;
    }

    pub fn select_node(&mut self, node_id: String) {
        self.selected_node = Some(node_id);
    }

    pub fn clear_selection(&mut self) {
        self.selected_dataflow = None;
        self.selected_node = None;
    }

    pub fn switch_tab(&mut self, tab: ExplorerTab) {
        self.active_tab = tab;
    }

    pub fn next_tab(&mut self) {
        self.active_tab = self.active_tab.next();
    }

    pub fn prev_tab(&mut self) {
        self.active_tab = self.active_tab.prev();
    }

    pub fn cycle_view_mode(&mut self) {
        self.view_mode = self.view_mode.next();
    }

    pub fn set_filter(&mut self, filter: String) {
        self.filter_text = filter;
    }

    pub fn toggle_show_stopped(&mut self) {
        self.show_stopped = !self.show_stopped;
    }

    pub fn mark_refreshed(&mut self) {
        self.last_refresh = Instant::now();
    }
}

/// Node selection state for node inspector panel
#[derive(Debug, Clone)]
pub struct NodeSelection {
    pub dataflow_id: String,
    pub node_id: String,
    pub last_updated: Instant,
}

impl NodeSelection {
    pub fn new(dataflow_id: String, node_id: String) -> Self {
        Self {
            dataflow_id,
            node_id,
            last_updated: Instant::now(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_explorer_tab_navigation() {
        let tab = ExplorerTab::Overview;
        assert_eq!(tab.next(), ExplorerTab::Nodes);
        assert_eq!(tab.prev(), ExplorerTab::Debug);
    }

    #[test]
    fn test_explorer_tab_cycle() {
        let mut tab = ExplorerTab::Overview;
        for _ in 0..6 {
            tab = tab.next();
        }
        assert_eq!(tab, ExplorerTab::Overview);
    }

    #[test]
    fn test_view_mode_cycle() {
        let mode = ViewMode::ListGrouped;
        assert_eq!(mode.next(), ViewMode::ListFlat);
        assert_eq!(mode.next().next(), ViewMode::DependencyTree);
        assert_eq!(mode.next().next().next(), ViewMode::ListGrouped);
    }

    #[test]
    fn test_explorer_state_default() {
        let state = ExplorerState::default();
        assert_eq!(state.active_tab, ExplorerTab::Overview);
        assert_eq!(state.view_mode, ViewMode::ListGrouped);
        assert!(state.selected_dataflow.is_none());
        assert!(state.show_stopped);
    }

    #[test]
    fn test_explorer_state_selection() {
        let mut state = ExplorerState::new();

        state.select_dataflow("df1".to_string());
        assert_eq!(state.selected_dataflow, Some("df1".to_string()));
        assert!(state.selected_node.is_none());

        state.select_node("node1".to_string());
        assert_eq!(state.selected_node, Some("node1".to_string()));

        state.clear_selection();
        assert!(state.selected_dataflow.is_none());
        assert!(state.selected_node.is_none());
    }

    #[test]
    fn test_explorer_state_tab_switching() {
        let mut state = ExplorerState::new();

        state.switch_tab(ExplorerTab::Nodes);
        assert_eq!(state.active_tab, ExplorerTab::Nodes);

        state.next_tab();
        assert_eq!(state.active_tab, ExplorerTab::Performance);

        state.prev_tab();
        assert_eq!(state.active_tab, ExplorerTab::Nodes);
    }

    #[test]
    fn test_explorer_state_view_mode() {
        let mut state = ExplorerState::new();
        assert_eq!(state.view_mode, ViewMode::ListGrouped);

        state.cycle_view_mode();
        assert_eq!(state.view_mode, ViewMode::ListFlat);

        state.cycle_view_mode();
        assert_eq!(state.view_mode, ViewMode::DependencyTree);
    }

    #[test]
    fn test_explorer_state_filter() {
        let mut state = ExplorerState::new();
        assert_eq!(state.filter_text, "");

        state.set_filter("test".to_string());
        assert_eq!(state.filter_text, "test");
    }

    #[test]
    fn test_explorer_state_toggle_stopped() {
        let mut state = ExplorerState::new();
        assert!(state.show_stopped);

        state.toggle_show_stopped();
        assert!(!state.show_stopped);

        state.toggle_show_stopped();
        assert!(state.show_stopped);
    }
}
