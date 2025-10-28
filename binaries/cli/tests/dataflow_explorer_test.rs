/// Tests for DataflowExplorerView (Issue #25 Phase 1)
#[cfg(test)]
mod dataflow_explorer_tests {
    use dora_cli::tui::views::{DataflowExplorerView, ExplorerState, ExplorerTab, ViewMode};

    #[test]
    fn test_explorer_tab_all_returns_six_tabs() {
        let tabs = ExplorerTab::all();
        assert_eq!(tabs.len(), 6);
        assert_eq!(tabs[0], ExplorerTab::Overview);
        assert_eq!(tabs[1], ExplorerTab::Nodes);
        assert_eq!(tabs[2], ExplorerTab::Performance);
        assert_eq!(tabs[3], ExplorerTab::Configuration);
        assert_eq!(tabs[4], ExplorerTab::Logs);
        assert_eq!(tabs[5], ExplorerTab::Debug);
    }

    #[test]
    fn test_explorer_tab_names() {
        assert_eq!(ExplorerTab::Overview.name(), "Overview");
        assert_eq!(ExplorerTab::Nodes.name(), "Nodes");
        assert_eq!(ExplorerTab::Performance.name(), "Performance");
        assert_eq!(ExplorerTab::Configuration.name(), "Configuration");
        assert_eq!(ExplorerTab::Logs.name(), "Logs");
        assert_eq!(ExplorerTab::Debug.name(), "Debug");
    }

    #[test]
    fn test_explorer_tab_shortcuts() {
        assert_eq!(ExplorerTab::Overview.shortcut(), "1");
        assert_eq!(ExplorerTab::Nodes.shortcut(), "2");
        assert_eq!(ExplorerTab::Performance.shortcut(), "3");
        assert_eq!(ExplorerTab::Configuration.shortcut(), "4");
        assert_eq!(ExplorerTab::Logs.shortcut(), "5");
        assert_eq!(ExplorerTab::Debug.shortcut(), "6");
    }

    #[test]
    fn test_explorer_tab_navigation() {
        let mut tab = ExplorerTab::Overview;

        // Test next()
        tab = tab.next();
        assert_eq!(tab, ExplorerTab::Nodes);

        tab = tab.next();
        assert_eq!(tab, ExplorerTab::Performance);

        tab = tab.next();
        assert_eq!(tab, ExplorerTab::Configuration);

        tab = tab.next();
        assert_eq!(tab, ExplorerTab::Logs);

        tab = tab.next();
        assert_eq!(tab, ExplorerTab::Debug);

        // Test wrapping
        tab = tab.next();
        assert_eq!(tab, ExplorerTab::Overview);
    }

    #[test]
    fn test_explorer_tab_prev_navigation() {
        let mut tab = ExplorerTab::Overview;

        // Test prev() wraps around
        tab = tab.prev();
        assert_eq!(tab, ExplorerTab::Debug);

        tab = tab.prev();
        assert_eq!(tab, ExplorerTab::Logs);

        tab = tab.prev();
        assert_eq!(tab, ExplorerTab::Configuration);
    }

    #[test]
    fn test_view_mode_all_returns_three_modes() {
        let modes = ViewMode::all();
        assert_eq!(modes.len(), 3);
        assert_eq!(modes[0], ViewMode::ListGrouped);
        assert_eq!(modes[1], ViewMode::ListFlat);
        assert_eq!(modes[2], ViewMode::DependencyTree);
    }

    #[test]
    fn test_view_mode_names() {
        assert_eq!(ViewMode::ListGrouped.name(), "Grouped");
        assert_eq!(ViewMode::ListFlat.name(), "Flat");
        assert_eq!(ViewMode::DependencyTree.name(), "Tree");
    }

    #[test]
    fn test_view_mode_cycling() {
        let mut mode = ViewMode::ListGrouped;

        mode = mode.next();
        assert_eq!(mode, ViewMode::ListFlat);

        mode = mode.next();
        assert_eq!(mode, ViewMode::DependencyTree);

        // Test wrapping
        mode = mode.next();
        assert_eq!(mode, ViewMode::ListGrouped);
    }

    #[test]
    fn test_explorer_state_default() {
        let state = ExplorerState::default();

        assert_eq!(state.active_tab, ExplorerTab::Overview);
        assert_eq!(state.view_mode, ViewMode::ListGrouped);
        assert!(state.selected_dataflow.is_none());
        assert!(state.selected_node.is_none());
        assert_eq!(state.filter_text, "");
        assert!(state.show_stopped);
    }

    #[test]
    fn test_explorer_state_new() {
        let state = ExplorerState::new();

        assert_eq!(state.active_tab, ExplorerTab::Overview);
        assert_eq!(state.view_mode, ViewMode::ListGrouped);
    }

    #[test]
    fn test_explorer_state_select_dataflow() {
        let mut state = ExplorerState::new();

        state.select_dataflow("test-dataflow".to_string());

        assert_eq!(state.selected_dataflow, Some("test-dataflow".to_string()));
        assert!(state.selected_node.is_none()); // Should clear node selection
    }

    #[test]
    fn test_explorer_state_select_node() {
        let mut state = ExplorerState::new();

        state.select_node("test-node".to_string());

        assert_eq!(state.selected_node, Some("test-node".to_string()));
    }

    #[test]
    fn test_explorer_state_clear_selection() {
        let mut state = ExplorerState::new();

        state.select_dataflow("test-dataflow".to_string());
        state.select_node("test-node".to_string());

        assert!(state.selected_dataflow.is_some());
        assert!(state.selected_node.is_some());

        state.clear_selection();

        assert!(state.selected_dataflow.is_none());
        assert!(state.selected_node.is_none());
    }

    #[test]
    fn test_explorer_state_switch_tab() {
        let mut state = ExplorerState::new();

        assert_eq!(state.active_tab, ExplorerTab::Overview);

        state.switch_tab(ExplorerTab::Nodes);
        assert_eq!(state.active_tab, ExplorerTab::Nodes);

        state.switch_tab(ExplorerTab::Performance);
        assert_eq!(state.active_tab, ExplorerTab::Performance);
    }

    #[test]
    fn test_explorer_state_next_tab() {
        let mut state = ExplorerState::new();

        assert_eq!(state.active_tab, ExplorerTab::Overview);

        state.next_tab();
        assert_eq!(state.active_tab, ExplorerTab::Nodes);

        state.next_tab();
        assert_eq!(state.active_tab, ExplorerTab::Performance);
    }

    #[test]
    fn test_explorer_state_prev_tab() {
        let mut state = ExplorerState::new();

        assert_eq!(state.active_tab, ExplorerTab::Overview);

        state.prev_tab();
        assert_eq!(state.active_tab, ExplorerTab::Debug); // Should wrap around
    }

    #[test]
    fn test_explorer_state_cycle_view_mode() {
        let mut state = ExplorerState::new();

        assert_eq!(state.view_mode, ViewMode::ListGrouped);

        state.cycle_view_mode();
        assert_eq!(state.view_mode, ViewMode::ListFlat);

        state.cycle_view_mode();
        assert_eq!(state.view_mode, ViewMode::DependencyTree);

        state.cycle_view_mode();
        assert_eq!(state.view_mode, ViewMode::ListGrouped); // Should wrap
    }

    #[test]
    fn test_explorer_state_set_filter() {
        let mut state = ExplorerState::new();

        assert_eq!(state.filter_text, "");

        state.set_filter("test-filter".to_string());
        assert_eq!(state.filter_text, "test-filter");

        state.set_filter("another-filter".to_string());
        assert_eq!(state.filter_text, "another-filter");
    }

    #[test]
    fn test_explorer_state_toggle_show_stopped() {
        let mut state = ExplorerState::new();

        assert!(state.show_stopped); // Default is true

        state.toggle_show_stopped();
        assert!(!state.show_stopped);

        state.toggle_show_stopped();
        assert!(state.show_stopped);
    }

    #[test]
    fn test_explorer_state_mark_refreshed() {
        let mut state = ExplorerState::new();

        let initial_time = state.last_refresh;

        // Sleep a tiny bit to ensure time has passed
        std::thread::sleep(std::time::Duration::from_millis(10));

        state.mark_refreshed();

        assert!(state.last_refresh > initial_time);
    }

    #[test]
    fn test_dataflow_explorer_view_creation() {
        use dora_cli::tui::theme::ThemeConfig;

        let theme = ThemeConfig::default();
        let view = DataflowExplorerView::new(&theme);

        // View should be created successfully
        // This test just verifies that construction doesn't panic
        drop(view);
    }

    #[test]
    fn test_explorer_tab_full_cycle() {
        let mut tab = ExplorerTab::Overview;

        // Go through a full cycle using next()
        for _ in 0..6 {
            tab = tab.next();
        }

        // Should be back at Overview
        assert_eq!(tab, ExplorerTab::Overview);
    }

    #[test]
    fn test_explorer_tab_full_reverse_cycle() {
        let mut tab = ExplorerTab::Overview;

        // Go through a full cycle using prev()
        for _ in 0..6 {
            tab = tab.prev();
        }

        // Should be back at Overview
        assert_eq!(tab, ExplorerTab::Overview);
    }

    #[test]
    fn test_view_mode_full_cycle() {
        let mut mode = ViewMode::ListGrouped;

        // Go through a full cycle
        for _ in 0..3 {
            mode = mode.next();
        }

        // Should be back at ListGrouped
        assert_eq!(mode, ViewMode::ListGrouped);
    }

    #[test]
    fn test_explorer_state_complex_workflow() {
        let mut state = ExplorerState::new();

        // Simulate a user workflow
        state.switch_tab(ExplorerTab::Nodes);
        state.select_dataflow("my-dataflow".to_string());
        state.select_node("node-1".to_string());
        state.cycle_view_mode();
        state.set_filter("running".to_string());

        // Verify state
        assert_eq!(state.active_tab, ExplorerTab::Nodes);
        assert_eq!(state.selected_dataflow, Some("my-dataflow".to_string()));
        assert_eq!(state.selected_node, Some("node-1".to_string()));
        assert_eq!(state.view_mode, ViewMode::ListFlat);
        assert_eq!(state.filter_text, "running");

        // Clear and verify
        state.clear_selection();
        assert!(state.selected_dataflow.is_none());
        assert!(state.selected_node.is_none());
    }
}
