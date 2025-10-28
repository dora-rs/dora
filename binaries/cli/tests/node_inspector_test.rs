/// Tests for NodeInspectorView (Issue #27)
#[cfg(test)]
mod node_inspector_tests {
    use dora_cli::tui::theme::ThemeConfig;
    use dora_cli::tui::views::{InspectorTab, NodeInspectorState, NodeInspectorView, NodeMetrics};

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

        tab = tab.prev();
        assert_eq!(tab, InspectorTab::Connections);

        tab = tab.prev();
        assert_eq!(tab, InspectorTab::Overview);
    }

    #[test]
    fn test_inspector_tab_full_cycle() {
        let mut tab = InspectorTab::Overview;

        // Go through a full cycle using next()
        for _ in 0..5 {
            tab = tab.next();
        }

        // Should be back at Overview
        assert_eq!(tab, InspectorTab::Overview);
    }

    #[test]
    fn test_inspector_tab_full_reverse_cycle() {
        let mut tab = InspectorTab::Overview;

        // Go through a full cycle using prev()
        for _ in 0..5 {
            tab = tab.prev();
        }

        // Should be back at Overview
        assert_eq!(tab, InspectorTab::Overview);
    }

    #[test]
    fn test_node_inspector_state_new() {
        let state = NodeInspectorState::new("test-node".to_string());

        assert_eq!(state.node_id, "test-node");
        assert_eq!(state.active_tab, InspectorTab::Overview);
        assert_eq!(state.selected_input, 0);
        assert_eq!(state.selected_output, 0);
        assert!(!state.show_detailed_metrics);
        assert_eq!(state.scroll_offset, 0);
        assert!(!state.edit_mode);
    }

    #[test]
    fn test_node_inspector_state_default() {
        let state = NodeInspectorState::default();

        assert_eq!(state.node_id, "");
        assert_eq!(state.active_tab, InspectorTab::Overview);
    }

    #[test]
    fn test_node_inspector_state_switch_tab() {
        let mut state = NodeInspectorState::new("test".to_string());

        assert_eq!(state.active_tab, InspectorTab::Overview);

        state.switch_tab(InspectorTab::Connections);
        assert_eq!(state.active_tab, InspectorTab::Connections);

        state.switch_tab(InspectorTab::Performance);
        assert_eq!(state.active_tab, InspectorTab::Performance);

        state.switch_tab(InspectorTab::Debug);
        assert_eq!(state.active_tab, InspectorTab::Debug);
    }

    #[test]
    fn test_node_inspector_state_next_prev_tab() {
        let mut state = NodeInspectorState::new("test".to_string());

        assert_eq!(state.active_tab, InspectorTab::Overview);

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
        let mut state = NodeInspectorState::new("test".to_string());

        assert_eq!(state.selected_input, 0);

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

        state.select_prev_input(3);
        assert_eq!(state.selected_input, 1);
    }

    #[test]
    fn test_node_inspector_state_output_selection() {
        let mut state = NodeInspectorState::new("test".to_string());

        assert_eq!(state.selected_output, 0);

        // Select next output
        state.select_next_output(2);
        assert_eq!(state.selected_output, 1);

        // Should wrap
        state.select_next_output(2);
        assert_eq!(state.selected_output, 0);

        // Select previous
        state.select_prev_output(2);
        assert_eq!(state.selected_output, 1);

        state.select_prev_output(2);
        assert_eq!(state.selected_output, 0);
    }

    #[test]
    fn test_node_inspector_state_empty_inputs_outputs() {
        let mut state = NodeInspectorState::new("test".to_string());

        // With 0 inputs/outputs, selection should stay at 0
        state.select_next_input(0);
        assert_eq!(state.selected_input, 0);

        state.select_prev_input(0);
        assert_eq!(state.selected_input, 0);

        state.select_next_output(0);
        assert_eq!(state.selected_output, 0);

        state.select_prev_output(0);
        assert_eq!(state.selected_output, 0);
    }

    #[test]
    fn test_node_inspector_state_toggle_detailed_metrics() {
        let mut state = NodeInspectorState::new("test".to_string());

        assert!(!state.show_detailed_metrics);

        state.toggle_detailed_metrics();
        assert!(state.show_detailed_metrics);

        state.toggle_detailed_metrics();
        assert!(!state.show_detailed_metrics);
    }

    #[test]
    fn test_node_inspector_state_scrolling() {
        let mut state = NodeInspectorState::new("test".to_string());

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
    }

    #[test]
    fn test_node_inspector_state_max_scroll() {
        let mut state = NodeInspectorState::new("test".to_string());

        // Scroll to max
        for _ in 0..20 {
            state.scroll_down(10);
        }

        // Should be capped at max_scroll (10)
        assert_eq!(state.scroll_offset, 10);
    }

    #[test]
    fn test_node_inspector_state_toggle_edit_mode() {
        let mut state = NodeInspectorState::new("test".to_string());

        assert!(!state.edit_mode);

        state.toggle_edit_mode();
        assert!(state.edit_mode);

        state.toggle_edit_mode();
        assert!(!state.edit_mode);
    }

    #[test]
    fn test_node_inspector_state_mark_refreshed() {
        let mut state = NodeInspectorState::new("test".to_string());

        let initial_time = state.last_refresh;

        std::thread::sleep(std::time::Duration::from_millis(10));

        state.mark_refreshed();

        assert!(state.last_refresh > initial_time);
    }

    #[test]
    fn test_tab_switching_resets_scroll() {
        let mut state = NodeInspectorState::new("test".to_string());

        state.scroll_down(10);
        assert_eq!(state.scroll_offset, 1);

        // Switching tabs should reset scroll
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
        assert_eq!(metrics.memory_mb, 0.0);
        assert_eq!(metrics.message_rate, 0.0);
        assert_eq!(metrics.processing_latency_ms, 0.0);
        assert_eq!(metrics.uptime_seconds, 0);
        assert_eq!(metrics.error_count, 0);
    }

    #[test]
    fn test_node_inspector_view_creation() {
        let theme = ThemeConfig::default();
        let view = NodeInspectorView::new(&theme, "test-node".to_string());

        assert_eq!(view.state.node_id, "test-node");
        assert_eq!(view.state.active_tab, InspectorTab::Overview);
    }

    #[test]
    fn test_node_inspector_state_complex_workflow() {
        let mut state = NodeInspectorState::new("my-node".to_string());

        // Simulate a user workflow
        state.switch_tab(InspectorTab::Connections);
        state.select_next_input(3);
        state.select_next_input(3);
        state.toggle_detailed_metrics();

        // Verify state
        assert_eq!(state.active_tab, InspectorTab::Connections);
        assert_eq!(state.selected_input, 2);
        assert!(state.show_detailed_metrics);

        // Switch to debug and scroll
        state.switch_tab(InspectorTab::Debug);
        state.scroll_down(20);
        state.scroll_down(20);

        assert_eq!(state.active_tab, InspectorTab::Debug);
        assert_eq!(state.scroll_offset, 2);

        // Switch back to overview (scroll should reset)
        state.switch_tab(InspectorTab::Overview);
        assert_eq!(state.scroll_offset, 0);
    }

    #[test]
    fn test_multiple_node_inspector_instances() {
        let theme = ThemeConfig::default();

        let view1 = NodeInspectorView::new(&theme, "node-1".to_string());
        let view2 = NodeInspectorView::new(&theme, "node-2".to_string());
        let view3 = NodeInspectorView::new(&theme, "node-3".to_string());

        assert_eq!(view1.state.node_id, "node-1");
        assert_eq!(view2.state.node_id, "node-2");
        assert_eq!(view3.state.node_id, "node-3");

        // Each should have independent state
        assert_eq!(view1.state.active_tab, InspectorTab::Overview);
        assert_eq!(view2.state.active_tab, InspectorTab::Overview);
        assert_eq!(view3.state.active_tab, InspectorTab::Overview);
    }

    #[test]
    fn test_inspector_state_preserves_node_id() {
        let mut state = NodeInspectorState::new("critical-node".to_string());

        // Node ID should never change during state operations
        state.next_tab();
        assert_eq!(state.node_id, "critical-node");

        state.toggle_detailed_metrics();
        assert_eq!(state.node_id, "critical-node");

        state.scroll_down(10);
        assert_eq!(state.node_id, "critical-node");

        state.select_next_input(5);
        assert_eq!(state.node_id, "critical-node");
    }
}
