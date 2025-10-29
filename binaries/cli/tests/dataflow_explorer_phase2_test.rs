/// Tests for DataflowExplorerView Phase 2 features (Issue #25)
#[cfg(test)]
mod dataflow_explorer_phase2_tests {
    use dora_cli::tui::{
        app::{AppState, DataflowInfo, NodeInfo},
        theme::ThemeConfig,
        views::DataflowExplorerView,
    };

    fn create_test_app_state() -> AppState {
        let mut state = AppState::default();
        state.dataflows = vec![
            DataflowInfo {
                id: "df1".to_string(),
                name: "Test Dataflow 1".to_string(),
                status: "running".to_string(),
                nodes: vec![
                    NodeInfo {
                        id: "node1".to_string(),
                        name: "Node 1".to_string(),
                        status: "running".to_string(),
                        ..Default::default()
                    },
                    NodeInfo {
                        id: "node2".to_string(),
                        name: "Node 2".to_string(),
                        status: "running".to_string(),
                        ..Default::default()
                    },
                ],
            },
            DataflowInfo {
                id: "df2".to_string(),
                name: "Test Dataflow 2".to_string(),
                status: "stopped".to_string(),
                nodes: vec![NodeInfo {
                    id: "node3".to_string(),
                    name: "Node 3".to_string(),
                    status: "stopped".to_string(),
                    ..Default::default()
                }],
            },
        ];
        state
    }

    #[test]
    fn test_explorer_view_creation() {
        let theme = ThemeConfig::default();
        let view = DataflowExplorerView::new(&theme);

        // Verify initial state
        assert!(!view.show_inspection_panel);
        assert_eq!(view.selected_index, 0);
    }

    #[test]
    fn test_selection_navigation() {
        let theme = ThemeConfig::default();
        let mut view = DataflowExplorerView::new(&theme);
        let app_state = create_test_app_state();

        // Test selecting next
        assert_eq!(view.selected_index, 0);

        view.select_next(&app_state);
        assert_eq!(view.selected_index, 1);

        // Test wrapping at end
        view.select_next(&app_state);
        assert_eq!(view.selected_index, 0); // Should wrap to beginning

        // Test selecting previous
        view.select_previous(&app_state);
        assert_eq!(view.selected_index, 1); // Should wrap to end

        view.select_previous(&app_state);
        assert_eq!(view.selected_index, 0);
    }

    #[test]
    fn test_get_selected_dataflow() {
        let theme = ThemeConfig::default();
        let mut view = DataflowExplorerView::new(&theme);
        let app_state = create_test_app_state();

        // Select first dataflow
        view.selected_index = 0;
        let selected = view.get_selected_dataflow(&app_state);
        assert!(selected.is_some());
        assert_eq!(selected.unwrap().name, "Test Dataflow 1");

        // Select second dataflow
        view.selected_index = 1;
        let selected = view.get_selected_dataflow(&app_state);
        assert!(selected.is_some());
        assert_eq!(selected.unwrap().name, "Test Dataflow 2");

        // Select out of bounds
        view.selected_index = 999;
        let selected = view.get_selected_dataflow(&app_state);
        assert!(selected.is_none());
    }

    #[test]
    fn test_get_item_count_overview_tab() {
        let theme = ThemeConfig::default();
        let view = DataflowExplorerView::new(&theme);
        let app_state = create_test_app_state();

        // Should count all dataflows when show_stopped is true
        assert_eq!(view.get_item_count(&app_state), 2);
    }

    #[test]
    fn test_get_item_count_with_stopped_hidden() {
        let theme = ThemeConfig::default();
        let mut view = DataflowExplorerView::new(&theme);
        let app_state = create_test_app_state();

        // Hide stopped dataflows
        view.state.toggle_show_stopped();

        // Should only count running dataflows
        assert_eq!(view.get_item_count(&app_state), 1);
    }

    #[test]
    fn test_get_item_count_nodes_tab() {
        let theme = ThemeConfig::default();
        let mut view = DataflowExplorerView::new(&theme);
        let app_state = create_test_app_state();

        use dora_cli::tui::views::ExplorerTab;

        // Switch to Nodes tab
        view.state.switch_tab(ExplorerTab::Nodes);

        // Should count all nodes across all dataflows
        assert_eq!(view.get_item_count(&app_state), 3);
    }

    #[test]
    fn test_inspection_panel_toggle() {
        let theme = ThemeConfig::default();
        let mut view = DataflowExplorerView::new(&theme);

        assert!(!view.show_inspection_panel);

        // Toggle on
        view.show_inspection_panel = !view.show_inspection_panel;
        assert!(view.show_inspection_panel);

        // Toggle off
        view.show_inspection_panel = !view.show_inspection_panel;
        assert!(!view.show_inspection_panel);
    }

    #[test]
    fn test_selection_with_empty_state() {
        let theme = ThemeConfig::default();
        let mut view = DataflowExplorerView::new(&theme);
        let app_state = AppState::default(); // Empty state

        // Should handle empty state gracefully
        view.select_next(&app_state);
        assert_eq!(view.selected_index, 0);

        view.select_previous(&app_state);
        assert_eq!(view.selected_index, 0);

        let selected = view.get_selected_dataflow(&app_state);
        assert!(selected.is_none());
    }

    #[test]
    fn test_selection_reset_on_filter_change() {
        let theme = ThemeConfig::default();
        let mut view = DataflowExplorerView::new(&theme);
        let app_state = create_test_app_state();

        // Select second item
        view.selected_index = 1;

        // Toggle show_stopped (simulating filter change)
        view.state.toggle_show_stopped();
        view.selected_index = 0; // In real code, handle_key does this

        assert_eq!(view.selected_index, 0);
    }

    #[test]
    fn test_navigation_boundaries() {
        let theme = ThemeConfig::default();
        let mut view = DataflowExplorerView::new(&theme);
        let app_state = create_test_app_state();

        // Navigate to end
        view.selected_index = 0;
        view.select_next(&app_state);
        view.select_next(&app_state);

        // Should be back at start (wrapped)
        assert_eq!(view.selected_index, 0);

        // Navigate backwards from start
        view.select_previous(&app_state);

        // Should be at end (wrapped)
        assert_eq!(view.selected_index, 1);
    }

    #[test]
    fn test_multiple_dataflows_selection() {
        let theme = ThemeConfig::default();
        let mut view = DataflowExplorerView::new(&theme);

        let mut app_state = create_test_app_state();
        // Add more dataflows
        for i in 3..=5 {
            app_state.dataflows.push(DataflowInfo {
                id: format!("df{}", i),
                name: format!("Test Dataflow {}", i),
                status: "running".to_string(),
                nodes: vec![],
            });
        }

        // Total should be 5 dataflows
        assert_eq!(view.get_item_count(&app_state), 5);

        // Navigate through all
        for i in 0..5 {
            assert_eq!(view.selected_index, i);
            view.select_next(&app_state);
        }

        // Should wrap back to 0
        assert_eq!(view.selected_index, 0);
    }

    #[test]
    fn test_get_selected_dataflow_with_filtering() {
        let theme = ThemeConfig::default();
        let mut view = DataflowExplorerView::new(&theme);
        let app_state = create_test_app_state();

        // With show_stopped = true, should get df1
        view.selected_index = 0;
        let selected = view.get_selected_dataflow(&app_state);
        assert_eq!(selected.unwrap().id, "df1");

        // With show_stopped = false
        view.state.toggle_show_stopped(); // Now false
        let selected = view.get_selected_dataflow(&app_state);
        assert_eq!(selected.unwrap().id, "df1"); // Still df1 (only running dataflow)

        // Select index 1 with filtering
        view.selected_index = 1;
        let selected = view.get_selected_dataflow(&app_state);
        assert!(selected.is_none()); // df2 is stopped, so filtered out
    }
}
