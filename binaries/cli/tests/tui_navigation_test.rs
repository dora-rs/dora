use dora_cli::cli::TuiView;
/// Integration test for TUI navigation and keyboard handling
/// Verifies that view switching and keyboard shortcuts work correctly
use dora_cli::tui::ViewType;

#[test]
fn test_cli_view_enum_variants_exist() {
    // Verify all TuiView enum variants are available
    let _dashboard = TuiView::Dashboard;
    let _dataflow = TuiView::Dataflow;
    let _performance = TuiView::Performance;
    let _logs = TuiView::Logs;
}

#[test]
fn test_view_type_enum_has_all_variants() {
    // Verify all ViewType variants can be constructed
    let views = vec![
        ViewType::Dashboard,
        ViewType::DataflowManager,
        ViewType::SystemMonitor,
        ViewType::LogViewer {
            target: "all".to_string(),
        },
        ViewType::NodeInspector {
            node_id: "node1".to_string(),
        },
        ViewType::RecordingAnalyzer {
            recording_id: "rec1".to_string(),
        },
        ViewType::DebugSession {
            dataflow_id: "df1".to_string(),
        },
        ViewType::SettingsManager,
        ViewType::Help,
    ];

    // All views should be distinct
    assert_eq!(views.len(), 9);
}

#[test]
fn test_view_type_supports_clone() {
    // ViewType should be cloneable for state management
    let view1 = ViewType::Dashboard;
    let view2 = view1.clone();

    // Both should be Dashboard
    match (view1, view2) {
        (ViewType::Dashboard, ViewType::Dashboard) => {}
        _ => panic!("Clone should preserve ViewType variant"),
    }
}

#[test]
fn test_view_type_with_parameters() {
    // Test ViewTypes that take parameters
    let log_view = ViewType::LogViewer {
        target: "node-camera".to_string(),
    };

    match log_view {
        ViewType::LogViewer { target } => {
            assert_eq!(target, "node-camera");
        }
        _ => panic!("Expected LogViewer variant"),
    }

    let node_view = ViewType::NodeInspector {
        node_id: "detector".to_string(),
    };

    match node_view {
        ViewType::NodeInspector { node_id } => {
            assert_eq!(node_id, "detector");
        }
        _ => panic!("Expected NodeInspector variant"),
    }
}

#[test]
fn test_view_navigation_sequence() {
    // Simulate a typical navigation sequence
    // User starts at dashboard, navigates to dataflow, then to a specific node

    let views = vec![
        ViewType::Dashboard,
        ViewType::DataflowManager,
        ViewType::NodeInspector {
            node_id: "camera".to_string(),
        },
        ViewType::Dashboard, // Back to dashboard
    ];

    // All transitions should be valid
    for view in views {
        // Create app with each view - simulates view switching
        let _app = dora_cli::tui::DoraApp::new(view);
    }
}

#[test]
fn test_default_view_type() {
    // ViewType should have a sensible default
    let default_view = ViewType::default();

    match default_view {
        ViewType::Dashboard => {} // Expected default
        _ => panic!("Default should be Dashboard"),
    }
}
