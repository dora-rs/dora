/// Integration test for TUI initialization
/// Tests that all TUI components can be created and initialized
use dora_cli::tui::{AppState, DoraApp, ThemeConfig, ViewType};

#[test]
fn test_app_state_creation() {
    let state = AppState::default();

    // Should have default values
    assert_eq!(state.dataflows.len(), 0);
    assert!(state.system_metrics.cpu_usage >= 0.0);
    assert!(state.system_metrics.memory_usage >= 0.0);
}

#[test]
fn test_theme_config_creation() {
    let theme = ThemeConfig::default();

    // Theme should be created successfully
    // Just verify it doesn't panic
    drop(theme);
}

#[test]
fn test_dora_app_creation() {
    // Test creating DoraApp with different initial views
    let app_dashboard = DoraApp::new(ViewType::Dashboard);
    drop(app_dashboard);

    let app_dataflows = DoraApp::new(ViewType::DataflowManager);
    drop(app_dataflows);

    let app_monitor = DoraApp::new(ViewType::SystemMonitor);
    drop(app_monitor);
}

#[test]
fn test_tui_components_dont_panic() {
    // This test ensures basic TUI initialization doesn't panic
    // even in a non-TTY environment

    let _state = AppState::default();
    let _theme = ThemeConfig::default();
    let _app = DoraApp::new(ViewType::Dashboard);

    // If we got here, all components initialized successfully
}

#[test]
fn test_view_type_variants() {
    // Ensure all ViewType variants can be constructed
    let views = vec![
        ViewType::Dashboard,
        ViewType::DataflowManager,
        ViewType::SystemMonitor,
        ViewType::LogViewer {
            target: "test".to_string(),
        },
        ViewType::NodeInspector {
            node_id: "test_node".to_string(),
        },
        ViewType::RecordingAnalyzer {
            recording_id: "test_recording".to_string(),
        },
        ViewType::DebugSession {
            dataflow_id: "test_dataflow".to_string(),
        },
        ViewType::SettingsManager,
        ViewType::Help,
    ];

    for view in views {
        let _app = DoraApp::new(view);
    }
}
