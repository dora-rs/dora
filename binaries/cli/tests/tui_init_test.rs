/// Integration test for TUI initialization
/// Tests that all TUI components can be created and initialized
use std::sync::Arc;

use dora_cli::tui::app::DoraApp;
use dora_cli::tui::{AppState, ThemeConfig, ViewType};
use tui_interface::{
    MockCoordinatorClient, MockLegacyCliService, MockPreferencesStore, MockTelemetryService,
    UserPreferencesSnapshot,
};

fn build_app_with_view(view: ViewType) -> DoraApp {
    let prefs = Arc::new(MockPreferencesStore::new());
    let coordinator = Arc::new(MockCoordinatorClient::new());
    let telemetry = Arc::new(MockTelemetryService::new());
    let legacy = Arc::new(MockLegacyCliService::new());

    prefs.set_load_result(Ok(UserPreferencesSnapshot {
        theme: "dark".to_string(),
        auto_refresh_interval_secs: 5,
        show_system_info: true,
        default_view: None,
    }));

    DoraApp::with_dependencies(view, prefs, coordinator, telemetry, legacy)
}

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
    drop(build_app_with_view(ViewType::Dashboard));
    drop(build_app_with_view(ViewType::DataflowManager));
    drop(build_app_with_view(ViewType::SystemMonitor));
}

#[test]
fn test_tui_components_dont_panic() {
    let _state = AppState::default();
    let _theme = ThemeConfig::default();
    drop(build_app_with_view(ViewType::Dashboard));
}

#[test]
fn test_view_type_variants() {
    let views = vec![
        ViewType::Dashboard,
        ViewType::DataflowManager,
        ViewType::SystemMonitor,
        ViewType::LogViewer {
            target: "test".to_string(),
        },
        ViewType::NodeInspector {
            dataflow_id: "test_dataflow".to_string(),
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
        drop(build_app_with_view(view));
    }
}
