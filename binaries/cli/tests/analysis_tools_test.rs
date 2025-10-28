// Comprehensive tests for Interactive Analysis Tools (Issue #33 - Phase 1)
// Tests cover all analysis types, navigation, state management, and mock data

use crossterm::event::{KeyCode, KeyEvent, KeyModifiers};
use dora_cli::tui::{
    app::AppState,
    theme::ThemeConfig,
    views::{
        AnalysisResults, AnalysisToolsState, AnalysisToolsView, AnalysisType, CorrelationPair,
        CorrelationStrength, DistributionStats, OutlierData, OutlierSeverity, TrendAnalysis,
        TrendDirection, View, ViewAction,
    },
};
use ratatui::{Terminal, backend::TestBackend};

// Helper functions
fn create_test_view() -> AnalysisToolsView {
    AnalysisToolsView::new(&ThemeConfig::default())
}

fn create_app_state() -> AppState {
    AppState::default()
}

// ===== View Creation Tests =====

#[test]
fn test_view_creation() {
    let view = create_test_view();
    assert_eq!(view.title(), "Interactive Analysis Tools");
    assert_eq!(view.state.current_analysis, AnalysisType::Distribution);
}

#[test]
fn test_initial_state() {
    let view = create_test_view();
    assert_eq!(view.state.selected_item, 0);
    assert_eq!(view.state.scroll_offset, 0);
    assert_eq!(view.state.current_analysis, AnalysisType::Distribution);
}

#[test]
fn test_auto_refresh() {
    let view = create_test_view();
    assert!(view.auto_refresh().is_some());
}

// ===== Analysis Type Navigation Tests =====

#[tokio::test]
async fn test_analysis_type_navigation_tab() {
    let mut view = create_test_view();
    let mut app_state = create_app_state();

    let key = KeyEvent::from(KeyCode::Tab);
    let _ = view.handle_key(key, &mut app_state).await;
    assert_eq!(view.state.current_analysis, AnalysisType::Correlation);

    let _ = view.handle_key(key, &mut app_state).await;
    assert_eq!(view.state.current_analysis, AnalysisType::Trend);

    let _ = view.handle_key(key, &mut app_state).await;
    assert_eq!(view.state.current_analysis, AnalysisType::Outlier);

    let _ = view.handle_key(key, &mut app_state).await;
    assert_eq!(view.state.current_analysis, AnalysisType::Distribution); // Wrap around
}

#[tokio::test]
async fn test_analysis_type_navigation_backtab() {
    let mut view = create_test_view();
    let mut app_state = create_app_state();

    let key = KeyEvent::from(KeyCode::BackTab);
    let _ = view.handle_key(key, &mut app_state).await;
    assert_eq!(view.state.current_analysis, AnalysisType::Outlier); // Wrap around

    let _ = view.handle_key(key, &mut app_state).await;
    assert_eq!(view.state.current_analysis, AnalysisType::Trend);
}

#[test]
fn test_analysis_data_updates_on_type_change() {
    let mut state = AnalysisToolsState::new();
    assert!(matches!(state.results, AnalysisResults::Distribution(_)));

    state.next_analysis();
    assert!(matches!(state.results, AnalysisResults::Correlation(_)));

    state.next_analysis();
    assert!(matches!(state.results, AnalysisResults::Trend(_)));
}

// ===== Item Selection Tests =====

#[tokio::test]
async fn test_select_next_item() {
    let mut view = create_test_view();
    let mut app_state = create_app_state();

    let key = KeyEvent::from(KeyCode::Down);
    let _ = view.handle_key(key, &mut app_state).await;
    assert_eq!(view.state.selected_item, 1);

    let _ = view.handle_key(key, &mut app_state).await;
    assert_eq!(view.state.selected_item, 2);
}

#[tokio::test]
async fn test_select_previous_item() {
    let mut view = create_test_view();
    let mut app_state = create_app_state();
    view.state.selected_item = 2;

    let key = KeyEvent::from(KeyCode::Up);
    let _ = view.handle_key(key, &mut app_state).await;
    assert_eq!(view.state.selected_item, 1);

    let _ = view.handle_key(key, &mut app_state).await;
    assert_eq!(view.state.selected_item, 0);
}

#[tokio::test]
async fn test_select_with_j_k_keys() {
    let mut view = create_test_view();
    let mut app_state = create_app_state();

    let key_j = KeyEvent::from(KeyCode::Char('j'));
    let _ = view.handle_key(key_j, &mut app_state).await;
    assert_eq!(view.state.selected_item, 1);

    let key_k = KeyEvent::from(KeyCode::Char('k'));
    let _ = view.handle_key(key_k, &mut app_state).await;
    assert_eq!(view.state.selected_item, 0);
}

// ===== Refresh Tests =====

#[tokio::test]
async fn test_refresh_with_r_key() {
    let mut view = create_test_view();
    let mut app_state = create_app_state();

    let initial_time = view.state.last_refresh;
    std::thread::sleep(std::time::Duration::from_millis(10));

    let key = KeyEvent::from(KeyCode::Char('r'));
    let _ = view.handle_key(key, &mut app_state).await;

    assert!(view.state.last_refresh > initial_time);
}

#[tokio::test]
async fn test_refresh_with_R_key() {
    let mut view = create_test_view();
    let mut app_state = create_app_state();

    let initial_time = view.state.last_refresh;
    std::thread::sleep(std::time::Duration::from_millis(10));

    let key = KeyEvent::from(KeyCode::Char('R'));
    let _ = view.handle_key(key, &mut app_state).await;

    assert!(view.state.last_refresh > initial_time);
}

// ===== Mock Data Tests =====

#[test]
fn test_mock_distribution_data() {
    let results = AnalysisResults::create_mock_distribution();
    if let AnalysisResults::Distribution(stats) = results {
        assert_eq!(stats.len(), 3);
        assert_eq!(stats[0].variable_name, "CPU Usage (%)");
        assert_eq!(stats[1].variable_name, "Memory Usage (%)");
        assert_eq!(stats[2].variable_name, "Latency (ms)");
        assert!(stats.iter().all(|s| s.sample_size > 0));
    } else {
        panic!("Expected Distribution results");
    }
}

#[test]
fn test_mock_correlation_data() {
    let results = AnalysisResults::create_mock_correlation();
    if let AnalysisResults::Correlation(pairs) = results {
        assert_eq!(pairs.len(), 5);
        assert!(pairs.iter().all(|p| p.coefficient.abs() <= 1.0));
        assert!(pairs.iter().all(|p| p.is_significant));
    } else {
        panic!("Expected Correlation results");
    }
}

#[test]
fn test_mock_trend_data() {
    let results = AnalysisResults::create_mock_trend();
    if let AnalysisResults::Trend(trends) = results {
        assert_eq!(trends.len(), 4);
        assert!(
            trends
                .iter()
                .all(|t| t.confidence >= 0.0 && t.confidence <= 1.0)
        );
        assert!(trends.iter().all(|t| t.prediction.is_some()));
    } else {
        panic!("Expected Trend results");
    }
}

#[test]
fn test_mock_outlier_data() {
    let results = AnalysisResults::create_mock_outlier();
    if let AnalysisResults::Outlier(outliers) = results {
        assert_eq!(outliers.len(), 2);
        assert!(outliers.iter().all(|o| o.outlier_count <= o.total_points));
        assert!(outliers.iter().all(|o| !o.outliers.is_empty()));
    } else {
        panic!("Expected Outlier results");
    }
}

// ===== Distribution Stats Tests =====

#[test]
fn test_distribution_stats_cpu() {
    let stats = DistributionStats::create_mock_cpu_stats();
    assert_eq!(stats.variable_name, "CPU Usage (%)");
    assert_eq!(stats.sample_size, 1000);
    assert!(stats.mean > 0.0);
    assert!(stats.median > 0.0);
    assert!(stats.std_dev > 0.0);
    assert!(stats.min < stats.max);
    assert!(stats.q1 < stats.q3);
}

#[test]
fn test_distribution_stats_memory() {
    let stats = DistributionStats::create_mock_memory_stats();
    assert_eq!(stats.variable_name, "Memory Usage (%)");
    assert_eq!(stats.sample_size, 1000);
}

#[test]
fn test_distribution_stats_latency() {
    let stats = DistributionStats::create_mock_latency_stats();
    assert_eq!(stats.variable_name, "Latency (ms)");
    assert_eq!(stats.sample_size, 1000);
}

// ===== Correlation Tests =====

#[test]
fn test_correlation_strength_classification() {
    assert_eq!(
        CorrelationStrength::from_coefficient(0.1),
        CorrelationStrength::VeryWeak
    );
    assert_eq!(
        CorrelationStrength::from_coefficient(0.3),
        CorrelationStrength::Weak
    );
    assert_eq!(
        CorrelationStrength::from_coefficient(0.5),
        CorrelationStrength::Moderate
    );
    assert_eq!(
        CorrelationStrength::from_coefficient(0.7),
        CorrelationStrength::Strong
    );
    assert_eq!(
        CorrelationStrength::from_coefficient(0.9),
        CorrelationStrength::VeryStrong
    );
}

#[test]
fn test_correlation_strength_labels() {
    assert_eq!(CorrelationStrength::VeryWeak.label(), "Very Weak");
    assert_eq!(CorrelationStrength::Weak.label(), "Weak");
    assert_eq!(CorrelationStrength::Moderate.label(), "Moderate");
    assert_eq!(CorrelationStrength::Strong.label(), "Strong");
    assert_eq!(CorrelationStrength::VeryStrong.label(), "Very Strong");
}

#[test]
fn test_correlation_pairs_mock() {
    let pairs = CorrelationPair::create_mock_correlations();
    assert!(!pairs.is_empty());
    assert!(pairs.iter().all(|p| p.coefficient.abs() <= 1.0));
    assert!(pairs.iter().all(|p| p.p_value >= 0.0 && p.p_value <= 1.0));
}

// ===== Trend Analysis Tests =====

#[test]
fn test_trend_direction_labels() {
    assert_eq!(TrendDirection::Increasing.label(), "Increasing");
    assert_eq!(TrendDirection::Decreasing.label(), "Decreasing");
    assert_eq!(TrendDirection::Stable.label(), "Stable");
    assert_eq!(TrendDirection::Volatile.label(), "Volatile");
}

#[test]
fn test_trend_direction_icons() {
    assert_eq!(TrendDirection::Increasing.icon(), "↗");
    assert_eq!(TrendDirection::Decreasing.icon(), "↘");
    assert_eq!(TrendDirection::Stable.icon(), "→");
    assert_eq!(TrendDirection::Volatile.icon(), "↕");
}

#[test]
fn test_trend_analysis_mock() {
    let trends = TrendAnalysis::create_mock_trends();
    assert!(!trends.is_empty());
    assert!(
        trends
            .iter()
            .all(|t| t.confidence >= 0.0 && t.confidence <= 1.0)
    );
    assert!(trends.iter().all(|t| t.prediction.is_some()));
}

// ===== Outlier Detection Tests =====

#[test]
fn test_outlier_severity_from_z_score() {
    assert_eq!(OutlierSeverity::from_z_score(1.5), OutlierSeverity::Mild);
    assert_eq!(
        OutlierSeverity::from_z_score(2.3),
        OutlierSeverity::Moderate
    );
    assert_eq!(OutlierSeverity::from_z_score(2.8), OutlierSeverity::Severe);
    assert_eq!(OutlierSeverity::from_z_score(3.5), OutlierSeverity::Extreme);
}

#[test]
fn test_outlier_severity_labels() {
    assert_eq!(OutlierSeverity::Mild.label(), "Mild");
    assert_eq!(OutlierSeverity::Moderate.label(), "Moderate");
    assert_eq!(OutlierSeverity::Severe.label(), "Severe");
    assert_eq!(OutlierSeverity::Extreme.label(), "Extreme");
}

#[test]
fn test_outlier_data_mock() {
    let outliers = OutlierData::create_mock_outliers();
    assert!(!outliers.is_empty());
    assert!(outliers.iter().all(|o| o.outlier_count <= o.total_points));
    assert!(outliers.iter().all(|o| !o.outliers.is_empty()));
}

// ===== Analysis Type Tests =====

#[test]
fn test_analysis_type_all() {
    let all = AnalysisType::all();
    assert_eq!(all.len(), 4);
    assert!(all.contains(&AnalysisType::Distribution));
    assert!(all.contains(&AnalysisType::Correlation));
    assert!(all.contains(&AnalysisType::Trend));
    assert!(all.contains(&AnalysisType::Outlier));
}

#[test]
fn test_analysis_type_names() {
    assert_eq!(AnalysisType::Distribution.name(), "Distribution Analysis");
    assert_eq!(AnalysisType::Correlation.name(), "Correlation Analysis");
    assert_eq!(AnalysisType::Trend.name(), "Trend Analysis");
    assert_eq!(AnalysisType::Outlier.name(), "Outlier Detection");
}

#[test]
fn test_analysis_type_descriptions() {
    assert!(
        AnalysisType::Distribution
            .description()
            .contains("Statistical")
    );
    assert!(
        AnalysisType::Correlation
            .description()
            .contains("Correlation")
    );
    assert!(AnalysisType::Trend.description().contains("Trend"));
    assert!(AnalysisType::Outlier.description().contains("Outlier"));
}

// ===== Key Handling Tests =====

#[tokio::test]
async fn test_quit_key() {
    let mut view = create_test_view();
    let mut app_state = create_app_state();

    let key = KeyEvent::from(KeyCode::Char('q'));
    let result = view.handle_key(key, &mut app_state).await.unwrap();

    assert!(matches!(result, ViewAction::PopView));
}

#[tokio::test]
async fn test_ctrl_c_key() {
    let mut view = create_test_view();
    let mut app_state = create_app_state();

    let key = KeyEvent::new(KeyCode::Char('c'), KeyModifiers::CONTROL);
    let result = view.handle_key(key, &mut app_state).await.unwrap();

    assert!(matches!(result, ViewAction::Quit));
}

#[tokio::test]
async fn test_help_key() {
    let mut view = create_test_view();
    let mut app_state = create_app_state();

    let key = KeyEvent::from(KeyCode::Char('?'));
    let result = view.handle_key(key, &mut app_state).await.unwrap();

    assert!(matches!(result, ViewAction::ShowHelp));
}

#[test]
fn test_help_text() {
    let view = create_test_view();
    let help = view.help_text();

    assert!(!help.is_empty());
    assert!(help.iter().any(|(key, _)| *key == "Tab"));
    assert!(help.iter().any(|(key, _)| *key == "↑/↓"));
    assert!(help.iter().any(|(key, _)| *key == "q"));
}

// ===== Rendering Tests =====

#[tokio::test]
async fn test_view_renders_without_error() {
    let mut view = create_test_view();
    let app_state = create_app_state();

    let backend = TestBackend::new(80, 24);
    let mut terminal = Terminal::new(backend).unwrap();

    terminal
        .draw(|f| {
            let area = ratatui::layout::Rect::new(0, 0, 80, 24);
            view.render(f, area, &app_state);
        })
        .unwrap();
}

#[tokio::test]
async fn test_all_analysis_types_render() {
    let mut view = create_test_view();
    let mut app_state = create_app_state();

    let backend = TestBackend::new(80, 24);
    let mut terminal = Terminal::new(backend).unwrap();

    for _ in 0..4 {
        terminal
            .draw(|f| {
                let area = ratatui::layout::Rect::new(0, 0, 80, 24);
                view.render(f, area, &app_state);
            })
            .unwrap();

        let key = KeyEvent::from(KeyCode::Tab);
        let _ = view.handle_key(key, &mut app_state).await;
    }
}

// ===== State Management Tests =====

#[test]
fn test_state_reset_on_analysis_change() {
    let mut state = AnalysisToolsState::new();
    state.selected_item = 5;
    state.scroll_offset = 10;

    state.next_analysis();

    assert_eq!(state.selected_item, 0);
    assert_eq!(state.scroll_offset, 0);
}

#[test]
fn test_state_scroll_limits() {
    let mut state = AnalysisToolsState::new();

    for _ in 0..30 {
        state.scroll_down();
    }
    assert_eq!(state.scroll_offset, 20);

    for _ in 0..30 {
        state.scroll_up();
    }
    assert_eq!(state.scroll_offset, 0);
}

// ===== Update Tests =====

#[tokio::test]
async fn test_view_update() {
    let mut view = create_test_view();
    let mut app_state = create_app_state();

    let result = view.update(&mut app_state).await;
    assert!(result.is_ok());
}
