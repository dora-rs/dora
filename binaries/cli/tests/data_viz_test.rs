// Comprehensive tests for Data Visualization View (Issue #32 - Phase 1)
// Tests cover all chart types, navigation, state management, and mock data

use crossterm::event::{KeyCode, KeyEvent, KeyModifiers};
use dora_cli::tui::{
    app::AppState,
    theme::ThemeConfig,
    views::{
        CategoryData, ChartData, ChartType, DataPoint, DataVizState, DataVizView, EventData,
        EventSeverity, GaugeData, TimeSeriesData, View, ViewAction,
    },
};
use ratatui::{Terminal, backend::TestBackend};

// Helper function to create a test view
fn create_test_view() -> DataVizView {
    DataVizView::new(&ThemeConfig::default())
}

// Helper function to create app state
fn create_app_state() -> AppState {
    AppState::default()
}

// ===== View Creation and Initialization Tests =====

#[test]
fn test_view_creation() {
    let view = create_test_view();
    assert_eq!(view.title(), "Data Visualization");
    assert_eq!(view.state.current_chart_type, ChartType::LineChart);
}

#[test]
fn test_initial_state() {
    let view = create_test_view();
    assert_eq!(view.state.zoom_level, 1.0);
    assert_eq!(view.state.scroll_offset, 0.0);
    assert_eq!(view.state.current_chart_type, ChartType::LineChart);
}

#[test]
fn test_auto_refresh() {
    let view = create_test_view();
    assert!(view.auto_refresh().is_some());
}

// ===== Chart Type Navigation Tests =====

#[tokio::test]
async fn test_chart_type_navigation_tab() {
    let mut view = create_test_view();
    let mut app_state = create_app_state();

    // Tab should cycle through chart types
    let key = KeyEvent::from(KeyCode::Tab);
    let _ = view.handle_key(key, &mut app_state).await;
    assert_eq!(view.state.current_chart_type, ChartType::BarChart);

    let _ = view.handle_key(key, &mut app_state).await;
    assert_eq!(view.state.current_chart_type, ChartType::ScatterPlot);

    let _ = view.handle_key(key, &mut app_state).await;
    assert_eq!(view.state.current_chart_type, ChartType::Gauge);

    let _ = view.handle_key(key, &mut app_state).await;
    assert_eq!(view.state.current_chart_type, ChartType::Timeline);

    let _ = view.handle_key(key, &mut app_state).await;
    assert_eq!(view.state.current_chart_type, ChartType::LineChart); // Wrap around
}

#[tokio::test]
async fn test_chart_type_navigation_backtab() {
    let mut view = create_test_view();
    let mut app_state = create_app_state();

    // BackTab should cycle backwards
    let key = KeyEvent::from(KeyCode::BackTab);
    let _ = view.handle_key(key, &mut app_state).await;
    assert_eq!(view.state.current_chart_type, ChartType::Timeline); // Wrap around

    let _ = view.handle_key(key, &mut app_state).await;
    assert_eq!(view.state.current_chart_type, ChartType::Gauge);
}

#[test]
fn test_chart_data_updates_on_type_change() {
    let mut state = DataVizState::new();
    assert!(matches!(state.chart_data, ChartData::TimeSeries(_)));

    state.next_chart_type();
    assert!(matches!(state.chart_data, ChartData::Category(_)));

    state.next_chart_type();
    assert!(matches!(state.chart_data, ChartData::TimeSeries(_)));
}

// ===== Zoom Tests =====

#[tokio::test]
async fn test_zoom_in() {
    let mut view = create_test_view();
    let mut app_state = create_app_state();

    let initial_zoom = view.state.zoom_level;
    let key = KeyEvent::from(KeyCode::Char('+'));
    let _ = view.handle_key(key, &mut app_state).await;

    assert!(view.state.zoom_level > initial_zoom);
}

#[tokio::test]
async fn test_zoom_out() {
    let mut view = create_test_view();
    let mut app_state = create_app_state();

    // First zoom in
    let key_in = KeyEvent::from(KeyCode::Char('+'));
    let _ = view.handle_key(key_in, &mut app_state).await;

    let zoomed_level = view.state.zoom_level;

    // Then zoom out
    let key_out = KeyEvent::from(KeyCode::Char('-'));
    let _ = view.handle_key(key_out, &mut app_state).await;

    assert!(view.state.zoom_level < zoomed_level);
}

#[tokio::test]
async fn test_zoom_with_equals_key() {
    let mut view = create_test_view();
    let mut app_state = create_app_state();

    let key = KeyEvent::from(KeyCode::Char('='));
    let _ = view.handle_key(key, &mut app_state).await;

    assert!(view.state.zoom_level > 1.0);
}

#[tokio::test]
async fn test_zoom_with_underscore_key() {
    let mut view = create_test_view();
    let mut app_state = create_app_state();

    // First zoom in
    let key_in = KeyEvent::from(KeyCode::Char('+'));
    let _ = view.handle_key(key_in, &mut app_state).await;

    // Then zoom out with underscore
    let key_out = KeyEvent::from(KeyCode::Char('_'));
    let _ = view.handle_key(key_out, &mut app_state).await;

    assert_eq!(view.state.zoom_level, 1.0);
}

#[tokio::test]
async fn test_reset_zoom() {
    let mut view = create_test_view();
    let mut app_state = create_app_state();

    // Zoom in multiple times
    let key = KeyEvent::from(KeyCode::Char('+'));
    for _ in 0..5 {
        let _ = view.handle_key(key, &mut app_state).await;
    }

    // Reset zoom
    let reset_key = KeyEvent::from(KeyCode::Char('0'));
    let _ = view.handle_key(reset_key, &mut app_state).await;

    assert_eq!(view.state.zoom_level, 1.0);
    assert_eq!(view.state.scroll_offset, 0.0);
}

#[test]
fn test_zoom_limits() {
    let mut state = DataVizState::new();

    // Test zoom in limit
    for _ in 0..20 {
        state.zoom_in();
    }
    assert_eq!(state.zoom_level, 5.0);

    // Test zoom out limit
    for _ in 0..20 {
        state.zoom_out();
    }
    assert_eq!(state.zoom_level, 0.5);
}

// ===== Scroll Tests =====

#[tokio::test]
async fn test_scroll_down() {
    let mut view = create_test_view();
    let mut app_state = create_app_state();

    let key = KeyEvent::from(KeyCode::Down);
    let _ = view.handle_key(key, &mut app_state).await;

    assert_eq!(view.state.scroll_offset, 5.0);
}

#[tokio::test]
async fn test_scroll_up() {
    let mut view = create_test_view();
    let mut app_state = create_app_state();

    // First scroll down
    let key_down = KeyEvent::from(KeyCode::Down);
    let _ = view.handle_key(key_down, &mut app_state).await;

    // Then scroll up
    let key_up = KeyEvent::from(KeyCode::Up);
    let _ = view.handle_key(key_up, &mut app_state).await;

    assert_eq!(view.state.scroll_offset, 0.0);
}

#[tokio::test]
async fn test_scroll_with_j_k_keys() {
    let mut view = create_test_view();
    let mut app_state = create_app_state();

    // j for scroll down
    let key_j = KeyEvent::from(KeyCode::Char('j'));
    let _ = view.handle_key(key_j, &mut app_state).await;
    assert_eq!(view.state.scroll_offset, 5.0);

    // k for scroll up
    let key_k = KeyEvent::from(KeyCode::Char('k'));
    let _ = view.handle_key(key_k, &mut app_state).await;
    assert_eq!(view.state.scroll_offset, 0.0);
}

#[test]
fn test_scroll_limits() {
    let mut state = DataVizState::new();

    // Test scroll down limit
    for _ in 0..30 {
        state.scroll_down();
    }
    assert_eq!(state.scroll_offset, 100.0);

    // Test scroll up limit
    for _ in 0..30 {
        state.scroll_up();
    }
    assert_eq!(state.scroll_offset, 0.0);
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
async fn test_refresh_with_shift_r_key() {
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
fn test_mock_line_chart_data() {
    if let ChartData::TimeSeries(series) = ChartData::create_mock_line_chart_data() {
        assert_eq!(series.len(), 2);
        assert_eq!(series[0].series_name, "CPU Usage");
        assert_eq!(series[1].series_name, "Memory Usage");
        assert_eq!(series[0].points.len(), 50);
        assert_eq!(series[1].points.len(), 50);
    } else {
        panic!("Expected TimeSeries data");
    }
}

#[test]
fn test_mock_bar_chart_data() {
    if let ChartData::Category(categories) = ChartData::create_mock_bar_chart_data() {
        assert_eq!(categories.len(), 6);
        assert_eq!(categories[0].label, "Node A");
        assert_eq!(categories[0].value, 85.0);
    } else {
        panic!("Expected Category data");
    }
}

#[test]
fn test_mock_scatter_plot_data() {
    if let ChartData::TimeSeries(series) = ChartData::create_mock_scatter_plot_data() {
        assert_eq!(series.len(), 1);
        assert_eq!(series[0].series_name, "Latency vs Throughput");
        assert_eq!(series[0].points.len(), 40);
    } else {
        panic!("Expected TimeSeries data");
    }
}

#[test]
fn test_mock_gauge_data() {
    if let ChartData::Gauge(gauge) = ChartData::create_mock_gauge_data() {
        assert_eq!(gauge.label, "System Performance");
        assert_eq!(gauge.current_value, 78.5);
        assert_eq!(gauge.min_value, 0.0);
        assert_eq!(gauge.max_value, 100.0);
        assert_eq!(gauge.target_value, Some(80.0));
        assert_eq!(gauge.unit, "%");
    } else {
        panic!("Expected Gauge data");
    }
}

#[test]
fn test_mock_timeline_data() {
    if let ChartData::Events(events) = ChartData::create_mock_timeline_data() {
        assert_eq!(events.len(), 7);
        assert_eq!(events[0].label, "System Start");
        assert_eq!(events[0].severity, EventSeverity::Info);
        assert_eq!(events[3].severity, EventSeverity::Error);
        assert_eq!(events[5].severity, EventSeverity::Critical);
    } else {
        panic!("Expected Events data");
    }
}

// ===== Data Type Tests =====

#[test]
fn test_data_point_creation() {
    let point = DataPoint::new(10.0, 20.0);
    assert_eq!(point.x, 10.0);
    assert_eq!(point.y, 20.0);
}

#[test]
fn test_time_series_bounds() {
    let points = vec![
        DataPoint::new(0.0, 10.0),
        DataPoint::new(5.0, 30.0),
        DataPoint::new(10.0, 20.0),
    ];
    let series = TimeSeriesData::new("Test".to_string(), points, 0);

    assert_eq!(series.min_x(), 0.0);
    assert_eq!(series.max_x(), 10.0);
    assert_eq!(series.min_y(), 10.0);
    assert_eq!(series.max_y(), 30.0);
}

#[test]
fn test_category_data_creation() {
    let cat = CategoryData::new("Test".to_string(), 42.0, 1);
    assert_eq!(cat.label, "Test");
    assert_eq!(cat.value, 42.0);
    assert_eq!(cat.color, 1);
}

#[test]
fn test_event_severity() {
    assert_eq!(EventSeverity::Info.color_index(), 0);
    assert_eq!(EventSeverity::Warning.color_index(), 1);
    assert_eq!(EventSeverity::Error.color_index(), 2);
    assert_eq!(EventSeverity::Critical.color_index(), 3);

    assert_eq!(EventSeverity::Info.label(), "INFO");
    assert_eq!(EventSeverity::Warning.label(), "WARN");
    assert_eq!(EventSeverity::Error.label(), "ERROR");
    assert_eq!(EventSeverity::Critical.label(), "CRITICAL");
}

#[test]
fn test_gauge_percentage() {
    let gauge = GaugeData::new(
        "Test".to_string(),
        75.0,
        0.0,
        100.0,
        Some(80.0),
        "%".to_string(),
    );
    assert_eq!(gauge.percentage(), 75.0);
}

#[test]
fn test_gauge_at_target() {
    let gauge1 = GaugeData::new(
        "Test".to_string(),
        80.0,
        0.0,
        100.0,
        Some(80.0),
        "%".to_string(),
    );
    assert!(gauge1.is_at_target());

    let gauge2 = GaugeData::new(
        "Test".to_string(),
        75.0,
        0.0,
        100.0,
        Some(80.0),
        "%".to_string(),
    );
    assert!(!gauge2.is_at_target());
}

#[test]
fn test_event_data_creation() {
    let event = EventData::new(
        10.0,
        "Test Event".to_string(),
        "Description".to_string(),
        EventSeverity::Warning,
    );
    assert_eq!(event.timestamp, 10.0);
    assert_eq!(event.label, "Test Event");
    assert_eq!(event.description, "Description");
    assert_eq!(event.severity, EventSeverity::Warning);
}

// ===== Chart Type Tests =====

#[test]
fn test_chart_type_all() {
    let all = ChartType::all();
    assert_eq!(all.len(), 5);
    assert!(all.contains(&ChartType::LineChart));
    assert!(all.contains(&ChartType::BarChart));
    assert!(all.contains(&ChartType::ScatterPlot));
    assert!(all.contains(&ChartType::Gauge));
    assert!(all.contains(&ChartType::Timeline));
}

#[test]
fn test_chart_type_names() {
    assert_eq!(ChartType::LineChart.name(), "Line Chart");
    assert_eq!(ChartType::BarChart.name(), "Bar Chart");
    assert_eq!(ChartType::ScatterPlot.name(), "Scatter Plot");
    assert_eq!(ChartType::Gauge.name(), "Gauge");
    assert_eq!(ChartType::Timeline.name(), "Timeline");
}

#[test]
fn test_chart_type_descriptions() {
    assert!(ChartType::LineChart.description().contains("time-series"));
    assert!(ChartType::BarChart.description().contains("categorical"));
    assert!(
        ChartType::ScatterPlot
            .description()
            .contains("relationships")
    );
    assert!(ChartType::Gauge.description().contains("progress"));
    assert!(ChartType::Timeline.description().contains("events"));
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
    assert!(help.iter().any(|(key, _)| *key == "+/-"));
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
async fn test_all_chart_types_render() {
    let mut view = create_test_view();
    let mut app_state = create_app_state();

    let backend = TestBackend::new(80, 24);
    let mut terminal = Terminal::new(backend).unwrap();

    for _ in 0..5 {
        terminal
            .draw(|f| {
                let area = ratatui::layout::Rect::new(0, 0, 80, 24);
                view.render(f, area, &app_state);
            })
            .unwrap();

        // Switch to next chart type
        let key = KeyEvent::from(KeyCode::Tab);
        let _ = view.handle_key(key, &mut app_state).await;
    }
}

// ===== Update Tests =====

#[tokio::test]
async fn test_view_update() {
    let mut view = create_test_view();
    let mut app_state = create_app_state();

    let result = view.update(&mut app_state).await;
    assert!(result.is_ok());
}
