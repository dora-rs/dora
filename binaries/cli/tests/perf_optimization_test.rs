// Integration tests for Performance Optimization Feature (Issue #36)

use crossterm::event::{KeyCode, KeyEvent};
use dora_cli::tui::{
    app::AppState,
    views::{
        View, ViewAction, perf_optimization::PerfOptimizationView, perf_optimization_types::*,
    },
};

#[tokio::test]
async fn test_perf_optimization_view_initialization() {
    let view = PerfOptimizationView::new();

    assert_eq!(view.state.current_section, PerfSection::PerformanceMetrics);
    assert_eq!(view.state.selected_index, 0);
    assert!(matches!(view.state.data, PerfData::Metrics(_)));
}

#[tokio::test]
async fn test_section_navigation() {
    let mut view = PerfOptimizationView::new();
    let mut app_state = AppState::default();

    // Navigate through all sections forward
    let sections = PerfSection::all();
    for (i, expected_section) in sections.iter().enumerate() {
        if i > 0 {
            let result = view
                .handle_key(KeyEvent::from(KeyCode::Tab), &mut app_state)
                .await;
            assert!(result.is_ok());
        }
        assert_eq!(view.state.current_section, *expected_section);
    }

    // Wrap around
    let result = view
        .handle_key(KeyEvent::from(KeyCode::Tab), &mut app_state)
        .await;
    assert!(result.is_ok());
    assert_eq!(view.state.current_section, PerfSection::PerformanceMetrics);
}

#[tokio::test]
async fn test_section_navigation_backward() {
    let mut view = PerfOptimizationView::new();
    let mut app_state = AppState::default();

    // Go backward from first section (should wrap to last)
    let result = view
        .handle_key(KeyEvent::from(KeyCode::BackTab), &mut app_state)
        .await;
    assert!(result.is_ok());
    assert_eq!(view.state.current_section, PerfSection::ResourceAnalysis);

    // Go backward again
    let result = view
        .handle_key(KeyEvent::from(KeyCode::BackTab), &mut app_state)
        .await;
    assert!(result.is_ok());
    assert_eq!(
        view.state.current_section,
        PerfSection::OptimizationSuggestions
    );
}

#[tokio::test]
async fn test_item_navigation() {
    let mut view = PerfOptimizationView::new();
    let mut app_state = AppState::default();

    // Navigate to section with multiple items
    view.state.current_section = PerfSection::BottleneckDetection;
    view.state.update_data();

    let initial_index = view.state.selected_index;

    // Navigate down
    let result = view
        .handle_key(KeyEvent::from(KeyCode::Down), &mut app_state)
        .await;
    assert!(result.is_ok());
    assert_eq!(view.state.selected_index, initial_index + 1);

    // Navigate up
    let result = view
        .handle_key(KeyEvent::from(KeyCode::Up), &mut app_state)
        .await;
    assert!(result.is_ok());
    assert_eq!(view.state.selected_index, initial_index);
}

#[tokio::test]
async fn test_vim_style_navigation() {
    let mut view = PerfOptimizationView::new();
    let mut app_state = AppState::default();

    view.state.current_section = PerfSection::OptimizationSuggestions;
    view.state.update_data();

    // Test 'j' (down)
    let result = view
        .handle_key(KeyEvent::from(KeyCode::Char('j')), &mut app_state)
        .await;
    assert!(result.is_ok());
    assert!(view.state.selected_index > 0);

    // Test 'k' (up)
    let result = view
        .handle_key(KeyEvent::from(KeyCode::Char('k')), &mut app_state)
        .await;
    assert!(result.is_ok());
    assert_eq!(view.state.selected_index, 0);
}

#[tokio::test]
async fn test_quit_action() {
    let mut view = PerfOptimizationView::new();
    let mut app_state = AppState::default();

    let result = view
        .handle_key(KeyEvent::from(KeyCode::Char('q')), &mut app_state)
        .await;
    assert!(result.is_ok());
    assert!(matches!(result.unwrap(), ViewAction::PopView));
}

#[tokio::test]
async fn test_refresh_action() {
    let mut view = PerfOptimizationView::new();
    let mut app_state = AppState::default();

    let result = view
        .handle_key(KeyEvent::from(KeyCode::Char('r')), &mut app_state)
        .await;
    assert!(result.is_ok());
    assert!(matches!(result.unwrap(), ViewAction::None));
}

#[test]
fn test_perf_section_properties() {
    let sections = PerfSection::all();

    for section in sections {
        // All sections should have non-empty title and description
        assert!(!section.title().is_empty());
        assert!(!section.description().is_empty());
    }
}

#[test]
fn test_performance_metrics_mock_data() {
    let metrics = create_mock_metrics();

    assert!(!metrics.is_empty());

    for metric in metrics {
        assert!(!metric.name.is_empty());
        assert!(!metric.unit.is_empty());
        assert!(metric.current_value >= 0.0);
        assert!(metric.min_value <= metric.max_value);
        assert!(metric.min_value <= metric.current_value);
        assert!(metric.current_value <= metric.max_value);
    }
}

#[test]
fn test_bottlenecks_mock_data() {
    let bottlenecks = create_mock_bottlenecks();

    assert!(!bottlenecks.is_empty());

    for bottleneck in bottlenecks {
        assert!(!bottleneck.bottleneck_id.is_empty());
        assert!(!bottleneck.impact.is_empty());
        assert!(!bottleneck.affected_components.is_empty());
        assert!(bottleneck.severity.priority() > 0);
    }
}

#[test]
fn test_suggestions_mock_data() {
    let suggestions = create_mock_suggestions();

    assert!(!suggestions.is_empty());

    for suggestion in suggestions {
        assert!(!suggestion.suggestion_id.is_empty());
        assert!(!suggestion.title.is_empty());
        assert!(!suggestion.description.is_empty());
        assert!(!suggestion.implementation_steps.is_empty());
        assert!(suggestion.priority.value() > 0);
    }
}

#[test]
fn test_resources_mock_data() {
    let resources = create_mock_resources();

    assert!(!resources.is_empty());

    for resource in resources {
        assert!(!resource.component.is_empty());
        assert!(resource.cpu_percent >= 0.0 && resource.cpu_percent <= 100.0);
        assert!(resource.memory_mb >= 0.0);
        assert!(resource.thread_count > 0);
    }
}

#[test]
fn test_metric_type_variants() {
    let types = vec![
        PerfMetricType::Throughput,
        PerfMetricType::Latency,
        PerfMetricType::CpuUsage,
        PerfMetricType::MemoryUsage,
        PerfMetricType::NetworkIO,
        PerfMetricType::DiskIO,
        PerfMetricType::MessageRate,
        PerfMetricType::ErrorRate,
    ];

    for metric_type in types {
        assert!(!metric_type.name().is_empty());
    }
}

#[test]
fn test_metric_status_variants() {
    let statuses = vec![
        MetricStatus::Healthy,
        MetricStatus::Warning,
        MetricStatus::Critical,
        MetricStatus::Unknown,
    ];

    for status in statuses {
        assert!(!status.name().is_empty());
    }
}

#[test]
fn test_bottleneck_type_variants() {
    let types = vec![
        BottleneckType::CpuBound,
        BottleneckType::MemoryBound,
        BottleneckType::IOBound,
        BottleneckType::NetworkBound,
        BottleneckType::Synchronization,
        BottleneckType::QueueBacklog,
    ];

    for bottleneck_type in types {
        assert!(!bottleneck_type.name().is_empty());
    }
}

#[test]
fn test_severity_ordering() {
    assert!(Severity::Critical.priority() > Severity::High.priority());
    assert!(Severity::High.priority() > Severity::Medium.priority());
    assert!(Severity::Medium.priority() > Severity::Low.priority());
}

#[test]
fn test_priority_ordering() {
    assert!(Priority::Critical.value() > Priority::High.value());
    assert!(Priority::High.value() > Priority::Medium.value());
    assert!(Priority::Medium.value() > Priority::Low.value());
}

#[test]
fn test_optimization_category_variants() {
    let categories = vec![
        OptimizationCategory::Architecture,
        OptimizationCategory::Configuration,
        OptimizationCategory::ResourceAllocation,
        OptimizationCategory::CodeOptimization,
        OptimizationCategory::Caching,
        OptimizationCategory::Parallelization,
    ];

    for category in categories {
        assert!(!category.name().is_empty());
    }
}

#[test]
fn test_impact_level_variants() {
    let levels = vec![
        ImpactLevel::Minimal,
        ImpactLevel::Moderate,
        ImpactLevel::Significant,
        ImpactLevel::Major,
    ];

    for level in levels {
        assert!(!level.name().is_empty());
    }
}

#[test]
fn test_effort_level_variants() {
    let levels = vec![
        EffortLevel::Trivial,
        EffortLevel::Low,
        EffortLevel::Medium,
        EffortLevel::High,
        EffortLevel::VeryHigh,
    ];

    for level in levels {
        assert!(!level.name().is_empty());
    }
}

#[test]
fn test_resource_type_variants() {
    let types = vec![
        ResourceType::Cpu,
        ResourceType::Memory,
        ResourceType::Network,
        ResourceType::Disk,
    ];

    for resource_type in types {
        assert!(!resource_type.name().is_empty());
    }
}

#[test]
fn test_trend_direction_variants() {
    let trends = vec![
        TrendDirection::Increasing,
        TrendDirection::Decreasing,
        TrendDirection::Stable,
        TrendDirection::Volatile,
    ];

    for trend in trends {
        assert!(!trend.name().is_empty());
    }
}

#[test]
fn test_bottleneck_location_display() {
    let node_loc = BottleneckLocation::Node {
        node_id: "test_node".to_string(),
    };
    assert!(node_loc.display().contains("test_node"));

    let link_loc = BottleneckLocation::Link {
        from: "source".to_string(),
        to: "dest".to_string(),
    };
    assert!(link_loc.display().contains("source"));
    assert!(link_loc.display().contains("dest"));

    let system_loc = BottleneckLocation::System;
    assert_eq!(system_loc.display(), "System");

    let network_loc = BottleneckLocation::Network;
    assert_eq!(network_loc.display(), "Network");
}

#[test]
fn test_perf_optimization_state_data_switching() {
    let mut state = PerfOptimizationState::new();

    // Start with PerformanceMetrics
    assert!(matches!(state.data, PerfData::Metrics(_)));

    // Switch to BottleneckDetection
    state.next_section();
    assert!(matches!(state.data, PerfData::Bottlenecks(_)));

    // Switch to OptimizationSuggestions
    state.next_section();
    assert!(matches!(state.data, PerfData::Suggestions(_)));

    // Switch to ResourceAnalysis
    state.next_section();
    assert!(matches!(state.data, PerfData::Resources(_)));
}

#[test]
fn test_perf_optimization_state_item_bounds() {
    let mut state = PerfOptimizationState::new();

    // Navigate to section with items
    state.current_section = PerfSection::OptimizationSuggestions;
    state.update_data();

    // Navigate down multiple times
    for _ in 0..100 {
        state.next_item();
    }

    // Should be bounded
    let max_index = match &state.data {
        PerfData::Suggestions(suggestions) => suggestions.len().saturating_sub(1),
        _ => 0,
    };
    assert!(state.selected_index <= max_index);
}

#[test]
fn test_perf_optimization_state_item_minimum() {
    let mut state = PerfOptimizationState::new();

    // Navigate up at index 0
    state.previous_item();
    assert_eq!(state.selected_index, 0);
}

#[test]
fn test_view_title_reflects_base() {
    let view = PerfOptimizationView::new();
    assert_eq!(view.title(), "Performance Optimization");
}

#[test]
fn test_view_help_text_completeness() {
    let view = PerfOptimizationView::new();
    let help = view.help_text();

    assert!(help.len() >= 4);
    assert!(help.iter().any(|(k, _)| k.contains("Tab")));
    assert!(help.iter().any(|(k, _)| k.contains("j") || k.contains("k")));
    assert!(help.iter().any(|(k, _)| k.contains("r")));
    assert!(help.iter().any(|(k, _)| k.contains("q")));
}

#[test]
fn test_metric_value_ranges_validity() {
    let metrics = create_mock_metrics();

    for metric in metrics {
        assert!(metric.min_value <= metric.avg_value);
        assert!(metric.avg_value <= metric.max_value);
        assert!(metric.min_value <= metric.current_value);
        assert!(metric.current_value <= metric.max_value);
    }
}

#[test]
fn test_resource_usage_validity() {
    let resources = create_mock_resources();

    for resource in resources {
        assert!(resource.cpu_percent <= 100.0);
        assert!(resource.memory_mb >= 0.0);
        assert!(resource.network_rx_mbps >= 0.0);
        assert!(resource.network_tx_mbps >= 0.0);
        assert!(resource.disk_read_mbps >= 0.0);
        assert!(resource.disk_write_mbps >= 0.0);
    }
}

#[test]
fn test_bottleneck_severity_levels() {
    let bottlenecks = create_mock_bottlenecks();

    for bottleneck in bottlenecks {
        let priority = bottleneck.severity.priority();
        assert!(priority >= 1 && priority <= 4);
    }
}

#[test]
fn test_suggestion_priority_levels() {
    let suggestions = create_mock_suggestions();

    for suggestion in suggestions {
        let value = suggestion.priority.value();
        assert!(value >= 1 && value <= 4);
    }
}

#[tokio::test]
async fn test_complete_navigation_cycle() {
    let mut view = PerfOptimizationView::new();
    let mut app_state = AppState::default();

    // Navigate through all sections and verify data updates
    for _ in 0..PerfSection::all().len() {
        // Navigate to next section
        let result = view
            .handle_key(KeyEvent::from(KeyCode::Tab), &mut app_state)
            .await;
        assert!(result.is_ok());

        // Verify selection resets
        assert_eq!(view.state.selected_index, 0);
    }

    // Should be back to start
    assert_eq!(view.state.current_section, PerfSection::PerformanceMetrics);
}

#[test]
fn test_mock_data_consistency() {
    // Verify mock data generators produce consistent results
    let metrics1 = create_mock_metrics();
    let metrics2 = create_mock_metrics();

    assert_eq!(metrics1.len(), metrics2.len());
    assert_eq!(metrics1[0].name, metrics2[0].name);

    let bottlenecks1 = create_mock_bottlenecks();
    let bottlenecks2 = create_mock_bottlenecks();

    assert_eq!(bottlenecks1.len(), bottlenecks2.len());
    assert_eq!(bottlenecks1[0].bottleneck_id, bottlenecks2[0].bottleneck_id);
}

#[test]
fn test_suggestion_implementation_steps() {
    let suggestions = create_mock_suggestions();

    for suggestion in suggestions {
        assert!(!suggestion.implementation_steps.is_empty());
        for step in &suggestion.implementation_steps {
            assert!(!step.is_empty());
        }
    }
}

#[test]
fn test_bottleneck_affected_components() {
    let bottlenecks = create_mock_bottlenecks();

    for bottleneck in bottlenecks {
        assert!(!bottleneck.affected_components.is_empty());
        for component in &bottleneck.affected_components {
            assert!(!component.is_empty());
        }
    }
}
