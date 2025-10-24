/// Tests for Dashboard Components (Issue #24 Phase 2)

use dora_cli::tui::{
    components::{
        Component, PerformanceChartsComponent, RecentActivityComponent,
        SystemOverviewComponent, DataflowSummaryComponent,
    },
    views::{
        SystemOverview, SystemStatus, MemoryUsage, DiskUsage, NetworkActivity,
        DataflowSummary, PerformanceMetrics, RecentActivity, ActivityItem, ActivityType,
    },
};
use std::time::{Duration, Instant};

// ===== SystemOverviewComponent Tests =====

#[test]
fn test_system_overview_component_creation() {
    let component = SystemOverviewComponent::new();
    assert!(!component.is_focused());
    assert!(!component.is_focusable());
}

#[test]
fn test_system_overview_component_set_data() {
    let mut component = SystemOverviewComponent::new();

    let overview = SystemOverview {
        status: SystemStatus::Connected,
        uptime: Duration::from_secs(3600),
        version: "0.1.0".to_string(),
        cpu_usage: 45.5,
        memory_usage: MemoryUsage {
            total_mb: 8192,
            used_mb: 4096,
            usage_percent: 50.0,
        },
        disk_usage: DiskUsage {
            total_gb: 500,
            used_gb: 250,
            usage_percent: 50.0,
        },
        network_activity: NetworkActivity {
            bytes_received: 1024,
            bytes_transmitted: 2048,
        },
        active_connections: 5,
    };

    component.set_data(overview);
    // Component should store data without panic
}

// ===== DataflowSummaryComponent Tests =====

#[test]
fn test_dataflow_summary_component_creation() {
    let component = DataflowSummaryComponent::new();
    assert!(!component.is_focused());
    assert!(component.is_focusable()); // This one is focusable
}

#[test]
fn test_dataflow_summary_component_set_data() {
    let mut component = DataflowSummaryComponent::new();

    let summary = DataflowSummary {
        total_dataflows: 10,
        running_dataflows: 7,
        failed_dataflows: 1,
        stopped_dataflows: 2,
        total_nodes: 50,
        healthy_nodes: 45,
        unhealthy_nodes: 5,
        recent_deployments: Vec::new(),
    };

    component.set_data(summary.clone());

    // Verify health calculations
    assert_eq!(summary.health_percentage(), 70.0); // 7/10 = 70%
    assert_eq!(summary.node_health_percentage(), 90.0); // 45/50 = 90%
}

// ===== PerformanceChartsComponent Tests =====

#[test]
fn test_performance_charts_component_creation() {
    let component = PerformanceChartsComponent::new();
    assert!(!component.is_focused());
    assert!(!component.is_focusable());
}

#[test]
fn test_performance_charts_add_metrics() {
    let mut component = PerformanceChartsComponent::new();

    // Add some metrics
    component.add_metrics(45.5, 60.2);
    component.add_metrics(50.0, 65.5);
    component.add_metrics(55.3, 70.0);

    // Component should store metrics without panic
}

#[test]
fn test_performance_metrics_history_management() {
    let mut metrics = PerformanceMetrics::new(5);

    // Add more points than the limit
    for i in 0..10 {
        metrics.add_cpu_point(i as f64);
    }

    // Should only keep last 5 points
    assert_eq!(metrics.cpu_history.len(), 5);
    assert_eq!(metrics.cpu_history[0].value, 5.0); // Oldest kept
    assert_eq!(metrics.cpu_history[4].value, 9.0); // Newest
}

// ===== RecentActivityComponent Tests =====

#[test]
fn test_recent_activity_component_creation() {
    let component = RecentActivityComponent::new(10);
    assert!(!component.is_focused());
    assert!(!component.is_focusable());
}

#[test]
fn test_recent_activity_component_set_data() {
    let mut component = RecentActivityComponent::new(10);

    let mut activity = RecentActivity::default();
    activity.add_activity(ActivityItem {
        timestamp: Instant::now(),
        event_type: ActivityType::DataflowStarted,
        description: "Test dataflow started".to_string(),
        dataflow_name: Some("test".to_string()),
    });

    component.set_activity(activity);
    // Component should store activity without panic
}

#[test]
fn test_recent_activity_limit() {
    let mut activity = RecentActivity::default();
    activity.max_items = 5;

    // Add more items than the limit
    for i in 0..10 {
        activity.add_activity(ActivityItem {
            timestamp: Instant::now(),
            event_type: ActivityType::SystemEvent,
            description: format!("Event {}", i),
            dataflow_name: None,
        });
    }

    // Should only keep last 5 items
    assert_eq!(activity.activities.len(), 5);
}

#[test]
fn test_recent_activity_get_recent() {
    let mut activity = RecentActivity::default();

    // Add some activities
    for i in 0..5 {
        activity.add_activity(ActivityItem {
            timestamp: Instant::now(),
            event_type: ActivityType::SystemEvent,
            description: format!("Event {}", i),
            dataflow_name: None,
        });
    }

    // Get recent 3
    let recent = activity.recent(3);
    assert_eq!(recent.len(), 3);

    // Should be in reverse order (newest first)
    assert!(recent[0].description.contains("4")); // Newest
    assert!(recent[2].description.contains("2")); // Oldest of the 3
}

#[test]
fn test_activity_type_icons() {
    assert_eq!(ActivityType::DataflowStarted.icon(), "▶");
    assert_eq!(ActivityType::DataflowStopped.icon(), "■");
    assert_eq!(ActivityType::DataflowFailed.icon(), "✗");
    assert_eq!(ActivityType::NodeStarted.icon(), "→");
    assert_eq!(ActivityType::SystemEvent.icon(), "ℹ");
}

// ===== Integration Tests =====

#[test]
fn test_all_dashboard_components_can_be_created() {
    let _system_overview = SystemOverviewComponent::new();
    let _dataflow_summary = DataflowSummaryComponent::new();
    let _performance_charts = PerformanceChartsComponent::new();
    let _recent_activity = RecentActivityComponent::new(10);

    // All components should be creatable without panic
}

#[test]
fn test_dashboard_data_structures_defaults() {
    let _overview = SystemOverview::default();
    let _summary = DataflowSummary::default();
    let _metrics = PerformanceMetrics::default();
    let _activity = RecentActivity::default();

    // All data structures should have sensible defaults
}
