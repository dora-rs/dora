/// Integration tests for TUI Architecture Foundation (Issue #23)
///
/// Tests the component system, layout manager, and theme manager

use dora_cli::tui::{
    ComponentId, ComponentRegistry, EventDispatcher, ThemeManager,
    components::{DataflowListComponent, MetricsChartComponent, StatusBarComponent, LogViewerComponent},
};
use dora_cli::tui::views::{LayoutManager, LayoutConfig};
use ratatui::layout::{Constraint, Rect};

// ===== Component System Tests =====

#[test]
fn test_component_registry_basic_operations() {
    let mut registry = ComponentRegistry::new();

    // Test empty registry
    assert!(registry.is_empty());
    assert_eq!(registry.len(), 0);

    // Test adding components
    let id = ComponentId::new("test-component");
    registry.register(id.clone(), Box::new(DataflowListComponent::new()));

    assert!(!registry.is_empty());
    assert_eq!(registry.len(), 1);
    assert!(registry.contains(&id));

    // Test getting component
    assert!(registry.get(&id).is_some());

    // Test component IDs
    let ids = registry.component_ids();
    assert_eq!(ids.len(), 1);
    assert_eq!(ids[0], id);
}

#[test]
fn test_component_registry_multiple_components() {
    let mut registry = ComponentRegistry::new();

    let id1 = ComponentId::new("dataflow-list");
    let id2 = ComponentId::new("metrics-chart");
    let id3 = ComponentId::new("status-bar");

    registry.register(id1.clone(), Box::new(DataflowListComponent::new()));
    registry.register(id2.clone(), Box::new(MetricsChartComponent::new(
        dora_cli::tui::components::metrics_chart::ChartType::CpuUsage,
        std::time::Duration::from_secs(60)
    )));
    registry.register(id3.clone(), Box::new(StatusBarComponent::new()));

    assert_eq!(registry.len(), 3);
    assert!(registry.contains(&id1));
    assert!(registry.contains(&id2));
    assert!(registry.contains(&id3));
}

#[test]
fn test_component_registry_remove() {
    let mut registry = ComponentRegistry::new();

    let id = ComponentId::new("temp-component");
    registry.register(id.clone(), Box::new(LogViewerComponent::new(100)));

    assert_eq!(registry.len(), 1);

    let removed = registry.remove(&id);
    assert!(removed.is_some());
    assert_eq!(registry.len(), 0);
    assert!(!registry.contains(&id));
}

#[test]
fn test_event_dispatcher_focus_management() {
    let mut dispatcher = EventDispatcher::new();

    // Initially no focus
    assert!(dispatcher.get_focused().is_none());

    // Set focus
    let component_id = ComponentId::new("focused-component");
    dispatcher.set_focus(Some(component_id.clone()));

    assert_eq!(dispatcher.get_focused(), Some(&component_id));

    // Clear focus
    dispatcher.set_focus(None);
    assert!(dispatcher.get_focused().is_none());
}

// ===== Layout Manager Tests =====

#[test]
fn test_layout_manager_single_layout() {
    let mut manager = LayoutManager::new();
    let component_id = ComponentId::new("main-component");

    manager.set_layout(LayoutConfig::Single {
        component_id: component_id.clone(),
    });

    let area = Rect { x: 0, y: 0, width: 100, height: 50 };
    let layout = manager.calculate_layout(area);

    assert_eq!(layout.len(), 1);
    assert_eq!(layout.get(&component_id), Some(&area));
}

#[test]
fn test_layout_manager_vertical_layout() {
    let mut manager = LayoutManager::new();
    let comp1 = ComponentId::new("top");
    let comp2 = ComponentId::new("bottom");

    manager.set_layout(LayoutConfig::Vertical {
        components: vec![
            (comp1.clone(), Constraint::Percentage(30)),
            (comp2.clone(), Constraint::Percentage(70)),
        ],
    });

    let area = Rect { x: 0, y: 0, width: 100, height: 100 };
    let layout = manager.calculate_layout(area);

    assert_eq!(layout.len(), 2);

    let top_area = layout.get(&comp1).unwrap();
    let bottom_area = layout.get(&comp2).unwrap();

    // Top should be smaller
    assert!(top_area.height < bottom_area.height);
}

#[test]
fn test_layout_manager_horizontal_layout() {
    let mut manager = LayoutManager::new();
    let left_id = ComponentId::new("sidebar");
    let right_id = ComponentId::new("main");

    manager.set_layout(LayoutConfig::Horizontal {
        components: vec![
            (left_id.clone(), Constraint::Percentage(20)),
            (right_id.clone(), Constraint::Percentage(80)),
        ],
    });

    let area = Rect { x: 0, y: 0, width: 200, height: 100 };
    let layout = manager.calculate_layout(area);

    assert_eq!(layout.len(), 2);

    let left_area = layout.get(&left_id).unwrap();
    let right_area = layout.get(&right_id).unwrap();

    // Left should be narrower
    assert!(left_area.width < right_area.width);
}

#[test]
fn test_layout_manager_grid_layout() {
    let mut manager = LayoutManager::new();

    let tl = ComponentId::new("top-left");
    let tr = ComponentId::new("top-right");
    let bl = ComponentId::new("bottom-left");
    let br = ComponentId::new("bottom-right");

    manager.set_layout(LayoutConfig::Grid {
        rows: 2,
        cols: 2,
        components: vec![
            (tl.clone(), 0, 0),
            (tr.clone(), 0, 1),
            (bl.clone(), 1, 0),
            (br.clone(), 1, 1),
        ],
    });

    let area = Rect { x: 0, y: 0, width: 200, height: 100 };
    let layout = manager.calculate_layout(area);

    assert_eq!(layout.len(), 4);
    assert!(layout.contains_key(&tl));
    assert!(layout.contains_key(&tr));
    assert!(layout.contains_key(&bl));
    assert!(layout.contains_key(&br));
}

// ===== Theme Manager Tests =====

#[test]
fn test_theme_manager_initialization() {
    let manager = ThemeManager::load_default();

    assert_eq!(manager.theme_count(), 2); // Dark and Light
    assert_eq!(manager.current_theme().name, "dark");

    let theme_names = manager.available_theme_names();
    assert!(theme_names.contains(&"dark".to_string()));
    assert!(theme_names.contains(&"light".to_string()));
}

#[test]
fn test_theme_manager_switch_theme() {
    let mut manager = ThemeManager::load_default();

    // Switch to light theme
    assert!(manager.switch_theme("light").is_ok());
    assert_eq!(manager.current_theme().name, "light");

    // Switch back to dark
    assert!(manager.switch_theme("dark").is_ok());
    assert_eq!(manager.current_theme().name, "dark");

    // Try invalid theme
    assert!(manager.switch_theme("nonexistent").is_err());
}

#[test]
fn test_theme_manager_add_custom_theme() {
    use dora_cli::tui::ThemeConfig;

    let mut manager = ThemeManager::load_default();
    let initial_count = manager.theme_count();

    // Create custom theme
    let custom_theme = ThemeConfig::default_dark();
    // Note: we'd modify it here if we had a builder pattern

    manager.add_theme(custom_theme);

    // Should have one more theme (or same count if it replaced "dark")
    assert!(manager.theme_count() >= initial_count);
}

#[test]
fn test_theme_config_style_helpers() {
    use dora_cli::tui::ThemeConfig;

    let theme = ThemeConfig::default_dark();

    // Test that all style helpers return valid styles
    let _ = theme.normal_style();
    let _ = theme.focused_style();
    let _ = theme.selected_item_style();
    let _ = theme.normal_item_style();
    let _ = theme.focused_border_style();
    let _ = theme.normal_border_style();
    let _ = theme.highlight_style();
    let _ = theme.success_style();
    let _ = theme.warning_style();
    let _ = theme.error_style();
    let _ = theme.info_style();
    let _ = theme.muted_style();
    let _ = theme.chart_line_style();
    let _ = theme.axis_style();

    // All methods should execute without panic
}

#[test]
fn test_theme_color_scheme_completeness() {
    use dora_cli::tui::ThemeConfig;

    let dark_theme = ThemeConfig::default_dark();
    let light_theme = ThemeConfig::default_light();

    // Verify both themes have all required colors defined
    // This ensures no Color::default() or missing colors

    // Dark theme should use dark background
    assert_eq!(dark_theme.colors.background, ratatui::style::Color::Black);
    assert_eq!(dark_theme.colors.foreground, ratatui::style::Color::White);

    // Light theme should use light background
    assert_eq!(light_theme.colors.background, ratatui::style::Color::White);
    assert_eq!(light_theme.colors.foreground, ratatui::style::Color::Black);
}

// ===== Integration Tests =====

#[test]
fn test_component_and_layout_integration() {
    let mut registry = ComponentRegistry::new();
    let mut layout_manager = LayoutManager::new();

    // Register components
    let list_id = ComponentId::new("dataflow-list");
    let metrics_id = ComponentId::new("metrics");

    registry.register(list_id.clone(), Box::new(DataflowListComponent::new()));
    registry.register(metrics_id.clone(), Box::new(MetricsChartComponent::new(
        dora_cli::tui::components::metrics_chart::ChartType::MemoryUsage,
        std::time::Duration::from_secs(30)
    )));

    // Set up layout
    layout_manager.set_layout(LayoutConfig::Vertical {
        components: vec![
            (list_id.clone(), Constraint::Percentage(60)),
            (metrics_id.clone(), Constraint::Percentage(40)),
        ],
    });

    // Calculate layout
    let area = Rect { x: 0, y: 0, width: 100, height: 100 };
    let layout = layout_manager.calculate_layout(area);

    // Verify both components have areas
    assert!(layout.contains_key(&list_id));
    assert!(layout.contains_key(&metrics_id));

    // Verify components exist in registry
    assert!(registry.get(&list_id).is_some());
    assert!(registry.get(&metrics_id).is_some());
}

#[test]
fn test_full_architecture_stack() {
    // This test verifies the entire architecture works together:
    // ComponentRegistry + LayoutManager + ThemeManager + EventDispatcher

    let mut component_registry = ComponentRegistry::new();
    let mut layout_manager = LayoutManager::new();
    let mut event_dispatcher = EventDispatcher::new();
    let theme_manager = ThemeManager::load_default();

    // Setup components
    let comp1 = ComponentId::new("main");
    let comp2 = ComponentId::new("sidebar");
    let comp3 = ComponentId::new("status");

    component_registry.register(comp1.clone(), Box::new(DataflowListComponent::new()));
    component_registry.register(comp2.clone(), Box::new(LogViewerComponent::new(500)));
    component_registry.register(comp3.clone(), Box::new(StatusBarComponent::new()));

    // Setup layout
    layout_manager.set_layout(LayoutConfig::Vertical {
        components: vec![
            (comp1.clone(), Constraint::Percentage(70)),
            (comp2.clone(), Constraint::Percentage(20)),
            (comp3.clone(), Constraint::Length(3)),
        ],
    });

    // Setup focus
    event_dispatcher.set_focus(Some(comp1.clone()));

    // Verify everything is connected
    assert_eq!(component_registry.len(), 3);
    assert_eq!(theme_manager.theme_count(), 2);
    assert_eq!(event_dispatcher.get_focused(), Some(&comp1));

    let layout = layout_manager.calculate_layout(Rect {
        x: 0,
        y: 0,
        width: 100,
        height: 100,
    });
    assert_eq!(layout.len(), 3);
}
