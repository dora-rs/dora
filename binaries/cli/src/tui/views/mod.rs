use crossterm::event::KeyEvent;
use ratatui::{Frame, layout::Rect};
use std::time::Duration;

use crate::tui::{
    Result,
    app::{AppState, ViewType},
};

pub mod analysis_tools;
pub mod analysis_tools_types;
pub mod collab_features;
pub mod collab_features_types;
pub mod context_awareness;
pub mod context_awareness_types;
pub mod dashboard;
pub mod dashboard_types;
pub mod data_viz;
pub mod data_viz_types;
pub mod dataflow_explorer;
pub mod dataflow_explorer_types;
pub mod dataflow_manager;
pub mod debug_session;
pub mod debug_session_types;
pub mod help;
pub mod help_browser;
pub mod help_browser_types;
pub mod layout;
pub mod log_viewer;
pub mod log_viewer_types;
pub mod node_inspector;
pub mod node_inspector_types;
pub mod perf_optimization;
pub mod perf_optimization_types;
pub mod performance_analyzer;
pub mod performance_analyzer_types;
pub mod settings;
pub mod settings_types;
pub mod system_monitor;

pub use analysis_tools::AnalysisToolsView;
pub use analysis_tools_types::*;
pub use collab_features::CollabFeaturesView;
pub use collab_features_types::*;
pub use context_awareness::ContextAwarenessView;
pub use context_awareness_types::*;
pub use dashboard::DashboardView;
pub use dashboard_types::*;
pub use data_viz::DataVizView;
pub use data_viz_types::*;
pub use dataflow_explorer::DataflowExplorerView;
pub use dataflow_explorer_types::*;
pub use dataflow_manager::DataflowManagerView;
pub use debug_session::DebugSessionView;
pub use debug_session_types::*;
pub use help::HelpView;
pub use help_browser::HelpBrowserView;
pub use help_browser_types::*;
pub use layout::{LayoutConfig, LayoutManager};
pub use log_viewer::LogViewerView;
pub use log_viewer_types::*;
pub use node_inspector::NodeInspectorView;
pub use node_inspector_types::*;
pub use perf_optimization::PerfOptimizationView;
pub use perf_optimization_types::TrendDirection as PerfTrendDirection;
pub use perf_optimization_types::{
    Bottleneck, BottleneckLocation, BottleneckType, EffortLevel, ImpactLevel, MetricStatus,
    OptimizationCategory, OptimizationSuggestion, PerfData, PerfMetricType, PerfOptimizationState,
    PerfSection, PerformanceMetric, Priority, ResourceTrend, ResourceType, ResourceUsage, Severity,
    create_mock_bottlenecks, create_mock_metrics, create_mock_resources, create_mock_suggestions,
};
pub use performance_analyzer::PerformanceAnalyzerView;
pub use performance_analyzer_types::*;
pub use settings::SettingsView;
pub use settings_types::*;
pub use system_monitor::SystemMonitorView;

/// Core trait that all TUI views must implement
pub trait View {
    /// Render the view content
    fn render(&mut self, f: &mut Frame, area: Rect, app_state: &AppState);

    /// Handle key events
    fn handle_key(
        &mut self,
        key: KeyEvent,
        _app_state: &mut AppState,
    ) -> impl std::future::Future<Output = Result<ViewAction>> + Send;

    /// Update view state (called periodically)
    fn update(
        &mut self,
        _app_state: &mut AppState,
    ) -> impl std::future::Future<Output = Result<()>> + Send;

    /// View-specific help text
    fn help_text(&self) -> Vec<(&str, &str)>;

    /// Whether this view should auto-refresh
    fn auto_refresh(&self) -> Option<Duration> {
        None
    }

    /// View title for display
    fn title(&self) -> &str;

    /// Whether this view can handle focus
    fn can_focus(&self) -> bool {
        true
    }

    /// Called when view gains focus
    fn on_focus(&mut self) {}

    /// Called when view loses focus
    fn on_blur(&mut self) {}

    /// Called when view is first shown
    fn on_mount(
        &mut self,
        _app_state: &mut AppState,
    ) -> impl std::future::Future<Output = Result<()>> + Send {
        async { Ok(()) }
    }

    /// Called when view is hidden/destroyed
    fn on_unmount(&mut self) {}
}

/// Actions that views can return to the main application
#[derive(Debug, Clone)]
pub enum ViewAction {
    /// No action needed
    None,

    /// Switch to a different view
    SwitchView(ViewType),

    /// Push a new view onto the stack
    PushView(ViewType),

    /// Pop the current view from the stack
    PopView,

    /// Quit the application
    Quit,

    /// Execute a CLI command
    ExecuteCommand(String),

    /// Show help
    ShowHelp,

    /// Show an error message
    ShowError(String),

    /// Show a status message
    ShowStatus(String),

    /// Refresh the current view
    Refresh,

    /// Request focus on a specific widget
    FocusWidget(String),

    /// Show a confirmation dialog
    ShowConfirmation {
        message: String,
        confirm_action: String,
    },

    /// Show an input dialog
    ShowInput {
        prompt: String,
        default: Option<String>,
    },

    /// Update application state
    UpdateState(StateUpdate),
}

/// State updates that views can request
#[derive(Debug, Clone)]
pub enum StateUpdate {
    /// Refresh dataflow list
    RefreshDataflows,

    /// Update system metrics
    UpdateSystemMetrics,

    /// Clear error state
    ClearError,

    /// Set user preference
    SetUserPreference(String, String),

    /// Add status message
    AddStatusMessage(String, crate::tui::app::MessageLevel),
}

/// Base view implementation with common functionality
pub struct BaseView {
    pub title: String,
    pub focused: bool,
    pub last_update: Option<std::time::Instant>,
    pub auto_refresh_interval: Option<Duration>,
    pub component_registry: crate::tui::ComponentRegistry,
    pub event_dispatcher: crate::tui::EventDispatcher,
    pub layout_manager: LayoutManager,
}

impl BaseView {
    pub fn new(title: String) -> Self {
        Self {
            title,
            focused: false,
            last_update: None,
            auto_refresh_interval: None,
            component_registry: crate::tui::ComponentRegistry::new(),
            event_dispatcher: crate::tui::EventDispatcher::new(),
            layout_manager: LayoutManager::new(),
        }
    }

    pub fn with_auto_refresh(mut self, interval: Duration) -> Self {
        self.auto_refresh_interval = Some(interval);
        self
    }

    pub fn set_focused(&mut self, focused: bool) {
        self.focused = focused;
    }

    pub fn needs_refresh(&self) -> bool {
        if let Some(interval) = self.auto_refresh_interval {
            if let Some(last_update) = self.last_update {
                std::time::Instant::now().duration_since(last_update) >= interval
            } else {
                true
            }
        } else {
            false
        }
    }

    pub fn mark_updated(&mut self) {
        self.last_update = Some(std::time::Instant::now());
    }

    /// Register a component with this view
    pub fn add_component(
        &mut self,
        id: crate::tui::ComponentId,
        component: Box<dyn crate::tui::Component>,
    ) {
        self.component_registry.register(id, component);
    }

    /// Set which component has focus
    pub fn set_component_focus(&mut self, component_id: Option<crate::tui::ComponentId>) {
        self.event_dispatcher.set_focus(component_id);
    }

    /// Get currently focused component ID
    pub fn focused_component(&self) -> Option<&crate::tui::ComponentId> {
        self.event_dispatcher.get_focused()
    }

    /// Set the layout configuration
    pub fn set_layout(&mut self, config: LayoutConfig) {
        self.layout_manager.set_layout(config);
    }

    /// Render all components according to the layout
    pub fn render_components(
        &self,
        frame: &mut Frame,
        area: Rect,
        theme: &crate::tui::ThemeConfig,
        app_state: &AppState,
    ) {
        let layout_areas = self.layout_manager.calculate_layout(area);

        for (component_id, component_area) in layout_areas {
            if let Some(component) = self.component_registry.get(&component_id) {
                component.render(frame, component_area, theme, app_state);
            }
        }
    }

    /// Update all components
    pub async fn update_components(&mut self, _app_state: &AppState) -> Result<()> {
        for component_id in self.component_registry.component_ids() {
            // Note: We can't get mutable reference to components from registry
            // This will be addressed when we implement proper component updates
            // For now, components will update themselves when needed
            let _ = component_id;
        }
        Ok(())
    }
}

/// Utility functions for view implementations
pub mod utils {
    use ratatui::{
        layout::{Constraint, Direction, Layout, Rect},
        style::{Color, Style},
        text::{Line, Span},
        widgets::{Block, Borders, Cell, Gauge, List, ListItem, Row, Table},
    };

    /// Create a centered popup area
    pub fn centered_rect(percent_x: u16, percent_y: u16, r: Rect) -> Rect {
        let popup_layout = Layout::default()
            .direction(Direction::Vertical)
            .constraints([
                Constraint::Percentage((100 - percent_y) / 2),
                Constraint::Percentage(percent_y),
                Constraint::Percentage((100 - percent_y) / 2),
            ])
            .split(r);

        Layout::default()
            .direction(Direction::Horizontal)
            .constraints([
                Constraint::Percentage((100 - percent_x) / 2),
                Constraint::Percentage(percent_x),
                Constraint::Percentage((100 - percent_x) / 2),
            ])
            .split(popup_layout[1])[1]
    }

    /// Create a progress bar widget
    pub fn progress_bar(percent: u16, label: &str) -> Gauge {
        Gauge::default()
            .block(Block::default().title(label).borders(Borders::ALL))
            .gauge_style(Style::default().fg(Color::Cyan))
            .percent(percent)
    }

    /// Create a status indicator
    pub fn status_indicator(status: &str) -> Span {
        let (symbol, color) = match status.to_lowercase().as_str() {
            "running" | "active" | "healthy" => ("●", Color::Green),
            "stopped" | "inactive" | "down" => ("●", Color::Red),
            "warning" | "degraded" => ("●", Color::Yellow),
            "pending" | "starting" => ("◐", Color::Yellow),
            _ => ("○", Color::Gray),
        };

        Span::styled(symbol, Style::default().fg(color))
    }

    /// Format bytes for display
    pub fn format_bytes(bytes: u64) -> String {
        const UNITS: &[&str] = &["B", "KB", "MB", "GB", "TB"];
        let mut size = bytes as f64;
        let mut unit_index = 0;

        while size >= 1024.0 && unit_index < UNITS.len() - 1 {
            size /= 1024.0;
            unit_index += 1;
        }

        if unit_index == 0 {
            format!("{} {}", bytes, UNITS[unit_index])
        } else {
            format!("{:.1} {}", size, UNITS[unit_index])
        }
    }

    /// Format duration for display
    pub fn format_duration(duration: std::time::Duration) -> String {
        let total_seconds = duration.as_secs();

        if total_seconds < 60 {
            format!("{total_seconds}s")
        } else if total_seconds < 3600 {
            format!("{}m {}s", total_seconds / 60, total_seconds % 60)
        } else {
            format!("{}h {}m", total_seconds / 3600, (total_seconds % 3600) / 60)
        }
    }

    /// Create a simple list widget
    pub fn simple_list<'a>(items: Vec<String>, title: &'a str) -> List<'a> {
        let list_items: Vec<ListItem> = items
            .into_iter()
            .map(|item| ListItem::new(Line::from(item)))
            .collect();

        List::new(list_items).block(Block::default().title(title).borders(Borders::ALL))
    }

    /// Create a table with headers
    pub fn simple_table<'a>(
        headers: Vec<&'a str>,
        rows: Vec<Vec<String>>,
        title: &'a str,
    ) -> Table<'a> {
        let header_cells = headers
            .iter()
            .map(|h| Cell::from(*h).style(Style::default().fg(Color::Yellow)))
            .collect::<Vec<_>>();

        let header = Row::new(header_cells).style(Style::default().fg(Color::Yellow));

        let data_rows = rows
            .into_iter()
            .map(|row| {
                let cells = row.into_iter().map(Cell::from).collect::<Vec<_>>();
                Row::new(cells)
            })
            .collect::<Vec<_>>();

        Table::new(data_rows, [Constraint::Percentage(25); 4])
            .header(header)
            .block(Block::default().title(title).borders(Borders::ALL))
    }

    /// Split an area into equal columns
    pub fn split_columns(area: Rect, count: usize) -> Vec<Rect> {
        let constraints: Vec<Constraint> = (0..count)
            .map(|_| Constraint::Percentage(100 / count as u16))
            .collect();

        Layout::default()
            .direction(Direction::Horizontal)
            .constraints(constraints)
            .split(area)
            .to_vec()
    }

    /// Split an area into equal rows
    pub fn split_rows(area: Rect, count: usize) -> Vec<Rect> {
        let constraints: Vec<Constraint> = (0..count)
            .map(|_| Constraint::Percentage(100 / count as u16))
            .collect();

        Layout::default()
            .direction(Direction::Vertical)
            .constraints(constraints)
            .split(area)
            .to_vec()
    }
}
