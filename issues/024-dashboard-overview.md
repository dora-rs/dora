# Issue #024: Build Interactive Dashboard Overview

## 📋 Summary
Implement a comprehensive interactive dashboard that provides a real-time overview of the entire Dora system, including dataflows, system resources, recent activity, and key metrics. This dashboard serves as the central hub for system monitoring and navigation to detailed views.

## 🎯 Objectives
- Create a comprehensive system overview with real-time updates
- Implement interactive navigation to detailed views for each system component
- Add customizable dashboard layout with user-defined widgets
- Provide at-a-glance health status and performance indicators
- Enable quick actions and shortcuts for common tasks

**Success Metrics:**
- Dashboard refresh rate maintains 2-second intervals without performance impact
- Widget rendering completes within 50ms for typical configurations
- User satisfaction with dashboard information density exceeds 85%
- Navigation to detailed views completes within 200ms
- Dashboard customization adoption rate above 60% among regular users

## 🛠️ Technical Requirements

### What to Build

#### 1. Dashboard View Implementation
```rust
// src/tui/views/dashboard.rs
use super::BaseView;
use crate::tui::{
    components::*,
    theme::Theme,
    AppContext, View, ViewEvent, ViewEventResult, ViewType
};
use async_trait::async_trait;
use ratatui::{
    layout::{Constraint, Direction, Layout, Rect},
    style::{Color, Style},
    widgets::{Block, Borders, Gauge, Paragraph},
    Frame,
};
use std::collections::HashMap;
use tokio::time::{Duration, Instant};

pub struct DashboardView {
    base: BaseView,
    dashboard_state: DashboardState,
    widget_manager: DashboardWidgetManager,
    refresh_manager: RefreshManager,
    navigation_shortcuts: NavigationShortcuts,
    last_refresh: Instant,
}

#[derive(Debug, Clone)]
pub struct DashboardState {
    pub system_overview: SystemOverview,
    pub dataflow_summary: DataflowSummary,
    pub performance_metrics: PerformanceMetrics,
    pub recent_activity: RecentActivity,
    pub alerts: Vec<Alert>,
    pub quick_stats: QuickStats,
}

#[derive(Debug, Clone)]
pub struct SystemOverview {
    pub status: SystemStatus,
    pub uptime: Duration,
    pub version: String,
    pub cpu_usage: f64,
    pub memory_usage: MemoryUsage,
    pub disk_usage: DiskUsage,
    pub network_activity: NetworkActivity,
    pub active_connections: u32,
}

#[derive(Debug, Clone)]
pub struct DataflowSummary {
    pub total_dataflows: u32,
    pub running_dataflows: u32,
    pub failed_dataflows: u32,
    pub total_nodes: u32,
    pub healthy_nodes: u32,
    pub recent_deployments: Vec<RecentDeployment>,
}

impl DashboardView {
    pub fn new() -> Self {
        let mut base = BaseView::new();
        
        // Configure dashboard layout
        let layout = LayoutConfig::grid(vec![
            LayoutRow {
                height: Constraint::Length(8),
                columns: vec![
                    LayoutColumn {
                        width: Constraint::Percentage(50),
                        component_id: ComponentId("system_overview".to_string()),
                    },
                    LayoutColumn {
                        width: Constraint::Percentage(50),
                        component_id: ComponentId("dataflow_summary".to_string()),
                    },
                ],
            },
            LayoutRow {
                height: Constraint::Min(12),
                columns: vec![
                    LayoutColumn {
                        width: Constraint::Percentage(60),
                        component_id: ComponentId("performance_charts".to_string()),
                    },
                    LayoutColumn {
                        width: Constraint::Percentage(40),
                        component_id: ComponentId("recent_activity".to_string()),
                    },
                ],
            },
            LayoutRow {
                height: Constraint::Length(3),
                columns: vec![
                    LayoutColumn {
                        width: Constraint::Percentage(100),
                        component_id: ComponentId("status_bar".to_string()),
                    },
                ],
            },
        ]);
        
        base.set_layout(layout);
        
        // Add dashboard components
        base.add_component(
            ComponentId("system_overview".to_string()),
            SystemOverviewComponent::new(),
        );
        
        base.add_component(
            ComponentId("dataflow_summary".to_string()),
            DataflowSummaryComponent::new(),
        );
        
        base.add_component(
            ComponentId("performance_charts".to_string()),
            PerformanceChartsComponent::new(),
        );
        
        base.add_component(
            ComponentId("recent_activity".to_string()),
            RecentActivityComponent::new(),
        );
        
        base.add_component(
            ComponentId("status_bar".to_string()),
            StatusBarComponent::new(),
        );
        
        Self {
            base,
            dashboard_state: DashboardState::default(),
            widget_manager: DashboardWidgetManager::new(),
            refresh_manager: RefreshManager::new(Duration::from_secs(2)),
            navigation_shortcuts: NavigationShortcuts::new(),
            last_refresh: Instant::now(),
        }
    }
    
    async fn refresh_dashboard_data(&mut self) -> io::Result<()> {
        // Refresh system overview
        self.dashboard_state.system_overview = self.collect_system_overview().await?;
        
        // Refresh dataflow summary
        self.dashboard_state.dataflow_summary = self.collect_dataflow_summary().await?;
        
        // Refresh performance metrics
        self.dashboard_state.performance_metrics = self.collect_performance_metrics().await?;
        
        // Refresh recent activity
        self.dashboard_state.recent_activity = self.collect_recent_activity().await?;
        
        // Check for alerts
        self.dashboard_state.alerts = self.collect_alerts().await?;
        
        // Update quick stats
        self.dashboard_state.quick_stats = self.calculate_quick_stats();
        
        self.last_refresh = Instant::now();
        Ok(())
    }
    
    async fn collect_system_overview(&self) -> io::Result<SystemOverview> {
        let system_info = System::new_all();
        
        // Get CPU usage
        let cpu_usage = system_info.global_cpu_info().cpu_usage() as f64;
        
        // Get memory usage
        let total_memory = system_info.total_memory();
        let used_memory = system_info.used_memory();
        let memory_usage = MemoryUsage {
            total_mb: total_memory / 1024 / 1024,
            used_mb: used_memory / 1024 / 1024,
            usage_percent: (used_memory as f64 / total_memory as f64) * 100.0,
        };
        
        // Get disk usage for main disk
        let disk_usage = system_info.disks().iter()
            .next()
            .map(|disk| {
                let total = disk.total_space();
                let available = disk.available_space();
                let used = total - available;
                
                DiskUsage {
                    total_gb: total / 1024 / 1024 / 1024,
                    used_gb: used / 1024 / 1024 / 1024,
                    usage_percent: (used as f64 / total as f64) * 100.0,
                }
            })
            .unwrap_or_default();
        
        // Get network activity
        let network_activity = NetworkActivity {
            bytes_received: system_info.networks().iter()
                .map(|(_, network)| network.received())
                .sum(),
            bytes_transmitted: system_info.networks().iter()
                .map(|(_, network)| network.transmitted())
                .sum(),
        };
        
        // Get system status
        let status = if let Ok(daemon_client) = DaemonClient::connect().await {
            SystemStatus::Connected
        } else {
            SystemStatus::Disconnected
        };
        
        Ok(SystemOverview {
            status,
            uptime: self.get_system_uptime(),
            version: env!("CARGO_PKG_VERSION").to_string(),
            cpu_usage,
            memory_usage,
            disk_usage,
            network_activity,
            active_connections: self.get_active_connections().await.unwrap_or(0),
        })
    }
    
    async fn collect_dataflow_summary(&self) -> io::Result<DataflowSummary> {
        if let Ok(daemon_client) = DaemonClient::connect().await {
            let dataflows = daemon_client.list_dataflows().await.unwrap_or_default();
            
            let total_dataflows = dataflows.len() as u32;
            let running_dataflows = dataflows.iter()
                .filter(|df| df.status == DataflowStatus::Running)
                .count() as u32;
            let failed_dataflows = dataflows.iter()
                .filter(|df| df.status == DataflowStatus::Failed)
                .count() as u32;
            
            let mut total_nodes = 0;
            let mut healthy_nodes = 0;
            
            for dataflow in &dataflows {
                if let Ok(nodes) = daemon_client.get_dataflow_nodes(&dataflow.name).await {
                    total_nodes += nodes.len() as u32;
                    healthy_nodes += nodes.iter()
                        .filter(|node| node.is_healthy())
                        .count() as u32;
                }
            }
            
            Ok(DataflowSummary {
                total_dataflows,
                running_dataflows,
                failed_dataflows,
                total_nodes,
                healthy_nodes,
                recent_deployments: self.get_recent_deployments().await.unwrap_or_default(),
            })
        } else {
            Ok(DataflowSummary::default())
        }
    }
}

#[async_trait]
impl View for DashboardView {
    async fn initialize(&mut self, context: &AppContext) -> io::Result<()> {
        // Initialize dashboard state
        self.refresh_dashboard_data().await?;
        
        // Initialize all components
        for component in self.base.component_registry.components_mut() {
            component.update(context).await?;
        }
        
        // Start refresh timer
        self.refresh_manager.start();
        
        Ok(())
    }
    
    async fn update(&mut self, context: &mut AppContext) -> io::Result<()> {
        // Check if it's time to refresh
        if self.refresh_manager.should_refresh() {
            self.refresh_dashboard_data().await?;
        }
        
        // Update all components with latest data
        for component in self.base.component_registry.components_mut() {
            component.update(context).await?;
        }
        
        Ok(())
    }
    
    fn render(&self, frame: &mut Frame, area: Rect, theme: &Theme, context: &AppContext) {
        // Render main dashboard layout
        self.base.render_components(frame, area, theme, context);
        
        // Render dashboard-specific overlays
        self.render_alerts_overlay(frame, area, theme);
        self.render_shortcuts_help(frame, area, theme);
    }
    
    async fn handle_event(&mut self, event: ViewEvent, context: &mut AppContext) -> io::Result<ViewEventResult> {
        // Handle dashboard-specific key bindings
        if let ViewEvent::Key(key_event) = &event {
            match key_event.code {
                crossterm::event::KeyCode::Char('r') => {
                    // Manual refresh
                    self.refresh_dashboard_data().await?;
                    return Ok(ViewEventResult::None);
                },
                
                crossterm::event::KeyCode::Char('d') => {
                    // Navigate to dataflows list
                    return Ok(ViewEventResult::PushView(ViewType::DataflowList));
                },
                
                crossterm::event::KeyCode::Char('p') => {
                    // Navigate to performance analyzer
                    return Ok(ViewEventResult::PushView(ViewType::PerformanceAnalyzer {
                        analysis_type: AnalysisType::Comprehensive
                    }));
                },
                
                crossterm::event::KeyCode::Char('l') => {
                    // Navigate to log viewer
                    return Ok(ViewEventResult::PushView(ViewType::LogViewer {
                        target: LogTarget::System,
                        filters: LogFilters::default(),
                    }));
                },
                
                crossterm::event::KeyCode::Char('s') => {
                    // Navigate to settings
                    return Ok(ViewEventResult::PushView(ViewType::Settings));
                },
                
                crossterm::event::KeyCode::Char('h') | crossterm::event::KeyCode::F1 => {
                    // Show help
                    return Ok(ViewEventResult::PushView(ViewType::HelpBrowser { topic: None }));
                },
                
                crossterm::event::KeyCode::F5 => {
                    // Force refresh
                    self.refresh_dashboard_data().await?;
                    return Ok(ViewEventResult::None);
                },
                
                _ => {}
            }
        }
        
        // Forward event to components
        self.base.handle_component_event(event, context).await
    }
    
    async fn handle_resize(&mut self, width: u16, height: u16) -> io::Result<()> {
        // Recalculate layout for new dimensions
        self.base.layout_manager.handle_resize(width, height);
        Ok(())
    }
    
    async fn cleanup(&self, context: &AppContext) -> io::Result<()> {
        self.refresh_manager.stop();
        Ok(())
    }
    
    fn view_type(&self) -> ViewType {
        ViewType::Dashboard
    }
    
    fn title(&self) -> String {
        "Dora Dashboard".to_string()
    }
    
    fn help_text(&self) -> Vec<HelpItem> {
        vec![
            HelpItem::new("r", "Refresh dashboard"),
            HelpItem::new("d", "View dataflows"),
            HelpItem::new("p", "Performance analyzer"),
            HelpItem::new("l", "View logs"),
            HelpItem::new("s", "Settings"),
            HelpItem::new("h/F1", "Help"),
            HelpItem::new("F5", "Force refresh"),
            HelpItem::new("Esc", "Exit"),
        ]
    }
}
```

#### 2. Dashboard Components
```rust
// src/tui/components/dashboard/system_overview.rs
pub struct SystemOverviewComponent {
    system_data: Option<SystemOverview>,
    focused: bool,
    last_update: Instant,
}

impl SystemOverviewComponent {
    pub fn new() -> Self {
        Self {
            system_data: None,
            focused: false,
            last_update: Instant::now(),
        }
    }
    
    fn render_system_status(&self, frame: &mut Frame, area: Rect, theme: &Theme) {
        if let Some(system) = &self.system_data {
            let status_color = match system.status {
                SystemStatus::Connected => theme.colors.success,
                SystemStatus::Disconnected => theme.colors.error,
                SystemStatus::Connecting => theme.colors.warning,
            };
            
            let status_text = format!("● {}", system.status);
            let status_paragraph = Paragraph::new(status_text)
                .style(Style::default().fg(status_color));
            
            frame.render_widget(status_paragraph, area);
        }
    }
    
    fn render_resource_gauges(&self, frame: &mut Frame, area: Rect, theme: &Theme) {
        if let Some(system) = &self.system_data {
            let chunks = Layout::default()
                .direction(Direction::Vertical)
                .constraints([
                    Constraint::Length(2),
                    Constraint::Length(2),
                    Constraint::Length(2),
                ].as_ref())
                .split(area);
            
            // CPU gauge
            let cpu_gauge = Gauge::default()
                .block(Block::default().title("CPU"))
                .gauge_style(if system.cpu_usage > 80.0 {
                    theme.error_style()
                } else if system.cpu_usage > 60.0 {
                    theme.warning_style()
                } else {
                    theme.success_style()
                })
                .percent(system.cpu_usage as u16)
                .label(format!("{:.1}%", system.cpu_usage));
            
            frame.render_widget(cpu_gauge, chunks[0]);
            
            // Memory gauge
            let memory_gauge = Gauge::default()
                .block(Block::default().title("Memory"))
                .gauge_style(if system.memory_usage.usage_percent > 80.0 {
                    theme.error_style()
                } else if system.memory_usage.usage_percent > 60.0 {
                    theme.warning_style()
                } else {
                    theme.success_style()
                })
                .percent(system.memory_usage.usage_percent as u16)
                .label(format!("{:.1}%", system.memory_usage.usage_percent));
            
            frame.render_widget(memory_gauge, chunks[1]);
            
            // Disk gauge
            let disk_gauge = Gauge::default()
                .block(Block::default().title("Disk"))
                .gauge_style(if system.disk_usage.usage_percent > 90.0 {
                    theme.error_style()
                } else if system.disk_usage.usage_percent > 75.0 {
                    theme.warning_style()
                } else {
                    theme.success_style()
                })
                .percent(system.disk_usage.usage_percent as u16)
                .label(format!("{:.1}%", system.disk_usage.usage_percent));
            
            frame.render_widget(disk_gauge, chunks[2]);
        }
    }
}

#[async_trait]
impl Component for SystemOverviewComponent {
    async fn update(&mut self, context: &AppContext) -> io::Result<()> {
        // Data is updated from the dashboard view
        Ok(())
    }
    
    fn render(&self, frame: &mut Frame, area: Rect, theme: &Theme, context: &AppContext) {
        let block = Block::default()
            .title("System Overview")
            .borders(Borders::ALL)
            .border_style(if self.focused {
                theme.focused_border_style()
            } else {
                theme.normal_border_style()
            });
        
        let inner_area = block.inner(area);
        frame.render_widget(block, area);
        
        // Split area for status and gauges
        let chunks = Layout::default()
            .direction(Direction::Vertical)
            .constraints([
                Constraint::Length(3),
                Constraint::Min(6),
            ].as_ref())
            .split(inner_area);
        
        self.render_system_status(frame, chunks[0], theme);
        self.render_resource_gauges(frame, chunks[1], theme);
    }
    
    async fn handle_event(&mut self, event: ComponentEvent, context: &AppContext) -> io::Result<ViewEventResult> {
        Ok(ViewEventResult::None)
    }
    
    fn component_type(&self) -> ComponentType {
        ComponentType::SystemOverview
    }
    
    fn is_focusable(&self) -> bool {
        false
    }
    
    fn is_focused(&self) -> bool {
        self.focused
    }
    
    fn set_focus(&mut self, focused: bool) {
        self.focused = focused;
    }
}

// src/tui/components/dashboard/dataflow_summary.rs
pub struct DataflowSummaryComponent {
    dataflow_data: Option<DataflowSummary>,
    focused: bool,
}

impl DataflowSummaryComponent {
    pub fn new() -> Self {
        Self {
            dataflow_data: None,
            focused: false,
        }
    }
    
    fn render_dataflow_stats(&self, frame: &mut Frame, area: Rect, theme: &Theme) {
        if let Some(data) = &self.dataflow_data {
            let stats_text = vec![
                format!("Total Dataflows: {}", data.total_dataflows),
                format!("Running: {}", data.running_dataflows),
                format!("Failed: {}", data.failed_dataflows),
                format!(""),
                format!("Total Nodes: {}", data.total_nodes),
                format!("Healthy: {}", data.healthy_nodes),
            ];
            
            let paragraph = Paragraph::new(stats_text.join("\n"))
                .style(theme.normal_style());
            
            frame.render_widget(paragraph, area);
        }
    }
    
    fn render_health_indicator(&self, frame: &mut Frame, area: Rect, theme: &Theme) {
        if let Some(data) = &self.dataflow_data {
            let health_percentage = if data.total_dataflows > 0 {
                (data.running_dataflows as f64 / data.total_dataflows as f64) * 100.0
            } else {
                100.0
            };
            
            let health_color = if health_percentage >= 90.0 {
                theme.colors.success
            } else if health_percentage >= 70.0 {
                theme.colors.warning
            } else {
                theme.colors.error
            };
            
            let health_gauge = Gauge::default()
                .block(Block::default().title("System Health"))
                .gauge_style(Style::default().fg(health_color))
                .percent(health_percentage as u16)
                .label(format!("{:.0}%", health_percentage));
            
            frame.render_widget(health_gauge, area);
        }
    }
}

#[async_trait]
impl Component for DataflowSummaryComponent {
    async fn update(&mut self, context: &AppContext) -> io::Result<()> {
        Ok(())
    }
    
    fn render(&self, frame: &mut Frame, area: Rect, theme: &Theme, context: &AppContext) {
        let block = Block::default()
            .title("Dataflow Summary")
            .borders(Borders::ALL)
            .border_style(if self.focused {
                theme.focused_border_style()
            } else {
                theme.normal_border_style()
            });
        
        let inner_area = block.inner(area);
        frame.render_widget(block, area);
        
        let chunks = Layout::default()
            .direction(Direction::Vertical)
            .constraints([
                Constraint::Min(6),
                Constraint::Length(3),
            ].as_ref())
            .split(inner_area);
        
        self.render_dataflow_stats(frame, chunks[0], theme);
        self.render_health_indicator(frame, chunks[1], theme);
    }
    
    async fn handle_event(&mut self, event: ComponentEvent, context: &AppContext) -> io::Result<ViewEventResult> {
        // Double-click or Enter navigates to dataflow list
        if let ComponentEvent::Key(key_event) = event {
            if key_event.code == crossterm::event::KeyCode::Enter {
                return Ok(ViewEventResult::PushView(ViewType::DataflowList));
            }
        }
        
        Ok(ViewEventResult::None)
    }
    
    fn component_type(&self) -> ComponentType {
        ComponentType::DataflowSummary
    }
    
    fn is_focusable(&self) -> bool {
        true
    }
    
    fn is_focused(&self) -> bool {
        self.focused
    }
    
    fn set_focus(&mut self, focused: bool) {
        self.focused = focused;
    }
}
```

#### 3. Dashboard Widget Management
```rust
// src/tui/components/dashboard/widget_manager.rs
#[derive(Debug)]
pub struct DashboardWidgetManager {
    widget_configs: HashMap<String, WidgetConfig>,
    layout_presets: Vec<LayoutPreset>,
    current_layout: String,
    customization_enabled: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WidgetConfig {
    pub widget_type: DashboardWidgetType,
    pub position: WidgetPosition,
    pub size: WidgetSize,
    pub refresh_interval: Duration,
    pub visible: bool,
    pub settings: HashMap<String, serde_json::Value>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum DashboardWidgetType {
    SystemOverview,
    DataflowSummary,
    PerformanceCharts,
    RecentActivity,
    QuickActions,
    AlertsPanel,
    NetworkStatus,
    ResourceUsage,
    LogStream,
}

impl DashboardWidgetManager {
    pub fn new() -> Self {
        Self {
            widget_configs: Self::create_default_widgets(),
            layout_presets: Self::create_default_layouts(),
            current_layout: "default".to_string(),
            customization_enabled: true,
        }
    }
    
    fn create_default_widgets() -> HashMap<String, WidgetConfig> {
        let mut widgets = HashMap::new();
        
        widgets.insert("system_overview".to_string(), WidgetConfig {
            widget_type: DashboardWidgetType::SystemOverview,
            position: WidgetPosition { row: 0, column: 0 },
            size: WidgetSize { width: 50, height: 8 },
            refresh_interval: Duration::from_secs(5),
            visible: true,
            settings: HashMap::new(),
        });
        
        widgets.insert("dataflow_summary".to_string(), WidgetConfig {
            widget_type: DashboardWidgetType::DataflowSummary,
            position: WidgetPosition { row: 0, column: 1 },
            size: WidgetSize { width: 50, height: 8 },
            refresh_interval: Duration::from_secs(3),
            visible: true,
            settings: HashMap::new(),
        });
        
        widgets.insert("performance_charts".to_string(), WidgetConfig {
            widget_type: DashboardWidgetType::PerformanceCharts,
            position: WidgetPosition { row: 1, column: 0 },
            size: WidgetSize { width: 60, height: 12 },
            refresh_interval: Duration::from_secs(2),
            visible: true,
            settings: HashMap::new(),
        });
        
        widgets.insert("recent_activity".to_string(), WidgetConfig {
            widget_type: DashboardWidgetType::RecentActivity,
            position: WidgetPosition { row: 1, column: 1 },
            size: WidgetSize { width: 40, height: 12 },
            refresh_interval: Duration::from_secs(1),
            visible: true,
            settings: HashMap::new(),
        });
        
        widgets
    }
    
    pub fn get_layout_for_size(&self, width: u16, height: u16) -> LayoutConfig {
        // Adaptive layout based on terminal size
        if width < 80 || height < 24 {
            self.get_compact_layout()
        } else if width >= 120 {
            self.get_wide_layout()
        } else {
            self.get_standard_layout()
        }
    }
    
    fn get_compact_layout(&self) -> LayoutConfig {
        // Single column layout for small terminals
        LayoutConfig::grid(vec![
            LayoutRow {
                height: Constraint::Length(6),
                columns: vec![
                    LayoutColumn {
                        width: Constraint::Percentage(100),
                        component_id: ComponentId("system_overview".to_string()),
                    },
                ],
            },
            LayoutRow {
                height: Constraint::Length(6),
                columns: vec![
                    LayoutColumn {
                        width: Constraint::Percentage(100),
                        component_id: ComponentId("dataflow_summary".to_string()),
                    },
                ],
            },
            LayoutRow {
                height: Constraint::Min(8),
                columns: vec![
                    LayoutColumn {
                        width: Constraint::Percentage(100),
                        component_id: ComponentId("recent_activity".to_string()),
                    },
                ],
            },
        ])
    }
    
    fn get_wide_layout(&self) -> LayoutConfig {
        // Three column layout for wide terminals
        LayoutConfig::grid(vec![
            LayoutRow {
                height: Constraint::Length(8),
                columns: vec![
                    LayoutColumn {
                        width: Constraint::Percentage(33),
                        component_id: ComponentId("system_overview".to_string()),
                    },
                    LayoutColumn {
                        width: Constraint::Percentage(33),
                        component_id: ComponentId("dataflow_summary".to_string()),
                    },
                    LayoutColumn {
                        width: Constraint::Percentage(34),
                        component_id: ComponentId("quick_actions".to_string()),
                    },
                ],
            },
            LayoutRow {
                height: Constraint::Min(12),
                columns: vec![
                    LayoutColumn {
                        width: Constraint::Percentage(50),
                        component_id: ComponentId("performance_charts".to_string()),
                    },
                    LayoutColumn {
                        width: Constraint::Percentage(25),
                        component_id: ComponentId("recent_activity".to_string()),
                    },
                    LayoutColumn {
                        width: Constraint::Percentage(25),
                        component_id: ComponentId("alerts_panel".to_string()),
                    },
                ],
            },
        ])
    }
}
```

### Why This Approach

**Comprehensive Overview:**
- Real-time system monitoring with key metrics
- Unified view of all system components
- Quick navigation to detailed views

**Adaptive Interface:**
- Responsive layout for different terminal sizes
- Customizable widget arrangement
- Context-aware information display

**Performance Optimized:**
- Efficient refresh scheduling
- Component-based rendering
- Minimal resource usage

### How to Implement

#### Step 1: Dashboard View Core (4 hours)
1. **Implement DashboardView** with comprehensive data collection
2. **Add real-time refresh** management and scheduling
3. **Create navigation shortcuts** and event handling
4. **Add dashboard state** management and persistence

#### Step 2: Dashboard Components (6 hours)
1. **Implement SystemOverviewComponent** with resource gauges
2. **Add DataflowSummaryComponent** with health indicators
3. **Create PerformanceChartsComponent** for metrics visualization
4. **Add RecentActivityComponent** for activity tracking

#### Step 3: Widget Management (3 hours)
1. **Implement DashboardWidgetManager** with layout presets
2. **Add adaptive layout** for different terminal sizes
3. **Create widget configuration** and customization system
4. **Add layout persistence** and user preferences

#### Step 4: Data Collection and Refresh (2 hours)
1. **Add comprehensive data** collection from system and daemon
2. **Implement efficient refresh** scheduling and rate limiting
3. **Add error handling** for disconnected states
4. **Create data caching** for smooth updates

#### Step 5: Testing and Polish (2 hours)
1. **Add comprehensive unit tests** for all components
2. **Test dashboard performance** under various loads
3. **Validate adaptive layouts** for different terminal sizes
4. **Test navigation** and event handling

## 🔗 Dependencies
**Depends On:**
- Issue #023 (TUI Architecture Foundation) - Base view and component system
- Phase 2 enhanced commands for navigation targets

**Enables:**
- Central navigation hub for all TUI views
- System monitoring and overview capabilities

## 🧪 Testing Requirements

### Unit Tests
```rust
#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_dashboard_layout_adaptation() {
        let widget_manager = DashboardWidgetManager::new();
        
        let compact_layout = widget_manager.get_layout_for_size(60, 20);
        let standard_layout = widget_manager.get_layout_for_size(100, 30);
        let wide_layout = widget_manager.get_layout_for_size(140, 40);
        
        assert!(compact_layout.is_single_column());
        assert!(standard_layout.is_two_column());
        assert!(wide_layout.is_three_column());
    }
    
    #[test]
    fn test_system_overview_data_collection() {
        let dashboard = DashboardView::new();
        
        let system_overview = dashboard.collect_system_overview().await.unwrap();
        
        assert!(system_overview.cpu_usage >= 0.0);
        assert!(system_overview.memory_usage.total_mb > 0);
    }
    
    #[test]
    fn test_refresh_manager_timing() {
        let refresh_manager = RefreshManager::new(Duration::from_secs(2));
        
        assert!(!refresh_manager.should_refresh());
        
        tokio::time::sleep(Duration::from_secs(3)).await;
        assert!(refresh_manager.should_refresh());
    }
}
```

## ✅ Definition of Done
- [ ] DashboardView provides comprehensive system overview with real-time updates
- [ ] Dashboard components render system, dataflow, and performance information clearly
- [ ] Adaptive layout works correctly for different terminal sizes
- [ ] Navigation shortcuts provide quick access to detailed views
- [ ] Refresh management maintains data currency without performance impact
- [ ] Widget management enables dashboard customization
- [ ] Performance targets met for rendering and data collection
- [ ] Dashboard responds smoothly to user interactions
- [ ] Error handling gracefully manages disconnected states
- [ ] Comprehensive unit tests validate all dashboard functionality
- [ ] Integration tests confirm navigation and data flow
- [ ] Manual testing validates user experience and information utility

This interactive dashboard serves as the central hub for the Dora TUI system, providing users with a comprehensive overview and efficient navigation to all system components and detailed views.