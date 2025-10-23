# Issue #027: Build Interactive Node Inspector View

## 📋 Summary
Implement a comprehensive interactive node inspector that provides detailed examination of individual nodes within dataflows, including real-time metrics, configuration details, input/output analysis, and debugging capabilities. This view serves as a critical tool for developers to understand node behavior and diagnose issues at the node level.

## 🎯 Objectives
- Create detailed node inspection interface with comprehensive information display
- Implement real-time monitoring of node metrics and performance
- Add interactive debugging and configuration capabilities
- Provide input/output stream analysis and visualization
- Enable direct node manipulation and testing from the inspector

**Success Metrics:**
- Node data refresh completes within 100ms for real-time monitoring
- Inspector handles nodes with 1000+ connections without performance degradation
- Debugging session setup completes within 500ms
- User task completion rate for node troubleshooting exceeds 90%
- Information clarity and usefulness rated 9/10 or higher by developers

## 🛠️ Technical Requirements

### What to Build

#### 1. Node Inspector View Implementation
```rust
// src/tui/views/node_inspector.rs
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
    widgets::{Block, Borders, Clear, Tabs},
    Frame,
};
use std::collections::HashMap;
use tokio::time::{Duration, Instant};

pub struct NodeInspectorView {
    base: BaseView,
    node_context: NodeInspectionContext,
    inspector_state: NodeInspectorState,
    tabs: NodeInspectorTabs,
    real_time_monitor: RealTimeNodeMonitor,
    debug_session: Option<NodeDebugSession>,
    last_update: Instant,
}

#[derive(Debug, Clone)]
pub struct NodeInspectionContext {
    pub dataflow_name: String,
    pub node_name: String,
    pub node_id: String,
    pub node_type: NodeType,
    pub inspection_mode: InspectionMode,
}

#[derive(Debug)]
pub struct NodeInspectorState {
    pub node_details: Option<NodeDetails>,
    pub metrics_data: NodeMetricsData,
    pub configuration: NodeConfiguration,
    pub connections: NodeConnections,
    pub logs: NodeLogs,
    pub performance_data: NodePerformanceData,
    pub health_status: NodeHealthStatus,
}

#[derive(Debug)]
pub struct NodeInspectorTabs {
    current_tab: usize,
    tab_names: Vec<String>,
    tab_components: HashMap<String, Box<dyn Component>>,
}

impl NodeInspectorView {
    pub fn new(dataflow_name: String, node_name: String) -> Self {
        let mut base = BaseView::new();
        
        let node_context = NodeInspectionContext {
            dataflow_name: dataflow_name.clone(),
            node_name: node_name.clone(),
            node_id: format!("{}::{}", dataflow_name, node_name),
            node_type: NodeType::Unknown, // Will be resolved during initialization
            inspection_mode: InspectionMode::Overview,
        };
        
        // Configure layout for node inspector
        let layout = LayoutConfig::tabs(vec![
            TabConfig {
                name: "Overview".to_string(),
                layout: LayoutConfig::split_vertical(vec![
                    LayoutSection {
                        constraint: Constraint::Percentage(40),
                        component_id: ComponentId("node_details".to_string()),
                    },
                    LayoutSection {
                        constraint: Constraint::Percentage(60),
                        component_id: ComponentId("node_metrics".to_string()),
                    },
                ]),
            },
            TabConfig {
                name: "Configuration".to_string(),
                layout: LayoutConfig::single(ComponentId("node_configuration".to_string())),
            },
            TabConfig {
                name: "Connections".to_string(),
                layout: LayoutConfig::split_horizontal(vec![
                    LayoutSection {
                        constraint: Constraint::Percentage(50),
                        component_id: ComponentId("input_connections".to_string()),
                    },
                    LayoutSection {
                        constraint: Constraint::Percentage(50),
                        component_id: ComponentId("output_connections".to_string()),
                    },
                ]),
            },
            TabConfig {
                name: "Performance".to_string(),
                layout: LayoutConfig::grid(vec![
                    LayoutRow {
                        height: Constraint::Percentage(50),
                        columns: vec![
                            LayoutColumn {
                                width: Constraint::Percentage(50),
                                component_id: ComponentId("cpu_usage_chart".to_string()),
                            },
                            LayoutColumn {
                                width: Constraint::Percentage(50),
                                component_id: ComponentId("memory_usage_chart".to_string()),
                            },
                        ],
                    },
                    LayoutRow {
                        height: Constraint::Percentage(50),
                        columns: vec![
                            LayoutColumn {
                                width: Constraint::Percentage(50),
                                component_id: ComponentId("throughput_chart".to_string()),
                            },
                            LayoutColumn {
                                width: Constraint::Percentage(50),
                                component_id: ComponentId("latency_chart".to_string()),
                            },
                        ],
                    },
                ]),
            },
            TabConfig {
                name: "Logs".to_string(),
                layout: LayoutConfig::single(ComponentId("node_logs".to_string())),
            },
            TabConfig {
                name: "Debug".to_string(),
                layout: LayoutConfig::split_vertical(vec![
                    LayoutSection {
                        constraint: Constraint::Length(8),
                        component_id: ComponentId("debug_controls".to_string()),
                    },
                    LayoutSection {
                        constraint: Constraint::Min(12),
                        component_id: ComponentId("debug_session".to_string()),
                    },
                ]),
            },
        ]);
        
        base.set_layout(layout);
        
        // Add components
        base.add_component(
            ComponentId("node_details".to_string()),
            NodeDetailsComponent::new(node_context.clone()),
        );
        
        base.add_component(
            ComponentId("node_metrics".to_string()),
            NodeMetricsComponent::new(node_context.clone()),
        );
        
        base.add_component(
            ComponentId("node_configuration".to_string()),
            NodeConfigurationComponent::new(node_context.clone()),
        );
        
        base.add_component(
            ComponentId("input_connections".to_string()),
            InputConnectionsComponent::new(node_context.clone()),
        );
        
        base.add_component(
            ComponentId("output_connections".to_string()),
            OutputConnectionsComponent::new(node_context.clone()),
        );
        
        base.add_component(
            ComponentId("cpu_usage_chart".to_string()),
            NodePerformanceChartComponent::new(node_context.clone(), ChartType::CpuUsage),
        );
        
        base.add_component(
            ComponentId("memory_usage_chart".to_string()),
            NodePerformanceChartComponent::new(node_context.clone(), ChartType::MemoryUsage),
        );
        
        base.add_component(
            ComponentId("throughput_chart".to_string()),
            NodePerformanceChartComponent::new(node_context.clone(), ChartType::Throughput),
        );
        
        base.add_component(
            ComponentId("latency_chart".to_string()),
            NodePerformanceChartComponent::new(node_context.clone(), ChartType::Latency),
        );
        
        base.add_component(
            ComponentId("node_logs".to_string()),
            NodeLogsComponent::new(node_context.clone()),
        );
        
        base.add_component(
            ComponentId("debug_controls".to_string()),
            NodeDebugControlsComponent::new(node_context.clone()),
        );
        
        base.add_component(
            ComponentId("debug_session".to_string()),
            NodeDebugSessionComponent::new(node_context.clone()),
        );
        
        Self {
            base,
            node_context,
            inspector_state: NodeInspectorState::default(),
            tabs: NodeInspectorTabs::new(),
            real_time_monitor: RealTimeNodeMonitor::new(),
            debug_session: None,
            last_update: Instant::now(),
        }
    }
    
    async fn refresh_node_data(&mut self) -> io::Result<()> {
        // Refresh node details
        self.inspector_state.node_details = self.collect_node_details().await?;
        
        // Refresh real-time metrics
        self.inspector_state.metrics_data = self.collect_node_metrics().await?;
        
        // Refresh performance data
        self.inspector_state.performance_data = self.collect_performance_data().await?;
        
        // Update health status
        self.inspector_state.health_status = self.assess_node_health().await?;
        
        // Refresh logs if logs tab is active
        if self.tabs.current_tab_name() == "Logs" {
            self.inspector_state.logs = self.collect_node_logs().await?;
        }
        
        self.last_update = Instant::now();
        Ok(())
    }
    
    async fn collect_node_details(&self) -> io::Result<Option<NodeDetails>> {
        if let Ok(daemon_client) = DaemonClient::connect().await {
            if let Ok(node_info) = daemon_client.get_node_details(
                &self.node_context.dataflow_name,
                &self.node_context.node_name
            ).await {
                Ok(Some(NodeDetails {
                    basic_info: NodeBasicInfo {
                        name: node_info.name,
                        node_type: node_info.node_type,
                        status: node_info.status,
                        uptime: node_info.uptime,
                        last_restart: node_info.last_restart,
                    },
                    resource_usage: NodeResourceUsage {
                        cpu_usage: node_info.cpu_usage,
                        memory_usage: node_info.memory_usage,
                        disk_usage: node_info.disk_usage,
                        network_usage: node_info.network_usage,
                    },
                    runtime_info: NodeRuntimeInfo {
                        process_id: node_info.process_id,
                        thread_count: node_info.thread_count,
                        file_descriptors: node_info.file_descriptors,
                        environment: node_info.environment,
                    },
                }))
            } else {
                Ok(None)
            }
        } else {
            Ok(None)
        }
    }
    
    async fn collect_node_metrics(&self) -> io::Result<NodeMetricsData> {
        if let Ok(daemon_client) = DaemonClient::connect().await {
            if let Ok(metrics) = daemon_client.get_node_metrics(
                &self.node_context.dataflow_name,
                &self.node_context.node_name
            ).await {
                Ok(NodeMetricsData {
                    message_counts: MessageCounts {
                        messages_received: metrics.messages_received,
                        messages_sent: metrics.messages_sent,
                        messages_processed: metrics.messages_processed,
                        messages_dropped: metrics.messages_dropped,
                    },
                    performance_metrics: PerformanceMetrics {
                        processing_latency: metrics.processing_latency,
                        throughput: metrics.throughput,
                        error_rate: metrics.error_rate,
                        queue_depth: metrics.queue_depth,
                    },
                    timing_metrics: TimingMetrics {
                        average_processing_time: metrics.average_processing_time,
                        p95_processing_time: metrics.p95_processing_time,
                        p99_processing_time: metrics.p99_processing_time,
                        max_processing_time: metrics.max_processing_time,
                    },
                })
            } else {
                Ok(NodeMetricsData::default())
            }
        } else {
            Ok(NodeMetricsData::default())
        }
    }
    
    async fn start_debug_session(&mut self) -> io::Result<()> {
        if let Ok(daemon_client) = DaemonClient::connect().await {
            let debug_session = daemon_client.start_node_debug_session(
                &self.node_context.dataflow_name,
                &self.node_context.node_name,
                NodeDebugConfig::default()
            ).await?;
            
            self.debug_session = Some(NodeDebugSession {
                session_id: debug_session.session_id,
                breakpoints: Vec::new(),
                variable_watches: Vec::new(),
                call_stack: Vec::new(),
                current_state: DebugState::Paused,
            });
        }
        
        Ok(())
    }
}

#[async_trait]
impl View for NodeInspectorView {
    async fn initialize(&mut self, context: &AppContext) -> io::Result<()> {
        // Initialize node inspection data
        self.refresh_node_data().await?;
        
        // Resolve node type based on collected data
        if let Some(details) = &self.inspector_state.node_details {
            self.node_context.node_type = details.basic_info.node_type.clone();
        }
        
        // Initialize all components
        for component in self.base.component_registry.components_mut() {
            component.update(context).await?;
        }
        
        // Start real-time monitoring
        self.real_time_monitor.start(&self.node_context).await?;
        
        Ok(())
    }
    
    async fn update(&mut self, context: &mut AppContext) -> io::Result<()> {
        // Check if it's time to refresh data
        if self.last_update.elapsed() > Duration::from_millis(500) {
            self.refresh_node_data().await?;
        }
        
        // Update all components with latest data
        for component in self.base.component_registry.components_mut() {
            component.update(context).await?;
        }
        
        // Update debug session if active
        if let Some(debug_session) = &mut self.debug_session {
            debug_session.update().await?;
        }
        
        Ok(())
    }
    
    fn render(&self, frame: &mut Frame, area: Rect, theme: &Theme, context: &AppContext) {
        // Render tab bar
        let tabs_area = Rect::new(area.x, area.y, area.width, 3);
        let content_area = Rect::new(area.x, area.y + 3, area.width, area.height - 3);
        
        self.render_tab_bar(frame, tabs_area, theme);
        
        // Render current tab content
        self.base.render_components(frame, content_area, theme, context);
        
        // Render node inspector overlays
        self.render_health_status_overlay(frame, area, theme);
        
        if self.debug_session.is_some() {
            self.render_debug_overlay(frame, area, theme);
        }
    }
    
    async fn handle_event(&mut self, event: ViewEvent, context: &mut AppContext) -> io::Result<ViewEventResult> {
        // Handle tab switching
        if let ViewEvent::Key(key_event) = &event {
            match key_event.code {
                crossterm::event::KeyCode::Tab => {
                    self.tabs.next_tab();
                    return Ok(ViewEventResult::None);
                },
                
                crossterm::event::KeyCode::BackTab => {
                    self.tabs.previous_tab();
                    return Ok(ViewEventResult::None);
                },
                
                crossterm::event::KeyCode::Char('r') => {
                    // Manual refresh
                    self.refresh_node_data().await?;
                    return Ok(ViewEventResult::None);
                },
                
                crossterm::event::KeyCode::Char('d') => {
                    // Start/stop debug session
                    if self.debug_session.is_none() {
                        self.start_debug_session().await?;
                        self.tabs.switch_to_tab("Debug");
                    } else {
                        self.debug_session = None;
                    }
                    return Ok(ViewEventResult::None);
                },
                
                crossterm::event::KeyCode::Char('c') => {
                    // Configure node
                    return Ok(ViewEventResult::PushView(ViewType::NodeConfiguration {
                        dataflow_name: self.node_context.dataflow_name.clone(),
                        node_name: self.node_context.node_name.clone(),
                    }));
                },
                
                crossterm::event::KeyCode::Char('l') => {
                    // View detailed logs
                    return Ok(ViewEventResult::PushView(ViewType::LogViewer {
                        target: LogTarget::Node {
                            dataflow: self.node_context.dataflow_name.clone(),
                            node: self.node_context.node_name.clone(),
                        },
                        filters: LogFilters::default(),
                    }));
                },
                
                crossterm::event::KeyCode::Char('p') => {
                    // Performance analysis
                    return Ok(ViewEventResult::PushView(ViewType::PerformanceAnalyzer {
                        analysis_type: AnalysisType::Node {
                            dataflow: self.node_context.dataflow_name.clone(),
                            node: self.node_context.node_name.clone(),
                        }
                    }));
                },
                
                _ => {}
            }
        }
        
        // Forward event to components
        self.base.handle_component_event(event, context).await
    }
    
    async fn handle_resize(&mut self, width: u16, height: u16) -> io::Result<()> {
        self.base.layout_manager.handle_resize(width, height);
        Ok(())
    }
    
    async fn cleanup(&self, context: &AppContext) -> io::Result<()> {
        // Stop real-time monitoring
        self.real_time_monitor.stop().await;
        
        // End debug session if active
        if let Some(debug_session) = &self.debug_session {
            if let Ok(daemon_client) = DaemonClient::connect().await {
                let _ = daemon_client.end_debug_session(&debug_session.session_id).await;
            }
        }
        
        Ok(())
    }
    
    fn view_type(&self) -> ViewType {
        ViewType::NodeInspector {
            dataflow_name: self.node_context.dataflow_name.clone(),
            node_name: self.node_context.node_name.clone(),
        }
    }
    
    fn title(&self) -> String {
        format!("Node Inspector: {}", self.node_context.node_name)
    }
    
    fn help_text(&self) -> Vec<HelpItem> {
        vec![
            HelpItem::new("Tab", "Switch tabs"),
            HelpItem::new("r", "Refresh data"),
            HelpItem::new("d", "Debug session"),
            HelpItem::new("c", "Configure node"),
            HelpItem::new("l", "View logs"),
            HelpItem::new("p", "Performance analysis"),
            HelpItem::new("Esc", "Back"),
        ]
    }
}
```

#### 2. Node Details Component
```rust
// src/tui/components/node/node_details.rs
pub struct NodeDetailsComponent {
    node_context: NodeInspectionContext,
    node_data: Option<NodeDetails>,
    focused: bool,
    last_update: Instant,
}

impl NodeDetailsComponent {
    pub fn new(node_context: NodeInspectionContext) -> Self {
        Self {
            node_context,
            node_data: None,
            focused: false,
            last_update: Instant::now(),
        }
    }
    
    fn render_basic_info(&self, frame: &mut Frame, area: Rect, theme: &Theme) {
        if let Some(details) = &self.node_data {
            let info_text = vec![
                format!("Name: {}", details.basic_info.name),
                format!("Type: {}", details.basic_info.node_type),
                format!("Status: {}", details.basic_info.status),
                format!("Uptime: {}", format_duration(details.basic_info.uptime)),
                format!("Last Restart: {}", details.basic_info.last_restart
                    .map(|t| t.format("%Y-%m-%d %H:%M:%S").to_string())
                    .unwrap_or_else(|| "Never".to_string())),
            ];
            
            let paragraph = Paragraph::new(info_text.join("\n"))
                .style(theme.normal_style())
                .wrap(ratatui::widgets::Wrap { trim: true });
            
            frame.render_widget(paragraph, area);
        }
    }
    
    fn render_resource_usage(&self, frame: &mut Frame, area: Rect, theme: &Theme) {
        if let Some(details) = &self.node_data {
            let usage = &details.resource_usage;
            
            let chunks = Layout::default()
                .direction(Direction::Vertical)
                .constraints([
                    Constraint::Length(2),
                    Constraint::Length(2),
                    Constraint::Length(2),
                    Constraint::Length(2),
                ].as_ref())
                .split(area);
            
            // CPU usage gauge
            let cpu_gauge = Gauge::default()
                .block(Block::default().title("CPU"))
                .gauge_style(if usage.cpu_usage > 80.0 {
                    theme.error_style()
                } else if usage.cpu_usage > 60.0 {
                    theme.warning_style()
                } else {
                    theme.success_style()
                })
                .percent(usage.cpu_usage as u16)
                .label(format!("{:.1}%", usage.cpu_usage));
            
            frame.render_widget(cpu_gauge, chunks[0]);
            
            // Memory usage gauge
            let memory_gauge = Gauge::default()
                .block(Block::default().title("Memory"))
                .gauge_style(if usage.memory_usage.usage_percent > 80.0 {
                    theme.error_style()
                } else if usage.memory_usage.usage_percent > 60.0 {
                    theme.warning_style()
                } else {
                    theme.success_style()
                })
                .percent(usage.memory_usage.usage_percent as u16)
                .label(format!("{:.1}%", usage.memory_usage.usage_percent));
            
            frame.render_widget(memory_gauge, chunks[1]);
            
            // Disk usage gauge
            let disk_gauge = Gauge::default()
                .block(Block::default().title("Disk"))
                .gauge_style(if usage.disk_usage.usage_percent > 90.0 {
                    theme.error_style()
                } else if usage.disk_usage.usage_percent > 75.0 {
                    theme.warning_style()
                } else {
                    theme.success_style()
                })
                .percent(usage.disk_usage.usage_percent as u16)
                .label(format!("{:.1}%", usage.disk_usage.usage_percent));
            
            frame.render_widget(disk_gauge, chunks[2]);
            
            // Network usage indicator
            let network_text = format!(
                "Network: ↑{}/s ↓{}/s",
                format_bytes(usage.network_usage.bytes_sent_per_sec),
                format_bytes(usage.network_usage.bytes_received_per_sec)
            );
            
            let network_paragraph = Paragraph::new(network_text)
                .style(theme.normal_style());
            
            frame.render_widget(network_paragraph, chunks[3]);
        }
    }
}

#[async_trait]
impl Component for NodeDetailsComponent {
    async fn update(&mut self, context: &AppContext) -> io::Result<()> {
        // Data is updated from the parent view
        Ok(())
    }
    
    fn render(&self, frame: &mut Frame, area: Rect, theme: &Theme, context: &AppContext) {
        let block = Block::default()
            .title("Node Details")
            .borders(Borders::ALL)
            .border_style(if self.focused {
                theme.focused_border_style()
            } else {
                theme.normal_border_style()
            });
        
        let inner_area = block.inner(area);
        frame.render_widget(block, area);
        
        // Split area for basic info and resource usage
        let chunks = Layout::default()
            .direction(Direction::Horizontal)
            .constraints([
                Constraint::Percentage(50),
                Constraint::Percentage(50),
            ].as_ref())
            .split(inner_area);
        
        self.render_basic_info(frame, chunks[0], theme);
        self.render_resource_usage(frame, chunks[1], theme);
    }
    
    async fn handle_event(&mut self, event: ComponentEvent, context: &AppContext) -> io::Result<ViewEventResult> {
        Ok(ViewEventResult::None)
    }
    
    fn component_type(&self) -> ComponentType {
        ComponentType::NodeDetails
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
```

#### 3. Real-Time Node Monitor
```rust
// src/tui/components/node/real_time_monitor.rs
#[derive(Debug)]
pub struct RealTimeNodeMonitor {
    monitoring_active: bool,
    metrics_stream: Option<MetricsStream>,
    performance_tracker: PerformanceTracker,
    alert_manager: NodeAlertManager,
}

impl RealTimeNodeMonitor {
    pub fn new() -> Self {
        Self {
            monitoring_active: false,
            metrics_stream: None,
            performance_tracker: PerformanceTracker::new(),
            alert_manager: NodeAlertManager::new(),
        }
    }
    
    pub async fn start(&mut self, node_context: &NodeInspectionContext) -> io::Result<()> {
        if let Ok(daemon_client) = DaemonClient::connect().await {
            self.metrics_stream = Some(
                daemon_client.create_node_metrics_stream(
                    &node_context.dataflow_name,
                    &node_context.node_name,
                    Duration::from_millis(100) // 10 Hz refresh rate
                ).await?
            );
            
            self.monitoring_active = true;
            self.performance_tracker.start();
        }
        
        Ok(())
    }
    
    pub async fn stop(&self) {
        // Stop metrics stream and cleanup
        if let Some(stream) = &self.metrics_stream {
            stream.close().await;
        }
    }
    
    pub async fn get_latest_metrics(&self) -> Option<NodeMetricsData> {
        if let Some(stream) = &self.metrics_stream {
            stream.get_latest().await
        } else {
            None
        }
    }
}
```

### Why This Approach

**Comprehensive Node Visibility:**
- Detailed inspection of individual nodes with all relevant information
- Real-time monitoring and performance tracking
- Interactive debugging capabilities

**Developer-Focused Interface:**
- Tab-based organization for different aspects of node inspection
- Direct access to configuration and debugging tools
- Integration with logs and performance analysis

**Performance Optimized:**
- Efficient real-time data updates without overwhelming the interface
- Selective data loading based on active tabs
- Optimized rendering for large amounts of node data

### How to Implement

#### Step 1: Node Inspector View Core (5 hours)
1. **Implement NodeInspectorView** with comprehensive tab system
2. **Add real-time data** collection and refresh management
3. **Create node context** handling and resolution
4. **Add navigation** and event handling

#### Step 2: Node Details and Metrics Components (4 hours)
1. **Implement NodeDetailsComponent** with resource usage visualization
2. **Add NodeMetricsComponent** with real-time metrics display
3. **Create performance charts** for CPU, memory, throughput, and latency
4. **Add connection visualization** components

#### Step 3: Configuration and Debug Components (4 hours)
1. **Implement NodeConfigurationComponent** with editable settings
2. **Add NodeDebugControlsComponent** for debug session management
3. **Create NodeDebugSessionComponent** for interactive debugging
4. **Add breakpoint and variable watch** functionality

#### Step 4: Real-Time Monitoring (3 hours)
1. **Implement RealTimeNodeMonitor** with efficient data streaming
2. **Add performance tracking** and alert management
3. **Create metrics aggregation** and historical data handling
4. **Add health status** assessment and monitoring

#### Step 5: Integration and Testing (2 hours)
1. **Add comprehensive unit tests** for all components
2. **Test real-time monitoring** performance and accuracy
3. **Validate debugging session** functionality
4. **Test tab navigation** and event handling

## 🔗 Dependencies
**Depends On:**
- Issue #023 (TUI Architecture Foundation) - Base view and component system
- Issue #024 (Dashboard Overview) - Navigation integration
- Phase 2 enhanced commands for inspector data

**Enables:**
- Deep node analysis and troubleshooting capabilities
- Integration with debugging and performance analysis tools

## 🧪 Testing Requirements

### Unit Tests
```rust
#[cfg(test)]\nmod tests {\n    use super::*;\n    \n    #[test]\n    fn test_node_inspector_initialization() {\n        let inspector = NodeInspectorView::new(\n            \"test-dataflow\".to_string(),\n            \"test-node\".to_string()\n        );\n        \n        assert_eq!(inspector.node_context.dataflow_name, \"test-dataflow\");\n        assert_eq!(inspector.node_context.node_name, \"test-node\");\n        assert_eq!(inspector.tabs.current_tab(), 0);\n    }\n    \n    #[test]\n    fn test_real_time_monitor_lifecycle() {\n        let mut monitor = RealTimeNodeMonitor::new();\n        let context = NodeInspectionContext::test_context();\n        \n        monitor.start(&context).await.unwrap();\n        assert!(monitor.monitoring_active);\n        \n        monitor.stop().await;\n        assert!(!monitor.monitoring_active);\n    }\n    \n    #[test]\n    fn test_node_details_rendering() {\n        let component = NodeDetailsComponent::new(NodeInspectionContext::test_context());\n        \n        // Test that component renders without panicking\n        let mut frame = TestFrame::new(80, 24);\n        let area = Rect::new(0, 0, 80, 24);\n        let theme = Theme::default_dark();\n        let context = AppContext::test_context();\n        \n        component.render(&mut frame, area, &theme, &context);\n        \n        // Verify basic structure is rendered\n        assert!(frame.buffer_contains(\"Node Details\"));\n    }\n}
```

## ✅ Definition of Done
- [ ] NodeInspectorView provides comprehensive node examination with tabbed interface
- [ ] Real-time monitoring displays current metrics without performance impact
- [ ] Node details component shows all relevant node information clearly
- [ ] Configuration component allows interactive node settings modification
- [ ] Connection components visualize input/output relationships effectively
- [ ] Performance charts display historical data with proper scaling
- [ ] Debug session integration enables interactive node debugging
- [ ] Tab navigation works smoothly with proper focus management
- [ ] Real-time data updates maintain target refresh rates
- [ ] Memory usage stays within limits during extended monitoring
- [ ] Comprehensive unit tests validate all inspector functionality
- [ ] Integration tests confirm navigation and data accuracy
- [ ] Manual testing validates developer workflow effectiveness

This interactive node inspector provides developers with comprehensive visibility into individual node behavior, enabling effective debugging and optimization at the node level within the Dora TUI system.