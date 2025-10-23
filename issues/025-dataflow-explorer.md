# Issue #025: Build Interactive Dataflow Explorer

## 📋 Summary
Implement a comprehensive interactive dataflow explorer that provides detailed visualization, real-time monitoring, and management of individual dataflows and their nodes. This view serves as the primary interface for dataflow inspection, debugging, and optimization with rich visual representations and interactive controls.

## 🎯 Objectives
- Create comprehensive dataflow visualization with node graph and dependency mapping
- Implement real-time monitoring of dataflow performance and node health
- Add interactive node inspection and configuration management
- Provide dataflow lifecycle management (start, stop, restart, configure)
- Enable visual debugging with message flow tracing and bottleneck identification

**Success Metrics:**
- Dataflow visualization rendering completes within 200ms for typical dataflows
- Real-time updates maintain 1-second refresh rate without performance degradation
- Node interaction response time under 100ms
- Visual debugging accuracy identifies 90% of common bottlenecks
- User task completion rate above 85% for dataflow management operations

## 🛠️ Technical Requirements

### What to Build

#### 1. Dataflow Explorer View
```rust
// src/tui/views/dataflow_explorer.rs
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
    widgets::{Block, Borders, List, ListItem, Paragraph, Tabs},
    Frame,
};
use std::collections::HashMap;

pub struct DataflowExplorerView {
    base: BaseView,
    dataflow_name: String,
    dataflow_state: DataflowState,
    current_tab: DataflowTab,
    node_selection: NodeSelection,
    visualization_mode: VisualizationMode,
    real_time_monitoring: RealTimeMonitoring,
    interaction_manager: DataflowInteractionManager,
}

#[derive(Debug, Clone)]
pub struct DataflowState {
    pub dataflow_info: DataflowInfo,
    pub nodes: Vec<NodeInfo>,
    pub connections: Vec<NodeConnection>,
    pub configuration: DataflowConfiguration,
    pub runtime_metrics: DataflowMetrics,
    pub message_flows: Vec<MessageFlow>,
    pub current_issues: Vec<DataflowIssue>,
}

#[derive(Debug, Clone)]
pub enum DataflowTab {
    Overview,
    Nodes,
    Performance,
    Configuration,
    Logs,
    Debug,
}

#[derive(Debug, Clone)]
pub struct NodeSelection {
    pub selected_node: Option<String>,
    pub hovered_node: Option<String>,
    pub multi_selection: Vec<String>,
}

#[derive(Debug, Clone)]
pub enum VisualizationMode {
    GraphView,
    ListViewGrouped,
    ListViewFlat,
    DependencyTree,
    PerformanceHeatmap,
}

impl DataflowExplorerView {
    pub fn new(dataflow_name: String) -> Self {
        let mut base = BaseView::new();
        
        // Configure explorer layout
        let layout = LayoutConfig::custom(vec![
            LayoutSection {
                section_type: SectionType::Tabs,
                area: SectionArea::TopBar(3),
                component_id: ComponentId("tabs".to_string()),
            },
            LayoutSection {
                section_type: SectionType::Main,
                area: SectionArea::MainContent,
                component_id: ComponentId("main_content".to_string()),
            },
            LayoutSection {
                section_type: SectionType::Sidebar,
                area: SectionArea::RightSidebar(30),
                component_id: ComponentId("node_inspector".to_string()),
            },
            LayoutSection {
                section_type: SectionType::StatusBar,
                area: SectionArea::BottomBar(3),
                component_id: ComponentId("status_bar".to_string()),
            },
        ]);
        
        base.set_layout(layout);
        
        // Add dataflow explorer components
        base.add_component(
            ComponentId("tabs".to_string()),
            DataflowTabsComponent::new(),
        );
        
        base.add_component(
            ComponentId("main_content".to_string()),
            DataflowMainContentComponent::new(),
        );
        
        base.add_component(
            ComponentId("node_inspector".to_string()),
            NodeInspectorComponent::new(),
        );
        
        base.add_component(
            ComponentId("status_bar".to_string()),
            DataflowStatusBarComponent::new(),
        );
        
        Self {
            base,
            dataflow_name,
            dataflow_state: DataflowState::default(),
            current_tab: DataflowTab::Overview,
            node_selection: NodeSelection::default(),
            visualization_mode: VisualizationMode::GraphView,
            real_time_monitoring: RealTimeMonitoring::new(),
            interaction_manager: DataflowInteractionManager::new(),
        }
    }
    
    async fn refresh_dataflow_data(&mut self) -> io::Result<()> {
        if let Ok(daemon_client) = DaemonClient::connect().await {
            // Get dataflow information
            self.dataflow_state.dataflow_info = daemon_client
                .get_dataflow_info(&self.dataflow_name)
                .await
                .unwrap_or_default();
            
            // Get nodes
            self.dataflow_state.nodes = daemon_client
                .get_dataflow_nodes(&self.dataflow_name)
                .await
                .unwrap_or_default();
            
            // Get node connections and dependencies
            self.dataflow_state.connections = self.analyze_node_connections().await?;
            
            // Get configuration
            self.dataflow_state.configuration = daemon_client
                .get_dataflow_configuration(&self.dataflow_name)
                .await
                .unwrap_or_default();
            
            // Get runtime metrics
            self.dataflow_state.runtime_metrics = daemon_client
                .get_dataflow_metrics(&self.dataflow_name)
                .await
                .unwrap_or_default();
            
            // Analyze message flows
            self.dataflow_state.message_flows = self.analyze_message_flows().await?;
            
            // Detect current issues
            self.dataflow_state.current_issues = self.detect_dataflow_issues().await?;
        }
        
        Ok(())
    }
    
    async fn analyze_node_connections(&self) -> io::Result<Vec<NodeConnection>> {
        let mut connections = Vec::new();
        
        // Analyze input/output connections between nodes
        for node in &self.dataflow_state.nodes {
            // Parse node inputs and outputs from configuration
            for input in &node.inputs {
                if let Some(source_node) = self.find_source_node(&input.source) {
                    connections.push(NodeConnection {
                        source_node: source_node.clone(),
                        target_node: node.name.clone(),
                        connection_type: ConnectionType::DataFlow,
                        message_rate: input.message_rate.unwrap_or(0.0),
                        latency: input.latency.unwrap_or(Duration::ZERO),
                        health_status: self.assess_connection_health(&source_node, &node.name),
                    });
                }
            }
        }
        
        Ok(connections)
    }
    
    async fn analyze_message_flows(&self) -> io::Result<Vec<MessageFlow>> {
        let mut message_flows = Vec::new();
        
        // Trace message flows through the dataflow
        for connection in &self.dataflow_state.connections {
            let flow = MessageFlow {
                flow_id: format!("{}_{}", connection.source_node, connection.target_node),
                path: vec![connection.source_node.clone(), connection.target_node.clone()],
                message_count: self.get_message_count_for_connection(connection).await?,
                average_latency: connection.latency,
                throughput: connection.message_rate,
                bottleneck_score: self.calculate_bottleneck_score(connection),
                flow_health: connection.health_status.clone(),
            };
            
            message_flows.push(flow);
        }
        
        Ok(message_flows)
    }
    
    async fn detect_dataflow_issues(&self) -> io::Result<Vec<DataflowIssue>> {
        let mut issues = Vec::new();
        
        // Detect stalled nodes
        for node in &self.dataflow_state.nodes {
            if node.status == NodeStatus::Stalled || node.last_activity.elapsed() > Duration::from_secs(30) {
                issues.push(DataflowIssue {
                    issue_type: IssueType::NodeStalled,
                    affected_component: node.name.clone(),
                    severity: IssueSeverity::High,
                    description: format!("Node '{}' appears to be stalled", node.name),
                    detected_at: Utc::now(),
                    suggested_actions: vec![
                        "Check node logs for errors".to_string(),
                        "Restart the node".to_string(),
                        "Verify input data availability".to_string(),
                    ],
                });
            }
        }
        
        // Detect performance bottlenecks
        for connection in &self.dataflow_state.connections {
            if connection.latency > Duration::from_millis(1000) {
                issues.push(DataflowIssue {
                    issue_type: IssueType::HighLatency,
                    affected_component: format!("{} -> {}", connection.source_node, connection.target_node),
                    severity: IssueSeverity::Medium,
                    description: format!("High latency detected: {:.1}ms", connection.latency.as_millis()),
                    detected_at: Utc::now(),
                    suggested_actions: vec![
                        "Check network connectivity".to_string(),
                        "Optimize message processing".to_string(),
                        "Scale processing capacity".to_string(),
                    ],
                });
            }
        }
        
        // Detect resource exhaustion
        for node in &self.dataflow_state.nodes {
            if let Some(cpu_usage) = node.cpu_usage {
                if cpu_usage > 90.0 {
                    issues.push(DataflowIssue {
                        issue_type: IssueType::ResourceExhaustion,
                        affected_component: node.name.clone(),
                        severity: IssueSeverity::High,
                        description: format!("High CPU usage: {:.1}%", cpu_usage),
                        detected_at: Utc::now(),
                        suggested_actions: vec![
                            "Scale node horizontally".to_string(),
                            "Optimize node processing".to_string(),
                            "Review resource allocation".to_string(),
                        ],
                    });
                }
            }
        }
        
        Ok(issues)
    }
}

#[async_trait]
impl View for DataflowExplorerView {
    async fn initialize(&mut self, context: &AppContext) -> io::Result<()> {
        // Load dataflow data
        self.refresh_dataflow_data().await?;
        
        // Initialize components with dataflow data
        for component in self.base.component_registry.components_mut() {
            component.update(context).await?;
        }
        
        // Start real-time monitoring
        self.real_time_monitoring.start(&self.dataflow_name).await?;
        
        Ok(())
    }
    
    async fn update(&mut self, context: &mut AppContext) -> io::Result<()> {
        // Check for real-time updates
        if let Some(update) = self.real_time_monitoring.get_latest_update() {
            self.apply_real_time_update(update);
        }
        
        // Periodic refresh
        if self.should_refresh_data() {
            self.refresh_dataflow_data().await?;
        }
        
        // Update all components
        for component in self.base.component_registry.components_mut() {
            component.update(context).await?;
        }
        
        Ok(())
    }
    
    fn render(&self, frame: &mut Frame, area: Rect, theme: &Theme, context: &AppContext) {
        // Render main explorer layout
        self.base.render_components(frame, area, theme, context);
        
        // Render overlays based on current mode
        match self.current_tab {
            DataflowTab::Debug => {
                self.render_debug_overlay(frame, area, theme);
            },
            _ => {}
        }
        
        // Render issues overlay if there are critical issues
        if self.has_critical_issues() {
            self.render_issues_overlay(frame, area, theme);
        }
    }
    
    async fn handle_event(&mut self, event: ViewEvent, context: &mut AppContext) -> io::Result<ViewEventResult> {
        // Handle dataflow explorer specific key bindings
        if let ViewEvent::Key(key_event) = &event {
            match key_event.code {
                crossterm::event::KeyCode::Tab => {
                    self.next_tab();
                    return Ok(ViewEventResult::None);
                },
                
                crossterm::event::KeyCode::BackTab => {
                    self.previous_tab();
                    return Ok(ViewEventResult::None);
                },
                
                crossterm::event::KeyCode::Char('g') => {
                    // Switch to graph view
                    self.visualization_mode = VisualizationMode::GraphView;
                    return Ok(ViewEventResult::None);
                },
                
                crossterm::event::KeyCode::Char('l') => {
                    // Switch to list view
                    self.visualization_mode = VisualizationMode::ListViewGrouped;
                    return Ok(ViewEventResult::None);
                },
                
                crossterm::event::KeyCode::Char('h') => {
                    // Switch to heatmap view
                    self.visualization_mode = VisualizationMode::PerformanceHeatmap;
                    return Ok(ViewEventResult::None);
                },
                
                crossterm::event::KeyCode::Char('s') => {
                    // Start/stop dataflow
                    self.toggle_dataflow().await?;
                    return Ok(ViewEventResult::None);
                },
                
                crossterm::event::KeyCode::Char('r') => {
                    // Restart dataflow
                    self.restart_dataflow().await?;
                    return Ok(ViewEventResult::None);
                },
                
                crossterm::event::KeyCode::Char('i') => {
                    // Inspect selected node
                    if let Some(node_name) = &self.node_selection.selected_node {
                        return Ok(ViewEventResult::PushView(ViewType::NodeInspector {
                            dataflow_name: self.dataflow_name.clone(),
                            node_name: node_name.clone(),
                        }));
                    }
                },
                
                crossterm::event::KeyCode::Char('d') => {
                    // Debug mode
                    return Ok(ViewEventResult::PushView(ViewType::DebugSession {
                        session_id: Uuid::new_v4().to_string(),
                        debug_mode: DebugMode::Interactive,
                    }));
                },
                
                crossterm::event::KeyCode::F5 => {
                    // Force refresh
                    self.refresh_dataflow_data().await?;
                    return Ok(ViewEventResult::None);
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
        self.real_time_monitoring.stop().await?;
        Ok(())
    }
    
    fn view_type(&self) -> ViewType {
        ViewType::DataflowDetail { dataflow_name: self.dataflow_name.clone() }
    }
    
    fn title(&self) -> String {
        format!("Dataflow: {}", self.dataflow_name)
    }
    
    fn help_text(&self) -> Vec<HelpItem> {
        vec![
            HelpItem::new("Tab", "Next tab"),
            HelpItem::new("Shift+Tab", "Previous tab"),
            HelpItem::new("g", "Graph view"),
            HelpItem::new("l", "List view"),
            HelpItem::new("h", "Heatmap view"),
            HelpItem::new("s", "Start/stop dataflow"),
            HelpItem::new("r", "Restart dataflow"),
            HelpItem::new("i", "Inspect selected node"),
            HelpItem::new("d", "Debug mode"),
            HelpItem::new("F5", "Refresh"),
            HelpItem::new("Esc", "Back"),
        ]
    }
}
```

#### 2. Dataflow Visualization Components
```rust
// src/tui/components/dataflow/graph_view.rs
pub struct DataflowGraphComponent {
    graph_data: GraphData,
    layout_engine: GraphLayoutEngine,
    node_positions: HashMap<String, Position>,
    selected_node: Option<String>,
    zoom_level: f32,
    pan_offset: (f32, f32),
    focused: bool,
}

#[derive(Debug, Clone)]
pub struct GraphData {
    pub nodes: Vec<GraphNode>,
    pub edges: Vec<GraphEdge>,
    pub clusters: Vec<NodeCluster>,
}

#[derive(Debug, Clone)]
pub struct GraphNode {
    pub id: String,
    pub label: String,
    pub node_type: NodeType,
    pub status: NodeStatus,
    pub position: Position,
    pub size: Size,
    pub metrics: NodeMetrics,
}

#[derive(Debug, Clone)]
pub struct GraphEdge {
    pub source: String,
    pub target: String,
    pub edge_type: EdgeType,
    pub message_rate: f32,
    pub latency: Duration,
    pub health: EdgeHealth,
}

impl DataflowGraphComponent {
    pub fn new() -> Self {
        Self {
            graph_data: GraphData::default(),
            layout_engine: GraphLayoutEngine::new(LayoutAlgorithm::ForceDirected),
            node_positions: HashMap::new(),
            selected_node: None,
            zoom_level: 1.0,
            pan_offset: (0.0, 0.0),
            focused: false,
        }
    }
    
    fn render_graph_nodes(&self, frame: &mut Frame, area: Rect, theme: &Theme) {
        for node in &self.graph_data.nodes {
            let node_area = self.calculate_node_area(&node, area);
            self.render_single_node(frame, node_area, node, theme);
        }
    }
    
    fn render_single_node(&self, frame: &mut Frame, area: Rect, node: &GraphNode, theme: &Theme) {
        let is_selected = self.selected_node.as_ref() == Some(&node.id);
        
        let node_style = match (node.status, is_selected) {
            (NodeStatus::Running, true) => theme.success_style().add_modifier(ratatui::style::Modifier::BOLD),
            (NodeStatus::Running, false) => theme.success_style(),
            (NodeStatus::Failed, _) => theme.error_style(),
            (NodeStatus::Starting, _) => theme.warning_style(),
            (NodeStatus::Stopped, _) => theme.muted_style(),
            _ => theme.normal_style(),
        };
        
        let border_style = if is_selected {
            theme.focused_border_style()
        } else {
            theme.normal_border_style()
        };
        
        let block = Block::default()
            .title(node.label.clone())
            .borders(Borders::ALL)
            .border_style(border_style);
        
        frame.render_widget(block.clone(), area);
        
        // Render node content
        let inner_area = block.inner(area);
        self.render_node_content(frame, inner_area, node, theme);
    }
    
    fn render_node_content(&self, frame: &mut Frame, area: Rect, node: &GraphNode, theme: &Theme) {
        let content = vec![
            format!("Type: {}", node.node_type),
            format!("CPU: {:.1}%", node.metrics.cpu_usage.unwrap_or(0.0)),
            format!("Mem: {}MB", node.metrics.memory_usage_mb.unwrap_or(0)),
            format!("Rate: {:.1}/s", node.metrics.message_rate.unwrap_or(0.0)),
        ];
        
        let paragraph = Paragraph::new(content.join("\n"))
            .style(theme.normal_style());
        
        frame.render_widget(paragraph, area);
    }
    
    fn render_graph_edges(&self, frame: &mut Frame, area: Rect, theme: &Theme) {
        for edge in &self.graph_data.edges {
            self.render_single_edge(frame, area, edge, theme);
        }
    }
    
    fn render_single_edge(&self, frame: &mut Frame, area: Rect, edge: &GraphEdge, theme: &Theme) {
        // Get source and target node positions
        let source_pos = self.node_positions.get(&edge.source);
        let target_pos = self.node_positions.get(&edge.target);
        
        if let (Some(source), Some(target)) = (source_pos, target_pos) {
            // Calculate edge path
            let edge_path = self.calculate_edge_path(source, target, area);
            
            // Determine edge style based on health and traffic
            let edge_style = match edge.health {
                EdgeHealth::Healthy => theme.success_style(),
                EdgeHealth::Degraded => theme.warning_style(),
                EdgeHealth::Failed => theme.error_style(),
            };
            
            // Render edge line (simplified ASCII representation)
            self.render_edge_line(frame, &edge_path, edge_style);
            
            // Render message rate indicator
            if edge.message_rate > 0.0 {
                self.render_message_rate_indicator(frame, &edge_path, edge.message_rate, theme);
            }
        }
    }
    
    fn calculate_node_area(&self, node: &GraphNode, container: Rect) -> Rect {
        let pos = &node.position;
        let size = &node.size;
        
        // Apply zoom and pan transformations
        let scaled_x = ((pos.x + self.pan_offset.0) * self.zoom_level) as u16;
        let scaled_y = ((pos.y + self.pan_offset.1) * self.zoom_level) as u16;
        let scaled_width = (size.width * self.zoom_level) as u16;
        let scaled_height = (size.height * self.zoom_level) as u16;
        
        Rect {
            x: container.x + scaled_x.min(container.width),
            y: container.y + scaled_y.min(container.height),
            width: scaled_width.min(container.width - scaled_x),
            height: scaled_height.min(container.height - scaled_y),
        }
    }
}

#[async_trait]
impl Component for DataflowGraphComponent {
    async fn update(&mut self, context: &AppContext) -> io::Result<()> {
        // Update graph layout if nodes have changed
        if self.should_recalculate_layout() {
            self.layout_engine.calculate_layout(&mut self.graph_data);
            self.update_node_positions();
        }
        
        Ok(())
    }
    
    fn render(&self, frame: &mut Frame, area: Rect, theme: &Theme, context: &AppContext) {
        let block = Block::default()
            .title("Dataflow Graph")
            .borders(Borders::ALL)
            .border_style(if self.focused {
                theme.focused_border_style()
            } else {
                theme.normal_border_style()
            });
        
        let inner_area = block.inner(area);
        frame.render_widget(block, area);
        
        // Render graph content
        self.render_graph_edges(frame, inner_area, theme);
        self.render_graph_nodes(frame, inner_area, theme);
        
        // Render zoom and pan indicators
        self.render_navigation_indicators(frame, area, theme);
    }
    
    async fn handle_event(&mut self, event: ComponentEvent, context: &AppContext) -> io::Result<ViewEventResult> {
        if !self.focused {
            return Ok(ViewEventResult::None);
        }
        
        match event {
            ComponentEvent::Key(key_event) => {
                match key_event.code {
                    crossterm::event::KeyCode::Up => {
                        self.pan_offset.1 += 1.0;
                    },
                    crossterm::event::KeyCode::Down => {
                        self.pan_offset.1 -= 1.0;
                    },
                    crossterm::event::KeyCode::Left => {
                        self.pan_offset.0 += 1.0;
                    },
                    crossterm::event::KeyCode::Right => {
                        self.pan_offset.0 -= 1.0;
                    },
                    crossterm::event::KeyCode::Char('+') => {
                        self.zoom_level = (self.zoom_level * 1.1).min(3.0);
                    },
                    crossterm::event::KeyCode::Char('-') => {
                        self.zoom_level = (self.zoom_level * 0.9).max(0.1);
                    },
                    crossterm::event::KeyCode::Char('0') => {
                        self.zoom_level = 1.0;
                        self.pan_offset = (0.0, 0.0);
                    },
                    crossterm::event::KeyCode::Enter => {
                        if let Some(node_id) = &self.selected_node {
                            return Ok(ViewEventResult::PushView(ViewType::NodeInspector {
                                dataflow_name: "".to_string(), // Will be filled by parent
                                node_name: node_id.clone(),
                            }));
                        }
                    },
                    _ => {}
                }
            },
            
            ComponentEvent::Mouse(mouse_event) => {
                // Handle node selection and dragging
                if let crossterm::event::MouseEventKind::Down(crossterm::event::MouseButton::Left) = mouse_event.kind {
                    self.handle_node_selection(mouse_event.column, mouse_event.row);
                }
            },
            
            _ => {}
        }
        
        Ok(ViewEventResult::None)
    }
    
    fn component_type(&self) -> ComponentType {
        ComponentType::DataflowGraph
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

#### 3. Node Inspector Component
```rust
// src/tui/components/dataflow/node_inspector.rs
pub struct NodeInspectorComponent {
    selected_node: Option<NodeInfo>,
    inspector_tabs: Vec<InspectorTab>,
    current_tab: usize,
    node_metrics_history: VecDeque<NodeMetrics>,
    focused: bool,
}

#[derive(Debug, Clone)]
pub enum InspectorTab {
    Overview,
    Configuration,
    Metrics,
    Logs,
    Dependencies,
}

impl NodeInspectorComponent {
    pub fn new() -> Self {
        Self {
            selected_node: None,
            inspector_tabs: vec![
                InspectorTab::Overview,
                InspectorTab::Configuration,
                InspectorTab::Metrics,
                InspectorTab::Dependencies,
            ],
            current_tab: 0,
            node_metrics_history: VecDeque::with_capacity(100),
            focused: false,
        }
    }
    
    fn render_overview_tab(&self, frame: &mut Frame, area: Rect, theme: &Theme) {
        if let Some(node) = &self.selected_node {
            let chunks = Layout::default()
                .direction(Direction::Vertical)
                .constraints([
                    Constraint::Length(8),
                    Constraint::Min(4),
                ].as_ref())
                .split(area);
            
            // Basic info
            self.render_node_basic_info(frame, chunks[0], node, theme);
            
            // Current status and metrics
            self.render_node_status(frame, chunks[1], node, theme);
        }
    }
    
    fn render_node_basic_info(&self, frame: &mut Frame, area: Rect, node: &NodeInfo, theme: &Theme) {
        let info_text = vec![
            format!("Name: {}", node.name),
            format!("Type: {}", node.node_type),
            format!("Status: {}", node.status),
            format!("Uptime: {}", format_duration(node.uptime.unwrap_or(Duration::ZERO))),
            format!("PID: {}", node.process_id.unwrap_or(0)),
            format!("Version: {}", node.version.as_deref().unwrap_or("N/A")),
        ];
        
        let paragraph = Paragraph::new(info_text.join("\n"))
            .block(Block::default().title("Node Information").borders(Borders::ALL))
            .style(theme.normal_style());
        
        frame.render_widget(paragraph, area);
    }
    
    fn render_node_status(&self, frame: &mut Frame, area: Rect, node: &NodeInfo, theme: &Theme) {
        let chunks = Layout::default()
            .direction(Direction::Horizontal)
            .constraints([
                Constraint::Percentage(50),
                Constraint::Percentage(50),
            ].as_ref())
            .split(area);
        
        // Resource usage
        self.render_resource_usage(frame, chunks[0], node, theme);
        
        // Message processing stats
        self.render_message_stats(frame, chunks[1], node, theme);
    }
    
    fn render_resource_usage(&self, frame: &mut Frame, area: Rect, node: &NodeInfo, theme: &Theme) {
        let usage_text = vec![
            format!("CPU: {:.1}%", node.cpu_usage.unwrap_or(0.0)),
            format!("Memory: {}MB", node.memory_usage_mb.unwrap_or(0)),
            format!("Network In: {}/s", format_bytes(node.network_bytes_in.unwrap_or(0))),
            format!("Network Out: {}/s", format_bytes(node.network_bytes_out.unwrap_or(0))),
        ];
        
        let paragraph = Paragraph::new(usage_text.join("\n"))
            .block(Block::default().title("Resource Usage").borders(Borders::ALL))
            .style(theme.normal_style());
        
        frame.render_widget(paragraph, area);
    }
    
    fn render_message_stats(&self, frame: &mut Frame, area: Rect, node: &NodeInfo, theme: &Theme) {
        let stats_text = vec![
            format!("Messages/sec: {:.1}", node.message_rate.unwrap_or(0.0)),
            format!("Total Processed: {}", node.total_messages.unwrap_or(0)),
            format!("Errors: {}", node.error_count.unwrap_or(0)),
            format!("Last Activity: {}", 
                node.last_activity.map(|t| format_timestamp(t))
                    .unwrap_or_else(|| "N/A".to_string())
            ),
        ];
        
        let paragraph = Paragraph::new(stats_text.join("\n"))
            .block(Block::default().title("Message Processing").borders(Borders::ALL))
            .style(theme.normal_style());
        
        frame.render_widget(paragraph, area);
    }
    
    fn render_metrics_tab(&self, frame: &mut Frame, area: Rect, theme: &Theme) {
        // Render historical metrics as charts
        if !self.node_metrics_history.is_empty() {
            let chunks = Layout::default()
                .direction(Direction::Vertical)
                .constraints([
                    Constraint::Percentage(50),
                    Constraint::Percentage(50),
                ].as_ref())
                .split(area);
            
            // CPU usage chart
            self.render_cpu_chart(frame, chunks[0], theme);
            
            // Memory usage chart
            self.render_memory_chart(frame, chunks[1], theme);
        }
    }
}

#[async_trait]
impl Component for NodeInspectorComponent {
    async fn update(&mut self, context: &AppContext) -> io::Result<()> {
        // Update metrics history
        if let Some(node) = &self.selected_node {
            if let Some(cpu_usage) = node.cpu_usage {
                let metrics = NodeMetrics {
                    timestamp: Utc::now(),
                    cpu_usage: Some(cpu_usage),
                    memory_usage_mb: node.memory_usage_mb,
                    message_rate: node.message_rate,
                    error_count: node.error_count,
                };
                
                self.node_metrics_history.push_back(metrics);
                
                // Limit history size
                while self.node_metrics_history.len() > 100 {
                    self.node_metrics_history.pop_front();
                }
            }
        }
        
        Ok(())
    }
    
    fn render(&self, frame: &mut Frame, area: Rect, theme: &Theme, context: &AppContext) {
        let block = Block::default()
            .title("Node Inspector")
            .borders(Borders::ALL)
            .border_style(if self.focused {
                theme.focused_border_style()
            } else {
                theme.normal_border_style()
            });
        
        let inner_area = block.inner(area);
        frame.render_widget(block, area);
        
        if self.selected_node.is_some() {
            let chunks = Layout::default()
                .direction(Direction::Vertical)
                .constraints([
                    Constraint::Length(3),
                    Constraint::Min(4),
                ].as_ref())
                .split(inner_area);
            
            // Render tabs
            self.render_inspector_tabs(frame, chunks[0], theme);
            
            // Render current tab content
            match self.inspector_tabs[self.current_tab] {
                InspectorTab::Overview => self.render_overview_tab(frame, chunks[1], theme),
                InspectorTab::Metrics => self.render_metrics_tab(frame, chunks[1], theme),
                InspectorTab::Configuration => self.render_configuration_tab(frame, chunks[1], theme),
                InspectorTab::Dependencies => self.render_dependencies_tab(frame, chunks[1], theme),
                _ => {}
            }
        } else {
            let help_text = "Select a node to inspect";
            let paragraph = Paragraph::new(help_text)
                .style(theme.muted_style())
                .block(Block::default().borders(Borders::NONE));
            
            frame.render_widget(paragraph, inner_area);
        }
    }
    
    async fn handle_event(&mut self, event: ComponentEvent, context: &AppContext) -> io::Result<ViewEventResult> {
        if !self.focused {
            return Ok(ViewEventResult::None);
        }
        
        match event {
            ComponentEvent::Key(key_event) => {
                match key_event.code {
                    crossterm::event::KeyCode::Tab => {
                        self.current_tab = (self.current_tab + 1) % self.inspector_tabs.len();
                    },
                    crossterm::event::KeyCode::BackTab => {
                        self.current_tab = if self.current_tab == 0 {
                            self.inspector_tabs.len() - 1
                        } else {
                            self.current_tab - 1
                        };
                    },
                    _ => {}
                }
            },
            _ => {}
        }
        
        Ok(ViewEventResult::None)
    }
    
    fn component_type(&self) -> ComponentType {
        ComponentType::NodeInspector
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

### Why This Approach

**Comprehensive Visualization:**
- Multiple view modes for different use cases and preferences
- Interactive graph navigation with zoom and pan
- Real-time updates showing current system state

**Rich Node Inspection:**
- Detailed node metrics and configuration
- Historical performance data visualization
- Dependency mapping and relationship analysis

**Interactive Management:**
- Direct dataflow control (start/stop/restart)
- Node-level debugging and inspection
- Visual issue identification and resolution

### How to Implement

#### Step 1: Core Explorer View (5 hours)
1. **Implement DataflowExplorerView** with tab-based navigation
2. **Add real-time monitoring** and data refresh management
3. **Create dataflow lifecycle** management (start/stop/restart)
4. **Add issue detection** and alerting system

#### Step 2: Graph Visualization (6 hours)
1. **Implement DataflowGraphComponent** with interactive graph rendering
2. **Add graph layout engine** with multiple algorithm support
3. **Create node and edge** rendering with status visualization
4. **Add zoom, pan, and selection** functionality

#### Step 3: Node Inspector (4 hours)
1. **Implement NodeInspectorComponent** with tabbed interface
2. **Add metrics visualization** with historical charts
3. **Create configuration display** and editing capabilities
4. **Add dependency analysis** and relationship mapping

#### Step 4: Real-time Monitoring (3 hours)
1. **Implement RealTimeMonitoring** for live data updates
2. **Add message flow** tracing and visualization
3. **Create performance metrics** collection and display
4. **Add bottleneck detection** and highlighting

#### Step 5: Testing and Polish (2 hours)
1. **Add comprehensive unit tests** for all components
2. **Test graph rendering** performance with large dataflows
3. **Validate real-time updates** and monitoring accuracy
4. **Test interactive controls** and navigation

## 🔗 Dependencies
**Depends On:**
- Issue #023 (TUI Architecture Foundation) - Base view and component system
- Phase 2 enhanced commands for integration

**Enables:**
- Detailed dataflow management and monitoring
- Visual debugging and optimization workflows

## 🧪 Testing Requirements

### Unit Tests
```rust
#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_graph_layout_calculation() {
        let mut graph_component = DataflowGraphComponent::new();
        let test_graph_data = create_test_graph_data();
        
        graph_component.graph_data = test_graph_data;
        graph_component.layout_engine.calculate_layout(&mut graph_component.graph_data);
        
        assert!(!graph_component.node_positions.is_empty());
    }
    
    #[test]
    fn test_node_selection() {
        let mut graph_component = DataflowGraphComponent::new();
        
        graph_component.handle_node_selection(10, 5);
        
        assert!(graph_component.selected_node.is_some());
    }
    
    #[test]
    fn test_dataflow_issue_detection() {
        let explorer = DataflowExplorerView::new("test-dataflow".to_string());
        
        let issues = explorer.detect_dataflow_issues().await.unwrap();
        
        assert!(issues.iter().any(|i| matches!(i.issue_type, IssueType::NodeStalled)));
    }
}
```

## ✅ Definition of Done
- [ ] DataflowExplorerView provides comprehensive dataflow visualization and management
- [ ] Graph visualization renders nodes, edges, and relationships clearly
- [ ] Interactive navigation (zoom, pan, selection) works smoothly
- [ ] Node inspector displays detailed metrics and configuration
- [ ] Real-time monitoring updates dataflow state accurately
- [ ] Dataflow lifecycle management (start/stop/restart) functions correctly
- [ ] Issue detection identifies common problems automatically
- [ ] Performance targets met for rendering and interaction response
- [ ] Multiple visualization modes accommodate different user preferences
- [ ] Comprehensive unit tests validate all explorer functionality
- [ ] Integration tests confirm dataflow management workflows
- [ ] Manual testing validates user experience and visual clarity

This interactive dataflow explorer provides users with comprehensive visualization, monitoring, and management capabilities for their dataflows, enabling efficient troubleshooting and optimization workflows through an intuitive graphical interface.