# Issue #032: Build Advanced Data Visualization Engine

## 📋 Summary
Implement a comprehensive data visualization engine that provides real-time, interactive charts, graphs, and visual analytics for Dora dataflows, performance metrics, and system behavior. This engine serves as the foundation for advanced data exploration, trend analysis, and visual debugging capabilities across the entire Dora ecosystem.

## 🎯 Objectives
- Create flexible visualization engine supporting multiple chart types and data sources
- Implement real-time data streaming with smooth animations and transitions
- Add interactive features like zooming, panning, filtering, and drill-down capabilities
- Provide customizable dashboards with drag-and-drop layout management
- Enable data export and sharing with multiple format support

**Success Metrics:**
- Visualization rendering completes within 100ms for datasets up to 100K points
- Real-time updates maintain 60 FPS with data streams up to 1K points/second
- Interactive operations (zoom, pan, filter) respond within 50ms
- Memory usage stays under 100MB for typical visualization workloads
- User visualization creation efficiency improves by 80% over manual analysis

## 🛠️ Technical Requirements

### What to Build

#### 1. Visualization Engine Core
```rust
// src/visualization/engine.rs
use ratatui::{
    layout::Rect,
    style::{Color, Style},
    widgets::canvas::{Canvas, Context, Map, MapResolution, Rectangle},
    Frame,
};
use std::collections::{HashMap, VecDeque};
use tokio::sync::mpsc;

#[derive(Debug)]
pub struct VisualizationEngine {
    chart_registry: ChartRegistry,
    data_manager: DataManager,
    render_pipeline: RenderPipeline,
    animation_engine: AnimationEngine,
    interaction_handler: InteractionHandler,
    export_manager: ExportManager,
    theme_manager: VisualizationThemeManager,
}

#[derive(Debug, Clone)]
pub enum ChartType {
    LineChart {
        multi_series: bool,
        smooth: bool,
        fill_area: bool,
    },
    BarChart {
        orientation: BarOrientation,
        stacked: bool,
        grouped: bool,
    },
    ScatterPlot {
        bubble_size: bool,
        trend_line: bool,
        color_coding: bool,
    },
    Heatmap {
        color_scale: ColorScale,
        show_values: bool,
        clustering: bool,
    },
    TreeMap {
        hierarchical: bool,
        color_by_value: bool,
        interactive_drill: bool,
    },
    NetworkGraph {
        layout_algorithm: GraphLayoutAlgorithm,
        node_sizing: NodeSizing,
        edge_bundling: bool,
    },
    Timeline {
        multi_track: bool,
        event_markers: bool,
        zoom_levels: Vec<TimeScale>,
    },
    Gauge {
        gauge_type: GaugeType,
        threshold_bands: Vec<ThresholdBand>,
        animation: bool,
    },
    Sankey {
        flow_direction: FlowDirection,
        node_alignment: NodeAlignment,
        link_curvature: f64,
    },
    Candlestick {
        volume_overlay: bool,
        technical_indicators: Vec<TechnicalIndicator>,
        time_aggregation: TimeAggregation,
    },
}

impl VisualizationEngine {
    pub fn new() -> Self {
        Self {
            chart_registry: ChartRegistry::new(),
            data_manager: DataManager::new(),
            render_pipeline: RenderPipeline::new(),
            animation_engine: AnimationEngine::new(),
            interaction_handler: InteractionHandler::new(),
            export_manager: ExportManager::new(),
            theme_manager: VisualizationThemeManager::new(),
        }
    }
    
    pub async fn create_visualization(&mut self, config: VisualizationConfig) -> Result<VisualizationId, VisualizationError> {
        // Validate configuration
        self.validate_config(&config)?;
        
        // Register data sources
        let data_source_id = self.data_manager.register_data_source(config.data_source).await?;
        
        // Create chart instance
        let chart = self.chart_registry.create_chart(
            config.chart_type,
            config.chart_config,
            data_source_id,
        )?;
        
        // Setup real-time updates if needed
        if config.real_time {
            self.setup_real_time_updates(&chart, config.update_frequency).await?;
        }
        
        // Setup animations
        if config.animations_enabled {
            self.animation_engine.setup_chart_animations(&chart, config.animation_config)?;
        }
        
        // Register for interactions
        self.interaction_handler.register_chart(&chart, config.interaction_config)?;
        
        Ok(chart.id())
    }
    
    pub async fn update_visualization_data(&mut self, viz_id: &VisualizationId, data: DataUpdate) -> Result<(), VisualizationError> {
        let chart = self.chart_registry.get_mut(viz_id)
            .ok_or(VisualizationError::ChartNotFound)?;
        
        // Process data update
        let processed_data = self.data_manager.process_data_update(data).await?;
        
        // Apply data to chart
        chart.update_data(processed_data)?;
        
        // Trigger animation if enabled
        if chart.has_animations() {
            self.animation_engine.trigger_data_update_animation(viz_id)?;
        }
        
        Ok(())
    }
    
    pub fn render_visualization(&self, viz_id: &VisualizationId, frame: &mut Frame, area: Rect, theme: &Theme) -> Result<(), VisualizationError> {
        let chart = self.chart_registry.get(viz_id)
            .ok_or(VisualizationError::ChartNotFound)?;
        
        // Get current animation state
        let animation_state = self.animation_engine.get_animation_state(viz_id);
        
        // Create render context
        let render_context = RenderContext {
            area,
            theme: &self.theme_manager.get_chart_theme(chart.chart_type(), theme),
            animation_state,
            interaction_state: self.interaction_handler.get_interaction_state(viz_id),
        };
        
        // Render through pipeline
        self.render_pipeline.render_chart(chart, frame, render_context)?;
        
        Ok(())
    }
    
    async fn setup_real_time_updates(&mut self, chart: &Chart, frequency: Duration) -> Result<(), VisualizationError> {
        let (tx, mut rx) = mpsc::unbounded_channel();
        let chart_id = chart.id();
        
        // Spawn data update task
        tokio::spawn(async move {
            let mut interval = tokio::time::interval(frequency);
            
            loop {
                interval.tick().await;
                
                // Fetch latest data
                if let Ok(data) = chart.data_source().fetch_latest().await {
                    if tx.send((chart_id, data)).is_err() {
                        break; // Channel closed
                    }
                }
            }
        });
        
        // Store receiver for processing
        self.data_manager.register_real_time_stream(chart_id, rx);
        
        Ok(())
    }
}
```

#### 2. Advanced Chart Components
```rust
// src/visualization/charts/line_chart.rs
#[derive(Debug)]
pub struct AdvancedLineChart {
    chart_config: LineChartConfig,
    data_series: Vec<DataSeries>,
    viewport: Viewport,
    interaction_state: InteractionState,
    animation_state: AnimationState,
    performance_optimizer: PerformanceOptimizer,
}

#[derive(Debug, Clone)]
pub struct LineChartConfig {
    pub multi_series: bool,
    pub smooth_curves: bool,
    pub fill_area: bool,
    pub show_points: bool,
    pub line_thickness: u8,
    pub point_size: u8,
    pub grid_config: GridConfig,
    pub axis_config: AxisConfig,
    pub legend_config: LegendConfig,
    pub zoom_config: ZoomConfig,
}

impl AdvancedLineChart {
    pub fn new(config: LineChartConfig) -> Self {
        Self {
            chart_config: config,
            data_series: Vec::new(),
            viewport: Viewport::default(),
            interaction_state: InteractionState::default(),
            animation_state: AnimationState::default(),
            performance_optimizer: PerformanceOptimizer::new(),
        }
    }
    
    pub fn add_series(&mut self, series: DataSeries) {
        // Optimize data for rendering
        let optimized_series = self.performance_optimizer.optimize_series(series);
        
        self.data_series.push(optimized_series);
        
        // Update viewport to include new data
        self.viewport.update_bounds(&self.data_series);
    }
    
    pub fn render(&self, frame: &mut Frame, area: Rect, render_context: &RenderContext) {
        // Calculate rendering areas
        let render_areas = self.calculate_render_areas(area);
        
        // Render background and grid
        self.render_background_and_grid(frame, &render_areas, render_context);
        
        // Render axes
        self.render_axes(frame, &render_areas, render_context);
        
        // Render data series
        self.render_data_series(frame, &render_areas, render_context);
        
        // Render legend
        if self.chart_config.legend_config.visible {
            self.render_legend(frame, &render_areas, render_context);
        }
        
        // Render interaction overlays
        self.render_interaction_overlays(frame, &render_areas, render_context);
    }
    
    fn render_data_series(&self, frame: &mut Frame, areas: &RenderAreas, context: &RenderContext) {
        let canvas = Canvas::default()
            .block(Block::default())
            .x_bounds([self.viewport.x_min, self.viewport.x_max])
            .y_bounds([self.viewport.y_min, self.viewport.y_max])
            .paint(|ctx| {
                for (series_index, series) in self.data_series.iter().enumerate() {
                    self.render_series_on_canvas(ctx, series, series_index, context);
                }
            });
        
        frame.render_widget(canvas, areas.chart_area);
    }
    
    fn render_series_on_canvas(&self, ctx: &mut Context, series: &DataSeries, series_index: usize, context: &RenderContext) {
        let color = context.theme.series_colors[series_index % context.theme.series_colors.len()];
        
        // Level of detail based on zoom level
        let points = self.performance_optimizer.get_lod_points(series, &self.viewport);
        
        if self.chart_config.fill_area {
            self.render_area_fill(ctx, &points, color);
        }
        
        if self.chart_config.smooth_curves {
            self.render_smooth_line(ctx, &points, color);
        } else {
            self.render_straight_line(ctx, &points, color);
        }
        
        if self.chart_config.show_points {
            self.render_data_points(ctx, &points, color);
        }
        
        // Render animations if active
        if let Some(animation) = context.animation_state.get_series_animation(series_index) {
            self.render_animation_effects(ctx, animation, &points, color);
        }
    }
    
    fn render_smooth_line(&self, ctx: &mut Context, points: &[DataPoint], color: Color) {
        if points.len() < 2 {
            return;
        }
        
        // Generate Bézier curve control points
        let control_points = self.calculate_bezier_control_points(points);
        
        // Render smooth curves between points
        for i in 0..points.len() - 1 {
            let start = points[i];
            let end = points[i + 1];
            let (cp1, cp2) = control_points[i];
            
            // Render Bézier curve segments
            self.render_bezier_segment(ctx, start, cp1, cp2, end, color);
        }
    }
    
    fn calculate_bezier_control_points(&self, points: &[DataPoint]) -> Vec<(DataPoint, DataPoint)> {
        // Implement Catmull-Rom spline or similar smooth curve algorithm
        let mut control_points = Vec::new();
        
        for i in 0..points.len() - 1 {
            let p0 = if i == 0 { points[0] } else { points[i - 1] };
            let p1 = points[i];
            let p2 = points[i + 1];
            let p3 = if i + 2 < points.len() { points[i + 2] } else { points[i + 1] };
            
            // Calculate control points using Catmull-Rom spline
            let tension = 0.5;
            let cp1 = DataPoint {
                x: p1.x + (p2.x - p0.x) * tension / 6.0,
                y: p1.y + (p2.y - p0.y) * tension / 6.0,
            };
            let cp2 = DataPoint {
                x: p2.x - (p3.x - p1.x) * tension / 6.0,
                y: p2.y - (p3.y - p1.y) * tension / 6.0,
            };
            
            control_points.push((cp1, cp2));
        }
        
        control_points
    }
    
    fn render_bezier_segment(&self, ctx: &mut Context, start: DataPoint, cp1: DataPoint, cp2: DataPoint, end: DataPoint, color: Color) {
        // Render Bézier curve using line segments
        let segments = 20; // Number of line segments to approximate curve
        let mut last_point = start;
        
        for i in 1..=segments {
            let t = i as f64 / segments as f64;
            let point = self.evaluate_bezier(t, start, cp1, cp2, end);
            
            ctx.draw(&ratatui::widgets::canvas::Line {
                x1: last_point.x,
                y1: last_point.y,
                x2: point.x,
                y2: point.y,
                color,
            });
            
            last_point = point;
        }
    }
    
    fn evaluate_bezier(&self, t: f64, p0: DataPoint, p1: DataPoint, p2: DataPoint, p3: DataPoint) -> DataPoint {
        let u = 1.0 - t;
        let tt = t * t;
        let uu = u * u;
        let uuu = uu * u;
        let ttt = tt * t;
        
        DataPoint {
            x: uuu * p0.x + 3.0 * uu * t * p1.x + 3.0 * u * tt * p2.x + ttt * p3.x,
            y: uuu * p0.y + 3.0 * uu * t * p1.y + 3.0 * u * tt * p2.y + ttt * p3.y,
        }
    }
}

// src/visualization/charts/network_graph.rs
#[derive(Debug)]
pub struct NetworkGraphChart {
    nodes: Vec<GraphNode>,
    edges: Vec<GraphEdge>,
    layout_engine: GraphLayoutEngine,
    physics_simulation: PhysicsSimulation,
    interaction_state: GraphInteractionState,
    visual_config: NetworkVisualConfig,
}

impl NetworkGraphChart {
    pub fn new(config: NetworkGraphConfig) -> Self {
        Self {
            nodes: Vec::new(),
            edges: Vec::new(),
            layout_engine: GraphLayoutEngine::new(config.layout_algorithm),
            physics_simulation: PhysicsSimulation::new(),
            interaction_state: GraphInteractionState::default(),
            visual_config: config.visual_config,
        }
    }
    
    pub fn add_node(&mut self, node: GraphNode) {
        self.nodes.push(node);
        self.layout_engine.add_node_to_layout(node.id);
    }
    
    pub fn add_edge(&mut self, edge: GraphEdge) {
        self.edges.push(edge);
        self.layout_engine.add_edge_to_layout(edge.source, edge.target, edge.weight);
    }
    
    pub fn update_layout(&mut self) {
        // Run physics simulation step
        self.physics_simulation.step(&mut self.nodes, &self.edges);
        
        // Apply layout algorithm
        self.layout_engine.update_positions(&mut self.nodes);
    }
    
    pub fn render(&self, frame: &mut Frame, area: Rect, render_context: &RenderContext) {
        let canvas = Canvas::default()
            .block(Block::default())
            .x_bounds([0.0, area.width as f64])
            .y_bounds([0.0, area.height as f64])
            .paint(|ctx| {
                // Render edges first
                self.render_edges(ctx, render_context);
                
                // Render nodes on top
                self.render_nodes(ctx, render_context);
                
                // Render labels if enabled
                if self.visual_config.show_labels {
                    self.render_labels(ctx, render_context);
                }
                
                // Render selection and hover effects
                self.render_interaction_effects(ctx, render_context);
            });
        
        frame.render_widget(canvas, area);
    }
    
    fn render_edges(&self, ctx: &mut Context, context: &RenderContext) {
        for edge in &self.edges {
            let source_node = &self.nodes[edge.source];
            let target_node = &self.nodes[edge.target];
            
            let color = if edge.highlighted {
                context.theme.edge_highlight_color
            } else {
                context.theme.edge_color
            };
            
            if self.visual_config.edge_bundling {
                self.render_bundled_edge(ctx, source_node, target_node, edge, color);
            } else {
                self.render_straight_edge(ctx, source_node, target_node, edge, color);
            }
        }
    }
    
    fn render_nodes(&self, ctx: &mut Context, context: &RenderContext) {
        for node in &self.nodes {
            let size = match self.visual_config.node_sizing {
                NodeSizing::Fixed(size) => size,
                NodeSizing::ByValue => (node.value * 10.0).max(5.0).min(20.0),
                NodeSizing::ByDegree => (node.degree as f64 * 2.0).max(5.0).min(15.0),
            };
            
            let color = if node.selected {
                context.theme.node_selected_color
            } else if node.hovered {
                context.theme.node_hover_color
            } else {
                node.color.unwrap_or(context.theme.node_default_color)
            };
            
            // Render node as circle
            ctx.draw(&ratatui::widgets::canvas::Circle {
                x: node.position.x,
                y: node.position.y,
                radius: size,
                color,
            });
            
            // Render node border if selected
            if node.selected {
                ctx.draw(&ratatui::widgets::canvas::Circle {
                    x: node.position.x,
                    y: node.position.y,
                    radius: size + 1.0,
                    color: context.theme.node_selected_border,
                });
            }
        }
    }
}
```

#### 3. Interactive Dashboard Builder
```rust
// src/visualization/dashboard.rs
#[derive(Debug)]
pub struct InteractiveDashboard {
    dashboard_config: DashboardConfig,
    layout_manager: DashboardLayoutManager,
    widget_registry: WidgetRegistry,
    data_pipeline: DataPipeline,
    interaction_manager: DashboardInteractionManager,
    export_manager: DashboardExportManager,
}

#[derive(Debug, Clone)]
pub struct DashboardConfig {
    pub name: String,
    pub description: String,
    pub layout: DashboardLayout,
    pub refresh_interval: Duration,
    pub auto_layout: bool,
    pub responsive: bool,
    pub theme: String,
}

#[derive(Debug, Clone)]
pub enum DashboardLayout {
    Grid {
        rows: usize,
        columns: usize,
        gap: u16,
    },
    Flexible {
        regions: Vec<DashboardRegion>,
    },
    Tabbed {
        tabs: Vec<DashboardTab>,
    },
    Masonry {
        column_width: u16,
        min_height: u16,
    },
}

impl InteractiveDashboard {
    pub fn new(config: DashboardConfig) -> Self {
        Self {
            dashboard_config: config.clone(),
            layout_manager: DashboardLayoutManager::new(config.layout),
            widget_registry: WidgetRegistry::new(),
            data_pipeline: DataPipeline::new(),
            interaction_manager: DashboardInteractionManager::new(),
            export_manager: DashboardExportManager::new(),
        }
    }
    
    pub async fn add_widget(&mut self, widget_config: WidgetConfig) -> Result<WidgetId, DashboardError> {
        // Create widget instance
        let widget = self.widget_registry.create_widget(widget_config.widget_type, widget_config.config)?;
        
        // Setup data connections
        for data_source in &widget_config.data_sources {
            self.data_pipeline.connect_widget_to_source(widget.id(), data_source).await?;
        }
        
        // Add to layout
        self.layout_manager.add_widget(widget.id(), widget_config.layout_hint)?;
        
        // Setup interactions
        self.interaction_manager.register_widget(widget.id(), widget_config.interaction_config)?;
        
        Ok(widget.id())
    }
    
    pub async fn update_dashboard(&mut self) -> Result<(), DashboardError> {
        // Update data pipeline
        self.data_pipeline.process_updates().await?;
        
        // Update all widgets
        for widget in self.widget_registry.widgets_mut() {
            if let Some(data) = self.data_pipeline.get_widget_data(widget.id()) {
                widget.update_data(data)?;
            }
        }
        
        // Update layout if auto-layout is enabled
        if self.dashboard_config.auto_layout {
            self.layout_manager.optimize_layout()?;
        }
        
        Ok(())
    }
    
    pub fn render(&self, frame: &mut Frame, area: Rect, theme: &Theme) {
        // Calculate widget positions
        let widget_areas = self.layout_manager.calculate_widget_areas(area);
        
        // Render each widget
        for (widget_id, widget_area) in widget_areas {
            if let Some(widget) = self.widget_registry.get_widget(&widget_id) {
                widget.render(frame, widget_area, theme);
            }
        }
        
        // Render dashboard chrome (title, controls, etc.)
        self.render_dashboard_chrome(frame, area, theme);
        
        // Render interaction overlays
        self.interaction_manager.render_overlays(frame, area, theme);
    }
    
    pub async fn handle_interaction(&mut self, interaction: DashboardInteraction) -> Result<(), DashboardError> {
        match interaction {
            DashboardInteraction::WidgetDrag { widget_id, new_position } => {
                self.layout_manager.move_widget(widget_id, new_position)?;
            },
            
            DashboardInteraction::WidgetResize { widget_id, new_size } => {
                self.layout_manager.resize_widget(widget_id, new_size)?;
            },
            
            DashboardInteraction::AddWidget { widget_type, position } => {
                let config = WidgetConfig::default_for_type(widget_type);
                self.add_widget(config).await?;
            },
            
            DashboardInteraction::RemoveWidget { widget_id } => {
                self.remove_widget(widget_id).await?;
            },
            
            DashboardInteraction::ConfigureWidget { widget_id, config } => {
                self.configure_widget(widget_id, config)?;
            },
            
            DashboardInteraction::ExportDashboard { format } => {
                self.export_dashboard(format).await?;
            },
        }
        
        Ok(())
    }
    
    async fn remove_widget(&mut self, widget_id: WidgetId) -> Result<(), DashboardError> {
        // Disconnect data sources
        self.data_pipeline.disconnect_widget(widget_id).await?;
        
        // Remove from layout
        self.layout_manager.remove_widget(widget_id)?;
        
        // Unregister interactions
        self.interaction_manager.unregister_widget(widget_id)?;
        
        // Remove from registry
        self.widget_registry.remove_widget(widget_id)?;
        
        Ok(())
    }
    
    async fn export_dashboard(&self, format: ExportFormat) -> Result<(), DashboardError> {
        match format {
            ExportFormat::Json => {
                let config = self.to_dashboard_config();
                self.export_manager.export_json(&config).await?;
            },
            
            ExportFormat::Html => {
                let html = self.export_manager.generate_html_export(self).await?;
                self.export_manager.save_html(&html).await?;
            },
            
            ExportFormat::Pdf => {
                let pdf_data = self.export_manager.generate_pdf_export(self).await?;
                self.export_manager.save_pdf(&pdf_data).await?;
            },
            
            ExportFormat::Image => {
                let image_data = self.export_manager.capture_dashboard_image(self).await?;
                self.export_manager.save_image(&image_data).await?;
            },
        }
        
        Ok(())
    }
}

#[derive(Debug)]
pub struct DashboardLayoutManager {
    layout_type: DashboardLayout,
    widget_positions: HashMap<WidgetId, WidgetPosition>,
    layout_constraints: LayoutConstraints,
    auto_layout_engine: AutoLayoutEngine,
}

impl DashboardLayoutManager {
    pub fn new(layout_type: DashboardLayout) -> Self {
        Self {
            layout_type,
            widget_positions: HashMap::new(),
            layout_constraints: LayoutConstraints::default(),
            auto_layout_engine: AutoLayoutEngine::new(),
        }
    }
    
    pub fn add_widget(&mut self, widget_id: WidgetId, hint: LayoutHint) -> Result<(), DashboardError> {
        let position = match &hint {
            LayoutHint::Specific(pos) => pos.clone(),
            LayoutHint::Auto => self.auto_layout_engine.find_best_position(&self.widget_positions)?,
            LayoutHint::Relative { relative_to, direction } => {
                self.calculate_relative_position(*relative_to, *direction)?
            },
        };
        
        self.widget_positions.insert(widget_id, position);
        Ok(())
    }
    
    pub fn calculate_widget_areas(&self, total_area: Rect) -> HashMap<WidgetId, Rect> {
        match &self.layout_type {
            DashboardLayout::Grid { rows, columns, gap } => {
                self.calculate_grid_layout(total_area, *rows, *columns, *gap)
            },
            
            DashboardLayout::Flexible { regions } => {
                self.calculate_flexible_layout(total_area, regions)
            },
            
            DashboardLayout::Tabbed { tabs } => {
                self.calculate_tabbed_layout(total_area, tabs)
            },
            
            DashboardLayout::Masonry { column_width, min_height } => {
                self.calculate_masonry_layout(total_area, *column_width, *min_height)
            },
        }
    }
    
    fn calculate_grid_layout(&self, area: Rect, rows: usize, columns: usize, gap: u16) -> HashMap<WidgetId, Rect> {
        let mut widget_areas = HashMap::new();
        
        let cell_width = (area.width - gap * (columns as u16 - 1)) / columns as u16;
        let cell_height = (area.height - gap * (rows as u16 - 1)) / rows as u16;
        
        for (widget_id, position) in &self.widget_positions {
            let x = area.x + (cell_width + gap) * position.grid_x as u16;
            let y = area.y + (cell_height + gap) * position.grid_y as u16;
            let width = cell_width * position.span_x as u16 + gap * (position.span_x as u16 - 1);
            let height = cell_height * position.span_y as u16 + gap * (position.span_y as u16 - 1);
            
            widget_areas.insert(*widget_id, Rect::new(x, y, width, height));
        }
        
        widget_areas
    }
    
    pub fn optimize_layout(&mut self) -> Result<(), DashboardError> {
        self.auto_layout_engine.optimize_positions(&mut self.widget_positions, &self.layout_constraints)?;
        Ok(())
    }
}
```

#### 4. Real-Time Data Pipeline
```rust
// src/visualization/data_pipeline.rs
#[derive(Debug)]
pub struct DataPipeline {
    data_sources: HashMap<DataSourceId, Box<dyn DataSource>>,
    data_processors: HashMap<ProcessorId, Box<dyn DataProcessor>>,
    streaming_connections: HashMap<WidgetId, StreamingConnection>,
    data_cache: DataCache,
    performance_monitor: DataPipelineMonitor,
}

#[derive(Debug)]
pub struct StreamingConnection {
    source_id: DataSourceId,
    processor_chain: Vec<ProcessorId>,
    buffer: CircularBuffer<DataPoint>,
    last_update: Instant,
    update_frequency: Duration,
    error_handler: ErrorHandler,
}

impl DataPipeline {
    pub fn new() -> Self {
        Self {
            data_sources: HashMap::new(),
            data_processors: HashMap::new(),
            streaming_connections: HashMap::new(),
            data_cache: DataCache::new(),
            performance_monitor: DataPipelineMonitor::new(),
        }
    }
    
    pub async fn register_data_source(&mut self, source: Box<dyn DataSource>) -> DataSourceId {
        let source_id = source.id();
        self.data_sources.insert(source_id, source);
        source_id
    }
    
    pub async fn connect_widget_to_source(&mut self, widget_id: WidgetId, source_config: &DataSourceConfig) -> Result<(), PipelineError> {
        let source_id = self.find_or_create_data_source(source_config).await?;
        
        // Create processor chain
        let processor_chain = self.create_processor_chain(&source_config.transformations)?;
        
        // Setup streaming connection
        let connection = StreamingConnection {
            source_id,
            processor_chain,
            buffer: CircularBuffer::new(source_config.buffer_size),
            last_update: Instant::now(),
            update_frequency: source_config.update_frequency,
            error_handler: ErrorHandler::new(),
        };
        
        self.streaming_connections.insert(widget_id, connection);
        
        // Start streaming if source supports it
        if let Some(source) = self.data_sources.get_mut(&source_id) {
            if source.supports_streaming() {
                source.start_streaming(widget_id).await?;
            }
        }
        
        Ok(())
    }
    
    pub async fn process_updates(&mut self) -> Result<(), PipelineError> {
        let mut update_tasks = Vec::new();
        
        // Process each streaming connection
        for (widget_id, connection) in &mut self.streaming_connections {
            if connection.last_update.elapsed() >= connection.update_frequency {
                update_tasks.push(self.process_connection_update(*widget_id, connection));
            }
        }
        
        // Process all updates concurrently
        let results = futures::future::join_all(update_tasks).await;
        
        // Handle any errors
        for result in results {
            if let Err(error) = result {
                self.performance_monitor.record_error(error);
            }
        }
        
        Ok(())
    }
    
    async fn process_connection_update(&mut self, widget_id: WidgetId, connection: &mut StreamingConnection) -> Result<(), PipelineError> {
        // Fetch new data from source
        let source = self.data_sources.get_mut(&connection.source_id)
            .ok_or(PipelineError::SourceNotFound)?;
        
        let raw_data = source.fetch_latest_data().await?;
        
        // Process data through transformation chain
        let mut processed_data = raw_data;
        for processor_id in &connection.processor_chain {
            if let Some(processor) = self.data_processors.get(processor_id) {
                processed_data = processor.process(processed_data).await?;
            }
        }
        
        // Add to buffer
        connection.buffer.extend(processed_data);
        connection.last_update = Instant::now();
        
        // Update cache
        self.data_cache.update_widget_data(widget_id, connection.buffer.get_latest(1000));
        
        // Record performance metrics
        self.performance_monitor.record_update(widget_id, connection.last_update.elapsed());
        
        Ok(())
    }
    
    pub fn get_widget_data(&self, widget_id: WidgetId) -> Option<&[DataPoint]> {
        self.data_cache.get_widget_data(widget_id)
    }
    
    async fn find_or_create_data_source(&mut self, config: &DataSourceConfig) -> Result<DataSourceId, PipelineError> {
        // Check if compatible source already exists
        for (source_id, source) in &self.data_sources {
            if source.is_compatible_with(config) {
                return Ok(*source_id);
            }
        }
        
        // Create new data source
        let source = self.create_data_source(config).await?;
        let source_id = source.id();
        self.data_sources.insert(source_id, source);
        
        Ok(source_id)
    }
    
    async fn create_data_source(&self, config: &DataSourceConfig) -> Result<Box<dyn DataSource>, PipelineError> {
        match &config.source_type {
            DataSourceType::DoraMetrics { dataflow, node } => {
                Ok(Box::new(DoraMetricsSource::new(dataflow.clone(), node.clone()).await?))
            },
            
            DataSourceType::SystemMetrics { metrics } => {
                Ok(Box::new(SystemMetricsSource::new(metrics.clone()).await?))
            },
            
            DataSourceType::LogStream { log_target, filters } => {
                Ok(Box::new(LogStreamSource::new(log_target.clone(), filters.clone()).await?))
            },
            
            DataSourceType::DatabaseQuery { connection, query } => {
                Ok(Box::new(DatabaseSource::new(connection.clone(), query.clone()).await?))
            },
            
            DataSourceType::ApiEndpoint { url, auth, headers } => {
                Ok(Box::new(ApiEndpointSource::new(url.clone(), auth.clone(), headers.clone()).await?))
            },
            
            DataSourceType::FileWatch { path, format } => {
                Ok(Box::new(FileWatchSource::new(path.clone(), *format).await?))
            },
        }
    }
}

#[async_trait]
pub trait DataSource: Send + Sync + std::fmt::Debug {
    fn id(&self) -> DataSourceId;
    fn supports_streaming(&self) -> bool;
    fn is_compatible_with(&self, config: &DataSourceConfig) -> bool;
    
    async fn fetch_latest_data(&mut self) -> Result<Vec<DataPoint>, PipelineError>;
    async fn start_streaming(&mut self, widget_id: WidgetId) -> Result<(), PipelineError>;
    async fn stop_streaming(&mut self, widget_id: WidgetId) -> Result<(), PipelineError>;
    
    async fn get_historical_data(&self, range: TimeRange) -> Result<Vec<DataPoint>, PipelineError>;
    async fn get_schema(&self) -> Result<DataSchema, PipelineError>;
}

// Implementation for Dora-specific metrics source
#[derive(Debug)]
pub struct DoraMetricsSource {
    dataflow: Option<String>,
    node: Option<String>,
    daemon_client: DaemonClient,
    streaming_widgets: HashSet<WidgetId>,
    last_timestamp: Option<DateTime<Utc>>,
}

impl DoraMetricsSource {
    pub async fn new(dataflow: Option<String>, node: Option<String>) -> Result<Self, PipelineError> {
        let daemon_client = DaemonClient::connect().await
            .map_err(|e| PipelineError::ConnectionFailed(e.to_string()))?;
        
        Ok(Self {
            dataflow,
            node,
            daemon_client,
            streaming_widgets: HashSet::new(),
            last_timestamp: None,
        })
    }
}

#[async_trait]
impl DataSource for DoraMetricsSource {
    fn id(&self) -> DataSourceId {
        DataSourceId(format!("dora_metrics_{}_{}", 
                            self.dataflow.as_deref().unwrap_or("all"),
                            self.node.as_deref().unwrap_or("all")))
    }
    
    fn supports_streaming(&self) -> bool {
        true
    }
    
    fn is_compatible_with(&self, config: &DataSourceConfig) -> bool {
        matches!(&config.source_type, DataSourceType::DoraMetrics { dataflow, node } 
                if dataflow == &self.dataflow && node == &self.node)
    }
    
    async fn fetch_latest_data(&mut self) -> Result<Vec<DataPoint>, PipelineError> {
        let metrics = match (&self.dataflow, &self.node) {
            (Some(dataflow), Some(node)) => {
                self.daemon_client.get_node_metrics(dataflow, node).await
                    .map_err(|e| PipelineError::DataFetchFailed(e.to_string()))?
            },
            (Some(dataflow), None) => {
                self.daemon_client.get_dataflow_metrics(dataflow).await
                    .map_err(|e| PipelineError::DataFetchFailed(e.to_string()))?
            },
            (None, None) => {
                self.daemon_client.get_system_metrics().await
                    .map_err(|e| PipelineError::DataFetchFailed(e.to_string()))?
            },
            _ => return Err(PipelineError::InvalidConfiguration("Invalid dataflow/node combination".to_string())),
        };
        
        // Convert metrics to data points
        let data_points = self.convert_metrics_to_data_points(metrics);
        
        // Update last timestamp
        if let Some(last_point) = data_points.last() {
            self.last_timestamp = Some(last_point.timestamp);
        }
        
        Ok(data_points)
    }
    
    async fn start_streaming(&mut self, widget_id: WidgetId) -> Result<(), PipelineError> {
        self.streaming_widgets.insert(widget_id);
        
        // If this is the first streaming widget, start the metrics stream
        if self.streaming_widgets.len() == 1 {
            // Implementation would start a background task that polls for metrics
            // and pushes updates to connected widgets
        }
        
        Ok(())
    }
    
    async fn stop_streaming(&mut self, widget_id: WidgetId) -> Result<(), PipelineError> {
        self.streaming_widgets.remove(&widget_id);
        
        // If no more streaming widgets, stop the metrics stream
        if self.streaming_widgets.is_empty() {
            // Stop background polling task
        }
        
        Ok(())
    }
    
    async fn get_historical_data(&self, range: TimeRange) -> Result<Vec<DataPoint>, PipelineError> {
        // Fetch historical metrics from daemon
        let historical_metrics = self.daemon_client.get_historical_metrics(
            self.dataflow.as_deref(),
            self.node.as_deref(),
            range.start,
            range.end
        ).await.map_err(|e| PipelineError::DataFetchFailed(e.to_string()))?;
        
        Ok(self.convert_metrics_to_data_points(historical_metrics))
    }
    
    async fn get_schema(&self) -> Result<DataSchema, PipelineError> {
        // Return schema describing the structure of Dora metrics data
        Ok(DataSchema {
            fields: vec![
                DataField { name: "timestamp".to_string(), data_type: DataType::DateTime },
                DataField { name: "cpu_usage".to_string(), data_type: DataType::Float },
                DataField { name: "memory_usage".to_string(), data_type: DataType::Float },
                DataField { name: "message_count".to_string(), data_type: DataType::Integer },
                DataField { name: "processing_latency".to_string(), data_type: DataType::Float },
                DataField { name: "throughput".to_string(), data_type: DataType::Float },
            ],
        })
    }
}
```

### Why This Approach

**Comprehensive Visualization Engine:**
- Supports multiple chart types with advanced features
- Real-time data streaming with smooth animations
- Interactive capabilities for data exploration

**Performance Optimized:**
- Level-of-detail rendering for large datasets
- Efficient data pipelines with caching
- 60 FPS target with smooth animations

**Highly Customizable:**
- Flexible dashboard builder with drag-and-drop
- Extensible chart system for custom visualizations
- Theme system for consistent visual design

### How to Implement

#### Step 1: Visualization Engine Core (6 hours)
1. **Implement VisualizationEngine** with chart registry and rendering pipeline
2. **Add animation engine** with smooth transitions and effects
3. **Create interaction handler** for zoom, pan, and selection
4. **Add performance optimization** with level-of-detail rendering

#### Step 2: Advanced Chart Components (8 hours)
1. **Implement AdvancedLineChart** with smooth curves and animations
2. **Add NetworkGraphChart** with physics simulation and layouts
3. **Create HeatmapChart** with color scales and clustering
4. **Add ScatterPlotChart** with bubble sizing and trend lines

#### Step 3: Interactive Dashboard (6 hours)
1. **Implement InteractiveDashboard** with layout management
2. **Add drag-and-drop** widget positioning and resizing
3. **Create widget registry** with extensible widget system
4. **Add dashboard export** with multiple format support

#### Step 4: Real-Time Data Pipeline (5 hours)
1. **Implement DataPipeline** with streaming connections
2. **Add data source abstraction** for multiple data types
3. **Create data processors** for transformations and aggregations
4. **Add performance monitoring** and error handling

#### Step 5: Integration and Testing (3 hours)
1. **Add comprehensive unit tests** for all visualization components
2. **Test performance** with large datasets and real-time streams
3. **Validate interactive features** and dashboard building
4. **Test data pipeline** reliability and error handling

## 🔗 Dependencies
**Depends On:**
- Issue #023 (TUI Architecture Foundation) - Base component system
- Issue #024 (Dashboard Overview) - Integration with main dashboard
- Phase 2 enhanced commands for data source integration

**Enables:**
- Advanced data exploration and analysis capabilities
- Real-time monitoring dashboards
- Custom visualization creation for specific use cases

## 🧪 Testing Requirements

### Unit Tests
```rust
#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_visualization_engine_performance() {
        let mut engine = VisualizationEngine::new();
        let large_dataset = create_test_dataset(100_000);
        
        let start_time = Instant::now();
        let viz_id = engine.create_visualization(VisualizationConfig::line_chart()).await.unwrap();
        engine.update_visualization_data(&viz_id, DataUpdate::Bulk(large_dataset)).await.unwrap();
        let creation_time = start_time.elapsed();
        
        assert!(creation_time < Duration::from_millis(100));
    }
    
    #[test]
    fn test_real_time_data_pipeline() {
        let mut pipeline = DataPipeline::new();
        let source = Box::new(MockDataSource::new());
        
        let source_id = pipeline.register_data_source(source).await;
        pipeline.connect_widget_to_source(WidgetId(1), &DataSourceConfig::mock()).await.unwrap();
        
        // Simulate data updates
        for _ in 0..100 {
            pipeline.process_updates().await.unwrap();
        }
        
        let data = pipeline.get_widget_data(WidgetId(1)).unwrap();
        assert!(data.len() > 0);
    }
    
    #[test]
    fn test_dashboard_interaction() {
        let mut dashboard = InteractiveDashboard::new(DashboardConfig::test_config());
        
        let widget_id = dashboard.add_widget(WidgetConfig::line_chart()).await.unwrap();
        
        dashboard.handle_interaction(DashboardInteraction::WidgetDrag {
            widget_id,
            new_position: Position { x: 100, y: 100 },
        }).await.unwrap();
        
        // Verify widget was moved
        let layout = dashboard.layout_manager.get_widget_position(widget_id).unwrap();
        assert_eq!(layout.x, 100);
        assert_eq!(layout.y, 100);
    }
}
```

## ✅ Definition of Done
- [ ] VisualizationEngine supports multiple chart types with real-time updates
- [ ] Advanced charts render smoothly with animations and interactions
- [ ] Interactive dashboard enables drag-and-drop widget management
- [ ] Real-time data pipeline handles high-frequency data streams efficiently
- [ ] Performance targets met for rendering and data processing
- [ ] Chart interactions (zoom, pan, selection) respond within target times
- [ ] Dashboard export functionality works for all supported formats
- [ ] Data sources integrate seamlessly with Dora metrics and logs
- [ ] Memory usage stays within limits during extended visualization sessions
- [ ] Animation engine provides smooth transitions without performance impact
- [ ] Comprehensive unit tests validate all visualization functionality
- [ ] Integration tests confirm real-time data flow and rendering accuracy
- [ ] Manual testing validates user experience for dashboard creation and interaction

This advanced data visualization engine provides developers with powerful tools for exploring, analyzing, and monitoring Dora system behavior through interactive, real-time visualizations and customizable dashboards.