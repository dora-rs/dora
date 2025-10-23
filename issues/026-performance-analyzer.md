# Issue #026: Build Interactive Performance Analyzer

## 📋 Summary
Implement a comprehensive interactive performance analyzer that provides real-time and historical performance visualization, bottleneck identification, and optimization recommendations. This TUI view enables deep performance analysis with advanced charting, correlation analysis, and predictive insights.

## 🎯 Objectives
- Create comprehensive performance visualization with multiple chart types and time ranges
- Implement real-time performance monitoring with configurable alerts and thresholds
- Add bottleneck detection and performance correlation analysis
- Provide optimization recommendations based on performance patterns
- Enable performance comparison across different time periods and configurations

**Success Metrics:**
- Performance data visualization renders within 150ms for typical datasets
- Real-time updates maintain sub-second latency without impacting system performance
- Bottleneck detection accuracy exceeds 85% for common performance issues
- Optimization recommendations improve system performance by 20% on average
- User satisfaction with performance insights exceeds 90%

## 🛠️ Technical Requirements

### What to Build

#### 1. Performance Analyzer View
```rust
// src/tui/views/performance_analyzer.rs
use super::BaseView;
use crate::tui::{
    components::*,
    theme::Theme,
    AppContext, View, ViewEvent, ViewEventResult, ViewType
};
use async_trait::async_trait;
use ratatui::{
    layout::{Constraint, Direction, Layout, Rect},
    widgets::{Block, Borders, Tabs},
    Frame,
};
use std::collections::VecDeque;

pub struct PerformanceAnalyzerView {
    base: BaseView,
    analysis_state: PerformanceAnalysisState,
    current_view_mode: ViewMode,
    time_range: TimeRange,
    metric_selection: MetricSelection,
    performance_data: PerformanceDataManager,
    analyzer_engine: PerformanceAnalyzerEngine,
    real_time_monitor: RealTimePerformanceMonitor,
    comparison_mode: ComparisonMode,
}

#[derive(Debug, Clone)]
pub struct PerformanceAnalysisState {
    pub current_metrics: HashMap<String, MetricSeries>,
    pub historical_data: HashMap<String, VecDeque<MetricPoint>>,
    pub detected_bottlenecks: Vec<PerformanceBottleneck>,
    pub optimization_suggestions: Vec<OptimizationSuggestion>,
    pub alerts: Vec<PerformanceAlert>,
    pub baseline_metrics: Option<BaselineMetrics>,
}

#[derive(Debug, Clone)]
pub enum ViewMode {
    Overview,
    DetailedCharts,
    BottleneckAnalysis,
    Comparison,
    Alerts,
    Recommendations,
}

#[derive(Debug, Clone)]
pub struct TimeRange {
    pub start: DateTime<Utc>,
    pub end: DateTime<Utc>,
    pub resolution: Duration,
}

#[derive(Debug, Clone)]
pub struct MetricSelection {
    pub selected_metrics: HashSet<String>,
    pub grouped_by: GroupingStrategy,
    pub aggregation: AggregationStrategy,
}

impl PerformanceAnalyzerView {
    pub fn new(analysis_type: AnalysisType) -> Self {
        let mut base = BaseView::new();
        
        // Configure performance analyzer layout
        let layout = LayoutConfig::custom(vec![
            LayoutSection {
                section_type: SectionType::Header,
                area: SectionArea::TopBar(4),
                component_id: ComponentId("control_panel".to_string()),
            },
            LayoutSection {
                section_type: SectionType::Main,
                area: SectionArea::MainContent,
                component_id: ComponentId("chart_area".to_string()),
            },
            LayoutSection {
                section_type: SectionType::Sidebar,
                area: SectionArea::RightSidebar(35),
                component_id: ComponentId("metrics_panel".to_string()),
            },
            LayoutSection {
                section_type: SectionType::Footer,
                area: SectionArea::BottomBar(8),
                component_id: ComponentId("insights_panel".to_string()),
            },
        ]);
        
        base.set_layout(layout);
        
        // Add performance analyzer components
        base.add_component(
            ComponentId("control_panel".to_string()),
            PerformanceControlPanelComponent::new(),
        );
        
        base.add_component(
            ComponentId("chart_area".to_string()),
            PerformanceChartsComponent::new(),
        );
        
        base.add_component(
            ComponentId("metrics_panel".to_string()),
            MetricsSelectionComponent::new(),
        );
        
        base.add_component(
            ComponentId("insights_panel".to_string()),
            PerformanceInsightsComponent::new(),
        );
        
        Self {
            base,
            analysis_state: PerformanceAnalysisState::default(),
            current_view_mode: ViewMode::Overview,
            time_range: TimeRange::last_hour(),
            metric_selection: MetricSelection::default(),
            performance_data: PerformanceDataManager::new(),
            analyzer_engine: PerformanceAnalyzerEngine::new(),
            real_time_monitor: RealTimePerformanceMonitor::new(),
            comparison_mode: ComparisonMode::None,
        }
    }
    
    async fn refresh_performance_data(&mut self) -> io::Result<()> {
        // Collect current metrics
        self.analysis_state.current_metrics = self.performance_data
            .collect_current_metrics(&self.metric_selection)
            .await?;
        
        // Update historical data
        for (metric_name, metric_series) in &self.analysis_state.current_metrics {
            self.analysis_state.historical_data
                .entry(metric_name.clone())
                .or_insert_with(|| VecDeque::with_capacity(1000))
                .extend(metric_series.data_points.iter().cloned());
            
            // Limit historical data size
            let historical = self.analysis_state.historical_data.get_mut(metric_name).unwrap();
            while historical.len() > 1000 {
                historical.pop_front();
            }
        }
        
        // Run bottleneck detection
        self.analysis_state.detected_bottlenecks = self.analyzer_engine
            .detect_bottlenecks(&self.analysis_state.current_metrics)
            .await?;
        
        // Generate optimization suggestions
        self.analysis_state.optimization_suggestions = self.analyzer_engine
            .generate_optimization_suggestions(
                &self.analysis_state.current_metrics,
                &self.analysis_state.detected_bottlenecks,
            )
            .await?;
        
        // Check for performance alerts
        self.analysis_state.alerts = self.analyzer_engine
            .check_performance_alerts(
                &self.analysis_state.current_metrics,
                &self.analysis_state.baseline_metrics,
            )
            .await?;
        
        Ok(())
    }
    
    async fn start_baseline_collection(&mut self) -> io::Result<()> {
        self.analysis_state.baseline_metrics = Some(
            self.analyzer_engine
                .collect_baseline_metrics(&self.time_range)
                .await?
        );
        
        Ok(())
    }
    
    fn switch_view_mode(&mut self, new_mode: ViewMode) {
        self.current_view_mode = new_mode;
        
        // Reconfigure layout based on view mode
        match new_mode {
            ViewMode::Overview => self.configure_overview_layout(),
            ViewMode::DetailedCharts => self.configure_detailed_charts_layout(),
            ViewMode::BottleneckAnalysis => self.configure_bottleneck_layout(),
            ViewMode::Comparison => self.configure_comparison_layout(),
            ViewMode::Alerts => self.configure_alerts_layout(),
            ViewMode::Recommendations => self.configure_recommendations_layout(),
        }
    }
    
    fn configure_overview_layout(&mut self) {
        // Four-quadrant overview layout
        let layout = LayoutConfig::grid(vec![
            LayoutRow {
                height: Constraint::Length(4),
                columns: vec![
                    LayoutColumn {
                        width: Constraint::Percentage(100),
                        component_id: ComponentId("control_panel".to_string()),
                    },
                ],
            },
            LayoutRow {
                height: Constraint::Percentage(60),
                columns: vec![
                    LayoutColumn {
                        width: Constraint::Percentage(50),
                        component_id: ComponentId("cpu_chart".to_string()),
                    },
                    LayoutColumn {
                        width: Constraint::Percentage(50),
                        component_id: ComponentId("memory_chart".to_string()),
                    },
                ],
            },
            LayoutRow {
                height: Constraint::Percentage(40),
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
        ]);
        
        self.base.set_layout(layout);
    }
    
    fn configure_detailed_charts_layout(&mut self) {
        // Full-screen chart with metrics panel
        let layout = LayoutConfig::custom(vec![
            LayoutSection {
                section_type: SectionType::Header,
                area: SectionArea::TopBar(4),
                component_id: ComponentId("control_panel".to_string()),
            },
            LayoutSection {
                section_type: SectionType::Main,
                area: SectionArea::MainContent,
                component_id: ComponentId("detailed_chart".to_string()),
            },
            LayoutSection {
                section_type: SectionType::Sidebar,
                area: SectionArea::RightSidebar(30),
                component_id: ComponentId("chart_controls".to_string()),
            },
        ]);
        
        self.base.set_layout(layout);
    }
}

#[async_trait]
impl View for PerformanceAnalyzerView {
    async fn initialize(&mut self, context: &AppContext) -> io::Result<()> {
        // Initialize performance data collection
        self.refresh_performance_data().await?;
        
        // Start baseline collection if needed
        if self.analysis_state.baseline_metrics.is_none() {
            self.start_baseline_collection().await?;
        }
        
        // Initialize all components
        for component in self.base.component_registry.components_mut() {
            component.update(context).await?;
        }
        
        // Start real-time monitoring
        self.real_time_monitor.start().await?;
        
        Ok(())
    }
    
    async fn update(&mut self, context: &mut AppContext) -> io::Result<()> {
        // Check for real-time updates
        if let Some(update) = self.real_time_monitor.get_latest_update() {
            self.apply_real_time_update(update);
        }
        
        // Periodic data refresh
        if self.should_refresh_data() {
            self.refresh_performance_data().await?;
        }
        
        // Update all components
        for component in self.base.component_registry.components_mut() {
            component.update(context).await?;
        }
        
        Ok(())
    }
    
    fn render(&self, frame: &mut Frame, area: Rect, theme: &Theme, context: &AppContext) {
        // Render main analyzer layout
        self.base.render_components(frame, area, theme, context);
        
        // Render mode-specific overlays
        match self.current_view_mode {
            ViewMode::Alerts => {
                self.render_alerts_overlay(frame, area, theme);
            },
            ViewMode::Comparison => {
                self.render_comparison_overlay(frame, area, theme);
            },
            _ => {}
        }
    }
    
    async fn handle_event(&mut self, event: ViewEvent, context: &mut AppContext) -> io::Result<ViewEventResult> {
        // Handle performance analyzer specific key bindings
        if let ViewEvent::Key(key_event) = &event {
            match key_event.code {
                crossterm::event::KeyCode::Char('1') => {
                    self.switch_view_mode(ViewMode::Overview);
                    return Ok(ViewEventResult::None);
                },
                
                crossterm::event::KeyCode::Char('2') => {
                    self.switch_view_mode(ViewMode::DetailedCharts);
                    return Ok(ViewEventResult::None);
                },
                
                crossterm::event::KeyCode::Char('3') => {
                    self.switch_view_mode(ViewMode::BottleneckAnalysis);
                    return Ok(ViewEventResult::None);
                },
                
                crossterm::event::KeyCode::Char('4') => {
                    self.switch_view_mode(ViewMode::Comparison);
                    return Ok(ViewEventResult::None);
                },
                
                crossterm::event::KeyCode::Char('5') => {
                    self.switch_view_mode(ViewMode::Alerts);
                    return Ok(ViewEventResult::None);
                },
                
                crossterm::event::KeyCode::Char('6') => {
                    self.switch_view_mode(ViewMode::Recommendations);
                    return Ok(ViewEventResult::None);
                },
                
                crossterm::event::KeyCode::Char('t') => {
                    // Cycle time range
                    self.cycle_time_range();
                    return Ok(ViewEventResult::None);
                },
                
                crossterm::event::KeyCode::Char('b') => {
                    // Start new baseline collection
                    self.start_baseline_collection().await?;
                    return Ok(ViewEventResult::None);
                },
                
                crossterm::event::KeyCode::Char('e') => {
                    // Export current analysis
                    self.export_analysis().await?;
                    return Ok(ViewEventResult::None);
                },
                
                crossterm::event::KeyCode::F5 => {
                    // Force refresh
                    self.refresh_performance_data().await?;
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
        self.real_time_monitor.stop().await?;
        Ok(())
    }
    
    fn view_type(&self) -> ViewType {
        ViewType::PerformanceAnalyzer { analysis_type: AnalysisType::Comprehensive }
    }
    
    fn title(&self) -> String {
        format!("Performance Analyzer - {}", self.current_view_mode.display_name())
    }
    
    fn help_text(&self) -> Vec<HelpItem> {
        vec![
            HelpItem::new("1-6", "Switch view modes"),
            HelpItem::new("t", "Cycle time range"),
            HelpItem::new("b", "Set new baseline"),
            HelpItem::new("e", "Export analysis"),
            HelpItem::new("F5", "Refresh data"),
            HelpItem::new("Esc", "Back"),
        ]
    }
}
```

#### 2. Performance Chart Components
```rust
// src/tui/components/performance/charts.rs
use ratatui::{
    symbols,
    widgets::{Axis, Chart, Dataset, GraphType},
};

pub struct PerformanceChartsComponent {
    chart_data: HashMap<String, ChartData>,
    active_charts: Vec<ChartConfig>,
    chart_layout: ChartLayout,
    time_range: TimeRange,
    focused: bool,
    zoom_level: f64,
    pan_offset: f64,
}

#[derive(Debug, Clone)]
pub struct ChartData {
    pub data_points: Vec<(f64, f64)>,
    pub chart_type: ChartType,
    pub color: Color,
    pub label: String,
    pub unit: String,
    pub min_value: f64,
    pub max_value: f64,
}

#[derive(Debug, Clone)]
pub enum ChartType {
    Line,
    Area,
    Bar,
    Scatter,
    Heatmap,
}

#[derive(Debug, Clone)]
pub struct ChartConfig {
    pub metric_name: String,
    pub chart_type: ChartType,
    pub y_axis_config: YAxisConfig,
    pub display_options: ChartDisplayOptions,
}

impl PerformanceChartsComponent {
    pub fn new() -> Self {
        Self {
            chart_data: HashMap::new(),
            active_charts: Vec::new(),
            chart_layout: ChartLayout::Single,
            time_range: TimeRange::last_hour(),
            focused: false,
            zoom_level: 1.0,
            pan_offset: 0.0,
        }
    }
    
    fn render_single_chart(&self, frame: &mut Frame, area: Rect, chart_config: &ChartConfig, theme: &Theme) {
        if let Some(chart_data) = self.chart_data.get(&chart_config.metric_name) {
            let datasets = vec![
                Dataset::default()
                    .name(&chart_data.label)
                    .marker(symbols::Marker::Braille)
                    .graph_type(match chart_config.chart_type {
                        ChartType::Line => GraphType::Line,
                        ChartType::Scatter => GraphType::Scatter,
                        _ => GraphType::Line,
                    })
                    .style(Style::default().fg(chart_data.color))
                    .data(&chart_data.data_points)
            ];
            
            let x_axis = Axis::default()
                .title("Time")
                .style(theme.axis_style())
                .bounds(self.calculate_x_bounds())
                .labels(self.generate_time_labels());
            
            let y_axis = Axis::default()
                .title(&format!("{} ({})", chart_data.label, chart_data.unit))
                .style(theme.axis_style())
                .bounds([chart_data.min_value, chart_data.max_value])
                .labels(self.generate_y_labels(chart_data.min_value, chart_data.max_value));
            
            let chart = Chart::new(datasets)
                .block(
                    Block::default()
                        .title(&chart_data.label)
                        .borders(Borders::ALL)
                        .border_style(if self.focused {
                            theme.focused_border_style()
                        } else {
                            theme.normal_border_style()
                        })
                )
                .x_axis(x_axis)
                .y_axis(y_axis);
            
            frame.render_widget(chart, area);
            
            // Render chart overlays
            self.render_chart_overlays(frame, area, chart_config, chart_data, theme);
        }
    }
    
    fn render_chart_overlays(&self, frame: &mut Frame, area: Rect, config: &ChartConfig, data: &ChartData, theme: &Theme) {
        // Render performance thresholds
        self.render_performance_thresholds(frame, area, config, theme);
        
        // Render anomaly markers
        self.render_anomaly_markers(frame, area, data, theme);
        
        // Render current value indicator
        self.render_current_value_indicator(frame, area, data, theme);
    }
    
    fn render_performance_thresholds(&self, frame: &mut Frame, area: Rect, config: &ChartConfig, theme: &Theme) {
        // Render warning and critical threshold lines
        if let Some(thresholds) = self.get_performance_thresholds(&config.metric_name) {
            // Warning threshold
            if let Some(warning_level) = thresholds.warning {
                self.render_horizontal_line(frame, area, warning_level, theme.warning_style());
            }
            
            // Critical threshold
            if let Some(critical_level) = thresholds.critical {
                self.render_horizontal_line(frame, area, critical_level, theme.error_style());
            }
        }
    }
    
    fn render_multi_chart_layout(&self, frame: &mut Frame, area: Rect, theme: &Theme) {
        match self.chart_layout {
            ChartLayout::Single => {
                if let Some(chart_config) = self.active_charts.first() {
                    self.render_single_chart(frame, area, chart_config, theme);
                }
            },
            
            ChartLayout::TwoByTwo => {
                let chunks = Layout::default()
                    .direction(Direction::Vertical)
                    .constraints([Constraint::Percentage(50), Constraint::Percentage(50)])
                    .split(area);
                
                let top_chunks = Layout::default()
                    .direction(Direction::Horizontal)
                    .constraints([Constraint::Percentage(50), Constraint::Percentage(50)])
                    .split(chunks[0]);
                
                let bottom_chunks = Layout::default()
                    .direction(Direction::Horizontal)
                    .constraints([Constraint::Percentage(50), Constraint::Percentage(50)])
                    .split(chunks[1]);
                
                let areas = [top_chunks[0], top_chunks[1], bottom_chunks[0], bottom_chunks[1]];
                
                for (i, area) in areas.iter().enumerate() {
                    if let Some(chart_config) = self.active_charts.get(i) {
                        self.render_single_chart(frame, *area, chart_config, theme);
                    }
                }
            },
            
            ChartLayout::Stacked => {
                let constraints: Vec<Constraint> = (0..self.active_charts.len())
                    .map(|_| Constraint::Percentage(100 / self.active_charts.len() as u16))
                    .collect();
                
                let chunks = Layout::default()
                    .direction(Direction::Vertical)
                    .constraints(constraints)
                    .split(area);
                
                for (i, chunk) in chunks.iter().enumerate() {
                    if let Some(chart_config) = self.active_charts.get(i) {
                        self.render_single_chart(frame, *chunk, chart_config, theme);
                    }
                }
            },
        }
    }
    
    fn calculate_x_bounds(&self) -> [f64; 2] {
        let end_time = self.time_range.end.timestamp() as f64;
        let start_time = self.time_range.start.timestamp() as f64;
        
        // Apply zoom and pan
        let duration = end_time - start_time;
        let zoomed_duration = duration / self.zoom_level;
        let pan_offset = self.pan_offset * duration;
        
        [start_time + pan_offset, start_time + pan_offset + zoomed_duration]
    }
    
    fn generate_time_labels(&self) -> Vec<String> {
        let bounds = self.calculate_x_bounds();
        let start_time = DateTime::<Utc>::from_timestamp(bounds[0] as i64, 0).unwrap();
        let end_time = DateTime::<Utc>::from_timestamp(bounds[1] as i64, 0).unwrap();
        
        // Generate 5 evenly spaced time labels
        let interval = (bounds[1] - bounds[0]) / 4.0;
        (0..5)
            .map(|i| {
                let timestamp = bounds[0] + (i as f64 * interval);
                let time = DateTime::<Utc>::from_timestamp(timestamp as i64, 0).unwrap();
                time.format("%H:%M:%S").to_string()
            })
            .collect()
    }
}

#[async_trait]
impl Component for PerformanceChartsComponent {
    async fn update(&mut self, context: &AppContext) -> io::Result<()> {
        // Update chart data from performance analyzer state
        if let Some(analyzer_state) = context.get_analyzer_state() {
            for (metric_name, metric_series) in &analyzer_state.current_metrics {
                let chart_data = ChartData {
                    data_points: metric_series.to_chart_points(&self.time_range),
                    chart_type: ChartType::Line,
                    color: self.get_metric_color(metric_name),
                    label: metric_name.clone(),
                    unit: metric_series.unit.clone(),
                    min_value: metric_series.min_value(),
                    max_value: metric_series.max_value(),
                };
                
                self.chart_data.insert(metric_name.clone(), chart_data);
            }
        }
        
        Ok(())
    }
    
    fn render(&self, frame: &mut Frame, area: Rect, theme: &Theme, context: &AppContext) {
        self.render_multi_chart_layout(frame, area, theme);
    }
    
    async fn handle_event(&mut self, event: ComponentEvent, context: &AppContext) -> io::Result<ViewEventResult> {
        if !self.focused {
            return Ok(ViewEventResult::None);
        }
        
        match event {
            ComponentEvent::Key(key_event) => {
                match key_event.code {
                    crossterm::event::KeyCode::Char('+') => {
                        self.zoom_level = (self.zoom_level * 1.2).min(10.0);
                    },
                    crossterm::event::KeyCode::Char('-') => {
                        self.zoom_level = (self.zoom_level / 1.2).max(0.1);
                    },
                    crossterm::event::KeyCode::Left => {
                        self.pan_offset -= 0.1 / self.zoom_level;
                    },
                    crossterm::event::KeyCode::Right => {
                        self.pan_offset += 0.1 / self.zoom_level;
                    },
                    crossterm::event::KeyCode::Char('0') => {
                        self.zoom_level = 1.0;
                        self.pan_offset = 0.0;
                    },
                    crossterm::event::KeyCode::Char('l') => {
                        self.chart_layout = match self.chart_layout {
                            ChartLayout::Single => ChartLayout::TwoByTwo,
                            ChartLayout::TwoByTwo => ChartLayout::Stacked,
                            ChartLayout::Stacked => ChartLayout::Single,
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
        ComponentType::PerformanceCharts
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

#### 3. Bottleneck Detection Component
```rust
// src/tui/components/performance/bottleneck_analyzer.rs
pub struct BottleneckAnalyzerComponent {
    detected_bottlenecks: Vec<PerformanceBottleneck>,
    analysis_results: BottleneckAnalysisResults,
    selected_bottleneck: Option<usize>,
    analysis_depth: AnalysisDepth,
    focused: bool,
}

#[derive(Debug, Clone)]
pub struct PerformanceBottleneck {
    pub bottleneck_type: BottleneckType,
    pub affected_component: String,
    pub severity: BottleneckSeverity,
    pub impact_score: f64,
    pub description: String,
    pub root_cause: Option<String>,
    pub metrics_evidence: Vec<MetricEvidence>,
    pub remediation_steps: Vec<RemediationStep>,
    pub estimated_improvement: Option<f64>,
}

#[derive(Debug, Clone)]
pub enum BottleneckType {
    CpuBound,
    MemoryBound,
    IoBound,
    NetworkBound,
    DiskBound,
    ContentionBound,
    AlgorithmicBound,
}

#[derive(Debug, Clone)]
pub struct MetricEvidence {
    pub metric_name: String,
    pub current_value: f64,
    pub threshold_value: f64,
    pub deviation_percentage: f64,
    pub trend: TrendDirection,
}

impl BottleneckAnalyzerComponent {
    pub fn new() -> Self {
        Self {
            detected_bottlenecks: Vec::new(),
            analysis_results: BottleneckAnalysisResults::default(),
            selected_bottleneck: None,
            analysis_depth: AnalysisDepth::Standard,
            focused: false,
        }
    }
    
    fn render_bottleneck_list(&self, frame: &mut Frame, area: Rect, theme: &Theme) {
        let items: Vec<ListItem> = self.detected_bottlenecks.iter()
            .enumerate()
            .map(|(i, bottleneck)| {
                let severity_icon = match bottleneck.severity {
                    BottleneckSeverity::Critical => "🔴",
                    BottleneckSeverity::High => "🟠",
                    BottleneckSeverity::Medium => "🟡",
                    BottleneckSeverity::Low => "🔵",
                };
                
                let content = format!(
                    "{} {} - {} (Impact: {:.1}%)",
                    severity_icon,
                    bottleneck.bottleneck_type.display_name(),
                    bottleneck.affected_component,
                    bottleneck.impact_score * 100.0
                );
                
                let style = if Some(i) == self.selected_bottleneck {
                    theme.selected_item_style()
                } else {
                    theme.normal_item_style()
                };
                
                ListItem::new(content).style(style)
            })
            .collect();
        
        let list = List::new(items)
            .block(
                Block::default()
                    .title("Detected Bottlenecks")
                    .borders(Borders::ALL)
            )
            .highlight_style(theme.highlight_style())
            .highlight_symbol("▶ ");
        
        frame.render_stateful_widget(
            list,
            area,
            &mut ratatui::widgets::ListState::default()
                .with_selected(self.selected_bottleneck)
        );
    }
    
    fn render_bottleneck_details(&self, frame: &mut Frame, area: Rect, theme: &Theme) {
        if let Some(selected_idx) = self.selected_bottleneck {
            if let Some(bottleneck) = self.detected_bottlenecks.get(selected_idx) {
                let chunks = Layout::default()
                    .direction(Direction::Vertical)
                    .constraints([
                        Constraint::Length(6),
                        Constraint::Length(6),
                        Constraint::Min(4),
                    ])
                    .split(area);
                
                // Basic information
                self.render_bottleneck_info(frame, chunks[0], bottleneck, theme);
                
                // Metric evidence
                self.render_metric_evidence(frame, chunks[1], bottleneck, theme);
                
                // Remediation steps
                self.render_remediation_steps(frame, chunks[2], bottleneck, theme);
            }
        }
    }
    
    fn render_bottleneck_info(&self, frame: &mut Frame, area: Rect, bottleneck: &PerformanceBottleneck, theme: &Theme) {
        let info_text = vec![
            format!("Type: {}", bottleneck.bottleneck_type.display_name()),
            format!("Component: {}", bottleneck.affected_component),
            format!("Severity: {}", bottleneck.severity.display_name()),
            format!("Impact Score: {:.1}%", bottleneck.impact_score * 100.0),
            format!("Description: {}", bottleneck.description),
        ];
        
        let paragraph = Paragraph::new(info_text.join("\n"))
            .block(Block::default().title("Bottleneck Details").borders(Borders::ALL))
            .style(theme.normal_style());
        
        frame.render_widget(paragraph, area);
    }
    
    fn render_metric_evidence(&self, frame: &mut Frame, area: Rect, bottleneck: &PerformanceBottleneck, theme: &Theme) {
        let evidence_text: Vec<String> = bottleneck.metrics_evidence.iter()
            .map(|evidence| {
                format!(
                    "{}: {:.2} (threshold: {:.2}, {:.1}% {})",
                    evidence.metric_name,
                    evidence.current_value,
                    evidence.threshold_value,
                    evidence.deviation_percentage,
                    match evidence.trend {
                        TrendDirection::Increasing => "↗",
                        TrendDirection::Decreasing => "↘",
                        TrendDirection::Stable => "→",
                    }
                )
            })
            .collect();
        
        let paragraph = Paragraph::new(evidence_text.join("\n"))
            .block(Block::default().title("Metric Evidence").borders(Borders::ALL))
            .style(theme.normal_style());
        
        frame.render_widget(paragraph, area);
    }
    
    fn render_remediation_steps(&self, frame: &mut Frame, area: Rect, bottleneck: &PerformanceBottleneck, theme: &Theme) {
        let steps_text: Vec<String> = bottleneck.remediation_steps.iter()
            .enumerate()
            .map(|(i, step)| {
                format!("{}. {} (Priority: {})", i + 1, step.description, step.priority.display_name())
            })
            .collect();
        
        let paragraph = Paragraph::new(steps_text.join("\n"))
            .block(Block::default().title("Remediation Steps").borders(Borders::ALL))
            .style(theme.normal_style());
        
        frame.render_widget(paragraph, area);
    }
    
    fn analyze_bottleneck_impact(&self, bottleneck: &PerformanceBottleneck) -> BottleneckImpactAnalysis {
        BottleneckImpactAnalysis {
            performance_degradation: bottleneck.impact_score,
            affected_operations: self.identify_affected_operations(bottleneck),
            cascade_effects: self.analyze_cascade_effects(bottleneck),
            remediation_complexity: self.assess_remediation_complexity(bottleneck),
        }
    }
}

#[async_trait]
impl Component for BottleneckAnalyzerComponent {
    async fn update(&mut self, context: &AppContext) -> io::Result<()> {
        // Update bottleneck data from analyzer state
        if let Some(analyzer_state) = context.get_analyzer_state() {
            self.detected_bottlenecks = analyzer_state.detected_bottlenecks.clone();
            
            // Ensure selected bottleneck is still valid
            if let Some(selected) = self.selected_bottleneck {
                if selected >= self.detected_bottlenecks.len() {
                    self.selected_bottleneck = if self.detected_bottlenecks.is_empty() {
                        None
                    } else {
                        Some(self.detected_bottlenecks.len() - 1)
                    };
                }
            }
        }
        
        Ok(())
    }
    
    fn render(&self, frame: &mut Frame, area: Rect, theme: &Theme, context: &AppContext) {
        let chunks = Layout::default()
            .direction(Direction::Horizontal)
            .constraints([Constraint::Percentage(40), Constraint::Percentage(60)])
            .split(area);
        
        // Render bottleneck list
        self.render_bottleneck_list(frame, chunks[0], theme);
        
        // Render bottleneck details
        self.render_bottleneck_details(frame, chunks[1], theme);
    }
    
    async fn handle_event(&mut self, event: ComponentEvent, context: &AppContext) -> io::Result<ViewEventResult> {
        if !self.focused {
            return Ok(ViewEventResult::None);
        }
        
        match event {
            ComponentEvent::Key(key_event) => {
                match key_event.code {
                    crossterm::event::KeyCode::Up => {
                        if !self.detected_bottlenecks.is_empty() {
                            self.selected_bottleneck = Some(
                                self.selected_bottleneck
                                    .map(|i| if i == 0 { self.detected_bottlenecks.len() - 1 } else { i - 1 })
                                    .unwrap_or(0)
                            );
                        }
                    },
                    crossterm::event::KeyCode::Down => {
                        if !self.detected_bottlenecks.is_empty() {
                            self.selected_bottleneck = Some(
                                self.selected_bottleneck
                                    .map(|i| (i + 1) % self.detected_bottlenecks.len())
                                    .unwrap_or(0)
                            );
                        }
                    },
                    crossterm::event::KeyCode::Enter => {
                        if let Some(selected_idx) = self.selected_bottleneck {
                            if let Some(_bottleneck) = self.detected_bottlenecks.get(selected_idx) {
                                // Open detailed bottleneck analysis view
                                return Ok(ViewEventResult::PushView(ViewType::BottleneckDetail {
                                    bottleneck_id: selected_idx.to_string(),
                                }));
                            }
                        }
                    },
                    _ => {}
                }
            },
            _ => {}
        }
        
        Ok(ViewEventResult::None)
    }
    
    fn component_type(&self) -> ComponentType {
        ComponentType::BottleneckAnalyzer
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

**Comprehensive Analysis:**
- Multi-dimensional performance monitoring with real-time updates
- Advanced bottleneck detection with root cause analysis
- Predictive insights and optimization recommendations

**Interactive Visualization:**
- Multiple chart types and layout options
- Zoom, pan, and time range controls
- Threshold visualization and anomaly highlighting

**Actionable Insights:**
- Detailed bottleneck analysis with remediation steps
- Performance comparison and trend analysis
- Export capabilities for further analysis

### How to Implement

#### Step 1: Core Analyzer View (4 hours)
1. **Implement PerformanceAnalyzerView** with multiple view modes
2. **Add time range management** and data refresh controls
3. **Create view mode switching** and layout configuration
4. **Add baseline collection** and comparison features

#### Step 2: Performance Charts (5 hours)
1. **Implement PerformanceChartsComponent** with multiple chart types
2. **Add interactive chart controls** (zoom, pan, layout switching)
3. **Create threshold visualization** and anomaly highlighting
4. **Add chart export** and data extraction features

#### Step 3: Bottleneck Analysis (4 hours)
1. **Implement BottleneckAnalyzerComponent** with detection algorithms
2. **Add bottleneck classification** and severity assessment
3. **Create remediation step** generation and prioritization
4. **Add impact analysis** and cascade effect detection

#### Step 4: Real-time Monitoring (3 hours)
1. **Implement RealTimePerformanceMonitor** for live updates
2. **Add performance alert** system with configurable thresholds
3. **Create optimization suggestion** engine
4. **Add performance baseline** management

#### Step 5: Testing and Polish (2 hours)
1. **Add comprehensive unit tests** for all components
2. **Test chart rendering** performance with large datasets
3. **Validate bottleneck detection** accuracy
4. **Test real-time monitoring** and alert systems

## 🔗 Dependencies
**Depends On:**
- Issue #023 (TUI Architecture Foundation) - Base view and component system
- Issue #014 (Resource Analysis System) - Performance analysis engines

**Enables:**
- Comprehensive performance monitoring and optimization
- Visual performance debugging and analysis

## 🧪 Testing Requirements

### Unit Tests
```rust
#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_bottleneck_detection() {
        let analyzer = BottleneckAnalyzerComponent::new();
        let test_metrics = create_test_performance_metrics();
        
        let bottlenecks = analyzer.detect_bottlenecks(&test_metrics).await.unwrap();
        
        assert!(!bottlenecks.is_empty());
        assert!(bottlenecks.iter().any(|b| matches!(b.bottleneck_type, BottleneckType::CpuBound)));
    }
    
    #[test]
    fn test_chart_data_transformation() {
        let chart_component = PerformanceChartsComponent::new();
        let metric_series = create_test_metric_series();
        
        let chart_data = chart_component.transform_metric_to_chart_data(&metric_series);
        
        assert!(!chart_data.data_points.is_empty());
        assert!(chart_data.min_value < chart_data.max_value);
    }
    
    #[test]
    fn test_time_range_calculations() {
        let time_range = TimeRange::last_hour();
        let chart_component = PerformanceChartsComponent::new();
        
        let x_bounds = chart_component.calculate_x_bounds();
        
        assert!(x_bounds[0] < x_bounds[1]);
    }
}
```

## ✅ Definition of Done
- [ ] PerformanceAnalyzerView provides comprehensive performance analysis with multiple view modes
- [ ] Interactive charts render performance data with zoom, pan, and layout controls
- [ ] Bottleneck detection identifies performance issues with accurate classification
- [ ] Real-time monitoring updates performance data without impacting system performance
- [ ] Optimization recommendations provide actionable insights for performance improvement
- [ ] Time range controls enable historical analysis and comparison
- [ ] Performance targets met for chart rendering and data processing
- [ ] Export functionality enables sharing and further analysis of performance data
- [ ] Alert system notifies users of performance threshold violations
- [ ] Comprehensive unit tests validate all analyzer functionality
- [ ] Integration tests confirm performance monitoring workflows
- [ ] Manual testing validates chart interactivity and bottleneck detection accuracy

This interactive performance analyzer provides users with powerful tools for monitoring, analyzing, and optimizing system performance through comprehensive visualization and intelligent analysis capabilities.