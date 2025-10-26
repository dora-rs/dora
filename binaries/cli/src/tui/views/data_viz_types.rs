// Data Visualization Types - Phase 1: Simplified Implementation with Mock Data
// TODO(Issue #32 Phase 2): Add real-time data streaming and advanced chart features

use std::time::Instant;

/// Chart type for data visualization
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChartType {
    LineChart,
    BarChart,
    ScatterPlot,
    Gauge,
    Timeline,
}

impl ChartType {
    pub fn all() -> Vec<ChartType> {
        vec![
            ChartType::LineChart,
            ChartType::BarChart,
            ChartType::ScatterPlot,
            ChartType::Gauge,
            ChartType::Timeline,
        ]
    }

    pub fn name(&self) -> &'static str {
        match self {
            ChartType::LineChart => "Line Chart",
            ChartType::BarChart => "Bar Chart",
            ChartType::ScatterPlot => "Scatter Plot",
            ChartType::Gauge => "Gauge",
            ChartType::Timeline => "Timeline",
        }
    }

    pub fn description(&self) -> &'static str {
        match self {
            ChartType::LineChart => "Visualize time-series data with connected lines",
            ChartType::BarChart => "Compare categorical data with vertical or horizontal bars",
            ChartType::ScatterPlot => "Display relationships between two variables",
            ChartType::Gauge => "Show progress or measurement against a target value",
            ChartType::Timeline => "Display events chronologically with markers",
        }
    }
}

/// Single data point for visualization
#[derive(Debug, Clone, Copy)]
pub struct DataPoint {
    pub x: f64,
    pub y: f64,
}

impl DataPoint {
    pub fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }
}

/// Time-series data for line charts and scatter plots
#[derive(Debug, Clone)]
pub struct TimeSeriesData {
    pub series_name: String,
    pub points: Vec<DataPoint>,
    pub color: u8, // Color index
}

impl TimeSeriesData {
    pub fn new(series_name: String, points: Vec<DataPoint>, color: u8) -> Self {
        Self {
            series_name,
            points,
            color,
        }
    }

    pub fn min_x(&self) -> f64 {
        self.points
            .iter()
            .map(|p| p.x)
            .min_by(|a, b| a.partial_cmp(b).unwrap())
            .unwrap_or(0.0)
    }

    pub fn max_x(&self) -> f64 {
        self.points
            .iter()
            .map(|p| p.x)
            .max_by(|a, b| a.partial_cmp(b).unwrap())
            .unwrap_or(100.0)
    }

    pub fn min_y(&self) -> f64 {
        self.points
            .iter()
            .map(|p| p.y)
            .min_by(|a, b| a.partial_cmp(b).unwrap())
            .unwrap_or(0.0)
    }

    pub fn max_y(&self) -> f64 {
        self.points
            .iter()
            .map(|p| p.y)
            .max_by(|a, b| a.partial_cmp(b).unwrap())
            .unwrap_or(100.0)
    }
}

/// Category data for bar charts
#[derive(Debug, Clone)]
pub struct CategoryData {
    pub label: String,
    pub value: f64,
    pub color: u8,
}

impl CategoryData {
    pub fn new(label: String, value: f64, color: u8) -> Self {
        Self {
            label,
            value,
            color,
        }
    }
}

/// Event data for timeline visualization
#[derive(Debug, Clone)]
pub struct EventData {
    pub timestamp: f64,
    pub label: String,
    pub description: String,
    pub severity: EventSeverity,
}

impl EventData {
    pub fn new(
        timestamp: f64,
        label: String,
        description: String,
        severity: EventSeverity,
    ) -> Self {
        Self {
            timestamp,
            label,
            description,
            severity,
        }
    }
}

/// Event severity for timeline visualization
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EventSeverity {
    Info,
    Warning,
    Error,
    Critical,
}

impl EventSeverity {
    pub fn color_index(&self) -> u8 {
        match self {
            EventSeverity::Info => 0,
            EventSeverity::Warning => 1,
            EventSeverity::Error => 2,
            EventSeverity::Critical => 3,
        }
    }

    pub fn label(&self) -> &'static str {
        match self {
            EventSeverity::Info => "INFO",
            EventSeverity::Warning => "WARN",
            EventSeverity::Error => "ERROR",
            EventSeverity::Critical => "CRITICAL",
        }
    }
}

/// Gauge data for gauge visualization
#[derive(Debug, Clone)]
pub struct GaugeData {
    pub label: String,
    pub current_value: f64,
    pub min_value: f64,
    pub max_value: f64,
    pub target_value: Option<f64>,
    pub unit: String,
}

impl GaugeData {
    pub fn new(
        label: String,
        current_value: f64,
        min_value: f64,
        max_value: f64,
        target_value: Option<f64>,
        unit: String,
    ) -> Self {
        Self {
            label,
            current_value,
            min_value,
            max_value,
            target_value,
            unit,
        }
    }

    pub fn percentage(&self) -> f64 {
        ((self.current_value - self.min_value) / (self.max_value - self.min_value) * 100.0)
            .clamp(0.0, 100.0)
    }

    pub fn is_at_target(&self) -> bool {
        if let Some(target) = self.target_value {
            (self.current_value - target).abs() < 0.01
        } else {
            false
        }
    }
}

/// Chart data enum to hold different data types
#[derive(Debug, Clone)]
pub enum ChartData {
    TimeSeries(Vec<TimeSeriesData>),
    Category(Vec<CategoryData>),
    Events(Vec<EventData>),
    Gauge(GaugeData),
}

impl ChartData {
    pub fn create_mock_line_chart_data() -> Self {
        let series1 = TimeSeriesData::new(
            "CPU Usage".to_string(),
            (0..50)
                .map(|i| {
                    let x = i as f64;
                    let y = 50.0 + (i as f64 * 0.5).sin() * 20.0 + (i as f64 * 0.1).cos() * 10.0;
                    DataPoint::new(x, y)
                })
                .collect(),
            0,
        );

        let series2 = TimeSeriesData::new(
            "Memory Usage".to_string(),
            (0..50)
                .map(|i| {
                    let x = i as f64;
                    let y = 60.0 + (i as f64 * 0.3).sin() * 15.0 + (i as f64 * 0.2).cos() * 8.0;
                    DataPoint::new(x, y)
                })
                .collect(),
            1,
        );

        ChartData::TimeSeries(vec![series1, series2])
    }

    pub fn create_mock_bar_chart_data() -> Self {
        ChartData::Category(vec![
            CategoryData::new("Node A".to_string(), 85.0, 0),
            CategoryData::new("Node B".to_string(), 65.0, 1),
            CategoryData::new("Node C".to_string(), 92.0, 2),
            CategoryData::new("Node D".to_string(), 73.0, 3),
            CategoryData::new("Node E".to_string(), 88.0, 4),
            CategoryData::new("Node F".to_string(), 56.0, 0),
        ])
    }

    pub fn create_mock_scatter_plot_data() -> Self {
        let series = TimeSeriesData::new(
            "Latency vs Throughput".to_string(),
            (0..40)
                .map(|i| {
                    let x = (i as f64 * 2.5) + (i as f64 * 0.1).sin() * 5.0;
                    let y = 100.0 - (i as f64 * 2.0) + (i as f64 * 0.2).cos() * 10.0;
                    DataPoint::new(x, y)
                })
                .collect(),
            0,
        );

        ChartData::TimeSeries(vec![series])
    }

    pub fn create_mock_gauge_data() -> Self {
        ChartData::Gauge(GaugeData::new(
            "System Performance".to_string(),
            78.5,
            0.0,
            100.0,
            Some(80.0),
            "%".to_string(),
        ))
    }

    pub fn create_mock_timeline_data() -> Self {
        ChartData::Events(vec![
            EventData::new(
                0.0,
                "System Start".to_string(),
                "Dora system initialized".to_string(),
                EventSeverity::Info,
            ),
            EventData::new(
                5.0,
                "Dataflow Loaded".to_string(),
                "example.yaml loaded successfully".to_string(),
                EventSeverity::Info,
            ),
            EventData::new(
                12.0,
                "High CPU".to_string(),
                "CPU usage exceeded 80%".to_string(),
                EventSeverity::Warning,
            ),
            EventData::new(
                18.0,
                "Node Failure".to_string(),
                "Node 'processor' stopped responding".to_string(),
                EventSeverity::Error,
            ),
            EventData::new(
                25.0,
                "Recovery".to_string(),
                "Node 'processor' restarted".to_string(),
                EventSeverity::Info,
            ),
            EventData::new(
                32.0,
                "Memory Critical".to_string(),
                "Memory usage at 95%".to_string(),
                EventSeverity::Critical,
            ),
            EventData::new(
                40.0,
                "System Stable".to_string(),
                "All metrics within normal range".to_string(),
                EventSeverity::Info,
            ),
        ])
    }
}

/// State for data visualization view
#[derive(Debug, Clone)]
pub struct DataVizState {
    pub current_chart_type: ChartType,
    pub chart_data: ChartData,
    pub zoom_level: f64,
    pub scroll_offset: f64,
    pub last_refresh: Instant,
}

impl DataVizState {
    pub fn new() -> Self {
        Self {
            current_chart_type: ChartType::LineChart,
            chart_data: ChartData::create_mock_line_chart_data(),
            zoom_level: 1.0,
            scroll_offset: 0.0,
            last_refresh: Instant::now(),
        }
    }

    pub fn next_chart_type(&mut self) {
        let all_types = ChartType::all();
        let current_index = all_types
            .iter()
            .position(|t| *t == self.current_chart_type)
            .unwrap_or(0);
        let next_index = (current_index + 1) % all_types.len();
        self.current_chart_type = all_types[next_index];
        self.load_chart_data();
    }

    pub fn previous_chart_type(&mut self) {
        let all_types = ChartType::all();
        let current_index = all_types
            .iter()
            .position(|t| *t == self.current_chart_type)
            .unwrap_or(0);
        let prev_index = if current_index == 0 {
            all_types.len() - 1
        } else {
            current_index - 1
        };
        self.current_chart_type = all_types[prev_index];
        self.load_chart_data();
    }

    pub fn load_chart_data(&mut self) {
        self.chart_data = match self.current_chart_type {
            ChartType::LineChart => ChartData::create_mock_line_chart_data(),
            ChartType::BarChart => ChartData::create_mock_bar_chart_data(),
            ChartType::ScatterPlot => ChartData::create_mock_scatter_plot_data(),
            ChartType::Gauge => ChartData::create_mock_gauge_data(),
            ChartType::Timeline => ChartData::create_mock_timeline_data(),
        };
        self.zoom_level = 1.0;
        self.scroll_offset = 0.0;
        self.last_refresh = Instant::now();
    }

    pub fn zoom_in(&mut self) {
        self.zoom_level = (self.zoom_level * 1.2).min(5.0);
    }

    pub fn zoom_out(&mut self) {
        self.zoom_level = (self.zoom_level / 1.2).max(0.5);
    }

    pub fn reset_zoom(&mut self) {
        self.zoom_level = 1.0;
        self.scroll_offset = 0.0;
    }

    pub fn scroll_up(&mut self) {
        self.scroll_offset = (self.scroll_offset - 5.0).max(0.0);
    }

    pub fn scroll_down(&mut self) {
        self.scroll_offset = (self.scroll_offset + 5.0).min(100.0);
    }

    pub fn refresh(&mut self) {
        self.load_chart_data();
    }
}

impl Default for DataVizState {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
    fn test_chart_type_name() {
        assert_eq!(ChartType::LineChart.name(), "Line Chart");
        assert_eq!(ChartType::BarChart.name(), "Bar Chart");
        assert_eq!(ChartType::ScatterPlot.name(), "Scatter Plot");
        assert_eq!(ChartType::Gauge.name(), "Gauge");
        assert_eq!(ChartType::Timeline.name(), "Timeline");
    }

    #[test]
    fn test_data_point() {
        let point = DataPoint::new(10.0, 20.0);
        assert_eq!(point.x, 10.0);
        assert_eq!(point.y, 20.0);
    }

    #[test]
    fn test_time_series_data_bounds() {
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
    fn test_category_data() {
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
    }

    #[test]
    fn test_event_severity_label() {
        assert_eq!(EventSeverity::Info.label(), "INFO");
        assert_eq!(EventSeverity::Warning.label(), "WARN");
        assert_eq!(EventSeverity::Error.label(), "ERROR");
        assert_eq!(EventSeverity::Critical.label(), "CRITICAL");
    }

    #[test]
    fn test_gauge_data_percentage() {
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
    fn test_gauge_data_is_at_target() {
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
    fn test_mock_line_chart_data() {
        if let ChartData::TimeSeries(series) = ChartData::create_mock_line_chart_data() {
            assert_eq!(series.len(), 2);
            assert_eq!(series[0].series_name, "CPU Usage");
            assert_eq!(series[1].series_name, "Memory Usage");
            assert_eq!(series[0].points.len(), 50);
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
        } else {
            panic!("Expected Events data");
        }
    }

    #[test]
    fn test_data_viz_state_new() {
        let state = DataVizState::new();
        assert_eq!(state.current_chart_type, ChartType::LineChart);
        assert_eq!(state.zoom_level, 1.0);
        assert_eq!(state.scroll_offset, 0.0);
    }

    #[test]
    fn test_data_viz_state_next_chart_type() {
        let mut state = DataVizState::new();
        assert_eq!(state.current_chart_type, ChartType::LineChart);

        state.next_chart_type();
        assert_eq!(state.current_chart_type, ChartType::BarChart);

        state.next_chart_type();
        assert_eq!(state.current_chart_type, ChartType::ScatterPlot);

        state.next_chart_type();
        assert_eq!(state.current_chart_type, ChartType::Gauge);

        state.next_chart_type();
        assert_eq!(state.current_chart_type, ChartType::Timeline);

        state.next_chart_type();
        assert_eq!(state.current_chart_type, ChartType::LineChart); // Wrap around
    }

    #[test]
    fn test_data_viz_state_previous_chart_type() {
        let mut state = DataVizState::new();
        assert_eq!(state.current_chart_type, ChartType::LineChart);

        state.previous_chart_type();
        assert_eq!(state.current_chart_type, ChartType::Timeline); // Wrap around

        state.previous_chart_type();
        assert_eq!(state.current_chart_type, ChartType::Gauge);
    }

    #[test]
    fn test_data_viz_state_zoom() {
        let mut state = DataVizState::new();
        assert_eq!(state.zoom_level, 1.0);

        state.zoom_in();
        assert!(state.zoom_level > 1.0);

        state.zoom_out();
        assert_eq!(state.zoom_level, 1.0);

        state.zoom_out();
        assert!(state.zoom_level < 1.0);
    }

    #[test]
    fn test_data_viz_state_zoom_limits() {
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

    #[test]
    fn test_data_viz_state_reset_zoom() {
        let mut state = DataVizState::new();
        state.zoom_in();
        state.scroll_offset = 50.0;

        state.reset_zoom();
        assert_eq!(state.zoom_level, 1.0);
        assert_eq!(state.scroll_offset, 0.0);
    }

    #[test]
    fn test_data_viz_state_scroll() {
        let mut state = DataVizState::new();
        assert_eq!(state.scroll_offset, 0.0);

        state.scroll_down();
        assert_eq!(state.scroll_offset, 5.0);

        state.scroll_up();
        assert_eq!(state.scroll_offset, 0.0);

        state.scroll_up(); // Should not go below 0
        assert_eq!(state.scroll_offset, 0.0);
    }

    #[test]
    fn test_data_viz_state_scroll_limits() {
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
}
