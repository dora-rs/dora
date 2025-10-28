// Interactive Analysis Tools Types - Phase 1: Simplified Implementation with Mock Data
// TODO(Issue #33 Phase 2): Add real statistical tests, ML tools, and automated insight generation

use std::time::Instant;

/// Analysis type for interactive data exploration
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AnalysisType {
    Distribution,
    Correlation,
    Trend,
    Outlier,
}

impl AnalysisType {
    pub fn all() -> Vec<AnalysisType> {
        vec![
            AnalysisType::Distribution,
            AnalysisType::Correlation,
            AnalysisType::Trend,
            AnalysisType::Outlier,
        ]
    }

    pub fn name(&self) -> &'static str {
        match self {
            AnalysisType::Distribution => "Distribution Analysis",
            AnalysisType::Correlation => "Correlation Analysis",
            AnalysisType::Trend => "Trend Analysis",
            AnalysisType::Outlier => "Outlier Detection",
        }
    }

    pub fn description(&self) -> &'static str {
        match self {
            AnalysisType::Distribution => {
                "Statistical distribution analysis with summary statistics"
            }
            AnalysisType::Correlation => {
                "Correlation matrix showing relationships between variables"
            }
            AnalysisType::Trend => "Trend detection and pattern analysis over time",
            AnalysisType::Outlier => "Outlier detection using statistical methods",
        }
    }
}

/// Distribution statistics for a variable
#[derive(Debug, Clone)]
pub struct DistributionStats {
    pub variable_name: String,
    pub sample_size: usize,
    pub mean: f64,
    pub median: f64,
    pub std_dev: f64,
    pub min: f64,
    pub max: f64,
    pub q1: f64,
    pub q3: f64,
    pub skewness: f64,
    pub kurtosis: f64,
}

impl DistributionStats {
    pub fn create_mock_cpu_stats() -> Self {
        Self {
            variable_name: "CPU Usage (%)".to_string(),
            sample_size: 1000,
            mean: 45.3,
            median: 42.5,
            std_dev: 12.8,
            min: 15.2,
            max: 89.7,
            q1: 35.8,
            q3: 55.2,
            skewness: 0.45,
            kurtosis: -0.23,
        }
    }

    pub fn create_mock_memory_stats() -> Self {
        Self {
            variable_name: "Memory Usage (%)".to_string(),
            sample_size: 1000,
            mean: 62.1,
            median: 60.3,
            std_dev: 15.4,
            min: 25.8,
            max: 95.2,
            q1: 50.5,
            q3: 73.8,
            skewness: 0.18,
            kurtosis: -0.45,
        }
    }

    pub fn create_mock_latency_stats() -> Self {
        Self {
            variable_name: "Latency (ms)".to_string(),
            sample_size: 1000,
            mean: 125.7,
            median: 98.2,
            std_dev: 68.3,
            min: 12.5,
            max: 520.8,
            q1: 72.3,
            q3: 145.6,
            skewness: 1.23,
            kurtosis: 2.15,
        }
    }
}

/// Correlation between two variables
#[derive(Debug, Clone)]
pub struct CorrelationPair {
    pub variable1: String,
    pub variable2: String,
    pub coefficient: f64,
    pub strength: CorrelationStrength,
    pub p_value: f64,
    pub is_significant: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CorrelationStrength {
    VeryWeak,
    Weak,
    Moderate,
    Strong,
    VeryStrong,
}

impl CorrelationStrength {
    pub fn from_coefficient(coef: f64) -> Self {
        let abs_coef = coef.abs();
        if abs_coef < 0.2 {
            CorrelationStrength::VeryWeak
        } else if abs_coef < 0.4 {
            CorrelationStrength::Weak
        } else if abs_coef < 0.6 {
            CorrelationStrength::Moderate
        } else if abs_coef < 0.8 {
            CorrelationStrength::Strong
        } else {
            CorrelationStrength::VeryStrong
        }
    }

    pub fn label(&self) -> &'static str {
        match self {
            CorrelationStrength::VeryWeak => "Very Weak",
            CorrelationStrength::Weak => "Weak",
            CorrelationStrength::Moderate => "Moderate",
            CorrelationStrength::Strong => "Strong",
            CorrelationStrength::VeryStrong => "Very Strong",
        }
    }
}

impl CorrelationPair {
    pub fn create_mock_correlations() -> Vec<Self> {
        vec![
            CorrelationPair {
                variable1: "CPU Usage".to_string(),
                variable2: "Memory Usage".to_string(),
                coefficient: 0.72,
                strength: CorrelationStrength::from_coefficient(0.72),
                p_value: 0.001,
                is_significant: true,
            },
            CorrelationPair {
                variable1: "CPU Usage".to_string(),
                variable2: "Latency".to_string(),
                coefficient: 0.45,
                strength: CorrelationStrength::from_coefficient(0.45),
                p_value: 0.012,
                is_significant: true,
            },
            CorrelationPair {
                variable1: "Memory Usage".to_string(),
                variable2: "Latency".to_string(),
                coefficient: 0.38,
                strength: CorrelationStrength::from_coefficient(0.38),
                p_value: 0.025,
                is_significant: true,
            },
            CorrelationPair {
                variable1: "CPU Usage".to_string(),
                variable2: "Throughput".to_string(),
                coefficient: -0.58,
                strength: CorrelationStrength::from_coefficient(-0.58),
                p_value: 0.003,
                is_significant: true,
            },
            CorrelationPair {
                variable1: "Memory Usage".to_string(),
                variable2: "Throughput".to_string(),
                coefficient: -0.42,
                strength: CorrelationStrength::from_coefficient(-0.42),
                p_value: 0.018,
                is_significant: true,
            },
        ]
    }
}

/// Trend analysis result
#[derive(Debug, Clone)]
pub struct TrendAnalysis {
    pub variable_name: String,
    pub trend_direction: TrendDirection,
    pub trend_strength: f64,
    pub confidence: f64,
    pub change_rate: f64,
    pub prediction: Option<f64>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TrendDirection {
    Increasing,
    Decreasing,
    Stable,
    Volatile,
}

impl TrendDirection {
    pub fn label(&self) -> &'static str {
        match self {
            TrendDirection::Increasing => "Increasing",
            TrendDirection::Decreasing => "Decreasing",
            TrendDirection::Stable => "Stable",
            TrendDirection::Volatile => "Volatile",
        }
    }

    pub fn icon(&self) -> &'static str {
        match self {
            TrendDirection::Increasing => "↗",
            TrendDirection::Decreasing => "↘",
            TrendDirection::Stable => "→",
            TrendDirection::Volatile => "↕",
        }
    }
}

impl TrendAnalysis {
    pub fn create_mock_trends() -> Vec<Self> {
        vec![
            TrendAnalysis {
                variable_name: "CPU Usage".to_string(),
                trend_direction: TrendDirection::Increasing,
                trend_strength: 0.68,
                confidence: 0.85,
                change_rate: 2.3,
                prediction: Some(52.8),
            },
            TrendAnalysis {
                variable_name: "Memory Usage".to_string(),
                trend_direction: TrendDirection::Stable,
                trend_strength: 0.12,
                confidence: 0.92,
                change_rate: 0.3,
                prediction: Some(62.5),
            },
            TrendAnalysis {
                variable_name: "Latency".to_string(),
                trend_direction: TrendDirection::Volatile,
                trend_strength: 0.45,
                confidence: 0.65,
                change_rate: 8.5,
                prediction: Some(135.2),
            },
            TrendAnalysis {
                variable_name: "Throughput".to_string(),
                trend_direction: TrendDirection::Decreasing,
                trend_strength: 0.58,
                confidence: 0.78,
                change_rate: -3.2,
                prediction: Some(245.5),
            },
        ]
    }
}

/// Outlier detection result
#[derive(Debug, Clone)]
pub struct OutlierData {
    pub variable_name: String,
    pub total_points: usize,
    pub outlier_count: usize,
    pub outlier_percentage: f64,
    pub outliers: Vec<OutlierPoint>,
    pub detection_method: String,
}

#[derive(Debug, Clone)]
pub struct OutlierPoint {
    pub index: usize,
    pub value: f64,
    pub z_score: f64,
    pub severity: OutlierSeverity,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OutlierSeverity {
    Mild,
    Moderate,
    Severe,
    Extreme,
}

impl OutlierSeverity {
    pub fn from_z_score(z: f64) -> Self {
        let abs_z = z.abs();
        if abs_z < 2.0 {
            OutlierSeverity::Mild
        } else if abs_z < 2.5 {
            OutlierSeverity::Moderate
        } else if abs_z < 3.0 {
            OutlierSeverity::Severe
        } else {
            OutlierSeverity::Extreme
        }
    }

    pub fn label(&self) -> &'static str {
        match self {
            OutlierSeverity::Mild => "Mild",
            OutlierSeverity::Moderate => "Moderate",
            OutlierSeverity::Severe => "Severe",
            OutlierSeverity::Extreme => "Extreme",
        }
    }

    pub fn color_index(&self) -> u8 {
        match self {
            OutlierSeverity::Mild => 0,
            OutlierSeverity::Moderate => 1,
            OutlierSeverity::Severe => 2,
            OutlierSeverity::Extreme => 3,
        }
    }
}

impl OutlierData {
    pub fn create_mock_outliers() -> Vec<Self> {
        vec![
            OutlierData {
                variable_name: "CPU Usage".to_string(),
                total_points: 1000,
                outlier_count: 23,
                outlier_percentage: 2.3,
                outliers: vec![
                    OutlierPoint {
                        index: 45,
                        value: 89.7,
                        z_score: 3.47,
                        severity: OutlierSeverity::from_z_score(3.47),
                    },
                    OutlierPoint {
                        index: 123,
                        value: 15.2,
                        z_score: -2.35,
                        severity: OutlierSeverity::from_z_score(-2.35),
                    },
                    OutlierPoint {
                        index: 567,
                        value: 87.3,
                        z_score: 3.28,
                        severity: OutlierSeverity::from_z_score(3.28),
                    },
                ],
                detection_method: "Z-Score (threshold: 2.0)".to_string(),
            },
            OutlierData {
                variable_name: "Latency".to_string(),
                total_points: 1000,
                outlier_count: 42,
                outlier_percentage: 4.2,
                outliers: vec![
                    OutlierPoint {
                        index: 78,
                        value: 520.8,
                        z_score: 5.78,
                        severity: OutlierSeverity::from_z_score(5.78),
                    },
                    OutlierPoint {
                        index: 234,
                        value: 485.2,
                        z_score: 5.26,
                        severity: OutlierSeverity::from_z_score(5.26),
                    },
                    OutlierPoint {
                        index: 456,
                        value: 12.5,
                        z_score: -1.66,
                        severity: OutlierSeverity::from_z_score(-1.66),
                    },
                ],
                detection_method: "Z-Score (threshold: 2.0)".to_string(),
            },
        ]
    }
}

/// Analysis results enum
#[derive(Debug, Clone)]
pub enum AnalysisResults {
    Distribution(Vec<DistributionStats>),
    Correlation(Vec<CorrelationPair>),
    Trend(Vec<TrendAnalysis>),
    Outlier(Vec<OutlierData>),
}

impl AnalysisResults {
    pub fn create_mock_distribution() -> Self {
        AnalysisResults::Distribution(vec![
            DistributionStats::create_mock_cpu_stats(),
            DistributionStats::create_mock_memory_stats(),
            DistributionStats::create_mock_latency_stats(),
        ])
    }

    pub fn create_mock_correlation() -> Self {
        AnalysisResults::Correlation(CorrelationPair::create_mock_correlations())
    }

    pub fn create_mock_trend() -> Self {
        AnalysisResults::Trend(TrendAnalysis::create_mock_trends())
    }

    pub fn create_mock_outlier() -> Self {
        AnalysisResults::Outlier(OutlierData::create_mock_outliers())
    }
}

/// State for analysis tools view
#[derive(Debug, Clone)]
pub struct AnalysisToolsState {
    pub current_analysis: AnalysisType,
    pub results: AnalysisResults,
    pub selected_item: usize,
    pub scroll_offset: usize,
    pub last_refresh: Instant,
}

impl AnalysisToolsState {
    pub fn new() -> Self {
        Self {
            current_analysis: AnalysisType::Distribution,
            results: AnalysisResults::create_mock_distribution(),
            selected_item: 0,
            scroll_offset: 0,
            last_refresh: Instant::now(),
        }
    }

    pub fn next_analysis(&mut self) {
        let all_types = AnalysisType::all();
        let current_index = all_types
            .iter()
            .position(|t| *t == self.current_analysis)
            .unwrap_or(0);
        let next_index = (current_index + 1) % all_types.len();
        self.current_analysis = all_types[next_index];
        self.load_analysis_results();
        self.selected_item = 0;
        self.scroll_offset = 0;
    }

    pub fn previous_analysis(&mut self) {
        let all_types = AnalysisType::all();
        let current_index = all_types
            .iter()
            .position(|t| *t == self.current_analysis)
            .unwrap_or(0);
        let prev_index = if current_index == 0 {
            all_types.len() - 1
        } else {
            current_index - 1
        };
        self.current_analysis = all_types[prev_index];
        self.load_analysis_results();
        self.selected_item = 0;
        self.scroll_offset = 0;
    }

    pub fn load_analysis_results(&mut self) {
        self.results = match self.current_analysis {
            AnalysisType::Distribution => AnalysisResults::create_mock_distribution(),
            AnalysisType::Correlation => AnalysisResults::create_mock_correlation(),
            AnalysisType::Trend => AnalysisResults::create_mock_trend(),
            AnalysisType::Outlier => AnalysisResults::create_mock_outlier(),
        };
        self.last_refresh = Instant::now();
    }

    pub fn select_next(&mut self) {
        let max_items = self.get_item_count();
        if max_items > 0 {
            self.selected_item = (self.selected_item + 1) % max_items;
        }
    }

    pub fn select_previous(&mut self) {
        let max_items = self.get_item_count();
        if max_items > 0 {
            self.selected_item = if self.selected_item == 0 {
                max_items - 1
            } else {
                self.selected_item - 1
            };
        }
    }

    pub fn scroll_down(&mut self) {
        self.scroll_offset = self.scroll_offset.saturating_add(1).min(20);
    }

    pub fn scroll_up(&mut self) {
        self.scroll_offset = self.scroll_offset.saturating_sub(1);
    }

    pub fn refresh(&mut self) {
        self.load_analysis_results();
    }

    fn get_item_count(&self) -> usize {
        match &self.results {
            AnalysisResults::Distribution(stats) => stats.len(),
            AnalysisResults::Correlation(pairs) => pairs.len(),
            AnalysisResults::Trend(trends) => trends.len(),
            AnalysisResults::Outlier(outliers) => outliers.len(),
        }
    }
}

impl Default for AnalysisToolsState {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_analysis_type_all() {
        let all = AnalysisType::all();
        assert_eq!(all.len(), 4);
        assert!(all.contains(&AnalysisType::Distribution));
        assert!(all.contains(&AnalysisType::Correlation));
        assert!(all.contains(&AnalysisType::Trend));
        assert!(all.contains(&AnalysisType::Outlier));
    }

    #[test]
    fn test_analysis_type_names() {
        assert_eq!(AnalysisType::Distribution.name(), "Distribution Analysis");
        assert_eq!(AnalysisType::Correlation.name(), "Correlation Analysis");
        assert_eq!(AnalysisType::Trend.name(), "Trend Analysis");
        assert_eq!(AnalysisType::Outlier.name(), "Outlier Detection");
    }

    #[test]
    fn test_distribution_stats_mock() {
        let stats = DistributionStats::create_mock_cpu_stats();
        assert_eq!(stats.variable_name, "CPU Usage (%)");
        assert_eq!(stats.sample_size, 1000);
        assert!(stats.mean > 0.0);
        assert!(stats.std_dev > 0.0);
    }

    #[test]
    fn test_correlation_strength_classification() {
        assert_eq!(
            CorrelationStrength::from_coefficient(0.1),
            CorrelationStrength::VeryWeak
        );
        assert_eq!(
            CorrelationStrength::from_coefficient(0.3),
            CorrelationStrength::Weak
        );
        assert_eq!(
            CorrelationStrength::from_coefficient(0.5),
            CorrelationStrength::Moderate
        );
        assert_eq!(
            CorrelationStrength::from_coefficient(0.7),
            CorrelationStrength::Strong
        );
        assert_eq!(
            CorrelationStrength::from_coefficient(0.9),
            CorrelationStrength::VeryStrong
        );
    }

    #[test]
    fn test_correlation_pairs_mock() {
        let pairs = CorrelationPair::create_mock_correlations();
        assert!(!pairs.is_empty());
        assert!(pairs.iter().all(|p| p.coefficient.abs() <= 1.0));
        assert!(pairs.iter().all(|p| p.p_value >= 0.0 && p.p_value <= 1.0));
    }

    #[test]
    fn test_trend_direction_labels() {
        assert_eq!(TrendDirection::Increasing.label(), "Increasing");
        assert_eq!(TrendDirection::Decreasing.label(), "Decreasing");
        assert_eq!(TrendDirection::Stable.label(), "Stable");
        assert_eq!(TrendDirection::Volatile.label(), "Volatile");
    }

    #[test]
    fn test_trend_direction_icons() {
        assert_eq!(TrendDirection::Increasing.icon(), "↗");
        assert_eq!(TrendDirection::Decreasing.icon(), "↘");
        assert_eq!(TrendDirection::Stable.icon(), "→");
        assert_eq!(TrendDirection::Volatile.icon(), "↕");
    }

    #[test]
    fn test_trend_analysis_mock() {
        let trends = TrendAnalysis::create_mock_trends();
        assert!(!trends.is_empty());
        assert!(
            trends
                .iter()
                .all(|t| t.confidence >= 0.0 && t.confidence <= 1.0)
        );
    }

    #[test]
    fn test_outlier_severity_from_z_score() {
        assert_eq!(OutlierSeverity::from_z_score(1.5), OutlierSeverity::Mild);
        assert_eq!(
            OutlierSeverity::from_z_score(2.3),
            OutlierSeverity::Moderate
        );
        assert_eq!(OutlierSeverity::from_z_score(2.8), OutlierSeverity::Severe);
        assert_eq!(OutlierSeverity::from_z_score(3.5), OutlierSeverity::Extreme);
    }

    #[test]
    fn test_outlier_data_mock() {
        let outliers = OutlierData::create_mock_outliers();
        assert!(!outliers.is_empty());
        assert!(outliers.iter().all(|o| o.outlier_count <= o.total_points));
    }

    #[test]
    fn test_analysis_tools_state_new() {
        let state = AnalysisToolsState::new();
        assert_eq!(state.current_analysis, AnalysisType::Distribution);
        assert_eq!(state.selected_item, 0);
        assert_eq!(state.scroll_offset, 0);
    }

    #[test]
    fn test_analysis_tools_state_navigation() {
        let mut state = AnalysisToolsState::new();
        assert_eq!(state.current_analysis, AnalysisType::Distribution);

        state.next_analysis();
        assert_eq!(state.current_analysis, AnalysisType::Correlation);

        state.next_analysis();
        assert_eq!(state.current_analysis, AnalysisType::Trend);

        state.next_analysis();
        assert_eq!(state.current_analysis, AnalysisType::Outlier);

        state.next_analysis();
        assert_eq!(state.current_analysis, AnalysisType::Distribution); // Wrap around
    }

    #[test]
    fn test_analysis_tools_state_previous_navigation() {
        let mut state = AnalysisToolsState::new();
        assert_eq!(state.current_analysis, AnalysisType::Distribution);

        state.previous_analysis();
        assert_eq!(state.current_analysis, AnalysisType::Outlier); // Wrap around

        state.previous_analysis();
        assert_eq!(state.current_analysis, AnalysisType::Trend);
    }

    #[test]
    fn test_analysis_tools_state_select_next() {
        let mut state = AnalysisToolsState::new();
        assert_eq!(state.selected_item, 0);

        state.select_next();
        assert_eq!(state.selected_item, 1);

        state.select_next();
        assert_eq!(state.selected_item, 2);
    }

    #[test]
    fn test_analysis_tools_state_select_previous() {
        let mut state = AnalysisToolsState::new();
        state.selected_item = 2;

        state.select_previous();
        assert_eq!(state.selected_item, 1);

        state.select_previous();
        assert_eq!(state.selected_item, 0);
    }

    #[test]
    fn test_analysis_tools_state_scroll() {
        let mut state = AnalysisToolsState::new();
        assert_eq!(state.scroll_offset, 0);

        state.scroll_down();
        assert_eq!(state.scroll_offset, 1);

        state.scroll_up();
        assert_eq!(state.scroll_offset, 0);
    }

    #[test]
    fn test_analysis_tools_state_refresh() {
        let mut state = AnalysisToolsState::new();
        let initial_time = state.last_refresh;

        std::thread::sleep(std::time::Duration::from_millis(10));
        state.refresh();

        assert!(state.last_refresh > initial_time);
    }

    #[test]
    fn test_mock_distribution_results() {
        let results = AnalysisResults::create_mock_distribution();
        if let AnalysisResults::Distribution(stats) = results {
            assert_eq!(stats.len(), 3);
            assert_eq!(stats[0].variable_name, "CPU Usage (%)");
            assert_eq!(stats[1].variable_name, "Memory Usage (%)");
            assert_eq!(stats[2].variable_name, "Latency (ms)");
        } else {
            panic!("Expected Distribution results");
        }
    }

    #[test]
    fn test_mock_correlation_results() {
        let results = AnalysisResults::create_mock_correlation();
        if let AnalysisResults::Correlation(pairs) = results {
            assert_eq!(pairs.len(), 5);
            assert!(pairs.iter().all(|p| p.is_significant));
        } else {
            panic!("Expected Correlation results");
        }
    }

    #[test]
    fn test_mock_trend_results() {
        let results = AnalysisResults::create_mock_trend();
        if let AnalysisResults::Trend(trends) = results {
            assert_eq!(trends.len(), 4);
            assert!(trends.iter().all(|t| t.prediction.is_some()));
        } else {
            panic!("Expected Trend results");
        }
    }

    #[test]
    fn test_mock_outlier_results() {
        let results = AnalysisResults::create_mock_outlier();
        if let AnalysisResults::Outlier(outliers) = results {
            assert_eq!(outliers.len(), 2);
            assert!(outliers.iter().all(|o| !o.outliers.is_empty()));
        } else {
            panic!("Expected Outlier results");
        }
    }
}
