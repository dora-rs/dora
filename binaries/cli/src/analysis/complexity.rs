// Result Complexity Analysis for Issue #19

use super::types::*;
use eyre::Result;

/// Result complexity analysis for interface selection
#[derive(Debug, Clone)]
pub struct ResultComplexity {
    pub overall_score: f32,
    pub data_volume_score: f32,
    pub dimension_score: f32,
    pub insight_score: f32,
    pub problem_score: f32,
    pub visualization_score: f32,
    pub factors: Vec<ComplexityFactor>,
}

#[derive(Debug, Clone)]
pub struct ComplexityFactor {
    pub factor_type: FactorType,
    pub impact: f32,
    pub description: String,
    pub evidence: Vec<String>,
}

#[derive(Debug, Clone)]
pub enum FactorType {
    DataVolume,
    AnalysisDimensions,
    InsightComplexity,
    ProblemComplexity,
    VisualizationComplexity,
    TrendComplexity,
}

/// Complexity analyzer for analysis results
pub struct ResultComplexityAnalyzer;

impl ResultComplexityAnalyzer {
    pub fn new() -> Self {
        Self
    }

    pub fn analyze_complexity(
        &self,
        results: &AnalysisResults,
        live_mode: bool,
        compare_baseline: bool,
    ) -> Result<ResultComplexity> {
        let mut complexity_score = 0.0;
        let mut complexity_factors = Vec::new();

        // Data volume complexity
        let data_points = results.session_info.total_data_points;
        let volume_complexity = match data_points {
            0..=1000 => 1.0,
            1001..=10000 => 2.0,
            10001..=100000 => 4.0,
            _ => 6.0,
        };
        complexity_score += volume_complexity;

        if data_points > 10000 {
            complexity_factors.push(ComplexityFactor {
                factor_type: FactorType::DataVolume,
                impact: volume_complexity,
                description: format!("Large dataset with {} data points", data_points),
                evidence: vec![format!("{} metrics analyzed", data_points)],
            });
        }

        // Analysis dimension complexity
        let analysis_dimensions = self.count_analysis_dimensions(results);
        let dimension_complexity = analysis_dimensions as f32 * 0.5;
        complexity_score += dimension_complexity;

        if analysis_dimensions > 4 {
            complexity_factors.push(ComplexityFactor {
                factor_type: FactorType::AnalysisDimensions,
                impact: dimension_complexity,
                description: format!(
                    "Multi-dimensional analysis across {} areas",
                    analysis_dimensions
                ),
                evidence: self.get_analysis_dimension_names(results),
            });
        }

        // Result insight complexity
        let insights_count = results.insights.len();
        let insight_complexity = match insights_count {
            0..=3 => 0.0,
            4..=8 => 2.0,
            9..=15 => 4.0,
            _ => 6.0,
        };
        complexity_score += insight_complexity;

        // Anomaly and issue complexity
        let anomaly_count = self.count_anomalies(results);
        let issue_count = self.count_issues(results);
        let problem_complexity = (anomaly_count + issue_count) as f32 * 0.5;
        complexity_score += problem_complexity;

        if anomaly_count > 0 || issue_count > 0 {
            complexity_factors.push(ComplexityFactor {
                factor_type: FactorType::ProblemComplexity,
                impact: problem_complexity,
                description: format!(
                    "{} anomalies and {} issues detected",
                    anomaly_count, issue_count
                ),
                evidence: vec![
                    format!("Anomalies: {}", anomaly_count),
                    format!("Issues: {}", issue_count),
                ],
            });
        }

        // Visualization complexity
        let visualization_complexity = self.assess_visualization_complexity(results, live_mode);
        complexity_score += visualization_complexity;

        // Trend complexity
        if let Some(trend_analysis) = &results.trend_analysis {
            let trend_complexity = trend_analysis.trends.len() as f32 * 0.3;
            complexity_score += trend_complexity;

            if trend_analysis.trends.len() > 5 {
                complexity_factors.push(ComplexityFactor {
                    factor_type: FactorType::TrendComplexity,
                    impact: trend_complexity,
                    description: format!(
                        "{} trends identified requiring analysis",
                        trend_analysis.trends.len()
                    ),
                    evidence: trend_analysis
                        .trends
                        .iter()
                        .take(3)
                        .map(|t| format!("{}: {:?}", t.metric_name, t.direction))
                        .collect(),
                });
            }
        }

        // Comparative analysis adds complexity
        if compare_baseline {
            complexity_score += 2.0;
        }

        Ok(ResultComplexity {
            overall_score: complexity_score.min(10.0),
            data_volume_score: volume_complexity,
            dimension_score: dimension_complexity,
            insight_score: insight_complexity,
            problem_score: problem_complexity,
            visualization_score: visualization_complexity,
            factors: complexity_factors,
        })
    }

    fn count_analysis_dimensions(&self, results: &AnalysisResults) -> usize {
        let mut dimensions = 0;

        if results.performance_analysis.is_some() {
            dimensions += 1;
        }
        if results.health_analysis.is_some() {
            dimensions += 1;
        }
        if results.efficiency_analysis.is_some() {
            dimensions += 1;
        }
        if results.trend_analysis.is_some() {
            dimensions += 1;
        }
        if results.pattern_analysis.is_some() {
            dimensions += 1;
        }
        if results.security_analysis.is_some() {
            dimensions += 1;
        }
        if results.predictive_analysis.is_some() {
            dimensions += 1;
        }
        if results.comparative_analysis.is_some() {
            dimensions += 1;
        }

        dimensions
    }

    fn get_analysis_dimension_names(&self, results: &AnalysisResults) -> Vec<String> {
        let mut names = Vec::new();

        if results.performance_analysis.is_some() {
            names.push("Performance".to_string());
        }
        if results.health_analysis.is_some() {
            names.push("Health".to_string());
        }
        if results.efficiency_analysis.is_some() {
            names.push("Efficiency".to_string());
        }
        if results.trend_analysis.is_some() {
            names.push("Trends".to_string());
        }
        if results.pattern_analysis.is_some() {
            names.push("Patterns".to_string());
        }
        if results.security_analysis.is_some() {
            names.push("Security".to_string());
        }
        if results.predictive_analysis.is_some() {
            names.push("Predictive".to_string());
        }
        if results.comparative_analysis.is_some() {
            names.push("Comparative".to_string());
        }

        names
    }

    fn count_anomalies(&self, results: &AnalysisResults) -> usize {
        results
            .trend_analysis
            .as_ref()
            .map(|t| t.anomalies.len())
            .unwrap_or(0)
    }

    fn count_issues(&self, results: &AnalysisResults) -> usize {
        let mut total = 0;

        if let Some(perf) = &results.performance_analysis {
            total += perf.issues.len();
        }
        if let Some(health) = &results.health_analysis {
            total += health.health_issues.len();
        }

        total
    }

    fn assess_visualization_complexity(&self, results: &AnalysisResults, live_mode: bool) -> f32 {
        let mut complexity = 0.0;

        // Time series data complexity
        if let Some(performance) = &results.performance_analysis {
            complexity += performance.throughput.time_series.len() as f32 * 0.001;
            complexity += performance.latency.time_series.len() as f32 * 0.001;
        }

        // Multi-dimensional visualization complexity
        if results.comparative_analysis.is_some() {
            complexity += 2.0; // Comparative visualizations are complex
        }

        if let Some(trends) = &results.trend_analysis {
            if trends.trends.len() > 3 {
                complexity += 1.5; // Multiple trend visualizations
            }
        }

        // Real-time complexity
        if live_mode {
            complexity += 2.0; // Real-time updates add complexity
        }

        complexity.min(5.0)
    }
}

impl Default for ResultComplexityAnalyzer {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_complexity_calculation() {
        let analyzer = ResultComplexityAnalyzer::new();
        let mut results = AnalysisResults::new(&AnalysisSession::default());

        // Add some complexity
        results.session_info.total_data_points = 50000;
        results.performance_analysis = Some(PerformanceAnalysisResult {
            throughput: ThroughputAnalysis::empty(),
            latency: LatencyAnalysis {
                current: 0.0,
                average: 0.0,
                percentiles: Percentiles::default(),
                trend: TrendDirection::Stable,
                time_series: Vec::new(),
            },
            resource_usage: ResourceUsageAnalysis {
                cpu_usage: 0.0,
                memory_usage: 0.0,
                disk_io: 0.0,
                network_io: 0.0,
                resource_efficiency: 0.0,
            },
            bottlenecks: Vec::new(),
            performance_scores: PerformanceScores::default(),
            issues: Vec::new(),
            insights: Vec::new(),
            analysis_metadata: AnalysisMetadata {
                analysis_duration_secs: 0.0,
                data_points_analyzed: 0,
                time_range: TimeWindow {
                    start: chrono::Utc::now(),
                    end: chrono::Utc::now(),
                    duration_description: "1h".to_string(),
                },
            },
        });

        let complexity = analyzer
            .analyze_complexity(&results, false, false)
            .unwrap();

        assert!(complexity.overall_score > 0.0);
        assert!(complexity.data_volume_score > 0.0);
    }

    #[test]
    fn test_dimension_counting() {
        let analyzer = ResultComplexityAnalyzer::new();
        let mut results = AnalysisResults::new(&AnalysisSession::default());

        results.performance_analysis = Some(PerformanceAnalysisResult {
            throughput: ThroughputAnalysis::empty(),
            latency: LatencyAnalysis {
                current: 0.0,
                average: 0.0,
                percentiles: Percentiles::default(),
                trend: TrendDirection::Stable,
                time_series: Vec::new(),
            },
            resource_usage: ResourceUsageAnalysis {
                cpu_usage: 0.0,
                memory_usage: 0.0,
                disk_io: 0.0,
                network_io: 0.0,
                resource_efficiency: 0.0,
            },
            bottlenecks: Vec::new(),
            performance_scores: PerformanceScores::default(),
            issues: Vec::new(),
            insights: Vec::new(),
            analysis_metadata: AnalysisMetadata {
                analysis_duration_secs: 0.0,
                data_points_analyzed: 0,
                time_range: TimeWindow {
                    start: chrono::Utc::now(),
                    end: chrono::Utc::now(),
                    duration_description: "1h".to_string(),
                },
            },
        });
        results.health_analysis = Some(HealthAnalysisResult {
            overall_health_score: 0.0,
            component_health: Vec::new(),
            health_issues: Vec::new(),
            health_trend: TrendDirection::Stable,
            recommendations: Vec::new(),
        });

        let dimensions = analyzer.count_analysis_dimensions(&results);
        assert_eq!(dimensions, 2);
    }
}
