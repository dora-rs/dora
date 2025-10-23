// Analysis Engines for Issue #19

use super::types::*;
use chrono::Utc;
use eyre::Result;
use std::time::Instant;

/// Collection of all analysis engines
#[derive(Debug)]
pub struct AnalysisEngines {
    pub performance: PerformanceAnalysisEngine,
    pub health: HealthAnalysisEngine,
    pub efficiency: EfficiencyAnalysisEngine,
    pub trends: TrendAnalysisEngine,
    pub patterns: PatternAnalysisEngine,
}

impl AnalysisEngines {
    pub fn new() -> Self {
        Self {
            performance: PerformanceAnalysisEngine::new(),
            health: HealthAnalysisEngine::new(),
            efficiency: EfficiencyAnalysisEngine::new(),
            trends: TrendAnalysisEngine::new(),
            patterns: PatternAnalysisEngine::new(),
        }
    }
}

impl Default for AnalysisEngines {
    fn default() -> Self {
        Self::new()
    }
}

/// Performance Analysis Engine
#[derive(Debug)]
pub struct PerformanceAnalysisEngine;

impl PerformanceAnalysisEngine {
    pub fn new() -> Self {
        Self
    }

    pub async fn analyze(&self, data: &DataCollection) -> Result<PerformanceAnalysisResult> {
        let start_time = Instant::now();

        // Analyze core performance metrics
        let throughput_analysis = self.analyze_throughput(&data.metrics).await?;
        let latency_analysis = self.analyze_latency(&data.metrics).await?;
        let resource_analysis = self.analyze_resource_usage(&data.metrics).await?;

        // Detect performance bottlenecks
        let bottlenecks = self.detect_bottlenecks(data).await?;

        // Calculate performance scores
        let performance_scores = self.calculate_performance_scores(
            &throughput_analysis,
            &latency_analysis,
            &resource_analysis,
        );

        // Identify performance issues
        let issues = self.identify_performance_issues(
            &throughput_analysis,
            &latency_analysis,
            &bottlenecks,
        );

        // Generate performance insights
        let insights = self.generate_performance_insights(
            &performance_scores,
            &bottlenecks,
            &issues,
        );

        let analysis_duration = start_time.elapsed();

        Ok(PerformanceAnalysisResult {
            throughput: throughput_analysis,
            latency: latency_analysis,
            resource_usage: resource_analysis,
            bottlenecks,
            performance_scores,
            issues,
            insights,
            analysis_metadata: AnalysisMetadata {
                analysis_duration_secs: analysis_duration.as_secs_f32(),
                data_points_analyzed: data.metrics.len(),
                time_range: data.time_range.clone(),
            },
        })
    }

    async fn analyze_throughput(&self, metrics: &[MetricPoint]) -> Result<ThroughputAnalysis> {
        let throughput_metrics: Vec<_> = metrics
            .iter()
            .filter(|m| {
                m.metric_name.contains("throughput") || m.metric_name.contains("messages_per_second")
            })
            .collect();

        if throughput_metrics.is_empty() {
            return Ok(ThroughputAnalysis::empty());
        }

        let current_throughput = throughput_metrics.last().map(|m| m.value).unwrap_or(0.0);

        let average_throughput = throughput_metrics.iter().map(|m| m.value).sum::<f32>()
            / throughput_metrics.len() as f32;

        let peak_throughput = throughput_metrics
            .iter()
            .map(|m| m.value)
            .fold(0.0f32, |acc, val| acc.max(val));

        let throughput_trend = self.calculate_throughput_trend(&throughput_metrics);
        let throughput_stability = self.calculate_throughput_stability(&throughput_metrics);

        Ok(ThroughputAnalysis {
            current: current_throughput,
            average: average_throughput,
            peak: peak_throughput,
            trend: throughput_trend,
            stability: throughput_stability,
            percentiles: self.calculate_throughput_percentiles(&throughput_metrics),
            time_series: throughput_metrics
                .iter()
                .map(|m| TimeSeriesPoint {
                    timestamp: m.timestamp,
                    value: m.value,
                })
                .collect(),
        })
    }

    fn calculate_throughput_trend(&self, _metrics: &[&MetricPoint]) -> TrendDirection {
        // Simplified trend calculation - would use proper time series analysis in production
        TrendDirection::Stable
    }

    fn calculate_throughput_stability(&self, metrics: &[&MetricPoint]) -> f32 {
        if metrics.len() < 2 {
            return 100.0;
        }

        let values: Vec<f32> = metrics.iter().map(|m| m.value).collect();
        let mean = values.iter().sum::<f32>() / values.len() as f32;
        let variance =
            values.iter().map(|v| (v - mean).powi(2)).sum::<f32>() / values.len() as f32;
        let std_dev = variance.sqrt();

        // Stability score: 100 - (coefficient of variation * 100)
        if mean > 0.0 {
            (100.0 - ((std_dev / mean) * 100.0)).max(0.0)
        } else {
            0.0
        }
    }

    fn calculate_throughput_percentiles(&self, metrics: &[&MetricPoint]) -> Percentiles {
        let mut values: Vec<f32> = metrics.iter().map(|m| m.value).collect();
        values.sort_by(|a, b| a.partial_cmp(b).unwrap());

        let len = values.len();
        if len == 0 {
            return Percentiles::default();
        }

        Percentiles {
            p50: values[len * 50 / 100],
            p75: values[len * 75 / 100],
            p90: values[len * 90 / 100],
            p95: values[len * 95 / 100],
            p99: values[len * 99 / 100],
        }
    }

    async fn analyze_latency(&self, metrics: &[MetricPoint]) -> Result<LatencyAnalysis> {
        let latency_metrics: Vec<_> = metrics
            .iter()
            .filter(|m| m.metric_name.contains("latency") || m.metric_name.contains("duration"))
            .collect();

        if latency_metrics.is_empty() {
            return Ok(LatencyAnalysis {
                current: 0.0,
                average: 0.0,
                percentiles: Percentiles::default(),
                trend: TrendDirection::Stable,
                time_series: Vec::new(),
            });
        }

        let current = latency_metrics.last().map(|m| m.value).unwrap_or(0.0);
        let average =
            latency_metrics.iter().map(|m| m.value).sum::<f32>() / latency_metrics.len() as f32;

        Ok(LatencyAnalysis {
            current,
            average,
            percentiles: self.calculate_latency_percentiles(&latency_metrics),
            trend: self.calculate_latency_trend(&latency_metrics),
            time_series: latency_metrics
                .iter()
                .map(|m| TimeSeriesPoint {
                    timestamp: m.timestamp,
                    value: m.value,
                })
                .collect(),
        })
    }

    fn calculate_latency_percentiles(&self, metrics: &[&MetricPoint]) -> Percentiles {
        let mut values: Vec<f32> = metrics.iter().map(|m| m.value).collect();
        values.sort_by(|a, b| a.partial_cmp(b).unwrap());

        let len = values.len();
        if len == 0 {
            return Percentiles::default();
        }

        Percentiles {
            p50: values[len * 50 / 100],
            p75: values[len * 75 / 100],
            p90: values[len * 90 / 100],
            p95: values[len * 95 / 100],
            p99: values[len * 99 / 100],
        }
    }

    fn calculate_latency_trend(&self, _metrics: &[&MetricPoint]) -> TrendDirection {
        TrendDirection::Stable
    }

    async fn analyze_resource_usage(&self, _metrics: &[MetricPoint]) -> Result<ResourceUsageAnalysis> {
        // Mock resource usage - would query actual system metrics in production
        Ok(ResourceUsageAnalysis {
            cpu_usage: 45.0,
            memory_usage: 60.0,
            disk_io: 25.0,
            network_io: 35.0,
            resource_efficiency: 75.0,
        })
    }

    async fn detect_bottlenecks(&self, _data: &DataCollection) -> Result<Vec<Bottleneck>> {
        // Mock bottleneck detection - would analyze actual data flow in production
        Ok(vec![])
    }

    fn calculate_performance_scores(
        &self,
        throughput: &ThroughputAnalysis,
        latency: &LatencyAnalysis,
        resource: &ResourceUsageAnalysis,
    ) -> PerformanceScores {
        // Simplified scoring algorithm
        let throughput_score = if throughput.peak > 0.0 {
            (throughput.average / throughput.peak * 100.0).min(100.0)
        } else {
            0.0
        };

        let latency_score = if latency.percentiles.p95 > 0.0 {
            (100.0 - (latency.percentiles.p95 / 1000.0 * 100.0)).max(0.0)
        } else {
            100.0
        };

        let resource_efficiency_score = resource.resource_efficiency;

        let overall_performance_score =
            (throughput_score + latency_score + resource_efficiency_score) / 3.0;

        PerformanceScores {
            throughput_score,
            latency_score,
            resource_efficiency_score,
            overall_performance_score,
        }
    }

    fn identify_performance_issues(
        &self,
        throughput: &ThroughputAnalysis,
        latency: &LatencyAnalysis,
        _bottlenecks: &[Bottleneck],
    ) -> Vec<PerformanceIssue> {
        let mut issues = Vec::new();

        // Check for low throughput
        if throughput.current < throughput.average * 0.5 && throughput.average > 0.0 {
            issues.push(PerformanceIssue {
                issue_type: "Low Throughput".to_string(),
                severity: "High".to_string(),
                description: format!(
                    "Current throughput ({:.2}) is significantly below average ({:.2})",
                    throughput.current, throughput.average
                ),
                metrics: vec!["throughput".to_string()],
            });
        }

        // Check for high latency
        if latency.percentiles.p95 > 1000.0 {
            issues.push(PerformanceIssue {
                issue_type: "High Latency".to_string(),
                severity: "Medium".to_string(),
                description: format!(
                    "P95 latency is high: {:.1}ms",
                    latency.percentiles.p95
                ),
                metrics: vec!["latency".to_string()],
            });
        }

        issues
    }

    fn generate_performance_insights(
        &self,
        scores: &PerformanceScores,
        _bottlenecks: &[Bottleneck],
        issues: &[PerformanceIssue],
    ) -> Vec<PerformanceInsight> {
        let mut insights = Vec::new();

        if scores.overall_performance_score < 50.0 {
            insights.push(PerformanceInsight {
                title: "Overall Performance Below Average".to_string(),
                description: format!(
                    "System performance score is {:.1}/100",
                    scores.overall_performance_score
                ),
                impact: "High".to_string(),
            });
        }

        if !issues.is_empty() {
            insights.push(PerformanceInsight {
                title: format!("{} Performance Issues Detected", issues.len()),
                description: "Multiple performance issues require attention".to_string(),
                impact: "Medium".to_string(),
            });
        }

        insights
    }
}

impl Default for PerformanceAnalysisEngine {
    fn default() -> Self {
        Self::new()
    }
}

/// Trend Analysis Engine
#[derive(Debug)]
pub struct TrendAnalysisEngine;

impl TrendAnalysisEngine {
    pub fn new() -> Self {
        Self
    }

    pub async fn analyze(&self, data: &DataCollection) -> Result<TrendAnalysisResult> {
        let mut trends = Vec::new();

        // Group metrics by name
        let metric_groups = Self::group_metrics_by_name(&data.metrics);

        // Analyze trends for each metric
        for (metric_name, metric_data) in metric_groups {
            if let Some(trend) = self.analyze_metric_trend(&metric_name, &metric_data).await? {
                trends.push(trend);
            }
        }

        // Generate trend summary
        let trend_summary = self.generate_trend_summary(&trends);

        Ok(TrendAnalysisResult {
            trends,
            seasonal_patterns: Vec::new(),
            anomalies: Vec::new(),
            predictions: Vec::new(),
            trend_summary,
        })
    }

    fn group_metrics_by_name(metrics: &[MetricPoint]) -> Vec<(String, Vec<&MetricPoint>)> {
        use std::collections::HashMap;

        let mut groups: HashMap<String, Vec<&MetricPoint>> = HashMap::new();

        for metric in metrics {
            groups
                .entry(metric.metric_name.clone())
                .or_insert_with(Vec::new)
                .push(metric);
        }

        groups.into_iter().collect()
    }

    async fn analyze_metric_trend(
        &self,
        metric_name: &str,
        metric_data: &[&MetricPoint],
    ) -> Result<Option<Trend>> {
        if metric_data.len() < 2 {
            return Ok(None);
        }

        // Calculate moving averages
        let values: Vec<f32> = metric_data.iter().map(|m| m.value).collect();
        let trend_direction = self.determine_trend_direction(&values);
        let trend_strength = self.calculate_trend_strength(&values);

        Ok(Some(Trend {
            metric_name: metric_name.to_string(),
            direction: trend_direction,
            strength: trend_strength,
            duration_secs: 3600, // Mock duration
            changes: Vec::new(),
            confidence: 0.8,
            significance: if trend_strength > 0.7 {
                TrendSignificance::High
            } else if trend_strength > 0.4 {
                TrendSignificance::Medium
            } else {
                TrendSignificance::Low
            },
        }))
    }

    fn determine_trend_direction(&self, values: &[f32]) -> TrendDirection {
        if values.len() < 2 {
            return TrendDirection::Stable;
        }

        let first_half_avg = values[..values.len() / 2].iter().sum::<f32>()
            / (values.len() / 2) as f32;
        let second_half_avg = values[values.len() / 2..].iter().sum::<f32>()
            / (values.len() - values.len() / 2) as f32;

        let change_percent = (second_half_avg - first_half_avg) / first_half_avg * 100.0;

        if change_percent > 5.0 {
            TrendDirection::Increasing
        } else if change_percent < -5.0 {
            TrendDirection::Decreasing
        } else {
            TrendDirection::Stable
        }
    }

    fn calculate_trend_strength(&self, values: &[f32]) -> f32 {
        if values.len() < 2 {
            return 0.0;
        }

        // Simple linear regression slope magnitude
        let n = values.len() as f32;
        let x_mean = (n - 1.0) / 2.0;
        let y_mean = values.iter().sum::<f32>() / n;

        let mut numerator = 0.0;
        let mut denominator = 0.0;

        for (i, &y) in values.iter().enumerate() {
            let x = i as f32;
            numerator += (x - x_mean) * (y - y_mean);
            denominator += (x - x_mean).powi(2);
        }

        if denominator > 0.0 {
            (numerator / denominator).abs().min(1.0)
        } else {
            0.0
        }
    }

    fn generate_trend_summary(&self, trends: &[Trend]) -> TrendSummary {
        let increasing_trends = trends
            .iter()
            .filter(|t| matches!(t.direction, TrendDirection::Increasing))
            .count();
        let decreasing_trends = trends
            .iter()
            .filter(|t| matches!(t.direction, TrendDirection::Decreasing))
            .count();
        let stable_trends = trends
            .iter()
            .filter(|t| matches!(t.direction, TrendDirection::Stable))
            .count();

        let mut key_findings = Vec::new();
        if increasing_trends > 0 {
            key_findings.push(format!("{} metrics showing increasing trends", increasing_trends));
        }
        if decreasing_trends > 0 {
            key_findings.push(format!("{} metrics showing decreasing trends", decreasing_trends));
        }

        TrendSummary {
            total_trends: trends.len(),
            increasing_trends,
            decreasing_trends,
            stable_trends,
            key_findings,
        }
    }
}

impl Default for TrendAnalysisEngine {
    fn default() -> Self {
        Self::new()
    }
}

/// Health Analysis Engine
#[derive(Debug)]
pub struct HealthAnalysisEngine;

impl HealthAnalysisEngine {
    pub fn new() -> Self {
        Self
    }

    pub async fn analyze(&self, _data: &DataCollection) -> Result<HealthAnalysisResult> {
        // Mock health analysis - would analyze actual system health in production
        let component_health = vec![
            ComponentHealth {
                component_name: "Coordinator".to_string(),
                health_score: 95.0,
                status: HealthStatus::Healthy,
                issues: Vec::new(),
            },
            ComponentHealth {
                component_name: "Runtime".to_string(),
                health_score: 88.0,
                status: HealthStatus::Healthy,
                issues: Vec::new(),
            },
        ];

        let overall_health_score = component_health.iter().map(|c| c.health_score).sum::<f32>()
            / component_health.len() as f32;

        Ok(HealthAnalysisResult {
            overall_health_score,
            component_health,
            health_issues: Vec::new(),
            health_trend: TrendDirection::Stable,
            recommendations: vec!["System health is good".to_string()],
        })
    }
}

impl Default for HealthAnalysisEngine {
    fn default() -> Self {
        Self::new()
    }
}

/// Efficiency Analysis Engine
#[derive(Debug)]
pub struct EfficiencyAnalysisEngine;

impl EfficiencyAnalysisEngine {
    pub fn new() -> Self {
        Self
    }

    pub async fn analyze(&self, _data: &DataCollection) -> Result<EfficiencyAnalysisResult> {
        Ok(EfficiencyAnalysisResult {
            resource_efficiency: 75.0,
            optimization_opportunities: vec![
                "Consider scaling down during off-peak hours".to_string(),
            ],
        })
    }
}

impl Default for EfficiencyAnalysisEngine {
    fn default() -> Self {
        Self::new()
    }
}

/// Pattern Analysis Engine
#[derive(Debug)]
pub struct PatternAnalysisEngine;

impl PatternAnalysisEngine {
    pub fn new() -> Self {
        Self
    }

    pub async fn analyze(&self, _data: &DataCollection) -> Result<PatternAnalysisResult> {
        Ok(PatternAnalysisResult {
            patterns: Vec::new(),
        })
    }
}

impl Default for PatternAnalysisEngine {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use chrono::Duration;

    fn create_test_data() -> DataCollection {
        let mut metrics = Vec::new();
        let now = Utc::now();

        // Create some test throughput metrics
        for i in 0..10 {
            metrics.push(MetricPoint {
                metric_name: "throughput".to_string(),
                value: 100.0 + i as f32 * 10.0,
                timestamp: now - Duration::seconds(i * 10),
            });
        }

        DataCollection {
            metrics,
            time_range: TimeWindow {
                start: now - Duration::hours(1),
                end: now,
                duration_description: "1h".to_string(),
            },
        }
    }

    #[tokio::test]
    async fn test_performance_analysis() {
        let engine = PerformanceAnalysisEngine::new();
        let data = create_test_data();

        let result = engine.analyze(&data).await.unwrap();

        assert!(result.throughput.current > 0.0);
        assert!(result.throughput.average > 0.0);
        assert_eq!(result.analysis_metadata.data_points_analyzed, 10);
    }

    #[tokio::test]
    async fn test_trend_analysis() {
        let engine = TrendAnalysisEngine::new();
        let data = create_test_data();

        let result = engine.analyze(&data).await.unwrap();

        assert!(!result.trends.is_empty());
        assert_eq!(result.trend_summary.total_trends, result.trends.len());
    }

    #[tokio::test]
    async fn test_health_analysis() {
        let engine = HealthAnalysisEngine::new();
        let data = create_test_data();

        let result = engine.analyze(&data).await.unwrap();

        assert!(result.overall_health_score > 0.0);
        assert!(!result.component_health.is_empty());
    }
}
