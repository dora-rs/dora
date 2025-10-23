use std::collections::HashMap;
use std::time::Duration;
use async_trait::async_trait;
use chrono::{DateTime, Utc, Duration as ChronoDuration};
use serde::{Serialize, Deserialize};
use eyre::Result;

use crate::analysis::time_series::TimeSeriesData;
use crate::analysis::anomaly_detection::AnomalySeverity;

/// Resource prediction engine that coordinates multiple prediction algorithms
#[derive(Debug)]
pub struct ResourcePredictionEngine {
    predictors: HashMap<String, Box<dyn ResourcePredictor>>,
    trend_analyzer: TrendAnalyzer,
    config: PredictionConfig,
}

/// Configuration for prediction engine
#[derive(Debug, Clone)]
pub struct PredictionConfig {
    pub default_prediction_horizon: ChronoDuration,
    pub min_historical_points: usize,
    pub confidence_threshold: f32,
    pub enable_seasonal_adjustment: bool,
    pub max_prediction_points: usize,
}

impl Default for PredictionConfig {
    fn default() -> Self {
        Self {
            default_prediction_horizon: ChronoDuration::hours(1),
            min_historical_points: 20,
            confidence_threshold: 0.6,
            enable_seasonal_adjustment: true,
            max_prediction_points: 60,
        }
    }
}

/// Complete prediction results for resource analysis
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ResourcePredictions {
    pub cpu_predictions: Vec<PredictionPoint>,
    pub memory_predictions: Vec<PredictionPoint>,
    pub network_predictions: Vec<PredictionPoint>,
    pub performance_predictions: PerformancePredictions,
    pub capacity_warnings: Vec<CapacityWarning>,
    pub optimization_opportunities: Vec<OptimizationOpportunity>,
    pub prediction_metadata: PredictionMetadata,
}

/// Individual prediction point with confidence intervals
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PredictionPoint {
    pub timestamp: DateTime<Utc>,
    pub predicted_value: f32,
    pub confidence_interval: (f32, f32),
    pub confidence: f32,
    pub prediction_type: PredictionType,
}

/// Types of predictions
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum PredictionType {
    Linear,
    Exponential,
    Seasonal,
    Composite,
}

/// Performance-specific predictions
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PerformancePredictions {
    pub throughput_forecast: Vec<PredictionPoint>,
    pub latency_forecast: Vec<PredictionPoint>,
    pub error_rate_forecast: Vec<PredictionPoint>,
    pub bottleneck_predictions: Vec<BottleneckPrediction>,
}

/// Predicted bottlenecks
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BottleneckPrediction {
    pub resource_type: String,
    pub metric_name: String,
    pub predicted_bottleneck_time: DateTime<Utc>,
    pub severity: AnomalySeverity,
    pub confidence: f32,
    pub mitigation_suggestions: Vec<String>,
}

/// Capacity warnings for resource exhaustion
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CapacityWarning {
    pub resource_type: String,
    pub resource_name: String,
    pub current_utilization: f32,
    pub predicted_full_time: Option<DateTime<Utc>>,
    pub time_to_capacity: Option<ChronoDuration>,
    pub severity: AnomalySeverity,
    pub confidence: f32,
    pub recommendations: Vec<String>,
    pub trend_info: TrendInfo,
}

/// Optimization opportunities identified through prediction
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OptimizationOpportunity {
    pub opportunity_type: OptimizationType,
    pub resource_involved: String,
    pub potential_savings: f32,
    pub confidence: f32,
    pub implementation_complexity: ComplexityLevel,
    pub estimated_impact: ImpactEstimate,
    pub action_items: Vec<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum OptimizationType {
    ResourceScaleDown,
    ResourceScaleUp,
    LoadBalancing,
    CacheOptimization,
    BatchingOptimization,
    SchedulingOptimization,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum ComplexityLevel {
    Low,
    Medium,
    High,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ImpactEstimate {
    pub performance_improvement: f32,
    pub cost_reduction: f32,
    pub reliability_improvement: f32,
}

/// Trend information for predictions
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TrendInfo {
    pub direction: TrendDirection,
    pub velocity: f32,
    pub acceleration: f32,
    pub seasonality_detected: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum TrendDirection {
    Increasing,
    Decreasing,
    Stable,
    Cyclical,
}

/// Metadata about prediction quality and methods
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PredictionMetadata {
    pub generation_time: DateTime<Utc>,
    pub prediction_horizon: ChronoDuration,
    pub algorithms_used: Vec<String>,
    pub overall_confidence: f32,
    pub data_quality_score: f32,
    pub limitations: Vec<String>,
}

/// Trait for resource prediction algorithms
#[async_trait]
pub trait ResourcePredictor: Send + Sync {
    async fn predict(
        &self,
        historical_data: &TimeSeriesData,
        prediction_horizon: ChronoDuration,
    ) -> Result<Vec<PredictionPoint>>;
    
    fn predictor_name(&self) -> &str;
    fn supported_metrics(&self) -> Vec<String>;
    fn confidence_level(&self) -> f32;
}

/// Linear trend predictor using least squares regression
pub struct LinearTrendPredictor {
    window_size: usize,
    confidence_threshold: f32,
}

impl LinearTrendPredictor {
    pub fn new(window_size: usize) -> Self {
        Self {
            window_size,
            confidence_threshold: 0.5,
        }
    }
}

#[async_trait]
impl ResourcePredictor for LinearTrendPredictor {
    async fn predict(
        &self,
        historical_data: &TimeSeriesData,
        prediction_horizon: ChronoDuration,
    ) -> Result<Vec<PredictionPoint>> {
        let mut predictions = Vec::new();
        
        for metric_name in self.supported_metrics() {
            if let Some(metric_history) = historical_data.get_metric_history(&metric_name) {
                let recent_data: Vec<f32> = metric_history
                    .iter()
                    .rev()
                    .take(self.window_size)
                    .map(|v| v.as_float())
                    .collect();
                
                if recent_data.len() < 5 {
                    continue; // Not enough data for prediction
                }
                
                let trend = self.calculate_linear_trend(&recent_data);
                let prediction_points = self.generate_prediction_points(
                    &trend,
                    prediction_horizon,
                    &metric_name,
                );
                
                predictions.extend(prediction_points);
            }
        }
        
        Ok(predictions)
    }
    
    fn predictor_name(&self) -> &str {
        "Linear Trend Predictor"
    }
    
    fn supported_metrics(&self) -> Vec<String> {
        vec![
            "cpu_usage".to_string(),
            "memory_usage".to_string(),
            "network_throughput".to_string(),
            "error_rate".to_string(),
            "disk_usage".to_string(),
            "latency".to_string(),
        ]
    }
    
    fn confidence_level(&self) -> f32 {
        0.7
    }
}

impl LinearTrendPredictor {
    fn calculate_linear_trend(&self, data_points: &[f32]) -> TrendLine {
        let n = data_points.len() as f32;
        let x_values: Vec<f32> = (0..data_points.len()).map(|i| i as f32).collect();
        let y_values = data_points;
        
        let x_mean = x_values.iter().sum::<f32>() / n;
        let y_mean = y_values.iter().sum::<f32>() / n;
        
        let numerator: f32 = x_values.iter()
            .zip(y_values.iter())
            .map(|(x, y)| (x - x_mean) * (y - y_mean))
            .sum();
        
        let denominator: f32 = x_values.iter()
            .map(|x| (x - x_mean).powi(2))
            .sum();
        
        let slope = if denominator != 0.0 { numerator / denominator } else { 0.0 };
        let intercept = y_mean - slope * x_mean;
        
        // Calculate R-squared for confidence
        let ss_res: f32 = x_values.iter()
            .zip(y_values.iter())
            .map(|(x, y)| {
                let predicted = slope * x + intercept;
                (y - predicted).powi(2)
            })
            .sum();
        
        let ss_tot: f32 = y_values.iter()
            .map(|y| (y - y_mean).powi(2))
            .sum();
        
        let r_squared = if ss_tot != 0.0 { 1.0 - (ss_res / ss_tot) } else { 0.0 };
        
        // Calculate standard error for confidence intervals
        let std_error = if n > 2.0 { (ss_res / (n - 2.0)).sqrt() } else { 0.0 };
        
        TrendLine {
            slope,
            intercept,
            confidence: r_squared.max(0.0).min(1.0),
            std_error,
            last_x: x_values.len() as f32 - 1.0,
            last_y: y_values[y_values.len() - 1],
        }
    }
    
    fn generate_prediction_points(
        &self,
        trend: &TrendLine,
        horizon: ChronoDuration,
        metric_name: &str,
    ) -> Vec<PredictionPoint> {
        let mut points = Vec::new();
        let now = Utc::now();
        
        // Generate prediction points at regular intervals
        let interval_minutes = 5; // 5-minute intervals
        let total_intervals = horizon.num_minutes() / interval_minutes;
        
        for i in 1..=total_intervals.min(60) { // Limit to 60 points max
            let minutes_ahead = i * interval_minutes;
            let timestamp = now + ChronoDuration::minutes(minutes_ahead);
            
            let x_future = trend.last_x + (minutes_ahead as f32 / interval_minutes as f32);
            let predicted_value = trend.slope * x_future + trend.intercept;
            
            // Calculate confidence interval (95% confidence)
            let t_value = 1.96; // Approximate for large samples
            let prediction_error = trend.std_error * (1.0 + 1.0/trend.last_x).sqrt();
            let margin_of_error = t_value * prediction_error;
            
            let confidence_interval = (
                predicted_value - margin_of_error,
                predicted_value + margin_of_error,
            );
            
            // Adjust confidence based on distance into future
            let time_decay = 1.0 - (minutes_ahead as f32 / horizon.num_minutes() as f32) * 0.3;
            let adjusted_confidence = trend.confidence * time_decay;
            
            points.push(PredictionPoint {
                timestamp,
                predicted_value,
                confidence_interval,
                confidence: adjusted_confidence.max(0.1).min(1.0),
                prediction_type: PredictionType::Linear,
            });
        }
        
        points
    }
}

/// Exponential smoothing predictor for trending data
pub struct ExponentialSmoothingPredictor {
    alpha: f32, // Smoothing parameter
    beta: f32,  // Trend parameter
    gamma: f32, // Seasonal parameter
}

impl ExponentialSmoothingPredictor {
    pub fn new(alpha: f32, beta: f32, gamma: f32) -> Self {
        Self { alpha, beta, gamma }
    }
}

#[async_trait]
impl ResourcePredictor for ExponentialSmoothingPredictor {
    async fn predict(
        &self,
        historical_data: &TimeSeriesData,
        prediction_horizon: ChronoDuration,
    ) -> Result<Vec<PredictionPoint>> {
        let mut predictions = Vec::new();
        
        for metric_name in self.supported_metrics() {
            if let Some(metric_history) = historical_data.get_metric_history(&metric_name) {
                let data: Vec<f32> = metric_history
                    .iter()
                    .map(|v| v.as_float())
                    .collect();
                
                if data.len() < 10 {
                    continue;
                }
                
                let forecast = self.exponential_smoothing_forecast(&data, prediction_horizon);
                predictions.extend(forecast);
            }
        }
        
        Ok(predictions)
    }
    
    fn predictor_name(&self) -> &str {
        "Exponential Smoothing Predictor"
    }
    
    fn supported_metrics(&self) -> Vec<String> {
        vec![
            "throughput".to_string(),
            "latency".to_string(),
            "cpu_usage".to_string(),
            "memory_usage".to_string(),
        ]
    }
    
    fn confidence_level(&self) -> f32 {
        0.8
    }
}

impl ExponentialSmoothingPredictor {
    fn exponential_smoothing_forecast(&self, data: &[f32], horizon: ChronoDuration) -> Vec<PredictionPoint> {
        let mut points = Vec::new();
        
        if data.len() < 3 {
            return points;
        }
        
        // Initialize level and trend
        let mut level = data[0];
        let mut trend = data[1] - data[0];
        
        // Apply exponential smoothing
        for &value in &data[1..] {
            let prev_level = level;
            level = self.alpha * value + (1.0 - self.alpha) * (level + trend);
            trend = self.beta * (level - prev_level) + (1.0 - self.beta) * trend;
        }
        
        // Generate forecasts
        let now = Utc::now();
        let intervals = (horizon.num_minutes() / 5).min(60); // 5-minute intervals, max 60 points
        
        for i in 1..=intervals {
            let timestamp = now + ChronoDuration::minutes(i * 5);
            let forecast = level + trend * i as f32;
            
            // Simple confidence calculation
            let confidence = (0.9 - (i as f32 * 0.01)).max(0.3);
            let error_margin = forecast * 0.1 * i as f32; // Increasing uncertainty
            
            points.push(PredictionPoint {
                timestamp,
                predicted_value: forecast,
                confidence_interval: (forecast - error_margin, forecast + error_margin),
                confidence,
                prediction_type: PredictionType::Exponential,
            });
        }
        
        points
    }
}

/// Trend analysis helper
#[derive(Debug)]
pub struct TrendAnalyzer {
    min_data_points: usize,
}

impl TrendAnalyzer {
    pub fn new() -> Self {
        Self {
            min_data_points: 10,
        }
    }
    
    pub fn analyze_capacity_trends(&self, data: &TimeSeriesData) -> Vec<CapacityWarning> {
        let mut warnings = Vec::new();
        
        for metric_name in ["cpu_usage", "memory_usage", "disk_usage"] {
            if let Some(stats) = data.calculate_stats(metric_name, Some(ChronoDuration::hours(24))) {
                if let Some(warning) = self.check_capacity_warning(metric_name, &stats) {
                    warnings.push(warning);
                }
            }
        }
        
        warnings
    }
    
    fn check_capacity_warning(&self, metric_name: &str, stats: &crate::analysis::time_series::TimeSeriesStats) -> Option<CapacityWarning> {
        // Check if trend is increasing towards capacity limit
        let is_increasing = matches!(stats.trend.direction, crate::analysis::time_series::TrendDirection::Increasing);
        let current_level = stats.mean;
        
        if !is_increasing || current_level < 50.0 {
            return None; // No warning needed
        }
        
        let capacity_limit = match metric_name {
            "cpu_usage" => 95.0,
            "memory_usage" => 90.0,
            "disk_usage" => 85.0,
            _ => 100.0,
        };
        
        let current_utilization = (current_level / capacity_limit) * 100.0;
        
        if current_utilization < 70.0 {
            return None;
        }
        
        let severity = match current_utilization {
            u if u >= 90.0 => AnomalySeverity::Critical,
            u if u >= 80.0 => AnomalySeverity::High,
            u if u >= 70.0 => AnomalySeverity::Medium,
            _ => AnomalySeverity::Low,
        };
        
        // Estimate time to capacity based on trend
        let time_to_capacity = if stats.trend.slope > 0.0 {
            let remaining_capacity = capacity_limit - current_level;
            let time_minutes = remaining_capacity / stats.trend.slope;
            Some(ChronoDuration::minutes(time_minutes as i64))
        } else {
            None
        };
        
        let predicted_full_time = time_to_capacity.map(|duration| Utc::now() + duration);
        
        Some(CapacityWarning {
            resource_type: metric_name.to_string(),
            resource_name: metric_name.to_string(),
            current_utilization,
            predicted_full_time,
            time_to_capacity,
            severity,
            confidence: 0.8,
            recommendations: self.generate_capacity_recommendations(metric_name, current_utilization),
            trend_info: TrendInfo {
                direction: TrendDirection::Increasing,
                velocity: stats.trend.slope,
                acceleration: 0.0, // Simplified
                seasonality_detected: stats.trend.seasonal_pattern.is_some(),
            },
        })
    }
    
    fn generate_capacity_recommendations(&self, metric_name: &str, utilization: f32) -> Vec<String> {
        let mut recommendations = Vec::new();
        
        match metric_name {
            "cpu_usage" => {
                recommendations.push("Consider horizontal scaling or upgrading CPU capacity".to_string());
                recommendations.push("Optimize CPU-intensive operations".to_string());
                if utilization > 85.0 {
                    recommendations.push("URGENT: Scale out immediately to prevent performance degradation".to_string());
                }
            },
            "memory_usage" => {
                recommendations.push("Increase memory allocation or optimize memory usage".to_string());
                recommendations.push("Check for memory leaks in applications".to_string());
                recommendations.push("Consider implementing memory-efficient algorithms".to_string());
            },
            "disk_usage" => {
                recommendations.push("Free up disk space or add additional storage".to_string());
                recommendations.push("Implement data archival strategies".to_string());
                recommendations.push("Set up log rotation and cleanup policies".to_string());
            },
            _ => {
                recommendations.push(format!("Monitor {} usage and plan for capacity expansion", metric_name));
            }
        }
        
        recommendations
    }
}

/// Helper struct for linear trend calculation
#[derive(Debug, Clone)]
pub struct TrendLine {
    pub slope: f32,
    pub intercept: f32,
    pub confidence: f32,
    pub std_error: f32,
    pub last_x: f32,
    pub last_y: f32,
}

impl ResourcePredictionEngine {
    pub fn new() -> Self {
        let mut engine = Self {
            predictors: HashMap::new(),
            trend_analyzer: TrendAnalyzer::new(),
            config: PredictionConfig::default(),
        };
        
        // Register default predictors
        engine.register_predictor("linear".to_string(), Box::new(LinearTrendPredictor::new(20)));
        engine.register_predictor("exponential".to_string(), Box::new(ExponentialSmoothingPredictor::new(0.3, 0.1, 0.1)));
        
        engine
    }
    
    pub fn register_predictor(&mut self, name: String, predictor: Box<dyn ResourcePredictor>) {
        self.predictors.insert(name, predictor);
    }
    
    pub async fn predict_resource_trends(
        &self,
        trend_data: &HashMap<String, Vec<(DateTime<Utc>, f32)>>,
    ) -> Result<ResourcePredictions> {
        // Convert trend data to TimeSeriesData format
        let mut time_series_data = TimeSeriesData::new();
        for (metric_name, points) in trend_data {
            for (timestamp, value) in points {
                time_series_data.add_point(
                    metric_name.clone(),
                    *value,
                    None,
                );
            }
        }
        
        // Generate predictions using all registered predictors
        let mut all_predictions = HashMap::new();
        
        for (name, predictor) in &self.predictors {
            let predictions = predictor.predict(&time_series_data, self.config.default_prediction_horizon).await?;
            all_predictions.insert(name.clone(), predictions);
        }
        
        // Aggregate and organize predictions
        let cpu_predictions = self.extract_predictions_for_metric(&all_predictions, "cpu_usage");
        let memory_predictions = self.extract_predictions_for_metric(&all_predictions, "memory_usage");
        let network_predictions = self.extract_predictions_for_metric(&all_predictions, "network_throughput");
        
        // Generate capacity warnings
        let capacity_warnings = self.trend_analyzer.analyze_capacity_trends(&time_series_data);
        
        // Generate optimization opportunities
        let optimization_opportunities = self.identify_optimization_opportunities(&time_series_data);
        
        // Create performance predictions
        let performance_predictions = PerformancePredictions {
            throughput_forecast: self.extract_predictions_for_metric(&all_predictions, "throughput"),
            latency_forecast: self.extract_predictions_for_metric(&all_predictions, "latency"),
            error_rate_forecast: self.extract_predictions_for_metric(&all_predictions, "error_rate"),
            bottleneck_predictions: self.predict_bottlenecks(&time_series_data),
        };
        
        // Calculate overall confidence and metadata
        let overall_confidence = self.calculate_overall_confidence(&all_predictions);
        let data_quality_score = self.assess_data_quality(&time_series_data);
        
        Ok(ResourcePredictions {
            cpu_predictions,
            memory_predictions,
            network_predictions,
            performance_predictions,
            capacity_warnings,
            optimization_opportunities,
            prediction_metadata: PredictionMetadata {
                generation_time: Utc::now(),
                prediction_horizon: self.config.default_prediction_horizon,
                algorithms_used: self.predictors.keys().cloned().collect(),
                overall_confidence,
                data_quality_score,
                limitations: self.identify_prediction_limitations(&time_series_data),
            },
        })
    }
    
    fn extract_predictions_for_metric(
        &self,
        all_predictions: &HashMap<String, Vec<PredictionPoint>>,
        metric_pattern: &str,
    ) -> Vec<PredictionPoint> {
        let mut metric_predictions = Vec::new();
        
        for predictions in all_predictions.values() {
            // In a real implementation, we'd filter by metric name
            // For now, we'll take a subset of predictions
            metric_predictions.extend(predictions.iter().take(10).cloned());
        }
        
        metric_predictions
    }
    
    fn identify_optimization_opportunities(&self, data: &TimeSeriesData) -> Vec<OptimizationOpportunity> {
        let mut opportunities = Vec::new();
        
        // Example optimization detection logic
        for metric_name in data.get_metric_names() {
            if let Some(stats) = data.calculate_stats(&metric_name, Some(ChronoDuration::hours(24))) {
                if metric_name.contains("cpu") && stats.mean < 30.0 {
                    opportunities.push(OptimizationOpportunity {
                        opportunity_type: OptimizationType::ResourceScaleDown,
                        resource_involved: metric_name.clone(),
                        potential_savings: 0.3,
                        confidence: 0.8,
                        implementation_complexity: ComplexityLevel::Low,
                        estimated_impact: ImpactEstimate {
                            performance_improvement: 0.0,
                            cost_reduction: 0.3,
                            reliability_improvement: 0.1,
                        },
                        action_items: vec![
                            "Consider reducing CPU allocation".to_string(),
                            "Evaluate workload patterns".to_string(),
                        ],
                    });
                }
            }
        }
        
        opportunities
    }
    
    fn predict_bottlenecks(&self, data: &TimeSeriesData) -> Vec<BottleneckPrediction> {
        let mut bottlenecks = Vec::new();
        
        // Simple bottleneck prediction based on trending metrics
        for metric_name in data.get_metric_names() {
            if let Some(stats) = data.calculate_stats(&metric_name, Some(ChronoDuration::hours(24))) {
                if stats.mean > 80.0 && matches!(stats.trend.direction, crate::analysis::time_series::TrendDirection::Increasing) {
                    bottlenecks.push(BottleneckPrediction {
                        resource_type: "system".to_string(),
                        metric_name: metric_name.clone(),
                        predicted_bottleneck_time: Utc::now() + ChronoDuration::hours(2),
                        severity: AnomalySeverity::High,
                        confidence: 0.7,
                        mitigation_suggestions: vec![
                            "Scale out resources".to_string(),
                            "Optimize workload distribution".to_string(),
                        ],
                    });
                }
            }
        }
        
        bottlenecks
    }
    
    fn calculate_overall_confidence(&self, all_predictions: &HashMap<String, Vec<PredictionPoint>>) -> f32 {
        let all_confidences: Vec<f32> = all_predictions
            .values()
            .flat_map(|predictions| predictions.iter().map(|p| p.confidence))
            .collect();
        
        if all_confidences.is_empty() {
            0.0
        } else {
            all_confidences.iter().sum::<f32>() / all_confidences.len() as f32
        }
    }
    
    fn assess_data_quality(&self, data: &TimeSeriesData) -> f32 {
        let size_stats = data.get_size_stats();
        let total_metrics = size_stats.get("total_metrics").unwrap_or(&0);
        let total_points = size_stats.get("total_points").unwrap_or(&0);
        
        // Simple quality assessment based on data availability
        match (total_metrics, total_points) {
            (m, p) if *m > 5 && *p > 100 => 0.9,
            (m, p) if *m > 3 && *p > 50 => 0.7,
            (m, p) if *m > 1 && *p > 20 => 0.5,
            _ => 0.3,
        }
    }
    
    fn identify_prediction_limitations(&self, data: &TimeSeriesData) -> Vec<String> {
        let mut limitations = Vec::new();
        
        let size_stats = data.get_size_stats();
        let total_points = size_stats.get("total_points").unwrap_or(&0);
        
        if *total_points < 50 {
            limitations.push("Limited historical data may reduce prediction accuracy".to_string());
        }
        
        if self.predictors.len() < 2 {
            limitations.push("Single prediction algorithm may miss complex patterns".to_string());
        }
        
        limitations.push("Predictions assume historical patterns will continue".to_string());
        limitations.push("External factors and system changes may affect accuracy".to_string());
        
        limitations
    }
}

impl Default for ResourcePredictionEngine {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::analysis::monitors::MetricValue;
    
    #[tokio::test]
    async fn test_linear_trend_predictor() {
        let predictor = LinearTrendPredictor::new(10);
        
        let mut time_series_data = TimeSeriesData::new();
        
        // Add trending data
        for i in 1..=20 {
            time_series_data.add_point("cpu_usage".to_string(), i as f32 * 2.0, None);
        }
        
        let predictions = predictor.predict(&time_series_data, ChronoDuration::hours(1)).await.unwrap();
        
        assert!(!predictions.is_empty());
        assert!(predictions.iter().all(|p| p.confidence > 0.0));
    }
    
    #[test]
    fn test_trend_line_calculation() {
        let predictor = LinearTrendPredictor::new(10);
        let data = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        
        let trend = predictor.calculate_linear_trend(&data);
        
        assert!(trend.slope > 0.0); // Should detect increasing trend
        assert!(trend.confidence > 0.9); // Perfect linear data should have high confidence
    }
    
    #[tokio::test]
    async fn test_exponential_smoothing_predictor() {
        let predictor = ExponentialSmoothingPredictor::new(0.3, 0.1, 0.1);
        
        let mut time_series_data = TimeSeriesData::new();
        
        // Add some data with noise
        for i in 1..=15 {
            let value = (i as f32 * 1.5) + (i as f32 % 3.0); // Trending with noise
            time_series_data.add_point("throughput".to_string(), value, None);
        }
        
        let predictions = predictor.predict(&time_series_data, ChronoDuration::minutes(30)).await.unwrap();
        
        assert!(!predictions.is_empty());
        assert!(predictions.iter().all(|p| matches!(p.prediction_type, PredictionType::Exponential)));
    }
    
    #[test]
    fn test_capacity_warning_generation() {
        let analyzer = TrendAnalyzer::new();
        let mut time_series_data = TimeSeriesData::new();
        
        // Add high CPU usage data with increasing trend
        for i in 1..=20 {
            let value = 70.0 + (i as f32 * 1.0); // 70% to 90% CPU usage
            time_series_data.add_point("cpu_usage".to_string(), value, None);
        }
        
        let warnings = analyzer.analyze_capacity_trends(&time_series_data);
        
        assert!(!warnings.is_empty());
        assert!(warnings.iter().any(|w| w.resource_type == "cpu_usage"));
    }
}