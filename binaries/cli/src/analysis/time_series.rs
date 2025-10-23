use std::collections::{HashMap, VecDeque};
use chrono::{DateTime, Utc, Duration};
use serde::{Serialize, Deserialize};

use crate::analysis::monitors::MetricValue;

/// Time series data storage and management
#[derive(Debug, Clone)]
pub struct TimeSeriesData {
    metrics: HashMap<String, TimeSeries>,
    retention_period: Duration,
    max_points_per_metric: usize,
}

/// Individual time series for a metric
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TimeSeries {
    points: VecDeque<TimePoint>,
    metric_name: String,
    unit: Option<String>,
    tags: HashMap<String, String>,
}

/// Single data point in a time series
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TimePoint {
    pub timestamp: DateTime<Utc>,
    pub value: f32,
    pub tags: HashMap<String, String>,
}

/// Aggregated statistics for time series analysis
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TimeSeriesStats {
    pub count: usize,
    pub mean: f32,
    pub std_dev: f32,
    pub min: f32,
    pub max: f32,
    pub percentiles: HashMap<u8, f32>,
    pub trend: TrendAnalysis,
}

/// Trend analysis results
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TrendAnalysis {
    pub direction: TrendDirection,
    pub slope: f32,
    pub correlation: f32,
    pub seasonal_pattern: Option<SeasonalPattern>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum TrendDirection {
    Increasing,
    Decreasing,
    Stable,
    Volatile,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SeasonalPattern {
    pub period_minutes: u32,
    pub amplitude: f32,
    pub phase: f32,
}

/// Query parameters for time series data
#[derive(Debug, Clone)]
pub struct TimeSeriesQuery {
    pub metric_name: String,
    pub start_time: Option<DateTime<Utc>>,
    pub end_time: Option<DateTime<Utc>>,
    pub aggregation: Option<Aggregation>,
    pub tags: HashMap<String, String>,
}

#[derive(Debug, Clone)]
pub enum Aggregation {
    Mean,
    Max,
    Min,
    Sum,
    Count,
    Percentile(u8),
}

impl TimeSeriesData {
    pub fn new() -> Self {
        Self {
            metrics: HashMap::new(),
            retention_period: Duration::days(30),
            max_points_per_metric: 10000,
        }
    }
    
    pub fn with_retention(retention_period: Duration, max_points: usize) -> Self {
        Self {
            metrics: HashMap::new(),
            retention_period,
            max_points_per_metric: max_points,
        }
    }
    
    /// Add a new data point to a metric
    pub fn add_point(&mut self, metric_name: String, value: f32, tags: Option<HashMap<String, String>>) {
        let timestamp = Utc::now();
        let point = TimePoint {
            timestamp,
            value,
            tags: tags.unwrap_or_default(),
        };
        
        self.metrics
            .entry(metric_name.clone())
            .or_insert_with(|| TimeSeries::new(metric_name))
            .add_point(point);
        
        // Cleanup old data
        self.cleanup_old_data();
    }
    
    /// Add multiple data points from MetricValue
    pub fn add_metric_value(&mut self, metric_name: String, value: &MetricValue, tags: Option<HashMap<String, String>>) {
        match value {
            MetricValue::Float(f) => self.add_point(metric_name, *f, tags),
            MetricValue::Integer(i) => self.add_point(metric_name, *i as f32, tags),
            MetricValue::Boolean(b) => self.add_point(metric_name, if *b { 1.0 } else { 0.0 }, tags),
            MetricValue::TimeSeries(points) => {
                for (timestamp, val) in points {
                    let point = TimePoint {
                        timestamp: *timestamp,
                        value: *val,
                        tags: tags.clone().unwrap_or_default(),
                    };
                    self.metrics
                        .entry(metric_name.clone())
                        .or_insert_with(|| TimeSeries::new(metric_name.clone()))
                        .add_point_with_timestamp(point);
                }
            },
            MetricValue::String(_) => {
                // Skip string metrics for time series
            }
        }
    }
    
    /// Get metric history as MetricValue vector for compatibility
    pub fn get_metric_history(&self, metric_name: &str) -> Option<Vec<MetricValue>> {
        self.metrics.get(metric_name).map(|ts| {
            ts.points.iter()
                .map(|point| MetricValue::Float(point.value))
                .collect()
        })
    }
    
    /// Get time series for a metric
    pub fn get_time_series(&self, metric_name: &str) -> Option<&TimeSeries> {
        self.metrics.get(metric_name)
    }
    
    /// Query time series data with filters
    pub fn query(&self, query: &TimeSeriesQuery) -> Option<Vec<TimePoint>> {
        let time_series = self.metrics.get(&query.metric_name)?;
        
        let filtered_points: Vec<TimePoint> = time_series.points
            .iter()
            .filter(|point| {
                // Time range filter
                if let Some(start) = query.start_time {
                    if point.timestamp < start {
                        return false;
                    }
                }
                if let Some(end) = query.end_time {
                    if point.timestamp > end {
                        return false;
                    }
                }
                
                // Tags filter
                for (key, value) in &query.tags {
                    if point.tags.get(key) != Some(value) {
                        return false;
                    }
                }
                
                true
            })
            .cloned()
            .collect();
        
        Some(filtered_points)
    }
    
    /// Calculate statistics for a metric
    pub fn calculate_stats(&self, metric_name: &str, window: Option<Duration>) -> Option<TimeSeriesStats> {
        let time_series = self.metrics.get(metric_name)?;
        
        let points = if let Some(window_duration) = window {
            let cutoff = Utc::now() - window_duration;
            time_series.points
                .iter()
                .filter(|p| p.timestamp >= cutoff)
                .collect::<Vec<_>>()
        } else {
            time_series.points.iter().collect::<Vec<_>>()
        };
        
        if points.is_empty() {
            return None;
        }
        
        let values: Vec<f32> = points.iter().map(|p| p.value).collect();
        let count = values.len();
        let mean = values.iter().sum::<f32>() / count as f32;
        
        let variance = values.iter()
            .map(|x| (x - mean).powi(2))
            .sum::<f32>() / count as f32;
        let std_dev = variance.sqrt();
        
        let mut sorted_values = values.clone();
        sorted_values.sort_by(|a, b| a.partial_cmp(b).unwrap());
        
        let min = sorted_values[0];
        let max = sorted_values[count - 1];
        
        // Calculate percentiles
        let mut percentiles = HashMap::new();
        for p in [5, 10, 25, 50, 75, 90, 95, 99] {
            let index = ((p as f32 / 100.0) * (count - 1) as f32) as usize;
            percentiles.insert(p, sorted_values[index]);
        }
        
        // Analyze trend
        let trend = self.analyze_trend(&points);
        
        Some(TimeSeriesStats {
            count,
            mean,
            std_dev,
            min,
            max,
            percentiles,
            trend,
        })
    }
    
    /// Get recent values for a metric
    pub fn get_recent_values(&self, metric_name: &str, count: usize) -> Vec<f32> {
        self.metrics
            .get(metric_name)
            .map(|ts| {
                ts.points
                    .iter()
                    .rev()
                    .take(count)
                    .map(|p| p.value)
                    .collect()
            })
            .unwrap_or_default()
    }
    
    /// Get trend data for prediction engines
    pub fn get_trend_data(&self) -> HashMap<String, Vec<(DateTime<Utc>, f32)>> {
        self.metrics
            .iter()
            .map(|(name, ts)| {
                let trend_points = ts.points
                    .iter()
                    .map(|p| (p.timestamp, p.value))
                    .collect();
                (name.clone(), trend_points)
            })
            .collect()
    }
    
    /// Add historical data for testing or initialization
    pub fn add_metric_history(&mut self, metric_name: String, values: Vec<MetricValue>) {
        let now = Utc::now();
        let values_len = values.len();
        let time_series = self.metrics
            .entry(metric_name.clone())
            .or_insert_with(|| TimeSeries::new(metric_name));
        
        for (i, value) in values.into_iter().enumerate() {
            let timestamp = now - Duration::minutes((values_len - i) as i64);
            let point = TimePoint {
                timestamp,
                value: value.as_float(),
                tags: HashMap::new(),
            };
            time_series.add_point_with_timestamp(point);
        }
    }
    
    /// Remove old data points beyond retention period
    fn cleanup_old_data(&mut self) {
        let cutoff = Utc::now() - self.retention_period;
        
        for time_series in self.metrics.values_mut() {
            time_series.cleanup_old_points(cutoff, self.max_points_per_metric);
        }
        
        // Remove empty time series
        self.metrics.retain(|_, ts| !ts.points.is_empty());
    }
    
    /// Analyze trend in time series data
    fn analyze_trend(&self, points: &[&TimePoint]) -> TrendAnalysis {
        if points.len() < 3 {
            return TrendAnalysis {
                direction: TrendDirection::Stable,
                slope: 0.0,
                correlation: 0.0,
                seasonal_pattern: None,
            };
        }
        
        // Calculate linear regression
        let n = points.len() as f32;
        let x_values: Vec<f32> = (0..points.len()).map(|i| i as f32).collect();
        let y_values: Vec<f32> = points.iter().map(|p| p.value).collect();
        
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
        
        // Calculate correlation coefficient
        let y_variance: f32 = y_values.iter()
            .map(|y| (y - y_mean).powi(2))
            .sum();
        
        let correlation = if denominator != 0.0 && y_variance != 0.0 {
            numerator / (denominator.sqrt() * y_variance.sqrt())
        } else {
            0.0
        };
        
        // Determine trend direction
        let direction = match slope {
            s if s > 0.1 => TrendDirection::Increasing,
            s if s < -0.1 => TrendDirection::Decreasing,
            _ => {
                // Check volatility
                let volatility = y_values.iter()
                    .map(|y| (y - y_mean).abs())
                    .sum::<f32>() / n;
                
                if volatility > y_mean * 0.2 {
                    TrendDirection::Volatile
                } else {
                    TrendDirection::Stable
                }
            }
        };
        
        // Simple seasonal pattern detection (placeholder)
        let seasonal_pattern = self.detect_seasonal_pattern(&y_values);
        
        TrendAnalysis {
            direction,
            slope,
            correlation,
            seasonal_pattern,
        }
    }
    
    /// Simple seasonal pattern detection
    fn detect_seasonal_pattern(&self, values: &[f32]) -> Option<SeasonalPattern> {
        if values.len() < 12 {
            return None;
        }
        
        // Very basic pattern detection - in a real implementation,
        // this would use FFT or autocorrelation
        let mean = values.iter().sum::<f32>() / values.len() as f32;
        let amplitude = values.iter()
            .map(|v| (v - mean).abs())
            .max_by(|a, b| a.partial_cmp(b).unwrap())
            .unwrap_or(0.0);
        
        if amplitude > mean * 0.1 {
            Some(SeasonalPattern {
                period_minutes: 60, // Assume hourly pattern
                amplitude,
                phase: 0.0,
            })
        } else {
            None
        }
    }
    
    /// Get all metric names
    pub fn get_metric_names(&self) -> Vec<String> {
        self.metrics.keys().cloned().collect()
    }
    
    /// Get data size statistics
    pub fn get_size_stats(&self) -> HashMap<String, usize> {
        let mut stats = HashMap::new();
        stats.insert("total_metrics".to_string(), self.metrics.len());
        
        let total_points: usize = self.metrics.values()
            .map(|ts| ts.points.len())
            .sum();
        stats.insert("total_points".to_string(), total_points);
        
        if !self.metrics.is_empty() {
            let avg_points = total_points / self.metrics.len();
            stats.insert("avg_points_per_metric".to_string(), avg_points);
        }
        
        stats
    }
}

impl TimeSeries {
    pub fn new(metric_name: String) -> Self {
        Self {
            points: VecDeque::new(),
            metric_name,
            unit: None,
            tags: HashMap::new(),
        }
    }
    
    pub fn with_unit(metric_name: String, unit: String) -> Self {
        Self {
            points: VecDeque::new(),
            metric_name,
            unit: Some(unit),
            tags: HashMap::new(),
        }
    }
    
    pub fn add_point(&mut self, point: TimePoint) {
        // Insert in chronological order
        match self.points.binary_search_by_key(&point.timestamp, |p| p.timestamp) {
            Ok(index) => {
                // Replace existing point at same timestamp
                self.points[index] = point;
            }
            Err(index) => {
                // Insert at correct position
                self.points.insert(index, point);
            }
        }
    }
    
    pub fn add_point_with_timestamp(&mut self, point: TimePoint) {
        self.add_point(point);
    }
    
    pub fn cleanup_old_points(&mut self, cutoff: DateTime<Utc>, max_points: usize) {
        // Remove points older than cutoff
        while let Some(front) = self.points.front() {
            if front.timestamp < cutoff {
                self.points.pop_front();
            } else {
                break;
            }
        }
        
        // Limit total number of points
        while self.points.len() > max_points {
            self.points.pop_front();
        }
    }
    
    pub fn get_latest_value(&self) -> Option<f32> {
        self.points.back().map(|p| p.value)
    }
    
    pub fn get_value_at(&self, timestamp: DateTime<Utc>) -> Option<f32> {
        self.points
            .iter()
            .find(|p| p.timestamp == timestamp)
            .map(|p| p.value)
    }
    
    pub fn get_values_in_range(&self, start: DateTime<Utc>, end: DateTime<Utc>) -> Vec<f32> {
        self.points
            .iter()
            .filter(|p| p.timestamp >= start && p.timestamp <= end)
            .map(|p| p.value)
            .collect()
    }
}

impl Default for TimeSeriesData {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_time_series_basic_operations() {
        let mut ts_data = TimeSeriesData::new();
        
        // Add some test data
        ts_data.add_point("cpu_usage".to_string(), 50.0, None);
        ts_data.add_point("cpu_usage".to_string(), 60.0, None);
        ts_data.add_point("cpu_usage".to_string(), 55.0, None);
        
        let history = ts_data.get_metric_history("cpu_usage").unwrap();
        assert_eq!(history.len(), 3);
        
        let recent = ts_data.get_recent_values("cpu_usage", 2);
        assert_eq!(recent.len(), 2);
    }
    
    #[test]
    fn test_time_series_stats() {
        let mut ts_data = TimeSeriesData::new();
        
        // Add test data with known statistical properties
        for i in 1..=10 {
            ts_data.add_point("test_metric".to_string(), i as f32, None);
        }
        
        let stats = ts_data.calculate_stats("test_metric", None).unwrap();
        assert_eq!(stats.count, 10);
        assert_eq!(stats.mean, 5.5);
        assert_eq!(stats.min, 1.0);
        assert_eq!(stats.max, 10.0);
    }
    
    #[test]
    fn test_time_series_query() {
        let mut ts_data = TimeSeriesData::new();
        
        // Add data with tags
        let mut tags = HashMap::new();
        tags.insert("host".to_string(), "server1".to_string());
        
        ts_data.add_point("cpu_usage".to_string(), 50.0, Some(tags.clone()));
        ts_data.add_point("cpu_usage".to_string(), 60.0, Some(tags));
        
        let query = TimeSeriesQuery {
            metric_name: "cpu_usage".to_string(),
            start_time: None,
            end_time: None,
            aggregation: None,
            tags: {
                let mut query_tags = HashMap::new();
                query_tags.insert("host".to_string(), "server1".to_string());
                query_tags
            },
        };
        
        let results = ts_data.query(&query).unwrap();
        assert_eq!(results.len(), 2);
    }
    
    #[test]
    fn test_trend_analysis() {
        let mut ts_data = TimeSeriesData::new();
        
        // Add increasing trend data
        for i in 1..=10 {
            ts_data.add_point("increasing_metric".to_string(), i as f32, None);
        }
        
        let stats = ts_data.calculate_stats("increasing_metric", None).unwrap();
        assert!(matches!(stats.trend.direction, TrendDirection::Increasing));
        assert!(stats.trend.slope > 0.0);
    }
    
    #[test]
    fn test_data_cleanup() {
        let mut ts_data = TimeSeriesData::with_retention(Duration::seconds(1), 5);
        
        // Add more data than max_points
        for i in 1..=10 {
            ts_data.add_point("test_metric".to_string(), i as f32, None);
        }
        
        // Wait for retention period and add new data to trigger cleanup
        std::thread::sleep(std::time::Duration::from_secs(2));
        ts_data.add_point("test_metric".to_string(), 11.0, None);
        
        let history = ts_data.get_metric_history("test_metric").unwrap();
        assert!(history.len() <= 5); // Should be limited by max_points
    }
}