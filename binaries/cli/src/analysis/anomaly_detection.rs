use std::collections::HashMap;
use async_trait::async_trait;
use chrono::{DateTime, Utc, Duration as ChronoDuration};
use serde::{Serialize, Deserialize};
use eyre::Result;
use uuid::Uuid;

use crate::analysis::monitors::{ResourceType, MetricValue};
use crate::analysis::time_series::TimeSeriesData;

/// Anomaly detection engine that coordinates multiple detection algorithms
#[derive(Debug)]
pub struct AnomalyDetector {
    detectors: Vec<Box<dyn AnomalyDetectionAlgorithm>>,
    baseline_calculator: BaselineCalculator,
    alert_manager: AlertManager,
    config: AnomalyDetectionConfig,
}

/// Configuration for anomaly detection
#[derive(Debug, Clone)]
pub struct AnomalyDetectionConfig {
    pub global_sensitivity: f32,
    pub min_data_points: usize,
    pub detection_window_minutes: i64,
    pub alert_cooldown_minutes: i64,
    pub enable_adaptive_thresholds: bool,
}

impl Default for AnomalyDetectionConfig {
    fn default() -> Self {
        Self {
            global_sensitivity: 0.5,
            min_data_points: 10,
            detection_window_minutes: 30,
            alert_cooldown_minutes: 5,
            enable_adaptive_thresholds: true,
        }
    }
}

/// Detected anomaly with context and suggestions
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Anomaly {
    pub id: String,
    pub anomaly_type: AnomalyType,
    pub severity: AnomalySeverity,
    pub description: String,
    pub affected_resource: String,
    pub metric_name: String,
    pub current_value: f32,
    pub expected_range: (f32, f32),
    pub confidence: f32,
    pub first_detected: DateTime<Utc>,
    pub last_seen: DateTime<Utc>,
    pub suggestions: Vec<String>,
    pub context: AnomalyContext,
}

/// Types of anomalies that can be detected
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum AnomalyType {
    PerformanceDegradation,
    ResourceExhaustion,
    ErrorRateSpike,
    LatencyIncrease,
    ThroughputDrop,
    MemoryLeak,
    CpuSpike,
    NetworkCongestion,
    DiskSpaceWarning,
    UnusualPattern,
    NodeUnresponsive,
    QueueBacklog,
    HeartbeatMissed,
}

/// Severity levels for anomalies
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum AnomalySeverity {
    Info,
    Low,
    Medium,
    High,
    Critical,
}

/// Additional context for anomalies
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AnomalyContext {
    pub affected_components: Vec<String>,
    pub related_metrics: HashMap<String, f32>,
    pub trend_direction: TrendDirection,
    pub impact_assessment: ImpactAssessment,
    pub detection_algorithm: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum TrendDirection {
    Increasing,
    Decreasing,
    Stable,
    Oscillating,
    Unknown,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ImpactAssessment {
    pub severity_score: f32,
    pub affected_user_experience: bool,
    pub system_stability_risk: bool,
    pub data_loss_risk: bool,
    pub estimated_resolution_time: Option<ChronoDuration>,
}

/// Trait for anomaly detection algorithms
#[async_trait]
pub trait AnomalyDetectionAlgorithm: Send + Sync {
    async fn detect_anomalies(
        &self,
        current_metrics: &HashMap<ResourceType, HashMap<String, MetricValue>>,
        historical_data: &TimeSeriesData,
    ) -> Result<Vec<Anomaly>>;
    
    fn algorithm_name(&self) -> &str;
    fn detection_sensitivity(&self) -> f32;
    fn supported_metric_types(&self) -> Vec<String>;
}

/// Statistical anomaly detector using z-score analysis
pub struct StatisticalAnomalyDetector {
    sensitivity: f32,
    window_size: usize,
    z_score_threshold: f32,
}

impl StatisticalAnomalyDetector {
    pub fn new(sensitivity: f32, window_size: usize) -> Self {
        Self {
            sensitivity,
            window_size,
            z_score_threshold: 2.0 / sensitivity.max(0.1), // Adjustable threshold
        }
    }
}

#[async_trait]
impl AnomalyDetectionAlgorithm for StatisticalAnomalyDetector {
    async fn detect_anomalies(
        &self,
        current_metrics: &HashMap<ResourceType, HashMap<String, MetricValue>>,
        historical_data: &TimeSeriesData,
    ) -> Result<Vec<Anomaly>> {
        let mut anomalies = Vec::new();
        
        for (resource_type, metrics) in current_metrics {
            for (metric_name, current_value) in metrics {
                if let Some(historical_values) = historical_data.get_metric_history(metric_name) {
                    if let Some(anomaly) = self.detect_statistical_anomaly(
                        resource_type,
                        metric_name,
                        current_value,
                        &historical_values,
                    ) {
                        anomalies.push(anomaly);
                    }
                }
            }
        }
        
        Ok(anomalies)
    }
    
    fn algorithm_name(&self) -> &str {
        "Statistical Z-Score Detector"
    }
    
    fn detection_sensitivity(&self) -> f32 {
        self.sensitivity
    }
    
    fn supported_metric_types(&self) -> Vec<String> {
        vec![
            "cpu_usage".to_string(),
            "memory_usage".to_string(),
            "network_in".to_string(),
            "network_out".to_string(),
            "latency".to_string(),
            "throughput".to_string(),
            "error_rate".to_string(),
        ]
    }
}

impl StatisticalAnomalyDetector {
    fn detect_statistical_anomaly(
        &self,
        resource_type: &ResourceType,
        metric_name: &str,
        current_value: &MetricValue,
        historical_values: &[MetricValue],
    ) -> Option<Anomaly> {
        let current_float = current_value.as_float();
        let historical_floats: Vec<f32> = historical_values
            .iter()
            .map(|v| v.as_float())
            .collect();
        
        if historical_floats.len() < self.window_size.min(10) {
            return None; // Not enough data
        }
        
        let stats = self.calculate_statistics(&historical_floats);
        let z_score = (current_float - stats.mean) / stats.std_dev;
        
        if z_score.abs() > self.z_score_threshold {
            let severity = self.calculate_severity(z_score.abs(), &stats);
            let anomaly_type = self.classify_anomaly_type(metric_name, z_score > 0.0, current_float);
            let confidence = (z_score.abs() / self.z_score_threshold).min(1.0);
            
            Some(Anomaly {
                id: Uuid::new_v4().to_string(),
                anomaly_type: anomaly_type.clone(),
                severity,
                description: format!(
                    "Metric '{}' has unusual value {:.2} (expected: {:.2} ± {:.2}, z-score: {:.2})",
                    metric_name, current_float, stats.mean, stats.std_dev, z_score
                ),
                affected_resource: format!("{:?}", resource_type),
                metric_name: metric_name.to_string(),
                current_value: current_float,
                expected_range: (stats.mean - stats.std_dev, stats.mean + stats.std_dev),
                confidence,
                first_detected: Utc::now(),
                last_seen: Utc::now(),
                suggestions: self.generate_suggestions(&anomaly_type, metric_name, current_float, &stats),
                context: AnomalyContext {
                    affected_components: vec![format!("{:?}", resource_type)],
                    related_metrics: HashMap::new(),
                    trend_direction: self.analyze_trend(&historical_floats),
                    impact_assessment: self.assess_impact(&anomaly_type, current_float, &stats),
                    detection_algorithm: self.algorithm_name().to_string(),
                },
            })
        } else {
            None
        }
    }
    
    fn calculate_statistics(&self, values: &[f32]) -> Statistics {
        let mean = values.iter().sum::<f32>() / values.len() as f32;
        let variance = values.iter()
            .map(|x| (x - mean).powi(2))
            .sum::<f32>() / values.len() as f32;
        let std_dev = variance.sqrt();
        
        let mut sorted_values = values.to_vec();
        sorted_values.sort_by(|a, b| a.partial_cmp(b).unwrap());
        
        let median = if sorted_values.len() % 2 == 0 {
            let mid = sorted_values.len() / 2;
            (sorted_values[mid - 1] + sorted_values[mid]) / 2.0
        } else {
            sorted_values[sorted_values.len() / 2]
        };
        
        Statistics {
            mean,
            std_dev,
            median,
            min: sorted_values[0],
            max: sorted_values[sorted_values.len() - 1],
            count: values.len(),
        }
    }
    
    fn calculate_severity(&self, z_score_abs: f32, stats: &Statistics) -> AnomalySeverity {
        match z_score_abs {
            z if z > 4.0 => AnomalySeverity::Critical,
            z if z > 3.0 => AnomalySeverity::High,
            z if z > 2.5 => AnomalySeverity::Medium,
            z if z > 2.0 => AnomalySeverity::Low,
            _ => AnomalySeverity::Info,
        }
    }
    
    fn classify_anomaly_type(&self, metric_name: &str, is_spike: bool, current_value: f32) -> AnomalyType {
        let metric_lower = metric_name.to_lowercase();
        
        match (metric_lower.as_str(), is_spike) {
            (name, true) if name.contains("cpu") => AnomalyType::CpuSpike,
            (name, true) if name.contains("memory") && current_value > 1_000_000.0 => AnomalyType::MemoryLeak,
            (name, true) if name.contains("error") => AnomalyType::ErrorRateSpike,
            (name, true) if name.contains("latency") => AnomalyType::LatencyIncrease,
            (name, false) if name.contains("throughput") => AnomalyType::ThroughputDrop,
            (name, true) if name.contains("network") => AnomalyType::NetworkCongestion,
            (name, true) if name.contains("disk") && name.contains("usage") => AnomalyType::DiskSpaceWarning,
            (name, true) if name.contains("queue") => AnomalyType::QueueBacklog,
            (name, _) if name.contains("heartbeat") => AnomalyType::HeartbeatMissed,
            (name, false) if name.contains("health") => AnomalyType::NodeUnresponsive,
            _ => AnomalyType::UnusualPattern,
        }
    }
    
    fn analyze_trend(&self, values: &[f32]) -> TrendDirection {
        if values.len() < 3 {
            return TrendDirection::Unknown;
        }
        
        let recent = &values[values.len().saturating_sub(5)..];
        let trend_score: f32 = recent.windows(2)
            .map(|pair| if pair[1] > pair[0] { 1.0 } else if pair[1] < pair[0] { -1.0 } else { 0.0 })
            .sum();
        
        match trend_score {
            score if score > 1.0 => TrendDirection::Increasing,
            score if score < -1.0 => TrendDirection::Decreasing,
            score if score.abs() <= 1.0 => {
                let variance = recent.iter()
                    .map(|&x| (x - recent.iter().sum::<f32>() / recent.len() as f32).powi(2))
                    .sum::<f32>() / recent.len() as f32;
                if variance > recent.iter().sum::<f32>() / recent.len() as f32 {
                    TrendDirection::Oscillating
                } else {
                    TrendDirection::Stable
                }
            },
            _ => TrendDirection::Unknown,
        }
    }
    
    fn assess_impact(&self, anomaly_type: &AnomalyType, current_value: f32, stats: &Statistics) -> ImpactAssessment {
        let severity_score = ((current_value - stats.mean).abs() / stats.std_dev).min(10.0);
        
        let (affects_ux, stability_risk, data_risk) = match anomaly_type {
            AnomalyType::ErrorRateSpike => (true, true, true),
            AnomalyType::LatencyIncrease => (true, false, false),
            AnomalyType::ThroughputDrop => (true, false, false),
            AnomalyType::MemoryLeak => (false, true, false),
            AnomalyType::CpuSpike => (true, true, false),
            AnomalyType::NetworkCongestion => (true, false, false),
            AnomalyType::DiskSpaceWarning => (false, true, true),
            AnomalyType::NodeUnresponsive => (true, true, true),
            AnomalyType::QueueBacklog => (true, false, false),
            _ => (false, false, false),
        };
        
        let estimated_resolution = match anomaly_type {
            AnomalyType::CpuSpike => Some(ChronoDuration::minutes(5)),
            AnomalyType::MemoryLeak => Some(ChronoDuration::hours(1)),
            AnomalyType::NetworkCongestion => Some(ChronoDuration::minutes(15)),
            AnomalyType::DiskSpaceWarning => Some(ChronoDuration::hours(4)),
            _ => None,
        };
        
        ImpactAssessment {
            severity_score,
            affected_user_experience: affects_ux,
            system_stability_risk: stability_risk,
            data_loss_risk: data_risk,
            estimated_resolution_time: estimated_resolution,
        }
    }
    
    fn generate_suggestions(&self, anomaly_type: &AnomalyType, metric_name: &str, current_value: f32, stats: &Statistics) -> Vec<String> {
        let mut suggestions = Vec::new();
        
        match anomaly_type {
            AnomalyType::CpuSpike => {
                suggestions.push("Check for runaway processes or infinite loops".to_string());
                suggestions.push("Consider scaling horizontally if sustained".to_string());
                suggestions.push("Review recent configuration changes".to_string());
                if current_value > 90.0 {
                    suggestions.push("URGENT: CPU usage critical - immediate intervention required".to_string());
                }
            },
            AnomalyType::MemoryLeak => {
                suggestions.push("Monitor memory usage trends over time".to_string());
                suggestions.push("Check for memory leaks in custom nodes".to_string());
                suggestions.push("Consider restarting affected dataflows".to_string());
                suggestions.push("Review memory allocation patterns in recent deployments".to_string());
            },
            AnomalyType::ErrorRateSpike => {
                suggestions.push("Check logs for error details and patterns".to_string());
                suggestions.push("Verify external service availability".to_string());
                suggestions.push("Review recent deployments or changes".to_string());
                suggestions.push("Implement circuit breaker if not present".to_string());
            },
            AnomalyType::ThroughputDrop => {
                suggestions.push("Check for downstream bottlenecks".to_string());
                suggestions.push("Verify network connectivity and bandwidth".to_string());
                suggestions.push("Review node processing capacity".to_string());
                suggestions.push("Analyze queue depths and backpressure".to_string());
            },
            AnomalyType::LatencyIncrease => {
                suggestions.push("Identify slow operations in the processing pipeline".to_string());
                suggestions.push("Check for resource contention".to_string());
                suggestions.push("Review database query performance".to_string());
                suggestions.push("Consider implementing caching strategies".to_string());
            },
            AnomalyType::NetworkCongestion => {
                suggestions.push("Monitor network bandwidth utilization".to_string());
                suggestions.push("Check for network packet loss".to_string());
                suggestions.push("Consider implementing QoS policies".to_string());
                suggestions.push("Review network topology and routing".to_string());
            },
            AnomalyType::DiskSpaceWarning => {
                suggestions.push("Free up disk space by cleaning temporary files".to_string());
                suggestions.push("Implement log rotation policies".to_string());
                suggestions.push("Consider archiving old data".to_string());
                suggestions.push("Monitor disk growth trends".to_string());
            },
            AnomalyType::QueueBacklog => {
                suggestions.push("Increase processing capacity".to_string());
                suggestions.push("Implement backpressure mechanisms".to_string());
                suggestions.push("Review queue size configurations".to_string());
                suggestions.push("Consider load balancing strategies".to_string());
            },
            _ => {
                suggestions.push(format!("Investigate unusual behavior in {}", metric_name));
                suggestions.push("Compare with historical patterns".to_string());
                suggestions.push("Consider system load and external factors".to_string());
            },
        }
        
        // Add context-aware suggestions based on deviation
        let deviation_ratio = (current_value - stats.mean).abs() / stats.std_dev;
        if deviation_ratio > 3.0 {
            suggestions.push("Consider this a high-priority issue requiring immediate attention".to_string());
        }
        
        suggestions
    }
}

/// Statistical measures for anomaly detection
#[derive(Debug, Clone)]
struct Statistics {
    mean: f32,
    std_dev: f32,
    median: f32,
    min: f32,
    max: f32,
    count: usize,
}

/// Baseline calculator for normal behavior
#[derive(Debug)]
pub struct BaselineCalculator {
    baseline_window: ChronoDuration,
    update_frequency: ChronoDuration,
    cached_baselines: HashMap<String, Baseline>,
}

#[derive(Debug, Clone)]
pub struct Baseline {
    pub mean: f32,
    pub std_dev: f32,
    pub percentiles: HashMap<u8, f32>,
    pub last_updated: DateTime<Utc>,
    pub sample_count: usize,
}

impl BaselineCalculator {
    pub fn new() -> Self {
        Self {
            baseline_window: ChronoDuration::days(7),
            update_frequency: ChronoDuration::hours(1),
            cached_baselines: HashMap::new(),
        }
    }
    
    pub fn calculate_baseline(&mut self, metric_name: &str, historical_data: &TimeSeriesData) -> Option<Baseline> {
        // Check if cached baseline is still valid
        if let Some(cached) = self.cached_baselines.get(metric_name) {
            if (Utc::now() - cached.last_updated) < self.update_frequency {
                return Some(cached.clone());
            }
        }
        
        // Calculate new baseline
        if let Some(values) = historical_data.get_metric_history(metric_name) {
            let floats: Vec<f32> = values.iter().map(|v| v.as_float()).collect();
            
            if floats.len() < 10 {
                return None;
            }
            
            let mean = floats.iter().sum::<f32>() / floats.len() as f32;
            let variance = floats.iter()
                .map(|x| (x - mean).powi(2))
                .sum::<f32>() / floats.len() as f32;
            let std_dev = variance.sqrt();
            
            // Calculate percentiles
            let mut sorted = floats.clone();
            sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
            
            let mut percentiles = HashMap::new();
            for p in [5, 25, 50, 75, 95] {
                let index = (p as f32 / 100.0 * (sorted.len() - 1) as f32) as usize;
                percentiles.insert(p, sorted[index]);
            }
            
            let baseline = Baseline {
                mean,
                std_dev,
                percentiles,
                last_updated: Utc::now(),
                sample_count: floats.len(),
            };
            
            self.cached_baselines.insert(metric_name.to_string(), baseline.clone());
            Some(baseline)
        } else {
            None
        }
    }
}

/// Alert manager for handling anomaly notifications
#[derive(Debug)]
pub struct AlertManager {
    active_alerts: HashMap<String, DateTime<Utc>>,
    cooldown_period: ChronoDuration,
    alert_history: Vec<AlertRecord>,
}

#[derive(Debug, Clone)]
pub struct AlertRecord {
    pub anomaly_id: String,
    pub alert_time: DateTime<Utc>,
    pub severity: AnomalySeverity,
    pub acknowledged: bool,
}

impl AlertManager {
    pub fn new() -> Self {
        Self {
            active_alerts: HashMap::new(),
            cooldown_period: ChronoDuration::minutes(5),
            alert_history: Vec::new(),
        }
    }
    
    pub fn should_alert(&mut self, anomaly: &Anomaly) -> bool {
        let key = format!("{}:{}", anomaly.metric_name, anomaly.anomaly_type);
        
        if let Some(last_alert_time) = self.active_alerts.get(&key) {
            if (Utc::now() - *last_alert_time) < self.cooldown_period {
                return false; // Still in cooldown
            }
        }
        
        // Check severity threshold
        matches!(anomaly.severity, AnomalySeverity::High | AnomalySeverity::Critical)
    }
    
    pub fn record_alert(&mut self, anomaly: &Anomaly) {
        let key = format!("{}:{}", anomaly.metric_name, anomaly.anomaly_type);
        self.active_alerts.insert(key, Utc::now());
        
        self.alert_history.push(AlertRecord {
            anomaly_id: anomaly.id.clone(),
            alert_time: Utc::now(),
            severity: anomaly.severity.clone(),
            acknowledged: false,
        });
    }
}

impl AnomalyDetector {
    pub fn new() -> Self {
        let mut detector = Self {
            detectors: Vec::new(),
            baseline_calculator: BaselineCalculator::new(),
            alert_manager: AlertManager::new(),
            config: AnomalyDetectionConfig::default(),
        };
        
        // Register default detectors
        detector.register_detector(Box::new(StatisticalAnomalyDetector::new(0.5, 20)));
        
        detector
    }
    
    pub fn register_detector(&mut self, detector: Box<dyn AnomalyDetectionAlgorithm>) {
        self.detectors.push(detector);
    }
    
    pub async fn detect_anomalies(
        &mut self,
        current_metrics: &HashMap<ResourceType, HashMap<String, MetricValue>>,
        historical_data: &TimeSeriesData,
    ) -> Result<Vec<Anomaly>> {
        let mut all_anomalies = Vec::new();
        
        for detector in &self.detectors {
            let anomalies = detector.detect_anomalies(current_metrics, historical_data).await?;
            all_anomalies.extend(anomalies);
        }
        
        // Deduplicate and prioritize anomalies
        let filtered_anomalies = self.filter_and_prioritize_anomalies(all_anomalies);
        
        // Handle alerting
        for anomaly in &filtered_anomalies {
            if self.alert_manager.should_alert(anomaly) {
                self.alert_manager.record_alert(anomaly);
            }
        }
        
        Ok(filtered_anomalies)
    }
    
    fn filter_and_prioritize_anomalies(&self, mut anomalies: Vec<Anomaly>) -> Vec<Anomaly> {
        // Sort by severity and confidence
        anomalies.sort_by(|a, b| {
            b.severity.cmp(&a.severity)
                .then_with(|| b.confidence.partial_cmp(&a.confidence).unwrap())
        });
        
        // Remove duplicates (same metric and type)
        let mut seen = std::collections::HashSet::new();
        anomalies.retain(|anomaly| {
            let key = format!("{}:{}", anomaly.metric_name, anomaly.anomaly_type);
            seen.insert(key)
        });
        
        anomalies
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::analysis::time_series::TimeSeriesData;
    
    #[tokio::test]
    async fn test_statistical_anomaly_detection() {
        let detector = StatisticalAnomalyDetector::new(1.0, 10);
        
        // Create test data with normal values and one outlier
        let mut historical_data = TimeSeriesData::new();
        let normal_values: Vec<MetricValue> = (1..=20)
            .map(|i| MetricValue::Float(10.0 + (i as f32 * 0.1)))
            .collect();
        
        historical_data.add_metric_history("test_metric".to_string(), normal_values);
        
        // Create current metrics with an anomalous value
        let mut current_metrics = HashMap::new();
        let mut resource_metrics = HashMap::new();
        resource_metrics.insert("test_metric".to_string(), MetricValue::Float(50.0)); // Clear outlier
        current_metrics.insert(ResourceType::System, resource_metrics);
        
        let anomalies = detector.detect_anomalies(&current_metrics, &historical_data).await.unwrap();
        
        assert!(!anomalies.is_empty());
        assert_eq!(anomalies[0].metric_name, "test_metric");
        assert!(anomalies[0].confidence > 0.5);
    }
    
    #[test]
    fn test_anomaly_type_classification() {
        let detector = StatisticalAnomalyDetector::new(1.0, 10);
        
        assert_eq!(
            detector.classify_anomaly_type("cpu_usage", true, 90.0),
            AnomalyType::CpuSpike
        );
        
        assert_eq!(
            detector.classify_anomaly_type("error_rate", true, 0.1),
            AnomalyType::ErrorRateSpike
        );
        
        assert_eq!(
            detector.classify_anomaly_type("throughput", false, 10.0),
            AnomalyType::ThroughputDrop
        );
    }
    
    #[test]
    fn test_trend_analysis() {
        let detector = StatisticalAnomalyDetector::new(1.0, 10);
        
        let increasing = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        assert_eq!(detector.analyze_trend(&increasing), TrendDirection::Increasing);
        
        let decreasing = vec![5.0, 4.0, 3.0, 2.0, 1.0];
        assert_eq!(detector.analyze_trend(&decreasing), TrendDirection::Decreasing);
        
        let stable = vec![5.0, 5.1, 4.9, 5.0, 5.1];
        assert_eq!(detector.analyze_trend(&stable), TrendDirection::Stable);
    }
    
    #[test]
    fn test_alert_cooldown() {
        let mut alert_manager = AlertManager::new();
        
        let anomaly = Anomaly {
            id: "test".to_string(),
            anomaly_type: AnomalyType::CpuSpike,
            severity: AnomalySeverity::High,
            description: "Test".to_string(),
            affected_resource: "system".to_string(),
            metric_name: "cpu_usage".to_string(),
            current_value: 90.0,
            expected_range: (0.0, 50.0),
            confidence: 0.9,
            first_detected: Utc::now(),
            last_seen: Utc::now(),
            suggestions: vec![],
            context: AnomalyContext {
                affected_components: vec![],
                related_metrics: HashMap::new(),
                trend_direction: TrendDirection::Increasing,
                impact_assessment: ImpactAssessment {
                    severity_score: 8.0,
                    affected_user_experience: true,
                    system_stability_risk: true,
                    data_loss_risk: false,
                    estimated_resolution_time: None,
                },
                detection_algorithm: "test".to_string(),
            },
        };
        
        // First alert should be allowed
        assert!(alert_manager.should_alert(&anomaly));
        alert_manager.record_alert(&anomaly);
        
        // Second alert immediately should be blocked by cooldown
        assert!(!alert_manager.should_alert(&anomaly));
    }
}