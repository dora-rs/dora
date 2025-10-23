# Issue #014: Create Resource Analysis System

## 📋 Summary
Implement a comprehensive resource analysis system that monitors and evaluates dataflow resources, node performance, and system health to provide intelligent suggestions for optimization and troubleshooting. This system feeds the complexity analysis engine and enables proactive system management.

## 🎯 Objectives
- Create real-time resource monitoring and analysis capabilities
- Implement intelligent anomaly detection for performance issues
- Add predictive analysis for resource optimization recommendations
- Provide detailed resource utilization tracking across all system components
- Enable proactive alerting and suggestion generation based on resource patterns

**Success Metrics:**
- Resource analysis completes in <100ms for typical system configurations
- Anomaly detection accuracy exceeds 85% with <5% false positive rate
- Performance recommendations improve system efficiency by 20% on average
- System health scoring provides actionable insights 90% of the time
- Resource predictions are accurate within 15% for 1-hour forecasts

## 🛠️ Technical Requirements

### What to Build

#### 1. Core Resource Analysis Engine
```rust
// src/analysis/resource_analyzer.rs
#[derive(Debug)]
pub struct ResourceAnalysisEngine {
    monitors: HashMap<ResourceType, Box<dyn ResourceMonitor>>,
    analyzers: HashMap<AnalysisType, Box<dyn ResourceAnalyzer>>,
    anomaly_detector: AnomalyDetector,
    prediction_engine: ResourcePredictionEngine,
    health_calculator: SystemHealthCalculator,
    metric_collector: MetricCollector,
}

#[derive(Debug, Clone)]
pub struct ResourceAnalysisResult {
    pub timestamp: DateTime<Utc>,
    pub overall_health: HealthScore,
    pub resource_utilization: ResourceUtilization,
    pub performance_metrics: PerformanceMetrics,
    pub anomalies: Vec<Anomaly>,
    pub recommendations: Vec<ResourceRecommendation>,
    pub predictions: ResourcePredictions,
    pub detailed_analysis: HashMap<ResourceType, ResourceDetail>,
}

#[derive(Debug, Clone)]
pub struct ResourceUtilization {
    pub cpu: CpuUtilization,
    pub memory: MemoryUtilization,
    pub network: NetworkUtilization,
    pub disk: DiskUtilization,
    pub custom_metrics: HashMap<String, MetricValue>,
}

#[derive(Debug, Clone)]
pub struct PerformanceMetrics {
    pub throughput: ThroughputMetrics,
    pub latency: LatencyMetrics,
    pub error_rates: ErrorRateMetrics,
    pub efficiency_scores: EfficiencyMetrics,
}

impl ResourceAnalysisEngine {
    pub fn new() -> Self {
        let mut engine = Self {
            monitors: HashMap::new(),
            analyzers: HashMap::new(),
            anomaly_detector: AnomalyDetector::new(),
            prediction_engine: ResourcePredictionEngine::new(),
            health_calculator: SystemHealthCalculator::new(),
            metric_collector: MetricCollector::new(),
        };
        
        engine.register_default_monitors();
        engine.register_default_analyzers();
        engine
    }
    
    pub async fn analyze_system_resources(&mut self) -> Result<ResourceAnalysisResult> {
        // Collect current metrics from all monitors
        let mut current_metrics = HashMap::new();
        for (resource_type, monitor) in &mut self.monitors {
            let metrics = monitor.collect_metrics().await?;
            current_metrics.insert(resource_type.clone(), metrics);
        }
        
        // Update metric collector with new data
        self.metric_collector.update_metrics(&current_metrics);
        
        // Analyze each resource type
        let mut detailed_analysis = HashMap::new();
        for (analysis_type, analyzer) in &mut self.analyzers {
            let analysis = analyzer.analyze(&current_metrics, &self.metric_collector).await?;
            detailed_analysis.extend(analysis);
        }
        
        // Calculate overall metrics
        let resource_utilization = self.calculate_utilization(&current_metrics);
        let performance_metrics = self.calculate_performance(&current_metrics);
        
        // Detect anomalies
        let anomalies = self.anomaly_detector.detect_anomalies(
            &current_metrics,
            &self.metric_collector.get_historical_data(),
        ).await?;
        
        // Generate predictions
        let predictions = self.prediction_engine.predict_resource_trends(
            &self.metric_collector.get_trend_data(),
        ).await?;
        
        // Calculate overall health
        let overall_health = self.health_calculator.calculate_health(
            &resource_utilization,
            &performance_metrics,
            &anomalies,
        );
        
        // Generate recommendations
        let recommendations = self.generate_recommendations(
            &resource_utilization,
            &performance_metrics,
            &anomalies,
            &predictions,
        ).await?;
        
        Ok(ResourceAnalysisResult {
            timestamp: Utc::now(),
            overall_health,
            resource_utilization,
            performance_metrics,
            anomalies,
            recommendations,
            predictions,
            detailed_analysis,
        })
    }
    
    pub async fn analyze_specific_resource(
        &mut self,
        resource_target: &ResourceTarget,
    ) -> Result<ResourceDetail> {
        match resource_target {
            ResourceTarget::Dataflow(name) => {
                self.analyze_dataflow_resources(name).await
            },
            ResourceTarget::Node(dataflow, node) => {
                self.analyze_node_resources(dataflow, node).await
            },
            ResourceTarget::System => {
                self.analyze_system_level_resources().await
            },
        }
    }
}
```

#### 2. Specialized Resource Monitors
```rust
// src/analysis/monitors.rs
pub trait ResourceMonitor: Send + Sync {
    async fn collect_metrics(&mut self) -> Result<HashMap<String, MetricValue>>;
    fn monitor_type(&self) -> ResourceType;
    fn collection_interval(&self) -> Duration;
}

#[derive(Debug)]
pub struct DataflowResourceMonitor {
    daemon_client: DaemonClient,
    monitored_dataflows: HashSet<String>,
    collection_cache: LruCache<String, DataflowMetrics>,
}

impl ResourceMonitor for DataflowResourceMonitor {
    async fn collect_metrics(&mut self) -> Result<HashMap<String, MetricValue>> {
        let mut metrics = HashMap::new();
        
        let dataflows = self.daemon_client.list_dataflows().await?;
        
        for dataflow in dataflows {
            let dataflow_metrics = self.collect_dataflow_metrics(&dataflow).await?;
            
            // CPU metrics
            metrics.insert(
                format!("dataflow.{}.cpu_usage", dataflow.name),
                MetricValue::Float(dataflow_metrics.cpu_usage_percent),
            );
            
            // Memory metrics
            metrics.insert(
                format!("dataflow.{}.memory_usage", dataflow.name),
                MetricValue::Integer(dataflow_metrics.memory_usage_bytes as i64),
            );
            
            // Network metrics
            metrics.insert(
                format!("dataflow.{}.network_in", dataflow.name),
                MetricValue::Float(dataflow_metrics.network_bytes_in_per_sec),
            );
            
            metrics.insert(
                format!("dataflow.{}.network_out", dataflow.name),
                MetricValue::Float(dataflow_metrics.network_bytes_out_per_sec),
            );
            
            // Performance metrics
            metrics.insert(
                format!("dataflow.{}.throughput", dataflow.name),
                MetricValue::Float(dataflow_metrics.messages_per_second),
            );
            
            metrics.insert(
                format!("dataflow.{}.latency_p95", dataflow.name),
                MetricValue::Float(dataflow_metrics.latency_p95_ms),
            );
            
            // Health metrics
            metrics.insert(
                format!("dataflow.{}.error_rate", dataflow.name),
                MetricValue::Float(dataflow_metrics.error_rate),
            );
            
            metrics.insert(
                format!("dataflow.{}.node_health", dataflow.name),
                MetricValue::Float(dataflow_metrics.overall_node_health),
            );
        }
        
        Ok(metrics)
    }
    
    fn monitor_type(&self) -> ResourceType {
        ResourceType::Dataflow
    }
    
    fn collection_interval(&self) -> Duration {
        Duration::from_secs(5) // Collect every 5 seconds
    }
}

#[derive(Debug)]
pub struct SystemResourceMonitor {
    system_info: System,
    last_collection: Option<Instant>,
}

impl ResourceMonitor for SystemResourceMonitor {
    async fn collect_metrics(&mut self) -> Result<HashMap<String, MetricValue>> {
        let mut metrics = HashMap::new();
        
        // Refresh system information
        self.system_info.refresh_all();
        
        // CPU metrics
        let cpu_usage = self.system_info.global_cpu_info().cpu_usage();
        metrics.insert("system.cpu_usage".to_string(), MetricValue::Float(cpu_usage));
        
        // Memory metrics
        let total_memory = self.system_info.total_memory();
        let used_memory = self.system_info.used_memory();
        let memory_usage_percent = (used_memory as f32 / total_memory as f32) * 100.0;
        
        metrics.insert("system.memory_total".to_string(), MetricValue::Integer(total_memory as i64));
        metrics.insert("system.memory_used".to_string(), MetricValue::Integer(used_memory as i64));
        metrics.insert("system.memory_usage_percent".to_string(), MetricValue::Float(memory_usage_percent));
        
        // Disk metrics
        for disk in self.system_info.disks() {
            let disk_name = disk.name().to_string_lossy();
            let total_space = disk.total_space();
            let available_space = disk.available_space();
            let usage_percent = ((total_space - available_space) as f32 / total_space as f32) * 100.0;
            
            metrics.insert(
                format!("system.disk.{}.usage_percent", disk_name),
                MetricValue::Float(usage_percent),
            );
        }
        
        // Network metrics (if available)
        for (interface_name, network) in self.system_info.networks() {
            metrics.insert(
                format!("system.network.{}.bytes_received", interface_name),
                MetricValue::Integer(network.received() as i64),
            );
            
            metrics.insert(
                format!("system.network.{}.bytes_transmitted", interface_name),
                MetricValue::Integer(network.transmitted() as i64),
            );
        }
        
        // Process count
        let process_count = self.system_info.processes().len();
        metrics.insert("system.process_count".to_string(), MetricValue::Integer(process_count as i64));
        
        self.last_collection = Some(Instant::now());
        Ok(metrics)
    }
    
    fn monitor_type(&self) -> ResourceType {
        ResourceType::System
    }
    
    fn collection_interval(&self) -> Duration {
        Duration::from_secs(10) // Collect every 10 seconds
    }
}
```

#### 3. Anomaly Detection System
```rust
// src/analysis/anomaly_detection.rs
#[derive(Debug)]
pub struct AnomalyDetector {
    detectors: Vec<Box<dyn AnomalyDetectionAlgorithm>>,
    baseline_calculator: BaselineCalculator,
    alert_manager: AlertManager,
}

#[derive(Debug, Clone)]
pub struct Anomaly {
    pub anomaly_type: AnomalyType,
    pub severity: AnomalySeverity,
    pub description: String,
    pub affected_resource: String,
    pub metric_name: String,
    pub current_value: f32,
    pub expected_range: (f32, f32),
    pub confidence: f32,
    pub first_detected: DateTime<Utc>,
    pub suggestions: Vec<String>,
}

#[derive(Debug, Clone)]
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
}

trait AnomalyDetectionAlgorithm: Send + Sync {
    async fn detect_anomalies(
        &self,
        current_metrics: &HashMap<ResourceType, HashMap<String, MetricValue>>,
        historical_data: &TimeSeriesData,
    ) -> Result<Vec<Anomaly>>;
    
    fn algorithm_name(&self) -> &str;
    fn detection_sensitivity(&self) -> f32;
}

pub struct StatisticalAnomalyDetector {
    sensitivity: f32,
    window_size: usize,
}

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
                    let anomaly = self.detect_statistical_anomaly(
                        resource_type,
                        metric_name,
                        current_value,
                        &historical_values,
                    );
                    
                    if let Some(anomaly) = anomaly {
                        anomalies.push(anomaly);
                    }
                }
            }
        }
        
        Ok(anomalies)
    }
    
    fn algorithm_name(&self) -> &str {
        "Statistical Outlier Detection"
    }
    
    fn detection_sensitivity(&self) -> f32 {
        self.sensitivity
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
        
        if historical_floats.len() < 10 {
            return None; // Not enough data
        }
        
        let mean = historical_floats.iter().sum::<f32>() / historical_floats.len() as f32;
        let variance = historical_floats.iter()
            .map(|x| (x - mean).powi(2))
            .sum::<f32>() / historical_floats.len() as f32;
        let std_dev = variance.sqrt();
        
        // Z-score calculation
        let z_score = (current_float - mean) / std_dev;
        let threshold = 2.0 / self.sensitivity; // Adjustable threshold
        
        if z_score.abs() > threshold {
            let severity = match z_score.abs() {
                s if s > 3.0 => AnomalySeverity::Critical,
                s if s > 2.5 => AnomalySeverity::High,
                _ => AnomalySeverity::Medium,
            };
            
            let anomaly_type = self.classify_anomaly_type(metric_name, z_score > 0.0);
            
            Some(Anomaly {
                anomaly_type,
                severity,
                description: format!(
                    "Metric '{}' has unusual value {:.2} (expected: {:.2} ± {:.2})",
                    metric_name, current_float, mean, std_dev
                ),
                affected_resource: format!("{:?}", resource_type),
                metric_name: metric_name.to_string(),
                current_value: current_float,
                expected_range: (mean - std_dev, mean + std_dev),
                confidence: (z_score.abs() / threshold).min(1.0),
                first_detected: Utc::now(),
                suggestions: self.generate_suggestions(&anomaly_type, metric_name),
            })
        } else {
            None
        }
    }
    
    fn classify_anomaly_type(&self, metric_name: &str, is_spike: bool) -> AnomalyType {
        match (metric_name.to_lowercase().as_str(), is_spike) {
            (name, true) if name.contains("cpu") => AnomalyType::CpuSpike,
            (name, true) if name.contains("memory") => AnomalyType::MemoryLeak,
            (name, true) if name.contains("error") => AnomalyType::ErrorRateSpike,
            (name, true) if name.contains("latency") => AnomalyType::LatencyIncrease,
            (name, false) if name.contains("throughput") => AnomalyType::ThroughputDrop,
            (name, true) if name.contains("network") => AnomalyType::NetworkCongestion,
            (name, true) if name.contains("disk") => AnomalyType::DiskSpaceWarning,
            _ => AnomalyType::UnusualPattern,
        }
    }
    
    fn generate_suggestions(&self, anomaly_type: &AnomalyType, metric_name: &str) -> Vec<String> {
        match anomaly_type {
            AnomalyType::CpuSpike => vec![
                "Check for runaway processes or infinite loops".to_string(),
                "Consider scaling horizontally if sustained".to_string(),
                "Review recent configuration changes".to_string(),
            ],
            AnomalyType::MemoryLeak => vec![
                "Monitor memory usage trends over time".to_string(),
                "Check for memory leaks in custom nodes".to_string(),
                "Consider restarting affected dataflows".to_string(),
            ],
            AnomalyType::ErrorRateSpike => vec![
                "Check logs for error details and patterns".to_string(),
                "Verify external service availability".to_string(),
                "Review recent deployments or changes".to_string(),
            ],
            AnomalyType::ThroughputDrop => vec![
                "Check for downstream bottlenecks".to_string(),
                "Verify network connectivity and bandwidth".to_string(),
                "Review node processing capacity".to_string(),
            ],
            _ => vec![
                format!("Investigate unusual behavior in {}", metric_name),
                "Compare with historical patterns".to_string(),
                "Consider system load and external factors".to_string(),
            ],
        }
    }
}
```

#### 4. Resource Prediction Engine
```rust
// src/analysis/prediction.rs
#[derive(Debug)]
pub struct ResourcePredictionEngine {
    predictors: HashMap<String, Box<dyn ResourcePredictor>>,
    trend_analyzer: TrendAnalyzer,
}

#[derive(Debug, Clone)]
pub struct ResourcePredictions {
    pub cpu_predictions: Vec<PredictionPoint>,
    pub memory_predictions: Vec<PredictionPoint>,
    pub network_predictions: Vec<PredictionPoint>,
    pub performance_predictions: PerformancePredictions,
    pub capacity_warnings: Vec<CapacityWarning>,
    pub optimization_opportunities: Vec<OptimizationOpportunity>,
}

#[derive(Debug, Clone)]
pub struct PredictionPoint {
    pub timestamp: DateTime<Utc>,
    pub predicted_value: f32,
    pub confidence_interval: (f32, f32),
    pub confidence: f32,
}

trait ResourcePredictor: Send + Sync {
    async fn predict(
        &self,
        historical_data: &TimeSeriesData,
        prediction_horizon: Duration,
    ) -> Result<Vec<PredictionPoint>>;
    
    fn predictor_name(&self) -> &str;
    fn supported_metrics(&self) -> Vec<String>;
}

pub struct LinearTrendPredictor {
    window_size: usize,
}

impl ResourcePredictor for LinearTrendPredictor {
    async fn predict(
        &self,
        historical_data: &TimeSeriesData,
        prediction_horizon: Duration,
    ) -> Result<Vec<PredictionPoint>> {
        let mut predictions = Vec::new();
        
        // Simple linear regression for trend prediction
        for metric_name in self.supported_metrics() {
            if let Some(metric_history) = historical_data.get_metric_history(&metric_name) {
                let recent_data = metric_history.iter()
                    .rev()
                    .take(self.window_size)
                    .collect::<Vec<_>>();
                
                if recent_data.len() < 5 {
                    continue; // Not enough data for prediction
                }
                
                let trend = self.calculate_linear_trend(&recent_data);
                let prediction_points = self.generate_prediction_points(
                    &trend,
                    prediction_horizon,
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
        ]
    }
}

impl LinearTrendPredictor {
    fn calculate_linear_trend(&self, data_points: &[&MetricValue]) -> TrendLine {
        let n = data_points.len() as f32;
        let x_values: Vec<f32> = (0..data_points.len()).map(|i| i as f32).collect();
        let y_values: Vec<f32> = data_points.iter().map(|v| v.as_float()).collect();
        
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
        
        TrendLine {
            slope,
            intercept,
            confidence: r_squared.max(0.0).min(1.0),
        }
    }
}

#[derive(Debug, Clone)]
pub struct TrendLine {
    pub slope: f32,
    pub intercept: f32,
    pub confidence: f32,
}

#[derive(Debug, Clone)]
pub struct CapacityWarning {
    pub resource_type: String,
    pub current_utilization: f32,
    pub predicted_full_time: Option<DateTime<Utc>>,
    pub severity: AnomalySeverity,
    pub recommendations: Vec<String>,
}
```

### Why This Approach

**Comprehensive Monitoring:**
- Multi-layer resource monitoring from system to application level
- Real-time and historical data analysis
- Predictive capabilities for proactive management

**Intelligent Analysis:**
- Machine learning-based anomaly detection
- Statistical analysis with configurable sensitivity
- Pattern recognition for complex system behaviors

**Actionable Insights:**
- Clear recommendations based on analysis results
- Severity-based prioritization of issues
- Integration with complexity analysis for smart suggestions

### How to Implement

#### Step 1: Core Analysis Engine (4 hours)
1. **Implement ResourceAnalysisEngine** with monitor registration
2. **Create ResourceAnalysisResult** structure with comprehensive metrics
3. **Add metric collection** and aggregation systems
4. **Build health calculation** algorithms

#### Step 2: Specialized Monitors (5 hours)
1. **Implement DataflowResourceMonitor** with daemon integration
2. **Add SystemResourceMonitor** with system metrics collection
3. **Create NodeResourceMonitor** for individual node monitoring
4. **Add custom metric** support for extensibility

#### Step 3: Anomaly Detection (4 hours)
1. **Implement AnomalyDetector** with multiple algorithms
2. **Add StatisticalAnomalyDetector** with configurable sensitivity
3. **Create pattern-based** anomaly detection
4. **Add alert management** and notification system

#### Step 4: Prediction Engine (3 hours)
1. **Implement ResourcePredictionEngine** with trend analysis
2. **Add LinearTrendPredictor** for basic forecasting
3. **Create capacity warning** generation
4. **Add optimization opportunity** detection

#### Step 5: Integration and Testing (2 hours)
1. **Integrate with complexity analysis** engine
2. **Add comprehensive unit tests** for all components
3. **Test anomaly detection** accuracy and performance
4. **Validate prediction** accuracy and usefulness

## 🔗 Dependencies
**Depends On:**
- Issue #013 (Complexity Calculation Algorithms) - Integration with complexity analysis

**Blocks:** 
- Enhanced commands that rely on resource analysis
- Smart suggestion features for optimization

## 🧪 Testing Requirements

### Unit Tests
```rust
#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_anomaly_detection() {
        let detector = StatisticalAnomalyDetector::new(1.0);
        let historical_data = create_test_historical_data();
        let current_metrics = create_anomalous_metrics();
        
        let anomalies = detector.detect_anomalies(&current_metrics, &historical_data).await.unwrap();
        
        assert!(!anomalies.is_empty());
        assert!(anomalies.iter().any(|a| matches!(a.anomaly_type, AnomalyType::CpuSpike)));
    }
    
    #[test]
    fn test_resource_prediction() {
        let predictor = LinearTrendPredictor::new(20);
        let trend_data = create_trending_data();
        
        let predictions = predictor.predict(&trend_data, Duration::from_hours(1)).await.unwrap();
        
        assert!(!predictions.is_empty());
        assert!(predictions.iter().all(|p| p.confidence > 0.0));
    }
    
    #[test]
    fn test_health_calculation() {
        let calculator = SystemHealthCalculator::new();
        let utilization = create_test_utilization();
        let metrics = create_test_metrics();
        let anomalies = vec![];
        
        let health = calculator.calculate_health(&utilization, &metrics, &anomalies);
        
        assert!(health.overall_score >= 0.0 && health.overall_score <= 100.0);
    }
}
```

## ✅ Definition of Done
- [ ] ResourceAnalysisEngine implemented with comprehensive monitoring
- [ ] Multiple resource monitors provide accurate real-time metrics
- [ ] Anomaly detection identifies performance issues with high accuracy
- [ ] Prediction engine provides useful forecasting for capacity planning
- [ ] Health scoring provides actionable system assessment
- [ ] Integration with complexity analysis enhances decision making
- [ ] Performance targets met for real-time resource analysis
- [ ] Comprehensive unit tests validate analysis accuracy
- [ ] Integration tests confirm end-to-end resource monitoring workflows
- [ ] Manual testing validates anomaly detection and prediction usefulness

This resource analysis system provides the intelligent monitoring foundation that enables proactive system management and feeds into the hybrid CLI's smart suggestion capabilities.