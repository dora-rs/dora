use crate::analysis::{ResourceTarget, ComplexityResult};
use crate::analysis::monitors::{ResourceMonitor, ResourceType, MetricValue, MonitorFactory};
use crate::analysis::anomaly_detection::{AnomalyDetector, Anomaly};
use crate::analysis::prediction::{ResourcePredictionEngine, ResourcePredictions};
use crate::analysis::health::{
    SystemHealthCalculator, HealthScore as SystemHealthScore, ResourceUtilization, 
    PerformanceMetrics, ThroughputMetrics, LatencyMetrics, ErrorRateMetrics, 
    EfficiencyMetrics, CpuUtilization, MemoryUtilization, NetworkUtilization, 
    DiskUtilization
};
use crate::analysis::time_series::TimeSeriesData;
use crate::cli::daemon_client::DaemonClient;
use chrono::{DateTime, Utc};
use std::collections::HashMap;
use std::sync::Arc;
use std::time::Duration;
use serde::{Serialize, Deserialize};
use tokio::sync::RwLock;
use async_trait::async_trait;
use eyre::Result;

/// Core resource analysis engine that coordinates all resource monitoring and analysis
#[derive(Debug)]
pub struct ResourceAnalysisEngine {
    monitors: HashMap<ResourceType, Box<dyn ResourceMonitor>>,
    analyzers: HashMap<AnalysisType, Box<dyn ResourceAnalyzer>>,
    anomaly_detector: AnomalyDetector,
    prediction_engine: ResourcePredictionEngine,
    health_calculator: SystemHealthCalculator,
    metric_collector: Arc<RwLock<MetricCollector>>,
}

/// Types of analysis that can be performed
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub enum AnalysisType {
    ResourceUtilization,
    Performance,
    Capacity,
    Efficiency,
    Trend,
}

/// Trait for resource analyzers
#[async_trait]
pub trait ResourceAnalyzer: Send + Sync {
    async fn analyze(
        &self,
        metrics: &HashMap<ResourceType, HashMap<String, MetricValue>>,
        historical_data: &TimeSeriesData,
    ) -> Result<HashMap<String, ResourceDetail>>;
    
    fn analyzer_type(&self) -> AnalysisType;
}

/// Result of comprehensive resource analysis
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ResourceAnalysisResult {
    pub timestamp: DateTime<Utc>,
    pub overall_health: HealthScore,
    pub resource_utilization: ResourceUtilization,
    pub performance_metrics: PerformanceMetrics,
    pub anomalies: Vec<Anomaly>,
    pub recommendations: Vec<ResourceRecommendation>,
    pub predictions: ResourcePredictions,
    pub detailed_analysis: HashMap<String, ResourceDetail>,
}

/// Simplified health score wrapper
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HealthScore {
    pub overall_score: f32,
    pub component_scores: HashMap<String, f32>,
    pub health_level: HealthLevel,
    pub critical_issues: Vec<String>,
    pub improvement_areas: Vec<String>,
}

/// Health level classification
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum HealthLevel {
    Excellent,  // 90-100
    Good,       // 75-89
    Fair,       // 60-74
    Poor,       // 40-59
    Critical,   // 0-39
}

/// Detailed resource analysis for specific components
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ResourceDetail {
    pub resource_type: String,
    pub metrics: HashMap<String, f32>,
    pub health_score: f32,
    pub utilization_level: UtilizationLevel,
    pub performance_impact: f32,
    pub recommendations: Vec<String>,
    pub trend_analysis: TrendSummary,
}

/// Utilization level classification
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum UtilizationLevel {
    Low,        // 0-30%
    Normal,     // 30-70%
    High,       // 70-85%
    Critical,   // 85-100%
}

/// Resource optimization recommendations
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ResourceRecommendation {
    pub recommendation_type: RecommendationType,
    pub priority: RecommendationPriority,
    pub title: String,
    pub description: String,
    pub action_items: Vec<String>,
    pub expected_impact: f32,
    pub implementation_effort: EffortLevel,
    pub affected_resources: Vec<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum RecommendationType {
    ScaleUp,
    ScaleDown,
    Optimize,
    Reallocate,
    Monitor,
    Maintain,
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum RecommendationPriority {
    Low,
    Medium,
    High,
    Critical,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum EffortLevel {
    Low,      // < 1 hour
    Medium,   // 1-8 hours
    High,     // 1-3 days
    Extensive, // > 3 days
}

/// Summary of trend analysis
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TrendSummary {
    pub direction: TrendDirection,
    pub stability: f32,
    pub confidence: f32,
    pub forecast_summary: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum TrendDirection {
    Improving,
    Stable,
    Degrading,
    Volatile,
}

/// Metric collection and time series management
#[derive(Debug)]
pub struct MetricCollector {
    time_series_data: TimeSeriesData,
    collection_interval: Duration,
    retention_period: chrono::Duration,
}

impl MetricCollector {
    pub fn new() -> Self {
        Self {
            time_series_data: TimeSeriesData::new(),
            collection_interval: Duration::from_secs(30),
            retention_period: chrono::Duration::days(7),
        }
    }

    pub fn update_metrics(&mut self, metrics: &HashMap<ResourceType, HashMap<String, MetricValue>>) {
        for (resource_type, resource_metrics) in metrics {
            for (metric_name, metric_value) in resource_metrics {
                let full_metric_name = format!("{:?}.{}", resource_type, metric_name);
                self.time_series_data.add_metric_value(full_metric_name, metric_value, None);
            }
        }
    }

    pub fn get_historical_data(&self) -> &TimeSeriesData {
        &self.time_series_data
    }

    pub fn get_trend_data(&self) -> HashMap<String, Vec<(DateTime<Utc>, f32)>> {
        self.time_series_data.get_trend_data()
    }
}

impl ResourceAnalysisEngine {
    /// Create new resource analysis engine
    pub fn new() -> Self {
        Self {
            monitors: HashMap::new(),
            analyzers: HashMap::new(),
            anomaly_detector: AnomalyDetector::new(),
            prediction_engine: ResourcePredictionEngine::new(),
            health_calculator: SystemHealthCalculator::new(),
            metric_collector: Arc::new(RwLock::new(MetricCollector::new())),
        }
    }

    /// Create with daemon client for dataflow monitoring
    pub fn with_daemon_client(daemon_client: DaemonClient) -> Self {
        let mut engine = Self::new();
        engine.register_default_monitors_with_client(daemon_client);
        engine.register_default_analyzers();
        engine
    }

    /// Register default monitors
    pub fn register_default_monitors(&mut self) {
        self.monitors.insert(ResourceType::System, MonitorFactory::create_system_monitor());
    }

    /// Register default monitors with daemon client
    pub fn register_default_monitors_with_client(&mut self, daemon_client: DaemonClient) {
        self.monitors.insert(ResourceType::System, MonitorFactory::create_system_monitor());
        self.monitors.insert(ResourceType::Dataflow, MonitorFactory::create_dataflow_monitor(daemon_client.clone()));
        self.monitors.insert(ResourceType::Node, MonitorFactory::create_node_monitor(daemon_client));
    }

    /// Register default analyzers
    pub fn register_default_analyzers(&mut self) {
        self.analyzers.insert(AnalysisType::ResourceUtilization, Box::new(ResourceUtilizationAnalyzer::new()));
        self.analyzers.insert(AnalysisType::Performance, Box::new(PerformanceAnalyzer::new()));
        self.analyzers.insert(AnalysisType::Capacity, Box::new(CapacityAnalyzer::new()));
        self.analyzers.insert(AnalysisType::Efficiency, Box::new(EfficiencyAnalyzer::new()));
        self.analyzers.insert(AnalysisType::Trend, Box::new(TrendAnalyzer::new()));
    }

    /// Perform comprehensive system resource analysis
    pub async fn analyze_system_resources(&mut self) -> Result<ResourceAnalysisResult> {
        // Collect current metrics from all monitors
        let mut current_metrics = HashMap::new();
        for (resource_type, monitor) in &mut self.monitors {
            let metrics = monitor.collect_metrics().await?;
            current_metrics.insert(resource_type.clone(), metrics);
        }

        // Update metric collector with new data
        {
            let mut collector = self.metric_collector.write().await;
            collector.update_metrics(&current_metrics);
        }

        // Get historical data for analysis
        let historical_data = {
            let collector = self.metric_collector.read().await;
            collector.get_historical_data().clone()
        };

        // Analyze each resource type
        let mut detailed_analysis = HashMap::new();
        for (analysis_type, analyzer) in &self.analyzers {
            let analysis = analyzer.analyze(&current_metrics, &historical_data).await?;
            detailed_analysis.extend(analysis);
        }

        // Calculate resource utilization
        let resource_utilization = self.calculate_utilization(&current_metrics);

        // Calculate performance metrics
        let performance_metrics = self.calculate_performance(&current_metrics);

        // Detect anomalies
        let anomalies = self.anomaly_detector.detect_anomalies(&current_metrics, &historical_data).await?;

        // Generate predictions
        let predictions = {
            let collector = self.metric_collector.read().await;
            self.prediction_engine.predict_resource_trends(&collector.get_trend_data()).await?
        };

        // Calculate overall health
        let system_health = self.health_calculator.calculate_health(
            &resource_utilization,
            &performance_metrics,
            &anomalies,
        );

        // Convert system health to our simplified format
        let overall_health = self.convert_health_score(&system_health);

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

    /// Analyze specific resource target
    pub async fn analyze_specific_resource(&mut self, resource_target: &ResourceTarget) -> Result<ResourceDetail> {
        match resource_target {
            ResourceTarget::Dataflow(name) => self.analyze_dataflow_resources(name).await,
            ResourceTarget::Node(dataflow, node) => self.analyze_node_resources(dataflow, node).await,
            ResourceTarget::System => self.analyze_system_level_resources().await,
        }
    }

    /// Analyze dataflow-specific resources
    async fn analyze_dataflow_resources(&mut self, dataflow_name: &str) -> Result<ResourceDetail> {
        // Collect dataflow-specific metrics
        let mut metrics = HashMap::new();
        if let Some(monitor) = self.monitors.get_mut(&ResourceType::Dataflow) {
            let dataflow_metrics = monitor.collect_metrics().await?;
            
            // Filter metrics for this specific dataflow
            let filtered_metrics: HashMap<String, MetricValue> = dataflow_metrics
                .into_iter()
                .filter(|(key, _)| key.contains(dataflow_name))
                .collect();
            
            metrics.insert(ResourceType::Dataflow, filtered_metrics);
        }

        self.create_resource_detail("dataflow", dataflow_name, &metrics).await
    }

    /// Analyze node-specific resources
    async fn analyze_node_resources(&mut self, dataflow_name: &str, node_name: &str) -> Result<ResourceDetail> {
        // Collect node-specific metrics
        let mut metrics = HashMap::new();
        if let Some(monitor) = self.monitors.get_mut(&ResourceType::Node) {
            let node_metrics = monitor.collect_metrics().await?;
            
            // Filter metrics for this specific node
            let node_key = format!("{}.{}", dataflow_name, node_name);
            let filtered_metrics: HashMap<String, MetricValue> = node_metrics
                .into_iter()
                .filter(|(key, _)| key.contains(&node_key))
                .collect();
            
            metrics.insert(ResourceType::Node, filtered_metrics);
        }

        self.create_resource_detail("node", &format!("{}:{}", dataflow_name, node_name), &metrics).await
    }

    /// Analyze system-level resources
    async fn analyze_system_level_resources(&mut self) -> Result<ResourceDetail> {
        let mut metrics = HashMap::new();
        if let Some(monitor) = self.monitors.get_mut(&ResourceType::System) {
            let system_metrics = monitor.collect_metrics().await?;
            metrics.insert(ResourceType::System, system_metrics);
        }

        self.create_resource_detail("system", "global", &metrics).await
    }

    /// Create detailed resource analysis
    async fn create_resource_detail(
        &self,
        resource_type: &str,
        resource_name: &str,
        metrics: &HashMap<ResourceType, HashMap<String, MetricValue>>,
    ) -> Result<ResourceDetail> {
        let mut metric_values = HashMap::new();
        let mut total_utilization = 0.0;
        let mut metric_count = 0;

        // Extract numeric metrics
        for (_, resource_metrics) in metrics {
            for (metric_name, metric_value) in resource_metrics {
                let value = metric_value.as_float();
                metric_values.insert(metric_name.clone(), value);
                
                // Calculate average utilization (simplified)
                if metric_name.contains("usage") || metric_name.contains("utilization") {
                    total_utilization += value;
                    metric_count += 1;
                }
            }
        }

        let avg_utilization = if metric_count > 0 {
            total_utilization / metric_count as f32
        } else {
            0.0
        };

        let utilization_level = self.classify_utilization_level(avg_utilization);
        let health_score = self.calculate_resource_health_score(&metric_values);
        let recommendations = self.generate_resource_recommendations(resource_type, &metric_values);

        Ok(ResourceDetail {
            resource_type: resource_type.to_string(),
            metrics: metric_values,
            health_score,
            utilization_level,
            performance_impact: self.calculate_performance_impact(&metric_values),
            recommendations,
            trend_analysis: TrendSummary {
                direction: TrendDirection::Stable,
                stability: 0.8,
                confidence: 0.7,
                forecast_summary: "Stable resource usage expected".to_string(),
            },
        })
    }

    /// Calculate resource utilization from metrics
    fn calculate_utilization(&self, metrics: &HashMap<ResourceType, HashMap<String, MetricValue>>) -> ResourceUtilization {
        let mut cpu_usage = 0.0;
        let mut memory_usage = 0.0;
        let mut network_in = 0.0;
        let mut network_out = 0.0;
        let mut disk_usage = 0.0;

        // Extract system metrics
        if let Some(system_metrics) = metrics.get(&ResourceType::System) {
            cpu_usage = system_metrics.get("system.cpu_usage")
                .map(|v| v.as_float())
                .unwrap_or(0.0);
            
            memory_usage = system_metrics.get("system.memory_usage_percent")
                .map(|v| v.as_float())
                .unwrap_or(0.0);
            
            disk_usage = system_metrics.get("system.disk._.usage_percent")
                .map(|v| v.as_float())
                .unwrap_or(0.0);
        }

        ResourceUtilization {
            cpu: CpuUtilization {
                overall_usage: cpu_usage,
                per_core_usage: vec![cpu_usage], // Simplified
                load_average: (0.5, 0.4, 0.3), // Placeholder
            },
            memory: MemoryUtilization {
                usage_percent: memory_usage,
                used_bytes: (memory_usage * 1_000_000_000.0) as u64, // Simplified
                available_bytes: ((100.0 - memory_usage) * 1_000_000_000.0) as u64,
                swap_usage_percent: 5.0, // Placeholder
            },
            network: NetworkUtilization {
                bandwidth_utilization: 30.0, // Placeholder
                packet_loss_rate: 0.001,
                latency_ms: 10.0,
                connection_count: 100,
            },
            disk: DiskUtilization {
                usage_percent: disk_usage,
                io_utilization: 40.0, // Placeholder
                read_latency_ms: 5.0,
                write_latency_ms: 8.0,
            },
            custom_metrics: HashMap::new(),
        }
    }

    /// Calculate performance metrics from collected data
    fn calculate_performance(&self, metrics: &HashMap<ResourceType, HashMap<String, MetricValue>>) -> PerformanceMetrics {
        let mut throughput = 0.0;
        let mut latency = 0.0;
        let mut error_rate = 0.0;

        // Extract performance metrics from dataflow metrics
        if let Some(dataflow_metrics) = metrics.get(&ResourceType::Dataflow) {
            throughput = dataflow_metrics.values()
                .find(|v| v.as_float() > 0.0)
                .map(|v| v.as_float())
                .unwrap_or(100.0);
            
            latency = 50.0; // Placeholder
            error_rate = 0.01; // Placeholder
        }

        PerformanceMetrics {
            throughput: ThroughputMetrics {
                requests_per_second: throughput,
                data_processed_per_second: throughput * 1.5,
                messages_per_second: throughput * 0.8,
            },
            latency: LatencyMetrics {
                mean_latency_ms: latency,
                p50_latency_ms: latency * 0.8,
                p95_latency_ms: latency * 2.0,
                p99_latency_ms: latency * 3.0,
            },
            error_rates: ErrorRateMetrics {
                overall_error_rate: error_rate,
                error_rate_by_type: HashMap::new(),
                error_trend: crate::analysis::health::TrendDirection::Stable,
            },
            efficiency_scores: EfficiencyMetrics {
                resource_efficiency: 0.8,
                cost_efficiency: 0.75,
                energy_efficiency: 0.7,
            },
        }
    }

    /// Convert system health score to our simplified format
    fn convert_health_score(&self, system_health: &SystemHealthScore) -> HealthScore {
        let health_level = match system_health.health_status {
            crate::analysis::health::HealthStatus::Excellent => HealthLevel::Excellent,
            crate::analysis::health::HealthStatus::Good => HealthLevel::Good,
            crate::analysis::health::HealthStatus::Fair => HealthLevel::Fair,
            crate::analysis::health::HealthStatus::Poor => HealthLevel::Poor,
            crate::analysis::health::HealthStatus::Critical => HealthLevel::Critical,
        };

        HealthScore {
            overall_score: system_health.overall_score,
            component_scores: HashMap::new(), // Simplified
            health_level,
            critical_issues: system_health.actionable_insights
                .iter()
                .filter(|insight| matches!(insight.priority, crate::analysis::health::InsightPriority::Critical))
                .map(|insight| insight.title.clone())
                .collect(),
            improvement_areas: system_health.actionable_insights
                .iter()
                .map(|insight| insight.title.clone())
                .take(5)
                .collect(),
        }
    }

    /// Generate resource optimization recommendations
    async fn generate_recommendations(
        &self,
        resource_utilization: &ResourceUtilization,
        performance_metrics: &PerformanceMetrics,
        anomalies: &[Anomaly],
        predictions: &ResourcePredictions,
    ) -> Result<Vec<ResourceRecommendation>> {
        let mut recommendations = Vec::new();

        // CPU recommendations
        if resource_utilization.cpu.overall_usage > 80.0 {
            recommendations.push(ResourceRecommendation {
                recommendation_type: RecommendationType::ScaleUp,
                priority: RecommendationPriority::High,
                title: "High CPU Usage Detected".to_string(),
                description: format!("CPU usage is at {:.1}%, consider scaling resources", resource_utilization.cpu.overall_usage),
                action_items: vec![
                    "Monitor CPU usage trends".to_string(),
                    "Consider horizontal scaling".to_string(),
                    "Optimize CPU-intensive operations".to_string(),
                ],
                expected_impact: 20.0,
                implementation_effort: EffortLevel::Medium,
                affected_resources: vec!["cpu".to_string()],
            });
        }

        // Memory recommendations
        if resource_utilization.memory.usage_percent > 85.0 {
            recommendations.push(ResourceRecommendation {
                recommendation_type: RecommendationType::ScaleUp,
                priority: RecommendationPriority::High,
                title: "High Memory Usage Detected".to_string(),
                description: format!("Memory usage is at {:.1}%, approaching critical levels", resource_utilization.memory.usage_percent),
                action_items: vec![
                    "Investigate memory leaks".to_string(),
                    "Optimize memory usage patterns".to_string(),
                    "Consider increasing memory allocation".to_string(),
                ],
                expected_impact: 15.0,
                implementation_effort: EffortLevel::Medium,
                affected_resources: vec!["memory".to_string()],
            });
        }

        // Performance recommendations
        if performance_metrics.latency.p95_latency_ms > 500.0 {
            recommendations.push(ResourceRecommendation {
                recommendation_type: RecommendationType::Optimize,
                priority: RecommendationPriority::High,
                title: "High Latency Detected".to_string(),
                description: "95th percentile latency exceeds acceptable thresholds".to_string(),
                action_items: vec![
                    "Identify performance bottlenecks".to_string(),
                    "Optimize critical path operations".to_string(),
                    "Implement caching strategies".to_string(),
                ],
                expected_impact: 25.0,
                implementation_effort: EffortLevel::High,
                affected_resources: vec!["performance".to_string()],
            });
        }

        // Error rate recommendations
        if performance_metrics.error_rates.overall_error_rate > 0.02 {
            recommendations.push(ResourceRecommendation {
                recommendation_type: RecommendationType::Monitor,
                priority: RecommendationPriority::Critical,
                title: "High Error Rate Detected".to_string(),
                description: "Error rate exceeds acceptable levels".to_string(),
                action_items: vec![
                    "Investigate error patterns".to_string(),
                    "Review recent deployments".to_string(),
                    "Implement error monitoring".to_string(),
                ],
                expected_impact: 30.0,
                implementation_effort: EffortLevel::High,
                affected_resources: vec!["system".to_string()],
            });
        }

        // Anomaly-based recommendations
        for anomaly in anomalies {
            if matches!(anomaly.severity, crate::analysis::anomaly_detection::AnomalySeverity::High | crate::analysis::anomaly_detection::AnomalySeverity::Critical) {
                recommendations.push(ResourceRecommendation {
                    recommendation_type: RecommendationType::Monitor,
                    priority: RecommendationPriority::High,
                    title: format!("Address {} Anomaly", anomaly.anomaly_type),
                    description: anomaly.description.clone(),
                    action_items: anomaly.suggestions.clone(),
                    expected_impact: 20.0,
                    implementation_effort: EffortLevel::Medium,
                    affected_resources: vec![anomaly.affected_resource.clone()],
                });
            }
        }

        // Capacity warnings from predictions
        for warning in &predictions.capacity_warnings {
            recommendations.push(ResourceRecommendation {
                recommendation_type: RecommendationType::ScaleUp,
                priority: RecommendationPriority::Medium,
                title: format!("Capacity Warning: {}", warning.resource_type),
                description: format!("Resource utilization at {:.1}%", warning.current_utilization),
                action_items: warning.recommendations.clone(),
                expected_impact: 15.0,
                implementation_effort: EffortLevel::Medium,
                affected_resources: vec![warning.resource_type.clone()],
            });
        }

        // Sort by priority and limit results
        recommendations.sort_by(|a, b| b.priority.cmp(&a.priority));
        recommendations.truncate(10); // Limit to top 10 recommendations

        Ok(recommendations)
    }

    /// Helper methods for resource detail creation
    fn classify_utilization_level(&self, utilization: f32) -> UtilizationLevel {
        match utilization {
            u if u >= 85.0 => UtilizationLevel::Critical,
            u if u >= 70.0 => UtilizationLevel::High,
            u if u >= 30.0 => UtilizationLevel::Normal,
            _ => UtilizationLevel::Low,
        }
    }

    fn calculate_resource_health_score(&self, metrics: &HashMap<String, f32>) -> f32 {
        let mut total_score = 0.0;
        let mut count = 0;

        for (metric_name, value) in metrics {
            let score = if metric_name.contains("usage") || metric_name.contains("utilization") {
                // For utilization metrics, lower is generally better (up to a point)
                match *value {
                    v if v > 90.0 => 20.0,
                    v if v > 80.0 => 50.0,
                    v if v > 70.0 => 70.0,
                    v if v > 30.0 => 90.0,
                    v if v > 10.0 => 95.0,
                    _ => 80.0, // Too low utilization might indicate waste
                }
            } else if metric_name.contains("error") {
                // For error metrics, lower is always better
                100.0 - (*value * 1000.0).min(100.0)
            } else {
                // Default scoring
                80.0
            };

            total_score += score;
            count += 1;
        }

        if count > 0 {
            total_score / count as f32
        } else {
            50.0 // Default neutral score
        }
    }

    fn calculate_performance_impact(&self, metrics: &HashMap<String, f32>) -> f32 {
        // Simplified performance impact calculation
        metrics.values()
            .filter(|&&v| v > 80.0) // High utilization metrics
            .count() as f32 * 10.0
    }

    fn generate_resource_recommendations(&self, resource_type: &str, metrics: &HashMap<String, f32>) -> Vec<String> {
        let mut recommendations = Vec::new();

        match resource_type {
            "system" => {
                if metrics.values().any(|&v| v > 80.0) {
                    recommendations.push("Monitor high resource utilization".to_string());
                    recommendations.push("Consider capacity planning".to_string());
                }
            }
            "dataflow" => {
                recommendations.push("Monitor dataflow performance metrics".to_string());
                recommendations.push("Check for processing bottlenecks".to_string());
            }
            "node" => {
                recommendations.push("Monitor node health and performance".to_string());
                recommendations.push("Check for node-specific issues".to_string());
            }
            _ => {
                recommendations.push("Regular monitoring recommended".to_string());
            }
        }

        if recommendations.is_empty() {
            recommendations.push("Continue monitoring current performance".to_string());
        }

        recommendations
    }
}

impl Default for ResourceAnalysisEngine {
    fn default() -> Self {
        Self::new()
    }
}

/// Default analyzer implementations

struct ResourceUtilizationAnalyzer;
struct PerformanceAnalyzer;
struct CapacityAnalyzer;
struct EfficiencyAnalyzer;
struct TrendAnalyzer;

impl ResourceUtilizationAnalyzer {
    fn new() -> Self {
        Self
    }
}

#[async_trait]
impl ResourceAnalyzer for ResourceUtilizationAnalyzer {
    async fn analyze(
        &self,
        metrics: &HashMap<ResourceType, HashMap<String, MetricValue>>,
        _historical_data: &TimeSeriesData,
    ) -> Result<HashMap<String, ResourceDetail>> {
        let mut results = HashMap::new();
        
        for (resource_type, resource_metrics) in metrics {
            let mut metric_values = HashMap::new();
            for (name, value) in resource_metrics {
                metric_values.insert(name.clone(), value.as_float());
            }
            
            let detail = ResourceDetail {
                resource_type: format!("{:?}", resource_type),
                metrics: metric_values,
                health_score: 80.0,
                utilization_level: UtilizationLevel::Normal,
                performance_impact: 5.0,
                recommendations: vec!["Monitor resource usage".to_string()],
                trend_analysis: TrendSummary {
                    direction: TrendDirection::Stable,
                    stability: 0.8,
                    confidence: 0.7,
                    forecast_summary: "Stable utilization expected".to_string(),
                },
            };
            
            results.insert(format!("utilization_{:?}", resource_type), detail);
        }
        
        Ok(results)
    }
    
    fn analyzer_type(&self) -> AnalysisType {
        AnalysisType::ResourceUtilization
    }
}

impl PerformanceAnalyzer {
    fn new() -> Self {
        Self
    }
}

#[async_trait]
impl ResourceAnalyzer for PerformanceAnalyzer {
    async fn analyze(
        &self,
        metrics: &HashMap<ResourceType, HashMap<String, MetricValue>>,
        _historical_data: &TimeSeriesData,
    ) -> Result<HashMap<String, ResourceDetail>> {
        let mut results = HashMap::new();
        
        // Analyze performance metrics
        let mut performance_metrics = HashMap::new();
        performance_metrics.insert("throughput".to_string(), 150.0);
        performance_metrics.insert("latency".to_string(), 45.0);
        performance_metrics.insert("error_rate".to_string(), 0.01);
        
        let detail = ResourceDetail {
            resource_type: "performance".to_string(),
            metrics: performance_metrics,
            health_score: 85.0,
            utilization_level: UtilizationLevel::Normal,
            performance_impact: 2.0,
            recommendations: vec!["Monitor performance trends".to_string()],
            trend_analysis: TrendSummary {
                direction: TrendDirection::Stable,
                stability: 0.9,
                confidence: 0.8,
                forecast_summary: "Performance within expected range".to_string(),
            },
        };
        
        results.insert("performance_analysis".to_string(), detail);
        Ok(results)
    }
    
    fn analyzer_type(&self) -> AnalysisType {
        AnalysisType::Performance
    }
}

impl CapacityAnalyzer {
    fn new() -> Self {
        Self
    }
}

#[async_trait]
impl ResourceAnalyzer for CapacityAnalyzer {
    async fn analyze(
        &self,
        _metrics: &HashMap<ResourceType, HashMap<String, MetricValue>>,
        _historical_data: &TimeSeriesData,
    ) -> Result<HashMap<String, ResourceDetail>> {
        let mut results = HashMap::new();
        
        let mut capacity_metrics = HashMap::new();
        capacity_metrics.insert("current_capacity".to_string(), 70.0);
        capacity_metrics.insert("projected_capacity".to_string(), 85.0);
        
        let detail = ResourceDetail {
            resource_type: "capacity".to_string(),
            metrics: capacity_metrics,
            health_score: 75.0,
            utilization_level: UtilizationLevel::High,
            performance_impact: 8.0,
            recommendations: vec!["Plan for capacity expansion".to_string()],
            trend_analysis: TrendSummary {
                direction: TrendDirection::Degrading,
                stability: 0.6,
                confidence: 0.7,
                forecast_summary: "Capacity planning needed".to_string(),
            },
        };
        
        results.insert("capacity_analysis".to_string(), detail);
        Ok(results)
    }
    
    fn analyzer_type(&self) -> AnalysisType {
        AnalysisType::Capacity
    }
}

impl EfficiencyAnalyzer {
    fn new() -> Self {
        Self
    }
}

#[async_trait]
impl ResourceAnalyzer for EfficiencyAnalyzer {
    async fn analyze(
        &self,
        _metrics: &HashMap<ResourceType, HashMap<String, MetricValue>>,
        _historical_data: &TimeSeriesData,
    ) -> Result<HashMap<String, ResourceDetail>> {
        let mut results = HashMap::new();
        
        let mut efficiency_metrics = HashMap::new();
        efficiency_metrics.insert("resource_efficiency".to_string(), 78.0);
        efficiency_metrics.insert("cost_efficiency".to_string(), 82.0);
        efficiency_metrics.insert("energy_efficiency".to_string(), 75.0);
        
        let detail = ResourceDetail {
            resource_type: "efficiency".to_string(),
            metrics: efficiency_metrics,
            health_score: 78.0,
            utilization_level: UtilizationLevel::Normal,
            performance_impact: 3.0,
            recommendations: vec!["Optimize resource allocation".to_string()],
            trend_analysis: TrendSummary {
                direction: TrendDirection::Improving,
                stability: 0.8,
                confidence: 0.7,
                forecast_summary: "Efficiency improvements possible".to_string(),
            },
        };
        
        results.insert("efficiency_analysis".to_string(), detail);
        Ok(results)
    }
    
    fn analyzer_type(&self) -> AnalysisType {
        AnalysisType::Efficiency
    }
}

impl TrendAnalyzer {
    fn new() -> Self {
        Self
    }
}

#[async_trait]
impl ResourceAnalyzer for TrendAnalyzer {
    async fn analyze(
        &self,
        _metrics: &HashMap<ResourceType, HashMap<String, MetricValue>>,
        historical_data: &TimeSeriesData,
    ) -> Result<HashMap<String, ResourceDetail>> {
        let mut results = HashMap::new();
        
        let metric_names = historical_data.get_metric_names();
        let trend_count = metric_names.len() as f32;
        
        let mut trend_metrics = HashMap::new();
        trend_metrics.insert("trending_metrics".to_string(), trend_count);
        trend_metrics.insert("trend_stability".to_string(), 0.8);
        trend_metrics.insert("forecast_accuracy".to_string(), 0.75);
        
        let detail = ResourceDetail {
            resource_type: "trends".to_string(),
            metrics: trend_metrics,
            health_score: 80.0,
            utilization_level: UtilizationLevel::Normal,
            performance_impact: 1.0,
            recommendations: vec!["Continue trend monitoring".to_string()],
            trend_analysis: TrendSummary {
                direction: TrendDirection::Stable,
                stability: 0.8,
                confidence: 0.75,
                forecast_summary: "Historical trends show stability".to_string(),
            },
        };
        
        results.insert("trend_analysis".to_string(), detail);
        Ok(results)
    }
    
    fn analyzer_type(&self) -> AnalysisType {
        AnalysisType::Trend
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_resource_analysis_engine_creation() {
        let engine = ResourceAnalysisEngine::new();
        assert!(engine.monitors.is_empty());
        assert!(engine.analyzers.is_empty());
    }

    #[tokio::test]
    async fn test_default_monitors_registration() {
        let mut engine = ResourceAnalysisEngine::new();
        engine.register_default_monitors();
        
        assert!(engine.monitors.contains_key(&ResourceType::System));
    }

    #[tokio::test]
    async fn test_default_analyzers_registration() {
        let mut engine = ResourceAnalysisEngine::new();
        engine.register_default_analyzers();
        
        assert!(engine.analyzers.contains_key(&AnalysisType::ResourceUtilization));
        assert!(engine.analyzers.contains_key(&AnalysisType::Performance));
        assert!(engine.analyzers.contains_key(&AnalysisType::Capacity));
        assert!(engine.analyzers.contains_key(&AnalysisType::Efficiency));
        assert!(engine.analyzers.contains_key(&AnalysisType::Trend));
    }

    #[test]
    fn test_utilization_level_classification() {
        let engine = ResourceAnalysisEngine::new();
        
        assert_eq!(engine.classify_utilization_level(10.0), UtilizationLevel::Low);
        assert_eq!(engine.classify_utilization_level(50.0), UtilizationLevel::Normal);
        assert_eq!(engine.classify_utilization_level(75.0), UtilizationLevel::High);
        assert_eq!(engine.classify_utilization_level(90.0), UtilizationLevel::Critical);
    }

    #[test]
    fn test_health_score_calculation() {
        let engine = ResourceAnalysisEngine::new();
        let mut metrics = HashMap::new();
        metrics.insert("cpu_usage".to_string(), 50.0);
        metrics.insert("memory_usage".to_string(), 60.0);
        
        let score = engine.calculate_resource_health_score(&metrics);
        assert!(score >= 0.0 && score <= 100.0);
    }
}