use std::collections::HashMap;
use serde::{Serialize, Deserialize};
use chrono::{DateTime, Utc};

use crate::analysis::anomaly_detection::{Anomaly, AnomalySeverity};

/// System health calculator that provides comprehensive health scoring
#[derive(Debug)]
pub struct SystemHealthCalculator {
    weights: HealthWeights,
    thresholds: HealthThresholds,
}

/// Weights for different health factors
#[derive(Debug, Clone)]
pub struct HealthWeights {
    pub resource_utilization: f32,
    pub performance_metrics: f32,
    pub anomaly_impact: f32,
    pub availability: f32,
    pub trend_stability: f32,
}

impl Default for HealthWeights {
    fn default() -> Self {
        Self {
            resource_utilization: 0.25,
            performance_metrics: 0.25,
            anomaly_impact: 0.3,
            availability: 0.15,
            trend_stability: 0.05,
        }
    }
}

/// Thresholds for health scoring
#[derive(Debug, Clone)]
pub struct HealthThresholds {
    pub critical_cpu_threshold: f32,
    pub critical_memory_threshold: f32,
    pub critical_error_rate_threshold: f32,
    pub warning_latency_threshold: f32,
    pub healthy_throughput_threshold: f32,
}

impl Default for HealthThresholds {
    fn default() -> Self {
        Self {
            critical_cpu_threshold: 90.0,
            critical_memory_threshold: 95.0,
            critical_error_rate_threshold: 0.05,
            warning_latency_threshold: 1000.0, // ms
            healthy_throughput_threshold: 100.0, // requests/sec
        }
    }
}

/// Overall system health score and breakdown
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HealthScore {
    pub overall_score: f32, // 0.0 to 100.0
    pub health_status: HealthStatus,
    pub component_scores: ComponentHealthScores,
    pub health_factors: HealthFactorBreakdown,
    pub actionable_insights: Vec<HealthInsight>,
    pub last_calculated: DateTime<Utc>,
}

/// Health status categories
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum HealthStatus {
    Excellent,  // 90-100
    Good,       // 75-89
    Fair,       // 60-74
    Poor,       // 40-59
    Critical,   // 0-39
}

/// Health scores for individual components
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ComponentHealthScores {
    pub system_resources: f32,
    pub dataflow_performance: f32,
    pub node_health: f32,
    pub network_connectivity: f32,
    pub storage_health: f32,
}

/// Breakdown of health factors
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HealthFactorBreakdown {
    pub resource_utilization_score: f32,
    pub performance_score: f32,
    pub anomaly_score: f32,
    pub availability_score: f32,
    pub trend_stability_score: f32,
    pub detailed_metrics: HashMap<String, MetricHealthInfo>,
}

/// Health information for individual metrics
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MetricHealthInfo {
    pub current_value: f32,
    pub health_score: f32,
    pub status: MetricStatus,
    pub trend: TrendHealth,
    pub impact_on_overall: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum MetricStatus {
    Healthy,
    Warning,
    Critical,
    Unknown,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TrendHealth {
    pub direction: TrendDirection,
    pub stability: f32, // 0.0 (very unstable) to 1.0 (very stable)
    pub predictability: f32, // 0.0 (unpredictable) to 1.0 (very predictable)
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum TrendDirection {
    Improving,
    Stable,
    Degrading,
    Volatile,
}

/// Actionable insights for system health improvement
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HealthInsight {
    pub insight_type: InsightType,
    pub priority: InsightPriority,
    pub title: String,
    pub description: String,
    pub recommended_actions: Vec<String>,
    pub potential_impact: f32, // Health score improvement potential
    pub effort_level: EffortLevel,
    pub affected_components: Vec<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum InsightType {
    ResourceOptimization,
    PerformanceImprovement,
    AnomalyResolution,
    CapacityPlanning,
    MaintenanceRequired,
    ConfigurationTuning,
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum InsightPriority {
    Low,
    Medium,
    High,
    Critical,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum EffortLevel {
    Low,     // < 1 hour
    Medium,  // 1-8 hours
    High,    // 1-3 days
    Extensive, // > 3 days
}

/// Resource utilization metrics
#[derive(Debug, Clone)]
pub struct ResourceUtilization {
    pub cpu: CpuUtilization,
    pub memory: MemoryUtilization,
    pub network: NetworkUtilization,
    pub disk: DiskUtilization,
    pub custom_metrics: HashMap<String, f32>,
}

#[derive(Debug, Clone)]
pub struct CpuUtilization {
    pub overall_usage: f32,
    pub per_core_usage: Vec<f32>,
    pub load_average: (f32, f32, f32), // 1min, 5min, 15min
}

#[derive(Debug, Clone)]
pub struct MemoryUtilization {
    pub usage_percent: f32,
    pub used_bytes: u64,
    pub available_bytes: u64,
    pub swap_usage_percent: f32,
}

#[derive(Debug, Clone)]
pub struct NetworkUtilization {
    pub bandwidth_utilization: f32,
    pub packet_loss_rate: f32,
    pub latency_ms: f32,
    pub connection_count: usize,
}

#[derive(Debug, Clone)]
pub struct DiskUtilization {
    pub usage_percent: f32,
    pub io_utilization: f32,
    pub read_latency_ms: f32,
    pub write_latency_ms: f32,
}

/// Performance metrics
#[derive(Debug, Clone)]
pub struct PerformanceMetrics {
    pub throughput: ThroughputMetrics,
    pub latency: LatencyMetrics,
    pub error_rates: ErrorRateMetrics,
    pub efficiency_scores: EfficiencyMetrics,
}

#[derive(Debug, Clone)]
pub struct ThroughputMetrics {
    pub requests_per_second: f32,
    pub data_processed_per_second: f32,
    pub messages_per_second: f32,
}

#[derive(Debug, Clone)]
pub struct LatencyMetrics {
    pub mean_latency_ms: f32,
    pub p50_latency_ms: f32,
    pub p95_latency_ms: f32,
    pub p99_latency_ms: f32,
}

#[derive(Debug, Clone)]
pub struct ErrorRateMetrics {
    pub overall_error_rate: f32,
    pub error_rate_by_type: HashMap<String, f32>,
    pub error_trend: TrendDirection,
}

#[derive(Debug, Clone)]
pub struct EfficiencyMetrics {
    pub resource_efficiency: f32,
    pub cost_efficiency: f32,
    pub energy_efficiency: f32,
}

impl SystemHealthCalculator {
    pub fn new() -> Self {
        Self {
            weights: HealthWeights::default(),
            thresholds: HealthThresholds::default(),
        }
    }

    pub fn with_custom_weights(weights: HealthWeights) -> Self {
        Self {
            weights,
            thresholds: HealthThresholds::default(),
        }
    }

    /// Calculate comprehensive health score
    pub fn calculate_health(
        &self,
        resource_utilization: &ResourceUtilization,
        performance_metrics: &PerformanceMetrics,
        anomalies: &[Anomaly],
    ) -> HealthScore {
        // Calculate individual factor scores
        let resource_score = self.calculate_resource_utilization_score(resource_utilization);
        let performance_score = self.calculate_performance_score(performance_metrics);
        let anomaly_score = self.calculate_anomaly_impact_score(anomalies);
        let availability_score = self.calculate_availability_score(resource_utilization, performance_metrics);
        let trend_stability_score = self.calculate_trend_stability_score(resource_utilization, performance_metrics);

        // Calculate weighted overall score
        let overall_score = 
            resource_score * self.weights.resource_utilization +
            performance_score * self.weights.performance_metrics +
            anomaly_score * self.weights.anomaly_impact +
            availability_score * self.weights.availability +
            trend_stability_score * self.weights.trend_stability;

        let health_status = self.determine_health_status(overall_score);
        
        // Calculate component scores
        let component_scores = self.calculate_component_scores(resource_utilization, performance_metrics);
        
        // Create detailed factor breakdown
        let health_factors = HealthFactorBreakdown {
            resource_utilization_score: resource_score,
            performance_score,
            anomaly_score,
            availability_score,
            trend_stability_score,
            detailed_metrics: self.create_detailed_metrics(resource_utilization, performance_metrics),
        };

        // Generate actionable insights
        let actionable_insights = self.generate_health_insights(
            resource_utilization,
            performance_metrics,
            anomalies,
            overall_score,
        );

        HealthScore {
            overall_score,
            health_status,
            component_scores,
            health_factors,
            actionable_insights,
            last_calculated: Utc::now(),
        }
    }

    fn calculate_resource_utilization_score(&self, utilization: &ResourceUtilization) -> f32 {
        let cpu_score = self.score_cpu_utilization(&utilization.cpu);
        let memory_score = self.score_memory_utilization(&utilization.memory);
        let network_score = self.score_network_utilization(&utilization.network);
        let disk_score = self.score_disk_utilization(&utilization.disk);

        // Weighted average of resource scores
        (cpu_score * 0.3 + memory_score * 0.3 + network_score * 0.2 + disk_score * 0.2) * 100.0
    }

    fn score_cpu_utilization(&self, cpu: &CpuUtilization) -> f32 {
        // Score based on utilization level
        match cpu.overall_usage {
            usage if usage > self.thresholds.critical_cpu_threshold => 0.2,
            usage if usage > 80.0 => 0.5,
            usage if usage > 60.0 => 0.8,
            usage if usage > 40.0 => 0.95,
            _ => 1.0,
        }
    }

    fn score_memory_utilization(&self, memory: &MemoryUtilization) -> f32 {
        let base_score = match memory.usage_percent {
            usage if usage > self.thresholds.critical_memory_threshold => 0.1,
            usage if usage > 85.0 => 0.4,
            usage if usage > 70.0 => 0.7,
            usage if usage > 50.0 => 0.9,
            _ => 1.0,
        };

        // Penalty for high swap usage
        let swap_penalty = if memory.swap_usage_percent > 50.0 { 0.3 } else { 0.0 };
        (base_score - swap_penalty).max(0.0)
    }

    fn score_network_utilization(&self, network: &NetworkUtilization) -> f32 {
        let bandwidth_score = match network.bandwidth_utilization {
            usage if usage > 90.0 => 0.3,
            usage if usage > 75.0 => 0.6,
            usage if usage > 50.0 => 0.8,
            _ => 1.0,
        };

        let latency_score = match network.latency_ms {
            latency if latency > 500.0 => 0.3,
            latency if latency > 100.0 => 0.7,
            latency if latency > 50.0 => 0.9,
            _ => 1.0,
        };

        let packet_loss_score = match network.packet_loss_rate {
            loss if loss > 0.05 => 0.2,
            loss if loss > 0.01 => 0.6,
            loss if loss > 0.001 => 0.8,
            _ => 1.0,
        };

        (bandwidth_score + latency_score + packet_loss_score) / 3.0
    }

    fn score_disk_utilization(&self, disk: &DiskUtilization) -> f32 {
        let space_score = match disk.usage_percent {
            usage if usage > 95.0 => 0.1,
            usage if usage > 85.0 => 0.4,
            usage if usage > 75.0 => 0.7,
            usage if usage > 60.0 => 0.9,
            _ => 1.0,
        };

        let io_score = match disk.io_utilization {
            io if io > 95.0 => 0.2,
            io if io > 80.0 => 0.5,
            io if io > 60.0 => 0.8,
            _ => 1.0,
        };

        (space_score + io_score) / 2.0
    }

    fn calculate_performance_score(&self, metrics: &PerformanceMetrics) -> f32 {
        let throughput_score = self.score_throughput(&metrics.throughput);
        let latency_score = self.score_latency(&metrics.latency);
        let error_score = self.score_error_rates(&metrics.error_rates);
        let efficiency_score = self.score_efficiency(&metrics.efficiency_scores);

        (throughput_score * 0.3 + latency_score * 0.3 + error_score * 0.3 + efficiency_score * 0.1) * 100.0
    }

    fn score_throughput(&self, throughput: &ThroughputMetrics) -> f32 {
        // Higher throughput is generally better, but this depends on expected load
        let rps_score = if throughput.requests_per_second > self.thresholds.healthy_throughput_threshold {
            1.0
        } else {
            throughput.requests_per_second / self.thresholds.healthy_throughput_threshold
        };

        rps_score.min(1.0)
    }

    fn score_latency(&self, latency: &LatencyMetrics) -> f32 {
        // Lower latency is better
        let p95_score = match latency.p95_latency_ms {
            lat if lat > self.thresholds.warning_latency_threshold => 0.2,
            lat if lat > 500.0 => 0.5,
            lat if lat > 200.0 => 0.7,
            lat if lat > 100.0 => 0.9,
            _ => 1.0,
        };

        let mean_score = match latency.mean_latency_ms {
            lat if lat > 500.0 => 0.3,
            lat if lat > 200.0 => 0.6,
            lat if lat > 100.0 => 0.8,
            lat if lat > 50.0 => 0.9,
            _ => 1.0,
        };

        (p95_score + mean_score) / 2.0
    }

    fn score_error_rates(&self, error_rates: &ErrorRateMetrics) -> f32 {
        // Lower error rates are better
        match error_rates.overall_error_rate {
            rate if rate > self.thresholds.critical_error_rate_threshold => 0.1,
            rate if rate > 0.02 => 0.4,
            rate if rate > 0.01 => 0.7,
            rate if rate > 0.005 => 0.9,
            _ => 1.0,
        }
    }

    fn score_efficiency(&self, efficiency: &EfficiencyMetrics) -> f32 {
        (efficiency.resource_efficiency + efficiency.cost_efficiency + efficiency.energy_efficiency) / 3.0
    }

    fn calculate_anomaly_impact_score(&self, anomalies: &[Anomaly]) -> f32 {
        if anomalies.is_empty() {
            return 100.0;
        }

        let total_impact: f32 = anomalies.iter()
            .map(|anomaly| self.calculate_anomaly_impact(anomaly))
            .sum();

        // Convert impact to score (higher impact = lower score)
        let max_possible_impact = anomalies.len() as f32 * 10.0; // Assuming max impact of 10 per anomaly
        let impact_ratio = total_impact / max_possible_impact;
        
        ((1.0 - impact_ratio) * 100.0).max(0.0)
    }

    fn calculate_anomaly_impact(&self, anomaly: &Anomaly) -> f32 {
        let severity_weight = match anomaly.severity {
            AnomalySeverity::Critical => 10.0,
            AnomalySeverity::High => 7.0,
            AnomalySeverity::Medium => 4.0,
            AnomalySeverity::Low => 2.0,
            AnomalySeverity::Info => 1.0,
        };

        severity_weight * anomaly.confidence
    }

    fn calculate_availability_score(&self, utilization: &ResourceUtilization, performance: &PerformanceMetrics) -> f32 {
        // Simplified availability calculation based on resource health and error rates
        let resource_availability = if utilization.cpu.overall_usage < 95.0 && 
                                    utilization.memory.usage_percent < 95.0 {
            0.95
        } else {
            0.7
        };

        let error_availability = 1.0 - performance.error_rates.overall_error_rate;
        
        ((resource_availability + error_availability) / 2.0) * 100.0
    }

    fn calculate_trend_stability_score(&self, _utilization: &ResourceUtilization, _performance: &PerformanceMetrics) -> f32 {
        // Simplified trend stability - in a real implementation, this would analyze historical trends
        85.0 // Placeholder score
    }

    fn determine_health_status(&self, score: f32) -> HealthStatus {
        match score {
            s if s >= 90.0 => HealthStatus::Excellent,
            s if s >= 75.0 => HealthStatus::Good,
            s if s >= 60.0 => HealthStatus::Fair,
            s if s >= 40.0 => HealthStatus::Poor,
            _ => HealthStatus::Critical,
        }
    }

    fn calculate_component_scores(&self, utilization: &ResourceUtilization, performance: &PerformanceMetrics) -> ComponentHealthScores {
        ComponentHealthScores {
            system_resources: self.calculate_resource_utilization_score(utilization),
            dataflow_performance: self.calculate_performance_score(performance),
            node_health: 85.0, // Placeholder
            network_connectivity: self.score_network_utilization(&utilization.network) * 100.0,
            storage_health: self.score_disk_utilization(&utilization.disk) * 100.0,
        }
    }

    fn create_detailed_metrics(&self, utilization: &ResourceUtilization, performance: &PerformanceMetrics) -> HashMap<String, MetricHealthInfo> {
        let mut metrics = HashMap::new();

        metrics.insert("cpu_usage".to_string(), MetricHealthInfo {
            current_value: utilization.cpu.overall_usage,
            health_score: self.score_cpu_utilization(&utilization.cpu) * 100.0,
            status: self.determine_metric_status(utilization.cpu.overall_usage, 60.0, 80.0),
            trend: TrendHealth {
                direction: TrendDirection::Stable,
                stability: 0.8,
                predictability: 0.7,
            },
            impact_on_overall: 0.3,
        });

        metrics.insert("memory_usage".to_string(), MetricHealthInfo {
            current_value: utilization.memory.usage_percent,
            health_score: self.score_memory_utilization(&utilization.memory) * 100.0,
            status: self.determine_metric_status(utilization.memory.usage_percent, 70.0, 85.0),
            trend: TrendHealth {
                direction: TrendDirection::Stable,
                stability: 0.7,
                predictability: 0.8,
            },
            impact_on_overall: 0.3,
        });

        metrics.insert("error_rate".to_string(), MetricHealthInfo {
            current_value: performance.error_rates.overall_error_rate,
            health_score: self.score_error_rates(&performance.error_rates) * 100.0,
            status: self.determine_metric_status(performance.error_rates.overall_error_rate, 0.01, 0.02),
            trend: TrendHealth {
                direction: TrendDirection::Stable,
                stability: 0.6,
                predictability: 0.5,
            },
            impact_on_overall: 0.4,
        });

        metrics
    }

    fn determine_metric_status(&self, value: f32, warning_threshold: f32, critical_threshold: f32) -> MetricStatus {
        if value > critical_threshold {
            MetricStatus::Critical
        } else if value > warning_threshold {
            MetricStatus::Warning
        } else {
            MetricStatus::Healthy
        }
    }

    fn generate_health_insights(
        &self,
        utilization: &ResourceUtilization,
        performance: &PerformanceMetrics,
        anomalies: &[Anomaly],
        overall_score: f32,
    ) -> Vec<HealthInsight> {
        let mut insights = Vec::new();

        // CPU insights
        if utilization.cpu.overall_usage > 80.0 {
            insights.push(HealthInsight {
                insight_type: InsightType::ResourceOptimization,
                priority: if utilization.cpu.overall_usage > 90.0 { InsightPriority::Critical } else { InsightPriority::High },
                title: "High CPU Utilization Detected".to_string(),
                description: format!("CPU usage is at {:.1}%, which may impact system performance", utilization.cpu.overall_usage),
                recommended_actions: vec![
                    "Review CPU-intensive processes".to_string(),
                    "Consider horizontal scaling".to_string(),
                    "Optimize critical path operations".to_string(),
                ],
                potential_impact: 15.0,
                effort_level: EffortLevel::Medium,
                affected_components: vec!["system_resources".to_string()],
            });
        }

        // Memory insights
        if utilization.memory.usage_percent > 85.0 {
            insights.push(HealthInsight {
                insight_type: InsightType::ResourceOptimization,
                priority: InsightPriority::High,
                title: "High Memory Utilization".to_string(),
                description: format!("Memory usage is at {:.1}%, approaching critical levels", utilization.memory.usage_percent),
                recommended_actions: vec![
                    "Investigate memory leaks".to_string(),
                    "Optimize memory-intensive operations".to_string(),
                    "Consider increasing memory capacity".to_string(),
                ],
                potential_impact: 12.0,
                effort_level: EffortLevel::Medium,
                affected_components: vec!["system_resources".to_string()],
            });
        }

        // Performance insights
        if performance.latency.p95_latency_ms > 500.0 {
            insights.push(HealthInsight {
                insight_type: InsightType::PerformanceImprovement,
                priority: InsightPriority::High,
                title: "High Latency Detected".to_string(),
                description: format!("95th percentile latency is {:.1}ms, exceeding acceptable thresholds", performance.latency.p95_latency_ms),
                recommended_actions: vec![
                    "Identify bottlenecks in request processing".to_string(),
                    "Optimize database queries".to_string(),
                    "Implement caching strategies".to_string(),
                ],
                potential_impact: 20.0,
                effort_level: EffortLevel::High,
                affected_components: vec!["dataflow_performance".to_string()],
            });
        }

        // Error rate insights
        if performance.error_rates.overall_error_rate > 0.01 {
            insights.push(HealthInsight {
                insight_type: InsightType::AnomalyResolution,
                priority: InsightPriority::Critical,
                title: "Elevated Error Rate".to_string(),
                description: format!("Error rate is {:.2}%, indicating system instability", performance.error_rates.overall_error_rate * 100.0),
                recommended_actions: vec![
                    "Investigate error logs for patterns".to_string(),
                    "Review recent deployments".to_string(),
                    "Implement circuit breakers".to_string(),
                ],
                potential_impact: 25.0,
                effort_level: EffortLevel::High,
                affected_components: vec!["dataflow_performance".to_string()],
            });
        }

        // Anomaly-based insights
        let critical_anomalies: Vec<_> = anomalies.iter()
            .filter(|a| matches!(a.severity, AnomalySeverity::Critical | AnomalySeverity::High))
            .collect();

        if !critical_anomalies.is_empty() {
            insights.push(HealthInsight {
                insight_type: InsightType::AnomalyResolution,
                priority: InsightPriority::Critical,
                title: format!("{} Critical Anomalies Detected", critical_anomalies.len()),
                description: "Multiple high-severity anomalies are affecting system health".to_string(),
                recommended_actions: vec![
                    "Address anomalies by severity".to_string(),
                    "Implement monitoring alerts".to_string(),
                    "Review system configuration".to_string(),
                ],
                potential_impact: 30.0,
                effort_level: EffortLevel::High,
                affected_components: vec!["system_resources".to_string(), "dataflow_performance".to_string()],
            });
        }

        // Overall health insights
        if overall_score < 60.0 {
            insights.push(HealthInsight {
                insight_type: InsightType::MaintenanceRequired,
                priority: InsightPriority::Critical,
                title: "System Health Below Acceptable Levels".to_string(),
                description: format!("Overall health score is {:.1}/100, requiring immediate attention", overall_score),
                recommended_actions: vec![
                    "Perform comprehensive system review".to_string(),
                    "Address top health issues first".to_string(),
                    "Consider emergency maintenance window".to_string(),
                ],
                potential_impact: 40.0,
                effort_level: EffortLevel::Extensive,
                affected_components: vec!["system_resources".to_string(), "dataflow_performance".to_string()],
            });
        }

        // Sort insights by priority
        insights.sort_by(|a, b| b.priority.cmp(&a.priority));
        insights
    }
}

impl Default for SystemHealthCalculator {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn create_test_utilization() -> ResourceUtilization {
        ResourceUtilization {
            cpu: CpuUtilization {
                overall_usage: 45.0,
                per_core_usage: vec![40.0, 50.0, 45.0, 40.0],
                load_average: (0.8, 0.7, 0.6),
            },
            memory: MemoryUtilization {
                usage_percent: 60.0,
                used_bytes: 6_000_000_000,
                available_bytes: 4_000_000_000,
                swap_usage_percent: 10.0,
            },
            network: NetworkUtilization {
                bandwidth_utilization: 30.0,
                packet_loss_rate: 0.001,
                latency_ms: 25.0,
                connection_count: 150,
            },
            disk: DiskUtilization {
                usage_percent: 70.0,
                io_utilization: 40.0,
                read_latency_ms: 5.0,
                write_latency_ms: 8.0,
            },
            custom_metrics: HashMap::new(),
        }
    }

    fn create_test_performance() -> PerformanceMetrics {
        PerformanceMetrics {
            throughput: ThroughputMetrics {
                requests_per_second: 500.0,
                data_processed_per_second: 1000.0,
                messages_per_second: 200.0,
            },
            latency: LatencyMetrics {
                mean_latency_ms: 50.0,
                p50_latency_ms: 45.0,
                p95_latency_ms: 120.0,
                p99_latency_ms: 200.0,
            },
            error_rates: ErrorRateMetrics {
                overall_error_rate: 0.005,
                error_rate_by_type: HashMap::new(),
                error_trend: TrendDirection::Stable,
            },
            efficiency_scores: EfficiencyMetrics {
                resource_efficiency: 0.8,
                cost_efficiency: 0.75,
                energy_efficiency: 0.7,
            },
        }
    }

    #[test]
    fn test_health_calculation() {
        let calculator = SystemHealthCalculator::new();
        let utilization = create_test_utilization();
        let performance = create_test_performance();
        let anomalies = vec![];

        let health = calculator.calculate_health(&utilization, &performance, &anomalies);

        assert!(health.overall_score >= 0.0 && health.overall_score <= 100.0);
        assert!(matches!(health.health_status, HealthStatus::Good | HealthStatus::Excellent));
        assert!(!health.health_factors.detailed_metrics.is_empty());
    }

    #[test]
    fn test_health_status_determination() {
        let calculator = SystemHealthCalculator::new();

        assert_eq!(calculator.determine_health_status(95.0), HealthStatus::Excellent);
        assert_eq!(calculator.determine_health_status(80.0), HealthStatus::Good);
        assert_eq!(calculator.determine_health_status(65.0), HealthStatus::Fair);
        assert_eq!(calculator.determine_health_status(45.0), HealthStatus::Poor);
        assert_eq!(calculator.determine_health_status(30.0), HealthStatus::Critical);
    }

    #[test]
    fn test_cpu_scoring() {
        let calculator = SystemHealthCalculator::new();
        
        let high_cpu = CpuUtilization {
            overall_usage: 95.0,
            per_core_usage: vec![95.0, 96.0, 94.0, 97.0],
            load_average: (3.0, 2.8, 2.5),
        };

        let score = calculator.score_cpu_utilization(&high_cpu);
        assert!(score < 0.5); // Should be low score for high CPU usage
    }

    #[test]
    fn test_insight_generation() {
        let calculator = SystemHealthCalculator::new();
        
        let mut utilization = create_test_utilization();
        utilization.cpu.overall_usage = 95.0; // High CPU to trigger insight
        
        let performance = create_test_performance();
        let anomalies = vec![];

        let health = calculator.calculate_health(&utilization, &performance, &anomalies);

        assert!(!health.actionable_insights.is_empty());
        assert!(health.actionable_insights.iter().any(|insight| 
            insight.title.contains("CPU") && matches!(insight.priority, InsightPriority::Critical)
        ));
    }
}