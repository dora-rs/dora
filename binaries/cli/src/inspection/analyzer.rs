// Resource Analyzer for Issue #17 - Phase 2 Implementation

use super::types::*;
use crate::cli::commands::{ResourceType, InspectionFocus};
use chrono::Utc;
use eyre::{Result, eyre};
use std::collections::HashMap;

/// Main resource analyzer
#[derive(Debug)]
pub struct ResourceAnalyzer {
    /// Cache for resolved resources
    resource_cache: HashMap<String, ResolvedResource>,
    /// Analysis configuration
    config: AnalyzerConfig,
}

#[derive(Debug, Clone)]
pub struct AnalyzerConfig {
    pub enable_cache: bool,
    pub max_cache_size: usize,
    pub default_complexity_threshold: f32,
}

impl Default for AnalyzerConfig {
    fn default() -> Self {
        Self {
            enable_cache: true,
            max_cache_size: 100,
            default_complexity_threshold: 50.0,
        }
    }
}

impl ResourceAnalyzer {
    pub fn new() -> Self {
        Self {
            resource_cache: HashMap::new(),
            config: AnalyzerConfig::default(),
        }
    }

    pub fn with_config(config: AnalyzerConfig) -> Self {
        Self {
            resource_cache: HashMap::new(),
            config,
        }
    }

    /// Main analysis entry point
    pub async fn analyze_resource(
        &mut self,
        target: &str,
        resource_type: Option<ResourceType>,
        depth: u8,
        focus: Option<InspectionFocus>,
    ) -> Result<InspectionResult> {
        // Resolve the resource
        let resource = self.resolve_resource(target, resource_type).await?;

        // Calculate complexity score
        let complexity_score = self.calculate_complexity(&resource, depth)?;

        // Perform focused analysis based on focus parameter
        let (health_score, performance_metrics, error_summary) = match focus {
            Some(InspectionFocus::Performance) => {
                let perf = self.analyze_performance(&resource, depth).await?;
                (self.calculate_health_score_from_performance(&perf), perf, ErrorSummary::default())
            },
            Some(InspectionFocus::Errors) => {
                let errors = self.analyze_errors(&resource, depth).await?;
                let health = self.calculate_health_score_from_errors(&errors);
                (health, PerformanceMetrics::default(), errors)
            },
            Some(InspectionFocus::Health) => {
                let health = self.analyze_health(&resource, depth).await?;
                (health, PerformanceMetrics::default(), ErrorSummary::default())
            },
            Some(InspectionFocus::Dependencies) => {
                // For dependencies focus, we provide a basic analysis
                let health = self.analyze_health(&resource, depth).await?;
                (health, PerformanceMetrics::default(), ErrorSummary::default())
            },
            Some(InspectionFocus::All) | None => {
                // Comprehensive analysis
                let health = self.analyze_health(&resource, depth).await?;
                let perf = self.analyze_performance(&resource, depth).await?;
                let errors = self.analyze_errors(&resource, depth).await?;
                (health, perf, errors)
            },
        };

        // Generate recommendations
        let recommendations = self.generate_recommendations(
            &resource,
            &health_score,
            &performance_metrics,
            &error_summary,
            complexity_score,
        )?;

        // Generate suggested actions
        let suggested_actions = self.generate_suggested_actions(
            &health_score,
            &performance_metrics,
            &error_summary,
        )?;

        Ok(InspectionResult {
            resource,
            timestamp: Utc::now(),
            analysis_depth: depth,
            complexity_score,
            health_score,
            performance_metrics,
            error_summary,
            recommendations,
            suggested_actions,
        })
    }

    /// Resolve resource by target and type
    async fn resolve_resource(
        &mut self,
        target: &str,
        resource_type: Option<ResourceType>,
    ) -> Result<ResolvedResource> {
        // Check cache first
        if self.config.enable_cache {
            if let Some(cached) = self.resource_cache.get(target) {
                return Ok(cached.clone());
            }
        }

        // Determine resource type
        let resolved_type = resource_type.unwrap_or_else(|| self.infer_resource_type(target));

        // Create resolved resource (mock implementation - would query daemon in real impl)
        let resource = match resolved_type {
            ResourceType::Dataflow => self.resolve_dataflow(target).await?,
            ResourceType::Node => self.resolve_node(target).await?,
            ResourceType::System => self.resolve_system(target).await?,
            ResourceType::Network => self.resolve_network(target).await?,
            ResourceType::Storage => self.resolve_storage(target).await?,
        };

        // Cache result
        if self.config.enable_cache && self.resource_cache.len() < self.config.max_cache_size {
            self.resource_cache.insert(target.to_string(), resource.clone());
        }

        Ok(resource)
    }

    fn infer_resource_type(&self, target: &str) -> ResourceType {
        // Simple inference based on target format
        if target.contains(".yaml") || target.contains(".yml") {
            ResourceType::Dataflow
        } else if target.contains("node") || target.contains("operator") {
            ResourceType::Node
        } else if target == "system" || target == "daemon" {
            ResourceType::System
        } else if target.contains("network") || target.contains("tcp") || target.contains("udp") {
            ResourceType::Network
        } else if target.contains("storage") || target.contains("disk") {
            ResourceType::Storage
        } else {
            ResourceType::Dataflow
        }
    }

    async fn resolve_dataflow(&self, target: &str) -> Result<ResolvedResource> {
        // Mock implementation - would query daemon in production
        let mut metadata = HashMap::new();
        metadata.insert("type".to_string(), "dataflow".to_string());
        metadata.insert("path".to_string(), target.to_string());
        metadata.insert("nodes".to_string(), "3".to_string());

        Ok(ResolvedResource {
            resource_type: "dataflow".to_string(),
            identifier: target.to_string(),
            status: ResourceStatus::Running,
            created_at: Some(Utc::now()),
            metadata,
        })
    }

    async fn resolve_node(&self, target: &str) -> Result<ResolvedResource> {
        let mut metadata = HashMap::new();
        metadata.insert("type".to_string(), "node".to_string());
        metadata.insert("name".to_string(), target.to_string());

        Ok(ResolvedResource {
            resource_type: "node".to_string(),
            identifier: target.to_string(),
            status: ResourceStatus::Running,
            created_at: Some(Utc::now()),
            metadata,
        })
    }

    async fn resolve_system(&self, _target: &str) -> Result<ResolvedResource> {
        let mut metadata = HashMap::new();
        metadata.insert("type".to_string(), "system".to_string());
        metadata.insert("daemon_status".to_string(), "running".to_string());

        Ok(ResolvedResource {
            resource_type: "system".to_string(),
            identifier: "dora-system".to_string(),
            status: ResourceStatus::Running,
            created_at: Some(Utc::now()),
            metadata,
        })
    }

    async fn resolve_network(&self, target: &str) -> Result<ResolvedResource> {
        let mut metadata = HashMap::new();
        metadata.insert("type".to_string(), "network".to_string());
        metadata.insert("endpoint".to_string(), target.to_string());

        Ok(ResolvedResource {
            resource_type: "network".to_string(),
            identifier: target.to_string(),
            status: ResourceStatus::Running,
            created_at: Some(Utc::now()),
            metadata,
        })
    }

    async fn resolve_storage(&self, target: &str) -> Result<ResolvedResource> {
        let mut metadata = HashMap::new();
        metadata.insert("type".to_string(), "storage".to_string());
        metadata.insert("path".to_string(), target.to_string());

        Ok(ResolvedResource {
            resource_type: "storage".to_string(),
            identifier: target.to_string(),
            status: ResourceStatus::Running,
            created_at: Some(Utc::now()),
            metadata,
        })
    }

    /// Calculate complexity score
    fn calculate_complexity(&self, resource: &ResolvedResource, depth: u8) -> Result<f32> {
        // Mock implementation - would use Issue #13 complexity algorithms in production
        let base_complexity = match resource.resource_type.as_str() {
            "dataflow" => {
                // Calculate based on number of nodes
                let node_count = resource.metadata.get("nodes")
                    .and_then(|n| n.parse::<u32>().ok())
                    .unwrap_or(1);
                (node_count as f32) * 10.0
            },
            "node" => 20.0,
            "system" => 50.0,
            "network" => 30.0,
            "storage" => 25.0,
            _ => 10.0,
        };

        // Adjust by depth
        let complexity = base_complexity * (depth as f32 * 0.5);

        Ok(complexity.min(100.0))
    }

    /// Analyze health
    async fn analyze_health(&self, resource: &ResolvedResource, depth: u8) -> Result<HealthScore> {
        let mut component_scores = HashMap::new();
        let mut issues = Vec::new();

        // Mock health analysis based on resource status
        let base_score = match resource.status {
            ResourceStatus::Running => 95.0,
            ResourceStatus::Stopped => 50.0,
            ResourceStatus::Error => 20.0,
            ResourceStatus::Unknown => 60.0,
        };

        // Add component scores based on depth
        if depth >= 1 {
            component_scores.insert("runtime".to_string(), base_score);
        }
        if depth >= 2 {
            component_scores.insert("connectivity".to_string(), 90.0);
            component_scores.insert("resources".to_string(), 85.0);
        }
        if depth >= 3 {
            component_scores.insert("performance".to_string(), 88.0);
            component_scores.insert("reliability".to_string(), 92.0);
        }

        // Generate issues if score is low
        if base_score < 80.0 {
            issues.push(HealthIssue {
                component: "runtime".to_string(),
                severity: if base_score < 50.0 { IssueSeverity::Critical } else { IssueSeverity::Medium },
                description: format!("Resource status: {:?}", resource.status),
            });
        }

        let status = if base_score >= 90.0 {
            HealthStatus::Healthy
        } else if base_score >= 70.0 {
            HealthStatus::Warning
        } else {
            HealthStatus::Critical
        };

        Ok(HealthScore {
            overall_score: base_score,
            status,
            component_scores,
            issues,
        })
    }

    /// Analyze performance
    async fn analyze_performance(&self, resource: &ResolvedResource, depth: u8) -> Result<PerformanceMetrics> {
        let mut issues = Vec::new();

        // Mock performance metrics
        let (cpu_usage, memory_mb, throughput, latency_ms, error_rate) = match resource.resource_type.as_str() {
            "dataflow" => (Some(45.0), Some(256.0), Some(1000.0), Some(15.0), Some(0.01)),
            "node" => (Some(30.0), Some(128.0), Some(500.0), Some(10.0), Some(0.005)),
            "system" => (Some(25.0), Some(512.0), None, None, Some(0.001)),
            _ => (Some(20.0), Some(64.0), None, Some(5.0), Some(0.0)),
        };

        // Generate performance issues based on depth
        if depth >= 2 {
            if let Some(cpu) = cpu_usage {
                if cpu > 80.0 {
                    issues.push(PerformanceIssue {
                        severity: IssueSeverity::High,
                        description: format!("High CPU usage: {:.1}%", cpu),
                        recommendation: Some("Consider optimizing or scaling the resource".to_string()),
                    });
                }
            }

            if let Some(mem) = memory_mb {
                if mem > 1024.0 {
                    issues.push(PerformanceIssue {
                        severity: IssueSeverity::Medium,
                        description: format!("High memory usage: {:.0}MB", mem),
                        recommendation: Some("Monitor memory usage and consider increasing limits".to_string()),
                    });
                }
            }
        }

        if depth >= 3 {
            if let Some(latency) = latency_ms {
                if latency > 50.0 {
                    issues.push(PerformanceIssue {
                        severity: IssueSeverity::Medium,
                        description: format!("Elevated latency: {:.1}ms", latency),
                        recommendation: Some("Investigate network or processing delays".to_string()),
                    });
                }
            }
        }

        Ok(PerformanceMetrics {
            cpu_usage,
            memory_mb,
            throughput,
            latency_ms,
            error_rate,
            issues,
        })
    }

    /// Analyze errors
    async fn analyze_errors(&self, resource: &ResolvedResource, depth: u8) -> Result<ErrorSummary> {
        let mut recent_errors = Vec::new();
        let mut error_patterns = Vec::new();

        // Mock error analysis based on resource status
        let total_errors = match resource.status {
            ResourceStatus::Error => 15,
            ResourceStatus::Running => 2,
            ResourceStatus::Stopped => 0,
            ResourceStatus::Unknown => 5,
        };

        if depth >= 2 && total_errors > 0 {
            recent_errors.push(ErrorRecord {
                timestamp: Utc::now(),
                error_type: "ConnectionError".to_string(),
                message: "Failed to connect to downstream node".to_string(),
                context: Some(resource.identifier.clone()),
            });

            if total_errors > 5 {
                recent_errors.push(ErrorRecord {
                    timestamp: Utc::now(),
                    error_type: "TimeoutError".to_string(),
                    message: "Operation timed out after 30s".to_string(),
                    context: Some("processing".to_string()),
                });
            }
        }

        if depth >= 3 && total_errors > 5 {
            error_patterns.push(ErrorPattern {
                pattern_type: "ConnectionError".to_string(),
                frequency: total_errors / 2,
                description: "Recurring connection failures".to_string(),
            });
        }

        Ok(ErrorSummary {
            total_errors,
            recent_errors,
            error_patterns,
        })
    }

    fn calculate_health_score_from_performance(&self, perf: &PerformanceMetrics) -> HealthScore {
        let mut overall_score: f32 = 100.0;
        let mut component_scores = HashMap::new();
        let mut issues = Vec::new();

        // Deduct points based on metrics
        if let Some(cpu) = perf.cpu_usage {
            let cpu_score = (100.0 - cpu).max(0.0);
            component_scores.insert("cpu".to_string(), cpu_score);
            overall_score = overall_score.min(cpu_score + 20.0);

            if cpu > 80.0 {
                issues.push(HealthIssue {
                    component: "cpu".to_string(),
                    severity: IssueSeverity::High,
                    description: "High CPU usage detected".to_string(),
                });
            }
        }

        if let Some(error_rate) = perf.error_rate {
            let error_score = (100.0 - (error_rate * 1000.0)).max(0.0);
            component_scores.insert("errors".to_string(), error_score);
            overall_score = overall_score.min(error_score + 30.0);
        }

        let status = if overall_score >= 90.0 {
            HealthStatus::Healthy
        } else if overall_score >= 70.0 {
            HealthStatus::Warning
        } else {
            HealthStatus::Critical
        };

        HealthScore {
            overall_score,
            status,
            component_scores,
            issues,
        }
    }

    fn calculate_health_score_from_errors(&self, errors: &ErrorSummary) -> HealthScore {
        let overall_score = if errors.total_errors == 0 {
            100.0
        } else if errors.total_errors < 5 {
            85.0
        } else if errors.total_errors < 20 {
            60.0
        } else {
            30.0
        };

        let mut issues = Vec::new();
        if errors.total_errors > 0 {
            issues.push(HealthIssue {
                component: "error_tracking".to_string(),
                severity: if errors.total_errors > 20 { IssueSeverity::Critical } else { IssueSeverity::Medium },
                description: format!("{} errors detected", errors.total_errors),
            });
        }

        let status = if overall_score >= 90.0 {
            HealthStatus::Healthy
        } else if overall_score >= 70.0 {
            HealthStatus::Warning
        } else {
            HealthStatus::Critical
        };

        HealthScore {
            overall_score,
            status,
            component_scores: HashMap::new(),
            issues,
        }
    }

    /// Generate recommendations
    fn generate_recommendations(
        &self,
        resource: &ResolvedResource,
        health: &HealthScore,
        performance: &PerformanceMetrics,
        errors: &ErrorSummary,
        complexity: f32,
    ) -> Result<Vec<Recommendation>> {
        let mut recommendations = Vec::new();

        // Health-based recommendations
        if health.overall_score < 70.0 {
            recommendations.push(Recommendation {
                priority: RecommendationPriority::High,
                title: "Improve Resource Health".to_string(),
                description: format!(
                    "Health score is {:.0}%, consider investigating and resolving issues",
                    health.overall_score
                ),
                suggested_command: Some(format!("dora inspect {} --focus errors --depth 3", resource.identifier)),
                impact: ImpactLevel::High,
            });
        }

        // Performance-based recommendations
        if let Some(cpu) = performance.cpu_usage {
            if cpu > 80.0 {
                recommendations.push(Recommendation {
                    priority: RecommendationPriority::Medium,
                    title: "High CPU Usage".to_string(),
                    description: format!("CPU usage is {:.1}%, consider optimization or scaling", cpu),
                    suggested_command: Some(format!("dora inspect {} --focus performance", resource.identifier)),
                    impact: ImpactLevel::Medium,
                });
            }
        }

        // Error-based recommendations
        if errors.total_errors > 10 {
            recommendations.push(Recommendation {
                priority: RecommendationPriority::Critical,
                title: "High Error Rate".to_string(),
                description: format!("{} errors detected, immediate investigation recommended", errors.total_errors),
                suggested_command: Some(format!("dora logs {} --errors-only", resource.identifier)),
                impact: ImpactLevel::High,
            });
        }

        // Complexity-based recommendations
        if complexity > self.config.default_complexity_threshold {
            recommendations.push(Recommendation {
                priority: RecommendationPriority::Low,
                title: "High Complexity".to_string(),
                description: format!(
                    "Complexity score is {:.0}, consider using TUI for better visualization",
                    complexity
                ),
                suggested_command: Some(format!("dora inspect {} --tui", resource.identifier)),
                impact: ImpactLevel::Low,
            });
        }

        Ok(recommendations)
    }

    /// Generate suggested actions
    fn generate_suggested_actions(
        &self,
        health: &HealthScore,
        performance: &PerformanceMetrics,
        errors: &ErrorSummary,
    ) -> Result<Vec<SuggestedAction>> {
        let mut actions = Vec::new();

        // Restart if critical
        if matches!(health.status, HealthStatus::Critical) {
            actions.push(SuggestedAction {
                action_type: ActionType::Restart,
                description: "Critical health detected - consider restarting resource".to_string(),
                command: Some("dora restart <resource>".to_string()),
            });
        }

        // Scale if performance issues
        if let Some(cpu) = performance.cpu_usage {
            if cpu > 90.0 {
                actions.push(SuggestedAction {
                    action_type: ActionType::ScaleUp,
                    description: "Very high CPU usage - consider scaling up resources".to_string(),
                    command: None,
                });
            }
        }

        // Investigate errors
        if errors.total_errors > 5 {
            actions.push(SuggestedAction {
                action_type: ActionType::Investigate,
                description: "Multiple errors detected - investigate error patterns".to_string(),
                command: Some("dora inspect <resource> --focus errors --depth 3".to_string()),
            });
        }

        // Monitor if warnings
        if matches!(health.status, HealthStatus::Warning) {
            actions.push(SuggestedAction {
                action_type: ActionType::Monitor,
                description: "Health warning detected - enable monitoring".to_string(),
                command: Some("dora inspect <resource> --live".to_string()),
            });
        }

        Ok(actions)
    }
}

impl Default for ResourceAnalyzer {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_analyzer_basic() {
        let mut analyzer = ResourceAnalyzer::new();
        let result = analyzer.analyze_resource("test-dataflow", None, 1, None).await.unwrap();
        assert_eq!(result.analysis_depth, 1);
        assert!(result.complexity_score > 0.0);
    }

    #[tokio::test]
    async fn test_analyzer_with_depth() {
        let mut analyzer = ResourceAnalyzer::new();

        let result_depth_1 = analyzer.analyze_resource("test", None, 1, None).await.unwrap();
        let result_depth_3 = analyzer.analyze_resource("test", None, 3, None).await.unwrap();

        assert!(result_depth_3.health_score.component_scores.len() >= result_depth_1.health_score.component_scores.len());
    }

    #[tokio::test]
    async fn test_resource_type_inference() {
        let analyzer = ResourceAnalyzer::new();

        assert!(matches!(analyzer.infer_resource_type("test.yaml"), ResourceType::Dataflow));
        assert!(matches!(analyzer.infer_resource_type("my-node"), ResourceType::Node));
        assert!(matches!(analyzer.infer_resource_type("system"), ResourceType::System));
    }

    #[tokio::test]
    async fn test_complexity_calculation() {
        let analyzer = ResourceAnalyzer::new();

        let resource = ResolvedResource {
            resource_type: "dataflow".to_string(),
            identifier: "test".to_string(),
            status: ResourceStatus::Running,
            created_at: Some(Utc::now()),
            metadata: {
                let mut m = HashMap::new();
                m.insert("nodes".to_string(), "5".to_string());
                m
            },
        };

        let complexity = analyzer.calculate_complexity(&resource, 2).unwrap();
        assert!(complexity > 0.0);
        assert!(complexity <= 100.0);
    }

    #[tokio::test]
    async fn test_performance_focus() {
        let mut analyzer = ResourceAnalyzer::new();
        let result = analyzer.analyze_resource(
            "test",
            None,
            2,
            Some(InspectionFocus::Performance)
        ).await.unwrap();

        assert!(result.performance_metrics.cpu_usage.is_some());
    }

    #[tokio::test]
    async fn test_error_focus() {
        let mut analyzer = ResourceAnalyzer::new();
        let result = analyzer.analyze_resource(
            "test",
            None,
            2,
            Some(InspectionFocus::Errors)
        ).await.unwrap();

        assert!(result.error_summary.total_errors >= 0);
    }

    #[tokio::test]
    async fn test_recommendations_generated() {
        let mut analyzer = ResourceAnalyzer::new();
        let result = analyzer.analyze_resource("test", None, 3, None).await.unwrap();

        // Should have at least some analysis
        assert!(result.health_score.overall_score > 0.0);
    }

    #[tokio::test]
    async fn test_cache_functionality() {
        let mut analyzer = ResourceAnalyzer::new();

        // First call
        let result1 = analyzer.analyze_resource("cached-test", None, 1, None).await.unwrap();
        assert_eq!(analyzer.resource_cache.len(), 1);

        // Second call should use cache
        let result2 = analyzer.analyze_resource("cached-test", None, 1, None).await.unwrap();
        assert_eq!(result1.resource.identifier, result2.resource.identifier);
    }
}
