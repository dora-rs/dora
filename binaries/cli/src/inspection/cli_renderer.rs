// CLI Renderer for Issue #17 - Phase 2 Implementation

use super::types::*;
use crate::cli::commands::InspectOutputFormat;
use colored::Colorize;
use eyre::Result;
use serde_json;

/// CLI Renderer for inspection results
pub struct CliRenderer {
    config: RendererConfig,
}

#[derive(Debug, Clone)]
pub struct RendererConfig {
    pub use_colors: bool,
    pub max_recommendations: usize,
    pub show_metadata: bool,
}

impl Default for RendererConfig {
    fn default() -> Self {
        Self {
            use_colors: true,
            max_recommendations: 5,
            show_metadata: true,
        }
    }
}

impl CliRenderer {
    pub fn new() -> Self {
        Self {
            config: RendererConfig::default(),
        }
    }

    pub fn with_config(config: RendererConfig) -> Self {
        Self { config }
    }

    /// Main rendering entry point
    pub fn render(
        &self,
        result: &InspectionResult,
        format: Option<InspectOutputFormat>,
    ) -> Result<String> {
        match format {
            Some(InspectOutputFormat::Json) => self.render_json(result),
            Some(InspectOutputFormat::Yaml) => self.render_yaml(result),
            Some(InspectOutputFormat::Minimal) => self.render_minimal(result),
            Some(InspectOutputFormat::Table) | None => self.render_table(result),
        }
    }

    /// Render as formatted table (default)
    fn render_table(&self, result: &InspectionResult) -> Result<String> {
        let mut output = Vec::new();

        // Header
        output.push(self.render_header(result));
        output.push(String::new());

        // Resource Information
        output.push(self.render_resource_section(&result.resource));
        output.push(String::new());

        // Health Overview
        output.push(self.render_health_section(&result.health_score, result.complexity_score));
        output.push(String::new());

        // Performance Metrics (if available)
        if result.performance_metrics.cpu_usage.is_some()
            || result.performance_metrics.memory_mb.is_some()
        {
            output.push(self.render_performance_section(&result.performance_metrics));
            output.push(String::new());
        }

        // Errors (if any)
        if result.error_summary.total_errors > 0 {
            output.push(self.render_error_section(&result.error_summary));
            output.push(String::new());
        }

        // Recommendations
        if !result.recommendations.is_empty() {
            output.push(self.render_recommendations_section(&result.recommendations));
            output.push(String::new());
        }

        // Suggested Actions
        if !result.suggested_actions.is_empty() {
            output.push(self.render_actions_section(&result.suggested_actions));
            output.push(String::new());
        }

        Ok(output.join("\n"))
    }

    /// Render as JSON
    fn render_json(&self, result: &InspectionResult) -> Result<String> {
        Ok(serde_json::to_string_pretty(result)?)
    }

    /// Render as YAML
    fn render_yaml(&self, result: &InspectionResult) -> Result<String> {
        // For now, just use JSON format (YAML serialization requires serde_yaml dependency)
        // TODO: Add serde_yaml dependency if YAML output is needed
        self.render_json(result)
    }

    /// Render minimal output (one-line summary)
    fn render_minimal(&self, result: &InspectionResult) -> Result<String> {
        Ok(format!(
            "{}: {} (Health: {:.0}%, Complexity: {:.0}, Errors: {})",
            result.resource.resource_type,
            result.resource.identifier,
            result.health_score.overall_score,
            result.complexity_score,
            result.error_summary.total_errors
        ))
    }

    fn render_header(&self, result: &InspectionResult) -> String {
        let title = format!("📊 Inspection Report: {}", result.resource.identifier);

        if self.config.use_colors {
            format!(
                "{}\n{}\n{}",
                title.bold().cyan(),
                "─".repeat(60).dimmed(),
                format!(
                    "Timestamp: {} | Depth: {} | Complexity: {:.0}",
                    result.timestamp.format("%Y-%m-%d %H:%M:%S"),
                    result.analysis_depth,
                    result.complexity_score
                )
                .dimmed()
            )
        } else {
            format!(
                "{}\n{}\nTimestamp: {} | Depth: {} | Complexity: {:.0}",
                title,
                "─".repeat(60),
                result.timestamp.format("%Y-%m-%d %H:%M:%S"),
                result.analysis_depth,
                result.complexity_score
            )
        }
    }

    fn render_resource_section(&self, resource: &ResolvedResource) -> String {
        let mut lines = vec![self.section_title("Resource Information")];

        let status_display = self.format_status(&resource.status);

        lines.push(format!("  Type:       {}", resource.resource_type));
        lines.push(format!("  Identifier: {}", resource.identifier));
        lines.push(format!("  Status:     {status_display}"));

        if let Some(created) = resource.created_at {
            lines.push(format!(
                "  Created:    {}",
                created.format("%Y-%m-%d %H:%M:%S")
            ));
        }

        if self.config.show_metadata && !resource.metadata.is_empty() {
            lines.push(String::new());
            lines.push("  Metadata:".to_string());
            for (key, value) in &resource.metadata {
                lines.push(format!("    {key}: {value}"));
            }
        }

        lines.join("\n")
    }

    fn render_health_section(&self, health: &HealthScore, complexity: f32) -> String {
        let mut lines = vec![self.section_title("Health Overview")];

        let health_bar = self.render_health_bar(health.overall_score);
        let status_display = self.format_health_status(&health.status);

        lines.push(format!(
            "  Overall Score: {:.0}% {} {}",
            health.overall_score, health_bar, status_display
        ));
        lines.push(format!("  Complexity:    {complexity:.0}"));

        if !health.component_scores.is_empty() {
            lines.push(String::new());
            lines.push("  Component Scores:".to_string());
            for (component, score) in &health.component_scores {
                let bar = self.render_mini_bar(*score);
                lines.push(format!("    {component:<15} {score:.0}% {bar}"));
            }
        }

        if !health.issues.is_empty() {
            lines.push(String::new());
            lines.push("  Issues:".to_string());
            for issue in &health.issues {
                let severity_icon = self.format_severity(&issue.severity);
                lines.push(format!(
                    "    {} [{}] {}",
                    severity_icon, issue.component, issue.description
                ));
            }
        }

        lines.join("\n")
    }

    fn render_performance_section(&self, perf: &PerformanceMetrics) -> String {
        let mut lines = vec![self.section_title("Performance Metrics")];

        if let Some(cpu) = perf.cpu_usage {
            let cpu_bar = self.render_mini_bar(100.0 - cpu);
            lines.push(format!("  CPU Usage:    {cpu:.1}% {cpu_bar}"));
        }

        if let Some(mem) = perf.memory_mb {
            lines.push(format!("  Memory:       {mem:.0} MB"));
        }

        if let Some(throughput) = perf.throughput {
            lines.push(format!("  Throughput:   {throughput:.0} ops/s"));
        }

        if let Some(latency) = perf.latency_ms {
            lines.push(format!("  Latency:      {latency:.1} ms"));
        }

        if let Some(error_rate) = perf.error_rate {
            lines.push(format!("  Error Rate:   {:.3}%", error_rate * 100.0));
        }

        if !perf.issues.is_empty() {
            lines.push(String::new());
            lines.push("  Performance Issues:".to_string());
            for issue in &perf.issues {
                let severity_icon = self.format_severity(&issue.severity);
                lines.push(format!("    {} {}", severity_icon, issue.description));
                if let Some(rec) = &issue.recommendation {
                    lines.push(format!("      💡 {rec}"));
                }
            }
        }

        lines.join("\n")
    }

    fn render_error_section(&self, errors: &ErrorSummary) -> String {
        let mut lines = vec![self.section_title(&format!("Errors ({})", errors.total_errors))];

        if !errors.recent_errors.is_empty() {
            lines.push("  Recent Errors:".to_string());
            for error in errors.recent_errors.iter().take(5) {
                lines.push(format!(
                    "    {} [{}] {}",
                    error.timestamp.format("%H:%M:%S"),
                    error.error_type,
                    error.message
                ));
                if let Some(ctx) = &error.context {
                    lines.push(format!("      Context: {ctx}"));
                }
            }
        }

        if !errors.error_patterns.is_empty() {
            lines.push(String::new());
            lines.push("  Error Patterns:".to_string());
            for pattern in &errors.error_patterns {
                lines.push(format!(
                    "    {} ({}x) - {}",
                    pattern.pattern_type, pattern.frequency, pattern.description
                ));
            }
        }

        lines.join("\n")
    }

    fn render_recommendations_section(&self, recommendations: &[Recommendation]) -> String {
        let mut lines = vec![self.section_title("Recommendations")];

        let display_count = self.config.max_recommendations.min(recommendations.len());

        for rec in recommendations.iter().take(display_count) {
            let priority_icon = self.format_priority(&rec.priority);
            let impact_badge = self.format_impact(&rec.impact);

            lines.push(format!(
                "  {} {} {}",
                priority_icon, rec.title, impact_badge
            ));
            lines.push(format!("     {}", rec.description));

            if let Some(cmd) = &rec.suggested_command {
                lines.push(format!("     💻 {}", self.format_command(cmd)));
            }
            lines.push(String::new());
        }

        if recommendations.len() > display_count {
            lines.push(format!(
                "  ... and {} more recommendations",
                recommendations.len() - display_count
            ));
        }

        lines.join("\n")
    }

    fn render_actions_section(&self, actions: &[SuggestedAction]) -> String {
        let mut lines = vec![self.section_title("Suggested Actions")];

        for action in actions {
            let action_icon = self.format_action_type(&action.action_type);
            lines.push(format!("  {} {}", action_icon, action.description));

            if let Some(cmd) = &action.command {
                lines.push(format!("     {}", self.format_command(cmd)));
            }
        }

        lines.join("\n")
    }

    // Formatting helpers

    fn section_title(&self, title: &str) -> String {
        if self.config.use_colors {
            format!("▶ {}", title.bold())
        } else {
            format!("▶ {title}")
        }
    }

    fn render_health_bar(&self, score: f32) -> String {
        let filled = ((score / 100.0) * 20.0) as usize;
        let empty = 20 - filled;

        let bar = format!("[{}{}]", "█".repeat(filled), "░".repeat(empty));

        if self.config.use_colors {
            if score >= 90.0 {
                bar.green().to_string()
            } else if score >= 70.0 {
                bar.yellow().to_string()
            } else {
                bar.red().to_string()
            }
        } else {
            bar
        }
    }

    fn render_mini_bar(&self, score: f32) -> String {
        let filled = ((score / 100.0) * 10.0) as usize;
        let empty = 10 - filled;

        let bar = format!("[{}{}]", "▰".repeat(filled), "▱".repeat(empty));

        if self.config.use_colors {
            if score >= 90.0 {
                bar.green().to_string()
            } else if score >= 70.0 {
                bar.yellow().to_string()
            } else {
                bar.red().to_string()
            }
        } else {
            bar
        }
    }

    fn format_status(&self, status: &ResourceStatus) -> String {
        let (text, color_fn): (&str, fn(String) -> colored::ColoredString) = match status {
            ResourceStatus::Running => ("Running", |s| s.green()),
            ResourceStatus::Stopped => ("Stopped", |s| s.yellow()),
            ResourceStatus::Error => ("Error", |s| s.red()),
            ResourceStatus::Unknown => ("Unknown", |s| s.dimmed()),
        };

        if self.config.use_colors {
            color_fn(text.to_string()).to_string()
        } else {
            text.to_string()
        }
    }

    fn format_health_status(&self, status: &HealthStatus) -> String {
        let (icon, color_fn): (&str, fn(String) -> colored::ColoredString) = match status {
            HealthStatus::Healthy => ("✓", |s| s.green()),
            HealthStatus::Warning => ("⚠", |s| s.yellow()),
            HealthStatus::Critical => ("✗", |s| s.red()),
            HealthStatus::Unknown => ("?", |s| s.dimmed()),
        };

        if self.config.use_colors {
            color_fn(icon.to_string()).to_string()
        } else {
            icon.to_string()
        }
    }

    fn format_severity(&self, severity: &IssueSeverity) -> String {
        let (icon, color_fn): (&str, fn(String) -> colored::ColoredString) = match severity {
            IssueSeverity::Critical => ("🔴", |s| s.red()),
            IssueSeverity::High => ("🟠", |s| s.red()),
            IssueSeverity::Medium => ("🟡", |s| s.yellow()),
            IssueSeverity::Low => ("🟢", |s| s.green()),
        };

        if self.config.use_colors {
            color_fn(icon.to_string()).to_string()
        } else {
            icon.to_string()
        }
    }

    fn format_priority(&self, priority: &RecommendationPriority) -> String {
        let (icon, color_fn): (&str, fn(String) -> colored::ColoredString) = match priority {
            RecommendationPriority::Critical => ("🔴", |s| s.red().bold()),
            RecommendationPriority::High => ("🟠", |s| s.red()),
            RecommendationPriority::Medium => ("🟡", |s| s.yellow()),
            RecommendationPriority::Low => ("🔵", |s| s.blue()),
        };

        if self.config.use_colors {
            color_fn(icon.to_string()).to_string()
        } else {
            icon.to_string()
        }
    }

    fn format_impact(&self, impact: &ImpactLevel) -> String {
        let text = match impact {
            ImpactLevel::High => "[HIGH IMPACT]",
            ImpactLevel::Medium => "[MEDIUM IMPACT]",
            ImpactLevel::Low => "[LOW IMPACT]",
        };

        if self.config.use_colors {
            text.dimmed().to_string()
        } else {
            text.to_string()
        }
    }

    fn format_action_type(&self, action_type: &ActionType) -> String {
        match action_type {
            ActionType::Restart => "🔄".to_string(),
            ActionType::ScaleUp => "📈".to_string(),
            ActionType::Investigate => "🔍".to_string(),
            ActionType::Optimize => "⚡".to_string(),
            ActionType::Monitor => "📊".to_string(),
        }
    }

    fn format_command(&self, cmd: &str) -> String {
        if self.config.use_colors {
            cmd.cyan().to_string()
        } else {
            cmd.to_string()
        }
    }
}

impl Default for CliRenderer {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use chrono::Utc;
    use std::collections::HashMap;

    fn create_test_result() -> InspectionResult {
        let mut metadata = HashMap::new();
        metadata.insert("type".to_string(), "test".to_string());

        InspectionResult {
            resource: ResolvedResource {
                resource_type: "dataflow".to_string(),
                identifier: "test-dataflow".to_string(),
                status: ResourceStatus::Running,
                created_at: Some(Utc::now()),
                metadata,
            },
            timestamp: Utc::now(),
            analysis_depth: 2,
            complexity_score: 45.0,
            health_score: HealthScore {
                overall_score: 85.0,
                status: HealthStatus::Healthy,
                component_scores: {
                    let mut scores = HashMap::new();
                    scores.insert("runtime".to_string(), 90.0);
                    scores.insert("connectivity".to_string(), 80.0);
                    scores
                },
                issues: vec![],
            },
            performance_metrics: PerformanceMetrics {
                cpu_usage: Some(45.0),
                memory_mb: Some(256.0),
                throughput: Some(1000.0),
                latency_ms: Some(15.0),
                error_rate: Some(0.01),
                issues: vec![],
            },
            error_summary: ErrorSummary {
                total_errors: 0,
                recent_errors: vec![],
                error_patterns: vec![],
            },
            recommendations: vec![Recommendation {
                priority: RecommendationPriority::Medium,
                title: "Test Recommendation".to_string(),
                description: "This is a test recommendation".to_string(),
                suggested_command: Some("dora inspect test".to_string()),
                impact: ImpactLevel::Low,
            }],
            suggested_actions: vec![SuggestedAction {
                action_type: ActionType::Monitor,
                description: "Monitor the resource".to_string(),
                command: Some("dora inspect test --live".to_string()),
            }],
        }
    }

    #[test]
    fn test_render_table() {
        let renderer = CliRenderer::new();
        let result = create_test_result();

        let output = renderer
            .render(&result, Some(InspectOutputFormat::Table))
            .unwrap();

        assert!(output.contains("Inspection Report"));
        assert!(output.contains("test-dataflow"));
        assert!(output.contains("Health Overview"));
    }

    #[test]
    fn test_render_json() {
        let renderer = CliRenderer::new();
        let result = create_test_result();

        let output = renderer
            .render(&result, Some(InspectOutputFormat::Json))
            .unwrap();

        assert!(output.contains("\"resource\""));
        assert!(output.contains("\"test-dataflow\""));
    }

    #[test]
    fn test_render_yaml() {
        let renderer = CliRenderer::new();
        let result = create_test_result();

        let output = renderer
            .render(&result, Some(InspectOutputFormat::Yaml))
            .unwrap();

        // YAML rendering currently uses JSON format (TODO: add serde_yaml dependency)
        assert!(output.contains("\"resource\""));
        assert!(output.contains("test-dataflow"));
    }

    #[test]
    fn test_health_bar_rendering() {
        let renderer = CliRenderer::new();

        let bar_high = renderer.render_health_bar(95.0);
        let bar_medium = renderer.render_health_bar(75.0);
        let bar_low = renderer.render_health_bar(45.0);

        assert!(bar_high.contains("█"));
        assert!(bar_medium.contains("█"));
        assert!(bar_low.contains("█"));
    }

    #[test]
    fn test_renderer_without_colors() {
        let config = RendererConfig {
            use_colors: false,
            max_recommendations: 5,
            show_metadata: true,
        };
        let renderer = CliRenderer::with_config(config);
        let result = create_test_result();

        let output = renderer.render(&result, None).unwrap();

        // Should not contain ANSI color codes
        assert!(!output.contains("\x1b["));
    }
}
