// Analysis Rendering for Issue #19

use super::types::*;
use colored::Colorize;
use eyre::Result;

/// CLI renderer for analysis results
pub struct AnalysisCliRenderer {
    supports_color: bool,
    terminal_width: usize,
}

impl AnalysisCliRenderer {
    pub fn new(supports_color: bool, terminal_width: usize) -> Self {
        Self {
            supports_color,
            terminal_width,
        }
    }

    pub fn render_analysis(&self, results: &AnalysisResults) -> Result<()> {
        // Header
        self.render_header(&results.session_info);

        // Key findings summary
        self.render_key_findings(results);

        // Performance summary (if available)
        if let Some(performance) = &results.performance_analysis {
            self.render_performance_summary(performance);
        }

        // Health summary (if available)
        if let Some(health) = &results.health_analysis {
            self.render_health_summary(health);
        }

        // Trend summary (if available)
        if let Some(trends) = &results.trend_analysis {
            self.render_trend_summary(trends);
        }

        // Insights and recommendations
        self.render_insights_and_recommendations(results);

        // Overall scores
        self.render_overall_scores(&results.overall_scores);

        Ok(())
    }

    fn render_header(&self, session_info: &AnalysisSessionInfo) {
        println!();
        if self.supports_color {
            println!("{}", "📊 Analysis Results".bold().cyan());
        } else {
            println!("=== Analysis Results ===");
        }
        println!("{}", "═".repeat(self.terminal_width.min(80)));
        println!();
        println!("Session ID: {}", session_info.session_id);
        println!(
            "Analysis Duration: {:.2}s",
            session_info.analysis_duration_secs
        );
        println!("Data Points: {}", session_info.total_data_points);
        println!();
    }

    fn render_key_findings(&self, results: &AnalysisResults) {
        if self.supports_color {
            println!("{}", "📌 Key Findings".bold());
        } else {
            println!("=== Key Findings ===");
        }
        println!();

        // Show top insights
        let top_insights: Vec<_> = results.insights.iter().take(3).collect();

        if top_insights.is_empty() {
            println!("  No significant findings");
        } else {
            for (i, insight) in top_insights.iter().enumerate() {
                let priority_icon = match insight.priority {
                    InsightPriority::Critical => "🔴",
                    InsightPriority::High => "🟠",
                    InsightPriority::Medium => "🟡",
                    InsightPriority::Low => "🔵",
                };

                println!("  {}. {} {}", i + 1, priority_icon, insight.title);

                // Wrap description to terminal width
                let wrapped_description =
                    self.wrap_text(&insight.description, self.terminal_width - 6);
                for line in wrapped_description {
                    println!("     {line}");
                }

                if let Some(metric_change) = &insight.metric_change {
                    let (change_color, reset_color) = if self.supports_color {
                        if metric_change.is_improvement() {
                            ("\x1b[32m", "\x1b[0m")
                        } else {
                            ("\x1b[31m", "\x1b[0m")
                        }
                    } else {
                        ("", "")
                    };

                    println!(
                        "     {}📈 {}: {}{}",
                        change_color,
                        metric_change.metric_name,
                        metric_change.format_change(),
                        reset_color
                    );
                }

                println!();
            }
        }
    }

    fn render_performance_summary(&self, performance: &PerformanceAnalysisResult) {
        if self.supports_color {
            println!("{}", "⚡ Performance Analysis".bold());
        } else {
            println!("=== Performance Analysis ===");
        }
        println!();

        // Throughput summary
        println!("  Throughput:");
        println!("    Current: {:.2} msg/s", performance.throughput.current);
        println!("    Average: {:.2} msg/s", performance.throughput.average);
        println!("    Peak: {:.2} msg/s", performance.throughput.peak);

        if performance.throughput.trend != TrendDirection::Stable {
            let trend_icon = match performance.throughput.trend {
                TrendDirection::Increasing => "📈",
                TrendDirection::Decreasing => "📉",
                _ => "➡️",
            };
            println!(
                "    Trend: {} {:?}",
                trend_icon, performance.throughput.trend
            );
        }

        // Latency summary
        println!("  Latency:");
        println!("    P50: {:.1}ms", performance.latency.percentiles.p50);
        println!("    P95: {:.1}ms", performance.latency.percentiles.p95);
        println!("    P99: {:.1}ms", performance.latency.percentiles.p99);

        // Resource usage
        println!("  Resource Usage:");
        println!("    CPU: {:.1}%", performance.resource_usage.cpu_usage);
        println!(
            "    Memory: {:.1}%",
            performance.resource_usage.memory_usage
        );
        println!(
            "    Efficiency: {:.1}%",
            performance.resource_usage.resource_efficiency
        );

        // Bottlenecks
        if !performance.bottlenecks.is_empty() {
            println!("  Bottlenecks Detected:");
            for bottleneck in performance.bottlenecks.iter().take(3) {
                let severity_icon = match bottleneck.severity {
                    BottleneckSeverity::Critical => "🔴",
                    BottleneckSeverity::High => "🟠",
                    BottleneckSeverity::Medium => "🟡",
                    BottleneckSeverity::Low => "🔵",
                };
                println!("    {} {}", severity_icon, bottleneck.description);
            }
        }

        println!();
    }

    fn render_health_summary(&self, health: &HealthAnalysisResult) {
        if self.supports_color {
            println!("{}", "❤️  Health Analysis".bold());
        } else {
            println!("=== Health Analysis ===");
        }
        println!();

        println!(
            "  Overall Health Score: {:.1}/100",
            health.overall_health_score
        );
        println!("  Health Trend: {:?}", health.health_trend);
        println!();

        if !health.component_health.is_empty() {
            println!("  Component Health:");
            for component in &health.component_health {
                let status_icon = match component.status {
                    HealthStatus::Healthy => "✅",
                    HealthStatus::Degraded => "⚠️",
                    HealthStatus::Critical => "🔴",
                    HealthStatus::Unknown => "❓",
                };
                println!(
                    "    {} {}: {:.1}/100",
                    status_icon, component.component_name, component.health_score
                );
            }
        }

        println!();
    }

    fn render_trend_summary(&self, trends: &TrendAnalysisResult) {
        if self.supports_color {
            println!("{}", "📈 Trend Analysis".bold());
        } else {
            println!("=== Trend Analysis ===");
        }
        println!();

        println!("  Total Trends: {}", trends.trend_summary.total_trends);
        println!("  Increasing: {}", trends.trend_summary.increasing_trends);
        println!("  Decreasing: {}", trends.trend_summary.decreasing_trends);
        println!("  Stable: {}", trends.trend_summary.stable_trends);

        if !trends.trend_summary.key_findings.is_empty() {
            println!();
            println!("  Key Findings:");
            for finding in &trends.trend_summary.key_findings {
                println!("    • {finding}");
            }
        }

        println!();
    }

    fn render_insights_and_recommendations(&self, results: &AnalysisResults) {
        if !results.recommendations.is_empty() {
            if self.supports_color {
                println!("{}", "💡 Recommendations".bold().cyan());
            } else {
                println!("=== Recommendations ===");
            }
            println!();

            for (i, rec) in results.recommendations.iter().take(5).enumerate() {
                let priority_icon = match rec.priority {
                    RecommendationPriority::Critical => "🔴",
                    RecommendationPriority::High => "🟠",
                    RecommendationPriority::Medium => "🟡",
                    RecommendationPriority::Low => "🔵",
                };

                println!("  {}. {} {}", i + 1, priority_icon, rec.title);
                println!("     {}", rec.description);
                if !rec.action_items.is_empty() {
                    println!("     Actions:");
                    for action in &rec.action_items {
                        println!("       • {action}");
                    }
                }
                println!();
            }
        }
    }

    fn render_overall_scores(&self, scores: &OverallScores) {
        if self.supports_color {
            println!("{}", "📊 Overall Scores".bold());
        } else {
            println!("=== Overall Scores ===");
        }
        println!();

        self.render_score_bar("Performance", scores.performance_score);
        self.render_score_bar("Health", scores.health_score);
        self.render_score_bar("Efficiency", scores.efficiency_score);
        self.render_score_bar("Reliability", scores.reliability_score);
        println!();
        self.render_score_bar("Overall", scores.overall_score);

        println!();
    }

    fn render_score_bar(&self, label: &str, score: f32) {
        let bar_width = 30;
        let filled = ((score / 100.0 * bar_width as f32) as usize).min(bar_width);
        let empty = bar_width - filled;

        let (color, reset) = if self.supports_color {
            if score >= 80.0 {
                ("\x1b[32m", "\x1b[0m") // Green
            } else if score >= 60.0 {
                ("\x1b[33m", "\x1b[0m") // Yellow
            } else {
                ("\x1b[31m", "\x1b[0m") // Red
            }
        } else {
            ("", "")
        };

        println!(
            "  {:12} [{}{}{}{}] {:.1}%",
            label,
            color,
            "█".repeat(filled),
            "░".repeat(empty),
            reset,
            score
        );
    }

    fn wrap_text(&self, text: &str, width: usize) -> Vec<String> {
        let mut lines = Vec::new();
        let mut current_line = String::new();

        for word in text.split_whitespace() {
            if current_line.len() + word.len() + 1 > width && !current_line.is_empty() {
                lines.push(current_line.clone());
                current_line.clear();
            }

            if !current_line.is_empty() {
                current_line.push(' ');
            }
            current_line.push_str(word);
        }

        if !current_line.is_empty() {
            lines.push(current_line);
        }

        lines
    }
}

/// Interactive analysis benefits calculator
pub fn get_interactive_analysis_benefits(
    complexity: &super::complexity::ResultComplexity,
    live_mode: bool,
    predict: bool,
) -> Vec<String> {
    let mut benefits = Vec::new();

    // Always available benefits
    benefits.push("Interactive data visualization and exploration".to_string());
    benefits.push("Drill-down analysis with dynamic filtering".to_string());

    // Complexity-based benefits
    if complexity.data_volume_score > 3.0 {
        benefits.push("Efficient navigation of large datasets".to_string());
    }

    if complexity.dimension_score > 2.0 {
        benefits.push("Multi-dimensional analysis correlation views".to_string());
    }

    if complexity.problem_score > 1.0 {
        benefits.push("Interactive problem investigation workflows".to_string());
    }

    if complexity.visualization_score > 2.0 {
        benefits.push("Advanced charting and trend visualization".to_string());
    }

    if live_mode {
        benefits.push("Real-time analysis updates and monitoring".to_string());
    }

    if predict {
        benefits.push("Interactive prediction modeling and scenarios".to_string());
    }

    benefits
}
