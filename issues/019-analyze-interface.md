# Issue #019: Build Enhanced `dora analyze` Interface

## 📋 Summary
Implement an intelligent `dora analyze` command that provides comprehensive dataflow and system analysis with adaptive interface selection based on data complexity, analysis depth, and user context. This command showcases the hybrid CLI's ability to handle complex analytical tasks while providing appropriate visualization and interaction modes.

## 🎯 Objectives
- Create comprehensive analysis capabilities across dataflows, nodes, and system components
- Implement intelligent interface selection based on data volume, complexity, and analysis type
- Add real-time and historical analysis with adaptive visualization
- Provide actionable insights with contextual recommendations
- Enable seamless transitions between quick CLI reports and detailed interactive analysis

**Success Metrics:**
- Analysis completion time under 10 seconds for typical dataflows
- Interface suggestions are appropriate 90% of the time based on analysis complexity
- Interactive analysis reduces insight discovery time by 40% compared to static reports
- User satisfaction with analysis depth and clarity exceeds 85%
- Analysis accuracy and relevance improves by 20% through user feedback

## 🛠️ Technical Requirements

### What to Build

#### 1. Enhanced Analyze Command Structure
```rust
// src/cli/commands/analyze.rs
#[derive(Debug, clap::Args)]
pub struct AnalyzeCommand {
    /// Target to analyze (dataflow, node, system, or recording)
    pub target: Option<String>,
    
    /// Analysis type
    #[clap(long, value_enum, default_value = "comprehensive")]
    pub analysis_type: AnalysisType,
    
    /// Analysis depth level
    #[clap(long, default_value = "normal")]
    pub depth: AnalysisDepth,
    
    /// Time window for analysis
    #[clap(long, default_value = "1h")]
    pub window: String,
    
    /// Focus areas for analysis
    #[clap(long, value_enum)]
    pub focus: Vec<AnalysisFocus>,
    
    /// Include comparative analysis with baseline
    #[clap(long)]
    pub compare: bool,
    
    /// Baseline period for comparison
    #[clap(long, default_value = "24h")]
    pub baseline: String,
    
    /// Generate detailed report
    #[clap(long)]
    pub detailed: bool,
    
    /// Export analysis results
    #[clap(long)]
    pub export: Option<PathBuf>,
    
    /// Export format
    #[clap(long, value_enum, default_value = "json")]
    pub format: ExportFormat,
    
    /// Real-time analysis mode
    #[clap(long)]
    pub live: bool,
    
    /// Analysis refresh interval for live mode
    #[clap(long, default_value = "5s")]
    pub refresh: String,
    
    /// Force CLI text output
    #[clap(long)]
    pub text: bool,
    
    /// Force TUI interactive mode
    #[clap(long)]
    pub tui: bool,
    
    /// Include predictive analysis
    #[clap(long)]
    pub predict: bool,
    
    /// Prediction horizon
    #[clap(long, default_value = "1h")]
    pub horizon: String,
    
    /// Analysis verbosity
    #[clap(long, default_value = "normal")]
    pub verbosity: AnalysisVerbosity,
}

#[derive(Debug, Clone, clap::ValueEnum)]
pub enum AnalysisType {
    Comprehensive,  // Full multi-dimensional analysis
    Performance,    // Performance-focused analysis
    Health,         // Health and diagnostics
    Security,       // Security and compliance analysis
    Efficiency,     // Resource efficiency analysis
    Trends,         // Trend and pattern analysis
    Comparison,     // Comparative analysis
    Custom,         // User-defined analysis
}

#[derive(Debug, Clone, clap::ValueEnum)]
pub enum AnalysisDepth {
    Quick,      // Surface-level analysis
    Normal,     // Standard depth analysis
    Deep,       // Comprehensive deep analysis
    Expert,     // Expert-level detailed analysis
}

#[derive(Debug, Clone, clap::ValueEnum)]
pub enum AnalysisFocus {
    Performance,
    Reliability,
    Security,
    Resource,
    Dependencies,
    Data,
    Network,
    Errors,
    Patterns,
    Anomalies,
}

impl AnalyzeCommand {
    pub async fn execute(&self, context: &ExecutionContext) -> Result<()> {
        // Initialize analysis session
        let analysis_session = self.initialize_analysis_session(context).await?;
        
        // Perform comprehensive analysis
        let analysis_results = self.perform_analysis(&analysis_session).await?;
        
        // Analyze result complexity for interface decision
        let result_complexity = self.analyze_result_complexity(&analysis_results, &analysis_session).await?;
        
        // Determine interface strategy
        let interface_selector = InterfaceSelector::new(context.clone(), UserConfig::load()?);
        let interface_decision = interface_selector.select_interface(&Command::Analyze(self.clone()));
        
        // Apply analysis-specific overrides
        let final_decision = self.apply_analysis_overrides(interface_decision, &result_complexity);
        
        match final_decision.strategy {
            InterfaceStrategy::CliOnly => {
                self.render_cli_analysis(&analysis_results, context).await?;
            },
            
            InterfaceStrategy::CliWithHint { hint, tui_command } => {
                self.render_cli_analysis(&analysis_results, context).await?;
                self.show_analysis_hint(&hint, &tui_command, &result_complexity);
            },
            
            InterfaceStrategy::PromptForTui { reason, default_yes } => {
                // Show CLI summary first
                self.render_analysis_summary(&analysis_results, context).await?;
                
                if self.should_launch_interactive_analysis(&reason, &result_complexity, default_yes)? {
                    self.launch_interactive_analysis(&analysis_session, analysis_results).await?;
                } else {
                    self.render_detailed_cli_analysis(&analysis_results, context).await?;
                }
            },
            
            InterfaceStrategy::AutoLaunchTui { reason, show_cli_first } => {
                if show_cli_first {
                    self.render_analysis_summary(&analysis_results, context).await?;
                    println!("\n📊 {}", reason);
                }
                self.launch_interactive_analysis(&analysis_session, analysis_results).await?;
            },
        }
        
        // Export results if requested
        if let Some(export_path) = &self.export {
            self.export_analysis_results(&analysis_results, export_path).await?;
        }
        
        Ok(())
    }
    
    async fn initialize_analysis_session(&self, context: &ExecutionContext) -> Result<AnalysisSession> {
        let session_id = Uuid::new_v4();
        let start_time = Utc::now();
        
        // Determine analysis target
        let analysis_target = if let Some(target) = &self.target {
            self.resolve_analysis_target(target).await?
        } else {
            self.auto_detect_analysis_target().await?
        };
        
        // Parse time window
        let time_window = self.parse_time_window(&self.window)?;
        let baseline_window = if self.compare {
            Some(self.parse_time_window(&self.baseline)?)
        } else {
            None
        };
        
        // Initialize analysis engines
        let analysis_engines = self.initialize_analysis_engines(&analysis_target).await?;
        
        Ok(AnalysisSession {
            session_id,
            start_time,
            analysis_target,
            analysis_type: self.analysis_type.clone(),
            depth: self.depth.clone(),
            focus_areas: self.focus.clone(),
            time_window,
            baseline_window,
            analysis_engines,
            configuration: AnalysisConfiguration::from_command(self),
        })
    }
    
    async fn perform_analysis(&self, session: &AnalysisSession) -> Result<AnalysisResults> {
        let mut results = AnalysisResults::new(session);
        
        // Collect data for analysis
        let data_collection = self.collect_analysis_data(session).await?;
        
        // Perform core analysis
        match self.analysis_type {
            AnalysisType::Comprehensive => {
                results.performance_analysis = Some(
                    session.analysis_engines.performance.analyze(&data_collection).await?
                );
                results.health_analysis = Some(
                    session.analysis_engines.health.analyze(&data_collection).await?
                );
                results.efficiency_analysis = Some(
                    session.analysis_engines.efficiency.analyze(&data_collection).await?
                );
                results.trend_analysis = Some(
                    session.analysis_engines.trends.analyze(&data_collection).await?
                );
            },
            
            AnalysisType::Performance => {
                results.performance_analysis = Some(
                    session.analysis_engines.performance.analyze(&data_collection).await?
                );
                if self.predict {
                    results.predictive_analysis = Some(
                        session.analysis_engines.prediction.analyze(&data_collection).await?
                    );
                }
            },
            
            AnalysisType::Health => {
                results.health_analysis = Some(
                    session.analysis_engines.health.analyze(&data_collection).await?
                );
                results.diagnostics = Some(
                    session.analysis_engines.diagnostics.analyze(&data_collection).await?
                );
            },
            
            AnalysisType::Security => {
                results.security_analysis = Some(
                    session.analysis_engines.security.analyze(&data_collection).await?
                );
            },
            
            AnalysisType::Trends => {
                results.trend_analysis = Some(
                    session.analysis_engines.trends.analyze(&data_collection).await?
                );
                results.pattern_analysis = Some(
                    session.analysis_engines.patterns.analyze(&data_collection).await?
                );
            },
            
            _ => {
                // Handle other analysis types
            }
        }
        
        // Perform comparative analysis if requested
        if self.compare && session.baseline_window.is_some() {
            results.comparative_analysis = Some(
                self.perform_comparative_analysis(session, &data_collection).await?
            );
        }
        
        // Generate insights and recommendations
        results.insights = self.generate_insights(&results).await?;
        results.recommendations = self.generate_recommendations(&results).await?;
        
        // Calculate overall scores
        results.overall_scores = self.calculate_overall_scores(&results);
        
        Ok(results)
    }
}
```

#### 2. Comprehensive Analysis Engines
```rust
// src/analysis/engines.rs
#[derive(Debug)]
pub struct AnalysisEngines {
    pub performance: PerformanceAnalysisEngine,
    pub health: HealthAnalysisEngine,
    pub efficiency: EfficiencyAnalysisEngine,
    pub trends: TrendAnalysisEngine,
    pub patterns: PatternAnalysisEngine,
    pub security: SecurityAnalysisEngine,
    pub prediction: PredictionAnalysisEngine,
    pub diagnostics: DiagnosticsAnalysisEngine,
}

#[derive(Debug, Clone)]
pub struct AnalysisResults {
    pub session_info: AnalysisSessionInfo,
    pub performance_analysis: Option<PerformanceAnalysisResult>,
    pub health_analysis: Option<HealthAnalysisResult>,
    pub efficiency_analysis: Option<EfficiencyAnalysisResult>,
    pub trend_analysis: Option<TrendAnalysisResult>,
    pub pattern_analysis: Option<PatternAnalysisResult>,
    pub security_analysis: Option<SecurityAnalysisResult>,
    pub predictive_analysis: Option<PredictiveAnalysisResult>,
    pub comparative_analysis: Option<ComparativeAnalysisResult>,
    pub diagnostics: Option<DiagnosticsResult>,
    pub insights: Vec<AnalysisInsight>,
    pub recommendations: Vec<AnalysisRecommendation>,
    pub overall_scores: OverallScores,
}

#[derive(Debug)]
pub struct PerformanceAnalysisEngine {
    metric_analyzer: MetricAnalyzer,
    bottleneck_detector: BottleneckDetector,
    efficiency_calculator: EfficiencyCalculator,
}

impl PerformanceAnalysisEngine {
    pub async fn analyze(&self, data: &DataCollection) -> Result<PerformanceAnalysisResult> {
        let start_time = Instant::now();
        
        // Analyze core performance metrics
        let throughput_analysis = self.analyze_throughput(&data.metrics).await?;
        let latency_analysis = self.analyze_latency(&data.metrics).await?;
        let resource_analysis = self.analyze_resource_usage(&data.metrics).await?;
        
        // Detect performance bottlenecks
        let bottlenecks = self.bottleneck_detector.detect_bottlenecks(data).await?;
        
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
                analysis_duration,
                data_points_analyzed: data.metrics.len(),
                time_range: data.time_range.clone(),
            },
        })
    }
    
    async fn analyze_throughput(&self, metrics: &[MetricPoint]) -> Result<ThroughputAnalysis> {
        let throughput_metrics: Vec<_> = metrics.iter()
            .filter(|m| m.metric_name.contains("throughput") || m.metric_name.contains("messages_per_second"))
            .collect();
        
        if throughput_metrics.is_empty() {
            return Ok(ThroughputAnalysis::empty());
        }
        
        let current_throughput = throughput_metrics.last()
            .map(|m| m.value)
            .unwrap_or(0.0);
        
        let average_throughput = throughput_metrics.iter()
            .map(|m| m.value)
            .sum::<f32>() / throughput_metrics.len() as f32;
        
        let peak_throughput = throughput_metrics.iter()
            .map(|m| m.value)
            .fold(0.0f32, |acc, &val| acc.max(val));
        
        let throughput_trend = self.calculate_throughput_trend(&throughput_metrics);
        let throughput_stability = self.calculate_throughput_stability(&throughput_metrics);
        
        Ok(ThroughputAnalysis {
            current: current_throughput,
            average: average_throughput,
            peak: peak_throughput,
            trend: throughput_trend,
            stability: throughput_stability,
            percentiles: self.calculate_throughput_percentiles(&throughput_metrics),
            time_series: throughput_metrics.iter()
                .map(|m| TimeSeriesPoint {
                    timestamp: m.timestamp,
                    value: m.value,
                })
                .collect(),
        })
    }
}

#[derive(Debug)]
pub struct TrendAnalysisEngine {
    pattern_detector: PatternDetector,
    seasonal_analyzer: SeasonalAnalyzer,
    anomaly_detector: TrendAnomalyDetector,
}

impl TrendAnalysisEngine {
    pub async fn analyze(&self, data: &DataCollection) -> Result<TrendAnalysisResult> {
        let mut trends = Vec::new();
        
        // Analyze trends for each metric
        for metric_group in data.metrics.group_by_metric_name() {
            let metric_trends = self.analyze_metric_trends(&metric_group).await?;
            trends.extend(metric_trends);
        }
        
        // Detect seasonal patterns
        let seasonal_patterns = self.seasonal_analyzer.detect_patterns(data).await?;
        
        // Identify anomalous trends
        let trend_anomalies = self.anomaly_detector.detect_anomalies(&trends).await?;
        
        // Generate trend predictions
        let predictions = self.generate_trend_predictions(&trends).await?;
        
        Ok(TrendAnalysisResult {
            trends,
            seasonal_patterns,
            anomalies: trend_anomalies,
            predictions,
            trend_summary: self.generate_trend_summary(&trends),
        })
    }
    
    async fn analyze_metric_trends(&self, metric_data: &MetricGroup) -> Result<Vec<Trend>> {
        let mut trends = Vec::new();
        
        // Calculate moving averages
        let short_ma = self.calculate_moving_average(&metric_data.values, 5);
        let long_ma = self.calculate_moving_average(&metric_data.values, 20);
        
        // Determine trend direction
        let trend_direction = if short_ma.last().unwrap_or(&0.0) > long_ma.last().unwrap_or(&0.0) {
            TrendDirection::Increasing
        } else {
            TrendDirection::Decreasing
        };
        
        // Calculate trend strength
        let trend_strength = self.calculate_trend_strength(&metric_data.values);
        
        // Detect trend changes
        let trend_changes = self.detect_trend_changes(&metric_data.values);
        
        trends.push(Trend {
            metric_name: metric_data.metric_name.clone(),
            direction: trend_direction,
            strength: trend_strength,
            duration: metric_data.time_span(),
            changes: trend_changes,
            confidence: self.calculate_trend_confidence(&metric_data.values),
            significance: self.assess_trend_significance(&metric_data.values),
        });
        
        Ok(trends)
    }
}
```

#### 3. Intelligent Result Complexity Analysis
```rust
// src/analysis/result_complexity.rs
impl AnalyzeCommand {
    async fn analyze_result_complexity(
        &self,
        results: &AnalysisResults,
        session: &AnalysisSession,
    ) -> Result<ResultComplexity> {
        let mut complexity_score = 0.0;
        let mut complexity_factors = Vec::new();
        
        // Data volume complexity
        let data_points = results.session_info.total_data_points;
        let volume_complexity = match data_points {
            0..=1000 => 1.0,
            1001..=10000 => 2.0,
            10001..=100000 => 4.0,
            _ => 6.0,
        };
        complexity_score += volume_complexity;
        
        if data_points > 10000 {
            complexity_factors.push(ComplexityFactor {
                factor_type: FactorType::DataVolume,
                impact: volume_complexity,
                description: format!("Large dataset with {} data points", data_points),
                evidence: vec![format!("{} metrics analyzed", data_points)],
            });
        }
        
        // Analysis dimension complexity
        let analysis_dimensions = self.count_analysis_dimensions(results);
        let dimension_complexity = analysis_dimensions as f32 * 0.5;
        complexity_score += dimension_complexity;
        
        if analysis_dimensions > 4 {
            complexity_factors.push(ComplexityFactor {
                factor_type: FactorType::AnalysisDimensions,
                impact: dimension_complexity,
                description: format!("Multi-dimensional analysis across {} areas", analysis_dimensions),
                evidence: self.get_analysis_dimension_names(results),
            });
        }
        
        // Result insight complexity
        let insights_count = results.insights.len();
        let insight_complexity = match insights_count {
            0..=3 => 0.0,
            4..=8 => 2.0,
            9..=15 => 4.0,
            _ => 6.0,
        };
        complexity_score += insight_complexity;
        
        // Anomaly and issue complexity
        let anomaly_count = self.count_anomalies(results);
        let issue_count = self.count_issues(results);
        let problem_complexity = (anomaly_count + issue_count) as f32 * 0.5;
        complexity_score += problem_complexity;
        
        if anomaly_count > 0 || issue_count > 0 {
            complexity_factors.push(ComplexityFactor {
                factor_type: FactorType::ProblemComplexity,
                impact: problem_complexity,
                description: format!("{} anomalies and {} issues detected", anomaly_count, issue_count),
                evidence: vec![
                    format!("Anomalies: {}", anomaly_count),
                    format!("Issues: {}", issue_count),
                ],
            });
        }
        
        // Visualization complexity
        let visualization_complexity = self.assess_visualization_complexity(results);
        complexity_score += visualization_complexity;
        
        // Trend complexity
        if let Some(trend_analysis) = &results.trend_analysis {
            let trend_complexity = trend_analysis.trends.len() as f32 * 0.3;
            complexity_score += trend_complexity;
            
            if trend_analysis.trends.len() > 5 {
                complexity_factors.push(ComplexityFactor {
                    factor_type: FactorType::TrendComplexity,
                    impact: trend_complexity,
                    description: format!("{} trends identified requiring analysis", trend_analysis.trends.len()),
                    evidence: trend_analysis.trends.iter()
                        .take(3)
                        .map(|t| format!("{}: {:?}", t.metric_name, t.direction))
                        .collect(),
                });
            }
        }
        
        Ok(ResultComplexity {
            overall_score: complexity_score.min(10.0),
            data_volume_score: volume_complexity,
            dimension_score: dimension_complexity,
            insight_score: insight_complexity,
            problem_score: problem_complexity,
            visualization_score: visualization_complexity,
            factors: complexity_factors,
        })
    }
    
    fn count_analysis_dimensions(&self, results: &AnalysisResults) -> usize {
        let mut dimensions = 0;
        
        if results.performance_analysis.is_some() { dimensions += 1; }
        if results.health_analysis.is_some() { dimensions += 1; }
        if results.efficiency_analysis.is_some() { dimensions += 1; }
        if results.trend_analysis.is_some() { dimensions += 1; }
        if results.pattern_analysis.is_some() { dimensions += 1; }
        if results.security_analysis.is_some() { dimensions += 1; }
        if results.predictive_analysis.is_some() { dimensions += 1; }
        if results.comparative_analysis.is_some() { dimensions += 1; }
        
        dimensions
    }
    
    fn assess_visualization_complexity(&self, results: &AnalysisResults) -> f32 {
        let mut complexity = 0.0;
        
        // Time series data complexity
        if let Some(performance) = &results.performance_analysis {
            complexity += performance.throughput.time_series.len() as f32 * 0.001;
            complexity += performance.latency.time_series.len() as f32 * 0.001;
        }
        
        // Multi-dimensional visualization complexity
        if results.comparative_analysis.is_some() {
            complexity += 2.0; // Comparative visualizations are complex
        }
        
        if let Some(trends) = &results.trend_analysis {
            if trends.trends.len() > 3 {
                complexity += 1.5; // Multiple trend visualizations
            }
        }
        
        // Real-time complexity
        if self.live {
            complexity += 2.0; // Real-time updates add complexity
        }
        
        complexity.min(5.0)
    }
}
```

#### 4. Adaptive CLI and TUI Rendering
```rust
// src/analysis/rendering.rs
impl AnalyzeCommand {
    async fn render_cli_analysis(
        &self,
        results: &AnalysisResults,
        context: &ExecutionContext,
    ) -> Result<()> {
        let terminal_width = context.terminal_size.map(|(w, _)| w as usize).unwrap_or(80);
        let supports_color = context.terminal_capabilities.supports_color;
        
        // Header
        self.render_analysis_header(&results.session_info, supports_color);
        
        // Key findings summary
        self.render_key_findings(results, terminal_width, supports_color);
        
        // Performance summary (if available)
        if let Some(performance) = &results.performance_analysis {
            self.render_performance_summary(performance, supports_color);
        }
        
        // Health summary (if available)
        if let Some(health) = &results.health_analysis {
            self.render_health_summary(health, supports_color);
        }
        
        // Insights and recommendations
        self.render_insights_and_recommendations(results, terminal_width, supports_color);
        
        // Overall scores
        self.render_overall_scores(&results.overall_scores, supports_color);
        
        Ok(())
    }
    
    fn render_key_findings(
        &self,
        results: &AnalysisResults,
        terminal_width: usize,
        supports_color: bool,
    ) {
        println!("📊 Key Findings");
        println!();
        
        // Show top insights
        let top_insights = results.insights.iter()
            .take(3)
            .collect::<Vec<_>>();
        
        for (i, insight) in top_insights.iter().enumerate() {
            let priority_icon = match insight.priority {
                InsightPriority::Critical => "🔴",
                InsightPriority::High => "🟠",
                InsightPriority::Medium => "🟡",
                InsightPriority::Low => "🔵",
            };
            
            println!("  {}. {} {}", i + 1, priority_icon, insight.title);
            
            // Wrap description to terminal width
            let wrapped_description = self.wrap_text(&insight.description, terminal_width - 6);
            for line in wrapped_description {
                println!("     {}", line);
            }
            
            if let Some(metric_change) = &insight.metric_change {
                let change_color = if supports_color {
                    if metric_change.is_improvement() { "\x1b[32m" } else { "\x1b[31m" }
                } else { "" };
                let reset_color = if supports_color { "\x1b[0m" } else { "" };
                
                println!("     {}📈 {}: {}{}", 
                    change_color, 
                    metric_change.metric_name, 
                    metric_change.format_change(),
                    reset_color
                );
            }
            
            println!();
        }
    }
    
    fn render_performance_summary(&self, performance: &PerformanceAnalysisResult, supports_color: bool) {
        println!("⚡ Performance Analysis");
        
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
            println!("    Trend: {} {:?}", trend_icon, performance.throughput.trend);
        }
        
        // Latency summary
        println!("  Latency:");
        println!("    P50: {:.1}ms", performance.latency.percentiles.p50);
        println!("    P95: {:.1}ms", performance.latency.percentiles.p95);
        println!("    P99: {:.1}ms", performance.latency.percentiles.p99);
        
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
    
    async fn launch_interactive_analysis(
        &self,
        session: &AnalysisSession,
        results: AnalysisResults,
    ) -> Result<()> {
        println!("📊 Launching interactive analysis...");
        
        // Create analysis context for TUI
        let analysis_context = InteractiveAnalysisContext {
            session: session.clone(),
            results,
            visualization_config: self.create_visualization_config(),
            interaction_state: AnalysisInteractionState::Overview,
        };
        
        // Launch TUI with analysis-specific view
        let tui_app = DoraApp::new_with_context(
            ViewType::AnalysisExplorer {
                session_id: session.session_id,
                analysis_type: self.analysis_type.clone(),
            },
            CliContext::from_analysis_session(self, analysis_context),
        );
        
        tui_app.run().await?;
        Ok(())
    }
    
    fn should_launch_interactive_analysis(
        &self,
        reason: &str,
        complexity: &ResultComplexity,
        default_yes: bool,
    ) -> Result<bool> {
        println!();
        println!("📊 {}", reason);
        
        // Show interactive analysis benefits
        let interactive_benefits = self.get_interactive_analysis_benefits(complexity);
        if !interactive_benefits.is_empty() {
            println!("Interactive analysis provides:");
            for benefit in interactive_benefits {
                println!("  • {}", benefit);
            }
        }
        
        let prompt = if default_yes {
            "Launch interactive analysis? [Y/n]: "
        } else {
            "Launch interactive analysis? [y/N]: "
        };
        
        print!("{}", prompt);
        io::stdout().flush()?;
        
        let mut input = String::new();
        io::stdin().read_line(&mut input)?;
        let input = input.trim().to_lowercase();
        
        Ok(match input.as_str() {
            "" => default_yes,
            "y" | "yes" => true,
            "n" | "no" => false,
            _ => default_yes,
        })
    }
    
    fn get_interactive_analysis_benefits(&self, complexity: &ResultComplexity) -> Vec<String> {
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
        
        if self.live {
            benefits.push("Real-time analysis updates and monitoring".to_string());
        }
        
        if self.predict {
            benefits.push("Interactive prediction modeling and scenarios".to_string());
        }
        
        benefits
    }
}
```

### Why This Approach

**Adaptive Intelligence:**
- Analysis complexity drives interface selection
- Data volume and dimensionality inform visualization needs
- User context shapes analysis depth and presentation

**Comprehensive Analysis:**
- Multi-engine analysis across all system dimensions
- Comparative and predictive capabilities
- Pattern recognition and anomaly detection

**Seamless Workflows:**
- CLI efficiency for quick insights
- TUI power for deep exploration
- Smooth transitions based on analysis complexity

### How to Implement

#### Step 1: Enhanced Command Structure (3 hours)
1. **Implement AnalyzeCommand** with comprehensive options
2. **Add analysis type** and depth configuration
3. **Create analysis session** initialization and management
4. **Add export and formatting** capabilities

#### Step 2: Analysis Engines (8 hours)
1. **Implement PerformanceAnalysisEngine** with detailed metrics analysis
2. **Add TrendAnalysisEngine** for pattern and seasonal analysis
3. **Create HealthAnalysisEngine** for system health assessment
4. **Add EfficiencyAnalysisEngine** for resource optimization analysis

#### Step 3: Result Complexity Analysis (3 hours)
1. **Implement result complexity** calculation algorithms
2. **Add visualization complexity** assessment
3. **Create complexity-based** interface selection logic
4. **Add interactive benefit** explanation system

#### Step 4: Adaptive Rendering (4 hours)
1. **Implement intelligent CLI** rendering with adaptive detail levels
2. **Add interactive analysis** launching and context creation
3. **Create analysis-specific TUI** views and workflows
4. **Add export functionality** for various formats

#### Step 5: Testing and Integration (2 hours)
1. **Add comprehensive unit tests** for all analysis engines
2. **Test complexity calculation** accuracy and performance
3. **Validate interface selection** logic for different scenarios
4. **Test interactive analysis** workflows and visualizations

## 🔗 Dependencies
**Depends On:**
- Issue #001 (Hybrid Command Framework) - CLI structure
- Issue #003 (Interface Selection Engine) - TUI suggestions
- Issue #013 (Complexity Calculation Algorithms) - Complexity analysis
- Issue #014 (Resource Analysis System) - Analysis engines
- Issue #016 (User Preference Handling) - User context

**Blocks:** Other enhanced commands that use similar analysis patterns

## 🧪 Testing Requirements

### Unit Tests
```rust
#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_performance_analysis() {
        let engine = PerformanceAnalysisEngine::new();
        let data = create_test_performance_data();
        
        let result = engine.analyze(&data).await.unwrap();
        
        assert!(result.throughput.current > 0.0);
        assert!(!result.performance_scores.is_empty());
    }
    
    #[test]
    fn test_result_complexity_calculation() {
        let cmd = AnalyzeCommand::default();
        let results = create_complex_analysis_results();
        let session = create_test_analysis_session();
        
        let complexity = cmd.analyze_result_complexity(&results, &session).await.unwrap();
        
        assert!(complexity.overall_score > 5.0);
        assert!(!complexity.factors.is_empty());
    }
    
    #[test]
    fn test_interface_selection_for_analysis() {
        let cmd = AnalyzeCommand {
            analysis_type: AnalysisType::Comprehensive,
            depth: AnalysisDepth::Deep,
            ..Default::default()
        };
        let high_complexity = create_high_complexity_results();
        
        let decision = cmd.apply_analysis_overrides(
            InterfaceDecision::cli_only(),
            &high_complexity
        );
        
        assert!(matches!(decision.strategy, InterfaceStrategy::PromptForTui { .. }));
    }
}
```

## ✅ Definition of Done
- [ ] AnalyzeCommand implemented with comprehensive analysis capabilities
- [ ] Multiple analysis engines provide accurate multi-dimensional analysis
- [ ] Result complexity analysis drives appropriate interface selection
- [ ] CLI rendering adapts to data volume and analysis complexity
- [ ] Interactive analysis provides powerful exploration and visualization
- [ ] Export functionality supports multiple formats and use cases
- [ ] Performance targets met for analysis speed and interface responsiveness
- [ ] Comprehensive unit tests validate analysis accuracy and interface decisions
- [ ] Integration tests confirm end-to-end analysis workflows
- [ ] Manual testing validates analysis quality and user experience

This enhanced analyze interface demonstrates the hybrid CLI's ability to handle complex analytical workloads while providing the most appropriate interface for data exploration and insight discovery.