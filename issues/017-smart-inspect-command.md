# Issue #017: Build Smart `dora inspect` Command

## 📋 Summary
Implement an intelligent `dora inspect` command that provides comprehensive resource inspection with smart TUI suggestions based on complexity analysis, real-time data visualization, and context-aware information presentation. This command demonstrates the full power of the hybrid CLI approach for detailed system analysis.

## 🎯 Objectives
- Create comprehensive resource inspection across dataflows, nodes, and system components
- Implement intelligent interface selection based on resource complexity and user context
- Add real-time data visualization and interactive exploration capabilities
- Provide actionable insights and optimization recommendations
- Enable seamless transitions between CLI overview and detailed TUI analysis

**Success Metrics:**
- Inspection completes in <200ms for simple resources, <2s for complex analyses
- TUI suggestions appear appropriately based on resource complexity (85% user approval)
- Interactive features reduce troubleshooting time by 40% compared to static output
- Resource analysis accuracy exceeds 90% for performance issue detection
- User preference learning improves suggestion quality by 25% after 50 interactions

## 🛠️ Technical Requirements

### What to Build

#### 1. Enhanced Inspect Command Structure
```rust
// src/cli/commands/inspect.rs
#[derive(Debug, clap::Args)]
pub struct InspectCommand {
    /// Resource to inspect (dataflow, node, system, recording, etc.)
    pub target: String,
    
    /// Resource type (auto-detected if not specified)
    #[clap(long, value_enum)]
    pub resource_type: Option<ResourceType>,
    
    /// Enable live monitoring mode with real-time updates
    #[clap(long)]
    pub live: bool,
    
    /// Inspection depth level (1=basic, 2=detailed, 3=comprehensive)
    #[clap(long, default_value = "2")]
    pub depth: u8,
    
    /// Focus on specific aspect
    #[clap(long, value_enum)]
    pub focus: Option<InspectionFocus>,
    
    /// Time window for historical analysis
    #[clap(long, default_value = "1h")]
    pub window: String,
    
    /// Include performance analysis
    #[clap(long)]
    pub performance: bool,
    
    /// Include dependency analysis
    #[clap(long)]
    pub dependencies: bool,
    
    /// Include error analysis
    #[clap(long)]
    pub errors: bool,
    
    /// Output format for CLI mode
    #[clap(long, value_enum)]
    pub format: Option<OutputFormat>,
    
    /// Force TUI mode
    #[clap(long)]
    pub tui: bool,
    
    /// Force CLI text output
    #[clap(long)]
    pub text: bool,
    
    /// Export analysis results to file
    #[clap(long)]
    pub export: Option<PathBuf>,
    
    /// Suppress hints and suggestions
    #[clap(long)]
    pub no_hints: bool,
}

#[derive(Debug, Clone, clap::ValueEnum)]
pub enum ResourceType {
    Dataflow,
    Node,
    System,
    Recording,
    Network,
    Storage,
    Process,
}

#[derive(Debug, Clone, clap::ValueEnum)]
pub enum InspectionFocus {
    Performance,
    Errors,
    Dependencies,
    Resource,
    Security,
    Configuration,
    Metrics,
}

impl InspectCommand {
    pub async fn execute(&self, context: &ExecutionContext) -> Result<()> {
        // Resolve target resource
        let resolved_resource = self.resolve_target_resource().await?;
        
        // Perform comprehensive resource analysis
        let analysis_result = self.analyze_resource(&resolved_resource, context).await?;
        
        // Determine interface strategy based on complexity and user preferences
        let interface_selector = InterfaceSelector::new(context.clone(), UserConfig::load()?);
        let decision = interface_selector.select_interface(&Command::Inspect(self.clone()));
        
        // Apply user preference overrides
        let final_decision = self.apply_explicit_overrides(decision);
        
        match final_decision.strategy {
            InterfaceStrategy::CliOnly => {
                self.render_cli_inspection(&analysis_result, context).await?;
            },
            
            InterfaceStrategy::CliWithHint { hint, tui_command } => {
                self.render_cli_inspection(&analysis_result, context).await?;
                if !self.no_hints {
                    self.show_inspection_hint(&hint, &tui_command, &analysis_result);
                }
            },
            
            InterfaceStrategy::PromptForTui { reason, default_yes } => {
                // Show CLI overview first
                self.render_cli_overview(&analysis_result, context).await?;
                
                if self.should_prompt_for_interactive(&reason, &analysis_result, default_yes)? {
                    self.launch_interactive_inspector(&resolved_resource, analysis_result).await?;
                } else {
                    self.render_detailed_cli_analysis(&analysis_result, context).await?;
                }
            },
            
            InterfaceStrategy::AutoLaunchTui { reason, show_cli_first } => {
                if show_cli_first {
                    self.render_cli_overview(&analysis_result, context).await?;
                    println!("\n🚀 {}", reason);
                }
                self.launch_interactive_inspector(&resolved_resource, analysis_result).await?;
            },
        }
        
        // Export results if requested
        if let Some(export_path) = &self.export {
            self.export_analysis_results(&analysis_result, export_path).await?;
        }
        
        Ok(())
    }
}
```

#### 2. Comprehensive Resource Analysis Engine
```rust
// src/inspection/analyzer.rs
#[derive(Debug)]
pub struct ResourceAnalyzer {
    complexity_analyzer: ComplexityAnalysisEngine,
    performance_analyzer: PerformanceAnalyzer,
    dependency_analyzer: DependencyAnalyzer,
    error_analyzer: ErrorAnalyzer,
    security_analyzer: SecurityAnalyzer,
    recommendation_engine: RecommendationEngine,
}

#[derive(Debug, Clone)]
pub struct InspectionResult {
    pub resource: ResolvedResource,
    pub timestamp: DateTime<Utc>,
    pub analysis_depth: u8,
    pub complexity_analysis: ComplexityResult,
    pub performance_analysis: PerformanceAnalysis,
    pub dependency_analysis: DependencyAnalysis,
    pub error_analysis: ErrorAnalysis,
    pub security_analysis: SecurityAnalysis,
    pub recommendations: Vec<Recommendation>,
    pub health_score: HealthScore,
    pub suggested_actions: Vec<SuggestedAction>,
}

#[derive(Debug, Clone)]
pub struct ResolvedResource {
    pub resource_type: ResourceType,
    pub identifier: String,
    pub metadata: ResourceMetadata,
    pub current_state: ResourceState,
    pub configuration: ResourceConfiguration,
    pub relationships: Vec<ResourceRelationship>,
}

impl ResourceAnalyzer {
    pub async fn analyze_resource(
        &mut self,
        resource: &ResolvedResource,
        depth: u8,
        focus: Option<InspectionFocus>,
        context: &ExecutionContext,
    ) -> Result<InspectionResult> {
        let start_time = Instant::now();
        
        // Base complexity analysis
        let complexity_request = ComplexityAnalysisRequest {
            command: Command::Inspect(InspectCommand {
                target: resource.identifier.clone(),
                resource_type: Some(resource.resource_type.clone()),
                depth,
                live: false,
                ..Default::default()
            }),
            context: context.clone(),
            target_resource: Some(ResourceTarget::from_resolved(resource)),
            system_state: SystemState::current().await?,
            user_expertise: UserExpertiseLevel::from_context(context),
        };
        
        let complexity_analysis = self.complexity_analyzer
            .analyze_complexity(&complexity_request).await?;
        
        // Conditional detailed analysis based on depth and focus
        let performance_analysis = if depth >= 2 || matches!(focus, Some(InspectionFocus::Performance)) {
            self.performance_analyzer.analyze(resource).await?
        } else {
            PerformanceAnalysis::basic()
        };
        
        let dependency_analysis = if depth >= 2 || matches!(focus, Some(InspectionFocus::Dependencies)) {
            self.dependency_analyzer.analyze(resource).await?
        } else {
            DependencyAnalysis::basic()
        };
        
        let error_analysis = if depth >= 1 || matches!(focus, Some(InspectionFocus::Errors)) {
            self.error_analyzer.analyze(resource).await?
        } else {
            ErrorAnalysis::basic()
        };
        
        let security_analysis = if depth >= 3 || matches!(focus, Some(InspectionFocus::Security)) {
            self.security_analyzer.analyze(resource).await?
        } else {
            SecurityAnalysis::basic()
        };
        
        // Calculate overall health score
        let health_score = self.calculate_health_score(
            &complexity_analysis,
            &performance_analysis,
            &error_analysis,
        );
        
        // Generate recommendations
        let recommendations = self.recommendation_engine.generate_recommendations(
            resource,
            &complexity_analysis,
            &performance_analysis,
            &dependency_analysis,
            &error_analysis,
        ).await?;
        
        // Generate suggested actions
        let suggested_actions = self.generate_suggested_actions(
            resource,
            &health_score,
            &recommendations,
        );
        
        let analysis_duration = start_time.elapsed();
        
        Ok(InspectionResult {
            resource: resource.clone(),
            timestamp: Utc::now(),
            analysis_depth: depth,
            complexity_analysis,
            performance_analysis,
            dependency_analysis,
            error_analysis,
            security_analysis,
            recommendations,
            health_score,
            suggested_actions,
        })
    }
    
    async fn analyze_dataflow_resource(&self, dataflow_name: &str) -> Result<ResolvedResource> {
        let daemon_client = DaemonClient::connect().await?;
        
        // Get dataflow information
        let dataflow_info = daemon_client.get_dataflow_info(dataflow_name).await?;
        let nodes = daemon_client.get_dataflow_nodes(dataflow_name).await?;
        let metrics = daemon_client.get_dataflow_metrics(dataflow_name).await?;
        let config = daemon_client.get_dataflow_configuration(dataflow_name).await?;
        
        // Build relationships
        let mut relationships = Vec::new();
        
        // Node relationships
        for node in &nodes {
            relationships.push(ResourceRelationship {
                relationship_type: RelationshipType::Contains,
                target_resource: ResourceReference {
                    resource_type: ResourceType::Node,
                    identifier: format!("{}.{}", dataflow_name, node.name),
                },
                metadata: HashMap::from([
                    ("status".to_string(), node.status.to_string()),
                    ("role".to_string(), node.role.clone().unwrap_or_default()),
                ]),
            });
        }
        
        // Dependency relationships
        for dependency in &config.dependencies {
            relationships.push(ResourceRelationship {
                relationship_type: RelationshipType::DependsOn,
                target_resource: ResourceReference {
                    resource_type: ResourceType::from_dependency_type(&dependency.dep_type),
                    identifier: dependency.name.clone(),
                },
                metadata: HashMap::from([
                    ("version".to_string(), dependency.version.clone().unwrap_or_default()),
                    ("required".to_string(), dependency.required.to_string()),
                ]),
            });
        }
        
        Ok(ResolvedResource {
            resource_type: ResourceType::Dataflow,
            identifier: dataflow_name.to_string(),
            metadata: ResourceMetadata {
                name: dataflow_info.name,
                created_at: dataflow_info.created_at,
                modified_at: dataflow_info.modified_at,
                version: dataflow_info.version,
                tags: dataflow_info.tags,
                description: dataflow_info.description,
            },
            current_state: ResourceState::Dataflow(DataflowState {
                status: dataflow_info.status,
                uptime: dataflow_info.uptime,
                node_count: nodes.len(),
                healthy_nodes: nodes.iter().filter(|n| n.is_healthy()).count(),
                error_count: nodes.iter().map(|n| n.error_count).sum(),
                throughput: metrics.total_throughput,
                latency_p95: metrics.latency_p95,
            }),
            configuration: ResourceConfiguration::Dataflow(config),
            relationships,
        })
    }
}
```

#### 3. Intelligent CLI Output Rendering
```rust
// src/inspection/cli_renderer.rs
impl InspectCommand {
    async fn render_cli_inspection(
        &self,
        result: &InspectionResult,
        context: &ExecutionContext,
    ) -> Result<()> {
        let format = self.format.unwrap_or_else(|| {
            if context.is_tty && !context.is_piped {
                OutputFormat::Table
            } else {
                OutputFormat::Minimal
            }
        });
        
        match format {
            OutputFormat::Table => self.render_table_format(result, context).await?,
            OutputFormat::Json => self.render_json_format(result)?,
            OutputFormat::Yaml => self.render_yaml_format(result)?,
            OutputFormat::Minimal => self.render_minimal_format(result)?,
            _ => self.render_table_format(result, context).await?,
        }
        
        Ok(())
    }
    
    async fn render_table_format(
        &self,
        result: &InspectionResult,
        context: &ExecutionContext,
    ) -> Result<()> {
        let supports_color = context.terminal_capabilities.supports_color;
        let terminal_width = context.terminal_size.map(|(w, _)| w as usize).unwrap_or(80);
        
        // Header
        self.render_resource_header(&result.resource, supports_color);
        
        // Health Score and Status
        self.render_health_overview(&result.health_score, supports_color);
        
        // Key Metrics (always shown)
        self.render_key_metrics(result, supports_color);
        
        // Conditional sections based on depth and findings
        if self.depth >= 2 || !result.performance_analysis.issues.is_empty() {
            self.render_performance_section(&result.performance_analysis, supports_color);
        }
        
        if self.depth >= 2 || !result.error_analysis.recent_errors.is_empty() {
            self.render_error_section(&result.error_analysis, supports_color);
        }
        
        if self.dependencies || !result.dependency_analysis.critical_dependencies.is_empty() {
            self.render_dependency_section(&result.dependency_analysis, supports_color);
        }
        
        // Recommendations (if any)
        if !result.recommendations.is_empty() {
            self.render_recommendations_section(&result.recommendations, supports_color);
        }
        
        // Suggested Actions
        if !result.suggested_actions.is_empty() {
            self.render_suggested_actions(&result.suggested_actions, supports_color);
        }
        
        Ok(())
    }
    
    fn render_resource_header(&self, resource: &ResolvedResource, supports_color: bool) {
        let resource_icon = match resource.resource_type {
            ResourceType::Dataflow => "📊",
            ResourceType::Node => "🔧",
            ResourceType::System => "💻",
            ResourceType::Recording => "📹",
            _ => "📋",
        };
        
        println!("{} Resource Inspection: {}", resource_icon, resource.identifier);
        println!("Type: {} | Created: {}", 
                resource.resource_type, 
                resource.metadata.created_at.format("%Y-%m-%d %H:%M:%S"));
        
        if let Some(description) = &resource.metadata.description {
            println!("Description: {}", description);
        }
        
        println!(); // Separator
    }
    
    fn render_health_overview(&self, health_score: &HealthScore, supports_color: bool) {
        let (health_icon, health_color) = match health_score.overall_score {
            score if score >= 90.0 => ("✅", "\x1b[32m"), // Green
            score if score >= 70.0 => ("⚠️", "\x1b[33m"),  // Yellow
            score if score >= 50.0 => ("🔶", "\x1b[31m"), // Orange
            _ => ("❌", "\x1b[31m"), // Red
        };
        
        if supports_color {
            println!("{}Health Score: {} {:.1}/100 - {}\x1b[0m", 
                    health_color, health_icon, health_score.overall_score, health_score.status);
        } else {
            println!("Health Score: {} {:.1}/100 - {}", 
                    health_icon, health_score.overall_score, health_score.status);
        }
        
        // Health breakdown
        if !health_score.component_scores.is_empty() {
            println!("  Component Health:");
            for (component, score) in &health_score.component_scores {
                let component_icon = if *score >= 80.0 { "✅" } else if *score >= 60.0 { "⚠️" } else { "❌" };
                println!("    {}: {} {:.1}", component, component_icon, score);
            }
        }
        
        println!(); // Separator
    }
    
    fn render_key_metrics(&self, result: &InspectionResult, supports_color: bool) {
        println!("📈 Key Metrics");
        
        match &result.resource.current_state {
            ResourceState::Dataflow(state) => {
                println!("  Status: {}", self.format_status(&state.status, supports_color));
                println!("  Uptime: {}", format_duration(state.uptime));
                println!("  Nodes: {}/{} healthy", state.healthy_nodes, state.node_count);
                
                if state.error_count > 0 {
                    let error_color = if supports_color { "\x1b[31m" } else { "" };
                    let reset_color = if supports_color { "\x1b[0m" } else { "" };
                    println!("  {}Errors: {}{}", error_color, state.error_count, reset_color);
                }
                
                println!("  Throughput: {:.2} msg/s", state.throughput);
                println!("  Latency (P95): {:.1}ms", state.latency_p95);
            },
            
            ResourceState::Node(state) => {
                println!("  Status: {}", self.format_status(&state.status, supports_color));
                println!("  CPU Usage: {:.1}%", state.cpu_usage);
                println!("  Memory: {}", format_memory(state.memory_usage_mb));
                println!("  Message Rate: {:.2}/s", state.message_rate);
                
                if let Some(last_error) = &state.last_error {
                    let error_color = if supports_color { "\x1b[31m" } else { "" };
                    let reset_color = if supports_color { "\x1b[0m" } else { "" };
                    println!("  {}Last Error: {}{}", error_color, last_error, reset_color);
                }
            },
            
            _ => {
                // Generic metrics for other resource types
                println!("  Status: Active");
            }
        }
        
        println!(); // Separator
    }
    
    fn render_performance_section(&self, analysis: &PerformanceAnalysis, supports_color: bool) {
        if analysis.issues.is_empty() && analysis.metrics.is_empty() {
            return;
        }
        
        println!("⚡ Performance Analysis");
        
        // Performance issues
        if !analysis.issues.is_empty() {
            println!("  Issues Detected:");
            for issue in &analysis.issues {
                let severity_icon = match issue.severity {
                    IssueSeverity::Critical => "🔴",
                    IssueSeverity::High => "🟠", 
                    IssueSeverity::Medium => "🟡",
                    IssueSeverity::Low => "🔵",
                };
                println!("    {} {}", severity_icon, issue.description);
                
                if let Some(recommendation) = &issue.recommendation {
                    println!("      💡 {}", recommendation);
                }
            }
        }
        
        // Key performance metrics
        if !analysis.metrics.is_empty() {
            println!("  Metrics:");
            for (metric_name, metric_value) in &analysis.metrics {
                println!("    {}: {}", metric_name, metric_value);
            }
        }
        
        println!(); // Separator
    }
    
    fn render_recommendations_section(&self, recommendations: &[Recommendation], supports_color: bool) {
        if recommendations.is_empty() {
            return;
        }
        
        println!("💡 Recommendations");
        
        for (i, rec) in recommendations.iter().enumerate() {
            let priority_icon = match rec.priority {
                RecommendationPriority::Critical => "🔴",
                RecommendationPriority::High => "🟠",
                RecommendationPriority::Medium => "🟡",
                RecommendationPriority::Low => "🔵",
            };
            
            println!("  {}. {} {}", i + 1, priority_icon, rec.title);
            println!("     {}", rec.description);
            
            if let Some(command) = &rec.suggested_command {
                println!("     💻 Try: {}", command);
            }
        }
        
        println!(); // Separator
    }
}
```

#### 4. Interactive TUI Inspector
```rust
// src/inspection/tui_inspector.rs
impl InspectCommand {
    async fn launch_interactive_inspector(
        &self,
        resource: &ResolvedResource,
        analysis_result: InspectionResult,
    ) -> Result<()> {
        println!("🚀 Launching interactive inspector for '{}'...", resource.identifier);
        
        // Create TUI inspector context
        let inspector_context = InspectorContext {
            resource: resource.clone(),
            analysis_result,
            inspection_options: InspectionOptions {
                live_mode: self.live,
                depth: self.depth,
                focus: self.focus.clone(),
                time_window: self.parse_time_window(&self.window)?,
                auto_refresh: true,
            },
        };
        
        let tui_app = DoraApp::new_with_context(
            ViewType::ResourceInspector {
                resource_id: resource.identifier.clone(),
                resource_type: resource.resource_type.clone(),
            },
            CliContext::from_inspect_command(self, inspector_context),
        );
        
        tui_app.run().await?;
        Ok(())
    }
    
    fn should_prompt_for_interactive(
        &self,
        reason: &str,
        analysis_result: &InspectionResult,
        default_yes: bool,
    ) -> Result<bool> {
        println!();
        println!("🔍 {}", reason);
        
        // Show what interactive mode offers
        let interactive_benefits = self.get_interactive_benefits(analysis_result);
        if !interactive_benefits.is_empty() {
            println!("Interactive inspector provides:");
            for benefit in interactive_benefits {
                println!("  • {}", benefit);
            }
        }
        
        let prompt = if default_yes {
            "Launch interactive inspector? [Y/n]: "
        } else {
            "Launch interactive inspector? [y/N]: "
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
    
    fn get_interactive_benefits(&self, analysis_result: &InspectionResult) -> Vec<String> {
        let mut benefits = Vec::new();
        
        // Always available benefits
        benefits.push("Real-time metrics and live updates".to_string());
        benefits.push("Interactive exploration of dependencies".to_string());
        
        // Conditional benefits based on analysis
        if !analysis_result.performance_analysis.issues.is_empty() {
            benefits.push("Performance timeline and trend analysis".to_string());
        }
        
        if !analysis_result.error_analysis.recent_errors.is_empty() {
            benefits.push("Error correlation and pattern analysis".to_string());
        }
        
        if analysis_result.dependency_analysis.dependency_count > 5 {
            benefits.push("Visual dependency graph and impact analysis".to_string());
        }
        
        if self.live {
            benefits.push("Live monitoring with alerting".to_string());
        }
        
        if analysis_result.complexity_analysis.overall_score > 7.0 {
            benefits.push("Guided troubleshooting and optimization workflow".to_string());
        }
        
        benefits
    }
    
    fn show_inspection_hint(
        &self,
        hint: &str,
        tui_command: &str,
        analysis_result: &InspectionResult,
    ) {
        println!();
        println!("💡 {}", hint);
        println!("   Try: {}", tui_command);
        
        // Add specific suggestions based on analysis
        if !analysis_result.recommendations.is_empty() {
            let high_priority_recs = analysis_result.recommendations.iter()
                .filter(|r| matches!(r.priority, RecommendationPriority::Critical | RecommendationPriority::High))
                .count();
            
            if high_priority_recs > 0 {
                println!("   📋 {} high-priority recommendations available in interactive mode", high_priority_recs);
            }
        }
        
        if analysis_result.health_score.overall_score < 70.0 {
            println!("   🔍 Health issues detected - interactive analysis recommended");
        }
    }
}

// Integration with interface selector for smart suggestions
impl InterfaceSelector {
    fn generate_inspect_hint(&self, analysis: &InspectionResult, resource: &ResolvedResource) -> Option<String> {
        let complexity_score = analysis.complexity_analysis.overall_score;
        let health_score = analysis.health_score.overall_score;
        let issue_count = analysis.performance_analysis.issues.len() + 
                         analysis.error_analysis.recent_errors.len();
        
        match (complexity_score, health_score, issue_count) {
            (complexity, health, _) if complexity > 8.0 && health < 60.0 => Some(
                "Critical issues detected in complex resource. Interactive analysis strongly recommended".to_string()
            ),
            (complexity, _, issues) if complexity > 6.0 && issues > 3 => Some(
                format!("Complex resource with {} issues. TUI provides guided troubleshooting", issues)
            ),
            (_, health, _) if health < 70.0 => Some(
                "Performance issues detected. Interactive inspector provides detailed analysis".to_string()
            ),
            (complexity, _, _) if complexity > 7.0 => Some(
                "Complex resource detected. Interactive explorer recommended for thorough analysis".to_string()
            ),
            (_, _, issues) if issues > 2 => Some(
                format!("{} issues found. Interactive mode provides correlation analysis", issues)
            ),
            _ => None,
        }
    }
}
```

### Why This Approach

**Comprehensive Analysis:**
- Multi-dimensional resource analysis across performance, dependencies, errors, and security
- Context-aware depth control based on resource complexity
- Real-time and historical analysis capabilities

**Intelligent Interface Selection:**
- Smart TUI suggestions based on actual analysis results
- User preference learning and adaptation
- Clear benefits explanation for interface choices

**Actionable Insights:**
- Specific recommendations with suggested commands
- Health scoring with component breakdown
- Suggested actions prioritized by impact

### How to Implement

#### Step 1: Enhanced Command Structure (3 hours)
1. **Implement InspectCommand** with comprehensive options
2. **Add resource type** detection and validation
3. **Create inspection focus** and depth handling
4. **Add explicit override** support

#### Step 2: Resource Analysis Engine (6 hours)
1. **Implement ResourceAnalyzer** with multi-dimensional analysis
2. **Add dataflow, node, and system** resource resolution
3. **Create performance, dependency, and error** analysis
4. **Add recommendation generation** system

#### Step 3: CLI Output Rendering (4 hours)
1. **Implement intelligent CLI** output with multiple formats
2. **Add color support** and terminal width adaptation
3. **Create conditional section** rendering based on findings
4. **Add health score** and metrics visualization

#### Step 4: TUI Integration (2 hours)
1. **Add interactive inspector** launching
2. **Create inspection context** passing to TUI
3. **Implement user prompting** with benefit explanation
4. **Add hint generation** based on analysis results

#### Step 5: Testing and Polish (2 hours)
1. **Add comprehensive unit tests** for all functionality
2. **Test with various resource** types and complexity levels
3. **Validate TUI suggestion** accuracy and appropriateness
4. **Test performance** with large and complex resources

## 🔗 Dependencies
**Depends On:**
- Issue #001 (Hybrid Command Framework) - CLI structure
- Issue #003 (Interface Selection Engine) - TUI suggestions
- Issue #013 (Complexity Calculation Algorithms) - Complexity analysis
- Issue #014 (Resource Analysis System) - Resource monitoring
- Issue #016 (User Preference Handling) - Preference integration

**Blocks:** Other enhanced commands that use similar analysis patterns

## 🧪 Testing Requirements

### Unit Tests
```rust
#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_resource_resolution() {
        let cmd = InspectCommand {
            target: "test-dataflow".to_string(),
            resource_type: Some(ResourceType::Dataflow),
            ..Default::default()
        };
        
        let resource = cmd.resolve_target_resource().await.unwrap();
        assert_eq!(resource.resource_type, ResourceType::Dataflow);
        assert_eq!(resource.identifier, "test-dataflow");
    }
    
    #[test]
    fn test_analysis_depth_levels() {
        let analyzer = ResourceAnalyzer::new();
        let resource = create_test_resource();
        
        let shallow_analysis = analyzer.analyze_resource(&resource, 1, None, &ExecutionContext::default()).await.unwrap();
        let deep_analysis = analyzer.analyze_resource(&resource, 3, None, &ExecutionContext::default()).await.unwrap();
        
        assert!(deep_analysis.security_analysis.findings.len() >= shallow_analysis.security_analysis.findings.len());
    }
    
    #[test]
    fn test_interface_suggestion_logic() {
        let selector = InterfaceSelector::new(ExecutionContext::default(), UserConfig::default());
        let analysis = create_complex_analysis_result();
        let resource = create_test_resource();
        
        let hint = selector.generate_inspect_hint(&analysis, &resource);
        assert!(hint.is_some());
        assert!(hint.unwrap().contains("complex"));
    }
}
```

## ✅ Definition of Done
- [ ] InspectCommand implemented with comprehensive resource analysis
- [ ] Multi-dimensional analysis covers performance, dependencies, errors, and security
- [ ] Intelligent interface selection based on analysis complexity and user context
- [ ] CLI output provides clear, actionable information with appropriate detail levels
- [ ] TUI integration offers compelling benefits explained to users
- [ ] Health scoring and recommendations help users understand resource status
- [ ] Performance targets met for analysis speed and interface responsiveness
- [ ] User preference learning improves suggestion accuracy over time
- [ ] Comprehensive unit tests validate analysis accuracy and interface decisions
- [ ] Integration tests confirm end-to-end inspection workflows
- [ ] Manual testing validates user experience across different resource types

This smart inspect command demonstrates the full power of the hybrid CLI approach, providing intelligent analysis with appropriate interface suggestions based on actual resource complexity and user needs.