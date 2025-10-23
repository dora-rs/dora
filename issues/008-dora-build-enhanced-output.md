# Issue #008: Add `dora build` with Enhanced Output

## 📋 Summary
Implement an enhanced `dora build` command that provides comprehensive build feedback, dependency analysis, and smart TUI suggestions for complex build scenarios. This command ensures reliable dataflow preparation with clear progress indication and intelligent optimization suggestions.

## 🎯 Objectives
- Create robust dataflow build process with dependency resolution
- Implement detailed progress feedback for multi-step build operations
- Add smart TUI suggestions for complex build scenarios and optimizations
- Provide clear error handling with actionable recovery suggestions
- Ensure excellent build performance with parallel processing where possible

**Success Metrics:**
- Build process provides clear progress feedback for operations >5 seconds
- Dependency resolution errors include specific fix suggestions
- Complex builds suggest optimization opportunities appropriately
- Build caching reduces rebuild times by 70% for unchanged components
- Error messages are actionable with clear next steps

## 🛠️ Technical Requirements

### What to Build

#### 1. Enhanced Build Command Structure
```rust
// src/cli/commands/build.rs
#[derive(Debug, clap::Args)]
pub struct BuildCommand {
    /// Dataflow configuration file to build
    pub dataflow_path: PathBuf,
    
    /// Force rebuild even if cache is valid
    #[clap(long)]
    pub force: bool,
    
    /// Build for specific target architecture
    #[clap(long)]
    pub target: Option<String>,
    
    /// Build in release mode (optimized)
    #[clap(long)]
    pub release: bool,
    
    /// Number of parallel build jobs
    #[clap(short, long)]
    pub jobs: Option<usize>,
    
    /// Skip dependency validation
    #[clap(long)]
    pub skip_deps: bool,
    
    /// Clean build cache before building
    #[clap(long)]
    pub clean: bool,
    
    /// Verbose build output
    #[clap(short, long)]
    pub verbose: bool,
    
    /// Analyze build performance and suggest optimizations
    #[clap(long)]
    pub analyze: bool,
    
    /// Output build artifacts to specific directory
    #[clap(long)]
    pub output_dir: Option<PathBuf>,
    
    /// Show progress even in non-interactive mode
    #[clap(long)]
    pub progress: bool,
    
    /// Suppress hints and suggestions
    #[clap(long)]
    pub no_hints: bool,
}

impl BuildCommand {
    pub async fn execute(&self, context: &ExecutionContext) -> Result<()> {
        // Load and validate configuration
        let dataflow_config = self.load_and_validate_config().await?;
        let build_plan = self.create_build_plan(&dataflow_config).await?;
        
        // Analyze build complexity for interface decisions
        let complexity_analysis = self.analyze_build_complexity(&build_plan);
        
        // Determine interface strategy
        let interface_selector = InterfaceSelector::new(context.clone(), UserConfig::load()?);
        let decision = interface_selector.select_interface(&Command::Build(self.clone()));
        
        match decision.strategy {
            InterfaceStrategy::CliOnly => {
                self.execute_build_cli(&build_plan, context).await?;
            },
            
            InterfaceStrategy::CliWithHint { hint, tui_command } => {
                self.execute_build_cli(&build_plan, context).await?;
                if !self.no_hints && complexity_analysis.should_suggest_optimizations() {
                    self.show_optimization_hint(&hint, &tui_command);
                }
            },
            
            InterfaceStrategy::PromptForTui { reason, default_yes } => {
                if complexity_analysis.is_complex() {
                    if !self.no_hints && self.should_prompt_for_interactive(&reason, default_yes)? {
                        self.execute_build_interactive(&build_plan).await?;
                        return Ok(());
                    }
                }
                self.execute_build_cli(&build_plan, context).await?;
            },
            
            InterfaceStrategy::AutoLaunchTui { reason, show_cli_first } => {
                if show_cli_first {
                    println!("🚀 {}", reason);
                    println!("Launching interactive build monitor...");
                }
                self.execute_build_interactive(&build_plan).await?;
            },
        }
        
        // Post-build analysis and suggestions
        if self.analyze {
            self.perform_build_analysis(&build_plan).await?;
        }
        
        Ok(())
    }
}
```

#### 2. Build Planning and Dependency Resolution
```rust
#[derive(Debug)]
pub struct BuildPlan {
    pub target_config: DataflowConfig,
    pub build_targets: Vec<BuildTarget>,
    pub dependency_graph: DependencyGraph,
    pub build_order: Vec<BuildStep>,
    pub cache_status: CacheAnalysis,
    pub estimated_duration: Duration,
}

#[derive(Debug, Clone)]
pub struct BuildTarget {
    pub name: String,
    pub target_type: BuildTargetType,
    pub source_path: PathBuf,
    pub dependencies: Vec<String>,
    pub build_config: BuildConfig,
    pub cache_key: String,
}

#[derive(Debug, Clone)]
pub enum BuildTargetType {
    RustNode,
    PythonNode,
    CppNode,
    OperatorLibrary,
    Configuration,
    DataFormat,
}

#[derive(Debug)]
pub struct BuildComplexityAnalysis {
    pub total_targets: usize,
    pub estimated_duration: Duration,
    pub dependency_complexity: u32,
    pub cache_hit_rate: f32,
    pub optimization_opportunities: Vec<OptimizationOpportunity>,
    pub parallel_potential: u32,
}

impl BuildComplexityAnalysis {
    pub fn is_complex(&self) -> bool {
        self.total_targets > 5 || 
        self.estimated_duration > Duration::from_secs(30) ||
        self.dependency_complexity > 10
    }
    
    pub fn should_suggest_optimizations(&self) -> bool {
        !self.optimization_opportunities.is_empty() ||
        self.cache_hit_rate < 0.5 ||
        self.parallel_potential > 2
    }
}

impl BuildCommand {
    async fn create_build_plan(&self, config: &DataflowConfig) -> Result<BuildPlan> {
        let mut build_targets = Vec::new();
        let mut dependency_graph = DependencyGraph::new();
        
        // Analyze each node for build requirements
        for node in &config.nodes {
            let build_target = self.analyze_node_build_requirements(node).await?;
            dependency_graph.add_node(&build_target.name, &build_target.dependencies);
            build_targets.push(build_target);
        }
        
        // Resolve build order based on dependencies
        let build_order = dependency_graph.topological_sort()?;
        
        // Analyze cache status
        let cache_analysis = self.analyze_cache_status(&build_targets).await?;
        
        // Estimate build duration
        let estimated_duration = self.estimate_build_duration(&build_targets, &cache_analysis);
        
        Ok(BuildPlan {
            target_config: config.clone(),
            build_targets,
            dependency_graph,
            build_order,
            cache_status: cache_analysis,
            estimated_duration,
        })
    }
    
    async fn analyze_build_complexity(&self, plan: &BuildPlan) -> BuildComplexityAnalysis {
        let total_targets = plan.build_targets.len();
        let estimated_duration = plan.estimated_duration;
        
        // Calculate dependency complexity (nodes with many dependencies)
        let dependency_complexity = plan.dependency_graph.complexity_score();
        
        // Analyze cache efficiency
        let cache_hits = plan.cache_status.cache_hits;
        let total_builds = plan.build_targets.len();
        let cache_hit_rate = if total_builds > 0 {
            cache_hits as f32 / total_builds as f32
        } else {
            1.0
        };
        
        // Identify optimization opportunities
        let optimization_opportunities = self.identify_optimization_opportunities(plan);
        
        // Calculate parallel build potential
        let parallel_potential = plan.dependency_graph.max_parallel_jobs();
        
        BuildComplexityAnalysis {
            total_targets,
            estimated_duration,
            dependency_complexity,
            cache_hit_rate,
            optimization_opportunities,
            parallel_potential,
        }
    }
}
```

#### 3. Enhanced Build Execution
```rust
impl BuildCommand {
    async fn execute_build_cli(&self, plan: &BuildPlan, context: &ExecutionContext) -> Result<()> {
        let progress_style = self.determine_progress_style(context, plan);
        
        match progress_style {
            ProgressStyle::None => self.build_simple(plan).await,
            ProgressStyle::Spinner => self.build_with_spinner(plan).await,
            ProgressStyle::ProgressBar => self.build_with_progress_bar(plan).await,
            ProgressStyle::Detailed => self.build_with_detailed_progress(plan).await,
        }
    }
    
    async fn build_with_detailed_progress(&self, plan: &BuildPlan) -> Result<()> {
        println!("🔧 Building dataflow '{}'...", plan.target_config.name);
        
        // Phase 1: Dependency Resolution
        let deps_spinner = self.create_spinner("Resolving dependencies...");
        let resolved_deps = self.resolve_dependencies(plan).await?;
        deps_spinner.finish_with_message("✅ Dependencies resolved");
        
        // Phase 2: Cache Analysis
        if !self.force {
            let cache_spinner = self.create_spinner("Analyzing build cache...");
            let cache_plan = self.create_cache_plan(plan).await?;
            cache_spinner.finish_with_message(format!(
                "✅ Cache analysis complete ({} cached, {} rebuild needed)",
                cache_plan.cache_hits,
                cache_plan.rebuilds_needed
            ));
        }
        
        // Phase 3: Parallel Build Execution
        let total_targets = plan.build_targets.len();
        let build_progress = self.create_progress_bar(total_targets, "Building targets");
        
        let parallel_executor = ParallelBuildExecutor::new(
            self.jobs.unwrap_or_else(|| num_cpus::get()),
            self.verbose
        );
        
        let mut completed = 0;
        let mut build_results = Vec::new();
        
        for build_step in &plan.build_order {
            let build_result = match build_step {
                BuildStep::Single(target) => {
                    build_progress.set_message(format!("Building {}", target.name));
                    parallel_executor.build_target(target).await?
                },
                BuildStep::Parallel(targets) => {
                    build_progress.set_message(format!("Building {} targets in parallel", targets.len()));
                    parallel_executor.build_targets_parallel(targets).await?
                },
            };
            
            build_results.extend(build_result);
            completed += 1;
            build_progress.set_position(completed);
        }
        
        build_progress.finish_with_message("✅ All targets built successfully");
        
        // Phase 4: Validation and Packaging
        let validation_spinner = self.create_spinner("Validating build artifacts...");
        self.validate_build_artifacts(&build_results).await?;
        validation_spinner.finish_with_message("✅ Build artifacts validated");
        
        if let Some(output_dir) = &self.output_dir {
            let package_spinner = self.create_spinner("Packaging build artifacts...");
            self.package_artifacts(&build_results, output_dir).await?;
            package_spinner.finish_with_message("✅ Artifacts packaged");
        }
        
        self.show_build_summary(&build_results, plan);
        Ok(())
    }
    
    fn show_build_summary(&self, results: &[BuildResult], plan: &BuildPlan) {
        println!();
        println!("🎉 Build completed successfully!");
        println!("   Targets built: {}", results.len());
        println!("   Total time: {}", format_duration(plan.estimated_duration));
        
        // Show cache efficiency
        let cache_hits = results.iter().filter(|r| r.from_cache).count();
        let cache_rate = cache_hits as f32 / results.len() as f32 * 100.0;
        println!("   Cache hit rate: {:.1}%", cache_rate);
        
        // Show build warnings if any
        let warnings: Vec<_> = results.iter()
            .flat_map(|r| &r.warnings)
            .collect();
        
        if !warnings.is_empty() {
            println!("\n⚠️ Build warnings:");
            for warning in warnings.iter().take(5) {
                println!("   • {}", warning);
            }
            if warnings.len() > 5 {
                println!("   ... and {} more", warnings.len() - 5);
            }
        }
        
        // Show optimization suggestions
        if self.analyze {
            self.show_optimization_suggestions(results, plan);
        }
    }
}
```

#### 4. Build Optimization and Analysis
```rust
#[derive(Debug)]
pub struct OptimizationOpportunity {
    pub opportunity_type: OptimizationType,
    pub description: String,
    pub potential_savings: Duration,
    pub implementation_effort: ImplementationEffort,
}

#[derive(Debug)]
pub enum OptimizationType {
    ParallelBuilds,
    CachingImprovements,
    DependencyOptimization,
    BuildToolUpgrade,
    ResourceAllocation,
}

impl BuildCommand {
    fn identify_optimization_opportunities(&self, plan: &BuildPlan) -> Vec<OptimizationOpportunity> {
        let mut opportunities = Vec::new();
        
        // Check for parallel build opportunities
        if plan.dependency_graph.max_parallel_jobs() > 1 && self.jobs.map_or(true, |j| j == 1) {
            opportunities.push(OptimizationOpportunity {
                opportunity_type: OptimizationType::ParallelBuilds,
                description: format!(
                    "Build could run up to {} jobs in parallel. Use --jobs {} for faster builds",
                    plan.dependency_graph.max_parallel_jobs(),
                    plan.dependency_graph.max_parallel_jobs()
                ),
                potential_savings: plan.estimated_duration / 2,
                implementation_effort: ImplementationEffort::Trivial,
            });
        }
        
        // Check for caching improvements
        if plan.cache_status.cache_hit_rate < 0.5 {
            opportunities.push(OptimizationOpportunity {
                opportunity_type: OptimizationType::CachingImprovements,
                description: "Low cache hit rate detected. Consider build cache optimization".to_string(),
                potential_savings: plan.estimated_duration / 3,
                implementation_effort: ImplementationEffort::Medium,
            });
        }
        
        // Check for dependency optimization
        let circular_deps = plan.dependency_graph.find_circular_dependencies();
        if !circular_deps.is_empty() {
            opportunities.push(OptimizationOpportunity {
                opportunity_type: OptimizationType::DependencyOptimization,
                description: format!("Circular dependencies detected: {:?}", circular_deps),
                potential_savings: Duration::from_secs(10),
                implementation_effort: ImplementationEffort::High,
            });
        }
        
        opportunities
    }
    
    fn show_optimization_suggestions(&self, results: &[BuildResult], plan: &BuildPlan) {
        let opportunities = self.identify_optimization_opportunities(plan);
        
        if opportunities.is_empty() {
            println!("\n✨ Build is well optimized - no obvious improvements detected");
            return;
        }
        
        println!("\n💡 Build optimization opportunities:");
        for (i, opp) in opportunities.iter().enumerate() {
            println!("   {}. {} (saves ~{})", 
                    i + 1, 
                    opp.description, 
                    format_duration(opp.potential_savings));
        }
        
        println!("\n🔍 For detailed optimization analysis, try:");
        println!("   dora ui build-analyzer {}", self.dataflow_path.display());
    }
    
    async fn execute_build_interactive(&self, plan: &BuildPlan) -> Result<()> {
        println!("🚀 Launching interactive build monitor...");
        
        // Create TUI build monitor with current command context
        let build_context = BuildMonitorContext {
            build_plan: plan.clone(),
            build_options: BuildOptions {
                force: self.force,
                parallel_jobs: self.jobs.unwrap_or_else(|| num_cpus::get()),
                verbose: self.verbose,
                release: self.release,
            },
        };
        
        let tui_app = DoraApp::new_with_context(
            ViewType::BuildMonitor {
                dataflow_path: self.dataflow_path.clone()
            },
            CliContext::from_build_command(self, build_context),
        );
        
        tui_app.run().await?;
        Ok(())
    }
    
    fn should_prompt_for_interactive(&self, reason: &str, default_yes: bool) -> Result<bool> {
        println!();
        println!("🔍 {}", reason);
        println!("Interactive build monitor provides:");
        println!("  • Real-time progress visualization");
        println!("  • Parallel build coordination display");
        println!("  • Build optimization recommendations");
        println!("  • Dependency graph visualization");
        
        let prompt = if default_yes {
            "Launch interactive build monitor? [Y/n]: "
        } else {
            "Launch interactive build monitor? [y/N]: "
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
}

// Integration with interface selector for smart suggestions
impl InterfaceSelector {
    fn generate_build_hint(&self, analysis: &BuildComplexityAnalysis) -> Option<String> {
        if analysis.optimization_opportunities.len() > 2 {
            Some(format!(
                "Multiple optimization opportunities detected. Interactive build analysis available"
            ))
        } else if analysis.estimated_duration > Duration::from_secs(60) {
            Some(format!(
                "Long build detected (~{}). Monitor progress interactively",
                format_duration(analysis.estimated_duration)
            ))
        } else if analysis.parallel_potential > 2 {
            Some(format!(
                "Build could be parallelized ({} jobs possible). Interactive monitor shows coordination",
                analysis.parallel_potential
            ))
        } else {
            None
        }
    }
}
```

### Why This Approach

**Comprehensive Build Process:**
- Proper dependency resolution and build ordering
- Intelligent caching for faster rebuilds
- Parallel execution where dependencies allow
- Clear progress feedback for complex builds

**Smart Optimization:**
- Automatic detection of optimization opportunities
- Performance analysis and recommendations
- Build cache efficiency monitoring
- Parallel build potential analysis

**Developer Experience:**
- Clear error messages with actionable suggestions
- Progress feedback that matches build complexity
- Interactive monitoring for complex scenarios
- Build artifact validation and packaging

### How to Implement

#### Step 1: Command Structure and Planning (4 hours)
1. **Define BuildCommand struct** with all CLI options
2. **Implement configuration loading** and validation
3. **Create build planning** and dependency resolution
4. **Add build complexity analysis**

#### Step 2: Build Execution Engine (5 hours)
1. **Implement parallel build executor** with job coordination
2. **Add build caching** and cache analysis
3. **Create progress feedback** system
4. **Add build artifact validation**

#### Step 3: Optimization Analysis (3 hours)
1. **Implement optimization opportunity** detection
2. **Add build performance** analysis
3. **Create optimization suggestion** system
4. **Add build summary** and reporting

#### Step 4: Interactive Build Monitor (2 hours)
1. **Add TUI build monitor** integration
2. **Implement build context** passing
3. **Create user prompting** for complex builds
4. **Add hint generation** system

#### Step 5: Testing and Polish (2 hours)
1. **Add comprehensive unit tests** for all functionality
2. **Test with various build scenarios** (simple, complex, failing)
3. **Validate performance** and caching efficiency
4. **Test optimization detection** accuracy

## 🔗 Dependencies
**Depends On:**
- Issue #001 (Hybrid Command Framework) - CLI structure
- Issue #002 (Execution Context Detection) - Progress style selection
- Issue #003 (Interface Selection Engine) - TUI suggestions
- Issue #009 (TUI Launcher Framework) - Interactive build monitor

**Blocks:** Build monitoring TUI view (future issue)

## 🧪 Testing Requirements

### Unit Tests
```rust
#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_build_plan_creation() {
        let config = create_test_dataflow_config();
        let build_plan = BuildCommand::default().create_build_plan(&config).await.unwrap();
        
        assert!(!build_plan.build_targets.is_empty());
        assert!(!build_plan.build_order.is_empty());
    }
    
    #[test]
    fn test_dependency_resolution() {
        let mut graph = DependencyGraph::new();
        graph.add_node("A", &["B", "C"]);
        graph.add_node("B", &["C"]);
        graph.add_node("C", &[]);
        
        let order = graph.topological_sort().unwrap();
        
        // C should come before B, B before A
        let c_pos = order.iter().position(|x| x.name == "C").unwrap();
        let b_pos = order.iter().position(|x| x.name == "B").unwrap();
        let a_pos = order.iter().position(|x| x.name == "A").unwrap();
        
        assert!(c_pos < b_pos);
        assert!(b_pos < a_pos);
    }
    
    #[test]
    fn test_optimization_detection() {
        let plan = create_complex_build_plan();
        let analysis = BuildCommand::default().analyze_build_complexity(&plan).await;
        
        assert!(analysis.is_complex());
        assert!(analysis.should_suggest_optimizations());
        assert!(!analysis.optimization_opportunities.is_empty());
    }
}
```

## ✅ Definition of Done
- [ ] BuildCommand implemented with all CLI options and build planning
- [ ] Dependency resolution works correctly with complex dependency graphs
- [ ] Build caching reduces rebuild times significantly
- [ ] Parallel build execution coordinates properly across multiple jobs
- [ ] Progress feedback provides appropriate detail based on build complexity
- [ ] Optimization analysis detects real improvement opportunities
- [ ] Interactive build monitor integration works seamlessly
- [ ] Error handling provides actionable recovery suggestions
- [ ] Performance targets met for build execution and caching
- [ ] Comprehensive unit tests cover all functionality
- [ ] Integration tests validate end-to-end build workflows
- [ ] Manual testing confirms user experience quality across build scenarios

This enhanced build command provides comprehensive build management that scales from simple single-node builds to complex multi-target builds with intelligent optimization suggestions and interactive monitoring capabilities.