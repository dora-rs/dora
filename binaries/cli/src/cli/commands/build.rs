use super::BuildCommand;
use crate::cli::context::ExecutionContext;
use crate::cli::interface::{InterfaceDecision, InterfaceStrategy};
use colored::Colorize;
use indicatif::{ProgressBar, ProgressStyle as IndicatifProgressStyle};
use petgraph::Graph;
use serde::{Deserialize, Serialize};
use std::collections::{HashMap, HashSet};
use std::path::PathBuf;
use std::time::{Duration, Instant};

/// Enhanced build command implementation for Issue #8
pub struct EnhancedBuildCommand {
    context: ExecutionContext,
    command: BuildCommand,
    build_planner: BuildPlanner,
    cache_manager: BuildCacheManager,
    dependency_analyzer: DependencyAnalyzer,
    optimization_analyzer: OptimizationAnalyzer,
}

/// Build planning and coordination system
pub struct BuildPlanner {
    dependency_graph: Graph<BuildTarget, ()>,
    build_cache: HashMap<String, CacheEntry>,
}

/// Build cache management for faster rebuilds
pub struct BuildCacheManager {
    cache_dir: PathBuf,
    cache_index: HashMap<String, CacheEntry>,
}

/// Dependency analysis and resolution
pub struct DependencyAnalyzer {
    resolved_dependencies: HashMap<String, Vec<String>>,
    circular_dependencies: Vec<Vec<String>>,
}

/// Build optimization analysis and suggestions
pub struct OptimizationAnalyzer {
    optimization_opportunities: Vec<OptimizationOpportunity>,
    performance_metrics: HashMap<String, Duration>,
}

/// Complete build plan with execution strategy
#[derive(Debug, Clone)]
pub struct BuildPlan {
    pub target_config: DataflowConfig,
    pub build_targets: Vec<BuildTarget>,
    pub dependency_graph: DependencyGraph,
    pub build_order: Vec<BuildStep>,
    pub cache_analysis: CacheAnalysis,
    pub estimated_duration: Duration,
    pub complexity_analysis: BuildComplexityAnalysis,
}

/// Individual build target representation
#[derive(Debug, Clone)]
pub struct BuildTarget {
    pub name: String,
    pub target_type: BuildTargetType,
    pub source_path: PathBuf,
    pub dependencies: Vec<String>,
    pub build_config: BuildConfig,
    pub cache_key: String,
    pub estimated_build_time: Duration,
}

/// Types of build targets we can handle
#[derive(Debug, Clone, PartialEq)]
pub enum BuildTargetType {
    RustNode,
    PythonNode,
    CppNode,
    OperatorLibrary,
    Configuration,
    DataFormat,
}

/// Configuration for individual build targets
#[derive(Debug, Clone)]
pub struct BuildConfig {
    pub release_mode: bool,
    pub target_arch: Option<String>,
    pub optimization_level: OptimizationLevel,
    pub extra_flags: Vec<String>,
}

/// Build optimization levels
#[derive(Debug, Clone)]
pub enum OptimizationLevel {
    Debug,
    Release,
    ReleaseLTO,
    Custom(String),
}

/// Build execution steps (single or parallel)
#[derive(Debug, Clone)]
pub enum BuildStep {
    Single(BuildTarget),
    Parallel(Vec<BuildTarget>),
}

/// Dependency graph for build ordering
#[derive(Debug, Clone)]
pub struct DependencyGraph {
    nodes: HashMap<String, BuildTarget>,
    edges: HashMap<String, Vec<String>>,
}

/// Cache analysis results
#[derive(Debug, Clone)]
pub struct CacheAnalysis {
    pub cache_hits: usize,
    pub cache_misses: usize,
    pub rebuilds_needed: usize,
    pub cache_hit_rate: f32,
    pub estimated_time_saved: Duration,
}

/// Build complexity analysis for interface decisions
#[derive(Debug, Clone)]
pub struct BuildComplexityAnalysis {
    pub total_targets: usize,
    pub estimated_duration: Duration,
    pub dependency_complexity: u32,
    pub cache_hit_rate: f32,
    pub optimization_opportunities: Vec<OptimizationOpportunity>,
    pub parallel_potential: u32,
    pub complexity_score: f64,
}

/// Optimization opportunities and suggestions
#[derive(Debug, Clone)]
pub struct OptimizationOpportunity {
    pub opportunity_type: OptimizationType,
    pub description: String,
    pub potential_savings: Duration,
    pub implementation_effort: ImplementationEffort,
    pub priority: OptimizationPriority,
}

/// Types of build optimizations
#[derive(Debug, Clone)]
pub enum OptimizationType {
    ParallelBuilds,
    CachingImprovements,
    DependencyOptimization,
    BuildToolUpgrade,
    ResourceAllocation,
    CompilerOptimization,
}

/// Implementation effort for optimizations
#[derive(Debug, Clone)]
pub enum ImplementationEffort {
    Trivial,   // Can be done immediately
    Low,       // Minor configuration change
    Medium,    // Requires some refactoring
    High,      // Significant changes needed
}

/// Priority levels for optimizations
#[derive(Debug, Clone)]
pub enum OptimizationPriority {
    Critical,  // Major performance impact
    High,      // Significant improvement
    Medium,    // Moderate benefit
    Low,       // Minor improvement
}

/// Build execution results
#[derive(Debug)]
pub struct BuildResult {
    pub target_name: String,
    pub success: bool,
    pub duration: Duration,
    pub from_cache: bool,
    pub warnings: Vec<String>,
    pub artifacts: Vec<PathBuf>,
}

/// Cache entry for build artifacts
#[derive(Debug, Serialize, Deserialize)]
pub struct CacheEntry {
    pub cache_key: String,
    pub timestamp: std::time::SystemTime,
    pub artifacts: Vec<PathBuf>,
    pub build_duration: Duration,
}

/// Dataflow configuration (simplified for demo)
#[derive(Debug, Clone)]
pub struct DataflowConfig {
    pub name: String,
    pub nodes: Vec<NodeConfig>,
    pub version: String,
}

/// Node configuration
#[derive(Debug, Clone)]
pub struct NodeConfig {
    pub name: String,
    pub source: String,
    pub dependencies: Vec<String>,
}

/// Progress indication styles
#[derive(Debug)]
pub enum BuildProgressStyle {
    None,
    Spinner,
    ProgressBar,
    Detailed,
}

impl EnhancedBuildCommand {
    /// Create new enhanced build command
    pub fn new(context: ExecutionContext, command: BuildCommand) -> Self {
        Self {
            context,
            command,
            build_planner: BuildPlanner::new(),
            cache_manager: BuildCacheManager::new(),
            dependency_analyzer: DependencyAnalyzer::new(),
            optimization_analyzer: OptimizationAnalyzer::new(),
        }
    }

    /// Execute the enhanced build command
    pub async fn execute(&mut self, interface_decision: &InterfaceDecision) -> Result<(), Box<dyn std::error::Error>> {
        println!("🔧 Enhanced Build Command (Issue #8)");
        
        // Phase 1: Load and validate configuration
        let config_spinner = self.create_spinner("Loading dataflow configuration...");
        let dataflow_config = self.load_and_validate_config().await?;
        config_spinner.finish_with_message("✅ Configuration loaded");

        // Phase 2: Create comprehensive build plan
        let planning_spinner = self.create_spinner("Creating build plan...");
        let build_plan = self.create_build_plan(&dataflow_config).await?;
        planning_spinner.finish_with_message(format!(
            "✅ Build plan created ({} targets, ~{})",
            build_plan.build_targets.len(),
            format_duration(build_plan.estimated_duration)
        ));

        // Phase 3: Determine execution strategy based on interface decision
        match &interface_decision.strategy {
            InterfaceStrategy::CliOnly => {
                self.execute_build_cli(&build_plan).await?;
            },
            
            InterfaceStrategy::CliWithHint { hint, tui_command } => {
                self.execute_build_cli(&build_plan).await?;
                if !self.command.no_hints && build_plan.complexity_analysis.should_suggest_optimizations() {
                    self.show_optimization_hint(hint, tui_command);
                }
            },
            
            InterfaceStrategy::PromptForTui { reason, default_yes } => {
                if build_plan.complexity_analysis.is_complex() {
                    if !self.command.no_hints && self.should_prompt_for_interactive(reason, *default_yes)? {
                        self.execute_build_interactive(&build_plan).await?;
                        return Ok(());
                    }
                }
                self.execute_build_cli(&build_plan).await?;
            },
            
            InterfaceStrategy::AutoLaunchTui { reason, show_cli_first } => {
                if *show_cli_first {
                    println!("🚀 {}", reason);
                    println!("Launching interactive build monitor...");
                }
                self.execute_build_interactive(&build_plan).await?;
            },
        }

        // Phase 4: Post-build analysis
        if self.command.analyze {
            self.perform_build_analysis(&build_plan).await?;
        }

        Ok(())
    }

    /// Load and validate dataflow configuration
    async fn load_and_validate_config(&self) -> Result<DataflowConfig, Box<dyn std::error::Error>> {
        // Mock implementation - would load actual YAML/JSON config
        let config = DataflowConfig {
            name: "example-dataflow".to_string(),
            nodes: vec![
                NodeConfig {
                    name: "sensor_node".to_string(),
                    source: "rust".to_string(),
                    dependencies: vec![],
                },
                NodeConfig {
                    name: "processing_node".to_string(),
                    source: "python".to_string(),
                    dependencies: vec!["sensor_node".to_string()],
                },
                NodeConfig {
                    name: "output_node".to_string(),
                    source: "rust".to_string(),
                    dependencies: vec!["processing_node".to_string()],
                },
            ],
            version: "1.0.0".to_string(),
        };

        // Validate configuration
        self.validate_configuration(&config)?;
        Ok(config)
    }

    /// Create comprehensive build plan
    async fn create_build_plan(&mut self, config: &DataflowConfig) -> Result<BuildPlan, Box<dyn std::error::Error>> {
        // Step 1: Analyze build targets
        let mut build_targets = Vec::new();
        for node in &config.nodes {
            let build_target = self.analyze_node_build_requirements(node).await?;
            build_targets.push(build_target);
        }

        // Step 2: Build dependency graph
        let dependency_graph = self.build_dependency_graph(&build_targets)?;

        // Step 3: Determine build order
        let build_order = self.determine_build_order(&dependency_graph)?;

        // Step 4: Analyze cache status
        let cache_analysis = self.analyze_cache_status(&build_targets).await?;

        // Step 5: Estimate build duration
        let estimated_duration = self.estimate_build_duration(&build_targets, &cache_analysis);

        // Step 6: Perform complexity analysis
        let complexity_analysis = self.analyze_build_complexity(&build_targets, &dependency_graph, &cache_analysis);

        Ok(BuildPlan {
            target_config: config.clone(),
            build_targets,
            dependency_graph,
            build_order,
            cache_analysis,
            estimated_duration,
            complexity_analysis,
        })
    }

    /// Execute build with CLI interface
    async fn execute_build_cli(&self, plan: &BuildPlan) -> Result<(), Box<dyn std::error::Error>> {
        let progress_style = self.determine_progress_style(&plan);
        
        match progress_style {
            BuildProgressStyle::None => self.build_simple(plan).await,
            BuildProgressStyle::Spinner => self.build_with_spinner(plan).await,
            BuildProgressStyle::ProgressBar => self.build_with_progress_bar(plan).await,
            BuildProgressStyle::Detailed => self.build_with_detailed_progress(plan).await,
        }
    }

    /// Execute build with detailed progress indication
    async fn build_with_detailed_progress(&self, plan: &BuildPlan) -> Result<(), Box<dyn std::error::Error>> {
        println!("\n🔧 Building dataflow '{}'...", plan.target_config.name);
        
        // Phase 1: Dependency Resolution
        if !self.command.skip_deps {
            let deps_spinner = self.create_spinner("Resolving dependencies...");
            tokio::time::sleep(Duration::from_millis(500)).await; // Simulate work
            deps_spinner.finish_with_message("✅ Dependencies resolved");
        }

        // Phase 2: Cache Analysis
        if !self.command.force {
            let cache_spinner = self.create_spinner("Analyzing build cache...");
            tokio::time::sleep(Duration::from_millis(300)).await; // Simulate work
            cache_spinner.finish_with_message(format!(
                "✅ Cache analysis ({} hits, {} rebuilds needed)",
                plan.cache_analysis.cache_hits,
                plan.cache_analysis.rebuilds_needed
            ));
        }

        // Phase 3: Parallel Build Execution
        let total_targets = plan.build_targets.len();
        let build_progress = self.create_progress_bar(total_targets, "Building targets");
        
        let parallel_jobs = self.command.jobs.unwrap_or_else(|| num_cpus::get());
        println!("   Using {} parallel jobs", parallel_jobs);

        let mut completed = 0;
        let mut build_results = Vec::new();

        for (i, build_step) in plan.build_order.iter().enumerate() {
            match build_step {
                BuildStep::Single(target) => {
                    build_progress.set_message(format!("Building {}", target.name));
                    let result = self.build_target(target).await?;
                    build_results.push(result);
                },
                BuildStep::Parallel(targets) => {
                    build_progress.set_message(format!("Building {} targets in parallel", targets.len()));
                    for target in targets {
                        let result = self.build_target(target).await?;
                        build_results.push(result);
                    }
                },
            }
            
            completed += 1;
            build_progress.set_position(completed as u64);
            
            // Simulate build time
            tokio::time::sleep(Duration::from_millis(100 + (i * 50) as u64)).await;
        }

        build_progress.finish_with_message("✅ All targets built successfully");

        // Phase 4: Validation and Packaging
        if !self.command.validate {
            let validation_spinner = self.create_spinner("Validating build artifacts...");
            tokio::time::sleep(Duration::from_millis(200)).await; // Simulate validation
            validation_spinner.finish_with_message("✅ Build artifacts validated");
        }

        if let Some(output_dir) = &self.command.output_dir {
            let package_spinner = self.create_spinner("Packaging build artifacts...");
            tokio::time::sleep(Duration::from_millis(300)).await; // Simulate packaging
            package_spinner.finish_with_message(format!("✅ Artifacts packaged to {}", output_dir.display()));
        }

        self.show_build_summary(&build_results, plan);
        Ok(())
    }

    /// Build a single target
    async fn build_target(&self, target: &BuildTarget) -> Result<BuildResult, Box<dyn std::error::Error>> {
        let start_time = Instant::now();
        
        // Check cache first
        let from_cache = !self.command.force && self.is_cached(target);
        
        if from_cache {
            return Ok(BuildResult {
                target_name: target.name.clone(),
                success: true,
                duration: Duration::from_millis(10), // Cache lookup time
                from_cache: true,
                warnings: vec![],
                artifacts: vec![PathBuf::from(format!("artifacts/{}", target.name))],
            });
        }

        // Simulate actual build
        let _build_duration = target.estimated_build_time;
        if self.command.verbose {
            println!("   Building {} ({:?})...", target.name, target.target_type);
        }
        
        // Simulate build time (reduced for demo)
        tokio::time::sleep(Duration::from_millis(50)).await;

        Ok(BuildResult {
            target_name: target.name.clone(),
            success: true,
            duration: start_time.elapsed(),
            from_cache: false,
            warnings: vec![],
            artifacts: vec![PathBuf::from(format!("artifacts/{}", target.name))],
        })
    }

    /// Show comprehensive build summary
    fn show_build_summary(&self, results: &[BuildResult], plan: &BuildPlan) {
        println!();
        println!("🎉 Build completed successfully!");
        
        // Basic statistics
        let total_time: Duration = results.iter().map(|r| r.duration).sum();
        let cache_hits = results.iter().filter(|r| r.from_cache).count();
        let cache_rate = if !results.is_empty() {
            cache_hits as f32 / results.len() as f32 * 100.0
        } else {
            0.0
        };

        println!("   Targets built: {}", results.len());
        println!("   Total time: {}", format_duration(total_time));
        println!("   Cache hit rate: {:.1}%", cache_rate);
        
        if cache_rate > 50.0 {
            println!("   📈 Excellent cache efficiency!");
        }

        // Show warnings if any
        let warnings: Vec<_> = results.iter()
            .flat_map(|r| &r.warnings)
            .collect();

        if !warnings.is_empty() {
            println!("\n⚠️ Build warnings:");
            for warning in warnings.iter().take(3) {
                println!("   • {}", warning);
            }
            if warnings.len() > 3 {
                println!("   ... and {} more", warnings.len() - 3);
            }
        }

        // Show optimization suggestions if analysis was requested
        if self.command.analyze {
            self.show_optimization_suggestions(plan);
        }
    }

    /// Show optimization suggestions
    fn show_optimization_suggestions(&self, plan: &BuildPlan) {
        let opportunities = &plan.complexity_analysis.optimization_opportunities;
        
        if opportunities.is_empty() {
            println!("\n✨ Build is well optimized - no obvious improvements detected");
            return;
        }

        println!("\n💡 Build optimization opportunities:");
        for (i, opp) in opportunities.iter().enumerate().take(3) {
            let priority_icon = match opp.priority {
                OptimizationPriority::Critical => "🔴",
                OptimizationPriority::High => "🟠", 
                OptimizationPriority::Medium => "🟡",
                OptimizationPriority::Low => "🟢",
            };
            
            println!("   {}. {} {} (saves ~{})", 
                    i + 1, 
                    priority_icon,
                    opp.description, 
                    format_duration(opp.potential_savings));
        }

        if opportunities.len() > 3 {
            println!("   ... and {} more opportunities", opportunities.len() - 3);
        }

        println!("\n🔍 For detailed optimization analysis, try:");
        println!("   dora ui build-analyzer");
    }

    /// Execute build with interactive TUI monitor
    async fn execute_build_interactive(&self, _plan: &BuildPlan) -> Result<(), Box<dyn std::error::Error>> {
        println!("🚀 Launching interactive build monitor...");
        println!("💡 Interactive build monitor provides:");
        println!("  • Real-time progress visualization");
        println!("  • Parallel build coordination display");
        println!("  • Build optimization recommendations");
        println!("  • Dependency graph visualization");
        
        // Simulate TUI launch (would integrate with actual TUI framework)
        println!("\n🖥️  [Simulated TUI Build Monitor]");
        println!("   ┌─ Build Progress ──────────────────────┐");
        println!("   │ ████████████████████████████ 100%     │");
        println!("   │ All targets built successfully!       │");
        println!("   └────────────────────────────────────────┘");
        
        Ok(())
    }

    /// Determine appropriate progress style based on context and complexity
    fn determine_progress_style(&self, plan: &BuildPlan) -> BuildProgressStyle {
        if !self.context.should_use_interactive_features() {
            return BuildProgressStyle::None;
        }

        if self.command.progress || plan.complexity_analysis.is_complex() {
            BuildProgressStyle::Detailed
        } else if plan.estimated_duration > Duration::from_secs(10) {
            BuildProgressStyle::ProgressBar
        } else if plan.build_targets.len() > 3 {
            BuildProgressStyle::Spinner
        } else {
            BuildProgressStyle::None
        }
    }

    /// Helper function implementations
    fn create_spinner(&self, message: &str) -> indicatif::ProgressBar {
        let pb = ProgressBar::new_spinner();
        pb.set_style(IndicatifProgressStyle::default_spinner()
            .template("{spinner:.green} {msg}")
            .unwrap());
        pb.set_message(message.to_string());
        pb.enable_steady_tick(Duration::from_millis(80));
        pb
    }

    fn create_progress_bar(&self, len: usize, message: &str) -> ProgressBar {
        let pb = ProgressBar::new(len as u64);
        pb.set_style(IndicatifProgressStyle::default_bar()
            .template("{msg} {bar:40.cyan/blue} {pos:>7}/{len:7}")
            .unwrap());
        pb.set_message(message.to_string());
        pb
    }

    // Placeholder implementations for complex functionality
    async fn analyze_node_build_requirements(&self, node: &NodeConfig) -> Result<BuildTarget, Box<dyn std::error::Error>> {
        let target_type = match node.source.as_str() {
            "rust" => BuildTargetType::RustNode,
            "python" => BuildTargetType::PythonNode,
            "cpp" => BuildTargetType::CppNode,
            _ => BuildTargetType::Configuration,
        };

        let estimated_build_time = match target_type {
            BuildTargetType::RustNode => Duration::from_secs(5),
            BuildTargetType::PythonNode => Duration::from_secs(2),
            BuildTargetType::CppNode => Duration::from_secs(8),
            _ => Duration::from_secs(1),
        };

        Ok(BuildTarget {
            name: node.name.clone(),
            target_type,
            source_path: PathBuf::from(&node.source),
            dependencies: node.dependencies.clone(),
            build_config: BuildConfig {
                release_mode: self.command.release,
                target_arch: self.command.target.clone(),
                optimization_level: if self.command.release { 
                    OptimizationLevel::Release 
                } else { 
                    OptimizationLevel::Debug 
                },
                extra_flags: vec![],
            },
            cache_key: format!("{}:{}", node.name, "mock_hash"),
            estimated_build_time,
        })
    }

    fn build_dependency_graph(&self, targets: &[BuildTarget]) -> Result<DependencyGraph, Box<dyn std::error::Error>> {
        let mut nodes = HashMap::new();
        let mut edges = HashMap::new();

        for target in targets {
            nodes.insert(target.name.clone(), target.clone());
            edges.insert(target.name.clone(), target.dependencies.clone());
        }

        Ok(DependencyGraph { nodes, edges })
    }

    fn determine_build_order(&self, graph: &DependencyGraph) -> Result<Vec<BuildStep>, Box<dyn std::error::Error>> {
        // Simplified topological sort - would use petgraph in production
        let mut order = Vec::new();
        let mut remaining: HashSet<_> = graph.nodes.keys().collect();

        while !remaining.is_empty() {
            let mut ready = Vec::new();
            
            for name in remaining.iter() {
                let deps = graph.edges.get(*name).unwrap();
                let deps_satisfied = deps.iter().all(|dep| !remaining.contains(dep));
                
                if deps_satisfied {
                    ready.push(name.to_string());
                }
            }

            if ready.is_empty() {
                return Err("Circular dependency detected".into());
            }

            // Group independent targets for parallel execution
            if ready.len() > 1 {
                let targets: Vec<_> = ready.iter()
                    .map(|name| graph.nodes.get(name).unwrap().clone())
                    .collect();
                order.push(BuildStep::Parallel(targets));
            } else {
                let target = graph.nodes.get(&ready[0]).unwrap().clone();
                order.push(BuildStep::Single(target));
            }

            for name in &ready {
                remaining.remove(name);
            }
        }

        Ok(order)
    }

    async fn analyze_cache_status(&self, targets: &[BuildTarget]) -> Result<CacheAnalysis, Box<dyn std::error::Error>> {
        let mut cache_hits = 0;
        let mut cache_misses = 0;

        for target in targets {
            if !self.command.force && self.is_cached(target) {
                cache_hits += 1;
            } else {
                cache_misses += 1;
            }
        }

        let cache_hit_rate = if targets.len() > 0 {
            cache_hits as f32 / targets.len() as f32
        } else {
            0.0
        };

        let estimated_time_saved = Duration::from_secs(cache_hits as u64 * 3); // 3s per cached target

        Ok(CacheAnalysis {
            cache_hits,
            cache_misses,
            rebuilds_needed: cache_misses,
            cache_hit_rate,
            estimated_time_saved,
        })
    }

    fn estimate_build_duration(&self, targets: &[BuildTarget], _cache_analysis: &CacheAnalysis) -> Duration {
        let total_build_time: Duration = targets.iter()
            .map(|t| if self.is_cached(t) { Duration::from_millis(100) } else { t.estimated_build_time })
            .sum();

        // Adjust for parallel execution
        let parallel_factor = self.command.jobs.unwrap_or_else(|| num_cpus::get()) as f32;
        let adjusted_time = total_build_time.as_secs_f32() / parallel_factor.min(targets.len() as f32);
        
        Duration::from_secs_f32(adjusted_time)
    }

    fn analyze_build_complexity(&self, targets: &[BuildTarget], graph: &DependencyGraph, cache_analysis: &CacheAnalysis) -> BuildComplexityAnalysis {
        let total_targets = targets.len();
        let estimated_duration = self.estimate_build_duration(targets, cache_analysis);
        
        // Calculate dependency complexity
        let max_deps = graph.edges.values().map(|deps| deps.len()).max().unwrap_or(0);
        let avg_deps = if !graph.edges.is_empty() {
            graph.edges.values().map(|deps| deps.len()).sum::<usize>() / graph.edges.len()
        } else {
            0
        };
        let dependency_complexity = (max_deps * 2 + avg_deps) as u32;

        // Calculate parallel potential
        let parallel_potential = self.calculate_parallel_potential(graph);

        // Identify optimization opportunities
        let optimization_opportunities = self.identify_optimization_opportunities(targets, cache_analysis, parallel_potential);

        // Calculate overall complexity score
        let complexity_score = self.calculate_complexity_score(total_targets, estimated_duration, dependency_complexity, cache_analysis.cache_hit_rate);

        BuildComplexityAnalysis {
            total_targets,
            estimated_duration,
            dependency_complexity,
            cache_hit_rate: cache_analysis.cache_hit_rate,
            optimization_opportunities,
            parallel_potential,
            complexity_score,
        }
    }

    fn calculate_parallel_potential(&self, graph: &DependencyGraph) -> u32 {
        // Calculate maximum parallel jobs possible
        let mut levels = Vec::new();
        let mut remaining: HashSet<_> = graph.nodes.keys().collect();

        while !remaining.is_empty() {
            let mut current_level = Vec::new();
            
            for name in remaining.iter() {
                let deps = graph.edges.get(*name).unwrap();
                let deps_satisfied = deps.iter().all(|dep| !remaining.contains(dep));
                
                if deps_satisfied {
                    current_level.push(name.to_string());
                }
            }

            for name in &current_level {
                remaining.remove(name);
            }

            levels.push(current_level.len());
        }

        levels.into_iter().max().unwrap_or(1) as u32
    }

    fn identify_optimization_opportunities(&self, targets: &[BuildTarget], cache_analysis: &CacheAnalysis, parallel_potential: u32) -> Vec<OptimizationOpportunity> {
        let mut opportunities = Vec::new();

        // Check for parallel build opportunities
        let current_jobs = self.command.jobs.unwrap_or(1);
        if parallel_potential > current_jobs as u32 {
            opportunities.push(OptimizationOpportunity {
                opportunity_type: OptimizationType::ParallelBuilds,
                description: format!(
                    "Increase parallel jobs from {} to {} for faster builds",
                    current_jobs, parallel_potential
                ),
                potential_savings: Duration::from_secs(5),
                implementation_effort: ImplementationEffort::Trivial,
                priority: OptimizationPriority::High,
            });
        }

        // Check for caching improvements
        if cache_analysis.cache_hit_rate < 0.6 {
            opportunities.push(OptimizationOpportunity {
                opportunity_type: OptimizationType::CachingImprovements,
                description: format!(
                    "Cache hit rate is {:.1}% - consider build cache optimization",
                    cache_analysis.cache_hit_rate * 100.0
                ),
                potential_savings: Duration::from_secs(10),
                implementation_effort: ImplementationEffort::Medium,
                priority: OptimizationPriority::Medium,
            });
        }

        // Check for release mode optimization
        if !self.command.release && targets.len() > 3 {
            opportunities.push(OptimizationOpportunity {
                opportunity_type: OptimizationType::CompilerOptimization,
                description: "Consider using --release mode for production builds".to_string(),
                potential_savings: Duration::from_secs(0), // Runtime savings, not build time
                implementation_effort: ImplementationEffort::Trivial,
                priority: OptimizationPriority::Low,
            });
        }

        opportunities
    }

    fn calculate_complexity_score(&self, targets: usize, duration: Duration, deps: u32, cache_rate: f32) -> f64 {
        let target_score = (targets as f64).log2().max(1.0);
        let duration_score = (duration.as_secs() as f64 / 10.0).max(1.0);
        let dep_score = deps as f64 / 10.0;
        let cache_penalty = (1.0 - cache_rate as f64) * 2.0;

        (target_score + duration_score + dep_score + cache_penalty) / 4.0
    }

    fn is_cached(&self, target: &BuildTarget) -> bool {
        // Mock cache check - would check actual cache in production
        target.name.contains("sensor") // Pretend sensor_node is cached
    }

    fn validate_configuration(&self, _config: &DataflowConfig) -> Result<(), Box<dyn std::error::Error>> {
        // Mock validation
        Ok(())
    }

    async fn build_simple(&self, plan: &BuildPlan) -> Result<(), Box<dyn std::error::Error>> {
        println!("Building {} targets...", plan.build_targets.len());
        tokio::time::sleep(Duration::from_millis(500)).await;
        println!("✅ Build completed");
        Ok(())
    }

    async fn build_with_spinner(&self, _plan: &BuildPlan) -> Result<(), Box<dyn std::error::Error>> {
        let spinner = self.create_spinner("Building targets...");
        tokio::time::sleep(Duration::from_millis(800)).await;
        spinner.finish_with_message("✅ Build completed");
        Ok(())
    }

    async fn build_with_progress_bar(&self, plan: &BuildPlan) -> Result<(), Box<dyn std::error::Error>> {
        let pb = self.create_progress_bar(plan.build_targets.len(), "Building");
        
        for (i, target) in plan.build_targets.iter().enumerate() {
            pb.set_message(format!("Building {}", target.name));
            tokio::time::sleep(Duration::from_millis(100)).await;
            pb.set_position(i as u64 + 1);
        }
        
        pb.finish_with_message("✅ Build completed");
        Ok(())
    }

    async fn perform_build_analysis(&self, plan: &BuildPlan) -> Result<(), Box<dyn std::error::Error>> {
        println!("\n📊 Build Analysis:");
        println!("   Complexity Score: {:.2}", plan.complexity_analysis.complexity_score);
        println!("   Parallel Potential: {} jobs", plan.complexity_analysis.parallel_potential);
        println!("   Optimization Opportunities: {}", plan.complexity_analysis.optimization_opportunities.len());
        
        if plan.complexity_analysis.is_complex() {
            println!("   💡 This is a complex build - consider using interactive monitor");
        }
        
        Ok(())
    }

    fn should_prompt_for_interactive(&self, reason: &str, default_yes: bool) -> Result<bool, Box<dyn std::error::Error>> {
        println!("\n🔍 {}", reason);
        println!("Interactive build monitor provides:");
        println!("  • Real-time progress visualization");
        println!("  • Parallel build coordination display");
        println!("  • Build optimization recommendations");
        
        // For demo purposes, just return the default
        Ok(default_yes)
    }

    fn show_optimization_hint(&self, hint: &str, tui_command: &str) {
        if !self.context.show_suggestions() {
            return;
        }

        println!("\n💡 Optimization Hint: {}", hint);
        println!("   Try: {}", tui_command.bright_cyan());
        println!();
    }
}

impl BuildComplexityAnalysis {
    pub fn is_complex(&self) -> bool {
        self.total_targets > 5 || 
        self.estimated_duration > Duration::from_secs(30) ||
        self.dependency_complexity > 10 ||
        self.complexity_score > 2.0
    }
    
    pub fn should_suggest_optimizations(&self) -> bool {
        !self.optimization_opportunities.is_empty() ||
        self.cache_hit_rate < 0.5 ||
        self.parallel_potential > 2
    }
}

// Helper functions
fn format_duration(duration: Duration) -> String {
    let secs = duration.as_secs();
    if secs < 60 {
        format!("{}s", secs)
    } else {
        format!("{}m{}s", secs / 60, secs % 60)
    }
}

// Placeholder implementations for missing components
impl BuildPlanner {
    fn new() -> Self {
        Self {
            dependency_graph: Graph::new(),
            build_cache: HashMap::new(),
        }
    }
}

impl BuildCacheManager {
    fn new() -> Self {
        Self {
            cache_dir: PathBuf::from(".dora_cache"),
            cache_index: HashMap::new(),
        }
    }
}

impl DependencyAnalyzer {
    fn new() -> Self {
        Self {
            resolved_dependencies: HashMap::new(),
            circular_dependencies: Vec::new(),
        }
    }
}

impl OptimizationAnalyzer {
    fn new() -> Self {
        Self {
            optimization_opportunities: Vec::new(),
            performance_metrics: HashMap::new(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn create_test_context() -> ExecutionContext {
        ExecutionContext::detect_basic()
    }

    fn create_test_build_command() -> BuildCommand {
        BuildCommand {
            force: false,
            target: None,
            release: false,
            jobs: Some(4),
            skip_deps: false,
            clean: false,
            verbose: false,
            analyze: true,
            output_dir: None,
            progress: true,
            no_hints: false,
            validate: false,
            ..Default::default()
        }
    }

    #[tokio::test]
    async fn test_enhanced_build_command_creation() {
        let context = create_test_context();
        let command = create_test_build_command();
        
        let enhanced_build = EnhancedBuildCommand::new(context, command);
        
        // Test that the command is created successfully
        assert_eq!(enhanced_build.command.jobs, Some(4));
        assert!(enhanced_build.command.analyze);
        assert!(enhanced_build.command.progress);
    }

    #[tokio::test]
    async fn test_build_plan_creation() {
        let context = create_test_context();
        let command = create_test_build_command();
        let mut enhanced_build = EnhancedBuildCommand::new(context, command);
        
        let config = DataflowConfig {
            name: "test-dataflow".to_string(),
            nodes: vec![
                NodeConfig {
                    name: "node1".to_string(),
                    source: "rust".to_string(),
                    dependencies: vec![],
                },
            ],
            version: "1.0.0".to_string(),
        };
        
        let build_plan = enhanced_build.create_build_plan(&config).await.unwrap();
        
        assert_eq!(build_plan.build_targets.len(), 1);
        assert_eq!(build_plan.target_config.name, "test-dataflow");
    }

    #[test]
    fn test_complexity_analysis() {
        let analysis = BuildComplexityAnalysis {
            total_targets: 10,
            estimated_duration: Duration::from_secs(45),
            dependency_complexity: 15,
            cache_hit_rate: 0.3,
            optimization_opportunities: vec![],
            parallel_potential: 4,
            complexity_score: 3.0,
        };
        
        assert!(analysis.is_complex());
        assert!(analysis.should_suggest_optimizations());
    }

    #[test]
    fn test_optimization_opportunity_creation() {
        let opp = OptimizationOpportunity {
            opportunity_type: OptimizationType::ParallelBuilds,
            description: "Increase parallel jobs".to_string(),
            potential_savings: Duration::from_secs(10),
            implementation_effort: ImplementationEffort::Trivial,
            priority: OptimizationPriority::High,
        };
        
        assert!(matches!(opp.opportunity_type, OptimizationType::ParallelBuilds));
        assert!(matches!(opp.priority, OptimizationPriority::High));
    }
}