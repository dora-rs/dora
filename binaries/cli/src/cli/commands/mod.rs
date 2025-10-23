use clap::Args;
use std::path::PathBuf;

// Basic command structures for now - will be enhanced in future issues

#[derive(Args, Clone, Debug, Default)]
pub struct CommonArgs {
    /// Working directory
    #[clap(short = 'C', long, global = true)]
    pub working_dir: Option<PathBuf>,
    
    /// Configuration file path
    #[clap(short, long, global = true)]
    pub config: Option<PathBuf>,
}

#[derive(Args, Clone, Debug, Default)]
pub struct DataflowArgs {
    /// Dataflow configuration file
    #[clap(value_name = "DATAFLOW")]
    pub dataflow: Option<PathBuf>,
    
    /// Dataflow name (for running dataflows)
    #[clap(long)]
    pub name: Option<String>,
}

#[derive(Args, Clone, Debug, Default)]
pub struct NodeArgs {
    /// Specific nodes to target
    #[clap(long, value_delimiter = ',')]
    pub nodes: Option<Vec<String>>,
}

// Tier 1: Core commands
#[derive(Args, Clone, Debug, Default)]
pub struct PsCommand {
    #[clap(flatten)]
    pub common: CommonArgs,
    
    #[clap(flatten)]
    pub dataflow: DataflowArgs,
    
    /// Show all dataflows
    #[clap(short, long)]
    pub all: bool,
}

#[derive(Args, Clone, Debug)]
pub struct StartCommand {
    #[clap(flatten)]
    pub common: CommonArgs,
    
    #[clap(flatten)]
    pub dataflow: DataflowArgs,
    
    #[clap(flatten)]
    pub nodes: NodeArgs,
    
    /// Enable debug mode
    #[clap(long)]
    pub debug: bool,
}

#[derive(Args, Clone, Debug)]
pub struct StopCommand {
    #[clap(flatten)]
    pub common: CommonArgs,
    
    #[clap(flatten)]
    pub dataflow: DataflowArgs,
    
    #[clap(flatten)]
    pub nodes: NodeArgs,
    
    /// Force stop
    #[clap(long)]
    pub force: bool,
}

#[derive(Args, Clone, Debug, Default)]
pub struct LogsCommand {
    #[clap(flatten)]
    pub common: CommonArgs,
    
    #[clap(flatten)]
    pub dataflow: DataflowArgs,
    
    #[clap(flatten)]
    pub nodes: NodeArgs,
    
    /// Follow log output
    #[clap(short, long)]
    pub follow: bool,
}

#[derive(Args, Clone, Debug)]
pub struct BuildCommand {
    #[clap(flatten)]
    pub common: CommonArgs,
    
    #[clap(flatten)]
    pub dataflow: DataflowArgs,
    
    /// Validate only
    #[clap(long)]
    pub validate: bool,
}

#[derive(Args, Clone, Debug)]
pub struct UpCommand {
    #[clap(flatten)]
    pub common: CommonArgs,
    
    #[clap(flatten)]
    pub dataflow: DataflowArgs,
    
    /// Run in background
    #[clap(short, long)]
    pub detach: bool,
}

#[derive(Args, Clone, Debug)]
pub struct DestroyCommand {
    #[clap(flatten)]
    pub common: CommonArgs,
    
    #[clap(flatten)]
    pub dataflow: DataflowArgs,
    
    /// Force destroy
    #[clap(long)]
    pub force: bool,
}

#[derive(Args, Clone, Debug)]
pub struct NewCommand {
    #[clap(flatten)]
    pub common: CommonArgs,
    
    /// Entity type to create
    #[clap(long, value_enum, default_value = "dataflow")]
    pub kind: NewKind,
    
    /// Programming language
    #[clap(long, value_enum, default_value = "rust")]
    pub lang: NewLang,
    
    /// Name of the entity
    pub name: String,
}

#[derive(clap::ValueEnum, Clone, Debug)]
pub enum NewKind {
    Dataflow,
    Node,
    Operator,
}

#[derive(clap::ValueEnum, Clone, Debug)]
pub enum NewLang {
    Rust,
    Python,
    C,
    Cxx,
}

#[derive(Args, Clone, Debug)]
pub struct CheckCommand {
    #[clap(flatten)]
    pub common: CommonArgs,
    
    #[clap(flatten)]
    pub dataflow: DataflowArgs,
}

#[derive(Args, Clone, Debug)]
pub struct GraphCommand {
    #[clap(flatten)]
    pub common: CommonArgs,
    
    #[clap(flatten)]
    pub dataflow: DataflowArgs,
    
    /// Output format
    #[clap(long, value_enum, default_value = "mermaid")]
    pub format: GraphFormat,
}

#[derive(clap::ValueEnum, Clone, Debug)]
pub enum GraphFormat {
    Mermaid,
    Dot,
    Json,
}

// Tier 2: Enhanced commands (Issue #17)
#[derive(Args, Clone, Debug, Default)]
pub struct InspectCommand {
    #[clap(flatten)]
    pub common: CommonArgs,

    #[clap(flatten)]
    pub dataflow: DataflowArgs,

    /// Resource to inspect (dataflow, node, system, etc.)
    pub target: Option<String>,

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

    /// Time window for historical analysis (e.g., "1h", "30m", "1d")
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
    pub format: Option<InspectOutputFormat>,

    /// Force TUI mode
    #[clap(long)]
    pub tui: bool,

    /// Force CLI text output
    #[clap(long)]
    pub text: bool,

    /// Export analysis results to file
    #[clap(long)]
    pub export: Option<std::path::PathBuf>,

    /// Suppress hints and suggestions
    #[clap(long)]
    pub no_hints: bool,
}

#[derive(clap::ValueEnum, Clone, Debug)]
pub enum ResourceType {
    Dataflow,
    Node,
    System,
    Network,
    Storage,
}

#[derive(clap::ValueEnum, Clone, Debug)]
pub enum InspectionFocus {
    Performance,
    Errors,
    Dependencies,
    Health,
    All,
}

#[derive(clap::ValueEnum, Clone, Debug)]
pub enum InspectOutputFormat {
    Table,
    Json,
    Yaml,
    Minimal,
}

#[derive(Args, Clone, Debug, Default)]
pub struct DebugCommand {
    #[clap(flatten)]
    pub common: CommonArgs,
    
    #[clap(flatten)]
    pub dataflow: DataflowArgs,
    
    /// Auto-detect issues
    #[clap(long)]
    pub auto: bool,
}

#[derive(Args, Clone, Debug)]
pub struct AnalyzeCommand {
    #[clap(flatten)]
    pub common: CommonArgs,
    
    #[clap(flatten)]
    pub dataflow: DataflowArgs,
    
    /// Analysis type
    #[clap(value_enum)]
    pub analysis_type: Option<AnalysisType>,
}

#[derive(clap::ValueEnum, Clone, Debug)]
pub enum AnalysisType {
    Performance,
    Resources,
    Trends,
    Complexity,
}

#[derive(Args, Clone, Debug)]
pub struct MonitorCommand {
    #[clap(flatten)]
    pub common: CommonArgs,
    
    #[clap(flatten)]
    pub dataflow: DataflowArgs,
    
    /// Monitoring interval
    #[clap(long)]
    pub interval: Option<String>,
}

// Tier 3: TUI commands
#[derive(Args, Clone, Debug)]
pub struct UiCommand {
    #[clap(flatten)]
    pub common: CommonArgs,
    
    /// Initial view
    #[clap(long, value_enum)]
    pub view: Option<TuiView>,
}

#[derive(clap::ValueEnum, Clone, Debug)]
pub enum TuiView {
    Dashboard,
    Dataflow,
    Performance,
    Logs,
}

#[derive(Args, Clone, Debug)]
pub struct DashboardCommand {
    #[clap(flatten)]
    pub common: CommonArgs,
    
    /// Dashboard layout
    #[clap(long)]
    pub layout: Option<String>,
}

// System commands
#[derive(Args, Clone, Debug)]
pub struct SystemCommand {
    #[clap(flatten)]
    pub common: CommonArgs,
    
    #[clap(subcommand)]
    pub subcommand: SystemSubcommand,
}

#[derive(clap::Subcommand, Clone, Debug)]
pub enum SystemSubcommand {
    Status,
    Info,
    Clean,
}

#[derive(Args, Clone, Debug)]
pub struct ConfigCommand {
    #[clap(flatten)]
    pub common: CommonArgs,
    
    #[clap(subcommand)]
    pub subcommand: ConfigSubcommand,
}

#[derive(clap::Subcommand, Clone, Debug)]
pub enum ConfigSubcommand {
    Get { key: String },
    Set { key: String, value: String },
    List,
}

#[derive(Args, Clone, Debug)]
pub struct DaemonCommand {
    #[clap(flatten)]
    pub common: CommonArgs,
    
    #[clap(subcommand)]
    pub subcommand: DaemonSubcommand,
}

#[derive(clap::Subcommand, Clone, Debug)]
pub enum DaemonSubcommand {
    Start,
    Stop,
    Status,
}

#[derive(Args, Clone, Debug)]
pub struct RuntimeCommand {
    #[clap(flatten)]
    pub common: CommonArgs,
    
    #[clap(subcommand)]
    pub subcommand: RuntimeSubcommand,
}

#[derive(clap::Subcommand, Clone, Debug)]
pub enum RuntimeSubcommand {
    Start,
    Stop,
    Status,
}

#[derive(Args, Clone, Debug)]
pub struct CoordinatorCommand {
    #[clap(flatten)]
    pub common: CommonArgs,
    
    #[clap(subcommand)]
    pub subcommand: CoordinatorSubcommand,
}

#[derive(clap::Subcommand, Clone, Debug)]
pub enum CoordinatorSubcommand {
    Start,
    Stop,
    Status,
}

#[derive(Args, Clone, Debug)]
pub struct SelfCommand {
    #[clap(subcommand)]
    pub subcommand: SelfSubcommand,
}

#[derive(clap::Subcommand, Clone, Debug)]
pub enum SelfSubcommand {
    Update,
    Version,
    Info,
}

// Issue #17: InspectCommand implementation
impl InspectCommand {
    /// Execute the inspect command
    pub async fn execute(&self) -> eyre::Result<()> {
        use crate::inspection::{ResourceAnalyzer, CliRenderer};

        // Determine the target
        let target = self.target.clone()
            .or_else(|| self.dataflow.dataflow.as_ref().map(|p| p.display().to_string()))
            .unwrap_or_else(|| "current".to_string());

        // Create analyzer
        let mut analyzer = ResourceAnalyzer::new();

        // Perform analysis
        let result = analyzer.analyze_resource(
            &target,
            self.resource_type.clone(),
            self.depth,
            self.focus.clone(),
        ).await?;

        // Check if we should use TUI
        if self.tui || (!self.text && self.should_suggest_tui(&result)) {
            // For now, just print a message - TUI integration will be in future issues
            println!("🖥️  TUI mode would be launched here (Issue #9)");
            println!("Complexity: {:.0}, Recommended for better visualization\n", result.complexity_score);
        }

        // Render output
        let renderer = CliRenderer::new();
        let output = renderer.render(&result, self.format.clone())?;
        println!("{}", output);

        // Export if requested
        if let Some(export_path) = &self.export {
            use std::fs;
            let export_format = export_path.extension()
                .and_then(|e| e.to_str())
                .map(|e| match e {
                    "json" => InspectOutputFormat::Json,
                    "yaml" | "yml" => InspectOutputFormat::Yaml,
                    _ => InspectOutputFormat::Table,
                });

            let export_output = renderer.render(&result, export_format)?;
            fs::write(export_path, export_output)?;
            println!("\n✅ Results exported to: {}", export_path.display());
        }

        // Show hints unless suppressed
        if !self.no_hints {
            self.show_hints(&result);
        }

        Ok(())
    }

    /// Determine if TUI should be suggested
    fn should_suggest_tui(&self, result: &crate::inspection::InspectionResult) -> bool {
        // Suggest TUI for:
        // 1. Complex resources (score > 50)
        // 2. Live monitoring mode
        // 3. Multiple issues or recommendations
        result.complexity_score > 50.0
            || self.live
            || result.recommendations.len() > 3
            || result.error_summary.total_errors > 5
    }

    /// Show helpful hints based on analysis
    fn show_hints(&self, result: &crate::inspection::InspectionResult) {
        use colored::Colorize;

        println!("\n{}", "💡 Hints:".bold().cyan());

        // Hint for live mode
        if !self.live && result.health_score.overall_score < 90.0 {
            println!("  • Use --live for real-time monitoring");
        }

        // Hint for depth
        if self.depth < 3 && !result.recommendations.is_empty() {
            println!("  • Use --depth 3 for comprehensive analysis");
        }

        // Hint for focus
        if self.focus.is_none() && result.error_summary.total_errors > 0 {
            println!("  • Use --focus errors to deep-dive into error patterns");
        }

        // Hint for TUI
        if !self.tui && result.complexity_score > 50.0 {
            println!("  • Use --tui for better visualization of complex resources");
        }

        // Hint for export
        if self.export.is_none() {
            println!("  • Use --export <path> to save analysis results");
        }
    }
}