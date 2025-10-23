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

// Tier 2: Enhanced commands
#[derive(Args, Clone, Debug, Default)]
pub struct InspectCommand {
    #[clap(flatten)]
    pub common: CommonArgs,
    
    #[clap(flatten)]
    pub dataflow: DataflowArgs,
    
    /// Resource to inspect
    pub resource: Option<String>,
    
    /// Deep inspection
    #[clap(long)]
    pub deep: bool,
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