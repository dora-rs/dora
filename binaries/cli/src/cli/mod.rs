use clap::{Parser, Subcommand, ValueEnum};
use serde::{Deserialize, Serialize};

pub mod advanced;
pub mod commands;
pub mod context;
pub mod interface;
pub mod output;
pub mod traits;

pub use commands::*;

/// Dora CLI - Hybrid command-line interface for dataflow runtime
#[derive(Debug, Parser)]
#[clap(
    name = "dora",
    about = "Dora dataflow runtime",
    version,
    arg_required_else_help = true,
    disable_help_subcommand = true
)]
pub struct Cli {
    #[clap(subcommand)]
    pub command: Option<Command>,

    /// Force interface mode
    #[clap(long, global = true, value_enum)]
    pub ui_mode: Option<UiMode>,

    /// Output format
    #[clap(long, global = true, value_enum, default_value = "auto")]
    pub output: OutputFormat,

    /// Disable TUI hints and prompts
    #[clap(long, global = true)]
    pub no_hints: bool,

    /// Verbose output
    #[clap(short, long, global = true)]
    pub verbose: bool,

    /// Quiet mode - suppress non-error output
    #[clap(short, long, global = true)]
    pub quiet: bool,
}

/// Three-tier command structure
#[derive(Debug, Clone, Subcommand)]
pub enum Command {
    // Tier 1: Core Docker-like commands (Basic operations)
    /// List running dataflows and their status
    #[clap(alias = "list")]
    Ps(PsCommand),

    /// Start a dataflow
    Start(StartCommand),

    /// Stop a dataflow
    Stop(StopCommand),

    /// View dataflow logs
    Logs(LogsCommand),

    /// Build dataflow configuration
    Build(BuildCommand),

    /// Run a complete dataflow (up + start)
    Up(UpCommand),

    /// Destroy a dataflow and clean up resources
    Destroy(DestroyCommand),

    /// Create new dataflow, node, or operator
    New(NewCommand),

    /// Check dataflow configuration for errors
    Check(CheckCommand),

    /// Generate dataflow graph visualization
    Graph(GraphCommand),

    // Tier 2: Enhanced commands with smart suggestions
    /// Intelligent resource inspection with TUI suggestions
    Inspect(InspectCommand),

    /// Enhanced debugging with auto-TUI features
    Debug(DebugCommand),

    /// Multi-modal analysis with adaptive interface
    Analyze(AnalyzeCommand),

    /// Real-time system monitoring
    Monitor(MonitorCommand),

    /// Smart help system with tutorials and contextual guidance
    Help(HelpCommand),

    // Tier 3: Explicit TUI modes
    /// Launch TUI interface
    #[clap(alias = "ui")]
    Tui(UiCommand),

    /// Launch interactive dashboard
    Dashboard(DashboardCommand),

    // System management
    /// System-level operations
    System(SystemCommand),

    /// Configuration management
    Config(ConfigCommand),

    /// Daemon operations
    Daemon(DaemonCommand),

    /// Runtime operations
    Runtime(RuntimeCommand),

    /// Coordinator operations
    Coordinator(CoordinatorCommand),

    /// Self-management (updates, etc.)
    #[clap(name = "self")]
    Self_(SelfCommand),
}

/// UI mode selection
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, ValueEnum, Serialize, Deserialize)]
pub enum UiMode {
    /// Smart decisions based on context
    Auto,
    /// Force CLI output only
    Cli,
    /// Force TUI when available
    Tui,
    /// Minimal output for CI/scripting
    Minimal,
}

/// Output format selection
#[derive(Clone, Debug, PartialEq, Eq, ValueEnum, Serialize, Deserialize)]
pub enum OutputFormat {
    /// Context-appropriate formatting
    Auto,
    /// Human-readable table format
    Table,
    /// Machine-readable JSON
    Json,
    /// YAML format
    Yaml,
    /// Minimal text output
    Minimal,
}

impl Default for UiMode {
    fn default() -> Self {
        UiMode::Auto
    }
}

impl Default for OutputFormat {
    fn default() -> Self {
        OutputFormat::Auto
    }
}

impl UiMode {
    /// Convert to u8 for caching and comparison
    pub fn as_u8(self) -> u8 {
        match self {
            UiMode::Auto => 0,
            UiMode::Cli => 1,
            UiMode::Tui => 2,
            UiMode::Minimal => 3,
        }
    }
}

// Re-export for backward compatibility with existing code
pub use crate::command::Command as LegacyCommand;
