use colored::Colorize;
use command::Executable;
use std::{
    net::{IpAddr, Ipv4Addr},
    path::PathBuf,
};

mod command;
mod common;
mod formatting;
pub mod output;
pub mod session;
mod template;

// New hybrid CLI module
pub mod cli;

// Issue #17: Inspection module
pub mod inspection;

pub use command::build;
pub use command::{run, run_func};

const LOCALHOST: IpAddr = IpAddr::V4(Ipv4Addr::new(127, 0, 0, 1));
const LISTEN_WILDCARD: IpAddr = IpAddr::V4(Ipv4Addr::new(0, 0, 0, 0));

#[derive(Debug, clap::Parser)]
#[clap(version)]
pub struct Args {
    #[clap(subcommand)]
    command: command::Command,
}

#[derive(Debug, clap::Args)]
pub struct CommandNew {
    /// The entity that should be created
    #[clap(long, value_enum, default_value_t = Kind::Dataflow)]
    kind: Kind,
    /// The programming language that should be used
    #[clap(long, value_enum, default_value_t = Lang::Rust)]
    lang: Lang,
    /// Desired name of the entity
    name: String,
    /// Where to create the entity
    #[clap(hide = true)]
    path: Option<PathBuf>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, clap::ValueEnum)]
enum Kind {
    Dataflow,
    Node,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, clap::ValueEnum)]
enum Lang {
    Rust,
    Python,
    C,
    Cxx,
}

/// Main entry point - maintains backward compatibility
pub fn lib_main(args: Args) {
    if let Err(err) = args.command.execute() {
        eprintln!("\n\n{}", "[ERROR]".bold().red());
        eprintln!("{err:?}");
        std::process::exit(1);
    }
}

/// New hybrid CLI entry point
pub fn hybrid_main(cli: cli::Cli) {
    // Create execution context with enhanced detection (Issue #2)
    let context = cli::context::ExecutionContext::from_cli(&cli);
    
    // Create interface selector (Issue #3)
    let config = cli::interface::UserConfig::default();
    let mut selector = cli::interface::InterfaceSelector::new(context.clone(), config);
    let interface_decision = selector.select_interface(&cli.command);
    
    // Display the new CLI structure and global flags
    println!("🚀 New Dora Hybrid CLI (Issues #1, #2 & #3 Complete)");
    println!("Global flags detected:");
    println!("  UI Mode: {:?}", cli.ui_mode.unwrap_or_default());
    println!("  Output Format: {:?}", cli.output);
    println!("  No Hints: {}", cli.no_hints);
    println!("  Verbose: {}", cli.verbose);
    println!("  Quiet: {}", cli.quiet);
    println!();
    
    // Display context detection results (Issue #2)
    println!("📋 Execution Context Detection:");
    println!("  TTY: {}", context.is_tty);
    println!("  Piped: {}", context.is_piped);
    println!("  Scripted: {}", context.is_scripted);
    if let Some((w, h)) = context.terminal_size {
        println!("  Terminal Size: {}x{}", w, h);
    }
    println!("  CI Environment: {:?}", context.environment.ci_environment);
    println!("  Shell: {:?}", context.environment.shell_type);
    println!("  Color Support: {}", context.terminal_capabilities.supports_color);
    println!("  Unicode Support: {}", context.terminal_capabilities.supports_unicode);
    println!("  TUI Capable: {}", context.terminal_capabilities.tui_capable);
    println!();
    
    // Display interface selection results (Issue #3)
    println!("🎯 Smart Interface Selection:");
    println!("  Selected Strategy: {:?}", interface_decision.strategy);
    println!("  Confidence: {:.2}", interface_decision.confidence);
    println!("  Reason: {}", interface_decision.reason);
    if let Some(fallback) = &interface_decision.fallback {
        println!("  Fallback: {:?}", fallback);
    }
    println!();
    
    match &cli.command {
        // Tier 1: Core commands (demonstrate structure)
        cli::Command::Ps(_) => {
            println!("Tier 1 Command: PS (List) - Enhanced with smart hints");
            println!("💡 In future issues, this will show intelligent suggestions");
        },
        cli::Command::Start(_) => {
            println!("Tier 1 Command: START - Enhanced with progress indicators");
            println!("💡 In future issues, this will show progress and auto-TUI");
        },
        cli::Command::Stop(_) => {
            println!("Tier 1 Command: STOP - Enhanced with graceful shutdown");
        },
        cli::Command::Logs(_) => {
            println!("Tier 1 Command: LOGS - Enhanced with smart filtering");
        },
        cli::Command::Build(_) => {
            println!("Tier 1 Command: BUILD - Enhanced with rich output");
        },
        cli::Command::Up(_) | cli::Command::Destroy(_) | cli::Command::New(_) |
        cli::Command::Check(_) | cli::Command::Graph(_) => {
            println!("Tier 1 Command: Basic Docker-like operation");
        },
        
        // Tier 2: Enhanced commands with smart suggestions
        cli::Command::Inspect(_) => {
            println!("🎯 Tier 2 Command: INSPECT - Smart resource inspection");
            println!("💡 Will implement complexity analysis and auto-TUI in Issue #17");
        },
        cli::Command::Debug(_) => {
            println!("🎯 Tier 2 Command: DEBUG - Enhanced debugging");
            println!("💡 Will implement auto-issue detection in Issue #18");
        },
        cli::Command::Analyze(_) => {
            println!("🎯 Tier 2 Command: ANALYZE - Multi-modal analysis");
            println!("💡 Will implement adaptive interface in Issue #19");
        },
        cli::Command::Monitor(_) => {
            println!("🎯 Tier 2 Command: MONITOR - Real-time monitoring");
            println!("💡 Will implement smart monitoring in Issue #14");
        },
        
        // Tier 3: TUI commands
        cli::Command::Ui(_) => {
            println!("🖥️  Tier 3 Command: TUI - Launch TUI interface");
            println!("💡 Will implement TUI launcher in Issue #9");
        },
        cli::Command::Dashboard(_) => {
            println!("🖥️  Tier 3 Command: DASHBOARD - Interactive dashboard");
            println!("💡 Will implement dashboard in Issue #24");
        },
        
        // System commands
        cli::Command::System(_) | cli::Command::Config(_) |
        cli::Command::Daemon(_) | cli::Command::Runtime(_) |
        cli::Command::Coordinator(_) | cli::Command::Self_(_) => {
            println!("⚙️  System Command - Management operation");
        }
    }
    
    println!();
    println!("✅ Issue #3 Complete: Interface Selection Engine");
    println!("📋 Next: Issue #4 (User Configuration System)");
    println!("🔗 See IMPLEMENTATION_ROADMAP.md for full plan");
}

// Legacy conversion will be implemented in future issues when needed

#[cfg(feature = "python")]
use clap::Parser;
#[cfg(feature = "python")]
use pyo3::{
    Bound, PyResult, Python, pyfunction, pymodule,
    types::{PyModule, PyModuleMethods},
    wrap_pyfunction,
};

#[cfg(feature = "python")]
#[pyfunction]
fn py_main(_py: Python) -> PyResult<()> {
    pyo3::prepare_freethreaded_python();
    // Skip first argument as it is a python call.
    let args = std::env::args_os().skip(1).collect::<Vec<_>>();

    match Args::try_parse_from(args) {
        Ok(args) => lib_main(args),
        Err(err) => {
            eprintln!("{err}");
        }
    }
    Ok(())
}

/// A Python module implemented in Rust.
#[cfg(feature = "python")]
#[pymodule]
fn dora_cli(_py: Python, m: Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(py_main, &m)?)?;
    m.add("__version__", env!("CARGO_PKG_VERSION"))?;
    Ok(())
}
