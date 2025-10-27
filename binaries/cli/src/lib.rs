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

// Issue #4: Configuration system
pub mod config;

// Issue #5: Automation detection module
pub mod automation;

// Issue #9: TUI module
pub mod tui;

// Issue #17: Inspection module
pub mod inspection;

// Issue #18: Debug module
pub mod debug;

// Issue #19: Analysis module
pub mod analysis;

// Issue #20: Logs module
pub mod logs;

// Issue #21: Help module
pub mod help;

// Issue #22: UX validation and integration module
pub mod ux;

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
    // Handle case where no command is provided (e.g., --version or --help)
    let Some(command) = &cli.command else {
        // Clap will have already handled --version and --help
        // If we get here with no command, something went wrong
        eprintln!("Error: No command provided. Use --help for usage information.");
        std::process::exit(1);
    };

    // Create execution context with enhanced detection (Issue #2)
    let context = cli::context::ExecutionContext::from_cli(&cli);

    // Create interface selector (Issue #3)
    let config = cli::interface::UserConfig::default();
    let mut selector = cli::interface::InterfaceSelector::new(context.clone(), config);
    let interface_decision = selector.select_interface(command);

    // Debug output (only if verbose flag is set)
    if cli.verbose {
        println!("🚀 Dora CLI - Verbose Mode");
        println!("Global flags:");
        println!("  UI Mode: {:?}", cli.ui_mode.unwrap_or_default());
        println!("  Output Format: {:?}", cli.output);
        println!();
        println!("Execution Context:");
        println!("  TTY: {}, Piped: {}, Scripted: {}", context.is_tty, context.is_piped, context.is_scripted);
        println!("  Selected Strategy: {:?}", interface_decision.strategy);
        println!();
    }

    match command {
        // Basic commands - placeholder for actual implementation
        cli::Command::Ps(_) | cli::Command::Start(_) | cli::Command::Stop(_) |
        cli::Command::Logs(_) | cli::Command::Build(_) | cli::Command::Up(_) |
        cli::Command::Destroy(_) | cli::Command::New(_) | cli::Command::Check(_) |
        cli::Command::Graph(_) => {
            // TODO: Implement actual command logic
            // For now, just silently succeed
        },

        // Enhanced commands
        cli::Command::Inspect(_) | cli::Command::Debug(_) | cli::Command::Analyze(_) |
        cli::Command::Monitor(_) | cli::Command::Help(_) => {
            // TODO: Implement actual command logic
        },

        // TUI commands
        cli::Command::Tui(cmd) => {
            if cli.verbose {
                println!("🖥️  Launching TUI interface...");
            }

            // Determine initial view from command
            let initial_view = if let Some(view) = &cmd.view {
                match view {
                    cli::TuiView::Dashboard => tui::ViewType::Dashboard,
                    cli::TuiView::Dataflow => tui::ViewType::DataflowManager,
                    cli::TuiView::Performance => tui::ViewType::SystemMonitor,
                    cli::TuiView::Logs => tui::ViewType::LogViewer { target: "all".to_string() },
                }
            } else {
                tui::ViewType::Dashboard
            };

            // Launch TUI
            if let Err(e) = launch_tui(initial_view) {
                eprintln!("❌ Failed to launch TUI: {}", e);
                std::process::exit(1);
            }
        },
        cli::Command::Dashboard(_) => {
            if cli.verbose {
                println!("🖥️  Launching Dashboard...");
            }

            // Launch TUI with dashboard view
            if let Err(e) = launch_tui(tui::ViewType::Dashboard) {
                eprintln!("❌ Failed to launch dashboard: {}", e);
                std::process::exit(1);
            }
        },

        // System commands
        cli::Command::System(_) | cli::Command::Config(_) |
        cli::Command::Daemon(_) | cli::Command::Runtime(_) |
        cli::Command::Coordinator(_) | cli::Command::Self_(_) => {
            // TODO: Implement actual system command logic
        }
    }
}

/// Launch TUI with specified initial view
fn launch_tui(initial_view: tui::ViewType) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    // Create tokio runtime for async TUI
    let runtime = tokio::runtime::Runtime::new()?;

    runtime.block_on(async {
        // Create TUI app
        let mut app = tui::DoraApp::new(initial_view);

        // Run the TUI
        app.run().await
    })
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
