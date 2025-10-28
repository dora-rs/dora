use clap::Parser;
use colored::Colorize;
use command::Executable;
use dora_core::{
    descriptor::{Descriptor, DescriptorExt},
    topics::DORA_COORDINATOR_PORT_CONTROL_DEFAULT,
};
use eyre::{Context, eyre};
use std::{
    net::{IpAddr, Ipv4Addr, SocketAddr},
    path::PathBuf,
};

use crate::command::check::check_environment;

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
    run_new_cli(cli)
}

pub fn run_new_cli(cli: cli::Cli) {
    let command_opt = cli.command.clone();

    match command_opt {
        Some(cli::Command::Tui(cmd)) => {
            if cli.verbose {
                println!("🖥️  Launching TUI interface...");
                println!("  Output Format: {:?}", cli.output);
                println!();
            }

            let initial_view = if let Some(view) = &cmd.view {
                match view {
                    cli::TuiView::Dashboard => tui::ViewType::Dashboard,
                    cli::TuiView::Dataflow => tui::ViewType::DataflowManager,
                    cli::TuiView::Performance => tui::ViewType::SystemMonitor,
                    cli::TuiView::Logs => tui::ViewType::LogViewer {
                        target: "all".to_string(),
                    },
                }
            } else {
                tui::ViewType::Dashboard
            };

            if let Err(e) = launch_tui(initial_view) {
                eprintln!("❌ Failed to launch TUI: {}", e);
                std::process::exit(1);
            }
        }
        Some(cli::Command::Dashboard(_)) => {
            if cli.verbose {
                println!("🖥️  Launching Dashboard...");
            }

            if let Err(e) = launch_tui(tui::ViewType::Dashboard) {
                eprintln!("❌ Failed to launch dashboard: {}", e);
                std::process::exit(1);
            }
        }
        Some(cli::Command::Check(cmd)) => {
            let context = cli::context::ExecutionContext::from_cli(&cli);
            if let Err(err) = execute_check_command(cmd, &context) {
                eprintln!("[ERROR] {err:?}");
                std::process::exit(1);
            }
        }
        _ => {
            let legacy_args = Args::parse();
            lib_main(legacy_args);
        }
    }
}

fn execute_check_command(
    cmd: crate::cli::commands::CheckCommand,
    context: &cli::context::ExecutionContext,
) -> eyre::Result<()> {
    let crate::cli::commands::CheckCommand { common, dataflow } = cmd;

    let base_dir = common
        .working_dir
        .clone()
        .map(|dir| {
            let candidate = PathBuf::from(dir);
            if candidate.is_relative() {
                context.working_dir.join(candidate)
            } else {
                candidate
            }
        })
        .unwrap_or_else(|| context.working_dir.clone());

    if let Some(dataflow) = dataflow.dataflow.clone() {
        let full_path = if dataflow.is_relative() {
            base_dir.join(dataflow)
        } else {
            dataflow
        };

        let descriptor_path = full_path
            .canonicalize()
            .wrap_err("failed to resolve dataflow path")?;

        let working_dir = descriptor_path
            .parent()
            .ok_or_else(|| eyre!("dataflow path has no parent directory"))?
            .to_path_buf();

        Descriptor::blocking_read(&descriptor_path)?
            .check(&working_dir)
            .wrap_err("dataflow validation failed")?;
    }

    let coordinator_addr: SocketAddr =
        (crate::LOCALHOST, DORA_COORDINATOR_PORT_CONTROL_DEFAULT).into();
    check_environment(coordinator_addr)
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
