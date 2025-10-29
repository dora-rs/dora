use clap::{CommandFactory, Parser};
use colored::Colorize;
use command::Executable;
use eyre::{Context, eyre};
use std::{
    net::{IpAddr, Ipv4Addr},
    path::{Path, PathBuf},
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
    run_new_cli(cli)
}

pub fn run_new_cli(cli: cli::Cli) {
    let command_opt = cli.command.clone();
    let context = cli::context::ExecutionContext::from_cli(&cli);

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
        Some(cli::Command::Debug(cmd)) => {
            if let Some(view) = debug_view_for_tui(&cli, &context, &cmd) {
                if let Err(e) = launch_tui(view) {
                    eprintln!("❌ Failed to launch debug TUI: {}", e);
                    std::process::exit(1);
                }
                return;
            }

            let formatter = cli::output::OutputFormatter::new(
                cli.output.clone(),
                cli.ui_mode.clone(),
                cli.no_hints,
            );
            let runtime = tokio::runtime::Runtime::new().unwrap_or_else(|err| {
                eprintln!("❌ Failed to initialize async runtime: {err}");
                std::process::exit(1);
            });

            if let Err(err) = runtime.block_on(async {
                crate::cli::advanced::run_debug_command(&cmd, &context, &formatter).await
            }) {
                eprintln!("❌ Debug command failed: {err}");
                std::process::exit(1);
            }
        }
        Some(cli::Command::Analyze(cmd)) => {
            let formatter = cli::output::OutputFormatter::new(
                cli.output.clone(),
                cli.ui_mode.clone(),
                cli.no_hints,
            );
            let runtime = tokio::runtime::Runtime::new().unwrap_or_else(|err| {
                eprintln!("❌ Failed to initialize async runtime: {err}");
                std::process::exit(1);
            });

            if let Err(err) = runtime.block_on(async {
                crate::cli::advanced::run_analyze_command(&cmd, &context, &formatter).await
            }) {
                eprintln!("❌ Analyze command failed: {err}");
                std::process::exit(1);
            }
        }
        Some(cli::Command::Monitor(cmd)) => {
            if let Some(view) = monitor_view_for_tui(&cli, &context, &cmd) {
                if let Err(e) = launch_tui(view) {
                    eprintln!("❌ Failed to launch monitor TUI: {}", e);
                    std::process::exit(1);
                }
                return;
            }

            let formatter = cli::output::OutputFormatter::new(
                cli.output.clone(),
                cli.ui_mode.clone(),
                cli.no_hints,
            );
            let runtime = tokio::runtime::Runtime::new().unwrap_or_else(|err| {
                eprintln!("❌ Failed to initialize async runtime: {err}");
                std::process::exit(1);
            });

            if let Err(err) = runtime.block_on(async {
                crate::cli::advanced::run_monitor_command(&cmd, &context, &formatter).await
            }) {
                eprintln!("❌ Monitor command failed: {err}");
                std::process::exit(1);
            }
        }
        Some(cli::Command::System(cmd)) => {
            let formatter = cli::output::OutputFormatter::new(
                cli.output.clone(),
                cli.ui_mode.clone(),
                cli.no_hints,
            );
            let runtime = tokio::runtime::Runtime::new().unwrap_or_else(|err| {
                eprintln!("❌ Failed to initialize async runtime: {err}");
                std::process::exit(1);
            });

            if let Err(err) = runtime.block_on(async {
                crate::cli::advanced::run_system_command(&cmd, &context, &formatter).await
            }) {
                eprintln!("❌ System command failed: {err}");
                std::process::exit(1);
            }
        }
        Some(other) => {
            eprintln!(
                "`dora {}` remains a legacy CLI workflow. Invoke it without the `tui` entry point or launch the TUI with `dora tui` for interactive features.",
                other.command_name()
            );
            std::process::exit(2);
        }
        _ => {
            let legacy_args = Args::parse();
            lib_main(legacy_args);
        }
    }
}

trait CommandName {
    fn command_name(&self) -> &'static str;
}

impl CommandName for cli::Command {
    fn command_name(&self) -> &'static str {
        match self {
            cli::Command::Ps(_) => "ps",
            cli::Command::Start(_) => "start",
            cli::Command::Stop(_) => "stop",
            cli::Command::Logs(_) => "logs",
            cli::Command::Build(_) => "build",
            cli::Command::Up(_) => "up",
            cli::Command::Destroy(_) => "destroy",
            cli::Command::New(_) => "new",
            cli::Command::Check(_) => "check",
            cli::Command::Graph(_) => "graph",
            cli::Command::Inspect(_) => "inspect",
            cli::Command::Debug(_) => "debug",
            cli::Command::Analyze(_) => "analyze",
            cli::Command::Monitor(_) => "monitor",
            cli::Command::Help(_) => "help",
            cli::Command::Tui(_) => "tui",
            cli::Command::Dashboard(_) => "dashboard",
            cli::Command::System(_) => "system",
            cli::Command::Config(_) => "config",
            cli::Command::Daemon(_) => "daemon",
            cli::Command::Runtime(_) => "runtime",
            cli::Command::Coordinator(_) => "coordinator",
            cli::Command::Self_(_) => "self",
        }
    }
}

pub fn print_legacy_help_with_hint() {
    println!("Legacy CLI (default `dora <command>`):\n");
    if let Err(err) = Args::command().print_help() {
        eprintln!("{err}");
    }
    println!("\nHint: run `dora tui --help` for interactive dashboard commands.\n");
}

#[allow(dead_code)]
pub fn execute_legacy_command<'a, I>(args: I, working_dir: Option<&Path>) -> eyre::Result<()>
where
    I: IntoIterator<Item = &'a str>,
{
    let mut argv = vec!["dora".to_string()];
    for arg in args {
        argv.push(arg.to_string());
    }

    let previous_dir = match working_dir {
        Some(dir) => {
            let current = std::env::current_dir().context("failed to read current directory")?;
            if let Err(err) = std::env::set_current_dir(dir) {
                eprintln!(
                    "warning: failed to change working directory to '{}': {err}. Continuing in current directory.",
                    dir.display()
                );
                None
            } else {
                Some(current)
            }
        }
        None => None,
    };

    let parse_result = Args::try_parse_from(&argv);
    let exec_result = match parse_result {
        Ok(args) => args.command.execute(),
        Err(err) => Err(eyre!(err.to_string())),
    };

    if let (Some(original_dir), Some(_)) = (previous_dir, working_dir) {
        if let Err(err) = std::env::set_current_dir(original_dir) {
            eprintln!("warning: failed to restore working directory: {err}");
        }
    }

    exec_result
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

fn debug_view_for_tui(
    cli: &cli::Cli,
    context: &cli::context::ExecutionContext,
    cmd: &cli::commands::DebugCommand,
) -> Option<tui::ViewType> {
    let wants_tui = matches!(cli.ui_mode, Some(cli::UiMode::Tui))
        || matches!(context.ui_mode, Some(cli::UiMode::Tui))
        || (context.terminal_capabilities.tui_capable
            && (cmd.live || matches!(context.user_preference, cli::UiMode::Tui)));

    if wants_tui {
        let dataflow_id = cmd
            .dataflow
            .dataflow
            .as_ref()
            .map(|df| df.to_string_lossy().to_string())
            .or_else(|| cmd.dataflow.name.clone())
            .unwrap_or_else(|| "current".to_string());

        Some(tui::ViewType::DebugSession { dataflow_id })
    } else {
        None
    }
}

fn monitor_view_for_tui(
    cli: &cli::Cli,
    context: &cli::context::ExecutionContext,
    cmd: &cli::commands::MonitorCommand,
) -> Option<tui::ViewType> {
    let wants_tui = matches!(cli.ui_mode, Some(cli::UiMode::Tui))
        || matches!(context.ui_mode, Some(cli::UiMode::Tui))
        || (context.terminal_capabilities.tui_capable && cmd.interval.is_some());

    if wants_tui {
        Some(tui::ViewType::SystemMonitor)
    } else {
        None
    }
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
