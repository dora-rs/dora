use clap::Parser;
use colored::Colorize;
use dora_cli::{run, Command};
#[cfg(feature = "tracing")]
use dora_tracing::set_up_tracing;
use dora_tracing::set_up_tracing_opts;
use eyre::Context;

fn main() {
    if let Err(err) = main_inner() {
        eprintln!("\n\n{}", "[ERROR]".bold().red());
        eprintln!("{err:#}");
        std::process::exit(1);
    }
}

#[derive(Debug, clap::Parser)]
#[clap(version)]
pub struct Args {
    #[clap(subcommand)]
    command: Command,
}

fn main_inner() -> eyre::Result<()> {
    let args = Args::parse();

    #[cfg(feature = "tracing")]
    match &args.command {
        Command::Daemon {
            quiet, machine_id, ..
        } => {
            let name = "dora-daemon";
            let filename = machine_id
                .as_ref()
                .map(|id| format!("{name}-{id}"))
                .unwrap_or(name.to_string());
            set_up_tracing_opts(name, !quiet, Some(&filename))
                .context("failed to set up tracing subscriber")?;
        }
        Command::Runtime => {
            // Do not set the runtime in the cli.
        }
        Command::Coordinator { quiet, .. } => {
            let name = "dora-coordinator";
            set_up_tracing_opts(name, !quiet, Some(name))
                .context("failed to set up tracing subscriber")?;
        }
        _ => {
            set_up_tracing("dora-cli").context("failed to set up tracing subscriber")?;
        }
    };

    run(args.command)
}
