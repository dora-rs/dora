use clap::Parser;
use dora_cli::Command;
#[cfg(feature = "tracing")]
use dora_tracing::set_up_tracing;
use eyre::Context;

fn main() {
    if let Err(err) = main_inner() {
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
    match args.command {
        Command::Daemon { .. } => {
            set_up_tracing("dora-daemon").context("failed to set up tracing subscriber")?;
        }
        Command::Runtime => {
            // Do not set the runtime in the cli.
        }
        Command::Coordinator { .. } => {
            set_up_tracing("dora-coordinator").context("failed to set up tracing subscriber")?;
        }
        _ => {
            set_up_tracing("dora-cli").context("failed to set up tracing subscriber")?;
        }
    };

    dora_cli::run(args.command)
}
