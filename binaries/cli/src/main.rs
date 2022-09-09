use clap::Parser;
use std::path::PathBuf;

mod graph;

#[derive(Debug, clap::Parser)]
#[clap(version)]
struct Args {
    #[clap(subcommand)]
    command: Command,
}

#[derive(Debug, clap::Subcommand)]
enum Command {
    Check,
    Graph { dataflow: PathBuf },
    Build,
    Templates,
    Dashboard,
    Start,
    Stop,
    Logs,
    Metrics,
    Stats,
    List,
    Get,
    Upgrade,
}

fn main() -> eyre::Result<()> {
    let args = Args::parse();

    match args.command {
        Command::Check => todo!(),
        Command::Graph { dataflow } => graph::run(dataflow)?,
        Command::Build => todo!(),
        Command::Templates => todo!(),
        Command::Dashboard => todo!(),
        Command::Start => todo!(),
        Command::Stop => todo!(),
        Command::Logs => todo!(),
        Command::Metrics => todo!(),
        Command::Stats => todo!(),
        Command::List => todo!(),
        Command::Get => todo!(),
        Command::Upgrade => todo!(),
    }

    Ok(())
}
