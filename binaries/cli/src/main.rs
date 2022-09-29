use clap::Parser;
use std::path::PathBuf;

mod build;
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
    Build { dataflow: PathBuf },
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
        Command::Graph { dataflow } => {
            let visualized = graph::visualize_as_mermaid(dataflow)?;
            println!("{visualized}");
            println!(
                "Paste the above output on https://mermaid.live/ or in a \
                ```mermaid code block on GitHub to display it."
            );
        }
        Command::Build { dataflow } => {
            build::build(&dataflow)?;
        }
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
