use clap::Parser;
use communication_layer_pub_sub::{zenoh::ZenohCommunicationLayer, CommunicationLayer};
use dora_core::topics::{
    ZENOH_CONTROL_PREFIX, ZENOH_CONTROL_START_DATAFLOW, ZENOH_CONTROL_STOP_ALL,
};
use eyre::{eyre, Context};
use std::{io::Write, path::PathBuf};
use tempfile::NamedTempFile;

mod build;
mod check;
mod graph;
mod template;

#[derive(Debug, clap::Parser)]
#[clap(version)]
struct Args {
    #[clap(subcommand)]
    command: Command,
}

#[derive(Debug, clap::Subcommand)]
enum Command {
    Check {
        dataflow: PathBuf,
        runtime_path: PathBuf,
    },
    Graph {
        dataflow: PathBuf,
        #[clap(long, action)]
        mermaid: bool,
        #[clap(long, action)]
        open: bool,
    },
    Build {
        dataflow: PathBuf,
    },
    New {
        #[clap(flatten)]
        args: CommandNew,
    },
    Dashboard,
    Up,
    Destroy,
    Start,
    Stop,
    Logs,
    Metrics,
    Stats,
    List,
    Get,
    Upgrade,
}

#[derive(Debug, clap::Args)]
pub struct CommandNew {
    #[clap(long, value_enum, default_value_t = Kind::Operator)]
    kind: Kind,
    #[clap(long, value_enum, default_value_t = Lang::Rust)]
    lang: Lang,
    name: String,
    path: Option<PathBuf>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, clap::ValueEnum)]
enum Kind {
    Operator,
    CustomNode,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, clap::ValueEnum)]
enum Lang {
    Rust,
    Python,
    C,
    Cxx,
}

fn main() -> eyre::Result<()> {
    let args = Args::parse();

    let mut session = None;

    match args.command {
        Command::Check {
            dataflow,
            runtime_path,
        } => check::check(&dataflow, &runtime_path)?,
        Command::Graph {
            dataflow,
            mermaid,
            open,
        } => {
            if mermaid {
                let visualized = graph::visualize_as_mermaid(&dataflow)?;
                println!("{visualized}");
                println!(
                    "Paste the above output on https://mermaid.live/ or in a \
                    ```mermaid code block on GitHub to display it."
                );
            } else {
                let html = graph::visualize_as_html(&dataflow)?;
                let mut file = NamedTempFile::new().context("failed to create temp file")?;
                file.as_file_mut().write_all(html.as_bytes())?;

                let path = file.path().to_owned();
                file.keep()?;

                println!(
                    "View graph by opening the following in your browser:\n  file://{}",
                    path.display()
                );

                if open {
                    webbrowser::open(path.as_os_str().to_str().unwrap())?;
                }
            }
        }
        Command::Build { dataflow } => {
            build::build(&dataflow)?;
        }
        Command::New { args } => template::create(args)?,
        Command::Dashboard => todo!(),
        Command::Up => todo!(),
        Command::Start => todo!(),
        Command::Stop => todo!(),
        Command::Destroy => {
            let publisher = zenoh_control_session(&mut session)?
                .publisher(ZENOH_CONTROL_STOP_ALL)
                .map_err(|err| eyre!(err))
                .wrap_err("failed to create publisher for stop message")?;
            publisher
                .publish(&[])
                .map_err(|err| eyre!(err))
                .wrap_err("failed to publish stop message")?;
        }
        Command::Logs => todo!(),
        Command::Metrics => todo!(),
        Command::Stats => todo!(),
        Command::List => todo!(),
        Command::Get => todo!(),
        Command::Upgrade => todo!(),
    }

    Ok(())
}

fn zenoh_control_session(
    session: &mut Option<ZenohCommunicationLayer>,
) -> eyre::Result<&mut ZenohCommunicationLayer> {
    Ok(match session {
        Some(session) => session,
        None => session.insert(
            ZenohCommunicationLayer::init(Default::default(), ZENOH_CONTROL_PREFIX.into())
                .map_err(|err| eyre!(err))
                .wrap_err("failed to open zenoh control session")?,
        ),
    })
}
