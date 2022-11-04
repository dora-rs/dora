use clap::Parser;
use dora_core::topics::{
    StartDataflowResult, StopDataflowResult, ZENOH_CONTROL_LIST, ZENOH_CONTROL_START,
    ZENOH_CONTROL_STOP,
};
use eyre::{bail, eyre, Context};
use std::{ops::Deref, path::PathBuf, sync::Arc};
use zenoh::{
    prelude::{Receiver, Selector, SplitBuffer},
    sync::ZFuture,
};

mod build;
mod check;
mod graph;
mod template;
mod up;

#[derive(Debug, clap::Parser)]
#[clap(version)]
struct Args {
    #[clap(subcommand)]
    command: Command,
}

#[derive(Debug, clap::Subcommand)]
enum Command {
    Check {
        #[clap(long)]
        dataflow: Option<PathBuf>,
        #[clap(long)]
        runtime_path: Option<PathBuf>,
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
    Up {
        #[clap(long)]
        config: Option<PathBuf>,
        #[clap(long)]
        roudi_path: Option<PathBuf>,
        #[clap(long)]
        coordinator_path: Option<PathBuf>,
    },
    Destroy,
    Start {
        dataflow: PathBuf,
    },
    Stop {
        uuid: String,
    },
    Logs,
    Metrics,
    Stats,
    List,
    Get,
    Upgrade,
}

#[derive(Debug, clap::Args)]
pub struct CommandNew {
    #[clap(long, value_enum, default_value_t = Kind::Dataflow)]
    kind: Kind,
    #[clap(long, value_enum, default_value_t = Lang::Rust)]
    lang: Lang,
    name: String,
    path: Option<PathBuf>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, clap::ValueEnum)]
enum Kind {
    Dataflow,
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
        } => match dataflow {
            Some(dataflow) => check::check_dataflow(&dataflow, runtime_path.as_deref())?,
            None => check::check_environment()?,
        },
        Command::Graph {
            dataflow,
            mermaid,
            open,
        } => {
            graph::create(dataflow, mermaid, open)?;
        }
        Command::Build { dataflow } => {
            build::build(&dataflow)?;
        }
        Command::New { args } => template::create(args)?,
        Command::Dashboard => todo!(),
        Command::Up {
            config,
            roudi_path,
            coordinator_path,
        } => up::up(
            config.as_deref(),
            roudi_path.as_deref(),
            coordinator_path.as_deref(),
        )?,
        Command::Start { dataflow } => start_dataflow(dataflow, &mut session)?,
        Command::List => list(&mut session)?,
        Command::Stop { uuid } => stop_dataflow(uuid, &mut session)?,
        Command::Destroy => up::destroy(&mut session)?,
        Command::Logs => todo!(),
        Command::Metrics => todo!(),
        Command::Stats => todo!(),
        Command::Get => todo!(),
        Command::Upgrade => todo!(),
    }

    Ok(())
}

fn start_dataflow(
    dataflow: PathBuf,
    session: &mut Option<Arc<zenoh::Session>>,
) -> Result<(), eyre::ErrReport> {
    let canonicalized = dataflow
        .canonicalize()
        .wrap_err("given dataflow file does not exist")?;
    let path = canonicalized
        .to_str()
        .ok_or_else(|| eyre!("dataflow path must be valid UTF-8"))?;
    let reply_receiver = zenoh_control_session(session)?
        .get(Selector {
            key_selector: ZENOH_CONTROL_START.into(),
            value_selector: path.into(),
        })
        .wait()
        .map_err(|err| eyre!(err))
        .wrap_err("failed to create publisher for start dataflow message")?;
    let reply = reply_receiver
        .recv()
        .wrap_err("failed to receive reply from coordinator")?;
    let raw = reply.sample.value.payload.contiguous();
    let result: StartDataflowResult =
        serde_json::from_slice(&raw).wrap_err("failed to parse reply")?;
    match result {
        StartDataflowResult::Ok { uuid } => {
            println!("{uuid}");
            Ok(())
        }
        StartDataflowResult::Error(err) => bail!(err),
    }
}

fn stop_dataflow(
    uuid: String,
    session: &mut Option<Arc<zenoh::Session>>,
) -> Result<(), eyre::ErrReport> {
    let reply_receiver = zenoh_control_session(session)?
        .get(Selector {
            key_selector: ZENOH_CONTROL_STOP.into(),
            value_selector: uuid.as_str().into(),
        })
        .wait()
        .map_err(|err| eyre!(err))
        .wrap_err("failed to create publisher for start dataflow message")?;
    let reply = reply_receiver
        .recv()
        .wrap_err("failed to receive reply from coordinator")?;
    let raw = reply.sample.value.payload.contiguous();
    let result: StopDataflowResult =
        serde_json::from_slice(&raw).wrap_err("failed to parse reply")?;
    match result {
        StopDataflowResult::Ok => Ok(()),
        StopDataflowResult::Error(err) => bail!(err),
    }
}

fn list(session: &mut Option<Arc<zenoh::Session>>) -> Result<(), eyre::ErrReport> {
    let reply_receiver = zenoh_control_session(session)?
        .get(ZENOH_CONTROL_LIST)
        .wait()
        .map_err(|err| eyre!(err))
        .wrap_err("failed to create publisher for list message")?;
    let reply = reply_receiver
        .recv()
        .wrap_err("failed to receive reply from coordinator")?;
    let raw = reply.sample.value.payload.contiguous();
    let reply_string = std::str::from_utf8(raw.deref()).wrap_err("reply is not valid UTF8")?;
    println!("{reply_string}");

    Ok(())
}

fn zenoh_control_session(
    session: &mut Option<Arc<zenoh::Session>>,
) -> eyre::Result<&Arc<zenoh::Session>> {
    Ok(match session {
        Some(session) => session,
        None => session.insert(
            zenoh::open(zenoh::config::Config::default())
                .wait()
                .map_err(|err| eyre!(err))?
                .into_arc(),
        ),
    })
}
