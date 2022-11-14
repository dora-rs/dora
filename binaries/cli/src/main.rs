use clap::Parser;
use communication_layer_request_reply::{RequestReplyLayer, TcpLayer, TcpRequestReplyConnection};
use dora_core::topics::{
    control_socket_addr, ControlRequest, StartDataflowResult, StopDataflowResult,
};
use eyre::{bail, Context};
use std::path::PathBuf;
use uuid::Uuid;

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
    Destroy {
        #[clap(long)]
        config: Option<PathBuf>,
    },
    Start {
        dataflow: PathBuf,
    },
    Stop {
        uuid: Option<Uuid>,
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
        Command::Stop { uuid } => match uuid {
            Some(uuid) => stop_dataflow(uuid, &mut session)?,
            None => stop_dataflow_interactive(&mut session)?,
        },
        Command::Destroy { config } => up::destroy(config.as_deref(), &mut session)?,
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
    session: &mut Option<Box<TcpRequestReplyConnection>>,
) -> Result<(), eyre::ErrReport> {
    let canonicalized = dataflow
        .canonicalize()
        .wrap_err("given dataflow file does not exist")?;
    let reply_raw = control_connection(session)?
        .request(
            &serde_json::to_vec(&ControlRequest::Start {
                dataflow_path: canonicalized,
            })
            .unwrap(),
        )
        .wrap_err("failed to send start dataflow message")?;

    let result: StartDataflowResult =
        serde_json::from_slice(&reply_raw).wrap_err("failed to parse reply")?;
    match result {
        StartDataflowResult::Ok { uuid } => {
            println!("{uuid}");
            Ok(())
        }
        StartDataflowResult::Error(err) => bail!(err),
    }
}

fn stop_dataflow_interactive(
    session: &mut Option<Box<TcpRequestReplyConnection>>,
) -> eyre::Result<()> {
    let uuids = query_running_dataflows(session).wrap_err("failed to query running dataflows")?;
    if uuids.is_empty() {
        eprintln!("No dataflows are running");
    } else {
        let selection = inquire::Select::new("Choose dataflow to stop:", uuids).prompt()?;
        stop_dataflow(selection, session)?;
    }

    Ok(())
}

fn stop_dataflow(
    uuid: Uuid,
    session: &mut Option<Box<TcpRequestReplyConnection>>,
) -> Result<(), eyre::ErrReport> {
    let reply_raw = control_connection(session)?
        .request(
            &serde_json::to_vec(&ControlRequest::Stop {
                dataflow_uuid: uuid,
            })
            .unwrap(),
        )
        .wrap_err("failed to send dataflow stop message")?;
    let result: StopDataflowResult =
        serde_json::from_slice(&reply_raw).wrap_err("failed to parse reply")?;
    match result {
        StopDataflowResult::Ok => Ok(()),
        StopDataflowResult::Error(err) => bail!(err),
    }
}

fn list(session: &mut Option<Box<TcpRequestReplyConnection>>) -> Result<(), eyre::ErrReport> {
    let uuids = query_running_dataflows(session)?;

    if uuids.is_empty() {
        eprintln!("No dataflows are running");
    } else {
        println!("Running dataflows:");
        for uuid in uuids {
            println!("- {uuid}");
        }
    }

    Ok(())
}

fn query_running_dataflows(
    session: &mut Option<Box<TcpRequestReplyConnection>>,
) -> Result<Vec<Uuid>, eyre::ErrReport> {
    let reply_raw = control_connection(session)?
        .request(&serde_json::to_vec(&ControlRequest::List).unwrap())
        .wrap_err("failed to send list message")?;
    let reply_string = std::str::from_utf8(&reply_raw).wrap_err("reply is not valid UTF8")?;

    let uuids = reply_string
        .lines()
        .map(Uuid::try_from)
        .collect::<Result<_, _>>()
        .wrap_err("failed to parse UUIDs returned by coordinator")?;

    Ok(uuids)
}

fn control_connection(
    session: &mut Option<Box<TcpRequestReplyConnection>>,
) -> eyre::Result<&mut Box<TcpRequestReplyConnection>> {
    Ok(match session {
        Some(session) => session,
        None => session.insert(TcpLayer::new().connect(control_socket_addr())?),
    })
}
