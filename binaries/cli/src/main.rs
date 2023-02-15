use clap::Parser;
use communication_layer_request_reply::{RequestReplyLayer, TcpLayer, TcpRequestReplyConnection};
use dora_core::topics::{
    control_socket_addr, ControlRequest, DataflowId, ListDataflowResult, StartDataflowResult,
    StopDataflowResult,
};
use duration_str::parse;
use eyre::{bail, Context};
use std::{path::PathBuf, time::Duration};
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
        coordinator_path: Option<PathBuf>,
        #[clap(long)]
        daemon_path: Option<PathBuf>,
    },
    Destroy {
        #[clap(long)]
        config: Option<PathBuf>,
    },
    Start {
        dataflow: PathBuf,
        #[clap(long)]
        name: Option<String>,
    },
    Stop {
        uuid: Option<Uuid>,
        #[clap(long)]
        name: Option<String>,
        #[arg(value_parser = parse)]
        grace_period: Option<Duration>,
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
            coordinator_path,
            daemon_path,
        } => up::up(
            config.as_deref(),
            coordinator_path.as_deref(),
            daemon_path.as_deref(),
        )?,
        Command::Start { dataflow, name } => start_dataflow(dataflow, name, &mut session)?,
        Command::List => list(&mut session)?,
        Command::Stop {
            uuid,
            name,
            grace_period,
        } => match (uuid, name) {
            (Some(uuid), _) => stop_dataflow(uuid, grace_period, &mut session)?,
            (None, Some(name)) => stop_dataflow_by_name(name, grace_period, &mut session)?,
            (None, None) => stop_dataflow_interactive(&mut session)?,
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
    name: Option<String>,
    session: &mut Option<Box<TcpRequestReplyConnection>>,
) -> Result<(), eyre::ErrReport> {
    let canonicalized = dataflow
        .canonicalize()
        .wrap_err("given dataflow file does not exist")?;
    let reply_raw = control_connection(session)?
        .request(
            &serde_json::to_vec(&ControlRequest::Start {
                dataflow_path: canonicalized,
                name,
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
        let uuid_selection = inquire::Select::new("Choose dataflow to stop:", uuids).prompt()?;
        let force_selection = inquire::Confirm::new("Kill the node after a grace period:")
            .with_default(false)
            .prompt()?;

        let grace_period = if force_selection {
            let grace_period = inquire::CustomType::<String>::new("How long is the grace period:")
                .with_validator(|resp: &String| {
                    if parse(resp).is_ok() {
                        Ok(inquire::validator::Validation::Valid)
                    } else {
                        Ok(inquire::validator::Validation::Invalid(
                            "You must give a period with a unit like: 1ms, 2sec, 5 minutes".into(),
                        ))
                    }
                })
                .prompt()?;
            Some(parse(&grace_period)?)
        } else {
            None
        };

        stop_dataflow(uuid_selection.uuid, grace_period, session)?;
    }

    Ok(())
}

fn stop_dataflow(
    uuid: Uuid,
    grace_period: Option<Duration>,
    session: &mut Option<Box<TcpRequestReplyConnection>>,
) -> Result<(), eyre::ErrReport> {
    let reply_raw = control_connection(session)?
        .request(
            &serde_json::to_vec(&ControlRequest::Stop {
                dataflow_uuid: uuid,
                grace_period,
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

fn stop_dataflow_by_name(
    name: String,
    grace_period: Option<Duration>,
    session: &mut Option<Box<TcpRequestReplyConnection>>,
) -> Result<(), eyre::ErrReport> {
    let reply_raw = control_connection(session)?
        .request(&serde_json::to_vec(&ControlRequest::StopByName { name, grace_period }).unwrap())
        .wrap_err("failed to send dataflow stop_by_name message")?;
    let result: StopDataflowResult =
        serde_json::from_slice(&reply_raw).wrap_err("failed to parse reply")?;
    match result {
        StopDataflowResult::Ok => Ok(()),
        StopDataflowResult::Error(err) => bail!(err),
    }
}

fn list(session: &mut Option<Box<TcpRequestReplyConnection>>) -> Result<(), eyre::ErrReport> {
    let ids = query_running_dataflows(session)?;

    if ids.is_empty() {
        eprintln!("No dataflows are running");
    } else {
        println!("Running dataflows:");
        for id in ids {
            println!("- {id}");
        }
    }

    Ok(())
}

fn query_running_dataflows(
    session: &mut Option<Box<TcpRequestReplyConnection>>,
) -> Result<Vec<DataflowId>, eyre::ErrReport> {
    let reply_raw = control_connection(session)?
        .request(&serde_json::to_vec(&ControlRequest::List).unwrap())
        .wrap_err("failed to send list message")?;
    let reply: ListDataflowResult =
        serde_json::from_slice(&reply_raw).wrap_err("failed to parse reply")?;
    let ids = match reply {
        ListDataflowResult::Ok { dataflows } => dataflows,
        ListDataflowResult::Error(err) => bail!(err),
    };

    Ok(ids)
}

fn control_connection(
    session: &mut Option<Box<TcpRequestReplyConnection>>,
) -> eyre::Result<&mut Box<TcpRequestReplyConnection>> {
    Ok(match session {
        Some(session) => session,
        None => session.insert(TcpLayer::new().connect(control_socket_addr())?),
    })
}
