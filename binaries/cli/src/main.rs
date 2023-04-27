use std::path::PathBuf;

use attach::attach_dataflow;
use clap::Parser;
use communication_layer_request_reply::{RequestReplyLayer, TcpLayer, TcpRequestReplyConnection};
use dora_core::{
    descriptor::Descriptor,
    topics::{control_socket_addr, ControlRequest, ControlRequestReply, DataflowId},
};
#[cfg(feature = "tracing")]
use dora_tracing::set_up_tracing;
use eyre::{bail, Context};
use uuid::Uuid;

mod attach;
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

/// dora-rs cli client
#[derive(Debug, clap::Subcommand)]
enum Command {
    /// Check if the coordinator and the daemon is running.
    Check {
        #[clap(long)]
        dataflow: Option<PathBuf>,
    },
    /// Generate a visualization of the given graph using mermaid.js. Use --open to open browser.
    Graph {
        dataflow: PathBuf,
        #[clap(long, action)]
        mermaid: bool,
        #[clap(long, action)]
        open: bool,
    },
    /// Run build commands provided in the given dataflow.
    Build { dataflow: PathBuf },
    /// Generate a new project, node or operator. Choose the language between Rust, Python, C or C++.
    New {
        #[clap(flatten)]
        args: CommandNew,
        #[clap(hide = true, long)]
        internal_create_with_path_dependencies: bool,
    },
    /// Spawn a coordinator and a daemon.
    Up {
        #[clap(long)]
        config: Option<PathBuf>,
        #[clap(long)]
        coordinator_path: Option<PathBuf>,
        #[clap(long)]
        daemon_path: Option<PathBuf>,
    },
    /// Destroy running coordinator and daemon. If some dataflows are still running, they will be stopped first.
    Destroy {
        #[clap(long)]
        config: Option<PathBuf>,
    },
    /// Start the given dataflow path. Attach a name to the running dataflow by using --name.
    Start {
        dataflow: PathBuf,
        #[clap(long)]
        name: Option<String>,
        #[clap(long, action)]
        attach: bool,
        #[clap(long, action)]
        hot_reload: bool,
    },
    /// Stop the given dataflow UUID. If no id is provided, you will be able to choose between the running dataflows.
    Stop {
        uuid: Option<Uuid>,
        #[clap(long)]
        name: Option<String>,
    },
    /// List running dataflows.
    List,
    // Planned for future releases:
    // Dashboard,
    // Logs,
    // Metrics,
    // Stats,
    // Get,
    // Upgrade,
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
    #[cfg(feature = "tracing")]
    set_up_tracing("dora-cli").context("failed to set up tracing subscriber")?;
    let args = Args::parse();

    let mut session = None;

    match args.command {
        Command::Check { dataflow } => match dataflow {
            Some(dataflow) => {
                let working_dir = dataflow
                    .parent()
                    .ok_or_else(|| eyre::eyre!("dataflow path has no parent dir"))?;
                Descriptor::blocking_read(&dataflow)?.check(working_dir)?;
                check::check_environment()?
            }
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
        Command::New {
            args,
            internal_create_with_path_dependencies,
        } => template::create(args, internal_create_with_path_dependencies)?,
        Command::Up {
            config,
            coordinator_path,
            daemon_path,
        } => up::up(
            config.as_deref(),
            coordinator_path.as_deref(),
            daemon_path.as_deref(),
        )?,
        Command::Start {
            dataflow,
            name,
            attach,
            hot_reload,
        } => {
            let dataflow_descriptor =
                Descriptor::blocking_read(&dataflow).wrap_err("Failed to read yaml dataflow")?;
            let working_dir = dataflow
                .parent()
                .ok_or_else(|| eyre::eyre!("dataflow path has no parent dir"))?;
            dataflow_descriptor
                .check(working_dir)
                .wrap_err("Could not validate yaml")?;
            let dataflow_id = start_dataflow(
                dataflow_descriptor.clone(),
                name,
                working_dir.to_owned(),
                &mut session,
            )?;

            if attach {
                attach_dataflow(
                    dataflow_descriptor,
                    dataflow,
                    dataflow_id,
                    &mut session,
                    hot_reload,
                )?
            }
        }
        Command::List => list(&mut session)?,
        Command::Stop { uuid, name } => match (uuid, name) {
            (Some(uuid), _) => stop_dataflow(uuid, &mut session)?,
            (None, Some(name)) => stop_dataflow_by_name(name, &mut session)?,
            (None, None) => stop_dataflow_interactive(&mut session)?,
        },
        Command::Destroy { config } => up::destroy(config.as_deref(), &mut session)?,
    }

    Ok(())
}

fn start_dataflow(
    dataflow: Descriptor,
    name: Option<String>,
    local_working_dir: PathBuf,
    session: &mut Option<Box<TcpRequestReplyConnection>>,
) -> Result<Uuid, eyre::ErrReport> {
    let reply_raw = control_connection(session)?
        .request(
            &serde_json::to_vec(&ControlRequest::Start {
                dataflow,
                name,
                local_working_dir,
            })
            .unwrap(),
        )
        .wrap_err("failed to send start dataflow message")?;

    let result: ControlRequestReply =
        serde_json::from_slice(&reply_raw).wrap_err("failed to parse reply")?;
    match result {
        ControlRequestReply::DataflowStarted { uuid } => {
            eprintln!("{uuid}");
            Ok(uuid)
        }
        ControlRequestReply::Error(err) => bail!("{err}"),
        other => bail!("unexpected start dataflow reply: {other:?}"),
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
        stop_dataflow(selection.uuid, session)?;
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
    let result: ControlRequestReply =
        serde_json::from_slice(&reply_raw).wrap_err("failed to parse reply")?;
    match result {
        ControlRequestReply::DataflowStopped { uuid: _ } => Ok(()),
        ControlRequestReply::Error(err) => bail!("{err}"),
        other => bail!("unexpected stop dataflow reply: {other:?}"),
    }
}

fn stop_dataflow_by_name(
    name: String,
    session: &mut Option<Box<TcpRequestReplyConnection>>,
) -> Result<(), eyre::ErrReport> {
    let reply_raw = control_connection(session)?
        .request(&serde_json::to_vec(&ControlRequest::StopByName { name }).unwrap())
        .wrap_err("failed to send dataflow stop_by_name message")?;
    let result: ControlRequestReply =
        serde_json::from_slice(&reply_raw).wrap_err("failed to parse reply")?;
    match result {
        ControlRequestReply::DataflowStopped { uuid: _ } => Ok(()),
        ControlRequestReply::Error(err) => bail!("{err}"),
        other => bail!("unexpected stop dataflow reply: {other:?}"),
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
    let reply: ControlRequestReply =
        serde_json::from_slice(&reply_raw).wrap_err("failed to parse reply")?;
    let ids = match reply {
        ControlRequestReply::DataflowList { dataflows } => dataflows,
        ControlRequestReply::Error(err) => bail!("{err}"),
        other => bail!("unexpected list dataflow reply: {other:?}"),
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
