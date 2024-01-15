use std::{net::Ipv4Addr, path::PathBuf};

use attach::attach_dataflow;
use clap::Parser;
use communication_layer_request_reply::{RequestReplyLayer, TcpLayer, TcpRequestReplyConnection};
use dora_coordinator::Event;
use dora_core::{
    descriptor::Descriptor,
    topics::{
        control_socket_addr, ControlRequest, ControlRequestReply, DataflowId,
        DORA_COORDINATOR_PORT_DEFAULT,
    },
};
use dora_daemon::Daemon;
#[cfg(feature = "tracing")]
use dora_tracing::set_up_tracing;
use eyre::{bail, Context};
use std::net::SocketAddr;
use tokio::runtime::Builder;
use uuid::Uuid;

mod attach;
mod build;
mod check;
mod graph;
mod logs;
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
    /// Show logs of a given dataflow and node.
    Logs { dataflow: String, node: String },
    // Metrics,
    // Stats,
    // Get,
    // Upgrade,
    /// Run daemon
    Daemon {
        #[clap(long)]
        machine_id: Option<String>,
        #[clap(long)]
        coordinator_addr: Option<SocketAddr>,

        #[clap(long)]
        run_dataflow: Option<PathBuf>,
    },
    /// Run runtime
    Runtime,
    /// Run coordinator
    Coordinator { port: Option<u16> },
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

fn main() {
    if let Err(err) = run() {
        eprintln!("{err:#}");
        std::process::exit(1);
    }
}

fn run() -> eyre::Result<()> {
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

    match args.command {
        Command::Check { dataflow } => match dataflow {
            Some(dataflow) => {
                let working_dir = dataflow
                    .canonicalize()
                    .context("failed to canonicalize dataflow path")?
                    .parent()
                    .ok_or_else(|| eyre::eyre!("dataflow path has no parent dir"))?
                    .to_owned();
                Descriptor::blocking_read(&dataflow)?.check(&working_dir)?;
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
        Command::Up { config } => up::up(config.as_deref())?,
        Command::Logs { dataflow, node } => {
            let uuid = Uuid::parse_str(&dataflow).ok();
            let name = if uuid.is_some() { None } else { Some(dataflow) };
            let mut session =
                connect_to_coordinator().wrap_err("failed to connect to dora coordinator")?;
            logs::logs(&mut *session, uuid, name, node)?
        }
        Command::Start {
            dataflow,
            name,
            attach,
            hot_reload,
        } => {
            let dataflow_descriptor =
                Descriptor::blocking_read(&dataflow).wrap_err("Failed to read yaml dataflow")?;
            let working_dir = dataflow
                .canonicalize()
                .context("failed to canonicalize dataflow path")?
                .parent()
                .ok_or_else(|| eyre::eyre!("dataflow path has no parent dir"))?
                .to_owned();
            dataflow_descriptor
                .check(&working_dir)
                .wrap_err("Could not validate yaml")?;
            let mut session =
                connect_to_coordinator().wrap_err("failed to connect to dora coordinator")?;
            let dataflow_id = start_dataflow(
                dataflow_descriptor.clone(),
                name,
                working_dir,
                &mut *session,
            )?;

            if attach {
                attach_dataflow(
                    dataflow_descriptor,
                    dataflow,
                    dataflow_id,
                    &mut *session,
                    hot_reload,
                )?
            }
        }
        Command::List => match connect_to_coordinator() {
            Ok(mut session) => list(&mut *session)?,
            Err(_) => {
                bail!("No dora coordinator seems to be running.");
            }
        },
        Command::Stop { uuid, name } => {
            let mut session =
                connect_to_coordinator().wrap_err("could not connect to dora coordinator")?;
            match (uuid, name) {
                (Some(uuid), _) => stop_dataflow(uuid, &mut *session)?,
                (None, Some(name)) => stop_dataflow_by_name(name, &mut *session)?,
                (None, None) => stop_dataflow_interactive(&mut *session)?,
            }
        }
        Command::Destroy { config } => up::destroy(config.as_deref())?,
        Command::Coordinator { port } => {
            let rt = Builder::new_multi_thread()
                .enable_all()
                .build()
                .context("tokio runtime failed")?;
            rt.block_on(async {
                let (_port, task) =
                    dora_coordinator::start(port, futures::stream::empty::<Event>()).await?;
                task.await
            })
            .context("failed to run dora-coordinator")?
        }
        Command::Daemon {
            coordinator_addr,
            machine_id,
            run_dataflow,
        } => {
            let rt = Builder::new_multi_thread()
                .enable_all()
                .build()
                .context("tokio runtime failed")?;
            rt.block_on(async {
                match run_dataflow {
                    Some(dataflow_path) => {
                        tracing::info!("Starting dataflow `{}`", dataflow_path.display());

                        Daemon::run_dataflow(
                            &dataflow_path,
                            coordinator_addr,
                            machine_id.unwrap_or_default(),
                        )
                        .await
                    }
                    None => {
                        let addr = coordinator_addr.unwrap_or_else(|| {
                            tracing::info!("Starting in local mode");
                            let localhost = Ipv4Addr::new(127, 0, 0, 1);
                            (localhost, DORA_COORDINATOR_PORT_DEFAULT).into()
                        });
                        Daemon::run(addr, machine_id.unwrap_or_default()).await
                    }
                }
            })
            .context("failed to run dora-daemon")?
        }
        Command::Runtime => dora_runtime::main().context("Failed to run dora-runtime")?,
    };

    Ok(())
}

fn start_dataflow(
    dataflow: Descriptor,
    name: Option<String>,
    local_working_dir: PathBuf,
    session: &mut TcpRequestReplyConnection,
) -> Result<Uuid, eyre::ErrReport> {
    let reply_raw = session
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

fn stop_dataflow_interactive(session: &mut TcpRequestReplyConnection) -> eyre::Result<()> {
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
    session: &mut TcpRequestReplyConnection,
) -> Result<(), eyre::ErrReport> {
    let reply_raw = session
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
        ControlRequestReply::DataflowStopped { uuid: _, result } => result
            .map_err(|err| eyre::eyre!(err))
            .wrap_err("dataflow failed"),
        ControlRequestReply::Error(err) => bail!("{err}"),
        other => bail!("unexpected stop dataflow reply: {other:?}"),
    }
}

fn stop_dataflow_by_name(
    name: String,
    session: &mut TcpRequestReplyConnection,
) -> Result<(), eyre::ErrReport> {
    let reply_raw = session
        .request(&serde_json::to_vec(&ControlRequest::StopByName { name }).unwrap())
        .wrap_err("failed to send dataflow stop_by_name message")?;
    let result: ControlRequestReply =
        serde_json::from_slice(&reply_raw).wrap_err("failed to parse reply")?;
    match result {
        ControlRequestReply::DataflowStopped { uuid: _, result } => result
            .map_err(|err| eyre::eyre!(err))
            .wrap_err("dataflow failed"),
        ControlRequestReply::Error(err) => bail!("{err}"),
        other => bail!("unexpected stop dataflow reply: {other:?}"),
    }
}

fn list(session: &mut TcpRequestReplyConnection) -> Result<(), eyre::ErrReport> {
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
    session: &mut TcpRequestReplyConnection,
) -> Result<Vec<DataflowId>, eyre::ErrReport> {
    let reply_raw = session
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

fn connect_to_coordinator() -> std::io::Result<Box<TcpRequestReplyConnection>> {
    TcpLayer::new().connect(control_socket_addr())
}
