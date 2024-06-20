use attach::attach_dataflow;
use clap::Parser;
use communication_layer_request_reply::{RequestReplyLayer, TcpLayer, TcpRequestReplyConnection};
use dora_coordinator::Event;
use dora_core::{
    descriptor::Descriptor,
    topics::{
        ControlRequest, ControlRequestReply, DataflowList, DORA_COORDINATOR_PORT_CONTROL_DEFAULT,
        DORA_COORDINATOR_PORT_DEFAULT, DORA_DAEMON_LOCAL_LISTEN_PORT_DEFAULT,
    },
};
use dora_daemon::Daemon;
#[cfg(feature = "tracing")]
use dora_tracing::set_up_tracing;
use dora_tracing::set_up_tracing_opts;
use duration_str::parse;
use eyre::{bail, Context};
use std::{io::Write, net::SocketAddr};
use std::{
    net::{IpAddr, Ipv4Addr},
    path::PathBuf,
    time::Duration,
};
use tabwriter::TabWriter;
use tokio::runtime::Builder;
use uuid::Uuid;

mod attach;
mod build;
mod check;
mod graph;
mod logs;
mod template;
mod up;

const LOCALHOST: IpAddr = IpAddr::V4(Ipv4Addr::new(127, 0, 0, 1));
const LISTEN_WILDCARD: IpAddr = IpAddr::V4(Ipv4Addr::new(0, 0, 0, 0));

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
        /// Path to the dataflow descriptor file (enables additional checks)
        #[clap(long, value_name = "PATH", value_hint = clap::ValueHint::FilePath)]
        dataflow: Option<PathBuf>,
        /// Address of the dora coordinator
        #[clap(long, value_name = "IP", default_value_t = LOCALHOST)]
        coordinator_addr: IpAddr,
        /// Port number of the coordinator control server
        #[clap(long, value_name = "PORT", default_value_t = DORA_COORDINATOR_PORT_CONTROL_DEFAULT)]
        coordinator_port: u16,
    },
    /// Generate a visualization of the given graph using mermaid.js. Use --open to open browser.
    Graph {
        /// Path to the dataflow descriptor file
        #[clap(value_name = "PATH", value_hint = clap::ValueHint::FilePath)]
        dataflow: PathBuf,
        /// Visualize the dataflow as a Mermaid diagram (instead of HTML)
        #[clap(long, action)]
        mermaid: bool,
        /// Open the HTML visualization in the browser
        #[clap(long, action)]
        open: bool,
    },
    /// Run build commands provided in the given dataflow.
    Build {
        /// Path to the dataflow descriptor file
        #[clap(value_name = "PATH", value_hint = clap::ValueHint::FilePath)]
        dataflow: PathBuf,
    },
    /// Generate a new project, node or operator. Choose the language between Rust, Python, C or C++.
    New {
        #[clap(flatten)]
        args: CommandNew,
        #[clap(hide = true, long)]
        internal_create_with_path_dependencies: bool,
    },
    /// Spawn coordinator and daemon in local mode (with default config)
    Up {
        /// Use a custom configuration
        #[clap(long, hide = true, value_name = "PATH", value_hint = clap::ValueHint::FilePath)]
        config: Option<PathBuf>,
    },
    /// Destroy running coordinator and daemon. If some dataflows are still running, they will be stopped first.
    Destroy {
        /// Use a custom configuration
        #[clap(long, hide = true)]
        config: Option<PathBuf>,
        /// Address of the dora coordinator
        #[clap(long, value_name = "IP", default_value_t = LOCALHOST)]
        coordinator_addr: IpAddr,
        /// Port number of the coordinator control server
        #[clap(long, value_name = "PORT", default_value_t = DORA_COORDINATOR_PORT_CONTROL_DEFAULT)]
        coordinator_port: u16,
    },
    /// Start the given dataflow path. Attach a name to the running dataflow by using --name.
    Start {
        /// Path to the dataflow descriptor file
        #[clap(value_name = "PATH", value_hint = clap::ValueHint::FilePath)]
        dataflow: PathBuf,
        /// Assign a name to the dataflow
        #[clap(long)]
        name: Option<String>,
        /// Address of the dora coordinator
        #[clap(long, value_name = "IP", default_value_t = LOCALHOST)]
        coordinator_addr: IpAddr,
        /// Port number of the coordinator control server
        #[clap(long, value_name = "PORT", default_value_t = DORA_COORDINATOR_PORT_CONTROL_DEFAULT)]
        coordinator_port: u16,
        /// Attach to the dataflow and wait for its completion
        #[clap(long, action)]
        attach: bool,
        /// Enable hot reloading (Python only)
        #[clap(long, action)]
        hot_reload: bool,
    },
    /// Stop the given dataflow UUID. If no id is provided, you will be able to choose between the running dataflows.
    Stop {
        /// UUID of the dataflow that should be stopped
        uuid: Option<Uuid>,
        /// Name of the dataflow that should be stopped
        #[clap(long)]
        name: Option<String>,
        /// Kill the dataflow if it doesn't stop after the given duration
        #[clap(long, value_name = "DURATION")]
        #[arg(value_parser = parse)]
        grace_duration: Option<Duration>,
        /// Address of the dora coordinator
        #[clap(long, value_name = "IP", default_value_t = LOCALHOST)]
        coordinator_addr: IpAddr,
        /// Port number of the coordinator control server
        #[clap(long, value_name = "PORT", default_value_t = DORA_COORDINATOR_PORT_CONTROL_DEFAULT)]
        coordinator_port: u16,
    },
    /// List running dataflows.
    List {
        /// Address of the dora coordinator
        #[clap(long, value_name = "IP", default_value_t = LOCALHOST)]
        coordinator_addr: IpAddr,
        /// Port number of the coordinator control server
        #[clap(long, value_name = "PORT", default_value_t = DORA_COORDINATOR_PORT_CONTROL_DEFAULT)]
        coordinator_port: u16,
    },
    // Planned for future releases:
    // Dashboard,
    /// Show logs of a given dataflow and node.
    #[command(allow_missing_positional = true)]
    Logs {
        /// Identifier of the dataflow
        #[clap(value_name = "UUID_OR_NAME")]
        dataflow: Option<String>,
        /// Show logs for the given node
        #[clap(value_name = "NAME")]
        node: String,
        /// Address of the dora coordinator
        #[clap(long, value_name = "IP", default_value_t = LOCALHOST)]
        coordinator_addr: IpAddr,
        /// Port number of the coordinator control server
        #[clap(long, value_name = "PORT", default_value_t = DORA_COORDINATOR_PORT_CONTROL_DEFAULT)]
        coordinator_port: u16,
    },
    // Metrics,
    // Stats,
    // Get,
    // Upgrade,
    /// Run daemon
    Daemon {
        /// Unique identifier for the machine (required for distributed dataflows)
        #[clap(long)]
        machine_id: Option<String>,
        /// The inter daemon IP address and port this daemon will bind to.
        #[clap(long, default_value_t = SocketAddr::new(LISTEN_WILDCARD, 0))]
        inter_daemon_addr: SocketAddr,
        /// Local listen port for event such as dynamic node.
        #[clap(long, default_value_t = DORA_DAEMON_LOCAL_LISTEN_PORT_DEFAULT)]
        local_listen_port: u16,
        /// Address and port number of the dora coordinator
        #[clap(long, default_value_t = SocketAddr::new(LOCALHOST, DORA_COORDINATOR_PORT_DEFAULT))]
        coordinator_addr: SocketAddr,
        #[clap(long, hide = true)]
        run_dataflow: Option<PathBuf>,
        /// Suppresses all log output to stdout.
        #[clap(long)]
        quiet: bool,
    },
    /// Run runtime
    Runtime,
    /// Run coordinator
    Coordinator {
        /// Network interface to bind to for daemon communication
        #[clap(long, default_value_t = LISTEN_WILDCARD)]
        interface: IpAddr,
        /// Port number to bind to for daemon communication
        #[clap(long, default_value_t = DORA_COORDINATOR_PORT_DEFAULT)]
        port: u16,
        /// Network interface to bind to for control communication
        #[clap(long, default_value_t = LISTEN_WILDCARD)]
        control_interface: IpAddr,
        /// Port number to bind to for control communication
        #[clap(long, default_value_t = DORA_COORDINATOR_PORT_CONTROL_DEFAULT)]
        control_port: u16,
        /// Suppresses all log output to stdout.
        #[clap(long)]
        quiet: bool,
    },
}

#[derive(Debug, clap::Args)]
pub struct CommandNew {
    /// The entity that should be created
    #[clap(long, value_enum, default_value_t = Kind::Dataflow)]
    kind: Kind,
    /// The programming language that should be used
    #[clap(long, value_enum, default_value_t = Lang::Rust)]
    lang: Lang,
    /// Desired name of the entity
    name: String,
    /// Where to create the entity
    #[clap(hide = true)]
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

    match args.command {
        Command::Check {
            dataflow,
            coordinator_addr,
            coordinator_port,
        } => match dataflow {
            Some(dataflow) => {
                let working_dir = dataflow
                    .canonicalize()
                    .context("failed to canonicalize dataflow path")?
                    .parent()
                    .ok_or_else(|| eyre::eyre!("dataflow path has no parent dir"))?
                    .to_owned();
                Descriptor::blocking_read(&dataflow)?.check(&working_dir)?;
                check::check_environment((coordinator_addr, coordinator_port).into())?
            }
            None => check::check_environment((coordinator_addr, coordinator_port).into())?,
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
        Command::Up { config } => {
            up::up(config.as_deref())?;
        }
        Command::Logs {
            dataflow,
            node,
            coordinator_addr,
            coordinator_port,
        } => {
            let mut session = connect_to_coordinator((coordinator_addr, coordinator_port).into())
                .wrap_err("failed to connect to dora coordinator")?;
            let list = query_running_dataflows(&mut *session)
                .wrap_err("failed to query running dataflows")?;
            if let Some(dataflow) = dataflow {
                let uuid = Uuid::parse_str(&dataflow).ok();
                let name = if uuid.is_some() { None } else { Some(dataflow) };
                logs::logs(&mut *session, uuid, name, node)?
            } else {
                let uuid = match &list.active[..] {
                    [] => bail!("No dataflows are running"),
                    [uuid] => uuid.clone(),
                    _ => inquire::Select::new("Choose dataflow to show logs:", list.active)
                        .prompt()?,
                };
                logs::logs(&mut *session, Some(uuid.uuid), None, node)?
            }
        }
        Command::Start {
            dataflow,
            name,
            coordinator_addr,
            coordinator_port,
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
            if !coordinator_addr.is_loopback() {
                dataflow_descriptor.check_in_daemon(&working_dir, &[], true)?;
            } else {
                dataflow_descriptor
                    .check(&working_dir)
                    .wrap_err("Could not validate yaml")?;
            }

            let mut session = connect_to_coordinator((coordinator_addr, coordinator_port).into())
                .wrap_err("failed to connect to dora coordinator")?;
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
        Command::List {
            coordinator_addr,
            coordinator_port,
        } => match connect_to_coordinator((coordinator_addr, coordinator_port).into()) {
            Ok(mut session) => list(&mut *session)?,
            Err(_) => {
                bail!("No dora coordinator seems to be running.");
            }
        },
        Command::Stop {
            uuid,
            name,
            grace_duration,
            coordinator_addr,
            coordinator_port,
        } => {
            let mut session = connect_to_coordinator((coordinator_addr, coordinator_port).into())
                .wrap_err("could not connect to dora coordinator")?;
            match (uuid, name) {
                (Some(uuid), _) => stop_dataflow(uuid, grace_duration, &mut *session)?,
                (None, Some(name)) => stop_dataflow_by_name(name, grace_duration, &mut *session)?,
                (None, None) => stop_dataflow_interactive(grace_duration, &mut *session)?,
            }
        }
        Command::Destroy {
            config,
            coordinator_addr,
            coordinator_port,
        } => up::destroy(
            config.as_deref(),
            (coordinator_addr, coordinator_port).into(),
        )?,
        Command::Coordinator {
            interface,
            port,
            control_interface,
            control_port,
            quiet,
        } => {
            let rt = Builder::new_multi_thread()
                .enable_all()
                .build()
                .context("tokio runtime failed")?;
            rt.block_on(async {
                let bind = SocketAddr::new(interface, port);
                let bind_control = SocketAddr::new(control_interface, control_port);
                let (port, task) =
                    dora_coordinator::start(bind, bind_control, futures::stream::empty::<Event>())
                        .await?;
                if !quiet {
                    println!("Listening for incoming daemon connection on {port}");
                }
                task.await
            })
            .context("failed to run dora-coordinator")?
        }
        Command::Daemon {
            coordinator_addr,
            inter_daemon_addr,
            local_listen_port,
            machine_id,
            run_dataflow,
            quiet: _,
        } => {
            let rt = Builder::new_multi_thread()
                .enable_all()
                .build()
                .context("tokio runtime failed")?;
            rt.block_on(async {
                match run_dataflow {
                    Some(dataflow_path) => {
                        tracing::info!("Starting dataflow `{}`", dataflow_path.display());
                        if coordinator_addr != SocketAddr::new(LOCALHOST, DORA_COORDINATOR_PORT_DEFAULT){
                            tracing::info!(
                                "Not using coordinator addr {} as `run_dataflow` is for local dataflow only. Please use the `start` command for remote coordinator",
                                coordinator_addr
                            );
                        }

                        Daemon::run_dataflow(&dataflow_path).await
                    }
                    None => {
                        if coordinator_addr.ip() == LOCALHOST {
                            tracing::info!("Starting in local mode");
                        }
                        Daemon::run(coordinator_addr, machine_id.unwrap_or_default(), inter_daemon_addr, local_listen_port).await
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

fn stop_dataflow_interactive(
    grace_duration: Option<Duration>,
    session: &mut TcpRequestReplyConnection,
) -> eyre::Result<()> {
    let list = query_running_dataflows(session).wrap_err("failed to query running dataflows")?;
    if list.active.is_empty() {
        eprintln!("No dataflows are running");
    } else {
        let selection = inquire::Select::new("Choose dataflow to stop:", list.active).prompt()?;
        stop_dataflow(selection.uuid, grace_duration, session)?;
    }

    Ok(())
}

fn stop_dataflow(
    uuid: Uuid,
    grace_duration: Option<Duration>,
    session: &mut TcpRequestReplyConnection,
) -> Result<(), eyre::ErrReport> {
    let reply_raw = session
        .request(
            &serde_json::to_vec(&ControlRequest::Stop {
                dataflow_uuid: uuid,
                grace_duration,
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
    grace_duration: Option<Duration>,
    session: &mut TcpRequestReplyConnection,
) -> Result<(), eyre::ErrReport> {
    let reply_raw = session
        .request(
            &serde_json::to_vec(&ControlRequest::StopByName {
                name,
                grace_duration,
            })
            .unwrap(),
        )
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
    let list = query_running_dataflows(session)?;

    let mut tw = TabWriter::new(vec![]);
    tw.write_all(b"UUID\tName\tStatus\n")?;

    for id in list.active {
        tw.write_all(
            format!(
                "{}\t{}\trunning\n",
                id.uuid,
                id.name.as_deref().unwrap_or_default()
            )
            .as_bytes(),
        )?;
    }

    for id in list.failed {
        tw.write_all(
            format!(
                "{}\t{}\tFAILED\n",
                id.uuid,
                id.name.as_deref().unwrap_or_default()
            )
            .as_bytes(),
        )?;
    }

    for id in list.finished {
        tw.write_all(
            format!(
                "{}\t{}\tfinished\n",
                id.uuid,
                id.name.as_deref().unwrap_or_default()
            )
            .as_bytes(),
        )?;
    }

    tw.flush()?;
    let formatted = String::from_utf8(tw.into_inner()?)?;
    println!("{formatted}");

    Ok(())
}

fn query_running_dataflows(
    session: &mut TcpRequestReplyConnection,
) -> Result<DataflowList, eyre::ErrReport> {
    let reply_raw = session
        .request(&serde_json::to_vec(&ControlRequest::List).unwrap())
        .wrap_err("failed to send list message")?;
    let reply: ControlRequestReply =
        serde_json::from_slice(&reply_raw).wrap_err("failed to parse reply")?;
    let ids = match reply {
        ControlRequestReply::DataflowList(list) => list,
        ControlRequestReply::Error(err) => bail!("{err}"),
        other => bail!("unexpected list dataflow reply: {other:?}"),
    };

    Ok(ids)
}

fn connect_to_coordinator(
    coordinator_addr: SocketAddr,
) -> std::io::Result<Box<TcpRequestReplyConnection>> {
    TcpLayer::new().connect(coordinator_addr)
}
