use attach::attach_dataflow;
use colored::Colorize;
use communication_layer_request_reply::{RequestReplyLayer, TcpLayer, TcpRequestReplyConnection};
use dora_coordinator::Event;
use dora_core::{
    descriptor::{source_is_url, Descriptor, DescriptorExt},
    topics::{
        DORA_COORDINATOR_PORT_CONTROL_DEFAULT, DORA_COORDINATOR_PORT_DEFAULT,
        DORA_DAEMON_LOCAL_LISTEN_PORT_DEFAULT,
    },
};
use dora_daemon::Daemon;
use dora_download::download_file;
use dora_message::{
    cli_to_coordinator::ControlRequest,
    coordinator_to_cli::{ControlRequestReply, DataflowList, DataflowResult, DataflowStatus},
};
#[cfg(feature = "tracing")]
use dora_tracing::set_up_tracing;
use dora_tracing::{set_up_tracing_opts, FileLogging};
use duration_str::parse;
use eyre::{bail, Context};
use formatting::FormatDataflowError;
use std::{env::current_dir, io::Write, net::SocketAddr};
use std::{
    net::{IpAddr, Ipv4Addr},
    path::PathBuf,
    time::Duration,
};
use tabwriter::TabWriter;
use tokio::runtime::Builder;
use tracing::level_filters::LevelFilter;
use uuid::Uuid;

mod attach;
mod build;
mod check;
mod formatting;
mod graph;
mod logs;
mod template;
mod up;

const LOCALHOST: IpAddr = IpAddr::V4(Ipv4Addr::new(127, 0, 0, 1));
const LISTEN_WILDCARD: IpAddr = IpAddr::V4(Ipv4Addr::new(0, 0, 0, 0));

#[derive(Debug, clap::Parser)]
#[clap(version)]
pub struct Args {
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
        #[clap(value_name = "PATH")]
        dataflow: String,
        // Use UV to build nodes.
        #[clap(long, action)]
        uv: bool,
    },
    /// Generate a new project or node. Choose the language between Rust, Python, C or C++.
    New {
        #[clap(flatten)]
        args: CommandNew,
        #[clap(hide = true, long)]
        internal_create_with_path_dependencies: bool,
    },
    /// Run a dataflow locally.
    ///
    /// Directly runs the given dataflow without connecting to a dora
    /// coordinator or daemon. The dataflow is executed on the local machine.
    Run {
        /// Path to the dataflow descriptor file
        #[clap(value_name = "PATH")]
        dataflow: String,
        // Use UV to run nodes.
        #[clap(long, action)]
        uv: bool,
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
        #[clap(value_name = "PATH")]
        dataflow: String,
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
        /// Run the dataflow in background
        #[clap(long, action)]
        detach: bool,
        /// Enable hot reloading (Python only)
        #[clap(long, action)]
        hot_reload: bool,
        // Use UV to run nodes.
        #[clap(long, action)]
        uv: bool,
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
        /// Local listen port for event such as dynamic node.
        #[clap(long, default_value_t = DORA_DAEMON_LOCAL_LISTEN_PORT_DEFAULT)]
        local_listen_port: u16,
        /// Address and port number of the dora coordinator
        #[clap(long, short, default_value_t = LOCALHOST)]
        coordinator_addr: IpAddr,
        /// Port number of the coordinator control server
        #[clap(long, default_value_t = DORA_COORDINATOR_PORT_DEFAULT)]
        coordinator_port: u16,
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
    /// Dora CLI self-management commands
    Self_ {
        #[clap(subcommand)]
        command: SelfSubCommand,
    },
}

#[derive(Debug, clap::Subcommand)]
enum SelfSubCommand {
    /// Check for updates or update the CLI
    Update {
        /// Only check for updates without installing
        #[clap(long)]
        check_only: bool,
    },
    /// Remove The Dora CLI from the system
    Uninstall {
        /// Force uninstallation without confirmation
        #[clap(long)]
        force: bool,
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
    Node,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, clap::ValueEnum)]
enum Lang {
    Rust,
    Python,
    C,
    Cxx,
}

pub fn lib_main(args: Args) {
    if let Err(err) = run(args) {
        eprintln!("\n\n{}", "[ERROR]".bold().red());
        eprintln!("{err:?}");
        std::process::exit(1);
    }
}

fn run(args: Args) -> eyre::Result<()> {
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
            let stdout = (!quiet).then_some("info,zenoh=warn");
            let file = Some(FileLogging {
                file_name: filename,
                filter: LevelFilter::INFO,
            });
            set_up_tracing_opts(name, stdout, file)
                .context("failed to set up tracing subscriber")?;
        }
        Command::Runtime => {
            // Do not set the runtime in the cli.
        }
        Command::Coordinator { quiet, .. } => {
            let name = "dora-coordinator";
            let stdout = (!quiet).then_some("info");
            let file = Some(FileLogging {
                file_name: name.to_owned(),
                filter: LevelFilter::INFO,
            });
            set_up_tracing_opts(name, stdout, file)
                .context("failed to set up tracing subscriber")?;
        }
        Command::Run { .. } => {
            let log_level = std::env::var("RUST_LOG").ok().or(Some("info".to_string()));
            set_up_tracing_opts("run", log_level.as_deref(), None)
                .context("failed to set up tracing subscriber")?;
        }
        _ => {
            set_up_tracing("dora-cli").context("failed to set up tracing subscriber")?;
        }
    };

    let log_level = env_logger::Builder::new()
        .filter_level(log::LevelFilter::Info)
        .parse_default_env()
        .build()
        .filter();

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
        Command::Build { dataflow, uv } => {
            build::build(dataflow, uv)?;
        }
        Command::New {
            args,
            internal_create_with_path_dependencies,
        } => template::create(args, internal_create_with_path_dependencies)?,
        Command::Run { dataflow, uv } => {
            let dataflow_path = resolve_dataflow(dataflow).context("could not resolve dataflow")?;
            let rt = Builder::new_multi_thread()
                .enable_all()
                .build()
                .context("tokio runtime failed")?;
            let result = rt.block_on(Daemon::run_dataflow(&dataflow_path, uv))?;
            handle_dataflow_result(result, None)?
        }
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
                let active = list.get_active();
                let uuid = match &active[..] {
                    [] => bail!("No dataflows are running"),
                    [uuid] => uuid.clone(),
                    _ => inquire::Select::new("Choose dataflow to show logs:", active).prompt()?,
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
            detach,
            hot_reload,
            uv,
        } => {
            let dataflow = resolve_dataflow(dataflow).context("could not resolve dataflow")?;
            let dataflow_descriptor =
                Descriptor::blocking_read(&dataflow).wrap_err("Failed to read yaml dataflow")?;
            let working_dir = dataflow
                .canonicalize()
                .context("failed to canonicalize dataflow path")?
                .parent()
                .ok_or_else(|| eyre::eyre!("dataflow path has no parent dir"))?
                .to_owned();

            let coordinator_socket = (coordinator_addr, coordinator_port).into();
            let mut session = connect_to_coordinator(coordinator_socket)
                .wrap_err("failed to connect to dora coordinator")?;
            let dataflow_id = start_dataflow(
                dataflow_descriptor.clone(),
                name,
                working_dir,
                &mut *session,
                uv,
            )?;

            let attach = match (attach, detach) {
                (true, true) => eyre::bail!("both `--attach` and `--detach` are given"),
                (true, false) => true,
                (false, true) => false,
                (false, false) => {
                    println!("attaching to dataflow (use `--detach` to run in background)");
                    true
                }
            };

            if attach {
                attach_dataflow(
                    dataflow_descriptor,
                    dataflow,
                    dataflow_id,
                    &mut *session,
                    hot_reload,
                    coordinator_socket,
                    log_level,
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
            coordinator_port,
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
                        if coordinator_addr != LOCALHOST {
                            tracing::info!(
                                "Not using coordinator addr {} as `run_dataflow` is for local dataflow only. Please use the `start` command for remote coordinator",
                                coordinator_addr
                            );
                        }

                        let result = Daemon::run_dataflow(&dataflow_path, false).await?;
                        handle_dataflow_result(result, None)
                    }
                    None => {
                        Daemon::run(SocketAddr::new(coordinator_addr, coordinator_port), machine_id, local_listen_port).await
                    }
                }
            })
            .context("failed to run dora-daemon")?
        }
        Command::Runtime => dora_runtime::main().context("Failed to run dora-runtime")?,
        Command::Self_ { command } => match command {
            SelfSubCommand::Update { check_only } => {
                println!("Checking for updates...");

                #[cfg(target_os = "linux")]
                let bin_path_in_archive = format!("dora-cli-{}/dora", env!("TARGET"));
                #[cfg(target_os = "macos")]
                let bin_path_in_archive = format!("dora-cli-{}/dora", env!("TARGET"));
                #[cfg(target_os = "windows")]
                let bin_path_in_archive = String::from("dora.exe");

                let status = self_update::backends::github::Update::configure()
                    .repo_owner("dora-rs")
                    .repo_name("dora")
                    .bin_path_in_archive(&bin_path_in_archive)
                    .bin_name("dora")
                    .show_download_progress(true)
                    .current_version(env!("CARGO_PKG_VERSION"))
                    .build()?;

                if check_only {
                    // Only check if an update is available
                    match status.get_latest_release() {
                        Ok(release) => {
                            let current_version = self_update::cargo_crate_version!();
                            if current_version != release.version {
                                println!(
                                    "An update is available: {}. Run 'dora self update' to update",
                                    release.version
                                );
                            } else {
                                println!(
                                    "Dora CLI is already at the latest version: {}",
                                    current_version
                                );
                            }
                        }
                        Err(e) => println!("Failed to check for updates: {}", e),
                    }
                } else {
                    // Perform the actual update
                    match status.update() {
                        Ok(update_status) => match update_status {
                            self_update::Status::UpToDate(version) => {
                                println!("Dora CLI is already at the latest version: {}", version);
                            }
                            self_update::Status::Updated(version) => {
                                println!("Successfully updated Dora CLI to version: {}", version);
                            }
                        },
                        Err(e) => println!("Failed to update: {}", e),
                    }
                }
            }
            SelfSubCommand::Uninstall { force } => {
                if !force {
                    let confirmed =
                        inquire::Confirm::new("Are you sure you want to uninstall Dora CLI?")
                            .with_default(false)
                            .prompt()
                            .wrap_err("Uninstallation cancelled")?;

                    if !confirmed {
                        println!("Uninstallation cancelled");
                        return Ok(());
                    }
                }

                println!("Uninstalling Dora CLI...");
                #[cfg(feature = "python")]
                {
                    println!("Detected Python installation...");

                    // Try uv pip uninstall first
                    let uv_status = std::process::Command::new("uv")
                        .args(["pip", "uninstall", "dora-rs-cli"])
                        .status();

                    if let Ok(status) = uv_status {
                        if status.success() {
                            println!("Dora CLI has been successfully uninstalled via uv pip.");
                            return Ok(());
                        }
                    }

                    // Fall back to regular pip uninstall
                    println!("Trying with pip...");
                    let status = std::process::Command::new("pip")
                        .args(["uninstall", "-y", "dora-rs-cli"])
                        .status()
                        .wrap_err("Failed to run pip uninstall")?;

                    if status.success() {
                        println!("Dora CLI has been successfully uninstalled via pip.");
                    } else {
                        bail!("Failed to uninstall Dora CLI via pip.");
                    }
                }
                #[cfg(not(feature = "python"))]
                {
                    match self_replace::self_delete() {
                        Ok(_) => {
                            println!("Dora CLI has been successfully uninstalled.");
                        }
                        Err(e) => {
                            bail!("Failed to uninstall Dora CLI: {}", e);
                        }
                    }
                }
            }
        },
    };

    Ok(())
}

fn start_dataflow(
    dataflow: Descriptor,
    name: Option<String>,
    local_working_dir: PathBuf,
    session: &mut TcpRequestReplyConnection,
    uv: bool,
) -> Result<Uuid, eyre::ErrReport> {
    let reply_raw = session
        .request(
            &serde_json::to_vec(&ControlRequest::Start {
                dataflow,
                name,
                local_working_dir,
                uv,
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
    let active = list.get_active();
    if active.is_empty() {
        eprintln!("No dataflows are running");
    } else {
        let selection = inquire::Select::new("Choose dataflow to stop:", active).prompt()?;
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
        ControlRequestReply::DataflowStopped { uuid, result } => {
            handle_dataflow_result(result, Some(uuid))
        }
        ControlRequestReply::Error(err) => bail!("{err}"),
        other => bail!("unexpected stop dataflow reply: {other:?}"),
    }
}

fn handle_dataflow_result(result: DataflowResult, uuid: Option<Uuid>) -> Result<(), eyre::Error> {
    if result.is_ok() {
        Ok(())
    } else {
        Err(match uuid {
            Some(uuid) => {
                eyre::eyre!("Dataflow {uuid} failed:\n{}", FormatDataflowError(&result))
            }
            None => {
                eyre::eyre!("Dataflow failed:\n{}", FormatDataflowError(&result))
            }
        })
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
        ControlRequestReply::DataflowStopped { uuid, result } => {
            handle_dataflow_result(result, Some(uuid))
        }
        ControlRequestReply::Error(err) => bail!("{err}"),
        other => bail!("unexpected stop dataflow reply: {other:?}"),
    }
}

fn list(session: &mut TcpRequestReplyConnection) -> Result<(), eyre::ErrReport> {
    let list = query_running_dataflows(session)?;

    let mut tw = TabWriter::new(vec![]);
    tw.write_all(b"UUID\tName\tStatus\n")?;
    for entry in list.0 {
        let uuid = entry.id.uuid;
        let name = entry.id.name.unwrap_or_default();
        let status = match entry.status {
            DataflowStatus::Running => "Running",
            DataflowStatus::Finished => "Succeeded",
            DataflowStatus::Failed => "Failed",
        };
        tw.write_all(format!("{uuid}\t{name}\t{status}\n").as_bytes())?;
    }
    tw.flush()?;
    let formatted = String::from_utf8(tw.into_inner()?)?;

    println!("{formatted}");

    Ok(())
}

fn query_running_dataflows(session: &mut TcpRequestReplyConnection) -> eyre::Result<DataflowList> {
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

fn resolve_dataflow(dataflow: String) -> eyre::Result<PathBuf> {
    let dataflow = if source_is_url(&dataflow) {
        // try to download the shared library
        let target_path = current_dir().context("Could not access the current dir")?;
        let rt = Builder::new_current_thread()
            .enable_all()
            .build()
            .context("tokio runtime failed")?;
        rt.block_on(async { download_file(&dataflow, &target_path).await })
            .wrap_err("failed to download dataflow yaml file")?
    } else {
        PathBuf::from(dataflow)
    };
    Ok(dataflow)
}

#[cfg(feature = "python")]
use clap::Parser;
#[cfg(feature = "python")]
use pyo3::{
    pyfunction, pymodule,
    types::{PyModule, PyModuleMethods},
    wrap_pyfunction, Bound, PyResult, Python,
};

#[cfg(feature = "python")]
#[pyfunction]
fn py_main(_py: Python) -> PyResult<()> {
    pyo3::prepare_freethreaded_python();
    // Skip first argument as it is a python call.
    let args = std::env::args_os().skip(1).collect::<Vec<_>>();

    match Args::try_parse_from(args) {
        Ok(args) => lib_main(args),
        Err(err) => {
            eprintln!("{err}");
        }
    }
    Ok(())
}

/// A Python module implemented in Rust.
#[cfg(feature = "python")]
#[pymodule]
fn dora_cli(_py: Python, m: Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(py_main, &m)?)?;
    m.add("__version__", env!("CARGO_PKG_VERSION"))?;
    Ok(())
}
