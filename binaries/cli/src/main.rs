use std::{
    fs::File,
    io::Write,
    net::{IpAddr, SocketAddr},
    path::PathBuf,
    time::Duration,
};

use clap::Parser;
use colored::Colorize;
use dora_cli::{check_environment, template, DoraConnection, LISTEN_WILDCARD, LOCALHOST};
use dora_core::{
    descriptor::Descriptor,
    topics::{
        DataflowResult, NodeErrorCause, DORA_COORDINATOR_PORT_CONTROL_DEFAULT,
        DORA_COORDINATOR_PORT_DEFAULT, DORA_DAEMON_LOCAL_LISTEN_PORT_DEFAULT,
    },
};
use dora_daemon::Daemon;
#[cfg(feature = "tracing")]
use dora_tracing::set_up_tracing;
use dora_tracing::set_up_tracing_opts;
use eyre::{bail, Context};
use tabwriter::TabWriter;
use tracing::info;
use uuid::Uuid;

fn main() {
    if let Err(err) = main_inner() {
        eprintln!("\n\n{}", "[ERROR]".bold().red());
        eprintln!("{err:#}");
        std::process::exit(1);
    }
}

fn main_inner() -> eyre::Result<()> {
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

    let dora_cli_path =
        std::env::current_exe().wrap_err("failed to get current executable path")?;
    run(args.command, dora_cli_path)
}

#[derive(Debug, clap::Parser)]
#[clap(version)]
pub struct Args {
    #[clap(subcommand)]
    command: Command,
}

/// dora-rs cli client
#[derive(Debug, clap::Subcommand)]
pub enum Command {
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
    /// Generate a new project or node. Choose the language between Rust, Python, C or C++.
    New {
        #[clap(flatten)]
        args: template::CreateArgs,
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
        /// Run the dataflow in background
        #[clap(long, action)]
        detach: bool,
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
        #[arg(value_parser = duration_str::parse)]
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

#[derive(Debug, Clone, Copy, PartialEq, Eq, clap::ValueEnum)]
enum Kind {
    Dataflow,
    CustomNode,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, clap::ValueEnum)]
enum Lang {
    Rust,
    Python,
    C,
    Cxx,
}

pub fn run(command: Command, dora_cli_path: PathBuf) -> eyre::Result<()> {
    let log_level = env_logger::Builder::new()
        .filter_level(log::LevelFilter::Info)
        .parse_default_env()
        .build()
        .filter();

    match command {
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
                check_environment((coordinator_addr, coordinator_port).into())?
            }
            None => check_environment((coordinator_addr, coordinator_port).into())?,
        },
        Command::Graph {
            dataflow,
            mermaid,
            open,
        } => {
            create_dataflow_graph(dataflow, mermaid, open)?;
        }
        Command::Build { dataflow } => {
            dora_cli::build_dataflow(&dataflow)?;
        }
        Command::New {
            args,
            internal_create_with_path_dependencies,
        } => template::create(args, internal_create_with_path_dependencies)?,
        Command::Up { config } => {
            dora_cli::up(config.as_deref(), &dora_cli_path)?;
        }
        Command::Logs {
            dataflow,
            node,
            coordinator_addr,
            coordinator_port,
        } => {
            let mut session = DoraConnection::connect((coordinator_addr, coordinator_port).into())
                .wrap_err("failed to connect to dora coordinator")?;
            let list = session
                .query_running_dataflows()
                .wrap_err("failed to query running dataflows")?;
            if let Some(dataflow) = dataflow {
                let uuid = Uuid::parse_str(&dataflow).ok();
                let name = if uuid.is_some() { None } else { Some(dataflow) };
                session.dataflow_logs(uuid, name, node)?
            } else {
                let active = list.get_active();
                let uuid = match &active[..] {
                    [] => bail!("No dataflows are running"),
                    [uuid] => uuid.clone(),
                    _ => inquire::Select::new("Choose dataflow to show logs:", active).prompt()?,
                };
                session.dataflow_logs(Some(uuid.uuid), None, node)?
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

            let coordinator_socket = (coordinator_addr, coordinator_port).into();
            let mut session = DoraConnection::connect(coordinator_socket)
                .wrap_err("failed to connect to dora coordinator")?;
            let dataflow_id =
                session.start_dataflow(dataflow_descriptor.clone(), name, working_dir)?;
            eprintln!("{dataflow_id}");

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
                let result = session.attach_to_dataflow(
                    dataflow_descriptor,
                    dataflow,
                    dataflow_id,
                    hot_reload,
                    coordinator_socket,
                    log_level,
                    &mut std::io::stdout(),
                )?;
                info!("dataflow {} stopped", result.uuid);
                handle_dataflow_result(result)?;
            }
        }
        Command::List {
            coordinator_addr,
            coordinator_port,
        } => match DoraConnection::connect((coordinator_addr, coordinator_port).into()) {
            Ok(mut session) => list_dataflows(&mut session)?,
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
            let mut session = DoraConnection::connect((coordinator_addr, coordinator_port).into())
                .wrap_err("could not connect to dora coordinator")?;
            let result = match (uuid, name) {
                (Some(uuid), _) => Some(session.stop_dataflow(uuid, grace_duration)?),
                (None, Some(name)) => Some(session.stop_dataflow_by_name(name, grace_duration)?),
                (None, None) => stop_dataflow_interactive(grace_duration, &mut session)?,
            };
            if let Some(result) = result {
                handle_dataflow_result(result)?;
            }
        }
        Command::Destroy {
            config: _,
            coordinator_addr,
            coordinator_port,
        } => destroy((coordinator_addr, coordinator_port).into())?,
        Command::Coordinator {
            interface,
            port,
            control_interface,
            control_port,
            quiet,
        } => {
            let rt = tokio::runtime::Builder::new_multi_thread()
                .enable_all()
                .build()
                .context("tokio runtime failed")?;
            rt.block_on(async {
                let bind = SocketAddr::new(interface, port);
                let bind_control = SocketAddr::new(control_interface, control_port);
                let (port, task) = dora_coordinator::start(
                    bind,
                    bind_control,
                    futures::stream::empty::<dora_coordinator::Event>(),
                )
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
            let rt = tokio::runtime::Builder::new_multi_thread()
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

                        let result = Daemon::run_dataflow(&dataflow_path, dora_cli_path.to_owned()).await?;
                        handle_dataflow_result(result)
                    }
                    None => {
                        if coordinator_addr.ip() == LOCALHOST {
                            tracing::info!("Starting in local mode");
                        }
                        Daemon::run(coordinator_addr, machine_id.unwrap_or_default(), inter_daemon_addr, local_listen_port, dora_cli_path.to_owned()).await
                    }
                }
            })
            .context("failed to run dora-daemon")?
        }
        Command::Runtime => dora_runtime::main().context("Failed to run dora-runtime")?,
    };

    Ok(())
}

fn create_dataflow_graph(
    dataflow: std::path::PathBuf,
    mermaid: bool,
    open: bool,
) -> eyre::Result<()> {
    if mermaid {
        let visualized = dora_cli::visualize_as_mermaid(&dataflow)?;
        println!("{visualized}");
        println!(
            "Paste the above output on https://mermaid.live/ or in a \
            ```mermaid code block on GitHub to display it."
        );
    } else {
        let html = dora_cli::visualize_as_html(&dataflow)?;

        let working_dir = std::env::current_dir().wrap_err("failed to get current working dir")?;
        let graph_filename = match dataflow.file_stem().and_then(|n| n.to_str()) {
            Some(name) => format!("{name}-graph"),
            None => "graph".into(),
        };
        let mut extra = 0;
        let path = loop {
            let adjusted_file_name = if extra == 0 {
                format!("{graph_filename}.html")
            } else {
                format!("{graph_filename}.{extra}.html")
            };
            let path = working_dir.join(&adjusted_file_name);
            if path.exists() {
                extra += 1;
            } else {
                break path;
            }
        };

        let mut file = File::create(&path).context("failed to create graph HTML file")?;
        file.write_all(html.as_bytes())?;

        println!(
            "View graph by opening the following in your browser:\n  file://{}",
            path.display()
        );

        if open {
            webbrowser::open(path.as_os_str().to_str().unwrap())?;
        }
    }
    Ok(())
}

fn list_dataflows(session: &mut DoraConnection) -> Result<(), eyre::ErrReport> {
    let list = session.query_running_dataflows()?;

    let mut tw = TabWriter::new(vec![]);
    tw.write_all(b"UUID\tName\tStatus\n")?;
    for entry in list.0 {
        let uuid = entry.id.uuid;
        let name = entry.id.name.unwrap_or_default();
        let status = match entry.status {
            dora_core::topics::DataflowStatus::Running => "Running",
            dora_core::topics::DataflowStatus::Finished => "Succeeded",
            dora_core::topics::DataflowStatus::Failed => "Failed",
        };
        tw.write_all(format!("{uuid}\t{name}\t{status}\n").as_bytes())?;
    }
    tw.flush()?;
    let formatted = String::from_utf8(tw.into_inner()?)?;

    println!("{formatted}");

    Ok(())
}

fn stop_dataflow_interactive(
    grace_duration: Option<Duration>,
    session: &mut DoraConnection,
) -> eyre::Result<Option<DataflowResult>> {
    let list = session
        .query_running_dataflows()
        .wrap_err("failed to query running dataflows")?;
    let active = list.get_active();
    if active.is_empty() {
        eprintln!("No dataflows are running");
        Ok(None)
    } else {
        let selection = inquire::Select::new("Choose dataflow to stop:", active).prompt()?;
        Ok(Some(session.stop_dataflow(selection.uuid, grace_duration)?))
    }
}

pub fn destroy(coordinator_addr: SocketAddr) -> Result<(), eyre::ErrReport> {
    match DoraConnection::connect(coordinator_addr) {
        Ok(session) => {
            session.destroy()?;
            println!("Send destroy command to dora-coordinator");
        }
        Err(_) => {
            eprintln!("Could not connect to dora-coordinator");
        }
    }

    Ok(())
}

fn handle_dataflow_result(result: dora_core::topics::DataflowResult) -> Result<(), eyre::Error> {
    if result.is_ok() {
        Ok(())
    } else {
        Err(eyre::eyre!(
            "Dataflow {} failed:\n{}",
            result.uuid,
            FormatDataflowError(&result)
        ))
    }
}

struct FormatDataflowError<'a>(pub &'a DataflowResult);

impl std::fmt::Display for FormatDataflowError<'_> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f)?;
        let failed = self
            .0
            .node_results
            .iter()
            .filter_map(|(id, r)| r.as_ref().err().map(|e| (id, e)));
        let total_failed = failed.clone().count();

        let mut non_cascading: Vec<_> = failed
            .clone()
            .filter(|(_, e)| !matches!(e.cause, NodeErrorCause::Cascading { .. }))
            .collect();
        non_cascading.sort_by_key(|(_, e)| e.timestamp);
        // try to print earliest non-cascading error
        let hidden = if !non_cascading.is_empty() {
            let printed = non_cascading.len();
            for (id, err) in non_cascading {
                writeln!(f, "Node `{id}` failed: {err}")?;
            }
            total_failed - printed
        } else {
            // no non-cascading errors -> print earliest cascading
            let mut all: Vec<_> = failed.collect();
            all.sort_by_key(|(_, e)| e.timestamp);
            if let Some((id, err)) = all.first() {
                write!(f, "Node `{id}` failed: {err}")?;
                total_failed - 1
            } else {
                write!(f, "unknown error")?;
                0
            }
        };

        if hidden > 1 {
            write!(
                f,
                "\n\nThere are {hidden} consequential errors. Check the `out/{}` folder for full details.",
                self.0.uuid
            )?;
        }

        Ok(())
    }
}
