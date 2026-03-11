use crate::command::{Executable, default_tracing};
use crate::common::CoordinatorOptions;
use crate::common::connect_to_coordinator;
use crate::formatting::OutputFormat;
use crate::ws_client::WsSession;
use adora_core::descriptor::Descriptor;
use adora_core::descriptor::DescriptorExt;
use adora_message::{
    cli_to_coordinator::ControlRequest,
    coordinator_to_cli::{ControlRequestReply, DataflowStatus},
};
use eyre::{Context, bail};
use serde::Serialize;
use std::path::PathBuf;
use std::{
    io::{IsTerminal, Write},
    net::SocketAddr,
};
use termcolor::{Color, ColorChoice, ColorSpec, WriteColor};

#[derive(Debug, Clone, Serialize)]
pub struct StatusOutput {
    pub coordinator: ComponentStatus,
    pub daemon: ComponentStatus,
    pub active_dataflows: Option<usize>,
}

#[derive(Debug, Clone, Serialize)]
pub struct ComponentStatus {
    pub status: &'static str,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub address: Option<String>,
}

fn collect_status(coordinator_addr: SocketAddr) -> (StatusOutput, bool) {
    let mut error_occurred = false;

    let (coordinator, session) = match connect_to_coordinator(coordinator_addr) {
        Ok(session) => (
            ComponentStatus {
                status: "running",
                address: Some(coordinator_addr.to_string()),
            },
            Some(session),
        ),
        Err(_) => {
            error_occurred = true;
            (
                ComponentStatus {
                    status: "not running",
                    address: None,
                },
                None,
            )
        }
    };

    let daemon_running = session
        .as_ref()
        .map(daemon_running)
        .transpose()
        .ok()
        .flatten();

    let daemon = if daemon_running == Some(true) {
        ComponentStatus {
            status: "running",
            address: None,
        }
    } else if session.is_some() {
        error_occurred = true;
        ComponentStatus {
            status: "not running",
            address: None,
        }
    } else {
        error_occurred = true;
        ComponentStatus {
            status: "unknown",
            address: None,
        }
    };

    let active_dataflows = session
        .as_ref()
        .and_then(|s| query_running_dataflow_count(s).ok());

    let output = StatusOutput {
        coordinator,
        daemon,
        active_dataflows,
    };

    (output, error_occurred)
}

fn print_status_table(output: &StatusOutput) -> eyre::Result<()> {
    let color_choice = if std::io::stdout().is_terminal() {
        ColorChoice::Auto
    } else {
        ColorChoice::Never
    };
    let mut stdout = termcolor::StandardStream::stdout(color_choice);

    // Coordinator
    if output.coordinator.status == "running" {
        let _ = stdout.set_color(ColorSpec::new().set_fg(Some(Color::Green)));
        write!(stdout, "✓ ")?;
        let _ = stdout.reset();
        writeln!(stdout, "Coordinator: Running")?;
        if let Some(addr) = &output.coordinator.address {
            writeln!(stdout, "  Address: {addr}")?;
        }
    } else {
        let _ = stdout.set_color(ColorSpec::new().set_fg(Some(Color::Red)));
        write!(stdout, "✗ ")?;
        let _ = stdout.reset();
        writeln!(stdout, "Coordinator: Not running")?;
    }

    // Daemon
    match output.daemon.status {
        "running" => {
            let _ = stdout.set_color(ColorSpec::new().set_fg(Some(Color::Green)));
            write!(stdout, "✓ ")?;
            let _ = stdout.reset();
            writeln!(stdout, "Daemon: Running")?;
        }
        "not running" => {
            let _ = stdout.set_color(ColorSpec::new().set_fg(Some(Color::Red)));
            write!(stdout, "✗ ")?;
            let _ = stdout.reset();
            writeln!(stdout, "Daemon: Not running")?;
        }
        _ => {
            let _ = stdout.set_color(ColorSpec::new().set_fg(Some(Color::Red)));
            write!(stdout, "✗ ")?;
            let _ = stdout.reset();
            writeln!(stdout, "Daemon: Unknown (coordinator not available)")?;
        }
    }

    // Dataflow count
    if let Some(count) = output.active_dataflows {
        writeln!(stdout, "Active dataflows: {count}")?;
    }

    Ok(())
}

pub fn check_environment(coordinator_addr: SocketAddr, format: OutputFormat) -> eyre::Result<()> {
    let (output, error_occurred) = collect_status(coordinator_addr);

    match format {
        OutputFormat::Table => print_status_table(&output)?,
        OutputFormat::Json => {
            println!("{}", serde_json::to_string_pretty(&output)?);
        }
    }

    if error_occurred {
        bail!("System check failed.");
    }

    Ok(())
}

pub fn daemon_running(session: &WsSession) -> Result<bool, eyre::ErrReport> {
    let reply_raw = session
        .request(&serde_json::to_vec(&ControlRequest::DaemonConnected).unwrap())
        .wrap_err("failed to send DaemonConnected message")?;

    let reply = serde_json::from_slice(&reply_raw).wrap_err("failed to parse reply")?;
    let running = match reply {
        ControlRequestReply::DaemonConnected(running) => running,
        other => bail!("unexpected reply to daemon connection check: {other:?}"),
    };

    Ok(running)
}

fn query_running_dataflow_count(session: &WsSession) -> Result<usize, eyre::ErrReport> {
    let reply_raw = session
        .request(&serde_json::to_vec(&ControlRequest::List).unwrap())
        .wrap_err("failed to send List message")?;

    let reply: ControlRequestReply =
        serde_json::from_slice(&reply_raw).wrap_err("failed to parse reply")?;

    match reply {
        ControlRequestReply::DataflowList(list) => Ok(list
            .0
            .iter()
            .filter(|d| d.status == DataflowStatus::Running)
            .count()),
        other => bail!("unexpected reply to list request: {other:?}"),
    }
}

/// Check system health and connectivity to coordinator and daemon.
#[derive(Debug, clap::Args)]
pub struct Status {
    /// Path to the dataflow descriptor file (enables additional checks)
    #[clap(long, value_name = "PATH", value_hint = clap::ValueHint::FilePath)]
    dataflow: Option<PathBuf>,
    /// Output format (table or json)
    #[clap(long, short = 'f', value_name = "FORMAT", default_value_t = OutputFormat::Table)]
    format: OutputFormat,
    #[clap(flatten)]
    coordinator: CoordinatorOptions,
}

impl Executable for Status {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;

        let addr = self.coordinator.socket_addr();
        if let Some(dataflow) = self.dataflow {
            let working_dir = dataflow
                .canonicalize()
                .context("failed to canonicalize dataflow path")?
                .parent()
                .ok_or_else(|| eyre::eyre!("dataflow path has no parent dir"))?
                .to_owned();
            Descriptor::blocking_read(&dataflow)?.check(&working_dir)?;
        }

        check_environment(addr, self.format)
    }
}
