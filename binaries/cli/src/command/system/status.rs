use crate::command::{Executable, default_tracing};
use crate::common::CoordinatorOptions;
use crate::common::{connect_to_coordinator, coordinator_rejected};
use crate::common::{expect_reply, send_control_request};
use crate::formatting::OutputFormat;
use crate::ws_client::WsSession;
use dora_core::descriptor::Descriptor;
use dora_core::descriptor::DescriptorExt;
use dora_message::{cli_to_coordinator::ControlRequest, coordinator_to_cli::DataflowStatus};
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
    /// Why a running coordinator refused this CLI (only for `"refused"`).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error: Option<String>,
}

fn collect_status(coordinator_addr: SocketAddr) -> (StatusOutput, bool) {
    let mut error_occurred = false;

    let (coordinator, session) = match connect_to_coordinator(coordinator_addr) {
        Ok(session) => (
            ComponentStatus {
                status: "running",
                address: Some(coordinator_addr.to_string()),
                error: None,
            },
            Some(session),
        ),
        // Reachable, so "not running" would send the user to `dora up`,
        // which cannot start a second coordinator on this port.
        Err(err) if coordinator_rejected(&err) => {
            error_occurred = true;
            (
                ComponentStatus {
                    status: "refused",
                    address: Some(coordinator_addr.to_string()),
                    error: Some(format!("{err:#}")),
                },
                None,
            )
        }
        Err(_) => {
            error_occurred = true;
            (
                ComponentStatus {
                    status: "not running",
                    address: None,
                    error: None,
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
            error: None,
        }
    } else if session.is_some() {
        error_occurred = true;
        ComponentStatus {
            status: "not running",
            address: None,
            error: None,
        }
    } else {
        error_occurred = true;
        ComponentStatus {
            status: "unknown",
            address: None,
            error: None,
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
    } else if output.coordinator.status == "refused" {
        let _ = stdout.set_color(ColorSpec::new().set_fg(Some(Color::Red)));
        write!(stdout, "✗ ")?;
        let _ = stdout.reset();
        writeln!(stdout, "Coordinator: Running, but refused this CLI")?;
        if let Some(addr) = &output.coordinator.address {
            writeln!(stdout, "  Address: {addr}")?;
        }
        if let Some(error) = &output.coordinator.error {
            for line in error.lines() {
                if line.is_empty() {
                    writeln!(stdout)?;
                } else {
                    writeln!(stdout, "  {line}")?;
                }
            }
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
            let reason = if output.coordinator.status == "refused" {
                "coordinator refused the connection"
            } else {
                "coordinator not available"
            };
            writeln!(stdout, "Daemon: Unknown ({reason})")?;
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
    let reply = send_control_request(session, &ControlRequest::DaemonConnected)?;
    Ok(expect_reply!(reply, DaemonConnected(running))?)
}

fn query_running_dataflow_count(session: &WsSession) -> Result<usize, eyre::ErrReport> {
    let reply = send_control_request(session, &ControlRequest::List)?;
    let list = expect_reply!(reply, DataflowList(list))?;
    Ok(list
        .0
        .iter()
        .filter(|d| d.status == DataflowStatus::Running)
        .count())
}

/// Check system health and connectivity to coordinator and daemon.
#[derive(Debug, clap::Args)]
pub struct Status {
    /// Path to the dataflow descriptor file (enables additional checks)
    #[clap(long, value_name = "PATH", value_hint = clap::ValueHint::FilePath)]
    dataflow: Option<PathBuf>,
    /// Output format
    ///
    /// `json` emits a single pretty-printed JSON document.
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

#[cfg(test)]
mod tests {
    use super::collect_status;
    use std::io::{Read, Write};
    use std::net::TcpListener;

    #[test]
    fn refused_coordinator_is_not_reported_as_not_running() {
        // `--auth` coordinator, wrong token: it is running, so "not running"
        // would send the user to `dora up`, which cannot start a second one.
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap();
        let server = std::thread::spawn(move || {
            let (mut stream, _) = listener.accept().unwrap();
            let mut buf = [0u8; 4096];
            let _ = stream.read(&mut buf);
            stream
                .write_all(b"HTTP/1.1 401 Unauthorized\r\nContent-Length: 0\r\n\r\n")
                .unwrap();
        });

        let (output, error_occurred) = collect_status(addr);
        server.join().unwrap();
        assert!(error_occurred);
        assert_eq!(output.coordinator.status, "refused");
        assert_eq!(output.coordinator.address, Some(addr.to_string()));
        let error = output.coordinator.error.expect("refusal carries its error");
        assert!(error.contains("authentication failed"), "{error}");
    }

    #[test]
    fn closed_port_is_reported_as_not_running() {
        let addr = TcpListener::bind("127.0.0.1:0")
            .unwrap()
            .local_addr()
            .unwrap();
        let (output, error_occurred) = collect_status(addr);
        assert!(error_occurred);
        assert_eq!(output.coordinator.status, "not running");
        assert!(output.coordinator.error.is_none());
    }
}
