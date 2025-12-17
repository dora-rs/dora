use crate::command::{Executable, default_tracing};
use crate::{LOCALHOST, common::connect_to_coordinator};
use communication_layer_request_reply::TcpRequestReplyConnection;
use dora_core::descriptor::DescriptorExt;
use dora_core::{descriptor::Descriptor, topics::DORA_COORDINATOR_PORT_CONTROL_DEFAULT};
use dora_message::{
    cli_to_coordinator::ControlRequest,
    coordinator_to_cli::{ControlRequestReply, DataflowStatus},
};
use eyre::{Context, bail};
use std::{
    io::{IsTerminal, Write},
    net::SocketAddr,
};
use std::{net::IpAddr, path::PathBuf};
use termcolor::{Color, ColorChoice, ColorSpec, WriteColor};

pub fn check_environment(coordinator_addr: SocketAddr) -> eyre::Result<()> {
    let mut error_occurred = false;

    let color_choice = if std::io::stdout().is_terminal() {
        ColorChoice::Auto
    } else {
        ColorChoice::Never
    };
    let mut stdout = termcolor::StandardStream::stdout(color_choice);

    // Coordinator status
    let mut session = match connect_to_coordinator(coordinator_addr) {
        Ok(session) => {
            let _ = stdout.set_color(ColorSpec::new().set_fg(Some(Color::Green)));
            write!(stdout, "✓ ")?;
            let _ = stdout.reset();
            writeln!(stdout, "Coordinator: Running")?;
            writeln!(
                stdout,
                "  Address: {}:{}",
                coordinator_addr.ip(),
                coordinator_addr.port()
            )?;
            Some(session)
        }
        Err(_) => {
            let _ = stdout.set_color(ColorSpec::new().set_fg(Some(Color::Red)));
            write!(stdout, "✗ ")?;
            let _ = stdout.reset();
            writeln!(stdout, "Coordinator: Not running")?;
            error_occurred = true;
            None
        }
    };

    // Daemon status
    let daemon_running = session.as_deref_mut().map(daemon_running).transpose()?;

    if daemon_running == Some(true) {
        let _ = stdout.set_color(ColorSpec::new().set_fg(Some(Color::Green)));
        write!(stdout, "✓ ")?;
        let _ = stdout.reset();
        writeln!(stdout, "Daemon: Running")?;
    } else if session.is_some() {
        let _ = stdout.set_color(ColorSpec::new().set_fg(Some(Color::Red)));
        write!(stdout, "✗ ")?;
        let _ = stdout.reset();
        writeln!(stdout, "Daemon: Not running")?;
        error_occurred = true;
    } else {
        let _ = stdout.set_color(ColorSpec::new().set_fg(Some(Color::Red)));
        write!(stdout, "✗ ")?;
        let _ = stdout.reset();
        writeln!(stdout, "Daemon: Unknown (coordinator not available)")?;
        error_occurred = true;
    }

    // Dataflow count
    if let Some(ref mut sess) = session {
        if let Ok(count) = query_running_dataflow_count(&mut **sess) {
            writeln!(stdout, "Active dataflows: {}", count)?;
        }
    }

    if error_occurred {
        bail!("System check failed.");
    }

    Ok(())
}

pub fn daemon_running(session: &mut TcpRequestReplyConnection) -> Result<bool, eyre::ErrReport> {
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

fn query_running_dataflow_count(
    session: &mut TcpRequestReplyConnection,
) -> Result<usize, eyre::ErrReport> {
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

#[derive(Debug, clap::Args)]
pub struct Status {
    /// Path to the dataflow descriptor file (enables additional checks)
    #[clap(long, value_name = "PATH", value_hint = clap::ValueHint::FilePath)]
    dataflow: Option<PathBuf>,
    /// Address of the dora coordinator
    #[clap(long, value_name = "IP", default_value_t = LOCALHOST)]
    coordinator_addr: IpAddr,
    /// Port number of the coordinator control server
    #[clap(long, value_name = "PORT", default_value_t = DORA_COORDINATOR_PORT_CONTROL_DEFAULT)]
    coordinator_port: u16,
}

impl Executable for Status {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;

        match self.dataflow {
            Some(dataflow) => {
                let working_dir = dataflow
                    .canonicalize()
                    .context("failed to canonicalize dataflow path")?
                    .parent()
                    .ok_or_else(|| eyre::eyre!("dataflow path has no parent dir"))?
                    .to_owned();
                Descriptor::blocking_read(&dataflow)?.check(&working_dir)?;
                check_environment((self.coordinator_addr, self.coordinator_port).into())?
            }
            None => check_environment((self.coordinator_addr, self.coordinator_port).into())?,
        }

        Ok(())
    }
}
