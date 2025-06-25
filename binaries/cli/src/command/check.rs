use super::{default_tracing, Executable};
use crate::{common::connect_to_coordinator, LOCALHOST};
use communication_layer_request_reply::TcpRequestReplyConnection;
use dora_core::descriptor::DescriptorExt;
use dora_core::{descriptor::Descriptor, topics::DORA_COORDINATOR_PORT_CONTROL_DEFAULT};
use dora_message::{cli_to_coordinator::ControlRequest, coordinator_to_cli::ControlRequestReply};
use eyre::{bail, Context};
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

    // check whether coordinator is running
    write!(stdout, "Dora Coordinator: ")?;
    let mut session = match connect_to_coordinator(coordinator_addr) {
        Ok(session) => {
            let _ = stdout.set_color(ColorSpec::new().set_fg(Some(Color::Green)));
            writeln!(stdout, "ok")?;
            Some(session)
        }
        Err(_) => {
            let _ = stdout.set_color(ColorSpec::new().set_fg(Some(Color::Red)));
            writeln!(stdout, "not running")?;
            error_occurred = true;
            None
        }
    };

    let _ = stdout.reset();

    // check whether daemon is running
    write!(stdout, "Dora Daemon: ")?;
    if session
        .as_deref_mut()
        .map(daemon_running)
        .transpose()?
        .unwrap_or(false)
    {
        let _ = stdout.set_color(ColorSpec::new().set_fg(Some(Color::Green)));
        writeln!(stdout, "ok")?;
    } else {
        let _ = stdout.set_color(ColorSpec::new().set_fg(Some(Color::Red)));
        writeln!(stdout, "not running")?;
        error_occurred = true;
    }
    let _ = stdout.reset();

    writeln!(stdout)?;

    if error_occurred {
        bail!("Environment check failed.");
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

#[derive(Debug, clap::Args)]
/// Check if the coordinator and the daemon is running.
pub struct Check {
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

impl Executable for Check {
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
