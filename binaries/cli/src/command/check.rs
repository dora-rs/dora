use crate::connect_to_coordinator;
use communication_layer_request_reply::TcpRequestReplyConnection;
use dora_message::{cli_to_coordinator::ControlRequest, coordinator_to_cli::ControlRequestReply};
use eyre::{bail, Context};
use std::{
    io::{IsTerminal, Write},
    net::SocketAddr,
};
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
