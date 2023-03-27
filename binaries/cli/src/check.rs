use crate::control_connection;
use dora_core::topics::{ControlRequest, ControlRequestReply};
use eyre::{bail, Context};
use std::io::Write;
use termcolor::{Color, ColorChoice, ColorSpec, WriteColor};

pub fn check_environment() -> eyre::Result<()> {
    let mut error_occured = false;

    let color_choice = if atty::is(atty::Stream::Stdout) {
        ColorChoice::Auto
    } else {
        ColorChoice::Never
    };
    let mut stdout = termcolor::StandardStream::stdout(color_choice);

    // check whether coordinator is running
    write!(stdout, "Dora Coordinator: ")?;
    if coordinator_running()? {
        let _ = stdout.set_color(ColorSpec::new().set_fg(Some(Color::Green)));
        writeln!(stdout, "ok")?;
    } else {
        let _ = stdout.set_color(ColorSpec::new().set_fg(Some(Color::Red)));
        writeln!(stdout, "not running")?;
        error_occured = true;
    }
    let _ = stdout.reset();

    // check whether daemon is running
    write!(stdout, "Dora Daemon: ")?;
    if daemon_running()? {
        let _ = stdout.set_color(ColorSpec::new().set_fg(Some(Color::Green)));
        writeln!(stdout, "ok")?;
    } else {
        let _ = stdout.set_color(ColorSpec::new().set_fg(Some(Color::Red)));
        writeln!(stdout, "not running")?;
        error_occured = true;
    }
    let _ = stdout.reset();

    writeln!(stdout)?;

    if error_occured {
        bail!("Environment check failed.");
    }

    Ok(())
}

pub fn coordinator_running() -> Result<bool, eyre::ErrReport> {
    let mut control_session = None;
    let connected = control_connection(&mut control_session).is_ok();
    Ok(connected)
}

pub fn daemon_running() -> Result<bool, eyre::ErrReport> {
    let mut control_session = None;
    let running = match control_connection(&mut control_session) {
        Ok(connection) => {
            let reply_raw = connection
                .request(&serde_json::to_vec(&ControlRequest::DaemonConnected).unwrap())
                .wrap_err("failed to send DaemonConnected message")?;

            let reply = serde_json::from_slice(&reply_raw).wrap_err("failed to parse reply")?;
            match reply {
                ControlRequestReply::DaemonConnected(running) => running,
                other => bail!("unexpected reply to daemon connection check: {other:?}"),
            }
        }
        Err(_) => {
            // coordinator is not running
            false
        }
    };
    Ok(running)
}
