use eyre::bail;
use std::{
    io::{IsTerminal, Write},
    net::SocketAddr,
};
use termcolor::{Color, ColorChoice, ColorSpec, WriteColor};

use crate::DoraConnection;

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
    let mut session = match DoraConnection::connect(coordinator_addr) {
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
        .as_mut()
        .map(|c| c.daemon_running())
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
