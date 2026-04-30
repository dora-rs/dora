use crate::command::{Executable, default_tracing};
use std::io::{IsTerminal, Write};
use termcolor::{Color, ColorChoice, ColorSpec, WriteColor};

#[derive(Debug, clap::Args)]
pub struct Doctor {}

impl Executable for Doctor {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;
        run_doctor_checks()
    }
}

fn run_doctor_checks() -> eyre::Result<()> {
    #[cfg(target_os = "linux")]
    let mut issue_detected = false;

    let color_choice = if std::io::stdout().is_terminal() {
        ColorChoice::Auto
    } else {
        ColorChoice::Never
    };
    let mut stdout = termcolor::StandardStream::stdout(color_choice);

    #[cfg(target_os = "linux")]
    {
        if let Some((message, fix_hint)) = shared_memory_issue() {
            let _ = stdout.set_color(ColorSpec::new().set_fg(Some(Color::Yellow)));
            write!(stdout, "⚠ ")?;
            let _ = stdout.reset();
            writeln!(stdout, "Shared memory: {message}")?;
            writeln!(stdout, "  → Run: {fix_hint}")?;
            issue_detected = true;
        } else {
            let _ = stdout.set_color(ColorSpec::new().set_fg(Some(Color::Green)));
            write!(stdout, "✓ ")?;
            let _ = stdout.reset();
            writeln!(stdout, "Shared memory: Permissions OK")?;
        }
    }

    #[cfg(not(target_os = "linux"))]
    {
        let _ = stdout.set_color(ColorSpec::new().set_fg(Some(Color::Yellow)));
        write!(stdout, "⚠ ")?;
        let _ = stdout.reset();
        writeln!(
            stdout,
            "Shared memory: Check skipped (only implemented for Linux /dev/shm)"
        )?;
    }

    #[cfg(target_os = "linux")]
    if issue_detected {
        eyre::bail!("System doctor found issues.");
    }

    Ok(())
}

#[cfg(target_os = "linux")]
fn shared_memory_issue() -> Option<(&'static str, &'static str)> {
    use std::fs;
    use std::os::unix::fs::PermissionsExt;
    use std::path::Path;

    let shm_path = Path::new("/dev/shm");
    let metadata = match fs::metadata(shm_path) {
        Ok(metadata) => metadata,
        Err(_) => {
            return Some((
                "Directory not accessible",
                "sudo mkdir -p /dev/shm && sudo chmod 1777 /dev/shm",
            ));
        }
    };

    if !metadata.is_dir() {
        return Some(("Permission issue detected", "sudo chmod 1777 /dev/shm"));
    }

    let mode = metadata.permissions().mode() & 0o7777;
    let expected = 0o1777;
    if mode != expected {
        return Some(("Permission issue detected", "sudo chmod 1777 /dev/shm"));
    }

    None
}
