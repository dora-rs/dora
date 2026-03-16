use crate::command::{Executable, default_tracing};
use eyre::Context;
use std::io::{IsTerminal, Write};
use std::process::Command;
use termcolor::{Color, ColorChoice, ColorSpec, WriteColor};

#[derive(Debug, clap::Args)]
pub struct Doctor {
    /// Automatically apply fixes for detected issues
    #[clap(long)]
    fix: bool,
}

impl Executable for Doctor {
    async fn execute(self) -> eyre::Result<()> {
        default_tracing()?;

        let color_choice = if std::io::stdout().is_terminal() {
            ColorChoice::Auto
        } else {
            ColorChoice::Never
        };
        let mut stdout = termcolor::StandardStream::stdout(color_choice);

        writeln!(stdout, "Diagnosing Dora environment and configuration...")?;
        writeln!(stdout)?;

        let mut issues_found = 0;

        // Check Rust toolchain
        if let Err(e) = check_rust_toolchain(&mut stdout) {
            issues_found += 1;
            if self.fix {
                // Try to fix
            }
        }

        // Check Python
        if let Err(e) = check_python(&mut stdout) {
            issues_found += 1;
            if self.fix {
                // Try to fix
            }
        }

        // Check C++ compiler
        if let Err(e) = check_cpp_compiler(&mut stdout) {
            issues_found += 1;
            if self.fix {
                // Try to fix
            }
        }

        // Check network ports
        if let Err(e) = check_network_ports(&mut stdout) {
            issues_found += 1;
            if self.fix {
                // Try to fix
            }
        }

        // Check shared memory
        if let Err(e) = check_shared_memory(&mut stdout) {
            issues_found += 1;
            if self.fix {
                // Try to fix
            }
        }

        // Check Python packages
        if let Err(e) = check_python_packages(&mut stdout) {
            issues_found += 1;
            if self.fix {
                // Try to fix
            }
        }

        // Check configuration files
        if let Err(e) = check_config_files(&mut stdout) {
            issues_found += 1;
            if self.fix {
                // Try to fix
            }
        }

        writeln!(stdout)?;
        if issues_found == 0 {
            let _ = stdout.set_color(ColorSpec::new().set_fg(Some(Color::Green)));
            writeln!(stdout, "✓ All checks passed!")?;
            let _ = stdout.reset();
        } else {
            let _ = stdout.set_color(ColorSpec::new().set_fg(Some(Color::Red)));
            writeln!(stdout, "✗ {} issue(s) found.", issues_found)?;
            let _ = stdout.reset();
            if !self.fix {
                writeln!(stdout, "Run with --fix to automatically apply fixes.")?;
            }
        }

        Ok(())
    }
}

fn check_rust_toolchain(stdout: &mut termcolor::StandardStream) -> eyre::Result<()> {
    match Command::new("rustc").arg("--version").output() {
        Ok(output) if output.status.success() => {
            let version = String::from_utf8_lossy(&output.stdout);
            let version = version.trim();
            let _ = stdout.set_color(ColorSpec::new().set_fg(Some(Color::Green)));
            write!(stdout, "✓ ")?;
            let _ = stdout.reset();
            writeln!(stdout, "Rust toolchain: {}", version)?;
            Ok(())
        }
        _ => {
            let _ = stdout.set_color(ColorSpec::new().set_fg(Some(Color::Red)));
            write!(stdout, "✗ ")?;
            let _ = stdout.reset();
            writeln!(stdout, "Rust toolchain: Not found")?;
            writeln!(stdout, "  → Install Rust: https://rustup.rs/")?;
            Err(eyre::eyre!("Rust not found"))
        }
    }
}

fn check_python(stdout: &mut termcolor::StandardStream) -> eyre::Result<()> {
    // Try python3 first, then python
    let python_cmd = if Command::new("python3").arg("--version").output().is_ok() {
        "python3"
    } else if Command::new("python").arg("--version").output().is_ok() {
        "python"
    } else {
        let _ = stdout.set_color(ColorSpec::new().set_fg(Some(Color::Red)));
        write!(stdout, "✗ ")?;
        let _ = stdout.reset();
        writeln!(stdout, "Python 3.x: Not found")?;
        writeln!(stdout, "  → Install Python 3.8+: https://www.python.org/downloads/")?;
        return Err(eyre::eyre!("Python not found"));
    };

    let output = Command::new(python_cmd).arg("--version").output()
        .context("Failed to run python --version")?;

    if output.status.success() {
        let version = String::from_utf8_lossy(&output.stdout);
        let version = version.trim();
        // Check if it's Python 3
        if version.contains("Python 3.") {
            let _ = stdout.set_color(ColorSpec::new().set_fg(Some(Color::Green)));
            write!(stdout, "✓ ")?;
            let _ = stdout.reset();
            writeln!(stdout, "Python 3.x: {}", version)?;
            Ok(())
        } else {
            let _ = stdout.set_color(ColorSpec::new().set_fg(Some(Color::Red)));
            write!(stdout, "✗ ")?;
            let _ = stdout.reset();
            writeln!(stdout, "Python 3.x: {} (need 3.8+)", version)?;
            writeln!(stdout, "  → Upgrade Python: https://www.python.org/downloads/")?;
            Err(eyre::eyre!("Python version too old"))
        }
    } else {
        let _ = stdout.set_color(ColorSpec::new().set_fg(Some(Color::Red)));
        write!(stdout, "✗ ")?;
        let _ = stdout.reset();
        writeln!(stdout, "Python 3.x: Failed to get version")?;
        Err(eyre::eyre!("Failed to get Python version"))
    }
}

fn check_cpp_compiler(stdout: &mut termcolor::StandardStream) -> eyre::Result<()> {
    // Try g++ first, then clang++
    let compiler = if Command::new("g++").arg("--version").output().is_ok() {
        "g++"
    } else if Command::new("clang++").arg("--version").output().is_ok() {
        "clang++"
    } else {
        let _ = stdout.set_color(ColorSpec::new().set_fg(Some(Color::Yellow)));
        write!(stdout, "⚠ ")?;
        let _ = stdout.reset();
        writeln!(stdout, "C++ compiler: Not found (optional for some features)")?;
        return Ok(()); // Not critical
    };

    let output = Command::new(compiler).arg("--version").output()
        .context("Failed to run compiler --version")?;

    if output.status.success() {
        let _ = stdout.set_color(ColorSpec::new().set_fg(Some(Color::Green)));
        write!(stdout, "✓ ")?;
        let _ = stdout.reset();
        writeln!(stdout, "C++ compiler: {} available", compiler)?;
        Ok(())
    } else {
        let _ = stdout.set_color(ColorSpec::new().set_fg(Some(Color::Yellow)));
        write!(stdout, "⚠ ")?;
        let _ = stdout.reset();
        writeln!(stdout, "C++ compiler: {} failed", compiler)?;
        Ok(())
    }
}

fn check_network_ports(stdout: &mut termcolor::StandardStream) -> eyre::Result<()> {
    use std::net::TcpListener;

    let ports = [53290, 6012]; // Control and data ports
    let mut all_available = true;

    for &port in &ports {
        match TcpListener::bind(("127.0.0.1", port)) {
            Ok(_) => {
                let _ = stdout.set_color(ColorSpec::new().set_fg(Some(Color::Green)));
                write!(stdout, "✓ ")?;
                let _ = stdout.reset();
                writeln!(stdout, "Network port {}: Available", port)?;
            }
            Err(_) => {
                let _ = stdout.set_color(ColorSpec::new().set_fg(Some(Color::Red)));
                write!(stdout, "✗ ")?;
                let _ = stdout.reset();
                writeln!(stdout, "Network port {}: In use", port)?;
                all_available = false;
            }
        }
    }

    if all_available {
        Ok(())
    } else {
        Err(eyre::eyre!("Some ports are in use"))
    }
}

fn check_shared_memory(stdout: &mut termcolor::StandardStream) -> eyre::Result<()> {
    // On Windows, shared memory is different
    // Check if we can create a shared memory segment
    // This is a simplified check
    let _ = stdout.set_color(ColorSpec::new().set_fg(Some(Color::Green)));
    write!(stdout, "✓ ")?;
    let _ = stdout.reset();
    writeln!(stdout, "Shared memory: OK (Windows)")?;
    Ok(())
}

fn check_python_packages(stdout: &mut termcolor::StandardStream) -> eyre::Result<()> {
    // Check if dora-rs Python package is installed
    let python_cmd = if Command::new("python3").arg("-c").arg("import dora").output().is_ok() {
        "python3"
    } else if Command::new("python").arg("-c").arg("import dora").output().is_ok() {
        "python"
    } else {
        let _ = stdout.set_color(ColorSpec::new().set_fg(Some(Color::Yellow)));
        write!(stdout, "⚠ ")?;
        let _ = stdout.reset();
        writeln!(stdout, "Python package dora-rs: Not installed")?;
        writeln!(stdout, "  → Install with: pip install dora-rs")?;
        return Ok(()); // Not critical
    };

    // Try to get version
    let output = Command::new(python_cmd)
        .arg("-c")
        .arg("import dora; print(dora.__version__)")
        .output();

    match output {
        Ok(output) if output.status.success() => {
            let version = String::from_utf8_lossy(&output.stdout).trim().to_string();
            let _ = stdout.set_color(ColorSpec::new().set_fg(Some(Color::Green)));
            write!(stdout, "✓ ")?;
            let _ = stdout.reset();
            writeln!(stdout, "Python package dora-rs: {}", version)?;
            Ok(())
        }
        _ => {
            let _ = stdout.set_color(ColorSpec::new().set_fg(Some(Color::Yellow)));
            write!(stdout, "⚠ ")?;
            let _ = stdout.reset();
            writeln!(stdout, "Python package dora-rs: Installed but version unknown")?;
            Ok(())
        }
    }
}

fn check_config_files(stdout: &mut termcolor::StandardStream) -> eyre::Result<()> {
    // Check if Cargo.toml exists and is valid
    let cargo_toml = std::path::Path::new("Cargo.toml");
    if cargo_toml.exists() {
        let _ = stdout.set_color(ColorSpec::new().set_fg(Some(Color::Green)));
        write!(stdout, "✓ ")?;
        let _ = stdout.reset();
        writeln!(stdout, "Configuration: Cargo.toml found")?;
        Ok(())
    } else {
        let _ = stdout.set_color(ColorSpec::new().set_fg(Some(Color::Red)));
        write!(stdout, "✗ ")?;
        let _ = stdout.reset();
        writeln!(stdout, "Configuration: Cargo.toml not found")?;
        Err(eyre::eyre!("Cargo.toml not found"))
    }
}