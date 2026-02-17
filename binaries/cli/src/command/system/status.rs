use crate::command::{Executable, default_tracing};
use crate::{LOCALHOST, common::connect_to_coordinator};
use communication_layer_request_reply::TcpRequestReplyConnection;
use dora_core::descriptor::{Descriptor, DescriptorExt};
use dora_core::topics::DORA_COORDINATOR_PORT_CONTROL_DEFAULT;
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
                // 1. Resolve the directory of the YAML file
                let working_dir = dataflow
                    .canonicalize()
                    .context("failed to canonicalize dataflow path")?
                    .parent()
                    .ok_or_else(|| eyre::eyre!("dataflow path has no parent dir"))?
                    .to_owned();

                // 2. Read and Parse the YAML
                let descriptor = Descriptor::blocking_read(&dataflow)?;
                
                // 3. [NEW] Static Analysis: Check if node files exist!
                // We iterate over all nodes to verify their 'source' (path) exists.    
// --- START FULL CHECK ---
                println!("Checking dataflow descriptor...");
                let mut all_files_found = true; // Track success

                for node in &descriptor.nodes {
                    if let Some(path) = &node.path {
                        let node_path = working_dir.join(path);
                        
                        if !node_path.exists() {
                            // Print error but CONTINUE looking for others
                            eprintln!("❌ [ERROR] Node '{}': File not found.", node.id);
                            eprintln!("   Looking for: {}", node_path.display());
                            all_files_found = false; 
                        } else {
                            println!("✓ Node '{}': File exists", node.id);
                        }
                    }
                }

                if !all_files_found {
                    // NOW we exit, after showing all errors
                    eprintln!("\n💥 Static check failed: One or more source files are missing.");
                    std::process::exit(1);
                }
                // --- END FULL CHECK ---          
                // 4. Run the original internal check (syntax, etc.)
                descriptor.check(&working_dir)?;

                // 5. Check if the Backend (Daemon/Coordinator) is running
                println!("\nChecking Backend Status:");
                check_environment((self.coordinator_addr, self.coordinator_port).into())?
            }
            None => check_environment((self.coordinator_addr, self.coordinator_port).into())?,
        }

        Ok(())
    }
}