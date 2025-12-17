use crate::command::{Executable, default_tracing};
use crate::{LOCALHOST, common::connect_to_coordinator};
use communication_layer_request_reply::TcpRequestReplyConnection;
use dora_core::topics::DORA_COORDINATOR_PORT_CONTROL_DEFAULT;
use dora_message::{
    cli_to_coordinator::ControlRequest,
    coordinator_to_cli::{ControlRequestReply, DataflowStatus},
};
use eyre::{Context, bail};
use indicatif::{ProgressBar, ProgressStyle};
use std::io::{IsTerminal, Write};
use std::net::{IpAddr, SocketAddr};
use std::path::PathBuf;
use std::{fs, path::Path, time::Duration};

#[derive(Debug, clap::Args)]
/// Stop all running dataflows, coordinator and daemon
pub struct Stop {
    /// Use a custom configuration
    #[clap(long, hide = true, value_name = "PATH", value_hint = clap::ValueHint::FilePath)]
    config: Option<PathBuf>,
    /// Address of the dora coordinator
    #[clap(long, value_name = "IP", default_value_t = LOCALHOST)]
    coordinator_addr: IpAddr,
    /// Port number of the coordinator control server
    #[clap(long, value_name = "PORT", default_value_t = DORA_COORDINATOR_PORT_CONTROL_DEFAULT)]
    coordinator_port: u16,
    /// Force stop without confirmation
    #[clap(long, short = 'f')]
    force: bool,
    /// Skip confirmation prompt (same as --force)
    #[clap(long, short = 'y')]
    yes: bool,
}

impl Executable for Stop {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;
        let coordinator_addr = (self.coordinator_addr, self.coordinator_port).into();
        stop_system(
            self.config.as_deref(),
            coordinator_addr,
            self.force || self.yes,
        )
    }
}

#[derive(Debug, Default, serde::Serialize, serde::Deserialize)]
struct StopConfig {}

fn create_spinner(msg: &str) -> ProgressBar {
    let pb = ProgressBar::new_spinner();
    pb.set_style(
        ProgressStyle::default_spinner()
            .tick_chars("⣾⣽⣻⢿⡿⣟⣯⣷")
            .template("{spinner} {msg}")
            .unwrap(),
    );
    pb.set_message(msg.to_string());
    pb.enable_steady_tick(Duration::from_millis(80));
    pb
}

fn print_success(msg: &str) {
    println!("\x1b[32m✓\x1b[0m {}", msg);
}

fn print_error(msg: &str) {
    eprintln!("\x1b[31m✗\x1b[0m {}", msg);
}

pub fn stop_system(
    config_path: Option<&Path>,
    coordinator_addr: SocketAddr,
    skip_confirm: bool,
) -> eyre::Result<()> {
    let StopConfig {} = parse_config(config_path)?;

    let mut session = match connect_to_coordinator(coordinator_addr) {
        Ok(session) => session,
        Err(_) => {
            print_success("System is not running");
            return Ok(());
        }
    };

    // Check running dataflows
    let running_count = query_running_dataflow_count(&mut *session)?;

    if running_count > 0 && !skip_confirm {
        // Prompt for confirmation
        if std::io::stdin().is_terminal() {
            print!(
                "\x1b[33m?\x1b[0m {} dataflow{} running. Stop them? [y/N] ",
                running_count,
                if running_count == 1 { " is" } else { "s are" }
            );
            std::io::stdout().flush()?;

            let mut input = String::new();
            std::io::stdin().read_line(&mut input)?;
            let input = input.trim().to_lowercase();

            if input != "y" && input != "yes" {
                println!("Aborted.");
                return Ok(());
            }
        } else {
            // Non-interactive, require --force or --yes
            bail!(
                "{} dataflow{} running. Use --force or --yes to stop without confirmation.",
                running_count,
                if running_count == 1 { " is" } else { "s are" }
            );
        }
    }

    // Stop dataflows if any
    if running_count > 0 {
        let spinner = create_spinner("Stopping dataflows...");
        // The Destroy command stops all dataflows first
        spinner.finish_and_clear();
        print_success(&format!(
            "Stopped {} dataflow{}",
            running_count,
            if running_count == 1 { "" } else { "s" }
        ));
    }

    // Send destroy command
    let spinner = create_spinner("Stopping coordinator and daemon...");
    let reply_raw = session
        .request(&serde_json::to_vec(&ControlRequest::Destroy).unwrap())
        .wrap_err("failed to send destroy message")?;

    let result: ControlRequestReply =
        serde_json::from_slice(&reply_raw).wrap_err("failed to parse reply")?;

    match result {
        ControlRequestReply::DestroyOk => {
            spinner.finish_and_clear();
            print_success("Coordinator stopped");
            print_success("Daemon stopped");
        }
        ControlRequestReply::Error(err) => {
            spinner.finish_and_clear();
            print_error("Failed to stop system");
            bail!("Stop command failed with error: {}", err);
        }
        _ => {
            spinner.finish_and_clear();
            print_error("Unexpected response from coordinator");
            bail!("Unexpected reply from dora-coordinator");
        }
    }

    Ok(())
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

fn parse_config(config_path: Option<&Path>) -> Result<StopConfig, eyre::ErrReport> {
    let path = config_path.or_else(|| Some(Path::new("dora-config.yml")).filter(|p| p.exists()));
    let config = match path {
        Some(path) => {
            let raw = fs::read_to_string(path)
                .with_context(|| format!("failed to read `{}`", path.display()))?;
            serde_yaml::from_str(&raw)
                .with_context(|| format!("failed to parse `{}`", path.display()))?
        }
        None => Default::default(),
    };
    Ok(config)
}
