use crate::command::{Executable, default_tracing};
use crate::common::{connect_to_coordinator_rpc, rpc};
use dora_core::descriptor::DescriptorExt;
use dora_core::{descriptor::Descriptor, topics::DORA_COORDINATOR_PORT_CONTROL_DEFAULT};
use dora_message::{
    cli_to_coordinator::CliControlClient, coordinator_to_cli::DataflowStatus, tarpc,
};
use eyre::{Context, bail};
use std::{
    io::{IsTerminal, Write},
    net::SocketAddr,
};
use std::{net::IpAddr, path::PathBuf};
use termcolor::{Color, ColorChoice, ColorSpec, WriteColor};

pub async fn check_environment(coordinator_addr: SocketAddr) -> eyre::Result<()> {
    let mut error_occurred = false;

    let color_choice = if std::io::stdout().is_terminal() {
        ColorChoice::Auto
    } else {
        ColorChoice::Never
    };
    let mut stdout = termcolor::StandardStream::stdout(color_choice);

    // Coordinator status
    let client =
        match connect_to_coordinator_rpc(coordinator_addr.ip(), coordinator_addr.port()).await {
            Ok(client) => {
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
                Some(client)
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
    let daemon_running_result = match client.as_ref() {
        Some(c) => Some(daemon_running(c).await?),
        None => None,
    };

    if daemon_running_result == Some(true) {
        let _ = stdout.set_color(ColorSpec::new().set_fg(Some(Color::Green)));
        write!(stdout, "✓ ")?;
        let _ = stdout.reset();
        writeln!(stdout, "Daemon: Running")?;
    } else if client.is_some() {
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
    if let Some(ref c) = client {
        if let Ok(count) = query_running_dataflow_count(c).await {
            writeln!(stdout, "Active dataflows: {}", count)?;
        }
    }

    if error_occurred {
        bail!("System check failed.");
    }

    Ok(())
}

pub async fn daemon_running(client: &CliControlClient) -> Result<bool, eyre::ErrReport> {
    rpc::<bool, _>(
        "check daemon connection",
        client.daemon_connected(tarpc::context::current()),
    )
    .await
}

async fn query_running_dataflow_count(client: &CliControlClient) -> Result<usize, eyre::ErrReport> {
    let list = rpc::<dora_message::coordinator_to_cli::DataflowList, _>(
        "list dataflows",
        client.list(tarpc::context::current()),
    )
    .await?;
    Ok(list
        .0
        .iter()
        .filter(|d| d.status == DataflowStatus::Running)
        .count())
}

#[derive(Debug, clap::Args)]
pub struct Status {
    /// Path to the dataflow descriptor file (enables additional checks)
    #[clap(long, value_name = "PATH", value_hint = clap::ValueHint::FilePath)]
    dataflow: Option<PathBuf>,
    /// Address of the dora coordinator
    #[clap(long, value_name = "IP")]
    coordinator_addr: Option<IpAddr>,
    /// Port number of the coordinator control server
    #[clap(long, value_name = "PORT")]
    coordinator_port: Option<u16>,
}

impl Executable for Status {
    async fn execute(self) -> eyre::Result<()> {
        default_tracing()?;

        use crate::common::resolve_coordinator_addr;
        let (addr, port) = resolve_coordinator_addr(
            self.coordinator_addr,
            self.coordinator_port,
            DORA_COORDINATOR_PORT_CONTROL_DEFAULT,
        );

        match self.dataflow {
            Some(dataflow) => {
                let working_dir = dataflow
                    .canonicalize()
                    .context("failed to canonicalize dataflow path")?
                    .parent()
                    .ok_or_else(|| eyre::eyre!("dataflow path has no parent dir"))?
                    .to_owned();
                Descriptor::blocking_read(&dataflow)?.check(&working_dir)?;
                check_environment((addr, port).into()).await?
            }
            None => check_environment((addr, port).into()).await?,
        }

        Ok(())
    }
}
