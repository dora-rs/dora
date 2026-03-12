use crate::{LOCALHOST, formatting::FormatDataflowError, ws_client::WsSession};
use adora_core::{
    descriptor::{Descriptor, source_is_url},
    topics::ADORA_COORDINATOR_PORT_WS_DEFAULT,
};
use adora_download::download_file;
use adora_message::{
    cli_to_coordinator::ControlRequest,
    coordinator_to_cli::{ControlRequestReply, DataflowList, DataflowResult},
};
use eyre::{Context, ContextCompat, bail};
use std::{
    env::current_dir,
    io::IsTerminal,
    net::{IpAddr, SocketAddr},
    path::{Path, PathBuf},
};
use tokio::runtime::Builder;
use uuid::Uuid;

pub(crate) fn handle_dataflow_result(
    result: DataflowResult,
    uuid: Option<Uuid>,
) -> Result<(), eyre::Error> {
    if result.is_ok() {
        Ok(())
    } else {
        Err(match uuid {
            Some(uuid) => {
                eyre::eyre!("Dataflow {uuid} failed:\n{}", FormatDataflowError(&result))
            }
            None => {
                eyre::eyre!("Dataflow failed:\n{}", FormatDataflowError(&result))
            }
        })
    }
}

/// Send a control request and deserialize the reply.
pub(crate) fn send_control_request(
    session: &WsSession,
    request: &ControlRequest,
) -> eyre::Result<ControlRequestReply> {
    let reply_raw = session
        .request(&serde_json::to_vec(request).unwrap())
        .wrap_err("failed to send control request")?;
    serde_json::from_slice(&reply_raw).wrap_err("failed to parse reply")
}

pub(crate) fn query_running_dataflows(session: &WsSession) -> eyre::Result<DataflowList> {
    let reply_raw = session
        .request(&serde_json::to_vec(&ControlRequest::List).unwrap())
        .wrap_err("failed to send list message")?;
    let reply: ControlRequestReply =
        serde_json::from_slice(&reply_raw).wrap_err("failed to parse reply")?;
    let ids = match reply {
        ControlRequestReply::DataflowList(list) => list,
        ControlRequestReply::Error(err) => bail!("{err}"),
        other => bail!("unexpected list dataflow reply: {other:?}"),
    };

    Ok(ids)
}

pub(crate) fn resolve_dataflow_identifier_interactive(
    session: &WsSession,
    name_or_uuid: Option<&str>,
) -> eyre::Result<Uuid> {
    if let Some(uuid) = name_or_uuid.and_then(|s| Uuid::parse_str(s).ok()) {
        return Ok(uuid);
    }

    let list = query_running_dataflows(session).wrap_err("failed to query running dataflows")?;
    let active: Vec<adora_message::coordinator_to_cli::DataflowIdAndName> = list.get_active();
    if let Some(name) = name_or_uuid {
        let Some(dataflow) = active.iter().find(|it| it.name.as_deref() == Some(name)) else {
            let available: Vec<_> = active.iter().filter_map(|d| d.name.as_deref()).collect();
            if available.is_empty() {
                bail!(
                    "no dataflow with name `{name}` is running\n\n  \
                     hint: use `adora list` to see running dataflows"
                );
            } else {
                bail!(
                    "no dataflow with name `{name}` is running\n\n  \
                     hint: running dataflows: {}",
                    available.join(", ")
                );
            }
        };
        return Ok(dataflow.uuid);
    }
    Ok(match &active[..] {
        [] => bail!(
            "no dataflows are running\n\n  \
             hint: start a dataflow with `adora start` or `adora run`"
        ),
        [entry] => entry.uuid,
        _ => {
            if !std::io::stdin().is_terminal() {
                bail!("Multiple dataflows running. Specify a UUID or name.");
            }
            inquire::Select::new("Choose dataflow:", active)
                .prompt()?
                .uuid
        }
    })
}

#[derive(Debug, clap::Args)]
pub(crate) struct CoordinatorOptions {
    /// Address of the adora coordinator
    #[clap(long, value_name = "IP", default_value_t = LOCALHOST, env = "ADORA_COORDINATOR_ADDR")]
    pub coordinator_addr: IpAddr,
    /// Port number of the coordinator WebSocket server
    #[clap(long, value_name = "PORT", default_value_t = ADORA_COORDINATOR_PORT_WS_DEFAULT, env = "ADORA_COORDINATOR_PORT")]
    pub coordinator_port: u16,
}

impl CoordinatorOptions {
    pub fn socket_addr(&self) -> SocketAddr {
        (self.coordinator_addr, self.coordinator_port).into()
    }

    pub fn connect(&self) -> eyre::Result<WsSession> {
        connect_to_coordinator(self.socket_addr())
    }
}

pub(crate) fn connect_to_coordinator(coordinator_addr: SocketAddr) -> eyre::Result<WsSession> {
    WsSession::connect(coordinator_addr)
}

/// Try to connect to the coordinator, retrying for `timeout` before giving up.
pub(crate) fn connect_with_retry(
    addr: SocketAddr,
    timeout: std::time::Duration,
) -> eyre::Result<WsSession> {
    let deadline = std::time::Instant::now() + timeout;
    loop {
        match connect_to_coordinator(addr) {
            Ok(session) => return Ok(session),
            Err(_) if std::time::Instant::now() < deadline => {
                std::thread::sleep(std::time::Duration::from_millis(50));
            }
            Err(err) => return Err(err),
        }
    }
}

pub(crate) fn resolve_dataflow(dataflow: String) -> eyre::Result<PathBuf> {
    let dataflow = if source_is_url(&dataflow) {
        // try to download the shared library
        let target_path = current_dir().context("Could not access the current dir")?;
        let rt = Builder::new_current_thread()
            .enable_all()
            .build()
            .context("tokio runtime failed")?;
        rt.block_on(async { download_file(&dataflow, &target_path).await })
            .wrap_err("failed to download dataflow yaml file")?
    } else {
        PathBuf::from(dataflow)
    };
    Ok(dataflow)
}

pub(crate) fn local_working_dir(
    dataflow_path: &Path,
    dataflow_descriptor: &Descriptor,
    coordinator_session: &WsSession,
) -> eyre::Result<Option<PathBuf>> {
    Ok(
        if dataflow_descriptor
            .nodes
            .iter()
            .all(|n| n.deploy.as_ref().map(|d| d.machine.as_ref()).is_none())
            && cli_and_daemon_on_same_machine(coordinator_session)?
        {
            Some(
                dunce::canonicalize(dataflow_path)
                    .context("failed to canonicalize dataflow file path")?
                    .parent()
                    .context("dataflow path has no parent dir")?
                    .to_owned(),
            )
        } else {
            None
        },
    )
}

pub(crate) fn cli_and_daemon_on_same_machine(session: &WsSession) -> eyre::Result<bool> {
    let reply_raw = session
        .request(&serde_json::to_vec(&ControlRequest::CliAndDefaultDaemonOnSameMachine).unwrap())
        .wrap_err("failed to send start dataflow message")?;

    let result: ControlRequestReply =
        serde_json::from_slice(&reply_raw).wrap_err("failed to parse reply")?;
    match result {
        ControlRequestReply::CliAndDefaultDaemonIps {
            default_daemon,
            cli,
        } => Ok(default_daemon.is_some() && default_daemon == cli),
        ControlRequestReply::Error(err) => bail!("{err}"),
        other => bail!("unexpected start dataflow reply: {other:?}"),
    }
}

pub(crate) fn write_events_to() -> Option<PathBuf> {
    std::env::var("ADORA_WRITE_EVENTS_TO")
        .ok()
        .map(PathBuf::from)
}
