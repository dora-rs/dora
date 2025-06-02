use super::{default_tracing, Executable};
use crate::{
    attach::attach_dataflow,
    common::{connect_to_coordinator, resolve_dataflow},
};
use communication_layer_request_reply::TcpRequestReplyConnection;
use dora_core::{
    descriptor::{Descriptor, DescriptorExt},
    topics::{DORA_COORDINATOR_PORT_CONTROL_DEFAULT, LOCALHOST},
};
use dora_message::{cli_to_coordinator::ControlRequest, coordinator_to_cli::ControlRequestReply};
use eyre::{bail, Context};
use std::{net::IpAddr, path::PathBuf};
use uuid::Uuid;

#[derive(Debug, clap::Args)]
/// Start the given dataflow path. Attach a name to the running dataflow by using --name.
pub struct Start {
    /// Path to the dataflow descriptor file
    #[clap(value_name = "PATH")]
    dataflow: String,
    /// Assign a name to the dataflow
    #[clap(long)]
    name: Option<String>,
    /// Address of the dora coordinator
    #[clap(long, value_name = "IP", default_value_t = LOCALHOST)]
    coordinator_addr: IpAddr,
    /// Port number of the coordinator control server
    #[clap(long, value_name = "PORT", default_value_t = DORA_COORDINATOR_PORT_CONTROL_DEFAULT)]
    coordinator_port: u16,
    /// Attach to the dataflow and wait for its completion
    #[clap(long, action)]
    attach: bool,
    /// Run the dataflow in background
    #[clap(long, action)]
    detach: bool,
    /// Enable hot reloading (Python only)
    #[clap(long, action)]
    hot_reload: bool,
    // Use UV to run nodes.
    #[clap(long, action)]
    uv: bool,
}

impl Executable for Start {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;
        let dataflow = resolve_dataflow(self.dataflow).context("could not resolve dataflow")?;
        let dataflow_descriptor =
            Descriptor::blocking_read(&dataflow).wrap_err("Failed to read yaml dataflow")?;
        let working_dir = dataflow
            .canonicalize()
            .context("failed to canonicalize dataflow path")?
            .parent()
            .ok_or_else(|| eyre::eyre!("dataflow path has no parent dir"))?
            .to_owned();

        let coordinator_socket = (self.coordinator_addr, self.coordinator_port).into();
        let mut session = connect_to_coordinator(coordinator_socket)
            .wrap_err("failed to connect to dora coordinator")?;
        let dataflow_id = start_dataflow(
            dataflow_descriptor.clone(),
            self.name,
            working_dir,
            &mut *session,
            self.uv,
        )?;

        let attach = match (self.attach, self.detach) {
            (true, true) => eyre::bail!("both `--attach` and `--detach` are given"),
            (true, false) => true,
            (false, true) => false,
            (false, false) => {
                println!("attaching to dataflow (use `--detach` to run in background)");
                true
            }
        };

        if attach {
            attach_dataflow(
                dataflow_descriptor,
                dataflow,
                dataflow_id,
                &mut *session,
                self.hot_reload,
                coordinator_socket,
                env_logger::Builder::new()
                    .filter_level(log::LevelFilter::Info)
                    .parse_default_env()
                    .build()
                    .filter(),
            )?;
        }
        Ok(())
    }
}

fn start_dataflow(
    dataflow: Descriptor,
    name: Option<String>,
    local_working_dir: PathBuf,
    session: &mut TcpRequestReplyConnection,
    uv: bool,
) -> Result<Uuid, eyre::ErrReport> {
    let reply_raw = session
        .request(
            &serde_json::to_vec(&ControlRequest::Start {
                dataflow,
                name,
                local_working_dir,
                uv,
            })
            .unwrap(),
        )
        .wrap_err("failed to send start dataflow message")?;

    let result: ControlRequestReply =
        serde_json::from_slice(&reply_raw).wrap_err("failed to parse reply")?;
    match result {
        ControlRequestReply::DataflowStarted { uuid } => {
            eprintln!("{uuid}");
            Ok(uuid)
        }
        ControlRequestReply::Error(err) => bail!("{err}"),
        other => bail!("unexpected start dataflow reply: {other:?}"),
    }
}
