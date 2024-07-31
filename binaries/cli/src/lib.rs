pub use crate::{
    build::build as build_dataflow,
    check::check_environment,
    graph::{visualize_as_html, visualize_as_mermaid},
    up::up,
};
use attach::attach_to_dataflow;
use communication_layer_request_reply::{RequestReplyLayer, TcpLayer, TcpRequestReplyConnection};
use dora_core::{
    descriptor::Descriptor,
    topics::{ControlRequest, ControlRequestReply, DataflowList, DataflowResult},
};
use eyre::{bail, Context};
use std::{
    net::{IpAddr, Ipv4Addr, SocketAddr},
    path::PathBuf,
    time::Duration,
};
use uuid::Uuid;

mod attach;
mod build;
mod check;
mod graph;
mod logs;
pub mod template;
mod up;

pub const LOCALHOST: IpAddr = IpAddr::V4(Ipv4Addr::new(127, 0, 0, 1));
pub const LISTEN_WILDCARD: IpAddr = IpAddr::V4(Ipv4Addr::new(0, 0, 0, 0));

pub struct DoraConnection {
    session: Box<TcpRequestReplyConnection>,
}

impl DoraConnection {
    pub fn connect(coordinator_addr: SocketAddr) -> std::io::Result<Self> {
        Ok(Self {
            session: TcpLayer::new().connect(coordinator_addr)?,
        })
    }

    pub fn daemon_running(&mut self) -> eyre::Result<bool> {
        let reply_raw = self
            .session
            .request(&serde_json::to_vec(&ControlRequest::DaemonConnected).unwrap())
            .wrap_err("failed to send DaemonConnected message")?;

        let reply = serde_json::from_slice(&reply_raw).wrap_err("failed to parse reply")?;
        let running = match reply {
            ControlRequestReply::DaemonConnected(running) => running,
            other => bail!("unexpected reply to daemon connection check: {other:?}"),
        };

        Ok(running)
    }

    pub fn query_running_dataflows(&mut self) -> eyre::Result<DataflowList> {
        let reply_raw = self
            .session
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

    pub fn start_dataflow(
        &mut self,
        dataflow: Descriptor,
        name: Option<String>,
        local_working_dir: PathBuf,
    ) -> Result<Uuid, eyre::ErrReport> {
        let reply_raw = self
            .session
            .request(
                &serde_json::to_vec(&ControlRequest::Start {
                    dataflow,
                    name,
                    local_working_dir,
                })
                .unwrap(),
            )
            .wrap_err("failed to send start dataflow message")?;

        let result: ControlRequestReply =
            serde_json::from_slice(&reply_raw).wrap_err("failed to parse reply")?;
        match result {
            ControlRequestReply::DataflowStarted { uuid } => Ok(uuid),
            ControlRequestReply::Error(err) => bail!("{err}"),
            other => bail!("unexpected start dataflow reply: {other:?}"),
        }
    }

    pub fn dataflow_logs(
        &mut self,
        uuid: Option<Uuid>,
        name: Option<String>,
        node: String,
    ) -> eyre::Result<()> {
        logs::dataflow_logs(self, uuid, name, node)
    }

    #[allow(clippy::too_many_arguments)]
    pub fn attach_to_dataflow(
        &mut self,
        dataflow: Descriptor,
        dataflow_path: PathBuf,
        dataflow_id: Uuid,
        hot_reload: bool,
        coordinator_socket: SocketAddr,
        log_level: log::LevelFilter,
        log_output: &mut impl std::io::Write,
    ) -> eyre::Result<DataflowResult> {
        attach_to_dataflow(
            self,
            dataflow,
            dataflow_path,
            dataflow_id,
            hot_reload,
            coordinator_socket,
            log_level,
            log_output,
        )
    }

    pub fn stop_dataflow(
        &mut self,
        uuid: Uuid,
        grace_duration: Option<Duration>,
    ) -> eyre::Result<DataflowResult> {
        let reply_raw = self
            .session
            .request(
                &serde_json::to_vec(&ControlRequest::Stop {
                    dataflow_uuid: uuid,
                    grace_duration,
                })
                .unwrap(),
            )
            .wrap_err("failed to send dataflow stop message")?;
        let result: ControlRequestReply =
            serde_json::from_slice(&reply_raw).wrap_err("failed to parse reply")?;
        match result {
            ControlRequestReply::DataflowStopped { result } => Ok(result),
            ControlRequestReply::Error(err) => bail!("{err}"),
            other => bail!("unexpected stop dataflow reply: {other:?}"),
        }
    }

    pub fn stop_dataflow_by_name(
        &mut self,
        name: String,
        grace_duration: Option<Duration>,
    ) -> eyre::Result<DataflowResult> {
        let reply_raw = self
            .session
            .request(
                &serde_json::to_vec(&ControlRequest::StopByName {
                    name,
                    grace_duration,
                })
                .unwrap(),
            )
            .wrap_err("failed to send dataflow stop_by_name message")?;
        let result: ControlRequestReply =
            serde_json::from_slice(&reply_raw).wrap_err("failed to parse reply")?;
        match result {
            ControlRequestReply::DataflowStopped { result } => Ok(result),
            ControlRequestReply::Error(err) => bail!("{err}"),
            other => bail!("unexpected stop dataflow reply: {other:?}"),
        }
    }

    pub fn destroy(mut self) -> eyre::Result<()> {
        // send destroy command to dora-coordinator
        self.session
            .request(&serde_json::to_vec(&ControlRequest::Destroy).unwrap())
            .wrap_err("failed to send destroy message")?;
        Ok(())
    }
}
