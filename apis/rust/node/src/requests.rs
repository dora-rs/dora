use std::sync::Arc;

use dora_core::{
    topics::{DORA_DAEMON_LOCAL_LISTEN_PORT_DEFAULT, LOCALHOST},
    uhlc,
};
use dora_message::{
    common::Timestamped, daemon_to_node::DaemonReply, node_to_daemon::DaemonRequest, DataflowId,
};
use eyre::{bail, Context};

use crate::daemon_connection::DaemonChannel;

pub fn start_dataflow(
    dataflow: String,
    name: Option<String>,
    uv: bool,
) -> eyre::Result<DataflowId> {
    let mut channel = init_daemon_channel()?;
    let clock = Arc::new(uhlc::HLC::default());

    let request = DaemonRequest::StartDataflow { dataflow, name, uv };
    let reply = channel
        .request(&Timestamped {
            inner: request,
            timestamp: clock.new_timestamp(),
        })
        .wrap_err("failed to trigger dataflow start through daemon")?;
    match reply {
        DaemonReply::StartDataflowResult(Ok(dataflow_id)) => Ok(dataflow_id),
        DaemonReply::StartDataflowResult(Err(err)) => bail!("failed to start dataflow: {err}"),
        other => bail!("unexpected StartDataflow reply from daemon: {other:?}"),
    }
}

fn init_daemon_channel() -> eyre::Result<DaemonChannel> {
    let daemon_address = (LOCALHOST, DORA_DAEMON_LOCAL_LISTEN_PORT_DEFAULT).into();

    let channel =
        DaemonChannel::new_tcp(daemon_address).context("Could not connect to the daemon")?;
    Ok(channel)
}
