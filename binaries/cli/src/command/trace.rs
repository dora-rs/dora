use dora_message::{
    cli_to_coordinator::ControlRequest,
    coordinator_to_cli::{ControlRequestReply, TraceSummary},
};
use eyre::{Context, bail};

use crate::{command::Executable, ws_client::WsSession};

mod list;
mod view;

/// View coordinator tracing spans.
#[derive(Debug, clap::Subcommand)]
pub enum Trace {
    List(list::List),
    View(view::View),
}

impl Executable for Trace {
    fn execute(self) -> eyre::Result<()> {
        match self {
            Trace::List(cmd) => cmd.execute(),
            Trace::View(cmd) => cmd.execute(),
        }
    }
}

pub(crate) fn fetch_traces(session: &WsSession) -> eyre::Result<Vec<TraceSummary>> {
    let reply_raw = session
        .request(&serde_json::to_vec(&ControlRequest::GetTraces).unwrap())
        .wrap_err("failed to send GetTraces request")?;
    let reply: ControlRequestReply =
        serde_json::from_slice(&reply_raw).wrap_err("failed to parse reply")?;
    match reply {
        ControlRequestReply::TraceList(t) => Ok(t),
        ControlRequestReply::Error(err) => bail!("{err}"),
        other => bail!("unexpected reply: {other:?}"),
    }
}

pub(super) fn format_duration_us(us: u64) -> String {
    if us >= 1_000_000 {
        format!("{:.3}s", us as f64 / 1_000_000.0)
    } else if us >= 1_000 {
        format!("{:.1}ms", us as f64 / 1_000.0)
    } else {
        format!("{us}us")
    }
}
