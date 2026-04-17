use dora_message::{cli_to_coordinator::ControlRequest, coordinator_to_cli::TraceSummary};

use crate::{
    command::Executable,
    common::{expect_reply, send_control_request},
    ws_client::WsSession,
};

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
    let reply = send_control_request(session, &ControlRequest::GetTraces)?;
    Ok(expect_reply!(reply, TraceList(t))?)
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
