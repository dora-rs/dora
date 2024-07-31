use dora_core::topics::{ControlRequest, ControlRequestReply};
use eyre::{bail, Context, Result};
use uuid::Uuid;

use bat::{Input, PrettyPrinter};

use crate::DoraConnection;

pub fn dataflow_logs(
    connection: &mut DoraConnection,
    uuid: Option<Uuid>,
    name: Option<String>,
    node: String,
) -> Result<()> {
    let logs = {
        let reply_raw = connection
            .session
            .request(
                &serde_json::to_vec(&ControlRequest::Logs {
                    uuid,
                    name,
                    node: node.clone(),
                })
                .wrap_err("")?,
            )
            .wrap_err("failed to send Logs request message")?;

        let reply = serde_json::from_slice(&reply_raw).wrap_err("failed to parse reply")?;
        match reply {
            ControlRequestReply::Logs(logs) => logs,
            other => bail!("unexpected reply to daemon logs: {other:?}"),
        }
    };

    PrettyPrinter::new()
        .header(false)
        .grid(false)
        .line_numbers(false)
        .paging_mode(bat::PagingMode::QuitIfOneScreen)
        .inputs(vec![Input::from_bytes(&logs)
            .name("Logs")
            .title(format!("Logs from {node}.").as_str())])
        .print()
        .wrap_err("Something went wrong with viewing log file")?;

    Ok(())
}
