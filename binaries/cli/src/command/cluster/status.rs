use std::io::Write;

use clap::Args;
use eyre::Context;
use tabwriter::TabWriter;

use crate::{
    command::{Executable, default_tracing},
    common::{CoordinatorOptions, query_running_dataflows},
};

use super::query_connected_daemons;

/// Show the current status of the cluster.
///
/// Displays connected daemons and active dataflow count.
///
/// Examples:
///
///   adora cluster status
#[derive(Debug, Args)]
#[clap(verbatim_doc_comment)]
pub struct Status {
    #[clap(flatten)]
    coordinator: CoordinatorOptions,
}

impl Executable for Status {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;
        let session = self.coordinator.connect()?;

        let daemons = query_connected_daemons(&session)?;
        let dataflows = query_running_dataflows(&session)?;
        let active_count = dataflows.get_active().len();

        if daemons.is_empty() {
            println!("No daemons connected.");
        } else {
            let mut tw = TabWriter::new(std::io::stdout().lock());
            tw.write_all(b"DAEMON ID\tLAST HEARTBEAT\n")
                .context("write")?;
            for d in &daemons {
                tw.write_all(
                    format!("{}\t{}ms ago\n", d.daemon_id, d.last_heartbeat_ago_ms).as_bytes(),
                )
                .context("write")?;
            }
            tw.flush().context("flush")?;
        }

        println!("\nActive dataflows: {active_count}");
        Ok(())
    }
}
