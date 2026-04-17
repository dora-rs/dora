use std::io::Write;

use clap::Args;
use dora_message::coordinator_to_cli::TraceSummary;
use tabwriter::TabWriter;

use crate::{
    command::{Executable, default_tracing, trace::format_duration_us},
    common::CoordinatorOptions,
};

/// List recent traces captured by the coordinator.
///
/// Examples:
///
///   dora trace list
#[derive(Debug, Args)]
#[clap(verbatim_doc_comment)]
pub struct List {
    #[clap(flatten)]
    coordinator: CoordinatorOptions,
}

impl Executable for List {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;
        let session = self.coordinator.connect()?;
        let traces = super::fetch_traces(&session)?;

        if traces.is_empty() {
            println!("No traces captured yet.");
            return Ok(());
        }

        print_trace_table(&traces)
    }
}

fn print_trace_table(traces: &[TraceSummary]) -> eyre::Result<()> {
    let mut tw = TabWriter::new(std::io::stdout().lock());
    tw.write_all(b"TRACE ID\tROOT SPAN\tSPANS\tSTARTED\tDURATION\n")?;
    for t in traces {
        tw.write_all(
            format!(
                "{}\t{}\t{}\t{}\t{}\n",
                &t.trace_id[..12.min(t.trace_id.len())],
                t.root_span_name,
                t.span_count,
                format_unix_millis(t.start_time),
                format_duration_us(t.total_duration_us),
            )
            .as_bytes(),
        )?;
    }
    tw.flush()?;
    Ok(())
}

fn format_unix_millis(ms: u64) -> String {
    chrono::DateTime::from_timestamp_millis(ms as i64)
        .map(|dt| dt.format("%Y-%m-%d %H:%M:%S").to_string())
        .unwrap_or_else(|| ms.to_string())
}
