use std::collections::HashMap;

use adora_message::{
    cli_to_coordinator::ControlRequest,
    coordinator_to_cli::{ControlRequestReply, TraceSpan},
};
use clap::Args;
use eyre::{Context, bail};

use crate::{
    command::{Executable, default_tracing, trace::format_duration_us},
    common::CoordinatorOptions,
    ws_client::WsSession,
};

/// View spans for a specific trace.
///
/// Supports prefix matching on trace IDs.
///
/// Examples:
///
///   adora trace view a1b2c3d4
#[derive(Debug, Args)]
#[clap(verbatim_doc_comment)]
pub struct View {
    /// Trace ID (or unique prefix)
    pub trace_id: String,

    #[clap(flatten)]
    coordinator: CoordinatorOptions,
}

impl Executable for View {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;
        let session = self.coordinator.connect()?;

        let trace_id = resolve_trace_id(&session, &self.trace_id)?;

        let reply_raw = session
            .request(&serde_json::to_vec(&ControlRequest::GetTraceSpans { trace_id }).unwrap())
            .wrap_err("failed to send GetTraceSpans request")?;
        let reply: ControlRequestReply =
            serde_json::from_slice(&reply_raw).wrap_err("failed to parse reply")?;

        let spans = match reply {
            ControlRequestReply::TraceSpans(s) => s,
            ControlRequestReply::Error(err) => bail!("{err}"),
            other => bail!("unexpected reply: {other:?}"),
        };

        if spans.is_empty() {
            println!("No spans found for this trace.");
            return Ok(());
        }

        print_span_tree(&spans);
        Ok(())
    }
}

/// Resolve a possibly-prefix trace_id by querying GetTraces.
fn resolve_trace_id(session: &WsSession, prefix: &str) -> eyre::Result<String> {
    if prefix.is_empty() {
        bail!("trace ID prefix must not be empty");
    }

    // Full UUID — skip the round-trip.
    if uuid::Uuid::parse_str(prefix).is_ok() {
        return Ok(prefix.to_string());
    }

    let traces = super::fetch_traces(session)?;

    let matches: Vec<_> = traces
        .iter()
        .filter(|t| t.trace_id.starts_with(prefix))
        .collect();

    match matches.len() {
        0 => bail!("no trace found matching prefix `{prefix}`"),
        1 => Ok(matches[0].trace_id.clone()),
        n => bail!("prefix `{prefix}` is ambiguous ({n} matches). Use a longer prefix."),
    }
}

fn print_span_tree(spans: &[TraceSpan]) {
    let mut children: HashMap<Option<u64>, Vec<&TraceSpan>> = HashMap::new();
    for span in spans {
        children.entry(span.parent_span_id).or_default().push(span);
    }

    for list in children.values_mut() {
        list.sort_by_key(|s| s.start_time);
    }

    if let Some(roots) = children.get(&None) {
        for root in roots {
            print_span(root, &children, 0);
        }
    }
}

fn print_span(span: &TraceSpan, children: &HashMap<Option<u64>, Vec<&TraceSpan>>, depth: usize) {
    let indent = "  ".repeat(depth);
    let dur = format_duration_us(span.duration_us);
    let fields = if span.fields.is_empty() {
        String::new()
    } else {
        let pairs: Vec<_> = span
            .fields
            .iter()
            .map(|(k, v)| format!("{k}={v}"))
            .collect();
        format!(" {{{}}}", pairs.join(", "))
    };
    println!("{indent}{} [{} {}]{fields}", span.name, span.level, dur,);

    if let Some(kids) = children.get(&Some(span.span_id)) {
        for child in kids {
            print_span(child, children, depth + 1);
        }
    }
}
