//! Serving `GetTraces`/`GetTraceSpans` from the in-memory span store.

use crate::SpanStore;
use dora_message::coordinator_to_cli::ControlRequestReply;

#[cfg(feature = "tracing")]
#[allow(clippy::unnecessary_sort_by)]
pub(crate) fn handle_get_traces(span_store: &SpanStore) -> ControlRequestReply {
    use dora_message::coordinator_to_cli::TraceSummary;
    use std::collections::HashMap;

    let Some(store) = span_store else {
        return ControlRequestReply::TraceList(Vec::new());
    };

    // Snapshot spans under the lock, then release immediately.
    let records: Vec<_> = match store.lock() {
        Ok(store) => store.spans().iter().cloned().collect(),
        Err(e) => {
            tracing::warn!("span store mutex poisoned: {e}");
            return ControlRequestReply::TraceList(Vec::new());
        }
    };

    // Group spans by trace_id.
    let mut groups: HashMap<&str, Vec<&dora_tracing::span_store::SpanRecord>> = HashMap::new();
    for span in &records {
        groups.entry(&span.trace_id).or_default().push(span);
    }

    let mut summaries: Vec<TraceSummary> = groups
        .into_iter()
        .map(|(trace_id, spans)| {
            let root = spans
                .iter()
                .find(|s| s.parent_span_id.is_none())
                .unwrap_or(&spans[0]);
            let start_time = spans.iter().map(|s| s.start_time).min().unwrap_or(0);
            TraceSummary {
                trace_id: trace_id.to_string(),
                root_span_name: root.name.clone(),
                span_count: spans.len(),
                start_time,
                total_duration_us: root.duration_us,
            }
        })
        .collect();

    // Newest first.
    summaries.sort_by(|a, b| b.start_time.cmp(&a.start_time));
    ControlRequestReply::TraceList(summaries)
}

#[cfg(not(feature = "tracing"))]
pub(crate) fn handle_get_traces(_span_store: &SpanStore) -> ControlRequestReply {
    ControlRequestReply::TraceList(Vec::new())
}

#[cfg(feature = "tracing")]
pub(crate) fn handle_get_trace_spans(
    span_store: &SpanStore,
    trace_id: &str,
) -> ControlRequestReply {
    use dora_message::coordinator_to_cli::TraceSpan;

    let Some(store) = span_store else {
        return ControlRequestReply::TraceSpans(Vec::new());
    };

    // Snapshot matching spans under the lock, then release immediately.
    let records: Vec<_> = match store.lock() {
        Ok(store) => store
            .spans()
            .iter()
            .filter(|s| s.trace_id == trace_id)
            .cloned()
            .collect(),
        Err(e) => {
            tracing::warn!("span store mutex poisoned: {e}");
            return ControlRequestReply::TraceSpans(Vec::new());
        }
    };

    let spans: Vec<TraceSpan> = records
        .into_iter()
        .map(|s| TraceSpan {
            trace_id: s.trace_id,
            span_id: s.span_id,
            parent_span_id: s.parent_span_id,
            name: s.name,
            target: s.target,
            level: s.level,
            start_time: s.start_time,
            duration_us: s.duration_us,
            fields: s.fields,
        })
        .collect();

    ControlRequestReply::TraceSpans(spans)
}

#[cfg(not(feature = "tracing"))]
pub(crate) fn handle_get_trace_spans(
    _span_store: &SpanStore,
    _trace_id: &str,
) -> ControlRequestReply {
    ControlRequestReply::TraceSpans(Vec::new())
}
