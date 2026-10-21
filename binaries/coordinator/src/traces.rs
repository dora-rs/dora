//! Serving `GetTraces`/`GetTraceSpans` from the in-memory span store.

use crate::SpanStore;
use dora_message::coordinator_to_cli::ControlRequestReply;

#[cfg(feature = "tracing")]
#[allow(clippy::unnecessary_sort_by)]
pub(crate) fn handle_get_traces(span_store: &SpanStore) -> ControlRequestReply {
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

    let mut summaries: Vec<_> = groups
        .into_iter()
        .map(|(trace_id, spans)| summarize_trace(trace_id, &spans))
        .collect();

    // Newest first.
    summaries.sort_by(|a, b| b.start_time.cmp(&a.start_time));
    ControlRequestReply::TraceList(summaries)
}

/// Summarizes the captured spans of one trace. `spans` must not be empty.
///
/// Spans are only recorded when they close, and the store evicts the oldest
/// ones, so the real root is missing while it is still open or after it was
/// evicted. The summary then names the earliest span whose parent is absent
/// from the group — the top of what was captured — and reports the time
/// covered by the captured spans instead of a single span's duration.
#[cfg(feature = "tracing")]
fn summarize_trace(
    trace_id: &str,
    spans: &[&dora_tracing::span_store::SpanRecord],
) -> dora_message::coordinator_to_cli::TraceSummary {
    use std::collections::HashSet;

    let start_time = spans.iter().map(|s| s.start_time).min().unwrap_or(0);
    let (root, total_duration_us) = match spans.iter().find(|s| s.parent_span_id.is_none()) {
        Some(root) => (root, root.duration_us),
        None => {
            let ids: HashSet<u64> = spans.iter().map(|s| s.span_id).collect();
            let root = spans
                .iter()
                .filter(|s| s.parent_span_id.is_some_and(|p| !ids.contains(&p)))
                .min_by_key(|s| s.start_time)
                // Every span has a present parent only if they form a cycle.
                .unwrap_or(&spans[0]);
            // `start_time` is in unix milliseconds, durations in microseconds.
            let end_us = spans
                .iter()
                .map(|s| {
                    s.start_time
                        .saturating_mul(1000)
                        .saturating_add(s.duration_us)
                })
                .max()
                .unwrap_or(0);
            (root, end_us.saturating_sub(start_time.saturating_mul(1000)))
        }
    };
    dora_message::coordinator_to_cli::TraceSummary {
        trace_id: trace_id.to_string(),
        root_span_name: root.name.clone(),
        span_count: spans.len(),
        start_time,
        total_duration_us,
    }
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

#[cfg(all(test, feature = "tracing"))]
mod tests {
    use super::*;
    use dora_tracing::span_store::SpanRecord;

    fn span(
        span_id: u64,
        parent_span_id: Option<u64>,
        start_time: u64,
        duration_us: u64,
    ) -> SpanRecord {
        SpanRecord {
            trace_id: "t".to_string(),
            span_id,
            parent_span_id,
            name: format!("span{span_id}"),
            target: "test".to_string(),
            level: "INFO".to_string(),
            start_time,
            duration_us,
            fields: Vec::new(),
        }
    }

    #[test]
    fn complete_trace_uses_root_span() {
        let spans = [span(2, Some(1), 1_001, 5), span(1, None, 1_000, 3_000)];
        let refs: Vec<_> = spans.iter().collect();
        let summary = summarize_trace("t", &refs);
        assert_eq!(summary.root_span_name, "span1");
        assert_eq!(summary.total_duration_us, 3_000);
        assert_eq!(summary.start_time, 1_000);
        assert_eq!(summary.span_count, 2);
    }

    #[test]
    fn missing_root_uses_topmost_captured_span_and_extent() {
        // `op` (1) is still open: the store holds `leaf` (3, closed first)
        // and `child_a` (2).
        let spans = [span(3, Some(2), 1_002, 4), span(2, Some(1), 1_001, 2_500)];
        let refs: Vec<_> = spans.iter().collect();
        let summary = summarize_trace("t", &refs);
        assert_eq!(summary.root_span_name, "span2");
        // From child_a's start (1_001 ms) to its end (+2_500 us).
        assert_eq!(summary.total_duration_us, 2_500);
        assert_eq!(summary.start_time, 1_001);
    }

    #[test]
    fn missing_root_picks_earliest_orphan_and_spans_all_of_them() {
        // Two sibling subtrees of an evicted root.
        let spans = [span(3, Some(1), 1_005, 1_000), span(2, Some(1), 1_000, 10)];
        let refs: Vec<_> = spans.iter().collect();
        let summary = summarize_trace("t", &refs);
        assert_eq!(summary.root_span_name, "span2");
        // 1_000 ms .. 1_005 ms + 1_000 us.
        assert_eq!(summary.total_duration_us, 6_000);
    }
}
