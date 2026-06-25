use std::collections::{HashMap, HashSet};

use clap::Args;
use dora_message::{cli_to_coordinator::ControlRequest, coordinator_to_cli::TraceSpan};
use eyre::bail;

use crate::{
    command::{Executable, default_tracing, trace::format_duration_us},
    common::{CoordinatorOptions, expect_reply, send_control_request},
    ws_client::WsSession,
};

/// View spans for a specific trace.
///
/// Supports prefix matching on trace IDs.
///
/// Examples:
///
///   dora trace view a1b2c3d4
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

        let reply = send_control_request(&session, &ControlRequest::GetTraceSpans { trace_id })?;
        let spans = expect_reply!(reply, TraceSpans(s))?;

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
    let DisplayPlan { rows, partial } = plan_span_tree(spans);

    if partial {
        eprintln!(
            "note: trace is partial — some spans reference a parent that is absent from this \
             snapshot or form a cycle (shown as top-level roots)"
        );
    }

    for (span, depth) in rows {
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
        println!("{indent}{} [{} {}]{fields}", span.name, span.level, dur);
    }
}

/// The flattened, depth-annotated render order for a span snapshot, plus a flag
/// indicating whether the trace is incomplete (orphaned or cyclic spans).
struct DisplayPlan<'a> {
    rows: Vec<(&'a TraceSpan, usize)>,
    partial: bool,
}

/// Compute the render order for `spans`, ensuring every span appears exactly
/// once even when the snapshot is incomplete or malformed.
///
/// Two failure modes are handled so spans are never silently dropped:
/// - **orphans**: a span whose parent is absent from the snapshot (ring-buffer
///   eviction, distributed trace, missing root) is promoted to a top-level root.
/// - **cycles**: a parent-pointer cycle whose members are *all* present (e.g.
///   A→B→A) has no span that qualifies as a root above; such leftover spans are
///   surfaced from their earliest member instead of being dropped.
fn plan_span_tree(spans: &[TraceSpan]) -> DisplayPlan<'_> {
    let mut children: HashMap<Option<u64>, Vec<&TraceSpan>> = HashMap::new();
    for span in spans {
        children.entry(span.parent_span_id).or_default().push(span);
    }

    for list in children.values_mut() {
        list.sort_by_key(|s| s.start_time);
    }

    let present: HashSet<u64> = spans.iter().map(|s| s.span_id).collect();
    let mut roots: Vec<&TraceSpan> = spans
        .iter()
        .filter(|s| s.parent_span_id.is_none_or(|p| !present.contains(&p)))
        .collect();
    roots.sort_by_key(|s| s.start_time);

    let orphaned = roots.iter().any(|r| r.parent_span_id.is_some());

    let mut rows = Vec::new();
    let mut visited = HashSet::new();
    for root in &roots {
        collect_rows(root, &children, 0, &mut visited, &mut rows);
    }

    // Any span still unvisited belongs to an all-present parent-pointer cycle,
    // so it was never reached from a root above. Surface each leftover span
    // (earliest first) rather than dropping it.
    let mut leftover: Vec<&TraceSpan> = spans
        .iter()
        .filter(|s| !visited.contains(&s.span_id))
        .collect();
    leftover.sort_by_key(|s| s.start_time);
    let cyclic = !leftover.is_empty();
    for span in leftover {
        collect_rows(span, &children, 0, &mut visited, &mut rows);
    }

    DisplayPlan {
        rows,
        partial: orphaned || cyclic,
    }
}

fn collect_rows<'a>(
    span: &'a TraceSpan,
    children: &HashMap<Option<u64>, Vec<&'a TraceSpan>>,
    depth: usize,
    visited: &mut HashSet<u64>,
    rows: &mut Vec<(&'a TraceSpan, usize)>,
) {
    if !visited.insert(span.span_id) {
        return; // cycle guard: skip already-visited spans
    }
    rows.push((span, depth));

    if let Some(kids) = children.get(&Some(span.span_id)) {
        for child in kids {
            collect_rows(child, children, depth + 1, visited, rows);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn span(span_id: u64, parent_span_id: Option<u64>, start_time: u64) -> TraceSpan {
        TraceSpan {
            trace_id: "t".to_string(),
            span_id,
            parent_span_id,
            name: format!("span{span_id}"),
            target: "test".to_string(),
            level: "INFO".to_string(),
            start_time,
            duration_us: 0,
            fields: Vec::new(),
        }
    }

    fn rendered(spans: &[TraceSpan]) -> (Vec<(u64, usize)>, bool) {
        let plan = plan_span_tree(spans);
        let rows = plan.rows.iter().map(|(s, d)| (s.span_id, *d)).collect();
        (rows, plan.partial)
    }

    #[test]
    fn nests_children_under_their_root() {
        let spans = vec![span(1, None, 0), span(2, Some(1), 1), span(3, Some(1), 2)];
        let (rows, partial) = rendered(&spans);
        assert_eq!(rows, vec![(1, 0), (2, 1), (3, 1)]);
        assert!(!partial, "a complete trace is not partial");
    }

    #[test]
    fn promotes_orphan_whose_parent_is_absent() {
        // Parent span 99 is not in the snapshot, so span 1 is an orphan root.
        let spans = vec![span(1, Some(99), 0), span(2, Some(1), 1)];
        let (rows, partial) = rendered(&spans);
        assert_eq!(rows, vec![(1, 0), (2, 1)]);
        assert!(partial, "an orphaned span marks the trace partial");
    }

    #[test]
    fn surfaces_all_present_cycle_instead_of_dropping_it() {
        // A→B→A: both present, neither qualifies as a root. Without leftover
        // promotion the whole cycle would be silently dropped (issue #2301).
        let spans = vec![span(1, Some(2), 0), span(2, Some(1), 1)];
        let (rows, partial) = rendered(&spans);
        let ids: HashSet<u64> = rows.iter().map(|(id, _)| *id).collect();
        assert_eq!(ids, HashSet::from([1, 2]), "every cyclic span is shown");
        assert_eq!(rows.len(), 2, "each span appears exactly once");
        assert!(partial, "a cyclic trace marks the trace partial");
    }

    #[test]
    fn real_tree_alongside_isolated_cycle_renders_both_once() {
        // A normal root (1 → 2) plus a disjoint all-present cycle (3 ↔ 4).
        // Both must appear, each span exactly once, and the trace is partial.
        let spans = vec![
            span(1, None, 0),
            span(2, Some(1), 1),
            span(3, Some(4), 2),
            span(4, Some(3), 3),
        ];
        let (rows, partial) = rendered(&spans);
        assert_eq!(rows.len(), 4, "each span appears exactly once");
        let ids: HashSet<u64> = rows.iter().map(|(id, _)| *id).collect();
        assert_eq!(ids, HashSet::from([1, 2, 3, 4]), "every span is shown");
        // The real subtree keeps its nesting; cycle members surface as roots.
        assert_eq!(rows[0], (1, 0));
        assert_eq!(rows[1], (2, 1));
        assert!(partial);
    }
}
