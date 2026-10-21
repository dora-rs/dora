use dora_message::{
    common::{NodeError, NodeErrorCause},
    daemon_to_coordinator::DataflowDaemonResult,
};

/// Largest error cause reported for one node.
///
/// The cause travels to the coordinator inside the dataflow's finish report,
/// and the coordinator closes any daemon message over 1 MiB. Stderr lines are
/// only capped at 1 MiB each, so without this bound one long line (a large
/// exception repr, a dumped buffer) would drop the daemon connection, and the
/// same report is resent on every reconnect.
/// The bound is per node, and the report holds one cause per failed node, so
/// [`encode_bounded_finish_report`] bounds the report as a whole.
const MAX_ERROR_CAUSE_BYTES: usize = 16 * 1024;

/// Largest encoded finish report: the coordinator's 1 MiB limit for daemon
/// messages (`MAX_CONTROL_MESSAGE_BYTES`), minus headroom.
const MAX_FINISH_REPORT_BYTES: usize = 1024 * 1024 - 64 * 1024;

/// Encode a dataflow's finish report with `encode`, shrinking the failed
/// nodes' causes until the encoding fits in [`MAX_FINISH_REPORT_BYTES`].
///
/// [`MAX_ERROR_CAUSE_BYTES`] alone does not bound the report: it holds one
/// cause per failed node, and JSON escaping grows control characters (ANSI
/// colours) up to sixfold. An oversized report drops the daemon connection
/// and is resent on every reconnect, so it would never be recorded (#3900).
/// The per-node budget is halved until the report fits, down to only the
/// elision markers.
pub fn encode_bounded_finish_report(
    result: &DataflowDaemonResult,
    encode: impl Fn(&DataflowDaemonResult) -> eyre::Result<Vec<u8>>,
) -> eyre::Result<Vec<u8>> {
    let msg = encode(result)?;
    if msg.len() <= MAX_FINISH_REPORT_BYTES {
        return Ok(msg);
    }
    let mut budget = MAX_ERROR_CAUSE_BYTES;
    loop {
        budget /= 2;
        // elide from the original each round, so the marker counts every
        // dropped byte
        let mut bounded = result.clone();
        for node_result in bounded.node_results.values_mut() {
            if let Err(NodeError {
                cause: NodeErrorCause::Other { stderr: text } | NodeErrorCause::FailedToSpawn(text),
                ..
            }) = node_result
            {
                *text = elide_middle(std::mem::take(text), budget);
            }
        }
        let msg = encode(&bounded)?;
        if msg.len() <= MAX_FINISH_REPORT_BYTES || budget == 0 {
            return Ok(msg);
        }
    }
}

pub fn extract_err_from_stderr(stderr: Vec<String>) -> String {
    const FALLBACK_LINES: usize = 10;
    let mut matcher = StderrMatcher::default();

    // Try to find the start of the error message by scanning the captured
    // lines in reverse for known markers.
    //
    // A positive marker (`Error:`, a panic line, a Python traceback, …) is
    // always preferred: it points at where the error actually begins. A
    // negative marker (`Warning:`) is *not* part of an error message; it only
    // bounds the error region, so the error — if any — is the text that follows
    // it. We therefore remember the closest-to-end negative marker as a
    // fallback boundary but keep scanning further back for a positive marker,
    // so a warning printed *after* a panic (e.g. during shutdown/unwind) can no
    // longer swallow the panic that preceded it (#3411).
    let mut positive_idx_from_end = None;
    let mut negative_idx_from_end = None;
    for (idx, line) in stderr.iter().rev().enumerate() {
        if matcher.is_error_start_marker(line) {
            positive_idx_from_end = Some(idx);
            break;
        }
        // Only the first (closest-to-end) negative marker matters as a boundary;
        // record it but keep looking for a real error marker further back.
        if negative_idx_from_end.is_none() && matcher.is_negative_marker(line) {
            negative_idx_from_end = Some(idx);
        }
    }

    let start_line_idx_from_end = match (positive_idx_from_end, negative_idx_from_end) {
        // A real error marker wins, wherever it sits relative to the warning.
        (Some(idx), _) => idx,
        // No positive marker: the error is the text *after* the warning line, so
        // start one line closer to the end and exclude the warning itself. If
        // the warning is the very last line there is no such text, so fall back.
        (None, Some(0)) => FALLBACK_LINES - 1,
        (None, Some(idx)) => idx - 1,
        // Default to the last FALLBACK_LINES lines if no marker was found.
        (None, None) => FALLBACK_LINES - 1,
    };

    let start_line_idx = stderr
        .len()
        .saturating_sub(1)
        .saturating_sub(start_line_idx_from_end);

    elide_middle(
        stderr.into_iter().skip(start_line_idx).collect(),
        MAX_ERROR_CAUSE_BYTES,
    )
}

/// Shorten `text` to about `max` bytes by keeping its head (where the error
/// marker is) and its tail (where a Python traceback names the exception) and
/// replacing the middle with an elision marker.
fn elide_middle(mut text: String, max: usize) -> String {
    if text.len() <= max {
        return text;
    }
    let head_end = text.floor_char_boundary(max / 2);
    let tail_start = text.ceil_char_boundary(text.len() - max / 2);
    let marker = format!("\n[... {} bytes elided ...]\n", tail_start - head_end);
    text.replace_range(head_end..tail_start, &marker);
    text
}

#[derive(Debug, Clone, Default)]
struct StderrMatcher {
    python_syntax_error_found: bool,
}

impl StderrMatcher {
    fn is_error_start_marker(&mut self, line: &str) -> bool {
        const MARKERS: &[&str] = &[
            // common in the Rust world
            "Error:",
            // unhandled Python exceptions
            "Traceback (most recent call last):",
        ];

        if MARKERS.iter().any(|marker| line.starts_with(marker)) {
            return true;
        }

        if line.starts_with("SyntaxError: ") {
            self.python_syntax_error_found = true;
            return false;
        }

        if self.python_syntax_error_found && line.starts_with("  File ") {
            return true;
        }

        if line.starts_with("thread '") && line.contains(" panicked at ") {
            return true;
        }

        false
    }

    fn is_negative_marker(&self, line: &str) -> bool {
        // common non-error log lines
        const NEGATIVE_MARKERS: &[&str] = &["Warning:"];

        NEGATIVE_MARKERS
            .iter()
            .any(|marker| line.starts_with(marker))
    }
}

#[cfg(test)]
mod tests {
    use super::{
        MAX_ERROR_CAUSE_BYTES, MAX_FINISH_REPORT_BYTES, encode_bounded_finish_report,
        extract_err_from_stderr,
    };
    use dora_message::{
        common::{NodeError, NodeErrorCause, NodeExitStatus},
        daemon_to_coordinator::DataflowDaemonResult,
    };

    fn lines(s: &str) -> Vec<String> {
        s.lines().map(|l| format!("{}\n", l)).collect()
    }

    // T1: Positive marker — Error:
    #[test]
    fn test_error_marker() {
        let input = lines("log1\nError: something\nmore");
        let result = extract_err_from_stderr(input);
        assert_eq!(result, "Error: something\nmore\n");
    }

    // T2: Positive marker — Traceback
    #[test]
    fn test_traceback_marker() {
        let input =
            lines("log\nTraceback (most recent call last):\n  File x.py, line 1\nValueError: bad");
        let result = extract_err_from_stderr(input);
        assert_eq!(
            result,
            "Traceback (most recent call last):\n  File x.py, line 1\nValueError: bad\n"
        );
    }

    // T3: Positive marker — thread panicked
    #[test]
    fn test_panic_marker() {
        let input =
            lines("log\nthread 'main' panicked at src/main.rs:42\nnote: run with RUST_BACKTRACE=1");
        let result = extract_err_from_stderr(input);
        assert_eq!(
            result,
            "thread 'main' panicked at src/main.rs:42\nnote: run with RUST_BACKTRACE=1\n"
        );
    }

    // T4: Negative marker — Warning: followed by Error:
    #[test]
    fn test_negative_marker_warning() {
        let input = lines("info\nWarning: deprecated\nError: crash\ntrace");
        let result = extract_err_from_stderr(input);
        assert_eq!(result, "Error: crash\ntrace\n");
    }

    // T5: Positive marker followed by a trailing Warning — the earlier `Error:`
    // wins over the closer negative marker instead of being swallowed by it.
    #[test]
    fn test_negative_marker_last_line() {
        let input = lines("Error: something\nWarning: done");
        let result = extract_err_from_stderr(input);
        assert_eq!(result, "Error: something\nWarning: done\n");
    }

    // T6: Python SyntaxError with File line — real Python output order
    #[test]
    fn test_python_syntax_error_with_file() {
        let input =
            lines("non-error\n  File \"t.py\", line 1\n    x =\nSyntaxError: invalid syntax");
        let result = extract_err_from_stderr(input);
        assert_eq!(
            result,
            "  File \"t.py\", line 1\n    x =\nSyntaxError: invalid syntax\n"
        );
    }

    // T7: Python SyntaxError without File line — falls to fallback
    #[test]
    fn test_python_syntax_error_without_file() {
        let input = lines("log\nSyntaxError: invalid syntax\nother");
        let result = extract_err_from_stderr(input);
        assert_eq!(result, "log\nSyntaxError: invalid syntax\nother\n");
    }

    // T8: Fallback — exactly 10 last lines when no marker found
    #[test]
    fn test_fallback_exact() {
        let input: Vec<String> = (1..=20).map(|i| format!("{}\n", i)).collect();
        let result = extract_err_from_stderr(input);
        let expected: String = (11..=20)
            .map(|i| format!("{}\n", i))
            .collect::<Vec<_>>()
            .concat();
        assert_eq!(result, expected);
    }

    // T9: Fallback — fewer than 10 lines returns all
    #[test]
    fn test_fallback_fewer_than_10() {
        let input = lines("1\n2\n3");
        let result = extract_err_from_stderr(input);
        assert_eq!(result, "1\n2\n3\n");
    }

    // T10: [...] truncation marker — no fusion with next line (no markers)
    #[test]
    fn test_truncation_marker_newline() {
        let input = vec!["[...]\n".to_string(), "line1\n".to_string()];
        let result = extract_err_from_stderr(input);
        assert_eq!(result, "[...]\nline1\n");
    }

    // T11: Multiple positive markers — reverse scan selects closest to end
    #[test]
    fn test_first_relevant_marker_used() {
        let input = lines("Error: first\nlog\nError: second");
        let result = extract_err_from_stderr(input);
        assert_eq!(result, "Error: second\n");
    }

    // T12: Empty input
    #[test]
    fn test_empty_input() {
        let result = extract_err_from_stderr(vec![]);
        assert_eq!(result, "");
    }

    // T13: [...] + positive marker coexistence — positive marker takes priority
    #[test]
    fn test_truncation_marker_with_positive_marker() {
        let input = vec![
            "[...]\n".to_string(),
            "log\n".to_string(),
            "Error: crash\n".to_string(),
            "trace\n".to_string(),
        ];
        let result = extract_err_from_stderr(input);
        assert_eq!(result, "Error: crash\ntrace\n");
    }

    // T14: Panic followed by a trailing `Warning:` — the panic must win over the
    // later negative marker, not be discarded by it (regression for #3411).
    #[test]
    fn test_panic_then_trailing_warning() {
        let input = lines(
            "thread 'main' panicked at src/lib.rs:10:5:\n\
             called `Option::unwrap()` on a `None` value\n\
             note: run with RUST_BACKTRACE=1 environment variable to display a backtrace\n\
             Warning: dropping connection during shutdown",
        );
        let result = extract_err_from_stderr(input);
        assert_eq!(
            result,
            "thread 'main' panicked at src/lib.rs:10:5:\n\
             called `Option::unwrap()` on a `None` value\n\
             note: run with RUST_BACKTRACE=1 environment variable to display a backtrace\n\
             Warning: dropping connection during shutdown\n"
        );
    }

    // T15: Negative marker with no positive marker — the error is the text that
    // follows the warning, and the warning line itself is excluded.
    #[test]
    fn test_negative_marker_only_excludes_warning() {
        let input = lines("startup ok\nWarning: retrying\nconnection refused\naborting");
        let result = extract_err_from_stderr(input);
        assert_eq!(result, "connection refused\naborting\n");
    }

    // An over-long cause is bounded so the finish report stays under the
    // coordinator's 1 MiB message limit, keeping both ends of the error.
    #[test]
    fn test_long_cause_is_bounded() {
        let input = vec![
            "Error: boom\n".to_string(),
            format!("{}\n", "é".repeat(1024 * 1024)),
            "ValueError: last line\n".to_string(),
        ];
        let result = extract_err_from_stderr(input);
        assert!(
            result.len() <= MAX_ERROR_CAUSE_BYTES + 64,
            "{}",
            result.len()
        );
        assert!(result.starts_with("Error: boom\n"));
        assert!(result.ends_with("ValueError: last line\n"));
        assert!(result.contains(" bytes elided ...]"));
    }

    fn report(failed_nodes: usize, cause: &str) -> DataflowDaemonResult {
        let clock = dora_message::uhlc::HLC::default();
        DataflowDaemonResult {
            timestamp: clock.new_timestamp(),
            node_results: (0..failed_nodes)
                .map(|i| {
                    let error = NodeError {
                        timestamp: clock.new_timestamp(),
                        cause: NodeErrorCause::Other {
                            stderr: cause.to_owned(),
                        },
                        exit_status: NodeExitStatus::ExitCode(1),
                    };
                    (format!("node-{i}").into(), Err(error))
                })
                .collect(),
        }
    }

    fn encode(result: &DataflowDaemonResult) -> eyre::Result<Vec<u8>> {
        Ok(serde_json::to_vec(result)?)
    }

    // Each cause is capped, but a report with enough failed nodes, and JSON
    // escaping of ANSI colours, still exceeded the coordinator's 1 MiB limit
    // (#3900).
    #[test]
    fn test_finish_report_with_many_failed_nodes_is_bounded() {
        let ansi = "\x1b[31mError:\x1b[0m boom\n".repeat(1024);
        let cause = extract_err_from_stderr(vec![
            "Error: first line\n".to_string(),
            ansi,
            "ValueError: last line\n".to_string(),
        ]);
        let result = report(100, &cause);
        assert!(encode(&result).unwrap().len() > 1024 * 1024);

        let msg = encode_bounded_finish_report(&result, encode).unwrap();
        assert!(msg.len() <= MAX_FINISH_REPORT_BYTES, "{}", msg.len());
        let bounded: DataflowDaemonResult = serde_json::from_slice(&msg).unwrap();
        assert_eq!(bounded.node_results.len(), 100);
        for node_result in bounded.node_results.values() {
            let Err(NodeError {
                cause: NodeErrorCause::Other { stderr },
                ..
            }) = node_result
            else {
                panic!("unexpected node result {node_result:?}");
            };
            assert!(stderr.starts_with("Error: first line\n"), "{stderr}");
            assert!(stderr.ends_with("ValueError: last line\n"), "{stderr}");
            assert!(stderr.contains(" bytes elided ...]"), "{stderr}");
        }
    }

    // A report that already fits is sent unchanged.
    #[test]
    fn test_small_finish_report_is_unchanged() {
        let result = report(3, &"Error: boom\n".repeat(1000));
        let msg = encode_bounded_finish_report(&result, encode).unwrap();
        assert_eq!(msg, encode(&result).unwrap());
    }

    // Even when the elision markers alone are left, the encoder terminates.
    #[test]
    fn test_finish_report_bounding_terminates() {
        let result = report(10_000, "Error: boom\n");
        let msg = encode_bounded_finish_report(&result, encode).unwrap();
        assert!(msg.len() > MAX_FINISH_REPORT_BYTES);
    }
}
