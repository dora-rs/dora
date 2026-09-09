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

    stderr.into_iter().skip(start_line_idx).collect()
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

        for marker in MARKERS.iter() {
            if line.starts_with(marker) {
                return true;
            }
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

        for marker in NEGATIVE_MARKERS.iter() {
            if line.starts_with(marker) {
                return true;
            }
        }

        false
    }
}

#[cfg(test)]
mod tests {
    use super::extract_err_from_stderr;

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
}
