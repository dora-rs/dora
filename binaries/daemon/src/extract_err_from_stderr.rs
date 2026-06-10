pub fn extract_err_from_stderr(stderr: Vec<String>) -> String {
    const FALLBACK_LINES: usize = 10;
    let mut matcher = StderrMatcher::default();

    // try to find the start of the error message by looking for known markers
    let mut start_line_idx_from_end = None;
    for (idx, line) in stderr.iter().rev().enumerate() {
        if matcher.is_error_start_marker(line) {
            start_line_idx_from_end = Some(idx);
            break;
        }
        // check if the line is a negative marker, i.e. definitely not part of an error message
        if matcher.is_negative_marker(line) {
            // we found a line that is definitely not part of an error message
            // -> take the line of the previous iteration as the start of the error message
            start_line_idx_from_end = Some(idx.saturating_sub(1));
            break;
        }
    }
    // default to last FALLBACK_LINES lines if no start marker was found
    let start_line_idx_from_end = start_line_idx_from_end.unwrap_or(FALLBACK_LINES - 1);

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

    // T5: Negative marker boundary — Warning is the last line (idx==0)
    #[test]
    fn test_negative_marker_last_line() {
        let input = lines("Error: something\nWarning: done");
        let result = extract_err_from_stderr(input);
        assert_eq!(result, "Warning: done\n");
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
}
