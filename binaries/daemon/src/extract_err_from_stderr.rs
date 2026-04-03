pub fn extract_err_from_stderr(stderr: Vec<String>) -> String {
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
    // default to last 10 lines if no start marker was found
    let start_line_idx_from_end = start_line_idx_from_end.unwrap_or(10);

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
