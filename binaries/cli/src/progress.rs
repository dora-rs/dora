use indicatif::{ProgressBar, ProgressStyle};

/// A spinner for indeterminate-length operations.
///
/// All output while a spinner is active should go through [`Spinner::println`]
/// so that the spinner stays pinned at the bottom of the terminal.
pub struct Spinner {
    bar: ProgressBar,
}

impl Spinner {
    pub fn new(msg: &str) -> Self {
        let bar = ProgressBar::new_spinner();
        bar.set_style(
            ProgressStyle::with_template("{spinner:.green} {msg}")
                .expect("invalid spinner template"),
        );
        bar.enable_steady_tick(std::time::Duration::from_millis(100));
        bar.set_message(msg.to_string());
        Self { bar }
    }

    pub fn finish_with_message(&self, msg: &str) {
        self.bar.finish_with_message(msg.to_string());
    }

    pub fn fail_with_message(&self, msg: &str) {
        self.bar.set_style(
            ProgressStyle::with_template("{spinner:.red} {msg}").expect("invalid spinner template"),
        );
        self.bar.finish_with_message(msg.to_string());
    }

    /// Print a line above the spinner without mangling its display.
    pub fn println(&self, msg: impl AsRef<str>) {
        self.bar.println(msg);
    }

    /// Get a clone of the underlying [`ProgressBar`] for sharing across threads.
    ///
    /// Use `ProgressBar::println` on the clone to print without mangling the
    /// spinner.
    pub fn clone_bar(&self) -> ProgressBar {
        self.bar.clone()
    }
}

/// A progress bar for counted operations.
///
/// All output while the bar is active should go through [`BuildProgress::println`]
/// so that the bar stays pinned at the bottom of the terminal.
pub struct BuildProgress {
    bar: ProgressBar,
}

impl BuildProgress {
    pub fn new(total: u64, msg: &str) -> Self {
        let bar = ProgressBar::new(total);
        bar.set_style(
            ProgressStyle::with_template("{spinner:.green} {msg} [{bar:30.cyan/blue}] {pos}/{len}")
                .expect("invalid progress bar template")
                .progress_chars("=>-"),
        );
        bar.set_message(msg.to_string());
        Self { bar }
    }

    pub fn inc(&self, n: u64) {
        self.bar.inc(n);
    }

    pub fn finish(&self) {
        self.bar.finish_with_message("Build complete");
    }

    /// Print a line above the progress bar without mangling its display.
    pub fn println(&self, msg: impl AsRef<str>) {
        self.bar.println(msg);
    }

    /// Get a clone of the underlying [`ProgressBar`] for sharing across threads.
    pub fn clone_bar(&self) -> ProgressBar {
        self.bar.clone()
    }
}
