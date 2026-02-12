use indicatif::{ProgressBar, ProgressStyle};
use std::io::IsTerminal;
use std::time::Duration;

/// Progress reporter that automatically detects TTY and disables in non-interactive environments
pub struct ProgressReporter {
    enabled: bool,
}

impl ProgressReporter {
    /// Create a new progress reporter
    ///
    /// Progress bars are enabled only when stdout is a TTY (interactive terminal).
    /// This ensures progress bars don't pollute output in CI or when piped.
    pub fn new() -> Self {
        Self {
            enabled: std::io::stdout().is_terminal(),
        }
    }

    /// Create a spinner with the given message
    ///
    /// Returns a ProgressHandle that will show a spinner in interactive mode,
    /// or be a no-op in non-interactive mode.
    pub fn spinner(&self, message: impl Into<String>) -> ProgressHandle {
        if self.enabled {
            let pb = ProgressBar::new_spinner();
            // Use ASCII-safe spinner on Windows to avoid console issues
            let tick_strings: &[&str] = if cfg!(windows) {
                &["|", "/", "-", "\\"]
            } else {
                &["⠋", "⠙", "⠹", "⠸", "⠼", "⠴", "⠦", "⠧", "⠇", "⠏"]
            };
            pb.set_style(
                ProgressStyle::default_spinner()
                    .template("{spinner:.green} {msg}")
                    .unwrap()
                    .tick_strings(tick_strings),
            );
            pb.set_message(message.into());
            pb.enable_steady_tick(Duration::from_millis(100));
            ProgressHandle {
                inner: Some(pb),
                enabled: true,
                state: std::cell::Cell::new(ProgressState::Active),
            }
        } else {
            ProgressHandle {
                inner: None,
                enabled: false,
                state: std::cell::Cell::new(ProgressState::Active),
            }
        }
    }

    /// Create a progress bar with the given total and message
    ///
    /// Returns a ProgressHandle that will show a progress bar in interactive mode,
    /// or be a no-op in non-interactive mode.
    pub fn progress_bar(&self, total: u64, message: impl Into<String>) -> ProgressHandle {
        if self.enabled {
            let pb = ProgressBar::new(total);
            pb.set_style(
                ProgressStyle::default_bar()
                    .template("{msg} [{bar:40.cyan/blue}] {pos}/{len}")
                    .unwrap()
                    .progress_chars("=>-"),
            );
            pb.set_message(message.into());
            ProgressHandle {
                inner: Some(pb),
                enabled: true,
                state: std::cell::Cell::new(ProgressState::Active),
            }
        } else {
            ProgressHandle {
                inner: None,
                enabled: false,
                state: std::cell::Cell::new(ProgressState::Active),
            }
        }
    }
}

impl Default for ProgressReporter {
    fn default() -> Self {
        Self::new()
    }
}

/// State of a progress operation
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ProgressState {
    /// Operation is still active (default state)
    Active,
    /// Operation completed successfully
    Success,
    /// Operation failed or was abandoned
    Failed,
}

/// Handle to a progress indicator
///
/// This wraps an optional indicatif ProgressBar and provides a unified interface
/// that works in both interactive and non-interactive modes.
///
/// IMPORTANT: Success must be explicit via `finish_with_message()`.
/// If dropped without explicit success, the operation is treated as failed.
pub struct ProgressHandle {
    inner: Option<ProgressBar>,
    enabled: bool,
    state: std::cell::Cell<ProgressState>,
}

impl ProgressHandle {
    /// Update the progress message
    pub fn set_message(&self, message: impl Into<String>) {
        if let Some(pb) = &self.inner {
            pb.set_message(message.into());
        }
    }

    /// Increment the progress position by 1
    pub fn inc(&self, delta: u64) {
        if let Some(pb) = &self.inner {
            pb.inc(delta);
        }
    }

    /// Set the progress position
    pub fn set_position(&self, pos: u64) {
        if let Some(pb) = &self.inner {
            pb.set_position(pos);
        }
    }

    /// Finish the progress indicator with a success message
    ///
    /// This marks the operation as successful. If not called, Drop will treat
    /// the operation as failed.
    pub fn finish_with_message(&self, message: impl Into<String>) {
        self.state.set(ProgressState::Success);
        if let Some(pb) = &self.inner {
            pb.finish_with_message(message.into());
        }
    }

    /// Finish the progress indicator and clear it
    ///
    /// This marks the operation as successful without a message.
    pub fn finish_and_clear(&self) {
        self.state.set(ProgressState::Success);
        if let Some(pb) = &self.inner {
            pb.finish_and_clear();
        }
    }

    /// Abandon the progress indicator with a failure message
    ///
    /// This explicitly marks the operation as failed.
    pub fn abandon_with_message(&self, message: impl Into<String>) {
        self.state.set(ProgressState::Failed);
        if let Some(pb) = &self.inner {
            pb.abandon_with_message(message.into());
        }
    }

    /// Check if progress reporting is enabled
    pub fn is_enabled(&self) -> bool {
        self.enabled
    }
}

impl Drop for ProgressHandle {
    fn drop(&mut self) {
        // If the operation was not explicitly marked as success or failure,
        // treat it as failed (e.g., due to early return or panic)
        if self.state.get() == ProgressState::Active {
            if let Some(pb) = &self.inner {
                // Silently clear the progress bar to avoid showing success
                // when the operation didn't complete successfully.
                // Use finish_and_clear() instead of abandon to avoid any
                // potential stdout/stderr writes during Drop that could
                // interfere with process shutdown on Windows.
                pb.finish_and_clear();
            }
        }
    }
}
