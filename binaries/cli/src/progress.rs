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
            pb.set_style(
                ProgressStyle::default_spinner()
                    .template("{spinner:.green} {msg}")
                    .unwrap()
                    .tick_strings(&["⠋", "⠙", "⠹", "⠸", "⠼", "⠴", "⠦", "⠧", "⠇", "⠏"]),
            );
            pb.set_message(message.into());
            pb.enable_steady_tick(Duration::from_millis(100));
            ProgressHandle {
                inner: Some(pb),
                enabled: true,
            }
        } else {
            ProgressHandle {
                inner: None,
                enabled: false,
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
            }
        } else {
            ProgressHandle {
                inner: None,
                enabled: false,
            }
        }
    }
}

impl Default for ProgressReporter {
    fn default() -> Self {
        Self::new()
    }
}

/// Handle to a progress indicator
/// 
/// This wraps an optional indicatif ProgressBar and provides a unified interface
/// that works in both interactive and non-interactive modes.
pub struct ProgressHandle {
    inner: Option<ProgressBar>,
    enabled: bool,
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
    pub fn finish_with_message(&self, message: impl Into<String>) {
        if let Some(pb) = &self.inner {
            pb.finish_with_message(message.into());
        }
    }

    /// Finish the progress indicator and clear it
    pub fn finish_and_clear(&self) {
        if let Some(pb) = &self.inner {
            pb.finish_and_clear();
        }
    }

    /// Abandon the progress indicator (for errors)
    pub fn abandon_with_message(&self, message: impl Into<String>) {
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
        // Ensure progress bar is cleaned up on drop
        if let Some(pb) = &self.inner {
            if !pb.is_finished() {
                pb.finish_and_clear();
            }
        }
    }
}
