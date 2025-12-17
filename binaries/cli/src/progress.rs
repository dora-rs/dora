//! Shared progress bar components for time-consuming CLI operations.
//!
//! This module provides progress indicators using the [`indicatif`](https://docs.rs/indicatif/) crate.
//!
//! # Features
//!
//! - **Spinners**: For tasks with unknown duration (see [`ProgressBar::new_spinner`])
//! - **Progress bars**: For tasks with known steps (see [`ProgressBar::new`])
//! - **Multi-progress**: Display multiple concurrent operations (see [`MultiProgress`])
//!
//! # Behavior
//!
//! Progress indicators are automatically shown on stderr. If stderr is not a terminal
//! (e.g., when output is redirected or in CI environments), progress indicators are hidden
//! and the program outputs normally without any visual artifacts.
//!
//! # Usage Examples
//!
//! ## Spinner for unknown duration
//!
//! ```rust,ignore
//! use crate::progress::ProgressBar;
//!
//! let pb = ProgressBar::new_spinner("Building nodes...");
//! // do work...
//! pb.finish_with_message("Build complete!");
//! ```
//!
//! ## Progress bar with known steps
//!
//! ```rust,ignore
//! use crate::progress::ProgressBar;
//!
//! let pb = ProgressBar::new(10, "Processing items");
//! for i in 0..10 {
//!     // do work...
//!     pb.inc(1);
//! }
//! pb.finish();
//! ```
//!
//! ## Multiple concurrent operations
//!
//! ```rust,ignore
//! use crate::progress::MultiProgress;
//!
//! let multi = MultiProgress::new();
//! let pb1 = multi.add_spinner("Cloning repository...");
//! let pb2 = multi.add_spinner("Building node...");
//! // do work...
//! pb1.finish();
//! pb2.finish();
//! ```

use indicatif::{
    MultiProgress as IndicatifMultiProgress, ProgressBar as IndicatifProgressBar, ProgressStyle,
};
use std::io::IsTerminal;
use std::time::Duration;

/// Standard tick rate for spinners (milliseconds)
const SPINNER_TICK_MS: u64 = 80;

/// Characters used for spinner animation
const SPINNER_CHARS: &str = "⠋⠙⠹⠸⠼⠴⠦⠧⠇⠏";

/// Template for spinner-style progress indicators
const SPINNER_TEMPLATE: &str = "{spinner:.cyan.bold} {msg}";

/// Template for progress bars with percentage
const PROGRESS_TEMPLATE: &str =
    "{spinner:.cyan.bold} {msg} [{bar:40.cyan/blue}] {pos}/{len} ({percent}%)";

/// Template for progress bars with bytes
const BYTES_TEMPLATE: &str =
    "{spinner:.cyan.bold} {msg} [{bar:40.cyan/blue}] {bytes}/{total_bytes} ({bytes_per_sec})";

/// Template for completed progress bars
const FINISHED_TEMPLATE: &str = "✓ {msg}";

/// Template for failed progress bars
const FAILED_TEMPLATE: &str = "✗ {msg}";

/// A progress bar for displaying task progress.
///
/// This is a wrapper around `indicatif::ProgressBar` with dora-specific styling.
pub struct ProgressBar {
    inner: IndicatifProgressBar,
}

impl ProgressBar {
    /// Creates a new spinner-style progress indicator.
    ///
    /// Spinners are ideal for tasks where the total work is unknown.
    ///
    /// # Arguments
    ///
    /// * `message` - The message to display next to the spinner
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// let pb = ProgressBar::new_spinner("Processing...");
    /// // do work...
    /// pb.finish_with_message("Done!");
    /// ```
    pub fn new_spinner(message: impl Into<String>) -> Self {
        let pb = if std::io::stderr().is_terminal() {
            IndicatifProgressBar::new_spinner()
        } else {
            IndicatifProgressBar::hidden()
        };

        pb.set_style(
            ProgressStyle::default_spinner()
                .tick_chars(SPINNER_CHARS)
                .template(SPINNER_TEMPLATE)
                .expect("invalid template"),
        );
        pb.enable_steady_tick(Duration::from_millis(SPINNER_TICK_MS));
        pb.set_message(message.into());
        Self { inner: pb }
    }

    /// Creates a new progress bar with a known length.
    ///
    /// # Arguments
    ///
    /// * `len` - The total number of items/steps
    /// * `message` - The message to display
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// let pb = ProgressBar::new(10, "Building nodes");
    /// for i in 0..10 {
    ///     // do work...
    ///     pb.inc(1);
    /// }
    /// pb.finish();
    /// ```
    pub fn new(len: u64, message: impl Into<String>) -> Self {
        Self::new_inner(len, message, PROGRESS_TEMPLATE)
    }

    /// Creates a new progress bar for byte-based progress (downloads, file copies, etc.).
    ///
    /// # Arguments
    ///
    /// * `total_bytes` - The total number of bytes
    /// * `message` - The message to display
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// let pb = ProgressBar::new_bytes(1024 * 1024, "Downloading");
    /// // during download...
    /// pb.inc(512);
    /// ```
    pub fn new_bytes(total_bytes: u64, message: impl Into<String>) -> Self {
        Self::new_inner(total_bytes, message, BYTES_TEMPLATE)
    }

    /// Internal helper to create progress bars with different templates
    fn new_inner(len: u64, message: impl Into<String>, template: &str) -> Self {
        let pb = if std::io::stderr().is_terminal() {
            IndicatifProgressBar::new(len)
        } else {
            IndicatifProgressBar::hidden()
        };

        pb.set_style(
            ProgressStyle::default_bar()
                .tick_chars(SPINNER_CHARS)
                .template(template)
                .expect("invalid template")
                .progress_chars("=>-"),
        );
        pb.enable_steady_tick(Duration::from_millis(SPINNER_TICK_MS));
        pb.set_message(message.into());
        Self { inner: pb }
    }

    /// Creates a hidden progress bar (for non-TTY environments or when progress is disabled).
    pub fn hidden() -> Self {
        let pb = IndicatifProgressBar::hidden();
        Self { inner: pb }
    }

    /// Updates the message displayed by the progress bar.
    pub fn set_message(&self, message: impl Into<String>) {
        self.inner.set_message(message.into());
    }

    /// Increments the progress by the given amount.
    pub fn inc(&self, delta: u64) {
        self.inner.inc(delta);
    }

    /// Sets the current position.
    pub fn set_position(&self, pos: u64) {
        self.inner.set_position(pos);
    }

    /// Sets the length of the progress bar.
    pub fn set_length(&self, len: u64) {
        self.inner.set_length(len);
    }

    /// Finishes the progress bar with a success message.
    pub fn finish_with_message(&self, message: impl Into<String>) {
        self.inner.set_style(
            ProgressStyle::default_spinner()
                .template(FINISHED_TEMPLATE)
                .expect("invalid template"),
        );
        self.inner.finish_with_message(message.into());
    }

    /// Finishes the progress bar and clears it.
    pub fn finish_and_clear(&self) {
        self.inner.finish_and_clear();
    }

    /// Finishes the progress bar successfully.
    pub fn finish(&self) {
        let message = self.inner.message();
        self.finish_with_message(message);
    }

    /// Marks the progress bar as failed with an error message.
    pub fn fail_with_message(&self, message: impl Into<String>) {
        self.inner.set_style(
            ProgressStyle::default_spinner()
                .template(FAILED_TEMPLATE)
                .expect("invalid template"),
        );
        self.inner.finish_with_message(message.into());
    }

    /// Returns a reference to the underlying indicatif progress bar.
    ///
    /// This allows access to advanced features not exposed by this wrapper.
    pub fn inner(&self) -> &IndicatifProgressBar {
        &self.inner
    }
}

/// A manager for multiple concurrent progress bars.
///
/// This allows displaying progress for multiple parallel operations.
pub struct MultiProgress {
    inner: IndicatifMultiProgress,
}

impl MultiProgress {
    /// Creates a new multi-progress manager.
    pub fn new() -> Self {
        Self {
            inner: IndicatifMultiProgress::new(),
        }
    }

    /// Adds a spinner task to the multi-progress display.
    ///
    /// # Arguments
    ///
    /// * `message` - The message to display for this task
    ///
    /// # Returns
    ///
    /// A `ProgressBar` that can be used to update this task's progress
    pub fn add_spinner(&self, message: impl Into<String>) -> ProgressBar {
        let pb = ProgressBar::new_spinner(message);
        let added = self.inner.add(pb.inner.clone());
        ProgressBar { inner: added }
    }

    /// Adds a progress bar task to the multi-progress display.
    ///
    /// # Arguments
    ///
    /// * `len` - The total number of items/steps
    /// * `message` - The message to display for this task
    pub fn add_bar(&self, len: u64, message: impl Into<String>) -> ProgressBar {
        let pb = ProgressBar::new(len, message);
        let added = self.inner.add(pb.inner.clone());
        ProgressBar { inner: added }
    }

    /// Adds a byte-based progress bar task.
    pub fn add_bytes(&self, total_bytes: u64, message: impl Into<String>) -> ProgressBar {
        let pb = ProgressBar::new_bytes(total_bytes, message);
        let added = self.inner.add(pb.inner.clone());
        ProgressBar { inner: added }
    }

    /// Adds an existing progress bar to the multi-progress display.
    pub fn add(&self, pb: ProgressBar) -> ProgressBar {
        let added = self.inner.add(pb.inner);
        ProgressBar { inner: added }
    }

    /// Removes a progress bar from the display.
    pub fn remove(&self, pb: &ProgressBar) {
        self.inner.remove(&pb.inner);
    }

    /// Clears all progress bars.
    pub fn clear(&self) -> std::io::Result<()> {
        self.inner.clear()
    }

    /// Prints a message above the progress bars.
    ///
    /// This ensures that log messages appear above the pinned progress bars
    /// and can scroll up normally. On non-terminal outputs, this falls back
    /// to regular println! to ensure logs are still visible.
    ///
    /// # Arguments
    ///
    /// * `msg` - The message to print
    pub fn println(&self, msg: impl AsRef<str>) -> std::io::Result<()> {
        if std::io::stderr().is_terminal() {
            self.inner.println(msg)
        } else {
            // Fall back to regular println for non-terminal outputs (e.g., logs)
            println!("{}", msg.as_ref());
            Ok(())
        }
    }
}

impl Default for MultiProgress {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_progress_bar_creation() {
        let pb = ProgressBar::new_spinner("Test");
        pb.finish();
    }

    #[test]
    fn test_progress_bar_with_length() {
        let pb = ProgressBar::new(100, "Test");
        pb.inc(50);
        pb.finish();
    }

    #[test]
    fn test_multi_progress() {
        let multi = MultiProgress::new();
        let pb1 = multi.add_spinner("Task 1");
        let pb2 = multi.add_spinner("Task 2");
        pb1.finish();
        pb2.finish();
    }
}
