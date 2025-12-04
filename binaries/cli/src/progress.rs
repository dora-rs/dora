//! Shared progress bar components for time-consuming CLI operations.
//!
//! This module provides a unified interface for displaying progress feedback
//! across all dora CLI commands using the `indicatif` crate.
//!
//! # Design Principles
//!
//! - **Unified Style**: All progress bars share consistent styling and behavior
//! - **Reusable Components**: Common patterns are encapsulated for easy reuse
//! - **Multi-task Support**: Support for parallel operations with multiple progress bars
//! - **Graceful Degradation**: Works in environments without TTY support
//!
//! # Usage Examples
//!
//! ## Simple Progress Bar
//!
//! ```rust,ignore
//! use crate::progress::ProgressBar;
//!
//! let pb = ProgressBar::new_spinner("Building nodes...");
//! // do work...
//! pb.finish_with_message("Build complete!");
//! ```
//!
//! ## Multi-task Progress
//!
//! ```rust,ignore
//! use crate::progress::MultiProgress;
//!
//! let multi = MultiProgress::new();
//! let pb1 = multi.add_task("Cloning repository...");
//! let pb2 = multi.add_task("Building node...");
//! // do work...
//! pb1.finish();
//! pb2.finish();
//! ```

use indicatif::{
    MultiProgress as IndicatifMultiProgress, ProgressBar as IndicatifProgressBar, ProgressStyle,
};
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
        let pb = IndicatifProgressBar::new_spinner();
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
        let pb = IndicatifProgressBar::new(len);
        pb.set_style(
            ProgressStyle::default_bar()
                .tick_chars(SPINNER_CHARS)
                .template(PROGRESS_TEMPLATE)
                .expect("invalid template")
                .progress_chars("=>-"),
        );
        pb.enable_steady_tick(Duration::from_millis(SPINNER_TICK_MS));
        pb.set_message(message.into());
        Self { inner: pb }
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
        let pb = IndicatifProgressBar::new(total_bytes);
        pb.set_style(
            ProgressStyle::default_bar()
                .tick_chars(SPINNER_CHARS)
                .template(BYTES_TEMPLATE)
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
}

impl Default for MultiProgress {
    fn default() -> Self {
        Self::new()
    }
}

/// Builder for creating progress bars with custom configuration.
pub struct ProgressBarBuilder {
    message: String,
    length: Option<u64>,
    style_type: ProgressStyleType,
    hidden: bool,
}

#[derive(Debug, Clone, Copy)]
enum ProgressStyleType {
    Spinner,
    Bar,
    Bytes,
}

impl ProgressBarBuilder {
    /// Creates a new progress bar builder.
    pub fn new(message: impl Into<String>) -> Self {
        Self {
            message: message.into(),
            length: None,
            style_type: ProgressStyleType::Spinner,
            hidden: false,
        }
    }

    /// Sets the progress bar to display as a bar (requires length).
    pub fn bar(mut self, length: u64) -> Self {
        self.length = Some(length);
        self.style_type = ProgressStyleType::Bar;
        self
    }

    /// Sets the progress bar to display byte-based progress (requires length).
    pub fn bytes(mut self, total_bytes: u64) -> Self {
        self.length = Some(total_bytes);
        self.style_type = ProgressStyleType::Bytes;
        self
    }

    /// Sets the progress bar to display as a spinner.
    pub fn spinner(mut self) -> Self {
        self.style_type = ProgressStyleType::Spinner;
        self
    }

    /// Sets whether the progress bar should be hidden.
    pub fn hidden(mut self, hidden: bool) -> Self {
        self.hidden = hidden;
        self
    }

    /// Builds the progress bar.
    pub fn build(self) -> ProgressBar {
        if self.hidden {
            return ProgressBar::hidden();
        }

        match self.style_type {
            ProgressStyleType::Spinner => ProgressBar::new_spinner(self.message),
            ProgressStyleType::Bar => ProgressBar::new(self.length.unwrap_or(100), self.message),
            ProgressStyleType::Bytes => {
                ProgressBar::new_bytes(self.length.unwrap_or(0), self.message)
            }
        }
    }
}

/// Utility function to check if progress bars should be displayed.
///
/// Progress bars are hidden in the following cases:
/// - Not running in a TTY
/// - CI environment detected
/// - User explicitly disabled progress (via environment variable)
pub fn should_show_progress() -> bool {
    // Check if we're in a TTY
    if !atty::is(atty::Stream::Stderr) {
        return false;
    }

    // Check for common CI environment variables
    if std::env::var("CI").is_ok()
        || std::env::var("CONTINUOUS_INTEGRATION").is_ok()
        || std::env::var("GITHUB_ACTIONS").is_ok()
    {
        return false;
    }

    // Check for explicit disable
    if let Ok(val) = std::env::var("DORA_NO_PROGRESS") {
        if val == "1" || val.eq_ignore_ascii_case("true") {
            return false;
        }
    }

    true
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

    #[test]
    fn test_builder() {
        let pb = ProgressBarBuilder::new("Test").spinner().build();
        pb.finish();

        let pb = ProgressBarBuilder::new("Test").bar(100).build();
        pb.finish();

        let pb = ProgressBarBuilder::new("Test").bytes(1024).build();
        pb.finish();
    }
}
