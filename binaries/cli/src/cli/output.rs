use crate::cli::{OutputFormat, UiMode};
use eyre::Result;
use serde::Serialize;
use std::fmt::Display;

/// Output formatting utilities
pub struct OutputFormatter {
    format: OutputFormat,
    ui_mode: Option<UiMode>,
    no_hints: bool,
}

impl OutputFormatter {
    pub fn new(format: OutputFormat, ui_mode: Option<UiMode>, no_hints: bool) -> Self {
        Self {
            format,
            ui_mode,
            no_hints,
        }
    }

    pub fn format(&self) -> OutputFormat {
        self.format.clone()
    }

    pub fn ui_mode(&self) -> Option<UiMode> {
        self.ui_mode
    }

    /// Render data according to the specified format
    pub fn render<T: Serialize>(&self, data: &T) -> Result<String> {
        match self.format {
            OutputFormat::Json => self.render_json(data),
            OutputFormat::Yaml => self.render_yaml(data),
            OutputFormat::Table => self.render_table(data),
            OutputFormat::Minimal => self.render_minimal(data),
            OutputFormat::Auto => self.render_auto(data),
        }
    }

    /// Render as JSON
    fn render_json<T: Serialize>(&self, data: &T) -> Result<String> {
        Ok(serde_json::to_string_pretty(data)?)
    }

    /// Render as YAML
    fn render_yaml<T: Serialize>(&self, data: &T) -> Result<String> {
        Ok(serde_yaml::to_string(data)?)
    }

    /// Render as table (human-readable)
    fn render_table<T: Serialize>(&self, data: &T) -> Result<String> {
        // For now, fallback to JSON - will be enhanced in future issues
        self.render_json(data)
    }

    /// Render minimal output
    fn render_minimal<T: Serialize>(&self, data: &T) -> Result<String> {
        // Minimal output logic - will be enhanced
        Ok((serde_json::to_string(data)?).to_string())
    }

    /// Auto-select format based on context
    fn render_auto<T: Serialize>(&self, data: &T) -> Result<String> {
        match self.ui_mode {
            Some(UiMode::Minimal) | Some(UiMode::Cli) => self.render_table(data),
            Some(UiMode::Tui) => self.render_table(data), // TUI will handle its own rendering
            Some(UiMode::Auto) | None => {
                // Smart format selection based on context
                if self.is_ci_environment() {
                    self.render_minimal(data)
                } else {
                    self.render_table(data)
                }
            }
        }
    }

    /// Detect if running in CI environment
    fn is_ci_environment(&self) -> bool {
        std::env::var("CI").is_ok()
            || std::env::var("GITHUB_ACTIONS").is_ok()
            || std::env::var("GITLAB_CI").is_ok()
            || std::env::var("JENKINS_URL").is_ok()
    }

    /// Display hints if not suppressed
    pub fn display_hint(&self, hint: &str) {
        if !self.no_hints && !self.is_ci_environment() {
            eprintln!("💡 Hint: {hint}");
        }
    }

    /// Display TUI suggestion
    pub fn suggest_tui(&self, reason: &str) {
        if !self.no_hints && !self.is_ci_environment() {
            eprintln!("🚀 For a better experience with {reason}, try: dora --ui-mode tui");
        }
    }
}

/// Trait for objects that can be displayed with different formats
pub trait Displayable {
    fn display_with_format(&self, formatter: &OutputFormatter) -> Result<String>;
}

/// Helper for backward compatibility
impl Display for OutputFormat {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            OutputFormat::Auto => write!(f, "auto"),
            OutputFormat::Table => write!(f, "table"),
            OutputFormat::Json => write!(f, "json"),
            OutputFormat::Yaml => write!(f, "yaml"),
            OutputFormat::Minimal => write!(f, "minimal"),
        }
    }
}

impl Display for UiMode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            UiMode::Auto => write!(f, "auto"),
            UiMode::Cli => write!(f, "cli"),
            UiMode::Tui => write!(f, "tui"),
            UiMode::Minimal => write!(f, "minimal"),
        }
    }
}
