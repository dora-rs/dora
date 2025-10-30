use crate::{DataflowSummary, SystemMetrics, UserPreferencesSnapshot};
use std::path::Path;

/// Abstraction over coordinator interactions.
pub trait CoordinatorClient: Send + Sync {
    /// Fetch the current list of dataflows.
    fn list_dataflows(&self) -> Result<Vec<DataflowSummary>, crate::InterfaceError>;
}

/// Bridge that lets the TUI trigger legacy CLI operations.
pub trait LegacyCliService: Send + Sync {
    /// Execute a CLI command expressed as argv tokens using the given working directory.
    fn execute(&self, argv: &[String], working_dir: &Path) -> Result<(), crate::InterfaceError>;
}

/// Provides system telemetry to the TUI.
pub trait TelemetryService: Send + Sync {
    fn latest_metrics(&self) -> Result<SystemMetrics, crate::InterfaceError>;
}

/// Persists and loads user preferences relevant to the TUI.
pub trait PreferencesStore: Send + Sync {
    fn load(&self) -> Result<UserPreferencesSnapshot, crate::InterfaceError>;
    fn save(&self, prefs: &UserPreferencesSnapshot) -> Result<(), crate::InterfaceError>;
}
