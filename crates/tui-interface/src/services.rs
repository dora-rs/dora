//! Service abstractions that the TUI depends on.
//! Concrete implementations will live inside the Dora framework.

use crate::{DataflowSummary, SystemMetrics, UserPreferencesSnapshot};

/// Abstraction over coordinator interactions.
#[allow(async_fn_in_trait)]
pub trait CoordinatorClient {
    /// Fetch the current list of dataflows.
    async fn list_dataflows(&self) -> Result<Vec<DataflowSummary>, crate::InterfaceError>;
}

/// Bridge that lets the TUI trigger legacy CLI operations.
#[allow(async_fn_in_trait)]
pub trait LegacyCliService {
    /// Execute a CLI command expressed as argv tokens.
    async fn execute(&self, argv: &[String]) -> Result<(), crate::InterfaceError>;
}

/// Provides system telemetry to the TUI.
#[allow(async_fn_in_trait)]
pub trait TelemetryService {
    async fn latest_metrics(&self) -> Result<SystemMetrics, crate::InterfaceError>;
}

/// Persists and loads user preferences relevant to the TUI.
pub trait PreferencesStore: Send + Sync {
    fn load(&self) -> Result<UserPreferencesSnapshot, crate::InterfaceError>;
    fn save(&self, prefs: &UserPreferencesSnapshot) -> Result<(), crate::InterfaceError>;
}
