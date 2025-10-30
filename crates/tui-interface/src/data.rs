//! Placeholder data contracts consumed by the TUI.
//! These will be filled as we migrate real DTOs out of the core crates.

/// Stub representation of a dataflow summary.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct DataflowSummary {
    /// Unique identifier for the dataflow (UUID or human-readable alias).
    pub id: String,
    /// Optional human-readable name.
    pub name: Option<String>,
}

/// Stub representation of a node summary inside a dataflow.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct NodeSummary {
    /// Node identifier.
    pub id: String,
    /// Optional display name.
    pub display_name: Option<String>,
}

/// Stub for system metrics snapshot consumed by the dashboard.
#[derive(Debug, Clone, PartialEq)]
pub struct SystemMetrics {
    /// CPU usage percentage.
    pub cpu_usage: f32,
    /// Memory usage percentage.
    pub memory_usage: f32,
}

impl Default for SystemMetrics {
    fn default() -> Self {
        Self {
            cpu_usage: 0.0,
            memory_usage: 0.0,
        }
    }
}

/// Minimal user preference snapshot placeholder.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct UserPreferencesSnapshot {
    /// Preferred theme name (e.g., "dark", "light").
    pub theme: String,
}
