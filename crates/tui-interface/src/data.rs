//! Data contracts consumed by the Dora TUI.
//!
//! These mirror the structures the UI needs today. The goal is to converge the
//! rest of the codebase on these shared definitions so the boundary becomes
//! the single source of truth.

use dora_message::descriptor::ResolvedNode;

/// Summary information about a running or archived dataflow.
#[derive(Debug, Clone, Default)]
pub struct DataflowSummary {
    /// Unique identifier (UUID string) for the dataflow.
    pub id: String,
    /// Display name (falls back to `id` when unset).
    pub name: String,
    /// Human-readable status string (e.g., "running", "failed").
    pub status: String,
    /// Details about nodes inside the dataflow.
    pub nodes: Vec<NodeSummary>,
}

/// Details about a single node within a dataflow.
#[derive(Debug, Clone, Default)]
pub struct NodeSummary {
    pub id: String,
    pub name: String,
    pub status: String,
    pub kind: String,
    pub description: Option<String>,
    pub inputs: Vec<String>,
    pub outputs: Vec<String>,
    pub source: Option<String>,
    pub resolved: Option<ResolvedNode>,
}

/// Minimal system metrics snapshot placeholder.
#[derive(Debug, Clone, Default)]
pub struct SystemMetrics {
    pub cpu_usage: f32,
    pub memory_usage: f32,
}

/// Minimal user preference snapshot shared between CLI and TUI.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct UserPreferencesSnapshot {
    pub theme: String,
    pub auto_refresh_interval_secs: u64,
    pub show_system_info: bool,
}
