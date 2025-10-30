//! Data contracts consumed by the Dora TUI.
//!
//! These mirror the structures the UI needs today. The goal is to converge the
//! rest of the codebase on these shared definitions so the boundary becomes
//! the single source of truth.

use dora_message::descriptor::ResolvedNode;
use std::time::{Duration, Instant};

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

/// High-level system metrics snapshot used by the dashboard and monitor views.
#[derive(Debug, Clone)]
pub struct SystemMetrics {
    pub cpu_usage: f32,
    pub memory_usage: f32,
    pub network_io: (u64, u64),
    pub memory: MemoryMetrics,
    pub disk: DiskMetrics,
    pub network: NetworkMetrics,
    pub load_average: Option<LoadAverages>,
    pub uptime: Duration,
    pub process_count: usize,
    pub last_update: Option<Instant>,
}

impl Default for SystemMetrics {
    fn default() -> Self {
        Self {
            cpu_usage: 0.0,
            memory_usage: 0.0,
            network_io: (0, 0),
            memory: MemoryMetrics::default(),
            disk: DiskMetrics::default(),
            network: NetworkMetrics::default(),
            load_average: None,
            uptime: Duration::default(),
            process_count: 0,
            last_update: None,
        }
    }
}

/// Detailed memory usage information.
#[derive(Debug, Clone, Default)]
pub struct MemoryMetrics {
    pub total_bytes: u64,
    pub used_bytes: u64,
    pub free_bytes: u64,
    pub usage_percent: f32,
    pub swap_total_bytes: u64,
    pub swap_used_bytes: u64,
    pub swap_usage_percent: f32,
}

/// Disk utilisation snapshot.
#[derive(Debug, Clone, Default)]
pub struct DiskMetrics {
    pub total_bytes: u64,
    pub used_bytes: u64,
    pub usage_percent: f32,
}

/// Network throughput and totals.
#[derive(Debug, Clone, Default)]
pub struct NetworkMetrics {
    pub total_received: u64,
    pub total_transmitted: u64,
    pub received_per_second: f64,
    pub transmitted_per_second: f64,
}

/// Load averages over different windows.
#[derive(Debug, Clone, Default)]
pub struct LoadAverages {
    pub one: f64,
    pub five: f64,
    pub fifteen: f64,
}

/// Historical sample used for rendering trends.
#[derive(Debug, Clone)]
pub struct SystemMetricsSample {
    pub timestamp: Instant,
    pub cpu_usage: f32,
    pub memory_usage: f32,
    pub rx_rate: f64,
    pub tx_rate: f64,
}

impl Default for SystemMetricsSample {
    fn default() -> Self {
        Self {
            timestamp: Instant::now(),
            cpu_usage: 0.0,
            memory_usage: 0.0,
            rx_rate: 0.0,
            tx_rate: 0.0,
        }
    }
}

/// Minimal user preference snapshot shared between CLI and TUI.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct UserPreferencesSnapshot {
    pub theme: String,
    pub auto_refresh_interval_secs: u64,
    pub show_system_info: bool,
}
