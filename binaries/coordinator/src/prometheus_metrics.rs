//! Feature-gated Prometheus metrics endpoint.
//!
//! Exposes `/metrics` in OpenMetrics text format when the `prometheus` feature is enabled.

use prometheus::{Encoder, GaugeVec, IntGaugeVec, Opts, Registry, TextEncoder};
use std::sync::Arc;
use tokio::sync::Mutex;

/// Holds all registered Prometheus metric families.
pub(crate) struct Metrics {
    pub registry: Registry,
    // Node-level gauges
    pub node_cpu: GaugeVec,
    pub node_memory: IntGaugeVec,
    pub node_pending: IntGaugeVec,
    pub node_restarts: IntGaugeVec,
    // Dataflow-level
    pub dataflow_nodes: IntGaugeVec,
}

impl Metrics {
    pub fn new() -> Self {
        let registry = Registry::new();

        let node_cpu = GaugeVec::new(
            Opts::new("adora_node_cpu_usage", "CPU usage percentage per node"),
            &["dataflow", "node", "daemon"],
        )
        .unwrap();
        let node_memory = IntGaugeVec::new(
            Opts::new("adora_node_memory_bytes", "Memory usage in bytes per node"),
            &["dataflow", "node", "daemon"],
        )
        .unwrap();
        let node_pending = IntGaugeVec::new(
            Opts::new(
                "adora_node_pending_messages",
                "Pending messages in node queue",
            ),
            &["dataflow", "node", "daemon"],
        )
        .unwrap();
        let node_restarts = IntGaugeVec::new(
            Opts::new("adora_node_restart_count", "Current restart count per node"),
            &["dataflow", "node", "daemon"],
        )
        .unwrap();
        let dataflow_nodes = IntGaugeVec::new(
            Opts::new("adora_dataflow_nodes", "Number of nodes in a dataflow"),
            &["dataflow", "name"],
        )
        .unwrap();

        registry.register(Box::new(node_cpu.clone())).unwrap();
        registry.register(Box::new(node_memory.clone())).unwrap();
        registry.register(Box::new(node_pending.clone())).unwrap();
        registry.register(Box::new(node_restarts.clone())).unwrap();
        registry.register(Box::new(dataflow_nodes.clone())).unwrap();

        Self {
            registry,
            node_cpu,
            node_memory,
            node_pending,
            node_restarts,
            dataflow_nodes,
        }
    }

    /// Serialize all metrics to Prometheus text format.
    pub fn gather(&self) -> String {
        let encoder = TextEncoder::new();
        let metric_families = self.registry.gather();
        let mut buffer = Vec::new();
        encoder.encode(&metric_families, &mut buffer).unwrap();
        String::from_utf8(buffer).unwrap()
    }
}

pub(crate) type SharedMetrics = Arc<Mutex<Metrics>>;

pub(crate) fn new_shared() -> SharedMetrics {
    Arc::new(Mutex::new(Metrics::new()))
}

/// Axum handler for GET /metrics.
pub(crate) async fn metrics_handler(
    axum::extract::State(metrics): axum::extract::State<SharedMetrics>,
) -> impl axum::response::IntoResponse {
    let m = metrics.lock().await;
    (
        [(
            axum::http::header::CONTENT_TYPE,
            "text/plain; version=0.0.4; charset=utf-8",
        )],
        m.gather(),
    )
}
