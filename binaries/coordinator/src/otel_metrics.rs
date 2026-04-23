//! Coordinator metrics via the standard dora-metrics OpenTelemetry pipeline.
//!
//! When the `metrics` feature is enabled, the coordinator records node and
//! dataflow gauges through the global OTel meter. These flow through the
//! same OTLP export pipeline used by the daemon and runtime, so a single
//! OTel Collector handles all dora telemetry.

use opentelemetry::metrics::Gauge;
use opentelemetry::{KeyValue, global};
use std::sync::Arc;

/// OTel gauge instruments for coordinator-level metrics.
pub(crate) struct Metrics {
    pub node_cpu: Gauge<f64>,
    pub node_memory: Gauge<i64>,
    pub node_pending: Gauge<i64>,
    pub node_restarts: Gauge<i64>,
    pub dataflow_nodes: Gauge<i64>,
}

impl Metrics {
    pub fn new() -> Self {
        let meter = global::meter("dora-coordinator");

        Self {
            node_cpu: meter
                .f64_gauge("dora_node_cpu_usage")
                .with_description("CPU usage percentage per node")
                .build(),
            node_memory: meter
                .i64_gauge("dora_node_memory_bytes")
                .with_description("Memory usage in bytes per node")
                .build(),
            node_pending: meter
                .i64_gauge("dora_node_pending_messages")
                .with_description("Pending messages in node queue")
                .build(),
            node_restarts: meter
                .i64_gauge("dora_node_restart_count")
                .with_description("Current restart count per node")
                .build(),
            dataflow_nodes: meter
                .i64_gauge("dora_dataflow_nodes")
                .with_description("Number of nodes in a dataflow")
                .build(),
        }
    }
}

pub(crate) type SharedMetrics = Arc<Metrics>;

pub(crate) fn new_shared() -> SharedMetrics {
    Arc::new(Metrics::new())
}

/// Build OTel `KeyValue` attributes for a node metric.
pub(crate) fn node_attrs(dataflow_id: String, node_id: String, daemon_id: String) -> [KeyValue; 3] {
    [
        KeyValue::new("dataflow", dataflow_id),
        KeyValue::new("node", node_id),
        KeyValue::new("daemon", daemon_id),
    ]
}
