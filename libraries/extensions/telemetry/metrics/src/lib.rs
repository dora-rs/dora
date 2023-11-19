//! Enable system metric through opentelemetry exporter.
//!
//! This module fetch system information using [`sysinfo`] and
//! export those metrics via an [`opentelemetry-rust`] exporter with default configuration.
//! Observed metrics are:
//! - CPU usage.
//! - Memory and Virtual memory usage.
//! - disk usage (read and write).
//!
//! [`sysinfo`]: https://github.com/GuillaumeGomez/sysinfo
//! [`opentelemetry-rust`]: https://github.com/open-telemetry/opentelemetry-rust

use opentelemetry::metrics::{self};
use opentelemetry_sdk::{metrics::MeterProvider, runtime};

use opentelemetry_otlp::{ExportConfig, WithExportConfig};
/// Init opentelemetry meter
///
/// Use the default Opentelemetry exporter with default config
/// TODO: Make Opentelemetry configurable
///
pub fn init_metrics() -> metrics::Result<MeterProvider> {
    let endpoint = std::env::var("OTEL_EXPORTER_OTLP_METRICS_ENDPOINT")
        .unwrap_or_else(|_| "http://localhost:4317".to_string());
    let export_config = ExportConfig {
        endpoint,
        ..ExportConfig::default()
    };

    opentelemetry_otlp::new_pipeline()
        .metrics(runtime::Tokio)
        .with_exporter(
            opentelemetry_otlp::new_exporter()
                .tonic()
                .with_export_config(export_config),
        )
        .build()
}
