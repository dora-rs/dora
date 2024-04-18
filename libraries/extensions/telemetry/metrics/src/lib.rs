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

use std::time::Duration;

use eyre::{Context, Result};
use opentelemetry::metrics::{self, MeterProvider as _};
use opentelemetry_otlp::{ExportConfig, WithExportConfig};
use opentelemetry_sdk::{metrics::SdkMeterProvider, runtime};
use opentelemetry_system_metrics::init_process_observer;
/// Init opentelemetry meter
///
/// Use the default Opentelemetry exporter with default config
/// TODO: Make Opentelemetry configurable
///
pub fn init_metrics() -> metrics::Result<SdkMeterProvider> {
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
        .with_period(Duration::from_secs(10))
        .build()
}

pub fn init_meter_provider(meter_id: String) -> Result<SdkMeterProvider> {
    let meter_provider = init_metrics().context("Could not create opentelemetry meter")?;
    let meter = meter_provider.meter(meter_id);
    init_process_observer(meter).context("could not initiale system metrics observer")?;
    Ok(meter_provider)
}
