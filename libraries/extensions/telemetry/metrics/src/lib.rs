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
use opentelemetry::metrics::MeterProvider;
use opentelemetry_otlp::Protocol;
use opentelemetry_otlp::WithExportConfig;
use opentelemetry_sdk::metrics::{MetricResult, SdkMeterProvider};
use opentelemetry_system_metrics::init_process_observer;
/// Init opentelemetry meter
///
/// Use the default Opentelemetry exporter with default config
/// TODO: Make Opentelemetry configurable
///
pub fn init_metrics() -> MetricResult<SdkMeterProvider> {
    let endpoint = std::env::var("OTEL_EXPORTER_OTLP_METRICS_ENDPOINT")
        .unwrap_or_else(|_| "http://localhost:4318/v1/metrics".to_string());

    let exporter = opentelemetry_otlp::MetricExporter::builder()
        .with_tonic()
        .with_endpoint(endpoint)
        .with_protocol(Protocol::Grpc)
        .with_timeout(Duration::from_secs(10))
        .build()
        .unwrap();

    let reader = opentelemetry_sdk::metrics::PeriodicReader::builder(
        exporter,
        opentelemetry_sdk::runtime::Tokio,
    )
    .with_interval(std::time::Duration::from_secs(10))
    .with_timeout(Duration::from_secs(10))
    .build();

    Ok(opentelemetry_sdk::metrics::SdkMeterProvider::builder()
        .with_reader(reader)
        .build())
}

pub fn init_meter_provider(meter_id: String) -> Result<SdkMeterProvider> {
    let meter_id: &'static str = Box::leak(meter_id.into_boxed_str());
    let meter_provider = init_metrics().context("Could not create opentelemetry meter")?;
    let meter = meter_provider.meter(meter_id);
    init_process_observer(meter.clone()).context("could not initiate system metrics observer")?;
    Ok(meter_provider)
}
