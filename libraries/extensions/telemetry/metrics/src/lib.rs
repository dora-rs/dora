//! **Internal to dora — not a public API.**
//!
//! This crate is published to crates.io only because cargo requires every
//! dependency of a published crate to be published; `dora-node-api` and
//! `dora-cli` depend on it. It is not covered by dora's 1.0 stability
//! guarantee and may change in any release, including a patch.
//!
//! Depend on it directly at your own risk. See the "Stability scope at 1.0"
//! section of `docs/api-rust.md`.
//!
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

use eyre::{Result, WrapErr};
use opentelemetry::{InstrumentationScope, global};
use opentelemetry_otlp::{MetricExporter, WithExportConfig};
use opentelemetry_sdk::metrics::SdkMeterProvider;
use opentelemetry_system_metrics::init_process_observer;
/// Init opentelemetry meter, exporting via OTLP to `endpoint`.
///
/// `endpoint` should be the OTLP endpoint the caller derives from
/// `DORA_OTLP_ENDPOINT` (e.g. `"http://localhost:4317"`). Passing it
/// explicitly avoids the previous behaviour where the metric exporter ignored
/// `DORA_OTLP_ENDPOINT` and always fell back to the OTLP gRPC default
/// (`http://localhost:4317`), so system/process metrics silently diverged from
/// the trace exporter (which already threads the endpoint through) whenever a
/// remote collector was configured. Mirrors
/// `dora_tracing::telemetry::init_meter_provider`.
pub fn init_metrics(endpoint: &str) -> Result<SdkMeterProvider> {
    let exporter = MetricExporter::builder()
        .with_tonic()
        .with_endpoint(endpoint)
        .build()
        .wrap_err("failed to create metric exporter")?;

    Ok(SdkMeterProvider::builder()
        .with_periodic_exporter(exporter)
        .build())
}

pub async fn run_metrics_monitor(meter_id: String, endpoint: &str) -> Result<()> {
    let meter_provider = init_metrics(endpoint)?;
    global::set_meter_provider(meter_provider.clone());
    let scope = InstrumentationScope::builder(meter_id)
        .with_version("1.0")
        .build();
    let meter = global::meter_with_scope(scope);

    init_process_observer(meter).await
}
