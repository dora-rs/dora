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
/// Init opentelemetry meter.
///
/// When `endpoint` is `Some`, the OTLP exporter is pinned to it explicitly —
/// pass the `DORA_OTLP_ENDPOINT` value here (e.g. `"http://localhost:4317"`).
/// This avoids the previous behaviour where the metric exporter ignored
/// `DORA_OTLP_ENDPOINT` and always fell back to the OTLP gRPC default, so
/// system/process metrics silently diverged from the trace exporter (which
/// already threads the endpoint through) whenever a remote collector was
/// configured. Mirrors `dora_tracing::telemetry::init_meter_provider`.
///
/// When `endpoint` is `None`, no endpoint is set programmatically, so the
/// exporter resolves its target from the OTel-standard
/// `OTEL_EXPORTER_OTLP_ENDPOINT` / `OTEL_EXPORTER_OTLP_METRICS_ENDPOINT` env
/// vars (defaulting to `http://localhost:4317`). A programmatic
/// `.with_endpoint()` would take precedence over those vars, so it must be
/// skipped when `DORA_OTLP_ENDPOINT` is unset to keep them working.
pub fn init_metrics(endpoint: Option<&str>) -> Result<SdkMeterProvider> {
    let mut builder = MetricExporter::builder().with_tonic();
    if let Some(endpoint) = endpoint {
        builder = builder.with_endpoint(endpoint);
    }
    let exporter = builder
        .build()
        .wrap_err("failed to create metric exporter")?;

    Ok(SdkMeterProvider::builder()
        .with_periodic_exporter(exporter)
        .build())
}

pub async fn run_metrics_monitor(meter_id: String, endpoint: &str) -> Result<()> {
    let meter_provider = init_metrics(Some(endpoint))?;
    global::set_meter_provider(meter_provider.clone());
    let scope = InstrumentationScope::builder(meter_id)
        .with_version("1.0")
        .build();
    let meter = global::meter_with_scope(scope);

    init_process_observer(meter).await
}
