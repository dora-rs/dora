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

use eyre::Result;
use opentelemetry::{InstrumentationScope, global};
use opentelemetry_otlp::MetricExporter;
use opentelemetry_sdk::metrics::SdkMeterProvider;
use opentelemetry_system_metrics::init_process_observer;
/// Init opentelemetry meter
///
/// Use the default Opentelemetry exporter with default config
/// TODO: Make Opentelemetry configurable
///
pub fn init_metrics() -> SdkMeterProvider {
    let exporter = MetricExporter::builder()
        .with_tonic()
        .build()
        .expect("Failed to create metric exporter");

    SdkMeterProvider::builder()
        .with_periodic_exporter(exporter)
        .build()
}

pub async fn run_metrics_monitor(meter_id: String) -> Result<()> {
    let meter_provider = init_metrics();
    global::set_meter_provider(meter_provider.clone());
    let scope = InstrumentationScope::builder(meter_id)
        .with_version("1.0")
        .build();
    let meter = global::meter_with_scope(scope);

    init_process_observer(meter).await.unwrap();
    Ok(())
}
