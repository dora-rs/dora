use opentelemetry_otlp::{MetricExporter, WithExportConfig};
use opentelemetry_sdk::metrics::SdkMeterProvider;

/// Construct a [`SdkMeterProvider`] for the `MetricsLayer`, exporting via OTLP
/// to `endpoint`.
///
/// `endpoint` should be the same OTLP endpoint used for tracing (e.g.
/// `"http://localhost:4317"`) so that metrics and traces are sent to the same
/// collector. Passing the endpoint explicitly avoids the previous behaviour
/// where the metric exporter ignored `DORA_OTLP_ENDPOINT` and always fell back
/// to the OTLP gRPC default (`http://localhost:4317`), silently diverging from
/// the trace exporter when a remote collector was configured.
pub fn init_meter_provider(endpoint: &str) -> SdkMeterProvider {
    let exporter = MetricExporter::builder()
        .with_tonic()
        .with_endpoint(endpoint)
        .build()
        .expect("Failed to create metric exporter");

    SdkMeterProvider::builder()
        .with_periodic_exporter(exporter)
        .build()
}
