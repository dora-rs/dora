use eyre::WrapErr;
use opentelemetry_otlp::MetricExporter;
use opentelemetry_sdk::metrics::SdkMeterProvider;

// Construct MeterProvider for MetricsLayer
pub fn init_meter_provider() -> eyre::Result<SdkMeterProvider> {
    let exporter = MetricExporter::builder()
        .with_tonic()
        .build()
        .wrap_err("failed to create metric exporter")?;

    Ok(SdkMeterProvider::builder()
        .with_periodic_exporter(exporter)
        .build())
}
