use eyre::Result;
use opentelemetry::{InstrumentationScope, global};
use opentelemetry_sdk::metrics::{MeterProviderBuilder, PeriodicReader, SdkMeterProvider};
use opentelemetry_system_metrics::init_process_observer;

// Construct MeterProvider for MetricsLayer
pub fn init_meter_provider() -> SdkMeterProvider {
    let exporter = opentelemetry_otlp::MetricExporter::builder()
        .with_tonic()
        .with_temporality(opentelemetry_sdk::metrics::Temporality::default())
        .build()
        .unwrap();

    let reader = PeriodicReader::builder(exporter)
        .with_interval(std::time::Duration::from_secs(1))
        .build();

    let meter_provider = MeterProviderBuilder::default().with_reader(reader).build();

    meter_provider
}

pub async fn run_metrics_monitor(meter_id: String) -> Result<()> {
    let scope = InstrumentationScope::builder(meter_id).build();
    let meter = global::meter_with_scope(scope);
    println!("OK SO FAR");

    init_process_observer(meter).await
}
