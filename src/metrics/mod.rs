use futures::stream::Stream;
use futures::StreamExt;
use opentelemetry::metrics;
use opentelemetry::sdk::metrics::selectors;
use opentelemetry::sdk::metrics::PushController;
use opentelemetry_otlp::{ExportConfig, WithExportConfig};
use std::time::Duration;

// Skip first immediate tick from tokio, not needed for async_std.
fn delayed_interval(duration: Duration) -> impl Stream<Item = tokio::time::Instant> {
    opentelemetry::sdk::util::tokio_interval_stream(duration).skip(1)
}

pub fn init_meter() -> metrics::Result<PushController> {
    let export_config = ExportConfig::default();
    opentelemetry_otlp::new_pipeline()
        .metrics(tokio::spawn, delayed_interval)
        .with_exporter(
            opentelemetry_otlp::new_exporter()
                .tonic()
                .with_export_config(export_config),
        )
        .with_aggregator_selector(selectors::simple::Selector::Exact)
        .build()
}
