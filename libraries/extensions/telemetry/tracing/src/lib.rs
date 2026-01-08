//! Enable tracing using OpenTelemetry with OTLP.
//!
//! This module initializes a tracing propagator for Rust code that requires tracing, and is
//! able to serialize and deserialize context that has been sent via the middleware.
//! Supports any OTLP-compatible backend (Jaeger, Zipkin, Tempo, etc.).

use std::path::Path;

use eyre::Context as EyreContext;
use opentelemetry::trace::TracerProvider;
use opentelemetry_sdk::metrics::SdkMeterProvider;
use opentelemetry_sdk::trace::SdkTracerProvider;
use tracing::metadata::LevelFilter;

use tracing_opentelemetry::{MetricsLayer, OpenTelemetryLayer};
use tracing_subscriber::{
    EnvFilter, Layer, filter::FilterExt, prelude::__tracing_subscriber_SubscriberExt,
};

use tracing_subscriber::Registry;
pub mod metrics;
pub mod telemetry;

/// Setup tracing with a default configuration.
///
/// This will set up a global subscriber that logs to stdout with a filter level of "warn".
///
/// Should **ONLY** be used in `DoraNode` implementations.
pub fn set_up_tracing(name: &str) -> eyre::Result<()> {
    TracingBuilder::new(name)
        .with_stdout("warn", false)
        .build()
        .wrap_err(format!(
            "failed to set tracing global subscriber for {name}"
        ))?;
    Ok(())
}

pub struct OtelGuard {
    tracer_provider: SdkTracerProvider,
    meter_provider: SdkMeterProvider,
}

#[must_use = "call `build` to finalize the tracing setup"]
pub struct TracingBuilder {
    name: String,
    layers: Vec<Box<dyn Layer<Registry> + Send + Sync>>,
    pub guard: Option<OtelGuard>,
}

impl TracingBuilder {
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            layers: Vec::new(),
            guard: None,
        }
    }

    /// Add a layer that write logs to the [std::io::stdout] with the given filter.
    ///
    /// **DO NOT** use this in `DoraNode` implementations,
    /// it uses [std::io::stdout] which is synchronous
    /// and might block the logging thread.
    pub fn with_stdout(mut self, filter: impl AsRef<str>, json: bool) -> Self {
        let parsed = EnvFilter::builder()
            .parse_lossy(filter)
            .add_directive("hyper=off".parse().unwrap())
            .add_directive("tonic=off".parse().unwrap())
            .add_directive("h2=off".parse().unwrap())
            .add_directive("reqwest=off".parse().unwrap());
        let env_filter = EnvFilter::from_default_env().or(parsed);
        let layer = tracing_subscriber::fmt::layer()
            .compact()
            .with_writer(std::io::stdout);

        if json {
            let layer = layer.json().with_filter(env_filter);
            self.layers.push(layer.boxed());
        } else {
            let layer = layer.with_filter(env_filter);
            self.layers.push(layer.boxed());
        };
        self
    }

    /// Add a layer that write logs to a file with the given name and filter.
    pub fn with_file(
        mut self,
        file_name: impl Into<String>,
        filter: LevelFilter,
    ) -> eyre::Result<Self> {
        let file_name = file_name.into();
        let out_dir = Path::new("out");
        std::fs::create_dir_all(out_dir).context("failed to create `out` directory")?;
        let path = out_dir.join(file_name).with_extension("txt");
        let file = std::fs::OpenOptions::new()
            .create(true)
            .append(true)
            .open(path)
            .context("failed to create log file")?;
        let layer = tracing_subscriber::fmt::layer()
            .with_ansi(false)
            .json()
            .with_writer(file)
            .with_filter(filter);
        self.layers.push(layer.boxed());
        Ok(self)
    }

    /// Add OpenTelemetry tracing layer with OTLP exporter.
    ///
    /// Reads the OTLP endpoint from `DORA_OTLP_ENDPOINT` environment variable.
    /// If not set, falls back to `DORA_JAEGER_TRACING` for backward compatibility.
    ///
    /// The endpoint should be in the format: "http://localhost:4317"
    pub fn with_otlp_tracing(mut self) -> eyre::Result<Self> {
        let endpoint = std::env::var("DORA_OTLP_ENDPOINT")
            .or_else(|_| std::env::var("DORA_JAEGER_TRACING"))
            .wrap_err("DORA_OTLP_ENDPOINT or DORA_JAEGER_TRACING environment variable not set")?;

        // Initialize OTLP tracing - this returns a tracer and sets the global provider
        let sdk_tracer_provider = crate::telemetry::init_tracing(&self.name, &endpoint);
        let meter_provider = metrics::init_meter_provider();

        // TODO: Maybe this needs to be removed in favor of application level global.
        // global::set_meter_provider(meter_provider.clone());
        // Use the specific tracer instance returned from init_tracing
        let tracer = sdk_tracer_provider.tracer("tracing-otel-subscriber");

        let guard = OtelGuard {
            tracer_provider: sdk_tracer_provider,
            meter_provider: meter_provider.clone(),
        };

        self.guard = Some(guard);
        self.layers.push(MetricsLayer::new(meter_provider).boxed());
        let filter_otel = EnvFilter::new("trace")
            .add_directive("hyper=off".parse().unwrap())
            .add_directive("tonic=off".parse().unwrap())
            .add_directive("h2=off".parse().unwrap())
            .add_directive("reqwest=off".parse().unwrap());
        self.layers.push(
            OpenTelemetryLayer::new(tracer)
                .with_filter(filter_otel)
                .boxed(),
        );
        Ok(self)
    }

    /// Legacy method name for backward compatibility.
    #[deprecated(since = "0.4.0", note = "Use `with_otlp_tracing` instead")]
    pub fn with_jaeger_tracing(self) -> eyre::Result<Self> {
        self.with_otlp_tracing()
    }

    pub fn add_layer<L>(mut self, layer: L) -> Self
    where
        L: Layer<Registry> + Send + Sync + 'static,
    {
        self.layers.push(layer.boxed());
        self
    }

    pub fn with_layers<I, L>(mut self, layers: I) -> Self
    where
        I: IntoIterator<Item = L>,
        L: Layer<Registry> + Send + Sync + 'static,
    {
        for layer in layers {
            self.layers.push(layer.boxed());
        }
        self
    }

    pub fn build(self) -> eyre::Result<()> {
        let registry = Registry::default().with(self.layers);

        // TODO: Maybe this needs to be removed in favor of application level global.
        tracing::subscriber::set_global_default(registry).context(format!(
            "failed to set tracing global subscriber for {}",
            self.name
        ))
    }
}

impl Drop for OtelGuard {
    fn drop(&mut self) {
        self.meter_provider.force_flush().ok();
        self.meter_provider.shutdown().ok();
        self.tracer_provider.force_flush().ok();
        self.tracer_provider.shutdown().ok();
    }
}
