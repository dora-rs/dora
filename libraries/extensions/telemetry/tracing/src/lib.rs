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
use tracing_subscriber::filter::Directive;
pub mod metrics;
pub mod span_store;
pub mod telemetry;

/// Parse a tracing directive known to be syntactically valid by
/// construction. Panics on failure — call sites must only pass string
/// literals. Replaces ~20 individual `.parse().unwrap()` sites with a
/// single documented helper, and keeps the panic path centralized.
fn directive(literal: &'static str) -> Directive {
    literal
        .parse()
        .unwrap_or_else(|e| panic!("invalid hardcoded tracing directive `{literal}`: {e}"))
}

/// Standard set of noisy-crate suppressions applied to every filter in
/// this module. Centralized so the list is the same everywhere.
const NOISY_CRATES_OFF: &[&str] = &[
    "hyper=off",
    "tonic=off",
    "tokio=off",
    "process_wrap=off",
    "h2=off",
    "reqwest=off",
];

/// Add the `NOISY_CRATES_OFF` directives to an `EnvFilter`.
fn with_noisy_crates_off(mut filter: EnvFilter) -> EnvFilter {
    for d in NOISY_CRATES_OFF {
        filter = filter.add_directive(directive(d));
    }
    filter
}

/// Setup tracing with a default configuration.
///
/// This will set up a global subscriber that logs to stdout with a filter level of "warn".
/// Outputs structured JSONL that the adora daemon parses as `LogMessage`,
/// so `tracing::info!()` calls in user nodes are automatically routed through
/// the adora log pipeline (file, coordinator, `adora/logs` subscribers).
///
/// Should **ONLY** be used in `AdoraNode` implementations.
pub fn set_up_tracing(name: &str) -> eyre::Result<()> {
    TracingBuilder::new(name)
        .with_node_stdout("warn")
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

    /// Add a layer that writes structured JSONL to stdout, compatible with the
    /// adora daemon's `LogMessageHelper` parser.
    ///
    /// This is the recommended layer for user nodes: `tracing::info!()` calls
    /// are automatically parsed by the daemon and routed through the log pipeline.
    pub fn with_node_stdout(mut self, filter: impl AsRef<str>) -> Self {
        let mut parsed = with_noisy_crates_off(EnvFilter::builder().parse_lossy(filter));
        let env_log = std::env::var("RUST_LOG").unwrap_or_default();
        if !env_log.contains("zenoh") {
            parsed = parsed.add_directive(directive("zenoh=warn"));
        }
        let env_filter = EnvFilter::from_default_env().or(parsed);
        let layer = tracing_subscriber::fmt::layer()
            .json()
            .with_writer(std::io::stdout)
            .with_filter(env_filter);
        self.layers.push(layer.boxed());
        self
    }

    /// Add a layer that write logs to the [std::io::stdout] with the given filter.
    ///
    /// **DO NOT** use this in `AdoraNode` implementations,
    /// it uses [std::io::stdout] which is synchronous
    /// and might block the logging thread.
    pub fn with_stdout(mut self, filter: impl AsRef<str>, json: bool) -> Self {
        let mut parsed = with_noisy_crates_off(EnvFilter::builder().parse_lossy(filter));
        let env_log = std::env::var("RUST_LOG").unwrap_or_default();
        if !env_log.contains("adora_daemon") {
            parsed = parsed.add_directive(directive("adora_daemon=info"));
        }
        if !env_log.contains("adora_core") {
            parsed = parsed.add_directive(directive("adora_core=warn"));
        }
        if !env_log.contains("zenoh") {
            parsed = parsed.add_directive(directive("zenoh=warn"));
        }
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
    /// Reads the OTLP endpoint from `ADORA_OTLP_ENDPOINT` environment variable.
    /// If not set, falls back to `ADORA_JAEGER_TRACING` for backward compatibility.
    ///
    /// The endpoint should be in the format: "http://localhost:4317"
    pub fn with_otlp_tracing(mut self) -> eyre::Result<Self> {
        let endpoint = std::env::var("ADORA_OTLP_ENDPOINT")
            .or_else(|_| std::env::var("ADORA_JAEGER_TRACING"))
            .wrap_err("ADORA_OTLP_ENDPOINT or ADORA_JAEGER_TRACING environment variable not set")?;

        // Initialize OTLP tracing - this returns a tracer and sets the global provider
        let sdk_tracer_provider = crate::telemetry::init_tracing(&self.name, &endpoint)
            .wrap_err("failed to initialize OTLP tracing exporter")?;
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
        let mut filter_otel = with_noisy_crates_off(EnvFilter::new("trace"));
        let env_log = std::env::var("RUST_LOG").unwrap_or_default();
        if !env_log.contains("adora_daemon") {
            filter_otel = filter_otel.add_directive(directive("adora_daemon=debug"));
        }
        self.layers.push(
            OpenTelemetryLayer::new(tracer)
                .with_filter(filter_otel)
                .boxed(),
        );
        Ok(self)
    }

    /// Legacy method name for backward compatibility.
    #[deprecated(since = "0.1.0", note = "Use `with_otlp_tracing` instead")]
    pub fn with_jaeger_tracing(self) -> eyre::Result<Self> {
        self.with_otlp_tracing()
    }

    /// Add a layer that captures completed spans into the given store.
    ///
    /// Only captures spans from `adora_*` crates at info level to avoid noise
    /// from third-party dependencies.
    pub fn with_span_capture(mut self, store: span_store::SharedSpanStore) -> Self {
        let filter = EnvFilter::new("off")
            .add_directive(directive("adora_coordinator=info"))
            .add_directive(directive("adora_core=info"));
        let layer = span_store::SpanCaptureLayer::new(store).with_filter(filter);
        self.layers.push(layer.boxed());
        self
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

/// Initialize tracing with OTLP (if configured) or stdout/file logging.
///
/// This function should be called after creating a tokio runtime and calling `runtime.enter()`.
///
/// # Parameters
/// - `name`: Service name for tracing
/// - `stdout_filter`: Optional RUST_LOG-style filter for stdout logging (e.g., "info", "debug")
/// - `file_name`: Optional filename for file logging (will be placed in `out/` directory)
/// - `file_filter`: Level filter for file logging (only used if `file_name` is Some)
///
/// # Returns
/// Returns `Option<OtelGuard>` which must be kept alive for the duration of the program.
/// When dropped, the guard will flush and shutdown telemetry providers.
///
/// # Example
/// ```no_run
/// use adora_tracing::init_tracing_subscriber;
/// use tracing::level_filters::LevelFilter;
///
/// // Note: This function requires a tokio runtime context to be active
/// // when using OTLP tracing. Use runtime.enter() before calling.
/// let _guard = init_tracing_subscriber(
///     "my-service",
///     Some("info"),
///     Some("my-service"),
///     LevelFilter::INFO,
/// ).unwrap();
/// ```
pub fn init_tracing_subscriber(
    name: &str,
    stdout_filter: Option<&str>,
    file_name: Option<&str>,
    file_filter: LevelFilter,
) -> eyre::Result<Option<OtelGuard>> {
    let mut builder = TracingBuilder::new(name);
    let guard: Option<OtelGuard>;

    if std::env::var("ADORA_OTLP_ENDPOINT").is_ok() || std::env::var("ADORA_JAEGER_TRACING").is_ok()
    {
        builder = builder
            .with_otlp_tracing()
            .wrap_err("failed to set up OTLP tracing")?;
        guard = builder.guard.take();
    } else {
        if let Some(filter) = stdout_filter {
            builder = builder.with_stdout(filter, false);
        }
        guard = None;
    }

    if let Some(filename) = file_name {
        builder = builder.with_file(filename, file_filter)?;
    }

    builder
        .build()
        .wrap_err("failed to set up tracing subscriber")?;

    Ok(guard)
}
