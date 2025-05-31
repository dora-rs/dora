//! Enable tracing using Opentelemetry and Jaeger.
//!
//! This module init a tracing propagator for Rust code that requires tracing, and is
//! able to serialize and deserialize context that has been sent via the middleware.

use std::path::Path;

use eyre::Context as EyreContext;
use tracing::metadata::LevelFilter;
use tracing_subscriber::{
    filter::FilterExt, prelude::__tracing_subscriber_SubscriberExt, EnvFilter, Layer,
};

use tracing_subscriber::Registry;
pub mod telemetry;

/// Setup tracing with a default configuration.
///
/// This will set up a global subscriber that logs to stdout with a filter level of "warn".
///
/// Should **ONLY** be used in `DoraNode` implementations.
pub fn set_up_tracing(name: &str) -> eyre::Result<()> {
    TracingBuilder::new(name)
        .with_stdout("warn")
        .build()
        .wrap_err(format!(
            "failed to set tracing global subscriber for {name}"
        ))?;
    Ok(())
}

#[must_use = "call `build` to finalize the tracing setup"]
pub struct TracingBuilder {
    name: String,
    layers: Vec<Box<dyn Layer<Registry> + Send + Sync>>,
}

impl TracingBuilder {
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            layers: Vec::new(),
        }
    }

    /// Add a layer that write logs to the [std::io::stdout] with the given filter.
    ///
    /// **DO NOT** use this in `DoraNode` implementations,
    /// it uses [std::io::stdout] which is synchronous
    /// and might block the logging thread.
    pub fn with_stdout(mut self, filter: impl AsRef<str>) -> Self {
        let parsed = EnvFilter::builder().parse_lossy(filter);
        let env_filter = EnvFilter::from_default_env().or(parsed);
        let layer = tracing_subscriber::fmt::layer()
            .compact()
            .with_writer(std::io::stdout)
            .with_filter(env_filter);
        self.layers.push(layer.boxed());
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
            .with_writer(file)
            .with_filter(filter);
        self.layers.push(layer.boxed());
        Ok(self)
    }

    pub fn with_jaeger_tracing(mut self) -> eyre::Result<Self> {
        let endpoint = std::env::var("DORA_JAEGER_TRACING")
            .wrap_err("DORA_JAEGER_TRACING environment variable not set")?;
        let tracer = crate::telemetry::init_jaeger_tracing(&self.name, &endpoint)
            .wrap_err("Could not instantiate tracing")?;
        let telemetry = tracing_opentelemetry::layer().with_tracer(tracer);
        self.layers.push(telemetry.boxed());
        Ok(self)
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
        tracing::subscriber::set_global_default(registry).context(format!(
            "failed to set tracing global subscriber for {}",
            self.name
        ))
    }
}
