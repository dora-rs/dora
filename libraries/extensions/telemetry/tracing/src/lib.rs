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

use eyre::ContextCompat;
use tracing_subscriber::Registry;
pub mod telemetry;

pub fn set_up_tracing(name: &str) -> eyre::Result<()> {
    set_up_tracing_opts(name, true, None)
}

pub fn set_up_tracing_opts(name: &str, stdout: bool, filename: Option<&str>) -> eyre::Result<()> {
    let mut layers = Vec::new();

    if stdout {
        // Filter log using `RUST_LOG`. More useful for CLI.
        let env_filter = EnvFilter::from_default_env().or(LevelFilter::WARN);
        let layer = tracing_subscriber::fmt::layer()
            .compact()
            .with_filter(env_filter);
        layers.push(layer.boxed());
    }

    if let Some(filename) = filename {
        let out_dir = Path::new("out");
        std::fs::create_dir_all(out_dir).context("failed to create `out` directory")?;
        let path = out_dir.join(filename).with_extension("txt");
        let file = std::fs::OpenOptions::new()
            .create(true)
            .append(true)
            .open(path)
            .context("failed to create log file")?;
        // Filter log using `RUST_LOG`. More useful for CLI.
        let layer = tracing_subscriber::fmt::layer()
            .with_ansi(false)
            .with_writer(file)
            .with_filter(LevelFilter::INFO);
        layers.push(layer.boxed());
    }

    if let Some(endpoint) = std::env::var_os("DORA_JAEGER_TRACING") {
        let endpoint = endpoint
            .to_str()
            .wrap_err("Could not parse env variable: DORA_JAEGER_TRACING")?;
        let tracer = crate::telemetry::init_jaeger_tracing(name, endpoint)
            .wrap_err("Could not instantiate tracing")?;
        let telemetry = tracing_opentelemetry::layer().with_tracer(tracer);
        layers.push(telemetry.boxed());
    }

    let registry = Registry::default().with(layers);
    tracing::subscriber::set_global_default(registry).context(format!(
        "failed to set tracing global subscriber for {name}"
    ))
}
