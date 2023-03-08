#[cfg(feature = "tracing")]
use dora_tracing::set_up_tracing;
#[cfg(feature = "tracing")]
use eyre::Context;

#[tokio::main]
async fn main() -> eyre::Result<()> {
    #[cfg(feature = "tracing")]
    set_up_tracing().context("failed to set up tracing subscriber")?;

    let args = clap::Parser::parse();
    dora_coordinator::run(args).await
}
