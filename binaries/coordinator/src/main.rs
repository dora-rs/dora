use eyre::Context;

#[tokio::main]
async fn main() -> eyre::Result<()> {
    set_up_tracing().context("failed to set up tracing subscriber")?;

    let command = clap::Parser::parse();
    dora_coordinator::run(command).await
}

fn set_up_tracing() -> eyre::Result<()> {
    use tracing_subscriber::prelude::__tracing_subscriber_SubscriberExt;

    let stdout_log = tracing_subscriber::fmt::layer().pretty();
    let subscriber = tracing_subscriber::Registry::default().with(stdout_log);
    tracing::subscriber::set_global_default(subscriber)
        .context("failed to set tracing global subscriber")
}
