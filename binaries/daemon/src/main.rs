use dora_core::topics::DORA_COORDINATOR_PORT_DEFAULT;
use dora_daemon::Daemon;
use eyre::Context;
use std::net::Ipv4Addr;

#[tokio::main]
async fn main() -> eyre::Result<()> {
    // the tokio::main proc macro confuses some tools such as rust-analyzer, so
    // directly invoke a "normal" async function
    run().await
}

async fn run() -> eyre::Result<()> {
    set_up_tracing().wrap_err("failed to set up tracing subscriber")?;

    tracing::info!("Starting in local mode");
    let localhost = Ipv4Addr::new(127, 0, 0, 1);
    let coordinator_socket = (localhost, DORA_COORDINATOR_PORT_DEFAULT);

    let machine_id = String::new(); // TODO

    Daemon::run(coordinator_socket.into(), machine_id).await
}

fn set_up_tracing() -> eyre::Result<()> {
    use tracing_subscriber::prelude::__tracing_subscriber_SubscriberExt;

    let stdout_log = tracing_subscriber::fmt::layer().pretty();
    let subscriber = tracing_subscriber::Registry::default().with(stdout_log);
    tracing::subscriber::set_global_default(subscriber)
        .context("failed to set tracing global subscriber")
}
