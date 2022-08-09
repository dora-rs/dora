#[tokio::main]
async fn main() -> eyre::Result<()> {
    let command = clap::Parser::parse();
    dora_coordinator::run(command).await
}
