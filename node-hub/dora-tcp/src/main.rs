#[tokio::main]

async fn main() -> Result<(), eyre::Error> {
    dora_tcp::lib_main().await
}
