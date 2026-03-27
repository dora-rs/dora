use clap::Parser;
use dora_cli::Args;

#[tokio::main]
async fn main() {
    let args = Args::parse();
    dora_cli::lib_main(args).await;
}
