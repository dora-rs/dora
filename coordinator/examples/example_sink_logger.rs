use dora_api::{self, DoraOperator};
use eyre::bail;
use futures::StreamExt;
use std::time::Duration;

#[tokio::main]
async fn main() -> eyre::Result<()> {
    let operator = DoraOperator::init_from_args().await?;

    let mut inputs = operator.inputs().await?;

    loop {
        let timeout = Duration::from_secs(2);
        let input = match tokio::time::timeout(timeout, inputs.next()).await {
            Ok(Some(input)) => input,
            Ok(None) => break,
            Err(_) => bail!("timeout while waiting for input"),
        };

        println!("{}: {}", input.id, String::from_utf8_lossy(&input.data))
    }

    Ok(())
}
