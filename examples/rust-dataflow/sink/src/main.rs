use dora_node_api::{self, DoraNode};
use eyre::bail;
use futures::StreamExt;
use std::time::Duration;

#[tokio::main]
async fn main() -> eyre::Result<()> {
    let operator = DoraNode::init_from_env().await?;

    let mut inputs = operator.inputs().await?;

    loop {
        let timeout = Duration::from_secs(5);
        let input = match tokio::time::timeout(timeout, inputs.next()).await {
            Ok(Some(input)) => input,
            Ok(None) => break,
            Err(_) => bail!("timeout while waiting for input"),
        };

        match input.id.as_str() {
            "message" => {
                println!("received message: {}", String::from_utf8_lossy(&input.data));
            }
            other => eprintln!("Ignoring unexpected input `{other}`"),
        }
    }

    Ok(())
}
