use dora_node_api::{self, config::DataId, DoraNode};
use eyre::bail;
use futures::StreamExt;
use std::time::Duration;

#[tokio::main]
async fn main() -> eyre::Result<()> {
    let output = DataId::from("number".to_owned());

    let operator = DoraNode::init_from_env().await?;

    let mut inputs = operator.inputs().await?;

    loop {
        let timeout = Duration::from_secs(3);
        let input = match tokio::time::timeout(timeout, inputs.next()).await {
            Ok(Some(input)) => input,
            Ok(None) => break,
            Err(_) => bail!("timeout while waiting for input"),
        };

        match input.id.as_str() {
            "timestamp" => {
                let random: u64 = rand::random();
                operator.send_output(&output, &random.to_le_bytes()).await?;
            }
            other => eprintln!("Ignoring unexpected input `{other}`"),
        }
    }

    Ok(())
}
