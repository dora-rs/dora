use dora_node_api::{self, config::DataId, DoraNode, Metadata};
use eyre::bail;
use futures::StreamExt;
use std::time::Duration;

#[tokio::main]
async fn main() -> eyre::Result<()> {
    let output = DataId::from("random".to_owned());

    let operator = DoraNode::init_from_env().await?;

    let mut inputs = operator.inputs().await?;

    for _ in 0..20 {
        let timeout = Duration::from_secs(3);
        let input = match tokio::time::timeout(timeout, inputs.next()).await {
            Ok(Some(input)) => input,
            Ok(None) => break,
            Err(_) => bail!("timeout while waiting for input"),
        };

        match input.metadata.id.as_str() {
            "tick" => {
                let random: u64 = rand::random();
                operator
                    .send_output(
                        &Metadata {
                            id: output.clone(),
                            otel_context: input.metadata.otel_context.clone(),
                        },
                        &random.to_le_bytes(),
                    )
                    .await?;
                dbg!(input.metadata.otel_context);
            }
            other => eprintln!("Ignoring unexpected input `{other}`"),
        }
    }

    Ok(())
}
