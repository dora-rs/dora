use dora_node_api::{self, DoraNode};
use eyre::bail;
use futures::StreamExt;
use std::time::Duration;

#[tokio::main]
async fn main() -> eyre::Result<()> {
    let operator = DoraNode::init_from_env().await?;

    let mut inputs = operator.inputs().await?;

    let mut last_timestamp = None;

    loop {
        let timeout = Duration::from_secs(5);
        let input = match tokio::time::timeout(timeout, inputs.next()).await {
            Ok(Some(input)) => input,
            Ok(None) => break,
            Err(_) => bail!("timeout while waiting for input"),
        };

        match input.id.as_str() {
            "time" => {
                // only record it, but don't print anything
                last_timestamp = Some(String::from_utf8_lossy(&input.data).into_owned());
            }
            "random" => {
                let number = match input.data.try_into() {
                    Ok(bytes) => u64::from_le_bytes(bytes),
                    Err(_) => {
                        eprintln!("Malformed `random` message");
                        continue;
                    }
                };
                if let Some(timestamp) = &last_timestamp {
                    println!("random at {}: {}", timestamp, number);
                }
            }
            "timestamped-random" => {
                let data = String::from_utf8(input.data)?;
                println!("received timestamped random value: {data}");
            }
            "c-counter" => {
                println!("received C counter value: {:?}", input.data);
            }
            "python-counter" => {
                println!("received PYTHON counter value: {:?}", input.data);
            }

            other => eprintln!("Ignoring unexpected input `{other}`"),
        }
    }

    Ok(())
}
