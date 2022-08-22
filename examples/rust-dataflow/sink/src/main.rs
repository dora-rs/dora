use dora_node_api::{self, DoraNode};
use eyre::{bail, Context};
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

        match input.metadata.id.as_str() {
            "message" => {
                let received_string = String::from_utf8(input.data)
                    .wrap_err("received message was not utf8-encoded")?;
                println!("received message: {}", received_string);
                if !received_string.starts_with("operator received random value ") {
                    bail!("unexpected message format (should start with 'operator received random value')")
                }
                if !received_string.ends_with(" ticks") {
                    bail!("unexpected message format (should end with 'ticks')")
                }
            }
            other => eprintln!("Ignoring unexpected input `{other}`"),
        }
    }

    Ok(())
}
