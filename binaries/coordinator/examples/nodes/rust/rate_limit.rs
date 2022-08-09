use clap::StructOpt;
use dora_node_api::{self, config::DataId, DoraNode};
use eyre::bail;
use futures::StreamExt;
use std::time::{Duration, Instant};

#[derive(Debug, Clone, clap::Parser)]
#[clap(about = "Limit the rate of incoming data")]
struct Args {
    /// Minimal interval between two subsequent.
    ///
    /// Intermediate messages are ignored.
    #[clap(long)]
    seconds: f32,
}

#[tokio::main]
async fn main() -> eyre::Result<()> {
    let args = Args::parse();
    let min_interval = Duration::from_secs_f32(args.seconds);
    let output = DataId::from("rate_limited".to_owned());

    let operator = DoraNode::init_from_env().await?;

    let mut inputs = operator.inputs().await?;

    let mut last_message = Instant::now();

    loop {
        let timeout = Duration::from_secs(3);
        let input = match tokio::time::timeout(timeout, inputs.next()).await {
            Ok(Some(input)) => input,
            Ok(None) => break,
            Err(_) => bail!("timeout while waiting for input"),
        };

        match input.id.as_str() {
            "data" => {
                let elapsed = last_message.elapsed();
                if elapsed > min_interval {
                    last_message += elapsed;
                    operator.send_output(&output, &input.data).await?;
                }
            }
            other => eprintln!("Ignoring unexpected input `{other}`"),
        }
    }

    println!("rate limit finished");

    Ok(())
}
