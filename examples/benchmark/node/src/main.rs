use dora_node_api::{self, dora_core::config::DataId, DoraNode};
use eyre::{Context, ContextCompat, Error};
use rand::Rng;
use std::collections::HashMap;
use std::time::Duration;
use tracing_subscriber::Layer;

fn main() -> eyre::Result<()> {
    set_up_tracing().wrap_err("failed to set up tracing subscriber")?;

    let latency = DataId::from("latency".to_owned());
    let throughput = DataId::from("throughput".to_owned());

    let (mut node, _events) = DoraNode::init_from_env()?;
    let sizes = [
        0,
        8,
        64,
        512,
        2048,
        4096,
        4 * 4096,
        10 * 4096,
        100 * 4096,
        1000 * 4096,
    ];

    let mut data = HashMap::new();
    for size in sizes {
        let vec: Vec<u8> = rand::thread_rng()
            .sample_iter(rand::distributions::Standard)
            .take(size)
            .collect();

        data.insert(size, vec);
    }

    // test latency first
    for size in sizes {
        for _ in 0..100 {
            let data = data.get(&size).wrap_err(eyre::Report::msg(format!(
                "data not found for size {}",
                size
            )))?;

            node.send_output_raw(latency.clone(), Default::default(), data.len(), |out| {
                out.copy_from_slice(data);
            })?;

            // sleep a bit to avoid queue buildup
            std::thread::sleep(Duration::from_millis(10));
        }
    }

    // wait a bit to ensure that all throughput messages reached their target
    std::thread::sleep(Duration::from_secs(2));

    // then throughput with full speed
    for size in sizes {
        for _ in 0..100 {
            let data = data.get(&size).wrap_err(eyre::Report::msg(format!(
                "data not found for size {}",
                size
            )))?;

            node.send_output_raw(throughput.clone(), Default::default(), data.len(), |out| {
                out.copy_from_slice(data);
            })?;
        }
    }

    Ok(())
}

fn set_up_tracing() -> eyre::Result<()> {
    use tracing_subscriber::prelude::__tracing_subscriber_SubscriberExt;

    let stdout_log = tracing_subscriber::fmt::layer()
        .pretty()
        .with_filter(tracing::metadata::LevelFilter::DEBUG);
    let subscriber = tracing_subscriber::Registry::default().with(stdout_log);
    tracing::subscriber::set_global_default(subscriber)
        .context("failed to set tracing global subscriber")
}
