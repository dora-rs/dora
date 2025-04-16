use dora_node_api::{self, dora_core::config::DataId, DoraNode};
use eyre::Context;
use rand::RngCore;
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

    let data = sizes.map(|size| {
        let mut data = vec![0u8; size];
        rand::thread_rng().fill_bytes(&mut data);
        data
    });

    // test latency first
    for data in &data {
        for _ in 0..1 {
            node.send_output_raw(latency.clone(), Default::default(), data.len(), |out| {
                out.copy_from_slice(&data);
            })?;

            // sleep a bit to avoid queue buildup
            std::thread::sleep(Duration::from_millis(10));
        }
    }

    // wait a bit to ensure that all throughput messages reached their target
    std::thread::sleep(Duration::from_secs(2));

    // then throughput with full speed
    for data in &data {
        for _ in 0..100 {
            node.send_output_raw(throughput.clone(), Default::default(), data.len(), |out| {
                out.copy_from_slice(&data);
            })?;
        }
        // notify sink that all messages have been sent
        node.send_output_raw(throughput.clone(), Default::default(), 1, |out| {
            out.copy_from_slice(&[1]);
        })?;

        std::thread::sleep(Duration::from_secs(2));
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
