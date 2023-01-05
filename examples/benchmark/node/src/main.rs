use std::time::Duration;

use dora_node_api::{self, dora_core::config::DataId, DoraNode};
use rand::Rng;

fn main() -> eyre::Result<()> {
    let latency = DataId::from("latency".to_owned());
    let throughput = DataId::from("throughput".to_owned());

    let mut node = DoraNode::init_from_env()?;
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
        // 1000 * 4096,
        // 10000 * 4096,
    ];

    // test latency first
    for size in sizes {
        for _ in 0..100 {
            let data: Vec<u8> = rand::thread_rng()
                .sample_iter(rand::distributions::Standard)
                .take(size)
                .collect();
            node.send_output(&latency, Default::default(), data.len(), |out| {
                out.copy_from_slice(&data);
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
            let data: Vec<u8> = rand::thread_rng()
                .sample_iter(rand::distributions::Standard)
                .take(size)
                .collect();
            node.send_output(&throughput, Default::default(), data.len(), |out| {
                out.copy_from_slice(&data);
            })?;
        }
    }

    Ok(())
}
