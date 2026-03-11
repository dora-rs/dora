use adora_node_api::{self, AdoraNode, adora_core::config::DataId};
use rand::RngCore;
use std::time::Duration;

/// Number of latency samples per payload size.
const LATENCY_SAMPLES: u32 = 100;
/// Number of throughput messages per payload size.
const THROUGHPUT_MESSAGES: u32 = 100;
/// Sleep between latency samples to avoid queue buildup.
const LATENCY_SLEEP: Duration = Duration::from_millis(5);

fn main() -> eyre::Result<()> {
    let latency = DataId::from("latency".to_owned());
    let throughput = DataId::from("throughput".to_owned());

    let (mut node, _events) = AdoraNode::init_from_env()?;
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

    // test latency first: LATENCY_SAMPLES per size bracket
    for data in &data {
        for _ in 0..LATENCY_SAMPLES {
            node.send_output_raw(latency.clone(), Default::default(), data.len(), |out| {
                out.copy_from_slice(data);
            })?;
            std::thread::sleep(LATENCY_SLEEP);
        }
    }

    // wait for latency messages to drain before starting throughput
    std::thread::sleep(Duration::from_secs(2));

    // then throughput with full speed
    for data in &data {
        for _ in 0..THROUGHPUT_MESSAGES {
            node.send_output_raw(throughput.clone(), Default::default(), data.len(), |out| {
                out.copy_from_slice(data);
            })?;
        }
        // sentinel: single-byte marker signals end of this size bracket
        node.send_output_raw(throughput.clone(), Default::default(), 1, |out| {
            out.copy_from_slice(&[1]);
        })?;

        std::thread::sleep(Duration::from_secs(2));
    }

    Ok(())
}
