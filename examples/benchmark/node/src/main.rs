use dora_node_api::{self, DoraNode, dora_core::config::DataId};
use rand::RngCore;
use std::time::Duration;

const LATENCY_WARMUP: usize = 5;
const LATENCY_SAMPLES: usize = 50;
const THROUGHPUT_BASE_COUNT: usize = 1000;

fn main() -> eyre::Result<()> {
    let latency = DataId::from("latency".to_owned());
    let throughput = DataId::from("throughput".to_owned());

    let (mut node, _events) = DoraNode::init_from_env()?;
    let sizes: &[usize] = &[0, 8, 64, 512, 2048, 4096, 4 * 4096, 10 * 4096, 100 * 4096];

    let data: Vec<Vec<u8>> = sizes
        .iter()
        .map(|&size| {
            let mut data = vec![0u8; size];
            rand::thread_rng().fill_bytes(&mut data);
            data
        })
        .collect();

    // Warmup: send a few messages per size to prime SHM allocations and caches
    for data in &data {
        for _ in 0..LATENCY_WARMUP {
            node.send_output_raw(latency.clone(), Default::default(), data.len(), |out| {
                out.copy_from_slice(data);
            })?;
            std::thread::sleep(Duration::from_millis(10));
        }
    }

    // Signal end of warmup with a sync marker (size=1)
    node.send_output_raw(latency.clone(), Default::default(), 1, |out| {
        out.copy_from_slice(&[0]);
    })?;
    std::thread::sleep(Duration::from_millis(100));

    // Latency: send LATENCY_SAMPLES messages per size with spacing to avoid queue effects
    for data in &data {
        for _ in 0..LATENCY_SAMPLES {
            node.send_output_raw(latency.clone(), Default::default(), data.len(), |out| {
                out.copy_from_slice(data);
            })?;
            std::thread::sleep(Duration::from_millis(5));
        }
    }

    // Wait for latency messages to drain
    std::thread::sleep(Duration::from_secs(2));

    // Throughput: scale iteration count inversely with size to avoid exhausting /dev/shm
    for data in &data {
        let count = if data.len() <= 4096 {
            THROUGHPUT_BASE_COUNT
        } else {
            // Scale down for large messages: at least 50
            (THROUGHPUT_BASE_COUNT * 4096 / data.len()).max(50)
        };

        for _ in 0..count {
            node.send_output_raw(throughput.clone(), Default::default(), data.len(), |out| {
                out.copy_from_slice(data);
            })?;
        }
        // Sync marker to signal end of this size bracket
        node.send_output_raw(throughput.clone(), Default::default(), 1, |out| {
            out.copy_from_slice(&[1]);
        })?;

        std::thread::sleep(Duration::from_secs(2));
    }

    Ok(())
}
