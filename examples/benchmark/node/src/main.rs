use dora_node_api::{self, dora_core::config::DataId, DoraNode, Event};
use eyre::Context;
use rand::Rng;
use std::time::{Duration, Instant};
use tracing_subscriber::Layer;

fn main() -> eyre::Result<()> {
    set_up_tracing().wrap_err("failed to set up tracing subscriber")?;

    let node_id = std::env::var("NODE_ID");
    if let Ok(node_id) = node_id {
        let node_id = node_id.parse::<u32>().context("failed to parse NODE_ID")?;
        if node_id == 0 {
            let latency = DataId::from("latency0".to_owned());
            let throughput = DataId::from("throughput0".to_owned());

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

            // test latency first
            for size in sizes {
                for _ in 0..100 {
                    let data: Vec<u8> = rand::thread_rng()
                        .sample_iter(rand::distributions::Standard)
                        .take(size)
                        .collect();
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
            for size in sizes {
                for _ in 0..100 {
                    let data: Vec<u8> = rand::thread_rng()
                        .sample_iter(rand::distributions::Standard)
                        .take(size)
                        .collect();
                    node.send_output_raw(
                        throughput.clone(),
                        Default::default(),
                        data.len(),
                        |out| {
                            out.copy_from_slice(&data);
                        },
                    )?;
                }
            }
        } else {
            let latency_id = DataId::from(format!("latency{}", node_id));
            let throughput = DataId::from(format!("throughput{}", node_id));
            let (mut node, mut events) = DoraNode::init_from_env()?;
            let mut latency = true;
            let mut current_size = 0;
            let mut n = 0;
            let mut start = Instant::now();
            let mut latencies = Vec::new();

            println!("Latency:");

            while let Some(event) = events.recv() {
                match event {
                    Event::Input { id, metadata, data } => {
                        let data_len = data.len();
                        if data_len != current_size {
                            if n > 0 {
                                record_results(start, current_size, n, latencies, latency)
                            }
                            current_size = data_len;
                            n = 0;
                            start = Instant::now();
                            latencies = Vec::new();
                        }

                        match id.as_str() {
                            "latency" if latency => {
                                node.send_output(
                                    latency_id.clone(),
                                    Default::default(),
                                    data.to_owned(),
                                )?;
                            }
                            "throughput" if latency => {
                                latency = false;
                                node.send_output(
                                    throughput.clone(),
                                    Default::default(),
                                    data.to_owned(),
                                )?;
                                println!("Throughput:");
                            }
                            "throughput" => {
                                node.send_output(
                                    throughput.clone(),
                                    Default::default(),
                                    data.to_owned(),
                                )?;
                            }
                            other => {
                                eprintln!("Ignoring unexpected input `{other}`");
                                continue;
                            }
                        }

                        n += 1;
                        latencies.push(
                            metadata
                                .timestamp()
                                .get_time()
                                .to_system_time()
                                .elapsed()
                                .unwrap_or_default(),
                        );
                    }
                    Event::InputClosed { id } => {
                        println!("Input `{id}` was closed");
                    }
                    other => eprintln!("Received unexpected input: {other:?}"),
                }
            }

            record_results(start, current_size, n, latencies, latency);
        }
    } else {
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

        // test latency first
        for size in sizes {
            for _ in 0..100 {
                let data: Vec<u8> = rand::thread_rng()
                    .sample_iter(rand::distributions::Standard)
                    .take(size)
                    .collect();
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
        for size in sizes {
            for _ in 0..100 {
                let data: Vec<u8> = rand::thread_rng()
                    .sample_iter(rand::distributions::Standard)
                    .take(size)
                    .collect();
                node.send_output_raw(throughput.clone(), Default::default(), data.len(), |out| {
                    out.copy_from_slice(&data);
                })?;
            }
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

fn record_results(
    start: Instant,
    current_size: usize,
    n: u32,
    latencies: Vec<Duration>,
    latency: bool,
) {
    let msg = if latency {
        let avg_latency = latencies.iter().sum::<Duration>() / n;
        format!("size {current_size:<#8x}: {avg_latency:?}")
    } else {
        let duration = start.elapsed();
        let msg_per_sec = n as f64 / duration.as_secs_f64();
        format!("size {current_size:<#8x}: {msg_per_sec:.0} messages per second")
    };
    println!("{msg}");
}
