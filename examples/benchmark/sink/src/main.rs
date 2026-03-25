use dora_node_api::{self, DoraNode, Event};
use std::time::{Duration, Instant};

/// Phases of the benchmark, in order.
#[derive(PartialEq)]
enum Phase {
    Warmup,
    Latency,
    Throughput,
}

fn main() -> eyre::Result<()> {
    let (_node, mut events) = DoraNode::init_from_env()?;

    let mut phase = Phase::Warmup;
    let mut current_size = 0;
    let mut n: u32 = 0;
    let mut start = Instant::now();
    let mut latencies: Vec<Duration> = Vec::new();

    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, metadata, data } => {
                let data_len = data.len();

                // Sync marker (size=1) signals end of warmup
                if phase == Phase::Warmup && data_len == 1 {
                    phase = Phase::Latency;
                    current_size = 0;
                    n = 0;
                    latencies.clear();
                    println!("Latency (median / p99 / mean, 50 samples per size):");
                    continue;
                }

                // During warmup, just consume messages
                if phase == Phase::Warmup {
                    continue;
                }

                // Detect phase transition: latency -> throughput
                if phase == Phase::Latency && id.as_str() == "throughput" {
                    // Flush last latency bucket
                    if n > 0 && current_size != 1 {
                        print_latency(current_size, &mut latencies);
                    }
                    phase = Phase::Throughput;
                    current_size = 0;
                    n = 0;
                    start = Instant::now();
                    latencies.clear();
                    println!("\nThroughput (msgs/s, scaled iterations):");
                }

                // Handle size bracket transitions
                if data_len != current_size {
                    if n > 0 && current_size != 1 {
                        match phase {
                            Phase::Latency => print_latency(current_size, &mut latencies),
                            Phase::Throughput => print_throughput(current_size, n, start),
                            Phase::Warmup => {}
                        }
                    }
                    current_size = data_len;
                    n = 0;
                    start = Instant::now();
                    latencies.clear();
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

    Ok(())
}

fn print_latency(size: usize, latencies: &mut [Duration]) {
    if latencies.is_empty() {
        return;
    }
    latencies.sort();
    let n = latencies.len();
    let median = latencies[n / 2];
    let p99 = latencies[(n * 99 / 100).min(n - 1)];
    let mean = latencies.iter().sum::<Duration>() / n as u32;
    println!(
        "  size {size:<#10x} ({size_h:>6}):  median {median:>10.1?}  p99 {p99:>10.1?}  mean {mean:>10.1?}  (n={n})",
        size_h = human_size(size),
    );
}

fn print_throughput(size: usize, n: u32, start: Instant) {
    let duration = start.elapsed();
    let msg_per_sec = n as f64 / duration.as_secs_f64();
    let bytes_per_sec = msg_per_sec * size as f64;
    println!(
        "  size {size:<#10x} ({size_h:>6}):  {msg_per_sec:>10.0} msg/s  ({bw:>10})",
        size_h = human_size(size),
        bw = human_bandwidth(bytes_per_sec),
    );
}

fn human_size(bytes: usize) -> String {
    if bytes == 0 {
        "0 B".to_string()
    } else if bytes < 1024 {
        format!("{bytes} B")
    } else if bytes < 1024 * 1024 {
        format!("{} KiB", bytes / 1024)
    } else {
        format!("{} MiB", bytes / (1024 * 1024))
    }
}

fn human_bandwidth(bytes_per_sec: f64) -> String {
    if bytes_per_sec < 1024.0 {
        format!("{bytes_per_sec:.0} B/s")
    } else if bytes_per_sec < 1024.0 * 1024.0 {
        format!("{:.1} KiB/s", bytes_per_sec / 1024.0)
    } else if bytes_per_sec < 1024.0 * 1024.0 * 1024.0 {
        format!("{:.1} MiB/s", bytes_per_sec / (1024.0 * 1024.0))
    } else {
        format!("{:.2} GiB/s", bytes_per_sec / (1024.0 * 1024.0 * 1024.0))
    }
}
