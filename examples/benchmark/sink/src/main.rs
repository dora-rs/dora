use dora_node_api::{self, DoraNode};
use std::time::{Duration, Instant};

fn main() -> eyre::Result<()> {
    let mut node = DoraNode::init_from_env()?;
    let inputs = node.inputs()?;

    // latency is tested first
    let mut latency = true;

    let mut current_size = 0;
    let mut n = 0;
    let mut start = Instant::now();
    let mut latencies = Vec::new();

    println!("Latency:");

    while let Ok(input) = inputs.recv() {
        let data = input.data();

        // check if new size bracket
        if data.len() != current_size {
            if n > 0 {
                record_results(start, current_size, n, latencies, latency);
            }
            current_size = data.len();
            n = 0;
            start = Instant::now();
            latencies = Vec::new();
        }

        match input.id.as_str() {
            "latency" => {}
            "throughput" => {
                if latency {
                    latency = false;
                    println!("Throughput:");
                }
            }
            other => eprintln!("Ignoring unexpected input `{other}`"),
        }

        n += 1;
        latencies.push(
            input
                .metadata
                .timestamp()
                .get_time()
                .to_system_time()
                .elapsed()?,
        );
    }

    record_results(start, current_size, n, latencies, latency);

    Ok(())
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
