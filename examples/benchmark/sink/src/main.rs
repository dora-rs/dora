use dora_node_api::{self, DoraNode, Event};
use std::time::{Duration, Instant};

fn main() -> eyre::Result<()> {
    let (_node, mut events) = DoraNode::init_from_env()?;

    // latency is tested first
    let mut latency = true;

    let mut current_size = 0;
    let mut n = 0;
    let mut start = Instant::now();
    let mut latencies = Vec::new();

    let csv = std::env::var("BENCH_CSV").ok();

    println!("Latency:");

    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, metadata, data } => {
                let data_len = data.len();
                // check if new size bracket
                if data_len != current_size {
                    // data of length 1 is used as sentinel (skip it)
                    if n > 0 && current_size != 1 {
                        record_results(
                            start,
                            current_size,
                            n,
                            &mut latencies,
                            latency,
                            csv.as_deref(),
                        );
                    }
                    current_size = data_len;
                    n = 0;
                    start = Instant::now();
                    latencies = Vec::new();
                }

                match id.as_str() {
                    "latency" if latency => {}
                    "throughput" if latency => {
                        latency = false;
                        println!("\nThroughput:");
                    }
                    "throughput" => {}
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

    // Flush final size bracket
    if n > 0 && current_size != 1 {
        record_results(
            start,
            current_size,
            n,
            &mut latencies,
            latency,
            csv.as_deref(),
        );
    }

    Ok(())
}

/// Compute the percentile value from a **sorted** slice.
fn percentile(sorted: &[Duration], pct: f64) -> Duration {
    if sorted.is_empty() {
        return Duration::ZERO;
    }
    let idx = ((pct / 100.0) * (sorted.len() - 1) as f64).round() as usize;
    sorted[idx.min(sorted.len() - 1)]
}

fn format_size(bytes: usize) -> String {
    if bytes == 0 {
        "0B".into()
    } else if bytes < 1024 {
        format!("{bytes}B")
    } else if bytes < 1024 * 1024 {
        format!("{}KB", bytes / 1024)
    } else {
        format!("{}MB", bytes / (1024 * 1024))
    }
}

fn record_results(
    start: Instant,
    current_size: usize,
    n: u32,
    latencies: &mut [Duration],
    latency: bool,
    csv_path: Option<&str>,
) {
    let size_label = format_size(current_size);

    if latency {
        latencies.sort();

        let avg = latencies.iter().sum::<Duration>() / n;
        let p50 = percentile(latencies, 50.0);
        let p95 = percentile(latencies, 95.0);
        let p99 = percentile(latencies, 99.0);
        let p999 = percentile(latencies, 99.9);
        let min = latencies.first().copied().unwrap_or_default();
        let max = latencies.last().copied().unwrap_or_default();

        println!(
            "  {size_label:>6}  avg={avg:>10.3?}  p50={p50:>10.3?}  p95={p95:>10.3?}  \
             p99={p99:>10.3?}  p99.9={p999:>10.3?}  min={min:>10.3?}  max={max:>10.3?}  \
             (n={n})"
        );

        if let Some(path) = csv_path {
            append_csv(
                path,
                &format!(
                    "latency,{current_size},{size_label},{n},{},{},{},{},{},{},{}",
                    avg.as_nanos(),
                    p50.as_nanos(),
                    p95.as_nanos(),
                    p99.as_nanos(),
                    p999.as_nanos(),
                    min.as_nanos(),
                    max.as_nanos(),
                ),
            );
        }
    } else {
        let duration = start.elapsed();
        let msg_per_sec = n as f64 / duration.as_secs_f64();
        println!("  {size_label:>6}  {msg_per_sec:.0} msg/s  (n={n}, {duration:.3?})");

        if let Some(path) = csv_path {
            append_csv(
                path,
                &format!(
                    "throughput,{current_size},{size_label},{n},{msg_per_sec:.0},{},0,0,0,0,0",
                    duration.as_nanos(),
                ),
            );
        }
    }
}

fn append_csv(path: &str, line: &str) {
    use std::io::Write;
    let Ok(mut file) = std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(path)
    else {
        eprintln!("Warning: failed to open CSV file: {path}");
        return;
    };
    if writeln!(file, "{line}").is_err() {
        eprintln!("Warning: failed to write to CSV file: {path}");
    }
}
