use dora_node_api::{self, DoraNode};
use std::{
    fmt::Write as _,
    time::{Duration, Instant},
};

fn main() -> eyre::Result<()> {
    let mut node = DoraNode::init_from_env()?;
    let inputs = node.inputs()?;

    let mut current_size = 0;
    let mut n = 0;
    let mut start = Instant::now();
    let mut latencies = Vec::new();

    let mut summary = String::new();

    while let Ok(input) = inputs.recv() {
        match input.id.as_str() {
            "message" => {
                let data = input.data();

                if data.len() != current_size {
                    if n > 0 {
                        record_results(start, current_size, n, latencies, &mut summary);
                    }
                    current_size = data.len();
                    n = 0;
                    start = Instant::now();
                    latencies = Vec::new();
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
            other => eprintln!("Ignoring unexpected input `{other}`"),
        }
    }

    record_results(start, current_size, n, latencies, &mut summary);

    println!("\nSummary:\n{summary}");

    Ok(())
}

fn record_results(
    start: Instant,
    current_size: usize,
    n: u32,
    latencies: Vec<Duration>,
    summary: &mut String,
) {
    let duration = start.elapsed();
    let per_message = duration / n;
    let avg_latency = latencies.iter().sum::<Duration>() / n;
    let msg =
        format!("size {current_size:<#8x}: {per_message:?} per message (latency: {avg_latency:?})");
    println!("{msg}");
    writeln!(summary, "{msg}").unwrap();
}
