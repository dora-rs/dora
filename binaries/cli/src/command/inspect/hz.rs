use std::collections::{HashMap, VecDeque};
use std::io::{self, Write};
use std::time::{Duration, Instant};

use colored::Colorize;
use dora_core::topics::{open_zenoh_session, zenoh_output_publish_topic};
use dora_message::{common::Timestamped, daemon_to_daemon::InterDaemonEvent, id::{DataId, NodeId}};
use eyre::{eyre, Context};
use tokio::sync::mpsc;
use uuid::Uuid;

use crate::command::{default_tracing, inspect::selector::InspectSelector, Executable};

#[derive(Debug, clap::Args)]
pub struct Hz {
    #[clap(flatten)]
    selector: InspectSelector,

    /// Average window size in seconds
    #[clap(long, default_value_t = 10)]
    window: usize,
}

impl Executable for Hz {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;

        inspect_hz(self.selector, self.window)
    }
}

#[derive(Debug)]
struct HzStats {
    timestamps: VecDeque<Instant>,
    window_duration: Duration,
}

impl HzStats {
    fn new(window_secs: usize) -> Self {
        Self {
            timestamps: VecDeque::new(),
            window_duration: Duration::from_secs(window_secs as u64),
        }
    }

    fn record(&mut self, now: Instant) {
        self.timestamps.push_back(now);
        let cutoff = now - self.window_duration;
        while let Some(&first) = self.timestamps.front() {
            if first < cutoff {
                self.timestamps.pop_front();
            } else {
                break;
            }
        }
    }

    fn calculate(&self) -> Option<Stats> {
        if self.timestamps.len() < 2 {
            return None;
        }

        let mut intervals = Vec::new();
        for i in 1..self.timestamps.len() {
            let interval = self.timestamps[i]
                .duration_since(self.timestamps[i - 1])
                .as_secs_f64();
            if interval > 0.0 {
                intervals.push(1.0 / interval);
            }
        }

        if intervals.is_empty() {
            return None;
        }

        let sum: f64 = intervals.iter().sum();
        let avg = sum / intervals.len() as f64;

        let min = intervals.iter().cloned().fold(f64::INFINITY, f64::min);
        let max = intervals.iter().cloned().fold(f64::NEG_INFINITY, f64::max);

        let variance = intervals
            .iter()
            .map(|hz| (hz - avg).powi(2))
            .sum::<f64>()
            / intervals.len() as f64;
        let std = variance.sqrt();

        Some(Stats { avg, min, max, std })
    }
}

#[derive(Debug)]
struct Stats {
    avg: f64,
    min: f64,
    max: f64,
    std: f64,
}

fn inspect_hz(selector: InspectSelector, window: usize) -> eyre::Result<()> {
    let coordinator_addr = selector.coordinator_addr;
    let outputs = selector.resolve()?;

    let rt = tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .build()
        .context("tokio runtime failed")?;
    rt.block_on(async move {
        let zenoh_session = open_zenoh_session(Some(coordinator_addr))
            .await
            .context("failed to open zenoh session")?;

        let (tx, mut rx) = mpsc::unbounded_channel::<(String, Instant)>();

        // Spawn subscribers for each output
        for (dataflow_id, node_id, output_id) in outputs {
            let zenoh_session = zenoh_session.clone();
            let tx = tx.clone();
            let output_name = format!("{node_id}/{output_id}");
            
            tokio::spawn(async move {
                if let Err(e) = subscribe_output(zenoh_session, dataflow_id, node_id, output_id, output_name.clone(), tx).await {
                    eprintln!("Error subscribing to {output_name}: {e}");
                }
            });
        }
        drop(tx);

        let mut stats_map: HashMap<String, HzStats> = HashMap::new();
        let mut last_update = Instant::now();
        let update_interval = Duration::from_millis(100);

        loop {
            tokio::select! {
                msg = rx.recv() => {
                    match msg {
                        Some((output_name, timestamp)) => {
                            stats_map
                                .entry(output_name)
                                .or_insert_with(|| HzStats::new(window))
                                .record(timestamp);
                        }
                        None => break,
                    }
                }
                _ = tokio::time::sleep(update_interval.saturating_sub(last_update.elapsed())) => {
                    display_stats(&stats_map);
                    last_update = Instant::now();
                }
            }
        }

        Result::<_, eyre::Error>::Ok(())
    })
}

async fn subscribe_output(
    zenoh_session: zenoh::Session,
    dataflow_id: Uuid,
    node_id: NodeId,
    output_id: DataId,
    output_name: String,
    tx: mpsc::UnboundedSender<(String, Instant)>,
) -> eyre::Result<()> {
    let subscribe_topic = zenoh_output_publish_topic(dataflow_id, &node_id, &output_id);
    let subscriber = zenoh_session
        .declare_subscriber(subscribe_topic)
        .await
        .map_err(|e| eyre!(e))
        .wrap_err_with(|| format!("failed to subscribe to {output_name}"))?;

    while let Ok(sample) = subscriber.recv_async().await {
        let event = match Timestamped::deserialize_inter_daemon_event(&sample.payload().to_bytes()) {
            Ok(event) => event,
            Err(_) => continue,
        };
        
        match event.inner {
            InterDaemonEvent::Output { .. } => {
                let _ = tx.send((output_name.clone(), Instant::now()));
            }
            InterDaemonEvent::OutputClosed { .. } => {
                break;
            }
        }
    }

    Ok(())
}

fn display_stats(stats_map: &HashMap<String, HzStats>) {
    // Clear screen and move cursor to top
    print!("\x1B[2J\x1B[1;1H");
    
    println!("{}", "Data Output Frequency Statistics".bold().cyan());
    println!("{}", "=".repeat(80));
    println!(
        "{:<40} {:>10} {:>10} {:>10} {:>10}",
        "Output".bold(),
        "Avg (Hz)".bold(),
        "Min (Hz)".bold(),
        "Max (Hz)".bold(),
        "Std (Hz)".bold()
    );
    println!("{}", "-".repeat(80));

    let mut sorted_outputs: Vec<_> = stats_map.iter().collect();
    sorted_outputs.sort_by_key(|(name, _)| *name);

    for (output_name, hz_stats) in sorted_outputs {
        if let Some(stats) = hz_stats.calculate() {
            println!(
                "{:<40} {:>10.2} {:>10.2} {:>10.2} {:>10.2}",
                output_name.green(),
                stats.avg,
                stats.min,
                stats.max,
                stats.std
            );
        } else {
            println!(
                "{:<40} {}",
                output_name.green(),
                "waiting for data...".dimmed()
            );
        }
    }

    io::stdout().flush().ok();
}
