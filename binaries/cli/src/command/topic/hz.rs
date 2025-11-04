use crossterm::event::{Event, KeyCode, KeyModifiers};
use dora_core::topics::{open_zenoh_session, zenoh_output_publish_topic};
use dora_message::{common::Timestamped, daemon_to_daemon::InterDaemonEvent};
use eyre::{Context, eyre};
use itertools::Itertools;
use ratatui::{DefaultTerminal, prelude::*, widgets::*};
use std::{
    borrow::Cow,
    collections::VecDeque,
    iter,
    net::IpAddr,
    sync::{Arc, Mutex},
    time::{Duration, Instant},
};
use uuid::Uuid;

use crate::command::{
    Executable,
    topic::selector::{TopicIdentifier, TopicSelector},
};

/// Measure topic publish frequency (Hz).
///
/// Subscribe to one or more outputs and display per-topic statistics
/// (average, min, max, stddev) over a sliding window. The `--window` flag
/// controls the averaging window in seconds.
///
/// The `DATA` argument accepts `node_id/output_id` pairs. If no `DATA` is
/// provided, the command measures all outputs in the selected dataflow.
///
/// Examples:
/// - Measure a single topic: `dora topic hz -d my-dataflow robot1/pose`
/// - Measure multiple topics with a short window: `dora topic hz -d my-dataflow robot1/pose robot2/vel --window 5`
/// - Measure all topics: `dora topic hz -d my-dataflow --window 10`
///
/// Note: Requires `_unstable_debug.publish_all_messages_to_zenoh: true` in the
/// dataflow descriptor so runtime messages are available for inspection.
#[derive(Debug, clap::Args)]
pub struct Hz {
    #[clap(flatten)]
    selector: TopicSelector,

    /// Average window size in seconds
    #[clap(long, default_value_t = 10)]
    window: usize,
}

impl Executable for Hz {
    fn execute(self) -> eyre::Result<()> {
        let (_session, (dataflow_id, topics)) = self.selector.resolve()?;

        let rt = tokio::runtime::Builder::new_multi_thread()
            .enable_all()
            .build()
            .context("tokio runtime failed")?;
        let terminal = ratatui::init();
        rt.block_on(async move {
            run_hz(
                terminal,
                self.window,
                dataflow_id,
                topics,
                self.selector.dataflow.coordinator_addr,
            )
            .await
        })
        .inspect(|_| {
            ratatui::restore();
        })?;

        Ok(())
    }
}

#[derive(Debug)]
struct HzStats {
    timestamps: Mutex<VecDeque<Instant>>,
    window_duration: Duration,
}

impl HzStats {
    fn new(window_secs: usize) -> Self {
        Self {
            timestamps: Mutex::new(VecDeque::new()),
            window_duration: Duration::from_secs(window_secs as u64),
        }
    }

    fn record(&self, now: Instant) {
        let mut timestamps = self.timestamps.lock().unwrap();
        timestamps.push_back(now);
        let cutoff = now - self.window_duration;
        while let Some(&first) = timestamps.front() {
            if first < cutoff {
                timestamps.pop_front();
            } else {
                break;
            }
        }
    }

    fn calculate(&self) -> Option<Stats> {
        let intervals = self
            .timestamps
            .lock()
            .unwrap()
            .iter()
            .tuple_windows()
            .filter_map(|(a, b)| {
                let interval = b.duration_since(*a).as_secs_f64();
                if interval > 0.0 {
                    Some(1.0 / interval)
                } else {
                    None
                }
            })
            .collect::<Vec<_>>();
        if intervals.is_empty() {
            return None;
        }

        let sum: f64 = intervals.iter().sum();
        let avg = sum / intervals.len() as f64;

        let min = intervals.iter().cloned().fold(f64::INFINITY, f64::min);
        let max = intervals.iter().cloned().fold(f64::NEG_INFINITY, f64::max);

        let variance =
            intervals.iter().map(|hz| (hz - avg).powi(2)).sum::<f64>() / intervals.len() as f64;
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

async fn run_hz(
    mut terminal: DefaultTerminal,
    window: usize,
    dataflow_id: Uuid,
    outputs: Vec<TopicIdentifier>,
    coordinator_addr: IpAddr,
) -> eyre::Result<()> {
    let stats = outputs
        .iter()
        .map(|topic| (topic, Arc::new(HzStats::new(window))))
        .collect::<Vec<_>>();

    terminal.draw(|f| ui(f, &stats))?;

    let zenoh_session = open_zenoh_session(Some(coordinator_addr))
        .await
        .context("failed to open zenoh session")?;

    // Spawn subscribers for each output
    for (topic, hz_stats) in &stats {
        let zenoh_session = zenoh_session.clone();
        let topic = (*topic).clone();
        let hz_stats = hz_stats.clone();
        tokio::spawn(async move {
            if let Err(e) = subscribe_output(zenoh_session, dataflow_id, &topic, hz_stats).await {
                eprintln!("Error subscribing to {topic}: {e}");
            }
        });
    }

    loop {
        terminal.draw(|f| ui(f, &stats))?;

        if crossterm::event::poll(Duration::from_millis(50))? {
            if let Event::Key(key) = crossterm::event::read()? {
                if matches!(key.code, KeyCode::Char('q') | KeyCode::Esc)
                    || (key.modifiers.contains(KeyModifiers::CONTROL)
                        && key.code == KeyCode::Char('c'))
                {
                    break;
                }
            }
        }
    }

    Ok(())
}

async fn subscribe_output(
    zenoh_session: zenoh::Session,
    dataflow_id: Uuid,
    topic: &TopicIdentifier,
    hz_stats: Arc<HzStats>,
) -> eyre::Result<()> {
    let subscribe_topic = zenoh_output_publish_topic(dataflow_id, &topic.node_id, &topic.data_id);
    let subscriber = zenoh_session
        .declare_subscriber(subscribe_topic)
        .await
        .map_err(|e| eyre!(e))
        .wrap_err_with(|| format!("failed to subscribe to {topic}"))?;

    while let Ok(sample) = subscriber.recv_async().await {
        let event = match Timestamped::deserialize_inter_daemon_event(&sample.payload().to_bytes())
        {
            Ok(event) => event,
            Err(_) => continue,
        };

        match event.inner {
            InterDaemonEvent::Output { .. } => {
                hz_stats.record(Instant::now());
            }
            InterDaemonEvent::OutputClosed { .. } => {
                break;
            }
        }
    }

    Ok(())
}

fn ui(f: &mut Frame<'_>, stats: &[(&TopicIdentifier, Arc<HzStats>)]) {
    let header = Row::new(["Output", "Avg (Hz)", "Min (Hz)", "Max (Hz)", "Std (Hz)"])
        .style(Style::default().fg(Color::White).bg(Color::Blue).bold())
        .height(1);

    let rows = stats.iter().map(|(output_name, hz_stats)| {
        if let Some(stats) = hz_stats.calculate() {
            Row::new([
                output_name.to_string(),
                format!("{:.2}", stats.avg),
                format!("{:.2}", stats.min),
                format!("{:.2}", stats.max),
                format!("{:.2}", stats.std),
            ])
        } else {
            Row::new(
                iter::once(Cow::Owned(output_name.to_string()))
                    .chain(iter::repeat_n(Cow::Borrowed("-"), 4)),
            )
        }
        .height(1)
    });

    let table = Table::new(
        rows,
        [
            Constraint::Fill(1),
            Constraint::Length(12),
            Constraint::Length(12),
            Constraint::Length(12),
            Constraint::Length(12),
        ],
    )
    .header(header);

    f.render_widget(table, f.area());
}
