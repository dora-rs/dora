use adora_message::{common::Timestamped, daemon_to_daemon::InterDaemonEvent};
use crossterm::event::{Event, KeyCode, KeyModifiers};
use itertools::Itertools;
use ratatui::{DefaultTerminal, prelude::*, widgets::*};
use std::{
    borrow::Cow,
    collections::{BTreeMap, BTreeSet, VecDeque},
    fmt,
    io::{self, IsTerminal},
    iter,
    sync::{Arc, Mutex},
    time::{Duration, Instant},
};

use crate::{
    command::{
        Executable,
        topic::selector::{TopicIdentifier, TopicSelector},
    },
    common::CoordinatorOptions,
};

/// Measure topic publish intervals.
///
/// Subscribe to one or more outputs and display per-topic interval statistics
/// (average, min, max, stddev) over a sliding window. Average frequency (Hz)
/// is derived from the average interval.
///
/// If no `DATA` is provided, all outputs from the selected dataflow will be
/// echoed.
///
/// Examples:
///
/// Measure a single topic:
///   adora topic hz -d my-dataflow robot1/pose
///
/// Measure multiple topics with a short window:
///   adora topic hz -d my-dataflow robot1/pose robot2/vel --window 5
///
/// Measure all topics:
///   adora topic hz -d my-dataflow --window 10
///
/// Note: The dataflow descriptor must include the following snippet so that
/// runtime messages can be inspected:
///
/// ```yaml
/// _unstable_debug:
///   publish_all_messages_to_zenoh: true
/// ```
fn parse_window(s: &str) -> Result<usize, String> {
    let val: usize = s.parse().map_err(|e| format!("{e}"))?;
    if val == 0 {
        return Err("window must be at least 1".to_string());
    }
    Ok(val)
}

#[derive(Debug, clap::Args)]
#[clap(verbatim_doc_comment)]
pub struct Hz {
    #[clap(flatten)]
    selector: TopicSelector,

    /// Sliding window size in seconds
    #[clap(long, default_value_t = 10, value_parser = parse_window)]
    window: usize,

    #[clap(flatten)]
    coordinator: CoordinatorOptions,
}

impl Executable for Hz {
    fn execute(self) -> eyre::Result<()> {
        if !io::stdout().is_terminal() {
            eyre::bail!("`adora topic hz` requires an interactive terminal");
        }

        let session = self.coordinator.connect()?;
        let (dataflow_id, topics) = self.selector.resolve(&session)?;

        let ws_topics: Vec<_> = topics
            .iter()
            .map(|t| (t.node_id.clone(), t.data_id.clone()))
            .collect();

        let (_subscription_id, data_rx) = session.subscribe_topics(dataflow_id, ws_topics)?;

        let terminal = ratatui::init();
        let result = run_hz(terminal, self.window, topics, data_rx);
        ratatui::restore();
        result
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
        let mut timestamps = self.timestamps.lock().unwrap_or_else(|e| e.into_inner());
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

    fn intervals_ms(&self) -> Vec<f64> {
        self.timestamps
            .lock()
            .unwrap()
            .iter()
            .tuple_windows()
            .filter_map(|(a, b)| {
                let dt = b.duration_since(*a).as_secs_f64() * 1000.0;
                if dt > 0.0 { Some(dt) } else { None }
            })
            .collect()
    }

    fn calculate(&self) -> Option<Stats> {
        let intervals = self.intervals_ms();
        if intervals.is_empty() {
            return None;
        }

        let sum: f64 = intervals.iter().sum();
        let avg_ms = sum / intervals.len() as f64;

        let min_ms = intervals.iter().cloned().fold(f64::INFINITY, f64::min);
        let max_ms = intervals.iter().cloned().fold(f64::NEG_INFINITY, f64::max);

        let variance =
            intervals.iter().map(|x| (x - avg_ms).powi(2)).sum::<f64>() / intervals.len() as f64;
        let std_ms = variance.sqrt();

        let avg_hz = if avg_ms > 0.0 { 1000.0 / avg_ms } else { 0.0 };

        Some(Stats {
            avg_ms,
            avg_hz,
            min_ms,
            max_ms,
            std_ms,
        })
    }
}

#[derive(Debug)]
struct Stats {
    avg_ms: f64,
    avg_hz: f64,
    min_ms: f64,
    max_ms: f64,
    std_ms: f64,
}

/// Label for hz stats entries. Avoids creating a fake `TopicIdentifier` for the
/// aggregate row, which could collide with a real node name.
enum HzLabel<'a> {
    /// Aggregate of all topics.
    Aggregate,
    /// A specific topic.
    Topic(&'a TopicIdentifier),
}

impl fmt::Display for HzLabel<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            HzLabel::Aggregate => write!(f, "ALL"),
            HzLabel::Topic(t) => write!(f, "{t}"),
        }
    }
}

fn run_hz(
    mut terminal: DefaultTerminal,
    window: usize,
    outputs: BTreeSet<TopicIdentifier>,
    data_rx: std::sync::mpsc::Receiver<eyre::Result<Vec<u8>>>,
) -> eyre::Result<()> {
    // Build stats vec: index 0 is the aggregate, rest are per-topic
    let mut stats: Vec<(HzLabel<'_>, Arc<HzStats>)> = Vec::with_capacity(outputs.len() + 1);
    stats.push((HzLabel::Aggregate, Arc::new(HzStats::new(window))));
    for topic in &outputs {
        stats.push((HzLabel::Topic(topic), Arc::new(HzStats::new(window))));
    }

    // Build lookup map: (node_id, data_id) -> index in stats (skip index 0 = aggregate)
    let mut topic_index: BTreeMap<(String, String), usize> = BTreeMap::new();
    for (i, (label, _)) in stats.iter().enumerate().skip(1) {
        if let HzLabel::Topic(topic) = label {
            topic_index.insert((topic.node_id.to_string(), topic.data_id.to_string()), i);
        }
    }

    let mut selected: usize = 0;
    let sub_window = Duration::from_millis(1000);
    let mut rate_series: Vec<VecDeque<u64>> = vec![VecDeque::with_capacity(240); stats.len()];
    let start = Instant::now();

    terminal.draw(|f| {
        ui(
            f,
            &stats,
            selected,
            &rate_series,
            start,
            Duration::from_secs(window as u64),
        )
    })?;

    // Spawn receiver thread to feed stats from WS data
    let all_stats = stats[0].1.clone();
    let stats_clones: Vec<Arc<HzStats>> = stats.iter().map(|(_, s)| s.clone()).collect();
    let topic_index_clone = topic_index.clone();
    std::thread::spawn(move || {
        while let Ok(result) = data_rx.recv() {
            let payload = match result {
                Ok(p) => p,
                Err(_) => continue,
            };
            let event = match Timestamped::deserialize_inter_daemon_event(&payload) {
                Ok(e) => e,
                Err(_) => continue,
            };
            match event.inner {
                InterDaemonEvent::Output {
                    node_id, output_id, ..
                } => {
                    let now = Instant::now();
                    all_stats.record(now);
                    let key = (node_id.to_string(), output_id.to_string());
                    if let Some(&idx) = topic_index_clone.get(&key) {
                        stats_clones[idx].record(now);
                    }
                }
                InterDaemonEvent::OutputClosed { .. } => {}
            }
        }
    });

    loop {
        let now = Instant::now();
        for (i, (_topic, s)) in stats.iter().enumerate() {
            let cutoff = now - sub_window;
            let mut count = 0usize;
            let ts = s.timestamps.lock().unwrap_or_else(|e| e.into_inner());
            for &t in ts.iter().rev() {
                if t < cutoff {
                    break;
                }
                count += 1;
            }
            let hz = (count as f64) / sub_window.as_secs_f64();
            let v = hz.max(0.0).round() as u64;
            let buf = &mut rate_series[i];
            if buf.len() >= 240 {
                buf.pop_front();
            }
            buf.push_back(v);
        }

        terminal.draw(|f| {
            ui(
                f,
                &stats,
                selected,
                &rate_series,
                start,
                Duration::from_secs(window as u64),
            )
        })?;

        if crossterm::event::poll(Duration::from_millis(50))? {
            if let Event::Key(key) = crossterm::event::read()? {
                if matches!(key.code, KeyCode::Char('q') | KeyCode::Esc)
                    || (key.modifiers.contains(KeyModifiers::CONTROL)
                        && key.code == KeyCode::Char('c'))
                {
                    break;
                }

                match key.code {
                    KeyCode::Up => {
                        if selected == 0 {
                            selected = stats.len().saturating_sub(1);
                        } else {
                            selected -= 1;
                        }
                    }
                    KeyCode::Down => {
                        if stats.is_empty() {
                            selected = 0;
                        } else {
                            selected = (selected + 1) % stats.len();
                        }
                    }
                    _ => {}
                }
            }
        }
    }

    Ok(())
}

fn ui(
    f: &mut Frame<'_>,
    stats: &[(HzLabel<'_>, Arc<HzStats>)],
    selected: usize,
    rate_series: &[VecDeque<u64>],
    start: Instant,
    window_dur: Duration,
) {
    let chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([
            Constraint::Percentage(55),
            Constraint::Percentage(44),
            Constraint::Length(1),
        ])
        .split(f.area());

    let header = Row::new([
        "Output", "Avg (ms)", "Avg (Hz)", "Min (ms)", "Max (ms)", "Std (ms)",
    ])
    .style(Style::default().fg(Color::White).bg(Color::Blue).bold())
    .height(1);

    let rows = stats
        .iter()
        .enumerate()
        .map(|(i, (output_name, hz_stats))| {
            if let Some(s) = hz_stats.calculate() {
                Row::new([
                    output_name.to_string(),
                    format!("{:.2}", s.avg_ms),
                    format!("{:.2}", s.avg_hz),
                    format!("{:.2}", s.min_ms),
                    format!("{:.2}", s.max_ms),
                    format!("{:.2}", s.std_ms),
                ])
                .style(if i == selected {
                    Style::default().fg(Color::Yellow)
                } else {
                    Style::default()
                })
            } else {
                Row::new(
                    iter::once(Cow::Owned(output_name.to_string()))
                        .chain(iter::repeat_n(Cow::Borrowed("-"), 5)),
                )
                .style(if i == selected {
                    Style::default().fg(Color::Yellow)
                } else {
                    Style::default()
                })
            }
            .height(1)
        });

    let table = Table::new(
        rows,
        [
            Constraint::Fill(1),
            Constraint::Length(12),
            Constraint::Length(10),
            Constraint::Length(12),
            Constraint::Length(12),
            Constraint::Length(12),
        ],
    )
    .header(header);

    f.render_widget(table, chunks[0]);

    let chart_chunks = Layout::default()
        .direction(Direction::Horizontal)
        .constraints([Constraint::Percentage(50), Constraint::Percentage(50)])
        .split(chunks[1]);

    if let Some((name, selected_stats)) = stats.get(selected) {
        let intervals = selected_stats.intervals_ms();
        let now = Instant::now();

        let mut series: Vec<u64> = rate_series
            .get(selected)
            .map(|d| d.iter().copied().collect())
            .unwrap_or_default();
        if series.is_empty() {
            let info = Paragraph::new("Waiting for data...")
                .style(Style::default().fg(Color::Gray).italic())
                .block(
                    Block::default()
                        .title("Recent Rate (Hz)")
                        .borders(Borders::ALL),
                );
            f.render_widget(info, chart_chunks[0]);
        } else {
            let w = chart_chunks[0].width.saturating_sub(2) as usize;
            if series.len() > w {
                series = series[series.len() - w..].to_vec();
            }
            let spark = Sparkline::default()
                .data(&series)
                .style(Style::default().fg(Color::Cyan))
                .block(
                    Block::default()
                        .title(format!("Recent Rate (Hz) — {}", name))
                        .borders(Borders::ALL),
                );
            f.render_widget(spark, chart_chunks[0]);
        }

        if intervals.is_empty() {
            let info = Paragraph::new("No samples for histogram")
                .style(Style::default().fg(Color::Gray).italic())
                .block(
                    Block::default()
                        .title("Histogram (ms)")
                        .borders(Borders::ALL),
                );
            f.render_widget(info, chart_chunks[1]);
        } else {
            let min = intervals.iter().cloned().fold(f64::INFINITY, f64::min);
            let max = intervals.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
            let bins = 10usize
                .max((chart_chunks[1].width as usize).saturating_sub(8) / 4)
                .min(40);
            let span = (max - min).max(1e-9);
            let step = span / bins as f64;
            let mut counts = vec![0u64; bins];
            for &v in &intervals {
                let mut idx = ((v - min) / step).floor() as usize;
                if idx >= bins {
                    idx = bins - 1;
                }
                counts[idx] += 1;
            }

            let bars: Vec<Bar<'_>> = counts
                .iter()
                .enumerate()
                .map(|(i, &c)| {
                    let lo = min + i as f64 * step;
                    let hi = lo + step;
                    Bar::default()
                        .value(c)
                        .label(ratatui::text::Line::from(format!("{:.3}-{:.3}", lo, hi)))
                        .style(Style::default().fg(Color::Green))
                })
                .collect();

            let group = BarGroup::default().bars(&bars);
            let barchart = BarChart::default()
                .block(
                    Block::default()
                        .title(format!("Histogram (ms) — min={:.2}, max={:.2}", min, max))
                        .borders(Borders::ALL),
                )
                .data(group)
                .bar_width(3)
                .bar_gap(1);
            f.render_widget(barchart, chart_chunks[1]);
        }

        if now.duration_since(start) + Duration::from_millis(1) < window_dur {
            let warn = Paragraph::new(format!(
                "Filling window: {:.0}/{:.0} ms",
                now.duration_since(start).as_secs_f64() * 1000.0,
                window_dur.as_secs_f64() * 1000.0
            ))
            .style(Style::default().fg(Color::Yellow))
            .alignment(Alignment::Center);
            f.render_widget(warn, chunks[1]);
        }
    } else {
        let info = Paragraph::new("No topics selected")
            .style(Style::default().fg(Color::Gray))
            .alignment(Alignment::Center)
            .block(Block::default().borders(Borders::ALL));
        f.render_widget(info, chunks[1]);
    }

    let footer = Paragraph::new("Up/Down: Select  |  Exit: q / Ctrl-C / Esc")
        .style(Style::default().fg(Color::Yellow))
        .alignment(Alignment::Center);
    f.render_widget(footer, chunks[2]);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_window_valid() {
        assert_eq!(parse_window("10").unwrap(), 10);
    }

    #[test]
    fn parse_window_one() {
        assert_eq!(parse_window("1").unwrap(), 1);
    }

    #[test]
    fn parse_window_zero() {
        assert!(parse_window("0").is_err());
    }

    #[test]
    fn parse_window_non_numeric() {
        assert!(parse_window("abc").is_err());
    }
}
