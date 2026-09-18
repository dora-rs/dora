use crossterm::event::{Event, KeyCode, KeyModifiers};
use dora_core::descriptor::Descriptor;
use dora_message::{common::Timestamped, daemon_to_daemon::InterDaemonEvent};
use itertools::Itertools;
use ratatui::{DefaultTerminal, prelude::*, widgets::*};
use std::{
    borrow::Cow,
    collections::{BTreeMap, BTreeSet, HashMap, VecDeque},
    fmt,
    io::{self, IsTerminal},
    iter,
    sync::{Arc, Mutex},
    time::{Duration, Instant},
};

use crate::{
    command::{
        Executable,
        topic::selector::{TopicIdentifier, TopicSelector, wire_topic_output_id},
    },
    common::CoordinatorOptions,
};

fn parse_window(s: &str) -> Result<usize, String> {
    let val: usize = s.parse().map_err(|e| format!("{e}"))?;
    if val == 0 {
        return Err("window must be at least 1".to_string());
    }
    Ok(val)
}

/// Measure topic publish intervals.
///
/// Subscribe to one or more outputs and display per-topic interval statistics
/// (average, min, max, stddev) over a sliding window. Average frequency (Hz)
/// is derived from the average interval.
///
/// Topic inspection requires debug mode on the dataflow:
///
/// ```yaml
/// debug:
///   enable_debug_inspection: true
/// ```
///
/// If no `DATA` is provided, all outputs from the selected dataflow will be
/// echoed.
///
/// Examples:
///
/// Measure a single topic:
///   dora topic hz -d my-dataflow robot1/pose
///
/// Measure multiple topics with a short window:
///   dora topic hz -d my-dataflow robot1/pose robot2/vel --window 5
///
/// Measure all topics:
///   dora topic hz -d my-dataflow --window 10
///
#[derive(Debug, clap::Args)]
#[clap(verbatim_doc_comment)]
pub struct Hz {
    #[clap(flatten)]
    selector: TopicSelector,

    /// Sliding window size in seconds
    #[clap(long, default_value_t = 10, value_parser = parse_window)]
    window: usize,

    /// Run for this many seconds without TUI, print final stats, and exit.
    /// Required when stdout is not a terminal (e.g. CI, scripting).
    /// Must be at least 1.
    #[clap(long, value_name = "SECONDS", value_parser = clap::value_parser!(u64).range(1..))]
    duration: Option<u64>,

    #[clap(flatten)]
    coordinator: CoordinatorOptions,
}

impl Executable for Hz {
    fn execute(self) -> eyre::Result<()> {
        let session = self.coordinator.connect()?;
        let (dataflow_id, topics, descriptor) = self.selector.resolve_with_descriptor(&session)?;

        let ws_topics: Vec<_> = topics
            .iter()
            .map(|t| (t.node_id.clone(), t.data_id.clone()))
            .collect();

        let (_subscription_id, data_rx) = session.subscribe_topics(dataflow_id, ws_topics)?;

        // Non-interactive path: collect for `--duration`, print final stats.
        if let Some(secs) = self.duration {
            return run_hz_oneshot(self.window, topics, &descriptor, data_rx, secs);
        }

        if !io::stdout().is_terminal() {
            eyre::bail!(
                "`dora topic hz` requires an interactive terminal. \
                 Pass `--duration <SECONDS>` for non-interactive use."
            );
        }

        let terminal = ratatui::init();
        let result = run_hz(terminal, self.window, topics, &descriptor, data_rx);
        ratatui::restore();
        result
    }
}

/// Map `(node_id, wire_output_id) -> index into `stats`` for the per-topic rows.
///
/// Keyed by the id the **daemon** reports, not the public one the selector
/// produced: for a single-`operator:` node those differ (`op/image` vs
/// `image`), so indexing by the public id silently matched nothing and every
/// such topic reported 0 samples while the aggregate row showed real traffic
/// (dora-rs/dora#2893). Doing the translation here keeps the per-frame path a
/// plain map lookup.
fn build_topic_index(
    stats: &[(HzLabel<'_>, Arc<HzStats>)],
    descriptor: &Descriptor,
) -> BTreeMap<(String, String), usize> {
    let nodes: HashMap<_, _> = descriptor.nodes.iter().map(|n| (&n.id, n)).collect();
    let mut topic_index = BTreeMap::new();
    for (i, (label, _)) in stats.iter().enumerate().skip(1) {
        if let HzLabel::Topic(topic) = label {
            let wire = nodes
                .get(&topic.node_id)
                .map(|node| wire_topic_output_id(node, &topic.data_id))
                .unwrap_or_else(|| topic.data_id.clone());
            topic_index.insert((topic.node_id.to_string(), wire.to_string()), i);
        }
    }
    topic_index
}

/// Non-interactive sampler: subscribes for `seconds`, then prints
/// per-topic stats as a plain table and exits.
fn run_hz_oneshot(
    window: usize,
    outputs: BTreeSet<TopicIdentifier>,
    descriptor: &Descriptor,
    data_rx: std::sync::mpsc::Receiver<eyre::Result<Vec<u8>>>,
    seconds: u64,
) -> eyre::Result<()> {
    let mut stats: Vec<(HzLabel<'_>, Arc<HzStats>)> = Vec::with_capacity(outputs.len() + 1);
    stats.push((HzLabel::Aggregate, Arc::new(HzStats::new(window))));
    for topic in &outputs {
        stats.push((HzLabel::Topic(topic), Arc::new(HzStats::new(window))));
    }

    let topic_index = build_topic_index(&stats, descriptor);

    // Track the window as a start instant plus elapsed comparison rather than
    // `Instant::now() + Duration::from_secs(seconds)`: `seconds` comes straight
    // from `--duration`, which clap bounds only from below (`range(1..)`), so a
    // large-but-valid `u64` (e.g. `--duration 10000000000000000000`) would make
    // `Instant + Duration` overflow the monotonic clock and panic. This mirrors
    // the sibling `topic info` sampler, which measures the window the same way.
    let duration = Duration::from_secs(seconds);
    let start = Instant::now();
    while start.elapsed() < duration {
        let remaining = duration.saturating_sub(start.elapsed());
        match data_rx.recv_timeout(remaining) {
            Ok(Ok(payload)) => {
                let event = match Timestamped::deserialize_inter_daemon_event(&payload) {
                    Ok(e) => e,
                    Err(_) => continue,
                };
                if let InterDaemonEvent::Output {
                    node_id,
                    output_id,
                    metadata,
                    ..
                } = event.inner
                {
                    let stamp = metadata.timestamp().get_time().to_duration();
                    let arrived_at = Instant::now();
                    stats[0].1.record(stamp, arrived_at); // aggregate
                    let key = (node_id.to_string(), output_id.to_string());
                    if let Some(&idx) = topic_index.get(&key) {
                        stats[idx].1.record(stamp, arrived_at);
                    }
                }
            }
            Ok(Err(_)) => continue,
            Err(std::sync::mpsc::RecvTimeoutError::Timeout) => break,
            Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => break,
        }
    }

    println!("topic\tavg_ms\tavg_hz\tmin_ms\tmax_ms\tstd_ms\tsamples");
    for (label, hz_stats) in &stats {
        let samples = hz_stats
            .samples
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .len();
        match hz_stats.calculate() {
            Some(s) => println!(
                "{}\t{:.2}\t{:.2}\t{:.2}\t{:.2}\t{:.2}\t{}",
                label, s.avg_ms, s.avg_hz, s.min_ms, s.max_ms, s.std_ms, samples
            ),
            None => println!("{}\t-\t-\t-\t-\t-\t{}", label, samples),
        }
    }
    Ok(())
}

#[derive(Debug)]
struct HzStats {
    samples: Mutex<VecDeque<Sample>>,
    window_duration: Duration,
}

/// One received frame.
///
/// The node's producer HLC stamp drives the interval statistics: it is stamped
/// in `send_output` at the source, so the daemon->coordinator->CLI relay adds
/// only propagation and serialization latency, which would otherwise inflate
/// the measured intervals (dora-rs/dora#3509). The wall-clock arrival is kept
/// separately so the live TUI's liveness window still decays to 0 Hz when a
/// publisher stalls, instead of freezing on the last producer stamp.
#[derive(Debug, Clone, Copy)]
struct Sample {
    producer_stamp: Duration,
    arrived_at: Instant,
}

impl HzStats {
    fn new(window_secs: usize) -> Self {
        Self {
            samples: Mutex::new(VecDeque::new()),
            window_duration: Duration::from_secs(window_secs as u64),
        }
    }

    /// Record a frame received at wall-clock `arrived_at` with producer stamp
    /// `producer_stamp`.
    fn record(&self, producer_stamp: Duration, arrived_at: Instant) {
        let mut samples = self.samples.lock().unwrap_or_else(|e| e.into_inner());
        samples.push_back(Sample {
            producer_stamp,
            arrived_at,
        });
        self.prune(&mut samples, arrived_at);
    }

    /// Drop samples whose *arrival* fell out of the sliding window.
    fn prune(&self, samples: &mut VecDeque<Sample>, now: Instant) {
        while let Some(front) = samples.front() {
            if now.saturating_duration_since(front.arrived_at) > self.window_duration {
                samples.pop_front();
            } else {
                break;
            }
        }
    }

    /// Live rate in the last `window`, counting arrivals (not producer stamps):
    /// once a publisher stops, the reading falls to 0 Hz within `window`.
    fn rate_last(&self, now: Instant, window: Duration) -> f64 {
        let samples = self.samples.lock().unwrap_or_else(|e| e.into_inner());
        let count = samples
            .iter()
            .rev()
            .take_while(|s| now.saturating_duration_since(s.arrived_at) <= window)
            .count() as f64;
        count / window.as_secs_f64()
    }

    fn intervals_ms(&self) -> Vec<f64> {
        self.intervals_ms_at(Instant::now())
    }

    fn intervals_ms_at(&self, now: Instant) -> Vec<f64> {
        let mut samples = self.samples.lock().unwrap_or_else(|e| e.into_inner());
        self.prune(&mut samples, now);
        samples
            .iter()
            .tuple_windows()
            .filter_map(|(a, b)| {
                let dt = b
                    .producer_stamp
                    .saturating_sub(a.producer_stamp)
                    .as_secs_f64()
                    * 1000.0;
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
    descriptor: &Descriptor,
    data_rx: std::sync::mpsc::Receiver<eyre::Result<Vec<u8>>>,
) -> eyre::Result<()> {
    // Build stats vec: index 0 is the aggregate, rest are per-topic
    let mut stats: Vec<(HzLabel<'_>, Arc<HzStats>)> = Vec::with_capacity(outputs.len() + 1);
    stats.push((HzLabel::Aggregate, Arc::new(HzStats::new(window))));
    for topic in &outputs {
        stats.push((HzLabel::Topic(topic), Arc::new(HzStats::new(window))));
    }

    let topic_index = build_topic_index(&stats, descriptor);

    let mut selected: usize = 0;
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
                    node_id,
                    output_id,
                    metadata,
                    ..
                } => {
                    let stamp = metadata.timestamp().get_time().to_duration();
                    all_stats.record(stamp, Instant::now());
                    let key = (node_id.to_string(), output_id.to_string());
                    if let Some(&idx) = topic_index_clone.get(&key) {
                        stats_clones[idx].record(stamp, Instant::now());
                    }
                }
                InterDaemonEvent::OutputClosed { .. } => {}
                // `InterDaemonEvent` is `#[non_exhaustive]`: skip events this build predates.
                _ => {}
            }
        }
    });

    loop {
        // Live rate over the last 1 s of wall-clock *arrivals*, so a publisher
        // that stops publishing decays to 0 Hz instead of freezing at the last
        // producer stamp (the stamp timeline only drives the interval stats).
        let sub_window = Duration::from_millis(1000);
        let now = Instant::now();
        for (i, (_topic, s)) in stats.iter().enumerate() {
            let hz = s.rate_last(now, sub_window);
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

        if crossterm::event::poll(Duration::from_millis(50))?
            && let Event::Key(key) = crossterm::event::read()?
        {
            if matches!(key.code, KeyCode::Char('q') | KeyCode::Esc)
                || (key.modifiers.contains(KeyModifiers::CONTROL) && key.code == KeyCode::Char('c'))
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

    #[test]
    fn lock_reads_recover_from_poisoning() {
        use std::sync::Arc;

        let stats = Arc::new(HzStats::new(1));

        // Poison the samples mutex by panicking while holding the lock.
        let poisoner = Arc::clone(&stats);
        let _ = std::thread::spawn(move || {
            let _guard = poisoner.samples.lock().unwrap();
            panic!("poison the samples lock");
        })
        .join();

        // Writer and reader must still work despite the poisoned lock rather
        // than panicking on a bare `.unwrap()`.
        let now = Instant::now();
        stats.record(Duration::from_secs(1), now);
        stats.record(Duration::from_secs(2), now);
        let _ = stats.intervals_ms();
    }

    // A huge window (`parse_window` imposes no upper bound) must not panic in
    // `record`: the subtraction saturates rather than overflow. Regression for
    // the `dora topic hz --window <huge>` panic.
    #[test]
    fn record_does_not_underflow_on_huge_window() {
        let stats = HzStats::new(u32::MAX as usize);
        // Nothing pruned: the huge window saturates the cutoff, so both are kept.
        let now = Instant::now();
        stats.record(Duration::from_secs(100), now);
        stats.record(Duration::from_secs(101), now);
        assert_eq!(stats.samples.lock().unwrap().len(), 2);
    }

    // Intervals must use the producer's timeline, not the receiver's. Feed two
    // stamps that arrived AFTER their nominal production times: the first
    // interval reflects the stamp delta, not the arrival delta
    // (dora-rs/dora#3509).
    #[test]
    fn intervals_use_producer_timestamps_not_arrival_time() {
        let stats = HzStats::new(10);
        let now = Instant::now();
        stats.record(Duration::from_millis(1_000), now);
        // Producer says 100 ms apart...
        stats.record(
            Duration::from_millis(1_100),
            now + Duration::from_millis(500),
        );
        // ...but the receiver saw them 500 ms apart. Measuring against arrival
        // would report a ~2 Hz interval.

        let intervals = stats.intervals_ms();
        assert_eq!(intervals.len(), 1);
        // ~10 Hz, i.e. 100 ms, not 500 ms.
        assert!((intervals[0] - 100.0).abs() < 1.0);
    }

    // A stamp arriving out of order (producer timestamp older than the last
    // one seen) must not poison the stats: the interval computes to the stamp
    // delta, which can be zero or non-monotonic, and such pairs are skipped.
    #[test]
    fn record_skips_non_positive_intervals() {
        let stats = HzStats::new(10);
        let now = Instant::now();
        stats.record(Duration::from_millis(1_000), now);
        // Spurious duplicate / out-of-order stamp: zero interval.
        stats.record(Duration::from_millis(1_000), now + Duration::from_millis(1));
        stats.record(Duration::from_millis(1_200), now + Duration::from_millis(2));
        assert_eq!(stats.intervals_ms().len(), 1);
        assert!((stats.intervals_ms()[0] - 200.0).abs() < 1.0);
    }

    // The live gauge anchors to wall-clock arrivals, not producer stamps: a
    // publisher that stops publishing decays to 0 Hz instead of freezing at the
    // last displayed rate.
    #[test]
    fn rate_last_decays_when_publisher_stalls() {
        let stats = HzStats::new(10);
        let base = Instant::now();
        // A frame that arrived 5 s ago is outside a 1 s liveness window...
        stats.record(Duration::from_millis(1_000), base);
        assert_eq!(
            stats.rate_last(base + Duration::from_secs(5), Duration::from_secs(1)),
            0.0
        );
        // ...while a frame that arrived now is counted.
        stats.record(Duration::from_millis(1_100), base + Duration::from_secs(5));
        assert_eq!(
            stats.rate_last(base + Duration::from_secs(5), Duration::from_secs(1)),
            1.0
        );
    }

    // The 1 s window prunes by *arrival*: two samples that both arrived within
    // the last second stay, so a current producer stamp always yields intervals;
    // the moment their arrival ages past the window they drop out, regardless
    // of how recent their stamps look.
    #[test]
    fn stale_samples_are_pruned_by_arrival_not_stamp() {
        let stats = HzStats::new(1);
        let base = Instant::now();
        // Two stamps that read as fresh (1.05 s / 1.10 s)...
        stats.record(Duration::from_millis(1_050), base);
        stats.record(
            Duration::from_millis(1_100),
            base + Duration::from_millis(50),
        );
        // ...but by query time both arrivals are older than the 1 s window, so
        // both prune and no interval survives: pruning follows arrival, not stamp.
        assert!(
            stats
                .intervals_ms_at(base + Duration::from_secs(2))
                .is_empty()
        );
    }

    // `--duration` is bounded only from below (`range(1..)`), so a large-but-
    // valid `u64` must not abort the non-interactive sampler. Previously
    // `Instant::now() + Duration::from_secs(seconds)` overflowed the monotonic
    // clock and panicked before the sampling loop even started. A disconnected
    // receiver makes the loop break out immediately, so the only thing under
    // test is that setting up the (huge) window does not panic.
    #[test]
    fn run_hz_oneshot_does_not_panic_on_oversized_duration() {
        use dora_core::descriptor::DescriptorExt;
        use std::collections::BTreeSet;

        let descriptor = Descriptor::parse(b"nodes: []".to_vec()).unwrap();
        let (tx, rx) = std::sync::mpsc::channel::<eyre::Result<Vec<u8>>>();
        drop(tx); // disconnect so the sampler breaks out immediately

        // 10^19 s is a valid u64 but overflows `Instant + Duration`.
        run_hz_oneshot(
            10,
            BTreeSet::new(),
            &descriptor,
            rx,
            10_000_000_000_000_000_000,
        )
        .expect("oversized --duration must not panic or error");
    }
}
