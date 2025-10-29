use super::{BaseView, View, ViewAction};
use crate::tui::{
    Frame, Result,
    app::{AppState, SystemMetrics},
    theme::ThemeConfig,
};
use crossterm::event::KeyEvent;
use ratatui::{
    layout::{Constraint, Direction, Layout, Rect},
    style::Style,
    text::{Line, Span},
    widgets::{Block, Borders, Gauge, Paragraph},
};
use std::time::Duration;

pub struct SystemMonitorView {
    base: BaseView,
    theme: ThemeConfig,
}

impl SystemMonitorView {
    pub fn new(theme: &ThemeConfig) -> Self {
        Self {
            base: BaseView::new("System Monitor".to_string()),
            theme: theme.clone(),
        }
    }
}

impl View for SystemMonitorView {
    fn render(&mut self, f: &mut Frame, area: Rect, app_state: &AppState) {
        let block = self.theme.styled_block("System Monitor");
        let inner = block.inner(area);
        f.render_widget(block, area);

        let metrics = &app_state.system_metrics;

        let rows = Layout::default()
            .direction(Direction::Vertical)
            .constraints([
                Constraint::Length(5), // CPU
                Constraint::Length(5), // Memory
                Constraint::Length(5), // Disk
                Constraint::Length(6), // Network
                Constraint::Min(4),    // Load averages / uptime
            ])
            .split(inner);

        render_cpu(f, rows[0], metrics, &self.theme);
        render_memory(f, rows[1], metrics, &self.theme);
        render_disk(f, rows[2], metrics, &self.theme);
        render_network(f, rows[3], metrics, &self.theme);
        render_load_section(f, rows[4], metrics, &self.theme);
    }

    async fn handle_key(
        &mut self,
        _key: KeyEvent,
        _app_state: &mut AppState,
    ) -> Result<ViewAction> {
        Ok(ViewAction::None)
    }

    async fn update(&mut self, _app_state: &mut AppState) -> Result<()> {
        Ok(())
    }

    fn help_text(&self) -> Vec<(&str, &str)> {
        vec![("Esc", "Go back")]
    }

    fn title(&self) -> &str {
        &self.base.title
    }
}

fn render_cpu(f: &mut Frame, area: Rect, metrics: &SystemMetrics, theme: &ThemeConfig) {
    let cpu_gauge = Gauge::default()
        .block(Block::default().title("CPU").borders(Borders::ALL))
        .gauge_style(theme.percentage_style(metrics.cpu_usage))
        .percent(metrics.cpu_usage.clamp(0.0, 100.0).round() as u16)
        .label(format!(
            "{:.1}% | {} processes",
            metrics.cpu_usage, metrics.process_count
        ));
    f.render_widget(cpu_gauge, area);
}

fn render_memory(f: &mut Frame, area: Rect, metrics: &SystemMetrics, theme: &ThemeConfig) {
    let memory = &metrics.memory;
    let gauge = Gauge::default()
        .block(Block::default().title("Memory").borders(Borders::ALL))
        .gauge_style(theme.percentage_style(metrics.memory_usage))
        .percent(metrics.memory_usage.clamp(0.0, 100.0).round() as u16)
        .label(format!(
            "{:.1}% | {} / {}",
            metrics.memory_usage,
            format_bytes(memory.used_bytes),
            format_bytes(memory.total_bytes)
        ));

    f.render_widget(gauge, area);
}

fn render_disk(f: &mut Frame, area: Rect, metrics: &SystemMetrics, theme: &ThemeConfig) {
    let disk = &metrics.disk;
    let gauge = Gauge::default()
        .block(Block::default().title("Disk").borders(Borders::ALL))
        .gauge_style(theme.percentage_style(disk.usage_percent))
        .percent(disk.usage_percent.clamp(0.0, 100.0).round() as u16)
        .label(format!(
            "{:.1}% | {} / {}",
            disk.usage_percent,
            format_bytes(disk.used_bytes),
            format_bytes(disk.total_bytes)
        ));

    f.render_widget(gauge, area);
}

fn render_network(f: &mut Frame, area: Rect, metrics: &SystemMetrics, theme: &ThemeConfig) {
    let network = &metrics.network;
    let block = Block::default()
        .title("Network")
        .borders(Borders::ALL)
        .border_style(Style::default().fg(theme.colors.border));
    let inner = block.inner(area);
    f.render_widget(block, area);

    let lines = vec![
        Line::from(vec![
            Span::styled("RX: ", Style::default().fg(theme.colors.text)),
            Span::styled(
                format!("{}/s", format_rate(network.received_per_second)),
                Style::default().fg(theme.colors.primary),
            ),
            Span::raw("  "),
            Span::styled(
                format!("total {}", format_bytes(network.total_received)),
                Style::default().fg(theme.colors.muted),
            ),
        ]),
        Line::from(vec![
            Span::styled("TX: ", Style::default().fg(theme.colors.text)),
            Span::styled(
                format!("{}/s", format_rate(network.transmitted_per_second)),
                Style::default().fg(theme.colors.primary),
            ),
            Span::raw("  "),
            Span::styled(
                format!("total {}", format_bytes(network.total_transmitted)),
                Style::default().fg(theme.colors.muted),
            ),
        ]),
    ];

    let paragraph = Paragraph::new(lines);
    f.render_widget(paragraph, inner);
}

fn render_load_section(f: &mut Frame, area: Rect, metrics: &SystemMetrics, theme: &ThemeConfig) {
    let load = metrics.load_average.as_ref();
    let block = Block::default()
        .title("Load / Uptime")
        .borders(Borders::ALL)
        .border_style(Style::default().fg(theme.colors.border));
    let inner = block.inner(area);
    f.render_widget(block, area);

    let mut lines = Vec::with_capacity(3);

    if let Some(avg) = load {
        lines.push(Line::from(vec![
            Span::styled("Load: ", Style::default().fg(theme.colors.text)),
            Span::styled(
                format!("{:.2}, {:.2}, {:.2}", avg.one, avg.five, avg.fifteen),
                Style::default().fg(theme.colors.accent),
            ),
        ]));
    } else {
        lines.push(Line::from(vec![Span::styled(
            "Load: n/a",
            Style::default().fg(theme.colors.muted),
        )]));
    }

    lines.push(Line::from(vec![
        Span::styled("Uptime: ", Style::default().fg(theme.colors.text)),
        Span::styled(
            format_duration(metrics.uptime),
            Style::default().fg(theme.colors.primary),
        ),
    ]));

    if let Some(timestamp) = metrics.last_update {
        lines.push(Line::from(vec![
            Span::styled("Updated: ", Style::default().fg(theme.colors.text)),
            Span::styled(
                format!("{} ago", format_duration(timestamp.elapsed())),
                Style::default().fg(theme.colors.muted),
            ),
        ]));
    }

    let paragraph = Paragraph::new(lines);
    f.render_widget(paragraph, inner);
}

fn format_bytes(bytes: u64) -> String {
    const UNITS: [&str; 5] = ["B", "KB", "MB", "GB", "TB"];
    let mut value = bytes as f64;
    let mut unit_index = 0;

    while value >= 1024.0 && unit_index < UNITS.len() - 1 {
        value /= 1024.0;
        unit_index += 1;
    }

    if unit_index == 0 {
        format!("{:.0} {}", value, UNITS[unit_index])
    } else {
        format!("{:.1} {}", value, UNITS[unit_index])
    }
}

fn format_rate(rate: f64) -> String {
    const UNITS: [&str; 5] = ["B", "KB", "MB", "GB", "TB"];
    let mut value = rate;
    let mut unit_index = 0;

    while value >= 1024.0 && unit_index < UNITS.len() - 1 {
        value /= 1024.0;
        unit_index += 1;
    }

    if unit_index == 0 {
        format!("{:.0} {}", value, UNITS[unit_index])
    } else {
        format!("{:.1} {}", value, UNITS[unit_index])
    }
}

fn format_duration(duration: Duration) -> String {
    let total_secs = duration.as_secs();
    let days = total_secs / 86_400;
    let hours = (total_secs % 86_400) / 3_600;
    let minutes = (total_secs % 3_600) / 60;
    let seconds = total_secs % 60;

    if days > 0 {
        format!("{}d {}h", days, hours)
    } else if hours > 0 {
        format!("{}h {}m", hours, minutes)
    } else if minutes > 0 {
        format!("{}m", minutes)
    } else {
        format!("{}s", seconds)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn format_bytes_handles_units() {
        assert_eq!(format_bytes(500), "500 B");
        assert_eq!(format_bytes(1_500), "1.5 KB");
        assert_eq!(format_bytes(1_048_576), "1.0 MB");
    }

    #[test]
    fn format_rate_handles_units() {
        assert_eq!(format_rate(256.0), "256 B");
        assert_eq!(format_rate(2_048.0), "2.0 KB");
    }

    #[test]
    fn format_duration_handles_ranges() {
        assert_eq!(format_duration(Duration::from_secs(45)), "45s");
        assert_eq!(format_duration(Duration::from_secs(600)), "10m");
        assert_eq!(format_duration(Duration::from_secs(3_600)), "1h 0m");
        assert_eq!(format_duration(Duration::from_secs(172_800)), "2d 0h");
    }
}
