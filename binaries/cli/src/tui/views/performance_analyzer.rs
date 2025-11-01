use crossterm::event::{KeyCode, KeyEvent, KeyModifiers};
/// Performance Analyzer View - Real-time performance monitoring (Issue #26)
use ratatui::{
    Frame,
    layout::{Constraint, Direction, Layout, Rect},
    style::{Color, Modifier, Style},
    symbols,
    text::{Line, Span},
    widgets::{Axis, Block, Borders, Chart, Dataset, GraphType, List, ListItem, Paragraph, Wrap},
};
use std::time::Duration;

#[cfg(test)]
use super::TimeRange;
use super::{
    AlertSeverity, AnalyzerTab, BaseView, MetricType, PerformanceAlert, PerformanceAnalyzerState,
    View, ViewAction,
};
use crate::tui::{Result, app::AppState, theme::ThemeConfig};

/// Performance Analyzer View for system performance monitoring
pub struct PerformanceAnalyzerView {
    base: BaseView,
    theme: ThemeConfig,
    /// Current analyzer state
    pub state: PerformanceAnalyzerState,
}

impl PerformanceAnalyzerView {
    /// Create a new PerformanceAnalyzerView
    pub fn new(theme: &ThemeConfig) -> Self {
        Self {
            base: BaseView::new("Performance Analyzer".to_string())
                .with_auto_refresh(Duration::from_secs(1)),
            theme: theme.clone(),
            state: PerformanceAnalyzerState::new(),
        }
    }

    /// Generate mock performance data for a metric
    fn generate_mock_data(&self, metric_type: MetricType) -> Vec<(f64, f64)> {
        let data_points = self.state.time_range.data_points();
        let duration_secs = self.state.time_range.duration_secs() as f64;

        (0..data_points)
            .map(|i| {
                let x = i as f64 / data_points as f64 * duration_secs;
                let y = match metric_type {
                    MetricType::Cpu => {
                        // Simulated CPU usage with some variation
                        let base = 45.0;
                        let variation = ((i as f64 * 0.3).sin() * 15.0).abs();
                        base + variation
                    }
                    MetricType::Memory => {
                        // Simulated memory usage gradually increasing
                        let base = 60.0;
                        let trend = (i as f64 / data_points as f64) * 10.0;
                        let variation = ((i as f64 * 0.2).cos() * 5.0).abs();
                        base + trend + variation
                    }
                    MetricType::MessageRate => {
                        // Simulated message rate with bursts
                        let base = 150.0;
                        let burst = if i % 10 < 3 { 200.0 } else { 0.0 };
                        let variation = ((i as f64 * 0.4).sin() * 50.0).abs();
                        base + burst + variation
                    }
                    MetricType::Latency => {
                        // Simulated latency with spikes
                        let base = 25.0;
                        let spike = if i % 15 == 7 { 80.0 } else { 0.0 };
                        let variation = ((i as f64 * 0.5).sin() * 15.0).abs();
                        base + spike + variation
                    }
                };
                (x, y)
            })
            .collect()
    }

    /// Render tab bar
    fn render_tab_bar(&self, f: &mut Frame, area: Rect) {
        let tabs = AnalyzerTab::all();
        let tab_titles: Vec<Line> = tabs
            .iter()
            .map(|tab| {
                let style = if *tab == self.state.active_tab {
                    Style::default()
                        .fg(self.theme.colors.primary)
                        .add_modifier(Modifier::BOLD)
                } else {
                    Style::default().fg(self.theme.colors.muted)
                };

                Line::from(vec![
                    Span::raw(" ["),
                    Span::styled(tab.shortcut(), style),
                    Span::raw("] "),
                    Span::styled(tab.name(), style),
                    Span::raw(" "),
                ])
            })
            .collect();

        let tabs_text = tab_titles
            .into_iter()
            .flat_map(|line| vec![line, Line::from("")])
            .collect::<Vec<_>>();

        let time_range_info = format!("Time Range: {} [t]", self.state.time_range.name());
        let tabs_block = Paragraph::new(tabs_text).block(
            Block::default()
                .title(time_range_info)
                .borders(Borders::ALL),
        );

        f.render_widget(tabs_block, area);
    }

    /// Render help bar
    fn render_help_bar(&self, f: &mut Frame, area: Rect) {
        let help_text = match self.state.active_tab {
            AnalyzerTab::Overview => "[q]uit [←→]tabs [t]ime range [r]efresh",
            AnalyzerTab::Metrics => "[q]uit [←→]tabs [↑↓]metric [t]ime range",
            AnalyzerTab::Alerts => "[q]uit [←→]tabs [↑↓]navigate [c]lear",
        };

        let help = Paragraph::new(help_text)
            .style(Style::default().fg(self.theme.colors.muted))
            .block(Block::default().borders(Borders::ALL));

        f.render_widget(help, area);
    }

    /// Render a single performance chart
    fn render_chart(&self, f: &mut Frame, area: Rect, metric_type: MetricType) {
        let data = self.generate_mock_data(metric_type);

        if data.is_empty() {
            return;
        }

        // Calculate bounds
        let max_y = data.iter().map(|(_, y)| *y).fold(0.0f64, f64::max);
        let y_max = (max_y * 1.2).max(10.0); // Add 20% headroom
        let x_max = self.state.time_range.duration_secs() as f64;

        let datasets = vec![
            Dataset::default()
                .name(metric_type.name())
                .marker(symbols::Marker::Braille)
                .graph_type(GraphType::Line)
                .style(Style::default().fg(self.get_metric_color(metric_type)))
                .data(&data),
        ];

        let title = format!("{} ({})", metric_type.name(), metric_type.unit());

        let x_axis = Axis::default()
            .title("Time")
            .style(Style::default().fg(self.theme.colors.muted))
            .bounds([0.0, x_max])
            .labels(vec![Span::raw(""), Span::raw(""), Span::raw("")]);

        let y_axis = Axis::default()
            .title(metric_type.unit())
            .style(Style::default().fg(self.theme.colors.muted))
            .bounds([0.0, y_max])
            .labels(vec![
                Span::raw("0"),
                Span::raw(format!("{:.0}", y_max / 2.0)),
                Span::raw(format!("{y_max:.0}")),
            ]);

        let chart = Chart::new(datasets)
            .block(
                Block::default()
                    .title(title)
                    .borders(Borders::ALL)
                    .border_style(Style::default().fg(self.theme.colors.border)),
            )
            .x_axis(x_axis)
            .y_axis(y_axis);

        f.render_widget(chart, area);
    }

    /// Get color for a metric type
    fn get_metric_color(&self, metric_type: MetricType) -> Color {
        match metric_type {
            MetricType::Cpu => Color::Cyan,
            MetricType::Memory => Color::Green,
            MetricType::MessageRate => Color::Yellow,
            MetricType::Latency => Color::Magenta,
        }
    }

    /// Render Overview tab with 2x2 grid of charts
    fn render_overview_tab(&self, f: &mut Frame, area: Rect) {
        let vertical_chunks = Layout::default()
            .direction(Direction::Vertical)
            .constraints([Constraint::Percentage(50), Constraint::Percentage(50)])
            .split(area);

        let top_chunks = Layout::default()
            .direction(Direction::Horizontal)
            .constraints([Constraint::Percentage(50), Constraint::Percentage(50)])
            .split(vertical_chunks[0]);

        let bottom_chunks = Layout::default()
            .direction(Direction::Horizontal)
            .constraints([Constraint::Percentage(50), Constraint::Percentage(50)])
            .split(vertical_chunks[1]);

        // Render 4 charts in grid
        self.render_chart(f, top_chunks[0], MetricType::Cpu);
        self.render_chart(f, top_chunks[1], MetricType::Memory);
        self.render_chart(f, bottom_chunks[0], MetricType::MessageRate);
        self.render_chart(f, bottom_chunks[1], MetricType::Latency);
    }

    /// Render Metrics tab with detailed single metric view
    fn render_metrics_tab(&self, f: &mut Frame, area: Rect) {
        let chunks = Layout::default()
            .direction(Direction::Vertical)
            .constraints([Constraint::Min(10), Constraint::Length(8)])
            .split(area);

        // Render large chart for selected metric
        self.render_chart(f, chunks[0], self.state.selected_metric);

        // Render metric selector and stats
        self.render_metric_stats(f, chunks[1]);
    }

    /// Render metric statistics
    fn render_metric_stats(&self, f: &mut Frame, area: Rect) {
        let data = self.generate_mock_data(self.state.selected_metric);

        let (min, max, avg) = if !data.is_empty() {
            let values: Vec<f64> = data.iter().map(|(_, y)| *y).collect();
            let min_val = values.iter().fold(f64::INFINITY, |a, &b| a.min(b));
            let max_val = values.iter().fold(f64::NEG_INFINITY, |a, &b| a.max(b));
            let avg_val = values.iter().sum::<f64>() / values.len() as f64;
            (min_val, max_val, avg_val)
        } else {
            (0.0, 0.0, 0.0)
        };

        let stats_text = vec![
            Line::from(vec![
                Span::styled(
                    "Selected Metric: ",
                    Style::default()
                        .fg(self.theme.colors.primary)
                        .add_modifier(Modifier::BOLD),
                ),
                Span::styled(
                    self.state.selected_metric.name(),
                    Style::default().fg(self.theme.colors.text),
                ),
            ]),
            Line::from(""),
            Line::from(vec![
                Span::styled("Current: ", Style::default().fg(self.theme.colors.muted)),
                Span::raw(format!(
                    "{:.2} {}",
                    data.last().map(|(_, y)| *y).unwrap_or(0.0),
                    self.state.selected_metric.unit()
                )),
            ]),
            Line::from(vec![
                Span::styled("Min: ", Style::default().fg(self.theme.colors.muted)),
                Span::raw(format!("{:.2} {}", min, self.state.selected_metric.unit())),
            ]),
            Line::from(vec![
                Span::styled("Max: ", Style::default().fg(self.theme.colors.muted)),
                Span::raw(format!("{:.2} {}", max, self.state.selected_metric.unit())),
            ]),
            Line::from(vec![
                Span::styled("Avg: ", Style::default().fg(self.theme.colors.muted)),
                Span::raw(format!("{:.2} {}", avg, self.state.selected_metric.unit())),
            ]),
        ];

        let paragraph = Paragraph::new(stats_text)
            .block(
                Block::default()
                    .title("Statistics [Use ↑↓ to change metric]")
                    .borders(Borders::ALL),
            )
            .wrap(Wrap { trim: true });

        f.render_widget(paragraph, area);
    }

    /// Render Alerts tab
    fn render_alerts_tab(&self, f: &mut Frame, area: Rect) {
        if self.state.alerts.is_empty() {
            let no_alerts = Paragraph::new("No performance alerts")
                .block(
                    Block::default()
                        .title("Performance Alerts")
                        .borders(Borders::ALL),
                )
                .style(Style::default().fg(self.theme.colors.muted));
            f.render_widget(no_alerts, area);
            return;
        }

        let items: Vec<ListItem> = self
            .state
            .alerts
            .iter()
            .enumerate()
            .map(|(i, alert)| {
                let style = if i == self.state.selected_alert {
                    Style::default()
                        .fg(self.theme.colors.highlight)
                        .add_modifier(Modifier::BOLD)
                } else {
                    Style::default().fg(self.theme.colors.text)
                };

                let severity_color = match alert.severity {
                    AlertSeverity::Warning => Color::Yellow,
                    AlertSeverity::Critical => Color::Red,
                };

                let content = vec![
                    Span::styled(alert.severity.symbol(), Style::default().fg(severity_color)),
                    Span::raw(" "),
                    Span::raw(&alert.message),
                ];

                ListItem::new(Line::from(content)).style(style)
            })
            .collect();

        let list = List::new(items)
            .block(
                Block::default()
                    .title(format!(
                        "Performance Alerts ({}) [c]lear",
                        self.state.alerts.len()
                    ))
                    .borders(Borders::ALL),
            )
            .highlight_style(
                Style::default()
                    .fg(self.theme.colors.highlight)
                    .add_modifier(Modifier::BOLD),
            );

        f.render_widget(list, area);
    }

    /// Update performance alerts based on current metrics
    fn update_alerts(&mut self) {
        // Generate mock alerts based on thresholds
        let mut new_alerts = Vec::new();

        for metric_type in MetricType::all() {
            let data = self.generate_mock_data(metric_type);
            if let Some((_, current_value)) = data.last() {
                let warning_threshold = metric_type.warning_threshold();
                let critical_threshold = metric_type.critical_threshold();

                if *current_value >= critical_threshold {
                    new_alerts.push(PerformanceAlert::new(
                        metric_type,
                        AlertSeverity::Critical,
                        *current_value,
                        critical_threshold,
                    ));
                } else if *current_value >= warning_threshold {
                    new_alerts.push(PerformanceAlert::new(
                        metric_type,
                        AlertSeverity::Warning,
                        *current_value,
                        warning_threshold,
                    ));
                }
            }
        }

        self.state.alerts = new_alerts;
    }
}

impl View for PerformanceAnalyzerView {
    fn render(&mut self, f: &mut Frame, area: Rect, _app_state: &AppState) {
        // Main layout: title, tabs, content, help
        let chunks = Layout::default()
            .direction(Direction::Vertical)
            .constraints([
                Constraint::Length(3), // Title
                Constraint::Length(7), // Tabs
                Constraint::Min(10),   // Content
                Constraint::Length(3), // Help
            ])
            .split(area);

        // Render title
        let title = format!("Performance Analyzer - {}", self.state.time_range.name());
        let title_paragraph = Paragraph::new(title)
            .block(Block::default().borders(Borders::ALL))
            .style(
                Style::default()
                    .fg(self.theme.colors.primary)
                    .add_modifier(Modifier::BOLD),
            );
        f.render_widget(title_paragraph, chunks[0]);

        // Render tabs
        self.render_tab_bar(f, chunks[1]);

        // Render active tab content
        match self.state.active_tab {
            AnalyzerTab::Overview => self.render_overview_tab(f, chunks[2]),
            AnalyzerTab::Metrics => self.render_metrics_tab(f, chunks[2]),
            AnalyzerTab::Alerts => self.render_alerts_tab(f, chunks[2]),
        }

        // Render help bar
        self.render_help_bar(f, chunks[3]);
    }

    async fn handle_key(&mut self, key: KeyEvent, _app_state: &mut AppState) -> Result<ViewAction> {
        match key.code {
            // Quit
            KeyCode::Char('q') | KeyCode::Esc => {
                return Ok(ViewAction::PopView);
            }

            // Tab navigation
            KeyCode::Left | KeyCode::Char('h') => {
                self.state.prev_tab();
            }
            KeyCode::Right | KeyCode::Char('l') => {
                self.state.next_tab();
            }

            // Number keys for direct tab access
            KeyCode::Char('1') => self.state.switch_tab(AnalyzerTab::Overview),
            KeyCode::Char('2') => self.state.switch_tab(AnalyzerTab::Metrics),
            KeyCode::Char('3') => self.state.switch_tab(AnalyzerTab::Alerts),

            // Time range cycling
            KeyCode::Char('t') => {
                self.state.next_time_range();
            }

            // Metric selection (Metrics tab)
            KeyCode::Up | KeyCode::Char('k') if self.state.active_tab == AnalyzerTab::Metrics => {
                self.state.prev_metric();
            }
            KeyCode::Down | KeyCode::Char('j') if self.state.active_tab == AnalyzerTab::Metrics => {
                self.state.next_metric();
            }

            // Alert navigation (Alerts tab)
            KeyCode::Up | KeyCode::Char('k') if self.state.active_tab == AnalyzerTab::Alerts => {
                self.state.prev_alert();
            }
            KeyCode::Down | KeyCode::Char('j') if self.state.active_tab == AnalyzerTab::Alerts => {
                self.state.next_alert();
            }

            // Clear alerts
            KeyCode::Char('c') if self.state.active_tab == AnalyzerTab::Alerts => {
                self.state.clear_alerts();
            }

            // Refresh
            KeyCode::Char('r') if key.modifiers.contains(KeyModifiers::CONTROL) => {
                self.update_alerts();
                return Ok(ViewAction::Refresh);
            }

            _ => {}
        }

        Ok(ViewAction::None)
    }

    async fn update(&mut self, _app_state: &mut AppState) -> Result<()> {
        self.state.mark_refreshed();
        self.update_alerts();
        Ok(())
    }

    fn help_text(&self) -> Vec<(&str, &str)> {
        vec![
            ("q/Esc", "Back to previous view"),
            ("←/→", "Switch tabs"),
            ("1-3", "Jump to specific tab"),
            ("t", "Cycle time range"),
            ("↑/↓", "Navigate items"),
            ("c", "Clear alerts"),
            ("Ctrl+r", "Refresh"),
        ]
    }

    fn title(&self) -> &str {
        &self.base.title
    }

    fn auto_refresh(&self) -> Option<Duration> {
        self.base.auto_refresh_interval
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_performance_analyzer_view_creation() {
        let theme = ThemeConfig::default();
        let view = PerformanceAnalyzerView::new(&theme);

        assert_eq!(view.state.active_tab, AnalyzerTab::Overview);
        assert_eq!(view.state.time_range, TimeRange::LastHour);
    }

    #[test]
    fn test_generate_mock_data() {
        let theme = ThemeConfig::default();
        let view = PerformanceAnalyzerView::new(&theme);

        let data = view.generate_mock_data(MetricType::Cpu);
        assert_eq!(data.len(), 60); // Last hour = 60 data points

        // Verify data is in valid range
        for (x, y) in &data {
            assert!(*x >= 0.0);
            assert!(*y >= 0.0);
            assert!(*y <= 100.0); // CPU should be 0-100%
        }
    }

    #[test]
    fn test_metric_color_assignment() {
        let theme = ThemeConfig::default();
        let view = PerformanceAnalyzerView::new(&theme);

        assert_eq!(view.get_metric_color(MetricType::Cpu), Color::Cyan);
        assert_eq!(view.get_metric_color(MetricType::Memory), Color::Green);
        assert_eq!(
            view.get_metric_color(MetricType::MessageRate),
            Color::Yellow
        );
        assert_eq!(view.get_metric_color(MetricType::Latency), Color::Magenta);
    }
}
