use ratatui::{
    Frame,
    layout::{Constraint, Direction, Layout, Rect},
    style::Style,
    text::{Line, Span},
    widgets::{Block, Borders, Paragraph, Sparkline},
};
use std::future::Future;
/// Performance Charts Component for Dashboard (Issue #24)
use std::pin::Pin;

use crate::tui::{
    Result,
    app::AppState,
    components::{Component, ComponentEvent, ComponentType},
    theme::ThemeConfig,
    views::{PerformanceMetrics, ViewAction},
};

pub struct PerformanceChartsComponent {
    metrics: PerformanceMetrics,
    focused: bool,
}

impl PerformanceChartsComponent {
    pub fn new() -> Self {
        Self {
            metrics: PerformanceMetrics::new(60), // Store 60 data points
            focused: false,
        }
    }

    pub fn set_metrics(&mut self, metrics: PerformanceMetrics) {
        self.metrics = metrics;
    }

    pub fn add_metrics(&mut self, cpu: f64, memory: f64) {
        self.metrics.add_cpu_point(cpu);
        self.metrics.add_memory_point(memory);
    }

    fn render_cpu_chart(&self, frame: &mut Frame, area: Rect, theme: &ThemeConfig) {
        if self.metrics.cpu_history.is_empty() {
            return;
        }

        let data: Vec<u64> = self
            .metrics
            .cpu_history
            .iter()
            .map(|point| point.value as u64)
            .collect();

        let current_cpu = self
            .metrics
            .cpu_history
            .last()
            .map(|p| p.value)
            .unwrap_or(0.0);

        let sparkline = Sparkline::default()
            .block(
                Block::default()
                    .title(format!("CPU Usage: {current_cpu:.1}%"))
                    .borders(Borders::ALL),
            )
            .data(&data)
            .style(Style::default().fg(theme.colors.primary));

        frame.render_widget(sparkline, area);
    }

    fn render_memory_chart(&self, frame: &mut Frame, area: Rect, theme: &ThemeConfig) {
        if self.metrics.memory_history.is_empty() {
            return;
        }

        let data: Vec<u64> = self
            .metrics
            .memory_history
            .iter()
            .map(|point| point.value as u64)
            .collect();

        let current_mem = self
            .metrics
            .memory_history
            .last()
            .map(|p| p.value)
            .unwrap_or(0.0);

        let sparkline = Sparkline::default()
            .block(
                Block::default()
                    .title(format!("Memory Usage: {current_mem:.1}%"))
                    .borders(Borders::ALL),
            )
            .data(&data)
            .style(Style::default().fg(theme.colors.accent));

        frame.render_widget(sparkline, area);
    }

    fn render_stats(&self, frame: &mut Frame, area: Rect, theme: &ThemeConfig) {
        let cpu_avg = if !self.metrics.cpu_history.is_empty() {
            let sum: f64 = self.metrics.cpu_history.iter().map(|p| p.value).sum();
            sum / self.metrics.cpu_history.len() as f64
        } else {
            0.0
        };

        let mem_avg = if !self.metrics.memory_history.is_empty() {
            let sum: f64 = self.metrics.memory_history.iter().map(|p| p.value).sum();
            sum / self.metrics.memory_history.len() as f64
        } else {
            0.0
        };

        let stats_text = vec![
            Line::from(vec![
                Span::styled("CPU Avg: ", Style::default().fg(theme.colors.muted)),
                Span::styled(
                    format!("{cpu_avg:.1}%"),
                    Style::default().fg(theme.colors.primary),
                ),
            ]),
            Line::from(vec![
                Span::styled("Mem Avg: ", Style::default().fg(theme.colors.muted)),
                Span::styled(
                    format!("{mem_avg:.1}%"),
                    Style::default().fg(theme.colors.accent),
                ),
            ]),
            Line::from(vec![
                Span::styled("Points: ", Style::default().fg(theme.colors.muted)),
                Span::styled(
                    format!("{}", self.metrics.cpu_history.len()),
                    Style::default().fg(theme.colors.text),
                ),
            ]),
        ];

        let paragraph = Paragraph::new(stats_text);
        frame.render_widget(paragraph, area);
    }
}

impl Default for PerformanceChartsComponent {
    fn default() -> Self {
        Self::new()
    }
}

impl Component for PerformanceChartsComponent {
    fn update<'a>(
        &'a mut self,
        _app_state: &'a AppState,
    ) -> Pin<Box<dyn Future<Output = Result<()>> + Send + 'a>> {
        Box::pin(async move {
            // Metrics updated from dashboard view
            Ok(())
        })
    }

    fn render(&self, frame: &mut Frame, area: Rect, theme: &ThemeConfig, _app_state: &AppState) {
        let block = Block::default()
            .title("Performance Metrics")
            .borders(Borders::ALL)
            .border_style(if self.focused {
                theme.focused_border_style()
            } else {
                theme.normal_border_style()
            });

        let inner_area = block.inner(area);
        frame.render_widget(block, area);

        // Split into charts and stats
        let chunks = Layout::default()
            .direction(Direction::Horizontal)
            .constraints([Constraint::Percentage(75), Constraint::Percentage(25)])
            .split(inner_area);

        // Split charts area for CPU and Memory
        let chart_chunks = Layout::default()
            .direction(Direction::Vertical)
            .constraints([Constraint::Percentage(50), Constraint::Percentage(50)])
            .split(chunks[0]);

        self.render_cpu_chart(frame, chart_chunks[0], theme);
        self.render_memory_chart(frame, chart_chunks[1], theme);
        self.render_stats(frame, chunks[1], theme);
    }

    fn handle_event<'a>(
        &'a mut self,
        _event: ComponentEvent,
        _app_state: &'a AppState,
    ) -> Pin<Box<dyn Future<Output = Result<ViewAction>> + Send + 'a>> {
        Box::pin(async move { Ok(ViewAction::None) })
    }

    fn component_type(&self) -> ComponentType {
        ComponentType::PerformanceGraph
    }

    fn is_focusable(&self) -> bool {
        false
    }

    fn is_focused(&self) -> bool {
        self.focused
    }

    fn set_focus(&mut self, focused: bool) {
        self.focused = focused;
    }
}
