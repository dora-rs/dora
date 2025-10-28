// Performance Optimization View - Interactive performance analysis and optimization
// Issue #36: Performance Optimization Suite

use super::{BaseView, View, ViewAction};
use crate::tui::{Result, app::AppState, theme::ThemeConfig, views::perf_optimization_types::*};
use crossterm::event::{KeyCode, KeyEvent};
use ratatui::{
    Frame,
    layout::{Alignment, Constraint, Direction, Layout, Rect},
    style::{Modifier, Style},
    text::{Line, Span},
    widgets::{Block, Borders, List, ListItem, Paragraph},
};

/// Performance Optimization View
pub struct PerfOptimizationView {
    base: BaseView,
    pub state: PerfOptimizationState,
    theme: ThemeConfig,
}

impl PerfOptimizationView {
    pub fn new() -> Self {
        Self {
            base: BaseView::new("Performance Optimization".to_string()),
            state: PerfOptimizationState::new(),
            theme: ThemeConfig::default(),
        }
    }

    fn render_header(&self, frame: &mut Frame, area: Rect) {
        let sections = PerfSection::all();
        let tabs: Vec<Span> = sections
            .iter()
            .enumerate()
            .flat_map(|(i, section)| {
                let is_selected = section == &self.state.current_section;
                let style = if is_selected {
                    Style::default()
                        .fg(self.theme.colors.accent)
                        .add_modifier(Modifier::BOLD)
                } else {
                    Style::default().fg(self.theme.colors.muted)
                };

                let mut spans = vec![Span::styled(section.title(), style)];
                if i < sections.len() - 1 {
                    spans.push(Span::raw(" │ "));
                }
                spans
            })
            .collect();

        let header = Paragraph::new(Line::from(tabs))
            .block(Block::default().borders(Borders::BOTTOM))
            .alignment(Alignment::Center);

        frame.render_widget(header, area);
    }

    fn render_metrics(&self, frame: &mut Frame, area: Rect, metrics: &[PerformanceMetric]) {
        let items: Vec<ListItem> = metrics
            .iter()
            .enumerate()
            .map(|(i, metric)| {
                let is_selected = i == self.state.selected_index;
                let style = if is_selected {
                    Style::default()
                        .fg(self.theme.colors.accent)
                        .add_modifier(Modifier::BOLD)
                } else {
                    Style::default().fg(self.theme.colors.text)
                };

                let status_style = match metric.status {
                    MetricStatus::Healthy => Style::default().fg(self.theme.colors.success),
                    MetricStatus::Warning => Style::default().fg(self.theme.colors.warning),
                    MetricStatus::Critical => Style::default().fg(self.theme.colors.error),
                    MetricStatus::Unknown => Style::default().fg(self.theme.colors.muted),
                };

                let status_symbol = match metric.status {
                    MetricStatus::Healthy => "✓",
                    MetricStatus::Warning => "⚠",
                    MetricStatus::Critical => "✗",
                    MetricStatus::Unknown => "?",
                };

                let content = vec![
                    Line::from(vec![
                        Span::styled(status_symbol, status_style),
                        Span::raw(" "),
                        Span::styled(&metric.name, style),
                        Span::styled(
                            format!(" [{}]", metric.metric_type.name()),
                            Style::default().fg(self.theme.colors.muted),
                        ),
                    ]),
                    Line::from(vec![
                        Span::raw("  Current: "),
                        Span::styled(
                            format!("{:.1} {}", metric.current_value, metric.unit),
                            Style::default().fg(self.theme.colors.accent),
                        ),
                        Span::raw("  Avg: "),
                        Span::styled(
                            format!("{:.1}", metric.avg_value),
                            Style::default().fg(self.theme.colors.muted),
                        ),
                        Span::raw("  Range: "),
                        Span::styled(
                            format!("{:.1}-{:.1}", metric.min_value, metric.max_value),
                            Style::default().fg(self.theme.colors.muted),
                        ),
                    ]),
                    Line::from(""),
                ];

                ListItem::new(content)
            })
            .collect();

        let list = List::new(items).block(
            Block::default()
                .borders(Borders::ALL)
                .title(format!("Performance Metrics ({} monitored)", metrics.len()))
                .border_style(Style::default().fg(self.theme.colors.border)),
        );

        frame.render_widget(list, area);
    }

    fn render_bottlenecks(&self, frame: &mut Frame, area: Rect, bottlenecks: &[Bottleneck]) {
        let items: Vec<ListItem> = bottlenecks
            .iter()
            .enumerate()
            .map(|(i, bottleneck)| {
                let is_selected = i == self.state.selected_index;
                let style = if is_selected {
                    Style::default()
                        .fg(self.theme.colors.accent)
                        .add_modifier(Modifier::BOLD)
                } else {
                    Style::default().fg(self.theme.colors.text)
                };

                let severity_style = match bottleneck.severity {
                    Severity::Low => Style::default().fg(self.theme.colors.success),
                    Severity::Medium => Style::default().fg(self.theme.colors.warning),
                    Severity::High => Style::default().fg(self.theme.colors.error),
                    Severity::Critical => Style::default()
                        .fg(self.theme.colors.error)
                        .add_modifier(Modifier::BOLD),
                };

                let severity_symbol = "●".repeat(bottleneck.severity.priority() as usize);

                let content = vec![
                    Line::from(vec![
                        Span::styled(severity_symbol, severity_style),
                        Span::raw(" "),
                        Span::styled(bottleneck.location.display(), style),
                        Span::styled(
                            format!(" [{}]", bottleneck.bottleneck_type.name()),
                            Style::default().fg(self.theme.colors.muted),
                        ),
                    ]),
                    Line::from(vec![
                        Span::raw("  Impact: "),
                        Span::styled(
                            &bottleneck.impact,
                            Style::default().fg(self.theme.colors.muted),
                        ),
                    ]),
                    Line::from(vec![Span::styled(
                        format!(
                            "  Severity: {} | Affected: {}",
                            bottleneck.severity.name(),
                            bottleneck.affected_components.join(", ")
                        ),
                        Style::default().fg(self.theme.colors.muted),
                    )]),
                    Line::from(""),
                ];

                ListItem::new(content)
            })
            .collect();

        let list = List::new(items).block(
            Block::default()
                .borders(Borders::ALL)
                .title(format!(
                    "Detected Bottlenecks ({} found)",
                    bottlenecks.len()
                ))
                .border_style(Style::default().fg(self.theme.colors.border)),
        );

        frame.render_widget(list, area);
    }

    fn render_suggestions(
        &self,
        frame: &mut Frame,
        area: Rect,
        suggestions: &[OptimizationSuggestion],
    ) {
        let items: Vec<ListItem> = suggestions
            .iter()
            .enumerate()
            .map(|(i, suggestion)| {
                let is_selected = i == self.state.selected_index;
                let style = if is_selected {
                    Style::default()
                        .fg(self.theme.colors.accent)
                        .add_modifier(Modifier::BOLD)
                } else {
                    Style::default().fg(self.theme.colors.text)
                };

                let priority_style = match suggestion.priority {
                    Priority::Low => Style::default().fg(self.theme.colors.muted),
                    Priority::Medium => Style::default().fg(self.theme.colors.warning),
                    Priority::High => Style::default().fg(self.theme.colors.error),
                    Priority::Critical => Style::default()
                        .fg(self.theme.colors.error)
                        .add_modifier(Modifier::BOLD),
                };

                let impact_bar = "█".repeat(
                    (match suggestion.expected_impact {
                        ImpactLevel::Minimal => 1,
                        ImpactLevel::Moderate => 2,
                        ImpactLevel::Significant => 3,
                        ImpactLevel::Major => 4,
                    }) as usize,
                );

                let content = vec![
                    Line::from(vec![Span::styled(&suggestion.title, style)]),
                    Line::from(vec![
                        Span::raw("  Category: "),
                        Span::styled(
                            suggestion.category.name(),
                            Style::default().fg(self.theme.colors.muted),
                        ),
                        Span::raw("  Priority: "),
                        Span::styled(suggestion.priority.name(), priority_style),
                    ]),
                    Line::from(vec![
                        Span::raw("  Impact: "),
                        Span::styled(impact_bar, Style::default().fg(self.theme.colors.success)),
                        Span::raw(" "),
                        Span::styled(
                            suggestion.expected_impact.name(),
                            Style::default().fg(self.theme.colors.muted),
                        ),
                        Span::raw("  Effort: "),
                        Span::styled(
                            suggestion.estimated_effort.name(),
                            Style::default().fg(self.theme.colors.muted),
                        ),
                    ]),
                    Line::from(vec![
                        Span::raw("  "),
                        Span::styled(
                            &suggestion.description,
                            Style::default().fg(self.theme.colors.muted),
                        ),
                    ]),
                    Line::from(""),
                ];

                ListItem::new(content)
            })
            .collect();

        let list = List::new(items).block(
            Block::default()
                .borders(Borders::ALL)
                .title(format!(
                    "Optimization Suggestions ({} recommendations)",
                    suggestions.len()
                ))
                .border_style(Style::default().fg(self.theme.colors.border)),
        );

        frame.render_widget(list, area);
    }

    fn render_resources(&self, frame: &mut Frame, area: Rect, resources: &[ResourceUsage]) {
        let items: Vec<ListItem> = resources
            .iter()
            .enumerate()
            .map(|(i, resource)| {
                let is_selected = i == self.state.selected_index;
                let style = if is_selected {
                    Style::default()
                        .fg(self.theme.colors.accent)
                        .add_modifier(Modifier::BOLD)
                } else {
                    Style::default().fg(self.theme.colors.text)
                };

                let cpu_style = if resource.cpu_percent > 80.0 {
                    Style::default().fg(self.theme.colors.error)
                } else if resource.cpu_percent > 60.0 {
                    Style::default().fg(self.theme.colors.warning)
                } else {
                    Style::default().fg(self.theme.colors.success)
                };

                let cpu_bar = "█".repeat((resource.cpu_percent / 10.0) as usize);

                let content = vec![
                    Line::from(vec![
                        Span::styled(&resource.component, style),
                        Span::styled(
                            format!(" ({} threads)", resource.thread_count),
                            Style::default().fg(self.theme.colors.muted),
                        ),
                    ]),
                    Line::from(vec![
                        Span::raw("  CPU: "),
                        Span::styled(cpu_bar, cpu_style),
                        Span::raw(" "),
                        Span::styled(format!("{:.1}%", resource.cpu_percent), cpu_style),
                        Span::raw("  Mem: "),
                        Span::styled(
                            format!("{:.0} MB", resource.memory_mb),
                            Style::default().fg(self.theme.colors.muted),
                        ),
                    ]),
                    Line::from(vec![
                        Span::raw("  Net: "),
                        Span::styled(
                            format!(
                                "↓{:.1} ↑{:.1} MB/s",
                                resource.network_rx_mbps, resource.network_tx_mbps
                            ),
                            Style::default().fg(self.theme.colors.muted),
                        ),
                        Span::raw("  Disk: "),
                        Span::styled(
                            format!(
                                "R{:.1} W{:.1} MB/s",
                                resource.disk_read_mbps, resource.disk_write_mbps
                            ),
                            Style::default().fg(self.theme.colors.muted),
                        ),
                    ]),
                    Line::from(""),
                ];

                ListItem::new(content)
            })
            .collect();

        let list = List::new(items).block(
            Block::default()
                .borders(Borders::ALL)
                .title(format!("Resource Usage ({} components)", resources.len()))
                .border_style(Style::default().fg(self.theme.colors.border)),
        );

        frame.render_widget(list, area);
    }

    fn render_help(&self, frame: &mut Frame, area: Rect) {
        let help_text = vec![Line::from(vec![
            Span::styled(
                "Tab/Shift+Tab",
                Style::default().fg(self.theme.colors.accent),
            ),
            Span::raw(": Switch section  "),
            Span::styled("↑/↓ or j/k", Style::default().fg(self.theme.colors.accent)),
            Span::raw(": Navigate  "),
            Span::styled("r", Style::default().fg(self.theme.colors.accent)),
            Span::raw(": Refresh  "),
            Span::styled("q", Style::default().fg(self.theme.colors.accent)),
            Span::raw(": Back"),
        ])];

        let help = Paragraph::new(help_text)
            .block(Block::default().borders(Borders::TOP))
            .alignment(Alignment::Center);

        frame.render_widget(help, area);
    }
}

impl Default for PerfOptimizationView {
    fn default() -> Self {
        Self::new()
    }
}

impl View for PerfOptimizationView {
    fn render(&mut self, frame: &mut Frame, area: Rect, _app_state: &AppState) {
        let chunks = Layout::default()
            .direction(Direction::Vertical)
            .constraints([
                Constraint::Length(3), // Header
                Constraint::Min(10),   // Content
                Constraint::Length(3), // Help
            ])
            .split(area);

        self.render_header(frame, chunks[0]);

        match &self.state.data {
            PerfData::Metrics(metrics) => {
                self.render_metrics(frame, chunks[1], metrics);
            }
            PerfData::Bottlenecks(bottlenecks) => {
                self.render_bottlenecks(frame, chunks[1], bottlenecks);
            }
            PerfData::Suggestions(suggestions) => {
                self.render_suggestions(frame, chunks[1], suggestions);
            }
            PerfData::Resources(resources) => {
                self.render_resources(frame, chunks[1], resources);
            }
        }

        self.render_help(frame, chunks[2]);
    }

    async fn handle_key(&mut self, key: KeyEvent, _app_state: &mut AppState) -> Result<ViewAction> {
        match key.code {
            KeyCode::Char('q') => Ok(ViewAction::PopView),
            KeyCode::Tab => {
                self.state.next_section();
                Ok(ViewAction::None)
            }
            KeyCode::BackTab => {
                self.state.previous_section();
                Ok(ViewAction::None)
            }
            KeyCode::Up | KeyCode::Char('k') => {
                self.state.previous_item();
                Ok(ViewAction::None)
            }
            KeyCode::Down | KeyCode::Char('j') => {
                self.state.next_item();
                Ok(ViewAction::None)
            }
            KeyCode::Char('r') => {
                // Refresh data
                self.state.update_data();
                Ok(ViewAction::None)
            }
            _ => Ok(ViewAction::None),
        }
    }

    async fn update(&mut self, _app_state: &mut AppState) -> Result<()> {
        Ok(())
    }

    fn help_text(&self) -> Vec<(&str, &str)> {
        vec![
            ("Tab/Shift+Tab", "Switch section"),
            ("↑/↓, j/k", "Navigate items"),
            ("r", "Refresh data"),
            ("q", "Back to previous view"),
        ]
    }

    fn title(&self) -> &str {
        &self.base.title
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_perf_optimization_view_creation() {
        let view = PerfOptimizationView::new();
        assert_eq!(view.state.current_section, PerfSection::PerformanceMetrics);
        assert_eq!(view.state.selected_index, 0);
    }

    #[test]
    fn test_perf_optimization_view_default() {
        let view = PerfOptimizationView::default();
        assert_eq!(view.state.current_section, PerfSection::PerformanceMetrics);
    }

    #[test]
    fn test_view_title() {
        let view = PerfOptimizationView::new();
        assert_eq!(view.title(), "Performance Optimization");
    }

    #[test]
    fn test_view_help_text() {
        let view = PerfOptimizationView::new();
        let help = view.help_text();
        assert!(!help.is_empty());
        assert!(help.iter().any(|(key, _)| key.contains("Tab")));
        assert!(help.iter().any(|(key, _)| key.contains("q")));
    }

    #[tokio::test]
    async fn test_navigation_keys() {
        let mut view = PerfOptimizationView::new();
        let mut app_state = AppState::default();

        // Tab to next section
        let result = view
            .handle_key(KeyEvent::from(KeyCode::Tab), &mut app_state)
            .await;
        assert!(result.is_ok());
        assert_eq!(view.state.current_section, PerfSection::BottleneckDetection);

        // BackTab to previous section
        let result = view
            .handle_key(KeyEvent::from(KeyCode::BackTab), &mut app_state)
            .await;
        assert!(result.is_ok());
        assert_eq!(view.state.current_section, PerfSection::PerformanceMetrics);
    }

    #[tokio::test]
    async fn test_quit_key() {
        let mut view = PerfOptimizationView::new();
        let mut app_state = AppState::default();

        let result = view
            .handle_key(KeyEvent::from(KeyCode::Char('q')), &mut app_state)
            .await;

        assert!(result.is_ok());
        assert!(matches!(result.unwrap(), ViewAction::PopView));
    }
}
