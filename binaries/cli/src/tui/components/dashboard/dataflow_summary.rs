use ratatui::{
    Frame,
    layout::{Constraint, Direction, Layout, Rect},
    style::{Modifier, Style},
    text::{Line, Span},
    widgets::{Block, Borders, Gauge, Paragraph},
};
use std::future::Future;
/// Dataflow Summary Component for Dashboard (Issue #24)
use std::pin::Pin;

use crate::tui::{
    Result,
    app::{AppState, ViewType},
    components::{Component, ComponentEvent, ComponentType},
    theme::ThemeConfig,
    views::{DataflowSummary, ViewAction},
};
use crossterm::event::KeyCode;

pub struct DataflowSummaryComponent {
    dataflow_data: Option<DataflowSummary>,
    focused: bool,
}

impl DataflowSummaryComponent {
    pub fn new() -> Self {
        Self {
            dataflow_data: None,
            focused: false,
        }
    }

    pub fn set_data(&mut self, data: DataflowSummary) {
        self.dataflow_data = Some(data);
    }

    fn render_dataflow_stats(&self, frame: &mut Frame, area: Rect, theme: &ThemeConfig) {
        if let Some(data) = &self.dataflow_data {
            let stats_text = vec![
                Line::from(vec![
                    Span::styled("Total Dataflows: ", Style::default().fg(theme.colors.text)),
                    Span::styled(
                        data.total_dataflows.to_string(),
                        Style::default()
                            .fg(theme.colors.primary)
                            .add_modifier(Modifier::BOLD),
                    ),
                ]),
                Line::from(vec![
                    Span::styled("  Running: ", Style::default().fg(theme.colors.muted)),
                    Span::styled(
                        data.running_dataflows.to_string(),
                        Style::default().fg(theme.colors.success),
                    ),
                ]),
                Line::from(vec![
                    Span::styled("  Failed: ", Style::default().fg(theme.colors.muted)),
                    Span::styled(
                        data.failed_dataflows.to_string(),
                        Style::default().fg(theme.colors.error),
                    ),
                ]),
                Line::from(vec![
                    Span::styled("  Stopped: ", Style::default().fg(theme.colors.muted)),
                    Span::styled(
                        data.stopped_dataflows.to_string(),
                        Style::default().fg(theme.colors.warning),
                    ),
                ]),
                Line::from(""),
                Line::from(vec![
                    Span::styled("Total Nodes: ", Style::default().fg(theme.colors.text)),
                    Span::styled(
                        data.total_nodes.to_string(),
                        Style::default()
                            .fg(theme.colors.primary)
                            .add_modifier(Modifier::BOLD),
                    ),
                ]),
                Line::from(vec![
                    Span::styled("  Healthy: ", Style::default().fg(theme.colors.muted)),
                    Span::styled(
                        data.healthy_nodes.to_string(),
                        Style::default().fg(theme.colors.success),
                    ),
                ]),
                Line::from(vec![
                    Span::styled("  Unhealthy: ", Style::default().fg(theme.colors.muted)),
                    Span::styled(
                        data.unhealthy_nodes.to_string(),
                        Style::default().fg(theme.colors.error),
                    ),
                ]),
            ];

            let paragraph = Paragraph::new(stats_text);
            frame.render_widget(paragraph, area);
        }
    }

    fn render_health_indicators(&self, frame: &mut Frame, area: Rect, theme: &ThemeConfig) {
        if let Some(data) = &self.dataflow_data {
            let chunks = Layout::default()
                .direction(Direction::Vertical)
                .constraints([Constraint::Length(3), Constraint::Length(3)])
                .split(area);

            // Dataflow health gauge
            let dataflow_health = data.health_percentage();
            let dataflow_gauge = Gauge::default()
                .block(
                    Block::default()
                        .title("Dataflow Health")
                        .borders(Borders::ALL),
                )
                .gauge_style(theme.percentage_style(dataflow_health as f32))
                .percent(dataflow_health.min(100.0) as u16)
                .label(format!("{dataflow_health:.0}%"));

            frame.render_widget(dataflow_gauge, chunks[0]);

            // Node health gauge
            let node_health = data.node_health_percentage();
            let node_gauge = Gauge::default()
                .block(Block::default().title("Node Health").borders(Borders::ALL))
                .gauge_style(theme.percentage_style(node_health as f32))
                .percent(node_health.min(100.0) as u16)
                .label(format!("{node_health:.0}%"));

            frame.render_widget(node_gauge, chunks[1]);
        }
    }
}

impl Default for DataflowSummaryComponent {
    fn default() -> Self {
        Self::new()
    }
}

impl Component for DataflowSummaryComponent {
    fn update<'a>(
        &'a mut self,
        _app_state: &'a AppState,
    ) -> Pin<Box<dyn Future<Output = Result<()>> + Send + 'a>> {
        Box::pin(async move {
            // Data is set from dashboard view
            Ok(())
        })
    }

    fn render(&self, frame: &mut Frame, area: Rect, theme: &ThemeConfig, _app_state: &AppState) {
        let block = Block::default()
            .title("Dataflow Summary")
            .borders(Borders::ALL)
            .border_style(if self.focused {
                theme.focused_border_style()
            } else {
                theme.normal_border_style()
            });

        let inner_area = block.inner(area);
        frame.render_widget(block, area);

        let chunks = Layout::default()
            .direction(Direction::Vertical)
            .constraints([Constraint::Min(8), Constraint::Length(6)])
            .split(inner_area);

        self.render_dataflow_stats(frame, chunks[0], theme);
        self.render_health_indicators(frame, chunks[1], theme);
    }

    fn handle_event<'a>(
        &'a mut self,
        event: ComponentEvent,
        _app_state: &'a AppState,
    ) -> Pin<Box<dyn Future<Output = Result<ViewAction>> + Send + 'a>> {
        Box::pin(async move {
            // Enter key navigates to dataflow list
            if let ComponentEvent::Key(key_event) = event {
                if key_event.code == KeyCode::Enter {
                    return Ok(ViewAction::PushView(ViewType::DataflowManager));
                }
            }

            Ok(ViewAction::None)
        })
    }

    fn component_type(&self) -> ComponentType {
        ComponentType::PropertyInspector // Using existing type for now
    }

    fn is_focusable(&self) -> bool {
        true
    }

    fn is_focused(&self) -> bool {
        self.focused
    }

    fn set_focus(&mut self, focused: bool) {
        self.focused = focused;
    }
}
