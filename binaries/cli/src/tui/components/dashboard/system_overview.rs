use ratatui::{
    Frame,
    layout::{Constraint, Direction, Layout, Rect},
    style::{Modifier, Style},
    text::{Line, Span},
    widgets::{Block, Borders, Gauge, Paragraph},
};
use std::future::Future;
/// System Overview Component for Dashboard (Issue #24)
use std::pin::Pin;

use crate::tui::{
    Result,
    app::AppState,
    components::{Component, ComponentEvent, ComponentType},
    theme::ThemeConfig,
    views::{SystemOverview, SystemStatus, ViewAction},
};

pub struct SystemOverviewComponent {
    system_data: Option<SystemOverview>,
    focused: bool,
}

impl SystemOverviewComponent {
    pub fn new() -> Self {
        Self {
            system_data: None,
            focused: false,
        }
    }

    pub fn set_data(&mut self, data: SystemOverview) {
        self.system_data = Some(data);
    }

    fn render_system_status(&self, frame: &mut Frame, area: Rect, theme: &ThemeConfig) {
        if let Some(system) = &self.system_data {
            let status_color = match system.status {
                SystemStatus::Connected => theme.colors.success,
                SystemStatus::Disconnected => theme.colors.error,
                SystemStatus::Connecting => theme.colors.warning,
            };

            let status_text = vec![
                Line::from(vec![
                    Span::styled("Status: ", Style::default().fg(theme.colors.text)),
                    Span::styled(
                        format!("● {}", system.status),
                        Style::default()
                            .fg(status_color)
                            .add_modifier(Modifier::BOLD),
                    ),
                ]),
                Line::from(vec![
                    Span::styled("Uptime: ", Style::default().fg(theme.colors.text)),
                    Span::styled(
                        self.format_duration(system.uptime),
                        Style::default().fg(theme.colors.primary),
                    ),
                ]),
                Line::from(vec![
                    Span::styled("Version: ", Style::default().fg(theme.colors.text)),
                    Span::styled(&system.version, Style::default().fg(theme.colors.muted)),
                ]),
            ];

            let status_paragraph = Paragraph::new(status_text).style(Style::default());

            frame.render_widget(status_paragraph, area);
        }
    }

    fn render_resource_gauges(&self, frame: &mut Frame, area: Rect, theme: &ThemeConfig) {
        if let Some(system) = &self.system_data {
            let chunks = Layout::default()
                .direction(Direction::Vertical)
                .constraints([
                    Constraint::Length(3),
                    Constraint::Length(3),
                    Constraint::Length(3),
                ])
                .split(area);

            // CPU gauge
            let cpu_gauge = Gauge::default()
                .block(Block::default().title("CPU").borders(Borders::ALL))
                .gauge_style(theme.percentage_style(system.cpu_usage as f32))
                .percent(system.cpu_usage.min(100.0) as u16)
                .label(format!("{:.1}%", system.cpu_usage));

            frame.render_widget(cpu_gauge, chunks[0]);

            // Memory gauge
            let memory_gauge = Gauge::default()
                .block(Block::default().title("Memory").borders(Borders::ALL))
                .gauge_style(theme.percentage_style(system.memory_usage.usage_percent as f32))
                .percent(system.memory_usage.usage_percent.min(100.0) as u16)
                .label(format!(
                    "{:.1}% ({} / {} MB)",
                    system.memory_usage.usage_percent,
                    system.memory_usage.used_mb,
                    system.memory_usage.total_mb
                ));

            frame.render_widget(memory_gauge, chunks[1]);

            // Disk gauge
            let disk_gauge = Gauge::default()
                .block(Block::default().title("Disk").borders(Borders::ALL))
                .gauge_style(theme.percentage_style(system.disk_usage.usage_percent as f32))
                .percent(system.disk_usage.usage_percent.min(100.0) as u16)
                .label(format!(
                    "{:.1}% ({} / {} GB)",
                    system.disk_usage.usage_percent,
                    system.disk_usage.used_gb,
                    system.disk_usage.total_gb
                ));

            frame.render_widget(disk_gauge, chunks[2]);
        }
    }

    fn format_duration(&self, duration: std::time::Duration) -> String {
        let total_secs = duration.as_secs();
        let days = total_secs / 86400;
        let hours = (total_secs % 86400) / 3600;
        let minutes = (total_secs % 3600) / 60;

        if days > 0 {
            format!("{days}d {hours}h")
        } else if hours > 0 {
            format!("{hours}h {minutes}m")
        } else {
            format!("{minutes}m")
        }
    }
}

impl Default for SystemOverviewComponent {
    fn default() -> Self {
        Self::new()
    }
}

impl Component for SystemOverviewComponent {
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
            .title("System Overview")
            .borders(Borders::ALL)
            .border_style(if self.focused {
                theme.focused_border_style()
            } else {
                theme.normal_border_style()
            });

        let inner_area = block.inner(area);
        frame.render_widget(block, area);

        // Split area for status and gauges
        let chunks = Layout::default()
            .direction(Direction::Vertical)
            .constraints([Constraint::Length(4), Constraint::Min(9)])
            .split(inner_area);

        self.render_system_status(frame, chunks[0], theme);
        self.render_resource_gauges(frame, chunks[1], theme);
    }

    fn handle_event<'a>(
        &'a mut self,
        _event: ComponentEvent,
        _app_state: &'a AppState,
    ) -> Pin<Box<dyn Future<Output = Result<ViewAction>> + Send + 'a>> {
        Box::pin(async move { Ok(ViewAction::None) })
    }

    fn component_type(&self) -> ComponentType {
        ComponentType::PropertyInspector // Using existing type for now
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
