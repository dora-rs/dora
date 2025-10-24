/// Metrics Chart Component - reusable chart widget for displaying metrics

use ratatui::{
    layout::Rect,
    style::Style,
    widgets::{Block, Borders},
    Frame,
};
use std::collections::VecDeque;
use std::time::{Duration, Instant};

use crate::tui::{
    app::AppState,
    theme::ThemeConfig,
    views::ViewAction,
    Result,
};

use super::{Component, ComponentEvent, ComponentType};

/// Chart display types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChartType {
    CpuUsage,
    MemoryUsage,
    NetworkRx,
    NetworkTx,
    Custom(u32),
}

impl ChartType {
    pub fn display_name(&self) -> &str {
        match self {
            ChartType::CpuUsage => "CPU Usage",
            ChartType::MemoryUsage => "Memory Usage",
            ChartType::NetworkRx => "Network RX",
            ChartType::NetworkTx => "Network TX",
            ChartType::Custom(_) => "Custom Metric",
        }
    }

    pub fn unit(&self) -> &str {
        match self {
            ChartType::CpuUsage => "%",
            ChartType::MemoryUsage => "MB",
            ChartType::NetworkRx | ChartType::NetworkTx => "KB/s",
            ChartType::Custom(_) => "",
        }
    }
}

/// A single metric data point
#[derive(Debug, Clone)]
pub struct MetricPoint {
    pub timestamp: Instant,
    pub value: f64,
}

/// Reusable metrics chart component
pub struct MetricsChartComponent {
    metrics_data: VecDeque<MetricPoint>,
    chart_type: ChartType,
    time_window: Duration,
    focused: bool,
    auto_scale: bool,
    y_axis_range: (f64, f64),
}

impl MetricsChartComponent {
    pub fn new(chart_type: ChartType, time_window: Duration) -> Self {
        Self {
            metrics_data: VecDeque::new(),
            chart_type,
            time_window,
            focused: false,
            auto_scale: true,
            y_axis_range: (0.0, 100.0),
        }
    }

    fn calculate_y_axis_range(&mut self) {
        if self.metrics_data.is_empty() {
            return;
        }

        let min = self.metrics_data.iter().map(|p| p.value).fold(f64::INFINITY, f64::min);
        let max = self.metrics_data.iter().map(|p| p.value).fold(f64::NEG_INFINITY, f64::max);

        // Add 10% padding
        let padding = (max - min) * 0.1;
        self.y_axis_range = ((min - padding).max(0.0), max + padding);
    }

    fn add_data_point(&mut self, value: f64) {
        let now = Instant::now();
        let cutoff_time = now - self.time_window;

        // Remove old data points
        while let Some(front) = self.metrics_data.front() {
            if front.timestamp < cutoff_time {
                self.metrics_data.pop_front();
            } else {
                break;
            }
        }

        // Add new point
        self.metrics_data.push_back(MetricPoint {
            timestamp: now,
            value,
        });

        // Auto-scale if enabled
        if self.auto_scale {
            self.calculate_y_axis_range();
        }
    }
}

impl Component for MetricsChartComponent {
    fn update<'a>(&'a mut self, app_state: &'a AppState)
        -> std::pin::Pin<Box<dyn std::future::Future<Output = Result<()>> + Send + 'a>> {
        Box::pin(async move {
        // Add new metric value based on chart type
        let value = match self.chart_type {
            ChartType::CpuUsage => app_state.system_metrics.cpu_usage as f64,
            ChartType::MemoryUsage => app_state.system_metrics.memory_usage as f64,
            ChartType::NetworkRx => app_state.system_metrics.network_io.0 as f64 / 1024.0,
            ChartType::NetworkTx => app_state.system_metrics.network_io.1 as f64 / 1024.0,
            ChartType::Custom(_) => 0.0,
        };

            self.add_data_point(value);

            Ok(())
        })
    }

    fn render(&self, frame: &mut Frame, area: Rect, theme: &ThemeConfig, _app_state: &AppState) {
        let title = format!("{} - Last {}s",
            self.chart_type.display_name(),
            self.time_window.as_secs()
        );

        let block = Block::default()
            .title(title)
            .borders(Borders::ALL)
            .border_style(if self.focused {
                Style::default().fg(theme.colors.primary)
            } else {
                Style::default().fg(theme.colors.border)
            });

        // For now, just render the block
        // Full chart implementation would use ratatui::widgets::Chart
        frame.render_widget(block, area);
    }

    fn handle_event<'a>(&'a mut self, event: ComponentEvent, _app_state: &'a AppState)
        -> std::pin::Pin<Box<dyn std::future::Future<Output = Result<ViewAction>> + Send + 'a>> {
        Box::pin(async move {
            if !self.focused {
                return Ok(ViewAction::None);
            }

            match event {
                ComponentEvent::Key(key_event) => {
                    match key_event.code {
                        crossterm::event::KeyCode::Char('a') => {
                            self.auto_scale = !self.auto_scale;
                            return Ok(ViewAction::ShowStatus(
                                format!("Auto-scale: {}", if self.auto_scale { "ON" } else { "OFF" })
                            ));
                        },
                        _ => {}
                    }
                },
                _ => {}
            }

            Ok(ViewAction::None)
        })
    }

    fn component_type(&self) -> ComponentType {
        ComponentType::MetricsChart
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
