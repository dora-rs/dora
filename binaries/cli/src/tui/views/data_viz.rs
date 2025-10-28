// Data Visualization View - Phase 1: Simplified Implementation with Mock Data
// TODO(Issue #32 Phase 2): Add real-time data streaming and advanced interactive features

use super::data_viz_types::{
    CategoryData, ChartData, ChartType, DataVizState, EventData, GaugeData, TimeSeriesData,
};
use super::{BaseView, View, ViewAction};
use crate::tui::{Result, app::AppState, theme::ThemeConfig};
use crossterm::event::{KeyCode, KeyEvent, KeyModifiers};
use ratatui::{
    Frame,
    layout::{Constraint, Direction, Layout, Rect},
    style::{Modifier, Style},
    text::{Line, Span},
    widgets::{
        Block, Borders, List, ListItem, Paragraph,
        canvas::{Canvas, Line as CanvasLine, Points},
    },
};
use std::time::Duration;

pub struct DataVizView {
    base: BaseView,
    pub state: DataVizState,
    theme: ThemeConfig,
}

impl DataVizView {
    pub fn new(theme: &ThemeConfig) -> Self {
        let base = BaseView::new("Data Visualization".to_string())
            .with_auto_refresh(Duration::from_millis(500));

        Self {
            base,
            state: DataVizState::new(),
            theme: theme.clone(),
        }
    }

    fn render_header(&self, frame: &mut Frame, area: Rect) {
        let header_text = vec![
            Line::from(vec![
                Span::styled("Chart Type: ", Style::default()),
                Span::styled(
                    self.state.current_chart_type.name(),
                    Style::default().add_modifier(Modifier::BOLD),
                ),
            ]),
            Line::from(self.state.current_chart_type.description()),
            Line::from(""),
            Line::from(vec![
                Span::raw("Zoom: "),
                Span::styled(
                    format!("{:.1}x", self.state.zoom_level),
                    Style::default().add_modifier(Modifier::BOLD),
                ),
            ]),
        ];

        let header = Paragraph::new(header_text).block(
            Block::default()
                .borders(Borders::ALL)
                .border_style(self.theme.normal_border_style())
                .title(" Info "),
        );

        frame.render_widget(header, area);
    }

    fn render_shortcuts(&self, frame: &mut Frame, area: Rect) {
        let shortcuts = vec![
            "Tab/Shift+Tab: Switch chart",
            "+/-: Zoom in/out",
            "0: Reset zoom",
            "r: Refresh data",
            "↑/↓: Scroll",
            "q: Quit",
        ];

        let items: Vec<ListItem> = shortcuts
            .iter()
            .map(|s| ListItem::new(Line::from(*s)))
            .collect();

        let list = List::new(items).block(
            Block::default()
                .borders(Borders::ALL)
                .border_style(self.theme.normal_border_style())
                .title(" Shortcuts "),
        );

        frame.render_widget(list, area);
    }

    fn render_line_chart(&self, frame: &mut Frame, area: Rect, series_list: &[TimeSeriesData]) {
        if series_list.is_empty() {
            return;
        }

        // Calculate bounds across all series
        let min_x = series_list
            .iter()
            .map(|s| s.min_x())
            .min_by(|a, b| a.partial_cmp(b).unwrap())
            .unwrap_or(0.0);
        let max_x = series_list
            .iter()
            .map(|s| s.max_x())
            .max_by(|a, b| a.partial_cmp(b).unwrap())
            .unwrap_or(100.0);
        let min_y = series_list
            .iter()
            .map(|s| s.min_y())
            .min_by(|a, b| a.partial_cmp(b).unwrap())
            .unwrap_or(0.0);
        let max_y = series_list
            .iter()
            .map(|s| s.max_y())
            .max_by(|a, b| a.partial_cmp(b).unwrap())
            .unwrap_or(100.0);

        // Apply zoom
        let x_range = max_x - min_x;
        let y_range = max_y - min_y;
        let zoomed_x_range = x_range / self.state.zoom_level;
        let zoomed_y_range = y_range / self.state.zoom_level;

        let canvas = Canvas::default()
            .block(
                Block::default()
                    .borders(Borders::ALL)
                    .border_style(self.theme.focused_border_style())
                    .title(format!(" {} ", self.state.current_chart_type.name())),
            )
            .x_bounds([min_x, min_x + zoomed_x_range])
            .y_bounds([min_y, min_y + zoomed_y_range])
            .paint(|ctx| {
                let colors = [
                    ratatui::style::Color::Cyan,
                    ratatui::style::Color::Yellow,
                    ratatui::style::Color::Green,
                    ratatui::style::Color::Magenta,
                ];

                for series in series_list {
                    let color = colors[series.color as usize % colors.len()];

                    // Draw lines connecting points
                    for i in 0..series.points.len().saturating_sub(1) {
                        let p1 = &series.points[i];
                        let p2 = &series.points[i + 1];

                        ctx.draw(&CanvasLine {
                            x1: p1.x,
                            y1: p1.y,
                            x2: p2.x,
                            y2: p2.y,
                            color,
                        });
                    }

                    // Draw points
                    let points: Vec<(f64, f64)> =
                        series.points.iter().map(|p| (p.x, p.y)).collect();
                    ctx.draw(&Points {
                        coords: &points,
                        color,
                    });
                }
            });

        frame.render_widget(canvas, area);
    }

    fn render_bar_chart(&self, frame: &mut Frame, area: Rect, categories: &[CategoryData]) {
        if categories.is_empty() {
            return;
        }

        let max_value = categories
            .iter()
            .map(|c| c.value)
            .max_by(|a, b| a.partial_cmp(b).unwrap())
            .unwrap_or(100.0);

        let canvas = Canvas::default()
            .block(
                Block::default()
                    .borders(Borders::ALL)
                    .border_style(self.theme.focused_border_style())
                    .title(format!(" {} ", self.state.current_chart_type.name())),
            )
            .x_bounds([0.0, (categories.len() * 10) as f64])
            .y_bounds([0.0, max_value * 1.1])
            .paint(|ctx| {
                let colors = [
                    ratatui::style::Color::Cyan,
                    ratatui::style::Color::Yellow,
                    ratatui::style::Color::Green,
                    ratatui::style::Color::Magenta,
                    ratatui::style::Color::Red,
                ];

                for (i, category) in categories.iter().enumerate() {
                    let color = colors[category.color as usize % colors.len()];
                    let x = (i * 10 + 2) as f64;
                    let width = 6.0;

                    // Draw bar as series of horizontal lines
                    let steps = (category.value / 2.0).ceil() as i32;
                    for step in 0..steps {
                        let y = step as f64 * 2.0;
                        ctx.draw(&CanvasLine {
                            x1: x,
                            y1: y,
                            x2: x + width,
                            y2: y,
                            color,
                        });
                    }
                }
            });

        frame.render_widget(canvas, area);
    }

    fn render_scatter_plot(&self, frame: &mut Frame, area: Rect, series: &TimeSeriesData) {
        let min_x = series.min_x();
        let max_x = series.max_x();
        let min_y = series.min_y();
        let max_y = series.max_y();

        let canvas = Canvas::default()
            .block(
                Block::default()
                    .borders(Borders::ALL)
                    .border_style(self.theme.focused_border_style())
                    .title(format!(" {} ", self.state.current_chart_type.name())),
            )
            .x_bounds([min_x * 0.9, max_x * 1.1])
            .y_bounds([min_y * 0.9, max_y * 1.1])
            .paint(|ctx| {
                let points: Vec<(f64, f64)> = series.points.iter().map(|p| (p.x, p.y)).collect();
                ctx.draw(&Points {
                    coords: &points,
                    color: ratatui::style::Color::Cyan,
                });
            });

        frame.render_widget(canvas, area);
    }

    fn render_gauge(&self, frame: &mut Frame, area: Rect, gauge: &GaugeData) {
        let percentage = gauge.percentage();
        let bar_width = ((area.width as f64 - 4.0) * percentage / 100.0) as u16;

        let gauge_text = vec![
            Line::from(""),
            Line::from(vec![
                Span::raw("  "),
                Span::styled(
                    gauge.label.clone(),
                    Style::default().add_modifier(Modifier::BOLD),
                ),
            ]),
            Line::from(""),
            Line::from(vec![
                Span::raw("  Value: "),
                Span::styled(
                    format!("{:.1}{}", gauge.current_value, gauge.unit),
                    Style::default().add_modifier(Modifier::BOLD),
                ),
            ]),
            Line::from(""),
            Line::from(vec![
                Span::raw("  ["),
                Span::styled(
                    "█".repeat(bar_width as usize),
                    Style::default().fg(if gauge.is_at_target() {
                        ratatui::style::Color::Green
                    } else if percentage > 50.0 {
                        ratatui::style::Color::Yellow
                    } else {
                        ratatui::style::Color::Red
                    }),
                ),
                Span::raw(" ".repeat((area.width as usize).saturating_sub(bar_width as usize + 4))),
                Span::raw("]"),
            ]),
            Line::from(""),
            Line::from(vec![
                Span::raw("  Progress: "),
                Span::styled(
                    format!("{:.1}%", percentage),
                    Style::default().add_modifier(Modifier::BOLD),
                ),
            ]),
        ];

        if let Some(target) = gauge.target_value {
            let mut text_with_target = gauge_text;
            text_with_target.push(Line::from(vec![
                Span::raw("  Target: "),
                Span::styled(format!("{:.1}{}", target, gauge.unit), Style::default()),
            ]));

            let widget = Paragraph::new(text_with_target).block(
                Block::default()
                    .borders(Borders::ALL)
                    .border_style(self.theme.focused_border_style())
                    .title(format!(" {} ", self.state.current_chart_type.name())),
            );
            frame.render_widget(widget, area);
        } else {
            let widget = Paragraph::new(gauge_text).block(
                Block::default()
                    .borders(Borders::ALL)
                    .border_style(self.theme.focused_border_style())
                    .title(format!(" {} ", self.state.current_chart_type.name())),
            );
            frame.render_widget(widget, area);
        }
    }

    fn render_timeline(&self, frame: &mut Frame, area: Rect, events: &[EventData]) {
        if events.is_empty() {
            return;
        }

        let min_time = events
            .iter()
            .map(|e| e.timestamp)
            .min_by(|a, b| a.partial_cmp(b).unwrap())
            .unwrap_or(0.0);
        let max_time = events
            .iter()
            .map(|e| e.timestamp)
            .max_by(|a, b| a.partial_cmp(b).unwrap())
            .unwrap_or(100.0);

        let canvas = Canvas::default()
            .block(
                Block::default()
                    .borders(Borders::ALL)
                    .border_style(self.theme.focused_border_style())
                    .title(format!(" {} ", self.state.current_chart_type.name())),
            )
            .x_bounds([min_time, max_time])
            .y_bounds([0.0, 10.0])
            .paint(|ctx| {
                let colors = [
                    ratatui::style::Color::Cyan,    // Info
                    ratatui::style::Color::Yellow,  // Warning
                    ratatui::style::Color::Red,     // Error
                    ratatui::style::Color::Magenta, // Critical
                ];

                // Draw timeline base
                ctx.draw(&CanvasLine {
                    x1: min_time,
                    y1: 5.0,
                    x2: max_time,
                    y2: 5.0,
                    color: ratatui::style::Color::Gray,
                });

                // Draw event markers
                for event in events {
                    let color = colors[event.severity.color_index() as usize % colors.len()];

                    // Draw vertical line for event
                    ctx.draw(&CanvasLine {
                        x1: event.timestamp,
                        y1: 4.0,
                        x2: event.timestamp,
                        y2: 6.0,
                        color,
                    });

                    // Draw point at event
                    ctx.draw(&Points {
                        coords: &[(event.timestamp, 5.0)],
                        color,
                    });
                }
            });

        frame.render_widget(canvas, area);
    }
}

impl View for DataVizView {
    fn render(&mut self, frame: &mut Frame, area: Rect, _app_state: &AppState) {
        let chunks = Layout::default()
            .direction(Direction::Vertical)
            .constraints([
                Constraint::Length(6),  // Header
                Constraint::Min(10),    // Chart
                Constraint::Length(10), // Shortcuts
            ])
            .split(area);

        self.render_header(frame, chunks[0]);

        // Render appropriate chart based on current type
        match &self.state.chart_data {
            ChartData::TimeSeries(series) => {
                if self.state.current_chart_type == ChartType::LineChart {
                    self.render_line_chart(frame, chunks[1], series);
                } else if self.state.current_chart_type == ChartType::ScatterPlot {
                    if let Some(first_series) = series.first() {
                        self.render_scatter_plot(frame, chunks[1], first_series);
                    }
                }
            }
            ChartData::Category(categories) => {
                self.render_bar_chart(frame, chunks[1], categories);
            }
            ChartData::Gauge(gauge) => {
                self.render_gauge(frame, chunks[1], gauge);
            }
            ChartData::Events(events) => {
                self.render_timeline(frame, chunks[1], events);
            }
        }

        self.render_shortcuts(frame, chunks[2]);
    }

    async fn handle_key(&mut self, key: KeyEvent, _app_state: &mut AppState) -> Result<ViewAction> {
        match key.code {
            KeyCode::Char('q') => Ok(ViewAction::PopView),
            KeyCode::Char('c') if key.modifiers.contains(KeyModifiers::CONTROL) => {
                Ok(ViewAction::Quit)
            }
            KeyCode::Tab => {
                self.state.next_chart_type();
                Ok(ViewAction::None)
            }
            KeyCode::BackTab => {
                self.state.previous_chart_type();
                Ok(ViewAction::None)
            }
            KeyCode::Char('+') | KeyCode::Char('=') => {
                self.state.zoom_in();
                Ok(ViewAction::None)
            }
            KeyCode::Char('-') | KeyCode::Char('_') => {
                self.state.zoom_out();
                Ok(ViewAction::None)
            }
            KeyCode::Char('0') => {
                self.state.reset_zoom();
                Ok(ViewAction::None)
            }
            KeyCode::Char('r') | KeyCode::Char('R') => {
                self.state.refresh();
                Ok(ViewAction::None)
            }
            KeyCode::Up | KeyCode::Char('k') => {
                self.state.scroll_up();
                Ok(ViewAction::None)
            }
            KeyCode::Down | KeyCode::Char('j') => {
                self.state.scroll_down();
                Ok(ViewAction::None)
            }
            KeyCode::Char('?') => Ok(ViewAction::ShowHelp),
            _ => Ok(ViewAction::None),
        }
    }

    async fn update(&mut self, _app_state: &mut AppState) -> Result<()> {
        Ok(())
    }

    fn help_text(&self) -> Vec<(&str, &str)> {
        vec![
            ("Tab", "Next chart type"),
            ("Shift+Tab", "Previous chart type"),
            ("+/-", "Zoom in/out"),
            ("0", "Reset zoom"),
            ("r", "Refresh data"),
            ("↑/↓", "Scroll"),
            ("?", "Show help"),
            ("q", "Back"),
        ]
    }

    fn auto_refresh(&self) -> Option<Duration> {
        self.base.auto_refresh_interval
    }

    fn title(&self) -> &str {
        &self.base.title
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn create_test_view() -> DataVizView {
        DataVizView::new(&ThemeConfig::default())
    }

    #[test]
    fn test_data_viz_view_new() {
        let view = create_test_view();
        assert_eq!(view.state.current_chart_type, ChartType::LineChart);
        assert_eq!(view.title(), "Data Visualization");
    }

    #[tokio::test]
    async fn test_handle_tab_key() {
        let mut view = create_test_view();
        let mut app_state = AppState::default();
        let key = KeyEvent::from(KeyCode::Tab);
        let _ = view.handle_key(key, &mut app_state).await;
        assert_eq!(view.state.current_chart_type, ChartType::BarChart);
    }

    #[tokio::test]
    async fn test_handle_zoom_keys() {
        let mut view = create_test_view();
        let mut app_state = AppState::default();

        let zoom_in_key = KeyEvent::from(KeyCode::Char('+'));
        let _ = view.handle_key(zoom_in_key, &mut app_state).await;
        assert!(view.state.zoom_level > 1.0);

        let zoom_out_key = KeyEvent::from(KeyCode::Char('-'));
        let _ = view.handle_key(zoom_out_key, &mut app_state).await;
        assert_eq!(view.state.zoom_level, 1.0);
    }

    #[tokio::test]
    async fn test_handle_refresh_key() {
        let mut view = create_test_view();
        let mut app_state = AppState::default();
        let refresh_key = KeyEvent::from(KeyCode::Char('r'));
        let initial_time = view.state.last_refresh;

        std::thread::sleep(std::time::Duration::from_millis(10));
        let _ = view.handle_key(refresh_key, &mut app_state).await;

        assert!(view.state.last_refresh > initial_time);
    }

    #[test]
    fn test_help_text() {
        let view = create_test_view();
        let help = view.help_text();
        assert!(!help.is_empty());
        assert!(help.iter().any(|(key, _)| *key == "Tab"));
    }
}
