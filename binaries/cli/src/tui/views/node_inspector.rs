use crossterm::event::{KeyCode, KeyEvent, KeyModifiers};
/// Node Inspector View - Detailed node inspection and monitoring (Issue #27)
use ratatui::{
    Frame,
    layout::{Constraint, Direction, Layout, Rect},
    style::{Color, Modifier, Style},
    text::{Line, Span},
    widgets::{Block, Borders, Gauge, List, ListItem, Paragraph, Wrap},
};
use std::time::Duration;

use super::{BaseView, InspectorTab, NodeInspectorState, NodeMetrics, View, ViewAction};
use crate::tui::{Result, app::AppState, theme::ThemeConfig};

/// Node Inspector View for detailed node inspection and monitoring
pub struct NodeInspectorView {
    base: BaseView,
    theme: ThemeConfig,
    /// Current inspector state
    pub state: NodeInspectorState,
}

impl NodeInspectorView {
    /// Create a new NodeInspectorView
    pub fn new(theme: &ThemeConfig, node_id: String) -> Self {
        Self {
            base: BaseView::new("Node Inspector".to_string())
                .with_auto_refresh(Duration::from_secs(1)),
            theme: theme.clone(),
            state: NodeInspectorState::new(node_id),
        }
    }

    fn build_node_metrics(&self, app_state: &AppState) -> NodeMetrics {
        let sys = &app_state.system_metrics;
        NodeMetrics {
            cpu_percent: sys.cpu_usage as f64,
            memory_percent: sys.memory_usage as f64,
            message_rate: 0.0,
            processing_latency_ms: 0.0,
            uptime_seconds: sys.last_update.map(|_| 0).unwrap_or(0),
            error_count: 0,
        }
    }

    /// Render tab bar
    fn render_tab_bar(&self, f: &mut Frame, area: Rect) {
        let tabs = InspectorTab::all();
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

        let tabs_paragraph = Paragraph::new(tabs_text)
            .block(self.theme.styled_block("Tabs"))
            .wrap(Wrap { trim: true });

        f.render_widget(tabs_paragraph, area);
    }

    /// Render help bar
    fn render_help_bar(&self, f: &mut Frame, area: Rect) {
        let help_text = match self.state.active_tab {
            InspectorTab::Overview => "[q]uit [←→]tabs [d]etails [r]efresh",
            InspectorTab::Connections => "[q]uit [←→]tabs [↑↓]navigate [Tab]switch",
            InspectorTab::Performance => "[q]uit [←→]tabs [d]etails [r]efresh",
            InspectorTab::Configuration => "[q]uit [←→]tabs [e]dit [r]efresh",
            InspectorTab::Debug => "[q]uit [←→]tabs [↑↓]scroll [r]efresh",
        };

        let help = Paragraph::new(help_text)
            .style(Style::default().fg(self.theme.colors.muted))
            .block(Block::default().borders(Borders::ALL));

        f.render_widget(help, area);
    }

    /// Render Overview tab
    fn render_overview_tab(&self, f: &mut Frame, area: Rect, app_state: &AppState) {
        // Find the node in app_state
        let node_info = self.find_node_info(app_state);

        let node_name = node_info
            .as_ref()
            .map(|n| n.name.as_str())
            .unwrap_or(&self.state.node_id);

        let node_status = node_info
            .as_ref()
            .map(|n| n.status.as_str())
            .unwrap_or("unknown");

        // Create overview content
        let overview_text = vec![
            Line::from(vec![
                Span::styled(
                    "Node: ",
                    Style::default()
                        .fg(self.theme.colors.primary)
                        .add_modifier(Modifier::BOLD),
                ),
                Span::styled(node_name, Style::default().fg(self.theme.colors.text)),
            ]),
            Line::from(""),
            Line::from(vec![
                Span::styled("ID: ", Style::default().fg(self.theme.colors.muted)),
                Span::raw(&self.state.node_id),
            ]),
            Line::from(vec![
                Span::styled("Status: ", Style::default().fg(self.theme.colors.muted)),
                self.status_span(node_status),
            ]),
            Line::from(""),
            Line::from(vec![Span::styled(
                "─── Metrics ───",
                Style::default()
                    .fg(self.theme.colors.primary)
                    .add_modifier(Modifier::BOLD),
            )]),
            Line::from(""),
        ];

        let title = format!("Overview - {}", node_name);
        let paragraph = Paragraph::new(overview_text)
            .block(self.theme.styled_block(&title))
            .wrap(Wrap { trim: true });

        // Split area for overview and metrics
        let chunks = Layout::default()
            .direction(Direction::Vertical)
            .constraints([Constraint::Length(10), Constraint::Min(5)])
            .split(area);

        f.render_widget(paragraph, chunks[0]);

        // Render metrics gauges
        self.render_metrics_gauges(f, chunks[1], app_state);
    }

    /// Render metrics gauges
    fn render_metrics_gauges(&self, f: &mut Frame, area: Rect, app_state: &AppState) {
        let sys_metrics = &app_state.system_metrics;
        let node_metrics = NodeMetrics {
            cpu_percent: sys_metrics.cpu_usage as f64,
            memory_percent: sys_metrics.memory_usage as f64,
            message_rate: 0.0,
            processing_latency_ms: 0.0,
            uptime_seconds: sys_metrics.last_update.map(|_| 0).unwrap_or(0),
            error_count: 0,
        };

        let gauge_areas = Layout::default()
            .direction(Direction::Vertical)
            .constraints([
                Constraint::Length(3),
                Constraint::Length(3),
                Constraint::Length(3),
                Constraint::Length(3),
            ])
            .split(area);

        // CPU gauge
        let cpu_percent = node_metrics.cpu_percent.clamp(0.0, 100.0) as u16;
        let cpu_label = format!("CPU: {:.1}%", node_metrics.cpu_percent);
        let cpu_gauge = Gauge::default()
            .block(Block::default().borders(Borders::ALL))
            .gauge_style(Style::default().fg(Color::Cyan))
            .label(cpu_label)
            .percent(cpu_percent);
        f.render_widget(cpu_gauge, gauge_areas[0]);

        // Memory gauge (memory usage stored as percentage)
        let memory_percent = node_metrics.memory_percent.clamp(0.0, 100.0) as u16;
        let memory_label = format!("Memory: {:.1}%", node_metrics.memory_percent);
        let memory_gauge = Gauge::default()
            .block(Block::default().borders(Borders::ALL))
            .gauge_style(Style::default().fg(Color::Green))
            .label(memory_label)
            .percent(memory_percent);
        f.render_widget(memory_gauge, gauge_areas[1]);

        // Network throughput
        let (rx, tx) = sys_metrics.network_io;
        let throughput_label = format!("Network: ↓ {} KB  ↑ {} KB", rx / 1024, tx / 1024);
        let throughput = ((rx + tx) as f64 / 1024.0).min(100.0) as u16;
        let throughput_gauge = Gauge::default()
            .block(Block::default().borders(Borders::ALL))
            .gauge_style(Style::default().fg(Color::Yellow))
            .label(throughput_label)
            .percent(throughput);
        f.render_widget(throughput_gauge, gauge_areas[2]);

        // Latency
        let latency_label = "Latency: n/a".to_string();
        let latency_gauge = Gauge::default()
            .block(Block::default().borders(Borders::ALL))
            .gauge_style(Style::default().fg(Color::Magenta))
            .label(latency_label)
            .percent(0);
        f.render_widget(latency_gauge, gauge_areas[3]);
    }

    /// Render Connections tab
    fn render_connections_tab(&self, f: &mut Frame, area: Rect, app_state: &AppState) {
        let node_info = self.find_node_info(app_state);

        // Split into inputs and outputs
        let chunks = Layout::default()
            .direction(Direction::Horizontal)
            .constraints([Constraint::Percentage(50), Constraint::Percentage(50)])
            .split(area);

        // Render inputs
        let input_items: Vec<ListItem> = if let Some(_node) = node_info {
            // TODO: Get real inputs from node info
            vec![
                ListItem::new(Line::from(vec![
                    Span::styled("● ", Style::default().fg(Color::Green)),
                    Span::raw("camera_feed"),
                    Span::styled(" (Image)", Style::default().fg(self.theme.colors.muted)),
                ])),
                ListItem::new(Line::from(vec![
                    Span::styled("● ", Style::default().fg(Color::Green)),
                    Span::raw("control_signal"),
                    Span::styled(" (Command)", Style::default().fg(self.theme.colors.muted)),
                ])),
            ]
        } else {
            vec![ListItem::new("No inputs")]
        };

        let inputs_list = List::new(input_items)
            .block(self.theme.styled_block("Inputs"))
            .highlight_style(Style::default().add_modifier(Modifier::BOLD));

        f.render_widget(inputs_list, chunks[0]);

        // Render outputs
        let output_items: Vec<ListItem> = if let Some(_node) = node_info {
            // TODO: Get real outputs from node info
            vec![
                ListItem::new(Line::from(vec![
                    Span::styled("● ", Style::default().fg(Color::Green)),
                    Span::raw("detection_result"),
                    Span::styled(" (Detection)", Style::default().fg(self.theme.colors.muted)),
                ])),
                ListItem::new(Line::from(vec![
                    Span::styled("● ", Style::default().fg(Color::Green)),
                    Span::raw("debug_image"),
                    Span::styled(" (Image)", Style::default().fg(self.theme.colors.muted)),
                ])),
            ]
        } else {
            vec![ListItem::new("No outputs")]
        };

        let outputs_list = List::new(output_items)
            .block(self.theme.styled_block("Outputs"))
            .highlight_style(Style::default().add_modifier(Modifier::BOLD));

        f.render_widget(outputs_list, chunks[1]);
    }

    /// Render Performance tab
    fn render_performance_tab(&self, f: &mut Frame, area: Rect, app_state: &AppState) {
        let metrics = self.build_node_metrics(app_state);

        let perf_text = vec![
            Line::from(vec![Span::styled(
                "Performance Metrics",
                Style::default()
                    .fg(self.theme.colors.primary)
                    .add_modifier(Modifier::BOLD),
            )]),
            Line::from(""),
            Line::from(vec![
                Span::styled("CPU Usage: ", Style::default().fg(self.theme.colors.muted)),
                Span::raw(format!("{:.1}%", metrics.cpu_percent)),
            ]),
            Line::from(vec![
                Span::styled("Memory: ", Style::default().fg(self.theme.colors.muted)),
                Span::raw(format!("{:.1}%", metrics.memory_percent)),
            ]),
            Line::from(vec![
                Span::styled(
                    "Message Rate: ",
                    Style::default().fg(self.theme.colors.muted),
                ),
                Span::raw(format!("{:.1} msg/s", metrics.message_rate)),
            ]),
            Line::from(vec![
                Span::styled(
                    "Processing Latency: ",
                    Style::default().fg(self.theme.colors.muted),
                ),
                Span::raw(format!("{:.2} ms", metrics.processing_latency_ms)),
            ]),
            Line::from(vec![
                Span::styled("Uptime: ", Style::default().fg(self.theme.colors.muted)),
                Span::raw(format_uptime(metrics.uptime_seconds)),
            ]),
            Line::from(vec![
                Span::styled(
                    "Error Count: ",
                    Style::default().fg(self.theme.colors.muted),
                ),
                Span::raw(format!("{}", metrics.error_count)),
            ]),
            Line::from(""),
            Line::from(vec![Span::styled(
                "─── Message Statistics ───",
                Style::default().fg(self.theme.colors.primary),
            )]),
            Line::from(""),
            Line::from("Total Messages: 1,234"),
            Line::from("Avg Processing Time: 2.5 ms"),
            Line::from("Peak Message Rate: 145 msg/s"),
        ];

        let title = format!("Performance - {}", self.state.node_id);
        let paragraph = Paragraph::new(perf_text)
            .block(self.theme.styled_block(&title))
            .wrap(Wrap { trim: true });

        f.render_widget(paragraph, area);
    }

    /// Render Configuration tab
    fn render_configuration_tab(&self, f: &mut Frame, area: Rect, _app_state: &AppState) {
        let config_text = vec![
            Line::from(vec![Span::styled(
                "Node Configuration",
                Style::default()
                    .fg(self.theme.colors.primary)
                    .add_modifier(Modifier::BOLD),
            )]),
            Line::from(""),
            Line::from(vec![Span::styled(
                "node:",
                Style::default().fg(Color::Yellow),
            )]),
            Line::from(vec![
                Span::raw("  id: "),
                Span::styled(&self.state.node_id, Style::default().fg(Color::Cyan)),
            ]),
            Line::from(vec![
                Span::raw("  operator: "),
                Span::styled("detector", Style::default().fg(Color::Cyan)),
            ]),
            Line::from(""),
            Line::from(vec![Span::styled(
                "inputs:",
                Style::default().fg(Color::Yellow),
            )]),
            Line::from("  - camera_feed"),
            Line::from("  - control_signal"),
            Line::from(""),
            Line::from(vec![Span::styled(
                "outputs:",
                Style::default().fg(Color::Yellow),
            )]),
            Line::from("  - detection_result"),
            Line::from("  - debug_image"),
            Line::from(""),
            Line::from(vec![Span::styled(
                "parameters:",
                Style::default().fg(Color::Yellow),
            )]),
            Line::from("  confidence_threshold: 0.5"),
            Line::from("  max_detections: 100"),
            Line::from(""),
            Line::from(vec![Span::styled(
                "Press [e] to edit configuration",
                Style::default().fg(self.theme.colors.muted),
            )]),
        ];

        let title = format!("Configuration - {}", self.state.node_id);
        let paragraph = Paragraph::new(config_text)
            .block(self.theme.styled_block(&title))
            .wrap(Wrap { trim: true });

        f.render_widget(paragraph, area);
    }

    /// Render Debug tab
    fn render_debug_tab(&self, f: &mut Frame, area: Rect, _app_state: &AppState) {
        let debug_text = vec![
            Line::from(vec![Span::styled(
                "Debug Information",
                Style::default()
                    .fg(self.theme.colors.primary)
                    .add_modifier(Modifier::BOLD),
            )]),
            Line::from(""),
            Line::from(vec![
                Span::styled("Process ID: ", Style::default().fg(self.theme.colors.muted)),
                Span::raw("12345"),
            ]),
            Line::from(vec![
                Span::styled(
                    "Working Directory: ",
                    Style::default().fg(self.theme.colors.muted),
                ),
                Span::raw("/path/to/node"),
            ]),
            Line::from(vec![
                Span::styled("Command: ", Style::default().fg(self.theme.colors.muted)),
                Span::raw("python operator.py"),
            ]),
            Line::from(""),
            Line::from(vec![Span::styled(
                "─── Recent Logs ───",
                Style::default().fg(self.theme.colors.primary),
            )]),
            Line::from(""),
            Line::from("[INFO] Node started successfully"),
            Line::from("[INFO] Connected to input: camera_feed"),
            Line::from("[INFO] Processing message batch (10 items)"),
            Line::from("[DEBUG] Detection complete in 2.3ms"),
            Line::from("[INFO] Sent output to detection_result"),
            Line::from(""),
            Line::from(vec![Span::styled(
                "─── Environment Variables ───",
                Style::default().fg(self.theme.colors.primary),
            )]),
            Line::from(""),
            Line::from("DORA_NODE_ID: detector-node"),
            Line::from("PYTHONPATH: /usr/local/lib/python3.10"),
            Line::from(""),
            Line::from(vec![Span::styled(
                "Use ↑↓ to scroll",
                Style::default().fg(self.theme.colors.muted),
            )]),
        ];

        let title = format!("Debug - {}", self.state.node_id);
        let paragraph = Paragraph::new(debug_text)
            .block(self.theme.styled_block(&title))
            .wrap(Wrap { trim: true })
            .scroll((self.state.scroll_offset as u16, 0));

        f.render_widget(paragraph, area);
    }

    /// Get status indicator span
    fn status_span(&self, status: &str) -> Span {
        let (symbol, color) = match status.to_lowercase().as_str() {
            "running" | "active" => ("● running", Color::Green),
            "stopped" | "inactive" => ("● stopped", Color::Red),
            "error" | "failed" => ("● error", Color::Red),
            _ => ("○ unknown", Color::Gray),
        };

        Span::styled(symbol, Style::default().fg(color))
    }

    /// Find node info in app state
    fn find_node_info<'a>(&self, app_state: &'a AppState) -> Option<&'a crate::tui::app::NodeInfo> {
        for dataflow in &app_state.dataflows {
            for node in &dataflow.nodes {
                if node.id == self.state.node_id {
                    return Some(node);
                }
            }
        }
        None
    }
}

impl View for NodeInspectorView {
    fn render(&mut self, f: &mut Frame, area: Rect, app_state: &AppState) {
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
        let node_name = self
            .find_node_info(app_state)
            .map(|n| n.name.as_str())
            .unwrap_or(&self.state.node_id);
        let title = format!("Node Inspector: {}", node_name);
        let title_paragraph = Paragraph::new(title)
            .block(self.theme.styled_block(""))
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
            InspectorTab::Overview => self.render_overview_tab(f, chunks[2], app_state),
            InspectorTab::Connections => self.render_connections_tab(f, chunks[2], app_state),
            InspectorTab::Performance => self.render_performance_tab(f, chunks[2], app_state),
            InspectorTab::Configuration => self.render_configuration_tab(f, chunks[2], app_state),
            InspectorTab::Debug => self.render_debug_tab(f, chunks[2], app_state),
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
            KeyCode::Char('1') => self.state.switch_tab(InspectorTab::Overview),
            KeyCode::Char('2') => self.state.switch_tab(InspectorTab::Connections),
            KeyCode::Char('3') => self.state.switch_tab(InspectorTab::Performance),
            KeyCode::Char('4') => self.state.switch_tab(InspectorTab::Configuration),
            KeyCode::Char('5') => self.state.switch_tab(InspectorTab::Debug),

            // Scroll for Debug tab
            KeyCode::Up | KeyCode::Char('k') if self.state.active_tab == InspectorTab::Debug => {
                self.state.scroll_up();
            }
            KeyCode::Down | KeyCode::Char('j') if self.state.active_tab == InspectorTab::Debug => {
                self.state.scroll_down(20); // Max scroll lines
            }

            // Toggle detailed metrics
            KeyCode::Char('d') => {
                self.state.toggle_detailed_metrics();
            }

            // Refresh
            KeyCode::Char('r') if key.modifiers.contains(KeyModifiers::CONTROL) => {
                return Ok(ViewAction::Refresh);
            }

            // Edit mode toggle
            KeyCode::Char('e') if self.state.active_tab == InspectorTab::Configuration => {
                self.state.toggle_edit_mode();
            }

            _ => {}
        }

        Ok(ViewAction::None)
    }

    async fn update(&mut self, _app_state: &mut AppState) -> Result<()> {
        self.state.mark_refreshed();
        Ok(())
    }

    fn help_text(&self) -> Vec<(&str, &str)> {
        vec![
            ("q/Esc", "Back to previous view"),
            ("←/→", "Switch tabs"),
            ("1-5", "Jump to specific tab"),
            ("d", "Toggle detailed metrics"),
            ("r", "Refresh data"),
            ("↑/↓", "Scroll (Debug tab)"),
        ]
    }

    fn title(&self) -> &str {
        &self.base.title
    }

    fn auto_refresh(&self) -> Option<Duration> {
        self.base.auto_refresh_interval
    }
}

/// Format uptime duration in human-readable format
fn format_uptime(seconds: u64) -> String {
    if seconds < 60 {
        format!("{}s", seconds)
    } else if seconds < 3600 {
        format!("{}m {}s", seconds / 60, seconds % 60)
    } else {
        format!("{}h {}m", seconds / 3600, (seconds % 3600) / 60)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_node_inspector_view_creation() {
        let theme = ThemeConfig::default();
        let view = NodeInspectorView::new(&theme, "test-node".to_string());

        assert_eq!(view.state.node_id, "test-node");
        assert_eq!(view.state.active_tab, InspectorTab::Overview);
    }

    #[test]
    fn test_format_uptime() {
        assert_eq!(format_uptime(30), "30s");
        assert_eq!(format_uptime(90), "1m 30s");
        assert_eq!(format_uptime(3661), "1h 1m");
    }
}
