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
use dora_message::descriptor::CoreNodeKind;

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
        let node_info = self.find_node_info(app_state);

        let node_name = node_info
            .map(|n| n.name.as_str())
            .unwrap_or(&self.state.node_id);
        let node_status = node_info.map(|n| n.status.as_str()).unwrap_or("unknown");
        let node_kind = node_info.map(|n| n.kind.as_str()).unwrap_or("unknown");
        let node_source = node_info.and_then(|n| n.source.as_deref());
        let node_description = node_info.and_then(|n| n.description.as_deref());

        let mut overview_text = vec![
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
            Line::from(vec![
                Span::styled("Kind: ", Style::default().fg(self.theme.colors.muted)),
                Span::raw(node_kind),
            ]),
        ];

        if let Some(source) = node_source {
            overview_text.push(Line::from(vec![
                Span::styled("Source: ", Style::default().fg(self.theme.colors.muted)),
                Span::raw(source),
            ]));
        }

        if let Some(description) = node_description {
            overview_text.push(Line::from(""));
            overview_text.push(Line::from(vec![Span::styled(
                "Description:",
                Style::default()
                    .fg(self.theme.colors.primary)
                    .add_modifier(Modifier::BOLD),
            )]));
            overview_text.push(Line::from(description));
        }

        overview_text.push(Line::from(""));
        overview_text.push(Line::from(vec![Span::styled(
            "─── Metrics ───",
            Style::default()
                .fg(self.theme.colors.primary)
                .add_modifier(Modifier::BOLD),
        )]));
        overview_text.push(Line::from(""));

        let title = format!("Overview - {}", node_name);
        let paragraph = Paragraph::new(overview_text)
            .block(self.theme.styled_block(&title))
            .wrap(Wrap { trim: true });

        let chunks = Layout::default()
            .direction(Direction::Vertical)
            .constraints([Constraint::Length(12), Constraint::Min(5)])
            .split(area);

        f.render_widget(paragraph, chunks[0]);
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

        let input_items: Vec<ListItem> = if let Some(node) = node_info {
            if node.inputs.is_empty() {
                vec![ListItem::new(Line::from("No inputs configured"))]
            } else {
                node.inputs
                    .iter()
                    .map(|input| {
                        ListItem::new(Line::from(vec![
                            Span::styled("● ", Style::default().fg(Color::Green)),
                            Span::raw(input.clone()),
                        ]))
                    })
                    .collect()
            }
        } else {
            vec![ListItem::new(Line::from("Node not found"))]
        };

        let inputs_list = List::new(input_items)
            .block(self.theme.styled_block("Inputs"))
            .highlight_style(Style::default().add_modifier(Modifier::BOLD));
        f.render_widget(inputs_list, chunks[0]);

        let output_items: Vec<ListItem> = if let Some(node) = node_info {
            if node.outputs.is_empty() {
                vec![ListItem::new(Line::from("No outputs configured"))]
            } else {
                node.outputs
                    .iter()
                    .map(|output| {
                        ListItem::new(Line::from(vec![
                            Span::styled("● ", Style::default().fg(Color::Green)),
                            Span::raw(output.clone()),
                        ]))
                    })
                    .collect()
            }
        } else {
            vec![ListItem::new(Line::from("Node not found"))]
        };

        let outputs_list = List::new(output_items)
            .block(self.theme.styled_block("Outputs"))
            .highlight_style(Style::default().add_modifier(Modifier::BOLD));
        f.render_widget(outputs_list, chunks[1]);
    }

    /// Render Performance tab
    fn render_performance_tab(&self, f: &mut Frame, area: Rect, app_state: &AppState) {
        let metrics = self.build_node_metrics(app_state);

        let mut perf_text = Vec::new();

        if let Some(node) = self.find_node_info(app_state) {
            perf_text.push(Line::from(vec![
                Span::styled(
                    "Node Status: ",
                    Style::default().fg(self.theme.colors.muted),
                ),
                Span::raw(node.status.clone()),
            ]));
            perf_text.push(Line::from(vec![
                Span::styled("Inputs: ", Style::default().fg(self.theme.colors.muted)),
                Span::raw(node.inputs.len().to_string()),
                Span::raw("   "),
                Span::styled("Outputs: ", Style::default().fg(self.theme.colors.muted)),
                Span::raw(node.outputs.len().to_string()),
            ]));
            perf_text.push(Line::from(""));
        }

        perf_text.extend([
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
            Line::from("Live message statistics are not yet available."),
        ]);

        let title = format!("Performance - {}", self.state.node_id);
        let paragraph = Paragraph::new(perf_text)
            .block(self.theme.styled_block(&title))
            .wrap(Wrap { trim: true });

        f.render_widget(paragraph, area);
    }

    /// Render Configuration tab
    fn render_configuration_tab(&self, f: &mut Frame, area: Rect, app_state: &AppState) {
        let lines = if let Some(node) = self.find_node_info(app_state) {
            let mut lines = Vec::new();
            lines.push(Line::from(vec![Span::styled(
                "Node Configuration",
                Style::default()
                    .fg(self.theme.colors.primary)
                    .add_modifier(Modifier::BOLD),
            )]));
            lines.push(Line::from(""));
            lines.push(Line::from(vec![
                Span::styled("Node ID: ", Style::default().fg(self.theme.colors.muted)),
                Span::raw(node.id.clone()),
            ]));
            lines.push(Line::from(vec![
                Span::styled("Name: ", Style::default().fg(self.theme.colors.muted)),
                Span::raw(node.name.clone()),
            ]));
            lines.push(Line::from(vec![
                Span::styled("Kind: ", Style::default().fg(self.theme.colors.muted)),
                Span::raw(node.kind.clone()),
            ]));
            lines.push(Line::from(vec![
                Span::styled("Status: ", Style::default().fg(self.theme.colors.muted)),
                Span::raw(node.status.clone()),
            ]));
            if let Some(source) = &node.source {
                lines.push(Line::from(vec![
                    Span::styled("Source: ", Style::default().fg(self.theme.colors.muted)),
                    Span::raw(source.clone()),
                ]));
            }
            if let Some(description) = &node.description {
                lines.push(Line::from(""));
                lines.push(Line::from(vec![Span::styled(
                    "Description:",
                    Style::default()
                        .fg(self.theme.colors.primary)
                        .add_modifier(Modifier::BOLD),
                )]));
                lines.push(Line::from(description.clone()));
            }

            lines.push(Line::from(""));
            lines.push(Line::from(vec![Span::styled(
                "Inputs",
                Style::default()
                    .fg(self.theme.colors.primary)
                    .add_modifier(Modifier::BOLD),
            )]));
            if node.inputs.is_empty() {
                lines.push(Line::from("  (no inputs configured)"));
            } else {
                for input in &node.inputs {
                    lines.push(Line::from(format!("  - {}", input)));
                }
            }

            lines.push(Line::from(""));
            lines.push(Line::from(vec![Span::styled(
                "Outputs",
                Style::default()
                    .fg(self.theme.colors.primary)
                    .add_modifier(Modifier::BOLD),
            )]));
            if node.outputs.is_empty() {
                lines.push(Line::from("  (no outputs configured)"));
            } else {
                for output in &node.outputs {
                    lines.push(Line::from(format!("  - {}", output)));
                }
            }

            if let Some(resolved) = &node.resolved {
                lines.push(Line::from(""));
                match &resolved.kind {
                    CoreNodeKind::Custom(custom) => {
                        lines.push(Line::from(vec![Span::styled(
                            "Executable",
                            Style::default()
                                .fg(self.theme.colors.primary)
                                .add_modifier(Modifier::BOLD),
                        )]));
                        lines.push(Line::from(format!("  path: {}", custom.path)));
                        if let Some(args) = &custom.args {
                            lines.push(Line::from(format!("  args: {}", args)));
                        }
                        if let Some(send_stdout_as) = &custom.send_stdout_as {
                            lines.push(Line::from(format!(
                                "  redirect stdout as: {}",
                                send_stdout_as
                            )));
                        }
                    }
                    CoreNodeKind::Runtime(runtime) => {
                        lines.push(Line::from(vec![Span::styled(
                            "Operators",
                            Style::default()
                                .fg(self.theme.colors.primary)
                                .add_modifier(Modifier::BOLD),
                        )]));
                        if runtime.operators.is_empty() {
                            lines.push(Line::from("  (no operators registered)"));
                        } else {
                            for operator in &runtime.operators {
                                let name = operator
                                    .config
                                    .name
                                    .clone()
                                    .unwrap_or_else(|| operator.id.to_string());
                                lines.push(Line::from(format!("  - {}", name)));
                            }
                        }
                    }
                }

                if let Some(env) = &resolved.env {
                    lines.push(Line::from(""));
                    lines.push(Line::from(vec![Span::styled(
                        "Environment",
                        Style::default()
                            .fg(self.theme.colors.primary)
                            .add_modifier(Modifier::BOLD),
                    )]));
                    for (key, value) in env {
                        lines.push(Line::from(format!("  {key}={value}")));
                    }
                }
            }

            lines
        } else {
            vec![Line::from("Node not found")]
        };

        let title = format!("Configuration - {}", self.state.node_id);
        let paragraph = Paragraph::new(lines)
            .block(self.theme.styled_block(&title))
            .wrap(Wrap { trim: true });

        f.render_widget(paragraph, area);
    }

    /// Render Debug tab
    fn render_debug_tab(&self, f: &mut Frame, area: Rect, app_state: &AppState) {
        let mut debug_text = vec![
            Line::from(vec![Span::styled(
                "Debug Information",
                Style::default()
                    .fg(self.theme.colors.primary)
                    .add_modifier(Modifier::BOLD),
            )]),
            Line::from(""),
            Line::from("Live log streaming is not yet available in the TUI."),
            Line::from("Use `dora logs` for real-time output."),
        ];

        if let Some(node) = self.find_node_info(app_state) {
            if let Some(resolved) = &node.resolved {
                match &resolved.kind {
                    CoreNodeKind::Custom(custom) => {
                        debug_text.push(Line::from(""));
                        debug_text.push(Line::from(vec![Span::styled(
                            "Executable",
                            Style::default()
                                .fg(self.theme.colors.primary)
                                .add_modifier(Modifier::BOLD),
                        )]));
                        debug_text.push(Line::from(format!("Path: {}", custom.path)));
                        if let Some(args) = &custom.args {
                            debug_text.push(Line::from(format!("Args: {}", args)));
                        }
                    }
                    CoreNodeKind::Runtime(runtime) => {
                        debug_text.push(Line::from(""));
                        debug_text.push(Line::from(vec![Span::styled(
                            "Operators",
                            Style::default()
                                .fg(self.theme.colors.primary)
                                .add_modifier(Modifier::BOLD),
                        )]));
                        if runtime.operators.is_empty() {
                            debug_text.push(Line::from("  (no operators registered)"));
                        } else {
                            for operator in &runtime.operators {
                                let name = operator
                                    .config
                                    .name
                                    .clone()
                                    .unwrap_or_else(|| operator.id.to_string());
                                debug_text.push(Line::from(format!("  - {}", name)));
                            }
                        }
                    }
                }

                debug_text.push(Line::from(""));
                debug_text.push(Line::from(vec![Span::styled(
                    "Environment Variables",
                    Style::default()
                        .fg(self.theme.colors.primary)
                        .add_modifier(Modifier::BOLD),
                )]));
                if let Some(env) = &resolved.env {
                    if env.is_empty() {
                        debug_text.push(Line::from("  (none)"));
                    } else {
                        for (key, value) in env {
                            debug_text.push(Line::from(format!("  {key}={value}")));
                        }
                    }
                } else {
                    debug_text.push(Line::from("  (none)"));
                }
            }
        } else {
            debug_text.push(Line::from(""));
            debug_text.push(Line::from("Node not found."));
        }

        debug_text.push(Line::from(""));
        debug_text.push(Line::from(vec![Span::styled(
            "Use ↑↓ to scroll",
            Style::default().fg(self.theme.colors.muted),
        )]));

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
            "completed" => ("● completed", Color::Blue),
            "exited" => ("● exited", Color::Yellow),
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
