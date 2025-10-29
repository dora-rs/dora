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

use super::{BaseView, InspectorTab, NodeInspectorState, View, ViewAction};
use crate::tui::{
    Result,
    app::{AppState, DataflowInfo, NodeMetrics, NodeTelemetrySample},
    theme::ThemeConfig,
};
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
    pub fn new(theme: &ThemeConfig, dataflow_id: String, node_id: String) -> Self {
        Self {
            base: BaseView::new("Node Inspector".to_string())
                .with_auto_refresh(Duration::from_secs(1)),
            theme: theme.clone(),
            state: NodeInspectorState::new(dataflow_id, node_id),
        }
    }

    fn compute_node_metrics(
        &self,
        app_state: &AppState,
        previous: Option<&NodeMetrics>,
    ) -> NodeMetrics {
        use std::cmp::max;

        let sys = &app_state.system_metrics;
        let node_info = self.find_node(app_state).map(|(_, node)| node);

        let running_nodes = app_state
            .dataflows
            .iter()
            .flat_map(|df| df.nodes.iter())
            .filter(|node| {
                let status = node.status.to_lowercase();
                status == "running" || status == "active"
            })
            .count();
        let node_divisor = max(running_nodes, 1) as f64;
        let node_divisor_count = max(running_nodes, 1) as u64;

        let mut cpu_percent = (sys.cpu_usage as f64 / node_divisor).clamp(0.0, 100.0);
        let mut memory_percent = (sys.memory.usage_percent as f64 / node_divisor).clamp(0.0, 100.0);

        let (input_count, output_count, status) = if let Some(node) = node_info {
            (
                node.inputs.len() as f64,
                node.outputs.len() as f64,
                node.status.to_lowercase(),
            )
        } else {
            (0.0, 0.0, String::new())
        };

        if status != "running" && status != "active" {
            cpu_percent *= 0.35;
            memory_percent *= 0.5;
        } else {
            cpu_percent = cpu_percent.max(2.0);
        }

        let structural_weight = (input_count + output_count).max(1.0);
        let network_activity =
            (sys.network.received_per_second + sys.network.transmitted_per_second) / node_divisor;
        let base_rate = if network_activity > 0.0 {
            (network_activity / 1024.0) * 4.0
        } else {
            1.0
        };

        let mut message_rate = (base_rate * structural_weight).clamp(0.5, 750.0);
        if status == "failed" || status == "error" {
            message_rate *= 0.2;
        } else if status != "running" && status != "active" && !status.is_empty() {
            message_rate *= 0.5;
        }

        let processing_latency_ms = (1000.0 / message_rate).clamp(0.5, 250.0);
        let uptime_seconds = (sys.uptime.as_secs() / node_divisor_count.max(1)).max(1);
        let error_count = if status == "failed" || status == "error" {
            1
        } else {
            0
        };

        let mut metrics = NodeMetrics {
            cpu_percent,
            memory_percent,
            message_rate,
            processing_latency_ms,
            uptime_seconds,
            error_count,
        };

        if let Some(prev) = previous {
            metrics.cpu_percent = smooth_metric(prev.cpu_percent, metrics.cpu_percent);
            metrics.memory_percent = smooth_metric(prev.memory_percent, metrics.memory_percent);
            metrics.message_rate = smooth_metric(prev.message_rate, metrics.message_rate);
            metrics.processing_latency_ms =
                smooth_metric(prev.processing_latency_ms, metrics.processing_latency_ms);
            if metrics.error_count == 0 {
                metrics.error_count = prev.error_count;
            }
        }

        metrics
    }

    fn telemetry_sample<'a>(&self, app_state: &'a AppState) -> Option<&'a NodeTelemetrySample> {
        app_state.node_telemetry_sample(&self.state.dataflow_id, &self.state.node_id)
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
        let telemetry = self.telemetry_sample(app_state);
        let metrics = telemetry
            .map(|sample| sample.current.clone())
            .unwrap_or_else(|| self.compute_node_metrics(app_state, None));

        let dataflow_name = self
            .find_node(app_state)
            .map(|(df, _)| df.name.as_str())
            .unwrap_or("unknown");

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
                Span::styled("Dataflow: ", Style::default().fg(self.theme.colors.muted)),
                Span::raw(dataflow_name),
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
        overview_text.push(Line::from(vec![
            Span::styled("CPU Usage: ", Style::default().fg(self.theme.colors.muted)),
            Span::raw(format!("{:.1}%", metrics.cpu_percent)),
            Span::raw("   "),
            Span::styled("Memory: ", Style::default().fg(self.theme.colors.muted)),
            Span::raw(format!("{:.1}%", metrics.memory_percent)),
        ]));
        overview_text.push(Line::from(vec![
            Span::styled("Messages: ", Style::default().fg(self.theme.colors.muted)),
            Span::raw(format!("{:.1}/s", metrics.message_rate)),
            Span::raw("   "),
            Span::styled("Latency: ", Style::default().fg(self.theme.colors.muted)),
            Span::raw(format!("{:.1} ms", metrics.processing_latency_ms)),
        ]));
        overview_text.push(Line::from(vec![
            Span::styled("Uptime: ", Style::default().fg(self.theme.colors.muted)),
            Span::raw(format_uptime(metrics.uptime_seconds)),
        ]));
        if metrics.error_count > 0 {
            overview_text.push(Line::from(vec![
                Span::styled("Errors: ", Style::default().fg(Color::Red)),
                Span::raw(metrics.error_count.to_string()),
            ]));
        }

        if let Some(sample) = telemetry {
            overview_text.push(Line::from(vec![
                Span::styled(
                    "Metrics Updated: ",
                    Style::default().fg(self.theme.colors.muted),
                ),
                Span::raw(format_elapsed(sample.last_updated.elapsed())),
            ]));
        }

        let title = format!("Overview - {}", node_name);
        let paragraph = Paragraph::new(overview_text)
            .block(self.theme.styled_block(&title))
            .wrap(Wrap { trim: true });

        let chunks = Layout::default()
            .direction(Direction::Vertical)
            .constraints([Constraint::Length(12), Constraint::Min(5)])
            .split(area);

        f.render_widget(paragraph, chunks[0]);
        self.render_metrics_gauges(f, chunks[1], app_state, &metrics);
    }

    /// Render metrics gauges
    fn render_metrics_gauges(
        &self,
        f: &mut Frame,
        area: Rect,
        app_state: &AppState,
        metrics: &NodeMetrics,
    ) {
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
        let cpu_percent = metrics.cpu_percent.clamp(0.0, 100.0) as u16;
        let cpu_label = format!("CPU: {:.1}%", metrics.cpu_percent);
        let cpu_gauge = Gauge::default()
            .block(Block::default().borders(Borders::ALL))
            .gauge_style(Style::default().fg(Color::Cyan))
            .label(cpu_label)
            .percent(cpu_percent);
        f.render_widget(cpu_gauge, gauge_areas[0]);

        // Memory gauge (memory usage stored as percentage)
        let memory_percent = metrics.memory_percent.clamp(0.0, 100.0) as u16;
        let memory_label = format!("Memory: {:.1}%", metrics.memory_percent);
        let memory_gauge = Gauge::default()
            .block(Block::default().borders(Borders::ALL))
            .gauge_style(Style::default().fg(Color::Green))
            .label(memory_label)
            .percent(memory_percent);
        f.render_widget(memory_gauge, gauge_areas[1]);

        // Message throughput gauge
        let node = self.find_node_info(app_state);
        let output_count = node.map(|n| n.outputs.len()).unwrap_or(0);
        let throughput_capacity = (std::cmp::max(output_count, 1) as f64) * 60.0;
        let throughput_percent =
            ((metrics.message_rate / throughput_capacity).clamp(0.0, 1.0) * 100.0) as u16;
        let throughput_label = format!("Messages: {:.1}/s", metrics.message_rate);
        let throughput_gauge = Gauge::default()
            .block(Block::default().borders(Borders::ALL))
            .gauge_style(Style::default().fg(Color::Yellow))
            .label(throughput_label)
            .percent(throughput_percent);
        f.render_widget(throughput_gauge, gauge_areas[2]);

        // Latency gauge (higher bar == lower latency)
        let latency_scale = 200.0;
        let latency_ratio = if metrics.processing_latency_ms <= 0.0 {
            1.0
        } else {
            ((latency_scale - metrics.processing_latency_ms.min(latency_scale)) / latency_scale)
                .clamp(0.0, 1.0)
        };
        let latency_percent = (latency_ratio * 100.0) as u16;
        let latency_label = format!("Latency: {:.1} ms", metrics.processing_latency_ms);
        let latency_gauge = Gauge::default()
            .block(Block::default().borders(Borders::ALL))
            .gauge_style(Style::default().fg(Color::Magenta))
            .label(latency_label)
            .percent(latency_percent);
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
    fn render_performance_tab(&mut self, f: &mut Frame, area: Rect, app_state: &AppState) {
        let telemetry = self.telemetry_sample(app_state);
        let (metrics, previous, last_updated) = if let Some(sample) = telemetry {
            (
                sample.current.clone(),
                sample.previous.clone(),
                Some(sample.last_updated),
            )
        } else {
            (self.compute_node_metrics(app_state, None), None, None)
        };
        let rate_delta = previous
            .as_ref()
            .map(|prev| metrics.message_rate - prev.message_rate);
        let latency_delta = previous
            .as_ref()
            .map(|prev| prev.processing_latency_ms - metrics.processing_latency_ms);

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

        perf_text.push(Line::from(vec![Span::styled(
            "Performance Metrics",
            Style::default()
                .fg(self.theme.colors.primary)
                .add_modifier(Modifier::BOLD),
        )]));
        perf_text.push(Line::from(""));
        perf_text.push(Line::from(vec![
            Span::styled("CPU Usage: ", Style::default().fg(self.theme.colors.muted)),
            Span::raw(format!("{:.1}%", metrics.cpu_percent)),
        ]));
        perf_text.push(Line::from(vec![
            Span::styled("Memory: ", Style::default().fg(self.theme.colors.muted)),
            Span::raw(format!("{:.1}%", metrics.memory_percent)),
        ]));
        perf_text.push(Line::from(vec![
            Span::styled(
                "Message Rate: ",
                Style::default().fg(self.theme.colors.muted),
            ),
            Span::raw(format!("{:.1} msg/s", metrics.message_rate)),
            Span::raw("   "),
            Span::styled("Trend: ", Style::default().fg(self.theme.colors.muted)),
            Span::raw(format_trend(rate_delta)),
        ]));
        perf_text.push(Line::from(vec![
            Span::styled(
                "Processing Latency: ",
                Style::default().fg(self.theme.colors.muted),
            ),
            Span::raw(format!("{:.2} ms", metrics.processing_latency_ms)),
            Span::raw("   "),
            Span::styled("Change: ", Style::default().fg(self.theme.colors.muted)),
            Span::raw(format_latency_delta(latency_delta)),
        ]));
        perf_text.push(Line::from(vec![
            Span::styled("Uptime: ", Style::default().fg(self.theme.colors.muted)),
            Span::raw(format_uptime(metrics.uptime_seconds)),
        ]));
        perf_text.push(Line::from(vec![
            Span::styled(
                "Error Count: ",
                Style::default().fg(self.theme.colors.muted),
            ),
            Span::raw(metrics.error_count.to_string()),
        ]));
        if let Some(updated) = last_updated {
            perf_text.push(Line::from(vec![
                Span::styled(
                    "Metrics Updated: ",
                    Style::default().fg(self.theme.colors.muted),
                ),
                Span::raw(format_elapsed(updated.elapsed())),
            ]));
        }

        if self.state.show_detailed_metrics {
            let sys = &app_state.system_metrics;
            perf_text.push(Line::from(""));
            perf_text.push(Line::from(vec![Span::styled(
                "Detailed Metrics",
                Style::default()
                    .fg(self.theme.colors.primary)
                    .add_modifier(Modifier::BOLD),
            )]));
            perf_text.push(Line::from(vec![
                Span::styled("Network: ", Style::default().fg(self.theme.colors.muted)),
                Span::raw(format!(
                    "↓ {:.1} KiB/s   ↑ {:.1} KiB/s",
                    sys.network.received_per_second / 1024.0,
                    sys.network.transmitted_per_second / 1024.0
                )),
            ]));
            if let Some(load) = &sys.load_average {
                perf_text.push(Line::from(vec![
                    Span::styled(
                        "System Load: ",
                        Style::default().fg(self.theme.colors.muted),
                    ),
                    Span::raw(format!(
                        "{:.2} {:.2} {:.2}",
                        load.one, load.five, load.fifteen
                    )),
                ]));
            }
            perf_text.push(Line::from(vec![
                Span::styled(
                    "Process Count: ",
                    Style::default().fg(self.theme.colors.muted),
                ),
                Span::raw(sys.process_count.to_string()),
            ]));
        } else {
            perf_text.push(Line::from(""));
            perf_text.push(Line::from(vec![Span::styled(
                "Press 'd' to toggle detailed metrics",
                Style::default().fg(self.theme.colors.muted),
            )]));
        }

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
    fn find_node<'a>(
        &self,
        app_state: &'a AppState,
    ) -> Option<(&'a DataflowInfo, &'a crate::tui::app::NodeInfo)> {
        // Prefer matching by known dataflow id/name if provided.
        if let Some(dataflow) = self.find_dataflow(app_state) {
            if let Some(node) = dataflow.nodes.iter().find(|node| self.matches_node(node)) {
                return Some((dataflow, node));
            }
        }

        // Fallback to searching globally in case the dataflow id is stale.
        app_state.dataflows.iter().find_map(|df| {
            df.nodes
                .iter()
                .find(|node| self.matches_node(node))
                .map(|node| (df, node))
        })
    }

    fn find_dataflow<'a>(&self, app_state: &'a AppState) -> Option<&'a DataflowInfo> {
        if self.state.dataflow_id.is_empty() {
            return None;
        }

        app_state
            .dataflows
            .iter()
            .find(|df| df.id == self.state.dataflow_id || df.name == self.state.dataflow_id)
    }

    fn matches_node(&self, node: &crate::tui::app::NodeInfo) -> bool {
        node.id == self.state.node_id || node.name == self.state.node_id
    }

    fn find_node_info<'a>(&self, app_state: &'a AppState) -> Option<&'a crate::tui::app::NodeInfo> {
        self.find_node(app_state).map(|(_, node)| node)
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

    async fn update(&mut self, app_state: &mut AppState) -> Result<()> {
        let previous = app_state
            .node_telemetry_sample(&self.state.dataflow_id, &self.state.node_id)
            .map(|sample| &sample.current);
        let metrics = self.compute_node_metrics(app_state, previous);
        app_state.store_node_metrics(&self.state.dataflow_id, &self.state.node_id, metrics);
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

fn format_uptime(seconds: u64) -> String {
    let hours = seconds / 3600;
    let minutes = (seconds % 3600) / 60;
    let secs = seconds % 60;

    if hours > 0 {
        format!("{hours}h {minutes}m {secs}s")
    } else if minutes > 0 {
        format!("{minutes}m {secs}s")
    } else {
        format!("{secs}s")
    }
}

fn format_elapsed(duration: Duration) -> String {
    let secs = duration.as_secs();
    if secs < 60 {
        format!("{secs}s ago")
    } else if secs < 3600 {
        let minutes = secs / 60;
        let seconds = secs % 60;
        format!("{minutes}m {seconds}s ago")
    } else {
        let hours = secs / 3600;
        let minutes = (secs % 3600) / 60;
        format!("{hours}h {minutes}m ago")
    }
}

fn format_trend(delta: Option<f64>) -> String {
    match delta {
        Some(change) if change.abs() >= 0.05 => {
            if change > 0.0 {
                format!("↑ {:+.2}", change)
            } else {
                format!("↓ {:+.2}", change)
            }
        }
        Some(_) => "stable".to_string(),
        None => "collecting…".to_string(),
    }
}

fn smooth_metric(previous: f64, current: f64) -> f64 {
    if previous == 0.0 {
        current
    } else {
        (previous * 0.6) + (current * 0.4)
    }
}

fn format_latency_delta(delta: Option<f64>) -> String {
    match delta {
        Some(change) if change.abs() >= 0.05 => format!("{:+.2} ms", change),
        Some(_) => "stable".to_string(),
        None => "collecting…".to_string(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_node_inspector_view_creation() {
        let theme = ThemeConfig::default();
        let view = NodeInspectorView::new(&theme, "df-test".to_string(), "test-node".to_string());

        assert_eq!(view.state.node_id, "test-node");
        assert_eq!(view.state.dataflow_id, "df-test");
        assert_eq!(view.state.active_tab, InspectorTab::Overview);
    }

    #[test]
    fn test_format_uptime() {
        assert_eq!(format_uptime(30), "30s");
        assert_eq!(format_uptime(90), "1m 30s");
        assert_eq!(format_uptime(3661), "1h 1m 1s");
    }

    #[test]
    fn test_format_trend() {
        assert_eq!(format_trend(Some(0.2)), "↑ +0.20");
        assert_eq!(format_trend(Some(-0.6)), "↓ -0.60");
        assert_eq!(format_trend(Some(0.01)), "stable");
        assert_eq!(format_trend(None), "collecting…");
    }

    #[test]
    fn test_format_latency_delta() {
        assert_eq!(format_latency_delta(Some(-1.5)), "-1.50 ms");
        assert_eq!(format_latency_delta(Some(0.02)), "stable");
        assert_eq!(format_latency_delta(None), "collecting…");
    }
}
