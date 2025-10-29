use crossterm::event::{KeyCode, KeyEvent};
/// Dataflow Explorer View for Issue #25
use ratatui::{
    Frame,
    layout::{Alignment, Constraint, Direction, Layout, Rect},
    style::{Modifier, Style},
    text::{Line, Span},
    widgets::{Block, Borders, Cell, List, ListItem, Paragraph, Row, Table, Tabs},
};
use std::time::Duration;

use crate::tui::{
    Result,
    app::{AppState, ViewType},
    theme::ThemeConfig,
};

use super::{BaseView, ExplorerState, ExplorerTab, StateUpdate, View, ViewAction, ViewMode, utils};

pub struct DataflowExplorerView {
    base: BaseView,
    theme: ThemeConfig,
    /// Current explorer state
    pub state: ExplorerState,
    /// Currently selected index (for testing/internal use)
    #[doc(hidden)]
    pub selected_index: usize,
    /// Whether inspection panel is shown (for testing/internal use)
    #[doc(hidden)]
    pub show_inspection_panel: bool,
}

impl DataflowExplorerView {
    pub fn new(theme: &ThemeConfig) -> Self {
        Self {
            base: BaseView::new("Dataflow Explorer".to_string())
                .with_auto_refresh(Duration::from_secs(1)), // 1-second refresh per spec
            theme: theme.clone(),
            state: ExplorerState::new(),
            selected_index: 0,
            show_inspection_panel: false,
        }
    }

    /// Get currently selected dataflow (internal/testing use)
    #[doc(hidden)]
    pub fn get_selected_dataflow<'a>(
        &self,
        app_state: &'a AppState,
    ) -> Option<&'a crate::tui::app::DataflowInfo> {
        let filtered: Vec<_> = app_state
            .dataflows
            .iter()
            .filter(|df| self.state.show_stopped || df.status != "stopped")
            .collect();
        filtered.get(self.selected_index).copied()
    }

    fn get_selected_node<'a>(
        &self,
        app_state: &'a AppState,
    ) -> Option<(
        &'a crate::tui::app::DataflowInfo,
        &'a crate::tui::app::NodeInfo,
    )> {
        match self.state.active_tab {
            ExplorerTab::Nodes => {
                let mut remaining = self.selected_index;
                for dataflow in &app_state.dataflows {
                    for node in &dataflow.nodes {
                        if remaining == 0 {
                            return Some((dataflow, node));
                        }
                        remaining -= 1;
                    }
                }
                None
            }
            _ => {
                let dataflow = self.get_selected_dataflow(app_state)?;
                let node = select_preferred_node(&dataflow.nodes)?;
                Some((dataflow, node))
            }
        }
    }

    /// Get number of selectable items in current view (internal/testing use)
    #[doc(hidden)]
    pub fn get_item_count(&self, app_state: &AppState) -> usize {
        match self.state.active_tab {
            ExplorerTab::Overview => app_state
                .dataflows
                .iter()
                .filter(|df| self.state.show_stopped || df.status != "stopped")
                .count(),
            ExplorerTab::Nodes => app_state.dataflows.iter().flat_map(|df| &df.nodes).count(),
            _ => 0,
        }
    }

    /// Navigate selection up (internal/testing use)
    #[doc(hidden)]
    pub fn select_previous(&mut self, app_state: &AppState) {
        if self.selected_index > 0 {
            self.selected_index -= 1;
        } else {
            // Wrap to bottom
            let count = self.get_item_count(app_state);
            if count > 0 {
                self.selected_index = count - 1;
            }
        }
    }

    /// Navigate selection down (internal/testing use)
    #[doc(hidden)]
    pub fn select_next(&mut self, app_state: &AppState) {
        let count = self.get_item_count(app_state);
        if count > 0 {
            self.selected_index = (self.selected_index + 1) % count;
        }
    }

    /// Render the tab bar
    fn render_tabs(&self, f: &mut Frame, area: Rect) {
        let tab_titles: Vec<Line> = ExplorerTab::all()
            .iter()
            .map(|tab| {
                let shortcut = tab.shortcut();
                let name = tab.name();
                Line::from(vec![
                    Span::styled(shortcut, Style::default().fg(self.theme.colors.muted)),
                    Span::raw(":"),
                    Span::styled(name, Style::default().fg(self.theme.colors.text)),
                ])
            })
            .collect();

        let tabs = Tabs::new(tab_titles)
            .block(self.theme.styled_block("Tabs"))
            .select(self.state.active_tab as usize)
            .highlight_style(
                Style::default()
                    .fg(self.theme.colors.primary)
                    .add_modifier(Modifier::BOLD),
            )
            .divider(Span::raw(" │ "));

        f.render_widget(tabs, area);
    }

    /// Render the overview tab
    fn render_overview_tab(&self, f: &mut Frame, area: Rect, app_state: &AppState) {
        match self.state.view_mode {
            ViewMode::ListGrouped => self.render_grouped_list(f, area, app_state),
            ViewMode::ListFlat => self.render_flat_list(f, area, app_state),
            ViewMode::DependencyTree => self.render_dependency_tree(f, area, app_state),
        }
    }

    /// Render dataflows grouped by status
    fn render_grouped_list(&self, f: &mut Frame, area: Rect, app_state: &AppState) {
        // Group dataflows by status
        let running: Vec<_> = app_state
            .dataflows
            .iter()
            .filter(|df| df.status == "running")
            .collect();
        let stopped: Vec<_> = app_state
            .dataflows
            .iter()
            .filter(|df| df.status == "stopped")
            .collect();
        let failed: Vec<_> = app_state
            .dataflows
            .iter()
            .filter(|df| df.status == "failed")
            .collect();

        let mut items = Vec::new();

        // Running section
        if !running.is_empty() {
            items.push(ListItem::new(Line::from(vec![
                Span::styled(
                    "▼ RUNNING ",
                    Style::default()
                        .fg(self.theme.colors.success)
                        .add_modifier(Modifier::BOLD),
                ),
                Span::styled(
                    format!("({})", running.len()),
                    Style::default().fg(self.theme.colors.muted),
                ),
            ])));

            for df in running {
                let status_indicator = utils::status_indicator(&df.status);
                items.push(ListItem::new(Line::from(vec![
                    Span::raw("  "),
                    status_indicator,
                    Span::raw(" "),
                    Span::styled(&df.name, Style::default().fg(self.theme.colors.text)),
                    Span::raw(" "),
                    Span::styled(
                        format!("({} nodes)", df.nodes.len()),
                        Style::default().fg(self.theme.colors.muted),
                    ),
                ])));
            }
        }

        // Failed section
        if !failed.is_empty() {
            items.push(ListItem::new(""));
            items.push(ListItem::new(Line::from(vec![
                Span::styled(
                    "▼ FAILED ",
                    Style::default()
                        .fg(self.theme.colors.error)
                        .add_modifier(Modifier::BOLD),
                ),
                Span::styled(
                    format!("({})", failed.len()),
                    Style::default().fg(self.theme.colors.muted),
                ),
            ])));

            for df in failed {
                let status_indicator = utils::status_indicator(&df.status);
                items.push(ListItem::new(Line::from(vec![
                    Span::raw("  "),
                    status_indicator,
                    Span::raw(" "),
                    Span::styled(&df.name, Style::default().fg(self.theme.colors.error)),
                    Span::raw(" "),
                    Span::styled(
                        format!("({} nodes)", df.nodes.len()),
                        Style::default().fg(self.theme.colors.muted),
                    ),
                ])));
            }
        }

        // Stopped section
        if self.state.show_stopped && !stopped.is_empty() {
            items.push(ListItem::new(""));
            items.push(ListItem::new(Line::from(vec![
                Span::styled(
                    "▼ STOPPED ",
                    Style::default()
                        .fg(self.theme.colors.muted)
                        .add_modifier(Modifier::BOLD),
                ),
                Span::styled(
                    format!("({})", stopped.len()),
                    Style::default().fg(self.theme.colors.muted),
                ),
            ])));

            for df in stopped {
                let status_indicator = utils::status_indicator(&df.status);
                items.push(ListItem::new(Line::from(vec![
                    Span::raw("  "),
                    status_indicator,
                    Span::raw(" "),
                    Span::styled(&df.name, Style::default().fg(self.theme.colors.muted)),
                    Span::raw(" "),
                    Span::styled(
                        format!("({} nodes)", df.nodes.len()),
                        Style::default().fg(self.theme.colors.muted),
                    ),
                ])));
            }
        }

        if items.is_empty() {
            let title = format!("Dataflows [{}]", self.state.view_mode.name());
            let empty_msg = Paragraph::new(
                "No dataflows found.\nUse 'dora start <dataflow.yaml>' to start one.",
            )
            .block(self.theme.styled_block(&title))
            .style(Style::default().fg(self.theme.colors.muted))
            .alignment(Alignment::Center);
            f.render_widget(empty_msg, area);
            return;
        }

        let title = format!("Dataflows [{}]", self.state.view_mode.name());
        let list = List::new(items)
            .block(self.theme.styled_block(&title))
            .style(Style::default().fg(self.theme.colors.text));

        f.render_widget(list, area);
    }

    /// Render flat list of all dataflows
    fn render_flat_list(&self, f: &mut Frame, area: Rect, app_state: &AppState) {
        if app_state.dataflows.is_empty() {
            let title = format!("Dataflows [{}]", self.state.view_mode.name());
            let empty_msg = Paragraph::new(
                "No dataflows found.\nUse 'dora start <dataflow.yaml>' to start one.",
            )
            .block(self.theme.styled_block(&title))
            .style(Style::default().fg(self.theme.colors.muted))
            .alignment(Alignment::Center);
            f.render_widget(empty_msg, area);
            return;
        }

        let header_cells = ["Status", "Name", "Nodes", "Uptime"]
            .iter()
            .map(|h| Cell::from(*h).style(self.theme.table_header_style()))
            .collect::<Vec<_>>();

        let header = Row::new(header_cells)
            .style(self.theme.table_header_style())
            .height(1)
            .bottom_margin(1);

        let rows = app_state
            .dataflows
            .iter()
            .filter(|df| self.state.show_stopped || df.status != "stopped")
            .map(|df| {
                let status_style = self.theme.status_style(&df.status);
                let status_indicator = utils::status_indicator(&df.status);

                Row::new(vec![
                    Cell::from(Line::from(vec![
                        status_indicator,
                        Span::raw(" "),
                        Span::styled(&df.status, status_style),
                    ])),
                    Cell::from(df.name.clone()),
                    Cell::from(df.nodes.len().to_string()),
                    Cell::from("--"), // TODO: Track uptime
                ])
                .style(Style::default().fg(self.theme.colors.text))
            })
            .collect::<Vec<_>>();

        let widths = [
            Constraint::Percentage(20),
            Constraint::Percentage(40),
            Constraint::Percentage(20),
            Constraint::Percentage(20),
        ];

        let title = format!("Dataflows [{}]", self.state.view_mode.name());
        let table = Table::new(rows, widths)
            .header(header)
            .block(self.theme.styled_block(&title));

        f.render_widget(table, area);
    }

    /// Render ASCII tree showing dataflow dependencies
    fn render_dependency_tree(&self, f: &mut Frame, area: Rect, app_state: &AppState) {
        if app_state.dataflows.is_empty() {
            let title = format!("Dataflows [{}]", self.state.view_mode.name());
            let empty_msg = Paragraph::new(
                "No dataflows found.\nUse 'dora start <dataflow.yaml>' to start one.",
            )
            .block(self.theme.styled_block(&title))
            .style(Style::default().fg(self.theme.colors.muted))
            .alignment(Alignment::Center);
            f.render_widget(empty_msg, area);
            return;
        }

        let mut items = Vec::new();

        for (df_idx, dataflow) in app_state.dataflows.iter().enumerate() {
            let is_last_df = df_idx == app_state.dataflows.len() - 1;
            let df_prefix = if is_last_df { "└─" } else { "├─" };
            let node_prefix_base = if is_last_df { "  " } else { "│ " };

            // Dataflow line
            let status_indicator = utils::status_indicator(&dataflow.status);
            items.push(ListItem::new(Line::from(vec![
                Span::styled(df_prefix, Style::default().fg(self.theme.colors.muted)),
                Span::raw(" "),
                status_indicator,
                Span::raw(" "),
                Span::styled(
                    &dataflow.name,
                    Style::default()
                        .fg(self.theme.colors.primary)
                        .add_modifier(Modifier::BOLD),
                ),
                Span::raw(" "),
                Span::styled(
                    format!("({} nodes)", dataflow.nodes.len()),
                    Style::default().fg(self.theme.colors.muted),
                ),
            ])));

            // Node lines
            for (node_idx, node) in dataflow.nodes.iter().enumerate() {
                let is_last_node = node_idx == dataflow.nodes.len() - 1;
                let node_prefix = if is_last_node { "└─" } else { "├─" };
                let node_status = utils::status_indicator(&node.status);

                items.push(ListItem::new(Line::from(vec![
                    Span::styled(
                        node_prefix_base,
                        Style::default().fg(self.theme.colors.muted),
                    ),
                    Span::styled(node_prefix, Style::default().fg(self.theme.colors.muted)),
                    Span::raw(" "),
                    node_status,
                    Span::raw(" "),
                    Span::styled(&node.name, Style::default().fg(self.theme.colors.text)),
                ])));
            }
        }

        let title = format!("Dataflow Tree [{}]", self.state.view_mode.name());
        let list = List::new(items)
            .block(self.theme.styled_block(&title))
            .style(Style::default().fg(self.theme.colors.text));

        f.render_widget(list, area);
    }

    /// Render the nodes tab
    fn render_nodes_tab(&self, f: &mut Frame, area: Rect, app_state: &AppState) {
        // Collect all nodes from all dataflows
        let mut all_nodes = Vec::new();
        for dataflow in &app_state.dataflows {
            for node in &dataflow.nodes {
                all_nodes.push((dataflow.name.clone(), node));
            }
        }

        if all_nodes.is_empty() {
            let empty_msg = Paragraph::new("No nodes found.")
                .block(self.theme.styled_block("All Nodes"))
                .style(Style::default().fg(self.theme.colors.muted))
                .alignment(Alignment::Center);
            f.render_widget(empty_msg, area);
            return;
        }

        let selected_idx = if all_nodes.is_empty() {
            None
        } else {
            Some(self.selected_index.min(all_nodes.len() - 1))
        };

        let header_cells = ["Dataflow", "Node", "Status"]
            .iter()
            .map(|h| Cell::from(*h).style(self.theme.table_header_style()))
            .collect::<Vec<_>>();

        let header = Row::new(header_cells)
            .style(self.theme.table_header_style())
            .height(1)
            .bottom_margin(1);

        let rows = all_nodes
            .iter()
            .enumerate()
            .map(|(idx, (df_name, node))| {
                let status_style = self.theme.status_style(&node.status);
                let status_indicator = utils::status_indicator(&node.status);
                let is_selected = selected_idx == Some(idx);

                Row::new(vec![
                    Cell::from(df_name.clone()),
                    Cell::from(node.name.clone()),
                    Cell::from(Line::from(vec![
                        status_indicator,
                        Span::raw(" "),
                        Span::styled(&node.status, status_style),
                    ])),
                ])
                .style(self.theme.table_row_style(is_selected))
            })
            .collect::<Vec<_>>();

        let widths = [
            Constraint::Percentage(35),
            Constraint::Percentage(40),
            Constraint::Percentage(25),
        ];

        let table = Table::new(rows, widths)
            .header(header)
            .block(self.theme.styled_block("All Nodes"));

        f.render_widget(table, area);
    }

    /// Render the performance tab
    fn render_performance_tab(&self, f: &mut Frame, area: Rect, app_state: &AppState) {
        let metrics = &app_state.system_metrics;

        let perf_text = vec![
            Line::from(vec![
                Span::styled("CPU Usage: ", Style::default().fg(self.theme.colors.text)),
                Span::styled(
                    format!("{:.1}%", metrics.cpu_usage),
                    Style::default()
                        .fg(self.theme.colors.primary)
                        .add_modifier(Modifier::BOLD),
                ),
            ]),
            Line::from(vec![
                Span::styled(
                    "Memory Usage: ",
                    Style::default().fg(self.theme.colors.text),
                ),
                Span::styled(
                    format!("{:.1}%", metrics.memory_usage),
                    Style::default()
                        .fg(self.theme.colors.primary)
                        .add_modifier(Modifier::BOLD),
                ),
            ]),
            Line::from(vec![
                Span::styled("Network I/O: ", Style::default().fg(self.theme.colors.text)),
                Span::styled(
                    format!(
                        "↓{} ↑{}",
                        utils::format_bytes(metrics.network_io.0),
                        utils::format_bytes(metrics.network_io.1)
                    ),
                    Style::default().fg(self.theme.colors.muted),
                ),
            ]),
            Line::from(""),
            Line::from(Span::styled(
                "Per-Dataflow Performance",
                Style::default()
                    .fg(self.theme.colors.primary)
                    .add_modifier(Modifier::BOLD),
            )),
        ];

        let mut all_lines = perf_text;

        for dataflow in &app_state.dataflows {
            all_lines.push(Line::from(""));
            all_lines.push(Line::from(vec![
                Span::styled(
                    &dataflow.name,
                    Style::default()
                        .fg(self.theme.colors.text)
                        .add_modifier(Modifier::BOLD),
                ),
                Span::raw(" - "),
                Span::styled(
                    format!("{} nodes", dataflow.nodes.len()),
                    Style::default().fg(self.theme.colors.muted),
                ),
            ]));

            // TODO: Add per-dataflow metrics when available
            all_lines.push(Line::from(vec![
                Span::raw("  "),
                Span::styled(
                    "Metrics not yet available",
                    Style::default().fg(self.theme.colors.muted),
                ),
            ]));
        }

        let paragraph = Paragraph::new(all_lines)
            .block(self.theme.styled_block("Performance Metrics"))
            .alignment(Alignment::Left);

        f.render_widget(paragraph, area);
    }

    /// Render inspection panel for selected dataflow
    fn render_inspection_panel(&self, f: &mut Frame, area: Rect, app_state: &AppState) {
        if let Some(dataflow) = self.get_selected_dataflow(app_state) {
            let mut lines = vec![
                Line::from(vec![
                    Span::styled("Dataflow: ", Style::default().fg(self.theme.colors.muted)),
                    Span::styled(
                        &dataflow.name,
                        Style::default()
                            .fg(self.theme.colors.primary)
                            .add_modifier(Modifier::BOLD),
                    ),
                ]),
                Line::from(""),
                Line::from(vec![
                    Span::styled("ID: ", Style::default().fg(self.theme.colors.muted)),
                    Span::styled(&dataflow.id, Style::default().fg(self.theme.colors.text)),
                ]),
                Line::from(vec![
                    Span::styled("Status: ", Style::default().fg(self.theme.colors.muted)),
                    utils::status_indicator(&dataflow.status),
                    Span::raw(" "),
                    Span::styled(&dataflow.status, self.theme.status_style(&dataflow.status)),
                ]),
                Line::from(vec![
                    Span::styled("Nodes: ", Style::default().fg(self.theme.colors.muted)),
                    Span::styled(
                        dataflow.nodes.len().to_string(),
                        Style::default().fg(self.theme.colors.text),
                    ),
                ]),
                Line::from(""),
                Line::from(Span::styled(
                    "Nodes:",
                    Style::default()
                        .fg(self.theme.colors.primary)
                        .add_modifier(Modifier::BOLD),
                )),
            ];

            for node in &dataflow.nodes {
                lines.push(Line::from(vec![
                    Span::raw("  "),
                    utils::status_indicator(&node.status),
                    Span::raw(" "),
                    Span::styled(&node.name, Style::default().fg(self.theme.colors.text)),
                ]));
            }

            lines.push(Line::from(""));
            lines.push(Line::from(Span::styled(
                "Actions:",
                Style::default()
                    .fg(self.theme.colors.primary)
                    .add_modifier(Modifier::BOLD),
            )));
            lines.push(Line::from("  s - Start/Stop"));
            lines.push(Line::from("  r - Restart"));
            lines.push(Line::from("  i - Detailed inspect"));
            lines.push(Line::from("  l - View logs"));

            let panel = Paragraph::new(lines)
                .block(self.theme.styled_block("Inspector"))
                .wrap(ratatui::widgets::Wrap { trim: false });

            f.render_widget(panel, area);
        } else {
            let msg = Paragraph::new("No dataflow selected\n\nUse ↑↓ to select")
                .block(self.theme.styled_block("Inspector"))
                .style(Style::default().fg(self.theme.colors.muted))
                .alignment(Alignment::Center);
            f.render_widget(msg, area);
        }
    }

    /// Render configuration tab
    fn render_configuration_tab(&self, f: &mut Frame, area: Rect, app_state: &AppState) {
        if let Some(dataflow) = self.get_selected_dataflow(app_state) {
            let config_text = vec![
                Line::from(vec![
                    Span::styled(
                        "Configuration: ",
                        Style::default()
                            .fg(self.theme.colors.primary)
                            .add_modifier(Modifier::BOLD),
                    ),
                    Span::styled(&dataflow.name, Style::default().fg(self.theme.colors.text)),
                ]),
                Line::from(""),
                Line::from(Span::styled(
                    "# Dataflow Configuration",
                    Style::default().fg(self.theme.colors.muted),
                )),
                Line::from(""),
                Line::from(format!("id: {}", dataflow.id)),
                Line::from(format!("name: {}", dataflow.name)),
                Line::from(format!("status: {}", dataflow.status)),
                Line::from(""),
                Line::from(Span::styled(
                    "nodes:",
                    Style::default().fg(self.theme.colors.primary),
                )),
            ];

            let mut all_lines = config_text;
            for node in &dataflow.nodes {
                all_lines.push(Line::from(format!("  - id: {}", node.id)));
                all_lines.push(Line::from(format!("    name: {}", node.name)));
                all_lines.push(Line::from(format!("    status: {}", node.status)));
                all_lines.push(Line::from(""));
            }

            all_lines.push(Line::from(""));
            all_lines.push(Line::from(Span::styled(
                "Note: Full YAML configuration requires daemon integration",
                Style::default().fg(self.theme.colors.muted),
            )));

            let paragraph = Paragraph::new(all_lines)
                .block(self.theme.styled_block("Configuration"))
                .wrap(ratatui::widgets::Wrap { trim: false });

            f.render_widget(paragraph, area);
        } else {
            let msg = Paragraph::new("Select a dataflow to view configuration\n\nUse ↑↓ to select")
                .block(self.theme.styled_block("Configuration"))
                .style(Style::default().fg(self.theme.colors.muted))
                .alignment(Alignment::Center);
            f.render_widget(msg, area);
        }
    }

    /// Render logs tab
    fn render_logs_tab(&self, f: &mut Frame, area: Rect, app_state: &AppState) {
        if let Some(dataflow) = self.get_selected_dataflow(app_state) {
            let log_lines = vec![
                Line::from(vec![
                    Span::styled("[INFO] ", Style::default().fg(self.theme.colors.success)),
                    Span::raw(format!("Dataflow '{}' initialized", dataflow.name)),
                ]),
                Line::from(vec![
                    Span::styled("[INFO] ", Style::default().fg(self.theme.colors.success)),
                    Span::raw(format!("Loading {} nodes...", dataflow.nodes.len())),
                ]),
            ];

            let mut all_lines = log_lines;
            for node in &dataflow.nodes {
                all_lines.push(Line::from(vec![
                    Span::styled("[INFO] ", Style::default().fg(self.theme.colors.success)),
                    Span::raw(format!("Node '{}': {}", node.name, node.status)),
                ]));
            }

            all_lines.push(Line::from(""));
            all_lines.push(Line::from(vec![
                Span::styled("[INFO] ", Style::default().fg(self.theme.colors.muted)),
                Span::raw("Live log streaming requires daemon connection"),
            ]));
            all_lines.push(Line::from(vec![
                Span::styled("[TIP]  ", Style::default().fg(self.theme.colors.primary)),
                Span::raw("Press 'l' for full log viewer"),
            ]));

            let title = format!("Logs - {}", dataflow.name);
            let paragraph = Paragraph::new(all_lines)
                .block(self.theme.styled_block(&title))
                .wrap(ratatui::widgets::Wrap { trim: false });

            f.render_widget(paragraph, area);
        } else {
            let msg = Paragraph::new("Select a dataflow to view logs\n\nUse ↑↓ to select")
                .block(self.theme.styled_block("Logs"))
                .style(Style::default().fg(self.theme.colors.muted))
                .alignment(Alignment::Center);
            f.render_widget(msg, area);
        }
    }

    /// Render debug tab
    fn render_debug_tab(&self, f: &mut Frame, area: Rect, app_state: &AppState) {
        let debug_info = vec![
            Line::from(vec![Span::styled(
                "Debug Information",
                Style::default()
                    .fg(self.theme.colors.primary)
                    .add_modifier(Modifier::BOLD),
            )]),
            Line::from(""),
            Line::from(vec![Span::styled(
                "Explorer State:",
                Style::default()
                    .fg(self.theme.colors.text)
                    .add_modifier(Modifier::BOLD),
            )]),
            Line::from(format!("  Active Tab: {:?}", self.state.active_tab)),
            Line::from(format!("  View Mode: {:?}", self.state.view_mode)),
            Line::from(format!("  Selected Index: {}", self.selected_index)),
            Line::from(format!("  Show Stopped: {}", self.state.show_stopped)),
            Line::from(format!(
                "  Inspection Panel: {}",
                self.show_inspection_panel
            )),
            Line::from(""),
            Line::from(vec![Span::styled(
                "System State:",
                Style::default()
                    .fg(self.theme.colors.text)
                    .add_modifier(Modifier::BOLD),
            )]),
            Line::from(format!("  Total Dataflows: {}", app_state.dataflows.len())),
            Line::from(format!(
                "  Running: {}",
                app_state
                    .dataflows
                    .iter()
                    .filter(|df| df.status == "running")
                    .count()
            )),
            Line::from(format!(
                "  Stopped: {}",
                app_state
                    .dataflows
                    .iter()
                    .filter(|df| df.status == "stopped")
                    .count()
            )),
            Line::from(format!(
                "  Failed: {}",
                app_state
                    .dataflows
                    .iter()
                    .filter(|df| df.status == "failed")
                    .count()
            )),
            Line::from(""),
            Line::from(vec![Span::styled(
                "Metrics:",
                Style::default()
                    .fg(self.theme.colors.text)
                    .add_modifier(Modifier::BOLD),
            )]),
            Line::from(format!("  CPU: {:.1}%", app_state.system_metrics.cpu_usage)),
            Line::from(format!(
                "  Memory: {:.1}%",
                app_state.system_metrics.memory_usage
            )),
        ];

        if let Some(dataflow) = self.get_selected_dataflow(app_state) {
            let mut lines = debug_info;
            lines.push(Line::from(""));
            lines.push(Line::from(vec![Span::styled(
                "Selected Dataflow:",
                Style::default()
                    .fg(self.theme.colors.text)
                    .add_modifier(Modifier::BOLD),
            )]));
            lines.push(Line::from(format!("  Name: {}", dataflow.name)));
            lines.push(Line::from(format!("  ID: {}", dataflow.id)));
            lines.push(Line::from(format!("  Status: {}", dataflow.status)));
            lines.push(Line::from(format!("  Nodes: {}", dataflow.nodes.len())));

            let paragraph = Paragraph::new(lines)
                .block(self.theme.styled_block("Debug"))
                .wrap(ratatui::widgets::Wrap { trim: false });

            f.render_widget(paragraph, area);
        } else {
            let paragraph = Paragraph::new(debug_info)
                .block(self.theme.styled_block("Debug"))
                .wrap(ratatui::widgets::Wrap { trim: false });

            f.render_widget(paragraph, area);
        }
    }

    /// Render status bar with controls info
    fn render_status_bar(&self, f: &mut Frame, area: Rect) {
        let controls = vec![
            ("↑↓", "Navigate"),
            ("i", "Inspector"),
            ("s", "Start/Stop"),
            ("l", "Logs"),
            ("v", "View"),
            ("Enter", "Inspect"),
            ("q", "Back"),
        ];

        let control_spans: Vec<Span> = controls
            .iter()
            .flat_map(|(key, desc)| {
                vec![
                    Span::styled(
                        *key,
                        Style::default()
                            .fg(self.theme.colors.primary)
                            .add_modifier(Modifier::BOLD),
                    ),
                    Span::raw(":"),
                    Span::styled(*desc, Style::default().fg(self.theme.colors.text)),
                    Span::raw("  "),
                ]
            })
            .collect();

        let status = Paragraph::new(Line::from(control_spans))
            .block(Block::default().borders(Borders::TOP))
            .alignment(Alignment::Left);

        f.render_widget(status, area);
    }
}

impl View for DataflowExplorerView {
    fn render(&mut self, f: &mut Frame, area: Rect, app_state: &AppState) {
        let chunks = Layout::default()
            .direction(Direction::Vertical)
            .constraints([
                Constraint::Length(3), // Tab bar
                Constraint::Min(10),   // Content area
                Constraint::Length(2), // Status bar
            ])
            .split(area);

        // Render tab bar
        self.render_tabs(f, chunks[0]);

        // Split content area if inspection panel is shown
        let content_area =
            if self.show_inspection_panel && self.state.active_tab == ExplorerTab::Overview {
                let split = Layout::default()
                    .direction(Direction::Horizontal)
                    .constraints([
                        Constraint::Percentage(60), // Main content
                        Constraint::Percentage(40), // Inspection panel
                    ])
                    .split(chunks[1]);

                // Render inspection panel
                self.render_inspection_panel(f, split[1], app_state);

                split[0]
            } else {
                chunks[1]
            };

        // Render active tab content
        match self.state.active_tab {
            ExplorerTab::Overview => self.render_overview_tab(f, content_area, app_state),
            ExplorerTab::Nodes => self.render_nodes_tab(f, content_area, app_state),
            ExplorerTab::Performance => self.render_performance_tab(f, content_area, app_state),
            ExplorerTab::Configuration => self.render_configuration_tab(f, content_area, app_state),
            ExplorerTab::Logs => self.render_logs_tab(f, content_area, app_state),
            ExplorerTab::Debug => self.render_debug_tab(f, content_area, app_state),
        }

        // Render status bar
        self.render_status_bar(f, chunks[2]);
    }

    async fn handle_key(&mut self, key: KeyEvent, app_state: &mut AppState) -> Result<ViewAction> {
        match key.code {
            // Tab switching with number keys
            KeyCode::Char('1') => {
                self.state.switch_tab(ExplorerTab::Overview);
                Ok(ViewAction::None)
            }
            KeyCode::Char('2') => {
                self.state.switch_tab(ExplorerTab::Nodes);
                Ok(ViewAction::None)
            }
            KeyCode::Char('3') => {
                self.state.switch_tab(ExplorerTab::Performance);
                Ok(ViewAction::None)
            }
            KeyCode::Char('4') => {
                self.state.switch_tab(ExplorerTab::Configuration);
                Ok(ViewAction::None)
            }
            KeyCode::Char('5') => {
                self.state.switch_tab(ExplorerTab::Logs);
                Ok(ViewAction::None)
            }
            KeyCode::Char('6') => {
                self.state.switch_tab(ExplorerTab::Debug);
                Ok(ViewAction::None)
            }

            // Tab navigation
            KeyCode::Tab => {
                self.state.next_tab();
                Ok(ViewAction::None)
            }
            KeyCode::BackTab => {
                self.state.prev_tab();
                Ok(ViewAction::None)
            }

            // Navigation
            KeyCode::Up | KeyCode::Char('k') => {
                self.select_previous(app_state);
                Ok(ViewAction::None)
            }

            KeyCode::Down | KeyCode::Char('j') => {
                self.select_next(app_state);
                Ok(ViewAction::None)
            }

            // View mode cycling
            KeyCode::Char('v') => {
                self.state.cycle_view_mode();
                Ok(ViewAction::ShowStatus(format!(
                    "View mode: {}",
                    self.state.view_mode.name()
                )))
            }

            // Toggle inspection panel
            KeyCode::Char('i') => {
                self.show_inspection_panel = !self.show_inspection_panel;
                Ok(ViewAction::ShowStatus(format!(
                    "Inspection panel: {}",
                    if self.show_inspection_panel {
                        "on"
                    } else {
                        "off"
                    }
                )))
            }

            // Toggle show stopped
            KeyCode::Char('h') => {
                self.state.toggle_show_stopped();
                self.selected_index = 0; // Reset selection
                Ok(ViewAction::ShowStatus(format!(
                    "Show stopped: {}",
                    if self.state.show_stopped { "on" } else { "off" }
                )))
            }

            // Lifecycle controls
            KeyCode::Char('s') => {
                if let Some(dataflow) = self.get_selected_dataflow(app_state) {
                    let action = if dataflow.status == "running" {
                        format!("stop {}", dataflow.id)
                    } else {
                        format!("start {}", dataflow.id)
                    };
                    Ok(ViewAction::ExecuteCommand(action))
                } else {
                    Ok(ViewAction::ShowStatus("No dataflow selected".to_string()))
                }
            }

            // View logs for selected dataflow
            KeyCode::Char('l') => {
                if let Some(dataflow) = self.get_selected_dataflow(app_state) {
                    Ok(ViewAction::PushView(ViewType::LogViewer {
                        target: dataflow.id.clone(),
                    }))
                } else {
                    Ok(ViewAction::ShowStatus("No dataflow selected".to_string()))
                }
            }

            // Refresh
            KeyCode::F(5) => Ok(ViewAction::UpdateState(StateUpdate::RefreshDataflows)),

            // Help
            KeyCode::Char('?') | KeyCode::F(1) => Ok(ViewAction::ShowHelp),

            // Back/Quit
            KeyCode::Char('q') | KeyCode::Esc => Ok(ViewAction::PopView),

            // Enter to inspect selected dataflow
            KeyCode::Enter => {
                if let Some((dataflow, node)) = self.get_selected_node(app_state) {
                    Ok(ViewAction::PushView(ViewType::NodeInspector {
                        dataflow_id: dataflow.id.clone(),
                        node_id: node.id.clone(),
                    }))
                } else if self.get_selected_dataflow(app_state).is_some() {
                    Ok(ViewAction::ShowStatus(
                        "Selected dataflow has no nodes to inspect".to_string(),
                    ))
                } else {
                    Ok(ViewAction::ShowStatus("No node selected".to_string()))
                }
            }

            _ => Ok(ViewAction::None),
        }
    }

    async fn update(&mut self, _app_state: &mut AppState) -> Result<()> {
        if self.base.needs_refresh() {
            self.state.mark_refreshed();
            self.base.mark_updated();
        }
        Ok(())
    }

    fn help_text(&self) -> Vec<(&str, &str)> {
        vec![
            ("1-6", "Switch to tab"),
            ("Tab/Shift+Tab", "Next/previous tab"),
            ("↑↓ / j/k", "Navigate selection"),
            ("i", "Toggle inspection panel"),
            ("s", "Start/Stop selected dataflow"),
            ("l", "View logs for selected dataflow"),
            ("v", "Cycle view mode (grouped/flat/tree)"),
            ("h", "Toggle show stopped dataflows"),
            ("F5", "Refresh"),
            ("Enter", "Inspect selected dataflow"),
            ("?/F1", "Show help"),
            ("q/Esc", "Go back"),
        ]
    }

    fn title(&self) -> &str {
        &self.base.title
    }

    fn auto_refresh(&self) -> Option<Duration> {
        self.base.auto_refresh_interval
    }

    fn on_focus(&mut self) {
        self.base.set_focused(true);
    }

    fn on_blur(&mut self) {
        self.base.set_focused(false);
    }

    async fn on_mount(&mut self, _app_state: &mut AppState) -> Result<()> {
        self.state.mark_refreshed();
        self.base.mark_updated();
        Ok(())
    }
}

fn select_preferred_node(
    nodes: &[crate::tui::app::NodeInfo],
) -> Option<&crate::tui::app::NodeInfo> {
    nodes
        .iter()
        .find(|node| {
            let status = node.status.to_ascii_lowercase();
            status == "running" || status == "active"
        })
        .or_else(|| nodes.first())
}
