use crossterm::event::{KeyCode, KeyEvent};
use ratatui::{
    Frame,
    layout::{Alignment, Constraint, Direction, Layout, Rect},
    style::{Modifier, Style},
    text::{Line, Span},
    widgets::{Cell, List, ListItem, Paragraph, Row, Table},
};
use std::time::Duration;

use super::{
    BaseView, DashboardState, DataflowSummary, DiskUsage, MemoryUsage, NetworkActivity,
    RefreshManager, StateUpdate, SystemOverview, SystemStatus, View, ViewAction, utils,
};
use crate::tui::{
    Result,
    app::{AppState, NodeInfo, ViewType},
    components::{Component, DataflowSummaryComponent, SystemOverviewComponent},
    theme::ThemeConfig,
};

pub struct DashboardView {
    base: BaseView,
    theme: ThemeConfig,
    selected_dataflow: usize,
    show_system_info: bool,
    // Issue #24 additions
    dashboard_state: DashboardState,
    system_overview_component: SystemOverviewComponent,
    dataflow_summary_component: DataflowSummaryComponent,
    refresh_manager: RefreshManager,
}

impl DashboardView {
    pub fn new(theme: &ThemeConfig) -> Self {
        Self {
            base: BaseView::new("Dashboard".to_string()).with_auto_refresh(Duration::from_secs(2)), // Issue #24: 2-second refresh
            theme: theme.clone(),
            selected_dataflow: 0,
            show_system_info: true,
            // Issue #24 additions
            dashboard_state: DashboardState::default(),
            system_overview_component: SystemOverviewComponent::new(),
            dataflow_summary_component: DataflowSummaryComponent::new(),
            refresh_manager: RefreshManager::new(Duration::from_secs(2)),
        }
    }

    /// Collect system overview data (Issue #24)
    fn collect_system_overview(&self, app_state: &AppState) -> SystemOverview {
        let metrics = &app_state.system_metrics;

        // Determine system status based on daemon connectivity
        let status = if app_state.dataflows.is_empty() {
            SystemStatus::Disconnected
        } else {
            SystemStatus::Connected
        };

        let memory_total_mb = bytes_to_megabytes(metrics.memory.total_bytes);
        let memory_used_mb = bytes_to_megabytes(metrics.memory.used_bytes);

        let disk_total_gb = bytes_to_gigabytes(metrics.disk.total_bytes);
        let disk_used_gb = bytes_to_gigabytes(metrics.disk.used_bytes);

        let network_rx = metrics.network.received_per_second as u64;
        let network_tx = metrics.network.transmitted_per_second as u64;

        SystemOverview {
            status,
            uptime: metrics.uptime,
            version: env!("CARGO_PKG_VERSION").to_string(),
            cpu_usage: metrics.cpu_usage as f64,
            memory_usage: MemoryUsage {
                total_mb: memory_total_mb,
                used_mb: memory_used_mb,
                usage_percent: metrics.memory_usage as f64,
            },
            disk_usage: DiskUsage {
                total_gb: disk_total_gb,
                used_gb: disk_used_gb,
                usage_percent: metrics.disk.usage_percent as f64,
            },
            network_activity: NetworkActivity {
                bytes_received: network_rx,
                bytes_transmitted: network_tx,
            },
            active_connections: app_state.dataflows.len() as u32,
        }
    }

    /// Collect dataflow summary data (Issue #24)
    fn collect_dataflow_summary(&self, app_state: &AppState) -> DataflowSummary {
        let total = app_state.dataflows.len() as u32;
        let running = app_state
            .dataflows
            .iter()
            .filter(|df| df.status == "running")
            .count() as u32;
        let failed = app_state
            .dataflows
            .iter()
            .filter(|df| df.status == "failed")
            .count() as u32;
        let stopped = total - running - failed;

        let total_nodes = app_state
            .dataflows
            .iter()
            .map(|df| df.nodes.len() as u32)
            .sum();

        let healthy_nodes = app_state
            .dataflows
            .iter()
            .flat_map(|df| &df.nodes)
            .filter(|node| node.status == "running" || node.status == "healthy")
            .count() as u32;

        let unhealthy_nodes = total_nodes - healthy_nodes;

        DataflowSummary {
            total_dataflows: total,
            running_dataflows: running,
            failed_dataflows: failed,
            stopped_dataflows: stopped,
            total_nodes,
            healthy_nodes,
            unhealthy_nodes,
            recent_deployments: Vec::new(), // TODO: Track deployments
        }
    }

    /// Update dashboard data if refresh is needed (Issue #24)
    fn refresh_dashboard_data(&mut self, app_state: &AppState) {
        if self.refresh_manager.should_refresh() {
            // Collect fresh data
            let system_overview = self.collect_system_overview(app_state);
            let dataflow_summary = self.collect_dataflow_summary(app_state);

            // Update dashboard state
            self.dashboard_state.system_overview = system_overview.clone();
            self.dashboard_state.dataflow_summary = dataflow_summary.clone();

            // Update components
            self.system_overview_component.set_data(system_overview);
            self.dataflow_summary_component.set_data(dataflow_summary);

            self.refresh_manager.mark_refreshed();
        }
    }

    fn render_overview(&self, f: &mut Frame, area: Rect, app_state: &AppState) {
        let dataflow_count = app_state.dataflows.len();
        let running_count = app_state
            .dataflows
            .iter()
            .filter(|df| df.status == "running")
            .count();

        let overview_text = vec![
            Line::from(vec![
                Span::styled(
                    "Total Dataflows: ",
                    Style::default().fg(self.theme.colors.text),
                ),
                Span::styled(
                    dataflow_count.to_string(),
                    Style::default()
                        .fg(self.theme.colors.primary)
                        .add_modifier(Modifier::BOLD),
                ),
            ]),
            Line::from(vec![
                Span::styled("Running: ", Style::default().fg(self.theme.colors.text)),
                Span::styled(
                    running_count.to_string(),
                    Style::default()
                        .fg(self.theme.colors.success)
                        .add_modifier(Modifier::BOLD),
                ),
            ]),
            Line::from(vec![
                Span::styled("Stopped: ", Style::default().fg(self.theme.colors.text)),
                Span::styled(
                    (dataflow_count - running_count).to_string(),
                    Style::default()
                        .fg(self.theme.colors.error)
                        .add_modifier(Modifier::BOLD),
                ),
            ]),
        ];

        let overview = Paragraph::new(overview_text)
            .block(self.theme.styled_block("Overview"))
            .alignment(Alignment::Left);

        f.render_widget(overview, area);
    }

    fn render_dataflows(&self, f: &mut Frame, area: Rect, app_state: &AppState) {
        if app_state.dataflows.is_empty() {
            let empty_msg = Paragraph::new(
                "No dataflows found. Use 'dora start <dataflow.yaml>' to start one.",
            )
            .block(self.theme.styled_block("Dataflows"))
            .style(Style::default().fg(self.theme.colors.muted))
            .alignment(Alignment::Center);
            f.render_widget(empty_msg, area);
            return;
        }

        let header_cells = ["Name", "Status", "Nodes", "Actions"]
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
            .enumerate()
            .map(|(i, dataflow)| {
                let style = if i == self.selected_dataflow {
                    self.theme.styles.selection_style
                } else {
                    Style::default().fg(self.theme.colors.text)
                };

                let status_style = self.theme.status_style(&dataflow.status);
                let status_indicator = utils::status_indicator(&dataflow.status);

                Row::new(vec![
                    Cell::from(dataflow.name.clone()),
                    Cell::from(Line::from(vec![
                        status_indicator,
                        Span::raw(" "),
                        Span::styled(dataflow.status.clone(), status_style),
                    ])),
                    Cell::from(dataflow.nodes.len().to_string()),
                    Cell::from("inspect | logs | stop"),
                ])
                .style(style)
            })
            .collect::<Vec<_>>();

        let widths = [
            Constraint::Percentage(30),
            Constraint::Percentage(20),
            Constraint::Percentage(15),
            Constraint::Percentage(35),
        ];

        let table = Table::new(rows, widths)
            .header(header)
            .block(self.theme.styled_block("Dataflows"))
            .highlight_style(self.theme.styles.selection_style);

        f.render_widget(table, area);
    }

    fn render_system_metrics(&self, f: &mut Frame, area: Rect, app_state: &AppState) {
        if !self.show_system_info {
            return;
        }

        // Issue #24: Use SystemOverviewComponent
        self.system_overview_component
            .render(f, area, &self.theme, app_state);
    }

    fn render_quick_actions(&self, f: &mut Frame, area: Rect) {
        let actions = vec![
            "1: Dashboard",
            "2: Dataflows",
            "3: Monitor",
            "4: Logs",
            "5: Settings",
            "",
            "↑↓: Navigate",
            "Enter: Inspect",
            "Space: Toggle",
            ":: Command",
            "q: Quit",
        ];

        let list_items: Vec<ListItem> = actions
            .into_iter()
            .map(|action| {
                if action.is_empty() {
                    ListItem::new(Line::from(""))
                } else if action.contains(':') && !action.starts_with(char::is_numeric) {
                    ListItem::new(Line::from(action))
                        .style(Style::default().fg(self.theme.colors.muted))
                } else {
                    ListItem::new(Line::from(action))
                        .style(Style::default().fg(self.theme.colors.primary))
                }
            })
            .collect();

        let actions_list = List::new(list_items)
            .block(self.theme.styled_block("Quick Actions"))
            .style(Style::default().fg(self.theme.colors.text));

        f.render_widget(actions_list, area);
    }
}

fn select_preferred_node(nodes: &[NodeInfo]) -> Option<&NodeInfo> {
    nodes
        .iter()
        .find(|node| {
            let status = node.status.to_ascii_lowercase();
            status == "running" || status == "active"
        })
        .or_else(|| nodes.first())
}

fn bytes_to_megabytes(bytes: u64) -> u64 {
    bytes.saturating_div(1024 * 1024)
}

fn bytes_to_gigabytes(bytes: u64) -> u64 {
    bytes.saturating_div(1024 * 1024 * 1024)
}

impl View for DashboardView {
    fn render(&mut self, f: &mut Frame, area: Rect, app_state: &AppState) {
        let chunks = Layout::default()
            .direction(Direction::Vertical)
            .constraints([
                Constraint::Length(5), // Overview
                Constraint::Min(8),    // Dataflows table
            ])
            .split(area);

        let top_chunks = Layout::default()
            .direction(Direction::Horizontal)
            .constraints([
                Constraint::Percentage(60), // Overview
                Constraint::Percentage(40), // System metrics
            ])
            .split(chunks[0]);

        let bottom_chunks = Layout::default()
            .direction(Direction::Horizontal)
            .constraints([
                Constraint::Percentage(75), // Dataflows
                Constraint::Percentage(25), // Quick actions
            ])
            .split(chunks[1]);

        self.render_overview(f, top_chunks[0], app_state);
        self.render_system_metrics(f, top_chunks[1], app_state);
        self.render_dataflows(f, bottom_chunks[0], app_state);
        self.render_quick_actions(f, bottom_chunks[1]);
    }

    async fn handle_key(&mut self, key: KeyEvent, app_state: &mut AppState) -> Result<ViewAction> {
        match key.code {
            KeyCode::Up | KeyCode::Char('k') => {
                if self.selected_dataflow > 0 {
                    self.selected_dataflow -= 1;
                }
                Ok(ViewAction::None)
            }

            KeyCode::Down | KeyCode::Char('j') => {
                if self.selected_dataflow < app_state.dataflows.len().saturating_sub(1) {
                    self.selected_dataflow += 1;
                }
                Ok(ViewAction::None)
            }

            KeyCode::Enter => {
                if let Some(dataflow) = app_state.dataflows.get(self.selected_dataflow) {
                    if let Some(node) = select_preferred_node(&dataflow.nodes) {
                        Ok(ViewAction::PushView(ViewType::NodeInspector {
                            dataflow_id: dataflow.id.clone(),
                            node_id: node.id.clone(),
                        }))
                    } else {
                        Ok(ViewAction::ShowStatus(
                            "Selected dataflow has no nodes to inspect".to_string(),
                        ))
                    }
                } else {
                    Ok(ViewAction::None)
                }
            }

            KeyCode::Char(' ') => {
                if let Some(dataflow) = app_state.dataflows.get(self.selected_dataflow) {
                    let command = if dataflow.status == "running" {
                        format!("stop {}", dataflow.id)
                    } else {
                        format!("start {}", dataflow.id)
                    };
                    Ok(ViewAction::ExecuteCommand(command))
                } else {
                    Ok(ViewAction::None)
                }
            }

            KeyCode::Char('l') => {
                if let Some(dataflow) = app_state.dataflows.get(self.selected_dataflow) {
                    Ok(ViewAction::PushView(ViewType::LogViewer {
                        target: dataflow.id.clone(),
                    }))
                } else {
                    Ok(ViewAction::None)
                }
            }

            KeyCode::Char('r') => Ok(ViewAction::UpdateState(StateUpdate::RefreshDataflows)),

            KeyCode::Char('s') => {
                self.show_system_info = !self.show_system_info;
                Ok(ViewAction::ShowStatus(format!(
                    "System info {}",
                    if self.show_system_info {
                        "enabled"
                    } else {
                        "disabled"
                    }
                )))
            }

            KeyCode::Char('n') => Ok(ViewAction::ExecuteCommand("new dataflow".to_string())),

            KeyCode::Char('?') | KeyCode::F(1) => Ok(ViewAction::ShowHelp),

            // Issue #24: Enhanced navigation shortcuts
            KeyCode::Char('d') => Ok(ViewAction::PushView(ViewType::DataflowManager)),

            KeyCode::Char('p') => Ok(ViewAction::PushView(ViewType::SystemMonitor)),

            KeyCode::F(5) => {
                // Force refresh
                Ok(ViewAction::UpdateState(StateUpdate::RefreshDataflows))
            }

            _ => Ok(ViewAction::None),
        }
    }

    async fn update(&mut self, app_state: &mut AppState) -> Result<()> {
        // Issue #24: Refresh dashboard data at 2-second intervals
        self.refresh_dashboard_data(app_state);

        if self.base.needs_refresh() {
            // Request data refresh
            self.base.mark_updated();
        }
        Ok(())
    }

    fn help_text(&self) -> Vec<(&str, &str)> {
        vec![
            ("↑/k", "Move up"),
            ("↓/j", "Move down"),
            ("Enter", "Inspect selected dataflow"),
            ("Space", "Start/Stop selected dataflow"),
            ("d", "Navigate to dataflows"),           // Issue #24
            ("p", "Navigate to performance monitor"), // Issue #24
            ("l", "View logs for selected dataflow"),
            ("r", "Refresh dataflow list"),
            ("F5", "Force refresh"), // Issue #24
            ("s", "Toggle system info display"),
            ("n", "Create new dataflow"),
            ("?/F1", "Show help"),
            (":", "Enter command mode"),
            ("q", "Quit"),
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

    async fn on_mount(&mut self, app_state: &mut AppState) -> Result<()> {
        // Issue #24: Start refresh manager and collect initial data
        self.refresh_manager.start();
        self.refresh_dashboard_data(app_state);
        self.base.mark_updated();
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bytes_to_megabytes_converts() {
        assert_eq!(bytes_to_megabytes(1_048_576), 1);
        assert_eq!(bytes_to_megabytes(0), 0);
    }

    #[test]
    fn bytes_to_gigabytes_converts() {
        assert_eq!(bytes_to_gigabytes(1_073_741_824), 1);
        assert_eq!(bytes_to_gigabytes(512_000_000), 0);
    }
}
