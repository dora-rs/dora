use ratatui::{
    layout::{Constraint, Direction, Layout, Rect, Alignment},
    style::{Modifier, Style},
    widgets::{Block, Borders, List, ListItem, Paragraph, Gauge, Table, Row, Cell},
    text::{Line, Span},
};
use crossterm::event::{KeyCode, KeyEvent};
use std::time::Duration;

use crate::tui::{
    app::{AppState, ViewType},
    theme::ThemeConfig,
    Frame,
    Result,
};
use super::{BaseView, View, ViewAction, StateUpdate, utils};

pub struct DashboardView {
    base: BaseView,
    theme: ThemeConfig,
    selected_dataflow: usize,
    show_system_info: bool,
}

impl DashboardView {
    pub fn new(theme: &ThemeConfig) -> Self {
        Self {
            base: BaseView::new("Dashboard".to_string())
                .with_auto_refresh(Duration::from_secs(5)),
            theme: theme.clone(),
            selected_dataflow: 0,
            show_system_info: true,
        }
    }
    
    fn render_overview(&self, f: &mut Frame, area: Rect, app_state: &AppState) {
        let dataflow_count = app_state.dataflows.len();
        let running_count = app_state.dataflows
            .iter()
            .filter(|df| df.status == "running")
            .count();
        
        let overview_text = vec![
            Line::from(vec![
                Span::styled("Total Dataflows: ", Style::default().fg(self.theme.colors.text)),
                Span::styled(dataflow_count.to_string(), Style::default().fg(self.theme.colors.primary).add_modifier(Modifier::BOLD)),
            ]),
            Line::from(vec![
                Span::styled("Running: ", Style::default().fg(self.theme.colors.text)),
                Span::styled(running_count.to_string(), Style::default().fg(self.theme.colors.success).add_modifier(Modifier::BOLD)),
            ]),
            Line::from(vec![
                Span::styled("Stopped: ", Style::default().fg(self.theme.colors.text)),
                Span::styled((dataflow_count - running_count).to_string(), Style::default().fg(self.theme.colors.error).add_modifier(Modifier::BOLD)),
            ]),
        ];
        
        let overview = Paragraph::new(overview_text)
            .block(self.theme.styled_block("Overview"))
            .alignment(Alignment::Left);
        
        f.render_widget(overview, area);
    }
    
    fn render_dataflows(&self, f: &mut Frame, area: Rect, app_state: &AppState) {
        if app_state.dataflows.is_empty() {
            let empty_msg = Paragraph::new("No dataflows found. Use 'dora start <dataflow.yaml>' to start one.")
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
        
        let rows = app_state.dataflows
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
        
        let metrics = &app_state.system_metrics;
        
        let chunks = Layout::default()
            .direction(Direction::Vertical)
            .constraints([
                Constraint::Length(3),
                Constraint::Length(3),
                Constraint::Length(3),
            ])
            .split(area);
        
        // CPU usage
        let cpu_gauge = Gauge::default()
            .block(Block::default().title("CPU Usage").borders(Borders::ALL))
            .gauge_style(self.theme.percentage_style(metrics.cpu_usage))
            .percent(metrics.cpu_usage as u16)
            .label(format!("{:.1}%", metrics.cpu_usage));
        f.render_widget(cpu_gauge, chunks[0]);
        
        // Memory usage
        let memory_gauge = Gauge::default()
            .block(Block::default().title("Memory Usage").borders(Borders::ALL))
            .gauge_style(self.theme.percentage_style(metrics.memory_usage))
            .percent(metrics.memory_usage as u16)
            .label(format!("{:.1}%", metrics.memory_usage));
        f.render_widget(memory_gauge, chunks[1]);
        
        // Network I/O
        let network_text = format!(
            "RX: {} | TX: {}",
            utils::format_bytes(metrics.network_io.0),
            utils::format_bytes(metrics.network_io.1)
        );
        let network_info = Paragraph::new(network_text)
            .block(Block::default().title("Network I/O").borders(Borders::ALL))
            .style(Style::default().fg(self.theme.colors.text))
            .alignment(Alignment::Center);
        f.render_widget(network_info, chunks[2]);
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
                    ListItem::new(Line::from(action)).style(Style::default().fg(self.theme.colors.muted))
                } else {
                    ListItem::new(Line::from(action)).style(Style::default().fg(self.theme.colors.primary))
                }
            })
            .collect();
        
        let actions_list = List::new(list_items)
            .block(self.theme.styled_block("Quick Actions"))
            .style(Style::default().fg(self.theme.colors.text));
        
        f.render_widget(actions_list, area);
    }
}

impl View for DashboardView {
    fn render(&mut self, f: &mut Frame, area: Rect, app_state: &AppState) {
        let chunks = Layout::default()
            .direction(Direction::Vertical)
            .constraints([
                Constraint::Length(5),  // Overview
                Constraint::Min(8),     // Dataflows table
            ])
            .split(area);
        
        let top_chunks = Layout::default()
            .direction(Direction::Horizontal)
            .constraints([
                Constraint::Percentage(60),  // Overview
                Constraint::Percentage(40),  // System metrics
            ])
            .split(chunks[0]);
        
        let bottom_chunks = Layout::default()
            .direction(Direction::Horizontal)
            .constraints([
                Constraint::Percentage(75),  // Dataflows
                Constraint::Percentage(25),  // Quick actions
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
            },
            
            KeyCode::Down | KeyCode::Char('j') => {
                if self.selected_dataflow < app_state.dataflows.len().saturating_sub(1) {
                    self.selected_dataflow += 1;
                }
                Ok(ViewAction::None)
            },
            
            KeyCode::Enter => {
                if let Some(dataflow) = app_state.dataflows.get(self.selected_dataflow) {
                    Ok(ViewAction::PushView(ViewType::NodeInspector {
                        node_id: dataflow.id.clone(),
                    }))
                } else {
                    Ok(ViewAction::None)
                }
            },
            
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
            },
            
            KeyCode::Char('l') => {
                if let Some(dataflow) = app_state.dataflows.get(self.selected_dataflow) {
                    Ok(ViewAction::PushView(ViewType::LogViewer {
                        target: dataflow.id.clone(),
                    }))
                } else {
                    Ok(ViewAction::None)
                }
            },
            
            KeyCode::Char('r') => {
                Ok(ViewAction::UpdateState(StateUpdate::RefreshDataflows))
            },
            
            KeyCode::Char('s') => {
                self.show_system_info = !self.show_system_info;
                Ok(ViewAction::ShowStatus(format!(
                    "System info {}",
                    if self.show_system_info { "enabled" } else { "disabled" }
                )))
            },
            
            KeyCode::Char('n') => {
                Ok(ViewAction::ExecuteCommand("new dataflow".to_string()))
            },
            
            KeyCode::Char('?') => {
                Ok(ViewAction::ShowHelp)
            },
            
            _ => Ok(ViewAction::None),
        }
    }
    
    async fn update(&mut self, app_state: &mut AppState) -> Result<()> {
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
            ("l", "View logs for selected dataflow"),
            ("r", "Refresh dataflow list"),
            ("s", "Toggle system info display"),
            ("n", "Create new dataflow"),
            ("?", "Show help"),
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
        // Refresh data when view is first mounted
        self.base.mark_updated();
        Ok(())
    }
}