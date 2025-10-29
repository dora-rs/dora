/// Dataflow List Component - reusable dataflow listing widget
use ratatui::{
    Frame,
    layout::Rect,
    style::Style,
    widgets::{Block, Borders, List, ListItem, ListState},
};
use std::time::Instant;

use crate::tui::{
    Result,
    app::{AppState, DataflowInfo},
    theme::ThemeConfig,
    views::ViewAction,
};

use super::{Component, ComponentEvent, ComponentType};

/// Dataflow list sorting options
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DataflowSortBy {
    Name,
    Status,
    NodeCount,
    Uptime,
}

/// Dataflow list filtering options
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DataflowFilter {
    All,
    Running,
    Stopped,
    Failed,
}

/// Reusable dataflow list component
pub struct DataflowListComponent {
    selected_index: usize,
    focused: bool,
    last_update: Option<Instant>,
    sort_by: DataflowSortBy,
    filter: DataflowFilter,
}

impl DataflowListComponent {
    pub fn new() -> Self {
        Self {
            selected_index: 0,
            focused: false,
            last_update: None,
            sort_by: DataflowSortBy::Name,
            filter: DataflowFilter::All,
        }
    }

    pub fn selected_dataflow<'a>(&self, dataflows: &'a [DataflowInfo]) -> Option<&'a DataflowInfo> {
        dataflows.get(self.selected_index)
    }

    pub fn selected_index(&self) -> usize {
        self.selected_index
    }

    fn render_dataflow_item(
        &self,
        dataflow: &DataflowInfo,
        selected: bool,
        theme: &ThemeConfig,
    ) -> ListItem {
        let status_icon = match dataflow.status.as_str() {
            "running" => "🟢",
            "stopped" => "🔴",
            "starting" => "🟡",
            "stopping" => "🟠",
            "failed" => "❌",
            _ => "⚪",
        };

        let style = if selected {
            theme.styles.selection_style
        } else {
            Style::default().fg(theme.colors.text)
        };

        let content = format!(
            "{} {} | Nodes: {}",
            status_icon,
            dataflow.name,
            dataflow.nodes.len()
        );

        ListItem::new(content).style(style)
    }

    fn move_selection_up(&mut self) {
        if self.selected_index > 0 {
            self.selected_index -= 1;
        }
    }

    fn move_selection_down(&mut self, max: usize) {
        if self.selected_index + 1 < max {
            self.selected_index += 1;
        }
    }

    fn build_view_list<'a>(&self, dataflows: &'a [DataflowInfo]) -> Vec<&'a DataflowInfo> {
        let mut filtered: Vec<&DataflowInfo> = dataflows
            .iter()
            .filter(|df| match self.filter {
                DataflowFilter::All => true,
                DataflowFilter::Running => df.status.eq_ignore_ascii_case("running"),
                DataflowFilter::Stopped => df.status.eq_ignore_ascii_case("stopped"),
                DataflowFilter::Failed => df.status.eq_ignore_ascii_case("failed"),
            })
            .collect();

        match self.sort_by {
            DataflowSortBy::Name => filtered.sort_by(|a, b| a.name.cmp(&b.name)),
            DataflowSortBy::Status => filtered.sort_by(|a, b| a.status.cmp(&b.status)),
            DataflowSortBy::NodeCount => filtered.sort_by_key(|df| df.nodes.len()),
            DataflowSortBy::Uptime => filtered.sort_by_key(|df| df.status.clone()),
        }

        filtered
    }
}

impl Default for DataflowListComponent {
    fn default() -> Self {
        Self::new()
    }
}

impl Component for DataflowListComponent {
    fn update<'a>(
        &'a mut self,
        app_state: &'a AppState,
    ) -> std::pin::Pin<Box<dyn std::future::Future<Output = Result<()>> + Send + 'a>> {
        Box::pin(async move {
            // Ensure selected index is valid
            let display = self.build_view_list(&app_state.dataflows);
            if display.is_empty() {
                self.selected_index = 0;
            } else if self.selected_index >= display.len() {
                self.selected_index = display.len() - 1;
            }

            self.last_update = Some(Instant::now());
            Ok(())
        })
    }

    fn render(&self, frame: &mut Frame, area: Rect, theme: &ThemeConfig, app_state: &AppState) {
        let block = Block::default()
            .title("Dataflows")
            .borders(Borders::ALL)
            .border_style(if self.focused {
                Style::default().fg(theme.colors.primary)
            } else {
                Style::default().fg(theme.colors.border)
            });

        let display = self.build_view_list(&app_state.dataflows);
        let items: Vec<ListItem> = display
            .iter()
            .enumerate()
            .map(|(i, dataflow)| {
                self.render_dataflow_item(dataflow, i == self.selected_index, theme)
            })
            .collect();

        let list = List::new(items)
            .block(block)
            .highlight_style(theme.styles.highlight_style)
            .highlight_symbol("▶ ");

        let mut state = ListState::default().with_selected(Some(self.selected_index));
        frame.render_stateful_widget(list, area, &mut state);
    }

    fn handle_event<'a>(
        &'a mut self,
        event: ComponentEvent,
        app_state: &'a AppState,
    ) -> std::pin::Pin<Box<dyn std::future::Future<Output = Result<ViewAction>> + Send + 'a>> {
        Box::pin(async move {
            if !self.focused {
                return Ok(ViewAction::None);
            }

            match event {
                ComponentEvent::Key(key_event) => match key_event.code {
                    crossterm::event::KeyCode::Up | crossterm::event::KeyCode::Char('k') => {
                        self.move_selection_up();
                    }
                    crossterm::event::KeyCode::Down | crossterm::event::KeyCode::Char('j') => {
                        self.move_selection_down(app_state.dataflows.len());
                    }
                    crossterm::event::KeyCode::Enter => {
                        if let Some(selected) = self.selected_dataflow(&app_state.dataflows) {
                            return Ok(ViewAction::ShowStatus(format!(
                                "Selected dataflow: {}",
                                selected.name
                            )));
                        }
                    }
                    crossterm::event::KeyCode::Char('r') => {
                        return Ok(ViewAction::Refresh);
                    }
                    _ => {}
                },
                _ => {}
            }

            Ok(ViewAction::None)
        })
    }

    fn component_type(&self) -> ComponentType {
        ComponentType::DataflowList
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
