/// Log Viewer Component - reusable log display widget
use ratatui::{
    Frame,
    layout::Rect,
    style::Style,
    widgets::{Block, Borders, List, ListItem, ListState},
};
use std::collections::VecDeque;

use crate::tui::{Result, app::AppState, theme::ThemeConfig, views::ViewAction};

use super::{Component, ComponentEvent, ComponentType};

/// A single log entry
#[derive(Debug, Clone)]
pub struct LogEntry {
    pub timestamp: String,
    pub level: LogLevel,
    pub source: String,
    pub message: String,
}

/// Log severity levels
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LogLevel {
    Trace,
    Debug,
    Info,
    Warn,
    Error,
}

impl LogLevel {
    pub fn as_str(&self) -> &str {
        match self {
            LogLevel::Trace => "TRACE",
            LogLevel::Debug => "DEBUG",
            LogLevel::Info => "INFO ",
            LogLevel::Warn => "WARN ",
            LogLevel::Error => "ERROR",
        }
    }
}

/// Reusable log viewer component
pub struct LogViewerComponent {
    logs: VecDeque<LogEntry>,
    max_logs: usize,
    scroll_offset: usize,
    focused: bool,
    filter_level: Option<LogLevel>,
}

impl LogViewerComponent {
    pub fn new(max_logs: usize) -> Self {
        Self {
            logs: VecDeque::new(),
            max_logs,
            scroll_offset: 0,
            focused: false,
            filter_level: None,
        }
    }

    pub fn add_log(&mut self, entry: LogEntry) {
        self.logs.push_back(entry);

        // Trim if exceeds max
        while self.logs.len() > self.max_logs {
            self.logs.pop_front();
        }

        // Auto-scroll to bottom when new log arrives
        if !self.focused {
            self.scroll_to_bottom();
        }
    }

    pub fn scroll_up(&mut self) {
        if self.scroll_offset > 0 {
            self.scroll_offset -= 1;
        }
    }

    pub fn scroll_down(&mut self) {
        if self.scroll_offset + 1 < self.logs.len() {
            self.scroll_offset += 1;
        }
    }

    pub fn scroll_to_bottom(&mut self) {
        if !self.logs.is_empty() {
            self.scroll_offset = self.logs.len().saturating_sub(1);
        }
    }

    pub fn scroll_to_top(&mut self) {
        self.scroll_offset = 0;
    }

    fn level_style(&self, level: LogLevel, theme: &ThemeConfig) -> Style {
        match level {
            LogLevel::Trace => Style::default().fg(theme.colors.muted),
            LogLevel::Debug => Style::default().fg(theme.colors.text),
            LogLevel::Info => Style::default().fg(theme.colors.primary),
            LogLevel::Warn => Style::default().fg(theme.colors.warning),
            LogLevel::Error => Style::default().fg(theme.colors.error),
        }
    }
}

impl Default for LogViewerComponent {
    fn default() -> Self {
        Self::new(1000)
    }
}

impl Component for LogViewerComponent {
    fn update<'a>(
        &'a mut self,
        _app_state: &'a AppState,
    ) -> std::pin::Pin<Box<dyn std::future::Future<Output = Result<()>> + Send + 'a>> {
        Box::pin(async {
            // Log updates would come from a log stream
            Ok(())
        })
    }

    fn render(&self, frame: &mut Frame, area: Rect, theme: &ThemeConfig, _app_state: &AppState) {
        let block = Block::default()
            .title(format!("Logs ({}/{})", self.logs.len(), self.max_logs))
            .borders(Borders::ALL)
            .border_style(if self.focused {
                Style::default().fg(theme.colors.primary)
            } else {
                Style::default().fg(theme.colors.border)
            });

        let items: Vec<ListItem> = self
            .logs
            .iter()
            .filter(|entry| {
                if let Some(filter) = self.filter_level {
                    entry.level as u8 >= filter as u8
                } else {
                    true
                }
            })
            .map(|entry| {
                let style = self.level_style(entry.level, theme);
                let content = format!(
                    "[{}] {} {}: {}",
                    entry.timestamp,
                    entry.level.as_str(),
                    entry.source,
                    entry.message
                );
                ListItem::new(content).style(style)
            })
            .collect();

        let list = List::new(items).block(block);

        let mut state = ListState::default().with_selected(Some(self.scroll_offset));
        frame.render_stateful_widget(list, area, &mut state);
    }

    fn handle_event<'a>(
        &'a mut self,
        event: ComponentEvent,
        _app_state: &'a AppState,
    ) -> std::pin::Pin<Box<dyn std::future::Future<Output = Result<ViewAction>> + Send + 'a>> {
        Box::pin(async move {
            if !self.focused {
                return Ok(ViewAction::None);
            }

            match event {
                ComponentEvent::Key(key_event) => match key_event.code {
                    crossterm::event::KeyCode::Up | crossterm::event::KeyCode::Char('k') => {
                        self.scroll_up();
                    }
                    crossterm::event::KeyCode::Down | crossterm::event::KeyCode::Char('j') => {
                        self.scroll_down();
                    }
                    crossterm::event::KeyCode::Char('g') => {
                        self.scroll_to_top();
                    }
                    crossterm::event::KeyCode::Char('G') => {
                        self.scroll_to_bottom();
                    }
                    _ => {}
                },
                _ => {}
            }

            Ok(ViewAction::None)
        })
    }

    fn component_type(&self) -> ComponentType {
        ComponentType::LogViewer
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
