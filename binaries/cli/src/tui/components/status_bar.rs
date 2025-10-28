/// Status Bar Component - displays status messages and keybindings
use ratatui::{
    Frame,
    layout::{Alignment, Rect},
    style::Style,
    text::{Line, Span},
    widgets::{Block, Borders, Paragraph},
};

use crate::tui::{Result, app::AppState, theme::ThemeConfig, views::ViewAction};

use super::{Component, ComponentEvent, ComponentType};

/// Reusable status bar component
pub struct StatusBarComponent {
    show_keybindings: bool,
}

impl StatusBarComponent {
    pub fn new() -> Self {
        Self {
            show_keybindings: true,
        }
    }

    pub fn with_keybindings(mut self, show: bool) -> Self {
        self.show_keybindings = show;
        self
    }
}

impl Default for StatusBarComponent {
    fn default() -> Self {
        Self::new()
    }
}

impl Component for StatusBarComponent {
    fn update<'a>(
        &'a mut self,
        _app_state: &'a AppState,
    ) -> std::pin::Pin<Box<dyn std::future::Future<Output = Result<()>> + Send + 'a>> {
        Box::pin(async { Ok(()) })
    }

    fn render(&self, frame: &mut Frame, area: Rect, theme: &ThemeConfig, app_state: &AppState) {
        let mut spans = Vec::new();

        // Show latest status message if any
        if let Some(latest_msg) = app_state.status_messages.back() {
            let style = match latest_msg.level {
                crate::tui::app::MessageLevel::Info => Style::default().fg(theme.colors.primary),
                crate::tui::app::MessageLevel::Success => Style::default().fg(theme.colors.success),
                crate::tui::app::MessageLevel::Warning => Style::default().fg(theme.colors.warning),
                crate::tui::app::MessageLevel::Error => Style::default().fg(theme.colors.error),
            };
            spans.push(Span::styled(&latest_msg.message, style));
        }

        // Add keybindings if enabled
        if self.show_keybindings {
            if !spans.is_empty() {
                spans.push(Span::raw(" | "));
            }
            spans.push(Span::styled("q", theme.styles.highlight_style));
            spans.push(Span::raw(":quit "));
            spans.push(Span::styled("?", theme.styles.highlight_style));
            spans.push(Span::raw(":help "));
            spans.push(Span::styled(":", theme.styles.highlight_style));
            spans.push(Span::raw(":cmd"));
        }

        let status_line = Line::from(spans);
        let paragraph = Paragraph::new(status_line)
            .block(Block::default().borders(Borders::ALL))
            .alignment(Alignment::Left);

        frame.render_widget(paragraph, area);
    }

    fn handle_event<'a>(
        &'a mut self,
        _event: ComponentEvent,
        _app_state: &'a AppState,
    ) -> std::pin::Pin<Box<dyn std::future::Future<Output = Result<ViewAction>> + Send + 'a>> {
        Box::pin(async { Ok(ViewAction::None) })
    }

    fn component_type(&self) -> ComponentType {
        ComponentType::StatusBar
    }

    fn is_focusable(&self) -> bool {
        false
    }

    fn is_focused(&self) -> bool {
        false
    }
}
