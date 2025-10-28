use super::{BaseView, View, ViewAction};
use crate::tui::{Frame, Result, app::AppState, theme::ThemeConfig};
use crossterm::event::KeyEvent;
use ratatui::layout::Rect;

pub struct SystemMonitorView {
    base: BaseView,
    theme: ThemeConfig,
}

impl SystemMonitorView {
    pub fn new(theme: &ThemeConfig) -> Self {
        Self {
            base: BaseView::new("System Monitor".to_string()),
            theme: theme.clone(),
        }
    }
}

impl View for SystemMonitorView {
    fn render(&mut self, _f: &mut Frame, _area: Rect, _app_state: &AppState) {
        // TODO: Implement system monitor view
    }

    async fn handle_key(
        &mut self,
        _key: KeyEvent,
        _app_state: &mut AppState,
    ) -> Result<ViewAction> {
        Ok(ViewAction::None)
    }

    async fn update(&mut self, _app_state: &mut AppState) -> Result<()> {
        Ok(())
    }

    fn help_text(&self) -> Vec<(&str, &str)> {
        vec![("Esc", "Go back")]
    }

    fn title(&self) -> &str {
        &self.base.title
    }
}
