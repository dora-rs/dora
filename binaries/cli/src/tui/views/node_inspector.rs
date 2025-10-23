use ratatui::layout::Rect;
use crossterm::event::KeyEvent;
use crate::tui::{app::AppState, theme::ThemeConfig, Frame, Result};
use super::{BaseView, View, ViewAction};

pub struct NodeInspectorView {
    base: BaseView,
    theme: ThemeConfig,
    node_id: String,
}

impl NodeInspectorView {
    pub fn new(node_id: &str, theme: &ThemeConfig) -> Self {
        Self {
            base: BaseView::new(format!("Node Inspector: {}", node_id)),
            theme: theme.clone(),
            node_id: node_id.to_string(),
        }
    }
}

impl View for NodeInspectorView {
    fn render(&mut self, _f: &mut Frame, _area: Rect, _app_state: &AppState) {
        // TODO: Implement node inspector view
    }
    
    async fn handle_key(&mut self, _key: KeyEvent, _app_state: &mut AppState) -> Result<ViewAction> {
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