/// Settings View implementation (Issue #30 - Phase 1)
/// Provides settings configuration interface with mock data.

use ratatui::{
    layout::{Constraint, Direction, Layout, Rect},
    style::{Color, Modifier, Style},
    text::{Line, Span},
    widgets::{Block, Borders, List, ListItem, Paragraph, Wrap},
    Frame,
};
use crossterm::event::{KeyCode, KeyEvent, KeyModifiers};
use std::time::Duration;

use super::{
    settings_types::{SettingsCategory, SettingItem, SettingsState, SettingValue},
    BaseView, View, ViewAction,
};
use crate::tui::{
    app::{AppState, ViewType},
    theme::ThemeConfig,
    Result,
};

/// Settings View - Configuration interface
pub struct SettingsView {
    base: BaseView,
    pub state: SettingsState,
    theme: ThemeConfig,
}

impl SettingsView {
    /// Create a new Settings View
    pub fn new(theme: &ThemeConfig) -> Self {
        let base = BaseView::new("Settings".to_string())
            .with_auto_refresh(Duration::from_millis(500));

        let state = SettingsState::new();

        Self {
            base,
            state,
            theme: theme.clone(),
        }
    }

    /// Render category navigation panel
    fn render_categories(&self, f: &mut Frame, area: Rect) {
        let items: Vec<ListItem> = SettingsCategory::all()
            .into_iter()
            .map(|category| {
                let is_selected = category == self.state.current_category;
                let style = if is_selected {
                    self.theme.highlight_style()
                } else {
                    Style::default()
                };

                let content = format!("{} {}", category.icon(), category.name());
                ListItem::new(Line::from(content)).style(style)
            })
            .collect();

        let block = Block::default()
            .borders(Borders::ALL)
            .border_style(self.theme.focused_border_style())
            .title(" Categories ");

        let list = List::new(items).block(block);

        f.render_widget(list, area);
    }

    /// Render settings list panel
    fn render_settings_list(&self, f: &mut Frame, area: Rect) {
        let settings = self.state.current_settings();

        let items: Vec<ListItem> = settings
            .iter()
            .enumerate()
            .map(|(i, setting)| {
                let is_selected = i == self.state.selected_index;
                let style = if is_selected {
                    self.theme.highlight_style()
                } else {
                    Style::default()
                };

                ListItem::new(Line::from(setting.format_display())).style(style)
            })
            .collect();

        let title = format!(" {} Settings ({}) ",
            self.state.current_category.name(),
            settings.len()
        );

        let block = Block::default()
            .borders(Borders::ALL)
            .border_style(self.theme.normal_border_style())
            .title(title);

        let list = List::new(items).block(block);

        f.render_widget(list, area);
    }

    /// Render setting details panel
    fn render_setting_details(&self, f: &mut Frame, area: Rect) {
        if let Some(setting) = self.state.get_selected_setting() {
            let details = vec![
                Line::from(vec![
                    Span::styled("Name: ", Style::default().fg(Color::Gray)),
                    Span::styled(&setting.display_name, Style::default().fg(Color::Cyan)),
                ]),
                Line::from(""),
                Line::from(vec![
                    Span::styled("Description: ", Style::default().fg(Color::Gray)),
                ]),
                Line::from(setting.description.clone()),
                Line::from(""),
                Line::from(vec![
                    Span::styled("Type: ", Style::default().fg(Color::Gray)),
                    Span::raw(setting.setting_type.name()),
                ]),
                Line::from(vec![
                    Span::styled("Value: ", Style::default().fg(Color::Gray)),
                    Span::styled(
                        setting.value.format_display(),
                        Style::default().fg(Color::Yellow).add_modifier(Modifier::BOLD),
                    ),
                ]),
                Line::from(vec![
                    Span::styled("Default: ", Style::default().fg(Color::Gray)),
                    Span::raw(setting.default_value.format_display()),
                ]),
            ];

            let block = Block::default()
                .borders(Borders::ALL)
                .border_style(self.theme.normal_border_style())
                .title(" Details ");

            let paragraph = Paragraph::new(details)
                .block(block)
                .wrap(Wrap { trim: true });

            f.render_widget(paragraph, area);
        }
    }

    /// Render keyboard shortcuts help
    fn render_shortcuts(&self, f: &mut Frame, area: Rect) {
        let shortcuts = vec![
            Line::from(vec![
                Span::styled("Tab", Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD)),
                Span::raw("  Switch Category  "),
                Span::styled("Space", Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD)),
                Span::raw(" Toggle"),
            ]),
            Line::from(vec![
                Span::styled("↑/↓", Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD)),
                Span::raw("  Navigate  "),
                Span::styled("r", Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD)),
                Span::raw("  Reset  "),
                Span::styled("R", Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD)),
                Span::raw(" Reset All"),
            ]),
        ];

        let block = Block::default()
            .borders(Borders::ALL)
            .border_style(self.theme.normal_border_style())
            .title(" Shortcuts ");

        let paragraph = Paragraph::new(shortcuts).block(block);

        f.render_widget(paragraph, area);
    }

    /// Handle value toggle for boolean settings
    fn handle_toggle(&mut self) -> Result<ViewAction> {
        if let Some(setting) = self.state.get_selected_setting_mut() {
            if setting.toggle_boolean() {
                return Ok(ViewAction::ShowStatus(format!(
                    "Toggled {} to {}",
                    setting.display_name,
                    setting.value.format_display()
                )));
            }
        }
        Ok(ViewAction::None)
    }

    /// Handle reset to default for current setting
    fn handle_reset(&mut self) -> Result<ViewAction> {
        if let Some(setting) = self.state.get_selected_setting_mut() {
            let name = setting.display_name.clone();
            setting.reset_to_default();
            Ok(ViewAction::ShowStatus(format!("Reset {} to default", name)))
        } else {
            Ok(ViewAction::None)
        }
    }

    /// Handle reset all settings
    fn handle_reset_all(&mut self) -> Result<ViewAction> {
        self.state.reset_all_to_defaults();
        Ok(ViewAction::ShowStatus("Reset all settings to defaults".to_string()))
    }
}

impl View for SettingsView {
    fn render(&mut self, f: &mut Frame, area: Rect, _app_state: &AppState) {
        // Main layout: [Categories][Settings+Details][Shortcuts]
        let main_chunks = Layout::default()
            .direction(Direction::Vertical)
            .constraints([
                Constraint::Min(10),     // Main content
                Constraint::Length(4),   // Shortcuts
            ])
            .split(area);

        // Split main content horizontally
        let content_chunks = Layout::default()
            .direction(Direction::Horizontal)
            .constraints([
                Constraint::Percentage(25),  // Categories
                Constraint::Percentage(40),  // Settings list
                Constraint::Percentage(35),  // Details
            ])
            .split(main_chunks[0]);

        // Render panels
        self.render_categories(f, content_chunks[0]);
        self.render_settings_list(f, content_chunks[1]);
        self.render_setting_details(f, content_chunks[2]);
        self.render_shortcuts(f, main_chunks[1]);
    }

    async fn handle_key(&mut self, key: KeyEvent, _app_state: &mut AppState) -> Result<ViewAction> {
        match key.code {
            KeyCode::Char('q') => Ok(ViewAction::PopView),
            KeyCode::Char('c') if key.modifiers.contains(KeyModifiers::CONTROL) => {
                Ok(ViewAction::Quit)
            }
            KeyCode::Tab => {
                self.state.next_category();
                Ok(ViewAction::None)
            }
            KeyCode::BackTab => {
                self.state.prev_category();
                Ok(ViewAction::None)
            }
            KeyCode::Up | KeyCode::Char('k') => {
                self.state.navigate_up();
                Ok(ViewAction::None)
            }
            KeyCode::Down | KeyCode::Char('j') => {
                self.state.navigate_down();
                Ok(ViewAction::None)
            }
            KeyCode::Char(' ') => {
                self.handle_toggle()
            }
            KeyCode::Char('r') => {
                self.handle_reset()
            }
            KeyCode::Char('R') => {
                self.handle_reset_all()
            }
            KeyCode::Char('?') => Ok(ViewAction::ShowHelp),
            _ => Ok(ViewAction::None),
        }
    }

    async fn update(&mut self, _app_state: &mut AppState) -> Result<()> {
        self.state.mark_refreshed();
        Ok(())
    }

    fn help_text(&self) -> Vec<(&str, &str)> {
        vec![
            ("Tab", "Switch category"),
            ("↑/↓", "Navigate settings"),
            ("Space", "Toggle boolean"),
            ("r", "Reset to default"),
            ("R", "Reset all"),
            ("?", "Show help"),
            ("q", "Back"),
        ]
    }

    fn auto_refresh(&self) -> Option<Duration> {
        self.base.auto_refresh_interval
    }

    fn title(&self) -> &str {
        &self.base.title
    }

    fn on_focus(&mut self) {
        self.base.set_focused(true);
    }

    fn on_blur(&mut self) {
        self.base.set_focused(false);
    }

    async fn on_mount(&mut self, _app_state: &mut AppState) -> Result<()> {
        self.base.mark_updated();
        Ok(())
    }

    fn on_unmount(&mut self) {
        // Cleanup if needed
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_settings_view_creation() {
        let theme = ThemeConfig::default();
        let view = SettingsView::new(&theme);

        assert_eq!(view.title(), "Settings");
        assert_eq!(view.state.current_category, SettingsCategory::General);
    }

    #[test]
    fn test_settings_view_has_mock_data() {
        let theme = ThemeConfig::default();
        let view = SettingsView::new(&theme);

        assert!(view.state.general_settings.len() > 0);
        assert!(view.state.appearance_settings.len() > 0);
        assert!(view.state.performance_settings.len() > 0);
    }

    #[test]
    fn test_settings_view_auto_refresh() {
        let theme = ThemeConfig::default();
        let view = SettingsView::new(&theme);

        assert!(view.auto_refresh().is_some());
        assert_eq!(view.auto_refresh().unwrap(), Duration::from_millis(500));
    }

    #[test]
    fn test_settings_view_help_text() {
        let theme = ThemeConfig::default();
        let view = SettingsView::new(&theme);

        let help = view.help_text();
        assert!(help.len() >= 6);
        assert!(help.iter().any(|(key, _)| *key == "Tab"));
        assert!(help.iter().any(|(key, _)| *key == "Space"));
    }

    #[test]
    fn test_settings_view_initial_category() {
        let theme = ThemeConfig::default();
        let view = SettingsView::new(&theme);

        assert_eq!(view.state.current_category, SettingsCategory::General);
        assert_eq!(view.state.selected_index, 0);
    }
}
