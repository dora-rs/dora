/// Help Browser View implementation (Issue #31 - Phase 1)
/// Provides interactive help content browsing with mock data.

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
    help_browser_types::{HelpCategory, HelpBrowserState, HelpContent},
    BaseView, View, ViewAction,
};
use crate::tui::{
    app::{AppState, ViewType},
    theme::ThemeConfig,
    Result,
};

/// Help Browser View - Interactive help content browser
pub struct HelpBrowserView {
    base: BaseView,
    pub state: HelpBrowserState,
    theme: ThemeConfig,
}

impl HelpBrowserView {
    /// Create a new Help Browser View
    pub fn new(theme: &ThemeConfig) -> Self {
        let base = BaseView::new("Help Browser".to_string())
            .with_auto_refresh(Duration::from_millis(1000));

        let state = HelpBrowserState::new();

        Self {
            base,
            state,
            theme: theme.clone(),
        }
    }

    /// Render category navigation panel
    fn render_categories(&self, f: &mut Frame, area: Rect) {
        let items: Vec<ListItem> = HelpCategory::all()
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

    /// Render topics list panel
    fn render_topics(&self, f: &mut Frame, area: Rect) {
        let current_topics: Vec<_> = self.state.topics
            .iter()
            .filter(|t| t.category == self.state.current_category)
            .collect();

        let items: Vec<ListItem> = current_topics
            .iter()
            .enumerate()
            .map(|(i, topic)| {
                let is_selected = i == self.state.current_topic_index;
                let style = if is_selected {
                    self.theme.highlight_style()
                } else {
                    Style::default()
                };

                let content = vec![
                    Line::from(vec![
                        Span::styled(&topic.title, Style::default().add_modifier(Modifier::BOLD)),
                    ]),
                    Line::from(vec![
                        Span::styled(&topic.summary, Style::default().fg(Color::Gray)),
                    ]),
                ];

                ListItem::new(content).style(style)
            })
            .collect();

        let title = format!(" {} Topics ({}) ",
            self.state.current_category.name(),
            current_topics.len()
        );

        let block = Block::default()
            .borders(Borders::ALL)
            .border_style(self.theme.normal_border_style())
            .title(title);

        let list = List::new(items).block(block);

        f.render_widget(list, area);
    }

    /// Render content viewer panel
    fn render_content(&self, f: &mut Frame, area: Rect) {
        if let Some(content) = &self.state.current_content {
            let visible_height = (area.height as usize).saturating_sub(3); // Account for borders and title
            let total_lines = content.content.len();

            // Ensure scroll_offset is valid
            let scroll_offset = self.state.scroll_offset.min(total_lines.saturating_sub(visible_height));

            // Get visible content lines
            let visible_content: Vec<Line> = content.content
                .iter()
                .skip(scroll_offset)
                .take(visible_height)
                .map(|line| Line::from(line.clone()))
                .collect();

            let title = format!(" {} (Line {}/{}) ",
                content.title,
                scroll_offset + 1,
                total_lines
            );

            let block = Block::default()
                .borders(Borders::ALL)
                .border_style(self.theme.normal_border_style())
                .title(title);

            let paragraph = Paragraph::new(visible_content)
                .block(block)
                .wrap(Wrap { trim: false });

            f.render_widget(paragraph, area);
        } else {
            let no_content = vec![
                Line::from(""),
                Line::from(vec![
                    Span::styled("No topic selected", Style::default().fg(Color::Gray)),
                ]),
                Line::from(""),
                Line::from(vec![
                    Span::styled("Select a topic from the list to view its content", Style::default().fg(Color::DarkGray)),
                ]),
            ];

            let block = Block::default()
                .borders(Borders::ALL)
                .border_style(self.theme.normal_border_style())
                .title(" Content ");

            let paragraph = Paragraph::new(no_content).block(block);

            f.render_widget(paragraph, area);
        }
    }

    /// Render keyboard shortcuts help
    fn render_shortcuts(&self, f: &mut Frame, area: Rect) {
        let shortcuts = vec![
            Line::from(vec![
                Span::styled("Tab", Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD)),
                Span::raw("  Switch Category  "),
                Span::styled("↑/↓", Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD)),
                Span::raw("  Navigate Topics  "),
                Span::styled("j/k", Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD)),
                Span::raw(" Scroll Content"),
            ]),
            Line::from(vec![
                Span::styled("Enter", Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD)),
                Span::raw("  Select Topic  "),
                Span::styled("?", Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD)),
                Span::raw("  Help  "),
                Span::styled("q", Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD)),
                Span::raw(" Back"),
            ]),
        ];

        let block = Block::default()
            .borders(Borders::ALL)
            .border_style(self.theme.normal_border_style())
            .title(" Shortcuts ");

        let paragraph = Paragraph::new(shortcuts).block(block);

        f.render_widget(paragraph, area);
    }

    /// Handle topic selection
    fn handle_select_topic(&mut self) -> Result<ViewAction> {
        self.state.select_current_topic();
        Ok(ViewAction::ShowStatus("Topic loaded".to_string()))
    }
}

impl View for HelpBrowserView {
    fn render(&mut self, f: &mut Frame, area: Rect, _app_state: &AppState) {
        // Main layout: [Content Area][Shortcuts]
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
                Constraint::Percentage(20),  // Categories
                Constraint::Percentage(30),  // Topics list
                Constraint::Percentage(50),  // Content viewer
            ])
            .split(main_chunks[0]);

        // Render panels
        self.render_categories(f, content_chunks[0]);
        self.render_topics(f, content_chunks[1]);
        self.render_content(f, content_chunks[2]);
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
            KeyCode::Up => {
                self.state.navigate_up();
                Ok(ViewAction::None)
            }
            KeyCode::Down => {
                self.state.navigate_down();
                Ok(ViewAction::None)
            }
            KeyCode::Char('j') => {
                self.state.scroll_down();
                Ok(ViewAction::None)
            }
            KeyCode::Char('k') => {
                self.state.scroll_up();
                Ok(ViewAction::None)
            }
            KeyCode::Char('J') => {
                self.state.page_down();
                Ok(ViewAction::None)
            }
            KeyCode::Char('K') => {
                self.state.page_up();
                Ok(ViewAction::None)
            }
            KeyCode::Enter => {
                self.handle_select_topic()
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
            ("↑/↓", "Navigate topics"),
            ("j/k", "Scroll content"),
            ("J/K", "Page up/down"),
            ("Enter", "Select topic"),
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
        // Load initial topic
        self.state.select_current_topic();
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
    fn test_help_browser_view_creation() {
        let theme = ThemeConfig::default();
        let view = HelpBrowserView::new(&theme);

        assert_eq!(view.title(), "Help Browser");
        assert_eq!(view.state.current_category, HelpCategory::GettingStarted);
    }

    #[test]
    fn test_help_browser_view_has_mock_data() {
        let theme = ThemeConfig::default();
        let view = HelpBrowserView::new(&theme);

        assert!(view.state.topics.len() > 0);
        assert!(view.state.topics.len() >= 15); // We created 15 topics
    }

    #[test]
    fn test_help_browser_view_auto_refresh() {
        let theme = ThemeConfig::default();
        let view = HelpBrowserView::new(&theme);

        assert!(view.auto_refresh().is_some());
        assert_eq!(view.auto_refresh().unwrap(), Duration::from_millis(1000));
    }

    #[test]
    fn test_help_browser_view_help_text() {
        let theme = ThemeConfig::default();
        let view = HelpBrowserView::new(&theme);

        let help = view.help_text();
        assert!(help.len() >= 7);
        assert!(help.iter().any(|(key, _)| *key == "Tab"));
        assert!(help.iter().any(|(key, _)| *key == "Enter"));
        assert!(help.iter().any(|(key, _)| *key == "j/k"));
    }

    #[test]
    fn test_help_browser_view_initial_state() {
        let theme = ThemeConfig::default();
        let view = HelpBrowserView::new(&theme);

        assert_eq!(view.state.current_category, HelpCategory::GettingStarted);
        assert_eq!(view.state.current_topic_index, 0);
        assert_eq!(view.state.scroll_offset, 0);
    }
}
