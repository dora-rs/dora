use crossterm::event::{KeyCode, KeyEvent};
use ratatui::{
    layout::{Alignment, Constraint, Direction, Layout, Rect},
    style::{Modifier, Style},
    text::{Line, Span},
    widgets::{Block, Borders, List, ListItem, Paragraph},
};

use super::{BaseView, View, ViewAction};
use crate::tui::{Frame, Result, app::AppState, theme::ThemeConfig};

pub struct HelpView {
    base: BaseView,
    theme: ThemeConfig,
    scroll_offset: usize,
}

impl HelpView {
    pub fn new(theme: &ThemeConfig) -> Self {
        Self {
            base: BaseView::new("Help".to_string()),
            theme: theme.clone(),
            scroll_offset: 0,
        }
    }

    fn get_help_sections(&self) -> Vec<(&str, Vec<(&str, &str)>)> {
        vec![
            (
                "Global Shortcuts",
                vec![
                    ("q", "Quit application"),
                    (":", "Enter command mode"),
                    ("Esc", "Back/Cancel"),
                    ("F1", "Show this help"),
                    ("Ctrl+C", "Force quit"),
                    ("Ctrl+L", "Refresh current view"),
                ],
            ),
            (
                "Navigation",
                vec![
                    ("1", "Dashboard view"),
                    ("2", "Dataflow manager"),
                    ("3", "System monitor"),
                    ("4", "Log viewer"),
                    ("5", "Settings"),
                    ("Tab", "Next widget"),
                    ("Shift+Tab", "Previous widget"),
                ],
            ),
            (
                "Dashboard",
                vec![
                    ("↑/k", "Move up in list"),
                    ("↓/j", "Move down in list"),
                    ("Enter", "Inspect selected item"),
                    ("Space", "Toggle dataflow state"),
                    ("l", "View logs"),
                    ("r", "Refresh data"),
                    ("s", "Toggle system info"),
                    ("n", "New dataflow"),
                ],
            ),
            (
                "Command Mode",
                vec![
                    ("Enter", "Execute command"),
                    ("Esc", "Exit command mode"),
                    ("↑/↓", "Command history"),
                    ("Tab", "Auto-complete"),
                    ("Ctrl+A", "Move to beginning"),
                    ("Ctrl+E", "Move to end"),
                ],
            ),
            (
                "Common Commands",
                vec![
                    ("ps", "List dataflows"),
                    ("start <file>", "Start dataflow"),
                    ("stop <id>", "Stop dataflow"),
                    ("logs <target>", "View logs"),
                    ("inspect <target>", "Inspect resource"),
                    ("monitor", "System monitoring"),
                    ("ui <view>", "Switch TUI view"),
                ],
            ),
            (
                "Tips",
                vec![
                    ("", "• Use : to enter CLI commands within TUI"),
                    ("", "• Press q to quit from any view"),
                    ("", "• Use Esc to go back in view stack"),
                    ("", "• Tab completion works in command mode"),
                    ("", "• Arrow keys navigate in most views"),
                    ("", "• Many views auto-refresh data"),
                ],
            ),
        ]
    }
}

impl View for HelpView {
    fn render(&mut self, f: &mut Frame, area: Rect, _app_state: &AppState) {
        let sections = self.get_help_sections();

        // Calculate layout
        let chunks = Layout::default()
            .direction(Direction::Vertical)
            .constraints([
                Constraint::Length(3), // Title
                Constraint::Min(0),    // Content
                Constraint::Length(3), // Footer
            ])
            .split(area);

        // Render title
        let title = Paragraph::new("🚀 Dora CLI/TUI Help")
            .style(self.theme.styles.highlight_style)
            .alignment(Alignment::Center)
            .block(self.theme.styled_block("Help"));
        f.render_widget(title, chunks[0]);

        // Render content in columns
        let content_chunks = Layout::default()
            .direction(Direction::Horizontal)
            .constraints([Constraint::Percentage(50), Constraint::Percentage(50)])
            .split(chunks[1]);

        // Left column
        let left_sections = &sections[0..sections.len() / 2];
        self.render_help_column(f, content_chunks[0], left_sections);

        // Right column
        let right_sections = &sections[sections.len() / 2..];
        self.render_help_column(f, content_chunks[1], right_sections);

        // Footer
        let footer_text = "Press Esc to go back, ↑↓ to scroll, q to quit";
        let footer = Paragraph::new(footer_text)
            .style(Style::default().fg(self.theme.colors.muted))
            .alignment(Alignment::Center)
            .block(Block::default().borders(Borders::ALL));
        f.render_widget(footer, chunks[2]);
    }

    async fn handle_key(&mut self, key: KeyEvent, _app_state: &mut AppState) -> Result<ViewAction> {
        match key.code {
            KeyCode::Esc => Ok(ViewAction::PopView),
            KeyCode::Up | KeyCode::Char('k') => {
                if self.scroll_offset > 0 {
                    self.scroll_offset -= 1;
                }
                Ok(ViewAction::None)
            }
            KeyCode::Down | KeyCode::Char('j') => {
                self.scroll_offset += 1;
                Ok(ViewAction::None)
            }
            KeyCode::PageUp => {
                self.scroll_offset = self.scroll_offset.saturating_sub(10);
                Ok(ViewAction::None)
            }
            KeyCode::PageDown => {
                self.scroll_offset += 10;
                Ok(ViewAction::None)
            }
            KeyCode::Home => {
                self.scroll_offset = 0;
                Ok(ViewAction::None)
            }
            _ => Ok(ViewAction::None),
        }
    }

    async fn update(&mut self, _app_state: &mut AppState) -> Result<()> {
        Ok(())
    }

    fn help_text(&self) -> Vec<(&str, &str)> {
        vec![
            ("Esc", "Go back"),
            ("↑/k", "Scroll up"),
            ("↓/j", "Scroll down"),
            ("PgUp", "Page up"),
            ("PgDn", "Page down"),
            ("Home", "Go to top"),
        ]
    }

    fn title(&self) -> &str {
        &self.base.title
    }
}

impl HelpView {
    fn render_help_column(
        &self,
        f: &mut Frame,
        area: Rect,
        sections: &[(&str, Vec<(&str, &str)>)],
    ) {
        let mut items = Vec::new();

        for (section_title, entries) in sections {
            // Section header
            items.push(ListItem::new(Line::from(Span::styled(
                format!("▼ {section_title}"),
                Style::default()
                    .fg(self.theme.colors.primary)
                    .add_modifier(Modifier::BOLD),
            ))));

            // Section entries
            for (key, description) in entries {
                if key.is_empty() {
                    // Tip or description without key
                    items.push(ListItem::new(Line::from(Span::styled(
                        format!("  {description}"),
                        Style::default().fg(self.theme.colors.text),
                    ))));
                } else {
                    // Key binding
                    items.push(ListItem::new(Line::from(vec![
                        Span::raw("  "),
                        Span::styled(
                            format!("{key:<12}"),
                            Style::default()
                                .fg(self.theme.colors.secondary)
                                .add_modifier(Modifier::BOLD),
                        ),
                        Span::styled(
                            description.to_string(),
                            Style::default().fg(self.theme.colors.text),
                        ),
                    ])));
                }
            }

            // Add spacing between sections
            items.push(ListItem::new(Line::from("")));
        }

        let help_list = List::new(items)
            .block(Block::default().borders(Borders::ALL))
            .style(Style::default().fg(self.theme.colors.text));

        f.render_widget(help_list, area);
    }
}
