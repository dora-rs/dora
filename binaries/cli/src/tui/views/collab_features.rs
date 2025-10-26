// Collaborative Development Features View - Phase 1: Simplified Implementation with Mock Data
// TODO(Issue #34 Phase 2): Add real workspace management, live collaboration engine, and VCS integration

use super::collab_features_types::*;
use super::{BaseView, View, ViewAction};
use crate::tui::{app::AppState, theme::ThemeConfig, Result};
use crossterm::event::{KeyCode, KeyEvent, KeyModifiers};
use ratatui::{
    layout::{Constraint, Direction, Layout, Rect},
    style::{Modifier, Style},
    text::{Line, Span},
    widgets::{Block, Borders, List, ListItem, Paragraph, Row, Table},
    Frame,
};
use std::time::Duration;

pub struct CollabFeaturesView {
    base: BaseView,
    pub state: CollabFeaturesState,
    theme: ThemeConfig,
}

impl CollabFeaturesView {
    pub fn new(theme: &ThemeConfig) -> Self {
        let base = BaseView::new("Collaborative Features".to_string())
            .with_auto_refresh(Duration::from_millis(1000));

        Self {
            base,
            state: CollabFeaturesState::new(),
            theme: theme.clone(),
        }
    }

    fn render_header(&self, frame: &mut Frame, area: Rect) {
        let header_text = vec![
            Line::from(vec![
                Span::raw(self.state.current_section.icon()),
                Span::raw(" "),
                Span::styled(
                    self.state.current_section.name(),
                    Style::default().add_modifier(Modifier::BOLD),
                ),
            ]),
            Line::from(self.state.current_section.description()),
            Line::from(""),
            Line::from(vec![
                Span::raw("Selected: "),
                Span::styled(
                    format!("{}/{}", self.state.selected_item + 1, self.state.get_item_count()),
                    Style::default().add_modifier(Modifier::BOLD),
                ),
            ]),
        ];

        let header = Paragraph::new(header_text).block(
            Block::default()
                .borders(Borders::ALL)
                .border_style(self.theme.normal_border_style())
                .title(" Info "),
        );

        frame.render_widget(header, area);
    }

    fn render_shortcuts(&self, frame: &mut Frame, area: Rect) {
        let shortcuts = vec![
            "Tab/Shift+Tab: Switch section",
            "↑/↓: Select item",
            "j/k: Scroll",
            "r: Refresh",
            "q: Quit",
        ];

        let items: Vec<ListItem> = shortcuts
            .iter()
            .map(|s| ListItem::new(Line::from(*s)))
            .collect();

        let list = List::new(items).block(
            Block::default()
                .borders(Borders::ALL)
                .border_style(self.theme.normal_border_style())
                .title(" Shortcuts "),
        );

        frame.render_widget(list, area);
    }

    fn render_team_workspace(&self, frame: &mut Frame, area: Rect, members: &[TeamMember]) {
        if members.is_empty() {
            return;
        }

        let header = Row::new(vec!["Status", "Username", "Role", "Presence", "Contributions"])
            .style(Style::default().add_modifier(Modifier::BOLD));

        let rows: Vec<Row> = members
            .iter()
            .enumerate()
            .map(|(i, member)| {
                let style = if i == self.state.selected_item {
                    self.theme.highlight_style()
                } else {
                    Style::default()
                };

                Row::new(vec![
                    format!("{}", member.role.color_indicator()),
                    member.username.clone(),
                    member.role.label().to_string(),
                    format!("{} {}", member.presence.indicator(), member.presence.label()),
                    format!("{}", member.contributions),
                ])
                .style(style)
            })
            .collect();

        let table = Table::new(
            rows,
            [
                Constraint::Length(6),
                Constraint::Percentage(25),
                Constraint::Percentage(20),
                Constraint::Percentage(25),
                Constraint::Percentage(20),
            ],
        )
        .header(header)
        .block(
            Block::default()
                .borders(Borders::ALL)
                .border_style(self.theme.focused_border_style())
                .title(format!(" {} ", self.state.current_section.name())),
        );

        frame.render_widget(table, area);
    }

    fn render_live_sessions(&self, frame: &mut Frame, area: Rect, sessions: &[CollabSession]) {
        if sessions.is_empty() {
            return;
        }

        let header = Row::new(vec!["Type", "Host", "Participants", "Duration", "Activity"])
            .style(Style::default().add_modifier(Modifier::BOLD));

        let rows: Vec<Row> = sessions
            .iter()
            .enumerate()
            .map(|(i, session)| {
                let style = if i == self.state.selected_item {
                    self.theme.highlight_style()
                } else {
                    Style::default()
                };

                let duration = chrono::Utc::now()
                    .signed_duration_since(session.started_at)
                    .num_minutes();

                let status = if session.is_active { "🔴" } else { "⚫" };

                Row::new(vec![
                    format!("{} {}", session.session_type.icon(), session.session_type.label()),
                    session.host.clone(),
                    format!("{} users", session.participants.len()),
                    format!("{} {}min", status, duration),
                    format!("{} events", session.activity_count),
                ])
                .style(style)
            })
            .collect();

        let table = Table::new(
            rows,
            [
                Constraint::Percentage(25),
                Constraint::Percentage(20),
                Constraint::Percentage(20),
                Constraint::Percentage(18),
                Constraint::Percentage(17),
            ],
        )
        .header(header)
        .block(
            Block::default()
                .borders(Borders::ALL)
                .border_style(self.theme.focused_border_style())
                .title(format!(" {} ", self.state.current_section.name())),
        );

        frame.render_widget(table, area);
    }

    fn render_code_reviews(&self, frame: &mut Frame, area: Rect, reviews: &[CodeReview]) {
        if reviews.is_empty() {
            return;
        }

        let header = Row::new(vec!["Status", "ID", "Title", "Author", "Files", "Comments"])
            .style(Style::default().add_modifier(Modifier::BOLD));

        let rows: Vec<Row> = reviews
            .iter()
            .enumerate()
            .map(|(i, review)| {
                let style = if i == self.state.selected_item {
                    self.theme.highlight_style()
                } else {
                    Style::default()
                };

                let title = if review.title.len() > 30 {
                    format!("{}...", &review.title[..27])
                } else {
                    review.title.clone()
                };

                Row::new(vec![
                    format!("{} {}", review.status.color_indicator(), review.status.label()),
                    review.review_id.clone(),
                    title,
                    review.author.clone(),
                    format!("{}", review.files_changed),
                    format!("{}", review.comments),
                ])
                .style(style)
            })
            .collect();

        let table = Table::new(
            rows,
            [
                Constraint::Percentage(18),
                Constraint::Length(8),
                Constraint::Percentage(35),
                Constraint::Percentage(18),
                Constraint::Length(7),
                Constraint::Length(10),
            ],
        )
        .header(header)
        .block(
            Block::default()
                .borders(Borders::ALL)
                .border_style(self.theme.focused_border_style())
                .title(format!(" {} ", self.state.current_section.name())),
        );

        frame.render_widget(table, area);
    }

    fn render_knowledge_base(&self, frame: &mut Frame, area: Rect, articles: &[KnowledgeArticle]) {
        if articles.is_empty() {
            return;
        }

        let header = Row::new(vec!["Category", "Title", "Difficulty", "Read Time", "Engagement"])
            .style(Style::default().add_modifier(Modifier::BOLD));

        let rows: Vec<Row> = articles
            .iter()
            .enumerate()
            .map(|(i, article)| {
                let style = if i == self.state.selected_item {
                    self.theme.highlight_style()
                } else {
                    Style::default()
                };

                let title = if article.title.len() > 35 {
                    format!("{}...", &article.title[..32])
                } else {
                    article.title.clone()
                };

                Row::new(vec![
                    format!("{} {}", article.category.icon(), article.category.label()),
                    title,
                    format!("{} {}", article.difficulty.icon(), article.difficulty.label()),
                    format!("{}min", article.read_time_minutes),
                    format!("👁 {} 👍 {}", article.views, article.likes),
                ])
                .style(style)
            })
            .collect();

        let table = Table::new(
            rows,
            [
                Constraint::Percentage(22),
                Constraint::Percentage(35),
                Constraint::Percentage(18),
                Constraint::Length(8),
                Constraint::Percentage(20),
            ],
        )
        .header(header)
        .block(
            Block::default()
                .borders(Borders::ALL)
                .border_style(self.theme.focused_border_style())
                .title(format!(" {} ", self.state.current_section.name())),
        );

        frame.render_widget(table, area);
    }
}

impl View for CollabFeaturesView {
    fn render(&mut self, frame: &mut Frame, area: Rect, _app_state: &AppState) {
        let chunks = Layout::default()
            .direction(Direction::Vertical)
            .constraints([
                Constraint::Length(6),  // Header
                Constraint::Min(10),    // Content
                Constraint::Length(8),  // Shortcuts
            ])
            .split(area);

        self.render_header(frame, chunks[0]);

        // Render appropriate content based on current section
        match &self.state.data {
            CollabData::TeamWorkspace(members) => {
                self.render_team_workspace(frame, chunks[1], members);
            }
            CollabData::LiveSessions(sessions) => {
                self.render_live_sessions(frame, chunks[1], sessions);
            }
            CollabData::CodeReviews(reviews) => {
                self.render_code_reviews(frame, chunks[1], reviews);
            }
            CollabData::KnowledgeBase(articles) => {
                self.render_knowledge_base(frame, chunks[1], articles);
            }
        }

        self.render_shortcuts(frame, chunks[2]);
    }

    async fn handle_key(&mut self, key: KeyEvent, _app_state: &mut AppState) -> Result<ViewAction> {
        match key.code {
            KeyCode::Char('q') => Ok(ViewAction::PopView),
            KeyCode::Char('c') if key.modifiers.contains(KeyModifiers::CONTROL) => {
                Ok(ViewAction::Quit)
            }
            KeyCode::Tab => {
                self.state.next_section();
                Ok(ViewAction::None)
            }
            KeyCode::BackTab => {
                self.state.previous_section();
                Ok(ViewAction::None)
            }
            KeyCode::Up | KeyCode::Char('k') => {
                self.state.select_previous();
                Ok(ViewAction::None)
            }
            KeyCode::Down | KeyCode::Char('j') => {
                self.state.select_next();
                Ok(ViewAction::None)
            }
            KeyCode::Char('r') | KeyCode::Char('R') => {
                self.state.refresh();
                Ok(ViewAction::None)
            }
            KeyCode::Char('?') => Ok(ViewAction::ShowHelp),
            _ => Ok(ViewAction::None),
        }
    }

    async fn update(&mut self, _app_state: &mut AppState) -> Result<()> {
        Ok(())
    }

    fn help_text(&self) -> Vec<(&str, &str)> {
        vec![
            ("Tab", "Next section"),
            ("Shift+Tab", "Previous section"),
            ("↑/↓", "Select item"),
            ("j/k", "Scroll"),
            ("r", "Refresh data"),
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
}

#[cfg(test)]
mod tests {
    use super::*;

    fn create_test_view() -> CollabFeaturesView {
        CollabFeaturesView::new(&ThemeConfig::default())
    }

    #[test]
    fn test_collab_features_view_new() {
        let view = create_test_view();
        assert_eq!(view.state.current_section, CollabSection::TeamWorkspace);
        assert_eq!(view.title(), "Collaborative Features");
    }

    #[tokio::test]
    async fn test_handle_tab_key() {
        let mut view = create_test_view();
        let mut app_state = AppState::default();
        let key = KeyEvent::from(KeyCode::Tab);
        let _ = view.handle_key(key, &mut app_state).await;
        assert_eq!(view.state.current_section, CollabSection::LiveSessions);
    }

    #[tokio::test]
    async fn test_handle_backtab_key() {
        let mut view = create_test_view();
        let mut app_state = AppState::default();
        let key = KeyEvent::from(KeyCode::BackTab);
        let _ = view.handle_key(key, &mut app_state).await;
        assert_eq!(view.state.current_section, CollabSection::KnowledgeBase);
    }

    #[tokio::test]
    async fn test_handle_navigation_keys() {
        let mut view = create_test_view();
        let mut app_state = AppState::default();

        let key_down = KeyEvent::from(KeyCode::Down);
        let _ = view.handle_key(key_down, &mut app_state).await;
        assert_eq!(view.state.selected_item, 1);

        let key_up = KeyEvent::from(KeyCode::Up);
        let _ = view.handle_key(key_up, &mut app_state).await;
        assert_eq!(view.state.selected_item, 0);
    }

    #[tokio::test]
    async fn test_handle_refresh_key() {
        let mut view = create_test_view();
        let mut app_state = AppState::default();

        let initial_time = view.state.last_refresh;
        std::thread::sleep(std::time::Duration::from_millis(10));

        let key = KeyEvent::from(KeyCode::Char('r'));
        let _ = view.handle_key(key, &mut app_state).await;

        assert!(view.state.last_refresh > initial_time);
    }

    #[test]
    fn test_help_text() {
        let view = create_test_view();
        let help = view.help_text();
        assert!(!help.is_empty());
        assert!(help.iter().any(|(key, _)| *key == "Tab"));
        assert!(help.iter().any(|(key, _)| *key == "r"));
    }

    #[test]
    fn test_auto_refresh() {
        let view = create_test_view();
        assert!(view.auto_refresh().is_some());
    }
}
