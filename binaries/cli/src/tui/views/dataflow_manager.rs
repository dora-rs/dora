use super::{BaseView, View, ViewAction};
use crate::tui::{Frame, Result, app::AppState, theme::ThemeConfig};
use crossterm::event::KeyEvent;
use ratatui::{
    layout::{Constraint, Direction, Layout, Rect},
    style::{Color, Modifier, Style},
    text::{Line, Span},
    widgets::{Block, Borders, List, ListItem, Paragraph, Wrap},
};

pub struct DataflowManagerView {
    base: BaseView,
    theme: ThemeConfig,
}

impl DataflowManagerView {
    pub fn new(theme: &ThemeConfig) -> Self {
        Self {
            base: BaseView::new("Dataflow Manager".to_string()),
            theme: theme.clone(),
        }
    }
}

impl View for DataflowManagerView {
    fn render(&mut self, f: &mut Frame, area: Rect, app_state: &AppState) {
        let block = self
            .theme
            .styled_block(&self.base.title)
            .borders(Borders::ALL);

        let inner = block.inner(area);
        f.render_widget(block, area);

        if app_state.dataflows.is_empty() {
            let message = Paragraph::new(
                "No dataflows detected.\n\nLaunch a dataflow with `dora start` or refresh once \
                 the coordinator reports active flows.",
            )
            .style(Style::default().fg(self.theme.colors.muted))
            .wrap(Wrap { trim: true });
            f.render_widget(message, inner);
            return;
        }

        let layout = Layout::default()
            .direction(Direction::Vertical)
            .constraints([Constraint::Length(3), Constraint::Min(1)])
            .split(inner);

        let header = Paragraph::new(vec![Line::from(vec![
            Span::styled(
                "TOTAL: ",
                Style::default()
                    .fg(self.theme.colors.primary)
                    .add_modifier(Modifier::BOLD),
            ),
            Span::raw(app_state.dataflows.len().to_string()),
            Span::raw("   "),
            Span::styled(
                "RUNNING: ",
                Style::default()
                    .fg(Color::Green)
                    .add_modifier(Modifier::BOLD),
            ),
            Span::raw(
                app_state
                    .dataflows
                    .iter()
                    .filter(|df| df.status.eq_ignore_ascii_case("running"))
                    .count()
                    .to_string(),
            ),
            Span::raw("   "),
            Span::styled(
                "FAILED: ",
                Style::default().fg(Color::Red).add_modifier(Modifier::BOLD),
            ),
            Span::raw(
                app_state
                    .dataflows
                    .iter()
                    .filter(|df| df.status.eq_ignore_ascii_case("failed"))
                    .count()
                    .to_string(),
            ),
        ])])
        .wrap(Wrap { trim: true });
        f.render_widget(header, layout[0]);

        let items: Vec<ListItem> = app_state
            .dataflows
            .iter()
            .map(|df| {
                let status_style = match df.status.to_lowercase().as_str() {
                    "running" => Style::default().fg(Color::Green),
                    "finished" => Style::default().fg(Color::Blue),
                    "failed" => Style::default().fg(Color::Red),
                    _ => Style::default().fg(self.theme.colors.muted),
                };

                ListItem::new(vec![
                    Line::from(vec![
                        Span::styled(
                            df.name.clone(),
                            Style::default()
                                .fg(self.theme.colors.text)
                                .add_modifier(Modifier::BOLD),
                        ),
                        Span::raw(" "),
                        Span::styled(
                            format!("({})", df.id),
                            Style::default().fg(self.theme.colors.muted),
                        ),
                    ]),
                    Line::from(vec![
                        Span::styled("Status: ", Style::default().fg(self.theme.colors.muted)),
                        Span::styled(df.status.clone(), status_style),
                    ]),
                ])
            })
            .collect();

        let list = List::new(items)
            .block(Block::default().title("Dataflows").borders(Borders::ALL))
            .highlight_style(
                Style::default()
                    .fg(self.theme.colors.primary)
                    .add_modifier(Modifier::BOLD),
            );

        f.render_widget(list, layout[1]);
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
