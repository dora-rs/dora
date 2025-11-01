// Context Awareness View - Interactive display of context awareness system
// Issue #35: Advanced Context Awareness

use super::{BaseView, View, ViewAction};
use crate::tui::{Result, app::AppState, theme::ThemeConfig, views::context_awareness_types::*};
use crossterm::event::{KeyCode, KeyEvent};
use ratatui::{
    Frame,
    layout::{Alignment, Constraint, Direction, Layout, Rect},
    style::{Modifier, Style},
    text::{Line, Span},
    widgets::{Block, Borders, List, ListItem, Paragraph},
};

/// Context Awareness View
pub struct ContextAwarenessView {
    base: BaseView,
    pub state: ContextAwarenessState,
    theme: ThemeConfig,
}

impl ContextAwarenessView {
    pub fn new() -> Self {
        Self {
            base: BaseView::new("Context Awareness".to_string()),
            state: ContextAwarenessState::new(),
            theme: ThemeConfig::default(),
        }
    }

    fn render_header(&self, frame: &mut Frame, area: Rect) {
        let sections = ContextSection::all();
        let tabs: Vec<Span> = sections
            .iter()
            .enumerate()
            .flat_map(|(i, section)| {
                let is_selected = section == &self.state.current_section;
                let style = if is_selected {
                    Style::default()
                        .fg(self.theme.colors.accent)
                        .add_modifier(Modifier::BOLD)
                } else {
                    Style::default().fg(self.theme.colors.muted)
                };

                let mut spans = vec![Span::styled(section.title(), style)];
                if i < sections.len() - 1 {
                    spans.push(Span::raw(" │ "));
                }
                spans
            })
            .collect();

        let header = Paragraph::new(Line::from(tabs))
            .block(Block::default().borders(Borders::BOTTOM))
            .alignment(Alignment::Center);

        frame.render_widget(header, area);
    }

    fn render_execution_context(
        &self,
        frame: &mut Frame,
        area: Rect,
        data: &ExecutionEnvironmentData,
    ) {
        let chunks = Layout::default()
            .direction(Direction::Vertical)
            .constraints([
                Constraint::Length(8), // Environment
                Constraint::Length(8), // Capabilities
                Constraint::Min(6),    // Current Context
            ])
            .split(area);

        // Environment Section
        let env_lines = vec![
            Line::from(vec![
                Span::styled("TTY: ", Style::default().add_modifier(Modifier::BOLD)),
                Span::raw(if data.environment.is_tty { "Yes" } else { "No" }),
                Span::raw("  "),
                Span::styled("Piped: ", Style::default().add_modifier(Modifier::BOLD)),
                Span::raw(if data.environment.is_piped {
                    "Yes"
                } else {
                    "No"
                }),
                Span::raw("  "),
                Span::styled("Scripted: ", Style::default().add_modifier(Modifier::BOLD)),
                Span::raw(if data.environment.is_scripted {
                    "Yes"
                } else {
                    "No"
                }),
            ]),
            Line::from(vec![
                Span::styled("Terminal: ", Style::default().add_modifier(Modifier::BOLD)),
                Span::raw(
                    data.environment
                        .terminal_type
                        .as_deref()
                        .unwrap_or("unknown"),
                ),
            ]),
            Line::from(vec![
                Span::styled("Shell: ", Style::default().add_modifier(Modifier::BOLD)),
                Span::raw(data.environment.shell.as_deref().unwrap_or("unknown")),
            ]),
            Line::from(vec![
                Span::styled(
                    "CI Detected: ",
                    Style::default().add_modifier(Modifier::BOLD),
                ),
                Span::raw(
                    data.environment
                        .detected_ci
                        .as_ref()
                        .map(|ci| ci.name())
                        .unwrap_or("None"),
                ),
            ]),
        ];

        let env_block = Paragraph::new(env_lines).block(
            Block::default()
                .borders(Borders::ALL)
                .title("Execution Environment")
                .border_style(Style::default().fg(self.theme.colors.border)),
        );

        frame.render_widget(env_block, chunks[0]);

        // Capabilities Section
        let cap_lines = vec![
            Line::from(vec![
                Span::styled(
                    "Color Support: ",
                    Style::default().add_modifier(Modifier::BOLD),
                ),
                Span::raw(data.capabilities.color_level.name()),
            ]),
            Line::from(vec![
                Span::styled("Unicode: ", Style::default().add_modifier(Modifier::BOLD)),
                Span::raw(if data.capabilities.supports_unicode {
                    "Yes"
                } else {
                    "No"
                }),
                Span::raw("  "),
                Span::styled("Mouse: ", Style::default().add_modifier(Modifier::BOLD)),
                Span::raw(if data.capabilities.supports_mouse {
                    "Yes"
                } else {
                    "No"
                }),
            ]),
            Line::from(vec![
                Span::styled(
                    "Terminal Size: ",
                    Style::default().add_modifier(Modifier::BOLD),
                ),
                Span::raw(format!(
                    "{}x{}",
                    data.capabilities.terminal_width, data.capabilities.terminal_height
                )),
            ]),
        ];

        let cap_block = Paragraph::new(cap_lines).block(
            Block::default()
                .borders(Borders::ALL)
                .title("Terminal Capabilities")
                .border_style(Style::default().fg(self.theme.colors.border)),
        );

        frame.render_widget(cap_block, chunks[1]);

        // Current Context Section
        let ctx_lines = vec![
            Line::from(vec![
                Span::styled(
                    "Command Category: ",
                    Style::default().add_modifier(Modifier::BOLD),
                ),
                Span::raw(data.current_context.command_category.name()),
            ]),
            Line::from(vec![
                Span::styled(
                    "Time of Day: ",
                    Style::default().add_modifier(Modifier::BOLD),
                ),
                Span::raw(data.current_context.time_of_day.name()),
            ]),
            Line::from(vec![
                Span::styled(
                    "Terminal Type: ",
                    Style::default().add_modifier(Modifier::BOLD),
                ),
                Span::raw(data.current_context.terminal_type.name()),
            ]),
            Line::from(vec![
                Span::styled(
                    "Environment: ",
                    Style::default().add_modifier(Modifier::BOLD),
                ),
                Span::raw(data.current_context.environment_type.name()),
            ]),
        ];

        let ctx_block = Paragraph::new(ctx_lines).block(
            Block::default()
                .borders(Borders::ALL)
                .title("Current Context Key")
                .border_style(Style::default().fg(self.theme.colors.border)),
        );

        frame.render_widget(ctx_block, chunks[2]);
    }

    fn render_adaptation_rules(&self, frame: &mut Frame, area: Rect, rules: &[AdaptationRule]) {
        let items: Vec<ListItem> = rules
            .iter()
            .enumerate()
            .map(|(i, rule)| {
                let is_selected = i == self.state.selected_index;
                let style = if is_selected {
                    Style::default()
                        .fg(self.theme.colors.accent)
                        .add_modifier(Modifier::BOLD)
                } else {
                    Style::default().fg(self.theme.colors.text)
                };

                let status = if rule.is_active { "●" } else { "○" };
                let status_style = if rule.is_active {
                    Style::default().fg(self.theme.colors.success)
                } else {
                    Style::default().fg(self.theme.colors.muted)
                };

                let content = vec![
                    Line::from(vec![
                        Span::styled(status, status_style),
                        Span::raw(" "),
                        Span::styled(&rule.name, style),
                        Span::styled(
                            format!(" [{}]", rule.rule_type.name()),
                            Style::default().fg(self.theme.colors.muted),
                        ),
                    ]),
                    Line::from(vec![
                        Span::raw("  Condition: "),
                        Span::styled(
                            &rule.condition,
                            Style::default().fg(self.theme.colors.muted),
                        ),
                    ]),
                    Line::from(vec![
                        Span::raw("  Action: "),
                        Span::styled(&rule.action, Style::default().fg(self.theme.colors.muted)),
                    ]),
                    Line::from(vec![Span::styled(
                        format!(
                            "  Priority: {} | Triggers: {}",
                            rule.priority, rule.trigger_count
                        ),
                        Style::default().fg(self.theme.colors.muted),
                    )]),
                    Line::from(""),
                ];

                ListItem::new(content)
            })
            .collect();

        let list = List::new(items).block(
            Block::default()
                .borders(Borders::ALL)
                .title(format!(
                    "Adaptation Rules ({} total, {} active)",
                    rules.len(),
                    rules.iter().filter(|r| r.is_active).count()
                ))
                .border_style(Style::default().fg(self.theme.colors.border)),
        );

        frame.render_widget(list, area);
    }

    fn render_learning_patterns(
        &self,
        frame: &mut Frame,
        area: Rect,
        patterns: &[LearningPattern],
    ) {
        let items: Vec<ListItem> = patterns
            .iter()
            .enumerate()
            .map(|(i, pattern)| {
                let is_selected = i == self.state.selected_index;
                let style = if is_selected {
                    Style::default()
                        .fg(self.theme.colors.accent)
                        .add_modifier(Modifier::BOLD)
                } else {
                    Style::default().fg(self.theme.colors.text)
                };

                let confidence_bar = "█".repeat((pattern.confidence * 10.0) as usize);
                let confidence_style = if pattern.confidence >= 0.9 {
                    Style::default().fg(self.theme.colors.success)
                } else if pattern.confidence >= 0.7 {
                    Style::default().fg(self.theme.colors.warning)
                } else {
                    Style::default().fg(self.theme.colors.muted)
                };

                let content = vec![
                    Line::from(vec![Span::styled(&pattern.description, style)]),
                    Line::from(vec![
                        Span::raw("  Type: "),
                        Span::styled(
                            pattern.pattern_type.name(),
                            Style::default().fg(self.theme.colors.muted),
                        ),
                        Span::raw("  "),
                        Span::styled(confidence_bar, confidence_style),
                        Span::styled(
                            format!(" {:.0}%", pattern.confidence * 100.0),
                            Style::default().fg(self.theme.colors.muted),
                        ),
                    ]),
                    Line::from(vec![Span::styled(
                        format!("  Sample size: {} observations", pattern.sample_size),
                        Style::default().fg(self.theme.colors.muted),
                    )]),
                    Line::from(""),
                ];

                ListItem::new(content)
            })
            .collect();

        let list = List::new(items).block(
            Block::default()
                .borders(Borders::ALL)
                .title(format!("Learning Patterns ({} discovered)", patterns.len()))
                .border_style(Style::default().fg(self.theme.colors.border)),
        );

        frame.render_widget(list, area);
    }

    fn render_preference_levels(&self, frame: &mut Frame, area: Rect, levels: &[PreferenceLevel]) {
        let items: Vec<ListItem> = levels
            .iter()
            .enumerate()
            .map(|(i, level)| {
                let is_selected = i == self.state.selected_index;
                let style = if is_selected {
                    Style::default()
                        .fg(self.theme.colors.accent)
                        .add_modifier(Modifier::BOLD)
                } else {
                    Style::default().fg(self.theme.colors.text)
                };

                let priority_indicator = "◆".repeat(level.level.priority() as usize);

                let mut content = vec![
                    Line::from(vec![
                        Span::styled(
                            format!("{priority_indicator} "),
                            Style::default().fg(self.theme.colors.accent),
                        ),
                        Span::styled(level.level.name(), style),
                    ]),
                    Line::from(vec![
                        Span::raw("  "),
                        Span::styled(
                            level.level.description(),
                            Style::default().fg(self.theme.colors.muted),
                        ),
                    ]),
                ];

                if !level.preferences.is_empty() {
                    content.push(Line::from(vec![Span::styled(
                        format!("  {} preferences set", level.preferences.len()),
                        Style::default().fg(self.theme.colors.muted),
                    )]));

                    // Show first 2 preferences as examples
                    for (key, value) in level.preferences.iter().take(2) {
                        content.push(Line::from(vec![
                            Span::raw("    "),
                            Span::styled(key, Style::default().fg(self.theme.colors.muted)),
                            Span::raw(": "),
                            Span::styled(value, Style::default().fg(self.theme.colors.muted)),
                        ]));
                    }
                    if level.preferences.len() > 2 {
                        content.push(Line::from(vec![Span::styled(
                            format!("    ...and {} more", level.preferences.len() - 2),
                            Style::default().fg(self.theme.colors.muted),
                        )]));
                    }
                }

                content.push(Line::from(""));

                ListItem::new(content)
            })
            .collect();

        let list = List::new(items).block(
            Block::default()
                .borders(Borders::ALL)
                .title("Preference Resolution Hierarchy (High to Low Priority)")
                .border_style(Style::default().fg(self.theme.colors.border)),
        );

        frame.render_widget(list, area);
    }

    fn render_help(&self, frame: &mut Frame, area: Rect) {
        let help_text = vec![Line::from(vec![
            Span::styled(
                "Tab/Shift+Tab",
                Style::default().fg(self.theme.colors.accent),
            ),
            Span::raw(": Switch section  "),
            Span::styled("↑/↓ or j/k", Style::default().fg(self.theme.colors.accent)),
            Span::raw(": Navigate  "),
            Span::styled("r", Style::default().fg(self.theme.colors.accent)),
            Span::raw(": Refresh  "),
            Span::styled("q", Style::default().fg(self.theme.colors.accent)),
            Span::raw(": Back"),
        ])];

        let help = Paragraph::new(help_text)
            .block(Block::default().borders(Borders::TOP))
            .alignment(Alignment::Center);

        frame.render_widget(help, area);
    }
}

impl Default for ContextAwarenessView {
    fn default() -> Self {
        Self::new()
    }
}

impl View for ContextAwarenessView {
    fn render(&mut self, frame: &mut Frame, area: Rect, _app_state: &AppState) {
        let chunks = Layout::default()
            .direction(Direction::Vertical)
            .constraints([
                Constraint::Length(3), // Header
                Constraint::Min(10),   // Content
                Constraint::Length(3), // Help
            ])
            .split(area);

        self.render_header(frame, chunks[0]);

        match &self.state.data {
            ContextData::ExecutionEnvironment(data) => {
                self.render_execution_context(frame, chunks[1], data);
            }
            ContextData::AdaptationRules(rules) => {
                self.render_adaptation_rules(frame, chunks[1], rules);
            }
            ContextData::LearningPatterns(patterns) => {
                self.render_learning_patterns(frame, chunks[1], patterns);
            }
            ContextData::PreferenceLevels(levels) => {
                self.render_preference_levels(frame, chunks[1], levels);
            }
        }

        self.render_help(frame, chunks[2]);
    }

    async fn handle_key(&mut self, key: KeyEvent, _app_state: &mut AppState) -> Result<ViewAction> {
        match key.code {
            KeyCode::Char('q') => Ok(ViewAction::PopView),
            KeyCode::Tab => {
                self.state.next_section();
                Ok(ViewAction::None)
            }
            KeyCode::BackTab => {
                self.state.previous_section();
                Ok(ViewAction::None)
            }
            KeyCode::Up | KeyCode::Char('k') => {
                self.state.previous_item();
                Ok(ViewAction::None)
            }
            KeyCode::Down | KeyCode::Char('j') => {
                self.state.next_item();
                Ok(ViewAction::None)
            }
            KeyCode::Char('r') => {
                // Refresh data
                self.state.update_data();
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
            ("Tab/Shift+Tab", "Switch section"),
            ("↑/↓, j/k", "Navigate items"),
            ("r", "Refresh data"),
            ("q", "Back to previous view"),
        ]
    }

    fn title(&self) -> &str {
        &self.base.title
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_context_awareness_view_creation() {
        let view = ContextAwarenessView::new();
        assert_eq!(view.state.current_section, ContextSection::ExecutionContext);
        assert_eq!(view.state.selected_index, 0);
    }

    #[test]
    fn test_context_awareness_view_default() {
        let view = ContextAwarenessView::default();
        assert_eq!(view.state.current_section, ContextSection::ExecutionContext);
    }

    #[test]
    fn test_view_title() {
        let view = ContextAwarenessView::new();
        let title = view.title();
        assert_eq!(title, "Context Awareness");
    }

    #[test]
    fn test_view_help_text() {
        let view = ContextAwarenessView::new();
        let help = view.help_text();
        assert!(!help.is_empty());
        assert!(help.iter().any(|(key, _)| key.contains("Tab")));
        assert!(help.iter().any(|(key, _)| key.contains("q")));
    }

    #[tokio::test]
    async fn test_navigation_keys() {
        let mut view = ContextAwarenessView::new();
        let mut app_state = AppState::default();

        // Tab to next section
        let result = view
            .handle_key(KeyEvent::from(KeyCode::Tab), &mut app_state)
            .await;
        assert!(result.is_ok());
        assert_eq!(
            view.state.current_section,
            ContextSection::ContextPreferences
        );

        // BackTab to previous section
        let result = view
            .handle_key(KeyEvent::from(KeyCode::BackTab), &mut app_state)
            .await;
        assert!(result.is_ok());
        assert_eq!(view.state.current_section, ContextSection::ExecutionContext);
    }

    #[tokio::test]
    async fn test_quit_key() {
        let mut view = ContextAwarenessView::new();
        let mut app_state = AppState::default();

        let result = view
            .handle_key(KeyEvent::from(KeyCode::Char('q')), &mut app_state)
            .await;

        assert!(result.is_ok());
        assert!(matches!(result.unwrap(), ViewAction::PopView));
    }
}
