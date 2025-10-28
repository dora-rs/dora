// Interactive Analysis Tools View - Phase 1: Simplified Implementation with Mock Data
// TODO(Issue #33 Phase 2): Add real statistical tests, ML tools, and automated insights

#[cfg(test)]
use super::analysis_tools_types::AnalysisType;
use super::analysis_tools_types::{
    AnalysisResults, AnalysisToolsState, CorrelationPair, DistributionStats, OutlierData,
    TrendAnalysis,
};
use super::{BaseView, View, ViewAction};
use crate::tui::{Result, app::AppState, theme::ThemeConfig};
use crossterm::event::{KeyCode, KeyEvent, KeyModifiers};
use ratatui::{
    Frame,
    layout::{Constraint, Direction, Layout, Rect},
    style::{Modifier, Style},
    text::{Line, Span},
    widgets::{Block, Borders, List, ListItem, Paragraph, Row, Table},
};
use std::time::Duration;

pub struct AnalysisToolsView {
    base: BaseView,
    pub state: AnalysisToolsState,
    theme: ThemeConfig,
}

impl AnalysisToolsView {
    pub fn new(theme: &ThemeConfig) -> Self {
        let base = BaseView::new("Interactive Analysis Tools".to_string())
            .with_auto_refresh(Duration::from_millis(1000));

        Self {
            base,
            state: AnalysisToolsState::new(),
            theme: theme.clone(),
        }
    }

    fn render_header(&self, frame: &mut Frame, area: Rect) {
        let header_text = vec![
            Line::from(vec![
                Span::styled("Analysis Type: ", Style::default()),
                Span::styled(
                    self.state.current_analysis.name(),
                    Style::default().add_modifier(Modifier::BOLD),
                ),
            ]),
            Line::from(self.state.current_analysis.description()),
            Line::from(""),
            Line::from(vec![
                Span::raw("Selected Item: "),
                Span::styled(
                    format!("{}", self.state.selected_item + 1),
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
            "Tab/Shift+Tab: Switch analysis",
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

    fn render_distribution_analysis(
        &self,
        frame: &mut Frame,
        area: Rect,
        stats: &[DistributionStats],
    ) {
        if stats.is_empty() {
            return;
        }

        let selected_stat = &stats[self.state.selected_item.min(stats.len() - 1)];

        let content = vec![
            Line::from(""),
            Line::from(vec![
                Span::styled("Variable: ", Style::default()),
                Span::styled(
                    &selected_stat.variable_name,
                    Style::default().add_modifier(Modifier::BOLD),
                ),
            ]),
            Line::from(""),
            Line::from(vec![
                Span::raw("  Sample Size:  "),
                Span::styled(
                    format!("{}", selected_stat.sample_size),
                    Style::default().add_modifier(Modifier::BOLD),
                ),
            ]),
            Line::from(""),
            Line::from(vec![
                Span::raw("  Mean:         "),
                Span::styled(
                    format!("{:.2}", selected_stat.mean),
                    Style::default().add_modifier(Modifier::BOLD),
                ),
            ]),
            Line::from(vec![
                Span::raw("  Median:       "),
                Span::styled(format!("{:.2}", selected_stat.median), Style::default()),
            ]),
            Line::from(vec![
                Span::raw("  Std Dev:      "),
                Span::styled(format!("{:.2}", selected_stat.std_dev), Style::default()),
            ]),
            Line::from(""),
            Line::from(vec![
                Span::raw("  Min:          "),
                Span::styled(format!("{:.2}", selected_stat.min), Style::default()),
            ]),
            Line::from(vec![
                Span::raw("  Q1:           "),
                Span::styled(format!("{:.2}", selected_stat.q1), Style::default()),
            ]),
            Line::from(vec![
                Span::raw("  Q3:           "),
                Span::styled(format!("{:.2}", selected_stat.q3), Style::default()),
            ]),
            Line::from(vec![
                Span::raw("  Max:          "),
                Span::styled(format!("{:.2}", selected_stat.max), Style::default()),
            ]),
            Line::from(""),
            Line::from(vec![
                Span::raw("  Skewness:     "),
                Span::styled(format!("{:.3}", selected_stat.skewness), Style::default()),
            ]),
            Line::from(vec![
                Span::raw("  Kurtosis:     "),
                Span::styled(format!("{:.3}", selected_stat.kurtosis), Style::default()),
            ]),
        ];

        let widget = Paragraph::new(content).block(
            Block::default()
                .borders(Borders::ALL)
                .border_style(self.theme.focused_border_style())
                .title(format!(" {} ", self.state.current_analysis.name())),
        );

        frame.render_widget(widget, area);
    }

    fn render_correlation_analysis(
        &self,
        frame: &mut Frame,
        area: Rect,
        pairs: &[CorrelationPair],
    ) {
        if pairs.is_empty() {
            return;
        }

        let header = Row::new(vec![
            "Variable 1",
            "Variable 2",
            "Coefficient",
            "Strength",
            "P-Value",
        ])
        .style(Style::default().add_modifier(Modifier::BOLD));

        let rows: Vec<Row> = pairs
            .iter()
            .enumerate()
            .map(|(i, pair)| {
                let style = if i == self.state.selected_item {
                    self.theme.highlight_style()
                } else {
                    Style::default()
                };

                Row::new(vec![
                    pair.variable1.clone(),
                    pair.variable2.clone(),
                    format!("{:.3}", pair.coefficient),
                    pair.strength.label().to_string(),
                    format!("{:.4}", pair.p_value),
                ])
                .style(style)
            })
            .collect();

        let table = Table::new(
            rows,
            [
                Constraint::Percentage(25),
                Constraint::Percentage(25),
                Constraint::Percentage(15),
                Constraint::Percentage(20),
                Constraint::Percentage(15),
            ],
        )
        .header(header)
        .block(
            Block::default()
                .borders(Borders::ALL)
                .border_style(self.theme.focused_border_style())
                .title(format!(" {} ", self.state.current_analysis.name())),
        );

        frame.render_widget(table, area);
    }

    fn render_trend_analysis(&self, frame: &mut Frame, area: Rect, trends: &[TrendAnalysis]) {
        if trends.is_empty() {
            return;
        }

        let header = Row::new(vec![
            "Variable",
            "Trend",
            "Strength",
            "Confidence",
            "Change Rate",
        ])
        .style(Style::default().add_modifier(Modifier::BOLD));

        let rows: Vec<Row> = trends
            .iter()
            .enumerate()
            .map(|(i, trend)| {
                let style = if i == self.state.selected_item {
                    self.theme.highlight_style()
                } else {
                    Style::default()
                };

                Row::new(vec![
                    trend.variable_name.clone(),
                    format!(
                        "{} {}",
                        trend.trend_direction.icon(),
                        trend.trend_direction.label()
                    ),
                    format!("{:.2}", trend.trend_strength),
                    format!("{:.1}%", trend.confidence * 100.0),
                    format!("{:.1}%", trend.change_rate),
                ])
                .style(style)
            })
            .collect();

        let table = Table::new(
            rows,
            [
                Constraint::Percentage(25),
                Constraint::Percentage(20),
                Constraint::Percentage(15),
                Constraint::Percentage(20),
                Constraint::Percentage(20),
            ],
        )
        .header(header)
        .block(
            Block::default()
                .borders(Borders::ALL)
                .border_style(self.theme.focused_border_style())
                .title(format!(" {} ", self.state.current_analysis.name())),
        );

        frame.render_widget(table, area);
    }

    fn render_outlier_detection(&self, frame: &mut Frame, area: Rect, outliers: &[OutlierData]) {
        if outliers.is_empty() {
            return;
        }

        let selected_outlier = &outliers[self.state.selected_item.min(outliers.len() - 1)];

        let mut content = vec![
            Line::from(""),
            Line::from(vec![
                Span::styled("Variable: ", Style::default()),
                Span::styled(
                    &selected_outlier.variable_name,
                    Style::default().add_modifier(Modifier::BOLD),
                ),
            ]),
            Line::from(""),
            Line::from(vec![
                Span::raw("  Total Points:     "),
                Span::styled(
                    format!("{}", selected_outlier.total_points),
                    Style::default(),
                ),
            ]),
            Line::from(vec![
                Span::raw("  Outlier Count:    "),
                Span::styled(
                    format!("{}", selected_outlier.outlier_count),
                    Style::default().add_modifier(Modifier::BOLD),
                ),
            ]),
            Line::from(vec![
                Span::raw("  Outlier %:        "),
                Span::styled(
                    format!("{:.1}%", selected_outlier.outlier_percentage),
                    Style::default().add_modifier(Modifier::BOLD),
                ),
            ]),
            Line::from(vec![
                Span::raw("  Detection Method: "),
                Span::styled(&selected_outlier.detection_method, Style::default()),
            ]),
            Line::from(""),
            Line::from(vec![Span::styled(
                "Top Outliers:",
                Style::default().add_modifier(Modifier::BOLD),
            )]),
            Line::from(""),
        ];

        for (i, outlier) in selected_outlier.outliers.iter().take(5).enumerate() {
            content.push(Line::from(vec![
                Span::raw(format!("  {}. ", i + 1)),
                Span::raw(format!("Index: {:4}  ", outlier.index)),
                Span::raw(format!("Value: {:8.2}  ", outlier.value)),
                Span::raw(format!("Z-Score: {:5.2}  ", outlier.z_score)),
                Span::styled(
                    format!("[{}]", outlier.severity.label()),
                    Style::default().add_modifier(Modifier::BOLD),
                ),
            ]));
        }

        let widget = Paragraph::new(content).block(
            Block::default()
                .borders(Borders::ALL)
                .border_style(self.theme.focused_border_style())
                .title(format!(" {} ", self.state.current_analysis.name())),
        );

        frame.render_widget(widget, area);
    }
}

impl View for AnalysisToolsView {
    fn render(&mut self, frame: &mut Frame, area: Rect, _app_state: &AppState) {
        let chunks = Layout::default()
            .direction(Direction::Vertical)
            .constraints([
                Constraint::Length(6), // Header
                Constraint::Min(10),   // Analysis results
                Constraint::Length(8), // Shortcuts
            ])
            .split(area);

        self.render_header(frame, chunks[0]);

        // Render appropriate analysis based on current type
        match &self.state.results {
            AnalysisResults::Distribution(stats) => {
                self.render_distribution_analysis(frame, chunks[1], stats);
            }
            AnalysisResults::Correlation(pairs) => {
                self.render_correlation_analysis(frame, chunks[1], pairs);
            }
            AnalysisResults::Trend(trends) => {
                self.render_trend_analysis(frame, chunks[1], trends);
            }
            AnalysisResults::Outlier(outliers) => {
                self.render_outlier_detection(frame, chunks[1], outliers);
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
                self.state.next_analysis();
                Ok(ViewAction::None)
            }
            KeyCode::BackTab => {
                self.state.previous_analysis();
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
            ("Tab", "Next analysis type"),
            ("Shift+Tab", "Previous analysis type"),
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

    fn create_test_view() -> AnalysisToolsView {
        AnalysisToolsView::new(&ThemeConfig::default())
    }

    #[test]
    fn test_analysis_tools_view_new() {
        let view = create_test_view();
        assert_eq!(view.state.current_analysis, AnalysisType::Distribution);
        assert_eq!(view.title(), "Interactive Analysis Tools");
    }

    #[tokio::test]
    async fn test_handle_tab_key() {
        let mut view = create_test_view();
        let mut app_state = AppState::default();
        let key = KeyEvent::from(KeyCode::Tab);
        let _ = view.handle_key(key, &mut app_state).await;
        assert_eq!(view.state.current_analysis, AnalysisType::Correlation);
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
    }

    #[test]
    fn test_auto_refresh() {
        let view = create_test_view();
        assert!(view.auto_refresh().is_some());
    }
}
