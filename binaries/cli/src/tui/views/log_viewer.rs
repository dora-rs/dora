/// Interactive Log Viewer implementation (Issue #28 - Phase 1)
use super::{BaseView, View, ViewAction};
use crate::tui::{
    app::AppState,
    theme::ThemeConfig,
    Result,
};
use crossterm::event::{KeyCode, KeyEvent, KeyModifiers};
use ratatui::{
    layout::{Constraint, Direction, Layout, Rect},
    style::{Color, Modifier, Style},
    text::{Line, Span},
    widgets::{Block, Borders, List, ListItem, Paragraph, Wrap},
    Frame,
};
use std::time::Duration;

pub use super::log_viewer_types::*;

/// Interactive Log Viewer View
pub struct LogViewerView {
    base: BaseView,
    theme: ThemeConfig,
    pub state: LogViewerState,
    mock_log_counter: usize,
    search_mode: bool,
    search_input: String,
}

impl LogViewerView {
    /// Create a new log viewer view
    pub fn new(target: &str, theme: &ThemeConfig) -> Self {
        let title = if target.is_empty() {
            "Log Viewer".to_string()
        } else {
            format!("Logs: {}", target)
        };

        Self {
            base: BaseView::new(title)
                .with_auto_refresh(Duration::from_secs(1)),
            theme: theme.clone(),
            state: LogViewerState::new(),
            mock_log_counter: 0,
            search_mode: false,
            search_input: String::new(),
        }
    }

    /// Generate mock log entries for demonstration
    fn generate_mock_logs(&mut self, count: usize) {
        let sources = vec![
            "camera-node",
            "detection-node",
            "tracking-node",
            "planner-node",
            "controller-node",
        ];

        let messages = vec![
            ("Processing frame {}", LogLevel::Info),
            ("Detection completed in {}ms", LogLevel::Debug),
            ("Found {} objects", LogLevel::Info),
            ("Tracking update successful", LogLevel::Debug),
            ("Planning path to target", LogLevel::Info),
            ("Connection timeout, retrying...", LogLevel::Warn),
            ("Failed to read sensor data", LogLevel::Error),
            ("Memory usage: {}%", LogLevel::Debug),
            ("Network latency: {}ms", LogLevel::Trace),
            ("Critical: System overload detected", LogLevel::Error),
            ("Performance degradation warning", LogLevel::Warn),
            ("Initialization complete", LogLevel::Info),
        ];

        for _ in 0..count {
            let source = sources[self.mock_log_counter % sources.len()].to_string();
            let (msg_template, level) = &messages[self.mock_log_counter % messages.len()];

            // Generate realistic message content
            let message = match *level {
                LogLevel::Error => {
                    if msg_template.contains("Failed") {
                        "Failed to read sensor data: timeout after 5s".to_string()
                    } else {
                        "Critical: System overload detected, dropping frames".to_string()
                    }
                }
                LogLevel::Warn => {
                    if msg_template.contains("Connection") {
                        "Connection timeout, retrying... attempt 3/5".to_string()
                    } else {
                        "Performance degradation warning: latency >100ms".to_string()
                    }
                }
                LogLevel::Info => {
                    msg_template.replace("{}", &(self.mock_log_counter % 100).to_string())
                }
                LogLevel::Debug => {
                    msg_template.replace("{}", &(self.mock_log_counter % 50 + 10).to_string())
                }
                LogLevel::Trace => {
                    msg_template.replace("{}", &(self.mock_log_counter % 20 + 5).to_string())
                }
            };

            let entry = LogEntry::new(self.mock_log_counter, *level, source, message);
            self.state.add_log(entry);
            self.mock_log_counter += 1;
        }
    }

    /// Render the log list
    fn render_log_list(&self, f: &mut Frame, area: Rect) {
        let filtered_logs = self.state.get_filtered_logs();

        let items: Vec<ListItem> = filtered_logs
            .iter()
            .enumerate()
            .map(|(i, entry)| {
                let is_selected = i == self.state.selected_index;
                self.render_log_entry(entry, is_selected)
            })
            .collect();

        let title = if self.state.paused {
            "Logs (PAUSED)"
        } else {
            "Logs (Live)"
        };

        let block = Block::default()
            .title(title)
            .borders(Borders::ALL)
            .border_style(Style::default().fg(self.theme.colors.border));

        let list = List::new(items)
            .block(block)
            .highlight_style(
                Style::default()
                    .bg(self.theme.colors.selection)
                    .add_modifier(Modifier::BOLD),
            )
            .highlight_symbol("▶ ");

        f.render_widget(list, area);
    }

    /// Render a single log entry with syntax highlighting
    fn render_log_entry(&self, entry: &LogEntry, is_selected: bool) -> ListItem {
        let timestamp = entry.timestamp_str();
        let level_str = entry.level.short_name();
        let source = &entry.source;
        let message = &entry.message;

        // Color based on log level
        let level_color = match entry.level {
            LogLevel::Error => Color::Red,
            LogLevel::Warn => Color::Yellow,
            LogLevel::Info => Color::Blue,
            LogLevel::Debug => Color::Gray,
            LogLevel::Trace => Color::DarkGray,
        };

        let spans = vec![
            Span::styled(
                format!("[{}] ", timestamp),
                Style::default().fg(Color::DarkGray),
            ),
            Span::styled(
                format!("{:5} ", level_str),
                Style::default().fg(level_color).add_modifier(Modifier::BOLD),
            ),
            Span::styled(
                format!("{:15} ", source),
                Style::default().fg(Color::Cyan),
            ),
            Span::styled(
                message.to_string(),
                Style::default().fg(if is_selected {
                    Color::White
                } else {
                    Color::Reset
                }),
            ),
        ];

        ListItem::new(Line::from(spans))
    }

    /// Render log details panel
    fn render_log_details(&self, f: &mut Frame, area: Rect) {
        let content = if let Some(entry) = self.state.get_selected_log() {
            vec![
                Line::from(vec![
                    Span::styled("Level: ", Style::default().add_modifier(Modifier::BOLD)),
                    Span::styled(entry.level.name(), Style::default().fg(match entry.level {
                        LogLevel::Error => Color::Red,
                        LogLevel::Warn => Color::Yellow,
                        LogLevel::Info => Color::Blue,
                        LogLevel::Debug => Color::Gray,
                        LogLevel::Trace => Color::DarkGray,
                    })),
                ]),
                Line::from(vec![
                    Span::styled("Source: ", Style::default().add_modifier(Modifier::BOLD)),
                    Span::raw(&entry.source),
                ]),
                Line::from(vec![
                    Span::styled("Time: ", Style::default().add_modifier(Modifier::BOLD)),
                    Span::raw(entry.timestamp_str()),
                ]),
                Line::from(""),
                Line::from(vec![
                    Span::styled("Message:", Style::default().add_modifier(Modifier::BOLD)),
                ]),
                Line::from(entry.message.clone()),
            ]
        } else {
            vec![Line::from("No log selected")]
        };

        let paragraph = Paragraph::new(content)
            .block(
                Block::default()
                    .title("Details")
                    .borders(Borders::ALL)
                    .border_style(Style::default().fg(self.theme.colors.border)),
            )
            .wrap(Wrap { trim: true });

        f.render_widget(paragraph, area);
    }

    /// Render statistics panel
    fn render_stats(&self, f: &mut Frame, area: Rect) {
        let stats = self.state.stats();

        let content = vec![
            Line::from(vec![
                Span::styled("Total: ", Style::default().fg(Color::Gray)),
                Span::styled(format!("{}", stats.total), Style::default().fg(Color::White)),
                Span::styled(" | Filtered: ", Style::default().fg(Color::Gray)),
                Span::styled(format!("{}", stats.filtered), Style::default().fg(Color::Cyan)),
                Span::styled(" | ", Style::default().fg(Color::Gray)),
                Span::styled("E:", Style::default().fg(Color::Red)),
                Span::styled(format!("{}", stats.error_count), Style::default().fg(Color::Red)),
                Span::styled(" W:", Style::default().fg(Color::Yellow)),
                Span::styled(format!("{}", stats.warn_count), Style::default().fg(Color::Yellow)),
                Span::styled(" I:", Style::default().fg(Color::Blue)),
                Span::styled(format!("{}", stats.info_count), Style::default().fg(Color::Blue)),
                Span::styled(" D:", Style::default().fg(Color::Gray)),
                Span::styled(format!("{}", stats.debug_count), Style::default().fg(Color::Gray)),
                Span::styled(" T:", Style::default().fg(Color::DarkGray)),
                Span::styled(format!("{}", stats.trace_count), Style::default().fg(Color::DarkGray)),
            ]),
        ];

        let paragraph = Paragraph::new(content)
            .block(
                Block::default()
                    .title("Statistics")
                    .borders(Borders::ALL)
                    .border_style(Style::default().fg(self.theme.colors.border)),
            );

        f.render_widget(paragraph, area);
    }

    /// Render filter panel
    fn render_filter_panel(&self, f: &mut Frame, area: Rect) {
        let enabled_levels: Vec<String> = self
            .state
            .filter
            .enabled_levels
            .iter()
            .map(|l| l.short_name().to_string())
            .collect();

        let filter_status = if enabled_levels.is_empty() {
            "None".to_string()
        } else if enabled_levels.len() == 5 {
            "All".to_string()
        } else {
            enabled_levels.join(",")
        };

        let search_status = if self.state.filter.search_query.is_empty() {
            "None".to_string()
        } else {
            format!("\"{}\"", self.state.filter.search_query)
        };

        let content = vec![
            Line::from(vec![
                Span::styled("Levels: ", Style::default().fg(Color::Gray)),
                Span::styled(filter_status, Style::default().fg(Color::Cyan)),
                Span::styled(" | Search: ", Style::default().fg(Color::Gray)),
                Span::styled(search_status, Style::default().fg(Color::Yellow)),
            ]),
        ];

        let paragraph = Paragraph::new(content)
            .block(
                Block::default()
                    .title("Filters")
                    .borders(Borders::ALL)
                    .border_style(Style::default().fg(self.theme.colors.border)),
            );

        f.render_widget(paragraph, area);
    }

    /// Render search input overlay
    fn render_search_overlay(&self, f: &mut Frame, area: Rect) {
        let search_area = Rect {
            x: area.x + 2,
            y: area.y + 2,
            width: area.width.saturating_sub(4).min(60),
            height: 3,
        };

        let content = vec![Line::from(vec![
            Span::raw("Search: "),
            Span::styled(&self.search_input, Style::default().add_modifier(Modifier::BOLD)),
            Span::styled("_", Style::default().add_modifier(Modifier::SLOW_BLINK)),
        ])];

        let paragraph = Paragraph::new(content)
            .block(
                Block::default()
                    .title("Search (Esc to cancel, Enter to apply)")
                    .borders(Borders::ALL)
                    .border_style(Style::default().fg(Color::Yellow)),
            );

        f.render_widget(paragraph, search_area);
    }
}

impl View for LogViewerView {
    fn render(&mut self, f: &mut Frame, area: Rect, _app_state: &AppState) {
        // Main layout: [Filter] [Logs] [Details] [Stats]
        let chunks = Layout::default()
            .direction(Direction::Vertical)
            .constraints([
                Constraint::Length(3),  // Filter panel
                Constraint::Min(10),    // Log list
                Constraint::Length(8),  // Details panel
                Constraint::Length(3),  // Stats panel
            ])
            .split(area);

        self.render_filter_panel(f, chunks[0]);
        self.render_log_list(f, chunks[1]);
        self.render_log_details(f, chunks[2]);
        self.render_stats(f, chunks[3]);

        // Render search overlay if in search mode
        if self.search_mode {
            self.render_search_overlay(f, area);
        }
    }

    async fn handle_key(&mut self, key: KeyEvent, _app_state: &mut AppState) -> Result<ViewAction> {
        // Handle search mode separately
        if self.search_mode {
            match key.code {
                KeyCode::Esc => {
                    self.search_mode = false;
                    self.search_input.clear();
                    return Ok(ViewAction::None);
                }
                KeyCode::Enter => {
                    self.search_mode = false;
                    self.state.filter.set_search(self.search_input.clone());
                    self.search_input.clear();
                    return Ok(ViewAction::None);
                }
                KeyCode::Char(c) => {
                    self.search_input.push(c);
                    return Ok(ViewAction::None);
                }
                KeyCode::Backspace => {
                    self.search_input.pop();
                    return Ok(ViewAction::None);
                }
                _ => return Ok(ViewAction::None),
            }
        }

        // Normal mode key handling
        match key.code {
            KeyCode::Char('q') | KeyCode::Esc => Ok(ViewAction::PopView),

            // Navigation
            KeyCode::Up | KeyCode::Char('k') => {
                self.state.move_up();
                Ok(ViewAction::None)
            }
            KeyCode::Down | KeyCode::Char('j') => {
                self.state.move_down();
                Ok(ViewAction::None)
            }
            KeyCode::PageUp => {
                self.state.page_up();
                Ok(ViewAction::None)
            }
            KeyCode::PageDown => {
                self.state.page_down();
                Ok(ViewAction::None)
            }
            KeyCode::Home => {
                self.state.jump_to_start();
                Ok(ViewAction::None)
            }
            KeyCode::End => {
                self.state.jump_to_end();
                Ok(ViewAction::None)
            }

            // Pause/Resume
            KeyCode::Char('p') | KeyCode::Char(' ') => {
                self.state.toggle_pause();
                Ok(ViewAction::None)
            }

            // Clear logs
            KeyCode::Char('c') if key.modifiers.contains(KeyModifiers::CONTROL) => {
                self.state.clear();
                Ok(ViewAction::None)
            }

            // Search
            KeyCode::Char('/') => {
                self.search_mode = true;
                self.search_input.clear();
                Ok(ViewAction::None)
            }

            // Clear search
            KeyCode::Char('n') if key.modifiers.contains(KeyModifiers::CONTROL) => {
                self.state.filter.clear_search();
                Ok(ViewAction::None)
            }

            // Filter by level
            KeyCode::Char('1') => {
                self.state.filter.toggle_level(LogLevel::Error);
                Ok(ViewAction::None)
            }
            KeyCode::Char('2') => {
                self.state.filter.toggle_level(LogLevel::Warn);
                Ok(ViewAction::None)
            }
            KeyCode::Char('3') => {
                self.state.filter.toggle_level(LogLevel::Info);
                Ok(ViewAction::None)
            }
            KeyCode::Char('4') => {
                self.state.filter.toggle_level(LogLevel::Debug);
                Ok(ViewAction::None)
            }
            KeyCode::Char('5') => {
                self.state.filter.toggle_level(LogLevel::Trace);
                Ok(ViewAction::None)
            }

            // Toggle all filters
            KeyCode::Char('a') => {
                if self.state.filter.enabled_count() == 5 {
                    self.state.filter.disable_all();
                } else {
                    self.state.filter.enable_all();
                }
                Ok(ViewAction::None)
            }

            // Manual refresh
            KeyCode::Char('r') if key.modifiers.contains(KeyModifiers::CONTROL) => {
                self.generate_mock_logs(5);
                Ok(ViewAction::None)
            }

            _ => Ok(ViewAction::None),
        }
    }

    async fn update(&mut self, _app_state: &mut AppState) -> Result<()> {
        // Generate new mock logs if not paused
        if !self.state.paused {
            // Generate 1-3 new logs per update
            let new_logs = (self.mock_log_counter % 3) + 1;
            self.generate_mock_logs(new_logs);
        }

        self.state.mark_refreshed();
        Ok(())
    }

    fn help_text(&self) -> Vec<(&str, &str)> {
        vec![
            ("q/Esc", "Back"),
            ("↑↓/k/j", "Navigate"),
            ("PgUp/PgDn", "Page up/down"),
            ("Home/End", "Start/End"),
            ("p/Space", "Pause/Resume"),
            ("/", "Search"),
            ("Ctrl+n", "Clear search"),
            ("1-5", "Toggle level filter"),
            ("a", "Toggle all filters"),
            ("Ctrl+c", "Clear logs"),
            ("Ctrl+r", "Manual refresh"),
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
    fn test_log_viewer_creation() {
        let theme = ThemeConfig::default();
        let viewer = LogViewerView::new("", &theme);

        assert_eq!(viewer.state.buffer_count(), 0);
        assert!(!viewer.search_mode);
        assert_eq!(viewer.mock_log_counter, 0);
    }

    #[test]
    fn test_mock_log_generation() {
        let theme = ThemeConfig::default();
        let mut viewer = LogViewerView::new("", &theme);

        viewer.generate_mock_logs(10);
        assert_eq!(viewer.state.buffer_count(), 10);
        assert_eq!(viewer.mock_log_counter, 10);
    }

    #[test]
    fn test_view_title() {
        let theme = ThemeConfig::default();
        let viewer = LogViewerView::new("", &theme);

        assert_eq!(viewer.title(), "Log Viewer");
    }
}
