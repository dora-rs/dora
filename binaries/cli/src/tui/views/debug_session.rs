use crossterm::event::{KeyCode, KeyEvent, KeyModifiers};
/// Debug Session View implementation (Issue #29 - Phase 1)
/// Provides a comprehensive debugging interface with mock data.
use ratatui::{
    Frame,
    layout::{Constraint, Direction, Layout, Rect},
    style::{Color, Modifier, Style},
    text::{Line, Span},
    widgets::{Block, Borders, List, ListItem, Paragraph, Wrap},
};
use std::time::Duration;

use super::{
    BaseView, View, ViewAction,
    debug_session_types::{
        Breakpoint, CallStackFrame, DebugPanel, DebugSessionState, ExecutionState, Variable,
    },
};
use crate::tui::{Result, app::AppState, theme::ThemeConfig};

/// Debug Session View - Interactive debugging interface
pub struct DebugSessionView {
    base: BaseView,
    pub state: DebugSessionState,
    theme: ThemeConfig,
}

impl DebugSessionView {
    /// Create a new Debug Session View
    pub fn new(source: &str, theme: &ThemeConfig) -> Self {
        let base = BaseView::new(format!("Debug Session - {}", source))
            .with_auto_refresh(Duration::from_millis(500));

        // Initialize state with mock data
        let mut state = DebugSessionState::new(source.to_string());
        Self::populate_mock_data(&mut state);
        state.set_execution_state(ExecutionState::Paused);

        Self {
            base,
            state,
            theme: theme.clone(),
        }
    }

    /// Populate state with mock debugging data
    fn populate_mock_data(state: &mut DebugSessionState) {
        // Add mock variables
        state.add_variable(Variable::new(
            "node_id".to_string(),
            "\"camera-node\"".to_string(),
            "String".to_string(),
            "local".to_string(),
        ));
        state.add_variable(Variable::new(
            "frame_count".to_string(),
            "142".to_string(),
            "u64".to_string(),
            "local".to_string(),
        ));
        state.add_variable(Variable::new(
            "buffer_size".to_string(),
            "1024".to_string(),
            "usize".to_string(),
            "local".to_string(),
        ));
        state.add_variable(Variable::new(
            "is_processing".to_string(),
            "true".to_string(),
            "bool".to_string(),
            "local".to_string(),
        ));
        state.add_variable(Variable::new(
            "config".to_string(),
            "Config { ... }".to_string(),
            "Config".to_string(),
            "global".to_string(),
        ));

        // Add mock call stack
        state.add_call_frame(CallStackFrame::new(
            "process_frame".to_string(),
            "camera_node.rs".to_string(),
            142,
        ));
        state.add_call_frame(CallStackFrame::new(
            "handle_event".to_string(),
            "node.rs".to_string(),
            89,
        ));
        state.add_call_frame(CallStackFrame::new(
            "run_node".to_string(),
            "runtime.rs".to_string(),
            203,
        ));
        state.add_call_frame(CallStackFrame::new(
            "main".to_string(),
            "main.rs".to_string(),
            12,
        ));

        // Add mock breakpoints
        state.add_breakpoint(Breakpoint::new(1, "camera_node.rs".to_string(), 142));
        state.add_breakpoint(Breakpoint::new(2, "node.rs".to_string(), 89));
        state.add_breakpoint(Breakpoint::new(3, "runtime.rs".to_string(), 203));

        // Enable first breakpoint and add hit count
        if let Some(bp) = state.breakpoints.get_mut(0) {
            bp.hit_count = 12;
        }
    }

    /// Render execution controls panel
    fn render_controls(&self, f: &mut Frame, area: Rect) {
        let state_style = match self.state.execution_state {
            ExecutionState::Running => Style::default().fg(Color::Green),
            ExecutionState::Paused => Style::default().fg(Color::Yellow),
            ExecutionState::Stopped => Style::default().fg(Color::Red),
        };

        let controls = vec![
            Line::from(vec![
                Span::styled("Status: ", Style::default().fg(Color::Gray)),
                Span::styled(
                    self.state.execution_state.symbol(),
                    state_style.add_modifier(Modifier::BOLD),
                ),
                Span::raw(" "),
                Span::styled(
                    self.state.execution_state.name(),
                    state_style.add_modifier(Modifier::BOLD),
                ),
            ]),
            Line::from(""),
            Line::from(vec![
                Span::styled(
                    "F5",
                    Style::default()
                        .fg(Color::Cyan)
                        .add_modifier(Modifier::BOLD),
                ),
                Span::raw("  Continue  "),
                Span::styled(
                    "F10",
                    Style::default()
                        .fg(Color::Cyan)
                        .add_modifier(Modifier::BOLD),
                ),
                Span::raw(" Step Over"),
            ]),
            Line::from(vec![
                Span::styled(
                    "F11",
                    Style::default()
                        .fg(Color::Cyan)
                        .add_modifier(Modifier::BOLD),
                ),
                Span::raw(" Step Into  "),
                Span::styled(
                    "F9",
                    Style::default()
                        .fg(Color::Cyan)
                        .add_modifier(Modifier::BOLD),
                ),
                Span::raw("  Toggle BP"),
            ]),
            Line::from(vec![
                Span::styled(
                    "Tab",
                    Style::default()
                        .fg(Color::Cyan)
                        .add_modifier(Modifier::BOLD),
                ),
                Span::raw(" Switch Panel  "),
                Span::styled(
                    "c",
                    Style::default()
                        .fg(Color::Cyan)
                        .add_modifier(Modifier::BOLD),
                ),
                Span::raw(" Clear"),
            ]),
        ];

        let block = Block::default()
            .borders(Borders::ALL)
            .border_style(self.theme.normal_border_style())
            .title(" Debug Controls ");

        let paragraph = Paragraph::new(controls)
            .block(block)
            .wrap(Wrap { trim: true });

        f.render_widget(paragraph, area);
    }

    /// Render code viewer panel (mock)
    fn render_code_viewer(&self, f: &mut Frame, area: Rect) {
        let location = &self.state.current_location;

        let code_lines = vec![
            Line::from("  140 │     let frame = capture_frame()?;"),
            Line::from("  141 │     let processed = process(frame);"),
            Line::from(vec![
                Span::styled(
                    "▶",
                    Style::default()
                        .fg(Color::Yellow)
                        .add_modifier(Modifier::BOLD),
                ),
                Span::raw(" 142 │     "),
                Span::styled("send_output", Style::default().fg(Color::Cyan)),
                Span::raw("(processed);"),
            ]),
            Line::from("  143 │     frame_count += 1;"),
            Line::from("  144 │     Ok(())"),
            Line::from("  145 │ }"),
        ];

        let title = format!(
            " {} @ {}:{} ",
            location.function_name, location.file_name, location.line_number
        );

        let block = Block::default()
            .borders(Borders::ALL)
            .border_style(self.theme.normal_border_style())
            .title(title);

        let paragraph = Paragraph::new(code_lines)
            .block(block)
            .wrap(Wrap { trim: false });

        f.render_widget(paragraph, area);
    }

    /// Render variables panel
    fn render_variables(&self, f: &mut Frame, area: Rect) {
        let items: Vec<ListItem> = self
            .state
            .variables
            .iter()
            .enumerate()
            .map(|(i, var)| {
                let style = if i == self.state.selected_variable
                    && self.state.active_panel == DebugPanel::Variables
                {
                    self.theme.highlight_style()
                } else {
                    Style::default()
                };

                ListItem::new(Line::from(var.format_display())).style(style)
            })
            .collect();

        let border_style = if self.state.active_panel == DebugPanel::Variables {
            self.theme.focused_border_style()
        } else {
            self.theme.normal_border_style()
        };

        let title = format!(" Variables ({}) ", self.state.variables.len());

        let list = List::new(items).block(
            Block::default()
                .borders(Borders::ALL)
                .border_style(border_style)
                .title(title),
        );

        f.render_widget(list, area);
    }

    /// Render call stack panel
    fn render_call_stack(&self, f: &mut Frame, area: Rect) {
        let items: Vec<ListItem> = self
            .state
            .call_stack
            .iter()
            .enumerate()
            .map(|(i, frame)| {
                let style = if i == self.state.selected_frame
                    && self.state.active_panel == DebugPanel::CallStack
                {
                    self.theme.highlight_style()
                } else {
                    Style::default()
                };

                let prefix = if i == 0 { "▶ " } else { "  " };
                ListItem::new(Line::from(format!("{}{}", prefix, frame.format_display())))
                    .style(style)
            })
            .collect();

        let border_style = if self.state.active_panel == DebugPanel::CallStack {
            self.theme.focused_border_style()
        } else {
            self.theme.normal_border_style()
        };

        let title = format!(" Call Stack ({}) ", self.state.call_stack.len());

        let list = List::new(items).block(
            Block::default()
                .borders(Borders::ALL)
                .border_style(border_style)
                .title(title),
        );

        f.render_widget(list, area);
    }

    /// Render breakpoints panel
    fn render_breakpoints(&self, f: &mut Frame, area: Rect) {
        let items: Vec<ListItem> = self
            .state
            .breakpoints
            .iter()
            .enumerate()
            .map(|(i, bp)| {
                let style = if i == self.state.selected_breakpoint
                    && self.state.active_panel == DebugPanel::Breakpoints
                {
                    self.theme.highlight_style()
                } else if bp.enabled {
                    Style::default()
                } else {
                    Style::default().fg(Color::DarkGray)
                };

                ListItem::new(Line::from(bp.format_display())).style(style)
            })
            .collect();

        let border_style = if self.state.active_panel == DebugPanel::Breakpoints {
            self.theme.focused_border_style()
        } else {
            self.theme.normal_border_style()
        };

        let enabled_count = self
            .state
            .breakpoints
            .iter()
            .filter(|bp| bp.enabled)
            .count();
        let title = format!(
            " Breakpoints ({}/{}) ",
            enabled_count,
            self.state.breakpoints.len()
        );

        let list = List::new(items).block(
            Block::default()
                .borders(Borders::ALL)
                .border_style(border_style)
                .title(title),
        );

        f.render_widget(list, area);
    }

    /// Render status bar
    fn render_status(&self, f: &mut Frame, area: Rect) {
        let status_line = Line::from(vec![
            Span::styled("Target: ", Style::default().fg(Color::Gray)),
            Span::styled(&self.state.target_name, Style::default().fg(Color::Cyan)),
            Span::raw("  │  "),
            Span::styled("Panel: ", Style::default().fg(Color::Gray)),
            Span::styled(
                self.state.active_panel.name(),
                Style::default()
                    .fg(Color::Yellow)
                    .add_modifier(Modifier::BOLD),
            ),
            Span::raw("  │  "),
            Span::styled("q", Style::default().fg(Color::Cyan)),
            Span::raw(" Quit"),
        ]);

        let block = Block::default()
            .borders(Borders::ALL)
            .border_style(self.theme.normal_border_style());

        let paragraph = Paragraph::new(status_line).block(block);

        f.render_widget(paragraph, area);
    }

    /// Handle execution control keys
    async fn handle_execution_control(&mut self, key: KeyCode) -> Result<ViewAction> {
        match key {
            KeyCode::F(5) => {
                // Continue/Run
                match self.state.execution_state {
                    ExecutionState::Paused | ExecutionState::Stopped => {
                        self.state.set_execution_state(ExecutionState::Running);
                        Ok(ViewAction::ShowStatus(
                            "Continuing execution...".to_string(),
                        ))
                    }
                    ExecutionState::Running => {
                        self.state.set_execution_state(ExecutionState::Paused);
                        Ok(ViewAction::ShowStatus("Paused execution".to_string()))
                    }
                }
            }
            KeyCode::F(9) => {
                // Toggle breakpoint
                if self.state.active_panel == DebugPanel::Breakpoints {
                    let selected_idx = self.state.selected_breakpoint;
                    if let Some(bp) = self.state.breakpoints.get(selected_idx) {
                        let bp_id = bp.id;
                        self.state.toggle_breakpoint(bp_id);
                        Ok(ViewAction::ShowStatus("Toggled breakpoint".to_string()))
                    } else {
                        Ok(ViewAction::None)
                    }
                } else {
                    Ok(ViewAction::None)
                }
            }
            KeyCode::F(10) => {
                // Step Over
                if self.state.execution_state != ExecutionState::Stopped {
                    self.state.set_execution_state(ExecutionState::Paused);
                    Ok(ViewAction::ShowStatus("Step over...".to_string()))
                } else {
                    Ok(ViewAction::None)
                }
            }
            KeyCode::F(11) => {
                // Step Into
                if self.state.execution_state != ExecutionState::Stopped {
                    self.state.set_execution_state(ExecutionState::Paused);
                    Ok(ViewAction::ShowStatus("Step into...".to_string()))
                } else {
                    Ok(ViewAction::None)
                }
            }
            _ => Ok(ViewAction::None),
        }
    }
}

impl View for DebugSessionView {
    fn render(&mut self, f: &mut Frame, area: Rect, _app_state: &AppState) {
        // Main layout: [Controls][Code][Panels][Status]
        let main_chunks = Layout::default()
            .direction(Direction::Vertical)
            .constraints([
                Constraint::Length(8),  // Controls
                Constraint::Min(8),     // Code viewer
                Constraint::Length(15), // Debug panels
                Constraint::Length(3),  // Status bar
            ])
            .split(area);

        // Render controls
        self.render_controls(f, main_chunks[0]);

        // Render code viewer
        self.render_code_viewer(f, main_chunks[1]);

        // Split debug panels horizontally
        let panel_chunks = Layout::default()
            .direction(Direction::Horizontal)
            .constraints([
                Constraint::Percentage(33),
                Constraint::Percentage(33),
                Constraint::Percentage(34),
            ])
            .split(main_chunks[2]);

        // Render debug panels
        self.render_variables(f, panel_chunks[0]);
        self.render_call_stack(f, panel_chunks[1]);
        self.render_breakpoints(f, panel_chunks[2]);

        // Render status bar
        self.render_status(f, main_chunks[3]);
    }

    async fn handle_key(&mut self, key: KeyEvent, _app_state: &mut AppState) -> Result<ViewAction> {
        // Handle execution controls first
        if matches!(
            key.code,
            KeyCode::F(5) | KeyCode::F(9) | KeyCode::F(10) | KeyCode::F(11)
        ) {
            return self.handle_execution_control(key.code).await;
        }

        // Handle other keys
        match key.code {
            KeyCode::Char('q') => Ok(ViewAction::PopView),
            KeyCode::Char('c') if key.modifiers.contains(KeyModifiers::CONTROL) => {
                Ok(ViewAction::Quit)
            }
            KeyCode::Char('c') => {
                self.state.clear();
                Ok(ViewAction::ShowStatus("Cleared debug session".to_string()))
            }
            KeyCode::Tab => {
                self.state.next_panel();
                Ok(ViewAction::None)
            }
            KeyCode::BackTab => {
                self.state.prev_panel();
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
            KeyCode::Char('?') => Ok(ViewAction::ShowHelp),
            KeyCode::Char('r') => Ok(ViewAction::Refresh),
            _ => Ok(ViewAction::None),
        }
    }

    async fn update(&mut self, _app_state: &mut AppState) -> Result<()> {
        // Auto-refresh logic could go here
        // For mock implementation, we just mark as refreshed
        self.state.mark_refreshed();
        Ok(())
    }

    fn help_text(&self) -> Vec<(&str, &str)> {
        vec![
            ("F5", "Continue/Pause execution"),
            ("F9", "Toggle breakpoint"),
            ("F10", "Step over"),
            ("F11", "Step into"),
            ("Tab", "Switch panel"),
            ("↑/↓", "Navigate items"),
            ("c", "Clear session"),
            ("r", "Refresh"),
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
    fn test_debug_session_view_creation() {
        let theme = ThemeConfig::default();
        let view = DebugSessionView::new("test-target", &theme);

        assert_eq!(view.title(), "Debug Session - test");
        assert_eq!(view.state.target_name, "test-target");
    }

    #[test]
    fn test_mock_data_populated() {
        let theme = ThemeConfig::default();
        let view = DebugSessionView::new("test", &theme);

        assert!(view.state.variables.len() > 0);
        assert!(view.state.call_stack.len() > 0);
        assert!(view.state.breakpoints.len() > 0);
    }

    #[test]
    fn test_initial_state() {
        let theme = ThemeConfig::default();
        let view = DebugSessionView::new("test", &theme);

        assert_eq!(view.state.execution_state, ExecutionState::Paused);
        assert_eq!(view.state.active_panel, DebugPanel::Variables);
        assert_eq!(view.state.selected_variable, 0);
    }

    #[test]
    fn test_auto_refresh_enabled() {
        let theme = ThemeConfig::default();
        let view = DebugSessionView::new("test", &theme);

        assert!(view.auto_refresh().is_some());
        assert_eq!(view.auto_refresh().unwrap(), Duration::from_millis(500));
    }

    #[test]
    fn test_help_text_available() {
        let theme = ThemeConfig::default();
        let view = DebugSessionView::new("test", &theme);

        let help = view.help_text();
        assert!(help.len() >= 8);
        assert!(help.iter().any(|(key, _)| *key == "F5"));
        assert!(help.iter().any(|(key, _)| *key == "F9"));
    }
}
