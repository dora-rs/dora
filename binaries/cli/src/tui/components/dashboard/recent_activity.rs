use ratatui::{
    Frame,
    layout::Rect,
    style::{Color, Modifier, Style},
    text::{Line, Span},
    widgets::{Block, Borders, List, ListItem},
};
use std::future::Future;
/// Recent Activity Component for Dashboard (Issue #24)
use std::pin::Pin;

use crate::tui::{
    Result,
    app::AppState,
    components::{Component, ComponentEvent, ComponentType},
    theme::ThemeConfig,
    views::{ActivityType, RecentActivity, ViewAction},
};

pub struct RecentActivityComponent {
    activity: RecentActivity,
    focused: bool,
    display_limit: usize,
}

impl RecentActivityComponent {
    pub fn new(display_limit: usize) -> Self {
        Self {
            activity: RecentActivity::default(),
            focused: false,
            display_limit,
        }
    }

    pub fn set_activity(&mut self, activity: RecentActivity) {
        self.activity = activity;
    }

    fn format_elapsed(&self, elapsed: std::time::Duration) -> String {
        let secs = elapsed.as_secs();
        if secs < 60 {
            format!("{secs}s ago")
        } else if secs < 3600 {
            format!("{}m ago", secs / 60)
        } else if secs < 86400 {
            format!("{}h ago", secs / 3600)
        } else {
            format!("{}d ago", secs / 86400)
        }
    }

    fn activity_color(&self, activity_type: ActivityType, theme: &ThemeConfig) -> Color {
        match activity_type {
            ActivityType::DataflowStarted | ActivityType::NodeStarted => theme.colors.success,
            ActivityType::DataflowStopped | ActivityType::NodeStopped => theme.colors.warning,
            ActivityType::DataflowFailed | ActivityType::NodeFailed => theme.colors.error,
            ActivityType::SystemEvent => theme.colors.info,
            ActivityType::UserAction => theme.colors.primary,
        }
    }
}

impl Default for RecentActivityComponent {
    fn default() -> Self {
        Self::new(10)
    }
}

impl Component for RecentActivityComponent {
    fn update<'a>(
        &'a mut self,
        _app_state: &'a AppState,
    ) -> Pin<Box<dyn Future<Output = Result<()>> + Send + 'a>> {
        Box::pin(async move {
            // Activity updated from dashboard view
            Ok(())
        })
    }

    fn render(&self, frame: &mut Frame, area: Rect, theme: &ThemeConfig, _app_state: &AppState) {
        let block = Block::default()
            .title("Recent Activity")
            .borders(Borders::ALL)
            .border_style(if self.focused {
                theme.focused_border_style()
            } else {
                theme.normal_border_style()
            });

        let _inner_area = block.inner(area);

        if self.activity.activities.is_empty() {
            let empty_list = List::new(vec![ListItem::new(Line::from(Span::styled(
                "No recent activity",
                Style::default().fg(theme.colors.muted),
            )))])
            .block(block);

            frame.render_widget(empty_list, area);
            return;
        }

        // Get recent activities (newest first)
        let recent = self.activity.recent(self.display_limit);

        let items: Vec<ListItem> = recent
            .iter()
            .map(|item| {
                let elapsed = item.timestamp.elapsed();
                let time_str = self.format_elapsed(elapsed);
                let color = self.activity_color(item.event_type, theme);

                let line = Line::from(vec![
                    Span::styled(
                        item.event_type.icon(),
                        Style::default().fg(color).add_modifier(Modifier::BOLD),
                    ),
                    Span::raw(" "),
                    Span::styled(&item.description, Style::default().fg(theme.colors.text)),
                    Span::raw(" "),
                    Span::styled(
                        format!("({time_str})"),
                        Style::default().fg(theme.colors.muted),
                    ),
                ]);

                ListItem::new(line)
            })
            .collect();

        let list = List::new(items).block(block);

        frame.render_widget(list, area);
    }

    fn handle_event<'a>(
        &'a mut self,
        _event: ComponentEvent,
        _app_state: &'a AppState,
    ) -> Pin<Box<dyn Future<Output = Result<ViewAction>> + Send + 'a>> {
        Box::pin(async move { Ok(ViewAction::None) })
    }

    fn component_type(&self) -> ComponentType {
        ComponentType::PropertyInspector // Using existing type
    }

    fn is_focusable(&self) -> bool {
        false
    }

    fn is_focused(&self) -> bool {
        self.focused
    }

    fn set_focus(&mut self, focused: bool) {
        self.focused = focused;
    }
}
