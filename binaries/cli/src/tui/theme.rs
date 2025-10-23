use ratatui::{
    style::{Color, Modifier, Style},
    widgets::{Block, Borders, BorderType},
};

#[derive(Debug, Clone)]
pub struct ThemeConfig {
    pub name: String,
    pub colors: ColorScheme,
    pub styles: StyleConfig,
}

#[derive(Debug, Clone)]
pub struct ColorScheme {
    pub primary: Color,
    pub secondary: Color,
    pub success: Color,
    pub warning: Color,
    pub error: Color,
    pub background: Color,
    pub text: Color,
    pub muted: Color,
    pub border: Color,
}

#[derive(Debug, Clone)]
pub struct StyleConfig {
    pub border_style: BorderType,
    pub highlight_style: Style,
    pub selection_style: Style,
    pub status_style: Style,
}

impl ThemeConfig {
    pub fn load_user_theme() -> Self {
        // For now, load default theme
        // In the future, this would check user config files
        Self::default_dark()
    }
    
    pub fn default_dark() -> Self {
        Self {
            name: "dark".to_string(),
            colors: ColorScheme {
                primary: Color::Cyan,
                secondary: Color::Blue,
                success: Color::Green,
                warning: Color::Yellow,
                error: Color::Red,
                background: Color::Black,
                text: Color::White,
                muted: Color::Gray,
                border: Color::DarkGray,
            },
            styles: StyleConfig {
                border_style: BorderType::Rounded,
                highlight_style: Style::default()
                    .fg(Color::Cyan)
                    .add_modifier(Modifier::BOLD),
                selection_style: Style::default()
                    .bg(Color::DarkGray)
                    .add_modifier(Modifier::BOLD),
                status_style: Style::default()
                    .fg(Color::Cyan),
            },
        }
    }
    
    pub fn default_light() -> Self {
        Self {
            name: "light".to_string(),
            colors: ColorScheme {
                primary: Color::Blue,
                secondary: Color::Magenta,
                success: Color::Green,
                warning: Color::Yellow,
                error: Color::Red,
                background: Color::White,
                text: Color::Black,
                muted: Color::Gray,
                border: Color::Gray,
            },
            styles: StyleConfig {
                border_style: BorderType::Rounded,
                highlight_style: Style::default()
                    .fg(Color::Blue)
                    .add_modifier(Modifier::BOLD),
                selection_style: Style::default()
                    .bg(Color::LightBlue)
                    .add_modifier(Modifier::BOLD),
                status_style: Style::default()
                    .fg(Color::Blue),
            },
        }
    }
    
    pub fn styled_block<'a>(&self, title: &'a str) -> Block<'a> {
        Block::default()
            .title(title)
            .borders(Borders::ALL)
            .border_type(self.styles.border_style)
            .border_style(Style::default().fg(self.colors.border))
            .title_style(
                Style::default()
                    .fg(self.colors.text)
                    .add_modifier(Modifier::BOLD)
            )
    }
    
    pub fn status_style(&self, status: &str) -> Style {
        let color = match status.to_lowercase().as_str() {
            "running" | "active" | "healthy" | "ok" | "success" => self.colors.success,
            "warning" | "degraded" | "slow" | "pending" => self.colors.warning,
            "error" | "failed" | "stopped" | "critical" | "down" => self.colors.error,
            "info" | "idle" | "waiting" => self.colors.primary,
            _ => self.colors.text,
        };
        Style::default().fg(color)
    }
    
    pub fn priority_style(&self, priority: &str) -> Style {
        match priority.to_lowercase().as_str() {
            "high" | "critical" => Style::default().fg(self.colors.error).add_modifier(Modifier::BOLD),
            "medium" | "normal" => Style::default().fg(self.colors.warning),
            "low" => Style::default().fg(self.colors.muted),
            _ => Style::default().fg(self.colors.text),
        }
    }
    
    pub fn percentage_style(&self, percentage: f32) -> Style {
        let color = if percentage >= 90.0 {
            self.colors.error
        } else if percentage >= 70.0 {
            self.colors.warning
        } else if percentage >= 50.0 {
            self.colors.primary
        } else {
            self.colors.success
        };
        Style::default().fg(color)
    }
    
    pub fn diff_style(&self, change: i32) -> Style {
        if change > 0 {
            Style::default().fg(self.colors.success)
        } else if change < 0 {
            Style::default().fg(self.colors.error)
        } else {
            Style::default().fg(self.colors.muted)
        }
    }
    
    pub fn highlight_text(&self, _text: &str, highlighted: bool) -> Style {
        if highlighted {
            self.styles.selection_style
        } else {
            Style::default().fg(self.colors.text)
        }
    }
    
    pub fn table_header_style(&self) -> Style {
        Style::default()
            .fg(self.colors.primary)
            .add_modifier(Modifier::BOLD)
    }
    
    pub fn table_row_style(&self, selected: bool) -> Style {
        if selected {
            self.styles.selection_style
        } else {
            Style::default().fg(self.colors.text)
        }
    }
    
    pub fn progress_bar_style(&self) -> Style {
        Style::default().fg(self.colors.primary)
    }
    
    pub fn border_style(&self) -> Style {
        Style::default().fg(self.colors.border)
    }
}

impl Default for ThemeConfig {
    fn default() -> Self {
        Self::default_dark()
    }
}

// Note: Serialization for external ratatui types removed due to orphan rules
// If needed in the future, wrap these types in newtype wrappers