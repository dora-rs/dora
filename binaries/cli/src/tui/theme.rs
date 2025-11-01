use ratatui::{
    style::{Color, Modifier, Style},
    widgets::{Block, BorderType, Borders},
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
    pub info: Color,
    pub background: Color,
    pub foreground: Color,
    pub text: Color,
    pub muted: Color,
    pub border: Color,
    pub border_focused: Color,
    pub selection: Color,
    pub highlight: Color,
    pub accent: Color,
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

    pub fn from_name(name: &str) -> Self {
        match name.to_lowercase().as_str() {
            "light" => Self::default_light(),
            other => {
                if other != "dark" {
                    tracing::debug!("Unknown theme '{}', falling back to dark", other);
                }
                Self::default_dark()
            }
        }
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
                info: Color::Blue,
                background: Color::Black,
                foreground: Color::White,
                text: Color::White,
                muted: Color::Gray,
                border: Color::DarkGray,
                border_focused: Color::Cyan,
                selection: Color::Blue,
                highlight: Color::Yellow,
                accent: Color::Magenta,
            },
            styles: StyleConfig {
                border_style: BorderType::Rounded,
                highlight_style: Style::default()
                    .fg(Color::Cyan)
                    .add_modifier(Modifier::BOLD),
                selection_style: Style::default()
                    .bg(Color::DarkGray)
                    .add_modifier(Modifier::BOLD),
                status_style: Style::default().fg(Color::Cyan),
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
                warning: Color::Rgb(255, 165, 0), // Orange
                error: Color::Red,
                info: Color::Blue,
                background: Color::White,
                foreground: Color::Black,
                text: Color::Black,
                muted: Color::Gray,
                border: Color::DarkGray,
                border_focused: Color::Blue,
                selection: Color::LightBlue,
                highlight: Color::Yellow,
                accent: Color::Magenta,
            },
            styles: StyleConfig {
                border_style: BorderType::Rounded,
                highlight_style: Style::default()
                    .fg(Color::Blue)
                    .add_modifier(Modifier::BOLD),
                selection_style: Style::default()
                    .bg(Color::LightBlue)
                    .add_modifier(Modifier::BOLD),
                status_style: Style::default().fg(Color::Blue),
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
                    .add_modifier(Modifier::BOLD),
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
            "high" | "critical" => Style::default()
                .fg(self.colors.error)
                .add_modifier(Modifier::BOLD),
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

    // Additional helper methods matching Issue #23 requirements

    pub fn normal_style(&self) -> Style {
        Style::default()
            .fg(self.colors.foreground)
            .bg(self.colors.background)
    }

    pub fn focused_style(&self) -> Style {
        Style::default()
            .fg(self.colors.foreground)
            .bg(self.colors.background)
            .add_modifier(Modifier::BOLD)
    }

    pub fn selected_item_style(&self) -> Style {
        Style::default()
            .fg(self.colors.background)
            .bg(self.colors.selection)
            .add_modifier(Modifier::BOLD)
    }

    pub fn normal_item_style(&self) -> Style {
        Style::default()
            .fg(self.colors.foreground)
            .bg(self.colors.background)
    }

    pub fn focused_border_style(&self) -> Style {
        Style::default().fg(self.colors.border_focused)
    }

    pub fn normal_border_style(&self) -> Style {
        Style::default().fg(self.colors.border)
    }

    pub fn highlight_style(&self) -> Style {
        Style::default()
            .fg(self.colors.background)
            .bg(self.colors.highlight)
            .add_modifier(Modifier::BOLD)
    }

    pub fn success_style(&self) -> Style {
        Style::default().fg(self.colors.success)
    }

    pub fn warning_style(&self) -> Style {
        Style::default().fg(self.colors.warning)
    }

    pub fn error_style(&self) -> Style {
        Style::default().fg(self.colors.error)
    }

    pub fn info_style(&self) -> Style {
        Style::default().fg(self.colors.info)
    }

    pub fn muted_style(&self) -> Style {
        Style::default().fg(self.colors.muted)
    }

    pub fn chart_line_style(&self) -> Style {
        Style::default().fg(self.colors.primary)
    }

    pub fn axis_style(&self) -> Style {
        Style::default().fg(self.colors.muted)
    }
}

impl Default for ThemeConfig {
    fn default() -> Self {
        Self::default_dark()
    }
}

/// Theme manager for managing available themes and switching between them
#[derive(Debug)]
pub struct ThemeManager {
    current_theme: ThemeConfig,
    available_themes: Vec<ThemeConfig>,
}

impl ThemeManager {
    /// Load the theme manager with default themes
    pub fn load_default() -> Self {
        let themes = vec![ThemeConfig::default_dark(), ThemeConfig::default_light()];

        Self {
            current_theme: themes[0].clone(),
            available_themes: themes,
        }
    }

    /// Get the current theme
    pub fn current_theme(&self) -> &ThemeConfig {
        &self.current_theme
    }

    /// Switch to a theme by name
    pub fn switch_theme(&mut self, theme_name: &str) -> Result<(), String> {
        if let Some(theme) = self.available_themes.iter().find(|t| t.name == theme_name) {
            self.current_theme = theme.clone();
            Ok(())
        } else {
            Err(format!("Theme '{theme_name}' not found"))
        }
    }

    /// Get list of available theme names
    pub fn available_theme_names(&self) -> Vec<String> {
        self.available_themes
            .iter()
            .map(|t| t.name.clone())
            .collect()
    }

    /// Add a custom theme
    pub fn add_theme(&mut self, theme: ThemeConfig) {
        // Remove existing theme with same name
        self.available_themes.retain(|t| t.name != theme.name);
        self.available_themes.push(theme);
    }

    /// Get number of available themes
    pub fn theme_count(&self) -> usize {
        self.available_themes.len()
    }
}

impl Default for ThemeManager {
    fn default() -> Self {
        Self::load_default()
    }
}

// Note: Serialization for external ratatui types removed due to orphan rules
// If needed in the future, wrap these types in newtype wrappers
