/// Settings types and state management (Issue #30 - Phase 1)
/// Provides type definitions for settings configuration with mock data support.

use std::time::Instant;

/// Settings category
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SettingsCategory {
    General,
    Appearance,
    Performance,
    Debugging,
    Logging,
}

impl SettingsCategory {
    /// Get all available categories
    pub fn all() -> Vec<Self> {
        vec![
            Self::General,
            Self::Appearance,
            Self::Performance,
            Self::Debugging,
            Self::Logging,
        ]
    }

    /// Get display name for the category
    pub fn name(&self) -> &str {
        match self {
            Self::General => "General",
            Self::Appearance => "Appearance",
            Self::Performance => "Performance",
            Self::Debugging => "Debugging",
            Self::Logging => "Logging",
        }
    }

    /// Get icon for the category
    pub fn icon(&self) -> &str {
        match self {
            Self::General => "⚙",
            Self::Appearance => "🎨",
            Self::Performance => "⚡",
            Self::Debugging => "🐛",
            Self::Logging => "📝",
        }
    }

    /// Navigate to next category
    pub fn next(&self) -> Self {
        match self {
            Self::General => Self::Appearance,
            Self::Appearance => Self::Performance,
            Self::Performance => Self::Debugging,
            Self::Debugging => Self::Logging,
            Self::Logging => Self::General,
        }
    }

    /// Navigate to previous category
    pub fn prev(&self) -> Self {
        match self {
            Self::General => Self::Logging,
            Self::Appearance => Self::General,
            Self::Performance => Self::Appearance,
            Self::Debugging => Self::Performance,
            Self::Logging => Self::Debugging,
        }
    }
}

/// Setting value type
#[derive(Debug, Clone, PartialEq)]
pub enum SettingType {
    Boolean,
    Integer { min: i64, max: i64 },
    Float { min: f64, max: f64 },
    String { max_length: usize },
    Enum { options: Vec<String> },
}

impl SettingType {
    /// Get display name for the type
    pub fn name(&self) -> &str {
        match self {
            Self::Boolean => "Boolean",
            Self::Integer { .. } => "Integer",
            Self::Float { .. } => "Float",
            Self::String { .. } => "String",
            Self::Enum { .. } => "Enum",
        }
    }
}

/// Setting value
#[derive(Debug, Clone, PartialEq)]
pub enum SettingValue {
    Boolean(bool),
    Integer(i64),
    Float(f64),
    String(String),
}

impl SettingValue {
    /// Format value for display
    pub fn format_display(&self) -> String {
        match self {
            Self::Boolean(b) => if *b { "Enabled".to_string() } else { "Disabled".to_string() },
            Self::Integer(i) => i.to_string(),
            Self::Float(f) => format!("{:.2}", f),
            Self::String(s) => {
                if s.len() > 30 {
                    format!("{}...", &s[..27])
                } else {
                    s.clone()
                }
            }
        }
    }

    /// Check if value is compatible with type
    pub fn is_compatible_with(&self, setting_type: &SettingType) -> bool {
        match (self, setting_type) {
            (Self::Boolean(_), SettingType::Boolean) => true,
            (Self::Integer(i), SettingType::Integer { min, max }) => i >= min && i <= max,
            (Self::Float(f), SettingType::Float { min, max }) => f >= min && f <= max,
            (Self::String(s), SettingType::String { max_length }) => s.len() <= *max_length,
            (Self::String(_), SettingType::Enum { .. }) => true,
            _ => false,
        }
    }
}

/// Setting item
#[derive(Debug, Clone, PartialEq)]
pub struct SettingItem {
    pub key: String,
    pub display_name: String,
    pub description: String,
    pub value: SettingValue,
    pub setting_type: SettingType,
    pub default_value: SettingValue,
}

impl SettingItem {
    /// Create a new setting item
    pub fn new(
        key: String,
        display_name: String,
        description: String,
        value: SettingValue,
        setting_type: SettingType,
        default_value: SettingValue,
    ) -> Self {
        Self {
            key,
            display_name,
            description,
            value,
            setting_type,
            default_value,
        }
    }

    /// Format setting for display
    pub fn format_display(&self) -> String {
        format!("{}: {}", self.display_name, self.value.format_display())
    }

    /// Reset to default value
    pub fn reset_to_default(&mut self) {
        self.value = self.default_value.clone();
    }

    /// Toggle boolean value (only for boolean settings)
    pub fn toggle_boolean(&mut self) -> bool {
        if let SettingValue::Boolean(b) = &self.value {
            self.value = SettingValue::Boolean(!b);
            true
        } else {
            false
        }
    }
}

/// Settings state
pub struct SettingsState {
    pub current_category: SettingsCategory,
    pub general_settings: Vec<SettingItem>,
    pub appearance_settings: Vec<SettingItem>,
    pub performance_settings: Vec<SettingItem>,
    pub debugging_settings: Vec<SettingItem>,
    pub logging_settings: Vec<SettingItem>,
    pub selected_index: usize,
    pub last_refresh: Instant,
}

impl SettingsState {
    /// Create new settings state with mock data
    pub fn new() -> Self {
        Self {
            current_category: SettingsCategory::General,
            general_settings: Self::create_general_settings(),
            appearance_settings: Self::create_appearance_settings(),
            performance_settings: Self::create_performance_settings(),
            debugging_settings: Self::create_debugging_settings(),
            logging_settings: Self::create_logging_settings(),
            selected_index: 0,
            last_refresh: Instant::now(),
        }
    }

    /// Switch to a different category
    pub fn switch_category(&mut self, category: SettingsCategory) {
        self.current_category = category;
        self.selected_index = 0;
    }

    /// Navigate to next category
    pub fn next_category(&mut self) {
        self.current_category = self.current_category.next();
        self.selected_index = 0;
    }

    /// Navigate to previous category
    pub fn prev_category(&mut self) {
        self.current_category = self.current_category.prev();
        self.selected_index = 0;
    }

    /// Get current category settings
    pub fn current_settings(&self) -> &Vec<SettingItem> {
        match self.current_category {
            SettingsCategory::General => &self.general_settings,
            SettingsCategory::Appearance => &self.appearance_settings,
            SettingsCategory::Performance => &self.performance_settings,
            SettingsCategory::Debugging => &self.debugging_settings,
            SettingsCategory::Logging => &self.logging_settings,
        }
    }

    /// Get mutable current category settings
    pub fn current_settings_mut(&mut self) -> &mut Vec<SettingItem> {
        match self.current_category {
            SettingsCategory::General => &mut self.general_settings,
            SettingsCategory::Appearance => &mut self.appearance_settings,
            SettingsCategory::Performance => &mut self.performance_settings,
            SettingsCategory::Debugging => &mut self.debugging_settings,
            SettingsCategory::Logging => &mut self.logging_settings,
        }
    }

    /// Navigate down in settings list
    pub fn navigate_down(&mut self) {
        let count = self.current_settings().len();
        if count > 0 && self.selected_index + 1 < count {
            self.selected_index += 1;
        }
    }

    /// Navigate up in settings list
    pub fn navigate_up(&mut self) {
        if self.selected_index > 0 {
            self.selected_index -= 1;
        }
    }

    /// Get selected setting
    pub fn get_selected_setting(&self) -> Option<&SettingItem> {
        self.current_settings().get(self.selected_index)
    }

    /// Get selected setting (mutable)
    pub fn get_selected_setting_mut(&mut self) -> Option<&mut SettingItem> {
        let index = self.selected_index;
        self.current_settings_mut().get_mut(index)
    }

    /// Reset current category to defaults
    pub fn reset_category_to_defaults(&mut self) {
        for setting in self.current_settings_mut() {
            setting.reset_to_default();
        }
    }

    /// Reset all settings to defaults
    pub fn reset_all_to_defaults(&mut self) {
        for setting in &mut self.general_settings {
            setting.reset_to_default();
        }
        for setting in &mut self.appearance_settings {
            setting.reset_to_default();
        }
        for setting in &mut self.performance_settings {
            setting.reset_to_default();
        }
        for setting in &mut self.debugging_settings {
            setting.reset_to_default();
        }
        for setting in &mut self.logging_settings {
            setting.reset_to_default();
        }
    }

    /// Mark as refreshed
    pub fn mark_refreshed(&mut self) {
        self.last_refresh = Instant::now();
    }

    /// Get current category count
    pub fn current_category_count(&self) -> usize {
        self.current_settings().len()
    }

    // Mock data creation methods

    fn create_general_settings() -> Vec<SettingItem> {
        vec![
            SettingItem::new(
                "auto_save".to_string(),
                "Auto Save".to_string(),
                "Automatically save configuration changes".to_string(),
                SettingValue::Boolean(true),
                SettingType::Boolean,
                SettingValue::Boolean(true),
            ),
            SettingItem::new(
                "check_updates".to_string(),
                "Check for Updates".to_string(),
                "Automatically check for updates on startup".to_string(),
                SettingValue::Boolean(true),
                SettingType::Boolean,
                SettingValue::Boolean(true),
            ),
            SettingItem::new(
                "confirm_exit".to_string(),
                "Confirm Exit".to_string(),
                "Show confirmation dialog when exiting".to_string(),
                SettingValue::Boolean(false),
                SettingType::Boolean,
                SettingValue::Boolean(false),
            ),
        ]
    }

    fn create_appearance_settings() -> Vec<SettingItem> {
        vec![
            SettingItem::new(
                "theme".to_string(),
                "Theme".to_string(),
                "Visual theme for the user interface".to_string(),
                SettingValue::String("dark".to_string()),
                SettingType::Enum {
                    options: vec!["dark".to_string(), "light".to_string()],
                },
                SettingValue::String("dark".to_string()),
            ),
            SettingItem::new(
                "show_icons".to_string(),
                "Show Icons".to_string(),
                "Display icons in the interface".to_string(),
                SettingValue::Boolean(true),
                SettingType::Boolean,
                SettingValue::Boolean(true),
            ),
            SettingItem::new(
                "animation".to_string(),
                "Animations".to_string(),
                "Enable UI animations and transitions".to_string(),
                SettingValue::Boolean(true),
                SettingType::Boolean,
                SettingValue::Boolean(true),
            ),
        ]
    }

    fn create_performance_settings() -> Vec<SettingItem> {
        vec![
            SettingItem::new(
                "refresh_rate".to_string(),
                "Refresh Rate".to_string(),
                "UI update frequency in milliseconds".to_string(),
                SettingValue::Integer(100),
                SettingType::Integer { min: 50, max: 1000 },
                SettingValue::Integer(100),
            ),
            SettingItem::new(
                "buffer_size".to_string(),
                "Buffer Size".to_string(),
                "Maximum log entries to keep in memory".to_string(),
                SettingValue::Integer(1000),
                SettingType::Integer { min: 100, max: 10000 },
                SettingValue::Integer(1000),
            ),
            SettingItem::new(
                "max_dataflows".to_string(),
                "Max Dataflows".to_string(),
                "Maximum concurrent dataflows to display".to_string(),
                SettingValue::Integer(10),
                SettingType::Integer { min: 1, max: 50 },
                SettingValue::Integer(10),
            ),
        ]
    }

    fn create_debugging_settings() -> Vec<SettingItem> {
        vec![
            SettingItem::new(
                "enable_debug_mode".to_string(),
                "Debug Mode".to_string(),
                "Enable additional debugging information".to_string(),
                SettingValue::Boolean(false),
                SettingType::Boolean,
                SettingValue::Boolean(false),
            ),
            SettingItem::new(
                "break_on_error".to_string(),
                "Break on Error".to_string(),
                "Pause execution when errors occur".to_string(),
                SettingValue::Boolean(true),
                SettingType::Boolean,
                SettingValue::Boolean(true),
            ),
            SettingItem::new(
                "trace_level".to_string(),
                "Trace Level".to_string(),
                "Debug trace verbosity level".to_string(),
                SettingValue::String("info".to_string()),
                SettingType::Enum {
                    options: vec![
                        "error".to_string(),
                        "warn".to_string(),
                        "info".to_string(),
                        "debug".to_string(),
                        "trace".to_string(),
                    ],
                },
                SettingValue::String("info".to_string()),
            ),
        ]
    }

    fn create_logging_settings() -> Vec<SettingItem> {
        vec![
            SettingItem::new(
                "enable_logging".to_string(),
                "Enable Logging".to_string(),
                "Write logs to file".to_string(),
                SettingValue::Boolean(true),
                SettingType::Boolean,
                SettingValue::Boolean(true),
            ),
            SettingItem::new(
                "log_level".to_string(),
                "Log Level".to_string(),
                "Minimum log level to record".to_string(),
                SettingValue::String("info".to_string()),
                SettingType::Enum {
                    options: vec![
                        "error".to_string(),
                        "warn".to_string(),
                        "info".to_string(),
                        "debug".to_string(),
                    ],
                },
                SettingValue::String("info".to_string()),
            ),
            SettingItem::new(
                "max_log_files".to_string(),
                "Max Log Files".to_string(),
                "Maximum number of log files to keep".to_string(),
                SettingValue::Integer(10),
                SettingType::Integer { min: 1, max: 100 },
                SettingValue::Integer(10),
            ),
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_settings_category_all() {
        let categories = SettingsCategory::all();
        assert_eq!(categories.len(), 5);
        assert_eq!(categories[0], SettingsCategory::General);
        assert_eq!(categories[4], SettingsCategory::Logging);
    }

    #[test]
    fn test_settings_category_names() {
        assert_eq!(SettingsCategory::General.name(), "General");
        assert_eq!(SettingsCategory::Appearance.name(), "Appearance");
        assert_eq!(SettingsCategory::Performance.name(), "Performance");
    }

    #[test]
    fn test_settings_category_navigation() {
        let mut cat = SettingsCategory::General;
        cat = cat.next();
        assert_eq!(cat, SettingsCategory::Appearance);
        cat = cat.prev();
        assert_eq!(cat, SettingsCategory::General);
    }

    #[test]
    fn test_setting_type_names() {
        assert_eq!(SettingType::Boolean.name(), "Boolean");
        assert_eq!(
            SettingType::Integer { min: 0, max: 100 }.name(),
            "Integer"
        );
    }

    #[test]
    fn test_setting_value_format() {
        assert_eq!(SettingValue::Boolean(true).format_display(), "Enabled");
        assert_eq!(SettingValue::Boolean(false).format_display(), "Disabled");
        assert_eq!(SettingValue::Integer(42).format_display(), "42");
        assert_eq!(SettingValue::Float(3.14).format_display(), "3.14");
    }

    #[test]
    fn test_setting_value_compatibility() {
        let bool_val = SettingValue::Boolean(true);
        let bool_type = SettingType::Boolean;
        assert!(bool_val.is_compatible_with(&bool_type));

        let int_val = SettingValue::Integer(50);
        let int_type = SettingType::Integer { min: 0, max: 100 };
        assert!(int_val.is_compatible_with(&int_type));

        let int_val_out = SettingValue::Integer(150);
        assert!(!int_val_out.is_compatible_with(&int_type));
    }

    #[test]
    fn test_setting_item_creation() {
        let item = SettingItem::new(
            "test".to_string(),
            "Test Setting".to_string(),
            "A test setting".to_string(),
            SettingValue::Boolean(true),
            SettingType::Boolean,
            SettingValue::Boolean(false),
        );

        assert_eq!(item.key, "test");
        assert_eq!(item.display_name, "Test Setting");
        assert_eq!(item.value, SettingValue::Boolean(true));
    }

    #[test]
    fn test_setting_item_reset() {
        let mut item = SettingItem::new(
            "test".to_string(),
            "Test".to_string(),
            "Test".to_string(),
            SettingValue::Boolean(true),
            SettingType::Boolean,
            SettingValue::Boolean(false),
        );

        item.reset_to_default();
        assert_eq!(item.value, SettingValue::Boolean(false));
    }

    #[test]
    fn test_setting_item_toggle() {
        let mut item = SettingItem::new(
            "test".to_string(),
            "Test".to_string(),
            "Test".to_string(),
            SettingValue::Boolean(true),
            SettingType::Boolean,
            SettingValue::Boolean(true),
        );

        assert!(item.toggle_boolean());
        assert_eq!(item.value, SettingValue::Boolean(false));
        assert!(item.toggle_boolean());
        assert_eq!(item.value, SettingValue::Boolean(true));
    }

    #[test]
    fn test_settings_state_creation() {
        let state = SettingsState::new();
        assert_eq!(state.current_category, SettingsCategory::General);
        assert!(state.general_settings.len() > 0);
        assert!(state.appearance_settings.len() > 0);
    }

    #[test]
    fn test_settings_state_category_switch() {
        let mut state = SettingsState::new();
        state.switch_category(SettingsCategory::Performance);
        assert_eq!(state.current_category, SettingsCategory::Performance);
        assert_eq!(state.selected_index, 0);
    }

    #[test]
    fn test_settings_state_navigation() {
        let mut state = SettingsState::new();
        state.next_category();
        assert_eq!(state.current_category, SettingsCategory::Appearance);
        state.prev_category();
        assert_eq!(state.current_category, SettingsCategory::General);
    }

    #[test]
    fn test_settings_state_item_navigation() {
        let mut state = SettingsState::new();
        state.navigate_down();
        assert_eq!(state.selected_index, 1);
        state.navigate_up();
        assert_eq!(state.selected_index, 0);
        state.navigate_up(); // Should stay at 0
        assert_eq!(state.selected_index, 0);
    }

    #[test]
    fn test_settings_state_get_selected() {
        let state = SettingsState::new();
        let selected = state.get_selected_setting();
        assert!(selected.is_some());
        assert_eq!(selected.unwrap().key, "auto_save");
    }

    #[test]
    fn test_settings_state_reset_category() {
        let mut state = SettingsState::new();

        // Change a value
        if let Some(setting) = state.get_selected_setting_mut() {
            setting.value = SettingValue::Boolean(false);
        }

        // Reset
        state.reset_category_to_defaults();

        // Check reset
        let setting = state.get_selected_setting().unwrap();
        assert_eq!(setting.value, setting.default_value);
    }

    #[test]
    fn test_settings_state_current_settings() {
        let state = SettingsState::new();
        let settings = state.current_settings();
        assert!(settings.len() > 0);
    }

    #[test]
    fn test_settings_state_mark_refreshed() {
        let mut state = SettingsState::new();
        let before = state.last_refresh;
        std::thread::sleep(std::time::Duration::from_millis(10));
        state.mark_refreshed();
        assert!(state.last_refresh > before);
    }
}
