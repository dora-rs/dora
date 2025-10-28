/// Tests for Settings Configuration View (Issue #30)
#[cfg(test)]
mod settings_tests {
    use dora_cli::tui::theme::ThemeConfig;
    use dora_cli::tui::views::{
        SettingItem, SettingType, SettingValue, SettingsCategory, SettingsState, SettingsView, View,
    };

    // SettingsCategory tests
    #[test]
    fn test_settings_category_all() {
        let categories = SettingsCategory::all();
        assert_eq!(categories.len(), 5);
        assert_eq!(categories[0], SettingsCategory::General);
        assert_eq!(categories[1], SettingsCategory::Appearance);
        assert_eq!(categories[2], SettingsCategory::Performance);
        assert_eq!(categories[3], SettingsCategory::Debugging);
        assert_eq!(categories[4], SettingsCategory::Logging);
    }

    #[test]
    fn test_settings_category_names() {
        assert_eq!(SettingsCategory::General.name(), "General");
        assert_eq!(SettingsCategory::Appearance.name(), "Appearance");
        assert_eq!(SettingsCategory::Performance.name(), "Performance");
        assert_eq!(SettingsCategory::Debugging.name(), "Debugging");
        assert_eq!(SettingsCategory::Logging.name(), "Logging");
    }

    #[test]
    fn test_settings_category_icons() {
        assert_eq!(SettingsCategory::General.icon(), "⚙");
        assert_eq!(SettingsCategory::Appearance.icon(), "🎨");
        assert_eq!(SettingsCategory::Performance.icon(), "⚡");
        assert_eq!(SettingsCategory::Debugging.icon(), "🐛");
        assert_eq!(SettingsCategory::Logging.icon(), "📝");
    }

    #[test]
    fn test_settings_category_navigation() {
        let mut cat = SettingsCategory::General;
        cat = cat.next();
        assert_eq!(cat, SettingsCategory::Appearance);
        cat = cat.next();
        assert_eq!(cat, SettingsCategory::Performance);

        cat = cat.prev();
        assert_eq!(cat, SettingsCategory::Appearance);
        cat = cat.prev();
        assert_eq!(cat, SettingsCategory::General);

        // Test wrap-around
        cat = cat.prev();
        assert_eq!(cat, SettingsCategory::Logging);
    }

    // SettingType tests
    #[test]
    fn test_setting_type_names() {
        assert_eq!(SettingType::Boolean.name(), "Boolean");
        assert_eq!(SettingType::Integer { min: 0, max: 100 }.name(), "Integer");
        assert_eq!(SettingType::Float { min: 0.0, max: 1.0 }.name(), "Float");
        assert_eq!(SettingType::String { max_length: 50 }.name(), "String");
        assert_eq!(SettingType::Enum { options: vec![] }.name(), "Enum");
    }

    // SettingValue tests
    #[test]
    fn test_setting_value_format_boolean() {
        assert_eq!(SettingValue::Boolean(true).format_display(), "Enabled");
        assert_eq!(SettingValue::Boolean(false).format_display(), "Disabled");
    }

    #[test]
    fn test_setting_value_format_integer() {
        assert_eq!(SettingValue::Integer(42).format_display(), "42");
        assert_eq!(SettingValue::Integer(-10).format_display(), "-10");
    }

    #[test]
    fn test_setting_value_format_float() {
        assert_eq!(SettingValue::Float(3.14159).format_display(), "3.14");
        assert_eq!(SettingValue::Float(1.5).format_display(), "1.50");
    }

    #[test]
    fn test_setting_value_format_string() {
        assert_eq!(
            SettingValue::String("test".to_string()).format_display(),
            "test"
        );

        let long_string = "a".repeat(40);
        let formatted = SettingValue::String(long_string).format_display();
        assert!(formatted.ends_with("..."));
        assert_eq!(formatted.len(), 30);
    }

    #[test]
    fn test_setting_value_compatibility() {
        // Boolean compatibility
        let bool_val = SettingValue::Boolean(true);
        assert!(bool_val.is_compatible_with(&SettingType::Boolean));
        assert!(!bool_val.is_compatible_with(&SettingType::Integer { min: 0, max: 100 }));

        // Integer compatibility
        let int_val = SettingValue::Integer(50);
        let int_type = SettingType::Integer { min: 0, max: 100 };
        assert!(int_val.is_compatible_with(&int_type));

        let int_val_out = SettingValue::Integer(150);
        assert!(!int_val_out.is_compatible_with(&int_type));

        // Float compatibility
        let float_val = SettingValue::Float(0.5);
        let float_type = SettingType::Float { min: 0.0, max: 1.0 };
        assert!(float_val.is_compatible_with(&float_type));

        // String compatibility
        let string_val = SettingValue::String("test".to_string());
        let string_type = SettingType::String { max_length: 10 };
        assert!(string_val.is_compatible_with(&string_type));

        let long_string = SettingValue::String("a".repeat(20));
        assert!(!long_string.is_compatible_with(&string_type));
    }

    // SettingItem tests
    #[test]
    fn test_setting_item_creation() {
        let item = SettingItem::new(
            "auto_save".to_string(),
            "Auto Save".to_string(),
            "Automatically save changes".to_string(),
            SettingValue::Boolean(true),
            SettingType::Boolean,
            SettingValue::Boolean(false),
        );

        assert_eq!(item.key, "auto_save");
        assert_eq!(item.display_name, "Auto Save");
        assert_eq!(item.description, "Automatically save changes");
        assert_eq!(item.value, SettingValue::Boolean(true));
        assert_eq!(item.default_value, SettingValue::Boolean(false));
    }

    #[test]
    fn test_setting_item_format_display() {
        let item = SettingItem::new(
            "test".to_string(),
            "Test Setting".to_string(),
            "Test".to_string(),
            SettingValue::Integer(42),
            SettingType::Integer { min: 0, max: 100 },
            SettingValue::Integer(0),
        );

        let display = item.format_display();
        assert!(display.contains("Test Setting"));
        assert!(display.contains("42"));
    }

    #[test]
    fn test_setting_item_reset_to_default() {
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
        assert_eq!(item.value, item.default_value);
    }

    #[test]
    fn test_setting_item_toggle_boolean() {
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

        // Test non-boolean setting
        let mut int_item = SettingItem::new(
            "test".to_string(),
            "Test".to_string(),
            "Test".to_string(),
            SettingValue::Integer(42),
            SettingType::Integer { min: 0, max: 100 },
            SettingValue::Integer(0),
        );

        assert!(!int_item.toggle_boolean());
        assert_eq!(int_item.value, SettingValue::Integer(42));
    }

    // SettingsState tests
    #[test]
    fn test_settings_state_new() {
        let state = SettingsState::new();

        assert_eq!(state.current_category, SettingsCategory::General);
        assert_eq!(state.selected_index, 0);
        assert!(state.general_settings.len() > 0);
        assert!(state.appearance_settings.len() > 0);
        assert!(state.performance_settings.len() > 0);
        assert!(state.debugging_settings.len() > 0);
        assert!(state.logging_settings.len() > 0);
    }

    #[test]
    fn test_settings_state_switch_category() {
        let mut state = SettingsState::new();
        state.selected_index = 2;

        state.switch_category(SettingsCategory::Performance);

        assert_eq!(state.current_category, SettingsCategory::Performance);
        assert_eq!(state.selected_index, 0); // Reset on category switch
    }

    #[test]
    fn test_settings_state_next_prev_category() {
        let mut state = SettingsState::new();

        state.next_category();
        assert_eq!(state.current_category, SettingsCategory::Appearance);

        state.next_category();
        assert_eq!(state.current_category, SettingsCategory::Performance);

        state.prev_category();
        assert_eq!(state.current_category, SettingsCategory::Appearance);

        state.prev_category();
        assert_eq!(state.current_category, SettingsCategory::General);
    }

    #[test]
    fn test_settings_state_current_settings() {
        let state = SettingsState::new();
        let settings = state.current_settings();

        assert!(settings.len() > 0);
        assert_eq!(settings, &state.general_settings);
    }

    #[test]
    fn test_settings_state_navigate_up_down() {
        let mut state = SettingsState::new();

        assert_eq!(state.selected_index, 0);

        state.navigate_down();
        assert_eq!(state.selected_index, 1);

        state.navigate_down();
        assert_eq!(state.selected_index, 2);

        state.navigate_up();
        assert_eq!(state.selected_index, 1);

        state.navigate_up();
        assert_eq!(state.selected_index, 0);

        // Should not go below 0
        state.navigate_up();
        assert_eq!(state.selected_index, 0);
    }

    #[test]
    fn test_settings_state_navigate_bounds() {
        let mut state = SettingsState::new();
        let max_index = state.current_settings().len() - 1;

        // Navigate to the end
        for _ in 0..=max_index {
            state.navigate_down();
        }

        // Should stop at max_index
        assert_eq!(state.selected_index, max_index);

        // Try to go further - should stay at max_index
        state.navigate_down();
        assert_eq!(state.selected_index, max_index);
    }

    #[test]
    fn test_settings_state_get_selected_setting() {
        let state = SettingsState::new();
        let selected = state.get_selected_setting();

        assert!(selected.is_some());
        let setting = selected.unwrap();
        assert_eq!(setting.key, "auto_save");
    }

    #[test]
    fn test_settings_state_get_selected_setting_mut() {
        let mut state = SettingsState::new();

        if let Some(setting) = state.get_selected_setting_mut() {
            setting.value = SettingValue::Boolean(false);
        }

        let selected = state.get_selected_setting().unwrap();
        assert_eq!(selected.value, SettingValue::Boolean(false));
    }

    #[test]
    fn test_settings_state_reset_category_to_defaults() {
        let mut state = SettingsState::new();

        // Modify first setting
        if let Some(setting) = state.current_settings_mut().get_mut(0) {
            setting.value = SettingValue::Boolean(false);
        }

        // Reset category
        state.reset_category_to_defaults();

        // Check it's back to default
        let setting = state.current_settings().get(0).unwrap();
        assert_eq!(setting.value, setting.default_value);
    }

    #[test]
    fn test_settings_state_reset_all_to_defaults() {
        let mut state = SettingsState::new();

        // Modify settings in different categories
        state.general_settings[0].value = SettingValue::Boolean(false);
        state.appearance_settings[0].value = SettingValue::String("modified".to_string());

        // Reset all
        state.reset_all_to_defaults();

        // Check all are back to defaults
        assert_eq!(
            state.general_settings[0].value,
            state.general_settings[0].default_value
        );
        assert_eq!(
            state.appearance_settings[0].value,
            state.appearance_settings[0].default_value
        );
    }

    #[test]
    fn test_settings_state_mark_refreshed() {
        let mut state = SettingsState::new();
        let before = state.last_refresh;

        std::thread::sleep(std::time::Duration::from_millis(10));
        state.mark_refreshed();

        assert!(state.last_refresh > before);
    }

    #[test]
    fn test_settings_state_current_category_count() {
        let state = SettingsState::new();
        let count = state.current_category_count();

        assert!(count > 0);
        assert_eq!(count, state.general_settings.len());
    }

    // SettingsView tests
    #[test]
    fn test_settings_view_creation() {
        let theme = ThemeConfig::default();
        let view = SettingsView::new(&theme);

        assert_eq!(view.title(), "Settings");
        assert_eq!(view.state.current_category, SettingsCategory::General);
    }

    #[test]
    fn test_settings_view_has_mock_data() {
        let theme = ThemeConfig::default();
        let view = SettingsView::new(&theme);

        assert!(view.state.general_settings.len() > 0);
        assert!(view.state.appearance_settings.len() > 0);
        assert!(view.state.performance_settings.len() > 0);
        assert!(view.state.debugging_settings.len() > 0);
        assert!(view.state.logging_settings.len() > 0);
    }

    #[test]
    fn test_settings_view_auto_refresh() {
        let theme = ThemeConfig::default();
        let view = SettingsView::new(&theme);

        assert!(view.auto_refresh().is_some());
        assert_eq!(
            view.auto_refresh().unwrap(),
            std::time::Duration::from_millis(500)
        );
    }

    #[test]
    fn test_settings_view_help_text() {
        let theme = ThemeConfig::default();
        let view = SettingsView::new(&theme);

        let help = view.help_text();
        assert!(help.len() >= 6);
        assert!(help.iter().any(|(key, _)| *key == "Tab"));
        assert!(help.iter().any(|(key, _)| *key == "Space"));
        assert!(help.iter().any(|(key, _)| *key == "r"));
        assert!(help.iter().any(|(key, _)| *key == "R"));
    }

    #[test]
    fn test_complete_settings_workflow() {
        let mut state = SettingsState::new();

        // Start in General
        assert_eq!(state.current_category, SettingsCategory::General);

        // Navigate to Appearance
        state.next_category();
        assert_eq!(state.current_category, SettingsCategory::Appearance);

        // Select second setting
        state.navigate_down();
        assert_eq!(state.selected_index, 1);

        // Modify a boolean setting
        if let Some(setting) = state.get_selected_setting_mut() {
            if let SettingValue::Boolean(_) = setting.value {
                setting.toggle_boolean();
            }
        }

        // Navigate to Performance
        state.next_category();
        assert_eq!(state.current_category, SettingsCategory::Performance);
        assert_eq!(state.selected_index, 0); // Reset on category switch

        // Modify an integer setting
        if let Some(setting) = state.get_selected_setting_mut() {
            setting.value = SettingValue::Integer(200);
        }

        // Reset current category
        state.reset_category_to_defaults();
        let setting = state.get_selected_setting().unwrap();
        assert_eq!(setting.value, setting.default_value);

        // Go back to Appearance
        state.prev_category();
        assert_eq!(state.current_category, SettingsCategory::Appearance);

        // Reset all settings
        state.reset_all_to_defaults();

        // Verify all settings are at defaults
        for setting in &state.general_settings {
            assert_eq!(setting.value, setting.default_value);
        }
        for setting in &state.appearance_settings {
            assert_eq!(setting.value, setting.default_value);
        }
    }

    #[test]
    fn test_all_categories_have_settings() {
        let mut state = SettingsState::new();

        for category in SettingsCategory::all() {
            state.switch_category(category);
            let settings = match category {
                SettingsCategory::General => &state.general_settings,
                SettingsCategory::Appearance => &state.appearance_settings,
                SettingsCategory::Performance => &state.performance_settings,
                SettingsCategory::Debugging => &state.debugging_settings,
                SettingsCategory::Logging => &state.logging_settings,
            };
            assert!(
                settings.len() > 0,
                "Category {:?} should have settings",
                category
            );
        }
    }
}
