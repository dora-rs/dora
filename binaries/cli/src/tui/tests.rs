#[cfg(test)]

use crate::tui::{
    app::{DoraApp, ViewType, AppState, MessageLevel},
    theme::ThemeConfig,
    cli_integration::{CliContext, CommandHistory, TabCompletion, KeyBindings},
    views::{View, ViewAction, StateUpdate},
};

#[cfg(test)]
mod app_tests {
    use super::*;

    #[test]
    fn test_dora_app_creation() {
        let app = DoraApp::new(ViewType::Dashboard);
        assert!(matches!(app.current_view(), &ViewType::Dashboard));
        assert!(!app.should_quit());
    }

    #[test]
    fn test_app_with_context() {
        let context = CliContext::new();
        let app = DoraApp::new_with_context(ViewType::Dashboard, context);
        assert!(matches!(app.current_view(), &ViewType::Dashboard));
    }

    #[test]
    fn test_view_navigation() {
        let mut app = DoraApp::new(ViewType::Dashboard);
        
        // Test push view
        app.push_view(ViewType::DataflowManager);
        assert!(matches!(app.current_view(), &ViewType::DataflowManager));
        assert_eq!(app.view_stack_len(), 1);
        
        // Test push another view
        app.push_view(ViewType::SystemMonitor);
        assert!(matches!(app.current_view(), &ViewType::SystemMonitor));
        assert_eq!(app.view_stack_len(), 2);
        
        // Test pop view
        app.pop_view();
        assert!(matches!(app.current_view(), &ViewType::DataflowManager));
        assert_eq!(app.view_stack_len(), 1);
        
        // Test pop to original
        app.pop_view();
        assert!(matches!(app.current_view(), &ViewType::Dashboard));
        assert_eq!(app.view_stack_len(), 0);
        
        // Test pop when empty (should not crash)
        app.pop_view();
        assert!(matches!(app.current_view(), &ViewType::Dashboard));
        assert_eq!(app.view_stack_len(), 0);
    }

    #[test]
    fn test_view_switching() {
        let mut app = DoraApp::new(ViewType::Dashboard);
        
        app.switch_view(ViewType::Help);
        assert!(matches!(app.current_view(), &ViewType::Help));
        
        app.switch_view(ViewType::LogViewer { target: "test".to_string() });
        assert!(matches!(app.current_view(), &ViewType::LogViewer { .. }));
    }

    #[test]
    fn test_command_mode() {
        let mut app = DoraApp::new(ViewType::Dashboard);
        
        assert!(!app.is_in_command_mode());
        
        app.enter_command_mode();
        assert!(app.is_in_command_mode());
        
        app.exit_command_mode();
        assert!(!app.is_in_command_mode());
    }

    #[test]
    fn test_status_messages() {
        let mut app = DoraApp::new(ViewType::Dashboard);
        
        app.show_status_message("Test message".to_string(), MessageLevel::Info);
        assert!(app.has_status_messages());
        
        app.show_error_message("Error message".to_string());
        assert!(app.has_status_messages());
    }
}

#[cfg(test)]
mod theme_tests {
    use super::*;

    #[test]
    fn test_theme_creation() {
        let theme = ThemeConfig::default_dark();
        assert_eq!(theme.name, "dark");
        
        let theme = ThemeConfig::default_light();
        assert_eq!(theme.name, "light");
    }

    #[test]
    fn test_theme_loading() {
        let theme = ThemeConfig::load_user_theme();
        assert!(!theme.name.is_empty());
    }

    #[test]
    fn test_status_styling() {
        let theme = ThemeConfig::default_dark();
        
        // Test status-based colors
        let running_style = theme.status_style("running");
        let error_style = theme.status_style("error");
        let warning_style = theme.status_style("warning");
        
        // Colors should be different for different statuses
        assert_ne!(running_style, error_style);
        assert_ne!(error_style, warning_style);
    }

    #[test]
    fn test_percentage_styling() {
        let theme = ThemeConfig::default_dark();
        
        let low_style = theme.percentage_style(25.0);
        let high_style = theme.percentage_style(95.0);
        
        // High percentage should have different (warning) color
        assert_ne!(low_style, high_style);
    }
}

#[cfg(test)]
mod cli_integration_tests {
    use super::*;

    #[test]
    fn test_cli_context_creation() {
        let context = CliContext::new();
        assert!(context.working_directory.exists());
        assert!(!context.environment.is_empty());
    }

    #[test]
    fn test_command_history() {
        let mut history = CommandHistory::new();
        assert_eq!(history.commands.len(), 0);
        
        history.add_entry(
            "ps".to_string(), 
            true, 
            std::env::current_dir().unwrap()
        );
        
        assert_eq!(history.commands.len(), 1);
        assert_eq!(history.commands[0].command, "ps");
        assert!(history.commands[0].success);
        
        // Test search
        let results = history.search("ps");
        assert_eq!(results.len(), 1);
        
        let results = history.search("nonexistent");
        assert_eq!(results.len(), 0);
    }

    #[test]
    fn test_tab_completion() {
        let mut completion = TabCompletion::new();
        
        completion.update_suggestions("p");
        assert!(completion.suggestions.contains(&"ps".to_string()));
        
        completion.update_suggestions("st");
        assert!(completion.suggestions.contains(&"start".to_string()));
        
        completion.update_suggestions("--");
        assert!(completion.suggestions.iter().any(|s| s.starts_with("--")));
    }

    #[test]
    fn test_key_bindings() {
        let bindings = KeyBindings::default_bindings();
        
        // Test global bindings
        assert_eq!(bindings.get_binding("q", None), Some(&"quit".to_string()));
        assert_eq!(bindings.get_binding(":", None), Some(&"command_mode".to_string()));
        
        // Test view-specific bindings
        assert_eq!(
            bindings.get_binding("r", Some("dashboard")), 
            Some(&"refresh_dashboard".to_string())
        );
        
        // Test fallback to global
        assert_eq!(
            bindings.get_binding("q", Some("dashboard")), 
            Some(&"quit".to_string())
        );
    }

    #[test]
    fn test_key_bindings_modification() {
        let mut bindings = KeyBindings::default_bindings();
        
        // Add custom binding
        bindings.add_binding("x".to_string(), "custom_action".to_string(), None);
        assert_eq!(bindings.get_binding("x", None), Some(&"custom_action".to_string()));
        
        // Remove binding
        bindings.remove_binding("x", None);
        assert_eq!(bindings.get_binding("x", None), None);
    }
}

#[cfg(test)]
mod view_tests {
    use super::*;
    use crate::tui::views::{DashboardView, HelpView};

    #[test]
    fn test_dashboard_view() {
        let theme = ThemeConfig::default_dark();
        let view = DashboardView::new(&theme);
        
        assert_eq!(view.title(), "Dashboard");
        assert!(view.can_focus());
        
        let help_text = view.help_text();
        assert!(!help_text.is_empty());
    }

    #[test]
    fn test_help_view() {
        let theme = ThemeConfig::default_dark();
        let view = HelpView::new(&theme);
        
        assert_eq!(view.title(), "Help");
        
        let help_text = view.help_text();
        assert!(!help_text.is_empty());
        assert!(help_text.iter().any(|(key, _)| *key == "Esc"));
    }

    #[test]
    fn test_view_actions() {
        // Test ViewAction variants
        let action = ViewAction::SwitchView(ViewType::Dashboard);
        assert!(matches!(action, ViewAction::SwitchView(_)));
        
        let action = ViewAction::ExecuteCommand("ps".to_string());
        assert!(matches!(action, ViewAction::ExecuteCommand(_)));
        
        let action = ViewAction::ShowStatus("Test".to_string());
        assert!(matches!(action, ViewAction::ShowStatus(_)));
    }

    #[test]
    fn test_state_updates() {
        let update = StateUpdate::RefreshDataflows;
        assert!(matches!(update, StateUpdate::RefreshDataflows));
        
        let update = StateUpdate::UpdateSystemMetrics;
        assert!(matches!(update, StateUpdate::UpdateSystemMetrics));
        
        let update = StateUpdate::AddStatusMessage("Test".to_string(), MessageLevel::Info);
        assert!(matches!(update, StateUpdate::AddStatusMessage(_, _)));
    }
}

#[cfg(test)]
mod app_state_tests {
    use super::*;

    #[test]
    fn test_app_state_creation() {
        let state = AppState::default();
        assert!(state.dataflows.is_empty());
        assert!(state.status_messages.is_empty());
        assert!(state.last_error.is_none());
    }

    #[test]
    fn test_message_levels() {
        let info = MessageLevel::Info;
        let success = MessageLevel::Success;
        let warning = MessageLevel::Warning;
        let error = MessageLevel::Error;
        
        // Test that message levels can be cloned
        let _info_clone = info.clone();
        let _success_clone = success.clone();
        let _warning_clone = warning.clone();
        let _error_clone = error.clone();
    }
}

// Tests rely on the test helper methods in DoraApp implementation