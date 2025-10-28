// TODO(Issue #72): These tests need to be updated for the new TUI architecture
// They reference private methods and removed types (CommandSuggestion, SuggestionType).
// See TESTING_STATUS.md for details and action plan.
#[cfg(disabled)]
mod tests {
    use super::*;
    use crate::tui::app::AppState;
    use tokio::test;

    #[test]
    fn test_command_mode_activation() {
        let mut manager = CommandModeManager::new();
        assert!(!manager.is_active());

        manager.activate();
        assert!(manager.is_active());

        manager.deactivate();
        assert!(!manager.is_active());
    }

    #[test]
    fn test_completion_state() {
        let mut state = CompletionState::new();

        state.suggestions = vec![
            CommandSuggestion {
                text: "ps".to_string(),
                description: "List dataflows".to_string(),
                suggestion_type: SuggestionType::Command,
                priority: 80,
            },
            CommandSuggestion {
                text: "push".to_string(),
                description: "Push data".to_string(),
                suggestion_type: SuggestionType::Command,
                priority: 70,
            },
        ];

        // Test selection
        state.select_next();
        assert_eq!(state.selected_index, Some(0));

        state.select_next();
        assert_eq!(state.selected_index, Some(1));

        state.select_next(); // Should wrap around
        assert_eq!(state.selected_index, Some(0));

        state.select_previous();
        assert_eq!(state.selected_index, Some(1));
    }

    #[test]
    fn test_history_navigation() {
        let mut nav = HistoryNavigation::new();
        let history = vec![
            "ps".to_string(),
            "start test.yaml".to_string(),
            "logs test".to_string(),
        ];

        // Navigate up from empty
        assert_eq!(nav.navigate_up(&history), Some("logs test".to_string()));
        assert_eq!(
            nav.navigate_up(&history),
            Some("start test.yaml".to_string())
        );
        assert_eq!(nav.navigate_up(&history), Some("ps".to_string()));
        assert_eq!(nav.navigate_up(&history), None); // At oldest

        // Navigate down
        assert_eq!(
            nav.navigate_down(&history),
            Some("start test.yaml".to_string())
        );
        assert_eq!(nav.navigate_down(&history), Some("logs test".to_string()));
    }

    #[tokio::test]
    async fn test_completion_engine() {
        let engine = CompletionEngine::new();
        let app_state = AppState::default();

        let suggestions = engine.get_completions("p", 1, &app_state).await.unwrap();
        assert!(suggestions.iter().any(|s| s.text == "ps"));

        let suggestions = engine
            .get_completions("config ", 7, &app_state)
            .await
            .unwrap();
        assert!(suggestions.iter().any(|s| s.text == "get"));
        assert!(suggestions.iter().any(|s| s.text == "set"));
    }

    #[test]
    fn test_command_parsing() {
        let manager = CommandModeManager::new();

        if let Some(CommandModeAction::SwitchView(ViewType::Dashboard)) =
            manager.parse_tui_command("dashboard")
        {
            // Expected
        } else {
            panic!("Expected dashboard view switch");
        }

        assert!(manager.parse_tui_command("invalid_command").is_none());
    }

    #[tokio::test]
    async fn test_command_mode_key_handling() {
        use crossterm::event::{KeyCode, KeyEvent, KeyModifiers};

        let mut manager = CommandModeManager::new();
        let app_state = AppState::default();

        // Test activation
        manager.activate();
        assert!(manager.is_active());

        // Test character input
        let char_event = KeyEvent::new(KeyCode::Char('p'), KeyModifiers::empty());
        let action = manager
            .handle_key_event(char_event, &app_state)
            .await
            .unwrap();
        assert!(matches!(action, CommandModeAction::UpdateDisplay));

        // Test enter (should execute command)
        let enter_event = KeyEvent::new(KeyCode::Enter, KeyModifiers::empty());
        let action = manager
            .handle_key_event(enter_event, &app_state)
            .await
            .unwrap();
        assert!(matches!(action, CommandModeAction::ExecuteCommand { .. }));
        assert!(!manager.is_active()); // Should deactivate after execute

        // Test escape cancellation
        manager.activate();
        let esc_event = KeyEvent::new(KeyCode::Esc, KeyModifiers::empty());
        let action = manager
            .handle_key_event(esc_event, &app_state)
            .await
            .unwrap();
        assert!(matches!(action, CommandModeAction::Cancel));
        assert!(!manager.is_active()); // Should deactivate after cancel
    }

    #[test]
    fn test_command_history() {
        let mut manager = CommandModeManager::new();

        // Test adding commands to history
        manager.add_to_history("ps".to_string());
        manager.add_to_history("start example.yaml".to_string());
        manager.add_to_history("logs".to_string());

        let history = manager.get_command_history();
        assert_eq!(history.len(), 3);
        assert_eq!(history[0], "ps");
        assert_eq!(history[2], "logs");

        // Test duplicate prevention
        manager.add_to_history("logs".to_string());
        assert_eq!(manager.get_command_history().len(), 3); // Should still be 3
    }

    #[test]
    fn test_completion_context_analysis() {
        let engine = CompletionEngine::new();

        // Test command completion
        let context = engine.analyze_completion_context("p", 1);
        assert!(matches!(context.completion_type, CompletionType::Command));
        assert_eq!(context.prefix, "p");

        // Test flag completion
        let context = engine.analyze_completion_context("ps --", 5);
        assert!(matches!(context.completion_type, CompletionType::Flag));
        assert_eq!(context.prefix, "--");

        // Test value completion
        let context = engine.analyze_completion_context("start ", 6);
        assert!(matches!(context.completion_type, CompletionType::Value));
        assert_eq!(context.current_command, Some("start".to_string()));
    }

    #[test]
    fn test_apply_completion() {
        let manager = CommandModeManager::new();

        // Test basic completion
        let (new_buffer, new_cursor) = manager.apply_completion("p", 1, "ps");
        assert_eq!(new_buffer, "ps");
        assert_eq!(new_cursor, 2);

        // Test completion with existing text
        let (new_buffer, new_cursor) = manager.apply_completion("sta ", 4, "start");
        assert_eq!(new_buffer, "start ");
        assert_eq!(new_cursor, 5);

        // Test mid-word completion
        let (new_buffer, new_cursor) = manager.apply_completion("p test", 1, "ps");
        assert_eq!(new_buffer, "ps test");
        assert_eq!(new_cursor, 2);
    }
}
