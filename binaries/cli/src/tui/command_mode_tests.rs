mod tests {
    use crate::tui::ViewType;
    use crate::tui::app::AppState;
    use crate::tui::command_mode::{
        CommandModeAction, CommandModeManager, CommandSuggestion, CompletionEngine,
        CompletionState, HistoryNavigation, SuggestionType,
    };
    use crossterm::event::{KeyCode, KeyEvent, KeyModifiers};

    #[tokio::test]
    async fn test_command_mode_activation() {
        let mut manager = CommandModeManager::new();
        assert!(!manager.is_active());

        manager.activate();
        assert!(manager.is_active());

        // Typing characters should keep the manager active
        let app_state = AppState::default();
        let action = manager
            .handle_key_event(
                KeyEvent::new(KeyCode::Char('p'), KeyModifiers::empty()),
                &app_state,
            )
            .await
            .unwrap();
        assert!(matches!(action, CommandModeAction::UpdateDisplay));

        manager.deactivate();
        assert!(!manager.is_active());
    }

    #[test]
    fn test_completion_state_selection() {
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
    fn test_history_navigation_roundtrip() {
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
    async fn test_completion_engine_suggestions() {
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
        assert!(suggestions.iter().any(|s| s.text == "list"));
    }

    #[tokio::test]
    async fn test_command_mode_executes_and_records_history() {
        let mut manager = CommandModeManager::new();
        let app_state = AppState::default();

        manager.activate();
        type_text(&mut manager, "ps", &app_state).await;

        let action = press_key(&mut manager, KeyCode::Enter, &app_state).await;
        let history: Vec<_> = manager.get_command_history().iter().cloned().collect();

        match action {
            CommandModeAction::ExecuteCommand {
                command,
                show_output,
            } => {
                assert_eq!(command, "ps");
                assert!(show_output);
            }
            other => panic!("expected ExecuteCommand, got {:?}", other),
        };
        assert_eq!(history, vec!["ps"]);
        assert!(manager.get_last_execution_time().is_some());
    }

    #[tokio::test]
    async fn test_command_mode_switch_view_shortcuts() {
        let mut manager = CommandModeManager::new();
        let app_state = AppState::default();

        manager.activate();
        type_text(&mut manager, "dashboard", &app_state).await;
        let action = press_key(&mut manager, KeyCode::Enter, &app_state).await;

        match action {
            CommandModeAction::SwitchView(ViewType::Dashboard) => {}
            other => panic!("expected Dashboard switch, got {:?}", other),
        }
        assert!(!manager.is_active());
    }

    #[tokio::test]
    async fn test_tab_completion_applies_suggestion() {
        let mut manager = CommandModeManager::new();
        let app_state = AppState::default();

        manager.activate();
        type_text(&mut manager, "p", &app_state).await;

        let tab_action = press_key(&mut manager, KeyCode::Tab, &app_state).await;
        assert!(matches!(tab_action, CommandModeAction::UpdateDisplay));

        let execute = press_key(&mut manager, KeyCode::Enter, &app_state).await;
        match execute {
            CommandModeAction::ExecuteCommand { command, .. } => assert_eq!(command, "ps"),
            other => panic!("expected ps execution, got {:?}", other),
        }

        let history: Vec<_> = manager.get_command_history().iter().cloned().collect();
        assert_eq!(history, vec!["ps"]);
    }

    #[tokio::test]
    async fn test_escape_cancels_without_recording_history() {
        let mut manager = CommandModeManager::new();
        let app_state = AppState::default();

        manager.activate();
        type_text(&mut manager, "logs", &app_state).await;
        let action = press_key(&mut manager, KeyCode::Esc, &app_state).await;

        assert!(matches!(action, CommandModeAction::Cancel));
        assert!(manager.get_command_history().is_empty());
    }

    #[tokio::test]
    async fn test_consecutive_duplicates_are_ignored() {
        let mut manager = CommandModeManager::new();
        let app_state = AppState::default();

        manager.activate();
        type_text(&mut manager, "logs", &app_state).await;
        let first = press_key(&mut manager, KeyCode::Enter, &app_state).await;
        assert!(matches!(
            first,
            CommandModeAction::SwitchView(ViewType::LogViewer { .. })
        ));

        manager.activate();
        type_text(&mut manager, "logs", &app_state).await;
        let second = press_key(&mut manager, KeyCode::Enter, &app_state).await;
        assert!(matches!(
            second,
            CommandModeAction::SwitchView(ViewType::LogViewer { .. })
        ));

        let history: Vec<_> = manager.get_command_history().iter().cloned().collect();
        assert_eq!(history, vec!["logs"]);
    }

    async fn type_text(manager: &mut CommandModeManager, text: &str, app_state: &AppState) {
        for ch in text.chars() {
            let action = manager
                .handle_key_event(
                    KeyEvent::new(KeyCode::Char(ch), KeyModifiers::empty()),
                    app_state,
                )
                .await
                .unwrap();
            assert!(matches!(action, CommandModeAction::UpdateDisplay));
        }
    }

    async fn press_key(
        manager: &mut CommandModeManager,
        code: KeyCode,
        app_state: &AppState,
    ) -> CommandModeAction {
        manager
            .handle_key_event(KeyEvent::new(code, KeyModifiers::empty()), app_state)
            .await
            .unwrap()
    }
}
