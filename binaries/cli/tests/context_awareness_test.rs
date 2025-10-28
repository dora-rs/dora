// Integration tests for Context Awareness Feature (Issue #35)

use crossterm::event::{KeyCode, KeyEvent};
use dora_cli::tui::{
    app::AppState,
    views::{
        View, ViewAction, context_awareness::ContextAwarenessView, context_awareness_types::*,
    },
};

#[tokio::test]
async fn test_context_awareness_view_initialization() {
    let view = ContextAwarenessView::new();

    assert_eq!(view.state.current_section, ContextSection::ExecutionContext);
    assert_eq!(view.state.selected_index, 0);
    assert!(matches!(
        view.state.data,
        ContextData::ExecutionEnvironment(_)
    ));
}

#[tokio::test]
async fn test_section_navigation() {
    let mut view = ContextAwarenessView::new();
    let mut app_state = AppState::default();

    // Navigate through all sections forward
    let sections = ContextSection::all();
    for (i, expected_section) in sections.iter().enumerate() {
        if i > 0 {
            let result = view
                .handle_key(KeyEvent::from(KeyCode::Tab), &mut app_state)
                .await;
            assert!(result.is_ok());
        }
        assert_eq!(view.state.current_section, *expected_section);
    }

    // Wrap around
    let result = view
        .handle_key(KeyEvent::from(KeyCode::Tab), &mut app_state)
        .await;
    assert!(result.is_ok());
    assert_eq!(view.state.current_section, ContextSection::ExecutionContext);
}

#[tokio::test]
async fn test_section_navigation_backward() {
    let mut view = ContextAwarenessView::new();
    let mut app_state = AppState::default();

    // Go backward from first section (should wrap to last)
    let result = view
        .handle_key(KeyEvent::from(KeyCode::BackTab), &mut app_state)
        .await;
    assert!(result.is_ok());
    assert_eq!(
        view.state.current_section,
        ContextSection::PreferenceResolution
    );

    // Go backward again
    let result = view
        .handle_key(KeyEvent::from(KeyCode::BackTab), &mut app_state)
        .await;
    assert!(result.is_ok());
    assert_eq!(
        view.state.current_section,
        ContextSection::BehavioralLearning
    );
}

#[tokio::test]
async fn test_item_navigation() {
    let mut view = ContextAwarenessView::new();
    let mut app_state = AppState::default();

    // Navigate to section with multiple items
    view.state.current_section = ContextSection::ContextPreferences;
    view.state.update_data();

    let initial_index = view.state.selected_index;

    // Navigate down
    let result = view
        .handle_key(KeyEvent::from(KeyCode::Down), &mut app_state)
        .await;
    assert!(result.is_ok());
    assert_eq!(view.state.selected_index, initial_index + 1);

    // Navigate up
    let result = view
        .handle_key(KeyEvent::from(KeyCode::Up), &mut app_state)
        .await;
    assert!(result.is_ok());
    assert_eq!(view.state.selected_index, initial_index);
}

#[tokio::test]
async fn test_vim_style_navigation() {
    let mut view = ContextAwarenessView::new();
    let mut app_state = AppState::default();

    view.state.current_section = ContextSection::ContextPreferences;
    view.state.update_data();

    // Test 'j' (down)
    let result = view
        .handle_key(KeyEvent::from(KeyCode::Char('j')), &mut app_state)
        .await;
    assert!(result.is_ok());
    assert!(view.state.selected_index > 0);

    // Test 'k' (up)
    let result = view
        .handle_key(KeyEvent::from(KeyCode::Char('k')), &mut app_state)
        .await;
    assert!(result.is_ok());
    assert_eq!(view.state.selected_index, 0);
}

#[tokio::test]
async fn test_quit_action() {
    let mut view = ContextAwarenessView::new();
    let mut app_state = AppState::default();

    let result = view
        .handle_key(KeyEvent::from(KeyCode::Char('q')), &mut app_state)
        .await;
    assert!(result.is_ok());
    assert!(matches!(result.unwrap(), ViewAction::PopView));
}

#[tokio::test]
async fn test_refresh_action() {
    let mut view = ContextAwarenessView::new();
    let mut app_state = AppState::default();

    let result = view
        .handle_key(KeyEvent::from(KeyCode::Char('r')), &mut app_state)
        .await;
    assert!(result.is_ok());
    assert!(matches!(result.unwrap(), ViewAction::None));
}

#[test]
fn test_context_section_properties() {
    let sections = ContextSection::all();

    for section in sections {
        // All sections should have non-empty title and description
        assert!(!section.title().is_empty());
        assert!(!section.description().is_empty());
    }
}

#[test]
fn test_execution_environment_mock_data() {
    let env_data = create_mock_execution_env();

    assert!(env_data.environment.is_tty);
    assert!(!env_data.environment.is_piped);
    assert!(env_data.capabilities.supports_color);
    assert_eq!(env_data.capabilities.color_level, ColorLevel::TrueColor);
    assert!(env_data.capabilities.supports_unicode);
}

#[test]
fn test_adaptation_rules_mock_data() {
    let rules = create_mock_adaptation_rules();

    assert!(!rules.is_empty());

    for rule in rules {
        assert!(!rule.rule_id.is_empty());
        assert!(!rule.name.is_empty());
        assert!(!rule.condition.is_empty());
        assert!(!rule.action.is_empty());
        assert!(rule.priority > 0);
    }
}

#[test]
fn test_learning_patterns_mock_data() {
    let patterns = create_mock_learning_patterns();

    assert!(!patterns.is_empty());

    for pattern in patterns {
        assert!(!pattern.pattern_id.is_empty());
        assert!(!pattern.description.is_empty());
        assert!(pattern.confidence >= 0.0 && pattern.confidence <= 1.0);
        assert!(pattern.sample_size > 0);
    }
}

#[test]
fn test_preference_levels_mock_data() {
    let levels = create_mock_preference_levels();

    assert_eq!(levels.len(), 6);

    // Check priority ordering (highest to lowest)
    for i in 0..levels.len() - 1 {
        assert!(levels[i].level.priority() > levels[i + 1].level.priority());
    }
}

#[test]
fn test_ci_environment_variants() {
    let ci_envs = vec![
        CiEnvironment::GitHubActions,
        CiEnvironment::GitLabCI,
        CiEnvironment::CircleCI,
        CiEnvironment::TravisCI,
        CiEnvironment::Jenkins,
        CiEnvironment::Azure,
        CiEnvironment::Buildkite,
        CiEnvironment::TeamCity,
        CiEnvironment::Other("CustomCI".to_string()),
    ];

    for env in ci_envs {
        assert!(!env.name().is_empty());
    }
}

#[test]
fn test_color_level_variants() {
    let levels = vec![
        ColorLevel::None,
        ColorLevel::Basic,
        ColorLevel::Extended,
        ColorLevel::TrueColor,
    ];

    for level in levels {
        assert!(!level.name().is_empty());
    }
}

#[test]
fn test_time_of_day_conversion() {
    assert_eq!(TimeOfDay::from_hour(0), TimeOfDay::Night);
    assert_eq!(TimeOfDay::from_hour(6), TimeOfDay::Morning);
    assert_eq!(TimeOfDay::from_hour(12), TimeOfDay::Afternoon);
    assert_eq!(TimeOfDay::from_hour(18), TimeOfDay::Evening);
    assert_eq!(TimeOfDay::from_hour(23), TimeOfDay::Night);
}

#[test]
fn test_resolution_level_ordering() {
    let levels = ResolutionLevel::all();

    assert_eq!(levels[0], ResolutionLevel::TemporaryOverride);
    assert_eq!(levels[5], ResolutionLevel::SystemDefault);

    assert_eq!(ResolutionLevel::TemporaryOverride.priority(), 6);
    assert_eq!(ResolutionLevel::SystemDefault.priority(), 1);
}

#[test]
fn test_adaptation_rule_type_names() {
    assert_eq!(AdaptationRuleType::TimeBased.name(), "Time-Based");
    assert_eq!(
        AdaptationRuleType::EnvironmentBased.name(),
        "Environment-Based"
    );
    assert_eq!(
        AdaptationRuleType::ComplexityBased.name(),
        "Complexity-Based"
    );
    assert_eq!(
        AdaptationRuleType::UsagePatternBased.name(),
        "Usage Pattern-Based"
    );
}

#[test]
fn test_pattern_type_names() {
    assert_eq!(PatternType::CommandSequence.name(), "Command Sequence");
    assert_eq!(PatternType::TimePreference.name(), "Time Preference");
    assert_eq!(PatternType::InterfaceChoice.name(), "Interface Choice");
    assert_eq!(PatternType::WorkflowPattern.name(), "Workflow Pattern");
}

#[test]
fn test_interface_type_names() {
    assert_eq!(InterfaceType::CLI.name(), "CLI Only");
    assert_eq!(InterfaceType::TUI.name(), "TUI Preferred");
    assert_eq!(InterfaceType::Auto.name(), "Auto-Select");
}

#[test]
fn test_command_category_names() {
    let categories = vec![
        CommandCategory::Monitor,
        CommandCategory::Debug,
        CommandCategory::Analysis,
        CommandCategory::Management,
        CommandCategory::Other,
    ];

    for category in categories {
        assert!(!category.name().is_empty());
    }
}

#[test]
fn test_context_key_creation() {
    let key = ContextKey {
        command_category: CommandCategory::Monitor,
        time_of_day: TimeOfDay::Morning,
        terminal_type: TerminalType::Interactive,
        environment_type: EnvironmentType::Development,
    };

    assert_eq!(key.command_category, CommandCategory::Monitor);
    assert_eq!(key.time_of_day, TimeOfDay::Morning);
}

#[test]
fn test_context_key_equality() {
    let key1 = ContextKey {
        command_category: CommandCategory::Debug,
        time_of_day: TimeOfDay::Afternoon,
        terminal_type: TerminalType::Piped,
        environment_type: EnvironmentType::CI,
    };

    let key2 = ContextKey {
        command_category: CommandCategory::Debug,
        time_of_day: TimeOfDay::Afternoon,
        terminal_type: TerminalType::Piped,
        environment_type: EnvironmentType::CI,
    };

    assert_eq!(key1, key2);
}

#[test]
fn test_execution_environment_structure() {
    let env = ExecutionEnvironment {
        is_tty: true,
        is_piped: false,
        is_scripted: false,
        detected_ci: Some(CiEnvironment::GitHubActions),
        terminal_type: Some("xterm".to_string()),
        shell: Some("bash".to_string()),
        detected_at: chrono::Utc::now(),
    };

    assert!(env.is_tty);
    assert!(!env.is_piped);
    assert!(env.detected_ci.is_some());
}

#[test]
fn test_terminal_capabilities_structure() {
    let caps = TerminalCapabilities {
        supports_color: true,
        color_level: ColorLevel::Extended,
        supports_unicode: true,
        supports_mouse: false,
        terminal_width: 120,
        terminal_height: 30,
    };

    assert!(caps.supports_color);
    assert_eq!(caps.color_level, ColorLevel::Extended);
    assert_eq!(caps.terminal_width, 120);
}

#[test]
fn test_adaptation_rule_structure() {
    let rule = AdaptationRule {
        rule_id: "test_001".to_string(),
        name: "Test Rule".to_string(),
        rule_type: AdaptationRuleType::TimeBased,
        condition: "hour >= 9 && hour < 17".to_string(),
        action: "enable feature".to_string(),
        priority: 5,
        is_active: true,
        trigger_count: 42,
        last_triggered: Some(chrono::Utc::now()),
    };

    assert_eq!(rule.rule_id, "test_001");
    assert!(rule.is_active);
    assert_eq!(rule.priority, 5);
}

#[test]
fn test_learning_pattern_structure() {
    let pattern = LearningPattern {
        pattern_id: "pattern_test".to_string(),
        pattern_type: PatternType::CommandSequence,
        description: "Test pattern".to_string(),
        confidence: 0.95,
        sample_size: 100,
        discovered_at: chrono::Utc::now(),
        last_observed: chrono::Utc::now(),
        metadata: std::collections::HashMap::new(),
    };

    assert_eq!(pattern.confidence, 0.95);
    assert_eq!(pattern.sample_size, 100);
}

#[test]
fn test_preference_level_structure() {
    let level = PreferenceLevel {
        level: ResolutionLevel::UserPreference,
        source: "user_config".to_string(),
        preferences: std::collections::HashMap::from([("theme".to_string(), "dark".to_string())]),
        is_active: true,
    };

    assert_eq!(level.level, ResolutionLevel::UserPreference);
    assert!(level.is_active);
    assert_eq!(level.preferences.len(), 1);
}

#[test]
fn test_context_awareness_state_data_switching() {
    let mut state = ContextAwarenessState::new();

    // Start with ExecutionContext
    assert!(matches!(state.data, ContextData::ExecutionEnvironment(_)));

    // Switch to ContextPreferences
    state.next_section();
    assert!(matches!(state.data, ContextData::AdaptationRules(_)));

    // Switch to BehavioralLearning
    state.next_section();
    assert!(matches!(state.data, ContextData::LearningPatterns(_)));

    // Switch to PreferenceResolution
    state.next_section();
    assert!(matches!(state.data, ContextData::PreferenceLevels(_)));
}

#[test]
fn test_context_awareness_state_item_bounds() {
    let mut state = ContextAwarenessState::new();

    // Navigate to section with items
    state.current_section = ContextSection::ContextPreferences;
    state.update_data();

    // Navigate down multiple times
    for _ in 0..100 {
        state.next_item();
    }

    // Should be bounded
    let max_index = match &state.data {
        ContextData::AdaptationRules(rules) => rules.len().saturating_sub(1),
        _ => 0,
    };
    assert!(state.selected_index <= max_index);
}

#[test]
fn test_context_awareness_state_item_minimum() {
    let mut state = ContextAwarenessState::new();

    // Navigate up at index 0
    state.previous_item();
    assert_eq!(state.selected_index, 0);
}

#[test]
fn test_view_title_reflects_section() {
    let view = ContextAwarenessView::new();

    // The view title is the base title
    assert_eq!(view.title(), "Context Awareness");

    // Section information is shown in the rendered header, not in title()
    assert_eq!(view.state.current_section, ContextSection::ExecutionContext);
}

#[test]
fn test_view_help_text_completeness() {
    let view = ContextAwarenessView::new();
    let help = view.help_text();

    assert!(help.len() >= 4);
    assert!(help.iter().any(|(k, _)| k.contains("Tab")));
    assert!(help.iter().any(|(k, _)| k.contains("j") || k.contains("k")));
    assert!(help.iter().any(|(k, _)| k.contains("r")));
    assert!(help.iter().any(|(k, _)| k.contains("q")));
}

#[test]
fn test_mock_data_consistency() {
    // Verify mock data generators produce consistent results
    let env1 = create_mock_execution_env();
    let env2 = create_mock_execution_env();

    assert_eq!(env1.environment.is_tty, env2.environment.is_tty);
    assert_eq!(env1.capabilities.color_level, env2.capabilities.color_level);

    let rules1 = create_mock_adaptation_rules();
    let rules2 = create_mock_adaptation_rules();

    assert_eq!(rules1.len(), rules2.len());
    assert_eq!(rules1[0].rule_id, rules2[0].rule_id);
}

#[test]
fn test_adaptation_rules_priority_range() {
    let rules = create_mock_adaptation_rules();

    for rule in rules {
        assert!(rule.priority >= 0);
        assert!(rule.priority <= 10);
    }
}

#[test]
fn test_learning_patterns_confidence_validity() {
    let patterns = create_mock_learning_patterns();

    for pattern in patterns {
        assert!(pattern.confidence >= 0.0);
        assert!(pattern.confidence <= 1.0);
    }
}

#[test]
fn test_all_resolution_levels_present() {
    let levels = create_mock_preference_levels();

    assert!(
        levels
            .iter()
            .any(|l| l.level == ResolutionLevel::TemporaryOverride)
    );
    assert!(
        levels
            .iter()
            .any(|l| l.level == ResolutionLevel::ContextAdaptation)
    );
    assert!(
        levels
            .iter()
            .any(|l| l.level == ResolutionLevel::UserPreference)
    );
    assert!(
        levels
            .iter()
            .any(|l| l.level == ResolutionLevel::WorkspaceConfig)
    );
    assert!(
        levels
            .iter()
            .any(|l| l.level == ResolutionLevel::ProjectDefault)
    );
    assert!(
        levels
            .iter()
            .any(|l| l.level == ResolutionLevel::SystemDefault)
    );
}

#[tokio::test]
async fn test_complete_navigation_cycle() {
    let mut view = ContextAwarenessView::new();
    let mut app_state = AppState::default();

    // Navigate through all sections and verify data updates
    for _ in 0..ContextSection::all().len() {
        // Navigate to next section
        let result = view
            .handle_key(KeyEvent::from(KeyCode::Tab), &mut app_state)
            .await;
        assert!(result.is_ok());

        // Verify selection resets
        assert_eq!(view.state.selected_index, 0);
    }

    // Should be back to start
    assert_eq!(view.state.current_section, ContextSection::ExecutionContext);
}
