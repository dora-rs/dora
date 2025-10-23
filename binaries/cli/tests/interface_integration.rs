use dora_cli::cli::{
    Command, UiMode,
    interface::{InterfaceSelector, UserConfig, InterfaceStrategy},
    context::ExecutionContext,
    commands::*,
};

#[test]
fn test_end_to_end_interface_selection() {
    // Test simple command in interactive context
    let context = mock_interactive_context();
    let config = UserConfig::default();
    let mut selector = InterfaceSelector::new(context, config);
    
    let command = Command::Ps(PsCommand::default());
    let decision = selector.select_interface(&command);
    
    // Simple ps command should stay CLI
    assert!(matches!(decision.strategy, InterfaceStrategy::CliOnly));
    assert!(decision.confidence > 0.5);
}

#[test]
fn test_complex_command_suggestions() {
    let context = mock_interactive_context();
    let config = UserConfig::default();
    let mut selector = InterfaceSelector::new(context, config);
    
    let command = Command::Debug(DebugCommand::default());
    let decision = selector.select_interface(&command);
    
    // Complex debug command should suggest or auto-launch TUI
    match decision.strategy {
        InterfaceStrategy::PromptForTui { .. } |
        InterfaceStrategy::AutoLaunchTui { .. } |
        InterfaceStrategy::CliWithHint { .. } => {
            // Any of these are acceptable for a complex command
        },
        InterfaceStrategy::CliOnly => {
            // Should not be CLI only for debug in interactive context
            panic!("Debug command should suggest TUI in interactive context");
        }
    }
}

#[test]
fn test_non_interactive_context_forces_cli() {
    let context = mock_non_interactive_context();
    let config = UserConfig::default();
    let mut selector = InterfaceSelector::new(context, config);
    
    // Even complex commands should be CLI only in non-interactive contexts
    let command = Command::Debug(DebugCommand::default());
    let decision = selector.select_interface(&command);
    
    assert!(matches!(decision.strategy, InterfaceStrategy::CliOnly));
    assert!(decision.reason.contains("Non-interactive environment detected"));
}

#[test]
fn test_user_preference_override() {
    let context = mock_interactive_context();
    let mut config = UserConfig::default();
    config.global_ui_mode = UiMode::Cli;
    let mut selector = InterfaceSelector::new(context, config);
    
    // Even complex commands should respect user preference
    let command = Command::Debug(DebugCommand::default());
    let decision = selector.select_interface(&command);
    
    assert!(matches!(decision.strategy, InterfaceStrategy::CliOnly));
    assert!(decision.reason.contains("User global preference"));
}

#[test]
fn test_command_complexity_analysis() {
    let context = mock_interactive_context();
    let config = UserConfig::default();
    let mut selector = InterfaceSelector::new(context, config);
    
    // Test various command complexities
    let commands = vec![
        (Command::Ps(PsCommand::default()), "ps should be simple"),
        (Command::Logs(LogsCommand::default()), "logs should be moderate"),
        (Command::Debug(DebugCommand::default()), "debug should be complex"),
    ];
    
    for (command, description) in commands {
        let decision = selector.select_interface(&command);
        println!("{}: {:?} (confidence: {:.2})", 
                description, decision.strategy, decision.confidence);
        
        // All decisions should have reasonable confidence
        assert!(decision.confidence >= 0.5, 
               "Low confidence for {}: {:.2}", description, decision.confidence);
    }
}

#[test]
fn test_cache_consistency() {
    let context = mock_interactive_context();
    let config = UserConfig::default();
    let mut selector = InterfaceSelector::new(context, config);
    
    let command = Command::Ps(PsCommand::default());
    
    // Multiple calls should return identical results
    let decisions: Vec<_> = (0..5)
        .map(|_| selector.select_interface(&command))
        .collect();
    
    // All decisions should be identical
    for (i, decision) in decisions.iter().enumerate() {
        if i > 0 {
            assert_eq!(decision.strategy, decisions[0].strategy);
            assert_eq!(decision.confidence, decisions[0].confidence);
            assert_eq!(decision.reason, decisions[0].reason);
        }
    }
}

#[test]
fn test_different_ui_modes() {
    let context = mock_interactive_context();
    
    for ui_mode in [UiMode::Auto, UiMode::Cli, UiMode::Tui, UiMode::Minimal] {
        let mut config = UserConfig::default();
        config.global_ui_mode = ui_mode;
        let mut selector = InterfaceSelector::new(context.clone(), config);
        
        let command = Command::Ps(PsCommand::default());
        let decision = selector.select_interface(&command);
        
        match ui_mode {
            UiMode::Cli | UiMode::Minimal => {
                assert!(matches!(decision.strategy, InterfaceStrategy::CliOnly));
            },
            UiMode::Tui => {
                assert!(matches!(decision.strategy, InterfaceStrategy::AutoLaunchTui { .. }));
            },
            UiMode::Auto => {
                // Auto mode should make context-appropriate decisions
                // For simple ps command, likely CLI
                // We don't assert specific strategy as it depends on algorithm
            },
        }
    }
}

#[test]
fn test_hint_generation() {
    let context = mock_interactive_context();
    let config = UserConfig::default();
    let mut selector = InterfaceSelector::new(context, config);
    
    let command = Command::Inspect(InspectCommand::default());
    let decision = selector.select_interface(&command);
    
    if let InterfaceStrategy::CliWithHint { hint, tui_command } = decision.strategy {
        assert!(!hint.is_empty());
        assert!(!tui_command.is_empty());
        assert!(tui_command.contains("dora"));
    }
}

// Helper functions
fn mock_interactive_context() -> ExecutionContext {
    let mut context = ExecutionContext::detect_basic();
    context.is_tty = true;
    context.is_piped = false;
    context.is_scripted = false;
    context.terminal_capabilities.tui_capable = true;
    context
}

fn mock_non_interactive_context() -> ExecutionContext {
    let mut context = ExecutionContext::detect_basic();
    context.is_tty = false;
    context.is_piped = true;
    context.is_scripted = true;
    context.terminal_capabilities.tui_capable = false;
    context
}