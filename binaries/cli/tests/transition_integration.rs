use std::{
    collections::HashMap,
    time::{Duration, Instant},
};

use dora_cli::{
    cli::{
        Command, Cli,
        commands::*,
        context::ExecutionContext,
        transitions::{TransitionManager, TransitionTrigger, TuiLaunchContext, CliResumeContext, ExitReason},
        state_sync::{StateSynchronizer, StateUpdate, ConflictResolution},
        transition_triggers::{TransitionTriggerAnalyzer, InteractionEvent, InteractionOutcome},
    },
    tui::{ViewType, AppState, TuiCliExecutor, CommandResult},
};

#[tokio::test]
async fn test_full_cli_to_tui_transition_cycle() {
    // Start with CLI context
    let mut transition_manager = TransitionManager::new();
    
    // Create a command that should trigger TUI transition
    let inspect_cmd = Command::Inspect(InspectCommand {
        resource: Some("test_node".to_string()),
        deep: true,
        ..Default::default()
    });
    
    let trigger = TransitionTrigger::SmartSuggestion {
        original_command: inspect_cmd.clone(),
        reason: "Deep inspection benefits from interactive visualization".to_string(),
    };
    
    // Transition to TUI
    let launch_context = transition_manager
        .transition_to_tui(trigger.clone(), ViewType::NodeInspector { 
            node_id: "test_node".to_string() 
        })
        .await
        .expect("Failed to transition to TUI");
    
    // Verify launch context
    assert_eq!(launch_context.initial_view, ViewType::NodeInspector { 
        node_id: "test_node".to_string() 
    });
    assert!(matches!(launch_context.transition_trigger, TransitionTrigger::SmartSuggestion { .. }));
    
    // Simulate TUI session
    let tui_state = crate::cli::transitions::TuiState {
        current_view: ViewType::NodeInspector { node_id: "test_node".to_string() },
        view_stack: vec![ViewType::Dashboard],
        view_states: HashMap::new(),
        last_refresh: Instant::now(),
    };
    
    // Transition back to CLI
    let resume_context = transition_manager
        .transition_from_tui(tui_state, ExitReason::UserExit)
        .await
        .expect("Failed to transition from TUI");
    
    // Verify context preservation
    assert!(resume_context.preserve_context);
    assert!(resume_context.exit_message.is_some());
    
    // Check transition history
    let history = transition_manager.get_transition_history();
    assert_eq!(history.len(), 2); // CLI->TUI and TUI->CLI
    
    // Verify performance
    let performance = transition_manager.get_transition_performance();
    assert!(performance.avg_cli_to_tui.is_some());
    assert!(performance.avg_tui_to_cli.is_some());
    assert_eq!(performance.total_transitions, 2);
}

#[tokio::test]
async fn test_command_execution_within_tui() {
    let context = ExecutionContext::default();
    let mut executor = TuiCliExecutor::new(context);
    let mut app_state = AppState::default();
    
    // Test various command types
    
    // 1. Test view navigation command
    let result = executor
        .execute_command("ps", &mut app_state)
        .await
        .expect("Failed to execute ps command");
    
    match result {
        CommandResult::ViewSwitch(ViewType::Dashboard) => {},
        _ => panic!("Expected view switch to dashboard for ps command"),
    }
    
    // 2. Test state-changing command
    let result = executor
        .execute_command("start test_dataflow.yml", &mut app_state)
        .await
        .expect("Failed to execute start command");
    
    match result {
        CommandResult::StateUpdate(_) => {},
        CommandResult::Error(_) => {}, // Acceptable since we don't have real dataflow
        _ => panic!("Expected state update or error for start command"),
    }
    
    // 3. Test configuration command
    let result = executor
        .execute_command("config set theme dark", &mut app_state)
        .await
        .expect("Failed to execute config command");
    
    match result {
        CommandResult::StateUpdate(StateUpdate::ConfigurationChanged) => {},
        _ => panic!("Expected configuration change for config set command"),
    }
    
    // 4. Test inspect command
    let result = executor
        .execute_command("inspect test_node", &mut app_state)
        .await
        .expect("Failed to execute inspect command");
    
    match result {
        CommandResult::ViewSwitch(ViewType::NodeInspector { node_id }) => {
            assert_eq!(node_id, "test_node");
        },
        _ => panic!("Expected view switch to node inspector"),
    }
    
    // Verify command history
    let history = executor.get_command_history();
    assert_eq!(history.len(), 4);
    assert_eq!(history[0], "ps");
    assert_eq!(history[1], "start test_dataflow.yml");
    assert_eq!(history[2], "config set theme dark");
    assert_eq!(history[3], "inspect test_node");
}

#[tokio::test]
async fn test_state_synchronization() {
    let mut synchronizer = StateSynchronizer::new();
    let mut app_state = AppState::default();
    
    // Test CLI to TUI sync
    synchronizer
        .sync_cli_to_tui(&mut app_state)
        .await
        .expect("Failed to sync CLI to TUI");
    
    // Verify dataflows were populated
    assert!(!app_state.dataflows.is_empty());
    
    // Test adding pending update
    synchronizer.add_pending_update(StateUpdate::DataflowAdded(
        crate::tui::app::DataflowInfo {
            id: "test_df".to_string(),
            name: "test_dataflow".to_string(),
            status: "running".to_string(),
            nodes: vec![],
        }
    ));
    
    assert_eq!(synchronizer.get_pending_updates().len(), 1);
    
    // Test conflict resolution
    synchronizer.set_conflict_resolution(ConflictResolution::TuiWins);
    
    // Clear updates
    synchronizer.clear_pending_updates();
    assert!(synchronizer.get_pending_updates().is_empty());
}

#[tokio::test]
async fn test_transition_trigger_analysis() {
    let mut analyzer = TransitionTriggerAnalyzer::new();
    let context = ExecutionContext {
        is_tty: true,
        is_scripted: false,
        ..Default::default()
    };
    
    // Test debug command analysis (should trigger auto-launch)
    let debug_cmd = Command::Debug(DebugCommand::default());
    let decision = crate::cli::interface::InterfaceDecision {
        strategy: crate::cli::interface::InterfaceStrategy::AutoLaunchTui {
            reason: "Debug session complexity".to_string(),
            show_cli_first: false,
        },
        confidence: 0.9,
        reason: "Debug commands benefit from TUI".to_string(),
        fallback: None,
    };
    
    let trigger = analyzer.analyze_command(&debug_cmd, &context, &decision);
    assert!(trigger.is_some());
    
    match trigger.unwrap() {
        TransitionTrigger::AutoLaunch { complexity_score, .. } => {
            assert!(complexity_score >= 5); // Debug should be high complexity
        },
        _ => panic!("Expected auto-launch trigger for debug command"),
    }
    
    // Test logs command with follow (should suggest TUI)
    let logs_cmd = Command::Logs(LogsCommand {
        follow: true,
        ..Default::default()
    });
    
    let decision = crate::cli::interface::InterfaceDecision {
        strategy: crate::cli::interface::InterfaceStrategy::CliWithHint {
            hint: "Live logs benefit from interactive filtering".to_string(),
            tui_command: "ui logs".to_string(),
        },
        confidence: 0.7,
        reason: "Follow logs suggest TUI".to_string(),
        fallback: None,
    };
    
    let trigger = analyzer.analyze_command(&logs_cmd, &context, &decision);
    assert!(trigger.is_some());
    
    // Test smart launch context creation
    let smart_context = analyzer.create_smart_launch_context(
        TransitionTrigger::SmartSuggestion {
            original_command: logs_cmd.clone(),
            reason: "Live monitoring".to_string(),
        },
        &logs_cmd,
        &context,
    );
    
    assert!(smart_context.smart_features.auto_refresh);
    assert!(smart_context.smart_features.intelligent_filtering);
    
    // Test interaction recording
    analyzer.record_interaction(InteractionEvent {
        command: "logs --follow".to_string(),
        timestamp: Instant::now(),
        context: "terminal".to_string(),
        outcome: InteractionOutcome::AcceptedTui,
    });
    
    analyzer.update_preferences();
}

#[tokio::test]
async fn test_transition_performance_tracking() {
    let mut manager = TransitionManager::new();
    
    // Simulate multiple transitions
    for i in 0..5 {
        let trigger = TransitionTrigger::ExplicitCommand(Command::Ui(UiCommand::default()));
        let target_view = ViewType::Dashboard;
        
        let _context = manager.transition_to_tui(trigger, target_view.clone()).await.unwrap();
        
        // Simulate TUI session
        let tui_state = crate::cli::transitions::TuiState {
            current_view: target_view,
            view_stack: Vec::new(),
            view_states: HashMap::new(),
            last_refresh: Instant::now(),
        };
        
        // Add some artificial delay to test performance tracking
        tokio::time::sleep(Duration::from_millis(10 * i)).await;
        
        let _resume = manager.transition_from_tui(tui_state, ExitReason::UserExit).await.unwrap();
    }
    
    // Check performance metrics
    let performance = manager.get_transition_performance();
    assert_eq!(performance.total_transitions, 10); // 5 CLI->TUI + 5 TUI->CLI
    assert!(performance.avg_cli_to_tui.is_some());
    assert!(performance.avg_tui_to_cli.is_some());
    
    // Verify all transitions are recorded
    let history = manager.get_transition_history();
    assert_eq!(history.len(), 10);
    
    // Verify transition durations are recorded
    for transition in history.iter() {
        assert!(transition.duration.is_some());
    }
}

#[tokio::test]
async fn test_view_switch_recording() {
    let mut manager = TransitionManager::new();
    
    // Record some view switches
    manager.record_view_switch(ViewType::Dashboard, ViewType::NodeInspector { 
        node_id: "test".to_string() 
    });
    
    manager.record_view_switch(
        ViewType::NodeInspector { node_id: "test".to_string() },
        ViewType::LogViewer { target: "test".to_string() }
    );
    
    let history = manager.get_transition_history();
    assert_eq!(history.len(), 2);
    
    // Verify view switch transitions
    for transition in history.iter() {
        match &transition.transition_type {
            crate::cli::transitions::TransitionType::TuiViewSwitch { .. } => {},
            _ => panic!("Expected TuiViewSwitch transition type"),
        }
        
        // View switches should be instant
        assert_eq!(transition.duration, Some(Duration::from_millis(0)));
    }
}

#[tokio::test]
async fn test_error_handling_in_transitions() {
    let mut manager = TransitionManager::new();
    
    // Test transition with error recovery trigger
    let trigger = TransitionTrigger::ErrorRecovery {
        failed_command: Command::Start(StartCommand::default()),
        error: "Failed to start dataflow".to_string(),
    };
    
    let result = manager.transition_to_tui(trigger, ViewType::Dashboard).await;
    assert!(result.is_ok());
    
    // Test command execution with invalid command
    let context = ExecutionContext::default();
    let mut executor = TuiCliExecutor::new(context);
    let mut app_state = AppState::default();
    
    let result = executor.execute_command("invalid_command_that_does_not_exist", &mut app_state).await;
    
    // Should handle parsing errors gracefully
    match result {
        Ok(CommandResult::Error(_)) => {},
        Err(_) => {}, // Also acceptable
        _ => panic!("Expected error for invalid command"),
    }
}

#[tokio::test]
async fn test_context_preservation_across_transitions() {
    let mut manager = TransitionManager::new();
    
    // Set up initial CLI state
    manager.cli_state.working_directory = std::path::PathBuf::from("/test/directory");
    manager.cli_state.environment.insert("TEST_VAR".to_string(), "test_value".to_string());
    manager.cli_state.command_history.push("previous_command".to_string());
    
    // Transition to TUI
    let trigger = TransitionTrigger::ExplicitCommand(Command::Ui(UiCommand::default()));
    let launch_context = manager.transition_to_tui(trigger, ViewType::Dashboard).await.unwrap();
    
    // Verify context preservation
    assert_eq!(launch_context.cli_context.working_directory, std::path::PathBuf::from("/test/directory"));
    assert_eq!(launch_context.cli_context.environment.get("TEST_VAR"), Some(&"test_value".to_string()));
    assert_eq!(launch_context.cli_context.command_history.len(), 1);
    
    // Simulate TUI state with modifications
    let tui_state = crate::cli::transitions::TuiState {
        current_view: ViewType::Dashboard,
        view_stack: vec![ViewType::NodeInspector { node_id: "test".to_string() }],
        view_states: HashMap::new(),
        last_refresh: Instant::now(),
    };
    
    // Transition back to CLI
    let resume_context = manager.transition_from_tui(tui_state, ExitReason::UserExit).await.unwrap();
    
    // Verify context is preserved
    assert_eq!(resume_context.working_directory, std::path::PathBuf::from("/test/directory"));
    assert_eq!(resume_context.environment.get("TEST_VAR"), Some(&"test_value".to_string()));
    assert!(resume_context.preserve_context);
}

// Helper function to create a mock execution context
impl Default for ExecutionContext {
    fn default() -> Self {
        Self {
            is_tty: true,
            is_piped: false,
            is_scripted: false,
            terminal_size: Some((80, 24)),
            environment: crate::cli::context::Environment {
                ci_environment: None,
                shell_type: Some(crate::cli::context::ShellType::Bash),
                term_program: Some("test".to_string()),
            },
            terminal_capabilities: crate::cli::context::TerminalCapabilities {
                supports_color: true,
                supports_unicode: true,
                tui_capable: true,
            },
        }
    }
}