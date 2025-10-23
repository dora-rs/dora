use std::{
    collections::HashMap,
    time::{Duration, Instant},
};

use super::*;
use crate::cli::{Command, commands::UiCommand};
use crate::tui::ViewType;

#[tokio::test]
async fn test_transition_manager_creation() {
    let manager = TransitionManager::new();
    
    assert!(manager.transition_history.is_empty());
    assert!(manager.tui_state.is_none());
    assert_eq!(manager.cli_state.command_history.len(), 0);
}

#[tokio::test]
async fn test_cli_to_tui_transition() {
    let mut manager = TransitionManager::new();
    let trigger = TransitionTrigger::ExplicitCommand(Command::Ui(UiCommand::default()));
    
    let context = manager.transition_to_tui(trigger.clone(), ViewType::Dashboard).await.unwrap();
    
    // Verify launch context
    assert_eq!(context.initial_view, ViewType::Dashboard);
    assert!(matches!(context.transition_trigger, TransitionTrigger::ExplicitCommand(_)));
    assert!(!context.launch_options.show_transition_message); // Explicit commands shouldn't show message
    
    // Verify transition was recorded
    assert_eq!(manager.transition_history.len(), 1);
    
    let transition = &manager.transition_history[0];
    assert!(matches!(transition.transition_type, TransitionType::CliToTui { .. }));
    assert!(transition.duration.is_some());
}

#[tokio::test]
async fn test_tui_to_cli_transition() {
    let mut manager = TransitionManager::new();
    
    // First transition to TUI
    let trigger = TransitionTrigger::ExplicitCommand(Command::Ui(UiCommand::default()));
    let _context = manager.transition_to_tui(trigger, ViewType::Dashboard).await.unwrap();
    
    // Create TUI state
    let tui_state = TuiState {
        current_view: ViewType::Dashboard,
        view_stack: vec![ViewType::NodeInspector { node_id: "test".to_string() }],
        view_states: HashMap::new(),
        last_refresh: Instant::now(),
    };
    
    // Transition back to CLI
    let resume_context = manager.transition_from_tui(tui_state.clone(), ExitReason::UserExit).await.unwrap();
    
    // Verify resume context
    assert!(resume_context.preserve_context);
    assert!(resume_context.exit_message.is_some());
    assert_eq!(resume_context.exit_message.unwrap(), "Exited TUI");
    
    // Verify TUI state is cleared
    assert!(manager.tui_state.is_none());
    
    // Verify transition was recorded
    assert_eq!(manager.transition_history.len(), 2); // CLI->TUI and TUI->CLI
    
    let tui_to_cli_transition = &manager.transition_history[1];
    assert!(matches!(tui_to_cli_transition.transition_type, TransitionType::TuiToCli { .. }));
}

#[test]
fn test_view_switch_recording() {
    let mut manager = TransitionManager::new();
    
    manager.record_view_switch(ViewType::Dashboard, ViewType::NodeInspector { 
        node_id: "test".to_string() 
    });
    
    assert_eq!(manager.transition_history.len(), 1);
    
    let transition = &manager.transition_history[0];
    match &transition.transition_type {
        TransitionType::TuiViewSwitch { from_view, to_view } => {
            assert_eq!(*from_view, ViewType::Dashboard);
            assert_eq!(*to_view, ViewType::NodeInspector { node_id: "test".to_string() });
        },
        _ => panic!("Expected TuiViewSwitch"),
    }
    
    // View switches should be instant
    assert_eq!(transition.duration, Some(Duration::from_millis(0)));
}

#[tokio::test]
async fn test_transition_performance_tracking() {
    let mut manager = TransitionManager::new();
    
    // Add some transitions with varying durations
    for i in 0..3 {
        let trigger = TransitionTrigger::ExplicitCommand(Command::Ui(UiCommand::default()));
        let _context = manager.transition_to_tui(trigger, ViewType::Dashboard).await.unwrap();
        
        // Manually set duration for testing
        if let Some(last_transition) = manager.transition_history.back_mut() {
            last_transition.duration = Some(Duration::from_millis(100 + i * 50));
        }
        
        let tui_state = TuiState {
            current_view: ViewType::Dashboard,
            view_stack: Vec::new(),
            view_states: HashMap::new(),
            last_refresh: Instant::now(),
        };
        
        let _resume = manager.transition_from_tui(tui_state, ExitReason::UserExit).await.unwrap();
        
        // Manually set duration for testing
        if let Some(last_transition) = manager.transition_history.back_mut() {
            last_transition.duration = Some(Duration::from_millis(50 + i * 25));
        }
    }
    
    let performance = manager.get_transition_performance();
    
    assert_eq!(performance.total_transitions, 6); // 3 CLI->TUI + 3 TUI->CLI
    assert!(performance.avg_cli_to_tui.is_some());
    assert!(performance.avg_tui_to_cli.is_some());
    
    // Check that averages are reasonable
    let avg_cli_to_tui = performance.avg_cli_to_tui.unwrap();
    let avg_tui_to_cli = performance.avg_tui_to_cli.unwrap();
    
    assert!(avg_cli_to_tui.as_millis() > 100);
    assert!(avg_cli_to_tui.as_millis() < 200);
    assert!(avg_tui_to_cli.as_millis() > 50);
    assert!(avg_tui_to_cli.as_millis() < 100);
}

#[test]
fn test_transition_trigger_command_extraction() {
    let cmd = Command::Ui(UiCommand::default());
    
    let explicit_trigger = TransitionTrigger::ExplicitCommand(cmd.clone());
    assert!(explicit_trigger.command().is_some());
    
    let suggestion_trigger = TransitionTrigger::SmartSuggestion {
        original_command: cmd.clone(),
        reason: "test".to_string(),
    };
    assert!(suggestion_trigger.command().is_some());
    
    let user_action_trigger = TransitionTrigger::UserAction("test".to_string());
    assert!(user_action_trigger.command().is_none());
}

#[test]
fn test_cli_state_creation() {
    let cli_state = CliState::new();
    
    assert!(cli_state.current_command.is_none());
    assert!(cli_state.command_history.is_empty());
    assert!(cli_state.last_output.is_none());
    assert!(!cli_state.environment.is_empty()); // Should have environment variables
    assert!(cli_state.working_directory.exists() || cli_state.working_directory == std::path::PathBuf::from("/"));
}

#[test]
fn test_shared_context_creation() {
    let shared_context = SharedContext::new();
    
    assert!(shared_context.dataflows.is_empty());
    assert!(shared_context.active_filters.is_empty());
    assert_eq!(shared_context.system_metrics.cpu_usage, 0.0);
    assert_eq!(shared_context.system_metrics.memory_usage, 0.0);
}

#[test]
fn test_tui_launch_options_from_trigger() {
    // Explicit command should not show transition message
    let explicit_trigger = TransitionTrigger::ExplicitCommand(Command::Ui(UiCommand::default()));
    let options = TuiLaunchOptions::from_trigger(&explicit_trigger);
    assert!(!options.show_transition_message);
    assert!(!options.preserve_cli_output);
    
    // Smart suggestion should show message and preserve output
    let suggestion_trigger = TransitionTrigger::SmartSuggestion {
        original_command: Command::Ui(UiCommand::default()),
        reason: "test".to_string(),
    };
    let options = TuiLaunchOptions::from_trigger(&suggestion_trigger);
    assert!(options.show_transition_message);
    assert!(options.preserve_cli_output);
    
    // Auto launch should show message
    let auto_trigger = TransitionTrigger::AutoLaunch {
        original_command: Command::Ui(UiCommand::default()),
        complexity_score: 8,
    };
    let options = TuiLaunchOptions::from_trigger(&auto_trigger);
    assert!(options.show_transition_message);
    assert!(!options.preserve_cli_output);
}

#[tokio::test]
async fn test_context_snapshot_creation() {
    let cli_state = CliState::new();
    let cli_snapshot = ContextSnapshot::Cli(cli_state.clone());
    
    match cli_snapshot {
        ContextSnapshot::Cli(state) => {
            assert_eq!(state.working_directory, cli_state.working_directory);
        },
        _ => panic!("Expected CLI snapshot"),
    }
    
    let tui_state = TuiState {
        current_view: ViewType::Dashboard,
        view_stack: Vec::new(),
        view_states: HashMap::new(),
        last_refresh: Instant::now(),
    };
    let tui_snapshot = ContextSnapshot::Tui(tui_state.clone());
    
    match tui_snapshot {
        ContextSnapshot::Tui(state) => {
            assert_eq!(state.current_view, tui_state.current_view);
        },
        _ => panic!("Expected TUI snapshot"),
    }
}

#[test]
fn test_exit_reason_variants() {
    let user_exit = ExitReason::UserExit;
    let command_exit = ExitReason::CommandExit("test command".to_string());
    let error_exit = ExitReason::ErrorExit("test error".to_string());
    let switch_exit = ExitReason::SwitchToCommand(Command::Ui(UiCommand::default()));
    
    // Test that all variants can be created and cloned
    let _user_exit_clone = user_exit.clone();
    let _command_exit_clone = command_exit.clone();
    let _error_exit_clone = error_exit.clone();
    let _switch_exit_clone = switch_exit.clone();
}

#[test]
fn test_command_output_creation() {
    let output = CommandOutput {
        success: true,
        stdout: "test output".to_string(),
        stderr: "".to_string(),
        duration: Duration::from_millis(100),
    };
    
    assert!(output.success);
    assert_eq!(output.stdout, "test output");
    assert_eq!(output.duration, Duration::from_millis(100));
}

#[test]
fn test_view_state_creation() {
    let view_state = ViewState {
        last_update: Instant::now(),
        scroll_position: 10,
        selection: Some("test_item".to_string()),
        filters: HashMap::from([("test_filter".to_string(), "test_value".to_string())]),
    };
    
    assert_eq!(view_state.scroll_position, 10);
    assert_eq!(view_state.selection, Some("test_item".to_string()));
    assert_eq!(view_state.filters.get("test_filter"), Some(&"test_value".to_string()));
}

#[test]
fn test_transition_history_bounds() {
    let mut manager = TransitionManager::new();
    
    // Add many transitions to test bounds
    for i in 0..150 {
        let transition = Transition {
            transition_type: TransitionType::CliToTui {
                triggering_command: Command::Ui(UiCommand::default()),
                target_view: ViewType::Dashboard,
            },
            timestamp: Instant::now(),
            from_context: ContextSnapshot::Cli(CliState::new()),
            to_context: ContextSnapshot::Tui(TuiState {
                current_view: ViewType::Dashboard,
                view_stack: Vec::new(),
                view_states: HashMap::new(),
                last_refresh: Instant::now(),
            }),
            trigger: TransitionTrigger::ExplicitCommand(Command::Ui(UiCommand::default())),
            duration: Some(Duration::from_millis(i)),
        };
        
        manager.record_transition(transition);
    }
    
    // Should be bounded to MAX_HISTORY (100)
    assert_eq!(manager.transition_history.len(), 100);
    
    // Should contain the most recent transitions
    let last_transition = manager.transition_history.back().unwrap();
    assert_eq!(last_transition.duration, Some(Duration::from_millis(149)));
}

// Helper function for average duration calculation
#[test]
fn test_average_duration_calculation() {
    use super::average_duration;
    
    // Test empty slice
    assert!(average_duration(&[]).is_none());
    
    // Test single duration
    let single = vec![Duration::from_millis(100)];
    assert_eq!(average_duration(&single), Some(Duration::from_millis(100)));
    
    // Test multiple durations
    let multiple = vec![
        Duration::from_millis(100),
        Duration::from_millis(200),
        Duration::from_millis(300),
    ];
    assert_eq!(average_duration(&multiple), Some(Duration::from_millis(200)));
}