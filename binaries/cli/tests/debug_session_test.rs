/// Tests for Debug Session View (Issue #29)
#[cfg(test)]
mod debug_session_tests {
    use dora_cli::tui::theme::ThemeConfig;
    use dora_cli::tui::views::{
        Breakpoint, CallStackFrame, CodeLocation, DebugPanel, DebugSessionState, DebugSessionView,
        ExecutionState, Variable, View,
    };

    // ExecutionState tests
    #[test]
    fn test_execution_state_all() {
        let states = ExecutionState::all();
        assert_eq!(states.len(), 3);
        assert_eq!(states[0], ExecutionState::Running);
        assert_eq!(states[1], ExecutionState::Paused);
        assert_eq!(states[2], ExecutionState::Stopped);
    }

    #[test]
    fn test_execution_state_names() {
        assert_eq!(ExecutionState::Running.name(), "Running");
        assert_eq!(ExecutionState::Paused.name(), "Paused");
        assert_eq!(ExecutionState::Stopped.name(), "Stopped");
    }

    #[test]
    fn test_execution_state_symbols() {
        assert_eq!(ExecutionState::Running.symbol(), "▶");
        assert_eq!(ExecutionState::Paused.symbol(), "⏸");
        assert_eq!(ExecutionState::Stopped.symbol(), "⏹");
    }

    #[test]
    fn test_execution_state_color_codes() {
        assert_eq!(ExecutionState::Running.color_code(), "green");
        assert_eq!(ExecutionState::Paused.color_code(), "yellow");
        assert_eq!(ExecutionState::Stopped.color_code(), "red");
    }

    // DebugPanel tests
    #[test]
    fn test_debug_panel_all() {
        let panels = DebugPanel::all();
        assert_eq!(panels.len(), 3);
        assert_eq!(panels[0], DebugPanel::Variables);
        assert_eq!(panels[1], DebugPanel::CallStack);
        assert_eq!(panels[2], DebugPanel::Breakpoints);
    }

    #[test]
    fn test_debug_panel_names() {
        assert_eq!(DebugPanel::Variables.name(), "Variables");
        assert_eq!(DebugPanel::CallStack.name(), "Call Stack");
        assert_eq!(DebugPanel::Breakpoints.name(), "Breakpoints");
    }

    #[test]
    fn test_debug_panel_shortcuts() {
        assert_eq!(DebugPanel::Variables.shortcut(), "1");
        assert_eq!(DebugPanel::CallStack.shortcut(), "2");
        assert_eq!(DebugPanel::Breakpoints.shortcut(), "3");
    }

    #[test]
    fn test_debug_panel_navigation() {
        assert_eq!(DebugPanel::Variables.next(), DebugPanel::CallStack);
        assert_eq!(DebugPanel::CallStack.next(), DebugPanel::Breakpoints);
        assert_eq!(DebugPanel::Breakpoints.next(), DebugPanel::Variables);

        assert_eq!(DebugPanel::Variables.prev(), DebugPanel::Breakpoints);
        assert_eq!(DebugPanel::CallStack.prev(), DebugPanel::Variables);
        assert_eq!(DebugPanel::Breakpoints.prev(), DebugPanel::CallStack);
    }

    // Variable tests
    #[test]
    fn test_variable_creation() {
        let var = Variable::new(
            "test_var".to_string(),
            "42".to_string(),
            "i32".to_string(),
            "local".to_string(),
        );

        assert_eq!(var.name, "test_var");
        assert_eq!(var.value, "42");
        assert_eq!(var.var_type, "i32");
        assert_eq!(var.scope, "local");
    }

    #[test]
    fn test_variable_format_display() {
        let var = Variable::new(
            "counter".to_string(),
            "100".to_string(),
            "u64".to_string(),
            "global".to_string(),
        );

        let display = var.format_display();
        assert!(display.contains("counter"));
        assert!(display.contains("100"));
        assert!(display.contains("u64"));
    }

    // CallStackFrame tests
    #[test]
    fn test_call_stack_frame_creation() {
        let frame = CallStackFrame::new("main".to_string(), "main.rs".to_string(), 42);

        assert_eq!(frame.function_name, "main");
        assert_eq!(frame.file_name, "main.rs");
        assert_eq!(frame.line_number, 42);
    }

    #[test]
    fn test_call_stack_frame_format_display() {
        let frame = CallStackFrame::new("process_frame".to_string(), "node.rs".to_string(), 123);

        let display = frame.format_display();
        assert!(display.contains("process_frame"));
        assert!(display.contains("node.rs"));
        assert!(display.contains("123"));
    }

    // Breakpoint tests
    #[test]
    fn test_breakpoint_creation() {
        let bp = Breakpoint::new(1, "main.rs".to_string(), 42);

        assert_eq!(bp.id, 1);
        assert_eq!(bp.file_name, "main.rs");
        assert_eq!(bp.line_number, 42);
        assert!(bp.enabled);
        assert_eq!(bp.hit_count, 0);
    }

    #[test]
    fn test_breakpoint_toggle_enabled() {
        let mut bp = Breakpoint::new(1, "test.rs".to_string(), 10);

        assert!(bp.enabled);
        bp.toggle_enabled();
        assert!(!bp.enabled);
        bp.toggle_enabled();
        assert!(bp.enabled);
    }

    #[test]
    fn test_breakpoint_format_display() {
        let mut bp = Breakpoint::new(1, "node.rs".to_string(), 100);
        bp.hit_count = 5;

        let display = bp.format_display();
        assert!(display.contains("node.rs"));
        assert!(display.contains("100"));
        assert!(display.contains("5"));
    }

    // CodeLocation tests
    #[test]
    fn test_code_location_creation() {
        let loc = CodeLocation::new("main.rs".to_string(), 42, "main".to_string());

        assert_eq!(loc.file_name, "main.rs");
        assert_eq!(loc.line_number, 42);
        assert_eq!(loc.function_name, "main");
    }

    #[test]
    fn test_code_location_format_display() {
        let loc = CodeLocation::new("camera.rs".to_string(), 123, "capture_frame".to_string());

        let display = loc.format_display();
        assert!(display.contains("camera.rs"));
        assert!(display.contains("123"));
        assert!(display.contains("capture_frame"));
    }

    // DebugSessionState tests
    #[test]
    fn test_debug_session_state_new() {
        let state = DebugSessionState::new("test-target".to_string());

        assert_eq!(state.target_name, "test-target");
        assert_eq!(state.execution_state, ExecutionState::Stopped);
        assert_eq!(state.active_panel, DebugPanel::Variables);
        assert_eq!(state.variables.len(), 0);
        assert_eq!(state.call_stack.len(), 0);
        assert_eq!(state.breakpoints.len(), 0);
    }

    #[test]
    fn test_debug_session_state_add_variable() {
        let mut state = DebugSessionState::new("test".to_string());
        let var = Variable::new(
            "x".to_string(),
            "10".to_string(),
            "i32".to_string(),
            "local".to_string(),
        );

        state.add_variable(var);
        assert_eq!(state.variables.len(), 1);
        assert_eq!(state.variables[0].name, "x");
    }

    #[test]
    fn test_debug_session_state_add_call_frame() {
        let mut state = DebugSessionState::new("test".to_string());
        let frame = CallStackFrame::new("main".to_string(), "main.rs".to_string(), 10);

        state.add_call_frame(frame);
        assert_eq!(state.call_stack.len(), 1);
        assert_eq!(state.call_stack[0].function_name, "main");
    }

    #[test]
    fn test_debug_session_state_add_breakpoint() {
        let mut state = DebugSessionState::new("test".to_string());
        let bp = Breakpoint::new(1, "test.rs".to_string(), 42);

        state.add_breakpoint(bp);
        assert_eq!(state.breakpoints.len(), 1);
        assert_eq!(state.breakpoints[0].id, 1);
    }

    #[test]
    fn test_debug_session_state_remove_breakpoint() {
        let mut state = DebugSessionState::new("test".to_string());
        state.add_breakpoint(Breakpoint::new(1, "a.rs".to_string(), 10));
        state.add_breakpoint(Breakpoint::new(2, "b.rs".to_string(), 20));

        assert_eq!(state.breakpoints.len(), 2);
        state.remove_breakpoint(1);
        assert_eq!(state.breakpoints.len(), 1);
        assert_eq!(state.breakpoints[0].id, 2);
    }

    #[test]
    fn test_debug_session_state_toggle_breakpoint() {
        let mut state = DebugSessionState::new("test".to_string());
        state.add_breakpoint(Breakpoint::new(1, "test.rs".to_string(), 10));

        assert!(state.breakpoints[0].enabled);
        state.toggle_breakpoint(1);
        assert!(!state.breakpoints[0].enabled);
        state.toggle_breakpoint(1);
        assert!(state.breakpoints[0].enabled);
    }

    #[test]
    fn test_debug_session_state_panel_switching() {
        let mut state = DebugSessionState::new("test".to_string());

        assert_eq!(state.active_panel, DebugPanel::Variables);
        state.next_panel();
        assert_eq!(state.active_panel, DebugPanel::CallStack);
        state.next_panel();
        assert_eq!(state.active_panel, DebugPanel::Breakpoints);
        state.next_panel();
        assert_eq!(state.active_panel, DebugPanel::Variables);
    }

    #[test]
    fn test_debug_session_state_prev_panel() {
        let mut state = DebugSessionState::new("test".to_string());

        assert_eq!(state.active_panel, DebugPanel::Variables);
        state.prev_panel();
        assert_eq!(state.active_panel, DebugPanel::Breakpoints);
        state.prev_panel();
        assert_eq!(state.active_panel, DebugPanel::CallStack);
        state.prev_panel();
        assert_eq!(state.active_panel, DebugPanel::Variables);
    }

    #[test]
    fn test_debug_session_state_switch_panel_direct() {
        let mut state = DebugSessionState::new("test".to_string());

        state.switch_panel(DebugPanel::Breakpoints);
        assert_eq!(state.active_panel, DebugPanel::Breakpoints);

        state.switch_panel(DebugPanel::CallStack);
        assert_eq!(state.active_panel, DebugPanel::CallStack);
    }

    #[test]
    fn test_debug_session_state_execution_state_change() {
        let mut state = DebugSessionState::new("test".to_string());

        assert_eq!(state.execution_state, ExecutionState::Stopped);
        state.set_execution_state(ExecutionState::Running);
        assert_eq!(state.execution_state, ExecutionState::Running);
        state.set_execution_state(ExecutionState::Paused);
        assert_eq!(state.execution_state, ExecutionState::Paused);
    }

    #[test]
    fn test_debug_session_state_navigation_up_down() {
        let mut state = DebugSessionState::new("test".to_string());

        // Add variables
        for i in 0..5 {
            state.add_variable(Variable::new(
                format!("var{}", i),
                format!("{}", i),
                "i32".to_string(),
                "local".to_string(),
            ));
        }

        state.selected_variable = 2;
        state.navigate_up();
        assert_eq!(state.selected_variable, 1);
        state.navigate_up();
        assert_eq!(state.selected_variable, 0);
        state.navigate_up(); // Should stay at 0
        assert_eq!(state.selected_variable, 0);

        state.navigate_down();
        assert_eq!(state.selected_variable, 1);
        state.navigate_down();
        assert_eq!(state.selected_variable, 2);
    }

    #[test]
    fn test_debug_session_state_navigation_different_panels() {
        let mut state = DebugSessionState::new("test".to_string());

        // Add data to different panels
        for i in 0..3 {
            state.add_variable(Variable::new(
                format!("v{}", i),
                "0".to_string(),
                "i32".to_string(),
                "local".to_string(),
            ));
            state.add_call_frame(CallStackFrame::new(
                format!("func{}", i),
                "test.rs".to_string(),
                i,
            ));
            state.add_breakpoint(Breakpoint::new(i, "test.rs".to_string(), i * 10));
        }

        // Test Variables panel
        state.switch_panel(DebugPanel::Variables);
        state.selected_variable = 0;
        state.navigate_down();
        assert_eq!(state.selected_variable, 1);

        // Test CallStack panel
        state.switch_panel(DebugPanel::CallStack);
        state.selected_frame = 0;
        state.navigate_down();
        assert_eq!(state.selected_frame, 1);

        // Test Breakpoints panel
        state.switch_panel(DebugPanel::Breakpoints);
        state.selected_breakpoint = 0;
        state.navigate_down();
        assert_eq!(state.selected_breakpoint, 1);
    }

    #[test]
    fn test_debug_session_state_get_selected_item() {
        let mut state = DebugSessionState::new("test".to_string());

        // Add items
        state.add_variable(Variable::new(
            "v1".to_string(),
            "1".to_string(),
            "i32".to_string(),
            "local".to_string(),
        ));
        state.add_variable(Variable::new(
            "v2".to_string(),
            "2".to_string(),
            "i32".to_string(),
            "local".to_string(),
        ));

        state.switch_panel(DebugPanel::Variables);
        state.selected_variable = 1;
        assert!(state.get_selected_item().is_some());

        state.switch_panel(DebugPanel::CallStack);
        state.selected_frame = 0;
        assert!(state.get_selected_item().is_none()); // No call stack items added
    }

    #[test]
    fn test_debug_session_state_current_panel_count() {
        let mut state = DebugSessionState::new("test".to_string());

        // Add data
        for i in 0..3 {
            state.add_variable(Variable::new(
                format!("v{}", i),
                "0".to_string(),
                "i32".to_string(),
                "local".to_string(),
            ));
        }
        for i in 0..5 {
            state.add_call_frame(CallStackFrame::new(
                format!("f{}", i),
                "test.rs".to_string(),
                i,
            ));
        }
        for i in 0..2 {
            state.add_breakpoint(Breakpoint::new(i, "test.rs".to_string(), i * 10));
        }

        state.switch_panel(DebugPanel::Variables);
        assert_eq!(state.current_panel_count(), 3);

        state.switch_panel(DebugPanel::CallStack);
        assert_eq!(state.current_panel_count(), 5);

        state.switch_panel(DebugPanel::Breakpoints);
        assert_eq!(state.current_panel_count(), 2);
    }

    #[test]
    fn test_debug_session_state_clear() {
        let mut state = DebugSessionState::new("test".to_string());

        // Add data
        state.add_variable(Variable::new(
            "v".to_string(),
            "0".to_string(),
            "i32".to_string(),
            "local".to_string(),
        ));
        state.add_call_frame(CallStackFrame::new(
            "f".to_string(),
            "test.rs".to_string(),
            1,
        ));
        state.add_breakpoint(Breakpoint::new(1, "test.rs".to_string(), 10));
        state.selected_variable = 1;
        state.selected_frame = 1;

        state.clear();

        assert_eq!(state.variables.len(), 0);
        assert_eq!(state.call_stack.len(), 0);
        assert_eq!(state.breakpoints.len(), 0);
        assert_eq!(state.selected_variable, 0);
        assert_eq!(state.selected_frame, 0);
        assert_eq!(state.selected_breakpoint, 0);
    }

    #[test]
    fn test_debug_session_state_mark_refreshed() {
        let mut state = DebugSessionState::new("test".to_string());
        let before = state.last_refresh;

        std::thread::sleep(std::time::Duration::from_millis(10));
        state.mark_refreshed();

        assert!(state.last_refresh > before);
    }

    // DebugSessionView tests
    #[test]
    fn test_debug_session_view_creation() {
        let theme = ThemeConfig::default();
        let view = DebugSessionView::new("test-target", &theme);

        assert_eq!(view.title(), "Debug Session");
        assert_eq!(view.state.target_name, "test-target");
    }

    #[test]
    fn test_debug_session_view_mock_data_populated() {
        let theme = ThemeConfig::default();
        let view = DebugSessionView::new("test", &theme);

        assert!(view.state.variables.len() > 0);
        assert!(view.state.call_stack.len() > 0);
        assert!(view.state.breakpoints.len() > 0);
    }

    #[test]
    fn test_debug_session_view_initial_state() {
        let theme = ThemeConfig::default();
        let view = DebugSessionView::new("test", &theme);

        assert_eq!(view.state.execution_state, ExecutionState::Paused);
        assert_eq!(view.state.active_panel, DebugPanel::Variables);
    }

    #[test]
    fn test_debug_session_view_auto_refresh() {
        let theme = ThemeConfig::default();
        let view = DebugSessionView::new("test", &theme);

        assert!(view.auto_refresh().is_some());
        assert_eq!(
            view.auto_refresh().unwrap(),
            std::time::Duration::from_millis(500)
        );
    }

    #[test]
    fn test_debug_session_view_help_text() {
        let theme = ThemeConfig::default();
        let view = DebugSessionView::new("test", &theme);

        let help = view.help_text();
        assert!(help.len() >= 8);
        assert!(help.iter().any(|(key, _)| *key == "F5"));
        assert!(help.iter().any(|(key, _)| *key == "F9"));
        assert!(help.iter().any(|(key, _)| *key == "F10"));
        assert!(help.iter().any(|(key, _)| *key == "F11"));
        assert!(help.iter().any(|(key, _)| *key == "Tab"));
    }

    #[test]
    fn test_complex_debugging_scenario() {
        let mut state = DebugSessionState::new("complex-node".to_string());

        // Simulate a debugging session
        // Start with stopped state
        state.set_execution_state(ExecutionState::Stopped);

        // Add breakpoints
        state.add_breakpoint(Breakpoint::new(1, "main.rs".to_string(), 10));
        state.add_breakpoint(Breakpoint::new(2, "handler.rs".to_string(), 25));

        // Start execution (changes to running)
        state.set_execution_state(ExecutionState::Running);
        assert_eq!(state.execution_state, ExecutionState::Running);

        // Hit breakpoint (pause)
        state.set_execution_state(ExecutionState::Paused);

        // Add variables visible at breakpoint
        state.add_variable(Variable::new(
            "input".to_string(),
            "42".to_string(),
            "i32".to_string(),
            "local".to_string(),
        ));
        state.add_variable(Variable::new(
            "result".to_string(),
            "84".to_string(),
            "i32".to_string(),
            "local".to_string(),
        ));

        // Add call stack
        state.add_call_frame(CallStackFrame::new(
            "handler".to_string(),
            "handler.rs".to_string(),
            25,
        ));
        state.add_call_frame(CallStackFrame::new(
            "process".to_string(),
            "main.rs".to_string(),
            50,
        ));
        state.add_call_frame(CallStackFrame::new(
            "main".to_string(),
            "main.rs".to_string(),
            10,
        ));

        // Verify state
        assert_eq!(state.execution_state, ExecutionState::Paused);
        assert_eq!(state.variables.len(), 2);
        assert_eq!(state.call_stack.len(), 3);
        assert_eq!(state.breakpoints.len(), 2);

        // Navigate through panels
        state.switch_panel(DebugPanel::CallStack);
        state.navigate_down();
        assert_eq!(state.selected_frame, 1);

        // Disable a breakpoint
        state.switch_panel(DebugPanel::Breakpoints);
        state.toggle_breakpoint(2);
        assert!(!state.breakpoints[1].enabled);

        // Continue execution
        state.set_execution_state(ExecutionState::Running);
        assert_eq!(state.execution_state, ExecutionState::Running);
    }
}
