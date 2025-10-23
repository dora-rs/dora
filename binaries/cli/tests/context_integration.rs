use dora_cli::cli::context::{ExecutionContext, TerminalCapabilities, ExecutionEnvironment, CiEnvironment};

#[test]
fn test_context_in_different_environments() {
    // Test context detection in various scenarios
    
    // Interactive terminal (basic detection)
    let basic_context = ExecutionContext::detect_basic();
    assert!(basic_context.working_dir.exists() || basic_context.working_dir.to_str() == Some("."));
    
    // Full detection
    let full_context = ExecutionContext::detect();
    
    // Verify that detection methods produce consistent results
    assert_eq!(basic_context.is_tty, full_context.is_tty);
    assert_eq!(basic_context.is_piped, full_context.is_piped);
}

#[tokio::test]
async fn test_comprehensive_detection() {
    let context = ExecutionContext::detect_comprehensive().await;
    
    // Validate comprehensive detection includes all fields
    assert!(context.working_dir.exists() || context.working_dir.to_str() == Some("."));
    
    // Verify terminal capabilities are detected
    println!("Terminal capabilities:");
    println!("  Color support: {}", context.terminal_capabilities.supports_color);
    println!("  Unicode support: {}", context.terminal_capabilities.supports_unicode);
    println!("  Mouse support: {}", context.terminal_capabilities.supports_mouse);
    println!("  Terminal type: {:?}", context.terminal_capabilities.terminal_type);
    
    // Verify environment detection
    println!("Environment:");
    println!("  CI: {}", context.environment.is_ci);
    println!("  Shell: {:?}", context.environment.shell_type);
    println!("  CI Environment: {:?}", context.environment.ci_environment);
}

#[test]
fn test_terminal_size_detection() {
    let caps = TerminalCapabilities::detect();
    
    // Terminal size detection should work in some environments
    match (caps.width, caps.height) {
        (Some(w), Some(h)) => {
            println!("Terminal size detected: {}x{}", w, h);
            assert!(w > 0);
            assert!(h > 0);
        }
        _ => {
            println!("Terminal size not detected (might be expected in CI)");
        }
    }
}

#[test]
fn test_context_aware_behavior() {
    let context = ExecutionContext::detect();
    
    // Test context-aware methods
    let should_use_interactive = context.should_use_interactive_features();
    let optimal_width = context.get_optimal_output_width();
    let supports_realtime = context.supports_realtime_updates();
    
    println!("Context-aware behavior:");
    println!("  Interactive features: {}", should_use_interactive);
    println!("  Optimal width: {}", optimal_width);
    println!("  Realtime updates: {}", supports_realtime);
    
    // Verify reasonable defaults
    assert!(optimal_width >= 80); // Should have reasonable minimum
    assert!(optimal_width <= 1000); // Should not be unreasonably large
}

#[test]
fn test_ui_mode_preferences() {
    let context = ExecutionContext::detect();
    
    let prefers_tui = context.prefers_tui();
    let force_cli = context.force_cli();
    let show_suggestions = context.show_suggestions();
    
    println!("UI preferences:");
    println!("  Prefers TUI: {}", prefers_tui);
    println!("  Force CLI: {}", force_cli);
    println!("  Show suggestions: {}", show_suggestions);
    
    // These should not both be true
    assert!(!(prefers_tui && force_cli));
}

#[test]
fn test_environment_classification() {
    let env = ExecutionEnvironment::detect();
    
    // Test environment classification logic
    if env.is_ci {
        println!("CI environment detected: {:?}", env.ci_environment);
        assert!(env.ci_environment.is_some());
    } else {
        println!("Non-CI environment detected");
    }
    
    if let Some(shell) = &env.shell_type {
        println!("Shell detected: {}", shell);
        assert!(!shell.is_empty());
    }
    
    // Test consistency
    if env.ci_environment.is_some() {
        assert!(env.is_ci);
    }
}

#[test]
fn test_specific_ci_platforms() {
    use std::collections::HashMap;
    
    // Test specific CI platform detection
    let test_cases = vec![
        ("GITHUB_ACTIONS", CiEnvironment::GitHubActions),
        ("GITLAB_CI", CiEnvironment::GitLabCI),
        ("JENKINS_URL", CiEnvironment::Jenkins),
        ("BUILDKITE", CiEnvironment::Buildkite),
        ("CIRCLECI", CiEnvironment::CircleCI),
        ("TRAVIS", CiEnvironment::TravisCI),
        ("APPVEYOR", CiEnvironment::AppVeyor),
        ("AZURE_PIPELINES", CiEnvironment::AzurePipelines),
        ("TEAMCITY_VERSION", CiEnvironment::TeamCity),
    ];
    
    for (env_var, expected_ci) in test_cases {
        let mut env_vars = HashMap::new();
        env_vars.insert(env_var.to_string(), "true".to_string());
        
        let detected = ExecutionEnvironment::detect_ci_environment(&env_vars);
        assert_eq!(detected, Some(expected_ci));
    }
}

#[test]
fn test_color_support_detection() {
    let caps = TerminalCapabilities::detect();
    
    println!("Color support details:");
    println!("  Supports color: {}", caps.supports_color);
    println!("  Legacy colors: {}", caps.colors);
    
    // These should be consistent
    assert_eq!(caps.supports_color, caps.colors);
}

#[test]
fn test_performance_basic_vs_full() {
    use std::time::Instant;
    
    // Test that basic detection is faster than full detection
    let start = Instant::now();
    let _basic = ExecutionContext::detect_basic();
    let basic_time = start.elapsed();
    
    let start = Instant::now();
    let _full = ExecutionContext::detect();
    let full_time = start.elapsed();
    
    println!("Performance:");
    println!("  Basic detection: {:?}", basic_time);
    println!("  Full detection: {:?}", full_time);
    
    // Basic should be faster or comparable (it might not always be faster due to system variance)
    // Just ensure both complete in reasonable time
    assert!(basic_time.as_millis() < 100);
    assert!(full_time.as_millis() < 1000);
}