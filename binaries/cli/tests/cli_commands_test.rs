/// Integration tests for CLI command parsing and execution
/// Verifies that all hybrid CLI commands can be parsed correctly

use dora_cli::cli::{Cli, Command, TuiView};
use clap::Parser;

#[test]
fn test_basic_commands_parse() {
    // Test Tier 1 commands parse correctly
    let commands = vec![
        vec!["dora", "ps"],
        vec!["dora", "start"],
        vec!["dora", "stop"],
        vec!["dora", "logs"],
        vec!["dora", "build"],
    ];

    for cmd in commands {
        let result = Cli::try_parse_from(cmd.clone());
        assert!(
            result.is_ok(),
            "Failed to parse command: {:?}",
            cmd
        );
    }
}

#[test]
fn test_smart_commands_parse() {
    // Test Tier 2 smart commands parse correctly
    let commands = vec![
        vec!["dora", "inspect"],
        vec!["dora", "debug"],
        vec!["dora", "analyze"],
        vec!["dora", "monitor"],
        vec!["dora", "help"],
    ];

    for cmd in commands {
        let result = Cli::try_parse_from(cmd.clone());
        assert!(
            result.is_ok(),
            "Failed to parse smart command: {:?}",
            cmd
        );
    }
}

#[test]
fn test_tui_commands_parse() {
    // Test Tier 3 TUI commands parse correctly
    let commands = vec![
        vec!["dora", "ui"],
        vec!["dora", "dashboard"],
    ];

    for cmd in commands {
        let result = Cli::try_parse_from(cmd.clone());
        assert!(
            result.is_ok(),
            "Failed to parse TUI command: {:?}",
            cmd
        );
    }
}

#[test]
fn test_ui_command_with_view_flag() {
    // Test ui command with different view options
    let test_cases = vec![
        (vec!["dora", "ui", "--view", "dashboard"], TuiView::Dashboard),
        (vec!["dora", "ui", "--view", "dataflow"], TuiView::Dataflow),
        (vec!["dora", "ui", "--view", "performance"], TuiView::Performance),
        (vec!["dora", "ui", "--view", "logs"], TuiView::Logs),
    ];

    for (args, expected_view) in test_cases {
        let cli = Cli::try_parse_from(args.clone())
            .expect(&format!("Failed to parse: {:?}", args));

        match cli.command {
            Some(Command::Ui(cmd)) => {
                assert_eq!(
                    cmd.view,
                    Some(expected_view),
                    "View mismatch for args: {:?}",
                    args
                );
            }
            _ => panic!("Expected Ui command, got: {:?}", cli.command),
        }
    }
}

#[test]
fn test_global_flags() {
    // Test that global flags work with commands
    let cli = Cli::try_parse_from(vec!["dora", "--verbose", "ps"])
        .expect("Failed to parse with verbose flag");

    assert!(cli.verbose, "Verbose flag should be set");

    let cli = Cli::try_parse_from(vec!["dora", "--quiet", "ps"])
        .expect("Failed to parse with quiet flag");

    assert!(cli.quiet, "Quiet flag should be set");
}

#[test]
fn test_ui_mode_flag() {
    // Test --ui-mode global flag
    let test_cases = vec![
        vec!["dora", "--ui-mode", "auto", "ps"],
        vec!["dora", "--ui-mode", "cli", "ps"],
        vec!["dora", "--ui-mode", "tui", "ps"],
        vec!["dora", "--ui-mode", "minimal", "ps"],
    ];

    for args in test_cases {
        let result = Cli::try_parse_from(args.clone());
        assert!(
            result.is_ok(),
            "Failed to parse ui-mode: {:?}",
            args
        );
    }
}

#[test]
fn test_output_format_flag() {
    // Test --output global flag
    let test_cases = vec![
        vec!["dora", "--output", "auto", "ps"],
        vec!["dora", "--output", "table", "ps"],
        vec!["dora", "--output", "json", "ps"],
        vec!["dora", "--output", "yaml", "ps"],
        vec!["dora", "--output", "minimal", "ps"],
    ];

    for args in test_cases {
        let result = Cli::try_parse_from(args.clone());
        assert!(
            result.is_ok(),
            "Failed to parse output format: {:?}",
            args
        );
    }
}

#[test]
fn test_system_commands_parse() {
    // Test system management commands (all require subcommands)
    let commands = vec![
        vec!["dora", "system", "status"],
        vec!["dora", "config", "list"],
        vec!["dora", "daemon", "status"],
        vec!["dora", "runtime", "status"],
        vec!["dora", "coordinator", "status"],
    ];

    for cmd in commands {
        let result = Cli::try_parse_from(cmd.clone());
        assert!(
            result.is_ok(),
            "Failed to parse system command: {:?}",
            cmd
        );
    }
}

#[test]
fn test_command_has_value() {
    // Test that parsed commands actually contain a command
    let cli = Cli::try_parse_from(vec!["dora", "ps"])
        .expect("Failed to parse ps command");

    assert!(
        cli.command.is_some(),
        "Command should be present after parsing"
    );

    match cli.command {
        Some(Command::Ps(_)) => {}, // Expected
        _ => panic!("Expected Ps command"),
    }
}

#[test]
fn test_multiple_flags_together() {
    // Test multiple global flags work together
    let cli = Cli::try_parse_from(vec![
        "dora",
        "--verbose",
        "--no-hints",
        "--ui-mode", "cli",
        "--output", "json",
        "ps"
    ]).expect("Failed to parse with multiple flags");

    assert!(cli.verbose, "Verbose should be set");
    assert!(cli.no_hints, "No hints should be set");
    assert!(cli.ui_mode.is_some(), "UI mode should be set");
}

#[test]
fn test_invalid_command_fails() {
    // Test that invalid commands are rejected
    let result = Cli::try_parse_from(vec!["dora", "invalid-command-xyz"]);
    assert!(result.is_err(), "Invalid command should fail to parse");
}

#[test]
fn test_help_flag_is_handled() {
    // Test that --help is recognized (even though it causes exit)
    let result = Cli::try_parse_from(vec!["dora", "--help"]);

    // --help causes clap to return an error with help text
    assert!(result.is_err(), "Help flag should trigger help display");

    // Verify it's a DisplayHelp error
    if let Err(e) = result {
        use clap::error::ErrorKind;
        assert_eq!(e.kind(), ErrorKind::DisplayHelp);
    }
}

#[test]
fn test_version_flag_is_handled() {
    // Test that --version is recognized
    let result = Cli::try_parse_from(vec!["dora", "--version"]);

    // --version causes clap to return an error with version text
    assert!(result.is_err(), "Version flag should trigger version display");

    // Verify it's a DisplayVersion error
    if let Err(e) = result {
        use clap::error::ErrorKind;
        assert_eq!(e.kind(), ErrorKind::DisplayVersion);
    }
}

#[test]
fn test_all_tier_commands_represented() {
    // Ensure we have commands from all three tiers

    // Tier 1: Basic commands
    let tier1 = Cli::try_parse_from(vec!["dora", "ps"]).unwrap();
    assert!(tier1.command.is_some());

    // Tier 2: Smart commands
    let tier2 = Cli::try_parse_from(vec!["dora", "inspect"]).unwrap();
    assert!(tier2.command.is_some());

    // Tier 3: TUI commands
    let tier3 = Cli::try_parse_from(vec!["dora", "ui"]).unwrap();
    assert!(tier3.command.is_some());
}
