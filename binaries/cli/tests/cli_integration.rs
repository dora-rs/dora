use clap::Parser;
use dora_cli::cli::{Cli, Command, UiMode, OutputFormat};

#[test]
fn test_cli_parsing() {
    // Test basic command parsing
    let cli = Cli::try_parse_from(["dora", "ps"]).unwrap();
    assert!(matches!(cli.command, Command::Ps(_)));
}

#[test]
fn test_global_flags() {
    // Test UI mode flag
    let cli = Cli::try_parse_from(["dora", "--ui-mode", "cli", "ps"]).unwrap();
    assert_eq!(cli.ui_mode, Some(UiMode::Cli));
    
    // Test output format flag
    let cli = Cli::try_parse_from(["dora", "--output", "json", "ps"]).unwrap();
    assert_eq!(cli.output, OutputFormat::Json);
    
    // Test no-hints flag
    let cli = Cli::try_parse_from(["dora", "--no-hints", "ps"]).unwrap();
    assert!(cli.no_hints);
    
    // Test verbose flag
    let cli = Cli::try_parse_from(["dora", "--verbose", "ps"]).unwrap();
    assert!(cli.verbose);
    
    // Test quiet flag
    let cli = Cli::try_parse_from(["dora", "--quiet", "ps"]).unwrap();
    assert!(cli.quiet);
}

#[test]
fn test_backward_compatibility() {
    // Test aliases work
    let cli = Cli::try_parse_from(["dora", "list"]).unwrap();
    assert!(matches!(cli.command, Command::Ps(_)));
}

#[test]
fn test_three_tier_structure() {
    // Tier 1: Core commands
    let cli = Cli::try_parse_from(["dora", "ps"]).unwrap();
    assert!(matches!(cli.command, Command::Ps(_)));
    
    let cli = Cli::try_parse_from(["dora", "start", "dataflow.yml"]).unwrap();
    assert!(matches!(cli.command, Command::Start(_)));
    
    let cli = Cli::try_parse_from(["dora", "build"]).unwrap();
    assert!(matches!(cli.command, Command::Build(_)));
    
    // Tier 2: Enhanced commands
    let cli = Cli::try_parse_from(["dora", "inspect", "node1"]).unwrap();
    assert!(matches!(cli.command, Command::Inspect(_)));
    
    let cli = Cli::try_parse_from(["dora", "debug"]).unwrap();
    assert!(matches!(cli.command, Command::Debug(_)));
    
    let cli = Cli::try_parse_from(["dora", "analyze"]).unwrap();
    assert!(matches!(cli.command, Command::Analyze(_)));
    
    // Tier 3: TUI commands
    let cli = Cli::try_parse_from(["dora", "ui"]).unwrap();
    assert!(matches!(cli.command, Command::Ui(_)));
    
    let cli = Cli::try_parse_from(["dora", "dashboard"]).unwrap();
    assert!(matches!(cli.command, Command::Dashboard(_)));
}

#[test]
fn test_complex_command_with_global_flags() {
    let cli = Cli::try_parse_from([
        "dora", 
        "--ui-mode", "tui", 
        "--output", "yaml", 
        "--verbose", 
        "--no-hints",
        "inspect", 
        "--deep", 
        "node1"
    ]).unwrap();
    
    assert_eq!(cli.ui_mode, Some(UiMode::Tui));
    assert_eq!(cli.output, OutputFormat::Yaml);
    assert!(cli.verbose);
    assert!(cli.no_hints);
    assert!(matches!(cli.command, Command::Inspect(_)));
}

#[test]
fn test_help_generation() {
    // Test that help can be generated
    let result = Cli::try_parse_from(["dora", "--help"]);
    assert!(result.is_err()); // Help generates an error with exit code 0
    
    let error = result.unwrap_err();
    let output = error.to_string();
    assert!(output.contains("Dora dataflow runtime"));
    assert!(output.contains("--ui-mode"));
    assert!(output.contains("--output"));
    assert!(output.contains("--no-hints"));
}

#[test]
fn test_error_handling() {
    // Test invalid arguments produce helpful errors
    let result = Cli::try_parse_from(["dora", "invalid-command"]);
    assert!(result.is_err());
    
    let error = result.unwrap_err();
    let output = error.to_string();
    assert!(output.contains("unrecognized subcommand"));
}

#[test]
fn test_ui_mode_values() {
    // Test all UI mode values
    let cli = Cli::try_parse_from(["dora", "--ui-mode", "auto", "ps"]).unwrap();
    assert_eq!(cli.ui_mode, Some(UiMode::Auto));
    
    let cli = Cli::try_parse_from(["dora", "--ui-mode", "cli", "ps"]).unwrap();
    assert_eq!(cli.ui_mode, Some(UiMode::Cli));
    
    let cli = Cli::try_parse_from(["dora", "--ui-mode", "tui", "ps"]).unwrap();
    assert_eq!(cli.ui_mode, Some(UiMode::Tui));
    
    let cli = Cli::try_parse_from(["dora", "--ui-mode", "minimal", "ps"]).unwrap();
    assert_eq!(cli.ui_mode, Some(UiMode::Minimal));
}

#[test]
fn test_output_format_values() {
    // Test all output format values
    let cli = Cli::try_parse_from(["dora", "--output", "auto", "ps"]).unwrap();
    assert_eq!(cli.output, OutputFormat::Auto);
    
    let cli = Cli::try_parse_from(["dora", "--output", "table", "ps"]).unwrap();
    assert_eq!(cli.output, OutputFormat::Table);
    
    let cli = Cli::try_parse_from(["dora", "--output", "json", "ps"]).unwrap();
    assert_eq!(cli.output, OutputFormat::Json);
    
    let cli = Cli::try_parse_from(["dora", "--output", "yaml", "ps"]).unwrap();
    assert_eq!(cli.output, OutputFormat::Yaml);
    
    let cli = Cli::try_parse_from(["dora", "--output", "minimal", "ps"]).unwrap();
    assert_eq!(cli.output, OutputFormat::Minimal);
}