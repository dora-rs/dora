use std::env;
use std::fs;
use tempfile::tempdir;
use dora_cli::config::{ConfigManager, DoraConfig};
use dora_cli::cli::{UiMode, OutputFormat};

#[test]
fn test_config_file_loading() {
    let temp_dir = tempdir().unwrap();
    let config_path = temp_dir.path().join("config.toml");
    
    let config_content = r#"
[ui]
mode = "Tui"
complexity_threshold = 7
auto_launch_threshold = 0.8
show_hints = false
hint_frequency = "Always"

[ui.theme]
name = "dark"

[ui.command_modes]

[commands]
default_output_format = "Json"

[commands.refresh_intervals]

[commands.default_flags]

[commands.aliases]

[system.daemon]
url = "http://localhost:8888"
timeout = 45
retry_attempts = 3
retry_delay = 1000

[system.logging]
level = "Info"
file = ""
colored = true
include_timestamps = true

[system.performance]
decision_cache_size = 1000
max_concurrent_ops = 10
stream_buffer_size = 8192

[telemetry]
enabled = false

[telemetry.collection]
usage_data = false
performance_metrics = false
error_reporting = false

[telemetry.privacy]
anonymize_paths = true
anonymize_commands = false

[advanced]
experimental_features = false
debug_mode = false

[advanced.cache]
dataflow_validation = true
command_history = true

[advanced.compatibility]
legacy_command_support = true
"#;
    
    fs::write(&config_path, config_content).unwrap();
    
    // Test loading with custom path
    unsafe { env::set_var("DORA_CONFIG_PATH", config_path.to_str().unwrap()); }
    let manager = ConfigManager::load().unwrap();
    
    assert_eq!(manager.config().ui.mode, UiMode::Tui);
    assert_eq!(manager.config().ui.complexity_threshold, 7);
    assert_eq!(manager.config().ui.auto_launch_threshold, 0.8);
    assert!(!manager.config().ui.show_hints);
    assert_eq!(manager.config().ui.theme.name, "dark");
    assert_eq!(manager.config().commands.default_output_format, OutputFormat::Json);
    assert_eq!(manager.config().system.daemon.url, "http://localhost:8888");
    assert_eq!(manager.config().system.daemon.timeout, 45);
    
    unsafe { env::remove_var("DORA_CONFIG_PATH"); }
}

#[test]
fn test_config_validation() {
    let temp_dir = tempdir().unwrap();
    let config_path = temp_dir.path().join("config.toml");
    
    // Test invalid configuration (missing required fields - should be caught by validation)
    let invalid_config = r#"
[ui]
mode = "Auto"
complexity_threshold = 15
auto_launch_threshold = 1.5
hint_frequency = "Always"
show_hints = true

[ui.theme]
name = "dark"

[ui.command_modes]

[commands]
default_output_format = "Table"

[commands.refresh_intervals]

[commands.default_flags]

[commands.aliases]

[system.daemon]
url = "http://localhost:7777"
timeout = 30
retry_attempts = 3
retry_delay = 1000

[system.logging]
level = "Info"
file = ""
colored = true
include_timestamps = true

[system.performance]
decision_cache_size = 1000
max_concurrent_ops = 10
stream_buffer_size = 8192

[telemetry]
enabled = false

[telemetry.collection]
usage_data = false
performance_metrics = false
error_reporting = false

[telemetry.privacy]
anonymize_paths = true
anonymize_commands = false

[advanced]
experimental_features = false
debug_mode = false

[advanced.cache]
dataflow_validation = true
command_history = true

[advanced.compatibility]
legacy_command_support = true
"#;
    
    fs::write(&config_path, invalid_config).unwrap();
    unsafe { env::set_var("DORA_CONFIG_PATH", config_path.to_str().unwrap()); }
    
    // Should fail validation
    let result = ConfigManager::load();
    assert!(result.is_err());
    
    unsafe { env::remove_var("DORA_CONFIG_PATH"); }
}

#[test]
fn test_config_nested_access() {
    let mut manager = ConfigManager::new_with_config(DoraConfig::default());
    
    // Test setting nested values
    let tui_value = serde_json::Value::String("Tui".to_string());
    manager.set("ui.mode", tui_value).unwrap();
    
    let threshold_value = serde_json::Value::Number(8.into());
    manager.set("ui.complexity_threshold", threshold_value).unwrap();
    
    // Test getting nested values
    let mode_value = manager.get("ui.mode").unwrap();
    assert_eq!(mode_value, serde_json::Value::String("Tui".to_string()));
    
    let threshold_value = manager.get("ui.complexity_threshold").unwrap();
    assert_eq!(threshold_value, serde_json::Value::Number(8.into()));
}

#[test]
fn test_config_list_functionality() {
    let manager = ConfigManager::new_with_config(DoraConfig::default());
    
    // Test listing all values
    let all_values = manager.list(None, false).unwrap();
    assert!(!all_values.is_empty());
    
    // Verify some expected keys exist
    let keys: Vec<&String> = all_values.iter().map(|(k, _)| k).collect();
    assert!(keys.iter().any(|k| k.contains("ui.mode")));
    assert!(keys.iter().any(|k| k.contains("ui.complexity_threshold")));
    assert!(keys.iter().any(|k| k.contains("system.daemon.url")));
    
    // Test filtering
    let ui_values = manager.list(Some("ui"), false).unwrap();
    assert!(ui_values.iter().all(|(key, _)| key.starts_with("ui")));
    assert!(!ui_values.is_empty());
    
    // Test changed_only filter with default config
    let changed_values = manager.list(None, true).unwrap();
    assert!(changed_values.is_empty()); // Should be empty for default config
}

#[test]
fn test_config_reset_functionality() {
    let mut manager = ConfigManager::new_with_config(DoraConfig::default());
    
    // Change some values
    let cli_value = serde_json::Value::String("Cli".to_string());
    manager.set("ui.mode", cli_value).unwrap();
    let threshold_value = serde_json::Value::Number(8.into());
    manager.set("ui.complexity_threshold", threshold_value).unwrap();
    
    // Verify changes
    assert_eq!(manager.config().ui.mode, UiMode::Cli);
    assert_eq!(manager.config().ui.complexity_threshold, 8);
    
    // Reset specific key
    manager.reset(Some("ui.mode")).unwrap();
    assert_eq!(manager.config().ui.mode, UiMode::Auto); // Back to default
    assert_eq!(manager.config().ui.complexity_threshold, 8); // Should remain changed
    
    // Reset entire config
    manager.reset(None).unwrap();
    assert_eq!(manager.config().ui.mode, UiMode::Auto);
    assert_eq!(manager.config().ui.complexity_threshold, 5); // Back to default
}

#[test]
fn test_env_var_overrides() {
    // Set environment variables
    unsafe { env::set_var("DORA_UI_MODE", "Cli"); }
    unsafe { env::set_var("DORA_UI_COMPLEXITY_THRESHOLD", "8"); }
    unsafe { env::set_var("DORA_OUTPUT_FORMAT", "Json"); }
    unsafe { env::set_var("DORA_LOG_LEVEL", "Debug"); }
    
    let temp_dir = tempdir().unwrap();
    let config_path = temp_dir.path().join("config.toml");
    
    // Create a basic config file
    let config_content = r#"
[ui]
mode = "Auto"
complexity_threshold = 5
auto_launch_threshold = 0.5
hint_frequency = "Always"
show_hints = true

[ui.theme]
name = "default"

[ui.command_modes]

[commands]
default_output_format = "Table"

[commands.refresh_intervals]

[commands.default_flags]

[commands.aliases]

[system.daemon]
url = "http://localhost:7777"
timeout = 30
retry_attempts = 3
retry_delay = 1000

[system.logging]
level = "Info"
file = ""
colored = true
include_timestamps = true

[system.performance]
decision_cache_size = 1000
max_concurrent_ops = 10
stream_buffer_size = 8192

[telemetry]
enabled = false

[telemetry.collection]
usage_data = false
performance_metrics = false
error_reporting = false

[telemetry.privacy]
anonymize_paths = true
anonymize_commands = false

[advanced]
experimental_features = false
debug_mode = false

[advanced.cache]
dataflow_validation = true
command_history = true

[advanced.compatibility]
legacy_command_support = true
"#;
    
    fs::write(&config_path, config_content).unwrap();
    unsafe { env::set_var("DORA_CONFIG_PATH", config_path.to_str().unwrap()); }
    
    let manager = ConfigManager::load().unwrap();
    
    // Environment variables should override config file values
    assert_eq!(manager.config().ui.mode, UiMode::Cli);
    assert_eq!(manager.config().ui.complexity_threshold, 8);
    assert_eq!(manager.config().commands.default_output_format, OutputFormat::Json);
    assert_eq!(manager.config().system.logging.level, dora_cli::config::LogLevel::Debug);
    
    // Cleanup
    unsafe { env::remove_var("DORA_UI_MODE"); }
    unsafe { env::remove_var("DORA_UI_COMPLEXITY_THRESHOLD"); }
    unsafe { env::remove_var("DORA_OUTPUT_FORMAT"); }
    unsafe { env::remove_var("DORA_LOG_LEVEL"); }
    unsafe { env::remove_var("DORA_CONFIG_PATH"); }
}

#[test]
fn test_config_persistence() {
    // Clean up any environment variables from previous tests
    unsafe { env::remove_var("DORA_UI_MODE"); }
    unsafe { env::remove_var("DORA_UI_COMPLEXITY_THRESHOLD"); }
    unsafe { env::remove_var("DORA_OUTPUT_FORMAT"); }
    unsafe { env::remove_var("DORA_LOG_LEVEL"); }
    
    let temp_dir = tempdir().unwrap();
    let config_path = temp_dir.path().join("config.toml");
    
    unsafe { env::set_var("DORA_CONFIG_PATH", config_path.to_str().unwrap()); }
    
    // Create and save a configuration
    {
        let mut manager = ConfigManager::load().unwrap();
        let tui_value = serde_json::Value::String("Tui".to_string());
        manager.set("ui.mode", tui_value).unwrap();
        let threshold_value = serde_json::Value::Number(7.into());
        manager.set("ui.complexity_threshold", threshold_value).unwrap();
        manager.save().unwrap();
    }
    
    // Load it again and verify persistence
    {
        let manager = ConfigManager::load().unwrap();
        assert_eq!(manager.config().ui.mode, UiMode::Tui);
        assert_eq!(manager.config().ui.complexity_threshold, 7);
    }
    
    unsafe { env::remove_var("DORA_CONFIG_PATH"); }
}

#[test]
fn test_config_error_handling() {
    let temp_dir = tempdir().unwrap();
    let config_path = temp_dir.path().join("config.toml");
    
    // Test malformed TOML
    let malformed_config = r#"
[ui
mode = "Tui"
invalid syntax here
"#;
    
    fs::write(&config_path, malformed_config).unwrap();
    unsafe { env::set_var("DORA_CONFIG_PATH", config_path.to_str().unwrap()); }
    
    let result = ConfigManager::load();
    assert!(result.is_err());
    
    unsafe { env::remove_var("DORA_CONFIG_PATH"); }
}