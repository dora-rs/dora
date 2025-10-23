# Issue #004: Add Basic Configuration System

## 📋 Summary
Implement a comprehensive configuration system that manages user preferences, interface settings, and command defaults for the hybrid CLI architecture. This system provides persistent storage for user preferences and enables customization of the intelligent interface selection behavior.

## 🎯 Objectives
- Create hierarchical configuration system (global, user, project-specific)
- Implement user preference management for UI mode selection
- Add configuration validation and migration capabilities
- Provide CLI commands for configuration management
- Enable environment variable overrides for CI/automation

**Success Metrics:**
- Configuration loads in <10ms on startup
- All user preferences persist correctly across sessions
- Configuration validation catches 100% of invalid settings
- CLI configuration commands are intuitive and complete
- Environment variable overrides work reliably in all contexts

## 🛠️ Technical Requirements

### What to Build

#### 1. Configuration Structure and Storage
```rust
// src/config/mod.rs
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::path::PathBuf;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DoraConfig {
    /// UI behavior preferences
    pub ui: UiConfig,
    
    /// Command-specific settings
    pub commands: CommandConfig,
    
    /// System integration settings
    pub system: SystemConfig,
    
    /// Analytics and telemetry preferences
    pub telemetry: TelemetryConfig,
    
    /// Advanced user settings
    pub advanced: AdvancedConfig,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UiConfig {
    /// Global UI mode preference
    pub mode: UiMode,
    
    /// Complexity threshold for TUI suggestions (1-10)
    pub complexity_threshold: u8,
    
    /// Confidence threshold for auto-launching TUI (0.0-1.0)
    pub auto_launch_threshold: f32,
    
    /// Whether to show helpful hints
    pub show_hints: bool,
    
    /// Hint display frequency
    pub hint_frequency: HintFrequency,
    
    /// Theme preference for TUI
    pub theme: ThemeConfig,
    
    /// Per-command UI mode overrides
    pub command_modes: HashMap<String, UiMode>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CommandConfig {
    /// Default output format
    pub default_output_format: OutputFormat,
    
    /// Auto-refresh intervals for monitoring commands
    pub refresh_intervals: HashMap<String, u64>,
    
    /// Default flags for specific commands
    pub default_flags: HashMap<String, Vec<String>>,
    
    /// Command aliases
    pub aliases: HashMap<String, String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SystemConfig {
    /// Daemon connection settings
    pub daemon: DaemonConfig,
    
    /// Logging configuration
    pub logging: LoggingConfig,
    
    /// Performance settings
    pub performance: PerformanceConfig,
}

impl Default for DoraConfig {
    fn default() -> Self {
        Self {
            ui: UiConfig::default(),
            commands: CommandConfig::default(),
            system: SystemConfig::default(),
            telemetry: TelemetryConfig::default(),
            advanced: AdvancedConfig::default(),
        }
    }
}
```

#### 2. Configuration Management System
```rust
// src/config/manager.rs
#[derive(Debug)]
pub struct ConfigManager {
    config: DoraConfig,
    config_path: PathBuf,
    global_config_path: Option<PathBuf>,
    project_config_path: Option<PathBuf>,
}

impl ConfigManager {
    /// Load configuration with hierarchical precedence
    pub fn load() -> Result<Self> {
        let config_path = Self::get_user_config_path()?;
        let global_config_path = Self::get_global_config_path().ok();
        let project_config_path = Self::get_project_config_path().ok();
        
        let mut config = DoraConfig::default();
        
        // Load in order of precedence (global -> user -> project -> env vars)
        if let Some(global_path) = &global_config_path {
            if let Ok(global_config) = Self::load_config_file(global_path) {
                config = Self::merge_configs(config, global_config);
            }
        }
        
        if config_path.exists() {
            let user_config = Self::load_config_file(&config_path)?;
            config = Self::merge_configs(config, user_config);
        }
        
        if let Some(project_path) = &project_config_path {
            if let Ok(project_config) = Self::load_config_file(project_path) {
                config = Self::merge_configs(config, project_config);
            }
        }
        
        // Apply environment variable overrides
        config = Self::apply_env_overrides(config)?;
        
        // Validate final configuration
        config.validate()?;
        
        Ok(Self {
            config,
            config_path,
            global_config_path,
            project_config_path,
        })
    }
    
    /// Save current configuration to user config file
    pub fn save(&self) -> Result<()> {
        self.ensure_config_dir()?;
        let toml = toml::to_string_pretty(&self.config)?;
        std::fs::write(&self.config_path, toml)?;
        Ok(())
    }
    
    /// Get current configuration
    pub fn config(&self) -> &DoraConfig {
        &self.config
    }
    
    /// Update configuration value
    pub fn set<T: Serialize>(&mut self, key: &str, value: T) -> Result<()> {
        self.set_nested_value(key, value)?;
        self.config.validate()?;
        Ok(())
    }
    
    /// Get configuration value
    pub fn get(&self, key: &str) -> Result<serde_json::Value> {
        self.get_nested_value(key)
    }
}

impl ConfigManager {
    fn get_user_config_path() -> Result<PathBuf> {
        let home = dirs::home_dir()
            .ok_or_else(|| anyhow!("Could not determine home directory"))?;
        Ok(home.join(".config").join("dora").join("config.toml"))
    }
    
    fn get_global_config_path() -> Result<PathBuf> {
        // System-wide configuration (Linux: /etc/dora/, macOS: /usr/local/etc/dora/)
        #[cfg(target_os = "linux")]
        return Ok(PathBuf::from("/etc/dora/config.toml"));
        
        #[cfg(target_os = "macos")]
        return Ok(PathBuf::from("/usr/local/etc/dora/config.toml"));
        
        #[cfg(target_os = "windows")]
        return Ok(PathBuf::from("C:\\ProgramData\\Dora\\config.toml"));
    }
    
    fn get_project_config_path() -> Result<PathBuf> {
        let current_dir = std::env::current_dir()?;
        
        // Look for .dora/config.toml in current directory and parents
        let mut dir = current_dir.as_path();
        loop {
            let config_path = dir.join(".dora").join("config.toml");
            if config_path.exists() {
                return Ok(config_path);
            }
            
            if let Some(parent) = dir.parent() {
                dir = parent;
            } else {
                break;
            }
        }
        
        Err(anyhow!("No project configuration found"))
    }
    
    fn apply_env_overrides(mut config: DoraConfig) -> Result<DoraConfig> {
        // Environment variable mapping
        let env_mappings = [
            ("DORA_UI_MODE", "ui.mode"),
            ("DORA_UI_THEME", "ui.theme.name"),
            ("DORA_UI_COMPLEXITY_THRESHOLD", "ui.complexity_threshold"),
            ("DORA_OUTPUT_FORMAT", "commands.default_output_format"),
            ("DORA_LOG_LEVEL", "system.logging.level"),
            ("DORA_DAEMON_URL", "system.daemon.url"),
        ];
        
        for (env_var, config_key) in env_mappings {
            if let Ok(value) = std::env::var(env_var) {
                Self::set_config_from_string(&mut config, config_key, &value)?;
            }
        }
        
        Ok(config)
    }
}
```

#### 3. Configuration CLI Commands
```rust
// src/cli/commands/config.rs
#[derive(Debug, clap::Args)]
pub struct ConfigCommand {
    #[clap(subcommand)]
    pub action: ConfigAction,
}

#[derive(Debug, clap::Subcommand)]
pub enum ConfigAction {
    /// Get configuration value
    Get { key: String },
    
    /// Set configuration value
    Set { key: String, value: String },
    
    /// List all configuration values
    List {
        /// Filter by key prefix
        #[clap(long)]
        filter: Option<String>,
        
        /// Show only non-default values
        #[clap(long)]
        changed_only: bool,
    },
    
    /// Reset configuration to defaults
    Reset {
        /// Specific key to reset
        key: Option<String>,
        
        /// Confirm reset without prompt
        #[clap(long)]
        confirm: bool,
    },
    
    /// Show configuration file locations
    Paths,
    
    /// Validate current configuration
    Validate,
    
    /// Edit configuration in default editor
    Edit,
}

impl ConfigCommand {
    pub async fn execute(&self, context: &ExecutionContext) -> Result<()> {
        let mut config_manager = ConfigManager::load()?;
        
        match &self.action {
            ConfigAction::Get { key } => {
                let value = config_manager.get(key)?;
                println!("{}", serde_json::to_string_pretty(&value)?);
            },
            
            ConfigAction::Set { key, value } => {
                self.set_config_value(&mut config_manager, key, value)?;
                config_manager.save()?;
                println!("✅ Configuration updated: {} = {}", key, value);
            },
            
            ConfigAction::List { filter, changed_only } => {
                self.list_configuration(&config_manager, filter.as_deref(), *changed_only)?;
            },
            
            ConfigAction::Reset { key, confirm } => {
                self.reset_configuration(&mut config_manager, key.as_deref(), *confirm)?;
            },
            
            ConfigAction::Paths => {
                self.show_config_paths(&config_manager)?;
            },
            
            ConfigAction::Validate => {
                match config_manager.config().validate() {
                    Ok(()) => println!("✅ Configuration is valid"),
                    Err(e) => {
                        eprintln!("❌ Configuration validation failed: {}", e);
                        std::process::exit(1);
                    }
                }
            },
            
            ConfigAction::Edit => {
                self.edit_configuration(&config_manager)?;
            },
        }
        
        Ok(())
    }
    
    fn set_config_value(
        &self, 
        config_manager: &mut ConfigManager, 
        key: &str, 
        value: &str
    ) -> Result<()> {
        // Parse value based on expected type
        let parsed_value = self.parse_config_value(key, value)?;
        config_manager.set(key, parsed_value)?;
        Ok(())
    }
    
    fn parse_config_value(&self, key: &str, value: &str) -> Result<serde_json::Value> {
        // Type-aware parsing based on configuration schema
        match key {
            k if k.ends_with("threshold") => {
                let num: u8 = value.parse()
                    .with_context(|| format!("'{}' must be a number between 0-10", k))?;
                if num > 10 {
                    return Err(anyhow!("Threshold values must be between 0-10"));
                }
                Ok(serde_json::Value::Number(num.into()))
            },
            
            k if k.contains("mode") => {
                match value.to_lowercase().as_str() {
                    "auto" => Ok(serde_json::Value::String("Auto".to_string())),
                    "cli" => Ok(serde_json::Value::String("Cli".to_string())),
                    "tui" => Ok(serde_json::Value::String("Tui".to_string())),
                    "minimal" => Ok(serde_json::Value::String("Minimal".to_string())),
                    _ => Err(anyhow!("Invalid mode '{}'. Valid options: auto, cli, tui, minimal", value)),
                }
            },
            
            k if k.contains("bool") || matches!(value, "true" | "false") => {
                let bool_val: bool = value.parse()
                    .with_context(|| format!("'{}' must be true or false", k))?;
                Ok(serde_json::Value::Bool(bool_val))
            },
            
            _ => {
                // Try parsing as JSON first, fall back to string
                serde_json::from_str(value)
                    .or_else(|_| Ok(serde_json::Value::String(value.to_string())))
            }
        }
    }
}
```

#### 4. Configuration Validation System
```rust
impl DoraConfig {
    pub fn validate(&self) -> Result<()> {
        self.ui.validate()
            .context("UI configuration validation failed")?;
        
        self.commands.validate()
            .context("Commands configuration validation failed")?;
        
        self.system.validate()
            .context("System configuration validation failed")?;
        
        Ok(())
    }
}

impl UiConfig {
    pub fn validate(&self) -> Result<()> {
        if self.complexity_threshold > 10 {
            return Err(anyhow!("complexity_threshold must be between 0-10"));
        }
        
        if !(0.0..=1.0).contains(&self.auto_launch_threshold) {
            return Err(anyhow!("auto_launch_threshold must be between 0.0-1.0"));
        }
        
        // Validate command mode overrides
        for (command, mode) in &self.command_modes {
            if command.is_empty() {
                return Err(anyhow!("Command name cannot be empty"));
            }
            // mode is already validated by enum
        }
        
        self.theme.validate()
            .context("Theme configuration validation failed")?;
        
        Ok(())
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ThemeConfig {
    pub name: String,
    pub colors: Option<ColorConfig>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ColorConfig {
    pub primary: String,
    pub secondary: String,
    pub success: String,
    pub warning: String,
    pub error: String,
    pub background: String,
    pub text: String,
}

impl ThemeConfig {
    pub fn validate(&self) -> Result<()> {
        let valid_themes = ["default", "dark", "light", "cyberpunk", "minimal"];
        if !valid_themes.contains(&self.name.as_str()) {
            return Err(anyhow!(
                "Invalid theme '{}'. Valid themes: {}", 
                self.name, 
                valid_themes.join(", ")
            ));
        }
        
        if let Some(colors) = &self.colors {
            colors.validate()?;
        }
        
        Ok(())
    }
}
```

### Why This Approach

**Hierarchical Configuration Benefits:**
- Supports organizational defaults and user customization
- Project-specific overrides for team workflows
- Environment variable support for CI/automation
- Clear precedence order prevents confusion

**Type-Safe Configuration:**
- Compile-time validation of configuration structure
- Runtime validation of values and constraints
- Clear error messages for invalid configurations
- Automatic serialization/deserialization

**CLI Integration:**
- Intuitive configuration management commands
- Tab completion for configuration keys
- Human-readable output and error messages
- Editor integration for complex configurations

### How to Implement

#### Step 1: Core Configuration Structure (3 hours)
1. **Define configuration structs** with proper serde attributes
2. **Implement default values** for all configuration options
3. **Add validation methods** for each configuration section
4. **Create configuration file format** (TOML for human readability)

#### Step 2: Configuration Manager (4 hours)
1. **Implement hierarchical loading** (global -> user -> project -> env)
2. **Add configuration merging** logic with proper precedence
3. **Create save/load functionality** with error handling
4. **Implement nested key access** for dot-notation paths

#### Step 3: CLI Commands (3 hours)
1. **Implement config subcommands** (get, set, list, reset, etc.)
2. **Add type-aware value parsing** for different configuration types
3. **Create human-readable output** formatting
4. **Add interactive editing** support

#### Step 4: Environment Variable Support (2 hours)
1. **Define environment variable mappings** for key configuration options
2. **Implement override application** logic
3. **Add validation** for environment variable values
4. **Create documentation** for supported environment variables

#### Step 5: Integration and Testing (2 hours)
1. **Integrate with CLI framework** from Issue #001
2. **Add configuration loading** to application startup
3. **Implement hot-reloading** for development
4. **Create comprehensive test suite**

## 🔗 Dependencies
**Depends On:** Issue #001 (Hybrid Command Framework) - Required for CLI command structure

**Blocks:** 
- Issue #003 (Interface Selection Engine) - Needs user preferences
- All subsequent issues - Configuration system is foundational

## 🧪 Testing Requirements

### Unit Tests
```rust
#[cfg(test)]
mod tests {
    use super::*;
    use tempfile::tempdir;
    
    #[test]
    fn test_default_config_validation() {
        let config = DoraConfig::default();
        assert!(config.validate().is_ok());
    }
    
    #[test]
    fn test_config_serialization_roundtrip() {
        let config = DoraConfig::default();
        let toml = toml::to_string(&config).unwrap();
        let parsed: DoraConfig = toml::from_str(&toml).unwrap();
        assert_eq!(config.ui.mode, parsed.ui.mode);
    }
    
    #[test]
    fn test_invalid_complexity_threshold() {
        let mut config = DoraConfig::default();
        config.ui.complexity_threshold = 15; // Invalid: > 10
        assert!(config.validate().is_err());
    }
    
    #[test]
    fn test_config_merging() {
        let base = DoraConfig::default();
        let mut override_config = DoraConfig::default();
        override_config.ui.mode = UiMode::Cli;
        
        let merged = ConfigManager::merge_configs(base, override_config);
        assert_eq!(merged.ui.mode, UiMode::Cli);
    }
    
    #[test]
    fn test_nested_key_access() {
        let mut manager = ConfigManager::new_with_config(DoraConfig::default());
        
        manager.set("ui.mode", UiMode::Tui).unwrap();
        let value = manager.get("ui.mode").unwrap();
        assert_eq!(value, serde_json::Value::String("Tui".to_string()));
    }
    
    #[test]
    fn test_env_var_overrides() {
        std::env::set_var("DORA_UI_MODE", "cli");
        let config = ConfigManager::apply_env_overrides(DoraConfig::default()).unwrap();
        assert_eq!(config.ui.mode, UiMode::Cli);
        std::env::remove_var("DORA_UI_MODE");
    }
}
```

### Integration Tests
```rust
// tests/config_integration.rs
#[test]
fn test_config_file_loading() {
    let temp_dir = tempdir().unwrap();
    let config_path = temp_dir.path().join("config.toml");
    
    let config = DoraConfig {
        ui: UiConfig {
            mode: UiMode::Tui,
            complexity_threshold: 5,
            ..Default::default()
        },
        ..Default::default()
    };
    
    let toml = toml::to_string_pretty(&config).unwrap();
    std::fs::write(&config_path, toml).unwrap();
    
    // Test loading
    std::env::set_var("DORA_CONFIG_PATH", config_path.to_str().unwrap());
    let manager = ConfigManager::load().unwrap();
    assert_eq!(manager.config().ui.mode, UiMode::Tui);
    assert_eq!(manager.config().ui.complexity_threshold, 5);
}

#[test]
fn test_cli_config_commands() {
    let temp_dir = tempdir().unwrap();
    std::env::set_var("DORA_CONFIG_PATH", temp_dir.path().join("config.toml"));
    
    // Test set command
    let result = ConfigCommand::execute_set("ui.mode", "tui");
    assert!(result.is_ok());
    
    // Test get command
    let value = ConfigCommand::execute_get("ui.mode").unwrap();
    assert_eq!(value, "Tui");
    
    // Test validation
    let validation_result = ConfigCommand::execute_validate();
    assert!(validation_result.is_ok());
}
```

### Manual Testing Procedures
1. **Configuration Management**
   ```bash
   # Test configuration commands
   dora config set ui.mode tui
   dora config get ui.mode
   dora config list
   dora config validate
   ```

2. **Hierarchical Loading**
   ```bash
   # Create project config
   mkdir test-project && cd test-project
   mkdir .dora
   echo 'ui.mode = "cli"' > .dora/config.toml
   
   # Should use project config
   dora config get ui.mode
   ```

3. **Environment Variable Overrides**
   ```bash
   # Test environment overrides
   DORA_UI_MODE=minimal dora config get ui.mode
   DORA_UI_COMPLEXITY_THRESHOLD=8 dora config get ui.complexity_threshold
   ```

## 📚 Resources

### Configuration Management References
- [XDG Base Directory Specification](https://specifications.freedesktop.org/basedir-spec/basedir-spec-latest.html)
- [TOML Specification](https://toml.io/en/)
- [Serde Documentation](https://serde.rs/)

### Configuration File Examples
```toml
# ~/.config/dora/config.toml
[ui]
mode = "Auto"
complexity_threshold = 5
auto_launch_threshold = 0.7
show_hints = true
hint_frequency = "OncePerCommand"

[ui.theme]
name = "default"

[ui.command_modes]
inspect = "Tui"
debug = "Tui"
logs = "Cli"

[commands]
default_output_format = "Table"

[commands.aliases]
list = "ps"
status = "ps"

[system.daemon]
url = "http://localhost:7777"
timeout = 30

[system.logging]
level = "Info"
file = "~/.local/share/dora/logs/dora.log"
```

## ✅ Definition of Done
- [ ] DoraConfig structure defined with all necessary configuration sections
- [ ] ConfigManager implements hierarchical loading (global -> user -> project -> env)
- [ ] Configuration validation catches all invalid values and provides helpful errors
- [ ] CLI config commands (get, set, list, reset, validate, edit) work correctly
- [ ] Environment variable overrides function properly for automation
- [ ] Configuration file format is human-readable and well-documented
- [ ] Type-safe configuration parsing with clear error messages
- [ ] Configuration merging works correctly with proper precedence
- [ ] Unit tests cover all configuration operations and edge cases
- [ ] Integration tests validate end-to-end configuration workflows
- [ ] Manual testing confirms configuration persistence across sessions
- [ ] Performance benchmarks show configuration loading <10ms
- [ ] Documentation includes configuration reference and examples

## 📝 Implementation Notes

### Performance Considerations
- Cache loaded configuration to avoid repeated file I/O
- Use lazy loading for optional configuration sections
- Optimize TOML parsing for startup performance
- Consider configuration file watching for development

### Security Considerations
- Validate file permissions on configuration files
- Sanitize user input in configuration values
- Avoid storing sensitive data in plain text configuration
- Consider encrypted configuration sections for secrets

### Future Extensions
- Configuration schema versioning and migration
- Remote configuration management for enterprise deployments
- Configuration templating system
- Integration with external configuration management tools
- Real-time configuration updates without restart

This configuration system provides the flexible foundation needed for user preference management while maintaining simplicity and reliability across all deployment scenarios.