use std::collections::HashMap;
use std::path::PathBuf;
use anyhow::{anyhow, Context, Result};
use serde_json::Value;

use super::structs::DoraConfig;

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
            if global_path.exists() {
                if let Ok(global_config) = Self::load_config_file(global_path) {
                    config = Self::merge_configs(config, global_config);
                }
            }
        }
        
        if config_path.exists() {
            let user_config = Self::load_config_file(&config_path)?;
            config = Self::merge_configs(config, user_config);
        }
        
        if let Some(project_path) = &project_config_path {
            if project_path.exists() {
                if let Ok(project_config) = Self::load_config_file(project_path) {
                    config = Self::merge_configs(config, project_config);
                }
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
    
    /// Create ConfigManager with existing configuration (for testing)
    pub fn new_with_config(config: DoraConfig) -> Self {
        let config_path = Self::get_user_config_path().unwrap_or_else(|_| {
            std::env::temp_dir().join("test-dora-config.toml")
        });
        
        Self {
            config,
            config_path,
            global_config_path: None,
            project_config_path: None,
        }
    }
    
    /// Save current configuration to user config file
    pub fn save(&self) -> Result<()> {
        self.ensure_config_dir()?;
        let toml = toml::to_string_pretty(&self.config)
            .context("Failed to serialize configuration to TOML")?;
        std::fs::write(&self.config_path, toml)
            .with_context(|| format!("Failed to write configuration to {}", self.config_path.display()))?;
        Ok(())
    }
    
    /// Get current configuration
    pub fn config(&self) -> &DoraConfig {
        &self.config
    }
    
    /// Update configuration value using dot notation
    pub fn set(&mut self, key: &str, value: Value) -> Result<()> {
        self.set_nested_value(key, value)?;
        self.config.validate()?;
        Ok(())
    }
    
    /// Get configuration value using dot notation
    pub fn get(&self, key: &str) -> Result<Value> {
        self.get_nested_value(key)
    }
    
    /// List all configuration values as key-value pairs
    pub fn list(&self, filter: Option<&str>, changed_only: bool) -> Result<Vec<(String, Value)>> {
        let config_value = serde_json::to_value(&self.config)?;
        let default_value = serde_json::to_value(&DoraConfig::default())?;
        
        let mut results = Vec::new();
        self.collect_flattened_values("", &config_value, &mut results);
        
        // Apply filters
        if let Some(filter_prefix) = filter {
            results.retain(|(key, _)| key.starts_with(filter_prefix));
        }
        
        if changed_only {
            let mut default_values = Vec::new();
            self.collect_flattened_values("", &default_value, &mut default_values);
            let default_map: HashMap<String, Value> = default_values.into_iter().collect();
            
            results.retain(|(key, value)| {
                default_map.get(key).map_or(true, |default| default != value)
            });
        }
        
        results.sort_by(|a, b| a.0.cmp(&b.0));
        Ok(results)
    }
    
    /// Reset configuration to defaults
    pub fn reset(&mut self, key: Option<&str>) -> Result<()> {
        match key {
            Some(key_path) => {
                // Reset specific key to default
                let default_config = DoraConfig::default();
                let default_value = self.get_nested_value_from_config(&default_config, key_path)?;
                self.set_nested_value(key_path, default_value)?;
            },
            None => {
                // Reset entire configuration
                self.config = DoraConfig::default();
            }
        }
        
        self.config.validate()?;
        Ok(())
    }
    
    /// Get configuration file paths
    pub fn get_config_paths(&self) -> ConfigPaths {
        ConfigPaths {
            user: self.config_path.clone(),
            global: self.global_config_path.clone(),
            project: self.project_config_path.clone(),
        }
    }
}

#[derive(Debug)]
pub struct ConfigPaths {
    pub user: PathBuf,
    pub global: Option<PathBuf>,
    pub project: Option<PathBuf>,
}

impl ConfigManager {
    fn get_user_config_path() -> Result<PathBuf> {
        // Check environment variable override first
        if let Ok(custom_path) = std::env::var("DORA_CONFIG_PATH") {
            return Ok(PathBuf::from(custom_path));
        }
        
        let home = dirs::home_dir()
            .ok_or_else(|| anyhow!("Could not determine home directory"))?;
        Ok(home.join(".config").join("dora").join("config.toml"))
    }
    
    fn get_global_config_path() -> Result<PathBuf> {
        // System-wide configuration
        #[cfg(target_os = "linux")]
        return Ok(PathBuf::from("/etc/dora/config.toml"));
        
        #[cfg(target_os = "macos")]
        return Ok(PathBuf::from("/usr/local/etc/dora/config.toml"));
        
        #[cfg(target_os = "windows")]
        return Ok(PathBuf::from("C:\\ProgramData\\Dora\\config.toml"));
        
        #[cfg(not(any(target_os = "linux", target_os = "macos", target_os = "windows")))]
        return Err(anyhow!("Global configuration not supported on this platform"));
    }
    
    fn get_project_config_path() -> Result<PathBuf> {
        let current_dir = std::env::current_dir()
            .context("Failed to get current directory")?;
        
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
    
    fn load_config_file(path: &PathBuf) -> Result<DoraConfig> {
        let content = std::fs::read_to_string(path)
            .with_context(|| format!("Failed to read configuration file: {}", path.display()))?;
        
        let config: DoraConfig = toml::from_str(&content)
            .with_context(|| format!("Failed to parse configuration file: {}", path.display()))?;
        
        Ok(config)
    }
    
    /// Merge two configurations, with the override taking precedence
    pub fn merge_configs(base: DoraConfig, override_config: DoraConfig) -> DoraConfig {
        // For now, we'll do a simple field-by-field merge
        // In a more sophisticated implementation, we might use serde merge or custom logic
        
        let base_value = serde_json::to_value(&base).unwrap();
        let override_value = serde_json::to_value(&override_config).unwrap();
        
        let merged = Self::merge_json_values(base_value, override_value);
        serde_json::from_value(merged).unwrap()
    }
    
    fn merge_json_values(base: Value, override_val: Value) -> Value {
        match (base, override_val) {
            (Value::Object(mut base_map), Value::Object(override_map)) => {
                for (key, value) in override_map {
                    if let Some(base_value) = base_map.get(&key) {
                        base_map.insert(key, Self::merge_json_values(base_value.clone(), value));
                    } else {
                        base_map.insert(key, value);
                    }
                }
                Value::Object(base_map)
            },
            (_, override_val) => override_val,
        }
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
        
        let mut config_manager = Self::new_with_config(config);
        
        for (env_var, config_key) in env_mappings {
            if let Ok(value) = std::env::var(env_var) {
                let parsed_value = Self::parse_env_value(config_key, &value)?;
                config_manager.set_nested_value(config_key, parsed_value)?;
            }
        }
        
        Ok(config_manager.config)
    }
    
    fn parse_env_value(key: &str, value: &str) -> Result<Value> {
        match key {
            k if k.ends_with("threshold") => {
                let num: u8 = value.parse()
                    .with_context(|| format!("'{}' must be a number between 0-10", k))?;
                if num > 10 {
                    return Err(anyhow!("Threshold values must be between 0-10"));
                }
                Ok(Value::Number(num.into()))
            },
            
            k if k.contains("mode") => {
                let mode_value = match value.to_lowercase().as_str() {
                    "auto" => "Auto",
                    "cli" => "Cli",
                    "tui" => "Tui",
                    "minimal" => "Minimal",
                    _ => return Err(anyhow!("Invalid mode '{}'. Valid options: auto, cli, tui, minimal", value)),
                };
                Ok(Value::String(mode_value.to_string()))
            },
            
            k if k.contains("level") => {
                let level_value = match value.to_lowercase().as_str() {
                    "trace" => "Trace",
                    "debug" => "Debug",
                    "info" => "Info",
                    "warn" => "Warn",
                    "error" => "Error",
                    _ => return Err(anyhow!("Invalid log level '{}'. Valid options: trace, debug, info, warn, error", value)),
                };
                Ok(Value::String(level_value.to_string()))
            },
            
            k if matches!(value, "true" | "false") => {
                let bool_val: bool = value.parse()
                    .with_context(|| format!("'{}' must be true or false", k))?;
                Ok(Value::Bool(bool_val))
            },
            
            _ => {
                // Try parsing as JSON first, fall back to string
                serde_json::from_str(value)
                    .or_else(|_| Ok(Value::String(value.to_string())))
            }
        }
    }
    
    fn set_nested_value(&mut self, key: &str, value: Value) -> Result<()> {
        let config_value = serde_json::to_value(&self.config)?;
        let updated_value = Self::set_nested_json_value(config_value, key, value)?;
        self.config = serde_json::from_value(updated_value)?;
        Ok(())
    }
    
    fn set_nested_json_value(mut obj: Value, key: &str, value: Value) -> Result<Value> {
        let parts: Vec<&str> = key.split('.').collect();
        Self::set_nested_json_value_recursive(&mut obj, &parts, value)?;
        Ok(obj)
    }
    
    fn set_nested_json_value_recursive(obj: &mut Value, path: &[&str], value: Value) -> Result<()> {
        if path.is_empty() {
            return Err(anyhow!("Empty path"));
        }
        
        if path.len() == 1 {
            if let Value::Object(map) = obj {
                map.insert(path[0].to_string(), value);
                return Ok(());
            }
            return Err(anyhow!("Cannot set value on non-object"));
        }
        
        if let Value::Object(map) = obj {
            let key = path[0];
            if !map.contains_key(key) {
                map.insert(key.to_string(), Value::Object(serde_json::Map::new()));
            }
            
            if let Some(nested) = map.get_mut(key) {
                Self::set_nested_json_value_recursive(nested, &path[1..], value)?;
            }
        } else {
            return Err(anyhow!("Cannot navigate through non-object"));
        }
        
        Ok(())
    }
    
    fn get_nested_value(&self, key: &str) -> Result<Value> {
        self.get_nested_value_from_config(&self.config, key)
    }
    
    fn get_nested_value_from_config(&self, config: &DoraConfig, key: &str) -> Result<Value> {
        let config_value = serde_json::to_value(config)?;
        let parts: Vec<&str> = key.split('.').collect();
        Self::get_nested_json_value(&config_value, &parts)
    }
    
    fn get_nested_json_value(obj: &Value, path: &[&str]) -> Result<Value> {
        if path.is_empty() {
            return Ok(obj.clone());
        }
        
        if let Value::Object(map) = obj {
            if let Some(value) = map.get(path[0]) {
                if path.len() == 1 {
                    Ok(value.clone())
                } else {
                    Self::get_nested_json_value(value, &path[1..])
                }
            } else {
                Err(anyhow!("Key '{}' not found", path[0]))
            }
        } else {
            Err(anyhow!("Cannot navigate through non-object at '{}'", path[0]))
        }
    }
    
    fn collect_flattened_values(&self, prefix: &str, obj: &Value, results: &mut Vec<(String, Value)>) {
        match obj {
            Value::Object(map) => {
                for (key, value) in map {
                    let full_key = if prefix.is_empty() {
                        key.clone()
                    } else {
                        format!("{}.{}", prefix, key)
                    };
                    
                    if value.is_object() {
                        self.collect_flattened_values(&full_key, value, results);
                    } else {
                        results.push((full_key, value.clone()));
                    }
                }
            },
            _ => {
                results.push((prefix.to_string(), obj.clone()));
            }
        }
    }
    
    fn ensure_config_dir(&self) -> Result<()> {
        if let Some(parent) = self.config_path.parent() {
            std::fs::create_dir_all(parent)
                .with_context(|| format!("Failed to create config directory: {}", parent.display()))?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use tempfile::tempdir;
    use crate::cli::{UiMode, OutputFormat};
    
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
        assert_eq!(config.ui.complexity_threshold, parsed.ui.complexity_threshold);
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
        
        let tui_value = serde_json::Value::String("Tui".to_string());
        manager.set("ui.mode", tui_value).unwrap();
        
        let value = manager.get("ui.mode").unwrap();
        assert_eq!(value, serde_json::Value::String("Tui".to_string()));
    }
    
    #[test]
    fn test_env_var_overrides() {
        unsafe {
            std::env::set_var("DORA_UI_MODE", "cli");
        }
        let config = ConfigManager::apply_env_overrides(DoraConfig::default()).unwrap();
        assert_eq!(config.ui.mode, UiMode::Cli);
        unsafe {
            std::env::remove_var("DORA_UI_MODE");
        }
    }
    
    #[test]
    fn test_config_list_functionality() {
        let manager = ConfigManager::new_with_config(DoraConfig::default());
        
        // Test listing all values
        let all_values = manager.list(None, false).unwrap();
        assert!(!all_values.is_empty());
        
        // Test filtering
        let ui_values = manager.list(Some("ui"), false).unwrap();
        assert!(ui_values.iter().all(|(key, _)| key.starts_with("ui")));
    }
    
    #[test]
    fn test_config_reset() {
        let mut manager = ConfigManager::new_with_config(DoraConfig::default());
        
        // Change a value
        let cli_value = serde_json::Value::String("Cli".to_string());
        manager.set("ui.mode", cli_value).unwrap();
        assert_eq!(manager.config().ui.mode, UiMode::Cli);
        
        // Reset specific key
        manager.reset(Some("ui.mode")).unwrap();
        assert_eq!(manager.config().ui.mode, UiMode::Auto);
        
        // Reset entire config
        let five_value = serde_json::Value::Number(8.into());
        manager.set("ui.complexity_threshold", five_value).unwrap();
        manager.reset(None).unwrap();
        assert_eq!(manager.config().ui.complexity_threshold, 5);
    }
}