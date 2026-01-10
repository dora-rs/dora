use std::collections::HashMap;
use std::fs;
use std::path::PathBuf;

use config::{Config, File, FileFormat};
use eyre::{Context, ContextCompat, Result, bail};

/// Configuration manager for dora CLI
pub struct ConfigManager {
    global_path: PathBuf,
    project_path: PathBuf,
}

impl ConfigManager {
    /// Create a new configuration manager
    pub fn new() -> Result<Self> {
        let global_path = Self::global_config_path()?;
        let project_path = Self::project_config_path();

        Ok(Self {
            global_path,
            project_path,
        })
    }

    /// Create a new configuration manager with specific paths (useful for testing)
    pub fn new_with_paths(global_path: PathBuf, project_path: PathBuf) -> Self {
        Self {
            global_path,
            project_path,
        }
    }

    /// Get the global config file path (~/.dora/config.toml)
    fn global_config_path() -> Result<PathBuf> {
        let home = dirs::home_dir().context("Could not determine home directory")?;
        Ok(home.join(".dora").join("config.toml"))
    }

    /// Get the project config file path (./dora.toml)
    fn project_config_path() -> PathBuf {
        PathBuf::from("dora.toml")
    }

    /// Load merged configuration from all sources
    /// Priority: environment variables > project config > global config
    pub fn load(&self) -> Result<HashMap<String, toml::Value>> {
        let mut builder = Config::builder();

        // Load global config if it exists (lowest priority)
        if self.global_path.exists() {
            builder = builder.add_source(
                File::from(self.global_path.clone())
                    .format(FileFormat::Toml)
                    .required(false),
            );
        }

        // Load project config if it exists (higher priority)
        if self.project_path.exists() {
            builder = builder.add_source(
                File::from(self.project_path.clone())
                    .format(FileFormat::Toml)
                    .required(false),
            );
        }

        let config = builder.build().context("Failed to build configuration")?;

        // Convert to HashMap<String, toml::Value> with flattened keys
        let mut result = HashMap::new();

        // Get all keys from the config files
        if let Ok(table) = config.try_deserialize::<HashMap<String, toml::Value>>() {
            flatten_table(&table, String::new(), &mut result);
        }

        // Add environment variable overrides (highest priority)
        // Environment variables should be in format: DORA_<KEY> where KEY uses double underscores
        // e.g., DORA_COORDINATOR__ADDR for coordinator.addr
        for (key, value) in std::env::vars() {
            if key.starts_with("DORA_") {
                let config_key = key
                    .strip_prefix("DORA_")
                    .unwrap()
                    .to_lowercase()
                    .replace("__", "."); // Double underscore becomes dot
                result.insert(config_key, toml::Value::String(value));
            }
        }

        Ok(result)
    }

    /// Get a specific configuration value
    /// Checks environment variables first, then project config, then global config
    pub fn get(&self, key: &str) -> Result<String> {
        // First check environment variable (highest priority)
        let env_key = format!("DORA_{}", key.to_uppercase().replace('.', "__"));
        if let Ok(value) = std::env::var(&env_key) {
            return Ok(value);
        }

        // Then check loaded config
        let config = self.load()?;
        let val = config
            .get(key)
            .ok_or_else(|| eyre::eyre!("Configuration key '{}' not found", key))?;
        
        Ok(value_to_string_raw(val))
    }

    /// Set a configuration value in the specified config file
    pub fn set(&self, key: &str, value: &str, local: bool) -> Result<()> {
        let path = if local {
            &self.project_path
        } else {
            &self.global_path
        };

        // Ensure parent directory exists
        if let Some(parent) = path.parent() {
            fs::create_dir_all(parent).with_context(|| {
                format!("Failed to create config directory: {}", parent.display())
            })?;
        }

        // Load existing config or create new one
        let mut config_map = if path.exists() {
            let content = fs::read_to_string(path)
                .with_context(|| format!("Failed to read config file: {}", path.display()))?;
            toml::from_str::<HashMap<String, toml::Value>>(&content)
                .with_context(|| format!("Failed to parse config file: {}", path.display()))?
        } else {
            HashMap::new()
        };

        // Parse the key path and set the value
        let parts: Vec<&str> = key.split('.').collect();
        if parts.is_empty() {
            bail!("Invalid configuration key: {}", key);
        }

        // For now, we support simple nested keys (e.g., "coordinator.addr")
        if parts.len() == 1 {
            // Simple key
            config_map.insert(key.to_string(), toml::Value::String(value.to_string()));
        } else if parts.len() == 2 {
            // Nested key (e.g., "coordinator.addr")
            let section = parts[0];
            let field = parts[1];

            let section_table = config_map
                .entry(section.to_string())
                .or_insert_with(|| toml::Value::Table(toml::map::Map::new()));

            if let toml::Value::Table(table) = section_table {
                table.insert(field.to_string(), toml::Value::String(value.to_string()));
            } else {
                bail!("Configuration key '{}' is not a table", section);
            }
        } else {
            bail!("Configuration keys with more than 2 levels are not supported yet");
        }

        // Write back to file
        let toml_string =
            toml::to_string_pretty(&config_map).context("Failed to serialize configuration")?;
        fs::write(path, toml_string)
            .with_context(|| format!("Failed to write config file: {}", path.display()))?;

        Ok(())
    }

    /// Unset (remove) a configuration value from the specified config file
    pub fn unset(&self, key: &str, local: bool) -> Result<()> {
        let path = if local {
            &self.project_path
        } else {
            &self.global_path
        };

        if !path.exists() {
            bail!("Configuration file does not exist: {}", path.display());
        }

        // Load existing config
        let content = fs::read_to_string(path)
            .with_context(|| format!("Failed to read config file: {}", path.display()))?;
        let mut config_map = toml::from_str::<HashMap<String, toml::Value>>(&content)
            .with_context(|| format!("Failed to parse config file: {}", path.display()))?;

        // Parse the key path and remove the value
        let parts: Vec<&str> = key.split('.').collect();
        if parts.is_empty() {
            bail!("Invalid configuration key: {}", key);
        }

        let removed = if parts.len() == 1 {
            // Simple key
            config_map.remove(key).is_some()
        } else if parts.len() == 2 {
            // Nested key
            let section = parts[0];
            let field = parts[1];

            if let Some(toml::Value::Table(table)) = config_map.get_mut(section) {
                table.remove(field).is_some()
            } else {
                false
            }
        } else {
            bail!("Configuration keys with more than 2 levels are not supported yet");
        };

        if !removed {
            bail!(
                "Configuration key '{}' not found in {}",
                key,
                path.display()
            );
        }

        // Write back to file
        let toml_string =
            toml::to_string_pretty(&config_map).context("Failed to serialize configuration")?;
        fs::write(path, toml_string)
            .with_context(|| format!("Failed to write config file: {}", path.display()))?;

        Ok(())
    }

    /// List all configuration values
    pub fn list(&self) -> Result<Vec<(String, toml::Value)>> {
        let config = self.load()?;
        let mut items: Vec<(String, toml::Value)> = config.into_iter().collect();
        items.sort_by(|a, b| a.0.cmp(&b.0));
        Ok(items)
    }
}

/// Recursively flatten a TOML table into dot-notation keys
fn flatten_table(
    table: &HashMap<String, toml::Value>,
    prefix: String,
    result: &mut HashMap<String, toml::Value>,
) {
    for (key, value) in table {
        let full_key = if prefix.is_empty() {
            key.clone()
        } else {
            format!("{}.{}", prefix, key)
        };

        match value {
            toml::Value::Table(nested_table) => {
                // Recursively flatten nested tables
                let nested_map: HashMap<String, toml::Value> =
                    nested_table.clone().into_iter().collect();
                flatten_table(&nested_map, full_key, result);
            }
            _ => {
                // Leaf value - add to result
                result.insert(full_key, value.clone());
            }
        }
    }
}

/// Convert a toml::Value to a raw string representation (no quotes for strings)
fn value_to_string_raw(value: &toml::Value) -> String {
    match value {
        toml::Value::String(s) => s.clone(),
        toml::Value::Integer(i) => i.to_string(),
        toml::Value::Float(f) => f.to_string(),
        toml::Value::Boolean(b) => b.to_string(),
        toml::Value::Table(table) => {
            // For nested tables, create dot-notation keys
            let mut result = Vec::new();
            for (k, v) in table {
                result.push(format!("{} = {}", k, value_to_string_raw(v)));
            }
            result.join(", ")
        }
        toml::Value::Array(arr) => {
            let items: Vec<String> = arr.iter().map(value_to_string_raw).collect();
            format!("[{}]", items.join(", "))
        }
        toml::Value::Datetime(dt) => dt.to_string(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs::File;
    use std::io::Write;
    use tempfile::TempDir;

    #[test]
    fn test_config_precedence() -> Result<()> {
        let temp_dir = TempDir::new()?;
        let global_path = temp_dir.path().join("global_config.toml");
        let project_path = temp_dir.path().join("project_config.toml");

        let manager = ConfigManager::new_with_paths(global_path.clone(), project_path.clone());

        // 1. Set global
        manager.set("test.key", "global_value", false)?;
        assert_eq!(manager.get("test.key")?, "global_value");

        // 2. Set project (should override global)
        manager.set("test.key", "project_value", true)?;
        assert_eq!(manager.get("test.key")?, "project_value");

        // 3. Env var (should override project)
        // We can't easily test env vars in parallel tests without side effects, 
        // but we can verify the manager reads them if we set one locally.
        // unsafe { std::env::set_var("DORA_TEST__KEY", "env_value"); }
        // assert_eq!(manager.get("test.key")?, "env_value");
        // unsafe { std::env::remove_var("DORA_TEST__KEY"); }
        
        Ok(())
    }

    #[test]
    fn test_set_get_unset() -> Result<()> {
        let temp_dir = TempDir::new()?;
        let global_path = temp_dir.path().join("dora").join("config.toml");
        let project_path = temp_dir.path().join("dora.toml");

        let manager = ConfigManager::new_with_paths(global_path.clone(), project_path.clone());

        // Test global set
        manager.set("coordinator.addr", "127.0.0.1", false)?;
        assert!(global_path.exists());
        assert_eq!(manager.get("coordinator.addr")?, "127.0.0.1");

        // Test project set
        manager.set("coordinator.port", "8080", true)?;
        assert!(project_path.exists());
        assert_eq!(manager.get("coordinator.port")?, "8080");

        // List
        let items = manager.list()?;
        // Check that items contain "coordinator.addr" key with typed value
        let addr = items.iter().find(|(k, _)| k == "coordinator.addr").unwrap();
        match &addr.1 {
            toml::Value::String(s) => assert_eq!(s, "127.0.0.1"),
            _ => panic!("Expected string"),
        }
         
        let port = items.iter().find(|(k, _)| k == "coordinator.port").unwrap();
        match &port.1 {
            toml::Value::String(s) => assert_eq!(s, "8080"),
            _ => panic!("Expected string"), // "8080" was set as string in test
        }

        // Unset global
        manager.unset("coordinator.addr", false)?;
        assert!(manager.get("coordinator.addr").is_err());

        Ok(())
    }
    
    #[test]
    fn test_nested_keys() -> Result<()> {
        let temp_dir = TempDir::new()?;
        let global_path = temp_dir.path().join("config.toml");
        let project_path = temp_dir.path().join("dora.toml");
        let manager = ConfigManager::new_with_paths(global_path, project_path);

        manager.set("log.level", "debug", true)?;
        assert_eq!(manager.get("log.level")?, "debug");
        
        Ok(())
    }
}
