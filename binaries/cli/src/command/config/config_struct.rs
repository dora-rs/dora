use eyre::{Context, ContextCompat, Result, bail};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::fs;
use std::net::IpAddr;
use std::path::PathBuf;

/// Global configuration for the Dora CLI
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
#[serde(default, deny_unknown_fields)]
pub struct DoraConfig {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub coordinator: Option<CoordinatorConfig>,
}

/// Coordinator configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct CoordinatorConfig {
    /// IP address of the coordinator
    #[serde(skip_serializing_if = "Option::is_none")]
    pub addr: Option<IpAddr>,

    /// Port number of the coordinator control server
    #[serde(skip_serializing_if = "Option::is_none")]
    pub port: Option<u16>,
}

impl Default for CoordinatorConfig {
    fn default() -> Self {
        Self {
            addr: None,
            port: None,
        }
    }
}

impl DoraConfig {
    /// Load configuration from global and project config files
    /// Priority: project config > global config
    pub fn load() -> Result<Self> {
        let mut config = Self::default();

        // Load global config if it exists
        if let Ok(global_path) = Self::global_config_path() {
            if global_path.exists() {
                let global_config = Self::load_from_file(&global_path).wrap_err_with(|| {
                    format!(
                        "Failed to load global config from {}",
                        global_path.display()
                    )
                })?;
                config = config.merge(global_config);
            }
        }

        // Load project config if it exists (higher priority)
        let project_path = Self::project_config_path();
        if project_path.exists() {
            let project_config = Self::load_from_file(&project_path).wrap_err_with(|| {
                format!(
                    "Failed to load project config from {}",
                    project_path.display()
                )
            })?;
            config = config.merge(project_config);
        }

        Ok(config)
    }

    /// Load configuration from a specific file
    pub fn load_from_file(path: &PathBuf) -> Result<Self> {
        let content = fs::read_to_string(path)
            .wrap_err_with(|| format!("Failed to read config file: {}", path.display()))?;

        toml::from_str(&content)
            .wrap_err_with(|| format!("Failed to parse config file: {}", path.display()))
    }

    /// Get the global config file path (~/.dora/config.toml)
    pub fn global_config_path() -> Result<PathBuf> {
        let home = dirs::home_dir().context("Could not determine home directory")?;
        Ok(home.join(".dora").join("config.toml"))
    }

    /// Get the project config file path (./dora.toml)
    pub fn project_config_path() -> PathBuf {
        PathBuf::from("dora.toml")
    }

    /// Merge another config into this one (other takes priority)
    fn merge(mut self, other: Self) -> Self {
        if let Some(other_coordinator) = other.coordinator {
            let coordinator = self.coordinator.get_or_insert_with(Default::default);
            if other_coordinator.addr.is_some() {
                coordinator.addr = other_coordinator.addr;
            }
            if other_coordinator.port.is_some() {
                coordinator.port = other_coordinator.port;
            }
        }
        self
    }

    /// Get a configuration value by key (for backwards compatibility with config get)
    pub fn get_value(&self, key: &str) -> Result<String> {
        match key {
            "coordinator.addr" => self
                .coordinator
                .as_ref()
                .and_then(|c| c.addr.as_ref())
                .map(|a| a.to_string())
                .ok_or_else(|| eyre::eyre!("Configuration key '{}' not found", key)),
            "coordinator.port" => self
                .coordinator
                .as_ref()
                .and_then(|c| c.port.as_ref())
                .map(|p| p.to_string())
                .ok_or_else(|| eyre::eyre!("Configuration key '{}' not found", key)),
            _ => bail!("Unknown configuration key: {}", key),
        }
    }

    /// Set a configuration value by key
    pub fn set_value(&mut self, key: &str, value: &str) -> Result<()> {
        match key {
            "coordinator.addr" => {
                let addr: IpAddr = value
                    .parse()
                    .wrap_err_with(|| format!("Invalid IP address: {}", value))?;
                self.coordinator.get_or_insert_with(Default::default).addr = Some(addr);
            }
            "coordinator.port" => {
                let port: u16 = value
                    .parse()
                    .wrap_err_with(|| format!("Invalid port number: {}", value))?;
                if port == 0 {
                    bail!("Port number must be greater than 0");
                }
                self.coordinator.get_or_insert_with(Default::default).port = Some(port);
            }
            _ => bail!(
                "Unknown configuration key: {}. Valid keys are: coordinator.addr, coordinator.port",
                key
            ),
        }
        Ok(())
    }

    /// Unset a configuration value by key
    pub fn unset_value(&mut self, key: &str) -> Result<()> {
        match key {
            "coordinator.addr" => {
                if let Some(coordinator) = self.coordinator.as_mut() {
                    coordinator.addr = None;
                } else {
                    bail!("Configuration key '{}' not found", key);
                }
            }
            "coordinator.port" => {
                if let Some(coordinator) = self.coordinator.as_mut() {
                    coordinator.port = None;
                } else {
                    bail!("Configuration key '{}' not found", key);
                }
            }
            _ => bail!("Unknown configuration key: {}", key),
        }
        Ok(())
    }

    /// List all configuration values as key-value pairs
    pub fn list_values(&self) -> Vec<(String, String)> {
        let mut items = Vec::new();

        if let Some(coordinator) = &self.coordinator {
            if let Some(addr) = &coordinator.addr {
                items.push(("coordinator.addr".to_string(), addr.to_string()));
            }
            if let Some(port) = &coordinator.port {
                items.push(("coordinator.port".to_string(), port.to_string()));
            }
        }

        items.sort_by(|a, b| a.0.cmp(&b.0));
        items
    }

    /// Save configuration to a file
    pub fn save_to_file(&self, path: &PathBuf) -> Result<()> {
        // Ensure parent directory exists
        if let Some(parent) = path.parent() {
            fs::create_dir_all(parent).wrap_err_with(|| {
                format!("Failed to create config directory: {}", parent.display())
            })?;
        }

        let toml_string =
            toml::to_string_pretty(self).context("Failed to serialize configuration")?;

        fs::write(path, toml_string)
            .wrap_err_with(|| format!("Failed to write config file: {}", path.display()))?;

        Ok(())
    }

    /// Save to global config file
    pub fn save_global(&self) -> Result<()> {
        let path = Self::global_config_path()?;
        self.save_to_file(&path)
    }

    /// Save to project config file
    pub fn save_project(&self) -> Result<()> {
        let path = Self::project_config_path();
        self.save_to_file(&path)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use tempfile::TempDir;

    #[test]
    fn test_config_validation() -> Result<()> {
        let mut config = DoraConfig::default();

        // Valid IP address
        config.set_value("coordinator.addr", "192.168.1.100")?;
        assert_eq!(config.get_value("coordinator.addr")?, "192.168.1.100");

        // Invalid IP address should fail
        assert!(config.set_value("coordinator.addr", "not-an-ip").is_err());

        // Valid port
        config.set_value("coordinator.port", "8080")?;
        assert_eq!(config.get_value("coordinator.port")?, "8080");

        // Invalid port should fail
        assert!(config.set_value("coordinator.port", "not-a-port").is_err());
        assert!(config.set_value("coordinator.port", "0").is_err());
        assert!(config.set_value("coordinator.port", "70000").is_err());

        // Unknown key should fail
        assert!(config.set_value("unknown.key", "value").is_err());

        Ok(())
    }

    #[test]
    fn test_config_merge() -> Result<()> {
        let mut global = DoraConfig::default();
        global.set_value("coordinator.addr", "192.168.1.100")?;
        global.set_value("coordinator.port", "8080")?;

        let mut project = DoraConfig::default();
        project.set_value("coordinator.addr", "127.0.0.1")?;

        let merged = global.merge(project);

        // Project config should override global
        assert_eq!(merged.get_value("coordinator.addr")?, "127.0.0.1");
        // Port should remain from global
        assert_eq!(merged.get_value("coordinator.port")?, "8080");

        Ok(())
    }

    #[test]
    fn test_save_and_load() -> Result<()> {
        let temp_dir = TempDir::new()?;
        let config_path = temp_dir.path().join("config.toml");

        let mut config = DoraConfig::default();
        config.set_value("coordinator.addr", "192.168.1.100")?;
        config.set_value("coordinator.port", "8080")?;

        config.save_to_file(&config_path)?;

        let loaded = DoraConfig::load_from_file(&config_path)?;
        assert_eq!(loaded.get_value("coordinator.addr")?, "192.168.1.100");
        assert_eq!(loaded.get_value("coordinator.port")?, "8080");

        Ok(())
    }

    #[test]
    fn test_unknown_fields_rejected() {
        let invalid_toml = r#"
[coordinator]
addr = "127.0.0.1"
typo_port = 8080
"#;

        let result: Result<DoraConfig, _> = toml::from_str(invalid_toml);
        assert!(result.is_err());
        assert!(result.unwrap_err().to_string().contains("unknown field"));
    }
}
