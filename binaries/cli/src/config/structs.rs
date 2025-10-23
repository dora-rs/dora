use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use anyhow::{anyhow, Context, Result};

use crate::cli::{UiMode, OutputFormat};

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
    
    /// Auto-refresh intervals for monitoring commands (in seconds)
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

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DaemonConfig {
    /// Daemon connection URL
    pub url: String,
    
    /// Connection timeout in seconds
    pub timeout: u64,
    
    /// Retry attempts
    pub retry_attempts: u32,
    
    /// Retry delay in milliseconds
    pub retry_delay: u64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LoggingConfig {
    /// Log level
    pub level: LogLevel,
    
    /// Log file path (optional)
    pub file: Option<String>,
    
    /// Enable colored output
    pub colored: bool,
    
    /// Include timestamps
    pub include_timestamps: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PerformanceConfig {
    /// Cache size for interface decisions
    pub decision_cache_size: usize,
    
    /// Maximum concurrent operations
    pub max_concurrent_ops: usize,
    
    /// Buffer size for streaming operations
    pub stream_buffer_size: usize,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TelemetryConfig {
    /// Enable usage analytics
    pub enabled: bool,
    
    /// Include performance metrics
    pub include_performance: bool,
    
    /// Include error reporting
    pub include_errors: bool,
    
    /// Telemetry endpoint (optional)
    pub endpoint: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AdvancedConfig {
    /// Enable experimental features
    pub experimental_features: bool,
    
    /// Debug mode settings
    pub debug: DebugConfig,
    
    /// Development mode overrides
    pub development: DevelopmentConfig,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DebugConfig {
    /// Enable debug output
    pub enabled: bool,
    
    /// Debug output verbosity (1-5)
    pub verbosity: u8,
    
    /// Debug categories to enable
    pub categories: Vec<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DevelopmentConfig {
    /// Hot reload configuration files
    pub hot_reload_config: bool,
    
    /// Enable development shortcuts
    pub enable_shortcuts: bool,
    
    /// Mock external dependencies
    pub mock_dependencies: bool,
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

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum HintFrequency {
    Always,
    OncePerSession,
    OncePerCommand,
    Never,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum LogLevel {
    Trace,
    Debug,
    Info,
    Warn,
    Error,
}

// Default implementations
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

impl Default for UiConfig {
    fn default() -> Self {
        Self {
            mode: UiMode::Auto,
            complexity_threshold: 5,
            auto_launch_threshold: 0.7,
            show_hints: true,
            hint_frequency: HintFrequency::OncePerCommand,
            theme: ThemeConfig::default(),
            command_modes: HashMap::new(),
        }
    }
}

impl Default for CommandConfig {
    fn default() -> Self {
        let mut default_flags = HashMap::new();
        default_flags.insert("logs".to_string(), vec!["--follow".to_string()]);
        
        let mut aliases = HashMap::new();
        aliases.insert("list".to_string(), "ps".to_string());
        aliases.insert("status".to_string(), "ps".to_string());
        
        let mut refresh_intervals = HashMap::new();
        refresh_intervals.insert("ps".to_string(), 5);
        refresh_intervals.insert("logs".to_string(), 1);
        refresh_intervals.insert("monitor".to_string(), 2);
        
        Self {
            default_output_format: OutputFormat::Auto,
            refresh_intervals,
            default_flags,
            aliases,
        }
    }
}

impl Default for SystemConfig {
    fn default() -> Self {
        Self {
            daemon: DaemonConfig::default(),
            logging: LoggingConfig::default(),
            performance: PerformanceConfig::default(),
        }
    }
}

impl Default for DaemonConfig {
    fn default() -> Self {
        Self {
            url: "http://localhost:7777".to_string(),
            timeout: 30,
            retry_attempts: 3,
            retry_delay: 1000,
        }
    }
}

impl Default for LoggingConfig {
    fn default() -> Self {
        Self {
            level: LogLevel::Info,
            file: None,
            colored: true,
            include_timestamps: true,
        }
    }
}

impl Default for PerformanceConfig {
    fn default() -> Self {
        Self {
            decision_cache_size: 100,
            max_concurrent_ops: 10,
            stream_buffer_size: 8192,
        }
    }
}

impl Default for TelemetryConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            include_performance: false,
            include_errors: true,
            endpoint: None,
        }
    }
}

impl Default for AdvancedConfig {
    fn default() -> Self {
        Self {
            experimental_features: false,
            debug: DebugConfig::default(),
            development: DevelopmentConfig::default(),
        }
    }
}

impl Default for DebugConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            verbosity: 1,
            categories: vec!["error".to_string(), "warn".to_string()],
        }
    }
}

impl Default for DevelopmentConfig {
    fn default() -> Self {
        Self {
            hot_reload_config: false,
            enable_shortcuts: false,
            mock_dependencies: false,
        }
    }
}

impl Default for ThemeConfig {
    fn default() -> Self {
        Self {
            name: "default".to_string(),
            colors: None,
        }
    }
}

// Validation implementations
impl DoraConfig {
    pub fn validate(&self) -> Result<()> {
        self.ui.validate()
            .context("UI configuration validation failed")?;
        
        self.commands.validate()
            .context("Commands configuration validation failed")?;
        
        self.system.validate()
            .context("System configuration validation failed")?;
        
        self.telemetry.validate()
            .context("Telemetry configuration validation failed")?;
        
        self.advanced.validate()
            .context("Advanced configuration validation failed")?;
        
        Ok(())
    }
}

impl UiConfig {
    pub fn validate(&self) -> Result<()> {
        if self.complexity_threshold > 10 {
            return Err(anyhow!("complexity_threshold must be between 0-10, got {}", self.complexity_threshold));
        }
        
        if !(0.0..=1.0).contains(&self.auto_launch_threshold) {
            return Err(anyhow!("auto_launch_threshold must be between 0.0-1.0, got {}", self.auto_launch_threshold));
        }
        
        // Validate command mode overrides
        for (command, _mode) in &self.command_modes {
            if command.is_empty() {
                return Err(anyhow!("Command name cannot be empty in ui.command_modes"));
            }
        }
        
        self.theme.validate()
            .context("Theme configuration validation failed")?;
        
        Ok(())
    }
}

impl CommandConfig {
    pub fn validate(&self) -> Result<()> {
        // Validate refresh intervals
        for (command, interval) in &self.refresh_intervals {
            if command.is_empty() {
                return Err(anyhow!("Command name cannot be empty in refresh_intervals"));
            }
            if *interval == 0 {
                return Err(anyhow!("Refresh interval must be greater than 0 for command '{}'", command));
            }
            if *interval > 3600 {
                return Err(anyhow!("Refresh interval too large (>1 hour) for command '{}'", command));
            }
        }
        
        // Validate aliases
        for (alias, target) in &self.aliases {
            if alias.is_empty() || target.is_empty() {
                return Err(anyhow!("Alias and target cannot be empty"));
            }
            if alias == target {
                return Err(anyhow!("Alias cannot be the same as target: '{}'", alias));
            }
        }
        
        Ok(())
    }
}

impl SystemConfig {
    pub fn validate(&self) -> Result<()> {
        self.daemon.validate()
            .context("Daemon configuration validation failed")?;
        
        self.logging.validate()
            .context("Logging configuration validation failed")?;
        
        self.performance.validate()
            .context("Performance configuration validation failed")?;
        
        Ok(())
    }
}

impl DaemonConfig {
    pub fn validate(&self) -> Result<()> {
        if self.url.is_empty() {
            return Err(anyhow!("Daemon URL cannot be empty"));
        }
        
        // Basic URL validation
        if !self.url.starts_with("http://") && !self.url.starts_with("https://") {
            return Err(anyhow!("Daemon URL must start with http:// or https://"));
        }
        
        if self.timeout == 0 {
            return Err(anyhow!("Daemon timeout must be greater than 0"));
        }
        
        if self.timeout > 300 {
            return Err(anyhow!("Daemon timeout too large (>5 minutes)"));
        }
        
        if self.retry_attempts > 10 {
            return Err(anyhow!("Too many retry attempts (>10)"));
        }
        
        if self.retry_delay > 60000 {
            return Err(anyhow!("Retry delay too large (>60 seconds)"));
        }
        
        Ok(())
    }
}

impl LoggingConfig {
    pub fn validate(&self) -> Result<()> {
        // File path validation if specified
        if let Some(file_path) = &self.file {
            if file_path.is_empty() {
                return Err(anyhow!("Log file path cannot be empty"));
            }
        }
        
        Ok(())
    }
}

impl PerformanceConfig {
    pub fn validate(&self) -> Result<()> {
        if self.decision_cache_size == 0 {
            return Err(anyhow!("Decision cache size must be greater than 0"));
        }
        
        if self.decision_cache_size > 10000 {
            return Err(anyhow!("Decision cache size too large (>10000)"));
        }
        
        if self.max_concurrent_ops == 0 {
            return Err(anyhow!("Max concurrent operations must be greater than 0"));
        }
        
        if self.max_concurrent_ops > 100 {
            return Err(anyhow!("Too many concurrent operations (>100)"));
        }
        
        if self.stream_buffer_size == 0 {
            return Err(anyhow!("Stream buffer size must be greater than 0"));
        }
        
        Ok(())
    }
}

impl TelemetryConfig {
    pub fn validate(&self) -> Result<()> {
        if let Some(endpoint) = &self.endpoint {
            if endpoint.is_empty() {
                return Err(anyhow!("Telemetry endpoint cannot be empty"));
            }
            if !endpoint.starts_with("http://") && !endpoint.starts_with("https://") {
                return Err(anyhow!("Telemetry endpoint must be a valid HTTP URL"));
            }
        }
        
        Ok(())
    }
}

impl AdvancedConfig {
    pub fn validate(&self) -> Result<()> {
        self.debug.validate()
            .context("Debug configuration validation failed")?;
        
        self.development.validate()
            .context("Development configuration validation failed")?;
        
        Ok(())
    }
}

impl DebugConfig {
    pub fn validate(&self) -> Result<()> {
        if self.verbosity > 5 {
            return Err(anyhow!("Debug verbosity must be between 1-5, got {}", self.verbosity));
        }
        
        if self.verbosity == 0 {
            return Err(anyhow!("Debug verbosity must be at least 1"));
        }
        
        Ok(())
    }
}

impl DevelopmentConfig {
    pub fn validate(&self) -> Result<()> {
        // No specific validation needed for boolean flags
        Ok(())
    }
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

impl ColorConfig {
    pub fn validate(&self) -> Result<()> {
        let colors = [
            ("primary", &self.primary),
            ("secondary", &self.secondary),
            ("success", &self.success),
            ("warning", &self.warning),
            ("error", &self.error),
            ("background", &self.background),
            ("text", &self.text),
        ];
        
        for (name, color) in colors {
            if color.is_empty() {
                return Err(anyhow!("Color '{}' cannot be empty", name));
            }
            
            // Basic color format validation (hex colors)
            if color.starts_with('#') && color.len() != 7 {
                return Err(anyhow!("Invalid hex color format for '{}': '{}'", name, color));
            }
        }
        
        Ok(())
    }
}