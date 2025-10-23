use anyhow::{anyhow, Context, Result};
use std::process::Command;
use colored::Colorize;

use crate::config::{ConfigManager, ConfigPaths};
use crate::cli::{context::ExecutionContext, commands::ConfigSubcommand};

pub struct ConfigCommandHandler;

impl ConfigCommandHandler {
    pub fn execute(subcommand: &ConfigSubcommand, _context: &ExecutionContext) -> Result<()> {
        let mut config_manager = ConfigManager::load()?;
        
        match subcommand {
            ConfigSubcommand::Get { key } => {
                Self::handle_get(&config_manager, key)
            },
            
            ConfigSubcommand::Set { key, value } => {
                Self::handle_set(&mut config_manager, key, value)
            },
            
            ConfigSubcommand::List { filter, changed_only } => {
                Self::handle_list(&config_manager, filter.as_deref(), *changed_only)
            },
            
            ConfigSubcommand::Reset { key, confirm } => {
                Self::handle_reset(&mut config_manager, key.as_deref(), *confirm)
            },
            
            ConfigSubcommand::Paths => {
                Self::handle_paths(&config_manager)
            },
            
            ConfigSubcommand::Validate => {
                Self::handle_validate(&config_manager)
            },
            
            ConfigSubcommand::Edit => {
                Self::handle_edit(&config_manager)
            },
        }
    }
    
    fn handle_get(config_manager: &ConfigManager, key: &str) -> Result<()> {
        match config_manager.get(key) {
            Ok(value) => {
                if value.is_string() {
                    // Print strings without quotes for better UX
                    println!("{}", value.as_str().unwrap());
                } else {
                    println!("{}", serde_json::to_string_pretty(&value)?);
                }
                Ok(())
            },
            Err(e) => {
                eprintln!("{} {}", "Error:".red().bold(), e);
                std::process::exit(1);
            }
        }
    }
    
    fn handle_set(config_manager: &mut ConfigManager, key: &str, value: &str) -> Result<()> {
        let parsed_value = Self::parse_config_value(key, value)?;
        
        match config_manager.set(key, parsed_value) {
            Ok(()) => {
                config_manager.save()?;
                println!("{} Configuration updated: {} = {}", 
                    "✅".green(), 
                    key.cyan().bold(), 
                    value.yellow()
                );
                Ok(())
            },
            Err(e) => {
                eprintln!("{} Failed to set configuration: {}", "Error:".red().bold(), e);
                std::process::exit(1);
            }
        }
    }
    
    fn handle_list(config_manager: &ConfigManager, filter: Option<&str>, changed_only: bool) -> Result<()> {
        let values = config_manager.list(filter, changed_only)?;
        
        if values.is_empty() {
            if let Some(filter_str) = filter {
                println!("No configuration values found matching filter: {}", filter_str.cyan());
            } else if changed_only {
                println!("No configuration values have been changed from defaults");
            } else {
                println!("No configuration values found");
            }
            return Ok(());
        }
        
        // Print header
        if let Some(filter_str) = filter {
            println!("{} Configuration values matching '{}'{}", 
                "📋".cyan(), 
                filter_str.cyan().bold(),
                if changed_only { " (changed only)" } else { "" }
            );
        } else {
            println!("{} Configuration values{}", 
                "📋".cyan(),
                if changed_only { " (changed only)" } else { "" }
            );
        }
        println!();
        
        // Find the longest key for alignment
        let max_key_len = values.iter()
            .map(|(key, _)| key.len())
            .max()
            .unwrap_or(0);
        
        // Print values
        for (key, value) in values {
            let formatted_value = if value.is_string() {
                value.as_str().unwrap().to_string()
            } else {
                serde_json::to_string(&value)?
            };
            
            println!("{:width$} = {}", 
                key.cyan().bold(), 
                formatted_value.yellow(),
                width = max_key_len
            );
        }
        
        Ok(())
    }
    
    fn handle_reset(config_manager: &mut ConfigManager, key: Option<&str>, confirm: bool) -> Result<()> {
        let message = match key {
            Some(key_path) => format!("Reset configuration key '{}' to default?", key_path),
            None => "Reset entire configuration to defaults?".to_string(),
        };
        
        if !confirm {
            print!("{} {} [y/N]: ", "⚠️".yellow(), message);
            use std::io::{self, Write};
            io::stdout().flush()?;
            
            let mut input = String::new();
            io::stdin().read_line(&mut input)?;
            
            if !matches!(input.trim().to_lowercase().as_str(), "y" | "yes") {
                println!("Reset cancelled");
                return Ok(());
            }
        }
        
        match config_manager.reset(key) {
            Ok(()) => {
                config_manager.save()?;
                match key {
                    Some(key_path) => {
                        println!("{} Configuration key '{}' reset to default", 
                            "✅".green(), 
                            key_path.cyan().bold()
                        );
                    },
                    None => {
                        println!("{} Entire configuration reset to defaults", "✅".green());
                    }
                }
                Ok(())
            },
            Err(e) => {
                eprintln!("{} Failed to reset configuration: {}", "Error:".red().bold(), e);
                std::process::exit(1);
            }
        }
    }
    
    fn handle_paths(config_manager: &ConfigManager) -> Result<()> {
        let paths = config_manager.get_config_paths();
        
        println!("{} Configuration file locations:", "📁".cyan());
        println!();
        
        // User config
        println!("  {} {}", 
            "User:".bold(), 
            paths.user.display().to_string().yellow()
        );
        if paths.user.exists() {
            println!("    {} exists", "✅".green());
        } else {
            println!("    {} not found", "❌".red());
        }
        
        // Global config
        if let Some(global_path) = &paths.global {
            println!("  {} {}", 
                "Global:".bold(), 
                global_path.display().to_string().yellow()
            );
            if global_path.exists() {
                println!("    {} exists", "✅".green());
            } else {
                println!("    {} not found", "❌".red());
            }
        }
        
        // Project config
        if let Some(project_path) = &paths.project {
            println!("  {} {}", 
                "Project:".bold(), 
                project_path.display().to_string().yellow()
            );
            if project_path.exists() {
                println!("    {} exists", "✅".green());
            } else {
                println!("    {} not found", "❌".red());
            }
        } else {
            println!("  {} {}", 
                "Project:".bold(), 
                "no .dora/config.toml found".dimmed()
            );
        }
        
        Ok(())
    }
    
    fn handle_validate(config_manager: &ConfigManager) -> Result<()> {
        match config_manager.config().validate() {
            Ok(()) => {
                println!("{} Configuration is valid", "✅".green());
                Ok(())
            },
            Err(e) => {
                eprintln!("{} Configuration validation failed:", "❌".red().bold());
                eprintln!("  {}", e.to_string().yellow());
                std::process::exit(1);
            }
        }
    }
    
    fn handle_edit(config_manager: &ConfigManager) -> Result<()> {
        let config_path = &config_manager.get_config_paths().user;
        
        // Ensure config file exists
        if !config_path.exists() {
            println!("Creating default configuration file...");
            config_manager.save()?;
        }
        
        // Determine editor
        let editor = std::env::var("EDITOR")
            .or_else(|_| std::env::var("VISUAL"))
            .unwrap_or_else(|_| {
                // Default editors by platform
                #[cfg(target_os = "windows")]
                return "notepad".to_string();
                #[cfg(target_os = "macos")]
                return "nano".to_string();
                #[cfg(not(any(target_os = "windows", target_os = "macos")))]
                return "nano".to_string();
            });
        
        println!("Opening configuration file with {}...", editor.cyan());
        
        // Launch editor
        let status = Command::new(&editor)
            .arg(config_path)
            .status()
            .with_context(|| format!("Failed to launch editor: {}", editor))?;
        
        if !status.success() {
            return Err(anyhow!("Editor exited with error status"));
        }
        
        // Validate configuration after editing
        println!("Validating configuration...");
        match ConfigManager::load() {
            Ok(_) => {
                println!("{} Configuration saved and validated successfully", "✅".green());
            },
            Err(e) => {
                eprintln!("{} Configuration validation failed after editing:", "❌".red().bold());
                eprintln!("  {}", e.to_string().yellow());
                eprintln!("Please fix the configuration file and try again.");
                std::process::exit(1);
            }
        }
        
        Ok(())
    }
    
    fn parse_config_value(key: &str, value: &str) -> Result<serde_json::Value> {
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
                let mode_value = match value.to_lowercase().as_str() {
                    "auto" => "Auto",
                    "cli" => "Cli",
                    "tui" => "Tui",
                    "minimal" => "Minimal",
                    _ => return Err(anyhow!("Invalid mode '{}'. Valid options: auto, cli, tui, minimal", value)),
                };
                Ok(serde_json::Value::String(mode_value.to_string()))
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
                Ok(serde_json::Value::String(level_value.to_string()))
            },
            
            k if k.contains("format") => {
                let format_value = match value.to_lowercase().as_str() {
                    "auto" => "Auto",
                    "table" => "Table",
                    "json" => "Json",
                    "yaml" => "Yaml",
                    "minimal" => "Minimal",
                    _ => return Err(anyhow!("Invalid output format '{}'. Valid options: auto, table, json, yaml, minimal", value)),
                };
                Ok(serde_json::Value::String(format_value.to_string()))
            },
            
            k if k.contains("frequency") => {
                let freq_value = match value.to_lowercase().as_str() {
                    "always" => "Always",
                    "oncepersession" | "once-per-session" => "OncePerSession",
                    "oncepercommand" | "once-per-command" => "OncePerCommand",
                    "never" => "Never",
                    _ => return Err(anyhow!("Invalid hint frequency '{}'. Valid options: always, once-per-session, once-per-command, never", value)),
                };
                Ok(serde_json::Value::String(freq_value.to_string()))
            },
            
            k if matches!(value, "true" | "false") => {
                let bool_val: bool = value.parse()
                    .with_context(|| format!("'{}' must be true or false", k))?;
                Ok(serde_json::Value::Bool(bool_val))
            },
            
            k if k.ends_with("timeout") || k.ends_with("interval") || k.ends_with("delay") => {
                let num: u64 = value.parse()
                    .with_context(|| format!("'{}' must be a positive number", k))?;
                Ok(serde_json::Value::Number(num.into()))
            },
            
            k if k.ends_with("size") || k.ends_with("attempts") => {
                let num: u32 = value.parse()
                    .with_context(|| format!("'{}' must be a positive number", k))?;
                Ok(serde_json::Value::Number(num.into()))
            },
            
            _ => {
                // Try parsing as JSON first, fall back to string
                serde_json::from_str(value)
                    .or_else(|_| Ok(serde_json::Value::String(value.to_string())))
            }
        }
    }
}