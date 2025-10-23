use crate::cli::context::ExecutionContext;
use crate::config::preferences::*;
use crate::config::behavioral_learning::BehavioralLearningEngine;
use crate::tui::app::{DoraApp, ViewType};
use clap::{Args, Subcommand, ValueEnum};
use eyre::{Context as EyreContext, Result};
use std::collections::HashMap;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};

/// Preference management commands
#[derive(Debug, Args)]
pub struct PreferencesCommand {
    #[clap(subcommand)]
    pub action: PreferencesAction,
}

/// Available preference actions
#[derive(Debug, Subcommand)]
pub enum PreferencesAction {
    /// Show current preferences
    Show {
        /// Filter by preference category
        #[clap(long, value_enum)]
        category: Option<PreferenceCategory>,
        
        /// Show learned behavioral patterns
        #[clap(long)]
        patterns: bool,
        
        /// Show in JSON format
        #[clap(long)]
        json: bool,
    },
    
    /// Set a preference value
    Set {
        /// Preference key (dot notation, e.g., interface.default_ui_mode)
        key: String,
        
        /// Preference value
        value: String,
        
        /// Apply to specific environment
        #[clap(long)]
        environment: Option<String>,
        
        /// Show the updated preference after setting
        #[clap(long)]
        show: bool,
    },
    
    /// Get a preference value
    Get {
        /// Preference key (dot notation)
        key: String,
        
        /// Get for specific environment
        #[clap(long)]
        environment: Option<String>,
    },
    
    /// Reset preferences to defaults
    Reset {
        /// Specific category to reset
        #[clap(value_enum)]
        category: Option<PreferenceCategory>,
        
        /// Include behavioral learning data
        #[clap(long)]
        include_behavior: bool,
        
        /// Confirm reset without prompting
        #[clap(long)]
        confirm: bool,
    },
    
    /// Export preferences to file
    Export {
        /// Output file path
        file: PathBuf,
        
        /// Include behavioral data
        #[clap(long)]
        include_behavior: bool,
        
        /// Output format
        #[clap(long, value_enum, default_value = "toml")]
        format: ExportFormat,
    },
    
    /// Import preferences from file
    Import {
        /// Input file path
        file: PathBuf,
        
        /// Merge with existing preferences
        #[clap(long)]
        merge: bool,
        
        /// Dry run - show what would be imported
        #[clap(long)]
        dry_run: bool,
    },
    
    /// Configure behavioral learning
    Learning {
        /// Enable or disable learning
        #[clap(long)]
        enable: Option<bool>,
        
        /// Reset learning data
        #[clap(long)]
        reset: bool,
        
        /// Show learning statistics
        #[clap(long)]
        stats: bool,
        
        /// Adjust learning weights
        #[clap(long)]
        adjust_weights: bool,
    },
    
    /// Launch interactive preference editor
    Edit,
    
    /// Validate preference configuration
    Validate {
        /// File to validate (defaults to current preferences)
        #[clap(long)]
        file: Option<PathBuf>,
        
        /// Show detailed validation report
        #[clap(long)]
        detailed: bool,
    },
}

/// Preference categories for filtering
#[derive(Debug, Clone, ValueEnum)]
pub enum PreferenceCategory {
    Interface,
    Commands,
    Behavior,
    Environments,
}

/// Export formats
#[derive(Debug, Clone, ValueEnum)]
pub enum ExportFormat {
    Toml,
    Json,
    Yaml,
}

impl PreferencesCommand {
    pub async fn execute(&self, context: &ExecutionContext) -> Result<()> {
        match &self.action {
            PreferencesAction::Show { category, patterns, json } => {
                self.show_preferences(category.as_ref(), *patterns, *json).await?;
            },
            
            PreferencesAction::Set { key, value, environment, show } => {
                self.set_preference(key, value, environment.as_deref(), *show).await?;
            },
            
            PreferencesAction::Get { key, environment } => {
                self.get_preference(key, environment.as_deref()).await?;
            },
            
            PreferencesAction::Reset { category, include_behavior, confirm } => {
                self.reset_preferences(category.as_ref(), *include_behavior, *confirm).await?;
            },
            
            PreferencesAction::Export { file, include_behavior, format } => {
                self.export_preferences(file, *include_behavior, format).await?;
            },
            
            PreferencesAction::Import { file, merge, dry_run } => {
                self.import_preferences(file, *merge, *dry_run).await?;
            },
            
            PreferencesAction::Learning { enable, reset, stats, adjust_weights } => {
                self.manage_learning(*enable, *reset, *stats, *adjust_weights).await?;
            },
            
            PreferencesAction::Edit => {
                self.launch_preference_editor().await?;
            },
            
            PreferencesAction::Validate { file, detailed } => {
                self.validate_preferences(file.as_ref(), *detailed).await?;
            },
        }
        
        Ok(())
    }
    
    async fn show_preferences(
        &self,
        category: Option<&PreferenceCategory>,
        show_patterns: bool,
        json_format: bool,
    ) -> Result<()> {
        let preferences = UserPreferences::load_or_create()
            .context("Failed to load preferences")?;
        
        if json_format {
            let json_output = match category {
                Some(PreferenceCategory::Interface) => serde_json::to_string_pretty(&preferences.interface)?,
                Some(PreferenceCategory::Commands) => serde_json::to_string_pretty(&preferences.commands)?,
                Some(PreferenceCategory::Behavior) => serde_json::to_string_pretty(&preferences.behavior)?,
                Some(PreferenceCategory::Environments) => serde_json::to_string_pretty(&preferences.environments)?,
                None => serde_json::to_string_pretty(&preferences)?,
            };
            println!("{}", json_output);
            return Ok(());
        }
        
        match category {
            Some(PreferenceCategory::Interface) => {
                self.display_interface_preferences(&preferences.interface);
            },
            Some(PreferenceCategory::Commands) => {
                self.display_command_preferences(&preferences.commands);
            },
            Some(PreferenceCategory::Behavior) => {
                self.display_behavior_preferences(&preferences.behavior, show_patterns);
            },
            Some(PreferenceCategory::Environments) => {
                self.display_environment_preferences(&preferences.environments);
            },
            None => {
                self.display_all_preferences(&preferences, show_patterns);
            },
        }
        
        Ok(())
    }
    
    async fn set_preference(
        &self,
        key: &str,
        value: &str,
        environment: Option<&str>,
        show_result: bool,
    ) -> Result<()> {
        let mut preferences = UserPreferences::load_or_create()
            .context("Failed to load preferences")?;
        
        self.apply_preference_update(&mut preferences, key, value, environment)
            .context("Failed to update preference")?;
        
        preferences.save()
            .context("Failed to save preferences")?;
        
        println!("✅ Preference '{}' set to '{}'", key, value);
        
        if let Some(env) = environment {
            println!("   Applied to environment: {}", env);
        }
        
        if show_result {
            self.show_preference_value(&preferences, key, environment);
        }
        
        Ok(())
    }
    
    async fn get_preference(&self, key: &str, environment: Option<&str>) -> Result<()> {
        let preferences = UserPreferences::load_or_create()
            .context("Failed to load preferences")?;
        
        self.show_preference_value(&preferences, key, environment);
        Ok(())
    }
    
    async fn reset_preferences(
        &self,
        category: Option<&PreferenceCategory>,
        include_behavior: bool,
        confirm: bool,
    ) -> Result<()> {
        if !confirm {
            let category_str = category.map(|c| format!("{:?}", c)).unwrap_or_else(|| "all preferences".to_string());
            let behavior_str = if include_behavior { " (including behavioral data)" } else { "" };
            
            print!("Are you sure you want to reset {}{}? [y/N]: ", category_str, behavior_str);
            std::io::Write::flush(&mut std::io::stdout())?;
            
            let mut input = String::new();
            std::io::stdin().read_line(&mut input)?;
            
            if !input.trim().to_lowercase().starts_with('y') {
                println!("Reset cancelled.");
                return Ok(());
            }
        }
        
        let mut preferences = UserPreferences::load_or_create()
            .context("Failed to load preferences")?;
        
        match category {
            Some(PreferenceCategory::Interface) => {
                preferences.interface = InterfacePreferences::default();
                println!("✅ Interface preferences reset to defaults");
            },
            Some(PreferenceCategory::Commands) => {
                preferences.commands = CommandPreferences::default();
                println!("✅ Command preferences reset to defaults");
            },
            Some(PreferenceCategory::Behavior) => {
                preferences.behavior = BehaviorPreferences::default();
                println!("✅ Behavioral preferences reset to defaults");
            },
            Some(PreferenceCategory::Environments) => {
                preferences.environments.clear();
                println!("✅ Environment preferences cleared");
            },
            None => {
                if include_behavior {
                    preferences = UserPreferences::default();
                    println!("✅ All preferences reset to defaults (including behavioral data)");
                } else {
                    preferences.interface = InterfacePreferences::default();
                    preferences.commands = CommandPreferences::default();
                    preferences.environments.clear();
                    println!("✅ All preferences reset to defaults (behavioral data preserved)");
                }
            },
        }
        
        preferences.save()
            .context("Failed to save reset preferences")?;
        
        Ok(())
    }
    
    async fn export_preferences(
        &self,
        file: &PathBuf,
        include_behavior: bool,
        format: &ExportFormat,
    ) -> Result<()> {
        let mut preferences = UserPreferences::load_or_create()
            .context("Failed to load preferences")?;
        
        if !include_behavior {
            preferences.behavior = BehaviorPreferences::default();
        }
        
        let content = match format {
            ExportFormat::Toml => toml::to_string_pretty(&preferences)?,
            ExportFormat::Json => serde_json::to_string_pretty(&preferences)?,
            ExportFormat::Yaml => serde_yaml::to_string(&preferences)?,
        };
        
        if let Some(parent) = file.parent() {
            std::fs::create_dir_all(parent)
                .context("Failed to create export directory")?;
        }
        
        std::fs::write(file, content)
            .context("Failed to write export file")?;
        
        println!("✅ Preferences exported to: {}", file.display());
        if !include_behavior {
            println!("   (Behavioral data excluded)");
        }
        
        Ok(())
    }
    
    async fn import_preferences(
        &self,
        file: &PathBuf,
        merge: bool,
        dry_run: bool,
    ) -> Result<()> {
        if !file.exists() {
            return Err(eyre::eyre!("Import file does not exist: {}", file.display()));
        }
        
        let content = std::fs::read_to_string(file)
            .context("Failed to read import file")?;
        
        let imported_prefs: UserPreferences = if file.extension().and_then(|s| s.to_str()) == Some("json") {
            serde_json::from_str(&content)?
        } else if file.extension().and_then(|s| s.to_str()) == Some("yaml") || 
                 file.extension().and_then(|s| s.to_str()) == Some("yml") {
            serde_yaml::from_str(&content)?
        } else {
            toml::from_str(&content)?
        };
        
        imported_prefs.validate()
            .context("Imported preferences are invalid")?;
        
        if dry_run {
            println!("🔍 Dry run - would import the following preferences:");
            self.display_all_preferences(&imported_prefs, false);
            return Ok(());
        }
        
        let final_prefs = if merge {
            let mut current_prefs = UserPreferences::load_or_create()
                .context("Failed to load current preferences")?;
            
            // Merge preferences (imported takes precedence)
            self.merge_preferences(&mut current_prefs, imported_prefs);
            current_prefs
        } else {
            imported_prefs
        };
        
        final_prefs.save()
            .context("Failed to save imported preferences")?;
        
        println!("✅ Preferences imported from: {}", file.display());
        if merge {
            println!("   (Merged with existing preferences)");
        } else {
            println!("   (Replaced existing preferences)");
        }
        
        Ok(())
    }
    
    async fn manage_learning(
        &self,
        enable: Option<bool>,
        reset_data: bool,
        show_stats: bool,
        adjust_weights: bool,
    ) -> Result<()> {
        let mut preferences = UserPreferences::load_or_create()
            .context("Failed to load preferences")?;
        
        let mut changes_made = false;
        
        if let Some(enabled) = enable {
            preferences.behavior.learning_enabled = enabled;
            changes_made = true;
            println!("✅ Behavioral learning {}", if enabled { "enabled" } else { "disabled" });
        }
        
        if reset_data {
            preferences.behavior.interface_choices.clear();
            preferences.behavior.command_usage.clear();
            preferences.behavior.action_patterns.clear();
            changes_made = true;
            println!("✅ Behavioral learning data reset");
        }
        
        if show_stats {
            self.display_learning_statistics(&preferences.behavior);
        }
        
        if adjust_weights {
            println!("Current adaptation weights:");
            println!("  Behavioral: {:.2}", preferences.behavior.adaptation_weights.behavioral_weight);
            println!("  Context: {:.2}", preferences.behavior.adaptation_weights.context_weight);
            println!("  Complexity: {:.2}", preferences.behavior.adaptation_weights.complexity_weight);
            println!("  Explicit: {:.2}", preferences.behavior.adaptation_weights.explicit_weight);
            
            // Interactive weight adjustment would go here
            // For now, just show current values
        }
        
        if changes_made {
            preferences.save()
                .context("Failed to save preference changes")?;
        }
        
        Ok(())
    }
    
    async fn launch_preference_editor(&self) -> Result<()> {
        println!("🚀 Launching interactive preference editor...");
        
        // This would launch the TUI preference editor
        // For now, show a message about the feature
        println!("Interactive preference editor will be available in a future update.");
        println!("Use 'dora preferences show' to view current preferences.");
        println!("Use 'dora preferences set <key> <value>' to modify preferences.");
        
        Ok(())
    }
    
    async fn validate_preferences(&self, file: Option<&PathBuf>, detailed: bool) -> Result<()> {
        let preferences = if let Some(file_path) = file {
            let content = std::fs::read_to_string(file_path)
                .context("Failed to read preference file")?;
            
            toml::from_str::<UserPreferences>(&content)
                .context("Failed to parse preference file")?
        } else {
            UserPreferences::load_or_create()
                .context("Failed to load current preferences")?
        };
        
        match preferences.validate() {
            Ok(()) => {
                println!("✅ Preferences are valid");
                
                if detailed {
                    println!("\nValidation details:");
                    println!("  Schema version: {}", preferences.metadata.schema_version);
                    println!("  Total interface choices: {}", preferences.behavior.interface_choices.len());
                    println!("  Command usage entries: {}", preferences.behavior.command_usage.len());
                    println!("  Action patterns: {}", preferences.behavior.action_patterns.len());
                    println!("  Environment overrides: {}", preferences.environments.len());
                }
            },
            Err(e) => {
                println!("❌ Preferences validation failed: {}", e);
                return Err(e.into());
            }
        }
        
        Ok(())
    }
    
    // Helper methods for displaying preferences
    fn display_all_preferences(&self, prefs: &UserPreferences, show_patterns: bool) {
        println!("📋 Dora Preferences");
        println!("==================");
        
        self.display_interface_preferences(&prefs.interface);
        println!();
        self.display_command_preferences(&prefs.commands);
        println!();
        self.display_behavior_preferences(&prefs.behavior, show_patterns);
        
        if !prefs.environments.is_empty() {
            println!();
            self.display_environment_preferences(&prefs.environments);
        }
        
        println!();
        println!("📊 Metadata");
        println!("  Version: {}", prefs.metadata.version);
        println!("  Created: {}", prefs.metadata.created_at.format("%Y-%m-%d %H:%M:%S UTC"));
        println!("  Updated: {}", prefs.metadata.updated_at.format("%Y-%m-%d %H:%M:%S UTC"));
    }
    
    fn display_interface_preferences(&self, prefs: &InterfacePreferences) {
        println!("🎨 Interface Preferences");
        println!("  Default UI Mode: {:?}", prefs.default_ui_mode);
        println!("  Complexity Thresholds:");
        println!("    Suggestion: {}", prefs.complexity_thresholds.suggestion_threshold);
        println!("    Auto-launch: {}", prefs.complexity_thresholds.auto_launch_threshold);
        println!("    Hints: {}", prefs.complexity_thresholds.hint_threshold);
        println!("  Auto-launch:");
        println!("    Confidence threshold: {:.1}", prefs.auto_launch.confidence_threshold);
        println!("    Delay: {}ms", prefs.auto_launch.delay_ms);
        println!("  Hints: {} ({:?})", 
                if prefs.hints.show_hints { "enabled" } else { "disabled" },
                prefs.hints.hint_frequency);
        
        if !prefs.cli.command_aliases.is_empty() {
            println!("  CLI Aliases:");
            for (alias, command) in &prefs.cli.command_aliases {
                println!("    {} → {}", alias, command);
            }
        }
        
        println!("  TUI Settings:");
        println!("    Theme: {}", prefs.tui.theme);
        println!("    Auto-refresh: {}s", prefs.tui.auto_refresh_interval.as_secs());
        println!("    Mouse support: {}", prefs.tui.mouse_support);
    }
    
    fn display_command_preferences(&self, prefs: &CommandPreferences) {
        println!("⚡ Command Preferences");
        
        if !prefs.command_ui_modes.is_empty() {
            println!("  Command UI Overrides:");
            for (command, mode) in &prefs.command_ui_modes {
                println!("    {} → {:?}", command, mode);
            }
        }
        
        if !prefs.aliases.is_empty() {
            println!("  Command Aliases:");
            for (alias, command) in &prefs.aliases {
                println!("    {} → {}", alias, command);
            }
        }
        
        println!("  Completion:");
        println!("    Enabled: {}", prefs.completion.enabled);
        println!("    Show descriptions: {}", prefs.completion.show_descriptions);
        println!("    Max suggestions: {}", prefs.completion.max_suggestions);
        println!("    Context-aware: {}", prefs.completion.context_aware);
    }
    
    fn display_behavior_preferences(&self, prefs: &BehaviorPreferences, show_patterns: bool) {
        println!("🧠 Behavioral Learning");
        println!("  Learning enabled: {}", prefs.learning_enabled);
        println!("  Interface choices: {}", prefs.interface_choices.len());
        println!("  Command usage entries: {}", prefs.command_usage.len());
        println!("  Action patterns: {}", prefs.action_patterns.len());
        
        if !prefs.command_usage.is_empty() {
            println!("  Most used commands:");
            let mut sorted_commands: Vec<_> = prefs.command_usage.iter().collect();
            sorted_commands.sort_by_key(|(_, stats)| std::cmp::Reverse(stats.total_uses));
            
            for (command, stats) in sorted_commands.iter().take(5) {
                let primary_interface = stats.interface_distribution.iter()
                    .max_by_key(|(_, count)| *count)
                    .map(|(interface, _)| format!("{:?}", interface))
                    .unwrap_or_else(|| "Unknown".to_string());
                
                println!("    {} ({} uses, prefers {})", 
                        command, stats.total_uses, primary_interface);
            }
        }
        
        if show_patterns && !prefs.action_patterns.is_empty() {
            println!("  Learned patterns:");
            for (pattern_key, pattern) in prefs.action_patterns.iter().take(10) {
                println!("    {} (frequency: {:.2}, last seen: {})",
                        pattern_key,
                        pattern.frequency,
                        pattern.last_seen.format("%Y-%m-%d"));
            }
        }
    }
    
    fn display_environment_preferences(&self, envs: &HashMap<String, EnvironmentPreferences>) {
        println!("🌍 Environment Preferences");
        for (env_name, env_prefs) in envs {
            println!("  {}:", env_name);
            if let Some(ui_mode) = &env_prefs.ui_mode_override {
                println!("    UI mode: {:?}", ui_mode);
            }
            if let Some(threshold) = env_prefs.complexity_threshold_override {
                println!("    Complexity threshold: {}", threshold);
            }
            println!("    Learning disabled: {}", env_prefs.disable_learning);
        }
    }
    
    fn display_learning_statistics(&self, behavior: &BehaviorPreferences) {
        println!("📈 Learning Statistics");
        println!("  Total interface choices recorded: {}", behavior.interface_choices.len());
        println!("  Commands with usage data: {}", behavior.command_usage.len());
        println!("  Active patterns: {}", behavior.action_patterns.len());
        println!("  Learning enabled: {}", behavior.learning_enabled);
        
        println!("  Adaptation weights:");
        println!("    Behavioral: {:.2}", behavior.adaptation_weights.behavioral_weight);
        println!("    Context: {:.2}", behavior.adaptation_weights.context_weight);
        println!("    Complexity: {:.2}", behavior.adaptation_weights.complexity_weight);
        println!("    Explicit: {:.2}", behavior.adaptation_weights.explicit_weight);
        
        if !behavior.interface_choices.is_empty() {
            let recent_choices = behavior.interface_choices.iter()
                .rev()
                .take(10)
                .collect::<Vec<_>>();
            
            println!("  Recent interface choices:");
            for choice in recent_choices {
                println!("    {} → {:?} ({})",
                        choice.command,
                        choice.chosen_interface,
                        choice.timestamp.format("%m-%d %H:%M"));
            }
        }
    }
    
    fn apply_preference_update(
        &self,
        prefs: &mut UserPreferences,
        key: &str,
        value: &str,
        environment: Option<&str>,
    ) -> Result<()> {
        // Parse the key path (e.g., "interface.default_ui_mode")
        let parts: Vec<&str> = key.split('.').collect();
        
        match parts.as_slice() {
            ["interface", "default_ui_mode"] => {
                prefs.interface.default_ui_mode = self.parse_ui_mode(value)?;
            },
            ["interface", "complexity_thresholds", "suggestion_threshold"] => {
                prefs.interface.complexity_thresholds.suggestion_threshold = value.parse()?;
            },
            ["interface", "auto_launch", "confidence_threshold"] => {
                prefs.interface.auto_launch.confidence_threshold = value.parse()?;
            },
            ["interface", "hints", "show_hints"] => {
                prefs.interface.hints.show_hints = value.parse()?;
            },
            ["behavior", "learning_enabled"] => {
                prefs.behavior.learning_enabled = value.parse()?;
            },
            _ => {
                return Err(eyre::eyre!("Unknown preference key: {}", key));
            }
        }
        
        prefs.metadata.updated_at = chrono::Utc::now();
        Ok(())
    }
    
    fn parse_ui_mode(&self, value: &str) -> Result<UiMode> {
        match value.to_lowercase().as_str() {
            "auto" => Ok(UiMode::Auto),
            "cli" => Ok(UiMode::Cli),
            "tui" => Ok(UiMode::Tui),
            "minimal" => Ok(UiMode::Minimal),
            _ => Err(eyre::eyre!("Invalid UI mode: {}. Valid options: auto, cli, tui, minimal", value)),
        }
    }
    
    fn show_preference_value(&self, prefs: &UserPreferences, key: &str, environment: Option<&str>) {
        let parts: Vec<&str> = key.split('.').collect();
        
        let value_str = match parts.as_slice() {
            ["interface", "default_ui_mode"] => format!("{:?}", prefs.interface.default_ui_mode),
            ["interface", "complexity_thresholds", "suggestion_threshold"] => {
                prefs.interface.complexity_thresholds.suggestion_threshold.to_string()
            },
            ["behavior", "learning_enabled"] => prefs.behavior.learning_enabled.to_string(),
            _ => "Unknown key".to_string(),
        };
        
        println!("{} = {}", key, value_str);
        
        if let Some(env) = environment {
            println!("  (for environment: {})", env);
        }
    }
    
    fn merge_preferences(&self, current: &mut UserPreferences, imported: UserPreferences) {
        // Merge interface preferences
        current.interface = imported.interface;
        
        // Merge command preferences
        current.commands.command_ui_modes.extend(imported.commands.command_ui_modes);
        current.commands.aliases.extend(imported.commands.aliases);
        current.commands.default_flags.extend(imported.commands.default_flags);
        current.commands.completion = imported.commands.completion;
        
        // Optionally merge behavioral data (be careful not to lose existing learning)
        if !imported.behavior.interface_choices.is_empty() {
            current.behavior.interface_choices.extend(imported.behavior.interface_choices);
        }
        
        // Merge environment preferences
        current.environments.extend(imported.environments);
        
        // Update metadata
        current.metadata.updated_at = chrono::Utc::now();
    }
}