use clap::Args;
use std::path::PathBuf;
use std::time::{Duration, Instant};
use eyre::Result;
use serde::{Serialize, Deserialize};
use tokio;

use crate::cli::context::ExecutionContext;
use crate::cli::interface::{InterfaceSelector, InterfaceStrategy, UserConfig};
use crate::cli::progress::{ProgressStyle, ProgressFeedback, ProgressAware, PhaseTracker, ProgressPhase};
use crate::cli::Command;

#[derive(Debug, Args, Clone)]
pub struct StartCommand {
    /// Dataflow configuration file
    pub dataflow_path: Option<PathBuf>,
    
    /// Custom name for the dataflow instance
    #[clap(long)]
    pub name: Option<String>,
    
    /// Start in detached mode (don't wait for completion)
    #[clap(short, long)]
    pub detach: bool,
    
    /// Timeout for start operation (seconds)
    #[clap(long, default_value = "300")]
    pub timeout: u64,
    
    /// Show progress even in non-interactive mode
    #[clap(long)]
    pub progress: bool,
    
    /// Skip dependency validation
    #[clap(long)]
    pub skip_validation: bool,
    
    /// Environment variables to pass to nodes
    #[clap(long = "env")]
    pub environment: Vec<String>,
    
    /// Override configuration values
    #[clap(long = "set")]
    pub config_overrides: Vec<String>,
    
    /// Suppress hints and suggestions
    #[clap(long)]
    pub no_hints: bool,
}

impl Default for StartCommand {
    fn default() -> Self {
        Self {
            dataflow_path: None,
            name: None,
            detach: false,
            timeout: 300,
            progress: false,
            skip_validation: false,
            environment: Vec::new(),
            config_overrides: Vec::new(),
            no_hints: false,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DataflowConfig {
    pub name: String,
    pub description: Option<String>,
    pub nodes: Vec<NodeConfig>,
    pub operators: Vec<OperatorConfig>,
    pub metadata: DataflowMetadata,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NodeConfig {
    pub name: String,
    pub path: Option<PathBuf>,
    pub dependencies: Vec<String>,
    pub build_time_estimate: Option<u64>, // seconds
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OperatorConfig {
    pub name: String,
    pub path: Option<PathBuf>,
    pub dependencies: Vec<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DataflowMetadata {
    pub complexity: ComplexityLevel,
    pub estimated_start_time: Option<u64>, // seconds
    pub requires_monitoring: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum ComplexityLevel {
    Simple,
    Moderate,
    Complex,
    Enterprise,
}

impl ComplexityLevel {
    pub fn is_complex(&self) -> bool {
        matches!(self, ComplexityLevel::Complex | ComplexityLevel::Enterprise)
    }
}

pub struct StartCommandHandler;

impl StartCommandHandler {
    pub async fn execute(command: &StartCommand, context: &ExecutionContext) -> Result<()> {
        // Load and validate dataflow configuration
        let dataflow_config = Self::load_and_validate_config(command).await?;
        
        // Determine interface strategy
        let mut interface_selector = InterfaceSelector::new(context.clone(), UserConfig::load()?);
        let decision = interface_selector.select_interface(&Command::Start(command.clone()));
        
        match decision.strategy {
            InterfaceStrategy::CliOnly => {
                Self::start_with_cli_feedback(command, &dataflow_config, context).await?;
            },
            
            InterfaceStrategy::CliWithHint { hint, tui_command } => {
                Self::start_with_cli_feedback(command, &dataflow_config, context).await?;
                if !command.no_hints {
                    Self::show_monitoring_hint(&hint, &tui_command);
                }
            },
            
            InterfaceStrategy::PromptForTui { reason, default_yes } => {
                if dataflow_config.metadata.complexity.is_complex() && 
                   Self::should_prompt_for_monitoring(command, &reason, default_yes)? {
                    Self::start_with_interactive_monitoring(command, &dataflow_config).await?;
                } else {
                    Self::start_with_cli_feedback(command, &dataflow_config, context).await?;
                }
            },
            
            InterfaceStrategy::AutoLaunchTui { reason, show_cli_first: _ } => {
                println!("🚀 {}", reason);
                Self::start_with_interactive_monitoring(command, &dataflow_config).await?;
            },
        }
        
        Ok(())
    }
    
    async fn load_and_validate_config(command: &StartCommand) -> Result<DataflowConfig> {
        // For now, create a mock configuration
        // In a real implementation, this would load from the dataflow_path
        let config = DataflowConfig {
            name: command.name.clone().unwrap_or_else(|| "default-dataflow".to_string()),
            description: Some("Example dataflow configuration".to_string()),
            nodes: vec![
                NodeConfig {
                    name: "input-node".to_string(),
                    path: Some(PathBuf::from("./nodes/input")),
                    dependencies: vec![],
                    build_time_estimate: Some(5),
                },
                NodeConfig {
                    name: "processing-node".to_string(),
                    path: Some(PathBuf::from("./nodes/processor")),
                    dependencies: vec!["input-node".to_string()],
                    build_time_estimate: Some(10),
                },
                NodeConfig {
                    name: "output-node".to_string(),
                    path: Some(PathBuf::from("./nodes/output")),
                    dependencies: vec!["processing-node".to_string()],
                    build_time_estimate: Some(3),
                },
            ],
            operators: vec![],
            metadata: DataflowMetadata {
                complexity: ComplexityLevel::Moderate,
                estimated_start_time: Some(25),
                requires_monitoring: false,
            },
        };
        
        if !command.skip_validation {
            Self::validate_dependencies(&config).await?;
        }
        
        Ok(config)
    }
    
    async fn validate_dependencies(config: &DataflowConfig) -> Result<()> {
        // Validate that all node dependencies exist
        for node in &config.nodes {
            for dep in &node.dependencies {
                if !config.nodes.iter().any(|n| n.name == *dep) {
                    return Err(eyre::eyre!("Node '{}' depends on '{}' which does not exist", node.name, dep));
                }
            }
        }
        Ok(())
    }
    
    async fn start_with_cli_feedback(
        command: &StartCommand,
        config: &DataflowConfig,
        context: &ExecutionContext,
    ) -> Result<()> {
        let progress_style = Self::determine_progress_style(command, context, config);
        
        match progress_style {
            ProgressStyle::None => Self::start_simple(command, config).await,
            ProgressStyle::Spinner => Self::start_with_spinner(command, config).await,
            ProgressStyle::ProgressBar => Self::start_with_progress_bar(command, config).await,
            ProgressStyle::Detailed => Self::start_with_detailed_progress(command, config).await,
        }
    }
    
    async fn start_simple(command: &StartCommand, config: &DataflowConfig) -> Result<()> {
        println!("Starting dataflow '{}'...", config.name);
        
        // Simulate start process
        Self::simulate_dataflow_start(config).await?;
        
        println!("✅ Dataflow '{}' started successfully", config.name);
        
        if !command.detach {
            println!("💡 Use 'dora logs {}' to view output", config.name);
            println!("💡 Use 'dora stop {}' to stop the dataflow", config.name);
        }
        
        Ok(())
    }
    
    async fn start_with_spinner(command: &StartCommand, config: &DataflowConfig) -> Result<()> {
        println!("🔧 Starting dataflow '{}'...", config.name);
        
        let spinner = ProgressFeedback::create_simple_spinner("Building and starting nodes...");
        
        Self::simulate_dataflow_start(config).await?;
        
        spinner.finish_with_message("✅ Dataflow started successfully");
        
        Self::show_start_success(command, config);
        Ok(())
    }
    
    async fn start_with_progress_bar(command: &StartCommand, config: &DataflowConfig) -> Result<()> {
        println!("🔧 Starting dataflow '{}'...", config.name);
        
        let total_steps = config.nodes.len() * 2; // build + start for each node
        let progress = ProgressFeedback::create_simple_progress_bar(total_steps as u64, "Processing nodes");
        
        for node in &config.nodes {
            // Build phase
            progress.set_message(format!("Building {}", node.name));
            Self::simulate_build_node(node).await?;
            progress.inc(1);
            
            // Start phase
            progress.set_message(format!("Starting {}", node.name));
            Self::simulate_start_node(node).await?;
            progress.inc(1);
        }
        
        progress.finish_with_message("✅ All nodes started successfully");
        
        Self::show_start_success(command, config);
        Ok(())
    }
    
    async fn start_with_detailed_progress(command: &StartCommand, config: &DataflowConfig) -> Result<()> {
        println!("🔧 Starting dataflow '{}'...", config.name);
        
        let phases = vec![
            ProgressPhase {
                name: "validation".to_string(),
                description: "Validating configuration".to_string(),
                total_items: None,
            },
            ProgressPhase {
                name: "building".to_string(),
                description: "Building nodes".to_string(),
                total_items: Some(config.nodes.len() as u64),
            },
            ProgressPhase {
                name: "starting".to_string(),
                description: "Starting nodes".to_string(),
                total_items: Some(config.nodes.len() as u64),
            },
            ProgressPhase {
                name: "health_check".to_string(),
                description: "Performing health checks".to_string(),
                total_items: None,
            },
        ];
        
        let mut tracker = PhaseTracker::new(phases);
        
        // Phase 1: Validation
        if let Some(spinner) = tracker.start_phase(0) {
            Self::validate_dependencies(config).await?;
            spinner.finish_with_message("Configuration validated");
        }
        tracker.complete_phase(0, "Configuration validated");
        
        // Phase 2: Building
        if let Some(progress) = tracker.start_phase(1) {
            for (_i, node) in config.nodes.iter().enumerate() {
                progress.set_message(format!("Building {}", node.name));
                Self::simulate_build_node(node).await?;
                progress.inc(1);
            }
            progress.finish_with_message("All nodes built successfully");
        }
        tracker.complete_phase(1, "All nodes built successfully");
        
        // Phase 3: Starting
        let mut started_nodes = Vec::new();
        if let Some(progress) = tracker.start_phase(2) {
            for node in &config.nodes {
                progress.set_message(format!("Starting {}", node.name));
                
                match Self::simulate_start_node(node).await {
                    Ok(node_id) => {
                        started_nodes.push(node_id);
                        progress.inc(1);
                    },
                    Err(e) => {
                        progress.abandon_with_message(format!("Failed to start {}", node.name));
                        tracker.fail_phase(2, &format!("Failed to start {}: {}", node.name, e));
                        Self::cleanup_started_nodes(&started_nodes).await?;
                        return Err(e);
                    }
                }
            }
            progress.finish_with_message("All nodes started successfully");
        }
        tracker.complete_phase(2, "All nodes started successfully");
        
        // Phase 4: Health Check
        if let Some(spinner) = tracker.start_phase(3) {
            Self::wait_for_healthy_state(config, Duration::from_secs(30)).await?;
            spinner.finish_with_message("Dataflow is healthy");
        }
        tracker.complete_phase(3, "Dataflow is healthy");
        
        Self::show_start_success(command, config);
        Ok(())
    }
    
    async fn start_with_interactive_monitoring(command: &StartCommand, config: &DataflowConfig) -> Result<()> {
        println!("🚀 Starting dataflow with interactive monitoring...");
        
        // For now, fall back to detailed progress
        // In a real implementation, this would launch the TUI
        Self::start_with_detailed_progress(command, config).await?;
        
        println!("💡 Interactive monitoring would launch here in full implementation");
        println!("💡 This would integrate with the TUI system from Issue #9");
        
        Ok(())
    }
    
    fn determine_progress_style(
        command: &StartCommand,
        context: &ExecutionContext,
        config: &DataflowConfig,
    ) -> ProgressStyle {
        if !context.is_tty || context.is_piped {
            return ProgressStyle::None;
        }
        
        if command.progress {
            return ProgressStyle::Detailed;
        }
        
        // Auto-detect based on expected operation time
        let estimated_duration = Self::estimate_start_duration(config);
        match estimated_duration.as_secs() {
            0..=3 => ProgressStyle::None,
            4..=10 => ProgressStyle::Spinner,
            11..=30 => ProgressStyle::ProgressBar,
            _ => ProgressStyle::Detailed,
        }
    }
    
    fn estimate_start_duration(config: &DataflowConfig) -> Duration {
        let total_build_time: u64 = config.nodes.iter()
            .map(|n| n.build_time_estimate.unwrap_or(5))
            .sum();
        
        let start_overhead = config.nodes.len() as u64 * 2; // 2 seconds per node
        let health_check_time = 5; // 5 seconds for health check
        
        Duration::from_secs(total_build_time + start_overhead + health_check_time)
    }
    
    fn should_prompt_for_monitoring(
        _command: &StartCommand,
        reason: &str,
        default_yes: bool,
    ) -> Result<bool> {
        use std::io::{self, Write};
        
        println!();
        println!("🔍 {}", reason);
        println!("This operation may take several minutes to complete.");
        
        let prompt = if default_yes {
            "Monitor progress interactively? [Y/n]: "
        } else {
            "Monitor progress interactively? [y/N]: "
        };
        
        print!("{}", prompt);
        io::stdout().flush()?;
        
        let mut input = String::new();
        io::stdin().read_line(&mut input)?;
        let input = input.trim().to_lowercase();
        
        Ok(match input.as_str() {
            "" => default_yes,
            "y" | "yes" => true,
            "n" | "no" => false,
            _ => default_yes,
        })
    }
    
    fn show_monitoring_hint(hint: &str, tui_command: &str) {
        println!();
        println!("💡 {}", hint);
        println!("💡 For interactive monitoring, run: {}", tui_command);
    }
    
    fn show_start_success(command: &StartCommand, config: &DataflowConfig) {
        println!();
        println!("🎉 Dataflow '{}' started successfully!", config.name);
        
        if !command.detach {
            println!();
            println!("📋 Next steps:");
            println!("  • View logs: dora logs {}", config.name);
            println!("  • Monitor status: dora ps");
            println!("  • Stop dataflow: dora stop {}", config.name);
            
            if config.metadata.requires_monitoring {
                println!("  • Launch monitoring: dora ui --dataflow {}", config.name);
            }
        }
    }
    
    // Simulation functions for demo purposes
    async fn simulate_dataflow_start(config: &DataflowConfig) -> Result<()> {
        let estimated = Self::estimate_start_duration(config);
        tokio::time::sleep(Duration::from_millis(
            (estimated.as_millis() / 10) as u64 // Speed up for demo
        )).await;
        Ok(())
    }
    
    async fn simulate_build_node(node: &NodeConfig) -> Result<()> {
        let build_time = node.build_time_estimate.unwrap_or(5);
        tokio::time::sleep(Duration::from_millis(build_time * 100)).await; // Speed up for demo
        Ok(())
    }
    
    async fn simulate_start_node(node: &NodeConfig) -> Result<String> {
        tokio::time::sleep(Duration::from_millis(200)).await; // Speed up for demo
        Ok(format!("node-{}-{}", node.name, uuid::Uuid::new_v4().to_string()[..8].to_string()))
    }
    
    async fn cleanup_started_nodes(node_ids: &[String]) -> Result<()> {
        for node_id in node_ids {
            println!("🧹 Cleaning up node {}", node_id);
            tokio::time::sleep(Duration::from_millis(100)).await;
        }
        Ok(())
    }
    
    async fn wait_for_healthy_state(_config: &DataflowConfig, timeout: Duration) -> Result<()> {
        let start = Instant::now();
        while start.elapsed() < timeout {
            // Simulate health check
            tokio::time::sleep(Duration::from_millis(500)).await;
            
            // For demo, assume healthy after 2 seconds
            if start.elapsed() > Duration::from_secs(2) {
                return Ok(());
            }
        }
        
        Err(eyre::eyre!("Dataflow did not become healthy within timeout"))
    }
}

impl ProgressAware for StartCommand {
    fn determine_progress_style(&self, context: &ExecutionContext) -> ProgressStyle {
        if !context.is_tty || context.is_piped {
            return ProgressStyle::None;
        }
        
        if self.progress {
            return ProgressStyle::Detailed;
        }
        
        // Default to spinner for start commands
        ProgressStyle::Spinner
    }
    
    fn estimate_operation_duration(&self) -> Duration {
        Duration::from_secs(self.timeout.min(300)) // Cap at 5 minutes for estimation
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cli::context::{ExecutionContext, ExecutionEnvironment, CiEnvironment, TerminalCapabilities};
    use crate::cli::{UiMode, OutputFormat};
    use std::collections::HashMap;
    
    #[test]
    fn test_start_command_validation() {
        let cmd = StartCommand {
            dataflow_path: Some(PathBuf::from("test.yml")),
            timeout: 300,
            ..Default::default()
        };
        
        // Basic validation - command should be valid
        assert!(cmd.timeout > 0);
        assert!(!cmd.detach || cmd.timeout > 0);
    }
    
    #[test]
    fn test_progress_style_selection() {
        let context_interactive = ExecutionContext {
            working_dir: PathBuf::from("."),
            output_format: OutputFormat::Auto,
            ui_mode: Some(UiMode::Auto),
            no_hints: false,
            verbose: false,
            quiet: false,
            is_tty: true,
            is_piped: false,
            is_scripted: false,
            terminal_size: Some((80, 24)),
            user_preference: UiMode::Auto,
            environment: ExecutionEnvironment {
                ci_environment: None,
                shell_type: Some("bash".to_string()),
                relevant_env_vars: HashMap::new(),
                is_ci: false,
                is_automation: false,
            },
            terminal_capabilities: TerminalCapabilities {
                colors: true,
                interactive: true,
                width: Some(80),
                height: Some(24),
                tui_capable: true,
                supports_color: true,
                supports_unicode: true,
                supports_mouse: false,
                terminal_type: Some("xterm".to_string()),
            },
        };
        
        let context_piped = ExecutionContext {
            working_dir: PathBuf::from("."),
            output_format: OutputFormat::Auto,
            ui_mode: Some(UiMode::Auto),
            no_hints: false,
            verbose: false,
            quiet: false,
            is_tty: false,
            is_piped: true,
            is_scripted: false,
            terminal_size: None,
            user_preference: UiMode::Auto,
            environment: ExecutionEnvironment {
                ci_environment: None,
                shell_type: Some("bash".to_string()),
                relevant_env_vars: HashMap::new(),
                is_ci: false,
                is_automation: false,
            },
            terminal_capabilities: TerminalCapabilities {
                colors: false,
                interactive: false,
                width: None,
                height: None,
                tui_capable: false,
                supports_color: false,
                supports_unicode: false,
                supports_mouse: false,
                terminal_type: None,
            },
        };
        
        let cmd = StartCommand::default();
        let config = DataflowConfig {
            name: "test".to_string(),
            description: None,
            nodes: vec![],
            operators: vec![],
            metadata: DataflowMetadata {
                complexity: ComplexityLevel::Simple,
                estimated_start_time: Some(15),
                requires_monitoring: false,
            },
        };
        
        let style_interactive = StartCommandHandler::determine_progress_style(&cmd, &context_interactive, &config);
        let style_piped = StartCommandHandler::determine_progress_style(&cmd, &context_piped, &config);
        
        assert!(!matches!(style_interactive, ProgressStyle::None));
        assert!(matches!(style_piped, ProgressStyle::None));
    }
    
    #[test]
    fn test_complexity_detection() {
        let simple = ComplexityLevel::Simple;
        let complex = ComplexityLevel::Complex;
        
        assert!(!simple.is_complex());
        assert!(complex.is_complex());
    }
    
    #[test]
    fn test_duration_estimation() {
        let config = DataflowConfig {
            name: "test".to_string(),
            description: None,
            nodes: vec![
                NodeConfig {
                    name: "node1".to_string(),
                    path: None,
                    dependencies: vec![],
                    build_time_estimate: Some(10),
                },
                NodeConfig {
                    name: "node2".to_string(),
                    path: None,
                    dependencies: vec![],
                    build_time_estimate: Some(20),
                },
            ],
            operators: vec![],
            metadata: DataflowMetadata {
                complexity: ComplexityLevel::Moderate,
                estimated_start_time: None,
                requires_monitoring: false,
            },
        };
        
        let duration = StartCommandHandler::estimate_start_duration(&config);
        
        // Should be build time (30) + start overhead (4) + health check (5) = 39 seconds
        assert_eq!(duration.as_secs(), 39);
    }
}