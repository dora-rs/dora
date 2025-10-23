use clap::Args;
use std::time::{Duration, Instant};
use eyre::Result;
use serde::{Serialize, Deserialize};
use tokio;

use crate::cli::context::ExecutionContext;
use crate::cli::progress::{ProgressStyle, ProgressFeedback, ProgressAware, PhaseTracker, ProgressPhase};
use crate::cli::commands::start::DataflowConfig;

#[derive(Debug, Args, Clone)]
pub struct StopCommand {
    /// Dataflow name or ID to stop
    pub target: Option<String>,
    
    /// Stop all running dataflows
    #[clap(long, conflicts_with = "target")]
    pub all: bool,
    
    /// Force stop without graceful shutdown
    #[clap(short, long)]
    pub force: bool,
    
    /// Timeout for graceful shutdown (seconds)
    #[clap(long, default_value = "30")]
    pub timeout: u64,
    
    /// Remove dataflow configuration after stopping
    #[clap(long)]
    pub remove: bool,
    
    /// Suppress confirmation prompts
    #[clap(short, long)]
    pub yes: bool,
    
    /// Show progress even in non-interactive mode
    #[clap(long)]
    pub progress: bool,
}

impl Default for StopCommand {
    fn default() -> Self {
        Self {
            target: None,
            all: false,
            force: false,
            timeout: 30,
            remove: false,
            yes: false,
            progress: false,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DataflowInfo {
    pub id: String,
    pub name: String,
    pub status: DataflowStatus,
    pub nodes: Vec<NodeInfo>,
    pub uptime: Duration,
    pub config: Option<DataflowConfig>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NodeInfo {
    pub id: String,
    pub name: String,
    pub status: NodeStatus,
    pub pid: Option<u32>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum DataflowStatus {
    Running,
    Starting,
    Stopping,
    Stopped,
    Error(String),
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum NodeStatus {
    Running,
    Starting,
    Stopping,
    Stopped,
    Error(String),
}

pub struct StopCommandHandler;

impl StopCommandHandler {
    pub async fn execute(command: &StopCommand, context: &ExecutionContext) -> Result<()> {
        let targets = if command.all {
            Self::get_all_running_dataflows().await?
        } else if let Some(target) = &command.target {
            vec![Self::resolve_dataflow_target(target).await?]
        } else {
            return Err(eyre::eyre!("Must specify either a target dataflow or --all"));
        };
        
        if targets.is_empty() {
            println!("No running dataflows found.");
            return Ok(());
        }
        
        // Confirm dangerous operations
        if (command.all || targets.len() > 1 || command.remove) && !command.yes {
            Self::confirm_stop_operation(command, &targets)?;
        }
        
        match targets.len() {
            1 => Self::stop_single_dataflow(command, &targets[0], context).await?,
            _ => Self::stop_multiple_dataflows(command, &targets, context).await?,
        }
        
        Ok(())
    }
    
    async fn get_all_running_dataflows() -> Result<Vec<DataflowInfo>> {
        // Mock implementation - in real system would query daemon/coordinator
        Ok(vec![
            DataflowInfo {
                id: "df-001".to_string(),
                name: "example-dataflow".to_string(),
                status: DataflowStatus::Running,
                uptime: Duration::from_secs(3600),
                nodes: vec![
                    NodeInfo {
                        id: "node-001".to_string(),
                        name: "input-node".to_string(),
                        status: NodeStatus::Running,
                        pid: Some(12345),
                    },
                    NodeInfo {
                        id: "node-002".to_string(),
                        name: "processing-node".to_string(),
                        status: NodeStatus::Running,
                        pid: Some(12346),
                    },
                ],
                config: None,
            },
            DataflowInfo {
                id: "df-002".to_string(),
                name: "monitoring-dataflow".to_string(),
                status: DataflowStatus::Running,
                uptime: Duration::from_secs(1800),
                nodes: vec![
                    NodeInfo {
                        id: "node-003".to_string(),
                        name: "metrics-collector".to_string(),
                        status: NodeStatus::Running,
                        pid: Some(12347),
                    },
                ],
                config: None,
            },
        ])
    }
    
    async fn resolve_dataflow_target(target: &str) -> Result<DataflowInfo> {
        let all_dataflows = Self::get_all_running_dataflows().await?;
        
        // Try to find by exact name first
        if let Some(dataflow) = all_dataflows.iter().find(|df| df.name == target) {
            return Ok(dataflow.clone());
        }
        
        // Try to find by ID
        if let Some(dataflow) = all_dataflows.iter().find(|df| df.id == target) {
            return Ok(dataflow.clone());
        }
        
        // Try partial name matching
        let matches: Vec<_> = all_dataflows.iter()
            .filter(|df| df.name.contains(target))
            .collect();
        
        match matches.len() {
            0 => Err(eyre::eyre!("No dataflow found matching '{}'", target)),
            1 => Ok(matches[0].clone()),
            _ => {
                let names: Vec<String> = matches.iter().map(|df| df.name.clone()).collect();
                Err(eyre::eyre!(
                    "Multiple dataflows match '{}': {}. Please be more specific.",
                    target,
                    names.join(", ")
                ))
            }
        }
    }
    
    fn confirm_stop_operation(command: &StopCommand, targets: &[DataflowInfo]) -> Result<()> {
        use std::io::{self, Write};
        
        println!();
        if command.all {
            println!("⚠️  You are about to stop ALL running dataflows:");
        } else {
            println!("⚠️  You are about to stop {} dataflow(s):", targets.len());
        }
        
        for dataflow in targets {
            println!("  • {} ({})", dataflow.name, dataflow.id);
            if command.remove {
                println!("    ⚠️  Configuration will be REMOVED permanently");
            }
        }
        
        if command.force {
            println!("  ⚠️  Force stop enabled - nodes will be terminated immediately");
        }
        
        print!("\nContinue? [y/N]: ");
        io::stdout().flush()?;
        
        let mut input = String::new();
        io::stdin().read_line(&mut input)?;
        let input = input.trim().to_lowercase();
        
        if input != "y" && input != "yes" {
            println!("Operation cancelled.");
            std::process::exit(0);
        }
        
        Ok(())
    }
    
    async fn stop_single_dataflow(
        command: &StopCommand,
        dataflow: &DataflowInfo,
        context: &ExecutionContext,
    ) -> Result<()> {
        let progress_style = if command.force {
            ProgressStyle::Spinner
        } else {
            Self::determine_progress_style(command, context, dataflow)
        };
        
        match progress_style {
            ProgressStyle::None => Self::stop_simple(command, dataflow).await,
            ProgressStyle::Spinner => Self::stop_with_spinner(command, dataflow).await,
            ProgressStyle::Detailed => Self::stop_with_detailed_progress(command, dataflow).await,
            _ => Self::stop_with_spinner(command, dataflow).await,
        }
    }
    
    async fn stop_multiple_dataflows(
        command: &StopCommand,
        dataflows: &[DataflowInfo],
        _context: &ExecutionContext,
    ) -> Result<()> {
        println!("🛑 Stopping {} dataflows...", dataflows.len());
        
        let progress = ProgressFeedback::create_simple_progress_bar(
            dataflows.len() as u64,
            "Stopping dataflows"
        );
        
        for dataflow in dataflows {
            progress.set_message(format!("Stopping {}", dataflow.name));
            
            match Self::stop_single_dataflow_internal(command, dataflow).await {
                Ok(_) => {
                    progress.inc(1);
                },
                Err(e) => {
                    progress.println(format!("❌ Failed to stop {}: {}", dataflow.name, e));
                    progress.inc(1);
                }
            }
        }
        
        progress.finish_with_message("✅ All stop operations completed");
        Ok(())
    }
    
    async fn stop_simple(command: &StopCommand, dataflow: &DataflowInfo) -> Result<()> {
        println!("Stopping dataflow '{}'...", dataflow.name);
        
        Self::stop_single_dataflow_internal(command, dataflow).await?;
        
        println!("✅ Dataflow '{}' stopped successfully", dataflow.name);
        Ok(())
    }
    
    async fn stop_with_spinner(command: &StopCommand, dataflow: &DataflowInfo) -> Result<()> {
        println!("🛑 Stopping dataflow '{}'...", dataflow.name);
        
        let message = if command.force {
            "Force stopping all nodes..."
        } else {
            "Gracefully shutting down..."
        };
        
        let spinner = ProgressFeedback::create_simple_spinner(message);
        
        Self::stop_single_dataflow_internal(command, dataflow).await?;
        
        spinner.finish_with_message("✅ Dataflow stopped successfully");
        
        Self::show_stop_success(command, dataflow);
        Ok(())
    }
    
    async fn stop_with_detailed_progress(command: &StopCommand, dataflow: &DataflowInfo) -> Result<()> {
        println!("🛑 Stopping dataflow '{}'...", dataflow.name);
        
        if command.force {
            let spinner = ProgressFeedback::create_simple_spinner("Force stopping all nodes...");
            Self::force_stop_dataflow(dataflow).await?;
            spinner.finish_with_message("✅ Dataflow force stopped");
            Self::show_stop_success(command, dataflow);
            return Ok(());
        }
        
        // Graceful shutdown with detailed progress
        let phases = vec![
            ProgressPhase {
                name: "signal".to_string(),
                description: "Signaling shutdown".to_string(),
                total_items: Some(dataflow.nodes.len() as u64),
            },
            ProgressPhase {
                name: "wait".to_string(),
                description: "Waiting for graceful shutdown".to_string(),
                total_items: Some(dataflow.nodes.len() as u64),
            },
        ];
        
        let mut phases = phases;
        if command.remove {
            phases.push(ProgressPhase {
                name: "cleanup".to_string(),
                description: "Removing configuration".to_string(),
                total_items: None,
            });
        }
        
        let mut tracker = PhaseTracker::new(phases);
        
        // Phase 1: Signal shutdown
        if let Some(progress) = tracker.start_phase(0) {
            for node in &dataflow.nodes {
                progress.set_message(format!("Stopping {}", node.name));
                Self::signal_node_shutdown(node).await?;
                progress.inc(1);
            }
            progress.finish_with_message("Shutdown signals sent");
        }
        tracker.complete_phase(0, "Shutdown signals sent");
        
        // Phase 2: Wait for graceful shutdown
        if let Some(progress) = tracker.start_phase(1) {
            let timeout = Duration::from_secs(command.timeout);
            let start_time = Instant::now();
            
            let mut remaining_nodes = dataflow.nodes.clone();
            while !remaining_nodes.is_empty() && start_time.elapsed() < timeout {
                tokio::time::sleep(Duration::from_millis(500)).await;
                
                let mut still_running = Vec::new();
                for node in remaining_nodes {
                    if Self::is_node_running(&node).await? {
                        still_running.push(node);
                    } else {
                        progress.inc(1);
                    }
                }
                remaining_nodes = still_running;
            }
            
            if remaining_nodes.is_empty() {
                progress.finish_with_message("Graceful shutdown completed");
                tracker.complete_phase(1, "Graceful shutdown completed");
            } else {
                progress.abandon_with_message(format!(
                    "{} nodes didn't stop gracefully", 
                    remaining_nodes.len()
                ));
                tracker.fail_phase(1, "Some nodes didn't stop gracefully");
                
                // Force stop remaining nodes
                let force_spinner = ProgressFeedback::create_simple_spinner("Force stopping remaining nodes...");
                for node in &remaining_nodes {
                    Self::force_stop_node(node).await?;
                }
                force_spinner.finish_with_message("Remaining nodes force stopped");
            }
        }
        
        // Phase 3: Cleanup (if requested)
        if command.remove {
            if let Some(spinner) = tracker.start_phase(2) {
                Self::remove_dataflow_config(dataflow).await?;
                spinner.finish_with_message("Configuration removed");
            }
            tracker.complete_phase(2, "Configuration removed");
        }
        
        Self::show_stop_success(command, dataflow);
        Ok(())
    }
    
    async fn stop_single_dataflow_internal(command: &StopCommand, dataflow: &DataflowInfo) -> Result<()> {
        if command.force {
            Self::force_stop_dataflow(dataflow).await?;
        } else {
            Self::graceful_stop_dataflow(command, dataflow).await?;
        }
        
        if command.remove {
            Self::remove_dataflow_config(dataflow).await?;
        }
        
        Ok(())
    }
    
    async fn graceful_stop_dataflow(command: &StopCommand, dataflow: &DataflowInfo) -> Result<()> {
        // Signal all nodes to stop
        for node in &dataflow.nodes {
            Self::signal_node_shutdown(node).await?;
        }
        
        // Wait for graceful shutdown
        let timeout = Duration::from_secs(command.timeout);
        let start_time = Instant::now();
        
        let mut remaining_nodes = dataflow.nodes.clone();
        while !remaining_nodes.is_empty() && start_time.elapsed() < timeout {
            tokio::time::sleep(Duration::from_millis(500)).await;
            
            let mut still_running = Vec::new();
            for node in remaining_nodes {
                if Self::is_node_running(&node).await? {
                    still_running.push(node);
                }
            }
            remaining_nodes = still_running;
        }
        
        // Force stop any remaining nodes
        if !remaining_nodes.is_empty() {
            for node in &remaining_nodes {
                Self::force_stop_node(node).await?;
            }
        }
        
        Ok(())
    }
    
    async fn force_stop_dataflow(dataflow: &DataflowInfo) -> Result<()> {
        for node in &dataflow.nodes {
            Self::force_stop_node(node).await?;
        }
        Ok(())
    }
    
    fn determine_progress_style(
        command: &StopCommand,
        context: &ExecutionContext,
        dataflow: &DataflowInfo,
    ) -> ProgressStyle {
        if !context.is_tty || context.is_piped {
            return ProgressStyle::None;
        }
        
        if command.progress {
            return ProgressStyle::Detailed;
        }
        
        // Use detailed progress for complex dataflows
        if dataflow.nodes.len() > 3 || command.timeout > 10 {
            ProgressStyle::Detailed
        } else {
            ProgressStyle::Spinner
        }
    }
    
    fn show_stop_success(command: &StopCommand, dataflow: &DataflowInfo) {
        println!();
        println!("🏁 Dataflow '{}' stopped successfully!", dataflow.name);
        
        if command.remove {
            println!("🗑️  Configuration removed");
        }
        
        println!();
        println!("📋 Next steps:");
        println!("  • View all dataflows: dora ps");
        println!("  • Start again: dora start {}", dataflow.name);
        
        if command.remove {
            println!("  • Create new: dora new dataflow {}", dataflow.name);
        }
    }
    
    // Helper functions for node operations
    async fn signal_node_shutdown(_node: &NodeInfo) -> Result<()> {
        // Simulate sending shutdown signal
        tokio::time::sleep(Duration::from_millis(100)).await;
        Ok(())
    }
    
    async fn is_node_running(node: &NodeInfo) -> Result<bool> {
        // Simulate checking if node is still running
        tokio::time::sleep(Duration::from_millis(50)).await;
        
        // For demo, randomly determine if node is still running
        use std::collections::hash_map::DefaultHasher;
        use std::hash::{Hash, Hasher};
        
        let mut hasher = DefaultHasher::new();
        node.id.hash(&mut hasher);
        let hash = hasher.finish();
        
        // Gradually reduce chance of being running over time
        Ok((hash % 10) < 2) // 20% chance still running
    }
    
    async fn force_stop_node(_node: &NodeInfo) -> Result<()> {
        // Simulate force stopping a node
        tokio::time::sleep(Duration::from_millis(200)).await;
        Ok(())
    }
    
    async fn remove_dataflow_config(_dataflow: &DataflowInfo) -> Result<()> {
        // Simulate removing dataflow configuration
        tokio::time::sleep(Duration::from_millis(300)).await;
        Ok(())
    }
}

impl ProgressAware for StopCommand {
    fn determine_progress_style(&self, context: &ExecutionContext) -> ProgressStyle {
        if !context.is_tty || context.is_piped {
            return ProgressStyle::None;
        }
        
        if self.progress {
            return ProgressStyle::Detailed;
        }
        
        if self.force {
            ProgressStyle::Spinner
        } else {
            ProgressStyle::Detailed
        }
    }
    
    fn estimate_operation_duration(&self) -> Duration {
        if self.force {
            Duration::from_secs(5) // Force stop is quick
        } else {
            Duration::from_secs(self.timeout + 10) // Graceful + overhead
        }
    }
}

impl StopCommand {
    pub fn requires_confirmation(&self) -> bool {
        (self.all || self.remove) && !self.yes
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cli::context::{ExecutionContext, ExecutionEnvironment, TerminalCapabilities};
    use crate::cli::{UiMode, OutputFormat};
    use std::collections::HashMap;
    use std::path::PathBuf;
    
    #[test]
    fn test_stop_confirmation_logic() {
        let cmd = StopCommand {
            all: true,
            yes: false,
            ..Default::default()
        };
        
        // Should require confirmation for --all
        assert!(cmd.requires_confirmation());
        
        let cmd = StopCommand {
            target: Some("single-dataflow".to_string()),
            yes: true,
            ..Default::default()
        };
        
        // Should not require confirmation with --yes
        assert!(!cmd.requires_confirmation());
        
        let cmd = StopCommand {
            target: Some("test".to_string()),
            remove: true,
            yes: false,
            ..Default::default()
        };
        
        // Should require confirmation for --remove
        assert!(cmd.requires_confirmation());
    }
    
    #[test]
    fn test_progress_style_for_force_stop() {
        let context = ExecutionContext {
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
        
        let dataflow = DataflowInfo {
            id: "test".to_string(),
            name: "test".to_string(),
            status: DataflowStatus::Running,
            uptime: Duration::from_secs(100),
            nodes: vec![],
            config: None,
        };
        
        let force_cmd = StopCommand {
            force: true,
            ..Default::default()
        };
        
        let graceful_cmd = StopCommand {
            force: false,
            ..Default::default()
        };
        
        let force_style = StopCommandHandler::determine_progress_style(&force_cmd, &context, &dataflow);
        let graceful_style = StopCommandHandler::determine_progress_style(&graceful_cmd, &context, &dataflow);
        
        assert!(matches!(force_style, ProgressStyle::Spinner));
        assert!(matches!(graceful_style, ProgressStyle::Detailed));
    }
    
    #[test]
    fn test_duration_estimation() {
        let force_cmd = StopCommand {
            force: true,
            timeout: 30,
            ..Default::default()
        };
        
        let graceful_cmd = StopCommand {
            force: false,
            timeout: 30,
            ..Default::default()
        };
        
        let force_duration = force_cmd.estimate_operation_duration();
        let graceful_duration = graceful_cmd.estimate_operation_duration();
        
        assert_eq!(force_duration.as_secs(), 5);
        assert_eq!(graceful_duration.as_secs(), 40); // 30 + 10 overhead
    }
}