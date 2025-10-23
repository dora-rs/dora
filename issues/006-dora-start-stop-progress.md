# Issue #006: Create `dora start/stop` with Progress Feedback

## 📋 Summary
Enhance `dora start` and `dora stop` commands with intelligent progress feedback and TUI suggestions for complex operations. These commands should provide clear visual feedback during long-running operations and suggest interactive monitoring when appropriate.

## 🎯 Objectives
- Create reliable dataflow lifecycle management commands
- Implement progress indicators for long-running start/stop operations
- Add smart TUI suggestions for complex dataflows
- Provide clear error handling and recovery suggestions
- Ensure excellent user experience for both simple and complex scenarios

**Success Metrics:**
- Commands provide real-time progress feedback for operations >3 seconds
- Complex dataflow starts suggest interactive monitoring appropriately
- Error messages include actionable recovery steps
- 100% backward compatibility with existing start/stop usage
- Start/stop operations are interruptible with graceful cleanup

## 🛠️ Technical Requirements

### What to Build

#### 1. Enhanced Start Command
```rust
// src/cli/commands/start.rs
#[derive(Debug, clap::Args)]
pub struct StartCommand {
    /// Dataflow configuration file
    pub dataflow_path: PathBuf,
    
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

impl StartCommand {
    pub async fn execute(&self, context: &ExecutionContext) -> Result<()> {
        // Validate dataflow configuration
        let dataflow_config = self.load_and_validate_config().await?;
        let complexity = self.analyze_dataflow_complexity(&dataflow_config);
        
        // Determine interface strategy
        let interface_selector = InterfaceSelector::new(context.clone(), UserConfig::load()?);
        let decision = interface_selector.select_interface(&Command::Start(self.clone()));
        
        match decision.strategy {
            InterfaceStrategy::CliOnly => {
                self.start_with_cli_feedback(&dataflow_config, context).await?;
            },
            
            InterfaceStrategy::CliWithHint { hint, tui_command } => {
                self.start_with_cli_feedback(&dataflow_config, context).await?;
                if !self.no_hints {
                    self.show_monitoring_hint(&hint, &tui_command);
                }
            },
            
            InterfaceStrategy::PromptForTui { reason, default_yes } => {
                if complexity.is_complex() && self.should_prompt_for_monitoring(&reason, default_yes)? {
                    self.start_with_interactive_monitoring(&dataflow_config).await?;
                } else {
                    self.start_with_cli_feedback(&dataflow_config, context).await?;
                }
            },
            
            InterfaceStrategy::AutoLaunchTui { reason, show_cli_first } => {
                println!("🚀 {}", reason);
                self.start_with_interactive_monitoring(&dataflow_config).await?;
            },
        }
        
        Ok(())
    }
    
    async fn start_with_cli_feedback(
        &self,
        config: &DataflowConfig,
        context: &ExecutionContext,
    ) -> Result<()> {
        let progress_style = self.determine_progress_style(context);
        
        match progress_style {
            ProgressStyle::None => self.start_simple(config).await,
            ProgressStyle::Spinner => self.start_with_spinner(config).await,
            ProgressStyle::ProgressBar => self.start_with_progress_bar(config).await,
            ProgressStyle::Detailed => self.start_with_detailed_progress(config).await,
        }
    }
    
    async fn start_with_detailed_progress(&self, config: &DataflowConfig) -> Result<()> {
        println!("🔧 Starting dataflow '{}'...", config.name);
        
        // Phase 1: Validation
        let spinner = self.create_spinner("Validating configuration...");
        self.validate_dependencies(config).await?;
        spinner.finish_with_message("✅ Configuration validated");
        
        // Phase 2: Building
        let total_nodes = config.nodes.len();
        let build_progress = self.create_progress_bar(total_nodes, "Building nodes");
        
        for (i, node) in config.nodes.iter().enumerate() {
            build_progress.set_message(format!("Building {}", node.name));
            self.build_node(node).await?;
            build_progress.inc(1);
        }
        build_progress.finish_with_message("✅ All nodes built successfully");
        
        // Phase 3: Starting
        let start_progress = self.create_progress_bar(total_nodes, "Starting nodes");
        let mut started_nodes = Vec::new();
        
        for (i, node) in config.nodes.iter().enumerate() {
            start_progress.set_message(format!("Starting {}", node.name));
            
            match self.start_node(node).await {
                Ok(node_id) => {
                    started_nodes.push(node_id);
                    start_progress.inc(1);
                },
                Err(e) => {
                    start_progress.abandon_with_message(format!("❌ Failed to start {}", node.name));
                    self.cleanup_started_nodes(&started_nodes).await?;
                    return Err(e);
                }
            }
        }
        start_progress.finish_with_message("✅ All nodes started successfully");
        
        // Phase 4: Health Check
        let health_spinner = self.create_spinner("Performing health checks...");
        self.wait_for_healthy_state(config, Duration::from_secs(30)).await?;
        health_spinner.finish_with_message("✅ Dataflow is healthy");
        
        self.show_start_success(config);
        Ok(())
    }
}
```

#### 2. Enhanced Stop Command
```rust
// src/cli/commands/stop.rs
#[derive(Debug, clap::Args)]
pub struct StopCommand {
    /// Dataflow name or ID to stop
    pub target: String,
    
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
}

impl StopCommand {
    pub async fn execute(&self, context: &ExecutionContext) -> Result<()> {
        let targets = if self.all {
            self.get_all_running_dataflows().await?
        } else {
            vec![self.resolve_dataflow_target(&self.target).await?]
        };
        
        if targets.is_empty() {
            println!("No running dataflows found.");
            return Ok();
        }
        
        // Confirm dangerous operations
        if (self.all || targets.len() > 1 || self.remove) && !self.yes {
            self.confirm_stop_operation(&targets)?;
        }
        
        match targets.len() {
            1 => self.stop_single_dataflow(&targets[0], context).await?,
            _ => self.stop_multiple_dataflows(&targets, context).await?,
        }
        
        Ok(())
    }
    
    async fn stop_single_dataflow(
        &self,
        dataflow: &DataflowInfo,
        context: &ExecutionContext,
    ) -> Result<()> {
        let progress_style = if self.force {
            ProgressStyle::Spinner
        } else {
            self.determine_progress_style(context, dataflow)
        };
        
        match progress_style {
            ProgressStyle::None => self.stop_simple(dataflow).await,
            ProgressStyle::Spinner => self.stop_with_spinner(dataflow).await,
            ProgressStyle::Detailed => self.stop_with_detailed_progress(dataflow).await,
            _ => self.stop_with_spinner(dataflow).await,
        }
    }
    
    async fn stop_with_detailed_progress(&self, dataflow: &DataflowInfo) -> Result<()> {
        println!("🛑 Stopping dataflow '{}'...", dataflow.name);
        
        if self.force {
            let spinner = self.create_spinner("Force stopping all nodes...");
            self.force_stop_dataflow(dataflow).await?;
            spinner.finish_with_message("✅ Dataflow force stopped");
            return Ok();
        }
        
        // Graceful shutdown with detailed progress
        let nodes = self.get_dataflow_nodes(dataflow).await?;
        let total_nodes = nodes.len();
        
        // Phase 1: Signal shutdown
        let signal_progress = self.create_progress_bar(total_nodes, "Signaling shutdown");
        for node in &nodes {
            signal_progress.set_message(format!("Stopping {}", node.name));
            self.signal_node_shutdown(node).await?;
            signal_progress.inc(1);
        }
        signal_progress.finish_with_message("✅ Shutdown signals sent");
        
        // Phase 2: Wait for graceful shutdown
        let shutdown_progress = self.create_progress_bar(total_nodes, "Waiting for shutdown");
        let timeout = Duration::from_secs(self.timeout);
        let start_time = Instant::now();
        
        let mut remaining_nodes = nodes;
        while !remaining_nodes.is_empty() && start_time.elapsed() < timeout {
            tokio::time::sleep(Duration::from_millis(500)).await;
            
            let mut still_running = Vec::new();
            for node in remaining_nodes {
                if self.is_node_running(&node).await? {
                    still_running.push(node);
                } else {
                    shutdown_progress.inc(1);
                }
            }
            remaining_nodes = still_running;
        }
        
        if remaining_nodes.is_empty() {
            shutdown_progress.finish_with_message("✅ Graceful shutdown completed");
        } else {
            shutdown_progress.abandon_with_message(format!(
                "⚠️ {} nodes didn't stop gracefully", 
                remaining_nodes.len()
            ));
            
            // Force stop remaining nodes
            let force_spinner = self.create_spinner("Force stopping remaining nodes...");
            for node in &remaining_nodes {
                self.force_stop_node(node).await?;
            }
            force_spinner.finish_with_message("✅ Remaining nodes force stopped");
        }
        
        // Phase 3: Cleanup
        if self.remove {
            let cleanup_spinner = self.create_spinner("Removing dataflow configuration...");
            self.remove_dataflow_config(dataflow).await?;
            cleanup_spinner.finish_with_message("✅ Configuration removed");
        }
        
        self.show_stop_success(dataflow);
        Ok(())
    }
}
```

#### 3. Progress Feedback System
```rust
// src/cli/progress.rs
#[derive(Debug, Clone)]
pub enum ProgressStyle {
    None,        // No progress indication
    Spinner,     // Simple spinner for indeterminate progress
    ProgressBar, // Progress bar for determinate progress
    Detailed,    // Detailed multi-phase progress
}

pub struct ProgressManager {
    style: ProgressStyle,
    multi_progress: MultiProgress,
}

impl ProgressManager {
    pub fn new(style: ProgressStyle) -> Self {
        Self {
            style,
            multi_progress: MultiProgress::new(),
        }
    }
    
    pub fn create_spinner(&self, message: &str) -> ProgressBar {
        let spinner = ProgressBar::new_spinner();
        spinner.set_style(
            ProgressStyle::default_spinner()
                .tick_chars("⠁⠂⠄⡀⢀⠠⠐⠈ ")
                .template("{spinner:.green} {msg}")
                .unwrap()
        );
        spinner.set_message(message.to_string());
        spinner.enable_steady_tick(Duration::from_millis(100));
        
        self.multi_progress.add(spinner)
    }
    
    pub fn create_progress_bar(&self, total: usize, message: &str) -> ProgressBar {
        let progress = ProgressBar::new(total as u64);
        progress.set_style(
            ProgressStyle::default_bar()
                .template("{msg} [{bar:40.cyan/blue}] {pos}/{len} ({eta})")
                .unwrap()
                .progress_chars("#>-")
        );
        progress.set_message(message.to_string());
        
        self.multi_progress.add(progress)
    }
}

// Integration with commands
impl StartCommand {
    fn determine_progress_style(&self, context: &ExecutionContext) -> ProgressStyle {
        if !context.is_tty || context.is_piped {
            return ProgressStyle::None;
        }
        
        if self.progress {
            return ProgressStyle::Detailed;
        }
        
        // Auto-detect based on expected operation time
        let estimated_duration = self.estimate_start_duration();
        match estimated_duration.as_secs() {
            0..=3 => ProgressStyle::None,
            4..=10 => ProgressStyle::Spinner,
            11..=30 => ProgressStyle::ProgressBar,
            _ => ProgressStyle::Detailed,
        }
    }
    
    fn estimate_start_duration(&self) -> Duration {
        // Estimate based on dataflow complexity
        // This would analyze the config file and estimate build/start time
        Duration::from_secs(5) // Placeholder
    }
}
```

#### 4. Complex Operation Monitoring
```rust
impl StartCommand {
    async fn start_with_interactive_monitoring(&self, config: &DataflowConfig) -> Result<()> {
        println!("🚀 Starting dataflow with interactive monitoring...");
        
        // Start the dataflow in the background
        let start_handle = tokio::spawn({
            let config = config.clone();
            let command = self.clone();
            async move {
                command.start_dataflow_background(&config).await
            }
        });
        
        // Launch TUI monitoring interface
        let tui_app = DoraApp::new_with_context(
            ViewType::DataflowStartupMonitor {
                dataflow_name: config.name.clone(),
                start_handle: start_handle.into(),
            }
        );
        
        tui_app.run().await?;
        Ok(())
    }
    
    fn should_prompt_for_monitoring(&self, reason: &str, default_yes: bool) -> Result<bool> {
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
}
```

### Why This Approach

**Enhanced User Experience:**
- Clear progress feedback reduces anxiety during long operations
- Smart suggestions help users discover monitoring capabilities
- Graceful error handling with recovery suggestions
- Interruptible operations prevent frustration

**Docker-like Familiarity:**
- Similar flag patterns and behavior to Docker commands
- Consistent output formatting and status reporting
- Familiar concepts like detached mode and timeouts

**Progressive Enhancement:**
- Simple operations stay simple and fast
- Complex operations get enhanced monitoring when beneficial
- User choice respected at all levels

### How to Implement

#### Step 1: Command Structure (3 hours)
1. **Define StartCommand and StopCommand** structs with all options
2. **Implement argument parsing** and validation
3. **Add configuration loading** and analysis
4. **Create basic execution** framework

#### Step 2: Progress Feedback System (4 hours)
1. **Implement ProgressManager** with multiple progress styles
2. **Add spinner and progress bar** creation
3. **Create multi-phase progress** tracking
4. **Add context-aware progress** selection

#### Step 3: Dataflow Operations (5 hours)
1. **Implement dataflow starting** with all phases
2. **Add graceful shutdown** logic for stop command
3. **Create error handling** and cleanup procedures
4. **Add timeout and force-stop** capabilities

#### Step 4: Interactive Monitoring (3 hours)
1. **Integrate with TUI system** for complex operations
2. **Create startup monitoring** view
3. **Add user prompting** for monitoring options
4. **Implement background operation** management

#### Step 5: Testing and Polish (2 hours)
1. **Add comprehensive unit tests** for all scenarios
2. **Test with various dataflow complexities**
3. **Validate progress feedback** across different environments
4. **Test error handling** and recovery procedures

## 🔗 Dependencies
**Depends On:**
- Issue #001 (Hybrid Command Framework) - Command structure
- Issue #002 (Execution Context Detection) - Progress style selection
- Issue #003 (Interface Selection Engine) - TUI suggestions
- Issue #004 (Configuration System) - User preferences

**Blocks:** Issue #007-008 (other Docker-like commands)

## 🧪 Testing Requirements

### Unit Tests
```rust
#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_start_command_validation() {
        let cmd = StartCommand {
            dataflow_path: PathBuf::from("test.yml"),
            timeout: 300,
            ..Default::default()
        };
        
        assert!(cmd.validate().is_ok());
    }
    
    #[test]
    fn test_progress_style_selection() {
        let context_interactive = ExecutionContext::mock_interactive();
        let context_piped = ExecutionContext::mock_piped();
        
        let cmd = StartCommand::default();
        
        let style_interactive = cmd.determine_progress_style(&context_interactive);
        let style_piped = cmd.determine_progress_style(&context_piped);
        
        assert!(!matches!(style_interactive, ProgressStyle::None));
        assert!(matches!(style_piped, ProgressStyle::None));
    }
    
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
            target: "single-dataflow".to_string(),
            yes: true,
            ..Default::default()
        };
        
        // Should not require confirmation with --yes
        assert!(!cmd.requires_confirmation());
    }
}
```

## ✅ Definition of Done
- [ ] StartCommand and StopCommand implemented with all CLI options
- [ ] Progress feedback system provides appropriate visual feedback
- [ ] Complex operations suggest interactive monitoring when beneficial
- [ ] Graceful shutdown works correctly with proper timeout handling
- [ ] Error handling includes actionable recovery suggestions
- [ ] Operations are interruptible with proper cleanup
- [ ] Background operation monitoring integrates with TUI system
- [ ] Performance meets expectations for various dataflow sizes
- [ ] Comprehensive unit tests cover all scenarios
- [ ] Integration tests validate end-to-end operation workflows
- [ ] Manual testing confirms user experience quality
- [ ] Backward compatibility maintained with existing usage patterns

This enhanced start/stop system provides users with clear feedback and appropriate guidance while maintaining the simplicity and reliability expected from core dataflow management commands.