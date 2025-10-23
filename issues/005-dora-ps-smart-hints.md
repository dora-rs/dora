# Issue #005: Implement `dora ps` with Smart Hints

## 📋 Summary
Enhance the `dora ps` command to provide Docker-like process listing with intelligent TUI suggestions. This command serves as the primary entry point for system overview and demonstrates the hybrid CLI approach with helpful hints when appropriate.

## 🎯 Objectives
- Create fast, informative process listing similar to `docker ps`
- Implement smart TUI suggestions based on system complexity
- Add filtering, sorting, and formatting options
- Provide actionable hints for common follow-up operations
- Ensure excellent performance even with many dataflows

**Success Metrics:**
- Command completes in <200ms for up to 100 dataflows
- TUI suggestions appear appropriately based on context
- Output is clear and actionable for both beginners and experts
- Filtering and sorting work intuitively
- 100% backward compatibility with existing `dora list` usage

## 🛠️ Technical Requirements

### What to Build

#### 1. Enhanced Ps Command Structure
```rust
// src/cli/commands/ps.rs
#[derive(Debug, clap::Args)]
pub struct PsCommand {
    /// Show all dataflows (including stopped)
    #[clap(short, long)]
    pub all: bool,
    
    /// Filter by dataflow name pattern
    #[clap(short, long)]
    pub filter: Option<String>,
    
    /// Filter by status
    #[clap(long, value_enum)]
    pub status: Option<DataflowStatusFilter>,
    
    /// Show only dataflows with issues
    #[clap(long)]
    pub problems_only: bool,
    
    /// Sort by field
    #[clap(long, value_enum, default_value = "name")]
    pub sort: SortField,
    
    /// Reverse sort order
    #[clap(long)]
    pub reverse: bool,
    
    /// Output format
    #[clap(long, value_enum)]
    pub format: Option<OutputFormat>,
    
    /// Show detailed view with more columns
    #[clap(long)]
    pub detailed: bool,
    
    /// Refresh every N seconds (0 to disable)
    #[clap(long)]
    pub watch: Option<u64>,
    
    /// Suppress hints and suggestions
    #[clap(long)]
    pub no_hints: bool,
}

#[derive(Debug, Clone, clap::ValueEnum)]
pub enum DataflowStatusFilter {
    Running,
    Stopped,
    Error,
    Starting,
    Warning,
}

#[derive(Debug, Clone, clap::ValueEnum)]
pub enum SortField {
    Name,
    Status,
    Uptime,
    Nodes,
    Cpu,
    Memory,
    Created,
}

impl PsCommand {
    pub async fn execute(&self, context: &ExecutionContext) -> Result<()> {
        let dataflows = self.collect_dataflow_info().await?;
        let filtered_dataflows = self.apply_filters(&dataflows);
        let sorted_dataflows = self.apply_sorting(filtered_dataflows);
        
        // Determine interface strategy
        let interface_selector = InterfaceSelector::new(context.clone(), UserConfig::load()?);
        let decision = interface_selector.select_interface(&Command::Ps(self.clone()));
        
        match decision.strategy {
            InterfaceStrategy::CliOnly => {
                self.render_cli_output(&sorted_dataflows, context).await?;
            },
            
            InterfaceStrategy::CliWithHint { hint, tui_command } => {
                self.render_cli_output(&sorted_dataflows, context).await?;
                if !self.no_hints {
                    self.show_hint(&hint, &tui_command);
                }
            },
            
            InterfaceStrategy::PromptForTui { reason, default_yes } => {
                self.render_cli_output(&sorted_dataflows, context).await?;
                if !self.no_hints && self.should_prompt_for_tui(&reason, default_yes)? {
                    self.launch_tui_dashboard().await?;
                }
            },
            
            InterfaceStrategy::AutoLaunchTui { reason, show_cli_first } => {
                if show_cli_first {
                    self.render_cli_output(&sorted_dataflows, context).await?;
                    println!("\n🚀 {}", reason);
                }
                self.launch_tui_dashboard().await?;
            },
        }
        
        Ok(())
    }
}
```

#### 2. Data Collection and Processing
```rust
#[derive(Debug, Clone)]
pub struct DataflowInfo {
    pub name: String,
    pub status: DataflowStatus,
    pub uptime: Duration,
    pub nodes: NodeSummary,
    pub resource_usage: ResourceUsage,
    pub created: SystemTime,
    pub config_path: Option<PathBuf>,
    pub issues: Vec<Issue>,
}

#[derive(Debug, Clone)]
pub struct NodeSummary {
    pub total: usize,
    pub running: usize,
    pub failed: usize,
    pub warning: usize,
}

#[derive(Debug, Clone)]
pub struct ResourceUsage {
    pub cpu_percent: f32,
    pub memory_mb: u64,
    pub network_mb_per_sec: f32,
}

#[derive(Debug, Clone)]
pub struct Issue {
    pub severity: IssueSeverity,
    pub message: String,
    pub node: Option<String>,
}

impl PsCommand {
    async fn collect_dataflow_info(&self) -> Result<Vec<DataflowInfo>> {
        let daemon_client = DaemonClient::connect().await?;
        
        // Collect basic dataflow information
        let dataflows = daemon_client.list_dataflows().await?;
        let mut dataflow_info = Vec::new();
        
        for dataflow in dataflows {
            // Get detailed information for each dataflow
            let status = daemon_client.get_dataflow_status(&dataflow.id).await?;
            let nodes = daemon_client.get_dataflow_nodes(&dataflow.id).await?;
            let metrics = daemon_client.get_dataflow_metrics(&dataflow.id).await?;
            
            let node_summary = NodeSummary {
                total: nodes.len(),
                running: nodes.iter().filter(|n| n.status == NodeStatus::Running).count(),
                failed: nodes.iter().filter(|n| n.status == NodeStatus::Failed).count(),
                warning: nodes.iter().filter(|n| n.status == NodeStatus::Warning).count(),
            };
            
            let resource_usage = ResourceUsage {
                cpu_percent: metrics.cpu_usage,
                memory_mb: metrics.memory_usage_mb,
                network_mb_per_sec: metrics.network_throughput_mb_per_sec,
            };
            
            // Analyze for issues
            let issues = self.analyze_dataflow_issues(&status, &nodes, &metrics);
            
            dataflow_info.push(DataflowInfo {
                name: dataflow.name,
                status: status.status,
                uptime: status.uptime,
                nodes: node_summary,
                resource_usage,
                created: dataflow.created_at,
                config_path: dataflow.config_path,
                issues,
            });
        }
        
        Ok(dataflow_info)
    }
    
    fn analyze_dataflow_issues(
        &self,
        status: &DataflowStatus,
        nodes: &[NodeInfo],
        metrics: &DataflowMetrics,
    ) -> Vec<Issue> {
        let mut issues = Vec::new();
        
        // Check for high resource usage
        if metrics.cpu_usage > 80.0 {
            issues.push(Issue {
                severity: IssueSeverity::Warning,
                message: format!("High CPU usage: {:.1}%", metrics.cpu_usage),
                node: None,
            });
        }
        
        if metrics.memory_usage_mb > 2048 {
            issues.push(Issue {
                severity: IssueSeverity::Warning,
                message: format!("High memory usage: {:.1} MB", metrics.memory_usage_mb),
                node: None,
            });
        }
        
        // Check for failed nodes
        for node in nodes {
            if node.status == NodeStatus::Failed {
                issues.push(Issue {
                    severity: IssueSeverity::Error,
                    message: format!("Node failed: {}", node.error.as_deref().unwrap_or("Unknown error")),
                    node: Some(node.name.clone()),
                });
            }
        }
        
        // Check for performance issues
        for node in nodes {
            if let Some(latency) = node.average_latency {
                if latency > Duration::from_millis(100) {
                    issues.push(Issue {
                        severity: IssueSeverity::Warning,
                        message: format!("High latency: {:.1}ms", latency.as_millis()),
                        node: Some(node.name.clone()),
                    });
                }
            }
        }
        
        issues
    }
}
```

#### 3. Smart Output Rendering
```rust
impl PsCommand {
    async fn render_cli_output(
        &self,
        dataflows: &[DataflowInfo],
        context: &ExecutionContext,
    ) -> Result<()> {
        let format = self.format.unwrap_or_else(|| {
            if context.is_tty && !context.is_piped {
                OutputFormat::Table
            } else {
                OutputFormat::Minimal
            }
        });
        
        match format {
            OutputFormat::Table => self.render_table(dataflows, context).await?,
            OutputFormat::Json => self.render_json(dataflows)?,
            OutputFormat::Yaml => self.render_yaml(dataflows)?,
            OutputFormat::Minimal => self.render_minimal(dataflows)?,
            OutputFormat::Auto => self.render_table(dataflows, context).await?,
        }
        
        Ok(())
    }
    
    async fn render_table(&self, dataflows: &[DataflowInfo], context: &ExecutionContext) -> Result<()> {
        if dataflows.is_empty() {
            self.render_empty_state();
            return Ok(());
        }
        
        let terminal_width = context.terminal_size
            .map(|(w, _)| w as usize)
            .unwrap_or(80);
        
        // Create table with appropriate columns based on terminal width
        let columns = if self.detailed || terminal_width > 120 {
            self.get_detailed_columns()
        } else {
            self.get_compact_columns()
        };
        
        let mut table = Table::new(columns);
        table.set_style(self.get_table_style(context));
        
        // Add header
        let header = self.create_table_header(&columns);
        table.add_row(header);
        
        // Add separator
        table.add_separator();
        
        // Add dataflow rows
        for dataflow in dataflows {
            let row = self.create_dataflow_row(dataflow, &columns, context);
            table.add_row(row);
        }
        
        println!("{}", table);
        
        // Show summary information
        self.render_summary(dataflows);
        
        Ok(())
    }
    
    fn create_dataflow_row(
        &self,
        dataflow: &DataflowInfo,
        columns: &[TableColumn],
        context: &ExecutionContext,
    ) -> Vec<String> {
        columns.iter().map(|column| {
            match column {
                TableColumn::Name => {
                    let mut name = dataflow.name.clone();
                    if dataflow.name.len() > 20 {
                        name = format!("{}…", &dataflow.name[..17]);
                    }
                    name
                },
                
                TableColumn::Status => {
                    let status_symbol = match dataflow.status {
                        DataflowStatus::Running => "●",
                        DataflowStatus::Stopped => "○", 
                        DataflowStatus::Error => "✗",
                        DataflowStatus::Starting => "◐",
                        DataflowStatus::Warning => "⚠",
                    };
                    
                    if context.terminal_capabilities.supports_color {
                        let color = match dataflow.status {
                            DataflowStatus::Running => "\x1b[32m", // Green
                            DataflowStatus::Error => "\x1b[31m",   // Red
                            DataflowStatus::Warning => "\x1b[33m", // Yellow
                            _ => "\x1b[37m",                       // White
                        };
                        format!("{}{}\x1b[0m {}", color, status_symbol, dataflow.status)
                    } else {
                        format!("{} {}", status_symbol, dataflow.status)
                    }
                },
                
                TableColumn::Uptime => {
                    if dataflow.status == DataflowStatus::Stopped {
                        "-".to_string()
                    } else {
                        format_duration(dataflow.uptime)
                    }
                },
                
                TableColumn::Nodes => {
                    if dataflow.nodes.failed > 0 {
                        format!("{}/{} ({}❌)", dataflow.nodes.running, dataflow.nodes.total, dataflow.nodes.failed)
                    } else if dataflow.nodes.warning > 0 {
                        format!("{}/{} ({}⚠)", dataflow.nodes.running, dataflow.nodes.total, dataflow.nodes.warning)
                    } else {
                        format!("{}/{}", dataflow.nodes.running, dataflow.nodes.total)
                    }
                },
                
                TableColumn::Cpu => {
                    if dataflow.resource_usage.cpu_percent > 0.0 {
                        format!("{:.1}%", dataflow.resource_usage.cpu_percent)
                    } else {
                        "-".to_string()
                    }
                },
                
                TableColumn::Memory => {
                    if dataflow.resource_usage.memory_mb > 0 {
                        format_memory(dataflow.resource_usage.memory_mb)
                    } else {
                        "-".to_string()
                    }
                },
                
                TableColumn::Issues => {
                    match dataflow.issues.len() {
                        0 => "-".to_string(),
                        1 => "1 issue".to_string(),
                        n => format!("{} issues", n),
                    }
                },
            }
        }).collect()
    }
    
    fn render_summary(&self, dataflows: &[DataflowInfo]) {
        let total = dataflows.len();
        let running = dataflows.iter().filter(|d| d.status == DataflowStatus::Running).count();
        let stopped = dataflows.iter().filter(|d| d.status == DataflowStatus::Stopped).count();
        let errors = dataflows.iter().filter(|d| d.status == DataflowStatus::Error).count();
        let warnings = dataflows.iter().filter(|d| d.status == DataflowStatus::Warning).count();
        
        println!();
        println!("Summary: {} dataflows ({} running, {} stopped", total, running, stopped);
        if errors > 0 || warnings > 0 {
            print!(", {} errors, {} warnings", errors, warnings);
        }
        println!(")");
        
        // Show problematic dataflows if any
        let problem_dataflows: Vec<_> = dataflows.iter()
            .filter(|d| !d.issues.is_empty())
            .collect();
        
        if !problem_dataflows.is_empty() && problem_dataflows.len() <= 3 {
            println!("\nIssues detected:");
            for dataflow in problem_dataflows {
                for issue in &dataflow.issues {
                    let severity_symbol = match issue.severity {
                        IssueSeverity::Error => "❌",
                        IssueSeverity::Warning => "⚠️",
                        IssueSeverity::Info => "ℹ️",
                    };
                    println!("  {} {}: {}", severity_symbol, dataflow.name, issue.message);
                }
            }
        }
    }
    
    fn render_empty_state(&self) {
        println!("No dataflows found.");
        println!();
        println!("To get started:");
        println!("  dora start <dataflow.yml>    # Start a dataflow");
        println!("  dora ui                      # Launch interactive dashboard");
        println!("  dora --help                  # See all available commands");
    }
}
```

#### 4. Smart Hint System
```rust
impl PsCommand {
    fn show_hint(&self, hint: &str, tui_command: &str) {
        println!();
        println!("💡 {}", hint);
        println!("   Try: {}", tui_command);
    }
    
    fn should_prompt_for_tui(&self, reason: &str, default_yes: bool) -> Result<bool> {
        println!();
        println!("🔍 {}", reason);
        
        let prompt = if default_yes {
            "Launch interactive dashboard? [Y/n]: "
        } else {
            "Launch interactive dashboard? [y/N]: "
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
    
    async fn launch_tui_dashboard(&self) -> Result<()> {
        println!("Launching Dora dashboard...");
        
        // Launch TUI with dashboard view
        let tui_app = DoraApp::new(ViewType::Dashboard);
        tui_app.run().await?;
        
        Ok(())
    }
}

// Smart hint generation based on dataflow complexity and issues
impl InterfaceSelector {
    fn generate_ps_hint(&self, dataflows: &[DataflowInfo]) -> Option<String> {
        let total_dataflows = dataflows.len();
        let problem_dataflows = dataflows.iter().filter(|d| !d.issues.is_empty()).count();
        let complex_dataflows = dataflows.iter()
            .filter(|d| d.nodes.total > 5 || d.resource_usage.cpu_percent > 50.0)
            .count();
        
        match (total_dataflows, problem_dataflows, complex_dataflows) {
            (0, _, _) => None,
            (1..=2, 0, 0) => None, // Simple case, no hint needed
            (_, p, _) if p > 0 => Some(format!(
                "{} dataflow{} {} issues. Interactive dashboard provides detailed diagnostics",
                p,
                if p == 1 { " has" } else { "s have" },
                if p == 1 { "some" } else { "multiple" }
            )),
            (_, _, c) if c > 2 => Some(format!(
                "Complex dataflows detected. Dashboard view shows real-time metrics and topology"
            )),
            (n, _, _) if n > 5 => Some(format!(
                "Many dataflows active. Dashboard provides better overview and management"
            )),
            _ => None,
        }
    }
}
```

### Why This Approach

**Docker-like Familiarity:**
- Intuitive command structure that feels familiar to container users
- Consistent flag naming and behavior patterns
- Clear, scannable output format

**Progressive Complexity:**
- Simple output for basic scenarios
- Smart suggestions when complexity increases
- Detailed views available when needed
- Never overwhelming for beginners

**Performance Optimized:**
- Parallel data collection for fast response
- Efficient filtering and sorting
- Minimal API calls through batching
- Responsive even with many dataflows

### How to Implement

#### Step 1: Command Structure and Options (2 hours)
1. **Define PsCommand struct** with all CLI options
2. **Implement argument parsing** with clap derive
3. **Add filtering and sorting enums** with proper validation
4. **Create basic command execution** framework

#### Step 2: Data Collection System (4 hours)
1. **Implement dataflow information gathering** from daemon
2. **Add parallel collection** for performance
3. **Create issue analysis system** for problem detection
4. **Add resource usage calculation** and aggregation

#### Step 3: Output Rendering (5 hours)
1. **Implement table rendering** with dynamic column layout
2. **Add JSON/YAML output** for machine consumption
3. **Create minimal output** for piped usage
4. **Add responsive design** based on terminal width

#### Step 4: Smart Hint Integration (3 hours)
1. **Integrate with interface selector** from Issue #003
2. **Implement hint generation** based on dataflow complexity
3. **Add TUI launching** capability
4. **Create prompt handling** for user interaction

#### Step 5: Testing and Polish (2 hours)
1. **Add comprehensive unit tests** for all functionality
2. **Test with various dataflow scenarios** (empty, simple, complex, problematic)
3. **Validate performance** with large numbers of dataflows
4. **Test output formats** and terminal compatibility

## 🔗 Dependencies
**Depends On:**
- Issue #001 (Hybrid Command Framework) - Required for CLI structure
- Issue #002 (Execution Context Detection) - Required for smart interface selection
- Issue #003 (Interface Selection Engine) - Required for TUI suggestions
- Issue #004 (Configuration System) - Required for user preferences

**Blocks:** Issues #006-008 (other Docker-like commands) - Establishes patterns

## 🧪 Testing Requirements

### Unit Tests
```rust
#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_dataflow_filtering() {
        let dataflows = vec![
            create_test_dataflow("running-df", DataflowStatus::Running),
            create_test_dataflow("stopped-df", DataflowStatus::Stopped),
            create_test_dataflow("error-df", DataflowStatus::Error),
        ];
        
        let cmd = PsCommand {
            status: Some(DataflowStatusFilter::Running),
            ..Default::default()
        };
        
        let filtered = cmd.apply_filters(&dataflows);
        assert_eq!(filtered.len(), 1);
        assert_eq!(filtered[0].name, "running-df");
    }
    
    #[test]
    fn test_dataflow_sorting() {
        let mut dataflows = vec![
            create_test_dataflow_with_uptime("df-c", Duration::from_secs(300)),
            create_test_dataflow_with_uptime("df-a", Duration::from_secs(100)),
            create_test_dataflow_with_uptime("df-b", Duration::from_secs(200)),
        ];
        
        let cmd = PsCommand {
            sort: SortField::Uptime,
            reverse: false,
            ..Default::default()
        };
        
        let sorted = cmd.apply_sorting(dataflows);
        assert_eq!(sorted[0].name, "df-a"); // Shortest uptime first
        assert_eq!(sorted[2].name, "df-c"); // Longest uptime last
    }
    
    #[test]
    fn test_issue_analysis() {
        let status = create_test_status();
        let nodes = vec![
            create_failed_node("failed-node", "Connection timeout"),
            create_healthy_node("healthy-node"),
        ];
        let metrics = create_high_cpu_metrics();
        
        let cmd = PsCommand::default();
        let issues = cmd.analyze_dataflow_issues(&status, &nodes, &metrics);
        
        assert!(issues.len() >= 2); // CPU warning + failed node
        assert!(issues.iter().any(|i| i.message.contains("High CPU")));
        assert!(issues.iter().any(|i| i.message.contains("Node failed")));
    }
}
```

### Integration Tests
```rust
// tests/ps_integration.rs
#[tokio::test]
async fn test_ps_command_execution() {
    let context = ExecutionContext::mock_interactive();
    let cmd = PsCommand::default();
    
    // Should execute without error
    let result = cmd.execute(&context).await;
    assert!(result.is_ok());
}

#[tokio::test]
async fn test_ps_different_output_formats() {
    let context = ExecutionContext::mock_non_interactive();
    
    // Test JSON output
    let cmd = PsCommand {
        format: Some(OutputFormat::Json),
        ..Default::default()
    };
    let result = cmd.execute(&context).await;
    assert!(result.is_ok());
    
    // Test minimal output
    let cmd = PsCommand {
        format: Some(OutputFormat::Minimal),
        ..Default::default()
    };
    let result = cmd.execute(&context).await;
    assert!(result.is_ok());
}
```

### Manual Testing Procedures
1. **Basic Functionality**
   ```bash
   # Test basic listing
   dora ps
   dora ps --all
   dora ps --detailed
   ```

2. **Filtering and Sorting**
   ```bash
   # Test filtering
   dora ps --status running
   dora ps --filter "demo*"
   dora ps --problems-only
   
   # Test sorting
   dora ps --sort uptime
   dora ps --sort memory --reverse
   ```

3. **Output Formats**
   ```bash
   # Test different formats
   dora ps --format json
   dora ps --format minimal
   dora ps | grep running  # Test piped output
   ```

4. **Smart Hints**
   ```bash
   # Create complex scenario and verify hints
   # Start multiple dataflows with issues
   # Run dora ps and check for appropriate suggestions
   ```

## 📚 Resources

### Docker CLI Reference
- [Docker ps command documentation](https://docs.docker.com/engine/reference/commandline/ps/)
- [Docker CLI design principles](https://docs.docker.com/engine/reference/commandline/cli/)

### Table Formatting Libraries
- [tabled crate](https://docs.rs/tabled/latest/tabled/) - Advanced table formatting
- [cli-table crate](https://docs.rs/cli-table/latest/cli_table/) - Simple table rendering

### Code Examples
```rust
// Example of smart hint generation
fn generate_complexity_hint(dataflows: &[DataflowInfo]) -> Option<String> {
    let complexity_score = dataflows.iter()
        .map(|df| calculate_dataflow_complexity(df))
        .sum::<u32>();
    
    match complexity_score {
        0..=10 => None,
        11..=30 => Some("Use 'dora ui' for enhanced monitoring".to_string()),
        31..=60 => Some("Complex system detected. Interactive dashboard recommended".to_string()),
        _ => Some("High complexity system. TUI provides optimal management experience".to_string()),
    }
}
```

## ✅ Definition of Done
- [ ] PsCommand struct implemented with all CLI options and flags
- [ ] Dataflow information collection works efficiently with daemon API
- [ ] Table rendering adapts to terminal width and capabilities
- [ ] Filtering and sorting work correctly for all supported fields
- [ ] Issue analysis detects common problems and performance issues
- [ ] Smart hint system provides appropriate TUI suggestions
- [ ] JSON/YAML output formats work for machine consumption
- [ ] Performance meets <200ms target for up to 100 dataflows
- [ ] Empty state provides helpful getting-started guidance
- [ ] Summary information shows relevant system overview
- [ ] Comprehensive unit tests cover all functionality
- [ ] Integration tests validate end-to-end command execution
- [ ] Manual testing confirms usability and hint appropriateness
- [ ] Backward compatibility maintained with existing `dora list` usage

## 📝 Implementation Notes

### Performance Optimizations
- Batch API calls to daemon for efficient data collection
- Use async/await for parallel dataflow information gathering
- Cache terminal capabilities detection for responsive output
- Optimize table rendering for large numbers of dataflows

### User Experience Considerations
- Provide clear visual indicators for different dataflow states
- Use consistent color coding across all output formats
- Make filtering and sorting intuitive with clear documentation
- Ensure hints are helpful without being intrusive

### Future Enhancements
- Add dataflow grouping by project or tag
- Implement watch mode with diff highlighting
- Add export functionality for dataflow lists
- Support custom column selection and ordering
- Integration with system monitoring for resource trends

This enhanced `dora ps` command serves as the foundation for the hybrid CLI experience, demonstrating how traditional command-line tools can be enhanced with intelligent suggestions while maintaining full compatibility and performance.