use eyre::Result;
use clap::{Args, ValueEnum};
use colored::Colorize;
use serde::{Deserialize, Serialize};
use std::io::{self, Write};
use std::path::PathBuf;
use std::time::{Duration, SystemTime};
use tabled::{Table, Tabled};

use crate::cli::{context::ExecutionContext, interface::InterfaceSelector, Command, OutputFormat};

/// Enhanced PS command for listing dataflows with smart hints
#[derive(Debug, Args, Clone)]
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

impl Default for PsCommand {
    fn default() -> Self {
        Self {
            all: false,
            filter: None,
            status: None,
            problems_only: false,
            sort: SortField::Name,
            reverse: false,
            format: None,
            detailed: false,
            watch: None,
            no_hints: false,
        }
    }
}

#[derive(Debug, Clone, ValueEnum)]
pub enum DataflowStatusFilter {
    Running,
    Stopped,
    Error,
    Starting,
    Warning,
}

#[derive(Debug, Clone, ValueEnum)]
pub enum SortField {
    Name,
    Status,
    Uptime,
    Nodes,
    Cpu,
    Memory,
    Created,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum DataflowStatus {
    Running,
    Stopped,
    Error,
    Starting,
    Warning,
}

impl std::fmt::Display for DataflowStatus {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            DataflowStatus::Running => write!(f, "running"),
            DataflowStatus::Stopped => write!(f, "stopped"),
            DataflowStatus::Error => write!(f, "error"),
            DataflowStatus::Starting => write!(f, "starting"),
            DataflowStatus::Warning => write!(f, "warning"),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
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

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NodeSummary {
    pub total: usize,
    pub running: usize,
    pub failed: usize,
    pub warning: usize,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ResourceUsage {
    pub cpu_percent: f32,
    pub memory_mb: u64,
    pub network_mb_per_sec: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Issue {
    pub severity: IssueSeverity,
    pub message: String,
    pub node: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum IssueSeverity {
    Error,
    Warning,
    Info,
}

#[derive(Debug, Clone, Tabled)]
pub struct DataflowTableRow {
    #[tabled(rename = "NAME")]
    pub name: String,
    #[tabled(rename = "STATUS")]
    pub status: String,
    #[tabled(rename = "UPTIME")]
    pub uptime: String,
    #[tabled(rename = "NODES")]
    pub nodes: String,
    #[tabled(rename = "CPU")]
    pub cpu: String,
    #[tabled(rename = "MEMORY")]
    pub memory: String,
    #[tabled(rename = "ISSUES")]
    pub issues: String,
}

#[derive(Debug, Clone, Tabled)]
pub struct DataflowCompactRow {
    #[tabled(rename = "NAME")]
    pub name: String,
    #[tabled(rename = "STATUS")]
    pub status: String,
    #[tabled(rename = "UPTIME")]
    pub uptime: String,
    #[tabled(rename = "NODES")]
    pub nodes: String,
}

impl PsCommand {
    pub async fn execute(&self, context: &ExecutionContext) -> Result<()> {
        // Check if this is a watch mode command
        if let Some(interval) = self.watch {
            if interval > 0 {
                return self.execute_watch_mode(context, interval).await;
            }
        }

        // Collect dataflow information
        let dataflows: Vec<DataflowInfo> = self.collect_dataflow_info().await?;
        let filtered_dataflows: Vec<DataflowInfo> = self.apply_filters(&dataflows);
        let sorted_dataflows: Vec<DataflowInfo> = self.apply_sorting(filtered_dataflows);
        
        // Determine interface strategy using configuration and context
        let user_config = crate::cli::interface::UserConfig::default();
        let mut interface_selector = InterfaceSelector::new(context.clone(), user_config);
        let decision = interface_selector.select_interface(&Command::Ps(self.clone()));
        
        match decision.strategy {
            crate::cli::interface::InterfaceStrategy::CliOnly => {
                self.render_cli_output(&sorted_dataflows, context).await?;
            },
            
            crate::cli::interface::InterfaceStrategy::CliWithHint { hint, tui_command } => {
                self.render_cli_output(&sorted_dataflows, context).await?;
                if !self.no_hints {
                    self.show_hint(&hint, &tui_command);
                }
            },
            
            crate::cli::interface::InterfaceStrategy::PromptForTui { reason, default_yes } => {
                self.render_cli_output(&sorted_dataflows, context).await?;
                if !self.no_hints && self.should_prompt_for_tui(&reason, default_yes)? {
                    self.launch_tui_dashboard().await?;
                }
            },
            
            crate::cli::interface::InterfaceStrategy::AutoLaunchTui { reason, show_cli_first } => {
                if show_cli_first {
                    self.render_cli_output(&sorted_dataflows, context).await?;
                    println!("\n🚀 {}", reason);
                }
                self.launch_tui_dashboard().await?;
            },
        }
        
        Ok(())
    }

    async fn execute_watch_mode(&self, context: &ExecutionContext, interval: u64) -> Result<()> {
        println!("📊 Watching dataflows (refresh every {} seconds, press Ctrl+C to exit)...", interval);
        println!();

        // For now, just run once instead of infinite loop to avoid compilation issues
        // In a real implementation, this would use proper signal handling
        let iterations = 5; // Run 5 iterations as a demo
        for i in 0..iterations {
            if i > 0 {
                // Clear screen and show current time
                print!("\x1B[2J\x1B[1;1H");
            }
            
            println!("📊 Dora Dataflows - {} (refresh {}/{})", 
                chrono::Local::now().format("%Y-%m-%d %H:%M:%S"),
                i + 1,
                iterations
            );
            println!();

            // Collect and display dataflows
            let dataflows: Vec<DataflowInfo> = self.collect_dataflow_info().await?;
            let filtered_dataflows: Vec<DataflowInfo> = self.apply_filters(&dataflows);
            let sorted_dataflows: Vec<DataflowInfo> = self.apply_sorting(filtered_dataflows);
            
            self.render_cli_output(&sorted_dataflows, context).await?;

            if i < iterations - 1 {
                // Wait for next iteration
                tokio::time::sleep(Duration::from_secs(interval)).await;
            }
        }

        println!("Watch mode complete (demo version - limited to {} iterations)", iterations);
        Ok(())
    }

    async fn collect_dataflow_info(&self) -> Result<Vec<DataflowInfo>> {
        // For now, create mock data since we don't have daemon integration yet
        // In a real implementation, this would connect to the daemon and collect actual data
        let dataflows: Vec<DataflowInfo> = self.create_mock_dataflows();
        Ok(dataflows)
    }

    fn create_mock_dataflows(&self) -> Vec<DataflowInfo> {
        vec![
            DataflowInfo {
                name: "chat-demo".to_string(),
                status: DataflowStatus::Running,
                uptime: Duration::from_secs(3600), // 1 hour
                nodes: NodeSummary {
                    total: 5,
                    running: 5,
                    failed: 0,
                    warning: 0,
                },
                resource_usage: ResourceUsage {
                    cpu_percent: 12.3,
                    memory_mb: 256,
                    network_mb_per_sec: 0.5,
                },
                created: SystemTime::now() - Duration::from_secs(3600),
                config_path: Some(PathBuf::from("examples/chat/dataflow.yml")),
                issues: vec![],
            },
            DataflowInfo {
                name: "object-detection".to_string(),
                status: DataflowStatus::Running,
                uptime: Duration::from_secs(1800), // 30 minutes
                nodes: NodeSummary {
                    total: 8,
                    running: 7,
                    failed: 1,
                    warning: 0,
                },
                resource_usage: ResourceUsage {
                    cpu_percent: 85.7,
                    memory_mb: 2048,
                    network_mb_per_sec: 12.4,
                },
                created: SystemTime::now() - Duration::from_secs(1800),
                config_path: Some(PathBuf::from("examples/vision/object_detection.yml")),
                issues: vec![
                    Issue {
                        severity: IssueSeverity::Error,
                        message: "Node 'camera_input' failed: Device not found".to_string(),
                        node: Some("camera_input".to_string()),
                    },
                    Issue {
                        severity: IssueSeverity::Warning,
                        message: "High CPU usage: 85.7%".to_string(),
                        node: None,
                    },
                ],
            },
            DataflowInfo {
                name: "simple-test".to_string(),
                status: DataflowStatus::Stopped,
                uptime: Duration::from_secs(0),
                nodes: NodeSummary {
                    total: 2,
                    running: 0,
                    failed: 0,
                    warning: 0,
                },
                resource_usage: ResourceUsage {
                    cpu_percent: 0.0,
                    memory_mb: 0,
                    network_mb_per_sec: 0.0,
                },
                created: SystemTime::now() - Duration::from_secs(7200),
                config_path: Some(PathBuf::from("tests/simple.yml")),
                issues: vec![],
            },
        ]
    }

    fn apply_filters(&self, dataflows: &[DataflowInfo]) -> Vec<DataflowInfo> {
        let mut filtered = dataflows.to_vec();

        // Filter by status
        if let Some(status_filter) = &self.status {
            filtered.retain(|df| {
                match status_filter {
                    DataflowStatusFilter::Running => df.status == DataflowStatus::Running,
                    DataflowStatusFilter::Stopped => df.status == DataflowStatus::Stopped,
                    DataflowStatusFilter::Error => df.status == DataflowStatus::Error,
                    DataflowStatusFilter::Starting => df.status == DataflowStatus::Starting,
                    DataflowStatusFilter::Warning => df.status == DataflowStatus::Warning,
                }
            });
        }

        // Filter by name pattern
        if let Some(filter) = &self.filter {
            let pattern = filter.to_lowercase();
            filtered.retain(|df| {
                if pattern.contains('*') {
                    // Simple glob matching
                    let pattern = pattern.replace('*', "");
                    df.name.to_lowercase().contains(&pattern)
                } else {
                    df.name.to_lowercase().contains(&pattern)
                }
            });
        }

        // Filter by problems only
        if self.problems_only {
            filtered.retain(|df| !df.issues.is_empty());
        }

        // Don't show stopped dataflows unless --all flag is used
        if !self.all {
            filtered.retain(|df| df.status != DataflowStatus::Stopped);
        }

        filtered
    }

    fn apply_sorting(&self, mut dataflows: Vec<DataflowInfo>) -> Vec<DataflowInfo> {
        dataflows.sort_by(|a, b| {
            let comparison = match self.sort {
                SortField::Name => a.name.cmp(&b.name),
                SortField::Status => a.status.to_string().cmp(&b.status.to_string()),
                SortField::Uptime => a.uptime.cmp(&b.uptime),
                SortField::Nodes => a.nodes.total.cmp(&b.nodes.total),
                SortField::Cpu => a.resource_usage.cpu_percent.partial_cmp(&b.resource_usage.cpu_percent).unwrap_or(std::cmp::Ordering::Equal),
                SortField::Memory => a.resource_usage.memory_mb.cmp(&b.resource_usage.memory_mb),
                SortField::Created => a.created.cmp(&b.created),
            };

            if self.reverse {
                comparison.reverse()
            } else {
                comparison
            }
        });

        dataflows
    }

    async fn render_cli_output(
        &self,
        dataflows: &[DataflowInfo],
        context: &ExecutionContext,
    ) -> Result<()> {
        let format = self.format.clone().unwrap_or_else(|| {
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
        
        // Choose table format based on width and detailed flag
        if self.detailed || terminal_width > 120 {
            self.render_detailed_table(dataflows, context)?;
        } else {
            self.render_compact_table(dataflows, context)?;
        }
        
        // Show summary information
        self.render_summary(dataflows, context);
        
        Ok(())
    }

    fn render_detailed_table(&self, dataflows: &[DataflowInfo], context: &ExecutionContext) -> Result<()> {
        let rows: Vec<DataflowTableRow> = dataflows.iter().map(|df| {
            DataflowTableRow {
                name: self.format_name(&df.name),
                status: self.format_status(&df.status, context),
                uptime: self.format_uptime(df.uptime, &df.status),
                nodes: self.format_nodes(&df.nodes),
                cpu: self.format_cpu(df.resource_usage.cpu_percent),
                memory: self.format_memory(df.resource_usage.memory_mb),
                issues: self.format_issues(&df.issues),
            }
        }).collect();

        let table = Table::new(rows)
            .with(tabled::settings::Style::rounded())
            .to_string();

        println!("{}", table);
        Ok(())
    }

    fn render_compact_table(&self, dataflows: &[DataflowInfo], context: &ExecutionContext) -> Result<()> {
        let rows: Vec<DataflowCompactRow> = dataflows.iter().map(|df| {
            DataflowCompactRow {
                name: self.format_name(&df.name),
                status: self.format_status(&df.status, context),
                uptime: self.format_uptime(df.uptime, &df.status),
                nodes: self.format_nodes(&df.nodes),
            }
        }).collect();

        let table = Table::new(rows)
            .with(tabled::settings::Style::rounded())
            .to_string();

        println!("{}", table);
        Ok(())
    }

    fn format_name(&self, name: &str) -> String {
        if name.len() > 25 {
            format!("{}…", &name[..22])
        } else {
            name.to_string()
        }
    }

    fn format_status(&self, status: &DataflowStatus, context: &ExecutionContext) -> String {
        let (symbol, color) = match status {
            DataflowStatus::Running => ("●", "green"),
            DataflowStatus::Stopped => ("○", "white"),
            DataflowStatus::Error => ("✗", "red"),
            DataflowStatus::Starting => ("◐", "yellow"),
            DataflowStatus::Warning => ("⚠", "yellow"),
        };

        if context.terminal_capabilities.supports_color {
            match color {
                "green" => format!("{} {}", symbol.green(), status.to_string().green()),
                "red" => format!("{} {}", symbol.red(), status.to_string().red()),
                "yellow" => format!("{} {}", symbol.yellow(), status.to_string().yellow()),
                _ => format!("{} {}", symbol, status),
            }
        } else {
            format!("{} {}", symbol, status)
        }
    }

    fn format_uptime(&self, uptime: Duration, status: &DataflowStatus) -> String {
        if *status == DataflowStatus::Stopped {
            "-".to_string()
        } else {
            format_duration(uptime)
        }
    }

    fn format_nodes(&self, nodes: &NodeSummary) -> String {
        if nodes.failed > 0 {
            format!("{}/{} ({}❌)", nodes.running, nodes.total, nodes.failed)
        } else if nodes.warning > 0 {
            format!("{}/{} ({}⚠)", nodes.running, nodes.total, nodes.warning)
        } else {
            format!("{}/{}", nodes.running, nodes.total)
        }
    }

    fn format_cpu(&self, cpu_percent: f32) -> String {
        if cpu_percent > 0.0 {
            format!("{:.1}%", cpu_percent)
        } else {
            "-".to_string()
        }
    }

    fn format_memory(&self, memory_mb: u64) -> String {
        if memory_mb > 0 {
            format_memory(memory_mb)
        } else {
            "-".to_string()
        }
    }

    fn format_issues(&self, issues: &[Issue]) -> String {
        match issues.len() {
            0 => "-".to_string(),
            1 => "1 issue".to_string(),
            n => format!("{} issues", n),
        }
    }

    fn render_json(&self, dataflows: &[DataflowInfo]) -> Result<()> {
        let json = serde_json::to_string_pretty(&dataflows)?;
        println!("{}", json);
        Ok(())
    }

    fn render_yaml(&self, dataflows: &[DataflowInfo]) -> Result<()> {
        let yaml = serde_yaml::to_string(&dataflows)?;
        println!("{}", yaml);
        Ok(())
    }

    fn render_minimal(&self, dataflows: &[DataflowInfo]) -> Result<()> {
        for dataflow in dataflows {
            println!("{}\t{}\t{}", 
                dataflow.name, 
                dataflow.status,
                if dataflow.status == DataflowStatus::Stopped { 
                    "-".to_string() 
                } else { 
                    format_duration(dataflow.uptime) 
                }
            );
        }
        Ok(())
    }

    fn render_summary(&self, dataflows: &[DataflowInfo], context: &ExecutionContext) {
        let total = dataflows.len();
        let running = dataflows.iter().filter(|d| d.status == DataflowStatus::Running).count();
        let stopped = dataflows.iter().filter(|d| d.status == DataflowStatus::Stopped).count();
        let errors = dataflows.iter().filter(|d| d.status == DataflowStatus::Error).count();
        let warnings = dataflows.iter().filter(|d| d.status == DataflowStatus::Warning).count();
        
        println!();
        print!("Summary: {} dataflows ({} running", total, running);
        if stopped > 0 {
            print!(", {} stopped", stopped);
        }
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
                    if context.terminal_capabilities.supports_color {
                        println!("  {} {}: {}", 
                            severity_symbol, 
                            dataflow.name.cyan(), 
                            issue.message.yellow()
                        );
                    } else {
                        println!("  {} {}: {}", severity_symbol, dataflow.name, issue.message);
                    }
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

    fn show_hint(&self, hint: &str, tui_command: &str) {
        println!();
        println!("💡 {}", hint.bright_blue());
        println!("   Try: {}", tui_command.cyan());
    }
    
    fn should_prompt_for_tui(&self, reason: &str, default_yes: bool) -> Result<bool> {
        println!();
        println!("🔍 {}", reason.bright_blue());
        
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
        println!("🚀 Launching Dora dashboard...");
        
        // For now, just simulate TUI launch
        // In a real implementation, this would launch the TUI dashboard
        println!("💡 TUI dashboard would launch here (not yet implemented)");
        
        Ok(())
    }
}

// Utility functions

fn format_duration(duration: Duration) -> String {
    let total_seconds = duration.as_secs();
    
    if total_seconds < 60 {
        format!("{}s", total_seconds)
    } else if total_seconds < 3600 {
        let minutes = total_seconds / 60;
        let seconds = total_seconds % 60;
        if seconds == 0 {
            format!("{}m", minutes)
        } else {
            format!("{}m{}s", minutes, seconds)
        }
    } else if total_seconds < 86400 {
        let hours = total_seconds / 3600;
        let minutes = (total_seconds % 3600) / 60;
        if minutes == 0 {
            format!("{}h", hours)
        } else {
            format!("{}h{}m", hours, minutes)
        }
    } else {
        let days = total_seconds / 86400;
        let hours = (total_seconds % 86400) / 3600;
        if hours == 0 {
            format!("{}d", days)
        } else {
            format!("{}d{}h", days, hours)
        }
    }
}

fn format_memory(memory_mb: u64) -> String {
    if memory_mb < 1024 {
        format!("{}MB", memory_mb)
    } else {
        let gb = memory_mb as f64 / 1024.0;
        format!("{:.1}GB", gb)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_format_duration() {
        assert_eq!(format_duration(Duration::from_secs(30)), "30s");
        assert_eq!(format_duration(Duration::from_secs(90)), "1m30s");
        assert_eq!(format_duration(Duration::from_secs(3600)), "1h");
        assert_eq!(format_duration(Duration::from_secs(3661)), "1h1m");
        assert_eq!(format_duration(Duration::from_secs(86400)), "1d");
        assert_eq!(format_duration(Duration::from_secs(90000)), "1d1h");
    }

    #[test]
    fn test_format_memory() {
        assert_eq!(format_memory(512), "512MB");
        assert_eq!(format_memory(1024), "1.0GB");
        assert_eq!(format_memory(1536), "1.5GB");
        assert_eq!(format_memory(2048), "2.0GB");
    }

    #[test]
    fn test_dataflow_filtering() {
        let cmd = PsCommand {
            status: Some(DataflowStatusFilter::Running),
            ..Default::default()
        };
        
        let dataflows = cmd.create_mock_dataflows();
        let filtered = cmd.apply_filters(&dataflows);
        
        // Should only show running dataflows
        assert!(filtered.iter().all(|df| df.status == DataflowStatus::Running));
    }

    #[test]
    fn test_dataflow_sorting() {
        let cmd = PsCommand {
            sort: SortField::Name,
            reverse: false,
            ..Default::default()
        };
        
        let dataflows = cmd.create_mock_dataflows();
        let sorted = cmd.apply_sorting(dataflows);
        
        // Should be sorted by name alphabetically
        assert_eq!(sorted[0].name, "chat-demo");
        assert_eq!(sorted[1].name, "object-detection");
        assert_eq!(sorted[2].name, "simple-test");
    }

    #[test]
    fn test_problems_only_filter() {
        let cmd = PsCommand {
            problems_only: true,
            all: true, // Include stopped dataflows
            ..Default::default()
        };
        
        let dataflows = cmd.create_mock_dataflows();
        let filtered = cmd.apply_filters(&dataflows);
        
        // Should only show dataflows with issues
        assert!(filtered.iter().all(|df| !df.issues.is_empty()));
    }
}