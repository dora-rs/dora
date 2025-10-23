# DORA CLI Overhaul: A Docker-like Experience

This document outlines a comprehensive plan for transforming the `dora-cli` into a world-class developer tool that rivals Docker's user experience. The design prioritizes developer productivity, discoverability, and operational excellence.

## 🎯 Vision & Design Philosophy

**Target Experience**: Developers should feel as comfortable with Dora CLI as they do with `docker`, `kubectl`, or `git` - tools that set the gold standard for CLI design.

**Core Principles (Applied throughout all phases)**

*   **🏗️ Object-Oriented Command Structure**: `dora <resource> <action> [options]` (e.g., `dora dataflow start`, `dora node inspect`)
*   **🧭 Predictable Command Patterns**: Consistent verbs across resources (`start`, `stop`, `list`, `inspect`, `logs`)
*   **🔍 Discoverability First**: Every command discoverable via `--help`, with examples and common usage patterns
*   **📊 Smart Output Formatting**: Human-readable by default, structured data with `--output json|yaml|table`
*   **⚡ Real-time Feedback**: Progress indicators, timing info, and live status updates for long operations
*   **🎯 Actionable Error Messages**: Every error includes suggested fixes, relevant docs, and error codes
*   **🔧 Developer-Centric**: Built for daily development workflows, not just deployment

## 🚀 Implementation Strategy

### Current State Analysis
```bash
# Current inconsistent command structure
dora up                              # daemon management
dora start dataflow.yml --name test  # dataflow management 
dora list                           # status checking
dora stop --name test               # lifecycle management
dora logs                           # debugging
```

### Target Command Structure
```bash
# Proposed consistent, discoverable structure
dora daemon start|stop|status       # daemon lifecycle
dora dataflow start|stop|list|inspect|logs  # dataflow management
dora node list|inspect|exec         # node operations
dora system info|status|prune       # system management
dora recording start|stop|replay    # data workflow
```

---

## 🟢 Phase 1: Core Developer Experience (4-6 weeks)

**Goal:** Transform basic dataflow operations into a polished, developer-friendly experience that eliminates current friction points.

### 1.1 Enhanced Dataflow Lifecycle Management

#### `dora dataflow` command suite
```bash
# Core lifecycle operations
dora dataflow start <file.yml> [--name NAME] [--detach] [--wait-timeout 30s]
dora dataflow stop <name|id> [--grace-period 10s] [--force]
dora dataflow restart <name|id> [--rolling] [--wait-timeout 30s]
dora dataflow list [--all] [--quiet] [--filter status=running] [--output table|json|yaml]
dora dataflow inspect <name|id> [--live] [--format detailed|compact]
dora dataflow remove <name|id> [--force] [--volumes]
dora dataflow prune [--dry-run] [--filter 'status=stopped,age>1h']

# Advanced operations
dora dataflow pause <name|id>      # Pause execution without stopping
dora dataflow resume <name|id>     # Resume paused dataflow
dora dataflow validate <file.yml>  # Validate without starting
dora dataflow template <template>  # Create from template
```

#### Implementation Details
```rust
// Enhanced command structure in src/command/dataflow.rs
#[derive(Debug, clap::Subcommand)]
pub enum DataflowCommand {
    Start(DataflowStart),
    Stop(DataflowStop),
    List(DataflowList),
    Inspect(DataflowInspect),
    // ... etc
}

#[derive(Debug, clap::Args)]
pub struct DataflowStart {
    /// Dataflow configuration file
    #[clap(value_name = "FILE")]
    pub dataflow: PathBuf,
    
    /// Name for the dataflow instance
    #[clap(long, short)]
    pub name: Option<String>,
    
    /// Run in background (detached mode)
    #[clap(long, short)]
    pub detach: bool,
    
    /// Output format for status
    #[clap(long, value_enum, default_value = "table")]
    pub output: OutputFormat,
}
```

### 1.2 Real-time Progress & Feedback System

#### Interactive Progress Display
```bash
❯ dora dataflow start complex-pipeline.yml --name demo

🔧 Loading configuration...                    ✅ (0.2s)
📦 Building nodes:                             
  ├─ camera-capture     ████████████████████   100% ✅ (1.2s)
  ├─ yolo-detection     ████████████████████   100% ✅ (2.1s)  
  ├─ object-tracking    ████████████░░░░░░░░    60% 🔄 (3.2s)
  └─ visualization      ░░░░░░░░░░░░░░░░░░░░     0% ⏳ pending

🌐 Establishing connections:                   
  ├─ camera → yolo      ████████████████████   100% ✅ (0.3s)
  ├─ yolo → tracking    ████████████████████   100% ✅ (0.2s)
  └─ tracking → viz     ██████████░░░░░░░░░░    50% 🔄 (0.8s)

🚀 Starting dataflow 'demo'...                ✅ (0.5s)

✨ Dataflow 'demo' is running! 
💡 Use 'dora dataflow logs demo' to monitor output
📊 Use 'dora dataflow inspect demo' for detailed status
```

#### Implementation with indicatif
```rust
use indicatif::{MultiProgress, ProgressBar, ProgressStyle};

pub struct ProgressTracker {
    multi: MultiProgress,
    bars: HashMap<String, ProgressBar>,
}

impl ProgressTracker {
    pub fn new() -> Self {
        let multi = MultiProgress::new();
        Self {
            multi,
            bars: HashMap::new(),
        }
    }
    
    pub fn add_task(&mut self, name: &str, total: u64) {
        let pb = self.multi.add(ProgressBar::new(total));
        pb.set_style(
            ProgressStyle::default_bar()
                .template("{prefix:.bold} {bar:20.cyan/blue} {pos:>3}/{len:3} {msg}")
                .unwrap()
        );
        pb.set_prefix(name);
        self.bars.insert(name.to_string(), pb);
    }
    
    pub fn update_task(&self, name: &str, progress: u64, message: &str) {
        if let Some(pb) = self.bars.get(name) {
            pb.set_position(progress);
            pb.set_message(message.to_string());
        }
    }
}
```

### 1.3 Enhanced Error Messages with Error Codes

#### Before vs After Error Experience
```bash
# BEFORE: Cryptic and unhelpful
❯ dora start my-dataflow.yml
Error: failed to connect to daemon

# AFTER: Actionable and informative  
❯ dora dataflow start my-dataflow.yml
❌ DORA_E001: Failed to connect to daemon

🔍 Diagnosis:
  └─ No daemon process found on 127.0.0.1:53290

💡 Solutions:
  1. Start the daemon:     dora daemon start
  2. Check daemon status:  dora daemon status  
  3. Verify configuration: dora system info

📚 More help: https://dora-rs.ai/troubleshooting#daemon-connection
⏱️  Error occurred after 2.3s
```

#### Error Code System
```rust
// src/error.rs
#[derive(Debug, thiserror::Error)]
pub enum DoraCliError {
    #[error("DORA_E001: Failed to connect to daemon")]
    DaemonConnectionFailed {
        address: String,
        #[source]
        source: std::io::Error,
    },
    
    #[error("DORA_E002: Invalid dataflow configuration")]
    InvalidDataflow {
        file: PathBuf,
        line: Option<usize>,
        #[source] 
        source: serde_yaml::Error,
    },
    
    #[error("DORA_E003: Node build failed")]
    NodeBuildFailed {
        node_name: String,
        exit_code: i32,
        stderr: String,
    },
}

impl DoraCliError {
    pub fn error_code(&self) -> &'static str {
        match self {
            Self::DaemonConnectionFailed { .. } => "DORA_E001",
            Self::InvalidDataflow { .. } => "DORA_E002", 
            Self::NodeBuildFailed { .. } => "DORA_E003",
        }
    }
    
    pub fn suggestions(&self) -> Vec<String> {
        match self {
            Self::DaemonConnectionFailed { .. } => vec![
                "Start the daemon: dora daemon start".to_string(),
                "Check daemon status: dora daemon status".to_string(),
                "Verify configuration: dora system info".to_string(),
            ],
            Self::InvalidDataflow { file, .. } => vec![
                format!("Validate syntax: dora dataflow validate {}", file.display()),
                "Check YAML formatting and required fields".to_string(),
                "Review examples: dora dataflow template --list".to_string(),
            ],
            Self::NodeBuildFailed { node_name, .. } => vec![
                format!("Check node logs: dora node logs {}", node_name),
                format!("Rebuild node: dora node build {}", node_name),
                "Verify dependencies are installed".to_string(),
            ],
        }
    }
}
```

### 1.4 Enhanced Logging & Debugging

#### `dora dataflow logs` command
```bash
# Basic usage
dora dataflow logs demo                    # All logs from dataflow
dora dataflow logs demo --node camera     # Specific node logs
dora dataflow logs demo --follow          # Stream live logs
dora dataflow logs demo --tail 100        # Last 100 lines
dora dataflow logs demo --since 1h        # Last hour
dora dataflow logs demo --filter error    # Only error level
dora dataflow logs demo --output json     # Machine readable

# Advanced filtering
dora dataflow logs demo \
  --node camera,yolo \
  --level warn,error \
  --since "2024-01-01 10:00:00" \
  --grep "connection timeout"
```

#### Implementation Structure
```rust
#[derive(Debug, clap::Args)]
pub struct DataflowLogs {
    /// Dataflow name or ID
    pub dataflow: String,
    
    /// Filter by specific nodes
    #[clap(long, value_delimiter = ',')]
    pub node: Vec<String>,
    
    /// Follow log output (like tail -f)
    #[clap(long, short)]
    pub follow: bool,
    
    /// Number of lines to show from end
    #[clap(long, short)]
    pub tail: Option<usize>,
    
    /// Show logs since timestamp
    #[clap(long)]
    pub since: Option<chrono::DateTime<chrono::Utc>>,
    
    /// Filter by log level
    #[clap(long, value_enum)]
    pub level: Option<LogLevel>,
    
    /// Include timestamps
    #[clap(long, short)]
    pub timestamps: bool,
}
```

### 1.5 Interactive Project Initialization

#### `dora init` wizard with inquire
```bash
❯ dora init
✨ Welcome to Dora! Let's create your dataflow project.

? Project name: › my-robot-pipeline
? Project type: › 
  ❯ Computer Vision Pipeline
    Robotics Control System  
    Data Processing Pipeline
    Custom Dataflow
? Primary language: ›
  ❯ Rust
    Python
    Mixed (Rust + Python)
? Include examples: › Yes / No
? Initialize git repository: › Yes / No
? License: ›
  ❯ Apache-2.0
    MIT
    GPL-3.0
    None

🎯 Creating project structure...
  ├─ 📁 my-robot-pipeline/
  │  ├─ 📄 dataflow.yml
  │  ├─ 📄 Cargo.toml
  │  ├─ 📁 nodes/
  │  │  ├─ 📁 camera/
  │  │  └─ 📁 processor/
  │  └─ 📁 examples/

✅ Project created successfully!

🚀 Next steps:
  1. cd my-robot-pipeline
  2. dora dataflow validate dataflow.yml
  3. dora dataflow start dataflow.yml

📚 Learn more: https://dora-rs.ai/getting-started
```

### 1.6 Command Aliases and Shortcuts

#### Backward Compatibility & Convenience
```bash
# Maintain backward compatibility with aliases
dora up     → dora daemon start
dora start  → dora dataflow start  
dora stop   → dora dataflow stop
dora list   → dora dataflow list
dora ps     → dora dataflow list    # Docker-like alias

# Short forms for frequent operations
dora df     → dora dataflow         # df for dataflow
dora nd     → dora node            # nd for node  
dora sys    → dora system          # sys for system
```

### Success Metrics for Phase 1
- **Onboarding Time**: Reduce new user time-to-first-dataflow from 1+ hours to <15 minutes
- **Error Resolution**: 90% of CLI errors include actionable next steps
- **Command Discoverability**: All major operations discoverable within 2 levels of `--help`
- **Performance Feedback**: Real-time progress for all operations >2 seconds

---

## 🟡 Phase 2: Advanced Observability & System Management (3-4 weeks)

**Goal:** Provide developers with powerful inspection and debugging tools that rival enterprise monitoring solutions.

### 2.1 Comprehensive Node Management

#### `dora node` command suite
```bash
# Node discovery and listing
dora node list [--dataflow NAME] [--status running|stopped|error] [--output table|json]
dora node inspect <node-id> [--live] [--format compact|detailed|json]
dora node logs <node-id> [--follow] [--tail 100] [--since 1h]
dora node exec <node-id> <command> [--interactive] [--workdir /path]

# Node lifecycle management  
dora node restart <node-id> [--wait-timeout 30s]
dora node stop <node-id> [--force] [--grace-period 10s]
dora node start <node-id>    # Start stopped node
dora node status <node-id>   # Detailed status check

# Performance and debugging
dora node stats <node-id> [--live] [--duration 10m]
dora node trace <node-id> [--duration 30s] [--output flamegraph]
dora node profile <node-id> [--memory] [--cpu] [--duration 60s]
```

#### Advanced Node Inspection with ratatui
```bash
❯ dora node inspect camera-node --live

┌─ Node: camera-node ────────────────────────────────────┐
│ Status: Running (34m 12s)          PID: 15234          │
│ Dataflow: demo-pipeline            CPU: 12.3%          │
│ Memory: 245MB / 512MB              Network: 15MB/s ↓   │
└────────────────────────────────────────────────────────┘

┌─ Inputs ──────────────┐ ┌─ Outputs ─────────────────────┐
│ • tick (timer)        │ │ • image (848x480 BGR)        │
│   └─ Rate: 30 Hz      │ │   └─ Rate: 29.8 Hz           │
│   └─ Backlog: 0       │ │   └─ Consumers: 2            │
│                       │ │   └─ Total: 1.2M messages    │
└───────────────────────┘ └───────────────────────────────┘

┌─ Recent Activity ─────────────────────────────────────────────┐
│ [14:32:15] INFO  Frame captured: 1920x1080                   │
│ [14:32:14] DEBUG Processing frame 1445                       │
│ [14:32:13] INFO  Camera initialized successfully             │
│ [14:32:12] WARN  Frame dropped due to processing delay      │
└───────────────────────────────────────────────────────────────┘

┌─ Configuration ──────────────────────────────────────────────┐
│ Camera Device: /dev/video0                                   │
│ Resolution: 1920x1080                                        │
│ FPS Target: 30                                              │
│ Buffer Size: 4 frames                                       │
│ Working Dir: /tmp/dora-camera-node-15234                    │
└──────────────────────────────────────────────────────────────┘

Press 'q' to quit, 'r' to refresh, 'l' to view logs
```

#### Implementation Structure
```rust
// src/command/node.rs
use ratatui::{
    backend::CrosstermBackend,
    layout::{Constraint, Direction, Layout},
    style::{Color, Modifier, Style},
    text::{Line, Span},
    widgets::{Block, Borders, List, ListItem, Paragraph},
    Terminal,
};

pub struct NodeInspector {
    node_id: String,
    live_mode: bool,
    refresh_interval: Duration,
}

impl NodeInspector {
    pub async fn run(&mut self) -> Result<()> {
        let stdout = io::stdout();
        let backend = CrosstermBackend::new(stdout);
        let mut terminal = Terminal::new(backend)?;
        
        // Setup terminal
        enable_raw_mode()?;
        execute!(terminal.backend_mut(), EnterAlternateScreen, EnableMouseCapture)?;
        
        loop {
            let node_info = self.fetch_node_info().await?;
            terminal.draw(|f| self.draw_ui(f, &node_info))?;
            
            if self.handle_input().await? {
                break;
            }
            
            if self.live_mode {
                tokio::time::sleep(self.refresh_interval).await;
            }
        }
        
        // Cleanup
        disable_raw_mode()?;
        execute!(terminal.backend_mut(), LeaveAlternateScreen, DisableMouseCapture)?;
        terminal.show_cursor()?;
        
        Ok(())
    }
}
```

### 2.2 Real-time System Monitoring

#### `dora stats` - Live Dashboard
```bash
❯ dora stats --live

┌─ Dora System Overview ───────────────────── 14:32:45 ────────┐
│ Daemon: ● Running (2h 34m)    Coordinator: ● Running (2h 34m) │
│ Dataflows: 3 active, 1 stopped            Nodes: 12 running  │
└────────────────────────────────────────────────────────────────┘

┌─ Resource Usage ──────────────────────────────────────────────┐
│ CPU:  ████████████░░░░░░░░░░  60.2% (8/16 cores)            │
│ Mem:  ████████░░░░░░░░░░░░░░  35.8% (5.7GB/16GB)           │  
│ Disk: ███░░░░░░░░░░░░░░░░░░░  12.4% (124GB/1TB)            │
│ Net:  ↑ 15.3 MB/s  ↓ 42.1 MB/s                            │
└────────────────────────────────────────────────────────────────┘

┌─ Active Dataflows ────────────────────────────────────────────┐
│ Name              Nodes  CPU%   Memory    Network   Status    │
│ ─────────────────────────────────────────────────────────────│
│ demo-pipeline        4   23.1%   1.2GB    8.3MB/s   ● Running │
│ vision-processing    5   31.4%   2.8GB   15.2MB/s   ● Running │
│ control-system       3    5.7%   0.9GB    0.8MB/s   ● Running │
└────────────────────────────────────────────────────────────────┘

┌─ Node Performance (Top 5) ────────────────────────────────────┐
│ Node              CPU%   Memory   I/O Rate   Errors  Status   │
│ ─────────────────────────────────────────────────────────────│
│ yolo-detector    15.2%    856MB   5.2MB/s       0   ● Running │
│ image-processor  12.8%    623MB   3.1MB/s       0   ● Running │
│ camera-capture    8.9%    245MB   1.8MB/s       0   ● Running │
│ point-tracker     6.3%    312MB   0.9MB/s       2   ⚠ Warning │
│ visualization     3.1%    128MB   0.3MB/s       0   ● Running │
└────────────────────────────────────────────────────────────────┘

Press 'q' to quit, 'r' to refresh, 'd' for dataflows, 'n' for nodes
```

### 2.3 System Management & Diagnostics

#### `dora system` command suite
```bash
# System information and diagnostics
dora system info [--detailed] [--output json]
dora system status [--all-components]
dora system diagnose [--fix-common-issues]

# Resource management
dora system df [--human-readable] [--breakdown]
dora system prune [--dry-run] [--all] [--older-than 7d]
dora system cleanup [--cache] [--logs] [--temp-files]

# Performance and monitoring
dora system monitor [--duration 5m] [--interval 1s]
dora system benchmark [--component daemon|coordinator|ipc]
dora system export-logs [--since 24h] [--format tar.gz]
```

#### System Information Output
```bash
❯ dora system info

🖥️  Dora System Information
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

📋 Version Information
   Dora CLI:        v0.3.13
   Daemon:          v0.3.13 (running)
   Coordinator:     v0.3.13 (running) 
   Rust Version:    1.85.0
   Platform:        x86_64-unknown-linux-gnu

🏗️  System Architecture  
   OS:              Linux 5.4.0-84-generic
   CPU:             16 cores (Intel Xeon E5-2686 v4)
   Memory:          16.0 GB total, 10.3 GB available
   Storage:         1.0 TB SSD, 876 GB available

🔗 Network Configuration
   Daemon Address:  127.0.0.1:53290 ✅
   Coordinator:     127.0.0.1:53291 ✅ 
   IPC Method:      Shared Memory (optimal)
   Message Size:    10 MB max

📦 Resource Usage
   Active Dataflows: 3
   Running Nodes:    12
   Memory Usage:     5.7 GB (35.6%)
   Cache Size:       247 MB
   Log Files:        1.2 GB

🔧 Configuration
   Config Dir:       ~/.config/dora/
   Cache Dir:        ~/.cache/dora/
   Data Dir:         ~/.local/share/dora/
   Log Level:        info

💡 System Health: ✅ All systems operational
```

#### Resource Cleanup with Safety
```bash
❯ dora system prune --dry-run

🧹 Dora System Cleanup (DRY RUN)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

📂 Cache Files (safe to remove):
   ├─ Build cache (older than 7 days):     347 MB
   ├─ Download cache (unused):             89 MB  
   └─ Temporary files:                     23 MB
   
📝 Log Files (rotation suggested):
   ├─ Daemon logs (older than 30 days):    156 MB
   ├─ Node logs (older than 30 days):      234 MB
   └─ Debug logs (older than 7 days):      67 MB

💾 Data Files (review before removal):
   ├─ Stopped dataflow metadata:           12 MB
   └─ Orphaned node data:                  45 MB

🔍 Total space to recover: 973 MB

Run without --dry-run to perform cleanup
Use --force to skip confirmation prompts
```

### 2.4 Enhanced Daemon Management

#### `dora daemon` command enhancements
```bash
# Daemon lifecycle with better control
dora daemon start [--config CONFIG] [--log-level debug] [--background]
dora daemon stop [--graceful] [--timeout 30s]
dora daemon restart [--preserve-state] [--wait-healthy]
dora daemon status [--detailed] [--health-check]

# Advanced daemon operations
dora daemon config [--edit] [--validate] [--show]
dora daemon logs [--follow] [--level error] [--component all|ipc|scheduler]
dora daemon reload-config [--validate-first]
dora daemon debug [--dump-state] [--memory-profile]
```

### 2.5 Tab Completion & Shell Integration

#### Comprehensive Shell Completion
```bash
# Generate completion scripts
dora completion bash > /etc/bash_completion.d/dora
dora completion zsh > ~/.zsh/completions/_dora  
dora completion fish > ~/.config/fish/completions/dora.fish

# Smart context-aware completion
dora dataflow stop <TAB>        # Shows running dataflows
dora node inspect <TAB>         # Shows available nodes
dora dataflow logs demo --node <TAB>  # Shows nodes in 'demo' dataflow
```

#### Implementation
```rust
// Enhanced clap configuration for completions
use clap::CommandFactory;
use clap_complete::{generate, shells::*};

#[derive(Debug, clap::Subcommand)]
enum CompletionShell {
    Bash,
    Zsh, 
    Fish,
    PowerShell,
}

pub fn generate_completion(shell: CompletionShell) {
    let mut app = Args::command();
    match shell {
        CompletionShell::Bash => generate(Bash, &mut app, "dora", &mut io::stdout()),
        CompletionShell::Zsh => generate(Zsh, &mut app, "dora", &mut io::stdout()),
        CompletionShell::Fish => generate(Fish, &mut app, "dora", &mut io::stdout()),
        CompletionShell::PowerShell => generate(PowerShell, &mut app, "dora", &mut io::stdout()),
    }
}
```

### Success Metrics for Phase 2
- **Debug Efficiency**: Reduce time to identify issues from >30 minutes to <5 minutes
- **System Visibility**: Real-time monitoring of all system resources and components
- **Operational Control**: Complete lifecycle management for all Dora components
- **Performance Insights**: Detailed profiling and performance analysis capabilities

---

## 🔴 Phase 3: Data-Driven Development Workflow (4-6 weeks)

**Goal:** Transform Dora into a comprehensive data-driven development platform with industry-leading recording, replay, and analysis capabilities.

### 3.1 Advanced Recording System

#### `dora recording` command suite
```bash
# Recording lifecycle management
dora recording start <dataflow> [--topics topic1,topic2] [--duration 1h] [--size-limit 10GB]
dora recording stop <recording-id> [--graceful] [--export-immediately] 
dora recording pause <recording-id>     # Temporarily pause recording
dora recording resume <recording-id>    # Resume paused recording

# Recording discovery and management
dora recording list [--dataflow NAME] [--since 7d] [--size +100MB] [--status active|completed]
dora recording inspect <recording-id> [--topics] [--timeline] [--statistics]
dora recording export <recording-id> [--format bag|parquet|json] [--output /path]
dora recording remove <recording-id> [--force] [--keep-metadata]
dora recording prune [--older-than 30d] [--dry-run] [--size-threshold 1GB]

# Advanced recording features
dora recording tag <recording-id> <tag1,tag2>     # Add searchable tags
dora recording search [--tags robot-test] [--text "camera failure"]
dora recording clone <recording-id> [--topics subset] [--time-range start:end]
dora recording merge <rec1> <rec2> [--output merged-recording] [--sync-timestamps]
```

#### Smart Recording Configuration  
```yaml
# recording-config.yml
recording:
  name: "vision-system-test-{{timestamp}}"
  description: "Automated vision system validation run"
  
  # Selective topic recording
  topics:
    include: ["camera/*", "detection/*", "tracking/*"]
    exclude: ["debug/*", "telemetry/heartbeat"]
    
  # Quality and storage controls
  quality:
    compression: "lz4"          # lz4, zstd, snappy
    max_size: "5GB"
    max_duration: "2h"
    rotation_policy: "size"      # size, time, manual
    
  # Metadata enrichment
  metadata:
    environment: "test-lab-1"
    hardware: "jetson-xavier"
    software_version: "v2.1.0"
    operator: "ci-system"
    
  # Automatic triggers
  triggers:
    start_on_node: "camera-node"
    stop_on_error: true
    auto_export: true
```

#### Recording Output Example
```bash
❯ dora recording start vision-pipeline --config recording-config.yml

📹 Starting recording session
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

🎯 Session: vision-system-test-20240115-143022
📁 Output: ~/.local/share/dora/recordings/vision-system-test-20240115-143022/
⏱️  Duration: unlimited (max: 2h)
💾 Size: 0 MB (max: 5GB)

📊 Topic Status:
  ├─ camera/image           ● Recording (30 Hz, 12MB/s)
  ├─ detection/objects      ● Recording (30 Hz, 2MB/s)  
  ├─ tracking/trajectories  ● Recording (30 Hz, 1MB/s)
  └─ system/metrics         ● Recording (1 Hz, 0.1MB/s)

🔍 Live Preview:
  [14:30:25] Camera: Frame 1,834 (1920x1080)
  [14:30:25] Detection: 3 objects detected
  [14:30:24] Tracking: 2 trajectories active
  
📈 Session Stats: 1m 23s elapsed, 1.2GB recorded, 15.3MB/s avg

Press 'q' to stop recording, 's' for stats, 'p' to pause
```

### 3.2 Intelligent Replay System

#### `dora replay` command suite with advanced features
```bash
# Basic replay operations
dora replay <recording-id> [--speed 1.0] [--loop] [--start-time 30s] [--end-time 5m]
dora replay <recording-id> --dataflow custom-analysis.yml  # Replay into different dataflow
dora replay <recording-id> --topics camera/image,detection/* --speed 0.5

# Advanced replay modes
dora replay <recording-id> --interactive     # Step-through debugging mode
dora replay <recording-id> --sync external-clock  # Sync with external timing source
dora replay <recording-id> --inject-latency 50ms  # Simulate network delays
dora replay <recording-id> --randomize-order 0.1  # Introduce 10% message reordering

# Analysis and comparison
dora replay compare <rec1> <rec2> [--topics subset] [--metrics latency,throughput]
dora replay benchmark <recording-id> --dataflow analysis.yml [--iterations 10]
dora replay diff <rec1> <rec2> [--tolerance 0.1] [--output report.html]
```

#### Interactive Replay Mode
```bash
❯ dora replay vision-test-rec-001 --interactive

🎬 Interactive Replay Mode
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

📊 Timeline: [████████████████████░░░░░░░░] 15:32 / 19:45 (78%)
🎯 Current: Frame 1,234 @ 2024-01-15 14:30:25.123

┌─ Message Inspector ─────────────────────────────────────────┐
│ Topic: camera/image                                         │
│ Timestamp: 2024-01-15T14:30:25.123456Z                     │
│ Size: 2.4 MB (1920x1080 BGR)                              │
│ Sequence: 1,234                                           │
│ Metadata: {"exposure": 1/60, "iso": 400}                  │
└─────────────────────────────────────────────────────────────┘

┌─ Active Topics ──────────────────────────────────────────────┐
│ ● camera/image        (30 Hz)  [Next: +33ms]               │
│ ● detection/objects   (30 Hz)  [Next: +28ms]               │
│ ● tracking/paths      (15 Hz)  [Next: +12ms]               │
└─────────────────────────────────────────────────────────────┘

Controls: [space] play/pause, [→] step forward, [←] step back, 
         [↑] speed up, [↓] slow down, [g] goto time, [q] quit
```

### 3.3 Recording Analysis & Visualization

#### `dora analyze` command for deep insights
```bash
# Statistical analysis
dora analyze <recording-id> stats [--topics camera/*] [--output stats.json]
dora analyze <recording-id> latency [--node-to-node] [--percentiles 50,95,99]
dora analyze <recording-id> throughput [--window 1s] [--rolling-average 10s]
dora analyze <recording-id> gaps [--tolerance 33ms] [--report missing-frames.txt]

# Data quality assessment
dora analyze <recording-id> quality [--check-corruption] [--validate-schema]
dora analyze <recording-id> sync [--master-topic camera/image] [--tolerance 10ms]
dora analyze <recording-id> coverage [--expected-topics topics.yml]

# Performance profiling
dora analyze <recording-id> profile [--node performance-bottlenecks] [--flamegraph]
dora analyze <recording-id> memory [--peak-usage] [--growth-trends]
dora analyze <recording-id> network [--bandwidth-usage] [--protocol-breakdown]
```

#### Analysis Report Output
```bash
❯ dora analyze vision-test-rec-001 stats --output detailed

📊 Recording Analysis Report
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

📋 Session Information:
   Recording ID: vision-test-rec-001
   Duration: 19m 45s (1,185 seconds)  
   Total Size: 4.2 GB
   Topics: 12 active, 3 inactive
   Nodes: 8 participants

📈 Message Statistics:
   ┌──────────────────────┬─────────┬────────────┬─────────────┬──────────┐
   │ Topic                │ Count   │ Rate (Hz)  │ Size (MB)   │ Gaps     │
   ├──────────────────────┼─────────┼────────────┼─────────────┼──────────┤
   │ camera/image         │ 35,550  │ 30.0 ± 0.2 │ 3,890.4     │ 3        │
   │ detection/objects    │ 35,547  │ 30.0 ± 0.1 │ 245.6       │ 6        │
   │ tracking/paths       │ 17,775  │ 15.0 ± 0.3 │ 89.2        │ 1        │
   │ control/commands     │ 1,185   │ 1.0 ± 0.0  │ 1.2         │ 0        │
   └──────────────────────┴─────────┴────────────┴─────────────┴──────────┘

🔍 Quality Assessment:
   ✅ Message integrity: 99.97% (12 corrupted messages)
   ✅ Timestamp consistency: 99.95% (18 out-of-order messages) 
   ⚠️  Topic synchronization: 94.2% (camera/detection drift: 12ms avg)
   ✅ Schema validation: 100% compliant

⚡ Performance Insights:
   📊 Peak throughput: 87.3 MB/s @ 14:35:22
   🐌 Slowest node: yolo-detector (avg: 45ms latency)
   🚀 Fastest node: camera-capture (avg: 2ms latency)  
   💾 Memory peak: 2.1 GB @ 14:42:15

🎯 Recommendations:
   1. Investigate camera/detection sync drift (>10ms threshold)
   2. Consider yolo-detector optimization (>40ms latency)
   3. Add redundancy for critical topics (>5 gaps detected)

📁 Full report: ~/.local/share/dora/analysis/vision-test-rec-001-analysis.html
```

### 3.4 Recording Templates & Automation

#### Template System for Common Scenarios
```bash
# Create and manage recording templates
dora recording template create vision-validation \
  --topics "camera/*,detection/*,tracking/*" \
  --duration 10m \
  --quality high \
  --metadata environment=test

dora recording template list [--category robotics|vision|testing]
dora recording template show vision-validation
dora recording template edit vision-validation [--editor vim]

# Use templates for consistent recording
dora recording start my-dataflow --template vision-validation
dora recording batch-start --template stress-test --count 5 --interval 1h
```

#### CI/CD Integration
```bash
# Automated testing workflows
dora recording ci-test <dataflow> \
  --template validation-suite \
  --compare-baseline baseline-rec-001 \
  --tolerance 5% \
  --fail-on-regression

# Generate test reports
dora recording report <recording-id> \
  --baseline baseline-rec-001 \
  --format junit-xml \
  --output test-results.xml
```

### 3.5 Data Pipeline Integration

#### Export to External Systems
```bash
# Export for machine learning pipelines
dora recording export ml-dataset-001 \
  --format tensorflow-records \
  --topics camera/image,detection/labels \
  --split train:80,val:15,test:5 \
  --output /datasets/vision-v2/

# Integration with data lakes
dora recording sync <recording-id> \
  --destination s3://company-datalake/robotics/ \
  --format parquet \
  --partition-by date,environment \
  --compression gzip

# Stream processing integration
dora recording stream <recording-id> \
  --destination kafka://localhost:9092/dora-replay \
  --speed realtime \
  --loop-indefinitely
```

### Core Implementation Requirements

#### Enhanced dora-record Node
```rust
// Core recording infrastructure improvements
pub struct DoraRecorder {
    config: RecordingConfig,
    storage: Box<dyn StorageBackend>,     // Pluggable storage (local, s3, etc.)
    indexer: MessageIndexer,              // SQLite-based indexing
    compressor: Box<dyn CompressionEngine>, // Configurable compression
    metadata: SessionMetadata,
}

impl DoraRecorder {
    // Intelligent topic filtering
    pub fn should_record_topic(&self, topic: &str) -> bool {
        self.config.topic_filter.matches(topic)
    }
    
    // Efficient message indexing
    pub fn index_message(&mut self, msg: &Message) -> Result<()> {
        self.indexer.add_entry(MessageIndex {
            topic: msg.topic.clone(),
            timestamp: msg.timestamp,
            offset: self.storage.current_position(),
            size: msg.data.len(),
            checksum: msg.checksum(),
        })
    }
}
```

#### Enhanced dora-replay Node
```rust
pub struct DoraReplayer {
    index: RecordingIndex,
    playback_config: PlaybackConfig,
    timeline: Timeline,
    sync_controller: SynchronizationController,
}

impl DoraReplayer {
    // Precise timing control
    pub async fn replay_with_timing(&mut self) -> Result<()> {
        let mut next_message_time = self.timeline.start_time();
        
        while let Some(message) = self.index.next_message_after(next_message_time)? {
            // Apply speed scaling
            let delay = (message.timestamp - next_message_time) / self.playback_config.speed;
            tokio::time::sleep(delay).await;
            
            self.publish_message(message).await?;
            next_message_time = message.timestamp;
        }
        Ok(())
    }
    
    // Interactive debugging support
    pub fn step_forward(&mut self) -> Result<Option<Message>> {
        self.timeline.step(1)
    }
    
    pub fn goto_time(&mut self, target: Timestamp) -> Result<()> {
        self.timeline.seek(target)
    }
}
```

### Success Metrics for Phase 3
- **Development Velocity**: 50% faster iteration cycles using recorded data vs. live systems
- **Test Coverage**: Comprehensive regression testing using recorded scenarios
- **Debug Effectiveness**: 80% of issues reproducible and debuggable through recordings
- **Data Quality**: 99.9% message integrity and timestamp accuracy in recordings
- **CI Integration**: Automated recording-based testing in all major workflows

---

## 🛠️ Technical Implementation Plan

### Dependencies & Tools
```toml
# Additional Cargo.toml dependencies for CLI enhancements
[dependencies]
# Progress and UI
indicatif = "0.17"
ratatui = "0.24"
crossterm = "0.27"
inquire = "0.6"

# Enhanced output and formatting  
colored = "2.0"
tabled = "0.14"
serde_json = "1.0"
serde_yaml = "0.9"

# Shell completion
clap_complete = "4.4"

# Error handling
thiserror = "1.0"
eyre = "0.6"

# Time and date handling
chrono = { version = "0.4", features = ["serde"] }

# Async runtime
tokio = { version = "1.0", features = ["full"] }

# Recording and analysis
sqlite = "0.32"        # For recording indexes
lz4 = "1.24"          # For compression
arrow = "49.0"        # For data processing
```

### Incremental Rollout Strategy

#### Week 1-2: Foundation & Command Structure
- Refactor command structure to object-oriented pattern
- Implement basic progress indicators with `indicatif`
- Add enhanced error types with error codes

#### Week 3-4: Core UX Improvements  
- Complete dataflow command suite
- Implement real-time feedback for long operations
- Add interactive `dora init` wizard

#### Week 5-6: Advanced Observability
- Build node inspection with `ratatui`
- Implement live system monitoring dashboard
- Add comprehensive system diagnostics

#### Week 7-8: Recording Infrastructure
- Develop enhanced recording system
- Implement intelligent replay with timing control
- Add recording analysis and reporting

#### Week 9-10: Polish & Integration
- Complete shell completion system
- Add comprehensive testing suite
- Documentation and examples

### Backward Compatibility Strategy
- Maintain aliases for all existing commands
- Gradual deprecation with warnings, not breaking changes
- Environment variable to enable legacy mode if needed
- Migration guide for any workflow changes

This enhanced CLI design transforms Dora from a basic tool into a comprehensive development platform that rivals the best CLI tools in the industry. The phased approach ensures incremental value delivery while building toward an exceptional developer experience.