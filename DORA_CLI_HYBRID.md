# Dora CLI: Hybrid Architecture - Best of Both Worlds

> **Progressive Disclosure Design**: Simple CLI commands for quick tasks, intelligent TUI enhancement for complex operations. The right interface at the right time.

## 🎯 Design Philosophy

**Core Principle**: Start simple, scale complexity only when it adds value.

- **Docker-like simplicity** for common operations
- **Intelligent suggestions** for TUI when complexity warrants it
- **Always scriptable** - never break automation
- **User choice respected** - easy overrides for personal preference

### The Progressive Disclosure Model

```
Simple Task    →  CLI Output     →  Fast, Familiar
                                    
Medium Task    →  CLI + TUI Hint  →  Enhanced with Guidance
                                    
Complex Task   →  Smart TUI       →  Interactive, Visual
```

---

## 🚀 Command Structure Overview

### **Tier 1: Docker-Style Core Commands**
```bash
# Lightning-fast common operations
dora ps                           # Quick status overview
dora start <dataflow.yml>         # Start dataflow
dora stop <name>                  # Stop dataflow  
dora logs <name>                  # View logs
dora build <dataflow.yml>         # Build dependencies
```

### **Tier 2: Enhanced Commands with Smart Suggestions**
```bash
# CLI-first with TUI hints when valuable
dora inspect <resource>           # Smart TUI suggestion for complex resources
dora debug <dataflow>             # Auto-suggests interactive debugging
dora analyze <recording>          # Recommends visual analysis interface
dora monitor                      # Offers live dashboard option
```

### **Tier 3: Explicit TUI Modes**
```bash
# Direct access to rich interfaces
dora ui                          # Main dashboard
dora ui dataflow                 # Dataflow management interface
dora ui monitor                  # System monitoring dashboard
dora ui debug <dataflow>         # Interactive debugging session
```

---

## 🎨 Smart Interface Selection

### **Context Detection Engine**

```rust
#[derive(Debug)]
pub struct ExecutionContext {
    pub is_tty: bool,
    pub is_piped: bool,
    pub is_scripted: bool,
    pub terminal_size: Option<(u16, u16)>,
    pub user_preference: UiMode,
    pub command_complexity: u8,
}

#[derive(Debug, Clone)]
pub enum UiMode {
    Auto,        // Smart decisions based on context
    Cli,         // Always prefer CLI output
    Tui,         // Prefer TUI when available
    Minimal,     // Minimal output (CI/scripting)
}

pub struct InterfaceDecision {
    pub use_tui: bool,
    pub show_hint: bool,
    pub auto_launch: bool,
    pub reason: String,
}

impl ExecutionContext {
    pub fn decide_interface(&self, command: &Command) -> InterfaceDecision {
        // Never TUI in non-interactive contexts
        if !self.is_tty || self.is_piped || self.is_scripted {
            return InterfaceDecision {
                use_tui: false,
                show_hint: false,
                auto_launch: false,
                reason: "Non-interactive context".to_string(),
            };
        }

        // Respect user preferences
        match self.user_preference {
            UiMode::Cli => return InterfaceDecision::cli_only(),
            UiMode::Tui => return InterfaceDecision::tui_preferred(),
            UiMode::Minimal => return InterfaceDecision::minimal(),
            UiMode::Auto => {} // Continue with smart detection
        }

        // Smart detection based on command complexity
        match (command.base_type(), self.command_complexity) {
            // Simple queries - stay CLI
            (CommandType::Query, 0..=2) => InterfaceDecision {
                use_tui: false,
                show_hint: false,
                auto_launch: false,
                reason: "Simple query".to_string(),
            },
            
            // Medium complexity - suggest TUI
            (CommandType::Inspect | CommandType::Monitor, 3..=6) => InterfaceDecision {
                use_tui: false,
                show_hint: true,
                auto_launch: false,
                reason: "Complex data benefits from interactive view".to_string(),
            },
            
            // High complexity - auto-launch TUI
            (CommandType::Debug | CommandType::Analyze, 7..=10) => InterfaceDecision {
                use_tui: true,
                show_hint: false,
                auto_launch: true,
                reason: "Interactive interface optimal for this task".to_string(),
            },
            
            _ => InterfaceDecision::cli_only(),
        }
    }
}
```

---

## 📋 Command Categories & Behaviors

### **Category 1: Quick Status & Control**
*Docker-like simplicity - always CLI*

#### `dora ps` - Process Status
```bash
❯ dora ps
NAME              STATUS    UPTIME    NODES    CPU     MEMORY
demo-pipeline     Running   2h34m     4/4      45%     1.2GB
vision-system     Warning   45m       3/4      23%     856MB
control-loop      Stopped   -         0/2      0%      0MB

💡 Use 'dora ui' for interactive monitoring
```

#### `dora start` - Start Dataflow
```bash
❯ dora start vision-pipeline.yml
🔧 Loading configuration... ✅
📦 Building 4 nodes... ✅ (3.2s)
🌐 Starting dataflow... ✅
✨ Dataflow 'vision-pipeline' is running!

❯ dora start complex-ml-pipeline.yml --name production
⚠️  Complex dataflow detected (12 nodes, 3 GPU requirements)
🚀 This may take several minutes...

Monitor progress interactively? [Y/n]: y
# Launches TUI progress monitor
```

#### `dora stop` - Stop Dataflow
```bash
❯ dora stop demo-pipeline
🛑 Stopping dataflow 'demo-pipeline'...
⏳ Graceful shutdown (10s timeout)... ✅
✨ Dataflow stopped successfully

❯ dora stop --all
⚠️  This will stop 3 active dataflows
Continue? [y/N]: y
🛑 Stopping demo-pipeline... ✅
🛑 Stopping vision-system... ✅  
🛑 Stopping ml-inference... ✅
```

### **Category 2: Information Gathering**
*CLI-first with smart TUI suggestions*

#### `dora logs` - View Logs
```bash
# Simple log viewing - stays CLI
❯ dora logs demo-pipeline
[14:32:45] INFO  camera-node: Frame captured 1920x1080
[14:32:45] DEBUG yolo-detector: Processing frame 1234
[14:32:45] WARN  tracker: High latency detected: 45ms

# Complex log scenarios suggest TUI
❯ dora logs error-prone-system
[14:32:43] ERROR node-processor: Connection timeout
[14:32:44] ERROR node-processor: Retry failed  
[14:32:45] ERROR node-processor: Circuit breaker opened
... (47 more errors) ...

🚨 High error rate detected (50 errors in last minute)
📊 Launch interactive log analyzer? [Y/n]: y
# Opens TUI log viewer with filtering, search, and correlation
```

#### `dora inspect` - Resource Inspection
```bash
# Simple resource - CLI output
❯ dora inspect simple-node
Node: simple-node
Status: ● Running (1h 23m)
CPU: 5.2% | Memory: 128MB | Network: 1.2MB/s
Inputs: tick (1Hz) | Outputs: status (1Hz)

# Complex resource - suggests TUI
❯ dora inspect ml-vision-node
Node: ml-vision-node
Status: ● Running (45m) - 3 active connections, 156 recent messages
⚠️  Performance anomalies detected

📊 This node has complex state and metrics
🔍 Launch interactive inspector? [Y/n]: y
# Opens TUI node inspector with real-time graphs
```

### **Category 3: Analysis & Debugging**
*TUI-first with CLI fallback*

#### `dora debug` - Interactive Debugging
```bash
❯ dora debug vision-pipeline
🔍 Analyzing dataflow 'vision-pipeline'...
📊 4 nodes, 6 connections, 2 performance issues detected

Launching interactive debugger...
# Automatically opens TUI debugging interface

❯ dora debug vision-pipeline --text
# Forces CLI output
Debug Report for 'vision-pipeline':
- Node 'yolo-detector': High latency (avg: 45ms, p95: 78ms)
- Connection 'camera->yolo': Backlog detected (23 messages)
Recommendations: Consider GPU acceleration for yolo-detector
```

#### `dora analyze` - Recording Analysis
```bash
❯ dora analyze recording-001
📹 Analyzing recording 'recording-001' (4.2GB, 19m 45s)...
📊 12 topics, 847K messages, 3 data quality issues found

Best analyzed interactively due to data complexity.
Launch analysis interface? [Y/n]: y
# Opens TUI recording analyzer

❯ dora analyze recording-001 --summary
Recording Analysis Summary:
Duration: 19m 45s | Size: 4.2GB | Topics: 12
Quality: 99.7% message integrity
Issues: 3 timestamp drift events, 12 corrupted frames
Peak throughput: 87.3 MB/s
```

---

## 🎮 TUI Integration Patterns

### **Seamless CLI ↔ TUI Transitions**

#### From CLI to TUI
```bash
# Any CLI command can transition to TUI
❯ dora ps
# Shows dataflow list
# Press 'i' for interactive mode → launches TUI dashboard
# Press 'd' on specific dataflow → launches dataflow manager TUI
```

#### From TUI to CLI
```bash
# TUI shows CLI equivalents
# In TUI node inspector:
Status Bar: "CLI: dora inspect camera-node --text"

# Command mode in TUI (like vim)
Press ':' → : dataflow start new-pipeline.yml
# Executes CLI command and updates TUI view
```

#### Smart Suggestions
```bash
❯ dora node list
camera-node      ● Running   30Hz    245MB
yolo-detector    ⚠ Warning   29Hz    856MB   # High memory usage
tracker          ● Running   15Hz    312MB
visualizer       ● Running   30Hz    128MB

⚠️  'yolo-detector' showing performance issues
🔍 Inspect interactively? [y/N]: y
# Opens TUI node inspector focused on problematic node
```

### **Progressive Enhancement Examples**

#### Log Viewing Evolution
```bash
# Level 1: Basic logs
❯ dora logs my-node
[INFO] Normal operation...

# Level 2: Enhanced logs with context
❯ dora logs error-node  
[ERROR] Multiple failures detected...
💡 Try 'dora logs error-node --interactive' for advanced filtering

# Level 3: Interactive log analysis
❯ dora logs error-node --interactive
# Opens TUI log viewer with:
# - Real-time filtering
# - Error correlation
# - Timeline view
# - Search and highlighting
```

#### System Monitoring Evolution  
```bash
# Level 1: Quick status
❯ dora system status
Daemon: ● Running | CPU: 45% | Memory: 2.1GB

# Level 2: Detailed metrics
❯ dora system monitor
System Overview:
CPU: 45% ████████████░░░░░░░░
Memory: 35% ██████████░░░░░░░░░░
...
💡 Use 'dora ui monitor' for live dashboard

# Level 3: Live monitoring dashboard
❯ dora ui monitor
# Opens real-time TUI dashboard with:
# - Live resource graphs
# - Process monitoring  
# - Alert management
# - Interactive drill-down
```

---

## ⚙️ Configuration & Preferences

### **User Preference System**

```bash
# Set global UI preference
dora config set ui.mode auto      # Smart suggestions (default)
dora config set ui.mode cli       # Prefer CLI output  
dora config set ui.mode tui       # Prefer TUI when available
dora config set ui.mode minimal   # Minimal output (CI-friendly)

# Configure suggestion thresholds
dora config set ui.complexity_threshold 3    # When to suggest TUI (1-10)
dora config set ui.auto_launch_threshold 7   # When to auto-launch TUI

# Per-command preferences
dora config set commands.inspect.ui tui      # Always use TUI for inspect
dora config set commands.logs.ui cli         # Always use CLI for logs
```

### **Environment Variable Support**

```bash
# Temporary preference override
DORA_UI_MODE=cli dora inspect complex-node   # Force CLI
DORA_UI_MODE=tui dora logs my-node           # Force TUI

# CI/Automation friendly
DORA_UI_MODE=minimal dora ps --output json   # Machine-readable only

# Development mode
DORA_UI_MODE=auto DORA_UI_THRESHOLD=2 dora start complex.yml
```

### **Command-line Flags**

```bash
# Universal flags for any command
dora inspect node-01 --tui          # Force TUI mode
dora inspect node-01 --no-tui       # Force CLI mode  
dora inspect node-01 --minimal      # Minimal output
dora inspect node-01 --interactive  # Prompt for mode choice

# Output format control
dora ps --output table              # Human-readable table
dora ps --output json               # Machine-readable JSON
dora ps --output yaml               # YAML format
```

---

## 🛠️ Implementation Architecture

### **Core CLI Framework**

```rust
// src/cli/mod.rs - Main CLI framework
use clap::{Parser, Subcommand};

#[derive(Parser)]
#[clap(name = "dora", about = "Dora dataflow runtime")]
pub struct Cli {
    #[clap(subcommand)]
    pub command: Command,
    
    /// Force interface mode
    #[clap(long, global = true, value_enum)]
    pub ui_mode: Option<UiMode>,
    
    /// Output format
    #[clap(long, global = true, value_enum, default_value = "auto")]
    pub output: OutputFormat,
    
    /// Disable TUI hints and prompts
    #[clap(long, global = true)]
    pub no_hints: bool,
}

#[derive(Subcommand)]
pub enum Command {
    // Tier 1: Core Docker-like commands
    #[clap(alias = "list")]
    Ps(PsCommand),
    Start(StartCommand), 
    Stop(StopCommand),
    Logs(LogsCommand),
    Build(BuildCommand),
    
    // Tier 2: Enhanced commands with smart suggestions
    Inspect(InspectCommand),
    Debug(DebugCommand),
    Analyze(AnalyzeCommand),
    Monitor(MonitorCommand),
    
    // Tier 3: Explicit TUI modes
    Ui(UiCommand),
    
    // System management
    System(SystemCommand),
    Config(ConfigCommand),
}

#[derive(Clone, Debug, clap::ValueEnum)]
pub enum UiMode {
    Auto,     // Smart decisions
    Cli,      // Force CLI
    Tui,      // Force TUI
    Minimal,  // Minimal output
}

#[derive(Clone, Debug, clap::ValueEnum)]  
pub enum OutputFormat {
    Auto,     // Context-appropriate
    Table,    // Human-readable table
    Json,     // Machine-readable JSON
    Yaml,     // YAML format
    Minimal,  // Minimal text
}
```

### **Smart Interface Selection Engine**

```rust
// src/cli/interface.rs - Interface decision logic
pub struct InterfaceSelector {
    context: ExecutionContext,
    config: UserConfig,
}

impl InterfaceSelector {
    pub fn select_interface(&self, command: &Command) -> InterfaceStrategy {
        let base_decision = self.analyze_command(command);
        let context_decision = self.analyze_context();
        let user_preference = self.get_user_preference(command);
        
        // Combine factors to make final decision
        self.resolve_interface_strategy(base_decision, context_decision, user_preference)
    }
    
    fn analyze_command(&self, command: &Command) -> CommandAnalysis {
        CommandAnalysis {
            complexity: self.calculate_complexity(command),
            data_volume: self.estimate_data_volume(command),
            interaction_benefit: self.assess_interaction_benefit(command),
            automation_suitability: self.check_automation_suitability(command),
        }
    }
    
    fn calculate_complexity(&self, command: &Command) -> u8 {
        match command {
            Command::Ps(_) => 1,                    // Simple list
            Command::Start(_) => 2,                 // Basic operation
            Command::Logs(cmd) => {
                let mut score = 2;
                if cmd.follow { score += 1; }       // Real-time adds complexity
                if cmd.filter.is_some() { score += 1; } // Filtering adds complexity
                if cmd.error_analysis { score += 2; }   // Analysis adds complexity
                score
            },
            Command::Inspect(cmd) => {
                let mut score = 3;
                // Check resource complexity
                if self.is_complex_resource(&cmd.target) { score += 3; }
                if cmd.live_mode { score += 2; }    // Live monitoring adds complexity
                score
            },
            Command::Debug(_) => 8,                 // Always complex
            Command::Analyze(_) => 9,               // Always complex
            _ => 3,
        }
    }
}

pub enum InterfaceStrategy {
    CliOnly,
    CliWithHint { hint: String },
    PromptForTui { reason: String },
    AutoLaunchTui { reason: String },
}
```

### **Hybrid Command Implementation**

```rust
// src/commands/inspect.rs - Example hybrid command
#[derive(Debug, clap::Args)]
pub struct InspectCommand {
    /// Resource to inspect (dataflow, node, recording, etc.)
    pub target: String,
    
    /// Resource type (auto-detected if not specified)
    #[clap(long, value_enum)]
    pub resource_type: Option<ResourceType>,
    
    /// Force TUI mode
    #[clap(long)]
    pub tui: bool,
    
    /// Force CLI text output
    #[clap(long)]
    pub text: bool,
    
    /// Enable live monitoring mode
    #[clap(long)]
    pub live: bool,
    
    /// Output format for text mode
    #[clap(long, value_enum, default_value = "table")]
    pub format: OutputFormat,
}

impl InspectCommand {
    pub async fn execute(&self, context: &ExecutionContext) -> Result<()> {
        let resource = self.resolve_resource().await?;
        let interface_selector = InterfaceSelector::new(context);
        
        // Override with explicit flags
        let strategy = if self.tui {
            InterfaceStrategy::AutoLaunchTui { 
                reason: "Explicit TUI request".to_string() 
            }
        } else if self.text {
            InterfaceStrategy::CliOnly
        } else {
            interface_selector.select_interface(&Command::Inspect(self.clone()))
        };
        
        match strategy {
            InterfaceStrategy::CliOnly => {
                self.render_cli_output(&resource).await?;
            },
            
            InterfaceStrategy::CliWithHint { hint } => {
                self.render_cli_output(&resource).await?;
                println!("\n💡 {}", hint);
            },
            
            InterfaceStrategy::PromptForTui { reason } => {
                self.render_cli_output(&resource).await?;
                if self.prompt_for_tui(&reason)? {
                    self.launch_tui_inspector(&resource).await?;
                }
            },
            
            InterfaceStrategy::AutoLaunchTui { reason } => {
                println!("🚀 {}", reason);
                println!("Launching interactive inspector...");
                self.launch_tui_inspector(&resource).await?;
            },
        }
        
        Ok(())
    }
    
    async fn render_cli_output(&self, resource: &Resource) -> Result<()> {
        match self.format {
            OutputFormat::Table => self.render_table(resource),
            OutputFormat::Json => self.render_json(resource),
            OutputFormat::Yaml => self.render_yaml(resource),
            _ => self.render_compact(resource),
        }
    }
    
    fn prompt_for_tui(&self, reason: &str) -> Result<bool> {
        println!("\n🔍 {}", reason);
        print!("Launch interactive inspector? [Y/n]: ");
        io::stdout().flush()?;
        
        let mut input = String::new();
        io::stdin().read_line(&mut input)?;
        
        Ok(input.trim().is_empty() || input.to_lowercase().starts_with('y'))
    }
}
```

---

## 📊 TUI Integration Framework

### **Unified TUI Architecture**

```rust
// src/tui/app.rs - Main TUI application
pub struct DoraApp {
    current_view: ViewType,
    view_stack: Vec<ViewType>,
    global_state: AppState,
    event_handler: EventHandler,
    command_mode: bool,
    command_buffer: String,
}

#[derive(Clone, Debug)]
pub enum ViewType {
    Dashboard,
    DataflowManager,
    NodeInspector { node_id: String },
    SystemMonitor,
    LogViewer { target: String },
    RecordingAnalyzer { recording_id: String },
    DebugSession { dataflow_id: String },
}

impl DoraApp {
    pub async fn launch_with_context(
        initial_view: ViewType,
        cli_context: CliContext,
    ) -> Result<()> {
        let mut app = Self::new(initial_view);
        app.set_cli_context(cli_context);
        app.run().await
    }
    
    // Handle commands executed within TUI
    async fn execute_cli_command(&mut self, command: &str) -> Result<()> {
        let args = shell_words::split(command)?;
        let cli = Cli::try_parse_from(std::iter::once("dora").chain(args.iter()))?;
        
        // Execute command and update TUI state
        match cli.command {
            Command::Start(cmd) => {
                cmd.execute_silent().await?;
                self.refresh_dataflow_list().await?;
            },
            Command::Stop(cmd) => {
                cmd.execute_silent().await?;
                self.refresh_dataflow_list().await?;
            },
            // ... other commands
        }
        
        Ok(())
    }
}
```

### **Context-Aware TUI Launching**

```rust
// src/tui/launcher.rs - Smart TUI launching
pub struct TuiLauncher;

impl TuiLauncher {
    pub async fn launch_for_command(
        command: &Command,
        context: &ExecutionContext,
    ) -> Result<()> {
        let initial_view = match command {
            Command::Inspect(cmd) => {
                match cmd.resource_type {
                    Some(ResourceType::Node) => ViewType::NodeInspector { 
                        node_id: cmd.target.clone() 
                    },
                    Some(ResourceType::Dataflow) => ViewType::DataflowManager,
                    _ => ViewType::Dashboard, // Auto-detect and navigate
                }
            },
            Command::Debug(cmd) => ViewType::DebugSession { 
                dataflow_id: cmd.dataflow.clone() 
            },
            Command::Analyze(cmd) => ViewType::RecordingAnalyzer { 
                recording_id: cmd.recording.clone() 
            },
            Command::Monitor(_) => ViewType::SystemMonitor,
            Command::Logs(cmd) => ViewType::LogViewer { 
                target: cmd.target.clone() 
            },
            _ => ViewType::Dashboard,
        };
        
        let cli_context = CliContext::from_command(command);
        DoraApp::launch_with_context(initial_view, cli_context).await
    }
}
```

---

## 🚦 Implementation Roadmap

### **Phase 1: Foundation (2-3 weeks)**

#### Week 1: Core CLI Structure
- [ ] Implement hybrid command framework with clap
- [ ] Create execution context detection
- [ ] Build interface selection engine  
- [ ] Add basic configuration system

#### Week 2: Docker-like Commands
- [ ] Implement `dora ps` with smart hints
- [ ] Create `dora start/stop` with progress feedback
- [ ] Build `dora logs` with TUI suggestions
- [ ] Add `dora build` with enhanced output

#### Week 3: Interface Integration
- [ ] Create TUI launcher framework
- [ ] Implement CLI ↔ TUI transitions
- [ ] Add command mode in TUI
- [ ] Build preference system

### **Phase 2: Smart Suggestions (2-3 weeks)**

#### Week 1: Context Detection
- [ ] Implement complexity calculation algorithms
- [ ] Create resource analysis system
- [ ] Build automation context detection
- [ ] Add user preference handling

#### Week 2: Enhanced Commands
- [ ] Build smart `dora inspect` command
- [ ] Create `dora debug` with auto-TUI
- [ ] Implement `dora analyze` interface
- [ ] Add `dora monitor` with live updates

#### Week 3: User Experience Polish
- [ ] Add helpful hints and suggestions
- [ ] Implement progress indicators
- [ ] Create error handling with suggestions
- [ ] Build comprehensive help system

### **Phase 3: TUI Implementation (3-4 weeks)**

#### Week 1: Core TUI Framework
- [ ] Build main TUI application structure
- [ ] Create unified theme and styling
- [ ] Implement view management system
- [ ] Add event handling framework

#### Week 2: Primary Views
- [ ] Implement dashboard view
- [ ] Create dataflow manager interface
- [ ] Build node inspector with live data
- [ ] Add system monitor dashboard

#### Week 3: Advanced Views
- [ ] Create log viewer with filtering
- [ ] Build recording analyzer interface
- [ ] Implement debug session view
- [ ] Add settings management interface

#### Week 4: Integration & Polish
- [ ] Perfect CLI-TUI transitions
- [ ] Add keyboard shortcuts
- [ ] Implement mouse support
- [ ] Create comprehensive testing

### **Phase 4: Advanced Features (2-3 weeks)**

#### Week 1: Data Visualization
- [ ] Add real-time charts and graphs
- [ ] Implement ASCII-art node topology
- [ ] Create performance metric displays
- [ ] Build timeline visualizations

#### Week 2: Interactive Features
- [ ] Add interactive debugging tools
- [ ] Implement message inspection
- [ ] Create configuration editing
- [ ] Build batch operations interface

#### Week 3: Polish & Documentation
- [ ] Performance optimization
- [ ] Comprehensive error handling
- [ ] User documentation and guides
- [ ] Video tutorials and examples

---

## 🎯 Success Metrics

### **User Experience Goals**
- **Onboarding Time**: <10 minutes from install to first dataflow
- **Task Completion**: 90% of common tasks completable without documentation
- **Interface Satisfaction**: Users prefer hybrid approach over pure CLI/TUI
- **Error Recovery**: 95% of errors include actionable next steps

### **Technical Goals**
- **Performance**: TUI launches in <500ms
- **Responsiveness**: Live updates with <100ms latency
- **Compatibility**: 100% backward compatibility with existing scripts
- **Reliability**: Zero crashes in normal operation

### **Adoption Metrics**
- **TUI Usage**: 60% of interactive sessions use TUI features
- **CLI Retention**: 100% of automation workflows continue working
- **User Preference**: 80% of users prefer hybrid over alternatives
- **Productivity**: 40% reduction in time-to-resolution for debugging tasks

---

This hybrid architecture gives you the **ultimate CLI experience**: Docker-like simplicity when you need it fast, intelligent suggestions when complexity increases, and powerful TUI interfaces when the task demands it. Users get the best tool for each job without having to choose a single paradigm.

<function_calls>
<invoke name="TodoWrite">
<parameter name="todos">[{"id": "1", "content": "Create hybrid CLI architecture document", "status": "completed", "priority": "high"}, {"id": "2", "content": "Design progressive disclosure UX patterns", "status": "completed", "priority": "high"}, {"id": "3", "content": "Implement smart context detection logic", "status": "completed", "priority": "high"}, {"id": "4", "content": "Create implementation roadmap for hybrid approach", "status": "completed", "priority": "high"}]