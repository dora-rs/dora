# Dora CLI: TUI-First Architecture with Ratatui

> Transforming Dora CLI into a modern, interactive terminal interface that rivals GUI applications in usability and visual appeal.

## 🎯 TUI-First Philosophy

**Core Insight**: Modern developers expect rich, interactive interfaces even in terminal environments. By building on Ratatui, we can create a CLI that feels more like a modern application than traditional command-line tools.

### Why TUI-First Makes Sense for Dora

1. **Visual Data Flows**: Dora's dataflow paradigm is inherently visual - TUI can show real-time node graphs
2. **Complex State Management**: Multiple dataflows, nodes, and system resources benefit from structured layouts
3. **Real-time Monitoring**: Live data streams and metrics are perfect for TUI dashboards
4. **Developer Experience**: Interactive debugging and exploration vs. static text output
5. **Unique Positioning**: Most robotics/dataflow tools lack modern interfaces

### Hybrid Command + TUI Architecture

```bash
# Traditional commands for automation/scripting (CI/CD friendly)
dora dataflow start pipeline.yml --detach
dora node list --output json
dora system status --quiet

# Interactive TUI modes for development and debugging
dora ui                           # Launch main TUI dashboard
dora ui dataflow                  # Dataflow management interface
dora ui monitor                   # Real-time system monitoring
dora ui debug <dataflow-id>       # Interactive debugging session
dora ui recording <recording-id>  # Recording analysis interface
```

---

## 🏗️ TUI Architecture Design

### Core TUI Framework Structure

```rust
// src/tui/mod.rs - Main TUI framework
use ratatui::{
    backend::{Backend, CrosstermBackend},
    layout::{Constraint, Direction, Layout, Rect},
    style::{Color, Modifier, Style},
    terminal::{Frame, Terminal},
    widgets::*,
};

pub struct DoraApp {
    /// Current active view
    current_view: ViewType,
    /// Global application state
    state: AppState,
    /// View stack for navigation
    view_stack: Vec<ViewType>,
    /// Event handler
    events: EventHandler,
    /// Data providers
    data_sources: DataSources,
}

#[derive(Clone, Debug)]
pub enum ViewType {
    Dashboard,              // Main overview
    DataflowManager,        // Dataflow lifecycle management
    NodeInspector,          // Individual node details
    SystemMonitor,          // Resource monitoring
    RecordingStudio,        // Recording/replay interface
    LogViewer,              // Structured log viewing
    GraphVisualizer,        // Visual dataflow graphs
    SettingsManager,        // Configuration interface
}

impl DoraApp {
    pub async fn run(&mut self) -> Result<()> {
        let stdout = io::stdout();
        let backend = CrosstermBackend::new(stdout);
        let mut terminal = Terminal::new(backend)?;
        
        // Setup terminal
        enable_raw_mode()?;
        execute!(terminal.backend_mut(), EnterAlternateScreen, EnableMouseCapture)?;
        
        // Main event loop
        loop {
            terminal.draw(|f| self.ui(f))?;
            
            if let Ok(event) = self.events.next().await {
                if self.handle_event(event).await? {
                    break;
                }
            }
        }
        
        // Cleanup
        disable_raw_mode()?;
        execute!(terminal.backend_mut(), LeaveAlternateScreen, DisableMouseCapture)?;
        terminal.show_cursor()?;
        
        Ok(())
    }
    
    fn ui<B: Backend>(&mut self, f: &mut Frame<B>) {
        match self.current_view {
            ViewType::Dashboard => self.render_dashboard(f),
            ViewType::DataflowManager => self.render_dataflow_manager(f),
            ViewType::NodeInspector => self.render_node_inspector(f),
            ViewType::SystemMonitor => self.render_system_monitor(f),
            ViewType::RecordingStudio => self.render_recording_studio(f),
            ViewType::LogViewer => self.render_log_viewer(f),
            ViewType::GraphVisualizer => self.render_graph_visualizer(f),
            ViewType::SettingsManager => self.render_settings_manager(f),
        }
    }
}
```

### Unified Design System

```rust
// src/tui/theme.rs - Consistent styling across all views
pub struct DoraTheme {
    pub primary: Color,
    pub secondary: Color,
    pub success: Color,
    pub warning: Color,
    pub error: Color,
    pub background: Color,
    pub text: Color,
}

impl Default for DoraTheme {
    fn default() -> Self {
        Self {
            primary: Color::Cyan,
            secondary: Color::Blue,
            success: Color::Green,
            warning: Color::Yellow,
            error: Color::Red,
            background: Color::Black,
            text: Color::White,
        }
    }
}

// Consistent widget styling
pub fn styled_block(title: &str, theme: &DoraTheme) -> Block {
    Block::default()
        .title(title)
        .borders(Borders::ALL)
        .border_style(Style::default().fg(theme.primary))
        .title_style(Style::default().fg(theme.text).add_modifier(Modifier::BOLD))
}

pub fn status_style(status: &str, theme: &DoraTheme) -> Style {
    match status.to_lowercase().as_str() {
        "running" | "active" | "healthy" => Style::default().fg(theme.success),
        "warning" | "degraded" => Style::default().fg(theme.warning),
        "error" | "failed" | "stopped" => Style::default().fg(theme.error),
        _ => Style::default().fg(theme.text),
    }
}
```

---

## 📊 Main Dashboard Interface

### Primary Dashboard Layout

```rust
impl DoraApp {
    fn render_dashboard<B: Backend>(&mut self, f: &mut Frame<B>) {
        let size = f.size();
        
        // Main layout: header + body + footer
        let chunks = Layout::default()
            .direction(Direction::Vertical)
            .constraints([
                Constraint::Length(3),  // Header
                Constraint::Min(0),     // Body
                Constraint::Length(3),  // Footer
            ])
            .split(size);
        
        self.render_header(f, chunks[0]);
        self.render_dashboard_body(f, chunks[1]);
        self.render_footer(f, chunks[2]);
    }
    
    fn render_dashboard_body<B: Backend>(&mut self, f: &mut Frame<B>, area: Rect) {
        let chunks = Layout::default()
            .direction(Direction::Horizontal)
            .constraints([
                Constraint::Ratio(1, 3),  // System overview
                Constraint::Ratio(2, 3),  // Main content
            ])
            .split(area);
        
        // Left panel: System overview
        self.render_system_overview(f, chunks[0]);
        
        // Right panel: Context-specific content
        self.render_main_content(f, chunks[1]);
    }
}
```

### Dashboard Visual Example

```
┌─ Dora Control Center ─────────────────────────────────── 14:32:45 ┐
│ 🚀 v0.3.13 | Daemon: ● Running | Coordinator: ● Running          │
└────────────────────────────────────────────────────────────────────┘
┌─ System ─────────────┐ ┌─ Active Dataflows ─────────────────────────┐
│                      │ │                                            │
│ 📊 CPU:  60% ████████ │ │ ┌─ demo-pipeline ──────────────────────┐   │
│ 💾 RAM:  35% ████░░░░ │ │ │ Status: ● Running (2h 34m)           │   │
│ 💿 Disk: 12% ██░░░░░░ │ │ │ Nodes: 4 active, 0 failed           │   │
│ 🌐 Net:  ↑15MB/s     │ │ │ ├─ camera-node      ● 30Hz           │   │
│                      │ │ │ ├─ yolo-detector    ● 29Hz           │   │
│ 🔧 Dataflows:    3   │ │ │ ├─ tracker          ● 15Hz           │   │
│ ⚙️  Nodes:       12  │ │ │ └─ visualizer      ● 30Hz           │   │
│ 📹 Recordings:   5   │ │ └──────────────────────────────────────┘   │
│ 🚨 Alerts:       2   │ │                                            │
│                      │ │ ┌─ vision-analysis ─────────────────────┐   │
│ ┌─ Quick Actions ──┐ │ │ │ Status: ⚠️ Warning (5m 12s)          │   │
│ │ [d] Dataflows   │ │ │ │ Issue: High latency detected          │   │
│ │ [n] Nodes       │ │ │ │ Nodes: 5 active, 1 warning           │   │
│ │ [m] Monitor     │ │ │ └──────────────────────────────────────┘   │
│ │ [r] Recordings  │ │ │                                            │
│ │ [l] Logs        │ │ │ ┌─ Recent Activity ──────────────────────┐   │
│ │ [s] Settings    │ │ │ │ 14:32:43  demo-pipeline started       │   │
│ │ [q] Quit        │ │ │ │ 14:31:22  vision-analysis warning     │   │
│ └─────────────────┘ │ │ │ 14:30:15  Recording rec-003 completed │   │
└──────────────────────┘ │ │ 14:29:45  New node registered         │   │
                         │ └────────────────────────────────────────┘   │
                         └────────────────────────────────────────────┘
┌─ Navigation ──────────────────────────────────────────────────────┐
│ [Tab] Next Panel | [Enter] Select | [Esc] Back | [F1] Help        │
└────────────────────────────────────────────────────────────────────┘
```

---

## 🔄 Interactive Dataflow Manager

### Dataflow Management Interface

```rust
pub struct DataflowManagerView {
    dataflows: StatefulList<DataflowInfo>,
    selected_dataflow: Option<DataflowInfo>,
    action_mode: ActionMode,
    filter: String,
    sort_by: SortBy,
}

#[derive(Clone)]
pub enum ActionMode {
    Viewing,
    Creating,
    Editing,
    Confirming(Action),
}

impl DataflowManagerView {
    fn render<B: Backend>(&mut self, f: &mut Frame<B>, area: Rect) {
        let chunks = Layout::default()
            .direction(Direction::Horizontal)
            .constraints([
                Constraint::Ratio(1, 2),  // Dataflow list
                Constraint::Ratio(1, 2),  // Details/Actions
            ])
            .split(area);
        
        self.render_dataflow_list(f, chunks[0]);
        
        match self.action_mode {
            ActionMode::Viewing => self.render_dataflow_details(f, chunks[1]),
            ActionMode::Creating => self.render_create_form(f, chunks[1]),
            ActionMode::Editing => self.render_edit_form(f, chunks[1]),
            ActionMode::Confirming(ref action) => self.render_confirmation(f, chunks[1], action),
        }
    }
    
    fn render_dataflow_list<B: Backend>(&mut self, f: &mut Frame<B>, area: Rect) {
        let items: Vec<ListItem> = self.dataflows
            .items
            .iter()
            .map(|dataflow| {
                let status_symbol = match dataflow.status {
                    DataflowStatus::Running => "●",
                    DataflowStatus::Stopped => "○",
                    DataflowStatus::Error => "✗",
                    DataflowStatus::Starting => "◐",
                };
                
                let content = vec![Spans::from(vec![
                    Span::styled(
                        format!("{} ", status_symbol),
                        status_style(&dataflow.status.to_string(), &self.theme)
                    ),
                    Span::raw(&dataflow.name),
                    Span::styled(
                        format!(" ({})", dataflow.uptime_display()),
                        Style::default().fg(Color::Gray)
                    ),
                ])];
                
                ListItem::new(content)
            })
            .collect();
        
        let list = List::new(items)
            .block(styled_block("Dataflows", &self.theme))
            .highlight_style(Style::default().bg(Color::DarkGray))
            .highlight_symbol("▶ ");
        
        f.render_stateful_widget(list, area, &mut self.dataflows.state);
    }
}
```

### Visual Dataflow Editor

```
┌─ Dataflow Manager ─────────────────────────────────────────────────┐
│ Filter: [vision_____] Sort: [Name ▼] [F2] New [F3] Edit [Del] Remove │
└────────────────────────────────────────────────────────────────────┘
┌─ Dataflows ───────────────┐ ┌─ demo-pipeline ───────────────────────┐
│                           │ │ ┌─ Overview ─────────────────────────┐ │
│ ▶ ● demo-pipeline (2h34m) │ │ │ Status: ● Running                  │ │
│   ⚠️ vision-analysis (5m) │ │ │ Started: 2024-01-15 12:00:23      │ │
│   ○ control-system        │ │ │ Uptime: 2h 34m 12s                │ │
│   ○ data-processor        │ │ │ Config: /home/user/demo.yml        │ │
│                           │ │ └────────────────────────────────────┘ │
│                           │ │                                        │
│ [F2] New Dataflow         │ │ ┌─ Node Topology ────────────────────┐ │
│ [F3] Edit Selected        │ │ │     [camera] ──> [yolo] ──> [viz]  │ │
│ [F4] Start/Stop           │ │ │        │           │                │ │
│ [F5] Restart              │ │ │        └──> [tracker] ──────┘      │ │
│ [Del] Remove              │ │ └────────────────────────────────────┘ │
│                           │ │                                        │
│                           │ │ ┌─ Actions ──────────────────────────┐ │
│                           │ │ │ [Enter] Inspect Nodes              │ │
│                           │ │ │ [Space] Start/Stop                 │ │
│                           │ │ │ [r] Restart                        │ │
│                           │ │ │ [l] View Logs                      │ │
│                           │ │ │ [e] Edit Configuration             │ │
│                           │ │ │ [d] Delete                         │ │
│                           │ │ └────────────────────────────────────┘ │
└───────────────────────────┘ └────────────────────────────────────────┘
```

---

## 🔍 Real-time Node Inspector

### Node Inspection Interface

```rust
pub struct NodeInspectorView {
    node_id: String,
    node_info: Option<NodeInfo>,
    active_tab: InspectorTab,
    refresh_interval: Duration,
    auto_refresh: bool,
    message_log: CircularBuffer<LogEntry>,
}

#[derive(Clone)]
pub enum InspectorTab {
    Overview,
    Performance,
    Connections,
    Messages,
    Configuration,
}

impl NodeInspectorView {
    fn render<B: Backend>(&mut self, f: &mut Frame<B>, area: Rect) {
        let chunks = Layout::default()
            .direction(Direction::Vertical)
            .constraints([
                Constraint::Length(3),  // Tab bar
                Constraint::Min(0),     // Content
            ])
            .split(area);
        
        self.render_tab_bar(f, chunks[0]);
        
        match self.active_tab {
            InspectorTab::Overview => self.render_overview(f, chunks[1]),
            InspectorTab::Performance => self.render_performance(f, chunks[1]),
            InspectorTab::Connections => self.render_connections(f, chunks[1]),
            InspectorTab::Messages => self.render_messages(f, chunks[1]),
            InspectorTab::Configuration => self.render_configuration(f, chunks[1]),
        }
    }
    
    fn render_performance<B: Backend>(&mut self, f: &mut Frame<B>, area: Rect) {
        let chunks = Layout::default()
            .direction(Direction::Vertical)
            .constraints([
                Constraint::Ratio(1, 3),  // Resource usage
                Constraint::Ratio(1, 3),  // Throughput graphs
                Constraint::Ratio(1, 3),  // Latency metrics
            ])
            .split(area);
        
        // Real-time sparkline charts for performance metrics
        self.render_resource_usage(f, chunks[0]);
        self.render_throughput_chart(f, chunks[1]);
        self.render_latency_metrics(f, chunks[2]);
    }
}
```

### Node Inspector Visual

```
┌─ Node Inspector: camera-node ─────────────────────────────────────┐
│ [1] Overview [2] Performance [3] Connections [4] Messages [5] Config │
└────────────────────────────────────────────────────────────────────┘
┌─ Performance Metrics ──────────────────────────────────────────────┐
│ ┌─ Resource Usage ─────────────┐ ┌─ Throughput ─────────────────┐  │
│ │ CPU:  12.3% ████░░░░░░░░     │ │ Input Rate:  30.2 Hz         │  │
│ │ RAM:  245MB ███░░░░░░░░░     │ │ Output Rate: 29.8 Hz         │  │
│ │ GPU:  0%    ░░░░░░░░░░░░     │ │                              │  │
│ │ I/O:  15MB/s ██████░░░░░     │ │ ▁▂▃▄▅▆▇█▇▆▅▄▃▂▁▂▃▄▅▆▇█ 30Hz │  │
│ └──────────────────────────────┘ │ ▁▁▁▂▂▃▃▄▄▅▅▆▆▇▇▇▆▆▅▅▄▄▃ 29Hz │  │
│                                  └──────────────────────────────┘  │
│ ┌─ Latency Distribution ────────────────────────────────────────┐  │
│ │ P50: 2.1ms │ P95: 8.3ms │ P99: 15.2ms │ Max: 23.1ms       │  │
│ │                                                            │  │
│ │   23ms ┤                                               ▄   │  │
│ │   18ms ┤                                         ▄▄▄  ██   │  │
│ │   13ms ┤                                    ▄▄▄ ████ ███   │  │
│ │    8ms ┤                           ▄▄▄▄▄▄ ██████████████   │  │
│ │    3ms ┤    ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄████████████████████████   │  │
│ │    0ms └─────────────────────────────────────────────────   │  │
│ │         0    10   20   30   40   50   60   70   80   90    │  │
│ └────────────────────────────────────────────────────────────┘  │
└────────────────────────────────────────────────────────────────────┘
```

---

## 📹 Recording Studio Interface

### Advanced Recording Management

```rust
pub struct RecordingStudioView {
    recordings: StatefulList<RecordingInfo>,
    active_recording: Option<ActiveRecording>,
    playback_state: PlaybackState,
    timeline: Timeline,
    selected_topics: HashSet<String>,
    view_mode: StudioViewMode,
}

#[derive(Clone)]
pub enum StudioViewMode {
    RecordingList,
    ActiveRecording,
    Playback,
    Analysis,
}

impl RecordingStudioView {
    fn render_playback_interface<B: Backend>(&mut self, f: &mut Frame<B>, area: Rect) {
        let chunks = Layout::default()
            .direction(Direction::Vertical)
            .constraints([
                Constraint::Length(3),   // Playback controls
                Constraint::Length(5),   // Timeline
                Constraint::Min(0),      // Content viewer
            ])
            .split(area);
        
        self.render_playback_controls(f, chunks[0]);
        self.render_timeline(f, chunks[1]);
        self.render_message_viewer(f, chunks[2]);
    }
    
    fn render_timeline<B: Backend>(&mut self, f: &mut Frame<B>, area: Rect) {
        let progress = self.timeline.position() as f64 / self.timeline.duration() as f64;
        let gauge = Gauge::default()
            .block(styled_block("Timeline", &self.theme))
            .gauge_style(Style::default().fg(Color::Cyan))
            .percent((progress * 100.0) as u16)
            .label(format!(
                "{} / {} ({}x speed)",
                format_duration(self.timeline.position()),
                format_duration(self.timeline.duration()),
                self.playback_state.speed
            ));
        
        f.render_widget(gauge, area);
    }
}
```

### Recording Studio Visual

```
┌─ Recording Studio ─────────────────────────────────────────────────┐
│ [1] Recordings [2] Active [3] Playback [4] Analysis              │
└────────────────────────────────────────────────────────────────────┘
┌─ Playback Controls ────────────────────────────────────────────────┐
│ [Space] ⏸️ Pause | [←] Step Back | [→] Step Forward | Speed: 1.0x │
└────────────────────────────────────────────────────────────────────┘
┌─ Timeline ─────────────────────────────────────────────────────────┐
│ ████████████████████░░░░░░░░ 15:32 / 19:45 (78%) - 1.0x speed    │
└────────────────────────────────────────────────────────────────────┘
┌─ Message Viewer ───────────────────────────────────────────────────┐
│ ┌─ Topics ──────────┐ ┌─ Current Message ──────────────────────────┐ │
│ │ ● camera/image    │ │ Topic: camera/image                        │ │
│ │ ● detect/objects  │ │ Timestamp: 2024-01-15T14:30:25.123456Z   │ │
│ │ ○ track/paths     │ │ Size: 2.4 MB (1920x1080 BGR)             │ │
│ │ ○ control/cmd     │ │ Sequence: 1,234                          │ │
│ │                   │ │                                           │ │
│ │ [Enter] Toggle    │ │ Metadata:                                 │ │
│ │ [a] All           │ │ {                                         │ │
│ │ [n] None          │ │   "exposure": "1/60",                     │ │
│ └───────────────────┘ │   "iso": 400,                             │ │
│                       │   "camera_id": "cam_001"                  │ │
│                       │ }                                         │ │
│                       │                                           │ │
│                       │ Raw Data (first 256 bytes):              │ │
│                       │ 89 50 4E 47 0D 0A 1A 0A 00 00 00 0D ...  │ │
│                       └───────────────────────────────────────────┘ │
└────────────────────────────────────────────────────────────────────┘
```

---

## 📊 System Monitor Dashboard

### Real-time Monitoring Interface

```rust
pub struct SystemMonitorView {
    system_metrics: SystemMetrics,
    metric_history: CircularBuffer<SystemSnapshot>,
    selected_metric: MetricType,
    time_window: TimeWindow,
    auto_refresh: bool,
}

impl SystemMonitorView {
    fn render<B: Backend>(&mut self, f: &mut Frame<B>, area: Rect) {
        let chunks = Layout::default()
            .direction(Direction::Vertical)
            .constraints([
                Constraint::Length(8),   // System overview
                Constraint::Min(0),      // Detailed metrics
            ])
            .split(area);
        
        self.render_system_overview(f, chunks[0]);
        self.render_detailed_metrics(f, chunks[1]);
    }
    
    fn render_system_overview<B: Backend>(&mut self, f: &mut Frame<B>, area: Rect) {
        let chunks = Layout::default()
            .direction(Direction::Horizontal)
            .constraints([
                Constraint::Ratio(1, 4),  // CPU
                Constraint::Ratio(1, 4),  // Memory
                Constraint::Ratio(1, 4),  // Network
                Constraint::Ratio(1, 4),  // Disk
            ])
            .split(area);
        
        // Render gauge widgets for each metric
        self.render_cpu_gauge(f, chunks[0]);
        self.render_memory_gauge(f, chunks[1]);
        self.render_network_gauge(f, chunks[2]);
        self.render_disk_gauge(f, chunks[3]);
    }
    
    fn render_detailed_metrics<B: Backend>(&mut self, f: &mut Frame<B>, area: Rect) {
        match self.selected_metric {
            MetricType::Dataflows => self.render_dataflow_metrics(f, area),
            MetricType::Nodes => self.render_node_metrics(f, area),
            MetricType::Performance => self.render_performance_charts(f, area),
            MetricType::Network => self.render_network_analysis(f, area),
        }
    }
}
```

---

## 🎮 Interactive Commands Integration

### Seamless Command + TUI Workflow

```rust
// Enhanced CLI that can launch TUI modes
#[derive(Debug, clap::Subcommand)]
pub enum Command {
    // Traditional commands (CI/CD friendly)
    Dataflow(DataflowCommand),
    Node(NodeCommand),
    System(SystemCommand),
    Recording(RecordingCommand),
    
    // Interactive TUI modes
    Ui(UiCommand),
    Dashboard,                // Shortcut for `ui dashboard`
    Monitor,                  // Shortcut for `ui monitor`
}

#[derive(Debug, clap::Args)]
pub struct UiCommand {
    #[clap(subcommand)]
    mode: Option<UiMode>,
    
    /// Start in specific view
    #[clap(long)]
    view: Option<String>,
    
    /// Auto-refresh interval (seconds)
    #[clap(long, default_value = "1")]
    refresh: u64,
    
    /// Enable mouse support
    #[clap(long)]
    mouse: bool,
}

#[derive(Debug, clap::Subcommand)]
pub enum UiMode {
    Dashboard,              // Main overview
    Dataflow(UiDataflowArgs),  // Dataflow management
    Node(UiNodeArgs),          // Node inspection
    Monitor,                   // System monitoring
    Recording(UiRecordingArgs), // Recording interface
    Debug(UiDebugArgs),        // Interactive debugging
}

// Example usage patterns:
// dora ui                           # Launch main dashboard
// dora ui dataflow                  # Dataflow management TUI
// dora ui node camera-001           # Inspect specific node
// dora ui monitor --refresh 2       # System monitor (2s refresh)
// dora ui recording rec-001         # Recording analysis
// dora ui debug demo-pipeline       # Interactive debugging
```

### Smart Context Switching

```rust
impl DoraApp {
    /// Handle commands that can trigger TUI mode switches
    pub async fn handle_command_integration(&mut self, args: &[String]) -> Result<()> {
        match args[0].as_str() {
            "dataflow" if args.len() > 1 => {
                match args[1].as_str() {
                    "inspect" => {
                        // Switch to dataflow inspector for specific dataflow
                        let dataflow_id = &args[2];
                        self.switch_to_view(ViewType::DataflowManager);
                        self.focus_dataflow(dataflow_id);
                    },
                    "debug" => {
                        // Launch interactive debugging session
                        let dataflow_id = &args[2];
                        self.switch_to_view(ViewType::GraphVisualizer);
                        self.start_debug_session(dataflow_id).await?;
                    },
                    _ => {}
                }
            },
            "logs" if args.contains(&"--interactive".to_string()) => {
                // Switch to interactive log viewer
                self.switch_to_view(ViewType::LogViewer);
            },
            _ => {}
        }
        Ok(())
    }
}
```

---

## 🚀 Implementation Strategy

### Phase 1: TUI Foundation (2-3 weeks)

```rust
// Core TUI infrastructure
// src/tui/
├── mod.rs              // Main TUI app and event loop
├── theme.rs            // Consistent styling and colors
├── widgets/            // Custom widget implementations
│   ├── gauge_chart.rs  // Enhanced gauge with sparklines
│   ├── data_table.rs   // Sortable, filterable tables
│   ├── log_viewer.rs   // Structured log display
│   └── timeline.rs     // Interactive timeline widget
├── views/              // Individual view implementations
│   ├── dashboard.rs    // Main overview dashboard
│   ├── dataflow.rs     // Dataflow management interface
│   ├── node.rs         // Node inspection interface
│   ├── monitor.rs      // System monitoring interface
│   └── recording.rs    // Recording management interface
└── utils/              // Shared utilities
    ├── layout.rs       // Layout helpers and responsive design
    ├── events.rs       // Event handling and input management
    └── data.rs         // Data structures and state management
```

### Phase 2: Core Views (3-4 weeks)

1. **Dashboard Implementation** (Week 1)
   - System overview with live metrics
   - Quick actions and navigation
   - Status indicators and alerts

2. **Dataflow Manager** (Week 2)
   - Interactive dataflow list
   - Visual topology display
   - Lifecycle management (start/stop/restart)

3. **Node Inspector** (Week 3)
   - Real-time performance metrics
   - Message flow visualization
   - Configuration editing

4. **System Monitor** (Week 4)
   - Resource usage charts
   - Process monitoring
   - Performance analytics

### Phase 3: Advanced Features (2-3 weeks)

1. **Recording Studio Interface**
   - Interactive recording management
   - Timeline-based playback
   - Message inspection and analysis

2. **Visual Dataflow Graph**
   - ASCII-art node topology
   - Interactive node selection
   - Real-time data flow visualization

### Dependencies and Tools

```toml
[dependencies]
# Core TUI framework
ratatui = "0.24"
crossterm = "0.27"

# Enhanced widgets and charts
tui-logger = "0.9"          # Structured logging widget
tui-input = "0.8"           # Input field widgets  
ratatui-image = "1.0"       # Image display support

# Data visualization
textplots = "0.8"           # ASCII plots and charts
unicode-segmentation = "1.10" # Text rendering support

# Async runtime for real-time updates
tokio = { version = "1.0", features = ["full"] }
tokio-stream = "0.1"

# Data structures
circular-buffer = "0.1"     # For metric history
indexmap = "2.0"            # Ordered maps for consistent display

# Performance monitoring
sysinfo = "0.29"            # System metrics
procfs = "0.15"             # Linux process information
```

---

## 🎯 Benefits of TUI-First Architecture

### Developer Experience Advantages

1. **Immediate Visual Feedback**: See system state at a glance rather than parsing text output
2. **Interactive Exploration**: Navigate and drill down into details naturally
3. **Real-time Monitoring**: Live updates without manual refresh commands
4. **Consistent Interface**: Unified design language across all operations
5. **Keyboard-Driven Efficiency**: Fast navigation for power users

### Operational Benefits

1. **Reduced Cognitive Load**: Visual organization vs. scrolling through text
2. **Faster Debugging**: Interactive inspection vs. multiple CLI commands
3. **Better Situational Awareness**: Dashboard view of entire system state
4. **Improved Accessibility**: Better for users who prefer visual interfaces

### Competitive Positioning

1. **Unique in Robotics Space**: Most robotics tools have basic CLIs
2. **Modern Developer Expectations**: Matches tools like k9s, htop, gitui
3. **Easier Onboarding**: Visual interfaces are more approachable for new users
4. **Professional Appearance**: Elevates Dora's perceived quality and maturity

### Backward Compatibility

- All existing CLI commands remain functional
- TUI mode is opt-in via `dora ui` commands
- Scripting and automation unchanged
- CI/CD pipelines continue to work

This TUI-first approach transforms Dora CLI from a basic command-line tool into a modern, interactive development environment that significantly improves the developer experience while maintaining full compatibility with existing workflows.

<function_calls>
<invoke name="TodoWrite">
<parameter name="todos">[{"id": "1", "content": "Analyze Ratatui integration opportunities", "status": "completed", "priority": "high"}, {"id": "2", "content": "Redesign CLI architecture with TUI-first approach", "status": "completed", "priority": "high"}, {"id": "3", "content": "Create unified interface design patterns", "status": "completed", "priority": "high"}, {"id": "4", "content": "Update implementation examples with Ratatui", "status": "completed", "priority": "high"}]