# Dora CLI Manual: Developer & User Guide

## Table of Contents

1. [Introduction](#introduction)
2. [Quick Start](#quick-start)
3. [Architecture Overview](#architecture-overview)
4. [CLI Commands](#cli-commands)
5. [TUI Interface](#tui-interface)
6. [Development Workflows](#development-workflows)
7. [Configuration](#configuration)
8. [Troubleshooting](#troubleshooting)
9. [Contributing](#contributing)

---

## Introduction

The Dora CLI is a hybrid command-line interface that combines traditional CLI commands with an interactive Terminal User Interface (TUI). It provides progressive disclosure from simple commands to rich interactive visualizations.

### Key Features

- **Hybrid CLI/TUI Architecture**: Start with simple commands, escalate to TUI when needed
- **Smart Suggestions**: Context-aware recommendations and automatic TUI transitions
- **Real-time Monitoring**: Live system metrics and dataflow visualization
- **Interactive Analysis**: Advanced data visualization and statistical analysis tools
- **Collaborative Features**: Multi-user debugging and shared sessions (scaffolded)
- **Adaptive Interface**: Learns from user behavior and optimizes suggestions

### Target Audience

- Dora developers working with dataflow systems
- DevOps engineers managing Dora deployments
- Team leads implementing development workflows
- Contributors to the Dora ecosystem

---

## Quick Start

### Installation

```bash
# Build from source
git clone https://github.com/dora-rs/dora
cd dora
cargo build --package dora-cli --release

# Install locally
cargo install --path binaries/cli --force

# Verify installation
dora --version
```

### First Steps

```bash
# Check available commands
dora --help

# Launch interactive TUI dashboard
dora tui

# Start with a specific view
dora tui --view dashboard
dora tui --view dataflow
dora tui --view performance
```

### Essential Commands

```bash
# Dataflow operations
dora ps              # List running dataflows
dora start           # Start dataflow
dora stop            # Stop dataflow
dora logs            # View logs

# Smart inspection
dora inspect         # Intelligent resource inspection
dora debug           # Enhanced debugging with auto-TUI
dora analyze         # Multi-modal analysis

# Monitoring
dora monitor         # Real-time system monitoring
dora dashboard       # Launch monitoring dashboard
```

---

## Architecture Overview

### Design Philosophy

The Dora CLI implements a **Progressive Disclosure** architecture:

```
CLI Commands → Smart Analysis → TUI Interface → Advanced Features
     ↓              ↓                ↓                ↓
  Simple         Context-        Interactive     Collaborative
   Fast          Aware           Rich UX          Real-time
```

### Core Components

#### 1. Command Layer (`binaries/cli/src/cli/`)
- **Basic Commands**: Docker-like operations (ps, start, stop, logs)
- **Smart Commands**: Context-aware analysis (inspect, debug, analyze)
- **TUI Commands**: Interactive interface launchers (tui, dashboard, monitor)

#### 2. TUI Layer (`binaries/cli/src/tui/`)
- **Views**: Individual screens (dashboard, dataflow, performance, etc.)
- **Components**: Reusable UI widgets (charts, tables, graphs)
- **Layouts**: Adaptive layout management system

#### 3. Intelligence Layer (`binaries/cli/src/`)
- **Analysis**: Complexity assessment and performance analysis
- **Preferences**: User behavior learning and adaptation
- **Context**: Environment detection and smart suggestions

#### 4. Configuration (`binaries/cli/src/config/`)
- **Behavioral Learning**: Learns user preferences over time
- **Context Preferences**: Adapts to different execution contexts
- **Preferences Management**: User settings and customization

---

## CLI Commands

### Basic Commands

#### `dora ps`
List running dataflows and their status.

```bash
# List all dataflows
dora ps

# Detailed output
dora ps --verbose

# JSON output for scripting
dora ps --output json
```

#### `dora start`
Start a dataflow with the specified configuration.

```bash
# Start with default config
dora start dataflow.yaml

# Start in debug mode
dora start dataflow.yaml --debug

# Start with monitoring
dora start dataflow.yaml --monitor
```

#### `dora stop`
Stop running dataflows.

```bash
# Stop by name
dora stop my-dataflow

# Stop all
dora stop --all

# Force stop
dora stop my-dataflow --force
```

#### `dora logs`
View dataflow logs with intelligent filtering.

```bash
# Stream all logs
dora logs

# Filter by level
dora logs --level error

# Specific dataflow
dora logs my-dataflow

# Smart log analysis (may suggest TUI)
dora logs --analyze
```

### Smart Commands

These commands include intelligence features that automatically suggest TUI mode for complex operations.

#### `dora inspect`
Intelligent resource inspection with automatic complexity detection.

```bash
# Basic inspection
dora inspect

# Inspect specific resource
dora inspect node camera

# Deep inspection (may auto-launch TUI)
dora inspect --deep dataflow

# Export inspection report
dora inspect --export report.json
```

**Smart Behavior:**
- Detects resource complexity
- Automatically suggests TUI for complex resources
- Provides historical comparison
- Offers optimization recommendations

#### `dora debug`
Enhanced debugging with automatic issue detection.

```bash
# Start debug session
dora debug

# Auto-detect issues
dora debug --auto

# Debug specific node
dora debug node detector

# Collaborative debug (Phase 1 scaffolding)
dora debug --collaborate --session team-debug
```

**Features:**
- Automatic breakpoint suggestions
- Performance bottleneck detection
- Real-time state inspection
- Collaborative debugging support (scaffolded)

#### `dora analyze`
Multi-modal analysis with adaptive interface selection.

```bash
# General analysis
dora analyze

# Performance analysis
dora analyze performance

# Resource utilization
dora analyze resources

# With ML-based predictions
dora analyze trends --predict
```

**Analysis Types:**
- Performance metrics and bottlenecks
- Resource utilization patterns
- Trend detection and prediction
- Complexity assessment

#### `dora monitor`
Real-time system monitoring with alerting.

```bash
# Start monitoring
dora monitor

# Continuous monitoring
dora monitor --continuous

# With alerts
dora monitor --alert-thresholds config.yaml

# Monitor specific components
dora monitor --components "cpu,memory,network"
```

#### `dora help`
Smart help system with tutorials and contextual guidance.

```bash
# General help
dora help

# Interactive help with tutorials
dora help --interactive

# Command-specific help
dora help inspect

# Show examples
dora help --examples
```

### TUI Commands

#### `dora tui`
Launch the main Terminal User Interface.

```bash
# Launch main dashboard
dora tui

# Start with specific view
dora tui --view dashboard
dora tui --view dataflow
dora tui --view performance
dora tui --view logs

# Collaborative mode (Phase 1 scaffolding)
dora tui --collaborate --session team-session
```

**Available Views:**
- `dashboard`: System overview with real-time metrics
- `dataflow`: Interactive dataflow graph explorer
- `performance`: Performance analysis and monitoring
- `logs`: Advanced log viewer with filtering

**Alias:** The command `dora ui` also works as an alias for backwards compatibility.

#### `dora dashboard`
Launch the monitoring dashboard directly.

```bash
# Full dashboard
dora dashboard

# Custom layout
dora dashboard --layout custom.yaml

# Performance-focused
dora dashboard --focus performance
```

### Interface Modes

All commands support explicit interface mode selection:

```bash
# Force CLI output
dora inspect --ui-mode cli

# Force TUI interface
dora inspect --ui-mode tui

# Auto-select (default)
dora inspect --ui-mode auto

# Minimal output (for scripts)
dora inspect --ui-mode minimal
```

### Output Formats

Control output format for scripting and automation:

```bash
# Auto-detect format (default)
dora ps --output auto

# Human-readable table
dora ps --output table

# Machine-readable JSON
dora ps --output json

# YAML format
dora ps --output yaml

# Minimal text output
dora ps --output minimal
```

---

## TUI Interface

### Navigation

**Global Keys:**
- `Tab`: Cycle through panels/sections
- `Shift+Tab`: Cycle backwards
- `↑↓` or `j/k`: Navigate lists
- `Enter`: Select/drill down
- `Esc` or `q`: Back/quit
- `?`: Context-sensitive help
- `r`: Refresh data
- `Ctrl+C`: Exit

### Dashboard View

The main dashboard provides comprehensive system overview.

**Layout:**
```
┌─ System Overview ────────────┐┌─ Performance Metrics ─────┐
│ Status: ●Running             ││ CPU: 65%   Memory: 2.1GB  │
│ Dataflows: 3 active          ││ Network: ↑1.2MB/s         │
│ Nodes: 12/12                 ││ Latency: 8ms avg          │
└──────────────────────────────┘└───────────────────────────┘

┌─ Active Dataflows ────────────────────────────────────────┐
│ ● robot-vision    3 nodes     CPU: 45%    Mem: 1.2GB     │
│ ● sensor-fusion   5 nodes     CPU: 30%    Mem: 640MB     │
│ ● control-loop    4 nodes     CPU: 15%    Mem: 256MB     │
└───────────────────────────────────────────────────────────┘

┌─ Recent Events ───────────────────────────────────────────┐
│ 14:32:15 [INFO]  detector: Processing frame 12,345       │
│ 14:32:14 [WARN]  tracker: High CPU usage                 │
└───────────────────────────────────────────────────────────┘
```

**Features:**
- Real-time system metrics (Phase 1)
- Active dataflow monitoring (Phase 1)
- Recent events timeline (Phase 1)
- Performance indicators (Phase 1)
- Component-based architecture with customizable layouts

### Dataflow Explorer

Interactive visualization of dataflow graphs.

**Features:**
- Node relationship visualization (Phase 1 scaffolding)
- Real-time data flow animation (planned for Phase 2)
- Interactive node selection (Phase 1 scaffolding)
- Performance overlays (planned for Phase 2)
- Zoom and pan controls (planned for Phase 2)

**Controls:**
- `Space`: Play/pause animation (Phase 2)
- `f`: Fit to screen (Phase 2)
- `+/-`: Zoom in/out (Phase 2)

### Data Visualization View

Advanced data visualization with multiple chart types.

**Chart Types (Phase 1):**
1. **Line Chart**: Time-series data with trends
2. **Bar Chart**: Category comparisons
3. **Scatter Plot**: Correlation analysis
4. **Gauge**: Real-time metrics
5. **Timeline**: Event sequences

**Navigation:**
- `Tab/Shift+Tab`: Switch chart types
- `↑↓`: Select data series
- `r`: Refresh data
- `e`: Export chart (planned for Phase 2)

**Mock Data (Phase 1):**
- Realistic performance metrics
- CPU/Memory/Network trends
- Event timelines
- System health gauges

### Analysis Tools View

Interactive statistical analysis and insights.

**Analysis Types (Phase 1):**

1. **Distribution Analysis**
   - Mean, median, standard deviation
   - Quartiles (Q1, Q3)
   - Skewness and kurtosis
   - Sample statistics

2. **Correlation Analysis**
   - Variable correlations
   - Strength classification (weak/moderate/strong)
   - P-value significance testing
   - Correlation matrices

3. **Trend Analysis**
   - Trend direction detection (rising/falling/stable)
   - Trend strength measurement
   - Confidence intervals
   - Change rate calculation

4. **Outlier Detection**
   - Z-score based detection
   - Severity classification (mild/moderate/severe)
   - Outlier percentage
   - Top outliers display

**Navigation:**
- `Tab/Shift+Tab`: Switch analysis types
- `↑↓`: Select items
- `r`: Refresh analysis

**Future (Phase 2):**
- Real statistical tests (chi-square, t-test, ANOVA)
- ML-based anomaly detection
- Integration with actual dataflow metrics
- Export functionality

### Collaborative Features View

Team collaboration and development tools (Phase 1 scaffolding).

**Sections:**

1. **Team Workspace**
   - Team member list with roles (Owner, Admin, Developer, Viewer, Guest)
   - Presence status (Online, Away, Busy, Offline)
   - Contribution tracking
   - Role-based permissions (scaffolded)

2. **Live Sessions**
   - Active collaboration sessions
   - Session types: Code Review, Pair Programming, Team Debug, Live Demo
   - Participant tracking
   - Activity monitoring

3. **Code Reviews**
   - Review status tracking (Draft, Pending, InReview, Approved, Merged)
   - File changes and comments
   - Approval tracking
   - Automated analysis placeholders

4. **Knowledge Base**
   - Articles by category (Getting Started, Best Practices, Troubleshooting, etc.)
   - Difficulty levels (Beginner, Intermediate, Advanced, Expert)
   - View and engagement metrics
   - Search functionality (planned for Phase 2)

**Navigation:**
- `Tab/Shift+Tab`: Switch sections
- `↑↓`: Select items
- `Enter`: View details (Phase 2)

**Phase 2 Plans:**
- Real workspace management with RBAC
- Live collaboration engine
- Actual code review integration
- Knowledge base with search and content creation

### Performance Analyzer View

Comprehensive performance monitoring and analysis.

**Metrics (Phase 1 scaffolding):**
- CPU usage per node and system-wide
- Memory consumption and allocation patterns
- Network I/O and bandwidth
- Message latency and throughput
- Processing time distribution

**Analysis Features (planned for Phase 2):**
- Bottleneck detection
- Trend prediction
- Optimization suggestions
- Historical comparison

### Node Inspector View

Deep dive into individual node behavior.

**Information Panels (Phase 1 scaffolding):**
- Node configuration and parameters
- Current operational state
- Resource usage metrics
- Message flow statistics
- Error logs and warnings
- Dependencies (upstream/downstream)

**Interactive Features (planned for Phase 2):**
- Live configuration editing
- Safe node restart
- Breakpoint setting
- Message content inspection

### Log Viewer

Advanced log analysis with intelligent filtering.

**Features (Phase 1 scaffolding):**
- Real-time log streaming
- Severity level filtering (Error, Warn, Info, Debug, Trace)
- Pattern matching and search
- Multi-node log aggregation
- Time-based filtering

**Smart Features (planned for Phase 2):**
- Error pattern detection
- Anomaly highlighting
- Performance correlation
- Export capabilities

---

## Development Workflows

### Workflow 1: Starting a New Project

```bash
# 1. Check system status
dora ps

# 2. Start your dataflow
dora start my-dataflow.yaml

# 3. Monitor in real-time
dora tui --view dashboard

# 4. Check logs if issues arise
dora logs --level error

# 5. Stop when done
dora stop my-dataflow
```

### Workflow 2: Debugging Performance Issues

```bash
# 1. Run analysis
dora analyze performance

# 2. If complex, CLI suggests TUI automatically
# Accept suggestion or force TUI mode:
dora analyze performance --ui-mode tui

# 3. Use Performance Analyzer view
# (automatically opens in TUI)

# 4. Inspect specific nodes
dora inspect node detector --deep

# 5. Debug with enhanced tools
dora debug --auto node detector
```

### Workflow 3: Team Collaboration (Phase 1 Scaffolding)

```bash
# 1. Start collaborative session
dora debug --collaborate --session team-debug

# 2. Share session with team
# (Invite functionality in Phase 2)

# 3. Collaborative TUI session
dora tui --collaborate --session team-debug

# 4. View team workspace
# (In TUI, navigate to Collaborative Features view)
```

### Workflow 4: Continuous Monitoring

```bash
# 1. Set up continuous monitoring
dora monitor --continuous

# 2. Launch monitoring dashboard
dora dashboard

# 3. Set alert thresholds (config file)
# See Configuration section

# 4. Export metrics for analysis
dora monitor --export metrics.json
```

---

## Configuration

### Configuration File

Location: `~/.config/dora/cli-config.yaml` or `$DORA_CONFIG_PATH`

```yaml
# Core settings
core:
  log_level: "info"
  performance_mode: true

# TUI settings
tui:
  theme: "dark"  # or "light"
  update_rate: 60  # FPS
  animations: true
  mouse_support: true

# Interface behavior
interface:
  auto_tui_threshold: 0.7  # Complexity threshold for auto-TUI
  smart_suggestions: true
  context_learning: true

# Monitoring
monitoring:
  continuous: false
  alert_thresholds:
    cpu: 80
    memory: 90
    network: 95
  export_interval: "5m"

# Collaboration (Phase 1 scaffolding)
collaboration:
  auto_join_sessions: false
  default_role: "developer"
```

### Environment Variables

```bash
# Configuration
export DORA_CONFIG_PATH="/path/to/config.yaml"

# Logging
export DORA_LOG_LEVEL="debug"  # trace|debug|info|warn|error

# TUI
export DORA_TUI_THEME="dark"

# Performance
export DORA_PERFORMANCE_MODE="true"
export DORA_MEMORY_LIMIT="4GB"

# Terminal
export TERM="xterm-256color"  # For TUI compatibility
```

### User Preferences

The CLI learns from your behavior and adapts over time:

- **Command Patterns**: Frequently used command combinations
- **Interface Preferences**: CLI vs TUI mode preferences
- **Complexity Thresholds**: Personal complexity sensitivity
- **View Preferences**: Most used TUI views

Preferences are stored in: `~/.config/dora/preferences.json`

To reset preferences:
```bash
dora config reset-preferences
```

---

## Troubleshooting

### TUI Issues

#### TUI Fails to Launch

**Symptoms:** Error messages, garbled output, immediate exit

**Solutions:**
```bash
# Check terminal compatibility
echo $TERM

# Set compatible terminal type
export TERM=xterm-256color

# Force CLI mode
dora --ui-mode cli status

# Reset terminal
reset && dora tui
```

#### TUI Performance Issues

**Symptoms:** Lag, slow animations, high CPU usage

**Solutions:**
```bash
# Reduce update rate
dora tui --update-rate 30  # 30 FPS instead of 60

# Disable animations
dora config set tui.animations false

# Use performance mode
dora config set core.performance_mode true
```

### Common Issues

#### "Command not found: dora"

```bash
# Add to PATH
export PATH="$HOME/.cargo/bin:$PATH"

# Or reinstall
cargo install --path binaries/cli --force
```

#### "Unrecognized subcommand"

```bash
# Ensure you're using the latest build
cargo build --package dora-cli --release

# Use the local build
./target/release/dora tui

# Or reinstall system-wide
cargo install --path binaries/cli --force
```

#### High Memory Usage

```bash
# Set memory limit
dora config set core.memory_limit "2GB"

# Monitor memory
dora monitor --components memory

# Use minimal mode
dora --ui-mode minimal ps
```

### Debug Mode

For troubleshooting, enable verbose logging:

```bash
# Verbose output
dora -v command

# Debug logging
DORA_LOG_LEVEL=debug dora command

# Trace level (very verbose)
DORA_LOG_LEVEL=trace dora command 2>&1 | tee debug.log
```

---

## Contributing

### Development Setup

```bash
# Clone repository
git clone https://github.com/dora-rs/dora
cd dora

# Build CLI
cargo build --package dora-cli

# Run tests
cargo test --package dora-cli

# Run specific test
cargo test --package dora-cli test_name

# Build release version
cargo build --package dora-cli --release
```

### Adding New TUI Views

1. Create types file: `binaries/cli/src/tui/views/my_view_types.rs`
2. Create view file: `binaries/cli/src/tui/views/my_view.rs`
3. Create tests: `binaries/cli/tests/my_view_test.rs`
4. Update `binaries/cli/src/tui/views/mod.rs`

Example structure:
```rust
// my_view_types.rs
pub struct MyViewState {
    // State fields
}

// my_view.rs
pub struct MyView {
    base: BaseView,
    state: MyViewState,
    theme: ThemeConfig,
}

impl View for MyView {
    fn render(&mut self, frame: &mut Frame, area: Rect, app_state: &AppState) {
        // Render implementation
    }

    async fn handle_key(&mut self, key: KeyEvent, app_state: &mut AppState) -> Result<ViewAction> {
        // Key handling
    }

    async fn update(&mut self, app_state: &mut AppState) -> Result<()> {
        // Update logic
    }

    fn help_text(&self) -> Vec<(&str, &str)> {
        vec![
            ("Tab", "Description"),
            // More help entries
        ]
    }

    fn title(&self) -> &str {
        "My View Title"
    }
}
```

### Testing Guidelines

```bash
# Run all CLI tests
cargo test --package dora-cli

# Run specific view tests
cargo test --package dora-cli --test my_view_test

# Run with output
cargo test --package dora-cli -- --nocapture

# Check code coverage (requires tarpaulin)
cargo tarpaulin --package dora-cli
```

### Code Style

```bash
# Format code
cargo fmt --all

# Check formatting
cargo fmt --all -- --check

# Run clippy
cargo clippy --all

# Fix clippy warnings
cargo clippy --all --fix
```

### Pull Request Process

1. Create feature branch: `git checkout -b feature/my-feature`
2. Make changes with tests
3. Run tests: `cargo test --package dora-cli`
4. Format code: `cargo fmt --all`
5. Commit: `git commit -m "feat: Add my feature"`
6. Push: `git push origin feature/my-feature`
7. Create PR on GitHub

### Documentation

- Update this manual for new features
- Add inline documentation to code
- Include examples in help text
- Write integration tests that serve as examples

---

## Roadmap

### Current Status (Phase 1)

**Completed:**
- ✅ Hybrid CLI/TUI architecture
- ✅ Smart command suggestions
- ✅ Progressive disclosure pattern
- ✅ Dashboard with real-time metrics (scaffolding)
- ✅ Data visualization (5 chart types with mock data)
- ✅ Analysis tools (4 analysis types with mock data)
- ✅ Collaborative features (UI scaffolding)
- ✅ Context-aware interface selection
- ✅ Behavioral learning system (basic)
- ✅ Theme support

### Phase 2 (Planned)

**Integration & Real Data:**
- 🔄 Connect views to real dataflow metrics
- 🔄 Real-time data streaming
- 🔄 Statistical test implementations
- 🔄 ML-based anomaly detection
- 🔄 Performance optimization engine

**Collaboration:**
- 🔄 Real workspace management with RBAC
- 🔄 Live collaboration engine
- 🔄 Code review integration with VCS
- 🔄 Knowledge base with search
- 🔄 Real-time synchronization

**Advanced Features:**
- 🔄 Advanced context awareness (Issue #35)
- 🔄 Performance optimization suite (Issue #36)
- 🔄 Plugin system for extensibility
- 🔄 Custom dashboard layouts
- 🔄 Export and reporting tools

---

## Appendix

### Keyboard Shortcuts Reference

**Global:**
- `Ctrl+C`: Exit/Quit
- `?`: Show help
- `Esc`: Back/Cancel
- `q`: Quit current view

**Navigation:**
- `Tab`: Next panel/section
- `Shift+Tab`: Previous panel/section
- `↑/↓` or `j/k`: Up/Down
- `Enter`: Select/Open
- `r`: Refresh

**Views:**
- `1-9`: Quick switch views (dashboard-specific)
- `f`: Toggle fullscreen (where applicable)
- `Space`: Play/pause (animations)

### Glossary

- **Dataflow**: A graph of connected nodes processing data
- **Node**: A processing unit in a dataflow graph
- **TUI**: Terminal User Interface
- **CLI**: Command Line Interface
- **Progressive Disclosure**: Starting simple and escalating to complex interfaces as needed
- **Mock Data**: Placeholder data for demonstration (Phase 1)
- **Scaffolding**: UI framework implemented, awaiting backend integration (Phase 2)

### External Resources

- **Dora Repository**: https://github.com/dora-rs/dora
- **Documentation**: https://dora.carsmos.ai
- **Issue Tracker**: https://github.com/dora-rs/dora/issues
- **Discussions**: https://github.com/dora-rs/dora/discussions

### Version History

- **v0.3.13**: Current version
  - Hybrid CLI/TUI architecture
  - Phase 1 scaffolding complete
  - Smart suggestions and context awareness
  - Data visualization and analysis tools
  - Collaborative features UI (scaffolding)

---

*This manual corresponds to the current implementation as of the Phase 4 TUI scaffolding work (Issues #29-34). It will be updated as new features are implemented and integrated in Phase 2.*

**Last Updated**: 2025-01-27
**Maintainers**: Dora Development Team
