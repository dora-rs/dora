# Dora CLI Manual: Complete Developer Guide

## Table of Contents

1. [Introduction](#introduction)
2. [Quick Start Guide](#quick-start-guide)
3. [Architecture Overview](#architecture-overview)
4. [Core CLI Commands](#core-cli-commands)
5. [TUI Interface Guide](#tui-interface-guide)
6. [Use Case Driven Workflows](#use-case-driven-workflows)
7. [Development Guidelines](#development-guidelines)
8. [Advanced Features](#advanced-features)
9. [Troubleshooting](#troubleshooting)
10. [Performance Optimization](#performance-optimization)
11. [Team Collaboration](#team-collaboration)
12. [API Reference](#api-reference)

---

## Introduction

The Dora CLI is a hybrid command-line interface that combines the power of traditional CLI commands with an interactive Terminal User Interface (TUI). This manual provides comprehensive guidance for developers working with Dora projects, serving as both a reference and development guideline.

### Key Features
- **Hybrid CLI/TUI Architecture**: Progressive disclosure from simple commands to rich interactive interfaces
- **Smart Suggestions**: ML-powered command recommendations based on context and user behavior
- **Real-time Monitoring**: Live system metrics and dataflow visualization
- **Collaborative Development**: Multi-user debugging and shared sessions
- **Intelligent Analysis**: Automated complexity assessment and performance optimization suggestions

### Target Audience
- New Dora developers
- Team leads implementing development workflows
- Contributors to the Dora ecosystem
- DevOps engineers managing Dora deployments

---

## Quick Start Guide

### Installation
```bash
# Install Dora CLI
cargo install dora-cli

# Verify installation
dora --version

# Initialize a new project
dora init my-project
cd my-project
```

### First Steps
```bash
# Check system status
dora status

# Run the interactive dashboard
dora tui

# Get contextual help
dora help --interactive
```

### Essential Commands
```bash
dora build          # Build your dataflow graph
dora start          # Start the dataflow
dora stop           # Stop all nodes
dora logs           # View streaming logs
dora inspect        # Inspect resources with smart TUI
```

---

## Architecture Overview

### Hybrid Design Philosophy

The Dora CLI implements a **Progressive Disclosure** architecture where users can:

1. **Start Simple**: Use basic CLI commands for quick operations
2. **Scale Up**: Access TUI interfaces for complex analysis
3. **Go Deep**: Leverage advanced visualization and collaboration features

```
CLI Commands → TUI Views → Advanced Features
     ↓              ↓              ↓
Simple/Fast    Interactive    Collaborative
```

### Core Components

#### 1. Command Layer (`src/commands/`)
- **Basic Commands**: `build`, `start`, `stop`, `status`
- **Smart Commands**: `inspect`, `debug`, `analyze`, `logs`
- **TUI Commands**: `tui`, `dashboard`, `monitor`

#### 2. TUI Layer (`src/tui/`)
- **Dashboard**: System overview and real-time metrics
- **Dataflow Explorer**: Interactive graph visualization
- **Performance Analyzer**: Resource monitoring and optimization
- **Node Inspector**: Deep dive into individual nodes
- **Log Viewer**: Advanced log filtering and pattern detection

#### 3. Intelligence Layer (`src/intelligence/`)
- **Complexity Analysis**: ML-based assessment algorithms
- **Resource Monitor**: Anomaly detection and alerting
- **Preference Learning**: Adaptive user experience
- **Pattern Detection**: Automated issue identification

#### 4. Collaboration Layer (`src/collaboration/`)
- **Shared Sessions**: Multi-user debugging environments
- **Real-time Sync**: Operational transformation for concurrent editing
- **Role Management**: Team-based access control

---

## Core CLI Commands

### Basic Operations

#### `dora build`
Builds the dataflow graph with automatic dependency resolution.

```bash
# Basic build
dora build

# Build with specific configuration
dora build --config production.yaml

# Build with complexity analysis
dora build --analyze

# Interactive build (launches TUI for complex projects)
dora build --interactive
```

**Smart Features:**
- Automatic complexity detection triggers TUI for complex builds
- Dependency conflict resolution with suggestions
- Performance impact analysis

#### `dora start`
Starts the dataflow execution with intelligent resource allocation.

```bash
# Start with default configuration
dora start

# Start specific nodes
dora start --nodes "camera,detector"

# Start with resource monitoring
dora start --monitor

# Start in debug mode
dora start --debug --breakpoints "node1:init,node2:process"
```

**Smart Features:**
- Resource requirement analysis
- Automatic scaling recommendations
- Failure prediction and prevention

#### `dora stop`
Gracefully stops the dataflow with cleanup verification.

```bash
# Stop all nodes
dora stop

# Stop specific nodes
dora stop --nodes "camera,detector"

# Force stop with cleanup verification
dora stop --force --verify-cleanup
```

### Smart Commands

#### `dora inspect`
Intelligent resource inspection with automatic TUI recommendations.

```bash
# Basic inspection
dora inspect

# Inspect specific resource
dora inspect node camera

# Deep inspection (auto-launches TUI for complex resources)
dora inspect --deep node detector

# Export inspection report
dora inspect --export json node_analysis.json
```

**Behavior:**
- Complexity threshold detection automatically suggests TUI
- Historical comparison and trend analysis
- Resource relationship mapping

#### `dora debug`
Comprehensive debugging with automatic issue detection.

```bash
# Start debug session
dora debug

# Debug specific issue
dora debug --issue "high_latency"

# Auto-debug (ML-powered issue detection)
dora debug --auto

# Collaborative debug session
dora debug --share --session "team-debug-001"
```

**Features:**
- Automatic breakpoint suggestions
- Performance bottleneck detection
- Real-time collaboration support

#### `dora analyze`
Multi-modal analysis with adaptive interface selection.

```bash
# General analysis
dora analyze

# Performance analysis
dora analyze performance

# Resource utilization analysis
dora analyze resources --time-range "1h"

# Trend analysis with ML insights
dora analyze trends --predict --horizon "24h"
```

**Capabilities:**
- Statistical analysis and reporting
- Machine learning-based predictions
- Custom analysis plugin support

#### `dora logs`
Intelligent log streaming with pattern detection.

```bash
# Stream all logs
dora logs

# Filter by severity
dora logs --level error

# Pattern-based filtering
dora logs --pattern "timeout|connection"

# Smart log analysis (launches TUI for complex patterns)
dora logs --analyze
```

**Smart Features:**
- Automatic error pattern detection
- Log correlation across nodes
- Performance impact analysis

### TUI Commands

#### `dora tui`
Launch the main TUI interface with context-aware views.

```bash
# Launch main dashboard
dora tui

# Launch specific view
dora tui --view dataflow

# Launch with specific focus
dora tui --focus node:camera

# Collaborative TUI session
dora tui --collaborate --session "team-session"
```

#### `dora dashboard`
System overview with real-time metrics.

```bash
# Full dashboard
dora dashboard

# Performance-focused dashboard
dora dashboard --focus performance

# Custom dashboard layout
dora dashboard --layout custom.yaml
```

---

## TUI Interface Guide

### Dashboard View

The dashboard provides a comprehensive system overview with real-time updates.

#### Layout Components
```
┌─ System Overview ─────────────────────┐┌─ Performance Metrics ────────────────┐
│ Status: ●Running    Nodes: 12/12     ││ CPU: ████████░░ 80%   Memory: 4.2GB  │
│ Uptime: 2h 34m      Errors: 0        ││ Network: ↑1.2MB/s ↓800KB/s          │
│ Dataflow: camera→detector→tracker     ││ Latency: 12ms avg   Throughput: 30fps│
└───────────────────────────────────────┘└───────────────────────────────────────┘

┌─ Active Nodes ────────────────────────────────────────────────────────────────┐
│ ● camera        [Ready]    CPU: 15%   Mem: 256MB   Messages: 1,234           │
│ ● detector      [Running]  CPU: 65%   Mem: 1.2GB   Messages: 1,234           │
│ ● tracker       [Running]  CPU: 25%   Mem: 512MB   Messages: 1,156           │
│ ● output        [Idle]     CPU: 2%    Mem: 64MB    Messages: 1,156            │
└───────────────────────────────────────────────────────────────────────────────┘

┌─ Recent Events ───────────────────────────────────────────────────────────────┐
│ 14:32:15 [INFO]  detector: Processing frame 12,345                           │
│ 14:32:14 [WARN]  tracker: High CPU usage detected                            │
│ 14:32:12 [INFO]  camera: Frame rate stable at 30fps                         │
└───────────────────────────────────────────────────────────────────────────────┘
```

#### Navigation
- `Tab`: Cycle through panels
- `↑↓`: Navigate lists
- `Enter`: Drill down into details
- `Esc`: Return to previous view
- `q`: Quit to CLI
- `h`: Context-sensitive help

### Dataflow Explorer

Interactive visualization of the dataflow graph with real-time data flow.

#### Features
- **Graph Visualization**: Node relationships and data connections
- **Real-time Flow**: Animated data movement between nodes
- **Performance Overlays**: Latency, throughput, and resource usage
- **Interactive Editing**: Drag-and-drop node reconfiguration
- **Zoom and Pan**: Navigate large dataflow graphs

#### Controls
- `Mouse`: Pan and zoom
- `Click`: Select nodes/edges
- `Double-click`: Edit node properties
- `Space`: Play/pause animation
- `r`: Reset view
- `f`: Fit to screen

### Performance Analyzer

Comprehensive performance monitoring with trend analysis.

#### Metrics Displayed
- **CPU Usage**: Per-node and system-wide
- **Memory Consumption**: Allocation patterns and leaks
- **Network I/O**: Bandwidth utilization and packet loss
- **Latency Analysis**: End-to-end and per-hop timing
- **Throughput Monitoring**: Message rates and processing speed

#### Analysis Features
- **Bottleneck Detection**: Automated identification of performance issues
- **Trend Prediction**: ML-based forecasting of resource needs
- **Optimization Suggestions**: Automated recommendations for improvement
- **Historical Comparison**: Performance regression detection

### Node Inspector

Deep dive into individual node behavior and configuration.

#### Information Panels
- **Configuration**: Runtime parameters and settings
- **State Information**: Current operational state
- **Resource Usage**: Real-time metrics and history
- **Message Flow**: Input/output message analysis
- **Error Logs**: Node-specific error tracking
- **Dependencies**: Upstream and downstream connections

#### Interactive Features
- **Live Editing**: Modify configuration parameters
- **Restart Control**: Safe node restart with dependency management
- **Breakpoint Setting**: Debug-mode breakpoint configuration
- **Message Inspection**: Real-time message content viewing

### Log Viewer

Advanced log analysis with intelligent filtering and pattern detection.

#### Filtering Capabilities
- **Severity Levels**: Error, Warning, Info, Debug, Trace
- **Time Ranges**: Absolute and relative time filtering
- **Node Filtering**: Per-node or multi-node log streams
- **Pattern Matching**: Regex and fuzzy search support
- **Correlation**: Cross-node event correlation

#### Smart Features
- **Error Pattern Detection**: Automatic identification of error patterns
- **Anomaly Highlighting**: ML-based anomaly detection in logs
- **Performance Impact**: Correlation between logs and performance metrics
- **Export Capabilities**: Save filtered logs for analysis

---

## Use Case Driven Workflows

### Workflow 1: Project Setup and Initial Development

#### Phase 1: Project Initialization
```bash
# 1. Create new project
dora init my-robotics-project
cd my-robotics-project

# 2. Check initial status
dora status

# 3. Review generated configuration
dora inspect --config

# 4. Launch TUI for configuration review
dora tui --view config
```

#### Phase 2: Dataflow Design
```bash
# 1. Design dataflow interactively
dora tui --view dataflow

# 2. Validate design
dora build --validate

# 3. Run complexity analysis
dora analyze complexity

# 4. Optimize based on suggestions
dora optimize --apply-suggestions
```

#### Phase 3: Initial Testing
```bash
# 1. Start in debug mode
dora start --debug

# 2. Monitor performance
dora dashboard --focus performance

# 3. Analyze logs for issues
dora logs --analyze --auto-filter

# 4. Stop and review metrics
dora stop
dora analyze performance --summary
```

### Workflow 2: Performance Optimization

#### Phase 1: Performance Assessment
```bash
# 1. Run comprehensive analysis
dora analyze performance --detailed

# 2. Launch performance TUI
dora tui --view performance

# 3. Identify bottlenecks
dora debug --auto --focus performance

# 4. Generate optimization report
dora analyze --export performance_report.json
```

#### Phase 2: Optimization Implementation
```bash
# 1. Apply automated optimizations
dora optimize --auto --safe

# 2. Manual optimization with TUI
dora tui --view optimizer

# 3. A/B test configurations
dora test --compare baseline.yaml optimized.yaml

# 4. Validate improvements
dora benchmark --before-after
```

#### Phase 3: Performance Monitoring
```bash
# 1. Set up continuous monitoring
dora monitor --continuous --alert-thresholds config.yaml

# 2. Dashboard for ongoing observation
dora dashboard --layout performance_monitoring.yaml

# 3. Automated reporting
dora analyze --schedule daily --export-to metrics/
```

### Workflow 3: Debugging Complex Issues

#### Phase 1: Issue Detection
```bash
# 1. Run automated issue detection
dora debug --auto --comprehensive

# 2. Review system health
dora inspect --health-check

# 3. Analyze error patterns
dora logs --analyze --pattern-detection

# 4. Launch debug TUI
dora tui --view debug
```

#### Phase 2: Issue Investigation
```bash
# 1. Set strategic breakpoints
dora debug --breakpoints auto-suggest

# 2. Start collaborative debug session
dora debug --collaborate --session team-debug

# 3. Real-time state inspection
dora inspect --live --node problematic_node

# 4. Message flow analysis
dora analyze dataflow --focus error-path
```

#### Phase 3: Issue Resolution
```bash
# 1. Apply fixes with validation
dora fix --validate --test

# 2. Verify resolution
dora test --regression-check

# 3. Update monitoring for prevention
dora monitor --add-checks issue_prevention.yaml

# 4. Document resolution
dora debug --export-session debug_resolution.md
```

### Workflow 4: Team Collaboration

#### Phase 1: Shared Development Environment
```bash
# 1. Start collaborative session
dora collaborate --start --session project_dev

# 2. Invite team members
dora collaborate --invite user@domain.com --role developer

# 3. Shared debugging
dora debug --collaborate --real-time

# 4. Synchronized TUI session
dora tui --collaborate --sync-views
```

#### Phase 2: Code Review and Integration
```bash
# 1. Compare implementations
dora compare --branches feature/new-node main

# 2. Merge with conflict resolution
dora merge --interactive --auto-resolve

# 3. Integration testing
dora test --integration --collaborative

# 4. Performance impact analysis
dora analyze --compare-performance pre-merge post-merge
```

#### Phase 3: Deployment and Monitoring
```bash
# 1. Production deployment
dora deploy --environment production --verify

# 2. Collaborative monitoring
dora monitor --team-dashboard --role-based

# 3. Incident response
dora incident --collaborate --escalation-rules

# 4. Post-deployment analysis
dora analyze --deployment-impact --team-report
```

### Workflow 5: Continuous Integration and Testing

#### Phase 1: Automated Testing Setup
```bash
# 1. Configure test environments
dora test --setup-ci --environments staging,production

# 2. Performance baseline establishment
dora benchmark --establish-baseline --environment staging

# 3. Test automation configuration
dora ci --configure --test-matrix comprehensive

# 4. Integration with CI/CD pipeline
dora ci --integrate --platform github-actions
```

#### Phase 2: Continuous Monitoring
```bash
# 1. Automated performance monitoring
dora monitor --ci-mode --performance-gates

# 2. Regression detection
dora test --regression-monitor --alert-on-degradation

# 3. Dependency update impact analysis
dora analyze --dependency-updates --impact-assessment

# 4. Automated optimization suggestions
dora optimize --ci-suggestions --safe-only
```

#### Phase 3: Quality Assurance
```bash
# 1. Comprehensive quality analysis
dora qa --full-analysis --compliance-check

# 2. Security vulnerability assessment
dora security --scan --dependency-check

# 3. Performance certification
dora certify --performance --environment production

# 4. Release readiness assessment
dora release --readiness-check --comprehensive
```

---

## Development Guidelines

### Code Organization

#### Directory Structure
```
src/
├── commands/           # CLI command implementations
│   ├── basic/         # Basic commands (build, start, stop)
│   ├── smart/         # Smart commands (inspect, debug, analyze)
│   └── tui/           # TUI launch commands
├── tui/               # TUI interface components
│   ├── views/         # Individual TUI views
│   ├── components/    # Reusable UI components
│   └── layouts/       # Layout management
├── intelligence/      # AI/ML components
│   ├── analysis/      # Complexity and performance analysis
│   ├── learning/      # User preference learning
│   └── prediction/    # Predictive analytics
├── collaboration/     # Multi-user features
│   ├── sessions/      # Shared session management
│   ├── sync/          # Real-time synchronization
│   └── auth/          # Authentication and authorization
└── core/              # Core Dora functionality
    ├── dataflow/      # Dataflow management
    ├── monitoring/    # System monitoring
    └── config/        # Configuration management
```

#### Naming Conventions

**Commands:**
- Use verb-noun pattern: `analyze-performance`, `inspect-node`
- Smart commands end with context: `inspect --smart`, `debug --auto`

**TUI Components:**
- Views: `DashboardView`, `DataflowExplorer`, `PerformanceAnalyzer`
- Components: `NodePanel`, `MetricsWidget`, `LogDisplay`

**Intelligence Modules:**
- Analyzers: `ComplexityAnalyzer`, `PerformanceAnalyzer`, `ResourceAnalyzer`
- Learners: `PreferenceLearner`, `PatternLearner`, `BehaviorLearner`

### Implementation Patterns

#### Progressive Disclosure Pattern
```rust
// Command starts simple, escalates to TUI based on complexity
pub async fn execute_inspect(args: InspectArgs) -> Result<()> {
    let complexity = analyze_complexity(&args.target).await?;
    
    if complexity.requires_tui() {
        suggest_tui_mode(&args)?;
        if args.auto_tui || user_confirms()? {
            launch_tui_inspector(args).await?;
        }
    } else {
        display_simple_output(&args).await?;
    }
    
    Ok(())
}
```

#### Smart Suggestion Pattern
```rust
// ML-powered suggestion system
pub struct SmartSuggestionEngine {
    complexity_analyzer: ComplexityAnalyzer,
    preference_learner: PreferenceLearner,
    context_detector: ContextDetector,
}

impl SmartSuggestionEngine {
    pub async fn suggest_action(&self, context: &Context) -> Vec<Suggestion> {
        let complexity = self.complexity_analyzer.assess(context).await;
        let preferences = self.preference_learner.get_preferences(context.user_id).await;
        let automation_context = self.context_detector.detect(context).await;
        
        self.generate_suggestions(complexity, preferences, automation_context).await
    }
}
```

#### Collaborative State Pattern
```rust
// Operational transformation for collaborative editing
pub struct CollaborativeState<T> {
    state: T,
    operations: Vec<Operation>,
    conflict_resolver: ConflictResolver,
}

impl<T> CollaborativeState<T> {
    pub async fn apply_operation(&mut self, op: Operation, user_id: UserId) -> Result<()> {
        let transformed_op = self.transform_operation(op, &self.operations).await?;
        self.state.apply(transformed_op)?;
        self.operations.push(transformed_op);
        self.broadcast_to_peers(transformed_op, user_id).await?;
        Ok(())
    }
}
```

### Testing Guidelines

#### Unit Testing
```rust
#[cfg(test)]
mod tests {
    use super::*;
    use tokio_test;
    
    #[tokio::test]
    async fn test_complexity_analysis() {
        let analyzer = ComplexityAnalyzer::new();
        let simple_config = create_simple_dataflow();
        let complex_config = create_complex_dataflow();
        
        let simple_result = analyzer.analyze(&simple_config).await.unwrap();
        let complex_result = analyzer.analyze(&complex_config).await.unwrap();
        
        assert!(simple_result.complexity_score < 0.3);
        assert!(complex_result.complexity_score > 0.7);
        assert!(complex_result.requires_tui());
    }
    
    #[tokio::test]
    async fn test_smart_suggestion_engine() {
        let engine = SmartSuggestionEngine::new();
        let context = create_debug_context();
        
        let suggestions = engine.suggest_action(&context).await;
        
        assert!(!suggestions.is_empty());
        assert!(suggestions.iter().any(|s| s.action_type == ActionType::LaunchTUI));
    }
}
```

#### Integration Testing
```rust
#[tokio::test]
async fn test_cli_to_tui_escalation() {
    let cli = DoraCliTest::new();
    
    // Start with simple CLI command
    let result = cli.execute(&["inspect", "node", "complex_node"]).await?;
    
    // Verify TUI suggestion
    assert!(result.contains_suggestion(SuggestionType::LaunchTUI));
    
    // Accept suggestion and verify TUI launch
    let tui_result = cli.accept_suggestion().await?;
    assert!(tui_result.tui_launched);
    assert_eq!(tui_result.initial_view, "node_inspector");
}
```

#### Performance Testing
```rust
#[tokio::test]
async fn test_performance_requirements() {
    let dashboard = DashboardView::new();
    let start_time = Instant::now();
    
    // Test 60 FPS requirement
    for _ in 0..60 {
        dashboard.update().await?;
        dashboard.render().await?;
        
        let frame_time = start_time.elapsed();
        assert!(frame_time < Duration::from_millis(16)); // 60 FPS = ~16ms per frame
    }
}
```

### Error Handling

#### Error Types
```rust
#[derive(Debug, thiserror::Error)]
pub enum DoraCliError {
    #[error("Complexity analysis failed: {0}")]
    ComplexityAnalysis(#[from] AnalysisError),
    
    #[error("TUI initialization failed: {0}")]
    TuiInitialization(String),
    
    #[error("Collaboration session error: {0}")]
    CollaborationError(#[from] CollaborationError),
    
    #[error("Smart suggestion engine error: {0}")]
    SuggestionError(String),
}
```

#### Error Recovery
```rust
pub async fn execute_with_recovery<F, T>(operation: F) -> Result<T>
where
    F: FnOnce() -> Result<T>,
{
    match operation() {
        Ok(result) => Ok(result),
        Err(DoraCliError::TuiInitialization(_)) => {
            warn!("TUI initialization failed, falling back to CLI mode");
            execute_cli_fallback().await
        },
        Err(DoraCliError::CollaborationError(_)) => {
            warn!("Collaboration failed, continuing in single-user mode");
            execute_single_user_mode().await
        },
        Err(e) => Err(e),
    }
}
```

### Performance Guidelines

#### Memory Management
- Use `Arc<Mutex<T>>` for shared state
- Implement `Drop` for resource cleanup
- Use streaming for large datasets
- Cache expensive computations

#### Async/Await Best Practices
```rust
// Good: Concurrent operations
let (metrics, logs, config) = tokio::join!(
    fetch_metrics(),
    fetch_logs(),
    fetch_config()
);

// Good: Streaming large datasets
let mut stream = fetch_large_dataset().await?;
while let Some(chunk) = stream.next().await {
    process_chunk(chunk).await?;
}

// Good: Timeout for operations
let result = tokio::time::timeout(
    Duration::from_secs(30),
    expensive_operation()
).await??;
```

#### TUI Performance
- Target 60 FPS for smooth animations
- Use double buffering for flicker-free updates
- Implement dirty rectangle optimization
- Cache rendered components when possible

---

## Advanced Features

### Machine Learning Integration

#### Complexity Analysis Engine
The complexity analysis engine uses Bayesian learning to assess dataflow complexity and suggest appropriate interfaces.

```rust
pub struct ComplexityAnalyzer {
    bayesian_learner: BayesianPreferenceLearner,
    feature_extractor: FeatureExtractor,
    threshold_calculator: AdaptiveThresholdCalculator,
}

impl ComplexityAnalyzer {
    pub async fn analyze(&self, config: &DataflowConfig) -> Result<ComplexityAssessment> {
        let features = self.feature_extractor.extract(config).await?;
        let complexity_score = self.bayesian_learner.predict(&features).await?;
        let threshold = self.threshold_calculator.calculate_for_user(config.user_id).await?;
        
        Ok(ComplexityAssessment {
            score: complexity_score,
            threshold,
            requires_tui: complexity_score > threshold,
            suggested_interface: self.suggest_interface(complexity_score, &features).await?,
            confidence: self.bayesian_learner.get_confidence(),
        })
    }
}
```

#### User Preference Learning
The system learns from user behavior to improve suggestions over time.

```rust
pub struct PreferenceLearner {
    user_actions: Vec<UserAction>,
    feature_weights: HashMap<String, f64>,
    adaptation_rate: f64,
}

impl PreferenceLearner {
    pub async fn learn_from_action(&mut self, action: UserAction) -> Result<()> {
        self.user_actions.push(action.clone());
        
        if action.accepted_suggestion {
            self.reinforce_weights(&action.context_features, 0.1).await?;
        } else {
            self.diminish_weights(&action.context_features, 0.05).await?;
        }
        
        self.update_adaptation_rate().await?;
        Ok(())
    }
}
```

### Real-time Data Visualization

#### High-Performance Rendering
The TUI implements 60 FPS rendering with efficient update mechanisms.

```rust
pub struct HighPerformanceRenderer {
    double_buffer: DoubleBuffer,
    dirty_regions: Vec<Rect>,
    animation_scheduler: AnimationScheduler,
}

impl HighPerformanceRenderer {
    pub async fn render_frame(&mut self) -> Result<()> {
        let start_time = Instant::now();
        
        // Update only dirty regions
        for region in &self.dirty_regions {
            self.update_region(region).await?;
        }
        
        // Apply animations
        self.animation_scheduler.update().await?;
        
        // Swap buffers
        self.double_buffer.swap();
        
        // Ensure 60 FPS timing
        let frame_time = start_time.elapsed();
        if frame_time < Duration::from_millis(16) {
            tokio::time::sleep(Duration::from_millis(16) - frame_time).await;
        }
        
        Ok(())
    }
}
```

#### Interactive Data Exploration
Users can interact with visualizations using mouse and keyboard.

```rust
pub struct InteractiveDataExplorer {
    zoom_level: f64,
    pan_offset: (f64, f64),
    selected_nodes: HashSet<NodeId>,
    interaction_handler: InteractionHandler,
}

impl InteractiveDataExplorer {
    pub async fn handle_mouse_event(&mut self, event: MouseEvent) -> Result<()> {
        match event {
            MouseEvent::Wheel { delta, .. } => {
                self.zoom_level *= 1.0 + (delta as f64 * 0.1);
                self.mark_dirty().await?;
            },
            MouseEvent::Drag { start, end, .. } => {
                let delta = (end.x - start.x, end.y - start.y);
                self.pan_offset.0 += delta.0 as f64;
                self.pan_offset.1 += delta.1 as f64;
                self.mark_dirty().await?;
            },
            MouseEvent::Click { position, .. } => {
                let node_id = self.get_node_at_position(position).await?;
                if let Some(id) = node_id {
                    self.toggle_selection(id).await?;
                }
            },
        }
        Ok(())
    }
}
```

### Collaborative Development Features

#### Operational Transformation
The system supports real-time collaborative editing using operational transformation.

```rust
pub struct OperationalTransform {
    operations: Vec<Operation>,
    state_vector: HashMap<UserId, u64>,
    conflict_resolver: ConflictResolver,
}

impl OperationalTransform {
    pub async fn transform_operation(
        &self,
        operation: Operation,
        concurrent_ops: &[Operation]
    ) -> Result<Operation> {
        let mut transformed = operation;
        
        for concurrent_op in concurrent_ops {
            if self.needs_transformation(&transformed, concurrent_op).await? {
                transformed = self.apply_transformation(transformed, concurrent_op).await?;
            }
        }
        
        Ok(transformed)
    }
    
    async fn apply_transformation(
        &self,
        op1: Operation,
        op2: Operation
    ) -> Result<Operation> {
        match (op1.operation_type, op2.operation_type) {
            (OpType::Insert, OpType::Insert) => {
                self.transform_concurrent_inserts(op1, op2).await
            },
            (OpType::Delete, OpType::Insert) => {
                self.transform_delete_insert(op1, op2).await
            },
            (OpType::Insert, OpType::Delete) => {
                self.transform_insert_delete(op1, op2).await
            },
            (OpType::Delete, OpType::Delete) => {
                self.transform_concurrent_deletes(op1, op2).await
            },
        }
    }
}
```

#### Shared Debug Sessions
Multiple developers can collaboratively debug the same dataflow.

```rust
pub struct SharedDebugSession {
    session_id: SessionId,
    participants: HashMap<UserId, UserRole>,
    shared_state: Arc<Mutex<DebugState>>,
    event_bus: CollaborativeEventBus,
}

impl SharedDebugSession {
    pub async fn add_breakpoint(&self, user_id: UserId, breakpoint: Breakpoint) -> Result<()> {
        // Verify permissions
        self.check_permission(user_id, Permission::AddBreakpoint).await?;
        
        // Update shared state
        {
            let mut state = self.shared_state.lock().await;
            state.breakpoints.insert(breakpoint.id, breakpoint.clone());
        }
        
        // Broadcast to all participants
        self.event_bus.broadcast(Event::BreakpointAdded {
            user_id,
            breakpoint,
        }).await?;
        
        Ok(())
    }
    
    pub async fn handle_breakpoint_hit(&self, node_id: NodeId, context: ExecutionContext) -> Result<()> {
        // Pause execution
        self.pause_node(node_id).await?;
        
        // Notify all participants
        self.event_bus.broadcast(Event::BreakpointHit {
            node_id,
            context: context.clone(),
            timestamp: Utc::now(),
        }).await?;
        
        // Wait for collaborative decision
        let decision = self.wait_for_collaborative_decision().await?;
        
        match decision {
            DebugDecision::Continue => self.resume_node(node_id).await?,
            DebugDecision::StepOver => self.step_over(node_id).await?,
            DebugDecision::StepInto => self.step_into(node_id).await?,
            DebugDecision::Inspect => self.launch_collaborative_inspector(node_id, context).await?,
        }
        
        Ok(())
    }
}
```

### Advanced Analysis Tools

#### Statistical Analysis Engine
Comprehensive statistical analysis of dataflow performance and behavior.

```rust
pub struct StatisticalAnalysisEngine {
    time_series_analyzer: TimeSeriesAnalyzer,
    correlation_analyzer: CorrelationAnalyzer,
    anomaly_detector: AnomalyDetector,
    trend_predictor: TrendPredictor,
}

impl StatisticalAnalysisEngine {
    pub async fn analyze_performance_trends(&self, metrics: &[MetricPoint]) -> Result<TrendAnalysis> {
        let time_series = self.time_series_analyzer.analyze(metrics).await?;
        let correlations = self.correlation_analyzer.find_correlations(metrics).await?;
        let anomalies = self.anomaly_detector.detect_anomalies(metrics).await?;
        let predictions = self.trend_predictor.predict_trends(metrics, Duration::hours(24)).await?;
        
        Ok(TrendAnalysis {
            time_series,
            correlations,
            anomalies,
            predictions,
            confidence_intervals: self.calculate_confidence_intervals(&predictions).await?,
        })
    }
}
```

#### AutoML Integration
Automated machine learning for performance optimization and pattern detection.

```rust
pub struct AutoMLEngine {
    feature_engineering: FeatureEngineeringPipeline,
    model_selection: AutoModelSelection,
    hyperparameter_optimization: HyperparameterOptimizer,
    model_evaluation: ModelEvaluator,
}

impl AutoMLEngine {
    pub async fn optimize_dataflow(&self, config: &DataflowConfig, metrics: &[MetricPoint]) -> Result<OptimizationResult> {
        // Feature engineering
        let features = self.feature_engineering.extract_features(config, metrics).await?;
        
        // Model selection and training
        let models = self.model_selection.select_and_train(&features).await?;
        
        // Hyperparameter optimization
        let optimized_models = self.hyperparameter_optimization.optimize(models).await?;
        
        // Model evaluation
        let best_model = self.model_evaluation.select_best(optimized_models).await?;
        
        // Generate optimization recommendations
        let recommendations = best_model.generate_recommendations(config).await?;
        
        Ok(OptimizationResult {
            model: best_model,
            recommendations,
            expected_improvement: self.estimate_improvement(&recommendations).await?,
            confidence: best_model.confidence_score,
        })
    }
}
```

---

## Troubleshooting

### Common Issues and Solutions

#### Issue: TUI Fails to Launch
**Symptoms:**
- Error: "TUI initialization failed"
- Terminal displays garbled output
- Application exits immediately

**Solutions:**
```bash
# Check terminal compatibility
dora check-terminal

# Force CLI mode
dora --no-tui status

# Update terminal capabilities
export TERM=xterm-256color
dora tui

# Reset terminal state
reset && dora tui
```

**Root Causes:**
- Incompatible terminal emulator
- Insufficient terminal size
- Missing terminal capabilities
- Corrupted terminal state

#### Issue: High CPU Usage
**Symptoms:**
- System becomes unresponsive
- Dora CLI consumes >80% CPU
- TUI animations lag or freeze

**Diagnostic Commands:**
```bash
# Check CPU usage by component
dora analyze performance --component-breakdown

# Identify bottlenecks
dora debug --profile --cpu

# Monitor resource usage
dora monitor --resources --alert-threshold cpu:80
```

**Solutions:**
```bash
# Reduce update frequency
dora tui --update-rate 10  # 10 FPS instead of 60

# Disable animations
dora tui --no-animations

# Optimize dataflow
dora optimize --focus cpu

# Enable performance mode
dora config set performance_mode true
```

#### Issue: Memory Leaks
**Symptoms:**
- Memory usage continuously increases
- Out of memory errors
- System becomes sluggish over time

**Diagnostic Commands:**
```bash
# Memory usage analysis
dora analyze memory --leak-detection

# Profile memory allocation
dora debug --profile --memory

# Track memory trends
dora monitor --memory --trend-analysis
```

**Solutions:**
```bash
# Force garbage collection
dora gc --force

# Restart with memory monitoring
dora restart --monitor-memory

# Adjust memory limits
dora config set memory_limit 4GB

# Enable memory optimization
dora optimize --focus memory
```

#### Issue: Collaboration Sync Problems
**Symptoms:**
- Changes not visible to other users
- Conflicting operations
- Session connection failures

**Diagnostic Commands:**
```bash
# Check collaboration status
dora collaborate --status

# Test network connectivity
dora collaborate --test-connection

# Validate session state
dora collaborate --validate-state
```

**Solutions:**
```bash
# Resync session
dora collaborate --resync

# Resolve conflicts manually
dora collaborate --resolve-conflicts

# Restart collaboration session
dora collaborate --restart-session

# Switch to offline mode
dora collaborate --offline-mode
```

### Performance Optimization

#### Memory Optimization
```bash
# Analyze memory usage patterns
dora analyze memory --detailed

# Optimize memory allocation
dora optimize --memory --apply

# Set memory limits
dora config set memory_limit 2GB
dora config set memory_alert_threshold 1.5GB

# Enable memory monitoring
dora monitor --memory --continuous
```

#### CPU Optimization
```bash
# Profile CPU usage
dora debug --profile --cpu --duration 60s

# Optimize computational load
dora optimize --cpu --parallel-processing

# Adjust thread pool size
dora config set thread_pool_size 8

# Enable CPU monitoring
dora monitor --cpu --alert-threshold 80
```

#### Network Optimization
```bash
# Analyze network patterns
dora analyze network --bandwidth --latency

# Optimize message serialization
dora optimize --network --compression

# Configure network buffers
dora config set network_buffer_size 64KB

# Monitor network performance
dora monitor --network --continuous
```

### Debug Mode Operations

#### Enabling Comprehensive Debugging
```bash
# Start with full debugging enabled
dora start --debug --verbose --trace-all

# Set multiple breakpoints
dora debug --breakpoints "node1:init,node2:process,node3:output"

# Enable performance profiling
dora debug --profile --comprehensive

# Capture debug session
dora debug --capture --output debug_session.log
```

#### Advanced Debug Techniques
```bash
# Memory debugging
dora debug --memory --track-allocations

# Network debugging
dora debug --network --packet-capture

# Dataflow debugging
dora debug --dataflow --message-tracing

# Multi-node debugging
dora debug --distributed --sync-breakpoints
```

### Log Analysis

#### Intelligent Log Filtering
```bash
# Auto-detect error patterns
dora logs --auto-filter --pattern-detection

# Correlation analysis
dora logs --correlate --time-window 5m

# Performance impact analysis
dora logs --performance-impact

# Export filtered logs
dora logs --filter "error|timeout" --export debug_logs.txt
```

#### Log Troubleshooting Workflows
```bash
# Quick error detection
dora logs --errors --recent 1h

# Detailed investigation
dora logs --analyze --interactive

# Pattern matching
dora logs --pattern "connection.*timeout" --context 3

# Real-time monitoring
dora logs --follow --highlight-errors
```

---

## Performance Optimization

### System Performance Monitoring

#### Real-time Metrics Collection
The Dora CLI continuously monitors system performance to ensure optimal operation.

```bash
# Enable comprehensive monitoring
dora monitor --comprehensive --interval 1s

# Monitor specific components
dora monitor --components "cpu,memory,network" --alert-thresholds config.yaml

# Performance dashboard
dora dashboard --focus performance --update-rate 60fps

# Export performance data
dora monitor --export metrics.json --duration 1h
```

#### Performance Baseline Establishment
```bash
# Establish performance baseline
dora benchmark --establish-baseline --duration 10m

# Compare against baseline
dora benchmark --compare-baseline --tolerance 10%

# Automated performance testing
dora test --performance --regression-detection

# Performance certification
dora certify --performance --environment production
```

### Optimization Strategies

#### Automatic Optimization
The system provides ML-powered optimization suggestions.

```rust
pub struct AutoOptimizer {
    performance_analyzer: PerformanceAnalyzer,
    optimization_engine: OptimizationEngine,
    safety_validator: SafetyValidator,
}

impl AutoOptimizer {
    pub async fn optimize_system(&self, config: &SystemConfig) -> Result<OptimizationPlan> {
        // Analyze current performance
        let analysis = self.performance_analyzer.analyze(config).await?;
        
        // Generate optimization suggestions
        let optimizations = self.optimization_engine.generate_optimizations(&analysis).await?;
        
        // Validate safety of optimizations
        let safe_optimizations = self.safety_validator.filter_safe(optimizations).await?;
        
        Ok(OptimizationPlan {
            optimizations: safe_optimizations,
            expected_improvement: analysis.calculate_improvement(&safe_optimizations),
            risk_assessment: self.safety_validator.assess_risk(&safe_optimizations).await?,
        })
    }
}
```

#### Manual Optimization Techniques
```bash
# CPU optimization
dora optimize --cpu --parallel-execution --thread-pool-size auto

# Memory optimization
dora optimize --memory --garbage-collection aggressive

# Network optimization
dora optimize --network --compression lz4 --batching enabled

# I/O optimization
dora optimize --io --async-io --buffer-size 64KB
```

### Performance Profiling

#### CPU Profiling
```rust
pub struct CPUProfiler {
    sampling_interval: Duration,
    flame_graph_generator: FlameGraphGenerator,
    hotspot_detector: HotspotDetector,
}

impl CPUProfiler {
    pub async fn profile_execution(&self, duration: Duration) -> Result<CPUProfile> {
        let mut samples = Vec::new();
        let start_time = Instant::now();
        
        while start_time.elapsed() < duration {
            let sample = self.capture_stack_trace().await?;
            samples.push(sample);
            tokio::time::sleep(self.sampling_interval).await;
        }
        
        let flame_graph = self.flame_graph_generator.generate(&samples).await?;
        let hotspots = self.hotspot_detector.detect(&samples).await?;
        
        Ok(CPUProfile {
            flame_graph,
            hotspots,
            total_samples: samples.len(),
            sampling_rate: 1.0 / self.sampling_interval.as_secs_f64(),
        })
    }
}
```

#### Memory Profiling
```bash
# Memory allocation tracking
dora debug --memory --track-allocations --flame-graph

# Heap analysis
dora debug --memory --heap-analysis --leak-detection

# Memory usage patterns
dora analyze memory --patterns --optimization-suggestions

# Memory pressure testing
dora test --memory --stress-test --limits
```

#### Network Profiling
```bash
# Bandwidth analysis
dora debug --network --bandwidth-analysis

# Latency measurement
dora debug --network --latency-distribution

# Packet analysis
dora debug --network --packet-capture --analysis

# Network optimization testing
dora test --network --optimization-validation
```

### Scalability Optimization

#### Horizontal Scaling
```bash
# Multi-node deployment analysis
dora analyze --distributed --node-allocation

# Load balancing optimization
dora optimize --load-balancing --algorithm adaptive

# Cluster performance monitoring
dora monitor --cluster --cross-node-analysis

# Distributed debugging
dora debug --distributed --cluster-wide
```

#### Vertical Scaling
```bash
# Resource requirement analysis
dora analyze --resources --scaling-recommendations

# Automatic resource adjustment
dora scale --auto --resource-monitoring

# Performance impact assessment
dora scale --validate --performance-testing

# Resource optimization
dora optimize --resources --cost-efficiency
```

---

## Team Collaboration

### Multi-User Development Environment

#### Role-Based Access Control
The Dora CLI implements comprehensive role-based access control for team environments.

```rust
pub struct RoleBasedAccessControl {
    roles: HashMap<Role, Permissions>,
    user_roles: HashMap<UserId, Vec<Role>>,
    resource_policies: HashMap<ResourceType, AccessPolicy>,
}

impl RoleBasedAccessControl {
    pub async fn check_permission(
        &self,
        user_id: UserId,
        resource: &Resource,
        action: Action
    ) -> Result<bool> {
        let user_roles = self.user_roles.get(&user_id)
            .ok_or(Error::UserNotFound)?;
            
        for role in user_roles {
            let permissions = self.roles.get(role)
                .ok_or(Error::RoleNotFound)?;
                
            if permissions.allows(resource, action) {
                return Ok(true);
            }
        }
        
        // Check resource-specific policies
        let policy = self.resource_policies.get(&resource.resource_type())
            .unwrap_or(&AccessPolicy::default());
            
        Ok(policy.evaluate(user_id, resource, action).await?)
    }
}
```

#### Team Roles and Permissions
```bash
# Define team roles
dora team --create-role "senior-developer" --permissions "debug,modify,deploy"
dora team --create-role "junior-developer" --permissions "debug,view"
dora team --create-role "devops" --permissions "deploy,monitor,configure"

# Assign users to roles
dora team --assign-user alice@company.com --role "senior-developer"
dora team --assign-user bob@company.com --role "junior-developer"

# View team structure
dora team --list-members --show-permissions

# Modify permissions
dora team --modify-role "junior-developer" --add-permission "modify"
```

### Collaborative Development Workflows

#### Shared Debugging Sessions
```bash
# Start collaborative debug session
dora debug --collaborate --session "feature-debugging" --invite alice@company.com,bob@company.com

# Join existing session
dora debug --join-session "feature-debugging"

# Manage session participants
dora debug --session "feature-debugging" --add-participant charlie@company.com --role observer

# Session recording and playback
dora debug --session "feature-debugging" --record --output debug_session.recording
dora debug --playback debug_session.recording --interactive
```

#### Real-time Code Collaboration
```rust
pub struct CollaborativeEditor {
    operational_transform: OperationalTransform,
    conflict_resolver: ConflictResolver,
    real_time_sync: RealTimeSync,
    version_control: VersionControl,
}

impl CollaborativeEditor {
    pub async fn apply_edit(&mut self, edit: Edit, user_id: UserId) -> Result<()> {
        // Transform edit based on concurrent operations
        let transformed_edit = self.operational_transform
            .transform_edit(edit, &self.get_concurrent_operations()).await?;
            
        // Apply edit to local state
        self.apply_local_edit(&transformed_edit).await?;
        
        // Broadcast to all participants
        self.real_time_sync.broadcast_edit(transformed_edit, user_id).await?;
        
        // Update version control
        self.version_control.record_edit(transformed_edit, user_id).await?;
        
        Ok(())
    }
    
    pub async fn resolve_conflict(&mut self, conflict: EditConflict) -> Result<Resolution> {
        // Automatic conflict resolution
        if let Some(auto_resolution) = self.conflict_resolver.auto_resolve(&conflict).await? {
            return Ok(auto_resolution);
        }
        
        // Manual conflict resolution with team input
        let team_input = self.request_team_resolution(&conflict).await?;
        let resolution = self.conflict_resolver.resolve_with_input(&conflict, team_input).await?;
        
        Ok(resolution)
    }
}
```

#### Collaborative Analysis
```bash
# Start shared analysis session
dora analyze --collaborate --session "performance-analysis" --real-time

# Share analysis results
dora analyze performance --share --team-dashboard

# Collaborative optimization
dora optimize --collaborate --consensus-required

# Team review workflow
dora review --create --analysis-results performance_analysis.json --assignees alice@company.com,bob@company.com
```

### Communication and Coordination

#### Integrated Communication
```rust
pub struct TeamCommunication {
    chat_integration: ChatIntegration,
    notification_system: NotificationSystem,
    annotation_system: AnnotationSystem,
}

impl TeamCommunication {
    pub async fn send_context_message(&self, message: &str, context: &Context) -> Result<()> {
        let enriched_message = self.enrich_with_context(message, context).await?;
        
        // Send to integrated chat system
        self.chat_integration.send_message(enriched_message).await?;
        
        // Create contextual annotations
        self.annotation_system.create_annotation(context, message).await?;
        
        // Trigger relevant notifications
        self.notification_system.notify_relevant_users(context, message).await?;
        
        Ok(())
    }
}
```

#### Team Communication Features
```bash
# In-context messaging
dora chat --context "node:detector" --message "This node is showing high latency"

# Code annotations
dora annotate --file dataflow.yaml --line 42 --message "TODO: optimize this connection"

# Team notifications
dora notify --team --priority high --message "Production deployment starting"

# Status updates
dora status --team-view --real-time-updates
```

### Knowledge Sharing

#### Documentation Collaboration
```bash
# Collaborative documentation
dora docs --collaborate --session "api-documentation"

# Automatic documentation generation
dora docs --generate --from-code --team-review

# Knowledge base integration
dora knowledge --add --category "troubleshooting" --content troubleshooting_guide.md

# Team learning sessions
dora learn --session "best-practices" --interactive --record
```

#### Best Practices Sharing
```rust
pub struct BestPracticesEngine {
    pattern_detector: PatternDetector,
    knowledge_graph: KnowledgeGraph,
    recommendation_engine: RecommendationEngine,
}

impl BestPracticesEngine {
    pub async fn capture_best_practice(&self, context: &Context, action: &Action) -> Result<()> {
        // Detect successful patterns
        if self.pattern_detector.is_successful_pattern(context, action).await? {
            let best_practice = self.extract_best_practice(context, action).await?;
            
            // Add to knowledge graph
            self.knowledge_graph.add_practice(best_practice.clone()).await?;
            
            // Generate recommendations for similar contexts
            let recommendations = self.recommendation_engine
                .generate_from_practice(&best_practice).await?;
                
            // Share with team
            self.share_with_team(best_practice, recommendations).await?;
        }
        
        Ok(())
    }
}
```

### Team Analytics and Insights

#### Collaborative Performance Metrics
```bash
# Team performance dashboard
dora analytics --team-dashboard --metrics "productivity,quality,collaboration"

# Individual contribution analysis
dora analytics --individual --user alice@company.com --time-range "1month"

# Team collaboration patterns
dora analytics --collaboration-patterns --network-analysis

# Knowledge sharing metrics
dora analytics --knowledge-sharing --impact-analysis
```

#### Team Health Monitoring
```rust
pub struct TeamHealthMonitor {
    collaboration_analyzer: CollaborationAnalyzer,
    productivity_tracker: ProductivityTracker,
    satisfaction_monitor: SatisfactionMonitor,
}

impl TeamHealthMonitor {
    pub async fn assess_team_health(&self) -> Result<TeamHealthReport> {
        let collaboration_score = self.collaboration_analyzer.analyze().await?;
        let productivity_metrics = self.productivity_tracker.get_metrics().await?;
        let satisfaction_score = self.satisfaction_monitor.get_satisfaction().await?;
        
        Ok(TeamHealthReport {
            collaboration_score,
            productivity_metrics,
            satisfaction_score,
            recommendations: self.generate_recommendations(
                collaboration_score,
                &productivity_metrics,
                satisfaction_score
            ).await?,
        })
    }
}
```

---

## API Reference

### Core CLI Commands

#### `dora build`
Builds the dataflow graph with dependency resolution and optimization.

**Syntax:**
```bash
dora build [OPTIONS] [PATH]
```

**Arguments:**
- `PATH`: Path to dataflow configuration (default: current directory)

**Options:**
- `--config, -c <FILE>`: Specific configuration file
- `--validate`: Validate configuration without building
- `--analyze`: Run complexity analysis during build
- `--interactive`: Launch TUI for complex builds
- `--optimize`: Apply optimization suggestions
- `--parallel <N>`: Use N parallel build processes
- `--verbose, -v`: Verbose output
- `--quiet, -q`: Suppress non-error output

**Examples:**
```bash
# Basic build
dora build

# Build with optimization
dora build --optimize --parallel 4

# Validate configuration
dora build --validate --config production.yaml

# Interactive build for complex projects
dora build --interactive --analyze
```

**Exit Codes:**
- `0`: Success
- `1`: Build failure
- `2`: Configuration error
- `3`: Dependency resolution failure

#### `dora start`
Starts the dataflow execution with monitoring and debugging capabilities.

**Syntax:**
```bash
dora start [OPTIONS] [NODES...]
```

**Arguments:**
- `NODES`: Specific nodes to start (default: all nodes)

**Options:**
- `--config, -c <FILE>`: Configuration file
- `--debug`: Enable debug mode
- `--monitor`: Enable resource monitoring
- `--breakpoints <SPEC>`: Set debug breakpoints
- `--timeout <DURATION>`: Startup timeout
- `--resource-limits <SPEC>`: Resource limits
- `--background, -d`: Run in background
- `--log-level <LEVEL>`: Logging level (trace|debug|info|warn|error)

**Examples:**
```bash
# Start all nodes
dora start

# Start with debugging
dora start --debug --breakpoints "camera:init,detector:process"

# Start specific nodes with monitoring
dora start --monitor --nodes "camera,detector"

# Background execution with resource limits
dora start --background --resource-limits "cpu:80%,memory:4GB"
```

#### `dora stop`
Gracefully stops dataflow execution with cleanup verification.

**Syntax:**
```bash
dora stop [OPTIONS] [NODES...]
```

**Arguments:**
- `NODES`: Specific nodes to stop (default: all nodes)

**Options:**
- `--force`: Force stop without graceful shutdown
- `--timeout <DURATION>`: Shutdown timeout
- `--verify-cleanup`: Verify resource cleanup
- `--save-state`: Save current state before stopping
- `--kill-orphans`: Kill orphaned processes

**Examples:**
```bash
# Graceful stop
dora stop

# Force stop with cleanup verification
dora stop --force --verify-cleanup

# Stop specific nodes
dora stop --nodes "camera,detector" --save-state
```

### Smart Commands

#### `dora inspect`
Intelligent resource inspection with automatic TUI escalation.

**Syntax:**
```bash
dora inspect [OPTIONS] [RESOURCE]
```

**Arguments:**
- `RESOURCE`: Resource to inspect (node|connection|system)

**Options:**
- `--deep`: Deep inspection (may trigger TUI)
- `--format <FORMAT>`: Output format (json|yaml|table|tui)
- `--export <FILE>`: Export inspection results
- `--compare <BASELINE>`: Compare against baseline
- `--real-time`: Real-time inspection updates
- `--historical <RANGE>`: Historical data range

**Smart Behavior:**
- Automatically suggests TUI for complex resources
- ML-powered anomaly detection
- Historical trend analysis
- Performance impact assessment

**Examples:**
```bash
# Basic inspection
dora inspect node camera

# Deep inspection with TUI suggestion
dora inspect --deep --real-time system

# Export inspection report
dora inspect --export inspection_report.json --format json

# Historical comparison
dora inspect --historical "24h" --compare baseline.json
```

#### `dora debug`
Comprehensive debugging with collaborative features.

**Syntax:**
```bash
dora debug [OPTIONS] [TARGET]
```

**Arguments:**
- `TARGET`: Debug target (node|connection|dataflow)

**Options:**
- `--auto`: Automatic issue detection
- `--profile <TYPE>`: Profiling type (cpu|memory|network)
- `--collaborate`: Enable collaborative debugging
- `--session <NAME>`: Debug session name
- `--breakpoints <SPEC>`: Breakpoint specification
- `--trace`: Enable execution tracing
- `--capture <FILE>`: Capture debug session
- `--playback <FILE>`: Playback captured session

**Examples:**
```bash
# Auto-debug with issue detection
dora debug --auto --profile cpu

# Collaborative debugging session
dora debug --collaborate --session "team-debug" node detector

# Trace execution with breakpoints
dora debug --trace --breakpoints "detector:process" --capture debug.log
```

#### `dora analyze`
Multi-modal analysis with predictive insights.

**Syntax:**
```bash
dora analyze [ANALYSIS_TYPE] [OPTIONS]
```

**Analysis Types:**
- `performance`: Performance analysis
- `resources`: Resource utilization analysis
- `trends`: Trend analysis and prediction
- `complexity`: Complexity assessment
- `dependencies`: Dependency analysis

**Options:**
- `--time-range <RANGE>`: Analysis time range
- `--predict`: Enable predictive analysis
- `--horizon <DURATION>`: Prediction horizon
- `--export <FILE>`: Export analysis results
- `--interactive`: Interactive analysis (TUI)
- `--compare <BASELINE>`: Baseline comparison

**Examples:**
```bash
# Performance analysis with prediction
dora analyze performance --predict --horizon "6h"

# Interactive trend analysis
dora analyze trends --interactive --time-range "7d"

# Resource analysis with export
dora analyze resources --export resource_analysis.json
```

### TUI Commands

#### `dora tui`
Main TUI interface launcher with context-aware views.

**Syntax:**
```bash
dora tui [OPTIONS] [--view VIEW]
```

**Views:**
- `dashboard`: System overview dashboard
- `dataflow`: Dataflow graph explorer
- `performance`: Performance analysis view
- `nodes`: Node inspector view
- `logs`: Log viewer
- `debug`: Debug session view
- `settings`: Configuration management

**Options:**
- `--view <VIEW>`: Initial view
- `--focus <TARGET>`: Focus on specific target
- `--collaborate`: Enable collaborative mode
- `--session <NAME>`: Collaboration session name
- `--layout <FILE>`: Custom layout configuration
- `--theme <THEME>`: UI theme

**Examples:**
```bash
# Launch main dashboard
dora tui

# Start with dataflow view
dora tui --view dataflow --focus node:camera

# Collaborative TUI session
dora tui --collaborate --session "team-analysis"
```

#### `dora dashboard`
Real-time system monitoring dashboard.

**Syntax:**
```bash
dora dashboard [OPTIONS]
```

**Options:**
- `--layout <LAYOUT>`: Dashboard layout (default|performance|debug)
- `--update-rate <RATE>`: Update rate in FPS (default: 60)
- `--focus <METRICS>`: Focus on specific metrics
- `--export <FILE>`: Export dashboard configuration
- `--alerts`: Enable alert notifications

**Examples:**
```bash
# Performance-focused dashboard
dora dashboard --layout performance --focus "cpu,memory"

# High-refresh monitoring
dora dashboard --update-rate 120 --alerts
```

### Configuration Commands

#### `dora config`
Configuration management with validation and synchronization.

**Syntax:**
```bash
dora config [SUBCOMMAND] [OPTIONS]
```

**Subcommands:**
- `get <KEY>`: Get configuration value
- `set <KEY> <VALUE>`: Set configuration value
- `list`: List all configuration
- `validate`: Validate configuration
- `export <FILE>`: Export configuration
- `import <FILE>`: Import configuration
- `reset`: Reset to defaults

**Examples:**
```bash
# Set performance mode
dora config set performance_mode true

# Export configuration
dora config export my_config.yaml

# Validate current configuration
dora config validate --strict
```

### Collaboration Commands

#### `dora collaborate`
Team collaboration and session management.

**Syntax:**
```bash
dora collaborate [SUBCOMMAND] [OPTIONS]
```

**Subcommands:**
- `start`: Start collaboration session
- `join <SESSION>`: Join existing session
- `invite <USER>`: Invite user to session
- `leave`: Leave current session
- `status`: Show collaboration status
- `sessions`: List active sessions

**Examples:**
```bash
# Start new collaboration session
dora collaborate start --session "feature-development" --public

# Invite team members
dora collaborate invite alice@company.com --role developer

# Join existing session
dora collaborate join "feature-development"
```

### Monitoring Commands

#### `dora monitor`
Continuous monitoring with alerting and analysis.

**Syntax:**
```bash
dora monitor [OPTIONS]
```

**Options:**
- `--components <LIST>`: Components to monitor
- `--interval <DURATION>`: Monitoring interval
- `--alerts`: Enable alert system
- `--export <FILE>`: Export monitoring data
- `--dashboard`: Launch monitoring dashboard
- `--continuous`: Continuous monitoring mode

**Examples:**
```bash
# Monitor with alerts
dora monitor --components "cpu,memory,network" --alerts

# Export monitoring data
dora monitor --export monitoring_data.json --interval 5s
```

### Utility Commands

#### `dora logs`
Advanced log management and analysis.

**Syntax:**
```bash
dora logs [OPTIONS] [FILTER]
```

**Arguments:**
- `FILTER`: Log filter pattern

**Options:**
- `--level <LEVEL>`: Log level filter
- `--node <NODE>`: Node-specific logs
- `--follow, -f`: Follow log stream
- `--tail <N>`: Show last N lines
- `--analyze`: Enable log analysis
- `--export <FILE>`: Export logs

**Examples:**
```bash
# Follow error logs
dora logs --level error --follow

# Analyze logs for patterns
dora logs --analyze --pattern "timeout|error"

# Export specific node logs
dora logs --node detector --export detector_logs.txt
```

#### `dora test`
Testing framework with performance and regression testing.

**Syntax:**
```bash
dora test [TEST_TYPE] [OPTIONS]
```

**Test Types:**
- `unit`: Unit tests
- `integration`: Integration tests
- `performance`: Performance tests
- `regression`: Regression tests
- `stress`: Stress tests

**Options:**
- `--parallel <N>`: Parallel test execution
- `--coverage`: Generate coverage report
- `--benchmark`: Performance benchmarking
- `--report <FILE>`: Test report file

**Examples:**
```bash
# Run all tests with coverage
dora test --coverage --parallel 4

# Performance testing with benchmarks
dora test performance --benchmark --report perf_report.json
```

### Environment Variables

#### Core Environment Variables
- `DORA_CONFIG_PATH`: Configuration file path
- `DORA_LOG_LEVEL`: Default log level
- `DORA_TUI_THEME`: TUI theme name
- `DORA_COLLABORATION_SERVER`: Collaboration server URL
- `DORA_PERFORMANCE_MODE`: Enable performance optimizations

#### Advanced Environment Variables
- `DORA_MEMORY_LIMIT`: Memory usage limit
- `DORA_CPU_LIMIT`: CPU usage limit
- `DORA_NETWORK_TIMEOUT`: Network operation timeout
- `DORA_DEBUG_PORT`: Debug server port
- `DORA_METRICS_EXPORT`: Metrics export endpoint

#### Example Configuration
```bash
export DORA_CONFIG_PATH="/path/to/config.yaml"
export DORA_LOG_LEVEL="debug"
export DORA_TUI_THEME="dark"
export DORA_PERFORMANCE_MODE="true"
export DORA_MEMORY_LIMIT="4GB"
```

### Configuration File Reference

#### Main Configuration Structure
```yaml
# Dora CLI Configuration
version: "1.0"

# Core settings
core:
  log_level: "info"
  performance_mode: true
  memory_limit: "4GB"
  cpu_limit: "80%"

# TUI settings
tui:
  theme: "dark"
  update_rate: 60
  animations: true
  mouse_support: true

# Collaboration settings
collaboration:
  server_url: "https://collaborate.dora.dev"
  auto_join_sessions: false
  default_role: "developer"

# Intelligence settings
intelligence:
  complexity_threshold: 0.7
  auto_suggestions: true
  learning_enabled: true
  pattern_detection: true

# Monitoring settings
monitoring:
  continuous: true
  alert_thresholds:
    cpu: 80
    memory: 90
    disk: 95
  export_interval: "5m"
```

---

## Conclusion

This comprehensive Dora CLI manual serves as both a complete reference for developers and a foundation for use case driven development guidelines. The manual covers all aspects of the hybrid CLI/TUI system, from basic commands to advanced collaborative features.

### Key Takeaways

1. **Progressive Disclosure**: Start with simple CLI commands and escalate to rich TUI interfaces as needed
2. **Smart Intelligence**: Leverage ML-powered suggestions and automated analysis
3. **Team Collaboration**: Utilize real-time collaboration features for distributed development
4. **Performance Focus**: Maintain 60 FPS TUI performance and efficient resource utilization
5. **Comprehensive Analysis**: Use statistical analysis and predictive insights for optimization

### Development Philosophy

The Dora CLI embodies a philosophy of **intelligent simplicity** - providing powerful capabilities through intuitive interfaces that adapt to user needs and project complexity. By following the patterns and guidelines in this manual, teams can build robust, performant, and collaborative development workflows.

### Continuous Improvement

This manual will evolve with the Dora CLI system. Regular updates will incorporate new features, optimization techniques, and best practices discovered by the development community.

---

*This manual corresponds to GitHub issues #013-034 and provides comprehensive guidance for implementing and using the Dora CLI hybrid architecture.*