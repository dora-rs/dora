# Issue #018: Build Enhanced `dora debug` Command with Auto-TUI

## 📋 Summary
Implement an intelligent `dora debug` command that automatically launches interactive debugging TUI based on detected issues, system complexity, and error patterns. This command demonstrates the hybrid CLI's ability to seamlessly transition between CLI efficiency and TUI power for complex debugging workflows.

## 🎯 Objectives
- Create comprehensive debugging workflow with intelligent interface escalation
- Implement automatic issue detection and debugging session configuration
- Add real-time debugging data visualization and interactive problem solving
- Provide guided troubleshooting with contextual help and suggestions
- Enable seamless debugging across dataflows, nodes, and system components

**Success Metrics:**
- Debug session launch time under 3 seconds for most scenarios
- Issue detection accuracy exceeds 90% for common problems
- Interactive debugging reduces problem resolution time by 50% compared to manual approaches
- User satisfaction with guided troubleshooting exceeds 85%
- Automatic TUI suggestions are accepted 70% of the time when offered

## 🛠️ Technical Requirements

### What to Build

#### 1. Enhanced Debug Command Structure
```rust
// src/cli/commands/debug.rs
#[derive(Debug, clap::Args)]
pub struct DebugCommand {
    /// Target to debug (dataflow, node, system, or specific component)
    pub target: Option<String>,
    
    /// Debug mode (interactive, analysis, trace, profile)
    #[clap(long, value_enum, default_value = "interactive")]
    pub mode: DebugMode,
    
    /// Focus area for debugging
    #[clap(long, value_enum)]
    pub focus: Option<DebugFocus>,
    
    /// Automatic issue detection and triage
    #[clap(long)]
    pub auto_detect: bool,
    
    /// Enable live monitoring during debug session
    #[clap(long)]
    pub live: bool,
    
    /// Debug session timeout
    #[clap(long, default_value = "30m")]
    pub timeout: String,
    
    /// Capture debug artifacts (logs, traces, profiles)
    #[clap(long)]
    pub capture: bool,
    
    /// Debug output directory
    #[clap(long)]
    pub output_dir: Option<PathBuf>,
    
    /// Force CLI text output (disable auto-TUI)
    #[clap(long)]
    pub text: bool,
    
    /// Force TUI mode
    #[clap(long)]
    pub tui: bool,
    
    /// Debug verbosity level
    #[clap(long, default_value = "info")]
    pub verbosity: LogLevel,
    
    /// Include historical data in analysis
    #[clap(long, default_value = "1h")]
    pub history_window: String,
    
    /// Enable advanced debugging features
    #[clap(long)]
    pub advanced: bool,
}

#[derive(Debug, Clone, clap::ValueEnum)]
pub enum DebugMode {
    Interactive,  // Full interactive debugging with TUI
    Analysis,     // Automated analysis with report
    Trace,        // Message tracing and flow analysis
    Profile,      // Performance profiling
    Health,       // Health check and diagnostics
    Network,      // Network connectivity debugging
}

#[derive(Debug, Clone, clap::ValueEnum)]
pub enum DebugFocus {
    Performance,
    Errors,
    Memory,
    Network,
    Dependencies,
    Configuration,
    Data,
    Security,
}

impl DebugCommand {
    pub async fn execute(&self, context: &ExecutionContext) -> Result<()> {
        // Initialize debug session
        let debug_session = self.initialize_debug_session(context).await?;
        
        // Perform automatic issue detection if enabled
        let detected_issues = if self.auto_detect {
            self.perform_auto_detection(&debug_session).await?
        } else {
            Vec::new()
        };
        
        // Analyze complexity for interface decision
        let complexity_analysis = self.analyze_debug_complexity(&debug_session, &detected_issues).await?;
        
        // Determine interface strategy
        let interface_selector = InterfaceSelector::new(context.clone(), UserConfig::load()?);
        let interface_decision = interface_selector.select_interface(&Command::Debug(self.clone()));
        
        // Apply explicit user overrides
        let final_decision = self.apply_debug_overrides(interface_decision, &detected_issues);
        
        match final_decision.strategy {
            InterfaceStrategy::CliOnly => {
                self.execute_cli_debug(&debug_session, &detected_issues, context).await?;
            },
            
            InterfaceStrategy::CliWithHint { hint, tui_command } => {
                self.execute_cli_debug(&debug_session, &detected_issues, context).await?;
                self.show_debug_hint(&hint, &tui_command, &detected_issues);
            },
            
            InterfaceStrategy::PromptForTui { reason, default_yes } => {
                // Show CLI analysis first
                self.show_debug_overview(&debug_session, &detected_issues, context).await?;
                
                if self.should_launch_interactive_debug(&reason, &detected_issues, default_yes)? {
                    self.launch_interactive_debug_session(&debug_session, detected_issues).await?;
                } else {
                    self.continue_cli_debug(&debug_session, &detected_issues, context).await?;
                }
            },
            
            InterfaceStrategy::AutoLaunchTui { reason, show_cli_first } => {
                if show_cli_first {
                    self.show_debug_overview(&debug_session, &detected_issues, context).await?;
                    println!("\n🔍 {}", reason);
                }
                self.launch_interactive_debug_session(&debug_session, detected_issues).await?;
            },
        }
        
        // Save debug artifacts if requested
        if self.capture {
            self.save_debug_artifacts(&debug_session).await?;
        }
        
        Ok(())
    }
    
    async fn initialize_debug_session(&self, context: &ExecutionContext) -> Result<DebugSession> {
        let session_id = Uuid::new_v4();
        let start_time = Utc::now();
        
        // Determine debug target
        let debug_target = if let Some(target) = &self.target {
            self.resolve_debug_target(target).await?
        } else {
            self.auto_detect_debug_target().await?
        };
        
        // Initialize monitoring components
        let system_monitor = SystemDebugMonitor::new();
        let dataflow_monitor = DataflowDebugMonitor::new();
        let network_monitor = NetworkDebugMonitor::new();
        
        Ok(DebugSession {
            session_id,
            start_time,
            debug_target,
            mode: self.mode.clone(),
            focus: self.focus.clone(),
            monitors: DebugMonitors {
                system: system_monitor,
                dataflow: dataflow_monitor,
                network: network_monitor,
            },
            captured_data: CapturedDebugData::new(),
            active_traces: Vec::new(),
            configuration: DebugConfiguration::from_command(self),
        })
    }
}
```

#### 2. Automatic Issue Detection System
```rust
// src/debug/issue_detection.rs
#[derive(Debug)]
pub struct AutomaticIssueDetector {
    detectors: Vec<Box<dyn IssueDetector>>,
    triage_engine: IssueTriageEngine,
    severity_classifier: IssueSeverityClassifier,
    correlation_analyzer: IssueCorrelationAnalyzer,
}

#[derive(Debug, Clone)]
pub struct DetectedIssue {
    pub issue_id: String,
    pub issue_type: IssueType,
    pub severity: IssueSeverity,
    pub confidence: f32,
    pub title: String,
    pub description: String,
    pub affected_components: Vec<ComponentReference>,
    pub symptoms: Vec<Symptom>,
    pub possible_causes: Vec<PossibleCause>,
    pub suggested_actions: Vec<SuggestedAction>,
    pub debugging_hints: Vec<DebuggingHint>,
    pub first_detected: DateTime<Utc>,
    pub related_issues: Vec<String>,
}

#[derive(Debug, Clone)]
pub enum IssueType {
    PerformanceDegradation,
    MemoryLeak,
    NetworkConnectivity,
    DataflowStalled,
    NodeCrashing,
    ConfigurationError,
    DependencyFailure,
    ResourceExhaustion,
    MessageLoss,
    LatencySpike,
    ErrorRateIncrease,
    DeadlockDetected,
}

trait IssueDetector: Send + Sync {
    async fn detect_issues(&self, debug_session: &DebugSession) -> Result<Vec<DetectedIssue>>;
    fn detector_name(&self) -> &str;
    fn supported_issue_types(&self) -> Vec<IssueType>;
    fn detection_priority(&self) -> u8;
}

pub struct PerformanceIssueDetector {
    baseline_metrics: BaselineMetrics,
    anomaly_thresholds: AnomalyThresholds,
}

impl IssueDetector for PerformanceIssueDetector {
    async fn detect_issues(&self, debug_session: &DebugSession) -> Result<Vec<DetectedIssue>> {
        let mut issues = Vec::new();
        
        // Get current performance metrics
        let current_metrics = debug_session.monitors.system.get_current_metrics().await?;
        let baseline = self.baseline_metrics.get_baseline_for_time_window(
            &debug_session.configuration.history_window
        ).await?;
        
        // CPU Usage Analysis
        if let Some(cpu_issue) = self.analyze_cpu_performance(&current_metrics, &baseline) {
            issues.push(cpu_issue);
        }
        
        // Memory Usage Analysis
        if let Some(memory_issue) = self.analyze_memory_performance(&current_metrics, &baseline) {
            issues.push(memory_issue);
        }
        
        // Throughput Analysis
        if let Some(throughput_issue) = self.analyze_throughput_performance(&current_metrics, &baseline) {
            issues.push(throughput_issue);
        }
        
        // Latency Analysis
        if let Some(latency_issue) = self.analyze_latency_performance(&current_metrics, &baseline) {
            issues.push(latency_issue);
        }
        
        Ok(issues)
    }
    
    fn detector_name(&self) -> &str {
        "Performance Issue Detector"
    }
    
    fn supported_issue_types(&self) -> Vec<IssueType> {
        vec![
            IssueType::PerformanceDegradation,
            IssueType::MemoryLeak,
            IssueType::ResourceExhaustion,
            IssueType::LatencySpike,
        ]
    }
    
    fn detection_priority(&self) -> u8 {
        90
    }
}

impl PerformanceIssueDetector {
    fn analyze_cpu_performance(
        &self,
        current: &SystemMetrics,
        baseline: &BaselineMetrics,
    ) -> Option<DetectedIssue> {
        let cpu_usage = current.cpu_usage_percent;
        let baseline_cpu = baseline.average_cpu_usage;
        
        // Detect CPU spike (>50% above baseline and >80% absolute)
        if cpu_usage > baseline_cpu * 1.5 && cpu_usage > 80.0 {
            let severity = if cpu_usage > 95.0 {
                IssueSeverity::Critical
            } else if cpu_usage > 90.0 {
                IssueSeverity::High
            } else {
                IssueSeverity::Medium
            };
            
            Some(DetectedIssue {
                issue_id: format!("cpu-spike-{}", Utc::now().timestamp()),
                issue_type: IssueType::PerformanceDegradation,
                severity,
                confidence: 0.9,
                title: "High CPU Usage Detected".to_string(),
                description: format!(
                    "CPU usage is {:.1}%, significantly above baseline of {:.1}%",
                    cpu_usage, baseline_cpu
                ),
                affected_components: vec![ComponentReference::System("CPU".to_string())],
                symptoms: vec![
                    Symptom {
                        description: format!("CPU usage at {:.1}%", cpu_usage),
                        metric_value: Some(cpu_usage),
                        timestamp: Utc::now(),
                    }
                ],
                possible_causes: vec![
                    PossibleCause {
                        description: "Runaway process or infinite loop".to_string(),
                        likelihood: 0.7,
                        investigation_steps: vec![
                            "Check top processes by CPU usage".to_string(),
                            "Review recent code changes".to_string(),
                            "Look for infinite loops in node logic".to_string(),
                        ],
                    },
                    PossibleCause {
                        description: "Insufficient resources for current load".to_string(),
                        likelihood: 0.5,
                        investigation_steps: vec![
                            "Compare current load with historical patterns".to_string(),
                            "Check if system needs scaling".to_string(),
                        ],
                    },
                ],
                suggested_actions: vec![
                    SuggestedAction {
                        action: "Profile CPU usage by process".to_string(),
                        command: Some("dora debug --mode profile --focus performance".to_string()),
                        urgency: ActionUrgency::High,
                    },
                    SuggestedAction {
                        action: "Review system resource allocation".to_string(),
                        command: Some("dora inspect system --focus resource".to_string()),
                        urgency: ActionUrgency::Medium,
                    },
                ],
                debugging_hints: vec![
                    DebuggingHint {
                        hint: "Use interactive debugging to identify the specific process causing high CPU usage".to_string(),
                        interactive_features: vec![
                            "Real-time process monitoring".to_string(),
                            "CPU usage timeline visualization".to_string(),
                            "Process tree analysis".to_string(),
                        ],
                    },
                ],
                first_detected: Utc::now(),
                related_issues: Vec::new(),
            })
        } else {
            None
        }
    }
}

pub struct DataflowIssueDetector {
    flow_analyzer: DataflowAnalyzer,
}

impl IssueDetector for DataflowIssueDetector {
    async fn detect_issues(&self, debug_session: &DebugSession) -> Result<Vec<DetectedIssue>> {
        let mut issues = Vec::new();
        
        if let DebugTarget::Dataflow(dataflow_name) = &debug_session.debug_target {
            // Get dataflow status and metrics
            let dataflow_status = debug_session.monitors.dataflow
                .get_dataflow_status(dataflow_name).await?;
            
            // Check for stalled dataflow
            if let Some(stall_issue) = self.detect_dataflow_stall(&dataflow_status) {
                issues.push(stall_issue);
            }
            
            // Check for failing nodes
            if let Some(node_issues) = self.detect_failing_nodes(&dataflow_status) {
                issues.extend(node_issues);
            }
            
            // Check for message loss
            if let Some(message_loss) = self.detect_message_loss(&dataflow_status) {
                issues.push(message_loss);
            }
        }
        
        Ok(issues)
    }
    
    fn detector_name(&self) -> &str {
        "Dataflow Issue Detector"
    }
    
    fn supported_issue_types(&self) -> Vec<IssueType> {
        vec![
            IssueType::DataflowStalled,
            IssueType::NodeCrashing,
            IssueType::MessageLoss,
            IssueType::ConfigurationError,
        ]
    }
    
    fn detection_priority(&self) -> u8 {
        95
    }
}
```

#### 3. Intelligent Debug Interface Selection
```rust
// src/debug/interface_selection.rs
impl DebugCommand {
    async fn analyze_debug_complexity(
        &self,
        debug_session: &DebugSession,
        detected_issues: &[DetectedIssue],
    ) -> Result<DebugComplexityAnalysis> {
        let mut complexity_score = 0.0;
        let mut complexity_factors = Vec::new();
        
        // Base complexity from debug mode
        complexity_score += match self.mode {
            DebugMode::Interactive => 6.0,
            DebugMode::Analysis => 3.0,
            DebugMode::Trace => 5.0,
            DebugMode::Profile => 7.0,
            DebugMode::Health => 2.0,
            DebugMode::Network => 4.0,
        };
        
        // Issue complexity
        let critical_issues = detected_issues.iter()
            .filter(|i| matches!(i.severity, IssueSeverity::Critical))
            .count();
        let high_issues = detected_issues.iter()
            .filter(|i| matches!(i.severity, IssueSeverity::High))
            .count();
        
        let issue_complexity = (critical_issues as f32 * 3.0) + (high_issues as f32 * 2.0);
        complexity_score += issue_complexity;
        
        if critical_issues > 0 {
            complexity_factors.push(ComplexityFactor {
                factor_type: FactorType::IssueComplexity,
                impact: 3.0,
                description: format!("{} critical issues detected", critical_issues),
                evidence: detected_issues.iter()
                    .filter(|i| matches!(i.severity, IssueSeverity::Critical))
                    .map(|i| i.title.clone())
                    .collect(),
            });
        }
        
        // System complexity
        match &debug_session.debug_target {
            DebugTarget::System => {
                complexity_score += 4.0;
                complexity_factors.push(ComplexityFactor {
                    factor_type: FactorType::SystemComplexity,
                    impact: 4.0,
                    description: "System-wide debugging involves multiple components".to_string(),
                    evidence: vec!["Full system analysis required".to_string()],
                });
            },
            DebugTarget::Dataflow(name) => {
                // Get dataflow complexity
                let dataflow_complexity = self.calculate_dataflow_complexity(name).await?;
                complexity_score += dataflow_complexity;
                
                if dataflow_complexity > 3.0 {
                    complexity_factors.push(ComplexityFactor {
                        factor_type: FactorType::DataflowComplexity,
                        impact: dataflow_complexity,
                        description: "Complex dataflow with multiple nodes and dependencies".to_string(),
                        evidence: vec![format!("Dataflow: {}", name)],
                    });
                }
            },
            _ => {},
        }
        
        // Data volume complexity
        if self.live {
            complexity_score += 2.0;
            complexity_factors.push(ComplexityFactor {
                factor_type: FactorType::DataComplexity,
                impact: 2.0,
                description: "Live debugging involves real-time data streams".to_string(),
                evidence: vec!["Real-time monitoring enabled".to_string()],
            });
        }
        
        Ok(DebugComplexityAnalysis {
            overall_score: complexity_score.min(10.0),
            issue_complexity: issue_complexity,
            system_complexity: match &debug_session.debug_target {
                DebugTarget::System => 4.0,
                DebugTarget::Dataflow(_) => 3.0,
                _ => 1.0,
            },
            data_complexity: if self.live { 2.0 } else { 0.0 },
            factors: complexity_factors,
        })
    }
    
    fn apply_debug_overrides(
        &self,
        base_decision: InterfaceDecision,
        detected_issues: &[DetectedIssue],
    ) -> InterfaceDecision {
        // Force CLI if explicitly requested
        if self.text {
            return InterfaceDecision {
                strategy: InterfaceStrategy::CliOnly,
                confidence: 1.0,
                reasoning: "Explicit CLI mode requested".to_string(),
            };
        }
        
        // Force TUI if explicitly requested
        if self.tui {
            return InterfaceDecision {
                strategy: InterfaceStrategy::AutoLaunchTui {
                    reason: "Explicit TUI mode requested".to_string(),
                    show_cli_first: false,
                },
                confidence: 1.0,
                reasoning: "Explicit TUI mode requested".to_string(),
            };
        }
        
        // Auto-escalate to TUI for critical issues
        let has_critical_issues = detected_issues.iter()
            .any(|i| matches!(i.severity, IssueSeverity::Critical));
        
        if has_critical_issues && !matches!(base_decision.strategy, InterfaceStrategy::AutoLaunchTui { .. }) {
            return InterfaceDecision {
                strategy: InterfaceStrategy::AutoLaunchTui {
                    reason: "Critical issues detected - interactive debugging recommended".to_string(),
                    show_cli_first: true,
                },
                confidence: 0.95,
                reasoning: "Critical issues require interactive analysis".to_string(),
            };
        }
        
        base_decision
    }
}
```

#### 4. Interactive Debug Session TUI
```rust
// src/debug/interactive_session.rs
impl DebugCommand {
    async fn launch_interactive_debug_session(
        &self,
        debug_session: &DebugSession,
        detected_issues: Vec<DetectedIssue>,
    ) -> Result<()> {
        println!("🔍 Launching interactive debug session...");
        
        // Create debug session context
        let debug_context = InteractiveDebugContext {
            session: debug_session.clone(),
            issues: detected_issues,
            active_monitors: debug_session.monitors.clone(),
            user_actions: Vec::new(),
            debugging_state: DebuggingState::IssueOverview,
        };
        
        // Launch TUI with debug-specific view
        let tui_app = DoraApp::new_with_context(
            ViewType::DebugSession {
                session_id: debug_session.session_id,
                debug_mode: self.mode.clone(),
                focus: self.focus.clone(),
            },
            CliContext::from_debug_session(self, debug_context),
        );
        
        tui_app.run().await?;
        Ok(())
    }
    
    fn should_launch_interactive_debug(
        &self,
        reason: &str,
        detected_issues: &[DetectedIssue],
        default_yes: bool,
    ) -> Result<bool> {
        println!();
        println!("🔍 {}", reason);
        
        // Show what interactive debugging offers
        let interactive_benefits = self.get_debug_interactive_benefits(detected_issues);
        if !interactive_benefits.is_empty() {
            println!("Interactive debugging provides:");
            for benefit in interactive_benefits {
                println!("  • {}", benefit);
            }
        }
        
        let prompt = if default_yes {
            "Launch interactive debug session? [Y/n]: "
        } else {
            "Launch interactive debug session? [y/N]: "
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
    
    fn get_debug_interactive_benefits(&self, detected_issues: &[DetectedIssue]) -> Vec<String> {
        let mut benefits = Vec::new();
        
        // Always available benefits
        benefits.push("Real-time system monitoring and metric visualization".to_string());
        benefits.push("Interactive issue investigation with guided workflows".to_string());
        
        // Issue-specific benefits
        if detected_issues.iter().any(|i| matches!(i.issue_type, IssueType::PerformanceDegradation)) {
            benefits.push("Performance profiling with timeline visualization".to_string());
        }
        
        if detected_issues.iter().any(|i| matches!(i.issue_type, IssueType::DataflowStalled)) {
            benefits.push("Dataflow visualization with bottleneck identification".to_string());
        }
        
        if detected_issues.iter().any(|i| matches!(i.issue_type, IssueType::NetworkConnectivity)) {
            benefits.push("Network topology and connectivity analysis".to_string());
        }
        
        if self.live {
            benefits.push("Live debugging with real-time data capture".to_string());
        }
        
        if matches!(self.mode, DebugMode::Trace) {
            benefits.push("Interactive message tracing and flow visualization".to_string());
        }
        
        benefits
    }
    
    fn show_debug_hint(
        &self,
        hint: &str,
        tui_command: &str,
        detected_issues: &[DetectedIssue],
    ) {
        println!();
        println!("💡 {}", hint);
        println!("   Try: {}", tui_command);
        
        // Add specific debug hints based on issues
        if let Some(critical_issue) = detected_issues.iter()
            .find(|i| matches!(i.severity, IssueSeverity::Critical)) {
            println!("   🚨 Critical issue: {}", critical_issue.title);
            if let Some(action) = critical_issue.suggested_actions.first() {
                println!("   🔧 Suggested: {}", action.action);
            }
        }
        
        let high_priority_count = detected_issues.iter()
            .filter(|i| matches!(i.severity, IssueSeverity::High | IssueSeverity::Critical))
            .count();
        
        if high_priority_count > 1 {
            println!("   📊 {} high-priority issues detected - interactive analysis recommended", high_priority_count);
        }
    }
}
```

### Why This Approach

**Intelligent Escalation:**
- Automatic issue detection drives interface decisions
- Complexity-aware debugging mode selection
- Seamless escalation for critical problems

**Comprehensive Detection:**
- Multi-faceted issue detection across system layers
- Evidence-based problem diagnosis
- Contextual debugging suggestions

**Guided Troubleshooting:**
- Interactive workflows for complex debugging scenarios
- Real-time visualization of system state
- Expert-guided problem resolution

### How to Implement

#### Step 1: Enhanced Command Structure (4 hours)
1. **Implement DebugCommand** with comprehensive options
2. **Add debug mode** and focus area handling
3. **Create debug session** initialization and management
4. **Add artifact capture** and export functionality

#### Step 2: Automatic Issue Detection (6 hours)
1. **Implement AutomaticIssueDetector** with multiple specialized detectors
2. **Add PerformanceIssueDetector** for system performance problems
3. **Create DataflowIssueDetector** for dataflow-specific issues
4. **Add NetworkIssueDetector** and other specialized detectors

#### Step 3: Interface Selection Logic (3 hours)
1. **Implement debug complexity** analysis algorithms
2. **Add issue-based interface** escalation logic
3. **Create user override** handling for debug scenarios
4. **Add interactive benefit** explanation system

#### Step 4: Interactive Debug Session (4 hours)
1. **Implement interactive debug** session launching
2. **Create debug-specific TUI** context and views
3. **Add real-time monitoring** and visualization
4. **Implement guided troubleshooting** workflows

#### Step 5: Testing and Integration (2 hours)
1. **Add comprehensive unit tests** for all components
2. **Test issue detection** accuracy and performance
3. **Validate interface escalation** logic
4. **Test interactive debugging** workflows

## 🔗 Dependencies
**Depends On:**
- Issue #001 (Hybrid Command Framework) - CLI structure
- Issue #003 (Interface Selection Engine) - TUI suggestions
- Issue #013 (Complexity Calculation Algorithms) - Complexity analysis
- Issue #014 (Resource Analysis System) - Issue detection
- Issue #017 (Smart Inspect Command) - Analysis patterns

**Blocks:** Other enhanced commands that use similar debugging patterns

## 🧪 Testing Requirements

### Unit Tests
```rust
#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_issue_detection() {
        let detector = PerformanceIssueDetector::new();
        let debug_session = create_test_debug_session_with_performance_issues();
        
        let issues = detector.detect_issues(&debug_session).await.unwrap();
        
        assert!(!issues.is_empty());
        assert!(issues.iter().any(|i| matches!(i.issue_type, IssueType::PerformanceDegradation)));
    }
    
    #[test]
    fn test_debug_complexity_analysis() {
        let cmd = DebugCommand {
            mode: DebugMode::Interactive,
            live: true,
            ..Default::default()
        };
        let debug_session = create_test_debug_session();
        let critical_issues = vec![create_critical_issue()];
        
        let complexity = cmd.analyze_debug_complexity(&debug_session, &critical_issues).await.unwrap();
        
        assert!(complexity.overall_score > 7.0);
    }
    
    #[test]
    fn test_interface_escalation() {
        let cmd = DebugCommand::default();
        let base_decision = InterfaceDecision::cli_only();
        let critical_issues = vec![create_critical_issue()];
        
        let final_decision = cmd.apply_debug_overrides(base_decision, &critical_issues);
        
        assert!(matches!(final_decision.strategy, InterfaceStrategy::AutoLaunchTui { .. }));
    }
}
```

## ✅ Definition of Done
- [ ] DebugCommand implemented with comprehensive debugging capabilities
- [ ] Automatic issue detection identifies common problems accurately
- [ ] Interface escalation logic works correctly for different complexity levels
- [ ] Interactive debugging provides guided troubleshooting workflows
- [ ] Real-time monitoring and visualization enhance debugging experience
- [ ] Debug artifacts capture and export functionality works correctly
- [ ] Performance targets met for debug session initialization and operation
- [ ] Comprehensive unit tests validate debugging functionality
- [ ] Integration tests confirm end-to-end debug workflows
- [ ] Manual testing validates issue detection accuracy and user experience

This enhanced debug command demonstrates the hybrid CLI's ability to intelligently escalate from simple CLI debugging to powerful interactive debugging based on detected issues and system complexity.