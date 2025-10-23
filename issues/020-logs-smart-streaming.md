# Issue #020: Build Smart `dora logs` with Streaming Intelligence

## 📋 Summary
Implement an intelligent `dora logs` command that provides smart log streaming, filtering, and analysis with adaptive interface selection based on log volume, error patterns, and user context. This command demonstrates the hybrid CLI's ability to handle high-volume streaming data while providing intelligent insights and appropriate visualization modes.

## 🎯 Objectives
- Create intelligent log streaming with adaptive filtering and rate limiting
- Implement real-time log analysis and pattern detection
- Add smart interface escalation based on log complexity and error patterns
- Provide contextual log insights and troubleshooting guidance
- Enable seamless transitions between CLI streaming and interactive log exploration

**Success Metrics:**
- Log streaming performance handles 10,000+ logs/second without degradation
- Pattern detection identifies 90% of common issues within 30 seconds
- Interface suggestions are appropriate 85% of the time based on log patterns
- Interactive log analysis reduces debugging time by 60% compared to manual log review
- Smart filtering reduces information overload by 70% while maintaining relevance

## 🛠️ Technical Requirements

### What to Build

#### 1. Enhanced Logs Command Structure
```rust
// src/cli/commands/logs.rs
#[derive(Debug, clap::Args)]
pub struct LogsCommand {
    /// Target to get logs from (dataflow, node, or system)
    pub target: Option<String>,
    
    /// Follow log output (streaming mode)
    #[clap(short, long)]
    pub follow: bool,
    
    /// Number of lines to show from the end
    #[clap(short, long, default_value = "100")]
    pub tail: usize,
    
    /// Show logs since timestamp (e.g., "2h", "30m", "2023-10-01T10:00:00Z")
    #[clap(long)]
    pub since: Option<String>,
    
    /// Show logs until timestamp
    #[clap(long)]
    pub until: Option<String>,
    
    /// Filter logs by level
    #[clap(long, value_enum)]
    pub level: Vec<LogLevel>,
    
    /// Filter logs by pattern (regex supported)
    #[clap(long)]
    pub filter: Vec<String>,
    
    /// Exclude logs matching pattern
    #[clap(long)]
    pub exclude: Vec<String>,
    
    /// Show only logs containing errors
    #[clap(long)]
    pub errors_only: bool,
    
    /// Enable intelligent filtering
    #[clap(long)]
    pub smart_filter: bool,
    
    /// Show timestamps
    #[clap(long)]
    pub timestamps: bool,
    
    /// Output format
    #[clap(long, value_enum, default_value = "text")]
    pub format: LogFormat,
    
    /// Maximum log rate (logs per second)
    #[clap(long)]
    pub rate_limit: Option<u32>,
    
    /// Enable pattern detection and analysis
    #[clap(long)]
    pub analyze: bool,
    
    /// Force CLI text output
    #[clap(long)]
    pub text: bool,
    
    /// Force TUI interactive mode
    #[clap(long)]
    pub tui: bool,
    
    /// Auto-escalate to TUI for complex patterns
    #[clap(long)]
    pub auto_escalate: bool,
    
    /// Buffer size for streaming
    #[clap(long, default_value = "1000")]
    pub buffer_size: usize,
    
    /// Export logs to file
    #[clap(long)]
    pub export: Option<PathBuf>,
    
    /// Search for specific patterns
    #[clap(long)]
    pub search: Vec<String>,
    
    /// Context lines around matches
    #[clap(long, default_value = "3")]
    pub context: usize,
}

#[derive(Debug, Clone, clap::ValueEnum)]
pub enum LogLevel {
    Trace,
    Debug,
    Info,
    Warn,
    Error,
    Fatal,
}

#[derive(Debug, Clone, clap::ValueEnum)]
pub enum LogFormat {
    Text,      // Plain text output
    Json,      // JSON structured output
    Compact,   // Compact single-line format
    Detailed,  // Detailed multi-line format
    Colored,   // Colored text output
}

impl LogsCommand {
    pub async fn execute(&self, context: &ExecutionContext) -> Result<()> {
        // Initialize log streaming session
        let log_session = self.initialize_log_session(context).await?;
        
        // Set up log streaming with intelligent filtering
        let log_stream = self.create_log_stream(&log_session).await?;
        
        // Initialize pattern detection if enabled
        let pattern_detector = if self.analyze || self.auto_escalate {
            Some(LogPatternDetector::new(self.clone()))
        } else {
            None
        };
        
        // Determine initial interface strategy
        let interface_selector = InterfaceSelector::new(context.clone(), UserConfig::load()?);
        let initial_decision = interface_selector.select_interface(&Command::Logs(self.clone()));
        
        // Apply logs-specific overrides
        let final_decision = self.apply_logs_overrides(initial_decision);
        
        match final_decision.strategy {
            InterfaceStrategy::CliOnly => {
                self.execute_cli_streaming(&log_stream, pattern_detector, context).await?;
            },
            
            InterfaceStrategy::CliWithHint { hint, tui_command } => {
                // Start CLI streaming but monitor for escalation triggers
                if self.auto_escalate {
                    self.execute_adaptive_streaming(&log_stream, pattern_detector, context, Some((hint, tui_command))).await?;
                } else {
                    self.execute_cli_streaming(&log_stream, pattern_detector, context).await?;
                    self.show_logs_hint(&hint, &tui_command);
                }
            },
            
            InterfaceStrategy::PromptForTui { reason, default_yes } => {
                // Show initial logs, then prompt
                self.show_initial_logs(&log_stream, context).await?;
                
                if self.should_launch_interactive_logs(&reason, default_yes)? {
                    self.launch_interactive_logs(&log_session, log_stream).await?;
                } else {
                    self.execute_cli_streaming(&log_stream, pattern_detector, context).await?;
                }
            },
            
            InterfaceStrategy::AutoLaunchTui { reason, show_cli_first } => {
                if show_cli_first {
                    self.show_initial_logs(&log_stream, context).await?;
                    println!("\n📋 {}", reason);
                }
                self.launch_interactive_logs(&log_session, log_stream).await?;
            },
        }
        
        Ok(())
    }
    
    async fn initialize_log_session(&self, context: &ExecutionContext) -> Result<LogSession> {
        let session_id = Uuid::new_v4();
        let start_time = Utc::now();
        
        // Determine log target
        let log_target = if let Some(target) = &self.target {
            self.resolve_log_target(target).await?
        } else {
            LogTarget::System
        };
        
        // Parse time constraints
        let time_filter = TimeFilter {
            since: self.parse_time_constraint(&self.since)?,
            until: self.parse_time_constraint(&self.until)?,
        };
        
        // Create filter configuration
        let filter_config = LogFilterConfig {
            levels: if self.level.is_empty() { 
                vec![LogLevel::Info, LogLevel::Warn, LogLevel::Error, LogLevel::Fatal]
            } else { 
                self.level.clone() 
            },
            include_patterns: self.filter.clone(),
            exclude_patterns: self.exclude.clone(),
            smart_filtering: self.smart_filter,
            errors_only: self.errors_only,
            search_patterns: self.search.clone(),
        };
        
        // Initialize log collectors
        let collectors = self.initialize_log_collectors(&log_target).await?;
        
        Ok(LogSession {
            session_id,
            start_time,
            log_target,
            time_filter,
            filter_config,
            streaming_config: StreamingConfig {
                follow: self.follow,
                tail: self.tail,
                rate_limit: self.rate_limit,
                buffer_size: self.buffer_size,
            },
            collectors,
            pattern_buffer: RingBuffer::new(self.buffer_size),
        })
    }
}
```

#### 2. Intelligent Log Pattern Detection
```rust
// src/logs/pattern_detection.rs
#[derive(Debug)]
pub struct LogPatternDetector {
    error_detectors: Vec<Box<dyn ErrorPatternDetector>>,
    anomaly_detector: LogAnomalyDetector,
    pattern_buffer: PatternBuffer,
    analysis_window: Duration,
    escalation_thresholds: EscalationThresholds,
}

#[derive(Debug, Clone)]
pub struct DetectedPattern {
    pub pattern_id: String,
    pub pattern_type: LogPatternType,
    pub severity: PatternSeverity,
    pub confidence: f32,
    pub title: String,
    pub description: String,
    pub sample_logs: Vec<LogEntry>,
    pub frequency: PatternFrequency,
    pub first_seen: DateTime<Utc>,
    pub last_seen: DateTime<Utc>,
    pub suggested_actions: Vec<LogAction>,
    pub escalation_trigger: bool,
}

#[derive(Debug, Clone)]
pub enum LogPatternType {
    ErrorSpike,
    RepeatingError,
    PerformanceDegradation,
    SecurityAlert,
    UnusualActivity,
    SystemFailure,
    ResourceExhaustion,
    NetworkIssue,
    DataCorruption,
    ConfigurationError,
}

#[derive(Debug, Clone)]
pub struct PatternFrequency {
    pub count: u32,
    pub rate_per_minute: f32,
    pub trend: FrequencyTrend,
}

trait ErrorPatternDetector: Send + Sync {
    async fn detect_patterns(&self, logs: &[LogEntry]) -> Result<Vec<DetectedPattern>>;
    fn pattern_types(&self) -> Vec<LogPatternType>;
    fn detection_priority(&self) -> u8;
}

pub struct ErrorSpikeDetector {
    baseline_calculator: ErrorBaselineCalculator,
    spike_threshold: f32,
    window_size: Duration,
}

impl ErrorPatternDetector for ErrorSpikeDetector {
    async fn detect_patterns(&self, logs: &[LogEntry]) -> Result<Vec<DetectedPattern>> {
        let mut patterns = Vec::new();
        
        // Group logs by time windows
        let time_windows = self.group_logs_by_time_window(logs);
        
        for window in time_windows {
            let error_count = window.logs.iter()
                .filter(|log| matches!(log.level, LogLevel::Error | LogLevel::Fatal))
                .count();
            
            let baseline = self.baseline_calculator.get_baseline_for_period(&window.period).await?;
            
            // Detect spike
            if error_count as f32 > baseline.average_errors * self.spike_threshold {
                let pattern = DetectedPattern {
                    pattern_id: format!("error-spike-{}", window.period.start.timestamp()),
                    pattern_type: LogPatternType::ErrorSpike,
                    severity: self.calculate_spike_severity(error_count as f32, baseline.average_errors),
                    confidence: 0.9,
                    title: format!("Error Spike Detected: {} errors in {}", error_count, format_duration(self.window_size)),
                    description: format!(
                        "Error rate {} is {:.1}x higher than baseline of {:.1} errors/{}",
                        error_count,
                        error_count as f32 / baseline.average_errors,
                        baseline.average_errors,
                        format_duration(self.window_size)
                    ),
                    sample_logs: window.logs.iter()
                        .filter(|log| matches!(log.level, LogLevel::Error | LogLevel::Fatal))
                        .take(5)
                        .cloned()
                        .collect(),
                    frequency: PatternFrequency {
                        count: error_count as u32,
                        rate_per_minute: error_count as f32 / self.window_size.as_secs() as f32 * 60.0,
                        trend: FrequencyTrend::Increasing,
                    },
                    first_seen: window.period.start,
                    last_seen: window.period.end,
                    suggested_actions: vec![
                        LogAction {
                            action: "Investigate recent changes or deployments".to_string(),
                            command: Some("dora debug --focus errors --auto-detect".to_string()),
                            urgency: ActionUrgency::High,
                        },
                        LogAction {
                            action: "Check system resource usage".to_string(),
                            command: Some("dora analyze --focus resource".to_string()),
                            urgency: ActionUrgency::Medium,
                        },
                    ],
                    escalation_trigger: error_count as f32 > baseline.average_errors * 3.0,
                };
                
                patterns.push(pattern);
            }
        }
        
        Ok(patterns)
    }
    
    fn pattern_types(&self) -> Vec<LogPatternType> {
        vec![LogPatternType::ErrorSpike]
    }
    
    fn detection_priority(&self) -> u8 {
        95
    }
}

pub struct RepeatingErrorDetector {
    similarity_threshold: f32,
    frequency_threshold: u32,
}

impl ErrorPatternDetector for RepeatingErrorDetector {
    async fn detect_patterns(&self, logs: &[LogEntry]) -> Result<Vec<DetectedPattern>> {
        let mut patterns = Vec::new();
        
        // Group similar error messages
        let error_groups = self.group_similar_errors(logs);
        
        for group in error_groups {
            if group.count >= self.frequency_threshold {
                let pattern = DetectedPattern {
                    pattern_id: format!("repeating-error-{}", Self::hash_error_signature(&group.signature)),
                    pattern_type: LogPatternType::RepeatingError,
                    severity: self.calculate_repetition_severity(group.count),
                    confidence: group.similarity_score,
                    title: format!("Repeating Error: {} occurrences", group.count),
                    description: format!(
                        "Error pattern '{}' repeated {} times",
                        group.signature,
                        group.count
                    ),
                    sample_logs: group.sample_logs,
                    frequency: PatternFrequency {
                        count: group.count,
                        rate_per_minute: group.rate_per_minute,
                        trend: group.trend,
                    },
                    first_seen: group.first_seen,
                    last_seen: group.last_seen,
                    suggested_actions: vec![
                        LogAction {
                            action: "Analyze error pattern for root cause".to_string(),
                            command: Some(format!("dora logs --search '{}' --analyze", group.signature)),
                            urgency: ActionUrgency::High,
                        },
                    ],
                    escalation_trigger: group.count > self.frequency_threshold * 3,
                };
                
                patterns.push(pattern);
            }
        }
        
        Ok(patterns)
    }
    
    fn pattern_types(&self) -> Vec<LogPatternType> {
        vec![LogPatternType::RepeatingError]
    }
    
    fn detection_priority(&self) -> u8 {
        90
    }
}

impl LogPatternDetector {
    pub async fn analyze_log_stream(&mut self, log_entry: &LogEntry) -> Result<Option<EscalationTrigger>> {
        // Add log to pattern buffer
        self.pattern_buffer.add_log(log_entry.clone());
        
        // Run pattern detection on buffer
        let recent_logs = self.pattern_buffer.get_recent_logs(self.analysis_window);
        let mut detected_patterns = Vec::new();
        
        for detector in &self.error_detectors {
            let patterns = detector.detect_patterns(&recent_logs).await?;
            detected_patterns.extend(patterns);
        }
        
        // Check for escalation triggers
        for pattern in &detected_patterns {
            if pattern.escalation_trigger {
                return Ok(Some(EscalationTrigger {
                    trigger_type: EscalationTriggerType::PatternDetected,
                    severity: pattern.severity.clone(),
                    reason: format!("Critical pattern detected: {}", pattern.title),
                    detected_pattern: Some(pattern.clone()),
                    suggested_action: "Switch to interactive analysis for detailed investigation".to_string(),
                }));
            }
        }
        
        // Check anomaly-based escalation
        if let Some(anomaly_trigger) = self.check_anomaly_escalation(&recent_logs).await? {
            return Ok(Some(anomaly_trigger));
        }
        
        Ok(None)
    }
    
    async fn check_anomaly_escalation(&self, logs: &[LogEntry]) -> Result<Option<EscalationTrigger>> {
        let anomalies = self.anomaly_detector.detect_anomalies(logs).await?;
        
        let critical_anomalies = anomalies.iter()
            .filter(|a| matches!(a.severity, AnomalySeverity::Critical))
            .count();
        
        if critical_anomalies > 0 {
            Ok(Some(EscalationTrigger {
                trigger_type: EscalationTriggerType::AnomalyDetected,
                severity: PatternSeverity::Critical,
                reason: format!("{} critical anomalies detected in log stream", critical_anomalies),
                detected_pattern: None,
                suggested_action: "Interactive analysis recommended for anomaly investigation".to_string(),
            }))
        } else {
            Ok(None)
        }
    }
}
```

#### 3. Adaptive Log Streaming
```rust
// src/logs/adaptive_streaming.rs
impl LogsCommand {
    async fn execute_adaptive_streaming(
        &self,
        log_stream: &LogStream,
        mut pattern_detector: Option<LogPatternDetector>,
        context: &ExecutionContext,
        hint_info: Option<(String, String)>,
    ) -> Result<()> {
        let mut escalation_triggered = false;
        let mut log_buffer = LogBuffer::new(self.buffer_size);
        let mut rate_limiter = RateLimiter::new(self.rate_limit.unwrap_or(1000));
        
        // Set up signal handling for graceful shutdown
        let (shutdown_tx, mut shutdown_rx) = tokio::sync::oneshot::channel();
        self.setup_signal_handlers(shutdown_tx);
        
        println!("📋 Streaming logs... (Ctrl+C to stop)");
        if self.auto_escalate {
            println!("🤖 Smart escalation enabled - will suggest interactive mode for complex patterns");
        }
        
        loop {
            tokio::select! {
                log_result = log_stream.next_log() => {
                    match log_result {
                        Ok(Some(log_entry)) => {
                            // Apply rate limiting
                            if !rate_limiter.should_allow(&log_entry) {
                                continue;
                            }
                            
                            // Add to buffer
                            log_buffer.add(log_entry.clone());
                            
                            // Check for pattern-based escalation
                            if let Some(ref mut detector) = pattern_detector {
                                if let Some(escalation) = detector.analyze_log_stream(&log_entry).await? {
                                    escalation_triggered = true;
                                    
                                    if self.should_auto_escalate(&escalation) {
                                        println!("\n🚨 {}", escalation.reason);
                                        println!("💡 {}", escalation.suggested_action);
                                        
                                        if self.prompt_for_escalation(&escalation)? {
                                            // Launch interactive mode
                                            self.launch_interactive_logs_with_context(
                                                log_stream,
                                                &log_buffer,
                                                Some(escalation)
                                            ).await?;
                                            break;
                                        }
                                    }
                                }
                            }
                            
                            // Render log entry
                            self.render_log_entry(&log_entry, context)?;
                        },
                        Ok(None) => {
                            // End of stream
                            break;
                        },
                        Err(e) => {
                            eprintln!("Error reading logs: {}", e);
                            break;
                        }
                    }
                },
                
                _ = &mut shutdown_rx => {
                    println!("\nShutting down log streaming...");
                    break;
                }
            }
        }
        
        // Show hint if provided and no escalation occurred
        if !escalation_triggered {
            if let Some((hint, tui_command)) = hint_info {
                self.show_logs_hint(&hint, &tui_command);
            }
        }
        
        Ok(())
    }
    
    fn should_auto_escalate(&self, escalation: &EscalationTrigger) -> bool {
        match escalation.trigger_type {
            EscalationTriggerType::PatternDetected => {
                matches!(escalation.severity, PatternSeverity::Critical | PatternSeverity::High)
            },
            EscalationTriggerType::AnomalyDetected => {
                matches!(escalation.severity, PatternSeverity::Critical)
            },
            EscalationTriggerType::VolumeThreshold => {
                true // Always escalate for volume issues
            },
        }
    }
    
    fn prompt_for_escalation(&self, escalation: &EscalationTrigger) -> Result<bool> {
        println!();
        println!("Switch to interactive log analysis? [Y/n]: ");
        
        let mut input = String::new();
        io::stdin().read_line(&mut input)?;
        let input = input.trim().to_lowercase();
        
        Ok(match input.as_str() {
            "" | "y" | "yes" => true,
            "n" | "no" => false,
            _ => true, // Default to yes for critical issues
        })
    }
    
    fn render_log_entry(&self, log_entry: &LogEntry, context: &ExecutionContext) -> Result<()> {
        let timestamp_str = if self.timestamps {
            format!("{} ", log_entry.timestamp.format("%H:%M:%S%.3f"))
        } else {
            String::new()
        };
        
        let level_str = if context.terminal_capabilities.supports_color {
            self.colorize_log_level(&log_entry.level)
        } else {
            format!("[{}]", log_entry.level)
        };
        
        let source_str = if let Some(source) = &log_entry.source {
            format!(" {}", source)
        } else {
            String::new()
        };
        
        match self.format {
            LogFormat::Text | LogFormat::Colored => {
                println!("{}{}{}: {}", timestamp_str, level_str, source_str, log_entry.message);
            },
            LogFormat::Json => {
                let json_entry = serde_json::json!({
                    "timestamp": log_entry.timestamp,
                    "level": log_entry.level,
                    "source": log_entry.source,
                    "message": log_entry.message,
                    "fields": log_entry.fields,
                });
                println!("{}", json_entry);
            },
            LogFormat::Compact => {
                println!("{}{} {}", level_str, source_str, log_entry.message);
            },
            LogFormat::Detailed => {
                println!("┌─ {} {} {}", timestamp_str, level_str, source_str);
                println!("│  {}", log_entry.message);
                if !log_entry.fields.is_empty() {
                    for (key, value) in &log_entry.fields {
                        println!("│  {}: {}", key, value);
                    }
                }
                println!("└─");
            },
        }
        
        Ok(())
    }
    
    fn colorize_log_level(&self, level: &LogLevel) -> String {
        match level {
            LogLevel::Trace => "\x1b[90m[TRACE]\x1b[0m".to_string(),
            LogLevel::Debug => "\x1b[36m[DEBUG]\x1b[0m".to_string(),
            LogLevel::Info => "\x1b[32m[INFO]\x1b[0m".to_string(),
            LogLevel::Warn => "\x1b[33m[WARN]\x1b[0m".to_string(),
            LogLevel::Error => "\x1b[31m[ERROR]\x1b[0m".to_string(),
            LogLevel::Fatal => "\x1b[35m[FATAL]\x1b[0m".to_string(),
        }
    }
}
```

#### 4. Interactive Log Analysis TUI
```rust
// src/logs/interactive_analysis.rs
impl LogsCommand {
    async fn launch_interactive_logs(
        &self,
        log_session: &LogSession,
        log_stream: LogStream,
    ) -> Result<()> {
        println!("📋 Launching interactive log analysis...");
        
        // Create interactive log context
        let log_context = InteractiveLogContext {
            session: log_session.clone(),
            stream: log_stream,
            analysis_state: LogAnalysisState::StreamingOverview,
            detected_patterns: Vec::new(),
            filter_state: FilterState::from_config(&log_session.filter_config),
            search_state: SearchState::new(),
            view_config: LogViewConfig::default(),
        };
        
        // Launch TUI with log-specific view
        let tui_app = DoraApp::new_with_context(
            ViewType::LogAnalysis {
                session_id: log_session.session_id,
                target: log_session.log_target.clone(),
            },
            CliContext::from_log_session(self, log_context),
        );
        
        tui_app.run().await?;
        Ok(())
    }
    
    async fn launch_interactive_logs_with_context(
        &self,
        log_stream: &LogStream,
        log_buffer: &LogBuffer,
        escalation: Option<EscalationTrigger>,
    ) -> Result<()> {
        println!("📋 Launching interactive log analysis with detected patterns...");
        
        // Create enriched interactive context
        let mut log_context = InteractiveLogContext {
            session: LogSession::from_stream_context(log_stream),
            stream: log_stream.clone(),
            analysis_state: LogAnalysisState::PatternInvestigation,
            detected_patterns: Vec::new(),
            filter_state: FilterState::from_config(&log_stream.filter_config),
            search_state: SearchState::new(),
            view_config: LogViewConfig::default(),
        };
        
        // Add escalation context
        if let Some(escalation) = escalation {
            log_context.analysis_state = LogAnalysisState::EscalationTriggered {
                trigger: escalation.clone(),
            };
            
            if let Some(pattern) = escalation.detected_pattern {
                log_context.detected_patterns.push(pattern);
            }
        }
        
        // Pre-load recent logs from buffer
        log_context.session.preloaded_logs = log_buffer.get_all_logs();
        
        // Launch TUI with escalation context
        let tui_app = DoraApp::new_with_context(
            ViewType::LogAnalysis {
                session_id: log_context.session.session_id,
                target: log_context.session.log_target.clone(),
            },
            CliContext::from_log_escalation(self, log_context),
        );
        
        tui_app.run().await?;
        Ok(())
    }
    
    fn should_launch_interactive_logs(&self, reason: &str, default_yes: bool) -> Result<bool> {
        println!();
        println!("📋 {}", reason);
        
        // Show interactive log analysis benefits
        let interactive_benefits = self.get_interactive_log_benefits();
        if !interactive_benefits.is_empty() {
            println!("Interactive log analysis provides:");
            for benefit in interactive_benefits {
                println!("  • {}", benefit);
            }
        }
        
        let prompt = if default_yes {
            "Launch interactive log analysis? [Y/n]: "
        } else {
            "Launch interactive log analysis? [y/N]: "
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
    
    fn get_interactive_log_benefits(&self) -> Vec<String> {
        let mut benefits = Vec::new();
        
        // Always available benefits
        benefits.push("Real-time log filtering and search".to_string());
        benefits.push("Pattern visualization and correlation analysis".to_string());
        benefits.push("Interactive log context and drill-down capabilities".to_string());
        
        // Conditional benefits
        if self.follow {
            benefits.push("Advanced streaming controls and rate management".to_string());
        }
        
        if self.analyze {
            benefits.push("Automated pattern detection and anomaly highlighting".to_string());
        }
        
        if !self.search.is_empty() {
            benefits.push("Enhanced search with context visualization".to_string());
        }
        
        if self.errors_only || self.level.iter().any(|l| matches!(l, LogLevel::Error | LogLevel::Fatal)) {
            benefits.push("Error correlation and troubleshooting workflows".to_string());
        }
        
        benefits.push("Export and sharing of filtered log segments".to_string());
        
        benefits
    }
    
    fn show_logs_hint(&self, hint: &str, tui_command: &str) {
        println!();
        println!("💡 {}", hint);
        println!("   Try: {}", tui_command);
        
        if self.follow && !self.analyze {
            println!("   💡 Add --analyze flag for intelligent pattern detection");
        }
        
        if !self.errors_only && self.level.is_empty() {
            println!("   🔍 Use --errors-only for focused error analysis");
        }
    }
}
```

### Why This Approach

**Intelligent Streaming:**
- Adaptive rate limiting and filtering based on log volume
- Real-time pattern detection with automatic escalation
- Smart buffering for context preservation

**Pattern-Driven Escalation:**
- Error spikes and anomalies trigger interface suggestions
- Contextual escalation based on detected patterns
- Seamless transition to interactive analysis

**Comprehensive Analysis:**
- Multi-dimensional log pattern detection
- Real-time and historical log correlation
- Actionable insights and troubleshooting guidance

### How to Implement

#### Step 1: Enhanced Command Structure (3 hours)
1. **Implement LogsCommand** with comprehensive streaming options
2. **Add log filtering** and pattern configuration
3. **Create log session** initialization and management
4. **Add export and formatting** capabilities

#### Step 2: Pattern Detection System (6 hours)
1. **Implement LogPatternDetector** with multiple specialized detectors
2. **Add ErrorSpikeDetector** for anomalous error rate detection
3. **Create RepeatingErrorDetector** for pattern recognition
4. **Add LogAnomalyDetector** for unusual log behavior

#### Step 3: Adaptive Streaming (4 hours)
1. **Implement adaptive streaming** with pattern-based escalation
2. **Add rate limiting** and buffer management
3. **Create escalation trigger** logic and user prompting
4. **Add graceful shutdown** and signal handling

#### Step 4: Interactive Log Analysis (4 hours)
1. **Implement interactive log** analysis launching
2. **Create log-specific TUI** context and views
3. **Add pattern investigation** workflows
4. **Implement enhanced search** and filtering

#### Step 5: Testing and Integration (2 hours)
1. **Add comprehensive unit tests** for all components
2. **Test pattern detection** accuracy and performance
3. **Validate streaming performance** under high load
4. **Test interactive analysis** workflows

## 🔗 Dependencies
**Depends On:**
- Issue #001 (Hybrid Command Framework) - CLI structure
- Issue #003 (Interface Selection Engine) - TUI suggestions
- Issue #014 (Resource Analysis System) - Pattern detection
- Issue #016 (User Preference Handling) - User context

**Blocks:** Other enhanced commands that use similar streaming patterns

## 🧪 Testing Requirements

### Unit Tests
```rust
#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_error_spike_detection() {
        let detector = ErrorSpikeDetector::new();
        let logs = create_error_spike_logs();
        
        let patterns = detector.detect_patterns(&logs).await.unwrap();
        
        assert!(!patterns.is_empty());
        assert!(patterns.iter().any(|p| matches!(p.pattern_type, LogPatternType::ErrorSpike)));
    }
    
    #[test]
    fn test_adaptive_streaming_escalation() {
        let mut detector = LogPatternDetector::new(LogsCommand::default());
        let critical_log = create_critical_error_log();
        
        let escalation = detector.analyze_log_stream(&critical_log).await.unwrap();
        
        assert!(escalation.is_some());
        assert!(escalation.unwrap().escalation_trigger);
    }
    
    #[test]
    fn test_rate_limiting() {
        let rate_limiter = RateLimiter::new(Some(100));
        let logs = create_high_volume_logs(1000);
        
        let allowed_count = logs.iter()
            .filter(|log| rate_limiter.should_allow(log))
            .count();
        
        assert!(allowed_count <= 100);
    }
}
```

## ✅ Definition of Done
- [ ] LogsCommand implemented with comprehensive streaming capabilities
- [ ] Pattern detection identifies common log issues accurately
- [ ] Adaptive streaming provides intelligent escalation based on detected patterns
- [ ] Rate limiting and buffering handle high-volume log streams efficiently
- [ ] Interactive log analysis provides powerful exploration and investigation tools
- [ ] Real-time pattern detection triggers appropriate interface suggestions
- [ ] Performance targets met for streaming speed and pattern detection latency
- [ ] Comprehensive unit tests validate streaming functionality and pattern detection
- [ ] Integration tests confirm end-to-end log analysis workflows
- [ ] Manual testing validates pattern detection accuracy and user experience

This smart logs command demonstrates the hybrid CLI's ability to handle high-volume streaming data while providing intelligent insights and seamless escalation to interactive analysis when complex patterns are detected.