# Issue #007: Build `dora logs` with TUI Suggestions

## 📋 Summary
Implement an enhanced `dora logs` command that provides intelligent log viewing with smart TUI suggestions for complex log analysis scenarios. This command demonstrates the hybrid approach by offering traditional log viewing for simple cases and suggesting interactive log analysis for complex situations.

## 🎯 Objectives
- Create fast, efficient log viewing with Docker-like interface patterns
- Implement smart TUI suggestions for complex log analysis scenarios
- Add real-time log streaming with intelligent buffering
- Provide filtering, searching, and error correlation capabilities
- Ensure excellent performance even with high-volume log streams

**Success Metrics:**
- Log streaming starts in <100ms for any target
- TUI suggestions appear appropriately for error-heavy or complex log scenarios
- Filtering and search work efficiently with large log volumes
- Memory usage stays bounded even with continuous streaming
- Error correlation and analysis provide actionable insights

## 🛠️ Technical Requirements

### What to Build

#### 1. Enhanced Logs Command Structure
```rust
// src/cli/commands/logs.rs
#[derive(Debug, clap::Args)]
pub struct LogsCommand {
    /// Target to show logs for (dataflow, node, or system)
    pub target: String,
    
    /// Follow log output (like tail -f)
    #[clap(short, long)]
    pub follow: bool,
    
    /// Number of lines to show from the end
    #[clap(short, long, default_value = "100")]
    pub tail: usize,
    
    /// Show logs since timestamp (RFC3339 format)
    #[clap(long)]
    pub since: Option<DateTime<Utc>>,
    
    /// Show logs until timestamp (RFC3339 format)  
    #[clap(long)]
    pub until: Option<DateTime<Utc>>,
    
    /// Filter by log level
    #[clap(long, value_enum)]
    pub level: Option<LogLevel>,
    
    /// Filter by log message pattern (regex)
    #[clap(long)]
    pub filter: Option<String>,
    
    /// Search for specific text in logs
    #[clap(long)]
    pub search: Option<String>,
    
    /// Show only error-related logs and context
    #[clap(long)]
    pub errors_only: bool,
    
    /// Enable automatic error analysis
    #[clap(long)]
    pub analyze_errors: bool,
    
    /// Output format
    #[clap(long, value_enum)]
    pub format: Option<OutputFormat>,
    
    /// Show raw log format without processing
    #[clap(long)]
    pub raw: bool,
    
    /// Suppress hints and suggestions
    #[clap(long)]
    pub no_hints: bool,
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

impl LogsCommand {
    pub async fn execute(&self, context: &ExecutionContext) -> Result<()> {
        let log_target = self.resolve_log_target().await?;
        let log_stream = self.create_log_stream(&log_target).await?;
        
        // Analyze log complexity for interface decision
        let complexity_analysis = self.analyze_log_complexity(&log_target).await?;
        
        // Determine interface strategy
        let interface_selector = InterfaceSelector::new(context.clone(), UserConfig::load()?);
        let decision = interface_selector.select_interface(&Command::Logs(self.clone()));
        
        match decision.strategy {
            InterfaceStrategy::CliOnly => {
                self.stream_logs_cli(log_stream, context).await?;
            },
            
            InterfaceStrategy::CliWithHint { hint, tui_command } => {
                self.stream_logs_cli(log_stream, context).await?;
                if !self.no_hints && complexity_analysis.should_suggest_tui() {
                    self.show_log_analysis_hint(&hint, &tui_command);
                }
            },
            
            InterfaceStrategy::PromptForTui { reason, default_yes } => {
                // Show some initial logs first
                let preview_logs = self.get_log_preview(&log_target, 50).await?;
                self.render_log_preview(&preview_logs, context);
                
                if complexity_analysis.high_error_rate() || complexity_analysis.complex_patterns() {
                    if !self.no_hints && self.should_prompt_for_interactive(&reason, default_yes)? {
                        self.launch_interactive_log_viewer(&log_target).await?;
                        return Ok(());
                    }
                }
                
                self.stream_logs_cli(log_stream, context).await?;
            },
            
            InterfaceStrategy::AutoLaunchTui { reason, show_cli_first } => {
                if show_cli_first {
                    let preview_logs = self.get_log_preview(&log_target, 20).await?;
                    self.render_log_preview(&preview_logs, context);
                    println!("\n🚀 {}", reason);
                }
                self.launch_interactive_log_viewer(&log_target).await?;
            },
        }
        
        Ok(())
    }
}
```

#### 2. Log Streaming and Processing
```rust
#[derive(Debug)]
pub struct LogStream {
    target: LogTarget,
    receiver: Receiver<LogEntry>,
    filters: Vec<LogFilter>,
    buffer: CircularBuffer<LogEntry>,
    error_analyzer: ErrorAnalyzer,
}

#[derive(Debug, Clone)]
pub struct LogEntry {
    pub timestamp: DateTime<Utc>,
    pub level: LogLevel,
    pub source: String,
    pub message: String,
    pub context: HashMap<String, String>,
    pub raw: String,
}

#[derive(Debug)]
pub struct LogComplexityAnalysis {
    pub total_entries: usize,
    pub error_rate: f32,
    pub warning_rate: f32,
    pub unique_error_patterns: usize,
    pub log_volume_per_second: f32,
    pub correlation_opportunities: usize,
    pub analysis_benefit: AnalysisBenefit,
}

impl LogComplexityAnalysis {
    pub fn should_suggest_tui(&self) -> bool {
        self.error_rate > 0.1 || // >10% errors
        self.unique_error_patterns > 5 || // Many different error types
        self.log_volume_per_second > 50.0 || // High volume
        self.correlation_opportunities > 0 // Patterns to analyze
    }
    
    pub fn high_error_rate(&self) -> bool {
        self.error_rate > 0.2 || self.unique_error_patterns > 10
    }
    
    pub fn complex_patterns(&self) -> bool {
        self.correlation_opportunities > 3 || self.unique_error_patterns > 8
    }
}

impl LogsCommand {
    async fn stream_logs_cli(
        &self,
        mut log_stream: LogStream,
        context: &ExecutionContext,
    ) -> Result<()> {
        let formatter = self.create_log_formatter(context);
        let mut error_tracker = ErrorTracker::new();
        
        println!("📋 Streaming logs for '{}'... (Press Ctrl+C to stop)", self.target);
        
        // Handle Ctrl+C gracefully
        let ctrl_c = tokio::signal::ctrl_c();
        tokio::pin!(ctrl_c);
        
        loop {
            tokio::select! {
                log_entry = log_stream.next() => {
                    match log_entry {
                        Some(entry) => {
                            if self.should_display_entry(&entry) {
                                self.display_log_entry(&entry, &formatter, context);
                                
                                // Track errors for potential suggestions
                                if entry.level == LogLevel::Error {
                                    error_tracker.add_error(&entry);
                                    
                                    // Suggest interactive analysis after many errors
                                    if error_tracker.should_suggest_analysis() && !self.no_hints {
                                        self.suggest_error_analysis(&error_tracker);
                                    }
                                }
                            }
                        },
                        None => {
                            if !self.follow {
                                break;
                            }
                            // Brief pause before checking for more logs
                            tokio::time::sleep(Duration::from_millis(100)).await;
                        }
                    }
                },
                _ = &mut ctrl_c => {
                    println!("\n📊 Log streaming stopped.");
                    self.show_session_summary(&error_tracker);
                    break;
                }
            }
        }
        
        Ok(())
    }
    
    fn display_log_entry(
        &self,
        entry: &LogEntry,
        formatter: &LogFormatter,
        context: &ExecutionContext,
    ) {
        if self.raw {
            println!("{}", entry.raw);
            return;
        }
        
        let formatted = formatter.format_entry(entry);
        
        if context.terminal_capabilities.supports_color {
            self.display_colored_log(&formatted, entry);
        } else {
            println!("{}", formatted);
        }
    }
    
    fn display_colored_log(&self, formatted: &str, entry: &LogEntry) {
        let color_code = match entry.level {
            LogLevel::Error | LogLevel::Fatal => "\x1b[31m", // Red
            LogLevel::Warn => "\x1b[33m",                    // Yellow
            LogLevel::Info => "\x1b[37m",                    // White
            LogLevel::Debug => "\x1b[36m",                   // Cyan
            LogLevel::Trace => "\x1b[90m",                   // Gray
        };
        
        println!("{}{}\x1b[0m", color_code, formatted);
    }
    
    async fn analyze_log_complexity(&self, target: &LogTarget) -> Result<LogComplexityAnalysis> {
        // Sample recent logs to analyze complexity
        let sample_size = 1000;
        let sample_logs = self.get_log_sample(target, sample_size).await?;
        
        let total_entries = sample_logs.len();
        let error_count = sample_logs.iter().filter(|e| e.level == LogLevel::Error).count();
        let warning_count = sample_logs.iter().filter(|e| e.level == LogLevel::Warn).count();
        
        let error_rate = error_count as f32 / total_entries as f32;
        let warning_rate = warning_count as f32 / total_entries as f32;
        
        // Analyze error patterns
        let error_analyzer = ErrorAnalyzer::new();
        let error_patterns = error_analyzer.analyze_patterns(&sample_logs);
        let unique_error_patterns = error_patterns.len();
        
        // Calculate log volume (rough estimate)
        let time_span = if let (Some(first), Some(last)) = (sample_logs.first(), sample_logs.last()) {
            last.timestamp.signed_duration_since(first.timestamp).num_seconds() as f32
        } else {
            1.0
        };
        let log_volume_per_second = total_entries as f32 / time_span.max(1.0);
        
        // Identify correlation opportunities
        let correlation_opportunities = error_analyzer.find_correlation_opportunities(&sample_logs);
        
        let analysis_benefit = match (error_rate, unique_error_patterns, log_volume_per_second) {
            (e, _, _) if e > 0.3 => AnalysisBenefit::Critical,
            (e, p, _) if e > 0.1 && p > 5 => AnalysisBenefit::High,
            (_, p, v) if p > 3 || v > 100.0 => AnalysisBenefit::Medium,
            _ => AnalysisBenefit::Low,
        };
        
        Ok(LogComplexityAnalysis {
            total_entries,
            error_rate,
            warning_rate,
            unique_error_patterns,
            log_volume_per_second,
            correlation_opportunities,
            analysis_benefit,
        })
    }
}
```

#### 3. Error Analysis and Correlation
```rust
#[derive(Debug)]
pub struct ErrorAnalyzer {
    pattern_detector: PatternDetector,
    correlation_engine: CorrelationEngine,
}

#[derive(Debug)]
pub struct ErrorPattern {
    pub pattern: String,
    pub frequency: usize,
    pub first_seen: DateTime<Utc>,
    pub last_seen: DateTime<Utc>,
    pub affected_sources: HashSet<String>,
    pub severity: ErrorSeverity,
}

#[derive(Debug)]
pub struct ErrorTracker {
    recent_errors: VecDeque<LogEntry>,
    error_patterns: HashMap<String, ErrorPattern>,
    error_burst_detector: BurstDetector,
    suggestion_cooldown: Option<Instant>,
}

impl ErrorTracker {
    pub fn new() -> Self {
        Self {
            recent_errors: VecDeque::new(),
            error_patterns: HashMap::new(),
            error_burst_detector: BurstDetector::new(),
            suggestion_cooldown: None,
        }
    }
    
    pub fn add_error(&mut self, entry: &LogEntry) {
        self.recent_errors.push_back(entry.clone());
        
        // Keep only recent errors (last 100)
        if self.recent_errors.len() > 100 {
            self.recent_errors.pop_front();
        }
        
        // Update error patterns
        let pattern_key = self.extract_error_pattern(&entry.message);
        self.error_patterns.entry(pattern_key)
            .and_modify(|p| {
                p.frequency += 1;
                p.last_seen = entry.timestamp;
                p.affected_sources.insert(entry.source.clone());
            })
            .or_insert(ErrorPattern {
                pattern: entry.message.clone(),
                frequency: 1,
                first_seen: entry.timestamp,
                last_seen: entry.timestamp,
                affected_sources: {
                    let mut set = HashSet::new();
                    set.insert(entry.source.clone());
                    set
                },
                severity: self.classify_error_severity(&entry.message),
            });
        
        // Check for error bursts
        self.error_burst_detector.add_event(entry.timestamp);
    }
    
    pub fn should_suggest_analysis(&mut self) -> bool {
        // Avoid suggesting too frequently
        if let Some(last_suggestion) = self.suggestion_cooldown {
            if last_suggestion.elapsed() < Duration::from_secs(300) { // 5 minute cooldown
                return false;
            }
        }
        
        // Suggest if we have significant error activity
        let high_error_count = self.recent_errors.len() > 20;
        let error_burst = self.error_burst_detector.is_in_burst();
        let multiple_patterns = self.error_patterns.len() > 3;
        let critical_errors = self.error_patterns.values()
            .any(|p| p.severity == ErrorSeverity::Critical);
        
        if high_error_count || error_burst || multiple_patterns || critical_errors {
            self.suggestion_cooldown = Some(Instant::now());
            return true;
        }
        
        false
    }
    
    fn extract_error_pattern(&self, message: &str) -> String {
        // Simple pattern extraction - replace numbers and specific values with placeholders
        let pattern = message
            .replace(char::is_numeric, "N")
            .replace("localhost", "HOST")
            .replace(|c: char| c.is_ascii_hexdigit() && c.is_alphabetic(), "X");
        
        // Take first 100 chars to avoid too-specific patterns
        pattern.chars().take(100).collect()
    }
}

impl LogsCommand {
    fn suggest_error_analysis(&self, error_tracker: &ErrorTracker) {
        let error_count = error_tracker.recent_errors.len();
        let pattern_count = error_tracker.error_patterns.len();
        
        println!();
        println!("🚨 High error activity detected ({} errors, {} patterns)", 
                error_count, pattern_count);
        println!("💡 Consider interactive log analysis for better insights:");
        println!("   dora ui logs {}", self.target);
        println!("   - Error correlation and timeline view");
        println!("   - Pattern analysis and filtering");
        println!("   - Real-time error tracking");
        println!();
    }
}
```

#### 4. Interactive Log Viewer Integration
```rust
impl LogsCommand {
    async fn launch_interactive_log_viewer(&self, target: &LogTarget) -> Result<()> {
        println!("🔍 Launching interactive log viewer for '{}'...", target.name());
        
        // Create TUI log viewer with current command context
        let log_viewer_context = LogViewerContext {
            target: target.clone(),
            filters: self.create_filters(),
            follow_mode: self.follow,
            search_term: self.search.clone(),
            level_filter: self.level.clone(),
        };
        
        let tui_app = DoraApp::new_with_context(
            ViewType::LogViewer { 
                target: target.name().to_string() 
            },
            CliContext::from_logs_command(self, log_viewer_context),
        );
        
        tui_app.run().await?;
        Ok(())
    }
    
    fn should_prompt_for_interactive(&self, reason: &str, default_yes: bool) -> Result<bool> {
        println!();
        println!("🔍 {}", reason);
        println!("Interactive log viewer provides:");
        println!("  • Real-time error correlation and analysis");
        println!("  • Advanced filtering and search capabilities");
        println!("  • Timeline view and pattern detection");
        println!("  • Performance impact analysis");
        
        let prompt = if default_yes {
            "Launch interactive log viewer? [Y/n]: "
        } else {
            "Launch interactive log viewer? [y/N]: "
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
    
    fn show_log_analysis_hint(&self, hint: &str, tui_command: &str) {
        println!();
        println!("💡 {}", hint);
        println!("   Try: {}", tui_command);
        if self.analyze_errors {
            println!("   --analyze-errors detected complex error patterns");
        }
    }
}

// Integration with interface selector for smart suggestions
impl InterfaceSelector {
    fn generate_logs_hint(&self, analysis: &LogComplexityAnalysis, target: &str) -> Option<String> {
        match analysis.analysis_benefit {
            AnalysisBenefit::Critical => Some(format!(
                "Critical error rate detected ({:.1}%). Interactive analysis strongly recommended",
                analysis.error_rate * 100.0
            )),
            AnalysisBenefit::High => Some(format!(
                "Complex error patterns detected ({} types). TUI provides better error correlation",
                analysis.unique_error_patterns
            )),
            AnalysisBenefit::Medium => Some(format!(
                "High log volume ({:.0}/sec). Interactive filtering and search available",
                analysis.log_volume_per_second
            )),
            AnalysisBenefit::Low => {
                if analysis.correlation_opportunities > 0 {
                    Some("Error correlation opportunities detected. Try 'dora ui logs' for analysis".to_string())
                } else {
                    None
                }
            }
        }
    }
}
```

### Why This Approach

**Docker-like Familiarity:**
- Consistent flag naming with Docker logs command
- Familiar streaming and follow behavior
- Expected filtering and formatting options

**Smart Complexity Detection:**
- Analyzes log patterns to determine when TUI is beneficial
- Considers error rates, patterns, and volume
- Provides contextual suggestions based on actual log content

**Performance Optimized:**
- Efficient streaming with bounded memory usage
- Smart buffering and filtering
- Minimal overhead for simple log viewing

### How to Implement

#### Step 1: Command Structure and Basic Streaming (3 hours)
1. **Define LogsCommand struct** with all CLI options
2. **Implement basic log streaming** from daemon API
3. **Add filtering and search** functionality
4. **Create log formatting** system

#### Step 2: Complexity Analysis (4 hours)
1. **Implement log complexity analyzer** with pattern detection
2. **Add error correlation** and burst detection
3. **Create analysis benefit** calculation
4. **Add smart suggestion** generation

#### Step 3: Error Tracking and Analysis (3 hours)
1. **Implement ErrorTracker** for real-time error monitoring
2. **Add pattern extraction** and classification
3. **Create error burst detection** system
4. **Add suggestion cooldown** and timing logic

#### Step 4: TUI Integration (2 hours)
1. **Add interactive log viewer** launching
2. **Implement context passing** to TUI
3. **Create user prompting** for complex scenarios
4. **Add hint generation** system

#### Step 5: Testing and Polish (2 hours)
1. **Add comprehensive unit tests** for all functionality
2. **Test with various log scenarios** (high error rate, high volume, etc.)
3. **Validate performance** with large log streams
4. **Test TUI integration** and suggestion accuracy

## 🔗 Dependencies
**Depends On:**
- Issue #001 (Hybrid Command Framework) - CLI structure
- Issue #002 (Execution Context Detection) - Terminal capabilities
- Issue #003 (Interface Selection Engine) - TUI suggestions
- Issue #009 (TUI Launcher Framework) - Interactive log viewer

**Blocks:** TUI log viewer implementation (future issue)

## 🧪 Testing Requirements

### Unit Tests
```rust
#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_log_complexity_analysis() {
        let logs = vec![
            create_error_log("Connection timeout", "node-1"),
            create_error_log("Connection timeout", "node-2"),
            create_error_log("Memory allocation failed", "node-1"),
            create_info_log("Normal operation", "node-3"),
        ];
        
        let analysis = LogComplexityAnalysis::from_logs(&logs);
        assert!(analysis.error_rate > 0.5); // 3/4 are errors
        assert_eq!(analysis.unique_error_patterns, 2);
        assert!(analysis.should_suggest_tui());
    }
    
    #[test]
    fn test_error_pattern_extraction() {
        let tracker = ErrorTracker::new();
        
        let pattern1 = tracker.extract_error_pattern("Connection timeout to 192.168.1.100:8080");
        let pattern2 = tracker.extract_error_pattern("Connection timeout to 10.0.0.50:8080");
        
        // Should extract same pattern for similar errors
        assert_eq!(pattern1, pattern2);
    }
    
    #[test]
    fn test_suggestion_cooldown() {
        let mut tracker = ErrorTracker::new();
        
        // Add many errors
        for i in 0..25 {
            tracker.add_error(&create_error_log(&format!("Error {}", i), "test-node"));
        }
        
        // First suggestion should trigger
        assert!(tracker.should_suggest_analysis());
        
        // Second suggestion should be blocked by cooldown
        assert!(!tracker.should_suggest_analysis());
    }
}
```

## ✅ Definition of Done
- [ ] LogsCommand implemented with all CLI options and filtering
- [ ] Log streaming works efficiently with real-time updates
- [ ] Complexity analysis accurately detects when TUI is beneficial
- [ ] Error tracking and pattern analysis provide meaningful insights
- [ ] Smart suggestions appear appropriately based on log content
- [ ] Interactive log viewer integration works seamlessly
- [ ] Memory usage stays bounded even with high-volume log streams
- [ ] Performance targets met for log streaming and analysis
- [ ] Comprehensive unit tests cover all functionality
- [ ] Integration tests validate end-to-end log viewing workflows
- [ ] Manual testing confirms user experience quality across scenarios

This enhanced logs command provides intelligent log viewing that scales from simple log tailing to sophisticated error analysis, demonstrating the power of the hybrid CLI approach.