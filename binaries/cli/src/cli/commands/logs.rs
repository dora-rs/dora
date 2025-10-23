use super::{LogsCommand, LogLevel};
use crate::cli::context::ExecutionContext;
use crate::cli::interface::{InterfaceDecision, InterfaceStrategy};
use chrono::{DateTime, Utc};
use regex::Regex;
use std::collections::{HashMap, VecDeque};
use std::time::{Duration, Instant};
use tokio::time::interval;

/// Enhanced logs command implementation for Issue #7
pub struct EnhancedLogsCommand {
    context: ExecutionContext,
    command: LogsCommand,
    log_processor: LogProcessor,
    error_analyzer: ErrorAnalyzer,
    tui_suggester: TuiSuggester,
}

/// Log entry structure for processing
#[derive(Debug, Clone)]
pub struct LogEntry {
    pub timestamp: DateTime<Utc>,
    pub level: LogLevel,
    pub source: String,
    pub message: String,
    pub raw_line: String,
}

/// Log processing system with circular buffering
pub struct LogProcessor {
    buffer: VecDeque<LogEntry>,
    max_buffer_size: usize,
    filters: Vec<LogFilter>,
    start_time: Option<DateTime<Utc>>,
    end_time: Option<DateTime<Utc>>,
}

/// Error analysis and correlation engine
pub struct ErrorAnalyzer {
    error_patterns: Vec<ErrorPattern>,
    error_bursts: VecDeque<ErrorBurst>,
    correlation_window: Duration,
    burst_threshold: usize,
    pattern_cache: HashMap<String, usize>,
}

/// TUI suggestion engine based on log complexity
pub struct TuiSuggester {
    complexity_threshold: f64,
    error_rate_threshold: f64,
    volume_threshold: usize,
    suggestion_cooldown: Duration,
    last_suggestion: Option<Instant>,
}

/// Log filtering system
#[derive(Debug, Clone)]
pub enum LogFilter {
    Level(LogLevel),
    Regex(Regex),
    Search(String),
    ErrorsOnly,
    TimeRange { since: DateTime<Utc>, until: Option<DateTime<Utc>> },
}

/// Error pattern detection
#[derive(Debug, Clone)]
pub struct ErrorPattern {
    pub pattern: Regex,
    pub category: String,
    pub severity: u8,
    pub count: usize,
    pub last_seen: DateTime<Utc>,
}

/// Error burst detection
#[derive(Debug, Clone)]
pub struct ErrorBurst {
    pub start_time: DateTime<Utc>,
    pub end_time: Option<DateTime<Utc>>,
    pub error_count: usize,
    pub patterns: Vec<String>,
}

/// Log complexity analysis result
#[derive(Debug)]
pub struct LogComplexity {
    pub error_rate: f64,
    pub unique_patterns: usize,
    pub volume_per_minute: f64,
    pub burst_count: usize,
    pub complexity_score: f64,
}

/// TUI suggestion result
#[derive(Debug)]
pub struct TuiSuggestion {
    pub should_suggest: bool,
    pub reason: String,
    pub tui_command: String,
    pub benefits: Vec<String>,
}

impl EnhancedLogsCommand {
    /// Create new enhanced logs command
    pub fn new(context: ExecutionContext, command: LogsCommand) -> Self {
        Self {
            context,
            command,
            log_processor: LogProcessor::new(10000), // 10k entry buffer
            error_analyzer: ErrorAnalyzer::new(),
            tui_suggester: TuiSuggester::new(),
        }
    }

    /// Execute the enhanced logs command
    pub async fn execute(&mut self, interface_decision: &InterfaceDecision) -> Result<(), Box<dyn std::error::Error>> {
        // Setup log filters based on command options
        self.setup_filters()?;

        // Determine log streaming strategy
        let streaming_strategy = self.determine_streaming_strategy(interface_decision);

        match streaming_strategy {
            StreamingStrategy::CliWithAnalysis => {
                self.execute_cli_with_analysis().await
            },
            StreamingStrategy::CliWithSuggestions => {
                self.execute_cli_with_suggestions().await
            },
            StreamingStrategy::AutoLaunchTui => {
                self.execute_auto_tui().await
            },
            StreamingStrategy::SimpleStreaming => {
                self.execute_simple_streaming().await
            },
        }
    }

    /// Setup log filters based on command options
    fn setup_filters(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        let mut filters = Vec::new();

        // Level filter
        if let Some(level) = &self.command.level {
            filters.push(LogFilter::Level(level.clone()));
        }

        // Regex filter
        if let Some(pattern) = &self.command.filter {
            let regex = Regex::new(pattern)?;
            filters.push(LogFilter::Regex(regex));
        }

        // Search filter
        if let Some(search) = &self.command.search {
            filters.push(LogFilter::Search(search.clone()));
        }

        // Errors only filter
        if self.command.errors_only {
            filters.push(LogFilter::ErrorsOnly);
        }

        // Time range filter
        if let Some(since_str) = &self.command.since {
            let since = DateTime::parse_from_rfc3339(since_str)?.with_timezone(&Utc);
            let until = if let Some(until_str) = &self.command.until {
                Some(DateTime::parse_from_rfc3339(until_str)?.with_timezone(&Utc))
            } else {
                None
            };
            filters.push(LogFilter::TimeRange { since, until });
        }

        self.log_processor.set_filters(filters);
        Ok(())
    }

    /// Determine streaming strategy based on interface decision and log characteristics
    fn determine_streaming_strategy(&self, interface_decision: &InterfaceDecision) -> StreamingStrategy {
        match &interface_decision.strategy {
            InterfaceStrategy::CliOnly => StreamingStrategy::SimpleStreaming,
            InterfaceStrategy::CliWithHint { .. } => StreamingStrategy::CliWithSuggestions,
            InterfaceStrategy::PromptForTui { .. } => StreamingStrategy::CliWithAnalysis,
            InterfaceStrategy::AutoLaunchTui { .. } => StreamingStrategy::AutoLaunchTui,
        }
    }

    /// Execute CLI with real-time analysis and suggestions
    async fn execute_cli_with_analysis(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        println!("🔍 Enhanced logs with real-time analysis");
        
        // Start log streaming with analysis
        let mut log_stream = self.create_log_stream().await?;
        let mut analysis_interval = interval(Duration::from_secs(5));

        loop {
            tokio::select! {
                // Process incoming logs
                log_entry = log_stream.next() => {
                    if let Some(entry) = log_entry {
                        self.process_log_entry(entry).await?;
                    } else {
                        break; // Stream ended
                    }
                },
                
                // Periodic analysis and suggestions
                _ = analysis_interval.tick() => {
                    self.perform_analysis_cycle().await?;
                }
            }
        }

        Ok(())
    }

    /// Execute CLI with periodic TUI suggestions
    async fn execute_cli_with_suggestions(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        println!("📊 Enhanced logs with smart suggestions");
        
        // Implement CLI streaming with periodic TUI suggestions
        self.stream_logs_with_suggestions().await
    }

    /// Execute with auto-launch TUI
    async fn execute_auto_tui(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        println!("🖥️  Auto-launching TUI for complex log analysis");
        
        // Show brief CLI output first, then launch TUI
        self.show_brief_cli_output().await?;
        self.launch_tui_interface().await
    }

    /// Execute simple streaming (fallback)
    async fn execute_simple_streaming(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        println!("📝 Simple log streaming");
        
        // Basic log streaming without analysis
        self.stream_logs_basic().await
    }

    /// Perform analysis cycle for suggestions and insights
    async fn perform_analysis_cycle(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        // Analyze log complexity
        let complexity = self.analyze_log_complexity();
        
        // Check for TUI suggestion
        let suggestion = self.tui_suggester.should_suggest_tui(&complexity)?;
        
        if suggestion.should_suggest {
            self.display_tui_suggestion(&suggestion);
        }

        // Display error analysis if enabled
        if self.command.analyze_errors {
            self.display_error_analysis().await?;
        }

        Ok(())
    }

    /// Analyze current log complexity
    fn analyze_log_complexity(&self) -> LogComplexity {
        let buffer = &self.log_processor.buffer;
        let total_entries = buffer.len();
        
        if total_entries == 0 {
            return LogComplexity {
                error_rate: 0.0,
                unique_patterns: 0,
                volume_per_minute: 0.0,
                burst_count: 0,
                complexity_score: 0.0,
            };
        }

        // Calculate error rate
        let error_count = buffer.iter()
            .filter(|entry| matches!(entry.level, LogLevel::Error))
            .count();
        let error_rate = error_count as f64 / total_entries as f64;

        // Calculate unique patterns (simplified)
        let unique_patterns = buffer.iter()
            .map(|entry| &entry.message)
            .collect::<std::collections::HashSet<_>>()
            .len();

        // Calculate volume per minute
        let time_span = if let (Some(first), Some(last)) = (buffer.front(), buffer.back()) {
            last.timestamp.signed_duration_since(first.timestamp).num_minutes() as f64
        } else {
            1.0
        };
        let volume_per_minute = total_entries as f64 / time_span.max(1.0);

        // Get burst count from error analyzer
        let burst_count = self.error_analyzer.error_bursts.len();

        // Calculate overall complexity score
        let complexity_score = 
            error_rate * 0.4 +
            (unique_patterns as f64 / total_entries as f64) * 0.3 +
            (volume_per_minute / 100.0).min(1.0) * 0.2 +
            (burst_count as f64 / 10.0).min(1.0) * 0.1;

        LogComplexity {
            error_rate,
            unique_patterns,
            volume_per_minute,
            burst_count,
            complexity_score,
        }
    }

    /// Display TUI suggestion to user
    fn display_tui_suggestion(&self, suggestion: &TuiSuggestion) {
        if !self.context.show_suggestions() {
            return;
        }

        println!("\n💡 TUI Suggestion: {}", suggestion.reason);
        println!("   Try: {}", suggestion.tui_command.bright_cyan());
        
        if !suggestion.benefits.is_empty() {
            println!("   Benefits:");
            for benefit in &suggestion.benefits {
                println!("   • {}", benefit);
            }
        }
        println!();
    }

    /// Display error analysis results
    async fn display_error_analysis(&self) -> Result<(), Box<dyn std::error::Error>> {
        let patterns = &self.error_analyzer.error_patterns;
        let bursts = &self.error_analyzer.error_bursts;

        if !patterns.is_empty() || !bursts.is_empty() {
            println!("\n🔍 Error Analysis:");
            
            // Show top error patterns
            if !patterns.is_empty() {
                println!("  Top Error Patterns:");
                for (i, pattern) in patterns.iter().take(3).enumerate() {
                    println!("    {}. {} (count: {})", i + 1, pattern.category, pattern.count);
                }
            }

            // Show error bursts
            if !bursts.is_empty() {
                println!("  Recent Error Bursts: {}", bursts.len());
                if let Some(latest_burst) = bursts.back() {
                    println!("    Latest: {} errors starting at {}", 
                           latest_burst.error_count, 
                           latest_burst.start_time.format("%H:%M:%S"));
                }
            }
            println!();
        }

        Ok(())
    }

    /// Create log stream (placeholder - would connect to actual log sources)
    async fn create_log_stream(&self) -> Result<LogStream, Box<dyn std::error::Error>> {
        // This would connect to actual Dora log sources
        // For now, return a mock stream
        Ok(LogStream::new())
    }

    /// Process individual log entry
    async fn process_log_entry(&mut self, entry: LogEntry) -> Result<(), Box<dyn std::error::Error>> {
        // Add to processor buffer
        self.log_processor.add_entry(entry.clone());
        
        // Analyze for errors
        self.error_analyzer.analyze_entry(&entry);
        
        // Display entry if it passes filters
        if self.log_processor.passes_filters(&entry) {
            self.display_log_entry(&entry);
        }

        Ok(())
    }

    /// Display formatted log entry
    fn display_log_entry(&self, entry: &LogEntry) {
        if self.command.raw {
            println!("{}", entry.raw_line);
            return;
        }

        let timestamp = if self.command.timestamps {
            format!("{} ", entry.timestamp.format("%Y-%m-%d %H:%M:%S"))
        } else {
            String::new()
        };

        let level_color = match entry.level {
            LogLevel::Debug => "blue",
            LogLevel::Info => "green", 
            LogLevel::Warn => "yellow",
            LogLevel::Error => "red",
        };

        println!("{}[{:>5}] {}: {}", 
               timestamp,
               format!("{:?}", entry.level).color(level_color),
               entry.source,
               entry.message);
    }

    /// Stream logs with suggestions (placeholder)
    async fn stream_logs_with_suggestions(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        println!("Streaming logs with periodic suggestions...");
        // Implementation would go here
        Ok(())
    }

    /// Show brief CLI output before TUI
    async fn show_brief_cli_output(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        println!("Showing recent logs before launching TUI...");
        // Show last N entries, then launch TUI
        Ok(())
    }

    /// Launch TUI interface
    async fn launch_tui_interface(&self) -> Result<(), Box<dyn std::error::Error>> {
        println!("Launching TUI interface for interactive log viewing...");
        // Would launch actual TUI here
        Ok(())
    }

    /// Basic log streaming
    async fn stream_logs_basic(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        println!("Basic log streaming...");
        // Basic implementation
        Ok(())
    }
}

/// Streaming strategy enum
#[derive(Debug)]
enum StreamingStrategy {
    CliWithAnalysis,
    CliWithSuggestions,
    AutoLaunchTui,
    SimpleStreaming,
}

/// Mock log stream for demonstration
pub struct LogStream {
    // Mock implementation
}

impl LogStream {
    fn new() -> Self {
        Self {}
    }

    async fn next(&mut self) -> Option<LogEntry> {
        // Mock implementation - would read from actual log sources
        None
    }
}

impl LogProcessor {
    fn new(max_buffer_size: usize) -> Self {
        Self {
            buffer: VecDeque::new(),
            max_buffer_size,
            filters: Vec::new(),
            start_time: None,
            end_time: None,
        }
    }

    fn set_filters(&mut self, filters: Vec<LogFilter>) {
        self.filters = filters;
    }

    fn add_entry(&mut self, entry: LogEntry) {
        if self.buffer.len() >= self.max_buffer_size {
            self.buffer.pop_front();
        }
        self.buffer.push_back(entry);
    }

    fn passes_filters(&self, entry: &LogEntry) -> bool {
        for filter in &self.filters {
            if !self.apply_filter(filter, entry) {
                return false;
            }
        }
        true
    }

    fn apply_filter(&self, filter: &LogFilter, entry: &LogEntry) -> bool {
        match filter {
            LogFilter::Level(level) => matches!(
                (&entry.level, level),
                (LogLevel::Debug, LogLevel::Debug) |
                (LogLevel::Info, LogLevel::Debug | LogLevel::Info) |
                (LogLevel::Warn, LogLevel::Debug | LogLevel::Info | LogLevel::Warn) |
                (LogLevel::Error, _)
            ),
            LogFilter::Regex(regex) => regex.is_match(&entry.message),
            LogFilter::Search(search) => entry.message.contains(search),
            LogFilter::ErrorsOnly => matches!(entry.level, LogLevel::Error),
            LogFilter::TimeRange { since, until } => {
                entry.timestamp >= *since && 
                until.map_or(true, |until| entry.timestamp <= until)
            },
        }
    }
}

impl ErrorAnalyzer {
    fn new() -> Self {
        Self {
            error_patterns: Vec::new(),
            error_bursts: VecDeque::new(),
            correlation_window: Duration::from_secs(5 * 60),
            burst_threshold: 10,
            pattern_cache: HashMap::new(),
        }
    }

    fn analyze_entry(&mut self, entry: &LogEntry) {
        if matches!(entry.level, LogLevel::Error) {
            self.detect_patterns(entry);
            self.detect_bursts(entry);
        }
    }

    fn detect_patterns(&mut self, entry: &LogEntry) {
        // Simplified pattern detection
        let pattern_key = self.extract_pattern_key(&entry.message);
        *self.pattern_cache.entry(pattern_key).or_insert(0) += 1;
    }

    fn detect_bursts(&mut self, _entry: &LogEntry) {
        // Simplified burst detection
        // Implementation would track error frequency over time windows
    }

    fn extract_pattern_key(&self, message: &str) -> String {
        // Simplified pattern extraction
        // Would use more sophisticated pattern matching
        message.split_whitespace().take(3).collect::<Vec<_>>().join(" ")
    }
}

impl TuiSuggester {
    fn new() -> Self {
        Self {
            complexity_threshold: 0.6,
            error_rate_threshold: 0.1,
            volume_threshold: 100,
            suggestion_cooldown: Duration::from_secs(5 * 60),
            last_suggestion: None,
        }
    }

    fn should_suggest_tui(&mut self, complexity: &LogComplexity) -> Result<TuiSuggestion, Box<dyn std::error::Error>> {
        // Check cooldown
        if let Some(last) = self.last_suggestion {
            if last.elapsed() < self.suggestion_cooldown {
                return Ok(TuiSuggestion {
                    should_suggest: false,
                    reason: "Suggestion on cooldown".to_string(),
                    tui_command: String::new(),
                    benefits: Vec::new(),
                });
            }
        }

        let should_suggest = complexity.complexity_score > self.complexity_threshold ||
                           complexity.error_rate > self.error_rate_threshold ||
                           complexity.volume_per_minute > self.volume_threshold as f64;

        if should_suggest {
            self.last_suggestion = Some(Instant::now());
            
            let reason = if complexity.error_rate > self.error_rate_threshold {
                format!("High error rate detected ({:.1}%)", complexity.error_rate * 100.0)
            } else if complexity.volume_per_minute > self.volume_threshold as f64 {
                format!("High log volume ({:.0} logs/min)", complexity.volume_per_minute)
            } else {
                "Complex log patterns detected".to_string()
            };

            Ok(TuiSuggestion {
                should_suggest: true,
                reason,
                tui_command: "dora ui logs".to_string(),
                benefits: vec![
                    "Interactive filtering and search".to_string(),
                    "Real-time error correlation".to_string(),
                    "Visual pattern analysis".to_string(),
                ],
            })
        } else {
            Ok(TuiSuggestion {
                should_suggest: false,
                reason: "Log complexity is manageable in CLI".to_string(),
                tui_command: String::new(),
                benefits: Vec::new(),
            })
        }
    }
}

// Import colored trait for string coloring
use colored::Colorize;

#[cfg(test)]
mod tests {
    use super::*;
    use chrono::Utc;

    fn create_test_context() -> ExecutionContext {
        ExecutionContext::detect_basic()
    }

    fn create_test_logs_command() -> LogsCommand {
        LogsCommand {
            follow: false,
            tail: 100,
            level: Some(LogLevel::Info),
            errors_only: false,
            analyze_errors: true,
            timestamps: true,
            raw: false,
            since: None,
            until: None,
            filter: None,
            search: None,
            ..Default::default()
        }
    }

    fn create_test_log_entry(level: LogLevel, message: &str) -> LogEntry {
        LogEntry {
            timestamp: Utc::now(),
            level: level.clone(),
            source: "test-node".to_string(),
            message: message.to_string(),
            raw_line: format!("[{:?}] test-node: {}", level, message),
        }
    }

    #[test]
    fn test_log_processor_initialization() {
        let processor = LogProcessor::new(1000);
        assert_eq!(processor.buffer.len(), 0);
        assert_eq!(processor.max_buffer_size, 1000);
    }

    #[test]
    fn test_tui_suggester_initialization() {
        let suggester = TuiSuggester::new();
        assert_eq!(suggester.complexity_threshold, 0.6);
        assert_eq!(suggester.error_rate_threshold, 0.1);
        assert_eq!(suggester.volume_threshold, 100);
    }

    #[test]
    fn test_error_analyzer_initialization() {
        let analyzer = ErrorAnalyzer::new();
        assert_eq!(analyzer.error_patterns.len(), 0);
        assert_eq!(analyzer.error_bursts.len(), 0);
        assert_eq!(analyzer.burst_threshold, 10);
    }

    #[tokio::test]
    async fn test_enhanced_logs_command_creation() {
        let context = create_test_context();
        let command = create_test_logs_command();
        
        let enhanced_logs = EnhancedLogsCommand::new(context, command);
        
        // Test that the command is created successfully
        assert_eq!(enhanced_logs.log_processor.max_buffer_size, 10000);
        assert_eq!(enhanced_logs.error_analyzer.burst_threshold, 10);
        assert_eq!(enhanced_logs.tui_suggester.complexity_threshold, 0.6);
    }
}