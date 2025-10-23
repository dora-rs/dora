# Issue #028: Build Interactive Log Viewer

## 📋 Summary
Implement a comprehensive interactive log viewer that provides real-time log streaming, advanced filtering, pattern detection, and log analysis capabilities. This view serves as a critical tool for monitoring system behavior, debugging issues, and understanding dataflow execution patterns through intelligent log analysis.

## 🎯 Objectives
- Create real-time log streaming interface with intelligent buffering and display
- Implement advanced filtering and search capabilities for efficient log navigation
- Add pattern detection and anomaly highlighting for proactive issue identification
- Provide log correlation and context linking across dataflows and nodes
- Enable log export and sharing for collaboration and documentation

**Success Metrics:**
- Log streaming handles 10,000+ logs per second without UI lag
- Advanced filters reduce noise by 90%+ for relevant log identification
- Pattern detection identifies 95%+ of error patterns automatically
- Search and navigation complete within 100ms for any log buffer size
- User efficiency in log-based troubleshooting improves by 80%

## 🛠️ Technical Requirements

### What to Build

#### 1. Interactive Log Viewer Implementation
```rust
// src/tui/views/log_viewer.rs
use super::BaseView;
use crate::tui::{
    components::*,
    theme::Theme,
    AppContext, View, ViewEvent, ViewEventResult, ViewType
};
use async_trait::async_trait;
use ratatui::{
    layout::{Constraint, Direction, Layout, Rect},
    style::{Color, Modifier, Style},
    widgets::{Block, Borders, Clear, List, ListItem, Paragraph},
    Frame,
};
use std::collections::{HashMap, VecDeque};
use tokio::time::{Duration, Instant};

pub struct LogViewerView {
    base: BaseView,
    log_context: LogViewerContext,
    log_state: LogViewerState,
    log_stream: LogStream,
    filter_engine: LogFilterEngine,
    pattern_detector: LogPatternDetector,
    search_engine: LogSearchEngine,
    export_manager: LogExportManager,
    last_update: Instant,
}

#[derive(Debug, Clone)]
pub struct LogViewerContext {
    pub target: LogTarget,
    pub filters: LogFilters,
    pub stream_config: LogStreamConfig,
    pub view_mode: LogViewMode,
}

#[derive(Debug, Clone)]
pub enum LogTarget {
    System,
    Dataflow { name: String },
    Node { dataflow: String, node: String },
    Component { component_type: String },
    Process { pid: u32 },
    File { path: String },
}

#[derive(Debug)]
pub struct LogViewerState {
    pub log_buffer: LogBuffer,
    pub active_filters: ActiveFilters,
    pub search_state: SearchState,
    pub pattern_highlights: PatternHighlights,
    pub selection_state: SelectionState,
    pub streaming_stats: StreamingStats,
}

#[derive(Debug)]
pub struct LogBuffer {
    logs: VecDeque<LogEntry>,
    max_size: usize,
    total_logs_received: u64,
    filtered_logs_count: u64,
}

impl LogViewerView {
    pub fn new(target: LogTarget, filters: LogFilters) -> Self {
        let mut base = BaseView::new();
        
        let log_context = LogViewerContext {
            target: target.clone(),
            filters: filters.clone(),
            stream_config: LogStreamConfig::default(),
            view_mode: LogViewMode::RealTime,
        };
        
        // Configure layout for log viewer
        let layout = LayoutConfig::split_vertical(vec![
            LayoutSection {
                constraint: Constraint::Length(3),
                component_id: ComponentId("log_controls".to_string()),
            },
            LayoutSection {
                constraint: Constraint::Min(15),
                component_id: ComponentId("log_display".to_string()),
            },
            LayoutSection {
                constraint: Constraint::Length(8),
                component_id: ComponentId("log_details".to_string()),
            },
            LayoutSection {
                constraint: Constraint::Length(4),
                component_id: ComponentId("log_stats".to_string()),
            },
        ]);
        
        base.set_layout(layout);
        
        // Add components
        base.add_component(
            ComponentId("log_controls".to_string()),
            LogControlsComponent::new(log_context.clone()),
        );
        
        base.add_component(
            ComponentId("log_display".to_string()),
            LogDisplayComponent::new(log_context.clone()),
        );
        
        base.add_component(
            ComponentId("log_details".to_string()),
            LogDetailsComponent::new(log_context.clone()),
        );
        
        base.add_component(
            ComponentId("log_stats".to_string()),
            LogStatsComponent::new(log_context.clone()),
        );
        
        Self {
            base,
            log_context,
            log_state: LogViewerState::new(),
            log_stream: LogStream::new(),
            filter_engine: LogFilterEngine::new(),
            pattern_detector: LogPatternDetector::new(),
            search_engine: LogSearchEngine::new(),
            export_manager: LogExportManager::new(),
            last_update: Instant::now(),
        }
    }
    
    async fn start_log_streaming(&mut self) -> io::Result<()> {
        self.log_stream.start(&self.log_context.target, &self.log_context.stream_config).await?;
        
        // Initialize pattern detection
        self.pattern_detector.initialize(&self.log_context).await?;
        
        Ok(())
    }
    
    async fn process_new_logs(&mut self) -> io::Result<()> {
        let new_logs = self.log_stream.get_new_logs().await;
        
        for log_entry in new_logs {
            // Apply filters
            if self.filter_engine.should_include(&log_entry, &self.log_context.filters) {
                // Detect patterns
                let pattern_matches = self.pattern_detector.analyze_log(&log_entry).await;
                
                // Add to buffer with pattern highlights
                self.add_log_to_buffer(log_entry, pattern_matches);
            }
            
            self.log_state.streaming_stats.total_logs_processed += 1;
        }
        
        Ok(())
    }
    
    fn add_log_to_buffer(&mut self, log_entry: LogEntry, patterns: Vec<PatternMatch>) {
        // Add patterns to highlights
        for pattern in patterns {
            self.log_state.pattern_highlights.add_highlight(log_entry.id, pattern);
        }
        
        // Add to buffer
        self.log_state.log_buffer.add_log(log_entry);
        self.log_state.streaming_stats.displayed_logs_count += 1;
        
        // Auto-scroll if in real-time mode
        if self.log_context.view_mode == LogViewMode::RealTime {
            self.log_state.selection_state.auto_scroll_to_bottom();
        }
    }
    
    async fn search_logs(&mut self, query: &str) -> io::Result<Vec<SearchResult>> {
        let search_results = self.search_engine.search(
            &self.log_state.log_buffer,
            query,
            &SearchOptions::default()
        ).await?;
        
        self.log_state.search_state.update_results(search_results.clone());
        
        Ok(search_results)
    }
    
    async fn export_logs(&self, export_config: &LogExportConfig) -> io::Result<String> {
        let filtered_logs = self.log_state.log_buffer.get_filtered_logs(&self.log_context.filters);
        
        self.export_manager.export_logs(&filtered_logs, export_config).await
    }
}

#[async_trait]
impl View for LogViewerView {
    async fn initialize(&mut self, context: &AppContext) -> io::Result<()> {
        // Start log streaming
        self.start_log_streaming().await?;
        
        // Initialize all components
        for component in self.base.component_registry.components_mut() {
            component.update(context).await?;
        }
        
        Ok(())
    }
    
    async fn update(&mut self, context: &mut AppContext) -> io::Result<()> {
        // Process new logs
        self.process_new_logs().await?;
        
        // Update search if active
        if self.log_state.search_state.is_active() {
            self.search_engine.update_incremental_search(&self.log_state.log_buffer).await?;
        }
        
        // Update all components
        for component in self.base.component_registry.components_mut() {
            component.update(context).await?;
        }
        
        self.last_update = Instant::now();
        Ok(())
    }
    
    fn render(&self, frame: &mut Frame, area: Rect, theme: &Theme, context: &AppContext) {
        // Render main log viewer layout
        self.base.render_components(frame, area, theme, context);
        
        // Render overlays
        if self.log_state.search_state.is_active() {
            self.render_search_overlay(frame, area, theme);
        }
        
        if self.pattern_detector.has_active_alerts() {
            self.render_pattern_alerts(frame, area, theme);
        }
    }
    
    async fn handle_event(&mut self, event: ViewEvent, context: &mut AppContext) -> io::Result<ViewEventResult> {
        if let ViewEvent::Key(key_event) = &event {
            match key_event.code {
                crossterm::event::KeyCode::Char('/') => {
                    // Start search
                    self.log_state.search_state.start_search();
                    return Ok(ViewEventResult::None);
                },
                
                crossterm::event::KeyCode::Char('f') => {
                    // Open filter dialog
                    return Ok(ViewEventResult::PushView(ViewType::LogFilterDialog {
                        current_filters: self.log_context.filters.clone(),
                    }));
                },
                
                crossterm::event::KeyCode::Char('p') => {
                    // Pause/resume streaming
                    if self.log_stream.is_paused() {
                        self.log_stream.resume().await?;
                        self.log_context.view_mode = LogViewMode::RealTime;
                    } else {
                        self.log_stream.pause().await?;
                        self.log_context.view_mode = LogViewMode::Paused;
                    }
                    return Ok(ViewEventResult::None);
                },
                
                crossterm::event::KeyCode::Char('c') => {
                    // Clear log buffer
                    self.log_state.log_buffer.clear();
                    self.log_state.pattern_highlights.clear();
                    return Ok(ViewEventResult::None);
                },
                
                crossterm::event::KeyCode::Char('e') => {
                    // Export logs
                    return Ok(ViewEventResult::PushView(ViewType::LogExportDialog {
                        log_count: self.log_state.log_buffer.len(),
                        current_filters: self.log_context.filters.clone(),
                    }));
                },
                
                crossterm::event::KeyCode::Char('n') => {
                    // Navigate to next search result
                    if let Some(next_result) = self.log_state.search_state.next_result() {
                        self.log_state.selection_state.jump_to_log(next_result.log_id);
                    }
                    return Ok(ViewEventResult::None);
                },
                
                crossterm::event::KeyCode::Char('N') => {
                    // Navigate to previous search result
                    if let Some(prev_result) = self.log_state.search_state.previous_result() {
                        self.log_state.selection_state.jump_to_log(prev_result.log_id);
                    }
                    return Ok(ViewEventResult::None);
                },
                
                crossterm::event::KeyCode::Up => {
                    self.log_state.selection_state.move_up();
                    self.log_context.view_mode = LogViewMode::Paused;
                    return Ok(ViewEventResult::None);
                },
                
                crossterm::event::KeyCode::Down => {
                    self.log_state.selection_state.move_down();
                    return Ok(ViewEventResult::None);
                },
                
                crossterm::event::KeyCode::PageUp => {
                    self.log_state.selection_state.page_up();
                    self.log_context.view_mode = LogViewMode::Paused;
                    return Ok(ViewEventResult::None);
                },
                
                crossterm::event::KeyCode::PageDown => {
                    self.log_state.selection_state.page_down();
                    return Ok(ViewEventResult::None);
                },
                
                crossterm::event::KeyCode::Home => {
                    self.log_state.selection_state.jump_to_start();
                    self.log_context.view_mode = LogViewMode::Paused;
                    return Ok(ViewEventResult::None);
                },
                
                crossterm::event::KeyCode::End => {
                    self.log_state.selection_state.jump_to_end();
                    self.log_context.view_mode = LogViewMode::RealTime;
                    return Ok(ViewEventResult::None);
                },
                
                _ => {}
            }
        }
        
        // Forward event to components
        self.base.handle_component_event(event, context).await
    }
    
    async fn handle_resize(&mut self, width: u16, height: u16) -> io::Result<()> {
        self.base.layout_manager.handle_resize(width, height);
        
        // Adjust log buffer visible size
        self.log_state.log_buffer.adjust_visible_size(height as usize);
        
        Ok(())
    }
    
    async fn cleanup(&self, context: &AppContext) -> io::Result<()> {
        // Stop log streaming
        self.log_stream.stop().await;
        
        Ok(())
    }
    
    fn view_type(&self) -> ViewType {
        ViewType::LogViewer {
            target: self.log_context.target.clone(),
            filters: self.log_context.filters.clone(),
        }
    }
    
    fn title(&self) -> String {
        format!("Log Viewer: {}", self.log_context.target.display_name())
    }
    
    fn help_text(&self) -> Vec<HelpItem> {
        vec![
            HelpItem::new("/", "Search logs"),
            HelpItem::new("f", "Filter logs"),
            HelpItem::new("p", "Pause/Resume"),
            HelpItem::new("c", "Clear buffer"),
            HelpItem::new("e", "Export logs"),
            HelpItem::new("n/N", "Next/Prev search"),
            HelpItem::new("↑↓", "Navigate logs"),
            HelpItem::new("PgUp/PgDn", "Page up/down"),
            HelpItem::new("Home/End", "Start/End"),
            HelpItem::new("Esc", "Back"),
        ]
    }
}
```

#### 2. Log Display Component
```rust
// src/tui/components/log/log_display.rs
pub struct LogDisplayComponent {
    log_context: LogViewerContext,
    focused: bool,
    scroll_state: ScrollState,
    highlight_patterns: Vec<HighlightPattern>,
    last_update: Instant,
}

impl LogDisplayComponent {
    pub fn new(log_context: LogViewerContext) -> Self {
        Self {
            log_context,
            focused: true, // Usually focused by default
            scroll_state: ScrollState::new(),
            highlight_patterns: Self::create_default_highlights(),
            last_update: Instant::now(),
        }
    }
    
    fn create_default_highlights() -> Vec<HighlightPattern> {
        vec![
            HighlightPattern {
                pattern: regex::Regex::new(r"ERROR|FATAL").unwrap(),
                style: Style::default().fg(Color::Red).add_modifier(Modifier::BOLD),
            },
            HighlightPattern {
                pattern: regex::Regex::new(r"WARN|WARNING").unwrap(),
                style: Style::default().fg(Color::Yellow),
            },
            HighlightPattern {
                pattern: regex::Regex::new(r"INFO").unwrap(),
                style: Style::default().fg(Color::Blue),
            },
            HighlightPattern {
                pattern: regex::Regex::new(r"DEBUG|TRACE").unwrap(),
                style: Style::default().fg(Color::DarkGray),
            },
        ]
    }
    
    fn render_log_entry(&self, log_entry: &LogEntry, selected: bool, theme: &Theme) -> ListItem {
        let timestamp = log_entry.timestamp.format("%H:%M:%S%.3f").to_string();
        let level = format!("{:5}", log_entry.level);
        let source = match &log_entry.source {
            LogSource::Node { dataflow, node } => format!("{}::{}", dataflow, node),
            LogSource::Dataflow { name } => name.clone(),
            LogSource::System => "SYSTEM".to_string(),
            LogSource::Component { name } => name.clone(),
        };
        
        let mut content = format!(
            "{} {} [{}] {}",
            timestamp,
            level,
            source,
            log_entry.message
        );
        
        // Apply syntax highlighting
        let mut style = if selected {
            theme.selected_item_style()
        } else {
            self.get_level_style(&log_entry.level, theme)
        };
        
        // Add pattern highlights
        for highlight in &self.highlight_patterns {
            if highlight.pattern.is_match(&content) {
                style = highlight.style;
                break;
            }
        }
        
        // Add context information if available
        if let Some(context) = &log_entry.context {
            content.push_str(&format!(" | {}", context));
        }
        
        ListItem::new(content).style(style)
    }
    
    fn get_level_style(&self, level: &LogLevel, theme: &Theme) -> Style {
        match level {
            LogLevel::Error | LogLevel::Fatal => theme.error_style(),
            LogLevel::Warn => theme.warning_style(),
            LogLevel::Info => theme.info_style(),
            LogLevel::Debug | LogLevel::Trace => theme.muted_style(),
        }
    }
}

#[async_trait]
impl Component for LogDisplayComponent {
    async fn update(&mut self, context: &AppContext) -> io::Result<()> {
        // Update scroll state based on buffer changes
        if let Some(log_viewer_state) = context.get_component_state::<LogViewerState>("log_viewer") {
            self.scroll_state.update_from_selection(&log_viewer_state.selection_state);
        }
        
        Ok(())
    }
    
    fn render(&self, frame: &mut Frame, area: Rect, theme: &Theme, context: &AppContext) {
        let block = Block::default()
            .title("Logs")
            .borders(Borders::ALL)
            .border_style(if self.focused {
                theme.focused_border_style()
            } else {
                theme.normal_border_style()
            });
        
        let inner_area = block.inner(area);
        frame.render_widget(block, area);
        
        // Get log buffer from context
        if let Some(log_viewer_state) = context.get_component_state::<LogViewerState>("log_viewer") {
            let visible_logs = log_viewer_state.log_buffer.get_visible_logs();
            
            let items: Vec<ListItem> = visible_logs.iter()
                .enumerate()
                .map(|(i, log_entry)| {
                    let selected = i == log_viewer_state.selection_state.selected_index;
                    self.render_log_entry(log_entry, selected, theme)
                })
                .collect();
            
            let list = List::new(items)
                .highlight_style(theme.highlight_style())
                .highlight_symbol("▶ ");
            
            frame.render_stateful_widget(
                list,
                inner_area,
                &mut self.scroll_state.to_list_state()
            );
            
            // Render scroll indicator
            self.render_scroll_indicator(frame, area, theme, &log_viewer_state.log_buffer);
        }
    }
    
    async fn handle_event(&mut self, event: ComponentEvent, context: &AppContext) -> io::Result<ViewEventResult> {
        if !self.focused {
            return Ok(ViewEventResult::None);
        }
        
        match event {
            ComponentEvent::Key(key_event) => {
                match key_event.code {
                    crossterm::event::KeyCode::Enter => {
                        // Show detailed view of selected log
                        if let Some(log_viewer_state) = context.get_component_state::<LogViewerState>("log_viewer") {
                            if let Some(selected_log) = log_viewer_state.log_buffer.get_selected_log() {
                                return Ok(ViewEventResult::PushView(ViewType::LogDetailView {
                                    log_entry: selected_log.clone(),
                                }));
                            }
                        }
                    },
                    
                    crossterm::event::KeyCode::Char('j') => {
                        // Follow mode - jump to related logs
                        if let Some(log_viewer_state) = context.get_component_state::<LogViewerState>("log_viewer") {
                            if let Some(selected_log) = log_viewer_state.log_buffer.get_selected_log() {
                                return Ok(ViewEventResult::UpdateContext(ContextUpdate::FollowLogs {
                                    correlation_id: selected_log.correlation_id.clone(),
                                }));
                            }
                        }
                    },
                    
                    _ => {}
                }
            },
            _ => {}
        }
        
        Ok(ViewEventResult::None)
    }
    
    fn component_type(&self) -> ComponentType {
        ComponentType::LogDisplay
    }
    
    fn is_focusable(&self) -> bool {
        true
    }
    
    fn is_focused(&self) -> bool {
        self.focused
    }
    
    fn set_focus(&mut self, focused: bool) {
        self.focused = focused;
    }
}
```

#### 3. Log Pattern Detection Engine
```rust
// src/tui/components/log/pattern_detector.rs
#[derive(Debug)]
pub struct LogPatternDetector {
    error_patterns: Vec<ErrorPattern>,
    anomaly_detectors: Vec<AnomalyDetector>,
    correlation_engine: LogCorrelationEngine,
    alert_thresholds: AlertThresholds,
    pattern_cache: LruCache<String, Vec<PatternMatch>>,
}

#[derive(Debug, Clone)]
pub struct ErrorPattern {
    pub name: String,
    pub pattern: regex::Regex,
    pub severity: PatternSeverity,
    pub description: String,
    pub suggested_actions: Vec<String>,
}

#[derive(Debug, Clone)]
pub enum PatternSeverity {
    Critical,
    High,
    Medium,
    Low,
    Info,
}

impl LogPatternDetector {
    pub fn new() -> Self {
        Self {
            error_patterns: Self::create_default_patterns(),
            anomaly_detectors: Self::create_anomaly_detectors(),
            correlation_engine: LogCorrelationEngine::new(),
            alert_thresholds: AlertThresholds::default(),
            pattern_cache: LruCache::new(1000),
        }
    }
    
    fn create_default_patterns() -> Vec<ErrorPattern> {
        vec![
            ErrorPattern {
                name: "Out of Memory".to_string(),
                pattern: regex::Regex::new(r"OutOfMemoryError|OOM|out of memory").unwrap(),
                severity: PatternSeverity::Critical,
                description: "Application is running out of memory".to_string(),
                suggested_actions: vec![
                    "Check memory usage and allocation patterns".to_string(),
                    "Consider increasing memory limits".to_string(),
                    "Look for memory leaks in the application".to_string(),
                ],
            },
            ErrorPattern {
                name: "Connection Timeout".to_string(),
                pattern: regex::Regex::new(r"connection.*timeout|timeout.*connection").unwrap(),
                severity: PatternSeverity::High,
                description: "Network connection is timing out".to_string(),
                suggested_actions: vec![
                    "Check network connectivity".to_string(),
                    "Verify service availability".to_string(),
                    "Consider increasing timeout values".to_string(),
                ],
            },
            ErrorPattern {
                name: "File Not Found".to_string(),
                pattern: regex::Regex::new(r"file not found|no such file|FileNotFoundError").unwrap(),
                severity: PatternSeverity::Medium,
                description: "Required file or resource is missing".to_string(),
                suggested_actions: vec![
                    "Verify file paths and permissions".to_string(),
                    "Check if file was moved or deleted".to_string(),
                    "Ensure proper file deployment".to_string(),
                ],
            },
            ErrorPattern {
                name: "Performance Degradation".to_string(),
                pattern: regex::Regex::new(r"slow|performance|latency.*high").unwrap(),
                severity: PatternSeverity::Medium,
                description: "System performance is degrading".to_string(),
                suggested_actions: vec![
                    "Monitor system resources".to_string(),
                    "Check for resource bottlenecks".to_string(),
                    "Review recent changes".to_string(),
                ],
            },
        ]
    }
    
    pub async fn analyze_log(&mut self, log_entry: &LogEntry) -> Vec<PatternMatch> {
        let mut matches = Vec::new();
        
        // Check cache first
        if let Some(cached_matches) = self.pattern_cache.get(&log_entry.message) {
            return cached_matches.clone();
        }
        
        // Check error patterns
        for pattern in &self.error_patterns {
            if pattern.pattern.is_match(&log_entry.message) {
                matches.push(PatternMatch {
                    pattern_name: pattern.name.clone(),
                    severity: pattern.severity.clone(),
                    confidence: self.calculate_confidence(&pattern, log_entry),
                    suggested_actions: pattern.suggested_actions.clone(),
                    context: self.extract_pattern_context(&pattern.pattern, &log_entry.message),
                });
            }
        }
        
        // Check for anomalies
        for detector in &mut self.anomaly_detectors {
            if let Some(anomaly) = detector.detect_anomaly(log_entry).await {
                matches.push(PatternMatch {
                    pattern_name: format!("Anomaly: {}", anomaly.anomaly_type),
                    severity: PatternSeverity::Medium,
                    confidence: anomaly.confidence,
                    suggested_actions: anomaly.suggested_actions,
                    context: anomaly.context,
                });
            }
        }
        
        // Check for correlations
        if let Some(correlations) = self.correlation_engine.find_correlations(log_entry).await {
            for correlation in correlations {
                matches.push(PatternMatch {
                    pattern_name: format!("Correlation: {}", correlation.correlation_type),
                    severity: PatternSeverity::Info,
                    confidence: correlation.strength,
                    suggested_actions: vec!["Investigate related log entries".to_string()],
                    context: correlation.description,
                });
            }
        }
        
        // Cache results
        self.pattern_cache.put(log_entry.message.clone(), matches.clone());
        
        matches
    }
    
    fn calculate_confidence(&self, pattern: &ErrorPattern, log_entry: &LogEntry) -> f64 {
        let mut confidence = 0.5; // Base confidence
        
        // Increase confidence based on log level
        match log_entry.level {
            LogLevel::Error | LogLevel::Fatal => confidence += 0.3,
            LogLevel::Warn => confidence += 0.2,
            LogLevel::Info => confidence += 0.1,
            _ => {}
        }
        
        // Increase confidence for exact matches
        if pattern.pattern.find(&log_entry.message).unwrap().len() > 10 {
            confidence += 0.2;
        }
        
        confidence.min(1.0)
    }
    
    fn extract_pattern_context(&self, pattern: &regex::Regex, message: &str) -> String {
        if let Some(capture) = pattern.find(message) {
            let start = capture.start().saturating_sub(20);
            let end = (capture.end() + 20).min(message.len());
            message[start..end].to_string()
        } else {
            String::new()
        }
    }
}

#[derive(Debug)]
pub struct LogCorrelationEngine {
    correlation_window: Duration,
    correlation_patterns: HashMap<String, CorrelationPattern>,
    recent_logs: VecDeque<LogEntry>,
}

impl LogCorrelationEngine {
    pub fn new() -> Self {
        Self {
            correlation_window: Duration::from_secs(60),
            correlation_patterns: Self::create_correlation_patterns(),
            recent_logs: VecDeque::new(),
        }
    }
    
    fn create_correlation_patterns() -> HashMap<String, CorrelationPattern> {
        let mut patterns = HashMap::new();
        
        patterns.insert("error_cascade".to_string(), CorrelationPattern {
            name: "Error Cascade".to_string(),
            description: "Multiple related errors occurring in sequence".to_string(),
            time_window: Duration::from_secs(30),
            log_level_pattern: vec![LogLevel::Error, LogLevel::Error],
            source_pattern: SourcePattern::SameDataflow,
        });
        
        patterns.insert("performance_degradation".to_string(), CorrelationPattern {
            name: "Performance Degradation".to_string(),
            description: "Performance warnings followed by errors".to_string(),
            time_window: Duration::from_secs(120),
            log_level_pattern: vec![LogLevel::Warn, LogLevel::Error],
            source_pattern: SourcePattern::Related,
        });
        
        patterns
    }
    
    pub async fn find_correlations(&mut self, log_entry: &LogEntry) -> Option<Vec<LogCorrelation>> {
        // Add to recent logs
        self.recent_logs.push_back(log_entry.clone());
        
        // Remove old logs outside correlation window
        let cutoff_time = log_entry.timestamp - self.correlation_window;
        while let Some(front) = self.recent_logs.front() {
            if front.timestamp < cutoff_time {
                self.recent_logs.pop_front();
            } else {
                break;
            }
        }
        
        // Find correlations
        let mut correlations = Vec::new();
        
        for pattern in self.correlation_patterns.values() {
            if let Some(correlation) = self.check_correlation_pattern(pattern, log_entry) {
                correlations.push(correlation);
            }
        }
        
        if correlations.is_empty() {
            None
        } else {
            Some(correlations)
        }
    }
    
    fn check_correlation_pattern(&self, pattern: &CorrelationPattern, log_entry: &LogEntry) -> Option<LogCorrelation> {
        // Implementation for specific correlation pattern matching
        // This would check if the current log entry and recent logs match the pattern
        
        // For example, checking error cascade pattern
        if pattern.name == "Error Cascade" && log_entry.level == LogLevel::Error {
            let recent_errors: Vec<&LogEntry> = self.recent_logs.iter()
                .filter(|log| log.level == LogLevel::Error && log.source.is_related_to(&log_entry.source))
                .collect();
            
            if recent_errors.len() >= 2 {
                return Some(LogCorrelation {
                    correlation_type: pattern.name.clone(),
                    description: format!("Detected {} related errors in the last {} seconds", 
                                       recent_errors.len(), pattern.time_window.as_secs()),
                    strength: (recent_errors.len() as f64 / 5.0).min(1.0),
                    related_logs: recent_errors.iter().map(|log| log.id.clone()).collect(),
                });
            }
        }
        
        None
    }
}
```

### Why This Approach

**Real-Time Intelligence:**
- Advanced pattern detection identifies issues proactively
- Real-time streaming with intelligent buffering
- Correlation analysis connects related log events

**Advanced Navigation:**
- Powerful search and filtering capabilities
- Pattern-based highlighting and navigation
- Context-aware log correlation and linking

**Developer-Focused Interface:**
- Efficient keyboard navigation and shortcuts
- Export capabilities for collaboration
- Intelligent auto-scrolling and positioning

### How to Implement

#### Step 1: Log Viewer Core (5 hours)
1. **Implement LogViewerView** with streaming and buffer management
2. **Add real-time log** processing and display
3. **Create navigation** and selection handling
4. **Add view mode** management (real-time vs paused)

#### Step 2: Log Display and Controls (4 hours)
1. **Implement LogDisplayComponent** with syntax highlighting
2. **Add LogControlsComponent** for stream control and configuration
3. **Create LogDetailsComponent** for selected log examination
4. **Add LogStatsComponent** for streaming statistics

#### Step 3: Pattern Detection Engine (5 hours)
1. **Implement LogPatternDetector** with error pattern recognition
2. **Add anomaly detection** for unusual log patterns
3. **Create correlation engine** for related log analysis
4. **Add alerting system** for critical patterns

#### Step 4: Search and Filter Engine (3 hours)
1. **Implement LogSearchEngine** with fast text search
2. **Add advanced filtering** with multiple criteria
3. **Create search result** navigation and highlighting
4. **Add filter persistence** and management

#### Step 5: Export and Integration (2 hours)
1. **Implement LogExportManager** with multiple formats
2. **Add log sharing** and collaboration features
3. **Create integration** with external log analysis tools
4. **Add comprehensive testing** and validation

## 🔗 Dependencies
**Depends On:**
- Issue #023 (TUI Architecture Foundation) - Base view and component system
- Issue #024 (Dashboard Overview) - Navigation integration
- Phase 2 enhanced commands for log analysis integration

**Enables:**
- Advanced log-based debugging and monitoring
- Integration with debug sessions and performance analysis

## 🧪 Testing Requirements

### Unit Tests
```rust
#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_log_buffer_management() {
        let mut buffer = LogBuffer::new(1000);
        
        for i in 0..1500 {
            buffer.add_log(LogEntry::test_entry(i));
        }
        
        assert_eq!(buffer.len(), 1000);
        assert_eq!(buffer.total_logs_received, 1500);
    }
    
    #[test]
    fn test_pattern_detection() {
        let mut detector = LogPatternDetector::new();
        let log_entry = LogEntry {
            message: "OutOfMemoryError: Java heap space".to_string(),
            level: LogLevel::Error,
            ..LogEntry::default()
        };
        
        let matches = detector.analyze_log(&log_entry).await.unwrap();
        
        assert!(!matches.is_empty());
        assert_eq!(matches[0].pattern_name, "Out of Memory");
        assert_eq!(matches[0].severity, PatternSeverity::Critical);
    }
    
    #[test]
    fn test_log_search_performance() {
        let search_engine = LogSearchEngine::new();
        let large_buffer = LogBuffer::with_test_data(10000);
        
        let start_time = Instant::now();
        let results = search_engine.search(&large_buffer, "error", &SearchOptions::default()).await.unwrap();
        let search_time = start_time.elapsed();
        
        assert!(search_time < Duration::from_millis(100));
        assert!(!results.is_empty());
    }
}
```

## ✅ Definition of Done
- [ ] LogViewerView provides real-time log streaming with configurable targets
- [ ] Log display component renders logs efficiently with syntax highlighting
- [ ] Pattern detection engine identifies error patterns and anomalies automatically
- [ ] Search engine provides fast text search across large log buffers
- [ ] Filter engine enables advanced log filtering with multiple criteria
- [ ] Correlation engine detects related log events and provides context
- [ ] Navigation system enables efficient log browsing and selection
- [ ] Export functionality supports multiple formats for log sharing
- [ ] Performance targets met for streaming and search operations
- [ ] Memory usage stays within limits during extended log streaming
- [ ] Comprehensive unit tests validate all log viewer functionality
- [ ] Integration tests confirm log streaming and pattern detection accuracy
- [ ] Manual testing validates developer workflow effectiveness for log analysis

This interactive log viewer provides developers with powerful real-time log analysis capabilities, enabling efficient debugging and system monitoring through intelligent pattern detection and advanced navigation features.