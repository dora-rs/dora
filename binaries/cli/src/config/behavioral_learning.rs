use crate::cli::{Command, UiMode};
use crate::cli::context::ExecutionContext;
use crate::config::preferences::{
    UserPreferences, InterfaceChoice, ActionPattern, PatternType, ContextSnapshot,
    SatisfactionLevel, UsageStatistics, ContextFactor,
};
use chrono::{DateTime, Utc};
use std::collections::{HashMap, VecDeque};
use std::sync::{Arc, Mutex};
use std::time::Duration;

/// Trigger for interface decision
#[derive(Debug, Clone)]
pub enum DecisionTrigger {
    UserChoice,
    AutomaticSuggestion,
    ContextBasedSwitch,
    ComplexityThreshold,
}

/// Learning configuration
#[derive(Debug, Clone)]
pub struct LearningConfig {
    pub max_history_size: usize,
    pub pattern_analysis_window: usize,
    pub min_pattern_frequency: f32,
    pub learning_rate: f32,
    pub confidence_decay_rate: f32,
}

/// Main behavioral learning engine
#[derive(Debug)]
pub struct BehavioralLearningEngine {
    preferences: Arc<Mutex<UserPreferences>>,
    learning_config: LearningConfig,
    pattern_analyzer: PatternAnalyzer,
    session_id: String,
}

/// Pattern detection and analysis
#[derive(Debug)]
pub struct PatternAnalyzer {
    pattern_detectors: Vec<Box<dyn PatternDetector>>,
}

/// Trait for detecting different types of patterns
pub trait PatternDetector: Send + Sync {
    fn detect_patterns(&self, choices: &[InterfaceChoice]) -> HashMap<String, ActionPattern>;
    fn pattern_type(&self) -> PatternType;
}

/// Detects command sequence patterns
pub struct CommandSequenceDetector {
    min_sequence_length: usize,
    max_sequence_length: usize,
}

/// Detects time-based usage patterns
pub struct TimeBasedPatternDetector {
    time_window_hours: u8,
}

/// Detects context-based choice patterns
pub struct ContextPatternDetector;

impl BehavioralLearningEngine {
    pub fn new(preferences: Arc<Mutex<UserPreferences>>) -> Self {
        Self {
            preferences,
            learning_config: LearningConfig::default(),
            pattern_analyzer: PatternAnalyzer::new(),
            session_id: uuid::Uuid::new_v4().to_string(),
        }
    }
    
    pub fn with_config(preferences: Arc<Mutex<UserPreferences>>, config: LearningConfig) -> Self {
        Self {
            preferences,
            learning_config: config,
            pattern_analyzer: PatternAnalyzer::new(),
            session_id: uuid::Uuid::new_v4().to_string(),
        }
    }
    
    /// Record an interface choice made by the user
    pub async fn record_interface_choice(
        &self,
        command: &Command,
        context: &ExecutionContext,
        chosen_interface: UiMode,
        decision_trigger: DecisionTrigger,
    ) -> eyre::Result<()> {
        let interface_choice = InterfaceChoice {
            command: command.name().to_string(),
            context: ContextSnapshot::from_context(context),
            chosen_interface: chosen_interface.clone(),
            user_satisfaction: None, // Will be updated based on user feedback
            timestamp: Utc::now(),
            session_id: self.session_id.clone(),
        };
        
        let mut prefs = self.preferences.lock().unwrap();
        prefs.behavior.interface_choices.push_back(interface_choice);
        
        // Limit history size
        while prefs.behavior.interface_choices.len() > self.learning_config.max_history_size {
            prefs.behavior.interface_choices.pop_front();
        }
        
        // Update command usage statistics
        self.update_command_statistics(&mut prefs, command, &chosen_interface);
        
        // Analyze patterns and update preferences
        self.analyze_and_update_patterns(&mut prefs).await?;
        
        // Save preferences periodically
        if prefs.behavior.interface_choices.len() % 10 == 0 {
            drop(prefs); // Release lock before save
            self.save_preferences().await?;
        }
        
        Ok(())
    }
    
    /// Record user feedback about their satisfaction with the interface choice
    pub async fn record_user_feedback(
        &self,
        session_id: &str,
        satisfaction: SatisfactionLevel,
        feedback: Option<String>,
    ) -> eyre::Result<()> {
        let mut prefs = self.preferences.lock().unwrap();
        
        // Find recent interface choices for this session
        for choice in prefs.behavior.interface_choices.iter_mut().rev().take(5) {
            if choice.session_id == session_id && choice.user_satisfaction.is_none() {
                choice.user_satisfaction = Some(satisfaction.clone());
                break;
            }
        }
        
        // Update learning weights based on feedback
        self.adjust_learning_weights(&mut prefs, &satisfaction);
        
        // Log feedback if provided
        if let Some(feedback_text) = feedback {
            log::info!("User feedback for session {}: {} - {}", 
                      session_id, satisfaction.as_str(), feedback_text);
        }
        
        Ok(())
    }
    
    /// Get current session ID
    pub fn get_current_session_id(&self) -> String {
        self.session_id.clone()
    }
    
    /// Start a new session
    pub fn start_new_session(&mut self) {
        self.session_id = uuid::Uuid::new_v4().to_string();
    }
    
    /// Analyze patterns and update preferences
    async fn analyze_and_update_patterns(
        &self,
        prefs: &mut UserPreferences,
    ) -> eyre::Result<()> {
        // Get recent choices for analysis
        let recent_choices: Vec<_> = prefs.behavior.interface_choices.iter()
            .rev()
            .take(self.learning_config.pattern_analysis_window)
            .cloned()
            .collect();
        
        if recent_choices.len() < 3 {
            return Ok(()); // Not enough data for pattern analysis
        }
        
        // Analyze command patterns
        let command_patterns = self.pattern_analyzer.analyze_command_patterns(&recent_choices);
        
        // Analyze context patterns
        let context_patterns = self.pattern_analyzer.analyze_context_patterns(&recent_choices);
        
        // Analyze time patterns
        let time_patterns = self.pattern_analyzer.analyze_time_patterns(&recent_choices);
        
        // Update action patterns with decay
        for (pattern_key, pattern) in command_patterns {
            self.update_pattern_with_decay(&mut prefs.behavior.action_patterns, pattern_key, pattern);
        }
        
        for (pattern_key, pattern) in context_patterns {
            self.update_pattern_with_decay(&mut prefs.behavior.action_patterns, pattern_key, pattern);
        }
        
        for (pattern_key, pattern) in time_patterns {
            self.update_pattern_with_decay(&mut prefs.behavior.action_patterns, pattern_key, pattern);
        }
        
        // Clean old patterns
        self.clean_old_patterns(prefs);
        
        Ok(())
    }
    
    /// Update command usage statistics
    fn update_command_statistics(
        &self,
        prefs: &mut UserPreferences,
        command: &Command,
        chosen_interface: &UiMode,
    ) {
        let command_name = command.name().to_string();
        let stats = prefs.behavior.command_usage
            .entry(command_name)
            .or_insert_with(UsageStatistics::default);
        
        stats.total_uses += 1;
        *stats.interface_distribution.entry(chosen_interface.clone()).or_insert(0) += 1;
        stats.last_used = Utc::now();
        
        // Update success rate (simplified - assume 90% success for now)
        stats.success_rate = (stats.success_rate * (stats.total_uses - 1) as f32 + 0.9) / stats.total_uses as f32;
    }
    
    /// Update pattern with frequency decay
    fn update_pattern_with_decay(
        &self,
        patterns: &mut HashMap<String, ActionPattern>,
        pattern_key: String,
        new_pattern: ActionPattern,
    ) {
        if let Some(existing_pattern) = patterns.get_mut(&pattern_key) {
            // Apply frequency decay and update
            existing_pattern.frequency = existing_pattern.frequency * (1.0 - self.learning_config.confidence_decay_rate) + 
                                       new_pattern.frequency * self.learning_config.learning_rate;
            existing_pattern.last_seen = new_pattern.last_seen;
            
            // Update preferred interface if confidence is high enough
            if new_pattern.frequency > 0.7 {
                existing_pattern.preferred_interface = new_pattern.preferred_interface;
            }
        } else {
            patterns.insert(pattern_key, new_pattern);
        }
    }
    
    /// Adjust learning weights based on user satisfaction
    fn adjust_learning_weights(&self, prefs: &mut UserPreferences, satisfaction: &SatisfactionLevel) {
        let adjustment = match satisfaction {
            SatisfactionLevel::VeryUnsatisfied => -0.1,
            SatisfactionLevel::Unsatisfied => -0.05,
            SatisfactionLevel::Neutral => 0.0,
            SatisfactionLevel::Satisfied => 0.02,
            SatisfactionLevel::VerySatisfied => 0.05,
        };
        
        // Adjust behavioral weight based on satisfaction
        prefs.behavior.adaptation_weights.behavioral_weight = 
            (prefs.behavior.adaptation_weights.behavioral_weight + adjustment).clamp(0.1, 0.8);
        
        // Adjust complexity weight inversely for poor satisfaction
        if adjustment < 0.0 {
            prefs.behavior.adaptation_weights.complexity_weight = 
                (prefs.behavior.adaptation_weights.complexity_weight - adjustment * 0.5).clamp(0.1, 0.5);
        }
    }
    
    /// Clean old patterns that are no longer relevant
    fn clean_old_patterns(&self, prefs: &mut UserPreferences) {
        let cutoff_date = Utc::now() - chrono::Duration::days(30);
        
        prefs.behavior.action_patterns.retain(|_, pattern| {
            pattern.last_seen > cutoff_date && pattern.frequency > self.learning_config.min_pattern_frequency
        });
    }
    
    /// Save preferences to disk
    async fn save_preferences(&self) -> eyre::Result<()> {
        let prefs = self.preferences.lock().unwrap();
        prefs.save()
    }
}

impl PatternAnalyzer {
    pub fn new() -> Self {
        let pattern_detectors: Vec<Box<dyn PatternDetector>> = vec![
            Box::new(CommandSequenceDetector::new()),
            Box::new(TimeBasedPatternDetector::new()),
            Box::new(ContextPatternDetector::new()),
        ];
        
        Self { pattern_detectors }
    }
    
    pub fn analyze_command_patterns(&self, choices: &[InterfaceChoice]) -> HashMap<String, ActionPattern> {
        self.pattern_detectors.iter()
            .filter(|detector| matches!(detector.pattern_type(), PatternType::CommandSequence))
            .flat_map(|detector| detector.detect_patterns(choices))
            .collect()
    }
    
    pub fn analyze_context_patterns(&self, choices: &[InterfaceChoice]) -> HashMap<String, ActionPattern> {
        self.pattern_detectors.iter()
            .filter(|detector| matches!(detector.pattern_type(), PatternType::ContextBasedChoice))
            .flat_map(|detector| detector.detect_patterns(choices))
            .collect()
    }
    
    pub fn analyze_time_patterns(&self, choices: &[InterfaceChoice]) -> HashMap<String, ActionPattern> {
        self.pattern_detectors.iter()
            .filter(|detector| matches!(detector.pattern_type(), PatternType::TimeBasedUsage))
            .flat_map(|detector| detector.detect_patterns(choices))
            .collect()
    }
}

impl CommandSequenceDetector {
    pub fn new() -> Self {
        Self {
            min_sequence_length: 2,
            max_sequence_length: 4,
        }
    }
}

impl PatternDetector for CommandSequenceDetector {
    fn detect_patterns(&self, choices: &[InterfaceChoice]) -> HashMap<String, ActionPattern> {
        let mut patterns = HashMap::new();
        
        // Look for command sequences of different lengths
        for seq_len in self.min_sequence_length..=self.max_sequence_length.min(choices.len()) {
            for window in choices.windows(seq_len) {
                let sequence = window.iter()
                    .map(|c| c.command.clone())
                    .collect::<Vec<_>>()
                    .join(" -> ");
                
                // Check if this sequence has consistent interface choices
                let interfaces: Vec<_> = window.iter()
                    .map(|c| c.chosen_interface.clone())
                    .collect();
                
                // Calculate consistency score
                let most_common_interface = self.most_common_interface(&interfaces);
                let consistency = interfaces.iter()
                    .filter(|&i| i == &most_common_interface)
                    .count() as f32 / interfaces.len() as f32;
                
                if consistency >= 0.7 { // 70% consistency threshold
                    let pattern_key = format!("seq:{}", sequence);
                    let pattern = ActionPattern {
                        pattern_type: PatternType::CommandSequence,
                        frequency: consistency,
                        last_seen: window.last().unwrap().timestamp,
                        context_factors: vec![],
                        preferred_interface: Some(most_common_interface),
                    };
                    
                    patterns.insert(pattern_key, pattern);
                }
            }
        }
        
        patterns
    }
    
    fn pattern_type(&self) -> PatternType {
        PatternType::CommandSequence
    }
}

impl CommandSequenceDetector {
    fn most_common_interface(&self, interfaces: &[UiMode]) -> UiMode {
        let mut counts = HashMap::new();
        for interface in interfaces {
            *counts.entry(interface.clone()).or_insert(0) += 1;
        }
        
        counts.into_iter()
            .max_by_key(|(_, count)| *count)
            .map(|(interface, _)| interface)
            .unwrap_or(UiMode::Auto)
    }
}

impl TimeBasedPatternDetector {
    pub fn new() -> Self {
        Self {
            time_window_hours: 2,
        }
    }
}

impl PatternDetector for TimeBasedPatternDetector {
    fn detect_patterns(&self, choices: &[InterfaceChoice]) -> HashMap<String, ActionPattern> {
        let mut patterns = HashMap::new();
        
        // Group choices by time of day windows
        let mut time_groups: HashMap<u8, Vec<&InterfaceChoice>> = HashMap::new();
        
        for choice in choices {
            let time_window = choice.context.time_of_day / self.time_window_hours;
            time_groups.entry(time_window).or_insert_with(Vec::new).push(choice);
        }
        
        // Analyze patterns within each time window
        for (time_window, window_choices) in time_groups {
            if window_choices.len() < 3 {
                continue;
            }
            
            let interfaces: Vec<_> = window_choices.iter()
                .map(|c| c.chosen_interface.clone())
                .collect();
            
            let most_common = self.most_common_interface(&interfaces);
            let consistency = interfaces.iter()
                .filter(|&i| i == &most_common)
                .count() as f32 / interfaces.len() as f32;
            
            if consistency >= 0.6 { // 60% consistency for time patterns
                let pattern_key = format!("time:{}h-{}h", 
                                        time_window * self.time_window_hours,
                                        (time_window + 1) * self.time_window_hours);
                
                let pattern = ActionPattern {
                    pattern_type: PatternType::TimeBasedUsage,
                    frequency: consistency,
                    last_seen: window_choices.iter().map(|c| c.timestamp).max().unwrap(),
                    context_factors: vec![ContextFactor::TimeOfDay(time_window * self.time_window_hours)],
                    preferred_interface: Some(most_common),
                };
                
                patterns.insert(pattern_key, pattern);
            }
        }
        
        patterns
    }
    
    fn pattern_type(&self) -> PatternType {
        PatternType::TimeBasedUsage
    }
}

impl TimeBasedPatternDetector {
    fn most_common_interface(&self, interfaces: &[UiMode]) -> UiMode {
        let mut counts = HashMap::new();
        for interface in interfaces {
            *counts.entry(interface.clone()).or_insert(0) += 1;
        }
        
        counts.into_iter()
            .max_by_key(|(_, count)| *count)
            .map(|(interface, _)| interface)
            .unwrap_or(UiMode::Auto)
    }
}

impl ContextPatternDetector {
    pub fn new() -> Self {
        Self
    }
}

impl PatternDetector for ContextPatternDetector {
    fn detect_patterns(&self, choices: &[InterfaceChoice]) -> HashMap<String, ActionPattern> {
        let mut patterns = HashMap::new();
        
        // Group by context type
        let mut context_groups: HashMap<String, Vec<&InterfaceChoice>> = HashMap::new();
        
        for choice in choices {
            let context_key = format!("{}:tty{}", 
                                    choice.context.environment_type,
                                    choice.context.is_tty);
            context_groups.entry(context_key).or_insert_with(Vec::new).push(choice);
        }
        
        // Analyze patterns within each context
        for (context_key, context_choices) in context_groups {
            if context_choices.len() < 2 {
                continue;
            }
            
            let interfaces: Vec<_> = context_choices.iter()
                .map(|c| c.chosen_interface.clone())
                .collect();
            
            let most_common = self.most_common_interface(&interfaces);
            let consistency = interfaces.iter()
                .filter(|&i| i == &most_common)
                .count() as f32 / interfaces.len() as f32;
            
            if consistency >= 0.8 { // High consistency required for context patterns
                let pattern_key = format!("context:{}", context_key);
                
                let pattern = ActionPattern {
                    pattern_type: PatternType::ContextBasedChoice,
                    frequency: consistency,
                    last_seen: context_choices.iter().map(|c| c.timestamp).max().unwrap(),
                    context_factors: vec![ContextFactor::Environment(context_key.clone())],
                    preferred_interface: Some(most_common),
                };
                
                patterns.insert(pattern_key, pattern);
            }
        }
        
        patterns
    }
    
    fn pattern_type(&self) -> PatternType {
        PatternType::ContextBasedChoice
    }
}

impl ContextPatternDetector {
    fn most_common_interface(&self, interfaces: &[UiMode]) -> UiMode {
        let mut counts = HashMap::new();
        for interface in interfaces {
            *counts.entry(interface.clone()).or_insert(0) += 1;
        }
        
        counts.into_iter()
            .max_by_key(|(_, count)| *count)
            .map(|(interface, _)| interface)
            .unwrap_or(UiMode::Auto)
    }
}

impl Default for LearningConfig {
    fn default() -> Self {
        Self {
            max_history_size: 1000,
            pattern_analysis_window: 50,
            min_pattern_frequency: 0.3,
            learning_rate: 0.1,
            confidence_decay_rate: 0.05,
        }
    }
}

impl SatisfactionLevel {
    pub fn as_str(&self) -> &'static str {
        match self {
            SatisfactionLevel::VeryUnsatisfied => "very unsatisfied",
            SatisfactionLevel::Unsatisfied => "unsatisfied",
            SatisfactionLevel::Neutral => "neutral",
            SatisfactionLevel::Satisfied => "satisfied",
            SatisfactionLevel::VerySatisfied => "very satisfied",
        }
    }
    
    pub fn as_numeric(&self) -> u8 {
        match self {
            SatisfactionLevel::VeryUnsatisfied => 1,
            SatisfactionLevel::Unsatisfied => 2,
            SatisfactionLevel::Neutral => 3,
            SatisfactionLevel::Satisfied => 4,
            SatisfactionLevel::VerySatisfied => 5,
        }
    }
}