use crate::cli::{Command, UiMode};
use crate::cli::context::ExecutionContext;
use crate::cli::interface::{InterfaceDecision, InterfaceStrategy, UserConfig};
use crate::analysis::{
    ComplexityAnalysisEngine, ComplexityAnalysisRequest, SystemState, UserExpertiseLevel,
    RecommendationPriority, ComplexityResult,
};
use std::sync::Arc;
use std::collections::HashMap;
use tokio::sync::Mutex;

/// Enhanced interface selector using ML-based complexity analysis
#[derive(Debug)]
pub struct EnhancedInterfaceSelector {
    complexity_engine: Arc<Mutex<ComplexityAnalysisEngine>>,
    config: UserConfig,
    system_monitor: SystemMonitor,
}

/// System monitoring for real-time state
#[derive(Debug)]
pub struct SystemMonitor {
    last_system_state: Option<SystemState>,
    update_interval: std::time::Duration,
}

/// Enhanced decision with complexity insights
#[derive(Debug, Clone)]
pub struct EnhancedInterfaceDecision {
    pub base_decision: InterfaceDecision,
    pub complexity_score: f32,
    pub complexity_explanation: String,
    pub learning_feedback: Option<LearningFeedback>,
}

/// Feedback for continuous learning
#[derive(Debug, Clone)]
pub struct LearningFeedback {
    pub should_collect_feedback: bool,
    pub feedback_prompt: String,
    pub session_id: String,
}

impl EnhancedInterfaceSelector {
    /// Create new enhanced interface selector
    pub fn new(config: UserConfig) -> Self {
        Self {
            complexity_engine: Arc::new(Mutex::new(ComplexityAnalysisEngine::new())),
            config,
            system_monitor: SystemMonitor::new(),
        }
    }

    /// Select interface using advanced complexity analysis
    pub async fn select_interface(
        &mut self,
        command: &Command,
        context: &ExecutionContext,
    ) -> eyre::Result<EnhancedInterfaceDecision> {
        // Get current system state
        let system_state = self.system_monitor.get_current_state().await;
        
        // Determine user expertise level from config
        let user_expertise = self.determine_user_expertise(command, context);
        
        // Create complexity analysis request
        let analysis_request = ComplexityAnalysisRequest {
            command: command.clone(),
            context: context.clone(),
            target_resource: None, // Could be enhanced to detect target resources
            system_state,
            user_expertise,
        };
        
        // Analyze complexity
        let complexity_result = {
            let mut engine = self.complexity_engine.lock().await;
            engine.analyze_complexity(&analysis_request).await?
        };
        
        // Make interface decision based on complexity analysis
        let base_decision = self.make_complexity_based_decision(
            command,
            context,
            &complexity_result,
        );
        
        // Generate learning feedback if appropriate
        let learning_feedback = self.generate_learning_feedback(
            &complexity_result,
            &base_decision,
        );
        
        Ok(EnhancedInterfaceDecision {
            base_decision,
            complexity_score: complexity_result.overall_score,
            complexity_explanation: complexity_result.explanation.summary,
            learning_feedback,
        })
    }
    
    /// Make decision based on complexity analysis results
    fn make_complexity_based_decision(
        &self,
        command: &Command,
        context: &ExecutionContext,
        complexity_result: &crate::analysis::ComplexityResult,
    ) -> InterfaceDecision {
        // Check for forced contexts first
        if let Some(decision) = self.check_forced_contexts(context) {
            return decision;
        }
        
        // Check user preferences
        if let Some(decision) = self.check_user_preferences(command) {
            return decision;
        }
        
        // Use ML-based recommendations
        if let Some(primary_rec) = complexity_result.recommendations.first() {
            return self.create_ml_based_decision(primary_rec, complexity_result);
        }
        
        // Fallback to complexity score-based decision
        self.create_score_based_decision(complexity_result)
    }
    
    /// Create decision based on ML recommendation
    fn create_ml_based_decision(
        &self,
        recommendation: &crate::analysis::InterfaceRecommendation,
        complexity_result: &ComplexityResult,
    ) -> InterfaceDecision {
        let strategy = match recommendation.interface {
            UiMode::Cli => InterfaceStrategy::CliOnly,
            UiMode::Tui => {
                if recommendation.priority >= RecommendationPriority::High {
                    InterfaceStrategy::AutoLaunchTui {
                        reason: recommendation.reason.clone(),
                        show_cli_first: recommendation.confidence < 0.9,
                    }
                } else {
                    InterfaceStrategy::PromptForTui {
                        reason: recommendation.reason.clone(),
                        default_yes: recommendation.confidence > 0.7,
                    }
                }
            },
            UiMode::Auto => {
                if complexity_result.overall_score > 6.0 {
                    InterfaceStrategy::PromptForTui {
                        reason: "Moderate to high complexity detected".to_string(),
                        default_yes: complexity_result.overall_score > 7.0,
                    }
                } else {
                    InterfaceStrategy::CliWithHint {
                        hint: format!("TUI available for enhanced experience (complexity: {:.1}/10)", complexity_result.overall_score),
                        tui_command: format!("dora ui {}", self.extract_command_name(&complexity_result)),
                    }
                }
            },
            UiMode::Minimal => InterfaceStrategy::CliOnly,
        };
        
        InterfaceDecision {
            strategy,
            confidence: recommendation.confidence,
            reason: format!("ML-based recommendation: {}", recommendation.reason),
            fallback: Some(InterfaceStrategy::CliOnly),
        }
    }
    
    /// Create decision based on complexity score
    fn create_score_based_decision(
        &self,
        complexity_result: &ComplexityResult,
    ) -> InterfaceDecision {
        let score = complexity_result.overall_score;
        let confidence = complexity_result.confidence;
        
        let strategy = match score {
            s if s >= 8.0 => InterfaceStrategy::AutoLaunchTui {
                reason: format!("Very high complexity ({:.1}/10) requires interactive interface", s),
                show_cli_first: false,
            },
            s if s >= 6.0 => InterfaceStrategy::PromptForTui {
                reason: format!("High complexity ({:.1}/10) benefits from interactive features", s),
                default_yes: s >= 7.0,
            },
            s if s >= 4.0 => InterfaceStrategy::CliWithHint {
                hint: format!("Moderate complexity ({:.1}/10) - TUI available for enhanced experience", s),
                tui_command: format!("dora ui {}", self.extract_command_name(&complexity_result)),
            },
            s => InterfaceStrategy::CliOnly,
        };
        
        InterfaceDecision {
            strategy,
            confidence,
            reason: format!("Complexity-based decision (score: {:.1}/10)", score),
            fallback: Some(InterfaceStrategy::CliOnly),
        }
    }
    
    /// Check for contexts that force certain interfaces
    fn check_forced_contexts(&self, context: &ExecutionContext) -> Option<InterfaceDecision> {
        if !context.is_tty || context.is_piped || context.is_scripted || context.environment.is_ci {
            return Some(InterfaceDecision {
                strategy: InterfaceStrategy::CliOnly,
                confidence: 1.0,
                reason: "Non-interactive environment detected".to_string(),
                fallback: None,
            });
        }
        
        if !context.terminal_capabilities.tui_capable {
            return Some(InterfaceDecision {
                strategy: InterfaceStrategy::CliOnly,
                confidence: 0.9,
                reason: "Terminal not capable of TUI interface".to_string(),
                fallback: None,
            });
        }
        
        None
    }
    
    /// Check user preference overrides
    fn check_user_preferences(&self, command: &Command) -> Option<InterfaceDecision> {
        let command_name = self.extract_command_name_from_command(command);
        
        if let Some(mode) = self.config.command_preferences.get(&command_name) {
            return Some(self.create_preference_decision(mode, "User command preference"));
        }
        
        match self.config.global_ui_mode {
            UiMode::Cli => Some(self.create_preference_decision(&UiMode::Cli, "User global preference: CLI")),
            UiMode::Tui => Some(self.create_preference_decision(&UiMode::Tui, "User global preference: TUI")),
            UiMode::Minimal => Some(self.create_preference_decision(&UiMode::Minimal, "User global preference: Minimal")),
            UiMode::Auto => None,
        }
    }
    
    /// Create decision based on user preference
    fn create_preference_decision(&self, mode: &UiMode, reason: &str) -> InterfaceDecision {
        let strategy = match mode {
            UiMode::Cli => InterfaceStrategy::CliOnly,
            UiMode::Tui => InterfaceStrategy::AutoLaunchTui {
                reason: "User preference".to_string(),
                show_cli_first: false,
            },
            UiMode::Minimal => InterfaceStrategy::CliOnly,
            UiMode::Auto => unreachable!(),
        };
        
        InterfaceDecision {
            strategy,
            confidence: 1.0,
            reason: reason.to_string(),
            fallback: None,
        }
    }
    
    /// Determine user expertise level
    fn determine_user_expertise(&self, _command: &Command, context: &ExecutionContext) -> UserExpertiseLevel {
        // Simplified heuristic - in a real implementation, this could be learned
        if context.environment.is_ci || context.is_scripted {
            UserExpertiseLevel::Expert
        } else if context.is_piped {
            UserExpertiseLevel::Intermediate
        } else {
            UserExpertiseLevel::Intermediate // Default
        }
    }
    
    /// Generate learning feedback if appropriate
    fn generate_learning_feedback(
        &self,
        complexity_result: &crate::analysis::ComplexityResult,
        decision: &InterfaceDecision,
    ) -> Option<LearningFeedback> {
        // Only collect feedback for decisions with moderate confidence
        if complexity_result.confidence < 0.8 && complexity_result.confidence > 0.4 {
            Some(LearningFeedback {
                should_collect_feedback: true,
                feedback_prompt: format!(
                    "How was your experience with this {} operation? (1-5 scale)",
                    self.classify_complexity(complexity_result.overall_score)
                ),
                session_id: uuid::Uuid::new_v4().to_string(),
            })
        } else {
            None
        }
    }
    
    /// Classify complexity level for user feedback
    fn classify_complexity(&self, score: f32) -> &'static str {
        match score {
            s if s >= 8.0 => "very complex",
            s if s >= 6.0 => "complex",
            s if s >= 4.0 => "moderate",
            _ => "simple",
        }
    }
    
    /// Extract command name from complexity result (simplified)
    fn extract_command_name(&self, _complexity_result: &crate::analysis::ComplexityResult) -> String {
        "command".to_string() // Simplified - would extract from the actual command
    }
    
    /// Extract command name from command
    fn extract_command_name_from_command(&self, command: &Command) -> String {
        command.name()
    }
    
    /// Record user feedback for learning
    pub async fn record_feedback(
        &self,
        session_id: &str,
        satisfaction: crate::config::preferences::SatisfactionLevel,
        chosen_interface: UiMode,
        comments: Option<String>,
    ) -> eyre::Result<()> {
        // This would integrate with the ML model's feedback system
        // For now, just log the feedback
        log::info!(
            "User feedback recorded - Session: {}, Satisfaction: {:?}, Interface: {:?}, Comments: {:?}",
            session_id, satisfaction, chosen_interface, comments
        );
        Ok(())
    }
}

impl SystemMonitor {
    pub fn new() -> Self {
        Self {
            last_system_state: None,
            update_interval: std::time::Duration::from_secs(30),
        }
    }
    
    /// Get current system state
    pub async fn get_current_state(&mut self) -> SystemState {
        // Check if we need to update the cached state
        if let Some(ref state) = self.last_system_state {
            if state.last_updated.signed_duration_since(chrono::Utc::now()).num_seconds().abs() 
                < self.update_interval.as_secs() as i64 {
                return state.clone();
            }
        }
        
        // Collect fresh system state
        let state = self.collect_system_state().await;
        self.last_system_state = Some(state.clone());
        state
    }
    
    /// Collect current system state
    async fn collect_system_state(&self) -> SystemState {
        // In a real implementation, this would collect actual system metrics
        // For now, return a reasonable default with some variation
        SystemState {
            active_dataflows: self.count_active_dataflows().await,
            system_load: self.get_system_load().await,
            memory_usage: self.get_memory_usage().await,
            network_activity: self.get_network_activity().await,
            error_rate: self.get_error_rate().await,
            last_updated: chrono::Utc::now(),
        }
    }
    
    async fn count_active_dataflows(&self) -> usize {
        // Simplified - would query actual dataflow manager
        3
    }
    
    async fn get_system_load(&self) -> f32 {
        // Simplified - would use system APIs
        0.4
    }
    
    async fn get_memory_usage(&self) -> f32 {
        // Simplified - would use system APIs
        0.6
    }
    
    async fn get_network_activity(&self) -> f32 {
        // Simplified - would monitor network interfaces
        0.3
    }
    
    async fn get_error_rate(&self) -> f32 {
        // Simplified - would analyze recent error logs
        0.02
    }
}

impl Default for EnhancedInterfaceSelector {
    fn default() -> Self {
        Self::new(UserConfig::default())
    }
}

/// Integration helper for backward compatibility
pub fn create_enhanced_selector(config: UserConfig) -> EnhancedInterfaceSelector {
    EnhancedInterfaceSelector::new(config)
}