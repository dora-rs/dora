// Help Module Types for Issue #21
// Core type definitions for intelligent help system

use chrono::{DateTime, Duration, Utc};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use uuid::Uuid;

// ============================================================================
// Help Session Management
// ============================================================================

/// Represents a help session with user context and preferences
#[derive(Debug, Clone)]
pub struct HelpSession {
    pub session_id: Uuid,
    pub start_time: DateTime<Utc>,
    pub user_expertise: UserExpertiseLevel,
    pub system_context: Option<SystemContext>,
    pub interaction_history: Vec<HelpInteraction>,
    pub personalization_data: UserHelpPreferences,
}

/// User expertise level for adaptive help content
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum UserExpertiseLevel {
    Beginner,
    Intermediate,
    Expert,
}

/// System context for contextual help
#[derive(Debug, Clone)]
pub struct SystemContext {
    pub active_dataflows: Vec<DataflowInfo>,
    pub recent_errors: Vec<ErrorInfo>,
    pub performance_issues: Option<PerformanceIssues>,
    pub recent_commands: Vec<String>,
    pub environment_info: EnvironmentInfo,
}

/// Dataflow information for contextual help
#[derive(Debug, Clone)]
pub struct DataflowInfo {
    pub name: String,
    pub status: String,
    pub node_count: usize,
    pub uptime: Option<Duration>,
}

/// Error information for contextual troubleshooting
#[derive(Debug, Clone)]
pub struct ErrorInfo {
    pub error_type: String,
    pub message: String,
    pub timestamp: DateTime<Utc>,
    pub source: String,
}

/// Performance issues for contextual help
#[derive(Debug, Clone)]
pub struct PerformanceIssues {
    pub high_cpu_usage: bool,
    pub high_memory_usage: bool,
    pub slow_response_time: bool,
    pub description: String,
}

/// Environment information
#[derive(Debug, Clone)]
pub struct EnvironmentInfo {
    pub is_ci: bool,
    pub shell_type: Option<String>,
    pub terminal_capabilities: String,
}

/// User interaction with help system
#[derive(Debug, Clone)]
pub struct HelpInteraction {
    pub timestamp: DateTime<Utc>,
    pub topic: String,
    pub interaction_type: InteractionType,
    pub duration: Option<Duration>,
}

#[derive(Debug, Clone)]
pub enum InteractionType {
    TopicView,
    Search,
    Tutorial,
    Example,
    Navigation,
}

/// User preferences for help system
#[derive(Debug, Clone, Default)]
pub struct UserHelpPreferences {
    pub preferred_format: Option<HelpFormat>,
    pub preferred_detail_level: Option<DetailLevel>,
    pub show_examples: bool,
    pub enable_tutorials: bool,
    pub frequently_accessed_topics: Vec<String>,
}

// ============================================================================
// Help Content Structures
// ============================================================================

/// Comprehensive help content for a topic
#[derive(Debug, Clone)]
pub struct HelpContent {
    pub topic: String,
    pub title: String,
    pub summary: String,
    pub sections: Vec<HelpSection>,
    pub examples: Vec<HelpExample>,
    pub related_topics: Vec<String>,
    pub prerequisites: Vec<String>,
    pub difficulty_level: DifficultyLevel,
    pub estimated_time: Option<Duration>,
    pub tags: Vec<String>,
    pub last_updated: DateTime<Utc>,
}

impl HelpContent {
    /// Create a new empty help content
    pub fn new(topic: String) -> Self {
        Self {
            topic: topic.clone(),
            title: topic,
            summary: String::new(),
            sections: Vec::new(),
            examples: Vec::new(),
            related_topics: Vec::new(),
            prerequisites: Vec::new(),
            difficulty_level: DifficultyLevel::Beginner,
            estimated_time: None,
            tags: Vec::new(),
            last_updated: Utc::now(),
        }
    }

    /// Create "not found" help content
    pub fn not_found(query: &str) -> Self {
        Self {
            topic: "not-found".to_string(),
            title: "Help Topic Not Found".to_string(),
            summary: format!("No help found for '{}'", query),
            sections: vec![HelpSection {
                title: "Suggestions".to_string(),
                content: "Try searching with different keywords or use --list to see all available topics.".to_string(),
                section_type: SectionType::Usage,
                visibility_level: DetailLevel::Normal,
                interactive_elements: Vec::new(),
            }],
            examples: Vec::new(),
            related_topics: Vec::new(),
            prerequisites: Vec::new(),
            difficulty_level: DifficultyLevel::Beginner,
            estimated_time: None,
            tags: Vec::new(),
            last_updated: Utc::now(),
        }
    }
}

/// Section within help content
#[derive(Debug, Clone)]
pub struct HelpSection {
    pub title: String,
    pub content: String,
    pub section_type: SectionType,
    pub visibility_level: DetailLevel,
    pub interactive_elements: Vec<InteractiveElement>,
}

/// Type of help section
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SectionType {
    Overview,
    Usage,
    Examples,
    Parameters,
    Configuration,
    Troubleshooting,
    Advanced,
    References,
    Tutorial,
}

/// Interactive element within help content
#[derive(Debug, Clone)]
pub enum InteractiveElement {
    RunnableCommand {
        command: String,
        description: String,
    },
    ExampleSnippet {
        code: String,
        language: String,
    },
    QuickLink {
        text: String,
        target: String,
    },
}

/// Help example with runnable commands
#[derive(Debug, Clone)]
pub struct HelpExample {
    pub title: String,
    pub description: String,
    pub command: String,
    pub expected_output: Option<String>,
    pub explanation: String,
    pub difficulty: DifficultyLevel,
    pub prerequisites: Vec<String>,
    pub runnable: bool,
}

// ============================================================================
// Help Formats and Levels
// ============================================================================

/// Help output format
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, clap::ValueEnum)]
pub enum HelpFormat {
    Text,
    Markdown,
    Json,
    Html,
}

impl Default for HelpFormat {
    fn default() -> Self {
        HelpFormat::Text
    }
}

/// Detail level for help content
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, clap::ValueEnum)]
pub enum DetailLevel {
    Quick,
    Normal,
    Detailed,
    Expert,
}

impl Default for DetailLevel {
    fn default() -> Self {
        DetailLevel::Normal
    }
}

/// Difficulty level for content
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum DifficultyLevel {
    Beginner,
    Intermediate,
    Advanced,
    Expert,
}

// ============================================================================
// Tutorial System Types
// ============================================================================

/// Interactive tutorial definition
#[derive(Debug, Clone)]
pub struct Tutorial {
    pub id: String,
    pub title: String,
    pub description: String,
    pub difficulty: DifficultyLevel,
    pub estimated_duration: Duration,
    pub prerequisites: Vec<String>,
    pub steps: Vec<TutorialStep>,
    pub completion_criteria: CompletionCriteria,
    pub tags: Vec<String>,
}

/// Single step in a tutorial
#[derive(Debug, Clone)]
pub struct TutorialStep {
    pub step_number: usize,
    pub title: String,
    pub description: String,
    pub instruction: String,
    pub expected_command: Option<String>,
    pub expected_output: Option<String>,
    pub validation: StepValidation,
    pub hints: Vec<String>,
    pub common_mistakes: Vec<CommonMistake>,
    pub next_steps: Vec<NextStepOption>,
}

/// Tutorial step validation
#[derive(Debug, Clone)]
pub enum StepValidation {
    CommandExecution {
        command: String,
        expected_result: ValidationCriteria,
    },
    OutputContains {
        patterns: Vec<String>,
    },
    SystemState {
        state_check: StateValidation,
    },
    UserConfirmation,
    NoValidation,
}

/// Validation criteria for step completion
#[derive(Debug, Clone)]
pub struct ValidationCriteria {
    pub success_patterns: Vec<String>,
    pub failure_patterns: Vec<String>,
    pub min_execution_time: Option<Duration>,
    pub max_execution_time: Option<Duration>,
}

/// System state validation
#[derive(Debug, Clone)]
pub struct StateValidation {
    pub check_type: StateCheckType,
    pub expected_value: String,
}

#[derive(Debug, Clone)]
pub enum StateCheckType {
    DataflowExists,
    DataflowRunning,
    NodeCount,
    ConfigurationSet,
}

/// Common mistakes in tutorial steps
#[derive(Debug, Clone)]
pub struct CommonMistake {
    pub mistake: String,
    pub consequence: String,
    pub correction: String,
}

/// Next step options after completion
#[derive(Debug, Clone)]
pub struct NextStepOption {
    pub label: String,
    pub description: String,
    pub target_step: Option<usize>,
}

/// Criteria for tutorial completion
#[derive(Debug, Clone)]
pub struct CompletionCriteria {
    pub all_steps_required: bool,
    pub minimum_score: Option<f32>,
    pub time_limit: Option<Duration>,
}

/// Tutorial session state
#[derive(Debug, Clone)]
pub struct TutorialSession {
    pub tutorial_id: String,
    pub session_id: Uuid,
    pub start_time: DateTime<Utc>,
    pub current_step: usize,
    pub completed_steps: Vec<usize>,
    pub user_actions: Vec<TutorialUserAction>,
    pub hints_used: Vec<usize>,
}

/// User action in tutorial
#[derive(Debug, Clone)]
pub struct TutorialUserAction {
    pub timestamp: DateTime<Utc>,
    pub action_type: TutorialActionType,
    pub step_number: usize,
}

#[derive(Debug, Clone)]
pub enum TutorialActionType {
    StepStarted,
    CommandExecuted { command: String },
    HintRequested,
    StepCompleted,
    StepFailed,
}

/// User input for tutorial validation
#[derive(Debug, Clone)]
pub enum TutorialUserInput {
    Command { command: String, output: String },
    Confirmation { confirmed: bool },
    StateCheck { state: String },
}

/// Step validation result
#[derive(Debug, Clone)]
pub enum StepValidationResult {
    Success,
    PartialSuccess { feedback: String },
    Failure { reason: String },
}

/// Step execution result
#[derive(Debug, Clone)]
pub enum StepExecutionResult {
    StepCompleted {
        next_step: TutorialStep,
        progress: TutorialProgress,
    },
    TutorialCompleted {
        completion_time: DateTime<Utc>,
        total_hints_used: usize,
        completion_certificate: CompletionCertificate,
    },
    PartialSuccess {
        feedback: String,
        suggestions: Vec<String>,
    },
    StepFailed {
        reason: String,
        hints: Vec<String>,
        retry_suggestions: Vec<String>,
    },
}

/// Tutorial progress tracking
#[derive(Debug, Clone)]
pub struct TutorialProgress {
    pub completed_steps: usize,
    pub total_steps: usize,
    pub percentage: f32,
}

/// Tutorial completion certificate
#[derive(Debug, Clone)]
pub struct CompletionCertificate {
    pub tutorial_id: String,
    pub tutorial_title: String,
    pub completion_time: DateTime<Utc>,
    pub duration: Duration,
    pub score: Option<f32>,
}

// ============================================================================
// Help Complexity Analysis
// ============================================================================

/// Help content complexity analysis
#[derive(Debug, Clone)]
pub struct HelpComplexity {
    pub overall_score: f32,
    pub content_volume_score: f32,
    pub interaction_score: f32,
    pub tutorial_score: f32,
    pub factors: Vec<ComplexityFactor>,
}

/// Individual complexity factor
#[derive(Debug, Clone)]
pub struct ComplexityFactor {
    pub factor_type: FactorType,
    pub impact: f32,
    pub description: String,
    pub evidence: Vec<String>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FactorType {
    ContentVolume,
    InteractiveContent,
    TutorialComplexity,
    ExampleComplexity,
    SearchComplexity,
}

// ============================================================================
// Help Index and Search
// ============================================================================

/// Help content index
#[derive(Debug, Clone)]
pub struct HelpIndex {
    pub topics: HashMap<String, HelpTopicEntry>,
    pub tags: HashMap<String, Vec<String>>,
    pub categories: HashMap<String, Vec<String>>,
}

impl HelpIndex {
    pub fn new() -> Self {
        Self {
            topics: HashMap::new(),
            tags: HashMap::new(),
            categories: HashMap::new(),
        }
    }

    pub async fn get_topic(&self, topic: &str) -> eyre::Result<Option<HelpContent>> {
        // Mock implementation - would load from actual help content
        if let Some(entry) = self.topics.get(topic) {
            Ok(Some(entry.content.clone()))
        } else {
            Ok(None)
        }
    }
}

/// Help topic index entry
#[derive(Debug, Clone)]
pub struct HelpTopicEntry {
    pub topic_id: String,
    pub title: String,
    pub summary: String,
    pub content: HelpContent,
    pub keywords: Vec<String>,
}

/// Search result
#[derive(Debug, Clone)]
pub struct HelpSearchResult {
    pub topic: String,
    pub title: String,
    pub relevance_score: f32,
    pub matched_keywords: Vec<String>,
    pub snippet: String,
}
