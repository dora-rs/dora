// Context-Sensitive Preference Application - Issue #16

use super::types::*;
use crate::cli::context::ExecutionContext;
use chrono::{Datelike, Timelike, Utc};
use std::collections::HashMap;
use std::path::Path;

/// Analyzes context for preference resolution
#[derive(Debug)]
pub struct ContextAnalyzer {
    context_patterns: HashMap<String, ContextPattern>,
    temporal_analyzer: TemporalAnalyzer,
    environment_classifier: EnvironmentClassifier,
}

/// Result of context analysis
#[derive(Debug, Clone)]
pub struct ContextAnalysis {
    pub environment_type: EnvironmentType,
    pub temporal_context: TemporalContext,
    pub usage_patterns: Vec<UsagePattern>,
    pub complexity_indicators: ComplexityIndicators,
    pub user_state: UserState,
}

/// Type of environment detected
#[derive(Debug, Clone, PartialEq)]
pub enum EnvironmentType {
    Development { project_type: Option<String> },
    Production { criticality: CriticalityLevel },
    Testing { test_type: TestType },
    Debugging { debug_session: DebugSession },
    Learning { tutorial_step: Option<u32> },
    Demonstration { audience_type: AudienceType },
}

#[derive(Debug, Clone, PartialEq)]
pub enum CriticalityLevel {
    Low,
    Medium,
    High,
    Critical,
}

#[derive(Debug, Clone, PartialEq)]
pub enum TestType {
    Unit,
    Integration,
    EndToEnd,
    Performance,
}

#[derive(Debug, Clone, PartialEq)]
pub struct DebugSession {
    pub session_id: String,
    pub started_at: chrono::DateTime<Utc>,
}

#[derive(Debug, Clone, PartialEq)]
pub enum AudienceType {
    Technical,
    NonTechnical,
    Mixed,
}

/// Temporal context
#[derive(Debug, Clone)]
pub struct TemporalContext {
    pub time_of_day: TimeOfDay,
    pub day_of_week: DayOfWeek,
    pub session_duration: std::time::Duration,
    pub command_frequency: f32,
    pub break_duration: Option<std::time::Duration>,
}

#[derive(Debug, Clone, PartialEq)]
pub enum TimeOfDay {
    EarlyMorning,   // 5am - 9am
    Morning,        // 9am - 12pm
    Afternoon,      // 12pm - 5pm
    Evening,        // 5pm - 9pm
    Night,          // 9pm - 5am
}

#[derive(Debug, Clone, PartialEq)]
pub enum DayOfWeek {
    Monday,
    Tuesday,
    Wednesday,
    Thursday,
    Friday,
    Saturday,
    Sunday,
}

/// Usage pattern
#[derive(Debug, Clone)]
pub struct UsagePattern {
    pub pattern_type: String,
    pub frequency: f32,
    pub confidence: f32,
}

/// Complexity indicators
#[derive(Debug, Clone)]
pub struct ComplexityIndicators {
    pub overall_score: f32,  // 0.0 - 1.0
    pub node_count: usize,
    pub connection_density: f32,
    pub error_rate: f32,
}

/// User state
#[derive(Debug, Clone)]
pub struct UserState {
    pub expertise_level: f32,  // 0.0 - 1.0
    pub focus_level: f32,      // 0.0 - 1.0
    pub fatigue_level: f32,    // 0.0 - 1.0
}

/// Temporal analyzer
#[derive(Debug)]
pub struct TemporalAnalyzer;

impl TemporalAnalyzer {
    pub fn analyze_temporal_context(&self) -> TemporalContext {
        let now = Utc::now();
        let hour = now.hour();

        let time_of_day = match hour {
            5..=8 => TimeOfDay::EarlyMorning,
            9..=11 => TimeOfDay::Morning,
            12..=16 => TimeOfDay::Afternoon,
            17..=20 => TimeOfDay::Evening,
            _ => TimeOfDay::Night,
        };

        let day_of_week = match now.weekday() {
            chrono::Weekday::Mon => DayOfWeek::Monday,
            chrono::Weekday::Tue => DayOfWeek::Tuesday,
            chrono::Weekday::Wed => DayOfWeek::Wednesday,
            chrono::Weekday::Thu => DayOfWeek::Thursday,
            chrono::Weekday::Fri => DayOfWeek::Friday,
            chrono::Weekday::Sat => DayOfWeek::Saturday,
            chrono::Weekday::Sun => DayOfWeek::Sunday,
        };

        TemporalContext {
            time_of_day,
            day_of_week,
            session_duration: std::time::Duration::from_secs(0),
            command_frequency: 0.0,
            break_duration: None,
        }
    }
}

/// Environment classifier
#[derive(Debug)]
pub struct EnvironmentClassifier;

impl EnvironmentClassifier {
    pub fn classify(&self, context: &ExecutionContext, _user_context: &UserContext) -> EnvironmentType {
        // Check for CI/production indicators first
        if context.environment.is_ci {
            return EnvironmentType::Production {
                criticality: CriticalityLevel::High,
            };
        }

        // Check for development environment
        let current_dir = &context.working_dir;
        if self.is_development_environment(current_dir) {
            let project_type = self.detect_project_type(current_dir);
            return EnvironmentType::Development { project_type };
        }

        // Default to development
        EnvironmentType::Development { project_type: None }
    }

    fn is_development_environment(&self, current_dir: &Path) -> bool {
        let dev_indicators = [
            ".git", ".gitignore", "Cargo.toml", "package.json",
            "requirements.txt", "CMakeLists.txt", "Makefile"
        ];

        dev_indicators.iter().any(|indicator| {
            current_dir.join(indicator).exists() ||
            current_dir.ancestors().any(|ancestor| ancestor.join(indicator).exists())
        })
    }

    fn detect_project_type(&self, current_dir: &Path) -> Option<String> {
        if current_dir.join("Cargo.toml").exists() {
            Some("Rust".to_string())
        } else if current_dir.join("package.json").exists() {
            Some("JavaScript/TypeScript".to_string())
        } else if current_dir.join("requirements.txt").exists() || current_dir.join("setup.py").exists() {
            Some("Python".to_string())
        } else if current_dir.join("go.mod").exists() {
            Some("Go".to_string())
        } else if current_dir.join("pom.xml").exists() || current_dir.join("build.gradle").exists() {
            Some("Java".to_string())
        } else {
            None
        }
    }
}

impl ContextAnalyzer {
    pub fn new() -> Self {
        Self {
            context_patterns: HashMap::new(),
            temporal_analyzer: TemporalAnalyzer,
            environment_classifier: EnvironmentClassifier,
        }
    }

    pub fn analyze(
        &self,
        execution_context: &ExecutionContext,
        user_context: &UserContext,
    ) -> ContextAnalysis {
        let environment_type = self.environment_classifier.classify(execution_context, user_context);
        let temporal_context = self.temporal_analyzer.analyze_temporal_context();
        let usage_patterns = self.analyze_usage_patterns(user_context);
        let complexity_indicators = self.analyze_complexity_indicators(execution_context);
        let user_state = self.analyze_user_state(user_context);

        ContextAnalysis {
            environment_type,
            temporal_context,
            usage_patterns,
            complexity_indicators,
            user_state,
        }
    }

    fn analyze_usage_patterns(&self, user_context: &UserContext) -> Vec<UsagePattern> {
        let mut patterns = Vec::new();

        // Analyze command frequency
        if user_context.recent_command_count > 10 {
            patterns.push(UsagePattern {
                pattern_type: "High Activity".to_string(),
                frequency: user_context.recent_command_count as f32 / 100.0,
                confidence: 0.8,
            });
        }

        patterns
    }

    fn analyze_complexity_indicators(&self, _context: &ExecutionContext) -> ComplexityIndicators {
        // Simplified complexity analysis
        // In a real implementation, this would analyze dataflow graphs, etc.
        ComplexityIndicators {
            overall_score: 0.5,  // Neutral default
            node_count: 0,
            connection_density: 0.0,
            error_rate: 0.0,
        }
    }

    fn analyze_user_state(&self, user_context: &UserContext) -> UserState {
        UserState {
            expertise_level: user_context.expertise_level_f32(),
            focus_level: 0.7,  // Default assumption
            fatigue_level: 0.3,  // Default assumption
        }
    }
}

/// Matches context patterns to preferences
#[derive(Debug)]
pub struct PreferenceContextMatcher {
    context_rules: Vec<ContextRule>,
    pattern_matcher: PatternMatcher,
}

/// Context-based rule for preferences
#[derive(Debug, Clone)]
pub struct ContextRule {
    pub rule_id: String,
    pub conditions: Vec<ContextCondition>,
    pub preference_adjustments: HashMap<PreferenceType, PreferenceAdjustment>,
    pub priority: u8,
    pub description: String,
}

/// Condition for context matching
#[derive(Debug, Clone)]
pub enum ContextCondition {
    EnvironmentType(EnvironmentType),
    TimeOfDay(TimeRange),
    CommandType(CommandCategory),
    UserExpertise(ExpertiseRange),
    SessionDuration(DurationRange),
    ErrorRate(f32),
    ComplexityScore(f32),
}

#[derive(Debug, Clone)]
pub struct TimeRange {
    pub start: u32,
    pub end: u32,
}

impl TimeRange {
    pub fn contains(&self, time_of_day: &TimeOfDay) -> bool {
        let hour = match time_of_day {
            TimeOfDay::EarlyMorning => 7,
            TimeOfDay::Morning => 10,
            TimeOfDay::Afternoon => 14,
            TimeOfDay::Evening => 18,
            TimeOfDay::Night => 22,
        };
        hour >= self.start && hour < self.end
    }
}

#[derive(Debug, Clone)]
pub enum CommandCategory {
    Basic,
    Advanced,
    Debugging,
    Analysis,
}

#[derive(Debug, Clone)]
pub struct ExpertiseRange {
    pub min: f32,
    pub max: f32,
}

impl ExpertiseRange {
    pub fn contains(&self, expertise: &f32) -> bool {
        *expertise >= self.min && *expertise <= self.max
    }
}

#[derive(Debug, Clone)]
pub struct DurationRange {
    pub min: std::time::Duration,
    pub max: std::time::Duration,
}

impl DurationRange {
    pub fn contains(&self, duration: &std::time::Duration) -> bool {
        *duration >= self.min && *duration <= self.max
    }
}

/// Adjustment to preference based on context
#[derive(Debug, Clone)]
pub struct PreferenceAdjustment {
    pub adjustment_type: AdjustmentType,
    pub value: PreferenceValue,
    pub priority: u8,
}

#[derive(Debug, Clone)]
pub enum AdjustmentType {
    Override,
    Suggest,
    Hint,
}

/// Pattern matcher
#[derive(Debug)]
pub struct PatternMatcher;

impl PreferenceContextMatcher {
    pub fn new() -> Self {
        Self {
            context_rules: Self::default_rules(),
            pattern_matcher: PatternMatcher,
        }
    }

    fn default_rules() -> Vec<ContextRule> {
        vec![
            // Rule: CI environments should use minimal output
            ContextRule {
                rule_id: "ci_minimal".to_string(),
                conditions: vec![],
                preference_adjustments: HashMap::new(),
                priority: 200,
                description: "Use minimal output in CI environments".to_string(),
            },
            // Rule: Beginners should see more hints
            ContextRule {
                rule_id: "beginner_hints".to_string(),
                conditions: vec![
                    ContextCondition::UserExpertise(ExpertiseRange { min: 0.0, max: 0.5 }),
                ],
                preference_adjustments: HashMap::new(),
                priority: 100,
                description: "Show more hints for beginners".to_string(),
            },
        ]
    }

    pub fn apply_context_adjustments(
        &self,
        _base_preference: &PreferenceValue,
        context_analysis: &ContextAnalysis,
    ) -> Vec<PreferenceAdjustment> {
        let mut adjustments = Vec::new();

        for rule in &self.context_rules {
            if self.rule_matches_context(rule, context_analysis) {
                for (_pref_type, adjustment) in &rule.preference_adjustments {
                    adjustments.push(adjustment.clone());
                }
            }
        }

        // Sort by priority
        adjustments.sort_by_key(|adj| std::cmp::Reverse(adj.priority));
        adjustments
    }

    fn rule_matches_context(&self, rule: &ContextRule, context: &ContextAnalysis) -> bool {
        rule.conditions.iter().all(|condition| {
            self.condition_matches(condition, context)
        })
    }

    fn condition_matches(&self, condition: &ContextCondition, context: &ContextAnalysis) -> bool {
        match condition {
            ContextCondition::EnvironmentType(env_type) => {
                std::mem::discriminant(&context.environment_type) == std::mem::discriminant(env_type)
            },
            ContextCondition::TimeOfDay(time_range) => {
                time_range.contains(&context.temporal_context.time_of_day)
            },
            ContextCondition::UserExpertise(expertise_range) => {
                expertise_range.contains(&context.user_state.expertise_level)
            },
            ContextCondition::SessionDuration(duration_range) => {
                duration_range.contains(&context.temporal_context.session_duration)
            },
            ContextCondition::ComplexityScore(min_score) => {
                context.complexity_indicators.overall_score >= *min_score
            },
            _ => false,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_context_analysis() {
        let analyzer = ContextAnalyzer::new();
        let context = ExecutionContext::detect_basic();
        let user_context = UserContext::current();

        let analysis = analyzer.analyze(&context, &user_context);

        // Verify basic analysis works
        assert!(analysis.complexity_indicators.overall_score >= 0.0);
        assert!(analysis.complexity_indicators.overall_score <= 1.0);
    }

    #[test]
    fn test_environment_classification() {
        let classifier = EnvironmentClassifier;
        let context = ExecutionContext::detect_basic();
        let user_context = UserContext::current();

        let env_type = classifier.classify(&context, &user_context);

        // Should detect some environment type
        println!("Detected environment: {:?}", env_type);
    }

    #[test]
    fn test_temporal_analysis() {
        let analyzer = TemporalAnalyzer;
        let temporal = analyzer.analyze_temporal_context();

        // Verify time of day is detected
        println!("Time of day: {:?}", temporal.time_of_day);
        println!("Day of week: {:?}", temporal.day_of_week);
    }
}
