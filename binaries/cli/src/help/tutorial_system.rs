// Tutorial System for Issue #21
// Interactive tutorials with step validation and progress tracking

use super::types::*;
use chrono::{DateTime, Duration, Utc};
use eyre::{anyhow, Result};
use std::collections::HashMap;
use uuid::Uuid;

// ============================================================================
// Tutorial System
// ============================================================================

/// Manages interactive tutorials with validation and progress tracking
#[derive(Debug)]
pub struct TutorialSystem {
    tutorials: HashMap<String, Tutorial>,
    progress_tracker: TutorialProgressTracker,
}

impl TutorialSystem {
    pub fn new() -> Self {
        let mut tutorials = HashMap::new();

        // Add sample tutorials
        tutorials.insert(
            "getting-started".to_string(),
            Self::create_getting_started_tutorial(),
        );
        tutorials.insert(
            "dataflow-basics".to_string(),
            Self::create_dataflow_basics_tutorial(),
        );

        Self {
            tutorials,
            progress_tracker: TutorialProgressTracker::new(),
        }
    }

    /// Get available tutorials filtered by user expertise
    pub async fn get_available_tutorials(&self, user_expertise: UserExpertiseLevel) -> Vec<Tutorial> {
        self.tutorials.values()
            .filter(|tutorial| self.is_tutorial_appropriate(tutorial, user_expertise))
            .cloned()
            .collect()
    }

    /// Start a new tutorial session
    pub async fn start_tutorial(&mut self, tutorial_id: &str) -> Result<TutorialSession> {
        let tutorial = self.tutorials.get(tutorial_id)
            .ok_or_else(|| anyhow!("Tutorial '{}' not found", tutorial_id))?;

        // Initialize tutorial session
        let session = TutorialSession {
            tutorial_id: tutorial_id.to_string(),
            session_id: Uuid::new_v4(),
            start_time: Utc::now(),
            current_step: 0,
            completed_steps: Vec::new(),
            user_actions: Vec::new(),
            hints_used: Vec::new(),
        };

        // Track tutorial start
        self.progress_tracker.track_tutorial_start(&session);

        Ok(session)
    }

    /// Execute a tutorial step with user input
    pub async fn execute_step(
        &mut self,
        session: &mut TutorialSession,
        user_input: &TutorialUserInput,
    ) -> Result<StepExecutionResult> {
        let tutorial = self.tutorials.get(&session.tutorial_id)
            .ok_or_else(|| anyhow!("Tutorial not found"))?;

        if session.current_step >= tutorial.steps.len() {
            return Err(anyhow!("No more steps in tutorial"));
        }

        let current_step = &tutorial.steps[session.current_step];

        // Validate user input against step requirements
        let validation_result = self.validate_step_execution(current_step, user_input).await?;

        match validation_result {
            StepValidationResult::Success => {
                session.completed_steps.push(session.current_step);
                session.user_actions.push(TutorialUserAction {
                    timestamp: Utc::now(),
                    action_type: TutorialActionType::StepCompleted,
                    step_number: session.current_step,
                });
                session.current_step += 1;

                let result = if session.current_step >= tutorial.steps.len() {
                    StepExecutionResult::TutorialCompleted {
                        completion_time: Utc::now(),
                        total_hints_used: session.hints_used.len(),
                        completion_certificate: self.generate_completion_certificate(session, tutorial).await?,
                    }
                } else {
                    StepExecutionResult::StepCompleted {
                        next_step: tutorial.steps[session.current_step].clone(),
                        progress: TutorialProgress {
                            completed_steps: session.completed_steps.len(),
                            total_steps: tutorial.steps.len(),
                            percentage: (session.completed_steps.len() as f32 / tutorial.steps.len() as f32) * 100.0,
                        },
                    }
                };

                self.progress_tracker.track_step_completion(session, current_step);
                Ok(result)
            }

            StepValidationResult::PartialSuccess { feedback } => {
                Ok(StepExecutionResult::PartialSuccess {
                    feedback,
                    suggestions: self.generate_step_suggestions(current_step, user_input).await?,
                })
            }

            StepValidationResult::Failure { reason } => {
                session.user_actions.push(TutorialUserAction {
                    timestamp: Utc::now(),
                    action_type: TutorialActionType::StepFailed,
                    step_number: session.current_step,
                });

                let hints = self.generate_contextual_hints(current_step, user_input, &reason).await?;
                Ok(StepExecutionResult::StepFailed {
                    reason,
                    hints,
                    retry_suggestions: self.generate_retry_suggestions(current_step).await?,
                })
            }
        }
    }

    /// Get current step for a tutorial session
    pub async fn get_current_step(&self, session: &TutorialSession) -> Result<TutorialStep> {
        let tutorial = self.tutorials.get(&session.tutorial_id)
            .ok_or_else(|| anyhow!("Tutorial not found"))?;

        if session.current_step >= tutorial.steps.len() {
            return Err(anyhow!("Tutorial completed"));
        }

        Ok(tutorial.steps[session.current_step].clone())
    }

    /// Request a hint for the current step
    pub async fn get_hint(&mut self, session: &mut TutorialSession, hint_index: usize) -> Result<String> {
        let tutorial = self.tutorials.get(&session.tutorial_id)
            .ok_or_else(|| anyhow!("Tutorial not found"))?;

        let current_step = &tutorial.steps[session.current_step];

        if hint_index >= current_step.hints.len() {
            return Err(anyhow!("No more hints available"));
        }

        session.hints_used.push(session.current_step);
        session.user_actions.push(TutorialUserAction {
            timestamp: Utc::now(),
            action_type: TutorialActionType::HintRequested,
            step_number: session.current_step,
        });

        Ok(current_step.hints[hint_index].clone())
    }

    // ========================================================================
    // Private Helper Methods
    // ========================================================================

    fn is_tutorial_appropriate(&self, tutorial: &Tutorial, user_expertise: UserExpertiseLevel) -> bool {
        match (user_expertise, tutorial.difficulty) {
            (UserExpertiseLevel::Beginner, DifficultyLevel::Expert) => false,
            (UserExpertiseLevel::Beginner, DifficultyLevel::Advanced) => false,
            (UserExpertiseLevel::Intermediate, DifficultyLevel::Expert) => false,
            _ => true,
        }
    }

    async fn validate_step_execution(
        &self,
        step: &TutorialStep,
        user_input: &TutorialUserInput,
    ) -> Result<StepValidationResult> {
        match &step.validation {
            StepValidation::CommandExecution { command, expected_result } => {
                if let TutorialUserInput::Command { command: user_command, output } = user_input {
                    if self.normalize_command(user_command) == self.normalize_command(command) {
                        if self.validate_command_output(output, expected_result).await? {
                            Ok(StepValidationResult::Success)
                        } else {
                            Ok(StepValidationResult::PartialSuccess {
                                feedback: "Command executed but output doesn't match expected result".to_string(),
                            })
                        }
                    } else {
                        Ok(StepValidationResult::Failure {
                            reason: format!("Expected command '{}', got '{}'", command, user_command),
                        })
                    }
                } else {
                    Ok(StepValidationResult::Failure {
                        reason: "Expected command execution".to_string(),
                    })
                }
            }

            StepValidation::OutputContains { patterns } => {
                if let TutorialUserInput::Command { output, .. } = user_input {
                    let matches = patterns.iter()
                        .all(|pattern| output.contains(pattern));

                    if matches {
                        Ok(StepValidationResult::Success)
                    } else {
                        Ok(StepValidationResult::Failure {
                            reason: "Output doesn't contain expected patterns".to_string(),
                        })
                    }
                } else {
                    Ok(StepValidationResult::Failure {
                        reason: "Expected command output".to_string(),
                    })
                }
            }

            StepValidation::UserConfirmation => {
                if let TutorialUserInput::Confirmation { confirmed } = user_input {
                    if *confirmed {
                        Ok(StepValidationResult::Success)
                    } else {
                        Ok(StepValidationResult::Failure {
                            reason: "User did not confirm step completion".to_string(),
                        })
                    }
                } else {
                    Ok(StepValidationResult::Failure {
                        reason: "Expected user confirmation".to_string(),
                    })
                }
            }

            StepValidation::NoValidation => Ok(StepValidationResult::Success),

            _ => Ok(StepValidationResult::Success),
        }
    }

    fn normalize_command(&self, command: &str) -> String {
        command.trim().to_lowercase()
    }

    async fn validate_command_output(&self, output: &str, criteria: &ValidationCriteria) -> Result<bool> {
        // Check success patterns
        let has_success = criteria.success_patterns.is_empty() ||
            criteria.success_patterns.iter().any(|pattern| output.contains(pattern));

        // Check failure patterns
        let has_failure = criteria.failure_patterns.iter().any(|pattern| output.contains(pattern));

        Ok(has_success && !has_failure)
    }

    async fn generate_step_suggestions(
        &self,
        step: &TutorialStep,
        _user_input: &TutorialUserInput,
    ) -> Result<Vec<String>> {
        let mut suggestions = Vec::new();

        if let Some(expected_cmd) = &step.expected_command {
            suggestions.push(format!("Try: {}", expected_cmd));
        }

        if !step.hints.is_empty() {
            suggestions.push("Use --hint to get more guidance".to_string());
        }

        Ok(suggestions)
    }

    async fn generate_contextual_hints(
        &self,
        step: &TutorialStep,
        _user_input: &TutorialUserInput,
        _reason: &str,
    ) -> Result<Vec<String>> {
        let mut hints = Vec::new();

        // Return first hint if available
        if !step.hints.is_empty() {
            hints.push(step.hints[0].clone());
        }

        // Add common mistake hints
        for mistake in &step.common_mistakes {
            hints.push(format!("Common mistake: {} - {}", mistake.mistake, mistake.correction));
        }

        Ok(hints)
    }

    async fn generate_retry_suggestions(&self, step: &TutorialStep) -> Result<Vec<String>> {
        let mut suggestions = Vec::new();

        if let Some(expected_cmd) = &step.expected_command {
            suggestions.push(format!("Expected command: {}", expected_cmd));
        }

        suggestions.push("Review the step instructions carefully".to_string());
        suggestions.push("Use --hint for guidance".to_string());

        Ok(suggestions)
    }

    async fn generate_completion_certificate(
        &self,
        session: &TutorialSession,
        tutorial: &Tutorial,
    ) -> Result<CompletionCertificate> {
        let duration = Utc::now().signed_duration_since(session.start_time);

        Ok(CompletionCertificate {
            tutorial_id: tutorial.id.clone(),
            tutorial_title: tutorial.title.clone(),
            completion_time: Utc::now(),
            duration,
            score: Some(self.calculate_score(session, tutorial)),
        })
    }

    fn calculate_score(&self, session: &TutorialSession, tutorial: &Tutorial) -> f32 {
        let base_score = 100.0;

        // Deduct points for hints used
        let hint_penalty = session.hints_used.len() as f32 * 5.0;

        // Deduct points for failed attempts
        let failed_attempts = session.user_actions.iter()
            .filter(|action| matches!(action.action_type, TutorialActionType::StepFailed))
            .count() as f32;
        let failure_penalty = failed_attempts * 3.0;

        // Bonus for completing all steps
        let completion_bonus = if session.completed_steps.len() == tutorial.steps.len() {
            10.0
        } else {
            0.0
        };

        (base_score - hint_penalty - failure_penalty + completion_bonus).max(0.0).min(100.0)
    }

    // ========================================================================
    // Tutorial Creation Helpers
    // ========================================================================

    fn create_getting_started_tutorial() -> Tutorial {
        Tutorial {
            id: "getting-started".to_string(),
            title: "Getting Started with Dora".to_string(),
            description: "Learn the basics of Dora CLI and dataflow management".to_string(),
            difficulty: DifficultyLevel::Beginner,
            estimated_duration: Duration::minutes(15),
            prerequisites: Vec::new(),
            steps: vec![
                TutorialStep {
                    step_number: 1,
                    title: "Check Dora Installation".to_string(),
                    description: "Verify that Dora is installed correctly".to_string(),
                    instruction: "Run 'dora --version' to check your installation".to_string(),
                    expected_command: Some("dora --version".to_string()),
                    expected_output: Some("dora".to_string()),
                    validation: StepValidation::OutputContains {
                        patterns: vec!["dora".to_string()],
                    },
                    hints: vec![
                        "Type: dora --version".to_string(),
                        "Make sure dora is in your PATH".to_string(),
                    ],
                    common_mistakes: vec![
                        CommonMistake {
                            mistake: "Command not found".to_string(),
                            consequence: "Dora might not be installed or not in PATH".to_string(),
                            correction: "Install Dora or add it to your PATH".to_string(),
                        },
                    ],
                    next_steps: Vec::new(),
                },
                TutorialStep {
                    step_number: 2,
                    title: "List Dataflows".to_string(),
                    description: "Learn how to list running dataflows".to_string(),
                    instruction: "Run 'dora ps' to see active dataflows".to_string(),
                    expected_command: Some("dora ps".to_string()),
                    expected_output: None,
                    validation: StepValidation::UserConfirmation,
                    hints: vec![
                        "Type: dora ps".to_string(),
                        "This shows all currently running dataflows".to_string(),
                    ],
                    common_mistakes: Vec::new(),
                    next_steps: Vec::new(),
                },
            ],
            completion_criteria: CompletionCriteria {
                all_steps_required: true,
                minimum_score: Some(70.0),
                time_limit: None,
            },
            tags: vec!["beginner".to_string(), "basics".to_string()],
        }
    }

    fn create_dataflow_basics_tutorial() -> Tutorial {
        Tutorial {
            id: "dataflow-basics".to_string(),
            title: "Dataflow Management Basics".to_string(),
            description: "Learn how to create, start, and manage dataflows".to_string(),
            difficulty: DifficultyLevel::Intermediate,
            estimated_duration: Duration::minutes(20),
            prerequisites: vec!["getting-started".to_string()],
            steps: vec![
                TutorialStep {
                    step_number: 1,
                    title: "Create a Dataflow".to_string(),
                    description: "Learn how to create a new dataflow from a YAML file".to_string(),
                    instruction: "Create a dataflow using 'dora start dataflow.yaml'".to_string(),
                    expected_command: Some("dora start".to_string()),
                    expected_output: Some("started".to_string()),
                    validation: StepValidation::UserConfirmation,
                    hints: vec![
                        "Use: dora start <yaml-file>".to_string(),
                        "Make sure you have a valid dataflow YAML file".to_string(),
                    ],
                    common_mistakes: vec![
                        CommonMistake {
                            mistake: "File not found".to_string(),
                            consequence: "Dataflow won't start".to_string(),
                            correction: "Check the file path and name".to_string(),
                        },
                    ],
                    next_steps: Vec::new(),
                },
            ],
            completion_criteria: CompletionCriteria {
                all_steps_required: true,
                minimum_score: Some(70.0),
                time_limit: Some(Duration::minutes(30)),
            },
            tags: vec!["dataflow".to_string(), "intermediate".to_string()],
        }
    }
}

// ============================================================================
// Tutorial Progress Tracker
// ============================================================================

#[derive(Debug)]
pub struct TutorialProgressTracker {
    session_history: Vec<TutorialSessionRecord>,
}

impl TutorialProgressTracker {
    pub fn new() -> Self {
        Self {
            session_history: Vec::new(),
        }
    }

    pub fn track_tutorial_start(&mut self, session: &TutorialSession) {
        self.session_history.push(TutorialSessionRecord {
            session_id: session.session_id,
            tutorial_id: session.tutorial_id.clone(),
            start_time: session.start_time,
            status: TutorialSessionStatus::InProgress,
        });
    }

    pub fn track_step_completion(&mut self, _session: &TutorialSession, _step: &TutorialStep) {
        // Track step completion for analytics
    }

    pub fn track_tutorial_completion(&mut self, session_id: Uuid) {
        if let Some(record) = self.session_history.iter_mut().find(|r| r.session_id == session_id) {
            record.status = TutorialSessionStatus::Completed;
        }
    }
}

#[derive(Debug, Clone)]
struct TutorialSessionRecord {
    session_id: Uuid,
    tutorial_id: String,
    start_time: DateTime<Utc>,
    status: TutorialSessionStatus,
}

#[derive(Debug, Clone)]
enum TutorialSessionStatus {
    InProgress,
    Completed,
    Abandoned,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_start_tutorial() {
        let mut system = TutorialSystem::new();
        let session = system.start_tutorial("getting-started").await;

        assert!(session.is_ok());
        let session = session.unwrap();
        assert_eq!(session.tutorial_id, "getting-started");
        assert_eq!(session.current_step, 0);
    }

    #[tokio::test]
    async fn test_execute_step_success() {
        let mut system = TutorialSystem::new();
        let mut session = system.start_tutorial("getting-started").await.unwrap();

        let user_input = TutorialUserInput::Command {
            command: "dora --version".to_string(),
            output: "dora 0.3.0".to_string(),
        };

        let result = system.execute_step(&mut session, &user_input).await;
        assert!(result.is_ok());

        match result.unwrap() {
            StepExecutionResult::StepCompleted { progress, .. } => {
                assert_eq!(progress.completed_steps, 1);
            }
            _ => panic!("Expected StepCompleted"),
        }
    }

    #[tokio::test]
    async fn test_get_available_tutorials() {
        let system = TutorialSystem::new();
        let tutorials = system.get_available_tutorials(UserExpertiseLevel::Beginner).await;

        assert!(!tutorials.is_empty());
        assert!(tutorials.iter().any(|t| t.id == "getting-started"));
    }
}
