//! Core Automation Detection Engine
//!
//! This module implements the main AutomationDetector with comprehensive automation
//! context detection across CI/CD environments, scripting contexts, and machine interactions.

use super::{
    ci_detectors::{
        AzureDevOpsDetector, BuildkiteDetector, CiDetectionResult, CiEnvironmentDetector,
        CircleCiDetector, GitHubActionsDetector, GitLabCiDetector, JenkinsDetector,
        TeamCityDetector, TravisCiDetector,
    },
    confidence_calculator::ConfidenceCalculator,
    machine_detectors::{
        ApiClientDetector, AutomationToolDetector, CurlDetector, MachineInteractionDetector,
        MachineInteractionResult, WgetDetector,
    },
    pattern_analyzer::AutomationPatternAnalyzer,
    script_detectors::{
        BashScriptDetector, DockerContainerDetector, NodeScriptDetector, PowerShellScriptDetector,
        PythonScriptDetector, ScriptContextDetector, ScriptDetectionResult,
    },
};
use crate::cli::context::ExecutionContext;
use chrono::{DateTime, Utc};
use lru::LruCache;
use serde::{Deserialize, Serialize};
use std::num::NonZeroUsize;

#[derive(Debug)]
pub struct AutomationDetector {
    ci_detectors: Vec<Box<dyn CiEnvironmentDetector>>,
    script_detectors: Vec<Box<dyn ScriptContextDetector>>,
    machine_detectors: Vec<Box<dyn MachineInteractionDetector>>,
    pattern_analyzer: AutomationPatternAnalyzer,
    confidence_calculator: ConfidenceCalculator,
    detection_cache: LruCache<ContextKey, AutomationResult>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AutomationResult {
    pub is_automated: bool,
    pub automation_type: AutomationType,
    pub confidence: f32,
    pub detected_environment: Option<DetectedEnvironment>,
    pub interaction_patterns: Vec<InteractionPattern>,
    pub recommendations: AutomationRecommendations,
    pub evidence: Vec<AutomationEvidence>,
    pub timestamp: DateTime<Utc>,
}

#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize, Default)]
pub enum AutomationType {
    #[default]
    Interactive, // Human user in terminal
    CiCdPipeline,          // CI/CD environment
    ScriptedExecution,     // Shell script or automation script
    ApiIntegration,        // REST API or programmatic access
    ContainerizedWorkflow, // Docker/Kubernetes automation
    ScheduledTask,         // Cron job or scheduled execution
    TestingFramework,      // Automated testing environment
    MonitoringSystem,      // Health checks and monitoring
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DetectedEnvironment {
    pub platform: String,
    pub version: Option<String>,
    pub build_id: Option<String>,
    pub repository: Option<String>,
    pub branch: Option<String>,
    pub commit_hash: Option<String>,
    pub user_agent: Option<String>,
    pub parent_process: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InteractionPattern {
    pub pattern_type: String,
    pub confidence: f32,
    pub indicators: Vec<String>,
    pub automation_likelihood: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AutomationRecommendations {
    pub suggested_output_format: OutputFormatRecommendation,
    pub ui_mode_preference: UiModeRecommendation,
    pub behavior_adjustments: Vec<BehaviorAdjustment>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum OutputFormatRecommendation {
    Json,    // Machine-readable structured data
    Minimal, // Minimal text output for scripts
    Table,   // Human-readable table format
    Auto,    // Context-appropriate selection
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum UiModeRecommendation {
    ForceCliOnly,     // Never suggest TUI
    AllowInteractive, // TUI suggestions allowed
    PreferMinimal,    // Minimal output preferred
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BehaviorAdjustment {
    pub setting: String,
    pub recommended_value: String,
    pub reason: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AutomationEvidence {
    pub evidence_type: EvidenceType,
    pub source: String,
    pub description: String,
    pub confidence_weight: f32,
}

#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum EvidenceType {
    EnvironmentVariable,
    ProcessHierarchy,
    FileSystemIndicator,
    NetworkPattern,
    UserAgent,
    TerminalCharacteristics,
}

#[derive(Debug, Clone, Hash, PartialEq, Eq)]
struct ContextKey {
    tty_status: bool,
    piped_status: bool,
    scripted_status: bool,
    ci_env: String,
    shell_type: String,
    // Hash of environment variables to detect changes
    env_hash: u64,
}

impl ContextKey {
    fn from_execution_context(context: &ExecutionContext) -> Self {
        use std::collections::hash_map::DefaultHasher;
        use std::hash::{Hash, Hasher};

        let mut hasher = DefaultHasher::new();
        for (key, value) in &context.environment.relevant_env_vars {
            key.hash(&mut hasher);
            value.hash(&mut hasher);
        }
        let env_hash = hasher.finish();

        Self {
            tty_status: context.is_tty,
            piped_status: context.is_piped,
            scripted_status: context.is_scripted,
            ci_env: context
                .environment
                .ci_environment
                .as_ref()
                .map(|ci| format!("{ci:?}"))
                .unwrap_or_default(),
            shell_type: context.environment.shell_type.clone().unwrap_or_default(),
            env_hash,
        }
    }
}

impl Default for AutomationDetector {
    fn default() -> Self {
        Self::new()
    }
}

impl AutomationDetector {
    pub fn new() -> Self {
        let mut detector = Self {
            ci_detectors: Vec::new(),
            script_detectors: Vec::new(),
            machine_detectors: Vec::new(),
            pattern_analyzer: AutomationPatternAnalyzer::new(),
            confidence_calculator: ConfidenceCalculator::new(),
            detection_cache: LruCache::new(NonZeroUsize::new(500).unwrap()),
        };

        detector.register_default_detectors();
        detector
    }

    pub fn detect_automation_context(&mut self, context: &ExecutionContext) -> AutomationResult {
        let cache_key = ContextKey::from_execution_context(context);

        // Check cache first
        if let Some(cached_result) = self.detection_cache.get(&cache_key) {
            return cached_result.clone();
        }

        let mut evidence = Vec::new();
        let env_vars = &context.environment.relevant_env_vars;

        // Run CI/CD detection
        let ci_results = self.detect_ci_environment(env_vars, &mut evidence);

        // Run script context detection
        let script_results = self.detect_script_context(context, &mut evidence);

        // Run machine interaction detection
        let machine_results = self.detect_machine_interaction(context, &mut evidence);

        // Analyze interaction patterns
        let interaction_patterns = self.pattern_analyzer.analyze_patterns(context, &evidence);

        // Determine automation type
        let mut automation_type = self.determine_automation_type(
            &ci_results,
            &script_results,
            &machine_results,
            &interaction_patterns,
        );

        if !script_results.is_empty() {
            automation_type = match script_results[0].script_type.as_str() {
                "Docker Container" => AutomationType::ContainerizedWorkflow,
                "Python Test Framework" => AutomationType::TestingFramework,
                "Python Script" | "Node Script" | "Bash Script" | "PowerShell Script" => {
                    AutomationType::ScriptedExecution
                }
                _ => automation_type,
            };
        }

        // Calculate confidence
        let confidence = self.confidence_calculator.calculate_confidence(
            &automation_type,
            &evidence,
            &interaction_patterns,
        );

        // Generate recommendations
        let recommendations = self.generate_recommendations(&automation_type, context);

        // Detect specific environment details
        let detected_environment =
            self.detect_environment_details(&automation_type, &evidence, env_vars);

        let result = AutomationResult {
            is_automated: !matches!(automation_type, AutomationType::Interactive),
            automation_type,
            confidence,
            detected_environment,
            interaction_patterns,
            recommendations,
            evidence,
            timestamp: Utc::now(),
        };

        // Cache the result
        self.detection_cache.put(cache_key, result.clone());

        result
    }

    fn register_default_detectors(&mut self) {
        // CI/CD Environment Detectors
        self.ci_detectors.push(Box::new(GitHubActionsDetector));
        self.ci_detectors.push(Box::new(GitLabCiDetector));
        self.ci_detectors.push(Box::new(JenkinsDetector));
        self.ci_detectors.push(Box::new(CircleCiDetector));
        self.ci_detectors.push(Box::new(TravisCiDetector));
        self.ci_detectors.push(Box::new(BuildkiteDetector));
        self.ci_detectors.push(Box::new(AzureDevOpsDetector));
        self.ci_detectors.push(Box::new(TeamCityDetector));

        // Script Context Detectors
        self.script_detectors.push(Box::new(BashScriptDetector));
        self.script_detectors.push(Box::new(PythonScriptDetector));
        self.script_detectors.push(Box::new(NodeScriptDetector));
        self.script_detectors
            .push(Box::new(PowerShellScriptDetector));
        self.script_detectors
            .push(Box::new(DockerContainerDetector));

        // Machine Interaction Detectors
        self.machine_detectors.push(Box::new(ApiClientDetector));
        self.machine_detectors.push(Box::new(CurlDetector));
        self.machine_detectors.push(Box::new(WgetDetector));
        self.machine_detectors
            .push(Box::new(AutomationToolDetector));
    }

    fn detect_ci_environment(
        &self,
        env: &std::collections::HashMap<String, String>,
        evidence: &mut Vec<AutomationEvidence>,
    ) -> Vec<CiDetectionResult> {
        let mut results = Vec::new();

        for detector in &self.ci_detectors {
            if let Some(result) = detector.detect_ci_environment(env) {
                // Add evidence from CI detection
                for evidence_item in &result.evidence {
                    evidence.push(AutomationEvidence {
                        evidence_type: EvidenceType::EnvironmentVariable,
                        source: detector.platform_name().to_string(),
                        description: evidence_item.clone(),
                        confidence_weight: result.confidence * 0.8,
                    });
                }

                results.push(result);
            }
        }

        // Sort by confidence and priority
        results.sort_by(|a, b| {
            b.confidence
                .partial_cmp(&a.confidence)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        results
    }

    fn detect_script_context(
        &self,
        context: &ExecutionContext,
        evidence: &mut Vec<AutomationEvidence>,
    ) -> Vec<ScriptDetectionResult> {
        let mut results = Vec::new();

        for detector in &self.script_detectors {
            if let Some(result) = detector.detect_script_context(context) {
                // Add evidence from script detection
                for indicator in &result.indicators {
                    evidence.push(AutomationEvidence {
                        evidence_type: EvidenceType::ProcessHierarchy,
                        source: detector.script_type().to_string(),
                        description: indicator.clone(),
                        confidence_weight: result.confidence * 0.6,
                    });
                }

                results.push(result);
            }
        }

        // Sort by confidence
        results.sort_by(|a, b| {
            b.confidence
                .partial_cmp(&a.confidence)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        results
    }

    fn detect_machine_interaction(
        &self,
        context: &ExecutionContext,
        evidence: &mut Vec<AutomationEvidence>,
    ) -> Vec<MachineInteractionResult> {
        let mut results = Vec::new();

        for detector in &self.machine_detectors {
            if let Some(result) = detector.detect_machine_interaction(context) {
                // Add evidence from machine interaction detection
                for indicator in &result.api_indicators {
                    evidence.push(AutomationEvidence {
                        evidence_type: EvidenceType::UserAgent,
                        source: detector.interaction_type().to_string(),
                        description: indicator.clone(),
                        confidence_weight: result.confidence * 0.7,
                    });
                }

                results.push(result);
            }
        }

        // Sort by confidence
        results.sort_by(|a, b| {
            b.confidence
                .partial_cmp(&a.confidence)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        results
    }

    fn determine_automation_type(
        &self,
        ci_results: &[CiDetectionResult],
        script_results: &[ScriptDetectionResult],
        machine_results: &[MachineInteractionResult],
        interaction_patterns: &[InteractionPattern],
    ) -> AutomationType {
        // CI/CD has highest priority
        if !ci_results.is_empty() {
            return AutomationType::CiCdPipeline;
        }

        // Check script contexts
        for script_result in script_results {
            match script_result.script_type.as_str() {
                "Docker Container" => return AutomationType::ContainerizedWorkflow,
                "Python Script" | "Node Script" => {
                    // Could be testing framework
                    if script_result
                        .indicators
                        .iter()
                        .any(|i| i.contains("test") || i.contains("pytest") || i.contains("jest"))
                    {
                        return AutomationType::TestingFramework;
                    }
                    return AutomationType::ScriptedExecution;
                }
                "Bash Script" | "PowerShell Script" => return AutomationType::ScriptedExecution,
                _ => {}
            }
        }

        // Check for API/machine interaction
        for machine_result in machine_results {
            match machine_result.interaction_type.as_str() {
                "API Client" => return AutomationType::ApiIntegration,
                "Automation Tool" => return AutomationType::ContainerizedWorkflow,
                _ => {}
            }
        }

        // Check interaction patterns for monitoring/scheduled tasks
        for pattern in interaction_patterns {
            if pattern.pattern_type.contains("monitoring") {
                return AutomationType::MonitoringSystem;
            }
            if pattern.pattern_type.contains("scheduled") {
                return AutomationType::ScheduledTask;
            }
        }

        // Default to interactive if no automation indicators
        AutomationType::Interactive
    }

    fn generate_recommendations(
        &self,
        automation_type: &AutomationType,
        _context: &ExecutionContext,
    ) -> AutomationRecommendations {
        match automation_type {
            AutomationType::Interactive => AutomationRecommendations {
                suggested_output_format: OutputFormatRecommendation::Auto,
                ui_mode_preference: UiModeRecommendation::AllowInteractive,
                behavior_adjustments: vec![],
            },

            AutomationType::CiCdPipeline => AutomationRecommendations {
                suggested_output_format: OutputFormatRecommendation::Minimal,
                ui_mode_preference: UiModeRecommendation::ForceCliOnly,
                behavior_adjustments: vec![
                    BehaviorAdjustment {
                        setting: "no_hints".to_string(),
                        recommended_value: "true".to_string(),
                        reason: "CI environments should suppress interactive hints".to_string(),
                    },
                    BehaviorAdjustment {
                        setting: "progress_indicators".to_string(),
                        recommended_value: "minimal".to_string(),
                        reason: "CI logs should be clean and parseable".to_string(),
                    },
                ],
            },

            AutomationType::ApiIntegration => AutomationRecommendations {
                suggested_output_format: OutputFormatRecommendation::Json,
                ui_mode_preference: UiModeRecommendation::ForceCliOnly,
                behavior_adjustments: vec![BehaviorAdjustment {
                    setting: "output_format".to_string(),
                    recommended_value: "json".to_string(),
                    reason: "API clients expect structured data".to_string(),
                }],
            },

            AutomationType::ScriptedExecution => AutomationRecommendations {
                suggested_output_format: OutputFormatRecommendation::Minimal,
                ui_mode_preference: UiModeRecommendation::ForceCliOnly,
                behavior_adjustments: vec![BehaviorAdjustment {
                    setting: "no_hints".to_string(),
                    recommended_value: "true".to_string(),
                    reason: "Scripts should have predictable output".to_string(),
                }],
            },

            _ => AutomationRecommendations {
                suggested_output_format: OutputFormatRecommendation::Minimal,
                ui_mode_preference: UiModeRecommendation::PreferMinimal,
                behavior_adjustments: vec![],
            },
        }
    }

    fn detect_environment_details(
        &self,
        automation_type: &AutomationType,
        evidence: &[AutomationEvidence],
        env: &std::collections::HashMap<String, String>,
    ) -> Option<DetectedEnvironment> {
        match automation_type {
            AutomationType::CiCdPipeline => {
                // Extract CI/CD specific information from evidence
                let mut platform = String::new();
                let mut best_weight = f32::MIN;
                let mut build_id = None;
                let mut repository = None;
                let mut branch = None;
                let mut commit_hash = None;

                for evidence_item in evidence {
                    if evidence_item.evidence_type == EvidenceType::EnvironmentVariable {
                        if evidence_item.confidence_weight > best_weight {
                            platform = evidence_item.source.clone();
                            best_weight = evidence_item.confidence_weight;
                        }

                        // Extract specific details based on the description
                        if evidence_item.description.contains("GITHUB_RUN_ID") {
                            build_id = self.extract_env_value(
                                env,
                                &evidence_item.description,
                                "GITHUB_RUN_ID",
                            );
                        }
                        if evidence_item.description.contains("GITHUB_REPOSITORY") {
                            repository = self.extract_env_value(
                                env,
                                &evidence_item.description,
                                "GITHUB_REPOSITORY",
                            );
                        }
                        if evidence_item.description.contains("GITHUB_REF_NAME") {
                            branch = self.extract_env_value(
                                env,
                                &evidence_item.description,
                                "GITHUB_REF_NAME",
                            );
                        }
                        if evidence_item.description.contains("GITHUB_SHA") {
                            commit_hash = self.extract_env_value(
                                env,
                                &evidence_item.description,
                                "GITHUB_SHA",
                            );
                        }
                    }
                }

                Some(DetectedEnvironment {
                    platform,
                    version: None,
                    build_id,
                    repository,
                    branch,
                    commit_hash,
                    user_agent: None,
                    parent_process: None,
                })
            }

            AutomationType::ApiIntegration => {
                let user_agent = evidence
                    .iter()
                    .find(|e| e.evidence_type == EvidenceType::UserAgent)
                    .map(|e| e.description.clone());

                Some(DetectedEnvironment {
                    platform: "API Client".to_string(),
                    version: None,
                    build_id: None,
                    repository: None,
                    branch: None,
                    commit_hash: None,
                    user_agent,
                    parent_process: None,
                })
            }

            _ => None,
        }
    }

    fn extract_env_value(
        &self,
        env: &std::collections::HashMap<String, String>,
        description: &str,
        env_var: &str,
    ) -> Option<String> {
        if description.contains(env_var) {
            env.get(env_var).cloned()
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cli::context::{ExecutionContext, ExecutionEnvironment};

    #[test]
    fn test_automation_detector_creation() {
        let detector = AutomationDetector::new();
        assert!(!detector.ci_detectors.is_empty());
        assert!(!detector.script_detectors.is_empty());
        assert!(!detector.machine_detectors.is_empty());
    }

    #[test]
    fn test_interactive_detection() {
        let mut detector = AutomationDetector::new();
        let context = ExecutionContext {
            is_tty: true,
            is_piped: false,
            is_scripted: false,
            environment: ExecutionEnvironment {
                is_ci: false,
                is_automation: false,
                ci_environment: None,
                shell_type: Some("bash".to_string()),
                relevant_env_vars: std::collections::HashMap::new(),
            },
            ..ExecutionContext::detect_basic()
        };

        let result = detector.detect_automation_context(&context);
        assert_eq!(result.automation_type, AutomationType::Interactive);
        assert!(!result.is_automated);
    }

    #[test]
    fn test_context_key_generation() {
        let context = ExecutionContext::detect_basic();
        let key1 = ContextKey::from_execution_context(&context);
        let key2 = ContextKey::from_execution_context(&context);
        assert_eq!(key1, key2);
    }
}
