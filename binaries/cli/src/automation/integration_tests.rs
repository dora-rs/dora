//! Integration Tests for Automation Detection System
//!
//! This module contains comprehensive integration tests for the automation detection
//! system to ensure all components work together correctly.

#[cfg(test)]
mod tests {
    use super::super::*;
    use crate::cli::context::{ExecutionContext, ExecutionEnvironment};

    fn base_context() -> ExecutionContext {
        let mut context = ExecutionContext::detect_basic();
        context.environment.relevant_env_vars.clear();
        context.environment.is_ci = false;
        context.environment.is_automation = false;
        context.environment.ci_environment = None;
        context
    }

    fn apply_env(context: &mut ExecutionContext, entries: &[(&str, &str)]) {
        for (key, value) in entries {
            context
                .environment
                .relevant_env_vars
                .insert((*key).to_string(), (*value).to_string());
        }

        context.environment.ci_environment =
            ExecutionEnvironment::detect_ci_environment(&context.environment.relevant_env_vars);
        context.environment.is_ci = context.environment.ci_environment.is_some();
        context.environment.is_automation =
            context.environment.is_ci || !context.environment.relevant_env_vars.is_empty();
    }

    /// Test end-to-end automation detection for GitHub Actions
    #[test]
    fn test_github_actions_end_to_end() {
        let mut context = base_context();
        apply_env(
            &mut context,
            &[
                ("GITHUB_ACTIONS", "true"),
                ("GITHUB_WORKFLOW", "CI"),
                ("GITHUB_RUN_ID", "123456789"),
                ("GITHUB_REPOSITORY", "user/repo"),
                ("GITHUB_REF_NAME", "main"),
                ("GITHUB_SHA", "abc123def456"),
            ],
        );

        let context = context.with_automation_detection();

        // Verify automation detection
        assert!(context.is_automated_context());
        assert_eq!(context.get_automation_type(), AutomationType::CiCdPipeline);

        let automation_result = context.automation_result.as_ref().unwrap();
        assert!(automation_result.is_automated);
        assert!(automation_result.confidence > 0.8);
        assert_eq!(
            automation_result.automation_type,
            AutomationType::CiCdPipeline
        );

        // Verify detected environment details
        if let Some(detected_env) = &automation_result.detected_environment {
            assert_eq!(detected_env.platform, "GitHub Actions");
            assert!(detected_env.build_id.is_some());
            assert!(detected_env.repository.is_some());
        }
    }

    /// Test API client detection scenario
    #[test]
    fn test_api_client_detection() {
        let mut context = base_context();
        apply_env(
            &mut context,
            &[
                ("HTTP_USER_AGENT", "curl/7.68.0"),
                ("API_KEY", "test-api-key"),
                ("_", "/usr/bin/curl"),
            ],
        );
        context.environment.is_automation = true;
        context.is_tty = false;
        context.is_piped = true;
        let context = context.with_automation_detection();

        // Verify API integration detection
        assert!(context.is_automated_context());
        assert_eq!(
            context.get_automation_type(),
            AutomationType::ApiIntegration
        );

        let automation_result = context.automation_result.as_ref().unwrap();
        assert!(automation_result.is_automated);
        assert!(automation_result.confidence > 0.6);

        // Check for curl-specific evidence
        let has_curl_evidence = automation_result
            .evidence
            .iter()
            .any(|e| e.description.contains("curl"));
        assert!(has_curl_evidence);
    }

    /// Test Docker container detection
    #[test]
    fn test_docker_container_detection() {
        // This test would need actual container environment to fully test
        // We'll test the detection logic instead

        let mut automation_detector = AutomationDetector::new();
        let context = base_context();

        // In a real container, this would detect container indicators
        let result = automation_detector.detect_automation_context(&context);

        // For non-container environment, should detect as interactive
        assert_eq!(result.automation_type, AutomationType::Interactive);
        assert!(!result.is_automated);
    }

    /// Test Python script execution detection
    #[test]
    fn test_python_script_detection() {
        let mut context = base_context();
        apply_env(
            &mut context,
            &[
                ("PYTHONPATH", "/usr/lib/python3.9"),
                ("VIRTUAL_ENV", "/home/user/venv"),
                ("_", "/usr/bin/python3"),
            ],
        );
        context.environment.is_automation = true;
        context.is_tty = false;
        context.is_scripted = true;
        let context = context.with_automation_detection();

        // Verify script execution detection
        assert!(context.is_automated_context());
        assert_eq!(
            context.get_automation_type(),
            AutomationType::ScriptedExecution
        );

        let automation_result = context.automation_result.as_ref().unwrap();
        assert!(automation_result.confidence > 0.4);

        // Check for Python-specific evidence
        let has_python_evidence = automation_result
            .evidence
            .iter()
            .any(|e| e.source.contains("Python"));
        assert!(has_python_evidence);
    }

    /// Test testing framework detection
    #[test]
    fn test_pytest_detection() {
        let mut context = base_context();
        apply_env(
            &mut context,
            &[
                ("PYTEST_CURRENT_TEST", "test_example.py::test_function"),
                ("PYTHONPATH", "/usr/lib/python3.9"),
            ],
        );
        context.environment.is_automation = true;
        let context = context.with_automation_detection();

        // Verify testing framework detection
        assert!(context.is_automated_context());
        assert_eq!(
            context.get_automation_type(),
            AutomationType::TestingFramework
        );

        let automation_result = context.automation_result.as_ref().unwrap();
        assert!(automation_result.confidence > 0.6);
    }

    /// Test interactive context detection
    #[test]
    fn test_interactive_detection() {
        let mut context = base_context();
        context.is_tty = true;
        context.is_piped = false;
        context.is_scripted = false;
        context.environment.is_ci = false;
        context.environment.is_automation = false;
        let context = context.with_automation_detection();

        // Verify interactive detection
        assert!(!context.is_automated_context());
        assert_eq!(context.get_automation_type(), AutomationType::Interactive);

        let automation_result = context.automation_result.as_ref().unwrap();
        assert!(!automation_result.is_automated);
    }

    /// Test automation tool detection (Ansible)
    #[test]
    fn test_ansible_detection() {
        let mut context = base_context();
        apply_env(
            &mut context,
            &[
                ("ANSIBLE_INVENTORY", "/etc/ansible/hosts"),
                ("ANSIBLE_HOST_KEY_CHECKING", "False"),
            ],
        );
        context.environment.is_automation = true;
        let context = context.with_automation_detection();

        // Verify automation tool detection
        assert!(context.is_automated_context());
        assert_eq!(
            context.get_automation_type(),
            AutomationType::ContainerizedWorkflow
        );

        let automation_result = context.automation_result.as_ref().unwrap();
        assert!(automation_result.confidence > 0.6);

        // Check for Ansible-specific evidence
        let has_ansible_evidence = automation_result
            .evidence
            .iter()
            .any(|e| e.description.contains("Ansible"));
        assert!(has_ansible_evidence);
    }

    /// Test confidence calculation accuracy
    #[test]
    fn test_confidence_calculation() {
        let mut automation_detector = AutomationDetector::new();

        // Test high-confidence CI scenario
        let mut ci_context = base_context();
        apply_env(
            &mut ci_context,
            &[
                ("GITHUB_ACTIONS", "true"),
                ("GITHUB_WORKFLOW", "CI"),
                ("GITHUB_RUN_ID", "123"),
            ],
        );
        let result = automation_detector.detect_automation_context(&ci_context);

        // Should have very high confidence for CI
        assert!(result.confidence > 0.8);
        assert!(result.is_automated);

        // Test low-confidence scenario
        let basic_context = base_context();
        let basic_result = automation_detector.detect_automation_context(&basic_context);

        // Should have lower confidence for ambiguous context
        assert!(basic_result.confidence < 0.5);
    }

    /// Test pattern analysis integration
    #[test]
    fn test_pattern_analysis() {
        // Set up high-frequency automation scenario
        let mut context = base_context();
        apply_env(&mut context, &[("CI", "true"), ("TERM", "dumb")]);
        context.is_tty = false;
        context.is_piped = true;
        let context = context.with_automation_detection();

        let automation_result = context.automation_result.as_ref().unwrap();

        // Should detect automation patterns
        assert!(!automation_result.interaction_patterns.is_empty());

        // Check for non-interactive pattern
        let has_non_interactive_pattern = automation_result
            .interaction_patterns
            .iter()
            .any(|p| p.pattern_type.contains("Non-Interactive"));
        assert!(has_non_interactive_pattern);

        // Clean up
    }

    /// Test recommendations generation
    #[test]
    fn test_automation_recommendations() {
        // Test CI environment recommendations
        let mut ci_context = base_context();
        apply_env(&mut ci_context, &[("GITHUB_ACTIONS", "true")]);
        let ci_context = ci_context.with_automation_detection();
        let automation_result = ci_context.automation_result.as_ref().unwrap();

        assert!(matches!(
            automation_result.recommendations.suggested_output_format,
            detector::OutputFormatRecommendation::Minimal
        ));
        assert!(matches!(
            automation_result.recommendations.ui_mode_preference,
            detector::UiModeRecommendation::ForceCliOnly
        ));
        assert!(
            !automation_result
                .recommendations
                .behavior_adjustments
                .is_empty()
        );

        // Test interactive recommendations
        let mut interactive_context = base_context();
        interactive_context.environment.is_ci = false;
        interactive_context.environment.is_automation = false;
        interactive_context.is_tty = true;
        interactive_context.is_scripted = false;
        let interactive_context = interactive_context.with_automation_detection();
        let interactive_result = interactive_context.automation_result.as_ref().unwrap();

        assert!(matches!(
            interactive_result.recommendations.ui_mode_preference,
            detector::UiModeRecommendation::AllowInteractive
        ));

        // Test automation tool recommendations
        let mut ansible_context = base_context();
        apply_env(
            &mut ansible_context,
            &[("ANSIBLE_INVENTORY", "/etc/ansible/hosts")],
        );
        ansible_context.environment.is_automation = true;
        let ansible_context = ansible_context.with_automation_detection();
        let ansible_result = ansible_context.automation_result.as_ref().unwrap();

        assert!(matches!(
            ansible_result.recommendations.ui_mode_preference,
            detector::UiModeRecommendation::PreferMinimal
        ));
    }

    /// Test caching behavior
    #[test]
    fn test_detection_caching() {
        let mut detector = AutomationDetector::new();
        let context = ExecutionContext::detect_basic();

        // First detection
        let start = std::time::Instant::now();
        let result1 = detector.detect_automation_context(&context);
        let first_duration = start.elapsed();

        // Second detection (should be cached)
        let start = std::time::Instant::now();
        let result2 = detector.detect_automation_context(&context);
        let second_duration = start.elapsed();

        // Results should be identical
        assert_eq!(result1.automation_type, result2.automation_type);
        assert_eq!(result1.confidence, result2.confidence);

        // Second call should be faster (cached)
        assert!(second_duration < first_duration);
    }

    /// Test multiple CI platform detection priority
    #[test]
    fn test_ci_platform_priority() {
        // Set up multiple CI indicators (shouldn't happen in real scenarios)
        let mut context = base_context();
        apply_env(
            &mut context,
            &[
                ("CI", "true"),
                ("GITHUB_ACTIONS", "true"),
                ("GITLAB_CI", "true"),
            ],
        );
        let context = context.with_automation_detection();
        let automation_result = context.automation_result.as_ref().unwrap();

        // Should detect CI/CD pipeline
        assert_eq!(
            automation_result.automation_type,
            AutomationType::CiCdPipeline
        );

        // Should prioritize GitHub Actions (higher priority)
        if let Some(detected_env) = &automation_result.detected_environment {
            assert_eq!(detected_env.platform, "GitHub Actions");
        }
    }
}
