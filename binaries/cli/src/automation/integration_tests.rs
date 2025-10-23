//! Integration Tests for Automation Detection System
//! 
//! This module contains comprehensive integration tests for the automation detection
//! system to ensure all components work together correctly.

#[cfg(test)]
mod tests {
    use super::super::*;
    use crate::cli::context::{ExecutionContext, ExecutionEnvironment, CiEnvironment};
    use std::{collections::HashMap, env};
    
    // Helper for safe environment variable operations in tests
    fn safe_set_var(key: &str, value: &str) {
        unsafe { env::set_var(key, value) };
    }
    
    fn safe_remove_var(key: &str) {
        unsafe { env::remove_var(key) };
    }

    /// Test end-to-end automation detection for GitHub Actions
    #[test]
    fn test_github_actions_end_to_end() {
        // Set up GitHub Actions environment
        safe_set_var("GITHUB_ACTIONS", "true");
        safe_set_var("GITHUB_WORKFLOW", "CI");
        safe_set_var("GITHUB_RUN_ID", "123456789");
        safe_set_var("GITHUB_REPOSITORY", "user/repo");
        safe_set_var("GITHUB_REF_NAME", "main");
        safe_set_var("GITHUB_SHA", "abc123def456");

        // Create context and run detection
        let context = ExecutionContext::detect_basic().with_automation_detection();
        
        // Verify automation detection
        assert!(context.is_automated_context());
        assert_eq!(context.get_automation_type(), AutomationType::CiCdPipeline);
        
        let automation_result = context.automation_result.as_ref().unwrap();
        assert!(automation_result.is_automated);
        assert!(automation_result.confidence > 0.8);
        assert_eq!(automation_result.automation_type, AutomationType::CiCdPipeline);
        
        // Verify detected environment details
        if let Some(detected_env) = &automation_result.detected_environment {
            assert_eq!(detected_env.platform, "GitHub Actions");
            assert!(detected_env.build_id.is_some());
            assert!(detected_env.repository.is_some());
        }
        
        // Clean up
        safe_remove_var("GITHUB_ACTIONS");
        safe_remove_var("GITHUB_WORKFLOW");
        safe_remove_var("GITHUB_RUN_ID");
        safe_remove_var("GITHUB_REPOSITORY");
        safe_remove_var("GITHUB_REF_NAME");
        safe_remove_var("GITHUB_SHA");
    }
    
    /// Test API client detection scenario
    #[test]
    fn test_api_client_detection() {
        // Set up API client environment
        safe_set_var("HTTP_USER_AGENT", "curl/7.68.0");
        safe_set_var("API_KEY", "test-api-key");
        safe_set_var("_", "/usr/bin/curl");
        
        let mut context = ExecutionContext::detect_basic();
        context.is_tty = false;
        context.is_piped = true;
        context = context.with_automation_detection();
        
        // Verify API integration detection
        assert!(context.is_automated_context());
        assert_eq!(context.get_automation_type(), AutomationType::ApiIntegration);
        
        let automation_result = context.automation_result.as_ref().unwrap();
        assert!(automation_result.is_automated);
        assert!(automation_result.confidence > 0.6);
        
        // Check for curl-specific evidence
        let has_curl_evidence = automation_result.evidence.iter()
            .any(|e| e.description.contains("curl"));
        assert!(has_curl_evidence);
        
        // Clean up
        safe_remove_var("HTTP_USER_AGENT");
        safe_remove_var("API_KEY");
        safe_remove_var("_");
    }
    
    /// Test Docker container detection
    #[test]
    fn test_docker_container_detection() {
        // This test would need actual container environment to fully test
        // We'll test the detection logic instead
        
        let mut detector = AutomationDetector::new();
        let context = ExecutionContext::detect_basic();
        
        // In a real container, this would detect container indicators
        let result = detector.detect_automation_context(&context);
        
        // For non-container environment, should detect as interactive
        assert_eq!(result.automation_type, AutomationType::Interactive);
        assert!(!result.is_automated);
    }
    
    /// Test Python script execution detection
    #[test]
    fn test_python_script_detection() {
        // Set up Python script environment
        safe_set_var("PYTHONPATH", "/usr/lib/python3.9");
        safe_set_var("VIRTUAL_ENV", "/home/user/venv");
        safe_set_var("_", "/usr/bin/python3");
        
        let mut context = ExecutionContext::detect_basic();
        context.is_tty = false;
        context.is_scripted = true;
        context = context.with_automation_detection();
        
        // Verify script execution detection
        assert!(context.is_automated_context());
        assert_eq!(context.get_automation_type(), AutomationType::ScriptedExecution);
        
        let automation_result = context.automation_result.as_ref().unwrap();
        assert!(automation_result.confidence > 0.4);
        
        // Check for Python-specific evidence
        let has_python_evidence = automation_result.evidence.iter()
            .any(|e| e.source.contains("Python"));
        assert!(has_python_evidence);
        
        // Clean up
        safe_remove_var("PYTHONPATH");
        safe_remove_var("VIRTUAL_ENV");
        safe_remove_var("_");
    }
    
    /// Test testing framework detection
    #[test]
    fn test_pytest_detection() {
        // Set up pytest environment
        safe_set_var("PYTEST_CURRENT_TEST", "test_example.py::test_function");
        safe_set_var("PYTHONPATH", "/usr/lib/python3.9");
        
        let context = ExecutionContext::detect_basic().with_automation_detection();
        
        // Verify testing framework detection
        assert!(context.is_automated_context());
        assert_eq!(context.get_automation_type(), AutomationType::TestingFramework);
        
        let automation_result = context.automation_result.as_ref().unwrap();
        assert!(automation_result.confidence > 0.6);
        
        // Clean up
        safe_remove_var("PYTEST_CURRENT_TEST");
        safe_remove_var("PYTHONPATH");
    }
    
    /// Test interactive context detection
    #[test]
    fn test_interactive_detection() {
        // Clean environment to simulate interactive use
        let original_vars: Vec<_> = ["CI", "GITHUB_ACTIONS", "GITLAB_CI", "JENKINS_URL"]
            .iter()
            .filter_map(|var| env::var(var).ok().map(|val| (*var, val)))
            .collect();
        
        // Remove CI variables
        for (var, _) in &original_vars {
            safe_remove_var(var);
        }
        
        let mut context = ExecutionContext::detect_basic();
        context.is_tty = true;
        context.is_piped = false;
        context.is_scripted = false;
        context.environment.is_ci = false;
        context.environment.is_automation = false;
        context = context.with_automation_detection();
        
        // Verify interactive detection
        assert!(!context.is_automated_context());
        assert_eq!(context.get_automation_type(), AutomationType::Interactive);
        
        let automation_result = context.automation_result.as_ref().unwrap();
        assert!(!automation_result.is_automated);
        
        // Restore original environment
        for (var, val) in original_vars {
            safe_set_var(var, &val);
        }
    }
    
    /// Test automation tool detection (Ansible)
    #[test]
    fn test_ansible_detection() {
        // Set up Ansible environment
        safe_set_var("ANSIBLE_INVENTORY", "/etc/ansible/hosts");
        safe_set_var("ANSIBLE_HOST_KEY_CHECKING", "False");
        
        let context = ExecutionContext::detect_basic().with_automation_detection();
        
        // Verify automation tool detection
        assert!(context.is_automated_context());
        assert_eq!(context.get_automation_type(), AutomationType::ContainerizedWorkflow);
        
        let automation_result = context.automation_result.as_ref().unwrap();
        assert!(automation_result.confidence > 0.6);
        
        // Check for Ansible-specific evidence
        let has_ansible_evidence = automation_result.evidence.iter()
            .any(|e| e.description.contains("Ansible"));
        assert!(has_ansible_evidence);
        
        // Clean up
        safe_remove_var("ANSIBLE_INVENTORY");
        safe_remove_var("ANSIBLE_HOST_KEY_CHECKING");
    }
    
    /// Test confidence calculation accuracy
    #[test]
    fn test_confidence_calculation() {
        let mut detector = AutomationDetector::new();
        
        // Test high-confidence CI scenario
        safe_set_var("GITHUB_ACTIONS", "true");
        safe_set_var("GITHUB_WORKFLOW", "CI");
        safe_set_var("GITHUB_RUN_ID", "123");
        
        let context = ExecutionContext::detect_basic();
        let result = detector.detect_automation_context(&context);
        
        // Should have very high confidence for CI
        assert!(result.confidence > 0.8);
        assert!(result.is_automated);
        
        // Clean up
        safe_remove_var("GITHUB_ACTIONS");
        safe_remove_var("GITHUB_WORKFLOW");
        safe_remove_var("GITHUB_RUN_ID");
        
        // Test low-confidence scenario
        let basic_context = ExecutionContext::detect_basic();
        let basic_result = detector.detect_automation_context(&basic_context);
        
        // Should have lower confidence for ambiguous context
        assert!(basic_result.confidence < 0.5);
    }
    
    /// Test pattern analysis integration
    #[test]
    fn test_pattern_analysis() {
        // Set up high-frequency automation scenario
        safe_set_var("CI", "true");
        safe_set_var("TERM", "dumb");
        
        let mut context = ExecutionContext::detect_basic();
        context.is_tty = false;
        context.is_piped = true;
        context = context.with_automation_detection();
        
        let automation_result = context.automation_result.as_ref().unwrap();
        
        // Should detect automation patterns
        assert!(!automation_result.interaction_patterns.is_empty());
        
        // Check for non-interactive pattern
        let has_non_interactive_pattern = automation_result.interaction_patterns.iter()
            .any(|p| p.pattern_type.contains("Non-Interactive"));
        assert!(has_non_interactive_pattern);
        
        // Clean up
        safe_remove_var("CI");
        safe_remove_var("TERM");
    }
    
    /// Test recommendations generation
    #[test]
    fn test_automation_recommendations() {
        // Test CI environment recommendations
        safe_set_var("GITHUB_ACTIONS", "true");
        
        let context = ExecutionContext::detect_basic().with_automation_detection();
        let automation_result = context.automation_result.as_ref().unwrap();
        
        // Should recommend minimal output for CI
        assert!(matches!(
            automation_result.recommendations.suggested_output_format,
            detector::OutputFormatRecommendation::Minimal
        ));
        
        // Should force CLI-only mode
        assert!(matches!(
            automation_result.recommendations.ui_mode_preference,
            detector::UiModeRecommendation::ForceCliOnly
        ));
        
        // Should have behavior adjustments
        assert!(!automation_result.recommendations.behavior_adjustments.is_empty());
        
        // Clean up
        safe_remove_var("GITHUB_ACTIONS");
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
        safe_set_var("CI", "true");
        safe_set_var("GITHUB_ACTIONS", "true");
        safe_set_var("GITLAB_CI", "true");
        
        let context = ExecutionContext::detect_basic().with_automation_detection();
        let automation_result = context.automation_result.as_ref().unwrap();
        
        // Should detect CI/CD pipeline
        assert_eq!(automation_result.automation_type, AutomationType::CiCdPipeline);
        
        // Should prioritize GitHub Actions (higher priority)
        if let Some(detected_env) = &automation_result.detected_environment {
            assert_eq!(detected_env.platform, "GitHub Actions");
        }
        
        // Clean up
        safe_remove_var("CI");
        safe_remove_var("GITHUB_ACTIONS");
        safe_remove_var("GITLAB_CI");
    }
}