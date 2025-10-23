#[cfg(test)]
mod tests {
    use super::*;
    use crate::cli::{Command, commands::*};
    use crate::cli::context::ExecutionContext;
    use crate::analysis::{
        ComplexityAnalysisEngine, ComplexityAnalysisRequest, SystemState, ResourceTarget,
        ResourceType, UserExpertiseLevel, CommandComplexityAnalyzer, DataflowComplexityAnalyzer,
        SystemComplexityAnalyzer, ComplexityMLModel, ComplexityExplainer, MLInput,
    };
    use std::collections::HashMap;
    use std::path::PathBuf;
    use tempfile::TempDir;
    use tokio;

    /// Create a test execution context
    fn create_test_context() -> ExecutionContext {
        let mut context = ExecutionContext::detect_basic();
        context.is_tty = true;
        context.is_piped = false;
        context.is_scripted = false;
        context.terminal_size = Some((120, 40));
        context.terminal_capabilities.tui_capable = true;
        context.terminal_capabilities.supports_color = true;
        context
    }

    /// Create a test system state
    fn create_test_system_state() -> SystemState {
        SystemState {
            active_dataflows: 3,
            system_load: 0.5,
            memory_usage: 0.6,
            network_activity: 0.3,
            error_rate: 0.02,
            last_updated: chrono::Utc::now(),
        }
    }

    /// Create a test analysis request
    fn create_test_request(command: Command) -> ComplexityAnalysisRequest {
        ComplexityAnalysisRequest {
            command,
            context: create_test_context(),
            target_resource: None,
            system_state: create_test_system_state(),
            user_expertise: UserExpertiseLevel::Intermediate,
        }
    }

    #[test]
    fn test_complexity_analysis_engine_creation() {
        let engine = ComplexityAnalysisEngine::new();
        let (cache_used, cache_capacity) = engine.cache_stats();
        
        assert_eq!(cache_used, 0);
        assert_eq!(cache_capacity, 1000);
    }

    #[tokio::test]
    async fn test_simple_command_complexity_analysis() {
        let engine = ComplexityAnalysisEngine::new();
        let ps_command = Command::Ps(PsCommand::default());
        let request = create_test_request(ps_command);
        
        let result = engine.analyze_complexity(&request).await.unwrap();
        
        // PS command should have low complexity
        assert!(result.overall_score < 5.0);
        assert!(result.confidence > 0.5);
        assert!(!result.explanation.summary.is_empty());
        assert!(!result.recommendations.is_empty());
        
        // Should recommend CLI for simple commands
        let primary_recommendation = &result.recommendations[0];
        assert!(matches!(primary_recommendation.interface, UiMode::Cli));
    }

    #[tokio::test]
    async fn test_complex_command_complexity_analysis() {
        let engine = ComplexityAnalysisEngine::new();
        let debug_command = Command::Debug(DebugCommand::default());
        let request = create_test_request(debug_command);
        
        let result = engine.analyze_complexity(&request).await.unwrap();
        
        // Debug command should have high complexity
        assert!(result.overall_score > 7.0);
        assert!(result.confidence > 0.5);
        assert!(!result.explanation.primary_factors.is_empty());
        
        // Should recommend TUI for complex commands
        let primary_recommendation = &result.recommendations[0];
        assert!(matches!(primary_recommendation.interface, UiMode::Tui));
    }

    #[tokio::test]
    async fn test_complexity_analysis_with_resource_target() {
        let engine = ComplexityAnalysisEngine::new();
        let start_command = Command::Start(StartCommand {
            dataflow_path: PathBuf::from("test_dataflow.yml"),
            environment: HashMap::new(),
        });
        
        let mut request = create_test_request(start_command);
        request.target_resource = Some(ResourceTarget {
            resource_type: ResourceType::Dataflow,
            identifier: "complex_dataflow".to_string(),
            metadata: HashMap::new(),
        });
        
        let result = engine.analyze_complexity(&request).await.unwrap();
        
        // Should include data complexity in analysis
        assert!(result.component_scores.data_complexity > 0.0);
        assert!(result.overall_score > 3.0); // Start command with complex dataflow
    }

    #[test]
    fn test_command_complexity_analyzer() {
        let analyzer = CommandComplexityAnalyzer::new();
        
        // Test simple command
        let ps_command = Command::Ps(PsCommand::default());
        let context = create_test_context();
        
        let rt = tokio::runtime::Runtime::new().unwrap();
        let simple_score = rt.block_on(analyzer.analyze(&ps_command, &context)).unwrap();
        
        assert!(simple_score.base_complexity < 3.0);
        assert!(simple_score.normalized_score() < 5.0);
        
        // Test complex command
        let debug_command = Command::Debug(DebugCommand::default());
        let complex_score = rt.block_on(analyzer.analyze(&debug_command, &context)).unwrap();
        
        assert!(complex_score.base_complexity > 6.0);
        assert!(complex_score.normalized_score() > simple_score.normalized_score());
    }

    #[tokio::test]
    async fn test_dataflow_complexity_analyzer() {
        let analyzer = DataflowComplexityAnalyzer::new();
        
        // Test simple dataflow
        let simple_target = ResourceTarget {
            resource_type: ResourceType::Dataflow,
            identifier: "simple_dataflow".to_string(),
            metadata: HashMap::new(),
        };
        
        let simple_score = analyzer.analyze(&simple_target).await.unwrap();
        assert!(simple_score.normalized_score() < 6.0);
        
        // Test complex dataflow
        let complex_target = ResourceTarget {
            resource_type: ResourceType::Dataflow,
            identifier: "complex_dataflow".to_string(),
            metadata: HashMap::new(),
        };
        
        let complex_score = analyzer.analyze(&complex_target).await.unwrap();
        assert!(complex_score.graph_complexity > simple_score.graph_complexity);
    }

    #[tokio::test]
    async fn test_system_complexity_analyzer() {
        let analyzer = SystemComplexityAnalyzer::new();
        
        // Test low load system
        let low_load_state = SystemState {
            active_dataflows: 1,
            system_load: 0.2,
            memory_usage: 0.3,
            network_activity: 0.1,
            error_rate: 0.0,
            last_updated: chrono::Utc::now(),
        };
        
        let low_score = analyzer.analyze(&low_load_state).await.unwrap();
        assert!(low_score.normalized_score() < 4.0);
        
        // Test high load system
        let high_load_state = SystemState {
            active_dataflows: 15,
            system_load: 0.9,
            memory_usage: 0.95,
            network_activity: 0.8,
            error_rate: 0.1,
            last_updated: chrono::Utc::now(),
        };
        
        let high_score = analyzer.analyze(&high_load_state).await.unwrap();
        assert!(high_score.normalized_score() > low_score.normalized_score());
        assert!(high_score.load_complexity > 3.0);
    }

    #[test]
    fn test_ml_model_prediction() {
        let model = ComplexityMLModel::load_or_create();
        
        let input = MLInput {
            command_features: vec![0.3, 0.2, 0.1, 0.4, 0.5, 0.3, 0.2],
            data_features: vec![0.2, 0.3, 0.2, 0.1, 0.25, 0.15],
            system_features: vec![0.4, 0.5, 0.2, 0.3, 0.1, 0.35, 0.2],
            context_features: vec![1.0, 0.0, 0.0, 0.8, 1.0, 1.0, 0.0, 0.5],
        };
        
        let output = model.predict(&input).unwrap();
        
        assert!(output.predicted_complexity >= 0.0 && output.predicted_complexity <= 10.0);
        assert!(output.confidence >= 0.0 && output.confidence <= 1.0);
        assert!(!output.feature_importance.is_empty());
        assert!(!output.prediction_explanation.is_empty());
    }

    #[test]
    fn test_ml_model_training() {
        let mut model = ComplexityMLModel::load_or_create();
        
        let input = MLInput {
            command_features: vec![0.8, 0.7, 0.6, 0.9, 0.8, 0.85, 0.3],
            data_features: vec![0.6, 0.7, 0.5, 0.4, 0.6, 0.25],
            system_features: vec![0.3, 0.4, 0.2, 0.2, 0.1, 0.28, 0.15],
            context_features: vec![1.0, 0.0, 0.0, 0.9, 1.0, 1.0, 0.0, 0.3],
        };
        
        let feedback = UserComplexityFeedback {
            ml_input: input,
            predicted_complexity: 7.5,
            user_perceived_complexity: 8.0,
            satisfaction: crate::config::preferences::SatisfactionLevel::Satisfied,
            chosen_interface: UiMode::Tui,
            feedback_comments: Some("TUI was helpful for this complex operation".to_string()),
        };
        
        let rt = tokio::runtime::Runtime::new().unwrap();
        let result = rt.block_on(model.collect_training_data(feedback));
        assert!(result.is_ok());
    }

    #[test]
    fn test_complexity_explainer() {
        let explainer = ComplexityExplainer::new();
        
        let component_scores = ComponentScores {
            command_complexity: 8.0,
            data_complexity: 6.0,
            interaction_complexity: 9.0,
            output_complexity: 7.0,
            error_complexity: 5.0,
            performance_complexity: 4.0,
        };
        
        let ml_output = MLOutput {
            predicted_complexity: 7.5,
            confidence: 0.85,
            feature_importance: vec![],
            prediction_explanation: "Test explanation".to_string(),
        };
        
        // Test explanation for different user levels
        let beginner_explanation = explainer.explain_complexity(
            &component_scores,
            &ml_output,
            &UserExpertiseLevel::Beginner,
        );
        
        let expert_explanation = explainer.explain_complexity(
            &component_scores,
            &ml_output,
            &UserExpertiseLevel::Expert,
        );
        
        assert!(!beginner_explanation.summary.is_empty());
        assert!(!expert_explanation.summary.is_empty());
        assert!(beginner_explanation.summary != expert_explanation.summary);
        
        // Beginner explanation should be more descriptive
        assert!(beginner_explanation.summary.len() > expert_explanation.summary.len());
    }

    #[test]
    fn test_simplified_explanation() {
        let explainer = ComplexityExplainer::new();
        
        let component_scores = ComponentScores {
            command_complexity: 3.0,
            data_complexity: 2.0,
            interaction_complexity: 4.0,
            output_complexity: 3.5,
            error_complexity: 2.5,
            performance_complexity: 2.0,
        };
        
        let ml_output = MLOutput {
            predicted_complexity: 3.2,
            confidence: 0.75,
            feature_importance: vec![],
            prediction_explanation: "Test explanation".to_string(),
        };
        
        let simplified = explainer.explain_simplified(&component_scores, &ml_output);
        
        assert!(!simplified.overall_assessment.is_empty());
        assert!(!simplified.primary_reason.is_empty());
        assert!(!simplified.recommendation.is_empty());
        assert!(matches!(simplified.confidence_level, ConfidenceLevel::Medium | ConfidenceLevel::High));
    }

    #[tokio::test]
    async fn test_complexity_analysis_caching() {
        let engine = ComplexityAnalysisEngine::new();
        let ps_command = Command::Ps(PsCommand::default());
        let request = create_test_request(ps_command);
        
        // First analysis should populate cache
        let start_time = std::time::Instant::now();
        let result1 = engine.analyze_complexity(&request).await.unwrap();
        let first_duration = start_time.elapsed();
        
        // Second analysis should use cache and be faster
        let start_time = std::time::Instant::now();
        let result2 = engine.analyze_complexity(&request).await.unwrap();
        let second_duration = start_time.elapsed();
        
        // Results should be identical
        assert_eq!(result1.overall_score, result2.overall_score);
        assert_eq!(result1.confidence, result2.confidence);
        
        // Second call should be faster (cache hit)
        // Note: This might not always be true in tests due to system variance
        // but it demonstrates the caching mechanism
        let (cache_used, _) = engine.cache_stats();
        assert!(cache_used > 0);
    }

    #[test]
    fn test_user_expertise_level_serialization() {
        let beginner = UserExpertiseLevel::Beginner;
        let serialized = serde_json::to_string(&beginner).unwrap();
        let deserialized: UserExpertiseLevel = serde_json::from_str(&serialized).unwrap();
        assert_eq!(beginner, deserialized);
    }

    #[test]
    fn test_component_scores_normalization() {
        let scores = ComponentScores {
            command_complexity: 8.0,
            data_complexity: 6.0,
            interaction_complexity: 9.0,
            output_complexity: 7.0,
            error_complexity: 15.0, // Intentionally high
            performance_complexity: 4.0,
        };
        
        // Even with one very high score, normalized should be clamped
        let features = scores.to_features();
        assert!(features.iter().all(|&f| f <= 1.0));
    }

    #[test]
    fn test_factor_type_hash_and_eq() {
        let factor1 = FactorType::CommandComplexity;
        let factor2 = FactorType::CommandComplexity;
        let factor3 = FactorType::OutputComplexity;
        
        assert_eq!(factor1, factor2);
        assert_ne!(factor1, factor3);
        
        let mut map = HashMap::new();
        map.insert(factor1, "test");
        assert!(map.contains_key(&factor2));
        assert!(!map.contains_key(&factor3));
    }

    #[tokio::test]
    async fn test_complexity_analysis_with_different_contexts() {
        let engine = ComplexityAnalysisEngine::new();
        let debug_command = Command::Debug(DebugCommand::default());
        
        // Interactive context
        let mut interactive_request = create_test_request(debug_command.clone());
        interactive_request.context.is_tty = true;
        interactive_request.context.is_scripted = false;
        
        // CI context
        let mut ci_request = create_test_request(debug_command);
        ci_request.context.is_tty = false;
        ci_request.context.is_scripted = true;
        ci_request.context.environment.is_ci = true;
        
        let interactive_result = engine.analyze_complexity(&interactive_request).await.unwrap();
        let ci_result = engine.analyze_complexity(&ci_request).await.unwrap();
        
        // CI context should generally prefer CLI/minimal interfaces
        assert!(interactive_result.recommendations.len() > 0);
        assert!(ci_result.recommendations.len() > 0);
        
        // The specific recommendations may differ based on context
        let interactive_rec = &interactive_result.recommendations[0];
        let ci_rec = &ci_result.recommendations[0];
        
        // In CI, we should see different interface preferences
        assert_ne!(interactive_rec.interface, ci_rec.interface);
    }

    #[test]
    fn test_performance_requirements() {
        let engine = ComplexityAnalysisEngine::new();
        let start_time = std::time::Instant::now();
        
        // Create a simple request
        let ps_command = Command::Ps(PsCommand::default());
        let request = create_test_request(ps_command);
        
        let rt = tokio::runtime::Runtime::new().unwrap();
        let _result = rt.block_on(engine.analyze_complexity(&request)).unwrap();
        
        let duration = start_time.elapsed();
        
        // Should complete within 5ms for 95% of scenarios (as per issue requirements)
        // In tests, we'll be more lenient due to test overhead
        assert!(duration.as_millis() < 50, "Analysis took {}ms, should be under 50ms", duration.as_millis());
    }

    #[test]
    fn test_explanation_quality() {
        let explainer = ComplexityExplainer::new();
        
        let high_complexity_scores = ComponentScores {
            command_complexity: 9.0,
            data_complexity: 8.0,
            interaction_complexity: 9.5,
            output_complexity: 8.5,
            error_complexity: 7.0,
            performance_complexity: 6.0,
        };
        
        let ml_output = MLOutput {
            predicted_complexity: 8.5,
            confidence: 0.9,
            feature_importance: vec![],
            prediction_explanation: "High complexity operation".to_string(),
        };
        
        let explanation = explainer.explain_complexity(
            &high_complexity_scores,
            &ml_output,
            &UserExpertiseLevel::Intermediate,
        );
        
        // Explanation should be actionable and clear
        assert!(!explanation.summary.is_empty());
        assert!(explanation.summary.contains("complex"));
        assert!(!explanation.primary_factors.is_empty());
        
        // Should have multiple factors for high complexity
        assert!(explanation.primary_factors.len() > 1);
        
        // Factors should be properly ranked
        if explanation.primary_factors.len() > 1 {
            assert!(explanation.primary_factors[0].impact >= explanation.primary_factors[1].impact);
        }
    }

    #[tokio::test]
    async fn test_error_handling() {
        let analyzer = CommandComplexityAnalyzer::new();
        
        // Test with a command that might have dependency issues
        let start_command = Command::Start(StartCommand {
            dataflow_path: PathBuf::from("/nonexistent/path/dataflow.yml"),
            environment: HashMap::new(),
        });
        
        let context = create_test_context();
        let result = analyzer.analyze(&start_command, &context).await;
        
        // Should handle missing files gracefully
        assert!(result.is_ok());
        let score = result.unwrap();
        
        // Should still provide a score even with missing dependencies
        assert!(score.normalized_score() > 0.0);
        assert!(!score.factors.is_empty());
    }

    #[test]
    fn test_ml_model_feature_importance() {
        let model = ComplexityMLModel::load_or_create();
        let importance = model.get_feature_importance();
        
        assert!(!importance.is_empty());
        
        // Should have reasonable feature names
        for feature in &importance {
            assert!(!feature.feature_name.is_empty());
            assert!(feature.importance >= 0.0);
        }
    }

    #[test]
    fn test_complexity_thresholds() {
        use crate::analysis::system_complexity::SystemComplexityThresholds;
        
        let thresholds = SystemComplexityThresholds::default();
        
        assert!(thresholds.high_cpu_threshold > 0.0 && thresholds.high_cpu_threshold <= 1.0);
        assert!(thresholds.high_memory_threshold > 0.0 && thresholds.high_memory_threshold <= 1.0);
        assert!(thresholds.high_io_threshold > 0.0 && thresholds.high_io_threshold <= 1.0);
        assert!(thresholds.high_error_rate_threshold >= 0.0);
        assert!(thresholds.many_dataflows_threshold > 0);
    }
}