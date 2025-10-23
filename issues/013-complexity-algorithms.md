# Issue #013: Implement Complexity Calculation Algorithms

## 📋 Summary
Develop sophisticated algorithms to analyze command, dataflow, and system complexity to drive intelligent interface selection decisions. These algorithms form the core intelligence behind the hybrid CLI's ability to suggest the most appropriate interface for each user interaction.

## 🎯 Objectives
- Create comprehensive complexity analysis for commands, dataflows, and system state
- Implement machine learning-based complexity scoring that improves over time
- Add context-aware complexity calculation that considers user expertise and environment
- Provide explainable complexity decisions for transparency and debugging
- Ensure algorithm performance enables real-time decision making

**Success Metrics:**
- Complexity calculation completes in <5ms for 95% of scenarios
- Algorithm accuracy improves by 15% after 1000 user interactions
- Interface suggestions based on complexity are rated helpful by 85% of users
- Complexity explanations are clear and actionable for debugging
- Algorithm handles edge cases gracefully without performance degradation

## 🛠️ Technical Requirements

### What to Build

#### 1. Core Complexity Analysis Engine
```rust
// src/analysis/complexity.rs
#[derive(Debug)]
pub struct ComplexityAnalysisEngine {
    command_analyzer: CommandComplexityAnalyzer,
    dataflow_analyzer: DataflowComplexityAnalyzer,
    system_analyzer: SystemComplexityAnalyzer,
    ml_model: ComplexityMLModel,
    cache: LruCache<ComplexityKey, ComplexityResult>,
}

#[derive(Debug, Clone)]
pub struct ComplexityResult {
    pub overall_score: f32,       // 0.0 - 10.0 scale
    pub component_scores: ComponentScores,
    pub confidence: f32,          // 0.0 - 1.0
    pub explanation: ComplexityExplanation,
    pub recommendations: Vec<InterfaceRecommendation>,
    pub calculated_at: Instant,
}

#[derive(Debug, Clone)]
pub struct ComponentScores {
    pub command_complexity: f32,
    pub data_complexity: f32,
    pub interaction_complexity: f32,
    pub output_complexity: f32,
    pub error_complexity: f32,
    pub performance_complexity: f32,
}

#[derive(Debug, Clone)]
pub struct ComplexityExplanation {
    pub primary_factors: Vec<ComplexityFactor>,
    pub contributing_factors: Vec<ComplexityFactor>,
    pub mitigating_factors: Vec<ComplexityFactor>,
    pub summary: String,
}

#[derive(Debug, Clone)]
pub struct ComplexityFactor {
    pub factor_type: FactorType,
    pub impact: f32,              // -10.0 to +10.0
    pub description: String,
    pub evidence: Vec<String>,
}

impl ComplexityAnalysisEngine {
    pub fn new() -> Self {
        Self {
            command_analyzer: CommandComplexityAnalyzer::new(),
            dataflow_analyzer: DataflowComplexityAnalyzer::new(),
            system_analyzer: SystemComplexityAnalyzer::new(),
            ml_model: ComplexityMLModel::load_or_create(),
            cache: LruCache::new(NonZeroUsize::new(1000).unwrap()),
        }
    }
    
    pub async fn analyze_complexity(
        &mut self,
        request: &ComplexityAnalysisRequest,
    ) -> Result<ComplexityResult> {
        let cache_key = ComplexityKey::from_request(request);
        
        // Check cache first
        if let Some(cached_result) = self.cache.get(&cache_key) {
            if cached_result.calculated_at.elapsed() < Duration::from_secs(60) {
                return Ok(cached_result.clone());
            }
        }
        
        // Analyze components
        let command_score = self.command_analyzer
            .analyze(&request.command, &request.context).await?;
        
        let data_score = if let Some(target) = &request.target_resource {
            self.dataflow_analyzer.analyze(target).await?
        } else {
            DataComplexityScore::default()
        };
        
        let system_score = self.system_analyzer
            .analyze(&request.system_state).await?;
        
        // Combine scores using ML model
        let ml_input = MLInput {
            command_features: command_score.to_features(),
            data_features: data_score.to_features(),
            system_features: system_score.to_features(),
            context_features: request.context.to_features(),
        };
        
        let ml_result = self.ml_model.predict(&ml_input)?;
        
        // Create component scores
        let component_scores = ComponentScores {
            command_complexity: command_score.normalized_score(),
            data_complexity: data_score.normalized_score(),
            interaction_complexity: self.calculate_interaction_complexity(request),
            output_complexity: self.calculate_output_complexity(request),
            error_complexity: self.calculate_error_complexity(request),
            performance_complexity: system_score.normalized_score(),
        };
        
        // Generate explanation
        let explanation = self.generate_explanation(
            &command_score,
            &data_score,
            &system_score,
            &ml_result,
        );
        
        // Create recommendations
        let recommendations = self.generate_recommendations(&component_scores, &ml_result);
        
        let result = ComplexityResult {
            overall_score: ml_result.predicted_complexity,
            component_scores,
            confidence: ml_result.confidence,
            explanation,
            recommendations,
            calculated_at: Instant::now(),
        };
        
        // Cache result
        self.cache.put(cache_key, result.clone());
        
        Ok(result)
    }
}

#[derive(Debug, Clone)]
pub struct ComplexityAnalysisRequest {
    pub command: Command,
    pub context: ExecutionContext,
    pub target_resource: Option<ResourceTarget>,
    pub system_state: SystemState,
    pub user_expertise: UserExpertiseLevel,
}
```

#### 2. Command Complexity Analysis
```rust
// src/analysis/command_complexity.rs
#[derive(Debug)]
pub struct CommandComplexityAnalyzer {
    rule_engine: ComplexityRuleEngine,
    pattern_detector: CommandPatternDetector,
    feature_extractor: CommandFeatureExtractor,
}

#[derive(Debug, Clone)]
pub struct CommandComplexityScore {
    pub base_complexity: f32,
    pub parameter_complexity: f32,
    pub dependency_complexity: f32,
    pub output_complexity: f32,
    pub error_potential: f32,
    pub factors: Vec<ComplexityFactor>,
}

impl CommandComplexityAnalyzer {
    pub async fn analyze(
        &self,
        command: &Command,
        context: &ExecutionContext,
    ) -> Result<CommandComplexityScore> {
        let mut factors = Vec::new();
        
        // Base command complexity
        let base_complexity = self.analyze_base_command_complexity(command, &mut factors);
        
        // Parameter complexity
        let parameter_complexity = self.analyze_parameter_complexity(command, &mut factors);
        
        // Dependency complexity
        let dependency_complexity = self.analyze_dependency_complexity(command, &mut factors).await?;
        
        // Output complexity
        let output_complexity = self.analyze_output_complexity(command, context, &mut factors);
        
        // Error potential
        let error_potential = self.analyze_error_potential(command, &mut factors);
        
        Ok(CommandComplexityScore {
            base_complexity,
            parameter_complexity,
            dependency_complexity,
            output_complexity,
            error_potential,
            factors,
        })
    }
    
    fn analyze_base_command_complexity(
        &self,
        command: &Command,
        factors: &mut Vec<ComplexityFactor>,
    ) -> f32 {
        let base_score = match command {
            Command::Ps(_) => 1.0,
            Command::Start(cmd) => {
                let mut score = 3.0;
                
                if cmd.dataflow_path.to_string_lossy().ends_with(".complex.yml") {
                    score += 2.0;
                    factors.push(ComplexityFactor {
                        factor_type: FactorType::CommandComplexity,
                        impact: 2.0,
                        description: "Complex dataflow configuration detected".to_string(),
                        evidence: vec![format!("File: {}", cmd.dataflow_path.display())],
                    });
                }
                
                if cmd.environment.len() > 5 {
                    score += 1.0;
                    factors.push(ComplexityFactor {
                        factor_type: FactorType::ParameterComplexity,
                        impact: 1.0,
                        description: "Many environment variables specified".to_string(),
                        evidence: vec![format!("{} environment variables", cmd.environment.len())],
                    });
                }
                
                score
            },
            Command::Debug(_) => {
                factors.push(ComplexityFactor {
                    factor_type: FactorType::CommandComplexity,
                    impact: 7.0,
                    description: "Debug commands are inherently complex".to_string(),
                    evidence: vec!["Interactive debugging requires deep system understanding".to_string()],
                });
                7.0
            },
            Command::Analyze(_) => {
                factors.push(ComplexityFactor {
                    factor_type: FactorType::CommandComplexity,
                    impact: 8.0,
                    description: "Analysis commands involve complex data processing".to_string(),
                    evidence: vec!["Large datasets and complex algorithms involved".to_string()],
                });
                8.0
            },
            _ => 2.0,
        };
        
        base_score
    }
    
    async fn analyze_dependency_complexity(
        &self,
        command: &Command,
        factors: &mut Vec<ComplexityFactor>,
    ) -> Result<f32> {
        match command {
            Command::Start(cmd) => {
                let config = self.load_dataflow_config(&cmd.dataflow_path).await?;
                let dependency_count = config.nodes.len();
                let external_deps = config.count_external_dependencies();
                
                let mut score = dependency_count as f32 * 0.5;
                score += external_deps as f32 * 1.0;
                
                if dependency_count > 5 {
                    factors.push(ComplexityFactor {
                        factor_type: FactorType::DependencyComplexity,
                        impact: 2.0,
                        description: "Many nodes in dataflow".to_string(),
                        evidence: vec![format!("{} nodes", dependency_count)],
                    });
                }
                
                if external_deps > 0 {
                    factors.push(ComplexityFactor {
                        factor_type: FactorType::DependencyComplexity,
                        impact: external_deps as f32,
                        description: "External dependencies detected".to_string(),
                        evidence: vec![format!("{} external dependencies", external_deps)],
                    });
                }
                
                Ok(score.min(10.0))
            },
            _ => Ok(0.0),
        }
    }
    
    fn analyze_output_complexity(
        &self,
        command: &Command,
        context: &ExecutionContext,
        factors: &mut Vec<ComplexityFactor>,
    ) -> f32 {
        let mut score = 0.0;
        
        // Check if command produces large amounts of data
        match command {
            Command::Logs(cmd) if cmd.follow => {
                score += 3.0;
                factors.push(ComplexityFactor {
                    factor_type: FactorType::OutputComplexity,
                    impact: 3.0,
                    description: "Streaming log output".to_string(),
                    evidence: vec!["Real-time data streams require careful handling".to_string()],
                });
            },
            Command::Ps(_) => {
                // Dynamic based on expected number of dataflows
                score += 1.0; // Will be refined by system analyzer
            },
            Command::Inspect(cmd) if cmd.live_mode => {
                score += 4.0;
                factors.push(ComplexityFactor {
                    factor_type: FactorType::OutputComplexity,
                    impact: 4.0,
                    description: "Live data monitoring".to_string(),
                    evidence: vec!["Real-time metrics and updates".to_string()],
                });
            },
            _ => {}
        }
        
        // Consider terminal constraints
        if let Some((width, height)) = context.terminal_size {
            if width < 80 || height < 24 {
                score += 1.0;
                factors.push(ComplexityFactor {
                    factor_type: FactorType::OutputComplexity,
                    impact: 1.0,
                    description: "Small terminal constrains output".to_string(),
                    evidence: vec![format!("Terminal size: {}x{}", width, height)],
                });
            }
        }
        
        score
    }
}
```

#### 3. Machine Learning Model Integration
```rust
// src/analysis/ml_model.rs
#[derive(Debug)]
pub struct ComplexityMLModel {
    model: Box<dyn MLModel>,
    training_data: TrainingDataset,
    model_metadata: ModelMetadata,
}

#[derive(Debug, Clone)]
pub struct MLInput {
    pub command_features: Vec<f32>,
    pub data_features: Vec<f32>,
    pub system_features: Vec<f32>,
    pub context_features: Vec<f32>,
}

#[derive(Debug, Clone)]
pub struct MLOutput {
    pub predicted_complexity: f32,
    pub confidence: f32,
    pub feature_importance: Vec<FeatureImportance>,
    pub prediction_explanation: String,
}

trait MLModel: Send + Sync {
    fn predict(&self, input: &MLInput) -> Result<MLOutput>;
    fn update_model(&mut self, training_examples: &[TrainingExample]) -> Result<()>;
    fn get_feature_importance(&self) -> Vec<FeatureImportance>;
}

// Simple linear regression model for start
pub struct LinearRegressionModel {
    weights: Vec<f32>,
    bias: f32,
    feature_names: Vec<String>,
}

impl MLModel for LinearRegressionModel {
    fn predict(&self, input: &MLInput) -> Result<MLOutput> {
        let features = self.flatten_features(input);
        
        if features.len() != self.weights.len() {
            return Err(anyhow!("Feature count mismatch"));
        }
        
        let prediction = features.iter()
            .zip(&self.weights)
            .map(|(feature, weight)| feature * weight)
            .sum::<f32>() + self.bias;
        
        let confidence = self.calculate_confidence(&features);
        let feature_importance = self.calculate_feature_importance(&features);
        
        Ok(MLOutput {
            predicted_complexity: prediction.clamp(0.0, 10.0),
            confidence,
            feature_importance,
            prediction_explanation: self.generate_explanation(&features, prediction),
        })
    }
    
    fn update_model(&mut self, training_examples: &[TrainingExample]) -> Result<()> {
        // Simple gradient descent update
        let learning_rate = 0.01;
        
        for example in training_examples {
            let features = self.flatten_features(&example.input);
            let prediction = self.predict(&example.input)?.predicted_complexity;
            let error = example.actual_complexity - prediction;
            
            // Update weights
            for (i, &feature) in features.iter().enumerate() {
                self.weights[i] += learning_rate * error * feature;
            }
            
            // Update bias
            self.bias += learning_rate * error;
        }
        
        Ok(())
    }
    
    fn get_feature_importance(&self) -> Vec<FeatureImportance> {
        self.weights.iter()
            .enumerate()
            .map(|(i, &weight)| FeatureImportance {
                feature_name: self.feature_names.get(i)
                    .cloned()
                    .unwrap_or_else(|| format!("feature_{}", i)),
                importance: weight.abs(),
                positive_impact: weight > 0.0,
            })
            .collect()
    }
}

impl ComplexityMLModel {
    pub fn load_or_create() -> Self {
        // Try to load existing model, create new if not found
        match Self::load_from_disk() {
            Ok(model) => model,
            Err(_) => Self::create_default_model(),
        }
    }
    
    pub async fn collect_training_data(
        &mut self,
        user_feedback: UserComplexityFeedback,
        actual_result: ComplexityResult,
    ) -> Result<()> {
        let training_example = TrainingExample {
            input: user_feedback.ml_input.clone(),
            actual_complexity: user_feedback.user_perceived_complexity,
            user_satisfaction: user_feedback.satisfaction,
            interface_choice: user_feedback.chosen_interface,
            timestamp: Utc::now(),
        };
        
        self.training_data.add_example(training_example);
        
        // Retrain model periodically
        if self.training_data.len() % 50 == 0 {
            self.retrain_model().await?;
        }
        
        Ok(())
    }
    
    async fn retrain_model(&mut self) -> Result<()> {
        let recent_examples = self.training_data.get_recent_examples(500);
        self.model.update_model(&recent_examples)?;
        
        // Validate model performance
        let validation_score = self.validate_model(&recent_examples)?;
        
        if validation_score > self.model_metadata.best_validation_score {
            self.model_metadata.best_validation_score = validation_score;
            self.save_model().await?;
        }
        
        Ok(())
    }
}

#[derive(Debug, Clone)]
pub struct UserComplexityFeedback {
    pub ml_input: MLInput,
    pub predicted_complexity: f32,
    pub user_perceived_complexity: f32,
    pub satisfaction: SatisfactionLevel,
    pub chosen_interface: UiMode,
    pub feedback_comments: Option<String>,
}
```

#### 4. Explainable Complexity Decisions
```rust
// src/analysis/explainability.rs
#[derive(Debug)]
pub struct ComplexityExplainer {
    explanation_templates: HashMap<FactorType, ExplanationTemplate>,
    example_generator: ExampleGenerator,
}

#[derive(Debug, Clone)]
pub struct ExplanationTemplate {
    pub summary_template: String,
    pub detail_template: String,
    pub recommendation_template: String,
    pub examples: Vec<ExplanationExample>,
}

impl ComplexityExplainer {
    pub fn explain_complexity(
        &self,
        result: &ComplexityResult,
        user_expertise: UserExpertiseLevel,
    ) -> ComplexityExplanation {
        let primary_factors = self.identify_primary_factors(result);
        let contributing_factors = self.identify_contributing_factors(result);
        let mitigating_factors = self.identify_mitigating_factors(result);
        
        let summary = self.generate_summary(
            &primary_factors,
            result.overall_score,
            user_expertise,
        );
        
        ComplexityExplanation {
            primary_factors,
            contributing_factors,
            mitigating_factors,
            summary,
        }
    }
    
    fn generate_summary(
        &self,
        primary_factors: &[ComplexityFactor],
        overall_score: f32,
        user_expertise: UserExpertiseLevel,
    ) -> String {
        let complexity_level = match overall_score {
            0.0..=2.0 => "simple",
            2.0..=5.0 => "moderate",
            5.0..=7.0 => "complex",
            7.0..=10.0 => "very complex",
            _ => "extremely complex",
        };
        
        let main_factor = primary_factors.first()
            .map(|f| f.description.as_str())
            .unwrap_or("multiple factors");
        
        match user_expertise {
            UserExpertiseLevel::Beginner => {
                format!(
                    "This is a {} operation primarily due to {}. {}",
                    complexity_level,
                    main_factor,
                    self.get_beginner_guidance(overall_score)
                )
            },
            UserExpertiseLevel::Intermediate => {
                format!(
                    "Complexity score: {:.1}/10 ({}). Main factors: {}",
                    overall_score,
                    complexity_level,
                    primary_factors.iter()
                        .take(3)
                        .map(|f| f.description.as_str())
                        .collect::<Vec<_>>()
                        .join(", ")
                )
            },
            UserExpertiseLevel::Expert => {
                format!(
                    "Complexity: {:.2}/10. Factors: {} (impact: {:.1}), system load factor: {:.2}",
                    overall_score,
                    main_factor,
                    primary_factors.first().map(|f| f.impact).unwrap_or(0.0),
                    self.calculate_system_load_factor(primary_factors)
                )
            },
        }
    }
    
    fn get_beginner_guidance(&self, score: f32) -> &str {
        match score {
            0.0..=2.0 => "This should be straightforward to execute.",
            2.0..=5.0 => "Consider using the interactive interface for better guidance.",
            5.0..=7.0 => "The interactive interface is recommended for this complex operation.",
            _ => "This is a complex operation that benefits significantly from interactive guidance.",
        }
    }
}
```

### Why This Approach

**Multi-Dimensional Analysis:**
- Considers command, data, system, and user context factors
- Provides component-level scoring for detailed understanding
- Enables targeted optimization and improvement

**Machine Learning Enhancement:**
- Learns from user feedback to improve accuracy over time
- Provides explainable decisions for transparency
- Adapts to different user expertise levels and preferences

**Performance Optimized:**
- Caching for repeated analysis scenarios
- Efficient feature extraction and model inference
- Minimal overhead for real-time decision making

### How to Implement

#### Step 1: Core Analysis Engine (5 hours)
1. **Implement ComplexityAnalysisEngine** with component analyzers
2. **Create ComplexityResult** structure with comprehensive scoring
3. **Add caching system** for performance optimization
4. **Build complexity explanation** generation

#### Step 2: Command and Data Analysis (4 hours)
1. **Implement CommandComplexityAnalyzer** with rule-based scoring
2. **Add DataflowComplexityAnalyzer** for resource analysis
3. **Create SystemComplexityAnalyzer** for system state evaluation
4. **Add feature extraction** for machine learning integration

#### Step 3: Machine Learning Integration (4 hours)
1. **Implement ComplexityMLModel** with linear regression baseline
2. **Add training data collection** and management
3. **Create model retraining** and validation systems
4. **Add user feedback** integration for continuous learning

#### Step 4: Explainability System (2 hours)
1. **Implement ComplexityExplainer** with template-based explanations
2. **Add user expertise-aware** explanation generation
3. **Create factor identification** and ranking algorithms
4. **Add example generation** for clarity

#### Step 5: Testing and Validation (2 hours)
1. **Add comprehensive unit tests** for all components
2. **Test machine learning** accuracy and improvement
3. **Validate explanation** quality and clarity
4. **Test performance** under various load conditions

## 🔗 Dependencies
**Depends On:**
- Issue #001 (Hybrid Command Framework) - Command structures
- Issue #002 (Execution Context Detection) - Context analysis
- Issue #003 (Interface Selection Engine) - Integration point

**Blocks:** 
- All enhanced commands that rely on complexity analysis
- Smart suggestion features in subsequent issues

## 🧪 Testing Requirements

### Unit Tests
```rust
#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_command_complexity_analysis() {
        let analyzer = CommandComplexityAnalyzer::new();
        let simple_command = Command::Ps(PsCommand::default());
        let complex_command = Command::Debug(DebugCommand::default());
        
        let simple_score = analyzer.analyze(&simple_command, &ExecutionContext::default()).await.unwrap();
        let complex_score = analyzer.analyze(&complex_command, &ExecutionContext::default()).await.unwrap();
        
        assert!(simple_score.normalized_score() < complex_score.normalized_score());
    }
    
    #[test]
    fn test_ml_model_prediction() {
        let model = LinearRegressionModel::default();
        let input = MLInput::default();
        
        let output = model.predict(&input).unwrap();
        
        assert!(output.predicted_complexity >= 0.0 && output.predicted_complexity <= 10.0);
        assert!(output.confidence >= 0.0 && output.confidence <= 1.0);
    }
    
    #[test]
    fn test_complexity_explanation() {
        let explainer = ComplexityExplainer::new();
        let result = ComplexityResult::default();
        
        let explanation = explainer.explain_complexity(&result, UserExpertiseLevel::Intermediate);
        
        assert!(!explanation.summary.is_empty());
        assert!(!explanation.primary_factors.is_empty());
    }
}
```

## ✅ Definition of Done
- [ ] ComplexityAnalysisEngine implemented with comprehensive scoring
- [ ] Command, dataflow, and system complexity analyzers provide accurate assessments
- [ ] Machine learning model learns from user feedback and improves over time
- [ ] Complexity explanations are clear and appropriate for different user expertise levels
- [ ] Performance targets met for real-time complexity analysis
- [ ] Caching system optimizes repeated analysis scenarios
- [ ] User feedback integration enables continuous model improvement
- [ ] Comprehensive unit tests validate analysis accuracy
- [ ] Integration tests confirm end-to-end complexity evaluation workflows
- [ ] Manual testing validates explanation quality and usefulness

This complexity analysis system provides the intelligent foundation that enables the hybrid CLI to make sophisticated interface decisions while remaining transparent and explainable to users.