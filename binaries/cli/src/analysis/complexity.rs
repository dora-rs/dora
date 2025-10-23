use crate::cli::{Command, UiMode};
use crate::cli::context::ExecutionContext;
use crate::analysis::{
    CommandComplexityAnalyzer, DataflowComplexityAnalyzer, SystemComplexityAnalyzer,
    ComplexityMLModel, ComplexityExplainer, DataComplexityScore, SystemComplexityScore,
    MLInput, MLOutput, UserExpertiseLevel,
};
use chrono::{DateTime, Utc};
use lru::LruCache;
use std::collections::HashMap;
use std::num::NonZeroUsize;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use serde::{Serialize, Deserialize};

/// Core complexity analysis engine
#[derive(Debug)]
pub struct ComplexityAnalysisEngine {
    command_analyzer: CommandComplexityAnalyzer,
    dataflow_analyzer: DataflowComplexityAnalyzer,
    system_analyzer: SystemComplexityAnalyzer,
    ml_model: Arc<Mutex<ComplexityMLModel>>,
    explainer: ComplexityExplainer,
    cache: Arc<Mutex<LruCache<ComplexityKey, ComplexityResult>>>,
}

/// Request for complexity analysis
#[derive(Debug, Clone)]
pub struct ComplexityAnalysisRequest {
    pub command: Command,
    pub context: ExecutionContext,
    pub target_resource: Option<ResourceTarget>,
    pub system_state: SystemState,
    pub user_expertise: UserExpertiseLevel,
}

/// Result of complexity analysis
#[derive(Debug, Clone)]
pub struct ComplexityResult {
    pub overall_score: f32,
    pub component_scores: ComponentScores,
    pub confidence: f32,
    pub explanation: ComplexityExplanation,
    pub recommendations: Vec<InterfaceRecommendation>,
    pub calculated_at: Instant,
}

/// Component scores for different aspects of complexity
#[derive(Debug, Clone)]
pub struct ComponentScores {
    pub command_complexity: f32,
    pub data_complexity: f32,
    pub interaction_complexity: f32,
    pub output_complexity: f32,
    pub error_complexity: f32,
    pub performance_complexity: f32,
}

/// Explanation of complexity calculation
#[derive(Debug, Clone)]
pub struct ComplexityExplanation {
    pub primary_factors: Vec<ComplexityFactor>,
    pub contributing_factors: Vec<ComplexityFactor>,
    pub mitigating_factors: Vec<ComplexityFactor>,
    pub summary: String,
}

/// Individual complexity factor
#[derive(Debug, Clone)]
pub struct ComplexityFactor {
    pub factor_type: FactorType,
    pub impact: f32,
    pub description: String,
    pub evidence: Vec<String>,
}

/// Types of complexity factors
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum FactorType {
    CommandComplexity,
    ParameterComplexity,
    DependencyComplexity,
    OutputComplexity,
    ErrorComplexity,
    SystemComplexity,
    ContextComplexity,
    UserExperienceComplexity,
}

/// Interface recommendation
#[derive(Debug, Clone)]
pub struct InterfaceRecommendation {
    pub interface: UiMode,
    pub confidence: f32,
    pub reason: String,
    pub priority: RecommendationPriority,
}

/// Priority of interface recommendation
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub enum RecommendationPriority {
    Low,
    Medium,
    High,
    Critical,
}

/// Resource target for analysis
#[derive(Debug, Clone)]
pub struct ResourceTarget {
    pub resource_type: ResourceType,
    pub identifier: String,
    pub metadata: HashMap<String, String>,
}

/// Type of resource being analyzed
#[derive(Debug, Clone)]
pub enum ResourceType {
    Dataflow,
    Node,
    Operator,
    System,
}

/// Current system state
#[derive(Debug, Clone)]
pub struct SystemState {
    pub active_dataflows: usize,
    pub system_load: f32,
    pub memory_usage: f32,
    pub network_activity: f32,
    pub error_rate: f32,
    pub last_updated: DateTime<Utc>,
}

/// User expertise level
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum UserExpertiseLevel {
    Beginner,
    Intermediate,
    Expert,
}

/// Cache key for complexity results
#[derive(Debug, Clone, Hash, PartialEq, Eq)]
pub struct ComplexityKey {
    command_hash: u64,
    context_hash: u64,
    resource_hash: Option<u64>,
    system_state_hash: u64,
    user_expertise: UserExpertiseLevel,
}

impl ComplexityAnalysisEngine {
    pub fn new() -> Self {
        let cache_size = NonZeroUsize::new(1000).unwrap();
        
        Self {
            command_analyzer: CommandComplexityAnalyzer::new(),
            dataflow_analyzer: DataflowComplexityAnalyzer::new(),
            system_analyzer: SystemComplexityAnalyzer::new(),
            ml_model: Arc::new(Mutex::new(ComplexityMLModel::load_or_create())),
            explainer: ComplexityExplainer::new(),
            cache: Arc::new(Mutex::new(LruCache::new(cache_size))),
        }
    }
    
    /// Analyze complexity for a given request
    pub async fn analyze_complexity(
        &self,
        request: &ComplexityAnalysisRequest,
    ) -> eyre::Result<ComplexityResult> {
        let cache_key = ComplexityKey::from_request(request);
        
        // Check cache first
        {
            let mut cache = self.cache.lock().unwrap();
            if let Some(cached_result) = cache.get(&cache_key) {
                if cached_result.calculated_at.elapsed() < Duration::from_secs(60) {
                    return Ok(cached_result.clone());
                }
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
        
        // Create ML input features
        let ml_input = MLInput {
            command_features: command_score.to_features(),
            data_features: data_score.to_features(),
            system_features: system_score.to_features(),
            context_features: self.extract_context_features(&request.context),
        };
        
        // Get ML prediction
        let ml_result = {
            let model = self.ml_model.lock().unwrap();
            model.predict(&ml_input)?
        };
        
        // Calculate component scores
        let component_scores = ComponentScores {
            command_complexity: command_score.normalized_score(),
            data_complexity: data_score.normalized_score(),
            interaction_complexity: self.calculate_interaction_complexity(request),
            output_complexity: self.calculate_output_complexity(request),
            error_complexity: self.calculate_error_complexity(request),
            performance_complexity: system_score.normalized_score(),
        };
        
        // Generate explanation
        let explanation = self.explainer.explain_complexity(
            &component_scores,
            &ml_result,
            &request.user_expertise,
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
        {
            let mut cache = self.cache.lock().unwrap();
            cache.put(cache_key, result.clone());
        }
        
        Ok(result)
    }
    
    /// Calculate interaction complexity based on command characteristics
    fn calculate_interaction_complexity(&self, request: &ComplexityAnalysisRequest) -> f32 {
        let mut score = 0.0;
        
        match &request.command {
            Command::Ps(_) => {
                score += 2.0; // Basic interaction with filtering/sorting benefits
            },
            Command::Logs(_) => {
                score += 6.0; // High interaction benefit for real-time filtering
            },
            Command::Inspect(_) => {
                score += 8.0; // Very high interaction for navigation and drilling down
            },
            Command::Debug(_) => {
                score += 9.0; // Critical interaction for debugging workflows
            },
            Command::Analyze(_) => {
                score += 9.0; // Critical interaction for data analysis
            },
            Command::Monitor(_) => {
                score += 8.0; // High interaction for real-time monitoring
            },
            _ => {
                score += 1.0; // Minimal interaction benefit
            }
        }
        
        // Adjust based on user expertise
        match request.user_expertise {
            UserExpertiseLevel::Beginner => score * 1.3, // Beginners benefit more from interaction
            UserExpertiseLevel::Intermediate => score,
            UserExpertiseLevel::Expert => score * 0.8, // Experts can handle CLI better
        }
    }
    
    /// Calculate output complexity based on expected data volume and format
    fn calculate_output_complexity(&self, request: &ComplexityAnalysisRequest) -> f32 {
        let mut score = 0.0;
        
        match &request.command {
            Command::Ps(_) => {
                score += 3.0; // Structured table output
                if request.system_state.active_dataflows > 10 {
                    score += 2.0; // More dataflows = more complex output
                }
            },
            Command::Logs(_) => {
                score += 7.0; // Potentially large volume of streaming text
            },
            Command::Inspect(_) => {
                score += 6.0; // Complex structured data with metrics
            },
            Command::Debug(_) => {
                score += 8.0; // Complex debugging information
            },
            Command::Analyze(_) => {
                score += 9.0; // Very complex analysis results
            },
            Command::Monitor(_) => {
                score += 7.0; // Real-time streaming data
            },
            _ => {
                score += 2.0; // Basic output
            }
        }
        
        // Consider terminal constraints
        if let Some((width, height)) = request.context.terminal_size {
            if width < 80 || height < 24 {
                score += 2.0; // Small terminal increases output complexity
            }
        }
        
        score.min(10.0)
    }
    
    /// Calculate error complexity based on potential for errors and their impact
    fn calculate_error_complexity(&self, request: &ComplexityAnalysisRequest) -> f32 {
        let mut score = 0.0;
        
        match &request.command {
            Command::Start(_) => {
                score += 5.0; // Moderate error potential with dataflow startup
            },
            Command::Stop(_) => {
                score += 3.0; // Lower error potential
            },
            Command::Build(_) => {
                score += 6.0; // Build errors can be complex
            },
            Command::Debug(_) => {
                score += 8.0; // High error complexity in debugging
            },
            Command::Destroy(_) => {
                score += 4.0; // Potential for destructive errors
            },
            _ => {
                score += 2.0; // Basic error potential
            }
        }
        
        // Adjust based on system state
        score += request.system_state.error_rate * 3.0;
        
        score.min(10.0)
    }
    
    /// Generate interface recommendations based on complexity scores
    fn generate_recommendations(
        &self,
        component_scores: &ComponentScores,
        ml_result: &MLOutput,
    ) -> Vec<InterfaceRecommendation> {
        let mut recommendations = Vec::new();
        
        let overall_score = ml_result.predicted_complexity;
        let confidence = ml_result.confidence;
        
        // Primary recommendation based on overall score
        if overall_score >= 8.0 {
            recommendations.push(InterfaceRecommendation {
                interface: UiMode::Tui,
                confidence: confidence * 0.9,
                reason: "Very high complexity requires interactive interface".to_string(),
                priority: RecommendationPriority::Critical,
            });
        } else if overall_score >= 6.0 {
            recommendations.push(InterfaceRecommendation {
                interface: UiMode::Auto,
                confidence: confidence * 0.8,
                reason: "High complexity benefits from interactive features".to_string(),
                priority: RecommendationPriority::High,
            });
        } else if overall_score >= 4.0 {
            recommendations.push(InterfaceRecommendation {
                interface: UiMode::Auto,
                confidence: confidence * 0.7,
                reason: "Moderate complexity may benefit from TUI".to_string(),
                priority: RecommendationPriority::Medium,
            });
        } else {
            recommendations.push(InterfaceRecommendation {
                interface: UiMode::Cli,
                confidence: confidence * 0.9,
                reason: "Low complexity suitable for CLI".to_string(),
                priority: RecommendationPriority::Low,
            });
        }
        
        // Additional recommendations based on specific components
        if component_scores.interaction_complexity >= 7.0 {
            recommendations.push(InterfaceRecommendation {
                interface: UiMode::Tui,
                confidence: 0.8,
                reason: "High interaction complexity requires TUI".to_string(),
                priority: RecommendationPriority::High,
            });
        }
        
        if component_scores.output_complexity >= 6.0 {
            recommendations.push(InterfaceRecommendation {
                interface: UiMode::Tui,
                confidence: 0.75,
                reason: "Complex output benefits from interactive navigation".to_string(),
                priority: RecommendationPriority::Medium,
            });
        }
        
        recommendations
    }
    
    /// Extract context features for ML model
    fn extract_context_features(&self, context: &ExecutionContext) -> Vec<f32> {
        vec![
            if context.is_tty { 1.0 } else { 0.0 },
            if context.is_piped { 1.0 } else { 0.0 },
            if context.is_scripted { 1.0 } else { 0.0 },
            context.terminal_size.map(|(w, h)| (w * h) as f32 / 10000.0).unwrap_or(0.0),
            if context.terminal_capabilities.tui_capable { 1.0 } else { 0.0 },
            if context.terminal_capabilities.supports_color { 1.0 } else { 0.0 },
            if context.environment.is_ci { 1.0 } else { 0.0 },
            context.user_preference.as_u8() as f32 / 3.0,
        ]
    }
    
    /// Clear the analysis cache
    pub fn clear_cache(&self) {
        let mut cache = self.cache.lock().unwrap();
        cache.clear();
    }
    
    /// Get cache statistics
    pub fn cache_stats(&self) -> (usize, usize) {
        let cache = self.cache.lock().unwrap();
        (cache.len(), cache.cap().into())
    }
}

impl ComplexityKey {
    pub fn from_request(request: &ComplexityAnalysisRequest) -> Self {
        use std::collections::hash_map::DefaultHasher;
        use std::hash::{Hash, Hasher};
        
        let mut hasher = DefaultHasher::new();
        request.command.name().hash(&mut hasher);
        let command_hash = hasher.finish();
        
        let mut hasher = DefaultHasher::new();
        format!("{:?}", request.context).hash(&mut hasher);
        let context_hash = hasher.finish();
        
        let resource_hash = request.target_resource.as_ref().map(|r| {
            let mut hasher = DefaultHasher::new();
            format!("{:?}", r).hash(&mut hasher);
            hasher.finish()
        });
        
        let mut hasher = DefaultHasher::new();
        format!("{:?}", request.system_state).hash(&mut hasher);
        let system_state_hash = hasher.finish();
        
        Self {
            command_hash,
            context_hash,
            resource_hash,
            system_state_hash,
            user_expertise: request.user_expertise.clone(),
        }
    }
}

impl Default for SystemState {
    fn default() -> Self {
        Self {
            active_dataflows: 0,
            system_load: 0.0,
            memory_usage: 0.0,
            network_activity: 0.0,
            error_rate: 0.0,
            last_updated: Utc::now(),
        }
    }
}

impl Default for UserExpertiseLevel {
    fn default() -> Self {
        UserExpertiseLevel::Intermediate
    }
}

impl Default for ComponentScores {
    fn default() -> Self {
        Self {
            command_complexity: 0.0,
            data_complexity: 0.0,
            interaction_complexity: 0.0,
            output_complexity: 0.0,
            error_complexity: 0.0,
            performance_complexity: 0.0,
        }
    }
}