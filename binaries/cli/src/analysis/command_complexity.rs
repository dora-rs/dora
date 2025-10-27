use crate::cli::Command;
use crate::cli::context::ExecutionContext;
use crate::analysis::{ComplexityFactor, FactorType};
use std::collections::HashMap;
use std::path::Path;
use serde::{Serialize, Deserialize};

/// Analyzer for command-specific complexity
#[derive(Debug)]
pub struct CommandComplexityAnalyzer {
    rule_engine: ComplexityRuleEngine,
    pattern_detector: CommandPatternDetector,
    feature_extractor: CommandFeatureExtractor,
}

/// Result of command complexity analysis
#[derive(Debug, Clone)]
pub struct CommandComplexityScore {
    pub base_complexity: f32,
    pub parameter_complexity: f32,
    pub dependency_complexity: f32,
    pub output_complexity: f32,
    pub error_potential: f32,
    pub factors: Vec<ComplexityFactor>,
}

/// Rule-based complexity analysis engine
#[derive(Debug)]
pub struct ComplexityRuleEngine {
    base_rules: HashMap<String, f32>,
    parameter_rules: Vec<ParameterRule>,
    context_rules: Vec<ContextRule>,
}

/// Pattern detector for command sequences and configurations
#[derive(Debug)]
pub struct CommandPatternDetector {
    known_patterns: HashMap<String, PatternComplexity>,
}

/// Feature extractor for ML model input
#[derive(Debug)]
pub struct CommandFeatureExtractor;

/// Rule for parameter-based complexity adjustment
#[derive(Debug, Clone)]
pub struct ParameterRule {
    pub parameter_pattern: String,
    pub complexity_modifier: f32,
    pub description: String,
}

/// Rule for context-based complexity adjustment
#[derive(Debug, Clone)]
pub struct ContextRule {
    pub context_condition: String,
    pub complexity_modifier: f32,
    pub description: String,
}

/// Known pattern complexity information
#[derive(Debug, Clone)]
pub struct PatternComplexity {
    pub base_score: f32,
    pub confidence: f32,
    pub description: String,
}

/// Dataflow configuration for dependency analysis
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DataflowConfig {
    pub nodes: Vec<NodeConfig>,
    pub connections: Vec<ConnectionConfig>,
    pub metadata: HashMap<String, String>,
}

/// Node configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NodeConfig {
    pub id: String,
    pub operator: Option<String>,
    pub inputs: Vec<String>,
    pub outputs: Vec<String>,
    pub environment: HashMap<String, String>,
}

/// Connection configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConnectionConfig {
    pub from: String,
    pub to: String,
    pub output: String,
    pub input: String,
}

impl CommandComplexityAnalyzer {
    pub fn new() -> Self {
        Self {
            rule_engine: ComplexityRuleEngine::new(),
            pattern_detector: CommandPatternDetector::new(),
            feature_extractor: CommandFeatureExtractor::new(),
        }
    }
    
    /// Analyze command complexity
    pub async fn analyze(
        &self,
        command: &Command,
        context: &ExecutionContext,
    ) -> eyre::Result<CommandComplexityScore> {
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
    
    /// Analyze base command complexity
    fn analyze_base_command_complexity(
        &self,
        command: &Command,
        factors: &mut Vec<ComplexityFactor>,
    ) -> f32 {
        let base_score = match command {
            Command::Ps(_) => {
                factors.push(ComplexityFactor {
                    factor_type: FactorType::CommandComplexity,
                    impact: 1.0,
                    description: "Simple list operation".to_string(),
                    evidence: vec!["Read-only operation with minimal processing".to_string()],
                });
                1.0
            },
            Command::Start(cmd) => {
                let mut score = 3.0;
                
                // Check for complex dataflow patterns
                if let Some(path_str) = cmd.dataflow_path.to_str() {
                    if path_str.contains("complex") || path_str.contains("advanced") {
                        score += 2.0;
                        factors.push(ComplexityFactor {
                            factor_type: FactorType::CommandComplexity,
                            impact: 2.0,
                            description: "Complex dataflow configuration detected".to_string(),
                            evidence: vec![format!("File: {}", path_str)],
                        });
                    }
                }
                
                // Environment variables complexity
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
            Command::Stop(_) => {
                factors.push(ComplexityFactor {
                    factor_type: FactorType::CommandComplexity,
                    impact: 2.0,
                    description: "Moderate stop operation complexity".to_string(),
                    evidence: vec!["Requires careful resource cleanup".to_string()],
                });
                2.0
            },
            Command::Logs(cmd) => {
                let mut score = 4.0;
                
                if cmd.follow {
                    score += 2.0;
                    factors.push(ComplexityFactor {
                        factor_type: FactorType::OutputComplexity,
                        impact: 2.0,
                        description: "Real-time log streaming".to_string(),
                        evidence: vec!["Continuous data stream requires careful handling".to_string()],
                    });
                }
                
                score
            },
            Command::Build(_) => {
                factors.push(ComplexityFactor {
                    factor_type: FactorType::CommandComplexity,
                    impact: 5.0,
                    description: "Build operations involve multiple steps".to_string(),
                    evidence: vec!["Compilation, linking, and dependency resolution".to_string()],
                });
                5.0
            },
            Command::Up(_) => {
                factors.push(ComplexityFactor {
                    factor_type: FactorType::CommandComplexity,
                    impact: 4.0,
                    description: "Combined build and start operation".to_string(),
                    evidence: vec!["Multiple sequential operations with dependencies".to_string()],
                });
                4.0
            },
            Command::Destroy(_) => {
                factors.push(ComplexityFactor {
                    factor_type: FactorType::CommandComplexity,
                    impact: 3.0,
                    description: "Destructive operation with safety considerations".to_string(),
                    evidence: vec!["Resource cleanup and validation required".to_string()],
                });
                3.0
            },
            Command::New(_) => {
                factors.push(ComplexityFactor {
                    factor_type: FactorType::CommandComplexity,
                    impact: 3.0,
                    description: "Template generation and configuration".to_string(),
                    evidence: vec!["File system operations and template processing".to_string()],
                });
                3.0
            },
            Command::Check(_) => {
                factors.push(ComplexityFactor {
                    factor_type: FactorType::CommandComplexity,
                    impact: 4.0,
                    description: "Comprehensive validation and analysis".to_string(),
                    evidence: vec!["Configuration parsing and dependency checking".to_string()],
                });
                4.0
            },
            Command::Graph(_) => {
                factors.push(ComplexityFactor {
                    factor_type: FactorType::CommandComplexity,
                    impact: 5.0,
                    description: "Graph generation and visualization".to_string(),
                    evidence: vec!["Complex data processing and rendering".to_string()],
                });
                5.0
            },
            Command::Inspect(_) => {
                factors.push(ComplexityFactor {
                    factor_type: FactorType::CommandComplexity,
                    impact: 7.0,
                    description: "Deep inspection requires extensive analysis".to_string(),
                    evidence: vec!["Real-time metrics, state analysis, and data aggregation".to_string()],
                });
                7.0
            },
            Command::Debug(_) => {
                factors.push(ComplexityFactor {
                    factor_type: FactorType::CommandComplexity,
                    impact: 8.0,
                    description: "Interactive debugging is inherently complex".to_string(),
                    evidence: vec!["State inspection, breakpoints, and interactive control".to_string()],
                });
                8.0
            },
            Command::Analyze(_) => {
                factors.push(ComplexityFactor {
                    factor_type: FactorType::CommandComplexity,
                    impact: 9.0,
                    description: "Analysis involves complex algorithms and data processing".to_string(),
                    evidence: vec!["Statistical analysis, pattern detection, and reporting".to_string()],
                });
                9.0
            },
            Command::Monitor(_) => {
                factors.push(ComplexityFactor {
                    factor_type: FactorType::CommandComplexity,
                    impact: 7.0,
                    description: "Real-time monitoring with multiple data sources".to_string(),
                    evidence: vec!["Continuous data collection and real-time updates".to_string()],
                });
                7.0
            },
            Command::Tui(_) => {
                factors.push(ComplexityFactor {
                    factor_type: FactorType::CommandComplexity,
                    impact: 2.0,
                    description: "TUI launcher is moderately complex".to_string(),
                    evidence: vec!["Interface initialization and state management".to_string()],
                });
                2.0
            },
            Command::Dashboard(_) => {
                factors.push(ComplexityFactor {
                    factor_type: FactorType::CommandComplexity,
                    impact: 4.0,
                    description: "Dashboard requires data aggregation and visualization".to_string(),
                    evidence: vec!["Multiple data sources and real-time updates".to_string()],
                });
                4.0
            },
            Command::System(_) => {
                factors.push(ComplexityFactor {
                    factor_type: FactorType::CommandComplexity,
                    impact: 5.0,
                    description: "System operations involve low-level management".to_string(),
                    evidence: vec!["Process management and system integration".to_string()],
                });
                5.0
            },
            Command::Config(_) => {
                factors.push(ComplexityFactor {
                    factor_type: FactorType::CommandComplexity,
                    impact: 4.0,
                    description: "Configuration management with validation".to_string(),
                    evidence: vec!["Schema validation and configuration merging".to_string()],
                });
                4.0
            },
            Command::Daemon(_) => {
                factors.push(ComplexityFactor {
                    factor_type: FactorType::CommandComplexity,
                    impact: 6.0,
                    description: "Daemon operations involve process lifecycle management".to_string(),
                    evidence: vec!["Background process control and monitoring".to_string()],
                });
                6.0
            },
            Command::Runtime(_) => {
                factors.push(ComplexityFactor {
                    factor_type: FactorType::CommandComplexity,
                    impact: 6.0,
                    description: "Runtime operations manage execution environment".to_string(),
                    evidence: vec!["Resource allocation and runtime coordination".to_string()],
                });
                6.0
            },
            Command::Coordinator(_) => {
                factors.push(ComplexityFactor {
                    factor_type: FactorType::CommandComplexity,
                    impact: 7.0,
                    description: "Coordinator manages distributed dataflow execution".to_string(),
                    evidence: vec!["Distributed coordination and state synchronization".to_string()],
                });
                7.0
            },
            Command::Self_(_) => {
                factors.push(ComplexityFactor {
                    factor_type: FactorType::CommandComplexity,
                    impact: 3.0,
                    description: "Self-management operations".to_string(),
                    evidence: vec!["Update and maintenance operations".to_string()],
                });
                3.0
            },
            Command::Preferences(_) => {
                factors.push(ComplexityFactor {
                    factor_type: FactorType::CommandComplexity,
                    impact: 4.0,
                    description: "Preference management with behavioral learning".to_string(),
                    evidence: vec!["Configuration persistence and learning algorithms".to_string()],
                });
                4.0
            },
        };
        
        base_score
    }
    
    /// Analyze parameter complexity
    fn analyze_parameter_complexity(
        &self,
        command: &Command,
        factors: &mut Vec<ComplexityFactor>,
    ) -> f32 {
        // This is a simplified analysis - in a real implementation,
        // you would analyze actual command parameters
        match command {
            Command::Start(_) => {
                // Could analyze specific parameters like environment variables,
                // working directory, etc.
                2.0
            },
            Command::Logs(_) => {
                // Could analyze filters, time ranges, etc.
                1.5
            },
            Command::Build(_) => {
                // Could analyze build options, target platforms, etc.
                2.5
            },
            _ => 1.0,
        }
    }
    
    /// Analyze dependency complexity (simplified implementation)
    async fn analyze_dependency_complexity(
        &self,
        command: &Command,
        factors: &mut Vec<ComplexityFactor>,
    ) -> eyre::Result<f32> {
        match command {
            Command::Start(cmd) => {
                // In a real implementation, you would load and parse the dataflow config
                let dataflow_path = &cmd.dataflow_path;
                
                if dataflow_path.exists() {
                    // Simplified analysis based on file size and extension
                    let metadata = std::fs::metadata(dataflow_path)?;
                    let file_size = metadata.len();
                    
                    let mut score = 0.0;
                    
                    if file_size > 10_000 {
                        score += 3.0;
                        factors.push(ComplexityFactor {
                            factor_type: FactorType::DependencyComplexity,
                            impact: 3.0,
                            description: "Large dataflow configuration file".to_string(),
                            evidence: vec![format!("File size: {} bytes", file_size)],
                        });
                    } else if file_size > 1_000 {
                        score += 1.0;
                        factors.push(ComplexityFactor {
                            factor_type: FactorType::DependencyComplexity,
                            impact: 1.0,
                            description: "Medium dataflow configuration file".to_string(),
                            evidence: vec![format!("File size: {} bytes", file_size)],
                        });
                    }
                    
                    Ok(score)
                } else {
                    factors.push(ComplexityFactor {
                        factor_type: FactorType::ErrorComplexity,
                        impact: 2.0,
                        description: "Dataflow configuration file not found".to_string(),
                        evidence: vec![format!("Path: {}", dataflow_path.display())],
                    });
                    Ok(2.0)
                }
            },
            _ => Ok(0.0),
        }
    }
    
    /// Analyze output complexity
    fn analyze_output_complexity(
        &self,
        command: &Command,
        context: &ExecutionContext,
        factors: &mut Vec<ComplexityFactor>,
    ) -> f32 {
        let mut score = 0.0;
        
        // Analyze based on command type
        match command {
            Command::Logs(_) => {
                score += 5.0;
                factors.push(ComplexityFactor {
                    factor_type: FactorType::OutputComplexity,
                    impact: 5.0,
                    description: "Log output can be voluminous and requires filtering".to_string(),
                    evidence: vec!["Potentially streaming large amounts of text data".to_string()],
                });
            },
            Command::Ps(_) => {
                score += 2.0;
                factors.push(ComplexityFactor {
                    factor_type: FactorType::OutputComplexity,
                    impact: 2.0,
                    description: "Structured table output with multiple columns".to_string(),
                    evidence: vec!["Tabular data with status information".to_string()],
                });
            },
            Command::Inspect(_) => {
                score += 6.0;
                factors.push(ComplexityFactor {
                    factor_type: FactorType::OutputComplexity,
                    impact: 6.0,
                    description: "Complex nested data structures and metrics".to_string(),
                    evidence: vec!["Hierarchical data with real-time updates".to_string()],
                });
            },
            Command::Debug(_) => {
                score += 7.0;
                factors.push(ComplexityFactor {
                    factor_type: FactorType::OutputComplexity,
                    impact: 7.0,
                    description: "Interactive debugging output with state information".to_string(),
                    evidence: vec!["Variable states, call stacks, and execution traces".to_string()],
                });
            },
            Command::Analyze(_) => {
                score += 8.0;
                factors.push(ComplexityFactor {
                    factor_type: FactorType::OutputComplexity,
                    impact: 8.0,
                    description: "Complex analysis results with statistics and visualizations".to_string(),
                    evidence: vec!["Charts, graphs, and detailed statistical reports".to_string()],
                });
            },
            _ => {
                score += 1.0;
            }
        }
        
        // Consider terminal constraints
        if let Some((width, height)) = context.terminal_size {
            if width < 80 || height < 24 {
                score += 2.0;
                factors.push(ComplexityFactor {
                    factor_type: FactorType::OutputComplexity,
                    impact: 2.0,
                    description: "Small terminal size constrains output formatting".to_string(),
                    evidence: vec![format!("Terminal size: {}x{}", width, height)],
                });
            }
        }
        
        score.min(10.0)
    }
    
    /// Analyze error potential
    fn analyze_error_potential(
        &self,
        command: &Command,
        factors: &mut Vec<ComplexityFactor>,
    ) -> f32 {
        let score = match command {
            Command::Start(_) => {
                factors.push(ComplexityFactor {
                    factor_type: FactorType::ErrorComplexity,
                    impact: 4.0,
                    description: "Dataflow startup can fail due to configuration or resource issues".to_string(),
                    evidence: vec!["Port conflicts, missing dependencies, invalid configurations".to_string()],
                });
                4.0
            },
            Command::Build(_) => {
                factors.push(ComplexityFactor {
                    factor_type: FactorType::ErrorComplexity,
                    impact: 6.0,
                    description: "Build processes can fail due to compilation or dependency errors".to_string(),
                    evidence: vec!["Compilation errors, missing dependencies, version conflicts".to_string()],
                });
                6.0
            },
            Command::Debug(_) => {
                factors.push(ComplexityFactor {
                    factor_type: FactorType::ErrorComplexity,
                    impact: 5.0,
                    description: "Debugging can encounter complex runtime errors".to_string(),
                    evidence: vec!["Runtime exceptions, state corruption, timing issues".to_string()],
                });
                5.0
            },
            Command::Destroy(_) => {
                factors.push(ComplexityFactor {
                    factor_type: FactorType::ErrorComplexity,
                    impact: 3.0,
                    description: "Destructive operations can have unintended consequences".to_string(),
                    evidence: vec!["Data loss, resource leaks, incomplete cleanup".to_string()],
                });
                3.0
            },
            _ => 2.0,
        };
        
        score
    }
}

impl CommandComplexityScore {
    /// Get normalized complexity score (0.0 - 10.0)
    pub fn normalized_score(&self) -> f32 {
        let weighted_score = 
            self.base_complexity * 0.3 +
            self.parameter_complexity * 0.2 +
            self.dependency_complexity * 0.2 +
            self.output_complexity * 0.2 +
            self.error_potential * 0.1;
        
        weighted_score.min(10.0)
    }
    
    /// Convert to feature vector for ML model
    pub fn to_features(&self) -> Vec<f32> {
        vec![
            self.base_complexity / 10.0,
            self.parameter_complexity / 10.0,
            self.dependency_complexity / 10.0,
            self.output_complexity / 10.0,
            self.error_potential / 10.0,
            self.normalized_score() / 10.0,
            self.factors.len() as f32 / 20.0, // Normalize factor count
        ]
    }
}

impl ComplexityRuleEngine {
    pub fn new() -> Self {
        Self {
            base_rules: Self::create_base_rules(),
            parameter_rules: Self::create_parameter_rules(),
            context_rules: Self::create_context_rules(),
        }
    }
    
    fn create_base_rules() -> HashMap<String, f32> {
        let mut rules = HashMap::new();
        rules.insert("ps".to_string(), 1.0);
        rules.insert("start".to_string(), 3.0);
        rules.insert("stop".to_string(), 2.0);
        rules.insert("logs".to_string(), 4.0);
        rules.insert("build".to_string(), 5.0);
        rules.insert("debug".to_string(), 8.0);
        rules.insert("analyze".to_string(), 9.0);
        rules
    }
    
    fn create_parameter_rules() -> Vec<ParameterRule> {
        vec![
            ParameterRule {
                parameter_pattern: "follow".to_string(),
                complexity_modifier: 2.0,
                description: "Real-time following increases complexity".to_string(),
            },
            ParameterRule {
                parameter_pattern: "environment".to_string(),
                complexity_modifier: 0.5,
                description: "Each environment variable adds complexity".to_string(),
            },
        ]
    }
    
    fn create_context_rules() -> Vec<ContextRule> {
        vec![
            ContextRule {
                context_condition: "small_terminal".to_string(),
                complexity_modifier: 1.0,
                description: "Small terminal increases output complexity".to_string(),
            },
            ContextRule {
                context_condition: "non_interactive".to_string(),
                complexity_modifier: -1.0,
                description: "Non-interactive context reduces interaction complexity".to_string(),
            },
        ]
    }
}

impl CommandPatternDetector {
    pub fn new() -> Self {
        Self {
            known_patterns: Self::create_known_patterns(),
        }
    }
    
    fn create_known_patterns() -> HashMap<String, PatternComplexity> {
        let mut patterns = HashMap::new();
        
        patterns.insert("complex_dataflow".to_string(), PatternComplexity {
            base_score: 7.0,
            confidence: 0.8,
            description: "Complex dataflow with many nodes".to_string(),
        });
        
        patterns.insert("simple_dataflow".to_string(), PatternComplexity {
            base_score: 3.0,
            confidence: 0.9,
            description: "Simple dataflow with few nodes".to_string(),
        });
        
        patterns
    }
}

impl CommandFeatureExtractor {
    pub fn new() -> Self {
        Self
    }
    
    /// Extract features from command for ML model
    pub fn extract_features(&self, command: &Command, context: &ExecutionContext) -> Vec<f32> {
        // Extract various features that could be useful for complexity prediction
        vec![
            self.command_type_feature(command),
            self.parameter_count_feature(command),
            self.context_complexity_feature(context),
            self.terminal_size_feature(context),
            self.interaction_capability_feature(context),
        ]
    }
    
    fn command_type_feature(&self, command: &Command) -> f32 {
        // Map command types to numeric features
        match command {
            Command::Ps(_) => 0.1,
            Command::Start(_) => 0.3,
            Command::Stop(_) => 0.2,
            Command::Logs(_) => 0.4,
            Command::Build(_) => 0.5,
            Command::Debug(_) => 0.8,
            Command::Analyze(_) => 0.9,
            _ => 0.5,
        }
    }
    
    fn parameter_count_feature(&self, _command: &Command) -> f32 {
        // In a real implementation, count actual parameters
        0.5
    }
    
    fn context_complexity_feature(&self, context: &ExecutionContext) -> f32 {
        let mut score = 0.0;
        
        if !context.is_tty { score += 0.2; }
        if context.is_piped { score += 0.3; }
        if context.is_scripted { score += 0.4; }
        if context.environment.is_ci { score += 0.3; }
        
        score.min(1.0)
    }
    
    fn terminal_size_feature(&self, context: &ExecutionContext) -> f32 {
        context.terminal_size
            .map(|(w, h)| ((w * h) as f32 / 10000.0).min(1.0))
            .unwrap_or(0.5)
    }
    
    fn interaction_capability_feature(&self, context: &ExecutionContext) -> f32 {
        let mut score = 0.0;
        
        if context.terminal_capabilities.tui_capable { score += 0.4; }
        if context.terminal_capabilities.supports_color { score += 0.2; }
        if context.terminal_capabilities.supports_mouse { score += 0.2; }
        if context.terminal_capabilities.interactive { score += 0.2; }
        
        score
    }
}

impl DataflowConfig {
    /// Count external dependencies in the dataflow
    pub fn count_external_dependencies(&self) -> usize {
        // Simplified implementation - count nodes with external operators
        self.nodes.iter()
            .filter(|node| node.operator.is_some())
            .count()
    }
}