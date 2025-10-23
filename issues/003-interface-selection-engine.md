# Issue #003: Build Interface Selection Engine

## 📋 Summary
Implement the smart interface selection engine that combines execution context detection and command analysis to automatically choose the most appropriate interface (CLI, TUI, or hybrid) for each user interaction. This engine enables the progressive disclosure model at the heart of the hybrid CLI architecture.

## 🎯 Objectives
- Create intelligent interface decision logic based on context and command complexity
- Implement automatic TUI suggestions and launches for appropriate scenarios
- Provide seamless fallback mechanisms for non-interactive environments
- Enable user preference integration and override capabilities
- Support A/B testing and analytics for interface decisions

**Success Metrics:**
- 90% of users find interface suggestions helpful and appropriate
- No false positives for TUI launch in non-interactive contexts (CI/automation)
- Interface decisions complete in <5ms for responsive user experience
- User preference overrides work correctly 100% of the time

## 🛠️ Technical Requirements

### What to Build

#### 1. Interface Selector Core Structure
```rust
// src/cli/interface.rs
#[derive(Debug)]
pub struct InterfaceSelector {
    context: ExecutionContext,
    config: UserConfig,
    command_analyzer: CommandAnalyzer,
    decision_cache: LruCache<String, InterfaceDecision>,
}

#[derive(Debug, Clone)]
pub struct InterfaceDecision {
    pub strategy: InterfaceStrategy,
    pub confidence: f32,
    pub reason: String,
    pub fallback: Option<InterfaceStrategy>,
}

#[derive(Debug, Clone)]
pub enum InterfaceStrategy {
    CliOnly,
    CliWithHint { 
        hint: String, 
        tui_command: String 
    },
    PromptForTui { 
        reason: String, 
        default_yes: bool 
    },
    AutoLaunchTui { 
        reason: String, 
        show_cli_first: bool 
    },
}

impl InterfaceSelector {
    pub fn new(context: ExecutionContext, config: UserConfig) -> Self {
        Self {
            context,
            config,
            command_analyzer: CommandAnalyzer::new(),
            decision_cache: LruCache::new(NonZeroUsize::new(100).unwrap()),
        }
    }
    
    pub fn select_interface(&mut self, command: &Command) -> InterfaceDecision {
        // Check cache first for performance
        let cache_key = self.generate_cache_key(command);
        if let Some(cached) = self.decision_cache.get(&cache_key) {
            return cached.clone();
        }
        
        let decision = self.analyze_and_decide(command);
        self.decision_cache.put(cache_key, decision.clone());
        decision
    }
}
```

#### 2. Command Analysis System
```rust
#[derive(Debug)]
pub struct CommandAnalyzer {
    complexity_rules: ComplexityRules,
    interaction_patterns: InteractionPatterns,
}

#[derive(Debug, Clone)]
pub struct CommandAnalysis {
    pub complexity_score: u8,           // 0-10 scale
    pub data_volume_estimate: DataVolume,
    pub interaction_benefit: InteractionBenefit,
    pub automation_suitability: AutomationLevel,
    pub output_characteristics: OutputCharacteristics,
}

#[derive(Debug, Clone)]
pub enum DataVolume {
    Minimal,        // Single values, short lists
    Small,          // Tables under 50 rows
    Medium,         // Tables 50-500 rows, simple graphs
    Large,          // Complex data requiring visualization
    Streaming,      // Real-time data streams
}

#[derive(Debug, Clone)]
pub enum InteractionBenefit {
    None,           // Static output only
    Low,            // Minor filtering/sorting benefit
    Medium,         // Significant navigation benefit
    High,           // Essential for effective use
    Critical,       // Nearly unusable without interaction
}

impl CommandAnalyzer {
    pub fn analyze(&self, command: &Command) -> CommandAnalysis {
        let base_complexity = self.calculate_base_complexity(command);
        let context_modifiers = self.apply_context_modifiers(command);
        let final_complexity = (base_complexity + context_modifiers).clamp(0, 10);
        
        CommandAnalysis {
            complexity_score: final_complexity,
            data_volume_estimate: self.estimate_data_volume(command),
            interaction_benefit: self.assess_interaction_benefit(command),
            automation_suitability: self.check_automation_level(command),
            output_characteristics: self.analyze_output_characteristics(command),
        }
    }
    
    fn calculate_base_complexity(&self, command: &Command) -> u8 {
        match command {
            Command::Ps(_) => 1,                    // Simple list
            Command::Start(cmd) => {
                let mut score = 2;
                if cmd.dataflow_path.ends_with(".yml") {
                    // Check file complexity
                    if let Ok(complexity) = self.analyze_dataflow_complexity(&cmd.dataflow_path) {
                        score += complexity;
                    }
                }
                score
            },
            Command::Logs(cmd) => {
                let mut score = 3;
                if cmd.follow { score += 1; }           // Real-time adds complexity
                if cmd.filter.is_some() { score += 1; } // Filtering adds complexity
                if cmd.search.is_some() { score += 1; } // Search adds complexity
                if cmd.error_analysis { score += 2; }   // Analysis significantly complex
                score
            },
            Command::Inspect(cmd) => {
                let mut score = 4;
                match cmd.resource_type {
                    ResourceType::Node => score += 2,      // Node inspection is complex
                    ResourceType::Dataflow => score += 3,  // Dataflow even more so
                    ResourceType::Recording => score += 4, // Recording analysis very complex
                    _ => score += 1,
                }
                if cmd.live_mode { score += 2; }       // Live monitoring adds complexity
                score
            },
            Command::Debug(_) => 8,                     // Always high complexity
            Command::Analyze(_) => 9,                   // Always very high complexity
            Command::Monitor(_) => 7,                   // Real-time monitoring complex
            _ => 3,
        }
    }
}
```

#### 3. Decision Logic Engine
```rust
impl InterfaceSelector {
    fn analyze_and_decide(&self, command: &Command) -> InterfaceDecision {
        let analysis = self.command_analyzer.analyze(command);
        
        // Early exit conditions
        if let Some(decision) = self.check_forced_contexts(&analysis) {
            return decision;
        }
        
        if let Some(decision) = self.check_user_preferences(command) {
            return decision;
        }
        
        // Smart decision based on multiple factors
        self.make_intelligent_decision(command, &analysis)
    }
    
    fn check_forced_contexts(&self, analysis: &CommandAnalysis) -> Option<InterfaceDecision> {
        // Never TUI in non-interactive contexts
        if !self.context.is_tty || self.context.is_piped || self.context.is_scripted {
            return Some(InterfaceDecision {
                strategy: InterfaceStrategy::CliOnly,
                confidence: 1.0,
                reason: "Non-interactive environment detected".to_string(),
                fallback: None,
            });
        }
        
        // Force CLI for automation-optimized commands
        if analysis.automation_suitability == AutomationLevel::OptimizedForAutomation {
            return Some(InterfaceDecision {
                strategy: InterfaceStrategy::CliOnly,
                confidence: 0.9,
                reason: "Command optimized for automation".to_string(),
                fallback: None,
            });
        }
        
        None
    }
    
    fn make_intelligent_decision(&self, command: &Command, analysis: &CommandAnalysis) -> InterfaceDecision {
        let complexity_weight = 0.4;
        let interaction_weight = 0.3;
        let data_volume_weight = 0.2;
        let user_pattern_weight = 0.1;
        
        let complexity_score = analysis.complexity_score as f32 / 10.0;
        let interaction_score = self.interaction_benefit_score(&analysis.interaction_benefit);
        let data_volume_score = self.data_volume_score(&analysis.data_volume_estimate);
        let user_pattern_score = self.get_user_pattern_score(command);
        
        let weighted_score = 
            complexity_score * complexity_weight +
            interaction_score * interaction_weight +
            data_volume_score * data_volume_weight +
            user_pattern_score * user_pattern_weight;
        
        let confidence = self.calculate_confidence(analysis, weighted_score);
        
        match (weighted_score, confidence) {
            // Low score: CLI only
            (score, _) if score < 0.3 => InterfaceDecision {
                strategy: InterfaceStrategy::CliOnly,
                confidence,
                reason: "Simple operation best suited for CLI".to_string(),
                fallback: None,
            },
            
            // Medium score: CLI with hint
            (score, conf) if score < 0.6 && conf > 0.7 => InterfaceDecision {
                strategy: InterfaceStrategy::CliWithHint {
                    hint: self.generate_helpful_hint(command, analysis),
                    tui_command: self.generate_tui_command(command),
                },
                confidence,
                reason: "TUI available for enhanced experience".to_string(),
                fallback: Some(InterfaceStrategy::CliOnly),
            },
            
            // Medium-high score: Prompt for TUI
            (score, conf) if score < 0.8 => InterfaceDecision {
                strategy: InterfaceStrategy::PromptForTui {
                    reason: self.generate_prompt_reason(analysis),
                    default_yes: score > 0.65,
                },
                confidence,
                reason: "Interactive interface likely beneficial".to_string(),
                fallback: Some(InterfaceStrategy::CliOnly),
            },
            
            // High score: Auto-launch TUI
            (_, conf) => InterfaceDecision {
                strategy: InterfaceStrategy::AutoLaunchTui {
                    reason: "Complex operation best suited for interactive interface".to_string(),
                    show_cli_first: conf < 0.9,
                },
                confidence,
                reason: "TUI provides optimal experience for this operation".to_string(),
                fallback: Some(InterfaceStrategy::CliOnly),
            },
        }
    }
}
```

#### 4. User Preference Integration
```rust
#[derive(Debug, Clone)]
pub struct UserConfig {
    pub global_ui_mode: UiMode,
    pub complexity_threshold: u8,
    pub auto_launch_threshold: f32,
    pub command_preferences: HashMap<String, UiMode>,
    pub hint_preferences: HintPreferences,
}

#[derive(Debug, Clone)]
pub struct HintPreferences {
    pub show_hints: bool,
    pub hint_frequency: HintFrequency,
    pub dismissed_hints: HashSet<String>,
}

#[derive(Debug, Clone)]
pub enum HintFrequency {
    Always,
    OncePerSession,
    OncePerCommand,
    Never,
}

impl InterfaceSelector {
    fn check_user_preferences(&self, command: &Command) -> Option<InterfaceDecision> {
        // Check command-specific preferences first
        let command_name = command.name();
        if let Some(mode) = self.config.command_preferences.get(&command_name) {
            return Some(self.create_preference_decision(mode, "User command preference"));
        }
        
        // Check global preferences
        match self.config.global_ui_mode {
            UiMode::Cli => Some(self.create_preference_decision(&UiMode::Cli, "User global preference: CLI")),
            UiMode::Tui => Some(self.create_preference_decision(&UiMode::Tui, "User global preference: TUI")),
            UiMode::Minimal => Some(self.create_preference_decision(&UiMode::Minimal, "User global preference: Minimal")),
            UiMode::Auto => None, // Continue with smart detection
        }
    }
    
    fn create_preference_decision(&self, mode: &UiMode, reason: &str) -> InterfaceDecision {
        let strategy = match mode {
            UiMode::Cli => InterfaceStrategy::CliOnly,
            UiMode::Tui => InterfaceStrategy::AutoLaunchTui {
                reason: "User preference".to_string(),
                show_cli_first: false,
            },
            UiMode::Minimal => InterfaceStrategy::CliOnly,
            UiMode::Auto => unreachable!(),
        };
        
        InterfaceDecision {
            strategy,
            confidence: 1.0,
            reason: reason.to_string(),
            fallback: None,
        }
    }
}
```

### Why This Approach

**Progressive Disclosure Benefits:**
- Respects user expertise and preferences
- Provides gentle guidance for complex tasks
- Avoids overwhelming beginners with too many options
- Maintains CLI efficiency for simple operations

**Smart Detection Advantages:**
- Context-aware interface selection
- Learns from user patterns over time
- Provides consistent experience across commands
- Optimizes for both productivity and discoverability

**Fallback Strategy:**
- Always provides CLI fallback for automation
- Graceful degradation in unsupported environments
- User override capabilities at any level
- Emergency escape mechanisms

### How to Implement

#### Step 1: Core Infrastructure (4 hours)
1. **Create interface selection module** with base structures
2. **Implement ExecutionContext integration** from Issue #002
3. **Add basic decision logic framework** with weighted scoring
4. **Create interface decision caching** for performance

#### Step 2: Command Analysis Engine (6 hours)
1. **Implement complexity calculation** for all command types
2. **Add data volume estimation** based on command parameters
3. **Create interaction benefit assessment** logic
4. **Build automation suitability detection** system

#### Step 3: Decision Logic Implementation (5 hours)
1. **Implement weighted scoring algorithm** for interface selection
2. **Add confidence calculation** based on multiple factors
3. **Create fallback strategy logic** for robust operation
4. **Implement hint generation** and TUI command suggestion

#### Step 4: User Preference System (3 hours)
1. **Integrate user configuration** from Issue #004
2. **Add command-specific preference** handling
3. **Implement preference override** logic
4. **Create hint dismissal** and frequency management

#### Step 5: Integration and Testing (2 hours)
1. **Integrate with CLI framework** from Issue #001
2. **Add performance optimization** and caching
3. **Implement analytics hooks** for A/B testing
4. **Create comprehensive test suite**

## 🔗 Dependencies
**Depends On:** 
- Issue #001 (Hybrid Command Framework) - Required for command structures
- Issue #002 (Execution Context Detection) - Required for context analysis

**Blocks:** 
- Issue #005-008 (Docker-like Commands) - Need interface selection
- Issue #009-011 (TUI Integration) - Requires interface decisions

## 🧪 Testing Requirements

### Unit Tests
```rust
#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_simple_command_selection() {
        let context = ExecutionContext::mock_interactive();
        let config = UserConfig::default();
        let mut selector = InterfaceSelector::new(context, config);
        
        let command = Command::Ps(PsCommand::default());
        let decision = selector.select_interface(&command);
        
        assert!(matches!(decision.strategy, InterfaceStrategy::CliOnly));
        assert!(decision.confidence > 0.8);
    }
    
    #[test]
    fn test_complex_command_auto_tui() {
        let context = ExecutionContext::mock_interactive();
        let config = UserConfig::default();
        let mut selector = InterfaceSelector::new(context, config);
        
        let command = Command::Debug(DebugCommand {
            dataflow: "complex-pipeline".to_string(),
            interactive: true,
        });
        let decision = selector.select_interface(&command);
        
        assert!(matches!(decision.strategy, InterfaceStrategy::AutoLaunchTui { .. }));
    }
    
    #[test]
    fn test_non_interactive_context() {
        let context = ExecutionContext::mock_non_interactive();
        let config = UserConfig::default();
        let mut selector = InterfaceSelector::new(context, config);
        
        let command = Command::Debug(DebugCommand {
            dataflow: "any-pipeline".to_string(),
            interactive: false,
        });
        let decision = selector.select_interface(&command);
        
        assert!(matches!(decision.strategy, InterfaceStrategy::CliOnly));
        assert_eq!(decision.reason, "Non-interactive environment detected");
    }
    
    #[test]
    fn test_user_preference_override() {
        let context = ExecutionContext::mock_interactive();
        let mut config = UserConfig::default();
        config.global_ui_mode = UiMode::Cli;
        let mut selector = InterfaceSelector::new(context, config);
        
        let command = Command::Debug(DebugCommand {
            dataflow: "complex-pipeline".to_string(),
            interactive: true,
        });
        let decision = selector.select_interface(&command);
        
        assert!(matches!(decision.strategy, InterfaceStrategy::CliOnly));
        assert!(decision.reason.contains("User global preference"));
    }
    
    #[test]
    fn test_decision_caching() {
        let context = ExecutionContext::mock_interactive();
        let config = UserConfig::default();
        let mut selector = InterfaceSelector::new(context, config);
        
        let command = Command::Ps(PsCommand::default());
        
        // First call
        let start = std::time::Instant::now();
        let decision1 = selector.select_interface(&command);
        let first_duration = start.elapsed();
        
        // Second call (should use cache)
        let start = std::time::Instant::now();
        let decision2 = selector.select_interface(&command);
        let second_duration = start.elapsed();
        
        assert_eq!(decision1.strategy, decision2.strategy);
        assert!(second_duration < first_duration / 2); // Cache should be much faster
    }
}
```

### Integration Tests
```rust
// tests/interface_selection_integration.rs
#[tokio::test]
async fn test_end_to_end_interface_selection() {
    let cli = Cli::try_parse_from(["dora", "inspect", "complex-node"]).unwrap();
    let context = ExecutionContext::detect();
    let config = UserConfig::load().unwrap_or_default();
    
    let mut selector = InterfaceSelector::new(context, config);
    let decision = selector.select_interface(&cli.command);
    
    // Verify decision is reasonable for the context
    match context.is_tty {
        true => assert!(!matches!(decision.strategy, InterfaceStrategy::CliOnly)),
        false => assert!(matches!(decision.strategy, InterfaceStrategy::CliOnly)),
    }
}

#[test]
fn test_complexity_analysis_accuracy() {
    let analyzer = CommandAnalyzer::new();
    
    // Simple command should have low complexity
    let simple = Command::Ps(PsCommand::default());
    let analysis = analyzer.analyze(&simple);
    assert!(analysis.complexity_score <= 3);
    
    // Complex command should have high complexity
    let complex = Command::Analyze(AnalyzeCommand {
        recording: "large-recording".to_string(),
        interactive: true,
        deep_analysis: true,
    });
    let analysis = analyzer.analyze(&complex);
    assert!(analysis.complexity_score >= 7);
}
```

### Manual Testing Procedures
1. **Interface Selection Validation**
   ```bash
   # Test simple commands stay CLI
   dora ps                    # Should be CLI only
   dora start simple.yml      # Should be CLI only
   
   # Test complex commands suggest TUI
   dora inspect complex-node  # Should suggest or auto-launch TUI
   dora debug large-pipeline  # Should auto-launch TUI
   ```

2. **Context Detection Testing**
   ```bash
   # Test non-interactive contexts
   echo "dora debug pipeline" | bash    # Should force CLI
   CI=true dora debug pipeline          # Should force CLI
   dora debug pipeline | grep "error"   # Should force CLI
   ```

3. **User Preference Testing**
   ```bash
   # Test preference overrides
   dora config set ui.mode cli
   dora debug pipeline                  # Should stay CLI despite complexity
   
   dora config set ui.mode tui
   dora ps                             # Should suggest/launch TUI
   ```

## 📚 Resources

### Algorithm References
- [Progressive Disclosure in UI Design](https://www.nngroup.com/articles/progressive-disclosure/)
- [Context-Aware Computing Patterns](https://dl.acm.org/doi/10.1145/2858036.2858529)
- [Command Line Interface Design Principles](https://clig.dev/)

### Code Examples
```rust
// Example of generating helpful hints
impl InterfaceSelector {
    fn generate_helpful_hint(&self, command: &Command, analysis: &CommandAnalysis) -> String {
        match (command, analysis.interaction_benefit) {
            (Command::Logs(_), InteractionBenefit::High) => {
                "Try 'dora ui logs' for interactive filtering and search".to_string()
            },
            (Command::Inspect(_), InteractionBenefit::Medium) => {
                "Use 'dora ui inspect' for live metrics and interactive exploration".to_string()
            },
            _ => format!(
                "For enhanced experience, try 'dora ui {}'", 
                command.name()
            ),
        }
    }
    
    fn generate_tui_command(&self, command: &Command) -> String {
        match command {
            Command::Ps(_) => "dora ui".to_string(),
            Command::Inspect(cmd) => format!("dora ui inspect {}", cmd.target),
            Command::Logs(cmd) => format!("dora ui logs {}", cmd.target),
            Command::Debug(cmd) => format!("dora ui debug {}", cmd.dataflow),
            _ => format!("dora ui {}", command.name()),
        }
    }
}
```

## ✅ Definition of Done
- [ ] InterfaceSelector struct implemented with all core functionality
- [ ] Command complexity analysis works accurately across all command types
- [ ] Smart interface selection provides appropriate decisions based on context
- [ ] User preference system integrates correctly with global and command-specific settings
- [ ] Decision caching improves performance for repeated operations
- [ ] Non-interactive context detection prevents inappropriate TUI launches
- [ ] Fallback strategies work correctly when primary interface unavailable
- [ ] Comprehensive unit tests cover all decision paths and edge cases
- [ ] Integration tests validate end-to-end interface selection workflow
- [ ] Manual testing confirms appropriate interface selection in various scenarios
- [ ] Performance benchmarks show decision-making completes in <5ms
- [ ] Analytics hooks enable A/B testing and usage pattern analysis
- [ ] Documentation includes decision algorithm explanation and tuning guide

## 📝 Implementation Notes

### Performance Considerations
- Cache frequently accessed decisions to avoid repeated analysis
- Use lazy evaluation for expensive operations (file analysis, external checks)
- Optimize hot paths for common commands (ps, start, stop)
- Profile decision-making latency and optimize bottlenecks

### Edge Cases to Handle
- Large dataflow files requiring complexity analysis
- Network timeouts during resource analysis
- Corrupted or invalid user preference files
- Race conditions between context detection and decision making
- Memory constraints with decision cache growth

### Future Extensions
- Machine learning for personalized interface suggestions
- A/B testing framework for interface decision optimization
- Telemetry integration for usage pattern analysis
- Plugin system for custom interface decision rules
- Multi-user environment preference inheritance

This interface selection engine provides the intelligent foundation that makes the hybrid CLI architecture feel magical to users while maintaining full control and predictability for automation scenarios.