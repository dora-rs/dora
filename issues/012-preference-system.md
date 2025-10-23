# Issue #012: Build Preference System

## 📋 Summary
Implement a comprehensive user preference system that learns from user behavior and provides intelligent defaults for the hybrid CLI/TUI experience. This system adapts to user patterns while providing explicit control over interface decisions and behavior customization.

## 🎯 Objectives
- Create adaptive preference system that learns from user behavior patterns
- Implement explicit preference controls for all interface decisions
- Add context-aware preference application based on usage scenarios
- Provide preference import/export and synchronization capabilities
- Ensure preferences enhance rather than complicate user experience

**Success Metrics:**
- Preference system accurately predicts user interface preferences 90% of the time
- User customization options cover all major interface decision points
- Preference loading completes in <10ms on application startup
- Behavioral learning improves user experience without being intrusive
- Preference synchronization works reliably across different environments

## 🛠️ Technical Requirements

### What to Build

#### 1. Preference Management Core
```rust
// src/config/preferences.rs
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UserPreferences {
    /// Interface behavior preferences
    pub interface: InterfacePreferences,
    
    /// Command-specific preferences
    pub commands: CommandPreferences,
    
    /// Behavioral learning data
    pub behavior: BehaviorPreferences,
    
    /// Environment-specific overrides
    pub environments: HashMap<String, EnvironmentPreferences>,
    
    /// Preference metadata
    pub metadata: PreferenceMetadata,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InterfacePreferences {
    /// Global UI mode preference
    pub default_ui_mode: UiMode,
    
    /// Complexity thresholds for auto-suggestions
    pub complexity_thresholds: ComplexityThresholds,
    
    /// Auto-launch settings
    pub auto_launch: AutoLaunchPreferences,
    
    /// Hint and suggestion preferences
    pub hints: HintPreferences,
    
    /// TUI-specific preferences
    pub tui: TuiPreferences,
    
    /// CLI-specific preferences
    pub cli: CliPreferences,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BehaviorPreferences {
    /// User action patterns
    pub action_patterns: HashMap<String, ActionPattern>,
    
    /// Command usage statistics
    pub command_usage: HashMap<String, UsageStatistics>,
    
    /// Interface choice history
    pub interface_choices: VecDeque<InterfaceChoice>,
    
    /// Learning settings
    pub learning_enabled: bool,
    
    /// Adaptation weights
    pub adaptation_weights: AdaptationWeights,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ActionPattern {
    pub pattern_type: PatternType,
    pub frequency: f32,
    pub last_seen: DateTime<Utc>,
    pub context_factors: Vec<ContextFactor>,
    pub preferred_interface: Option<UiMode>,
}

impl UserPreferences {
    pub fn load_or_create() -> Result<Self> {
        let config_path = Self::get_preferences_path()?;
        
        if config_path.exists() {
            let content = std::fs::read_to_string(&config_path)?;
            let preferences: UserPreferences = toml::from_str(&content)?;
            preferences.validate()?;
            Ok(preferences)
        } else {
            let default_prefs = Self::default();
            default_prefs.save()?;
            Ok(default_prefs)
        }
    }
    
    pub fn save(&self) -> Result<()> {
        let config_path = Self::get_preferences_path()?;
        self.ensure_preferences_dir()?;
        
        let content = toml::to_string_pretty(self)?;
        std::fs::write(&config_path, content)?;
        Ok(())
    }
    
    pub fn predict_interface_preference(
        &self,
        command: &Command,
        context: &ExecutionContext,
    ) -> InterfacePreferencePrediction {
        let mut prediction = InterfacePreferencePrediction::new();
        
        // Apply explicit preferences first
        if let Some(explicit_pref) = self.get_explicit_preference(command) {
            prediction.add_factor(PreferenceFactor {
                factor_type: FactorType::ExplicitPreference,
                weight: 1.0,
                preference: explicit_pref,
                confidence: 1.0,
            });
        }
        
        // Apply behavioral learning
        if self.behavior.learning_enabled {
            if let Some(behavioral_pref) = self.get_behavioral_preference(command, context) {
                prediction.add_factor(PreferenceFactor {
                    factor_type: FactorType::BehavioralLearning,
                    weight: self.behavior.adaptation_weights.behavioral_weight,
                    preference: behavioral_pref.preference,
                    confidence: behavioral_pref.confidence,
                });
            }
        }
        
        // Apply context-based preferences
        if let Some(context_pref) = self.get_context_preference(command, context) {
            prediction.add_factor(PreferenceFactor {
                factor_type: FactorType::ContextBased,
                weight: self.behavior.adaptation_weights.context_weight,
                preference: context_pref.preference,
                confidence: context_pref.confidence,
            });
        }
        
        // Apply complexity-based preferences
        let complexity_pref = self.get_complexity_preference(command, context);
        prediction.add_factor(PreferenceFactor {
            factor_type: FactorType::ComplexityBased,
            weight: self.behavior.adaptation_weights.complexity_weight,
            preference: complexity_pref.preference,
            confidence: complexity_pref.confidence,
        });
        
        prediction.finalize()
    }
}
```

#### 2. Behavioral Learning System
```rust
// src/config/behavioral_learning.rs
#[derive(Debug)]
pub struct BehavioralLearningEngine {
    preferences: Arc<Mutex<UserPreferences>>,
    learning_config: LearningConfig,
    pattern_analyzer: PatternAnalyzer,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InterfaceChoice {
    pub command: String,
    pub context: ContextSnapshot,
    pub chosen_interface: UiMode,
    pub user_satisfaction: Option<SatisfactionLevel>,
    pub timestamp: DateTime<Utc>,
    pub session_id: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UsageStatistics {
    pub total_uses: u64,
    pub interface_distribution: HashMap<UiMode, u64>,
    pub success_rate: f32,
    pub average_session_length: Duration,
    pub last_used: DateTime<Utc>,
    pub user_rating: Option<f32>,
}

impl BehavioralLearningEngine {
    pub fn new(preferences: Arc<Mutex<UserPreferences>>) -> Self {
        Self {
            preferences,
            learning_config: LearningConfig::default(),
            pattern_analyzer: PatternAnalyzer::new(),
        }
    }
    
    pub async fn record_interface_choice(
        &self,
        command: &Command,
        context: &ExecutionContext,
        chosen_interface: UiMode,
        decision_trigger: DecisionTrigger,
    ) -> Result<()> {
        let interface_choice = InterfaceChoice {
            command: command.name().to_string(),
            context: ContextSnapshot::from_context(context),
            chosen_interface,
            user_satisfaction: None, // Will be updated based on user feedback
            timestamp: Utc::now(),
            session_id: self.get_current_session_id(),
        };
        
        let mut prefs = self.preferences.lock().unwrap();
        prefs.behavior.interface_choices.push_back(interface_choice);
        
        // Limit history size
        while prefs.behavior.interface_choices.len() > self.learning_config.max_history_size {
            prefs.behavior.interface_choices.pop_front();
        }
        
        // Update command usage statistics
        self.update_command_statistics(&mut prefs, command, &chosen_interface);
        
        // Analyze patterns and update preferences
        self.analyze_and_update_patterns(&mut prefs).await?;
        
        // Save preferences periodically
        if prefs.behavior.interface_choices.len() % 10 == 0 {
            drop(prefs); // Release lock before save
            self.save_preferences().await?;
        }
        
        Ok(())
    }
    
    pub async fn record_user_feedback(
        &self,
        session_id: &str,
        satisfaction: SatisfactionLevel,
        feedback: Option<String>,
    ) -> Result<()> {
        let mut prefs = self.preferences.lock().unwrap();
        
        // Find recent interface choices for this session
        for choice in prefs.behavior.interface_choices.iter_mut().rev().take(5) {
            if choice.session_id == session_id && choice.user_satisfaction.is_none() {
                choice.user_satisfaction = Some(satisfaction);
                break;
            }
        }
        
        // Update learning weights based on feedback
        self.adjust_learning_weights(&mut prefs, satisfaction);
        
        Ok(())
    }
    
    async fn analyze_and_update_patterns(
        &self,
        prefs: &mut UserPreferences,
    ) -> Result<()> {
        // Analyze command patterns
        let command_patterns = self.pattern_analyzer.analyze_command_patterns(
            &prefs.behavior.interface_choices
        );
        
        // Analyze context patterns
        let context_patterns = self.pattern_analyzer.analyze_context_patterns(
            &prefs.behavior.interface_choices
        );
        
        // Update action patterns
        for (pattern_key, pattern) in command_patterns {
            prefs.behavior.action_patterns.insert(pattern_key, pattern);
        }
        
        for (pattern_key, pattern) in context_patterns {
            prefs.behavior.action_patterns.insert(pattern_key, pattern);
        }
        
        // Clean old patterns
        self.clean_old_patterns(prefs);
        
        Ok(())
    }
    
    fn update_command_statistics(
        &self,
        prefs: &mut UserPreferences,
        command: &Command,
        chosen_interface: &UiMode,
    ) {
        let command_name = command.name().to_string();
        let stats = prefs.behavior.command_usage
            .entry(command_name)
            .or_insert_with(UsageStatistics::default);
        
        stats.total_uses += 1;
        *stats.interface_distribution.entry(chosen_interface.clone()).or_insert(0) += 1;
        stats.last_used = Utc::now();
    }
}

#[derive(Debug)]
pub struct PatternAnalyzer {
    pattern_detectors: Vec<Box<dyn PatternDetector>>,
}

trait PatternDetector: Send + Sync {
    fn detect_patterns(&self, choices: &[InterfaceChoice]) -> HashMap<String, ActionPattern>;
    fn pattern_type(&self) -> PatternType;
}

struct CommandSequenceDetector;

impl PatternDetector for CommandSequenceDetector {
    fn detect_patterns(&self, choices: &[InterfaceChoice]) -> HashMap<String, ActionPattern> {
        let mut patterns = HashMap::new();
        
        // Look for common command sequences
        for window in choices.windows(3) {
            let sequence = window.iter()
                .map(|c| c.command.clone())
                .collect::<Vec<_>>()
                .join(" -> ");
            
            // Check if this sequence has consistent interface choices
            let interfaces: Vec<_> = window.iter()
                .map(|c| c.chosen_interface.clone())
                .collect();
            
            if interfaces.iter().all(|i| i == &interfaces[0]) {
                let pattern = ActionPattern {
                    pattern_type: PatternType::CommandSequence,
                    frequency: 1.0, // Will be aggregated
                    last_seen: window.last().unwrap().timestamp,
                    context_factors: vec![],
                    preferred_interface: Some(interfaces[0].clone()),
                };
                
                patterns.insert(format!("seq:{}", sequence), pattern);
            }
        }
        
        patterns
    }
    
    fn pattern_type(&self) -> PatternType {
        PatternType::CommandSequence
    }
}
```

#### 3. Preference UI and Management
```rust
// src/cli/commands/preferences.rs
#[derive(Debug, clap::Args)]
pub struct PreferencesCommand {
    #[clap(subcommand)]
    pub action: PreferencesAction,
}

#[derive(Debug, clap::Subcommand)]
pub enum PreferencesAction {
    /// Show current preferences
    Show {
        /// Filter by preference category
        #[clap(long)]
        category: Option<PreferenceCategory>,
        
        /// Show learned behavioral patterns
        #[clap(long)]
        patterns: bool,
    },
    
    /// Set a preference value
    Set {
        /// Preference key (dot notation)
        key: String,
        
        /// Preference value
        value: String,
        
        /// Apply to specific environment
        #[clap(long)]
        environment: Option<String>,
    },
    
    /// Reset preferences to defaults
    Reset {
        /// Specific category to reset
        category: Option<PreferenceCategory>,
        
        /// Include behavioral learning data
        #[clap(long)]
        include_behavior: bool,
    },
    
    /// Export preferences to file
    Export {
        /// Output file path
        file: PathBuf,
        
        /// Include behavioral data
        #[clap(long)]
        include_behavior: bool,
    },
    
    /// Import preferences from file
    Import {
        /// Input file path
        file: PathBuf,
        
        /// Merge with existing preferences
        #[clap(long)]
        merge: bool,
    },
    
    /// Configure behavioral learning
    Learning {
        /// Enable or disable learning
        #[clap(long)]
        enable: Option<bool>,
        
        /// Reset learning data
        #[clap(long)]
        reset: bool,
        
        /// Show learning statistics
        #[clap(long)]
        stats: bool,
    },
    
    /// Launch interactive preference editor
    Edit,
}

impl PreferencesCommand {
    pub async fn execute(&self, context: &ExecutionContext) -> Result<()> {
        match &self.action {
            PreferencesAction::Show { category, patterns } => {
                self.show_preferences(category.as_ref(), *patterns).await?;
            },
            
            PreferencesAction::Set { key, value, environment } => {
                self.set_preference(key, value, environment.as_deref()).await?;
            },
            
            PreferencesAction::Reset { category, include_behavior } => {
                self.reset_preferences(category.as_ref(), *include_behavior).await?;
            },
            
            PreferencesAction::Export { file, include_behavior } => {
                self.export_preferences(file, *include_behavior).await?;
            },
            
            PreferencesAction::Import { file, merge } => {
                self.import_preferences(file, *merge).await?;
            },
            
            PreferencesAction::Learning { enable, reset, stats } => {
                self.manage_learning(*enable, *reset, *stats).await?;
            },
            
            PreferencesAction::Edit => {
                self.launch_preference_editor().await?;
            },
        }
        
        Ok(())
    }
    
    async fn show_preferences(
        &self,
        category: Option<&PreferenceCategory>,
        show_patterns: bool,
    ) -> Result<()> {
        let preferences = UserPreferences::load_or_create()?;
        
        match category {
            Some(PreferenceCategory::Interface) => {
                self.display_interface_preferences(&preferences.interface);
            },
            Some(PreferenceCategory::Commands) => {
                self.display_command_preferences(&preferences.commands);
            },
            Some(PreferenceCategory::Behavior) => {
                self.display_behavior_preferences(&preferences.behavior, show_patterns);
            },
            None => {
                self.display_all_preferences(&preferences, show_patterns);
            },
        }
        
        Ok(())
    }
    
    fn display_interface_preferences(&self, prefs: &InterfacePreferences) {
        println!("🎨 Interface Preferences");
        println!("  Default UI Mode: {}", prefs.default_ui_mode);
        println!("  Complexity Threshold: {}", prefs.complexity_thresholds.suggestion_threshold);
        println!("  Auto-launch Threshold: {:.1}", prefs.auto_launch.confidence_threshold);
        println!("  Show Hints: {}", prefs.hints.show_hints);
        println!("  Hint Frequency: {}", prefs.hints.hint_frequency);
        
        if !prefs.cli.command_aliases.is_empty() {
            println!("\n  CLI Aliases:");
            for (alias, command) in &prefs.cli.command_aliases {
                println!("    {} → {}", alias, command);
            }
        }
        
        println!("\n  TUI Settings:");
        println!("    Theme: {}", prefs.tui.theme);
        println!("    Auto-refresh: {}s", prefs.tui.auto_refresh_interval.as_secs());
        println!("    Mouse Support: {}", prefs.tui.mouse_support);
    }
    
    fn display_behavior_preferences(&self, prefs: &BehaviorPreferences, show_patterns: bool) {
        println!("🧠 Behavioral Learning");
        println!("  Learning Enabled: {}", prefs.learning_enabled);
        println!("  Total Interface Choices: {}", prefs.interface_choices.len());
        println!("  Command Usage Entries: {}", prefs.command_usage.len());
        
        if !prefs.command_usage.is_empty() {
            println!("\n  Most Used Commands:");
            let mut sorted_commands: Vec<_> = prefs.command_usage.iter().collect();
            sorted_commands.sort_by_key(|(_, stats)| std::cmp::Reverse(stats.total_uses));
            
            for (command, stats) in sorted_commands.iter().take(5) {
                let primary_interface = stats.interface_distribution.iter()
                    .max_by_key(|(_, count)| *count)
                    .map(|(interface, _)| interface.to_string())
                    .unwrap_or_else(|| "Unknown".to_string());
                
                println!("    {} ({} uses, prefers {})", 
                        command, stats.total_uses, primary_interface);
            }
        }
        
        if show_patterns && !prefs.action_patterns.is_empty() {
            println!("\n  Learned Patterns:");
            for (pattern_key, pattern) in &prefs.action_patterns {
                println!("    {} (frequency: {:.2}, last seen: {})",
                        pattern_key,
                        pattern.frequency,
                        pattern.last_seen.format("%Y-%m-%d"));
            }
        }
    }
    
    async fn launch_preference_editor(&self) -> Result<()> {
        println!("🚀 Launching interactive preference editor...");
        
        let tui_app = DoraApp::new_with_context(
            ViewType::PreferenceEditor,
            CliContext::from_preferences_command(self),
        );
        
        tui_app.run().await?;
        Ok(())
    }
}
```

#### 4. Context-Aware Preference Application
```rust
// src/config/context_preferences.rs
#[derive(Debug)]
pub struct ContextAwarePreferences {
    base_preferences: UserPreferences,
    context_cache: LruCache<ContextKey, ContextualPreferences>,
    adaptation_engine: AdaptationEngine,
}

#[derive(Debug, Clone, Hash, PartialEq, Eq)]
pub struct ContextKey {
    pub environment: String,
    pub time_of_day: u8, // Hour of day
    pub day_of_week: u8,
    pub terminal_type: String,
    pub command_category: String,
}

#[derive(Debug, Clone)]
pub struct ContextualPreferences {
    pub ui_mode_preference: UiMode,
    pub complexity_threshold: u8,
    pub auto_launch_threshold: f32,
    pub confidence: f32,
    pub last_updated: DateTime<Utc>,
}

impl ContextAwarePreferences {
    pub fn get_contextual_preference(
        &mut self,
        command: &Command,
        context: &ExecutionContext,
    ) -> ContextualPreferences {
        let context_key = self.create_context_key(command, context);
        
        if let Some(cached_prefs) = self.context_cache.get(&context_key) {
            if cached_prefs.last_updated > Utc::now() - Duration::hours(24) {
                return cached_prefs.clone();
            }
        }
        
        // Compute contextual preferences
        let contextual_prefs = self.compute_contextual_preferences(command, context);
        self.context_cache.put(context_key, contextual_prefs.clone());
        
        contextual_prefs
    }
    
    fn compute_contextual_preferences(
        &self,
        command: &Command,
        context: &ExecutionContext,
    ) -> ContextualPreferences {
        let mut preference_factors = Vec::new();
        
        // Time-based preferences
        let hour = Local::now().hour();
        match hour {
            6..=9 => {
                // Morning: prefer efficient CLI
                preference_factors.push((UiMode::Cli, 0.7));
            },
            10..=16 => {
                // Work hours: balanced approach
                preference_factors.push((UiMode::Auto, 0.8));
            },
            17..=22 => {
                // Evening: prefer interactive TUI
                preference_factors.push((UiMode::Tui, 0.6));
            },
            _ => {
                // Late night/early morning: prefer minimal
                preference_factors.push((UiMode::Minimal, 0.8));
            }
        }
        
        // Environment-based preferences
        if context.is_scripted || !context.is_tty {
            preference_factors.push((UiMode::Minimal, 0.9));
        }
        
        if context.terminal_size.map_or(false, |(w, h)| w < 80 || h < 24) {
            preference_factors.push((UiMode::Cli, 0.8));
        }
        
        // Command complexity-based preferences
        let complexity_score = self.calculate_command_complexity(command);
        if complexity_score > 7 {
            preference_factors.push((UiMode::Tui, 0.8));
        } else if complexity_score < 3 {
            preference_factors.push((UiMode::Cli, 0.7));
        }
        
        // Compute weighted preference
        let preference_weights: HashMap<UiMode, f32> = preference_factors.iter()
            .fold(HashMap::new(), |mut acc, (mode, weight)| {
                *acc.entry(mode.clone()).or_insert(0.0) += weight;
                acc
            });
        
        let preferred_mode = preference_weights.iter()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .map(|(mode, _)| mode.clone())
            .unwrap_or(UiMode::Auto);
        
        let confidence = preference_weights.values().max_by(|a, b| a.partial_cmp(b).unwrap())
            .copied()
            .unwrap_or(0.5);
        
        ContextualPreferences {
            ui_mode_preference: preferred_mode,
            complexity_threshold: self.base_preferences.interface.complexity_thresholds.suggestion_threshold,
            auto_launch_threshold: self.base_preferences.interface.auto_launch.confidence_threshold,
            confidence,
            last_updated: Utc::now(),
        }
    }
}
```

### Why This Approach

**Intelligent Adaptation:**
- Learns from user behavior without being intrusive
- Provides smart defaults that improve over time
- Respects explicit user preferences over learned behavior
- Adapts to different contexts and usage patterns

**Comprehensive Control:**
- Explicit preference controls for all interface decisions
- Environment-specific preference overrides
- Fine-grained control over learning and adaptation
- Import/export capabilities for preference management

**Performance Optimized:**
- Fast preference loading with caching
- Efficient pattern analysis and storage
- Minimal overhead for behavioral learning
- Context-aware caching for repeated scenarios

### How to Implement

#### Step 1: Preference Core Infrastructure (4 hours)
1. **Implement UserPreferences** structure with serialization
2. **Add preference loading** and saving mechanisms
3. **Create preference validation** and migration
4. **Add basic preference** prediction system

#### Step 2: Behavioral Learning Engine (5 hours)
1. **Implement BehavioralLearningEngine** with pattern analysis
2. **Add interface choice** recording and tracking
3. **Create pattern detection** and analysis algorithms
4. **Add user feedback** integration and learning adjustment

#### Step 3: Preference Management CLI (3 hours)
1. **Implement PreferencesCommand** with all subcommands
2. **Add preference editing** and display functionality
3. **Create import/export** capabilities
4. **Add learning management** controls

#### Step 4: Context-Aware Application (2 hours)
1. **Implement ContextAwarePreferences** with caching
2. **Add contextual preference** computation
3. **Create environment-specific** preference overrides
4. **Add time and usage-based** adaptations

#### Step 5: Testing and Integration (2 hours)
1. **Add comprehensive unit tests** for all functionality
2. **Test behavioral learning** accuracy and performance
3. **Validate preference application** across different contexts
4. **Test import/export** and migration functionality

## 🔗 Dependencies
**Depends On:**
- Issue #001 (Hybrid Command Framework) - Command structures
- Issue #003 (Interface Selection Engine) - Interface decision integration
- Issue #004 (Configuration System) - Base configuration infrastructure

**Blocks:** All subsequent issues that rely on user preference customization

## 🧪 Testing Requirements

### Unit Tests
```rust
#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_preference_prediction() {
        let preferences = UserPreferences::default();
        let command = Command::Ps(PsCommand::default());
        let context = ExecutionContext::mock_interactive();
        
        let prediction = preferences.predict_interface_preference(&command, &context);
        
        assert!(prediction.confidence > 0.0);
        assert!(prediction.factors.len() > 0);
    }
    
    #[test]
    fn test_behavioral_learning() {
        let mut engine = BehavioralLearningEngine::new(Arc::new(Mutex::new(UserPreferences::default())));
        let command = Command::Ps(PsCommand::default());
        let context = ExecutionContext::mock_interactive();
        
        engine.record_interface_choice(&command, &context, UiMode::Tui, DecisionTrigger::UserChoice).await.unwrap();
        
        // Verify choice was recorded
        let prefs = engine.preferences.lock().unwrap();
        assert_eq!(prefs.behavior.interface_choices.len(), 1);
    }
    
    #[test]
    fn test_contextual_preferences() {
        let mut context_prefs = ContextAwarePreferences::new(UserPreferences::default());
        let command = Command::Debug(DebugCommand::default());
        let context = ExecutionContext::mock_interactive();
        
        let contextual = context_prefs.get_contextual_preference(&command, &context);
        
        assert!(contextual.confidence > 0.0);
        assert!(matches!(contextual.ui_mode_preference, UiMode::Tui | UiMode::Auto));
    }
}
```

## ✅ Definition of Done
- [ ] UserPreferences structure implemented with full serialization support
- [ ] Behavioral learning engine records and analyzes user interface choices
- [ ] Preference prediction system provides accurate interface recommendations
- [ ] Context-aware preferences adapt to different usage scenarios
- [ ] CLI preference management commands provide comprehensive control
- [ ] Learning system improves user experience without being intrusive
- [ ] Preference import/export works reliably for configuration management
- [ ] Performance targets met for preference loading and prediction
- [ ] Comprehensive unit tests cover all preference functionality
- [ ] Integration tests validate preference application across scenarios
- [ ] Manual testing confirms intuitive preference management experience

This preference system provides the intelligent foundation that makes the hybrid CLI architecture feel personalized and adaptive while maintaining full user control over interface decisions.