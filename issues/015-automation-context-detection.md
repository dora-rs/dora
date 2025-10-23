# Issue #015: Build Automation Context Detection

## 📋 Summary
Implement sophisticated automation context detection that identifies CI/CD environments, scripting contexts, and automated workflows to ensure the hybrid CLI provides appropriate behavior for automation scenarios. This system prevents inappropriate TUI suggestions in automated environments while optimizing for human vs. machine interaction patterns.

## 🎯 Objectives
- Create comprehensive automation environment detection across all major CI/CD platforms
- Implement script context identification with high accuracy and minimal false positives
- Add machine interaction pattern recognition for API and automated tool usage
- Provide reliable automation-friendly behavior with appropriate output formatting
- Enable intelligent degradation from interactive to automation-optimized modes

**Success Metrics:**
- Automation detection accuracy exceeds 99% with <0.1% false positive rate
- CI/CD environment detection covers 95% of common platforms
- Script context identification works correctly in nested execution scenarios
- Machine interaction detection responds appropriately to API usage patterns
- Automated tool integration maintains 100% backward compatibility

## 🛠️ Technical Requirements

### What to Build

#### 1. Core Automation Detection Engine
```rust
// src/context/automation_detector.rs
#[derive(Debug)]
pub struct AutomationDetector {
    ci_detectors: Vec<Box<dyn CiEnvironmentDetector>>,
    script_detectors: Vec<Box<dyn ScriptContextDetector>>,
    machine_detectors: Vec<Box<dyn MachineInteractionDetector>>,
    pattern_analyzer: AutomationPatternAnalyzer,
    confidence_calculator: ConfidenceCalculator,
    detection_cache: LruCache<ContextKey, AutomationResult>,
}

#[derive(Debug, Clone)]
pub struct AutomationResult {
    pub is_automated: bool,
    pub automation_type: AutomationType,
    pub confidence: f32,
    pub detected_environment: Option<DetectedEnvironment>,
    pub interaction_patterns: Vec<InteractionPattern>,
    pub recommendations: AutomationRecommendations,
    pub evidence: Vec<AutomationEvidence>,
}

#[derive(Debug, Clone)]
pub enum AutomationType {
    Interactive,           // Human user in terminal
    CiCdPipeline,         // CI/CD environment
    ScriptedExecution,    // Shell script or automation script
    ApiIntegration,       // REST API or programmatic access
    ContainerizedWorkflow, // Docker/Kubernetes automation
    ScheduledTask,        // Cron job or scheduled execution
    TestingFramework,     // Automated testing environment
    MonitoringSystem,     // Health checks and monitoring
}

#[derive(Debug, Clone)]
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
        let mut detected_patterns = Vec::new();
        
        // Run CI/CD detection
        let ci_results = self.detect_ci_environment(&mut evidence);
        
        // Run script context detection
        let script_results = self.detect_script_context(context, &mut evidence);
        
        // Run machine interaction detection
        let machine_results = self.detect_machine_interaction(context, &mut evidence);
        
        // Analyze interaction patterns
        let interaction_patterns = self.pattern_analyzer.analyze_patterns(context, &evidence);
        
        // Determine automation type
        let automation_type = self.determine_automation_type(
            &ci_results,
            &script_results,
            &machine_results,
            &interaction_patterns,
        );
        
        // Calculate confidence
        let confidence = self.confidence_calculator.calculate_confidence(
            &automation_type,
            &evidence,
            &interaction_patterns,
        );
        
        // Generate recommendations
        let recommendations = self.generate_recommendations(&automation_type, context);
        
        // Detect specific environment details
        let detected_environment = self.detect_environment_details(&automation_type, &evidence);
        
        let result = AutomationResult {
            is_automated: !matches!(automation_type, AutomationType::Interactive),
            automation_type,
            confidence,
            detected_environment,
            interaction_patterns,
            recommendations,
            evidence,
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
        self.script_detectors.push(Box::new(PowerShellScriptDetector));
        self.script_detectors.push(Box::new(DockerContainerDetector));
        
        // Machine Interaction Detectors
        self.machine_detectors.push(Box::new(ApiClientDetector));
        self.machine_detectors.push(Box::new(CurlDetector));
        self.machine_detectors.push(Box::new(WgetDetector));
        self.machine_detectors.push(Box::new(AutomationToolDetector));
    }
}
```

#### 2. CI/CD Environment Detection
```rust
// src/context/ci_detectors.rs
pub trait CiEnvironmentDetector: Send + Sync {
    fn detect_ci_environment(&self) -> Option<CiDetectionResult>;
    fn platform_name(&self) -> &str;
    fn detection_priority(&self) -> u8; // Higher number = higher priority
}

#[derive(Debug, Clone)]
pub struct CiDetectionResult {
    pub platform: String,
    pub confidence: f32,
    pub environment_variables: HashMap<String, String>,
    pub build_information: Option<BuildInformation>,
    pub evidence: Vec<String>,
}

pub struct GitHubActionsDetector;

impl CiEnvironmentDetector for GitHubActionsDetector {
    fn detect_ci_environment(&self) -> Option<CiDetectionResult> {
        let github_actions_vars = [
            "GITHUB_ACTIONS",
            "GITHUB_WORKFLOW",
            "GITHUB_RUN_ID",
            "GITHUB_ACTOR",
            "GITHUB_REPOSITORY",
        ];
        
        let mut found_vars = HashMap::new();
        let mut evidence = Vec::new();
        
        for var in &github_actions_vars {
            if let Ok(value) = env::var(var) {
                found_vars.insert(var.to_string(), value);
                evidence.push(format!("Environment variable {} detected", var));
            }
        }
        
        if found_vars.contains_key("GITHUB_ACTIONS") {
            let confidence = if found_vars.len() >= 3 { 0.95 } else { 0.8 };
            
            let build_info = BuildInformation {
                build_id: found_vars.get("GITHUB_RUN_ID").cloned(),
                repository: found_vars.get("GITHUB_REPOSITORY").cloned(),
                branch: found_vars.get("GITHUB_REF_NAME").cloned(),
                commit: found_vars.get("GITHUB_SHA").cloned(),
                workflow: found_vars.get("GITHUB_WORKFLOW").cloned(),
                actor: found_vars.get("GITHUB_ACTOR").cloned(),
            };
            
            Some(CiDetectionResult {
                platform: "GitHub Actions".to_string(),
                confidence,
                environment_variables: found_vars,
                build_information: Some(build_info),
                evidence,
            })
        } else {
            None
        }
    }
    
    fn platform_name(&self) -> &str {
        "GitHub Actions"
    }
    
    fn detection_priority(&self) -> u8 {
        90
    }
}

pub struct GitLabCiDetector;

impl CiEnvironmentDetector for GitLabCiDetector {
    fn detect_ci_environment(&self) -> Option<CiDetectionResult> {
        let gitlab_vars = [
            "GITLAB_CI",
            "CI_JOB_ID",
            "CI_PIPELINE_ID",
            "CI_PROJECT_NAME",
            "CI_COMMIT_SHA",
        ];
        
        let mut found_vars = HashMap::new();
        let mut evidence = Vec::new();
        
        for var in &gitlab_vars {
            if let Ok(value) = env::var(var) {
                found_vars.insert(var.to_string(), value);
                evidence.push(format!("GitLab CI variable {} found", var));
            }
        }
        
        if found_vars.contains_key("GITLAB_CI") {
            let confidence = if found_vars.len() >= 3 { 0.95 } else { 0.8 };
            
            let build_info = BuildInformation {
                build_id: found_vars.get("CI_JOB_ID").cloned(),
                repository: found_vars.get("CI_PROJECT_NAME").cloned(),
                branch: found_vars.get("CI_COMMIT_REF_NAME").cloned(),
                commit: found_vars.get("CI_COMMIT_SHA").cloned(),
                workflow: found_vars.get("CI_PIPELINE_SOURCE").cloned(),
                actor: found_vars.get("GITLAB_USER_LOGIN").cloned(),
            };
            
            Some(CiDetectionResult {
                platform: "GitLab CI".to_string(),
                confidence,
                environment_variables: found_vars,
                build_information: Some(build_info),
                evidence,
            })
        } else {
            None
        }
    }
    
    fn platform_name(&self) -> &str {
        "GitLab CI"
    }
    
    fn detection_priority(&self) -> u8 {
        90
    }
}

pub struct JenkinsDetector;

impl CiEnvironmentDetector for JenkinsDetector {
    fn detect_ci_environment(&self) -> Option<CiDetectionResult> {
        let jenkins_vars = [
            "JENKINS_URL",
            "BUILD_NUMBER",
            "JOB_NAME",
            "WORKSPACE",
        ];
        
        let mut found_vars = HashMap::new();
        let mut evidence = Vec::new();
        
        for var in &jenkins_vars {
            if let Ok(value) = env::var(var) {
                found_vars.insert(var.to_string(), value);
                evidence.push(format!("Jenkins variable {} detected", var));
            }
        }
        
        // Also check for Jenkins-specific indicators
        if env::var("JENKINS_URL").is_ok() || env::var("HUDSON_URL").is_ok() {
            let confidence = if found_vars.len() >= 2 { 0.9 } else { 0.7 };
            
            let build_info = BuildInformation {
                build_id: found_vars.get("BUILD_NUMBER").cloned(),
                repository: found_vars.get("JOB_NAME").cloned(),
                branch: found_vars.get("GIT_BRANCH").cloned(),
                commit: found_vars.get("GIT_COMMIT").cloned(),
                workflow: found_vars.get("JOB_NAME").cloned(),
                actor: found_vars.get("BUILD_USER").cloned(),
            };
            
            Some(CiDetectionResult {
                platform: "Jenkins".to_string(),
                confidence,
                environment_variables: found_vars,
                build_information: Some(build_info),
                evidence,
            })
        } else {
            None
        }
    }
    
    fn platform_name(&self) -> &str {
        "Jenkins"
    }
    
    fn detection_priority(&self) -> u8 {
        85
    }
}
```

#### 3. Script Context Detection
```rust
// src/context/script_detectors.rs
pub trait ScriptContextDetector: Send + Sync {
    fn detect_script_context(&self, context: &ExecutionContext) -> Option<ScriptDetectionResult>;
    fn script_type(&self) -> &str;
}

#[derive(Debug, Clone)]
pub struct ScriptDetectionResult {
    pub script_type: String,
    pub confidence: f32,
    pub parent_process: Option<String>,
    pub execution_environment: ExecutionEnvironment,
    pub indicators: Vec<String>,
}

pub struct BashScriptDetector;

impl ScriptContextDetector for BashScriptDetector {
    fn detect_script_context(&self, context: &ExecutionContext) -> Option<ScriptDetectionResult> {
        let mut indicators = Vec::new();
        let mut confidence = 0.0;
        
        // Check parent process
        if let Ok(parent_process) = Self::get_parent_process_name() {
            match parent_process.as_str() {
                "bash" | "sh" | "zsh" | "fish" => {
                    indicators.push(format!("Parent process: {}", parent_process));
                    confidence += 0.4;
                },
                _ => {}
            }
        }
        
        // Check environment variables
        if env::var("BASH").is_ok() || env::var("SHELL").map_or(false, |s| s.contains("bash")) {
            indicators.push("Bash environment detected".to_string());
            confidence += 0.3;
        }
        
        // Check if running in non-interactive mode
        if !context.is_tty {
            indicators.push("Non-interactive execution".to_string());
            confidence += 0.2;
        }
        
        // Check for common script indicators
        if env::var("TERM").map_or(false, |t| t == "dumb") {
            indicators.push("TERM=dumb indicates script execution".to_string());
            confidence += 0.3;
        }
        
        // Check for pipe redirection
        if context.is_piped {
            indicators.push("Output is piped".to_string());
            confidence += 0.2;
        }
        
        if confidence > 0.5 {
            Some(ScriptDetectionResult {
                script_type: "Bash Script".to_string(),
                confidence,
                parent_process: Self::get_parent_process_name().ok(),
                execution_environment: ExecutionEnvironment::Script,
                indicators,
            })
        } else {
            None
        }
    }
    
    fn script_type(&self) -> &str {
        "Bash Script"
    }
}

impl BashScriptDetector {
    fn get_parent_process_name() -> Result<String> {
        // Get parent process information
        #[cfg(unix)]
        {
            use std::process;
            let ppid = unsafe { libc::getppid() };
            
            // Read process name from /proc filesystem
            let proc_path = format!("/proc/{}/comm", ppid);
            if let Ok(name) = std::fs::read_to_string(&proc_path) {
                Ok(name.trim().to_string())
            } else {
                Err(anyhow!("Could not read parent process name"))
            }
        }
        
        #[cfg(windows)]
        {
            // Windows implementation would use WMI or system APIs
            Err(anyhow!("Parent process detection not implemented for Windows"))
        }
    }
}

pub struct DockerContainerDetector;

impl ScriptContextDetector for DockerContainerDetector {
    fn detect_script_context(&self, context: &ExecutionContext) -> Option<ScriptDetectionResult> {
        let mut indicators = Vec::new();
        let mut confidence = 0.0;
        
        // Check for Docker-specific files
        if std::path::Path::new("/.dockerenv").exists() {
            indicators.push("/.dockerenv file detected".to_string());
            confidence += 0.8;
        }
        
        // Check cgroup information
        if let Ok(cgroup_content) = std::fs::read_to_string("/proc/1/cgroup") {
            if cgroup_content.contains("docker") || cgroup_content.contains("containerd") {
                indicators.push("Docker cgroup detected".to_string());
                confidence += 0.6;
            }
        }
        
        // Check for container-specific environment variables
        let container_vars = ["CONTAINER", "DOCKER_CONTAINER", "KUBERNETES_SERVICE_HOST"];
        for var in &container_vars {
            if env::var(var).is_ok() {
                indicators.push(format!("Container environment variable {} detected", var));
                confidence += 0.3;
            }
        }
        
        // Check init process
        if let Ok(init_cmdline) = std::fs::read_to_string("/proc/1/cmdline") {
            if init_cmdline.contains("docker-init") || init_cmdline.contains("tini") {
                indicators.push("Container init process detected".to_string());
                confidence += 0.4;
            }
        }
        
        if confidence > 0.6 {
            Some(ScriptDetectionResult {
                script_type: "Docker Container".to_string(),
                confidence,
                parent_process: None,
                execution_environment: ExecutionEnvironment::Container,
                indicators,
            })
        } else {
            None
        }
    }
    
    fn script_type(&self) -> &str {
        "Docker Container"
    }
}
```

#### 4. Machine Interaction Detection
```rust
// src/context/machine_detectors.rs
pub trait MachineInteractionDetector: Send + Sync {
    fn detect_machine_interaction(&self, context: &ExecutionContext) -> Option<MachineInteractionResult>;
    fn interaction_type(&self) -> &str;
}

#[derive(Debug, Clone)]
pub struct MachineInteractionResult {
    pub interaction_type: String,
    pub confidence: f32,
    pub user_agent: Option<String>,
    pub automation_tool: Option<String>,
    pub api_indicators: Vec<String>,
}

pub struct ApiClientDetector;

impl MachineInteractionDetector for ApiClientDetector {
    fn detect_machine_interaction(&self, context: &ExecutionContext) -> Option<MachineInteractionResult> {
        let mut api_indicators = Vec::new();
        let mut confidence = 0.0;
        
        // Check for HTTP User-Agent patterns
        if let Ok(user_agent) = env::var("HTTP_USER_AGENT") {
            let automation_patterns = [
                "curl", "wget", "python-requests", "node-fetch", "axios",
                "golang", "java", "ruby", "automation", "bot", "scraper"
            ];
            
            for pattern in &automation_patterns {
                if user_agent.to_lowercase().contains(pattern) {
                    api_indicators.push(format!("Automation user agent detected: {}", pattern));
                    confidence += 0.4;
                    break;
                }
            }
        }
        
        // Check for API-specific environment variables
        let api_vars = [
            "API_KEY", "ACCESS_TOKEN", "AUTH_TOKEN", "BEARER_TOKEN",
            "CLIENT_ID", "CLIENT_SECRET", "WEBHOOK_URL"
        ];
        
        for var in &api_vars {
            if env::var(var).is_ok() {
                api_indicators.push(format!("API authentication variable {} detected", var));
                confidence += 0.2;
            }
        }
        
        // Check for programmatic execution patterns
        if !context.is_tty && context.is_piped {
            api_indicators.push("Non-interactive piped execution".to_string());
            confidence += 0.3;
        }
        
        // Check for JSON output preference
        if env::var("ACCEPT").map_or(false, |a| a.contains("application/json")) {
            api_indicators.push("JSON output preference indicated".to_string());
            confidence += 0.3;
        }
        
        if confidence > 0.4 {
            Some(MachineInteractionResult {
                interaction_type: "API Client".to_string(),
                confidence,
                user_agent: env::var("HTTP_USER_AGENT").ok(),
                automation_tool: self.detect_automation_tool(),
                api_indicators,
            })
        } else {
            None
        }
    }
    
    fn interaction_type(&self) -> &str {
        "API Client"
    }
}

impl ApiClientDetector {
    fn detect_automation_tool(&self) -> Option<String> {
        // Check parent process for known automation tools
        if let Ok(parent) = std::env::var("_") {
            let automation_tools = [
                "ansible", "terraform", "puppet", "chef", "saltstack",
                "kubernetes", "helm", "docker-compose", "jenkins"
            ];
            
            for tool in &automation_tools {
                if parent.contains(tool) {
                    return Some(tool.to_string());
                }
            }
        }
        
        None
    }
}

pub struct AutomationToolDetector;

impl MachineInteractionDetector for AutomationToolDetector {
    fn detect_machine_interaction(&self, context: &ExecutionContext) -> Option<MachineInteractionResult> {
        let automation_tools = [
            ("ANSIBLE_INVENTORY", "Ansible"),
            ("TF_VAR_", "Terraform"),
            ("PUPPET_", "Puppet"),
            ("CHEF_", "Chef"),
            ("SALTSTACK_", "SaltStack"),
            ("KUBERNETES_", "Kubernetes"),
            ("HELM_", "Helm"),
        ];
        
        let mut detected_tools = Vec::new();
        let mut confidence = 0.0;
        
        for (env_prefix, tool_name) in &automation_tools {
            let matching_vars: Vec<_> = env::vars()
                .filter(|(key, _)| key.starts_with(env_prefix))
                .collect();
            
            if !matching_vars.is_empty() {
                detected_tools.push(format!("{} environment detected", tool_name));
                confidence += 0.7;
                
                return Some(MachineInteractionResult {
                    interaction_type: "Automation Tool".to_string(),
                    confidence,
                    user_agent: None,
                    automation_tool: Some(tool_name.to_string()),
                    api_indicators: detected_tools,
                });
            }
        }
        
        None
    }
    
    fn interaction_type(&self) -> &str {
        "Automation Tool"
    }
}
```

### Why This Approach

**Comprehensive Detection:**
- Multi-layered detection across CI/CD, scripting, and API contexts
- High accuracy with minimal false positives
- Support for nested execution scenarios

**Extensible Architecture:**
- Plugin-based detector system for easy extension
- Configurable confidence thresholds
- Caching for performance optimization

**Automation-Friendly:**
- Preserves backward compatibility
- Appropriate output formatting for different contexts
- Clear evidence trail for debugging detection logic

### How to Implement

#### Step 1: Core Detection Engine (3 hours)
1. **Implement AutomationDetector** with detector registration
2. **Create AutomationResult** structure with comprehensive information
3. **Add caching system** for performance optimization
4. **Build confidence calculation** algorithms

#### Step 2: CI/CD Environment Detection (4 hours)
1. **Implement major CI/CD platform detectors** (GitHub Actions, GitLab CI, Jenkins)
2. **Add build information** extraction and parsing
3. **Create environment variable** analysis systems
4. **Add platform-specific** detection logic

#### Step 3: Script Context Detection (3 hours)
1. **Implement script execution** detectors for major shells
2. **Add container environment** detection (Docker, Kubernetes)
3. **Create parent process** analysis for nested execution
4. **Add execution environment** classification

#### Step 4: Machine Interaction Detection (2 hours)
1. **Implement API client** detection with user agent analysis
2. **Add automation tool** detection for major platforms
3. **Create programmatic usage** pattern recognition
4. **Add authentication context** detection

#### Step 5: Integration and Testing (2 hours)
1. **Integrate with execution context** detection
2. **Add comprehensive unit tests** for all detectors
3. **Test detection accuracy** across different scenarios
4. **Validate performance** and caching effectiveness

## 🔗 Dependencies
**Depends On:**
- Issue #002 (Execution Context Detection) - Base context information

**Blocks:** 
- All commands that need automation-aware behavior
- Interface selection engine enhancements

## 🧪 Testing Requirements

### Unit Tests
```rust
#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_github_actions_detection() {
        env::set_var("GITHUB_ACTIONS", "true");
        env::set_var("GITHUB_WORKFLOW", "CI");
        
        let detector = GitHubActionsDetector;
        let result = detector.detect_ci_environment().unwrap();
        
        assert_eq!(result.platform, "GitHub Actions");
        assert!(result.confidence > 0.8);
        
        env::remove_var("GITHUB_ACTIONS");
        env::remove_var("GITHUB_WORKFLOW");
    }
    
    #[test]
    fn test_script_context_detection() {
        let context = ExecutionContext {
            is_tty: false,
            is_piped: true,
            ..Default::default()
        };
        
        let detector = BashScriptDetector;
        let result = detector.detect_script_context(&context);
        
        assert!(result.is_some());
        assert!(result.unwrap().confidence > 0.3);
    }
    
    #[test]
    fn test_automation_detector_integration() {
        let mut detector = AutomationDetector::new();
        let context = ExecutionContext::mock_ci_environment();
        
        let result = detector.detect_automation_context(&context);
        
        assert!(result.is_automated);
        assert!(matches!(result.automation_type, AutomationType::CiCdPipeline));
    }
}
```

## ✅ Definition of Done
- [ ] AutomationDetector implemented with comprehensive detection capabilities
- [ ] CI/CD environment detection covers all major platforms accurately
- [ ] Script context detection works correctly in nested execution scenarios
- [ ] Machine interaction detection identifies automated tool usage patterns
- [ ] Detection accuracy meets performance targets with minimal false positives
- [ ] Caching system optimizes repeated detection scenarios
- [ ] Integration with execution context provides seamless automation awareness
- [ ] Comprehensive unit tests validate detection accuracy across scenarios
- [ ] Integration tests confirm end-to-end automation context workflows
- [ ] Manual testing validates detection in real CI/CD and automation environments

This automation context detection system ensures the hybrid CLI behaves appropriately across all execution environments while maintaining the rich interactive experience for human users.