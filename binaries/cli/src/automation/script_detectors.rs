//! Script Context Detection
//!
//! This module implements detection for various scripting environments including
//! shell scripts, container environments, and other automation contexts.

use crate::cli::context::ExecutionContext;
use anyhow::{Result, anyhow};
use serde::{Deserialize, Serialize};
use std::path::Path;

pub trait ScriptContextDetector: Send + Sync + std::fmt::Debug {
    fn detect_script_context(&self, context: &ExecutionContext) -> Option<ScriptDetectionResult>;
    fn script_type(&self) -> &str;
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ScriptDetectionResult {
    pub script_type: String,
    pub confidence: f32,
    pub parent_process: Option<String>,
    pub execution_environment: ExecutionEnvironment,
    pub indicators: Vec<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum ExecutionEnvironment {
    Interactive,
    Script,
    Container,
    VirtualEnvironment,
    TestFramework,
}

#[derive(Debug)]
pub struct BashScriptDetector;

impl ScriptContextDetector for BashScriptDetector {
    fn detect_script_context(&self, context: &ExecutionContext) -> Option<ScriptDetectionResult> {
        let mut indicators = Vec::new();
        let mut confidence = 0.0;
        let env_map = &context.environment.relevant_env_vars;

        // Check parent process
        if let Ok(parent_process) = Self::get_parent_process_name() {
            match parent_process.as_str() {
                "bash" | "sh" | "zsh" | "fish" => {
                    indicators.push(format!("Parent process: {}", parent_process));
                    confidence += 0.4;
                }
                _ => {}
            }
        }

        // Check environment variables
        if env_map.contains_key("BASH")
            || env_map.get("SHELL").map_or(false, |s| s.contains("bash"))
        {
            indicators.push("Bash environment detected".to_string());
            confidence += 0.3;
        }

        // Check if running in non-interactive mode
        if !context.is_tty {
            indicators.push("Non-interactive execution".to_string());
            confidence += 0.2;
        }

        // Check for common script indicators
        if env_map.get("TERM").map_or(false, |t| t == "dumb") {
            indicators.push("TERM=dumb indicates script execution".to_string());
            confidence += 0.3;
        }

        // Check for pipe redirection
        if context.is_piped {
            indicators.push("Output is piped".to_string());
            confidence += 0.2;
        }

        // Check for script-specific environment variables
        if env_map.contains_key("BASH_EXECUTION_STRING") {
            indicators.push("Bash execution string detected".to_string());
            confidence += 0.4;
        }

        // Check for process substitution indicators
        if env_map.get("_").map_or(false, |underscore| {
            underscore.contains("bash") || underscore.contains("sh")
        }) {
            indicators.push("Script execution context detected via $_".to_string());
            confidence += 0.3;
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
            use std::fs;
            let ppid = unsafe { libc::getppid() };

            // Read process name from /proc filesystem
            let proc_path = format!("/proc/{}/comm", ppid);
            if let Ok(name) = fs::read_to_string(&proc_path) {
                Ok(name.trim().to_string())
            } else {
                // Fallback: try to read from /proc/pid/stat
                let stat_path = format!("/proc/{}/stat", ppid);
                if let Ok(stat_content) = fs::read_to_string(&stat_path) {
                    // Parse stat file - second field is the command name in parentheses
                    if let Some(start) = stat_content.find('(') {
                        if let Some(end) = stat_content.find(')') {
                            if end > start {
                                return Ok(stat_content[start + 1..end].to_string());
                            }
                        }
                    }
                }
                Err(anyhow!("Could not read parent process name"))
            }
        }

        #[cfg(windows)]
        {
            // Windows implementation would use WMI or system APIs
            Err(anyhow!(
                "Parent process detection not implemented for Windows"
            ))
        }
    }
}

#[derive(Debug)]
pub struct PythonScriptDetector;

impl ScriptContextDetector for PythonScriptDetector {
    fn detect_script_context(&self, context: &ExecutionContext) -> Option<ScriptDetectionResult> {
        let mut indicators = Vec::new();
        let mut confidence = 0.0;
        let env_map = &context.environment.relevant_env_vars;

        // Check for Python-specific environment variables
        if env_map.contains_key("PYTHONPATH") {
            indicators.push("PYTHONPATH environment variable detected".to_string());
            confidence += 0.3;
        }

        if env_map.contains_key("VIRTUAL_ENV") {
            indicators.push("Python virtual environment detected".to_string());
            confidence += 0.4;
        }

        if env_map.contains_key("CONDA_DEFAULT_ENV") {
            indicators.push("Conda environment detected".to_string());
            confidence += 0.4;
        }

        // Check parent process
        if let Ok(parent) = BashScriptDetector::get_parent_process_name() {
            if parent.contains("python") || parent.contains("pytest") || parent.contains("pip") {
                indicators.push(format!("Python parent process: {}", parent));
                confidence += 0.5;
            }
        }

        // Check for testing framework indicators
        if env_map.contains_key("PYTEST_CURRENT_TEST") {
            indicators.push("Pytest execution detected".to_string());
            confidence += 0.6;
            return Some(ScriptDetectionResult {
                script_type: "Python Test Framework".to_string(),
                confidence,
                parent_process: BashScriptDetector::get_parent_process_name().ok(),
                execution_environment: ExecutionEnvironment::TestFramework,
                indicators,
            });
        }

        // Check for non-interactive execution
        if !context.is_tty && context.is_piped {
            indicators.push("Non-interactive Python execution".to_string());
            confidence += 0.2;
        }

        // Check for script execution context
        if env_map
            .get("_")
            .map_or(false, |underscore| underscore.contains("python"))
        {
            indicators.push("Python script execution detected".to_string());
            confidence += 0.4;
        }

        if confidence > 0.4 {
            Some(ScriptDetectionResult {
                script_type: "Python Script".to_string(),
                confidence,
                parent_process: BashScriptDetector::get_parent_process_name().ok(),
                execution_environment: ExecutionEnvironment::Script,
                indicators,
            })
        } else {
            None
        }
    }

    fn script_type(&self) -> &str {
        "Python Script"
    }
}

#[derive(Debug)]
pub struct NodeScriptDetector;

impl ScriptContextDetector for NodeScriptDetector {
    fn detect_script_context(&self, context: &ExecutionContext) -> Option<ScriptDetectionResult> {
        let mut indicators = Vec::new();
        let mut confidence = 0.0;
        let env_map = &context.environment.relevant_env_vars;

        // Check for Node.js-specific environment variables
        if env_map.contains_key("NODE_ENV") {
            indicators.push("NODE_ENV environment variable detected".to_string());
            confidence += 0.3;
        }

        if env_map.contains_key("npm_config_user_config") {
            indicators.push("npm configuration detected".to_string());
            confidence += 0.4;
        }

        if env_map.contains_key("npm_lifecycle_event") {
            indicators.push("npm script execution detected".to_string());
            confidence += 0.5;
        }

        // Check parent process
        if let Ok(parent) = BashScriptDetector::get_parent_process_name() {
            if parent.contains("node") || parent.contains("npm") || parent.contains("yarn") {
                indicators.push(format!("Node.js parent process: {}", parent));
                confidence += 0.5;
            }
        }

        // Check for testing framework indicators
        if env_map.contains_key("JEST_WORKER_ID") {
            indicators.push("Jest test execution detected".to_string());
            confidence += 0.6;
            return Some(ScriptDetectionResult {
                script_type: "JavaScript Test Framework".to_string(),
                confidence,
                parent_process: BashScriptDetector::get_parent_process_name().ok(),
                execution_environment: ExecutionEnvironment::TestFramework,
                indicators,
            });
        }

        // Check for non-interactive execution
        if !context.is_tty && context.is_piped {
            indicators.push("Non-interactive Node.js execution".to_string());
            confidence += 0.2;
        }

        if confidence > 0.4 {
            Some(ScriptDetectionResult {
                script_type: "Node Script".to_string(),
                confidence,
                parent_process: BashScriptDetector::get_parent_process_name().ok(),
                execution_environment: ExecutionEnvironment::Script,
                indicators,
            })
        } else {
            None
        }
    }

    fn script_type(&self) -> &str {
        "Node Script"
    }
}

#[derive(Debug)]
pub struct PowerShellScriptDetector;

impl ScriptContextDetector for PowerShellScriptDetector {
    fn detect_script_context(&self, context: &ExecutionContext) -> Option<ScriptDetectionResult> {
        let mut indicators = Vec::new();
        let mut confidence = 0.0;
        let env_map = &context.environment.relevant_env_vars;

        // Check for PowerShell-specific environment variables
        if env_map.contains_key("PSModulePath") {
            indicators.push("PowerShell module path detected".to_string());
            confidence += 0.4;
        }

        if env_map.contains_key("PSExecutionPolicyPreference") {
            indicators.push("PowerShell execution policy detected".to_string());
            confidence += 0.3;
        }

        // Check parent process (mainly for Windows)
        if let Ok(parent) = BashScriptDetector::get_parent_process_name() {
            if parent.contains("powershell") || parent.contains("pwsh") {
                indicators.push(format!("PowerShell parent process: {}", parent));
                confidence += 0.6;
            }
        }

        // Check shell type from context
        if let Some(shell_type) = &context.environment.shell_type {
            if shell_type.contains("powershell") || shell_type.contains("pwsh") {
                indicators.push("PowerShell shell type detected".to_string());
                confidence += 0.5;
            }
        }

        // Check for non-interactive execution
        if !context.is_tty {
            indicators.push("Non-interactive PowerShell execution".to_string());
            confidence += 0.2;
        }

        if confidence > 0.4 {
            Some(ScriptDetectionResult {
                script_type: "PowerShell Script".to_string(),
                confidence,
                parent_process: BashScriptDetector::get_parent_process_name().ok(),
                execution_environment: ExecutionEnvironment::Script,
                indicators,
            })
        } else {
            None
        }
    }

    fn script_type(&self) -> &str {
        "PowerShell Script"
    }
}

#[derive(Debug)]
pub struct DockerContainerDetector;

impl ScriptContextDetector for DockerContainerDetector {
    fn detect_script_context(&self, context: &ExecutionContext) -> Option<ScriptDetectionResult> {
        let mut indicators = Vec::new();
        let mut confidence = 0.0;
        let env_map = &context.environment.relevant_env_vars;

        // Check for Docker-specific files
        if Path::new("/.dockerenv").exists() {
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
            if env_map.contains_key(*var) {
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

        // Check for Kubernetes-specific indicators
        if env_map.contains_key("KUBERNETES_SERVICE_HOST") {
            indicators.push("Kubernetes environment detected".to_string());
            confidence += 0.5;
        }

        // Check mounted filesystems for container indicators
        if let Ok(mounts_content) = std::fs::read_to_string("/proc/mounts") {
            if mounts_content.contains("overlay") || mounts_content.contains("aufs") {
                indicators.push("Container filesystem detected".to_string());
                confidence += 0.3;
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cli::context::ExecutionContext;

    #[test]
    fn test_bash_script_detection() {
        let mut context = ExecutionContext::detect_basic();
        context.is_tty = false;
        context.is_piped = true;
        context.is_scripted = true;
        context.environment.shell_type = Some("bash".to_string());
        context.environment.relevant_env_vars.clear();
        context
            .environment
            .relevant_env_vars
            .insert("TERM".to_string(), "dumb".to_string());

        let detector = BashScriptDetector;
        let result = detector.detect_script_context(&context);

        assert!(result.is_some());
        let result = result.unwrap();
        assert_eq!(result.script_type, "Bash Script");
        assert!(result.confidence > 0.3);
    }

    #[test]
    fn test_python_script_detection() {
        let mut context = ExecutionContext::detect_basic();
        context.environment.relevant_env_vars.clear();
        context
            .environment
            .relevant_env_vars
            .insert("PYTHONPATH".to_string(), "/usr/lib/python3.9".to_string());
        context
            .environment
            .relevant_env_vars
            .insert("VIRTUAL_ENV".to_string(), "/home/user/venv".to_string());

        let detector = PythonScriptDetector;
        let result = detector.detect_script_context(&context);

        assert!(result.is_some());
        let result = result.unwrap();
        assert_eq!(result.script_type, "Python Script");
        assert!(result.confidence > 0.4);
    }

    #[test]
    fn test_docker_container_detection() {
        let context = ExecutionContext::detect_basic();

        // This test can only pass in actual container environments
        let detector = DockerContainerDetector;
        let result = detector.detect_script_context(&context);

        // In most test environments, this should return None
        // But the test validates the structure is correct
        if let Some(result) = result {
            assert_eq!(result.script_type, "Docker Container");
            assert!(matches!(
                result.execution_environment,
                ExecutionEnvironment::Container
            ));
        }
    }

    #[test]
    fn test_pytest_detection() {
        let mut context = ExecutionContext::detect_basic();
        context.environment.relevant_env_vars.clear();
        context.environment.relevant_env_vars.insert(
            "PYTEST_CURRENT_TEST".to_string(),
            "test_example.py::test_function".to_string(),
        );

        let detector = PythonScriptDetector;
        let result = detector.detect_script_context(&context);

        assert!(result.is_some());
        let result = result.unwrap();
        assert_eq!(result.script_type, "Python Test Framework");
        assert!(matches!(
            result.execution_environment,
            ExecutionEnvironment::TestFramework
        ));
    }
}
