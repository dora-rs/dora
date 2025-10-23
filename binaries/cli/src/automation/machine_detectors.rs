//! Machine Interaction Detection
//! 
//! This module implements detection for machine-to-machine interactions including
//! API clients, automation tools, and programmatic usage patterns.

use crate::cli::context::ExecutionContext;
use serde::{Serialize, Deserialize};
use std::env;

pub trait MachineInteractionDetector: Send + Sync + std::fmt::Debug {
    fn detect_machine_interaction(&self, context: &ExecutionContext) -> Option<MachineInteractionResult>;
    fn interaction_type(&self) -> &str;
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MachineInteractionResult {
    pub interaction_type: String,
    pub confidence: f32,
    pub user_agent: Option<String>,
    pub automation_tool: Option<String>,
    pub api_indicators: Vec<String>,
}

#[derive(Debug)]
pub struct ApiClientDetector;

impl MachineInteractionDetector for ApiClientDetector {
    fn detect_machine_interaction(&self, context: &ExecutionContext) -> Option<MachineInteractionResult> {
        let mut api_indicators = Vec::new();
        let mut confidence = 0.0;
        
        // Check for HTTP User-Agent patterns
        if let Ok(user_agent) = env::var("HTTP_USER_AGENT") {
            let automation_patterns = [
                "curl", "wget", "python-requests", "node-fetch", "axios",
                "golang", "java", "ruby", "automation", "bot", "scraper",
                "httpie", "postman", "insomnia"
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
            "CLIENT_ID", "CLIENT_SECRET", "WEBHOOK_URL", "API_SECRET",
            "OAUTH_TOKEN", "JWT_TOKEN"
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
        
        // Check for REST client indicators
        if env::var("CONTENT_TYPE").map_or(false, |ct| ct.contains("application/json")) {
            api_indicators.push("JSON content type detected".to_string());
            confidence += 0.3;
        }
        
        // Check for automated tool execution context
        if let Ok(parent_cmd) = env::var("_") {
            let api_tools = [
                "curl", "wget", "httpie", "postman", "insomnia",
                "python", "node", "ruby", "go", "java"
            ];
            
            for tool in &api_tools {
                if parent_cmd.contains(tool) {
                    api_indicators.push(format!("API tool execution detected: {}", tool));
                    confidence += 0.4;
                    break;
                }
            }
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
        if let Ok(parent) = env::var("_") {
            let automation_tools = [
                "ansible", "terraform", "puppet", "chef", "saltstack",
                "kubernetes", "helm", "docker-compose", "jenkins",
                "curl", "wget", "httpie"
            ];
            
            for tool in &automation_tools {
                if parent.contains(tool) {
                    return Some(tool.to_string());
                }
            }
        }
        
        // Check for tool-specific environment variables
        if env::var("ANSIBLE_INVENTORY").is_ok() {
            return Some("ansible".to_string());
        }
        if env::var("TERRAFORM_VERSION").is_ok() {
            return Some("terraform".to_string());
        }
        
        None
    }
}

#[derive(Debug)]
pub struct CurlDetector;

impl MachineInteractionDetector for CurlDetector {
    fn detect_machine_interaction(&self, context: &ExecutionContext) -> Option<MachineInteractionResult> {
        let mut api_indicators = Vec::new();
        let mut confidence = 0.0;
        
        // Check if curl is the parent process
        if let Ok(parent) = env::var("_") {
            if parent.contains("curl") {
                api_indicators.push("curl execution detected".to_string());
                confidence += 0.8;
            }
        }
        
        // Check for curl-specific patterns
        if env::var("HTTP_USER_AGENT").map_or(false, |ua| ua.contains("curl")) {
            api_indicators.push("curl user agent detected".to_string());
            confidence += 0.7;
        }
        
        // Check for curl config file
        if env::var("CURL_CA_BUNDLE").is_ok() {
            api_indicators.push("curl configuration detected".to_string());
            confidence += 0.3;
        }
        
        // Check execution context
        if !context.is_tty && context.is_piped {
            api_indicators.push("curl-like execution pattern".to_string());
            confidence += 0.2;
        }
        
        if confidence > 0.6 {
            Some(MachineInteractionResult {
                interaction_type: "curl".to_string(),
                confidence,
                user_agent: env::var("HTTP_USER_AGENT").ok(),
                automation_tool: Some("curl".to_string()),
                api_indicators,
            })
        } else {
            None
        }
    }
    
    fn interaction_type(&self) -> &str {
        "curl"
    }
}

#[derive(Debug)]
pub struct WgetDetector;

impl MachineInteractionDetector for WgetDetector {
    fn detect_machine_interaction(&self, context: &ExecutionContext) -> Option<MachineInteractionResult> {
        let mut api_indicators = Vec::new();
        let mut confidence = 0.0;
        
        // Check if wget is the parent process
        if let Ok(parent) = env::var("_") {
            if parent.contains("wget") {
                api_indicators.push("wget execution detected".to_string());
                confidence += 0.8;
            }
        }
        
        // Check for wget-specific patterns
        if env::var("HTTP_USER_AGENT").map_or(false, |ua| ua.contains("Wget")) {
            api_indicators.push("wget user agent detected".to_string());
            confidence += 0.7;
        }
        
        // Check for wget config
        if env::var("WGETRC").is_ok() {
            api_indicators.push("wget configuration detected".to_string());
            confidence += 0.3;
        }
        
        // Check execution context
        if !context.is_tty && context.is_piped {
            api_indicators.push("wget-like execution pattern".to_string());
            confidence += 0.2;
        }
        
        if confidence > 0.6 {
            Some(MachineInteractionResult {
                interaction_type: "wget".to_string(),
                confidence,
                user_agent: env::var("HTTP_USER_AGENT").ok(),
                automation_tool: Some("wget".to_string()),
                api_indicators,
            })
        } else {
            None
        }
    }
    
    fn interaction_type(&self) -> &str {
        "wget"
    }
}

#[derive(Debug)]
pub struct AutomationToolDetector;

impl MachineInteractionDetector for AutomationToolDetector {
    fn detect_machine_interaction(&self, _context: &ExecutionContext) -> Option<MachineInteractionResult> {
        let automation_tools = [
            ("ANSIBLE_INVENTORY", "Ansible"),
            ("TF_VAR_", "Terraform"),
            ("PUPPET_", "Puppet"),
            ("CHEF_", "Chef"),
            ("SALTSTACK_", "SaltStack"),
            ("KUBERNETES_", "Kubernetes"),
            ("HELM_", "Helm"),
            ("DOCKER_", "Docker"),
            ("COMPOSE_", "Docker Compose"),
        ];
        
        let mut detected_tools = Vec::new();
        let mut confidence = 0.0;
        let mut automation_tool = None;
        
        for (env_prefix, tool_name) in &automation_tools {
            let matching_vars: Vec<_> = env::vars()
                .filter(|(key, _)| key.starts_with(env_prefix))
                .collect();
            
            if !matching_vars.is_empty() {
                detected_tools.push(format!("{} environment detected", tool_name));
                confidence += 0.7;
                automation_tool = Some(tool_name.to_string());
                
                // Return early with the first detected tool
                return Some(MachineInteractionResult {
                    interaction_type: "Automation Tool".to_string(),
                    confidence,
                    user_agent: None,
                    automation_tool,
                    api_indicators: detected_tools,
                });
            }
        }
        
        // Check for specific automation tool indicators
        let specific_tools = [
            ("ANSIBLE_HOST_KEY_CHECKING", "Ansible"),
            ("TERRAFORM_VERSION", "Terraform"),
            ("PUPPETSERVER_HOSTNAME", "Puppet"),
            ("CHEF_SERVER_URL", "Chef"),
            ("KUBE_CONFIG", "Kubernetes"),
            ("HELM_HOME", "Helm"),
        ];
        
        for (env_var, tool_name) in &specific_tools {
            if env::var(env_var).is_ok() {
                detected_tools.push(format!("{} detected via {}", tool_name, env_var));
                confidence += 0.8;
                automation_tool = Some(tool_name.to_string());
                
                return Some(MachineInteractionResult {
                    interaction_type: "Automation Tool".to_string(),
                    confidence,
                    user_agent: None,
                    automation_tool,
                    api_indicators: detected_tools,
                });
            }
        }
        
        // Check for generic automation indicators
        let generic_indicators = [
            "AUTOMATION", "BOT", "ROBOT", "SCHEDULER", "CRON",
            "PIPELINE", "WORKFLOW", "DEPLOY", "BUILD"
        ];
        
        for indicator in &generic_indicators {
            if env::vars().any(|(key, _)| key.contains(indicator)) {
                detected_tools.push(format!("Generic automation indicator detected: {}", indicator));
                confidence += 0.4;
            }
        }
        
        if confidence > 0.3 {
            Some(MachineInteractionResult {
                interaction_type: "Automation Tool".to_string(),
                confidence,
                user_agent: None,
                automation_tool,
                api_indicators: detected_tools,
            })
        } else {
            None
        }
    }
    
    fn interaction_type(&self) -> &str {
        "Automation Tool"
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cli::context::{ExecutionContext, ExecutionEnvironment};
    
    #[test]
    fn test_api_client_detection() {
        let context = ExecutionContext {
            is_tty: false,
            is_piped: true,
            is_scripted: false,
            environment: ExecutionEnvironment {
                is_ci: false,
                is_automation: false,
                ci_environment: None,
                shell_type: None,
                relevant_env_vars: std::collections::HashMap::new(),
            },
            ..ExecutionContext::detect_basic()
        };
        
        // Set environment to simulate API client
        unsafe {
            env::set_var("HTTP_USER_AGENT", "python-requests/2.25.1");
            env::set_var("API_KEY", "test-key");
        }
        
        let detector = ApiClientDetector;
        let result = detector.detect_machine_interaction(&context);
        
        assert!(result.is_some());
        let result = result.unwrap();
        assert_eq!(result.interaction_type, "API Client");
        assert!(result.confidence > 0.4);
        
        // Clean up
        unsafe {
            env::remove_var("HTTP_USER_AGENT");
            env::remove_var("API_KEY");
        }
    }
    
    #[test]
    fn test_curl_detection() {
        let context = ExecutionContext::detect_basic();
        
        // Set environment to simulate curl execution
        unsafe {
            env::set_var("_", "/usr/bin/curl");
            env::set_var("HTTP_USER_AGENT", "curl/7.68.0");
        }
        
        let detector = CurlDetector;
        let result = detector.detect_machine_interaction(&context);
        
        assert!(result.is_some());
        let result = result.unwrap();
        assert_eq!(result.interaction_type, "curl");
        assert!(result.confidence > 0.6);
        
        // Clean up
        unsafe {
            env::remove_var("_");
            env::remove_var("HTTP_USER_AGENT");
        }
    }
    
    #[test]
    fn test_automation_tool_detection() {
        let context = ExecutionContext::detect_basic();
        
        // Set environment to simulate Ansible execution
        unsafe {
            env::set_var("ANSIBLE_INVENTORY", "/etc/ansible/hosts");
            env::set_var("ANSIBLE_HOST_KEY_CHECKING", "False");
        }
        
        let detector = AutomationToolDetector;
        let result = detector.detect_machine_interaction(&context);
        
        assert!(result.is_some());
        let result = result.unwrap();
        assert_eq!(result.interaction_type, "Automation Tool");
        assert_eq!(result.automation_tool, Some("Ansible".to_string()));
        assert!(result.confidence > 0.6);
        
        // Clean up
        unsafe {
            env::remove_var("ANSIBLE_INVENTORY");
            env::remove_var("ANSIBLE_HOST_KEY_CHECKING");
        }
    }
    
    #[test]
    fn test_no_machine_interaction() {
        let context = ExecutionContext {
            is_tty: true,
            is_piped: false,
            is_scripted: false,
            environment: ExecutionEnvironment {
                is_ci: false,
                is_automation: false,
                ci_environment: None,
                shell_type: Some("bash".to_string()),
                relevant_env_vars: std::collections::HashMap::new(),
            },
            ..ExecutionContext::detect_basic()
        };
        
        let detector = ApiClientDetector;
        let result = detector.detect_machine_interaction(&context);
        
        // Should return None for interactive context without API indicators
        assert!(result.is_none());
    }
}