//! Machine Interaction Detection
//!
//! This module implements detection for machine-to-machine interactions including
//! API clients, automation tools, and programmatic usage patterns.

use crate::cli::context::ExecutionContext;
use serde::{Deserialize, Serialize};

pub trait MachineInteractionDetector: Send + Sync + std::fmt::Debug {
    fn detect_machine_interaction(
        &self,
        context: &ExecutionContext,
    ) -> Option<MachineInteractionResult>;
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
    fn detect_machine_interaction(
        &self,
        context: &ExecutionContext,
    ) -> Option<MachineInteractionResult> {
        let mut api_indicators = Vec::new();
        let mut confidence = 0.0;
        let env_map = &context.environment.relevant_env_vars;

        // Check for HTTP User-Agent patterns
        if let Some(user_agent) = env_map.get("HTTP_USER_AGENT") {
            let user_agent_lower = user_agent.to_lowercase();
            let automation_patterns = [
                "curl",
                "wget",
                "python-requests",
                "node-fetch",
                "axios",
                "golang",
                "java",
                "ruby",
                "automation",
                "bot",
                "scraper",
                "httpie",
                "postman",
                "insomnia",
            ];

            for pattern in &automation_patterns {
                if user_agent_lower.contains(pattern) {
                    api_indicators.push(format!("Automation user agent detected: {pattern}"));
                    confidence += 0.4;
                    break;
                }
            }
        }

        // Check for API-specific environment variables
        let api_vars = [
            "API_KEY",
            "ACCESS_TOKEN",
            "AUTH_TOKEN",
            "BEARER_TOKEN",
            "CLIENT_ID",
            "CLIENT_SECRET",
            "WEBHOOK_URL",
            "API_SECRET",
            "OAUTH_TOKEN",
            "JWT_TOKEN",
        ];

        for var in &api_vars {
            if env_map.contains_key(*var) {
                api_indicators.push(format!("API authentication variable {var} detected"));
                confidence += 0.2;
            }
        }

        // Check for programmatic execution patterns
        if !context.is_tty && context.is_piped {
            api_indicators.push("Non-interactive piped execution".to_string());
            confidence += 0.3;
        }

        // Check for JSON output preference
        if env_map
            .get("ACCEPT")
            .is_some_and(|a| a.contains("application/json"))
        {
            api_indicators.push("JSON output preference indicated".to_string());
            confidence += 0.3;
        }

        // Check for REST client indicators
        if env_map
            .get("CONTENT_TYPE")
            .is_some_and(|ct| ct.contains("application/json"))
        {
            api_indicators.push("JSON content type detected".to_string());
            confidence += 0.3;
        }

        // Check for automated tool execution context
        if let Some(parent_cmd) = env_map.get("_") {
            let api_tools = [
                "curl", "wget", "httpie", "postman", "insomnia", "python", "node", "ruby", "go",
                "java",
            ];

            for tool in &api_tools {
                if parent_cmd.contains(tool) {
                    api_indicators.push(format!("API tool execution detected: {tool}"));
                    confidence += 0.4;
                    break;
                }
            }
        }

        if confidence > 0.4 {
            Some(MachineInteractionResult {
                interaction_type: "API Client".to_string(),
                confidence,
                user_agent: env_map.get("HTTP_USER_AGENT").cloned(),
                automation_tool: self.detect_automation_tool(env_map),
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
    fn detect_automation_tool(
        &self,
        env: &std::collections::HashMap<String, String>,
    ) -> Option<String> {
        // Check parent process for known automation tools
        if let Some(parent) = env.get("_") {
            let automation_tools = [
                "ansible",
                "terraform",
                "puppet",
                "chef",
                "saltstack",
                "kubernetes",
                "helm",
                "docker-compose",
                "jenkins",
                "curl",
                "wget",
                "httpie",
            ];

            for tool in &automation_tools {
                if parent.contains(tool) {
                    return Some(tool.to_string());
                }
            }
        }

        // Check for tool-specific environment variables
        if env.contains_key("ANSIBLE_INVENTORY") {
            return Some("ansible".to_string());
        }
        if env.contains_key("TERRAFORM_VERSION") {
            return Some("terraform".to_string());
        }

        None
    }
}

#[derive(Debug)]
pub struct CurlDetector;

impl MachineInteractionDetector for CurlDetector {
    fn detect_machine_interaction(
        &self,
        context: &ExecutionContext,
    ) -> Option<MachineInteractionResult> {
        let mut api_indicators = Vec::new();
        let mut confidence = 0.0;
        let env_map = &context.environment.relevant_env_vars;

        // Check if curl is the parent process
        if let Some(parent) = env_map.get("_") {
            if parent.contains("curl") {
                api_indicators.push("curl execution detected".to_string());
                confidence += 0.8;
            }
        }

        // Check for curl-specific patterns
        if env_map
            .get("HTTP_USER_AGENT")
            .is_some_and(|ua| ua.contains("curl"))
        {
            api_indicators.push("curl user agent detected".to_string());
            confidence += 0.7;
        }

        // Check for curl config file
        if env_map.contains_key("CURL_CA_BUNDLE") {
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
                user_agent: env_map.get("HTTP_USER_AGENT").cloned(),
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
    fn detect_machine_interaction(
        &self,
        context: &ExecutionContext,
    ) -> Option<MachineInteractionResult> {
        let mut api_indicators = Vec::new();
        let mut confidence = 0.0;
        let env_map = &context.environment.relevant_env_vars;

        // Check if wget is the parent process
        if let Some(parent) = env_map.get("_") {
            if parent.contains("wget") {
                api_indicators.push("wget execution detected".to_string());
                confidence += 0.8;
            }
        }

        // Check for wget-specific patterns
        if env_map
            .get("HTTP_USER_AGENT")
            .is_some_and(|ua| ua.contains("Wget"))
        {
            api_indicators.push("wget user agent detected".to_string());
            confidence += 0.7;
        }

        // Check for wget config
        if env_map.contains_key("WGETRC") {
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
                user_agent: env_map.get("HTTP_USER_AGENT").cloned(),
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
    fn detect_machine_interaction(
        &self,
        context: &ExecutionContext,
    ) -> Option<MachineInteractionResult> {
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

        let env_map = &context.environment.relevant_env_vars;
        let mut detected_tools = Vec::new();
        let mut confidence = 0.0;
        let mut automation_tool = None;

        for (env_prefix, tool_name) in &automation_tools {
            let has_prefix = env_map.keys().any(|key| key.starts_with(env_prefix));

            if has_prefix {
                detected_tools.push(format!("{tool_name} environment detected"));
                confidence += 0.7;
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

        let specific_tools = [
            ("ANSIBLE_HOST_KEY_CHECKING", "Ansible"),
            ("TERRAFORM_VERSION", "Terraform"),
            ("PUPPETSERVER_HOSTNAME", "Puppet"),
            ("CHEF_SERVER_URL", "Chef"),
            ("KUBE_CONFIG", "Kubernetes"),
            ("HELM_HOME", "Helm"),
        ];

        for (env_var, tool_name) in &specific_tools {
            if env_map.contains_key(*env_var) {
                detected_tools.push(format!("{tool_name} detected via {env_var}"));
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

        let generic_indicators = [
            "AUTOMATION",
            "BOT",
            "ROBOT",
            "SCHEDULER",
            "CRON",
            "PIPELINE",
            "WORKFLOW",
            "DEPLOY",
            "BUILD",
        ];

        for indicator in &generic_indicators {
            if env_map.keys().any(|key| key.contains(indicator)) {
                detected_tools.push(format!(
                    "Generic automation indicator detected: {indicator}"
                ));
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
        let mut context = ExecutionContext {
            is_tty: false,
            is_piped: true,
            is_scripted: false,
            environment: ExecutionEnvironment {
                is_ci: false,
                is_automation: true,
                ci_environment: None,
                shell_type: None,
                relevant_env_vars: std::collections::HashMap::new(),
            },
            ..ExecutionContext::detect_basic()
        };

        context.environment.relevant_env_vars.insert(
            "HTTP_USER_AGENT".to_string(),
            "python-requests/2.25.1".to_string(),
        );
        context
            .environment
            .relevant_env_vars
            .insert("API_KEY".to_string(), "test-key".to_string());

        let detector = ApiClientDetector;
        let result = detector.detect_machine_interaction(&context);

        assert!(result.is_some());
        let result = result.unwrap();
        assert_eq!(result.interaction_type, "API Client");
        assert!(result.confidence > 0.4);
    }

    #[test]
    fn test_curl_detection() {
        let mut context = ExecutionContext::detect_basic();
        context.environment.is_automation = true;
        context
            .environment
            .relevant_env_vars
            .insert("_".to_string(), "/usr/bin/curl".to_string());
        context
            .environment
            .relevant_env_vars
            .insert("HTTP_USER_AGENT".to_string(), "curl/7.68.0".to_string());

        let detector = CurlDetector;
        let result = detector.detect_machine_interaction(&context);

        assert!(result.is_some());
        let result = result.unwrap();
        assert_eq!(result.interaction_type, "curl");
        assert!(result.confidence > 0.6);
    }

    #[test]
    fn test_automation_tool_detection() {
        let mut context = ExecutionContext::detect_basic();
        context.environment.is_automation = true;
        context.environment.relevant_env_vars.insert(
            "ANSIBLE_INVENTORY".to_string(),
            "/etc/ansible/hosts".to_string(),
        );
        context
            .environment
            .relevant_env_vars
            .insert("ANSIBLE_HOST_KEY_CHECKING".to_string(), "False".to_string());

        let detector = AutomationToolDetector;
        let result = detector.detect_machine_interaction(&context);

        assert!(result.is_some());
        let result = result.unwrap();
        assert_eq!(result.interaction_type, "Automation Tool");
        assert_eq!(result.automation_tool, Some("Ansible".to_string()));
        assert!(result.confidence > 0.6);
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
