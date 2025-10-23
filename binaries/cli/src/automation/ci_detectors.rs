//! CI/CD Environment Detection
//! 
//! This module implements detection for various CI/CD platforms including GitHub Actions,
//! GitLab CI, Jenkins, CircleCI, Travis CI, Buildkite, Azure DevOps, and TeamCity.

use serde::{Serialize, Deserialize};
use std::{collections::HashMap, env};

pub trait CiEnvironmentDetector: Send + Sync + std::fmt::Debug {
    fn detect_ci_environment(&self) -> Option<CiDetectionResult>;
    fn platform_name(&self) -> &str;
    fn detection_priority(&self) -> u8; // Higher number = higher priority
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CiDetectionResult {
    pub platform: String,
    pub confidence: f32,
    pub environment_variables: HashMap<String, String>,
    pub build_information: Option<BuildInformation>,
    pub evidence: Vec<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BuildInformation {
    pub build_id: Option<String>,
    pub repository: Option<String>,
    pub branch: Option<String>,
    pub commit: Option<String>,
    pub workflow: Option<String>,
    pub actor: Option<String>,
}

#[derive(Debug)]
pub struct GitHubActionsDetector;

impl CiEnvironmentDetector for GitHubActionsDetector {
    fn detect_ci_environment(&self) -> Option<CiDetectionResult> {
        let github_actions_vars = [
            "GITHUB_ACTIONS",
            "GITHUB_WORKFLOW",
            "GITHUB_RUN_ID",
            "GITHUB_ACTOR",
            "GITHUB_REPOSITORY",
            "GITHUB_REF_NAME",
            "GITHUB_SHA",
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

#[derive(Debug)]
pub struct GitLabCiDetector;

impl CiEnvironmentDetector for GitLabCiDetector {
    fn detect_ci_environment(&self) -> Option<CiDetectionResult> {
        let gitlab_vars = [
            "GITLAB_CI",
            "CI_JOB_ID",
            "CI_PIPELINE_ID",
            "CI_PROJECT_NAME",
            "CI_COMMIT_SHA",
            "CI_COMMIT_REF_NAME",
            "CI_PIPELINE_SOURCE",
            "GITLAB_USER_LOGIN",
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

#[derive(Debug)]
pub struct JenkinsDetector;

impl CiEnvironmentDetector for JenkinsDetector {
    fn detect_ci_environment(&self) -> Option<CiDetectionResult> {
        let jenkins_vars = [
            "JENKINS_URL",
            "HUDSON_URL",
            "BUILD_NUMBER",
            "JOB_NAME",
            "WORKSPACE",
            "GIT_BRANCH",
            "GIT_COMMIT",
            "BUILD_USER",
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

#[derive(Debug)]
pub struct CircleCiDetector;

impl CiEnvironmentDetector for CircleCiDetector {
    fn detect_ci_environment(&self) -> Option<CiDetectionResult> {
        let circleci_vars = [
            "CIRCLECI",
            "CIRCLE_BUILD_NUM",
            "CIRCLE_PROJECT_REPONAME",
            "CIRCLE_BRANCH",
            "CIRCLE_SHA1",
            "CIRCLE_WORKFLOW_ID",
            "CIRCLE_USERNAME",
        ];
        
        let mut found_vars = HashMap::new();
        let mut evidence = Vec::new();
        
        for var in &circleci_vars {
            if let Ok(value) = env::var(var) {
                found_vars.insert(var.to_string(), value);
                evidence.push(format!("CircleCI variable {} found", var));
            }
        }
        
        if found_vars.contains_key("CIRCLECI") {
            let confidence = if found_vars.len() >= 3 { 0.95 } else { 0.8 };
            
            let build_info = BuildInformation {
                build_id: found_vars.get("CIRCLE_BUILD_NUM").cloned(),
                repository: found_vars.get("CIRCLE_PROJECT_REPONAME").cloned(),
                branch: found_vars.get("CIRCLE_BRANCH").cloned(),
                commit: found_vars.get("CIRCLE_SHA1").cloned(),
                workflow: found_vars.get("CIRCLE_WORKFLOW_ID").cloned(),
                actor: found_vars.get("CIRCLE_USERNAME").cloned(),
            };
            
            Some(CiDetectionResult {
                platform: "CircleCI".to_string(),
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
        "CircleCI"
    }
    
    fn detection_priority(&self) -> u8 {
        85
    }
}

#[derive(Debug)]
pub struct TravisCiDetector;

impl CiEnvironmentDetector for TravisCiDetector {
    fn detect_ci_environment(&self) -> Option<CiDetectionResult> {
        let travis_vars = [
            "TRAVIS",
            "TRAVIS_BUILD_NUMBER",
            "TRAVIS_REPO_SLUG",
            "TRAVIS_BRANCH",
            "TRAVIS_COMMIT",
            "TRAVIS_JOB_ID",
        ];
        
        let mut found_vars = HashMap::new();
        let mut evidence = Vec::new();
        
        for var in &travis_vars {
            if let Ok(value) = env::var(var) {
                found_vars.insert(var.to_string(), value);
                evidence.push(format!("Travis CI variable {} found", var));
            }
        }
        
        if found_vars.contains_key("TRAVIS") {
            let confidence = if found_vars.len() >= 3 { 0.9 } else { 0.7 };
            
            let build_info = BuildInformation {
                build_id: found_vars.get("TRAVIS_BUILD_NUMBER").cloned(),
                repository: found_vars.get("TRAVIS_REPO_SLUG").cloned(),
                branch: found_vars.get("TRAVIS_BRANCH").cloned(),
                commit: found_vars.get("TRAVIS_COMMIT").cloned(),
                workflow: found_vars.get("TRAVIS_JOB_ID").cloned(),
                actor: None,
            };
            
            Some(CiDetectionResult {
                platform: "Travis CI".to_string(),
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
        "Travis CI"
    }
    
    fn detection_priority(&self) -> u8 {
        80
    }
}

#[derive(Debug)]
pub struct BuildkiteDetector;

impl CiEnvironmentDetector for BuildkiteDetector {
    fn detect_ci_environment(&self) -> Option<CiDetectionResult> {
        let buildkite_vars = [
            "BUILDKITE",
            "BUILDKITE_BUILD_NUMBER",
            "BUILDKITE_PIPELINE_SLUG",
            "BUILDKITE_BRANCH",
            "BUILDKITE_COMMIT",
            "BUILDKITE_BUILD_ID",
        ];
        
        let mut found_vars = HashMap::new();
        let mut evidence = Vec::new();
        
        for var in &buildkite_vars {
            if let Ok(value) = env::var(var) {
                found_vars.insert(var.to_string(), value);
                evidence.push(format!("Buildkite variable {} found", var));
            }
        }
        
        if found_vars.contains_key("BUILDKITE") {
            let confidence = if found_vars.len() >= 3 { 0.9 } else { 0.75 };
            
            let build_info = BuildInformation {
                build_id: found_vars.get("BUILDKITE_BUILD_NUMBER").cloned(),
                repository: found_vars.get("BUILDKITE_PIPELINE_SLUG").cloned(),
                branch: found_vars.get("BUILDKITE_BRANCH").cloned(),
                commit: found_vars.get("BUILDKITE_COMMIT").cloned(),
                workflow: found_vars.get("BUILDKITE_BUILD_ID").cloned(),
                actor: None,
            };
            
            Some(CiDetectionResult {
                platform: "Buildkite".to_string(),
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
        "Buildkite"
    }
    
    fn detection_priority(&self) -> u8 {
        80
    }
}

#[derive(Debug)]
pub struct AzureDevOpsDetector;

impl CiEnvironmentDetector for AzureDevOpsDetector {
    fn detect_ci_environment(&self) -> Option<CiDetectionResult> {
        let azure_vars = [
            "AZURE_PIPELINES",
            "BUILD_BUILDNUMBER",
            "BUILD_REPOSITORY_NAME",
            "BUILD_SOURCEBRANCH",
            "BUILD_SOURCEVERSION",
            "SYSTEM_TEAMPROJECT",
        ];
        
        let mut found_vars = HashMap::new();
        let mut evidence = Vec::new();
        
        for var in &azure_vars {
            if let Ok(value) = env::var(var) {
                found_vars.insert(var.to_string(), value);
                evidence.push(format!("Azure DevOps variable {} found", var));
            }
        }
        
        // Azure DevOps can be detected by TF_BUILD as well
        if found_vars.contains_key("AZURE_PIPELINES") || env::var("TF_BUILD").is_ok() {
            let confidence = if found_vars.len() >= 3 { 0.9 } else { 0.75 };
            
            let build_info = BuildInformation {
                build_id: found_vars.get("BUILD_BUILDNUMBER").cloned(),
                repository: found_vars.get("BUILD_REPOSITORY_NAME").cloned(),
                branch: found_vars.get("BUILD_SOURCEBRANCH").cloned(),
                commit: found_vars.get("BUILD_SOURCEVERSION").cloned(),
                workflow: found_vars.get("SYSTEM_TEAMPROJECT").cloned(),
                actor: None,
            };
            
            Some(CiDetectionResult {
                platform: "Azure DevOps".to_string(),
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
        "Azure DevOps"
    }
    
    fn detection_priority(&self) -> u8 {
        85
    }
}

#[derive(Debug)]
pub struct TeamCityDetector;

impl CiEnvironmentDetector for TeamCityDetector {
    fn detect_ci_environment(&self) -> Option<CiDetectionResult> {
        let teamcity_vars = [
            "TEAMCITY_VERSION",
            "BUILD_NUMBER",
            "TEAMCITY_PROJECT_NAME",
            "TEAMCITY_BUILDCONF_NAME",
        ];
        
        let mut found_vars = HashMap::new();
        let mut evidence = Vec::new();
        
        for var in &teamcity_vars {
            if let Ok(value) = env::var(var) {
                found_vars.insert(var.to_string(), value);
                evidence.push(format!("TeamCity variable {} found", var));
            }
        }
        
        if found_vars.contains_key("TEAMCITY_VERSION") {
            let confidence = if found_vars.len() >= 2 { 0.9 } else { 0.7 };
            
            let build_info = BuildInformation {
                build_id: found_vars.get("BUILD_NUMBER").cloned(),
                repository: found_vars.get("TEAMCITY_PROJECT_NAME").cloned(),
                branch: None,
                commit: None,
                workflow: found_vars.get("TEAMCITY_BUILDCONF_NAME").cloned(),
                actor: None,
            };
            
            Some(CiDetectionResult {
                platform: "TeamCity".to_string(),
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
        "TeamCity"
    }
    
    fn detection_priority(&self) -> u8 {
        80
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_github_actions_detection() {
        // Set up GitHub Actions environment
        unsafe {
            env::set_var("GITHUB_ACTIONS", "true");
            env::set_var("GITHUB_WORKFLOW", "CI");
            env::set_var("GITHUB_RUN_ID", "123456");
        }
        
        let detector = GitHubActionsDetector;
        let result = detector.detect_ci_environment().unwrap();
        
        assert_eq!(result.platform, "GitHub Actions");
        assert!(result.confidence > 0.8);
        assert!(result.environment_variables.contains_key("GITHUB_ACTIONS"));
        
        // Clean up
        unsafe {
            env::remove_var("GITHUB_ACTIONS");
            env::remove_var("GITHUB_WORKFLOW");
            env::remove_var("GITHUB_RUN_ID");
        }
    }
    
    #[test]
    fn test_gitlab_ci_detection() {
        // Set up GitLab CI environment
        unsafe {
            env::set_var("GITLAB_CI", "true");
            env::set_var("CI_JOB_ID", "789");
        }
        
        let detector = GitLabCiDetector;
        let result = detector.detect_ci_environment().unwrap();
        
        assert_eq!(result.platform, "GitLab CI");
        assert!(result.confidence > 0.7);
        
        // Clean up
        unsafe {
            env::remove_var("GITLAB_CI");
            env::remove_var("CI_JOB_ID");
        }
    }
    
    #[test]
    fn test_no_ci_detection() {
        let detector = GitHubActionsDetector;
        let result = detector.detect_ci_environment();
        
        // Should return None if no CI environment variables are set
        assert!(result.is_none());
    }
}