// Debug Session Management for Issue #18

use super::types::*;
use crate::cli::commands::DebugCommand;
use chrono::Utc;
use eyre::Result;
use uuid::Uuid;

/// Debug session manager
#[derive(Debug)]
pub struct DebugSessionManager;

impl DebugSessionManager {
    pub fn new() -> Self {
        Self
    }

    /// Initialize a debug session from command
    pub async fn initialize_session(&self, cmd: &DebugCommand) -> Result<DebugSession> {
        let session_id = Uuid::new_v4();
        let start_time = Utc::now();

        // Determine debug target
        let debug_target = if let Some(target) = &cmd.target {
            self.resolve_debug_target(target).await?
        } else if let Some(dataflow_path) = &cmd.dataflow.dataflow {
            DebugTarget::Dataflow(dataflow_path.display().to_string())
        } else if let Some(name) = &cmd.dataflow.name {
            DebugTarget::Dataflow(name.clone())
        } else {
            DebugTarget::Auto
        };

        Ok(DebugSession {
            session_id,
            start_time,
            debug_target,
            mode: cmd.mode.clone().unwrap_or_default(),
            focus: cmd.focus.clone(),
            configuration: DebugConfiguration {
                auto_detect: cmd.auto_detect,
                live_monitoring: cmd.live,
                capture_artifacts: cmd.capture,
                timeout: cmd.timeout.clone(),
                history_window: cmd.history_window.clone(),
                output_dir: cmd.output_dir.clone(),
            },
        })
    }

    async fn resolve_debug_target(&self, target: &str) -> Result<DebugTarget> {
        // Simple resolution logic
        if target == "system" {
            Ok(DebugTarget::System)
        } else if target.contains(".yaml") || target.contains(".yml") {
            Ok(DebugTarget::Dataflow(target.to_string()))
        } else if target.contains("node") {
            Ok(DebugTarget::Node(target.to_string()))
        } else {
            Ok(DebugTarget::Component(target.to_string()))
        }
    }
}

impl Default for DebugSessionManager {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_session_initialization() {
        let manager = DebugSessionManager::new();
        let cmd = DebugCommand::default();

        let session = manager.initialize_session(&cmd).await.unwrap();

        assert!(matches!(session.debug_target, DebugTarget::Auto));
        // auto_detect is false by default in DebugCommand
        assert!(!session.configuration.auto_detect);
    }

    #[tokio::test]
    async fn test_target_resolution() {
        let manager = DebugSessionManager::new();

        let system_target = manager.resolve_debug_target("system").await.unwrap();
        assert!(matches!(system_target, DebugTarget::System));

        let dataflow_target = manager.resolve_debug_target("test.yaml").await.unwrap();
        assert!(matches!(dataflow_target, DebugTarget::Dataflow(_)));
    }
}
