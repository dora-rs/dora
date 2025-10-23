use std::{
    collections::HashMap,
    time::{Duration, Instant},
};

use crate::{
    cli::transitions::{SharedContext, CliState, TuiState},
    tui::{
        AppState,
        app::{DataflowInfo, SystemMetrics, UserConfig, StatusMessage},
    },
};

/// Manages bidirectional state synchronization between CLI and TUI modes
#[derive(Debug)]
pub struct StateSynchronizer {
    last_sync: Instant,
    sync_interval: Duration,
    pending_updates: Vec<StateUpdate>,
    conflict_resolution: ConflictResolution,
}

/// Types of state updates that can occur
#[derive(Debug, Clone)]
pub enum StateUpdate {
    DataflowAdded(DataflowInfo),
    DataflowRemoved(String),
    DataflowStatusChanged { name: String, new_status: DataflowStatus },
    NodeStatusChanged { dataflow: String, node: String, status: NodeStatus },
    SystemMetricsUpdated(SystemMetrics),
    ConfigurationChanged(UserConfig),
    FilterUpdated { view: String, filter: FilterState },
    UserPreferenceChanged { key: String, value: String },
}

/// Dataflow status enumeration
#[derive(Debug, Clone, PartialEq)]
pub enum DataflowStatus {
    Stopped,
    Starting,
    Running,
    Stopping,
    Error(String),
}

/// Node status enumeration  
#[derive(Debug, Clone, PartialEq)]
pub enum NodeStatus {
    Idle,
    Running,
    Error(String),
    Completed,
}

/// Filter state for views
#[derive(Debug, Clone)]
pub struct FilterState {
    pub active: bool,
    pub value: String,
    pub last_applied: Instant,
}

/// Strategy for resolving state conflicts
#[derive(Debug, Clone)]
pub enum ConflictResolution {
    CliWins,        // CLI state takes precedence
    TuiWins,        // TUI state takes precedence  
    LastWriteWins,  // Most recent update wins
    UserPrompt,     // Ask user to resolve conflicts
}

/// Represents a state conflict between CLI and TUI
#[derive(Debug, Clone)]
pub struct StateConflict {
    pub conflict_type: ConflictType,
    pub cli_value: StateValue,
    pub tui_value: StateValue,
    pub timestamp: Instant,
}

/// Types of state conflicts
#[derive(Debug, Clone)]
pub enum ConflictType {
    DataflowStatus { dataflow_name: String },
    UserConfiguration { key: String },
    SystemState { component: String },
}

/// Generic state value for conflict resolution
#[derive(Debug, Clone)]
pub enum StateValue {
    String(String),
    Boolean(bool),
    Integer(i64),
    Float(f64),
    Complex(serde_json::Value),
}

impl StateSynchronizer {
    pub fn new() -> Self {
        Self {
            last_sync: Instant::now(),
            sync_interval: Duration::from_millis(500), // Sync every 500ms
            pending_updates: Vec::new(),
            conflict_resolution: ConflictResolution::LastWriteWins,
        }
    }
    
    /// Sync CLI state changes to TUI
    pub async fn sync_cli_to_tui(&mut self, app_state: &mut AppState) -> crate::tui::Result<()> {
        let current_dataflows = self.fetch_current_dataflows().await?;
        let previous_dataflows = &app_state.dataflows;
        
        // Detect changes
        let updates = self.detect_dataflow_changes(previous_dataflows, &current_dataflows);
        
        // Apply updates to TUI state
        for update in updates {
            self.apply_state_update_to_tui(update, app_state).await?;
        }
        
        // Update system metrics
        let new_metrics = self.fetch_system_metrics().await?;
        if self.metrics_changed(&app_state.system_metrics, &new_metrics) {
            app_state.system_metrics = new_metrics.clone();
            self.pending_updates.push(StateUpdate::SystemMetricsUpdated(new_metrics));
        }
        
        // Update last sync time
        self.last_sync = Instant::now();
        
        Ok(())
    }
    
    /// Sync TUI state changes back to CLI
    pub async fn sync_tui_to_cli(&mut self, tui_state: &TuiState) -> crate::tui::Result<()> {
        // Extract CLI-relevant state from TUI
        for update in &self.pending_updates {
            match update {
                StateUpdate::ConfigurationChanged(config) => {
                    self.update_cli_config(config).await?;
                },
                StateUpdate::UserPreferenceChanged { key, value } => {
                    self.update_cli_preference(key, value).await?;
                },
                StateUpdate::DataflowStatusChanged { name, new_status } => {
                    self.update_cli_dataflow_cache(name, new_status).await?;
                },
                _ => {} // Other updates don't affect CLI state
            }
        }
        
        self.pending_updates.clear();
        Ok(())
    }
    
    /// Force a full synchronization between CLI and TUI
    pub async fn force_sync(&mut self, 
        cli_state: &CliState, 
        app_state: &mut AppState
    ) -> crate::tui::Result<Vec<StateConflict>> {
        let mut conflicts = Vec::new();
        
        // Sync dataflows with conflict detection
        let cli_dataflows = self.extract_cli_dataflows(cli_state).await?;
        let tui_dataflows = &app_state.dataflows;
        
        for cli_df in &cli_dataflows {
            if let Some(tui_df) = tui_dataflows.iter().find(|df| df.name == cli_df.name) {
                if cli_df.status != tui_df.status {
                    conflicts.push(StateConflict {
                        conflict_type: ConflictType::DataflowStatus { 
                            dataflow_name: cli_df.name.clone() 
                        },
                        cli_value: StateValue::String(cli_df.status.clone()),
                        tui_value: StateValue::String(tui_df.status.clone()),
                        timestamp: Instant::now(),
                    });
                }
            }
        }
        
        // Resolve conflicts according to strategy
        self.resolve_conflicts(&conflicts, app_state).await?;
        
        Ok(conflicts)
    }
    
    /// Add a pending state update
    pub fn add_pending_update(&mut self, update: StateUpdate) {
        self.pending_updates.push(update);
    }
    
    /// Get pending updates without consuming them
    pub fn get_pending_updates(&self) -> &[StateUpdate] {
        &self.pending_updates
    }
    
    /// Clear all pending updates
    pub fn clear_pending_updates(&mut self) {
        self.pending_updates.clear();
    }
    
    /// Set conflict resolution strategy
    pub fn set_conflict_resolution(&mut self, strategy: ConflictResolution) {
        self.conflict_resolution = strategy;
    }
    
    // Private helper methods
    
    async fn fetch_current_dataflows(&self) -> crate::tui::Result<Vec<DataflowInfo>> {
        // Mock implementation - would connect to actual Dora runtime
        tokio::time::sleep(Duration::from_millis(10)).await;
        
        Ok(vec![
            DataflowInfo {
                id: "df_mock_1".to_string(),
                name: "mock_dataflow_1".to_string(),
                status: "running".to_string(),
                nodes: vec![],
            },
            DataflowInfo {
                id: "df_mock_2".to_string(),
                name: "mock_dataflow_2".to_string(),
                status: "stopped".to_string(),
                nodes: vec![],
            },
        ])
    }
    
    async fn fetch_system_metrics(&self) -> crate::tui::Result<SystemMetrics> {
        // Mock implementation - would gather actual system metrics
        tokio::time::sleep(Duration::from_millis(5)).await;
        
        Ok(SystemMetrics {
            cpu_usage: 42.5,
            memory_usage: 68.2,
            network_io: (2048, 1024),
            last_update: Some(Instant::now()),
        })
    }
    
    fn detect_dataflow_changes(
        &self,
        previous: &[DataflowInfo],
        current: &[DataflowInfo],
    ) -> Vec<StateUpdate> {
        let mut updates = Vec::new();
        
        // Find additions
        for dataflow in current {
            if !previous.iter().any(|d| d.name == dataflow.name) {
                updates.push(StateUpdate::DataflowAdded(dataflow.clone()));
            }
        }
        
        // Find removals
        for dataflow in previous {
            if !current.iter().any(|d| d.name == dataflow.name) {
                updates.push(StateUpdate::DataflowRemoved(dataflow.name.clone()));
            }
        }
        
        // Find status changes
        for current_df in current {
            if let Some(previous_df) = previous.iter().find(|d| d.name == current_df.name) {
                if current_df.status != previous_df.status {
                    updates.push(StateUpdate::DataflowStatusChanged {
                        name: current_df.name.clone(),
                        new_status: self.parse_dataflow_status(&current_df.status),
                    });
                }
            }
        }
        
        updates
    }
    
    async fn apply_state_update_to_tui(
        &mut self,
        update: StateUpdate,
        app_state: &mut AppState,
    ) -> crate::tui::Result<()> {
        match update {
            StateUpdate::DataflowAdded(dataflow) => {
                app_state.dataflows.push(dataflow);
            },
            StateUpdate::DataflowRemoved(name) => {
                app_state.dataflows.retain(|df| df.name != name);
            },
            StateUpdate::DataflowStatusChanged { name, new_status } => {
                if let Some(dataflow) = app_state.dataflows.iter_mut().find(|df| df.name == name) {
                    dataflow.status = format!("{:?}", new_status);
                }
            },
            StateUpdate::SystemMetricsUpdated(metrics) => {
                app_state.system_metrics = metrics;
            },
            StateUpdate::ConfigurationChanged(config) => {
                app_state.user_config = config;
            },
            _ => {} // Handle other update types as needed
        }
        
        Ok(())
    }
    
    fn metrics_changed(&self, old: &SystemMetrics, new: &SystemMetrics) -> bool {
        (old.cpu_usage - new.cpu_usage).abs() > 1.0 ||
        (old.memory_usage - new.memory_usage).abs() > 1.0 ||
        old.network_io != new.network_io
    }
    
    async fn update_cli_config(&self, _config: &UserConfig) -> crate::tui::Result<()> {
        // Update CLI configuration storage
        Ok(())
    }
    
    async fn update_cli_preference(&self, _key: &str, _value: &str) -> crate::tui::Result<()> {
        // Update CLI user preferences
        Ok(())
    }
    
    async fn update_cli_dataflow_cache(&self, _name: &str, _status: &DataflowStatus) -> crate::tui::Result<()> {
        // Update CLI dataflow cache
        Ok(())
    }
    
    async fn extract_cli_dataflows(&self, _cli_state: &CliState) -> crate::tui::Result<Vec<DataflowInfo>> {
        // Extract dataflow information from CLI state
        self.fetch_current_dataflows().await
    }
    
    async fn resolve_conflicts(
        &self,
        conflicts: &[StateConflict],
        app_state: &mut AppState,
    ) -> crate::tui::Result<()> {
        for conflict in conflicts {
            match &self.conflict_resolution {
                ConflictResolution::LastWriteWins => {
                    // Use the conflict timestamp to determine winner
                    // For simplicity, always use TUI value in this mock
                    self.apply_conflict_resolution(conflict, true, app_state)?;
                },
                ConflictResolution::TuiWins => {
                    self.apply_conflict_resolution(conflict, true, app_state)?;
                },
                ConflictResolution::CliWins => {
                    self.apply_conflict_resolution(conflict, false, app_state)?;
                },
                ConflictResolution::UserPrompt => {
                    // In a real implementation, this would prompt the user
                    // For now, default to TUI wins
                    self.apply_conflict_resolution(conflict, true, app_state)?;
                },
            }
        }
        
        Ok(())
    }
    
    fn apply_conflict_resolution(
        &self,
        conflict: &StateConflict,
        use_tui_value: bool,
        app_state: &mut AppState,
    ) -> crate::tui::Result<()> {
        let winning_value = if use_tui_value {
            &conflict.tui_value
        } else {
            &conflict.cli_value
        };
        
        match &conflict.conflict_type {
            ConflictType::DataflowStatus { dataflow_name } => {
                if let StateValue::String(status) = winning_value {
                    if let Some(dataflow) = app_state.dataflows.iter_mut()
                        .find(|df| df.name == *dataflow_name) {
                        dataflow.status = status.clone();
                    }
                }
            },
            ConflictType::UserConfiguration { key: _key } => {
                // Update user configuration
            },
            ConflictType::SystemState { component: _component } => {
                // Update system state component
            },
        }
        
        Ok(())
    }
    
    fn parse_dataflow_status(&self, status_str: &str) -> DataflowStatus {
        match status_str.to_lowercase().as_str() {
            "stopped" => DataflowStatus::Stopped,
            "starting" => DataflowStatus::Starting,
            "running" => DataflowStatus::Running,
            "stopping" => DataflowStatus::Stopping,
            error if error.starts_with("error") => {
                DataflowStatus::Error(error.to_string())
            },
            _ => DataflowStatus::Error(format!("Unknown status: {}", status_str)),
        }
    }
}

impl Default for StateSynchronizer {
    fn default() -> Self {
        Self::new()
    }
}

impl Default for ConflictResolution {
    fn default() -> Self {
        ConflictResolution::LastWriteWins
    }
}

impl std::fmt::Display for DataflowStatus {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            DataflowStatus::Stopped => write!(f, "stopped"),
            DataflowStatus::Starting => write!(f, "starting"),
            DataflowStatus::Running => write!(f, "running"),
            DataflowStatus::Stopping => write!(f, "stopping"),
            DataflowStatus::Error(msg) => write!(f, "error: {}", msg),
        }
    }
}

impl std::fmt::Display for NodeStatus {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            NodeStatus::Idle => write!(f, "idle"),
            NodeStatus::Running => write!(f, "running"),
            NodeStatus::Error(msg) => write!(f, "error: {}", msg),
            NodeStatus::Completed => write!(f, "completed"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_state_synchronizer_creation() {
        let sync = StateSynchronizer::new();
        assert!(sync.pending_updates.is_empty());
    }

    #[tokio::test]
    async fn test_dataflow_change_detection() {
        let sync = StateSynchronizer::new();
        
        let previous = vec![
            DataflowInfo {
                id: "df1".to_string(),
                name: "dataflow1".to_string(),
                status: "running".to_string(),
                nodes: vec![],
            }
        ];
        
        let current = vec![
            DataflowInfo {
                id: "df1".to_string(),
                name: "dataflow1".to_string(),
                status: "stopped".to_string(),
                nodes: vec![],
            },
            DataflowInfo {
                id: "df2".to_string(),
                name: "dataflow2".to_string(),
                status: "running".to_string(),
                nodes: vec![],
            }
        ];
        
        let updates = sync.detect_dataflow_changes(&previous, &current);
        
        assert_eq!(updates.len(), 2); // One status change, one addition
    }

    #[test]
    fn test_conflict_resolution_strategy() {
        let mut sync = StateSynchronizer::new();
        sync.set_conflict_resolution(ConflictResolution::CliWins);
        
        match sync.conflict_resolution {
            ConflictResolution::CliWins => {},
            _ => panic!("Expected CliWins strategy"),
        }
    }

    #[test]
    fn test_dataflow_status_parsing() {
        let sync = StateSynchronizer::new();
        
        assert!(matches!(sync.parse_dataflow_status("running"), DataflowStatus::Running));
        assert!(matches!(sync.parse_dataflow_status("stopped"), DataflowStatus::Stopped));
        assert!(matches!(sync.parse_dataflow_status("error: failed"), DataflowStatus::Error(_)));
    }

    #[tokio::test]
    async fn test_metrics_comparison() {
        let sync = StateSynchronizer::new();
        
        let old_metrics = SystemMetrics {
            cpu_usage: 10.0,
            memory_usage: 20.0,
            network_io: (100, 200),
            last_update: None,
        };
        
        let new_metrics = SystemMetrics {
            cpu_usage: 15.0,
            memory_usage: 25.0,
            network_io: (150, 250),
            last_update: None,
        };
        
        assert!(sync.metrics_changed(&old_metrics, &new_metrics));
    }
}