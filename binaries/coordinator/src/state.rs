use std::{collections::BTreeMap, sync::Arc};

use dashmap::DashMap;
use dora_core::uhlc::HLC;
use dora_message::{
    BuildId, DataflowId, common::DaemonId, daemon_to_coordinator::DataflowDaemonResult,
};
use tokio::sync::mpsc;

use crate::{
    ArchivedDataflow, BuildFinishedResult, CachedResult, DaemonConnections, Event, RunningBuild,
    RunningDataflow, persistence,
};

pub struct CoordinatorState {
    pub clock: Arc<HLC>,
    pub running_builds: DashMap<BuildId, RunningBuild>,
    pub finished_builds: DashMap<BuildId, CachedResult<BuildFinishedResult>>,
    pub running_dataflows: DashMap<DataflowId, RunningDataflow>,
    pub dataflow_results: DashMap<DataflowId, BTreeMap<DaemonId, DataflowDaemonResult>>,
    pub archived_dataflows: DashMap<DataflowId, ArchivedDataflow>,
    pub daemon_connections: DaemonConnections,
    pub daemon_events_tx: mpsc::Sender<Event>,
    pub abort_handle: futures::stream::AbortHandle,
    pub persistence: Option<persistence::CoordinatorPersistence>,
}

impl CoordinatorState {
    pub async fn persist_archived_state(&self) -> eyre::Result<()> {
        let Some(persistence) = &self.persistence else {
            return Ok(());
        };

        let archived_dataflows = self
            .archived_dataflows
            .iter()
            .map(|entry| persistence::PersistedArchivedDataflow {
                dataflow_id: *entry.key(),
                dataflow: entry.value().clone(),
            })
            .collect();
        let dataflow_results = self
            .dataflow_results
            .iter()
            .map(|entry| persistence::PersistedDataflowResult {
                dataflow_id: *entry.key(),
                daemon_results: entry
                    .value()
                    .iter()
                    .map(|(daemon_id, result)| persistence::PersistedDaemonResult {
                        daemon_id: daemon_id.clone(),
                        result: result.clone(),
                    })
                    .collect(),
            })
            .collect();

        let snapshot = persistence::PersistedCoordinatorState {
            archived_dataflows,
            dataflow_results,
            ..persistence::PersistedCoordinatorState::default()
        };
        persistence.save(&snapshot).await
    }
}
