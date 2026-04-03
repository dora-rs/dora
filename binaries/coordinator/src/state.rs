use std::{collections::BTreeMap, sync::Arc};

use dashmap::DashMap;
use dora_core::uhlc::HLC;
use dora_message::{
    BuildId, DataflowId, common::DaemonId, daemon_to_coordinator::DataflowDaemonResult,
};
use tokio::sync::mpsc;

use crate::{
    ArchivedDataflow, BuildFinishedResult, CachedResult, DaemonConnections, Event, RunningBuild,
    RunningDataflow,
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
}
