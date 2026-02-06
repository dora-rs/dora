use std::sync::Arc;

use dashmap::DashMap;
use dora_core::uhlc::HLC;
use dora_message::{BuildId, common::DaemonId, coordinator_to_daemon::CoordinatorToDaemonClient};

use crate::RunningBuild;

pub struct CoordinatorState {
    pub clock: Arc<HLC>,
    pub running_builds: DashMap<BuildId, RunningBuild>,
    pub daemon_connections: DashMap<DaemonId, CoordinatorToDaemonClient>,
}
