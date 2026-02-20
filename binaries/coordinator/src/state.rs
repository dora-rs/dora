use adora_core::config::NodeId;
use adora_message::{
    common::DaemonId,
    coordinator_to_cli::{ControlRequestReply, LogMessage},
    daemon_to_coordinator::NodeMetrics,
    descriptor::{Descriptor, ResolvedNode},
};
use crate::log_subscriber::LogSubscriber;
use eyre::eyre;
use std::collections::{BTreeMap, BTreeSet};
use std::time::Instant;
use tokio::net::TcpStream;
use tokio::sync::oneshot;
use uuid::Uuid;

#[derive(Default)]
pub(crate) struct DaemonConnections {
    daemons: BTreeMap<DaemonId, DaemonConnection>,
}

impl DaemonConnections {
    pub(crate) fn add(&mut self, daemon_id: DaemonId, connection: DaemonConnection) {
        let previous = self.daemons.insert(daemon_id.clone(), connection);
        if previous.is_some() {
            tracing::info!("closing previous connection `{daemon_id}` on new register");
        }
    }

    pub(crate) fn get(&self, id: &DaemonId) -> Option<&DaemonConnection> {
        self.daemons.get(id)
    }

    pub(crate) fn get_mut(&mut self, id: &DaemonId) -> Option<&mut DaemonConnection> {
        self.daemons.get_mut(id)
    }

    pub(crate) fn get_matching_daemon_id(&self, machine_id: &str) -> Option<&DaemonId> {
        self.daemons
            .keys()
            .find(|id| id.matches_machine_id(machine_id))
    }

    pub(crate) fn drain(&mut self) -> impl Iterator<Item = (DaemonId, DaemonConnection)> {
        std::mem::take(&mut self.daemons).into_iter()
    }

    pub(crate) fn is_empty(&self) -> bool {
        self.daemons.is_empty()
    }

    pub(crate) fn iter_mut(&mut self) -> impl Iterator<Item = (&DaemonId, &mut DaemonConnection)> {
        self.daemons.iter_mut()
    }

    pub(crate) fn remove(&mut self, daemon_id: &DaemonId) -> Option<DaemonConnection> {
        self.daemons.remove(daemon_id)
    }

    pub(crate) fn unnamed(&self) -> impl Iterator<Item = &DaemonId> {
        self.daemons.keys().filter(|id| id.machine_id().is_none())
    }
}

pub(crate) struct DaemonConnection {
    pub(crate) stream: TcpStream,
    pub(crate) last_heartbeat: Instant,
}

pub(crate) struct RunningBuild {
    pub(crate) errors: Vec<String>,
    pub(crate) build_result: CachedResult,

    /// Buffer for log messages that were sent before there were any subscribers.
    pub(crate) buffered_log_messages: Vec<LogMessage>,
    pub(crate) log_subscribers: Vec<LogSubscriber>,

    pub(crate) pending_build_results: BTreeSet<DaemonId>,
}

pub(crate) struct RunningDataflow {
    pub(crate) name: Option<String>,
    pub(crate) uuid: Uuid,
    pub(crate) descriptor: Descriptor,
    /// The IDs of the daemons that the dataflow is running on.
    pub(crate) daemons: BTreeSet<DaemonId>,
    /// IDs of daemons that are waiting until all nodes are started.
    pub(crate) pending_daemons: BTreeSet<DaemonId>,
    pub(crate) exited_before_subscribe: Vec<NodeId>,
    pub(crate) nodes: BTreeMap<NodeId, ResolvedNode>,
    /// Maps each node to the daemon it's running on
    pub(crate) node_to_daemon: BTreeMap<NodeId, DaemonId>,
    /// Latest metrics for each node (from daemons)
    pub(crate) node_metrics: BTreeMap<NodeId, NodeMetrics>,

    pub(crate) spawn_result: CachedResult,
    pub(crate) stop_reply_senders: Vec<oneshot::Sender<eyre::Result<ControlRequestReply>>>,

    /// Buffer for log messages that were sent before there were any subscribers.
    pub(crate) buffered_log_messages: Vec<LogMessage>,
    pub(crate) log_subscribers: Vec<LogSubscriber>,

    pub(crate) pending_spawn_results: BTreeSet<DaemonId>,
}

pub(crate) enum CachedResult {
    Pending {
        result_senders: Vec<oneshot::Sender<eyre::Result<ControlRequestReply>>>,
    },
    Cached {
        result: eyre::Result<ControlRequestReply>,
    },
}

impl Default for CachedResult {
    fn default() -> Self {
        Self::Pending {
            result_senders: Vec::new(),
        }
    }
}

impl CachedResult {
    pub(crate) fn register(
        &mut self,
        reply_sender: oneshot::Sender<eyre::Result<ControlRequestReply>>,
    ) {
        match self {
            CachedResult::Pending { result_senders } => result_senders.push(reply_sender),
            CachedResult::Cached { result } => {
                Self::send_result_to(result, reply_sender);
            }
        }
    }

    pub(crate) fn set_result(&mut self, result: eyre::Result<ControlRequestReply>) {
        match self {
            CachedResult::Pending { result_senders } => {
                for sender in result_senders.drain(..) {
                    Self::send_result_to(&result, sender);
                }
                *self = CachedResult::Cached { result };
            }
            CachedResult::Cached { .. } => {}
        }
    }

    fn send_result_to(
        result: &eyre::Result<ControlRequestReply>,
        sender: oneshot::Sender<eyre::Result<ControlRequestReply>>,
    ) {
        let result = match result {
            Ok(r) => Ok(r.clone()),
            Err(err) => Err(eyre!("{err:?}")),
        };
        let _ = sender.send(result);
    }
}

pub(crate) struct ArchivedDataflow {
    pub(crate) name: Option<String>,
    pub(crate) nodes: BTreeMap<NodeId, ResolvedNode>,
}

impl From<&RunningDataflow> for ArchivedDataflow {
    fn from(dataflow: &RunningDataflow) -> ArchivedDataflow {
        ArchivedDataflow {
            name: dataflow.name.clone(),
            nodes: dataflow.nodes.clone(),
        }
    }
}

impl PartialEq for RunningDataflow {
    fn eq(&self, other: &Self) -> bool {
        self.name == other.name && self.uuid == other.uuid && self.daemons == other.daemons
    }
}

impl Eq for RunningDataflow {}
