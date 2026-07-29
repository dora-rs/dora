//! Transport-neutral ROS 2 action endpoint expansion and goal orchestration.

use std::{
    collections::HashMap,
    sync::{
        Arc,
        atomic::{AtomicUsize, Ordering},
    },
};

use thiserror::Error;

pub const MAX_CONCURRENT_GOALS: usize = 8;

#[derive(Clone)]
pub struct ConcurrentGoalLimit {
    count: Arc<AtomicUsize>,
    limit: usize,
}

impl ConcurrentGoalLimit {
    pub fn new(limit: usize) -> Self {
        Self {
            count: Arc::new(AtomicUsize::new(0)),
            limit,
        }
    }
    pub fn try_acquire(&self) -> Result<GoalPermit, ActionError> {
        self.count
            .fetch_update(Ordering::AcqRel, Ordering::Acquire, |count| {
                (count < self.limit).then_some(count + 1)
            })
            .map_err(|_| ActionError::GoalLimit { limit: self.limit })?;
        Ok(GoalPermit {
            count: self.count.clone(),
        })
    }
    pub fn active(&self) -> usize {
        self.count.load(Ordering::Acquire)
    }
}

pub struct GoalPermit {
    count: Arc<AtomicUsize>,
}
impl Drop for GoalPermit {
    fn drop(&mut self) {
        self.count.fetch_sub(1, Ordering::AcqRel);
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ActionEndpoint {
    pub name: String,
    pub type_name: String,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ActionEndpoints {
    pub send_goal: ActionEndpoint,
    pub get_result: ActionEndpoint,
    pub cancel_goal: ActionEndpoint,
    pub feedback: ActionEndpoint,
    pub status: ActionEndpoint,
}

impl ActionEndpoints {
    pub fn new(name: &str, package: &str, action: &str) -> Self {
        let base = name.trim_end_matches('/');
        let [
            send_goal_type,
            get_result_type,
            cancel_goal_type,
            feedback_type,
            status_type,
        ] = dora_ros2_bridge_msg_gen::types::action_dds_type_names(package, action);
        Self {
            send_goal: ActionEndpoint {
                name: format!("{base}/_action/send_goal"),
                type_name: send_goal_type,
            },
            get_result: ActionEndpoint {
                name: format!("{base}/_action/get_result"),
                type_name: get_result_type,
            },
            cancel_goal: ActionEndpoint {
                name: format!("{base}/_action/cancel_goal"),
                type_name: cancel_goal_type,
            },
            feedback: ActionEndpoint {
                name: format!("{base}/_action/feedback"),
                type_name: feedback_type,
            },
            status: ActionEndpoint {
                name: format!("{base}/_action/status"),
                type_name: status_type,
            },
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum GoalStatus {
    Executing,
    Canceling,
    Succeeded,
    Canceled,
    Aborted,
}

impl GoalStatus {
    pub fn is_terminal(self) -> bool {
        matches!(self, Self::Succeeded | Self::Canceled | Self::Aborted)
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ResultAvailability<R> {
    Pending,
    Ready { status: GoalStatus, result: R },
    ServerLost,
}

#[derive(Debug, Error, Eq, PartialEq)]
pub enum ActionError {
    #[error("max concurrent goals ({limit}) reached")]
    GoalLimit { limit: usize },
    #[error("unknown goal ID")]
    UnknownGoal,
    #[error("goal is already terminal")]
    TerminalGoal,
    #[error("invalid terminal goal status")]
    InvalidTerminalStatus,
}

/// Bounded goal storage shared by the language bindings and standalone bridge.
pub struct GoalSlots<K, V> {
    values: HashMap<K, V>,
    limit: usize,
}

impl<K: std::hash::Hash + Eq, V> GoalSlots<K, V> {
    pub fn new(limit: usize) -> Self {
        Self {
            values: HashMap::new(),
            limit,
        }
    }
    pub fn insert(&mut self, key: K, value: V) -> Result<Option<V>, ActionError> {
        if self.values.len() >= self.limit && !self.values.contains_key(&key) {
            return Err(ActionError::GoalLimit { limit: self.limit });
        }
        Ok(self.values.insert(key, value))
    }
    pub fn get<Q>(&self, key: &Q) -> Option<&V>
    where
        K: std::borrow::Borrow<Q>,
        Q: ?Sized + std::hash::Hash + Eq,
    {
        self.values.get(key)
    }
    pub fn get_mut(&mut self, key: &K) -> Option<&mut V> {
        self.values.get_mut(key)
    }
    pub fn contains_key<Q>(&self, key: &Q) -> bool
    where
        K: std::borrow::Borrow<Q>,
        Q: ?Sized + std::hash::Hash + Eq,
    {
        self.values.contains_key(key)
    }
    pub fn remove<Q>(&mut self, key: &Q) -> Option<V>
    where
        K: std::borrow::Borrow<Q>,
        Q: ?Sized + std::hash::Hash + Eq,
    {
        self.values.remove(key)
    }
    pub fn clear(&mut self) {
        self.values.clear();
    }
    pub fn len(&self) -> usize {
        self.values.len()
    }
    pub fn is_empty(&self) -> bool {
        self.values.is_empty()
    }
    pub fn iter(&self) -> impl Iterator<Item = (&K, &V)> {
        self.values.iter()
    }
}

struct Goal<G, R> {
    value: G,
    status: GoalStatus,
    feedback_sequence: u64,
    result: Option<R>,
    server_lost: bool,
    /// Monotonic acceptance order — used to evict the oldest terminal goal.
    seq: u64,
}

pub struct ActionState<G, R> {
    goals: HashMap<[u8; 16], Goal<G, R>>,
    limit: usize,
    next_seq: u64,
}

impl<G, R: Clone> ActionState<G, R> {
    pub fn new(limit: usize) -> Self {
        Self {
            goals: HashMap::new(),
            limit,
            next_seq: 0,
        }
    }
    pub fn accept(&mut self, id: [u8; 16], value: G) -> Result<(), ActionError> {
        let active = self
            .goals
            .values()
            .filter(|goal| !goal.status.is_terminal())
            .count();
        if active >= self.limit {
            return Err(ActionError::GoalLimit { limit: self.limit });
        }
        // Only active goals count against `limit`; terminal goals linger until
        // their result is fetched (`remove`). Bound total retention at 2*limit
        // by evicting the OLDEST terminal goals first, so a recently
        // finished-but-unpolled result survives longer than an all-or-nothing
        // purge, while a client that never fetches can't grow the map unbounded.
        let cap = self.limit.saturating_mul(2);
        if self.goals.len() >= cap {
            let mut terminal: Vec<([u8; 16], u64)> = self
                .goals
                .iter()
                .filter(|(_, goal)| goal.status.is_terminal())
                .map(|(goal_id, goal)| (*goal_id, goal.seq))
                .collect();
            terminal.sort_by_key(|(_, seq)| *seq);
            for (goal_id, _) in terminal {
                if self.goals.len() < cap {
                    break;
                }
                self.goals.remove(&goal_id);
            }
        }
        let seq = self.next_seq;
        self.next_seq = self.next_seq.wrapping_add(1);
        self.goals.insert(
            id,
            Goal {
                value,
                status: GoalStatus::Executing,
                feedback_sequence: 0,
                result: None,
                server_lost: false,
                seq,
            },
        );
        Ok(())
    }
    pub fn reject(&mut self, id: [u8; 16]) {
        self.goals.remove(&id);
    }
    pub fn status(&self, id: [u8; 16]) -> Option<GoalStatus> {
        self.goals.get(&id).map(|goal| goal.status)
    }
    pub fn goal(&self, id: [u8; 16]) -> Result<&G, ActionError> {
        self.goals
            .get(&id)
            .map(|goal| &goal.value)
            .ok_or(ActionError::UnknownGoal)
    }
    pub fn feedback(&mut self, id: [u8; 16], _feedback: &[u8]) -> Result<u64, ActionError> {
        let goal = self.goals.get_mut(&id).ok_or(ActionError::UnknownGoal)?;
        if goal.status.is_terminal() {
            return Err(ActionError::TerminalGoal);
        }
        goal.feedback_sequence = goal
            .feedback_sequence
            .checked_add(1)
            .ok_or(ActionError::TerminalGoal)?;
        Ok(goal.feedback_sequence)
    }
    pub fn cancel(&mut self, id: [u8; 16]) -> Result<(), ActionError> {
        let goal = self.goals.get_mut(&id).ok_or(ActionError::UnknownGoal)?;
        if goal.status.is_terminal() {
            return Err(ActionError::TerminalGoal);
        }
        goal.status = GoalStatus::Canceling;
        Ok(())
    }
    pub fn finish(
        &mut self,
        id: [u8; 16],
        status: GoalStatus,
        result: R,
    ) -> Result<(), ActionError> {
        if !status.is_terminal() {
            return Err(ActionError::InvalidTerminalStatus);
        }
        let goal = self.goals.get_mut(&id).ok_or(ActionError::UnknownGoal)?;
        if goal.status.is_terminal() {
            return Err(ActionError::TerminalGoal);
        }
        goal.status = status;
        goal.result = Some(result);
        Ok(())
    }
    pub fn request_result(&self, id: [u8; 16]) -> Result<ResultAvailability<R>, ActionError> {
        let goal = self.goals.get(&id).ok_or(ActionError::UnknownGoal)?;
        if goal.server_lost {
            return Ok(ResultAvailability::ServerLost);
        }
        Ok(match &goal.result {
            Some(result) => ResultAvailability::Ready {
                status: goal.status,
                result: result.clone(),
            },
            None => ResultAvailability::Pending,
        })
    }
    pub fn remove(&mut self, id: [u8; 16]) -> Option<G> {
        self.goals.remove(&id).map(|goal| goal.value)
    }
    pub fn len(&self) -> usize {
        self.goals.len()
    }
    pub fn is_empty(&self) -> bool {
        self.goals.is_empty()
    }
    pub fn server_lost(&mut self) {
        for goal in self
            .goals
            .values_mut()
            .filter(|goal| !goal.status.is_terminal())
        {
            goal.status = GoalStatus::Aborted;
            goal.server_lost = true;
        }
    }
}

#[cfg(feature = "rmw-zenoh")]
pub mod zenoh {
    use std::{sync::Arc, time::Duration};

    use thiserror::Error;

    use crate::transport::{
        Ros2Qos,
        zenoh::{
            Node,
            keyexpr::TopicToken,
            pubsub::{NodePublisher, NodeSubscription, PubSubError},
            service::{NodeServiceClient, NodeServiceServer, ServiceError},
        },
    };

    /// Data keys for the three services followed by feedback and status.
    #[derive(Clone, Debug)]
    pub struct ActionKeys {
        pub send_goal: String,
        pub get_result: String,
        pub cancel_goal: String,
        pub feedback: String,
        pub status: String,
    }

    #[derive(Clone)]
    pub struct ActionTokens {
        pub send_goal: TopicToken,
        pub get_result: TopicToken,
        pub cancel_goal: TopicToken,
        pub feedback: TopicToken,
        pub status: TopicToken,
    }

    #[derive(Debug, Error)]
    pub enum ActionTransportError {
        #[error(transparent)]
        Service(#[from] ServiceError),
        #[error(transparent)]
        Topic(#[from] PubSubError),
        #[error("action server readiness timed out")]
        Timeout,
        #[error("ROS graph closed while waiting for action server")]
        Closed,
    }

    pub const GET_RESULT_TIMEOUT: Duration = Duration::from_secs(100 * 365 * 24 * 60 * 60);

    pub async fn wait_for_server(
        graph: &crate::transport::zenoh::graph::GraphCache,
        tokens: &ActionTokens,
        deadline: std::time::Instant,
    ) -> Result<(), ActionTransportError> {
        use crate::transport::zenoh::keyexpr::EntityKind;
        use futures::{FutureExt, select};
        loop {
            let snapshot = graph.snapshot();
            if snapshot.closed {
                return Err(ActionTransportError::Closed);
            }
            let matches = |kind: EntityKind, token: &TopicToken| {
                snapshot.entities.iter().any(|entity| {
                    entity.token.kind == kind
                        && entity
                            .token
                            .topic
                            .as_ref()
                            .is_some_and(|topic| topic == token)
                })
            };
            if matches(EntityKind::Service, &tokens.send_goal)
                && matches(EntityKind::Service, &tokens.get_result)
                && matches(EntityKind::Service, &tokens.cancel_goal)
                && matches(EntityKind::Publisher, &tokens.feedback)
                && matches(EntityKind::Publisher, &tokens.status)
            {
                return Ok(());
            }
            let remaining = deadline
                .checked_duration_since(std::time::Instant::now())
                .ok_or(ActionTransportError::Timeout)?;
            let changed = graph.wait_for_change(snapshot.generation).fuse();
            let timer = futures_timer::Delay::new(remaining).fuse();
            futures::pin_mut!(changed, timer);
            select! {
                result = changed => if result.is_err() { return Err(ActionTransportError::Closed); },
                _ = timer => return Err(ActionTransportError::Timeout),
            }
        }
    }

    pub struct ActionClient {
        pub send_goal: NodeServiceClient,
        pub get_result: NodeServiceClient,
        pub cancel_goal: NodeServiceClient,
        pub feedback: NodeSubscription<Vec<u8>>,
        pub status: NodeSubscription<Vec<u8>>,
    }

    impl ActionClient {
        pub async fn declare(
            node: &Node,
            keys: &ActionKeys,
            tokens: ActionTokens,
            qos: &Ros2Qos,
            max_payload_size: usize,
        ) -> Result<Self, ActionTransportError> {
            let decode = Arc::new(|bytes: &[u8]| Ok(bytes.to_vec()));
            let status_qos = action_status_qos(qos);
            Ok(Self {
                send_goal: NodeServiceClient::declare(node, &keys.send_goal, tokens.send_goal, 8)
                    .await?,
                get_result: NodeServiceClient::declare(
                    node,
                    &keys.get_result,
                    tokens.get_result,
                    8,
                )
                .await?,
                cancel_goal: NodeServiceClient::declare(
                    node,
                    &keys.cancel_goal,
                    tokens.cancel_goal,
                    8,
                )
                .await?,
                feedback: NodeSubscription::declare(
                    node,
                    &keys.feedback,
                    tokens.feedback,
                    qos,
                    8,
                    max_payload_size,
                    decode.clone(),
                )
                .await?,
                status: NodeSubscription::declare(
                    node,
                    &keys.status,
                    tokens.status,
                    &status_qos,
                    8,
                    max_payload_size,
                    decode,
                )
                .await?,
            })
        }
    }

    pub struct ActionServer {
        pub send_goal: NodeServiceServer,
        pub get_result: NodeServiceServer,
        pub cancel_goal: NodeServiceServer,
        pub feedback: NodePublisher,
        pub status: NodePublisher,
    }

    impl ActionServer {
        pub async fn declare(
            node: &Node,
            keys: &ActionKeys,
            tokens: ActionTokens,
            qos: &Ros2Qos,
        ) -> Result<Self, ActionTransportError> {
            let status_qos = action_status_qos(qos);
            Ok(Self {
                send_goal: NodeServiceServer::declare(
                    node,
                    &keys.send_goal,
                    tokens.send_goal,
                    64,
                    Duration::from_secs(30),
                )
                .await?,
                get_result: NodeServiceServer::declare(
                    node,
                    &keys.get_result,
                    tokens.get_result,
                    64,
                    GET_RESULT_TIMEOUT,
                )
                .await?,
                cancel_goal: NodeServiceServer::declare(
                    node,
                    &keys.cancel_goal,
                    tokens.cancel_goal,
                    64,
                    Duration::from_secs(30),
                )
                .await?,
                feedback: NodePublisher::declare(node, &keys.feedback, tokens.feedback, qos)
                    .await?,
                status: NodePublisher::declare(node, &keys.status, tokens.status, &status_qos)
                    .await?,
            })
        }
    }

    pub fn action_status_qos(qos: &Ros2Qos) -> Ros2Qos {
        Ros2Qos {
            reliability: crate::transport::Reliability::Reliable {
                max_blocking_time: Duration::ZERO,
            },
            durability: crate::transport::Durability::TransientLocal,
            history: crate::transport::History::KeepLast { depth: 1 },
            liveliness: qos.liveliness,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{ActionError, ActionState, GoalStatus, ResultAvailability};

    #[test]
    fn accept_evicts_oldest_terminal_first_keeping_recent_results() {
        // Under a flood of finished-but-unfetched goals, the OLDEST terminal
        // goals are evicted first, so the most-recently finished result is still
        // retrievable (rather than an all-or-nothing purge dropping it too).
        let mut state: ActionState<(), u8> = ActionState::new(2);
        let mut last = [0u8; 16];
        for i in 0..50u8 {
            let id = [i; 16];
            state.accept(id, ()).unwrap();
            state.finish(id, GoalStatus::Succeeded, i).unwrap();
            last = id;
        }
        assert!(matches!(
            state.request_result(last),
            Ok(ResultAvailability::Ready { .. })
        ));
    }

    #[test]
    fn accept_reclaims_terminal_goals_to_bound_retention() {
        // Only active goals count against the limit; terminal goals linger until
        // fetched. A peer that submits and finishes goals but never fetches
        // results must not grow the map without bound.
        let mut state: ActionState<(), ()> = ActionState::new(2);
        for i in 0..100u8 {
            let id = [i; 16];
            state.accept(id, ()).unwrap();
            state.finish(id, GoalStatus::Succeeded, ()).unwrap();
        }
        assert!(
            state.len() <= 2 * 2,
            "terminal goals must be reclaimed; retained {}",
            state.len()
        );
    }

    #[test]
    fn accept_still_enforces_the_active_limit() {
        let mut state: ActionState<(), ()> = ActionState::new(2);
        state.accept([1; 16], ()).unwrap();
        state.accept([2; 16], ()).unwrap();
        assert!(matches!(
            state.accept([3; 16], ()),
            Err(ActionError::GoalLimit { .. })
        ));
    }
}
