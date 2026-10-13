//! Typed ROS 2 action client and server on native Zenoh.
//!
//! These speak the `rcl_action` protocol over the raw [`ActionClient`] and
//! [`ActionServer`] endpoints: CDR-encoded `SendGoal`, `GetResult` and
//! `CancelGoal` services, the `FeedbackMessage` topic and the
//! `GoalStatusArray` topic. The goal, result and feedback types are any serde
//! types with the ROS field layout, such as the generated C++ binding types.

use std::{
    marker::PhantomData,
    sync::Arc,
    time::{Duration, Instant},
};

use futures::{FutureExt, future::BoxFuture, lock::Mutex, select};
use ros2_client::{
    action::{
        CancelGoalResponse, FeedbackMessage, GetResultRequest, GetResultResponse, GoalId, GoalInfo,
        GoalStatusEnum, SendGoalRequest, SendGoalResponse,
    },
    action_msgs::{CancelGoalResponseEnum, GoalStatus, GoalStatusArray},
    builtin_interfaces::Time,
};
use serde::{Deserialize, Serialize, de::DeserializeOwned};
use thiserror::Error;

use super::{
    ActionClient, ActionKeys, ActionServer, ActionTokens, ActionTransportError, GET_RESULT_TIMEOUT,
    wait_for_server,
};
use crate::transport::{
    RequestId, Ros2Qos,
    action::ActionError,
    zenoh::{
        Node, TransportCdrError, deserialize_cdr,
        pubsub::PubSubError,
        serialize_cdr,
        service::{ServiceError, ServiceRequest},
    },
};

#[derive(Debug, Error)]
#[non_exhaustive]
pub enum ActionCallError {
    #[error(transparent)]
    Service(#[from] ServiceError),
    #[error(transparent)]
    Topic(#[from] PubSubError),
    #[error(transparent)]
    Cdr(#[from] TransportCdrError),
    #[error(transparent)]
    Goal(#[from] ActionError),
    #[error(transparent)]
    Transport(#[from] ActionTransportError),
    #[error("the action server rejected the goal")]
    Rejected,
}

// Borrowing versions of the `ros2_client` wire types, so sending doesn't need
// to clone the user's value. Serde encodes `&T` exactly like `T`.
#[derive(Serialize)]
struct SendGoalWire<'a, G> {
    goal_id: GoalId,
    goal: &'a G,
}

#[derive(Serialize)]
struct GetResultWire<'a, R> {
    status: GoalStatusEnum,
    result: &'a R,
}

#[derive(Serialize)]
struct FeedbackWire<'a, F> {
    goal_id: GoalId,
    feedback: &'a F,
}

/// `action_msgs/srv/CancelGoal` request (`ros2_client`'s field is private).
#[derive(Serialize, Deserialize)]
struct CancelGoalWire {
    goal_info: GoalInfo,
}

/// Ties the message types to a client or server without owning a value, so
/// both stay `Send` and `Sync` whatever the types are.
type TypeMarker<T> = PhantomData<fn() -> T>;

/// Action client for goals of type `G` with result `R` and feedback `F`.
pub struct TypedActionClient<G, R, F> {
    endpoints: ActionClient,
    tokens: ActionTokens,
    _types: TypeMarker<(G, R, F)>,
}

impl<G, R, F> TypedActionClient<G, R, F>
where
    G: Serialize,
    R: DeserializeOwned,
    F: DeserializeOwned,
{
    pub async fn declare(
        node: &Node,
        keys: &ActionKeys,
        tokens: ActionTokens,
        qos: &Ros2Qos,
        max_payload_size: usize,
    ) -> Result<Self, ActionTransportError> {
        let endpoints =
            ActionClient::declare(node, keys, tokens.clone(), qos, max_payload_size).await?;
        Ok(Self {
            endpoints,
            tokens,
            _types: PhantomData,
        })
    }

    /// Waits until all five endpoints of a matching server are in `node`'s
    /// ROS graph.
    pub async fn wait_for_server(
        &self,
        node: &Node,
        deadline: Instant,
    ) -> Result<(), ActionTransportError> {
        wait_for_server(node.graph(), &self.tokens, deadline).await
    }

    /// Sends a goal under a new random id and returns that id once the server
    /// accepted it.
    pub async fn send_goal(&self, goal: &G, timeout: Duration) -> Result<GoalId, ActionCallError> {
        let goal_id = GoalId::new_random();
        let request = serialize_cdr(&SendGoalWire { goal_id, goal })?;
        let response = self.endpoints.send_goal.call(request, timeout).await?;
        let response: SendGoalResponse = deserialize_cdr(&response)?;
        if response.accepted {
            Ok(goal_id)
        } else {
            Err(ActionCallError::Rejected)
        }
    }

    /// Waits for the goal's final status and result, however long the goal
    /// runs.
    pub async fn get_result(
        &self,
        goal_id: GoalId,
    ) -> Result<(GoalStatusEnum, R), ActionCallError> {
        let request = serialize_cdr(&GetResultRequest { goal_id })?;
        let response = self
            .endpoints
            .get_result
            .call(request, GET_RESULT_TIMEOUT)
            .await?;
        let response: GetResultResponse<R> = deserialize_cdr(&response)?;
        Ok((response.status, response.result))
    }

    /// Asks the server to cancel one goal.
    pub async fn cancel_goal(
        &self,
        goal_id: GoalId,
        timeout: Duration,
    ) -> Result<CancelGoalResponse, ActionCallError> {
        let request = serialize_cdr(&CancelGoalWire {
            goal_info: GoalInfo {
                goal_id,
                stamp: Time::ZERO,
            },
        })?;
        let response = self.endpoints.cancel_goal.call(request, timeout).await?;
        Ok(deserialize_cdr(&response)?)
    }

    /// Next feedback message published on this action, for any goal.
    pub async fn recv_feedback(&self) -> Result<(GoalId, F), ActionCallError> {
        let (payload, _) = self.endpoints.feedback.recv_async().await?;
        let message: FeedbackMessage<F> = deserialize_cdr(&payload)?;
        Ok((message.goal_id, message.feedback))
    }

    /// Next status array published by the server.
    pub async fn recv_status(&self) -> Result<Vec<GoalStatus>, ActionCallError> {
        let (payload, _) = self.endpoints.status.recv_async().await?;
        Ok(deserialize_cdr::<GoalStatusArray>(&payload)?.status_list)
    }
}

#[derive(Clone, Copy, Debug)]
pub struct ActionServerOptions {
    /// Goals the server keeps at once. When full, the oldest finished goal
    /// makes room; if every goal is still running, new goals are rejected.
    pub max_goals: usize,
    /// A goal that runs this long without being finished is aborted (its
    /// result is the default value), so a handler that drops goals can't fill
    /// the server up for good or leave clients waiting forever.
    pub goal_timeout: Duration,
}

/// Runs the server's request handling. Spawn it on an executor; it ends when
/// the [`TypedActionServer`] is dropped.
pub type ActionServerPump = BoxFuture<'static, ()>;

/// Action server for goals of type `G` with result `R` and feedback `F`.
///
/// Every goal is accepted and set to executing unless the server is full or
/// the goal id is already in use. Accepted goals arrive on
/// [`goals`](Self::goals); the owner reports progress with
/// [`publish_feedback`](Self::publish_feedback) and ends the goal with
/// [`finish`](Self::finish). Results stay available to `get_result` until the
/// goal is evicted to make room for a new one. Like rclcpp, `get_result` for a
/// goal the server doesn't know returns `STATUS_UNKNOWN` and a default result.
pub struct TypedActionServer<G, R, F> {
    shared: Arc<Shared>,
    goals: flume::Receiver<(GoalId, G)>,
    _shutdown: flume::Sender<()>,
    _types: TypeMarker<(R, F)>,
}

struct Shared {
    endpoints: ActionServer,
    options: ActionServerOptions,
    // In accept order, which is also the order of the published status list.
    // At most `max_goals` entries, so a linear search is fine.
    goals: Mutex<Vec<ServerGoal>>,
    /// Encoded `GetResult` response with a default result for goals aborted
    /// because they ran past `goal_timeout`.
    aborted_response: Vec<u8>,
    /// The same with status `Unknown`, for goals the server doesn't know.
    unknown_response: Vec<u8>,
}

struct ServerGoal {
    id: GoalId,
    stamp: Time,
    accepted_at: Instant,
    status: GoalStatusEnum,
    /// The encoded `GetResult` response, set when the goal ends.
    result: Option<Vec<u8>>,
    /// `GetResult` requests that came in before the goal finished.
    waiters: Vec<RequestId>,
}

impl<G, R, F> TypedActionServer<G, R, F>
where
    G: DeserializeOwned + Send + 'static,
    R: Serialize + Default,
    F: Serialize,
{
    pub async fn declare(
        node: &Node,
        keys: &ActionKeys,
        tokens: ActionTokens,
        qos: &Ros2Qos,
        options: ActionServerOptions,
    ) -> Result<(Self, ActionServerPump), ActionCallError> {
        let default_response = |status| {
            serialize_cdr(&GetResultWire {
                status,
                result: &R::default(),
            })
        };
        let shared = Arc::new(Shared {
            aborted_response: default_response(GoalStatusEnum::Aborted)?,
            unknown_response: default_response(GoalStatusEnum::Unknown)?,
            endpoints: ActionServer::declare(node, keys, tokens, qos).await?,
            options,
            goals: Mutex::new(Vec::new()),
        });
        // Unbounded but still limited: a goal is only queued when it is
        // accepted, and at most `max_goals` goals run at once.
        let (goal_tx, goals) = flume::unbounded();
        let (shutdown, shutdown_rx) = flume::bounded(1);
        let pump = run_server(shared.clone(), goal_tx, shutdown_rx).boxed();
        Ok((
            Self {
                shared,
                goals,
                _shutdown: shutdown,
                _types: PhantomData,
            },
            pump,
        ))
    }

    /// Accepted goals, in the order they were accepted.
    pub fn goals(&self) -> &flume::Receiver<(GoalId, G)> {
        &self.goals
    }

    pub async fn status(&self, goal_id: GoalId) -> Option<GoalStatusEnum> {
        let goals = self.shared.goals.lock().await;
        goals
            .iter()
            .find(|goal| goal.id == goal_id)
            .map(|goal| goal.status)
    }

    pub async fn publish_feedback(
        &self,
        goal_id: GoalId,
        feedback: &F,
    ) -> Result<(), ActionCallError> {
        let payload = serialize_cdr(&FeedbackWire { goal_id, feedback })?;
        let goals = self.shared.goals.lock().await;
        let goal = goals
            .iter()
            .find(|goal| goal.id == goal_id)
            .ok_or(ActionError::UnknownGoal)?;
        if is_terminal(goal.status) {
            return Err(ActionError::TerminalGoal.into());
        }
        self.shared.endpoints.feedback.publish(&payload).await?;
        Ok(())
    }

    /// Ends a goal with a terminal `status` and answers every client waiting
    /// for its result.
    pub async fn finish(
        &self,
        goal_id: GoalId,
        status: GoalStatusEnum,
        result: &R,
    ) -> Result<(), ActionCallError> {
        if !is_terminal(status) {
            return Err(ActionError::InvalidTerminalStatus.into());
        }
        let response = serialize_cdr(&GetResultWire { status, result })?;
        let mut goals = self.shared.goals.lock().await;
        let index = goals
            .iter()
            .position(|goal| goal.id == goal_id)
            .ok_or(ActionError::UnknownGoal)?;
        let goal = &mut goals[index];
        if is_terminal(goal.status) {
            return Err(ActionError::TerminalGoal.into());
        }
        goal.status = status;
        let waiters = std::mem::take(&mut goal.waiters);
        let published = self.shared.publish_status(&goals).await;
        for request in waiters {
            if let Err(error) = self
                .shared
                .endpoints
                .get_result
                .reply(request, &response)
                .await
            {
                tracing::warn!("failed to send Zenoh action result: {error}");
            }
        }
        goals[index].result = Some(response);
        published
    }
}

async fn run_server<G: DeserializeOwned>(
    shared: Arc<Shared>,
    goal_tx: flume::Sender<(GoalId, G)>,
    shutdown: flume::Receiver<()>,
) {
    let endpoints = &shared.endpoints;
    // How often goals are checked against `goal_timeout`, so a goal is
    // aborted on time even when no other request comes in.
    let reap_interval = shared
        .options
        .goal_timeout
        .clamp(Duration::from_millis(10), Duration::from_secs(1));
    let mut next_reap = Instant::now() + reap_interval;
    loop {
        let until_reap = next_reap.saturating_duration_since(Instant::now());
        // Every `recv` is a flume `recv_async`, which loses nothing when the
        // `select!` drops it.
        let handled = select! {
            request = endpoints.send_goal.recv().fuse() => match request {
                Ok(request) => shared.handle_send_goal(request, &goal_tx).await,
                Err(_) => break,
            },
            request = endpoints.get_result.recv().fuse() => match request {
                Ok(request) => shared.handle_get_result(request).await,
                Err(_) => break,
            },
            request = endpoints.cancel_goal.recv().fuse() => match request {
                Ok(request) => shared.handle_cancel_goal(request).await,
                Err(_) => break,
            },
            _ = futures_timer::Delay::new(until_reap).fuse() => {
                next_reap = Instant::now() + reap_interval;
                shared.reap_abandoned().await
            },
            // Only ever fires when the server object is dropped.
            _ = shutdown.recv_async().fuse() => break,
        };
        // A client that went away mid-request must not stop the server.
        if let Err(error) = handled {
            tracing::warn!("Zenoh action server: {error}");
        }
    }
}

impl Shared {
    async fn handle_send_goal<G: DeserializeOwned>(
        &self,
        request: ServiceRequest,
        goal_tx: &flume::Sender<(GoalId, G)>,
    ) -> Result<(), ActionCallError> {
        let decoded: SendGoalRequest<G> = match deserialize_cdr(&request.payload) {
            Ok(decoded) => decoded,
            Err(error) => {
                self.endpoints
                    .send_goal
                    .reject(request.id, &format!("invalid goal: {error}"))
                    .await?;
                return Ok(());
            }
        };
        let mut goals = self.goals.lock().await;
        let mut changed = self.abort_abandoned(&mut goals).await;
        let admission = admit(&goals, decoded.goal_id, self.options.max_goals);
        let accepted = admission.is_some();
        let stamp = Time::from_nanos(unix_timestamp_ns());
        let response = serialize_cdr(&SendGoalResponse { accepted, stamp })?;
        let replied = self.endpoints.send_goal.reply(request.id, &response).await;
        // Only change the goal table once the client was told: if the reply
        // fails, the evicted goal's result is kept.
        if let (Some(evict), Ok(())) = (admission, &replied) {
            if let Some(oldest) = evict {
                goals.remove(oldest);
            }
            goals.push(ServerGoal {
                id: decoded.goal_id,
                stamp,
                accepted_at: Instant::now(),
                status: GoalStatusEnum::Executing,
                result: None,
                waiters: Vec::new(),
            });
            changed = true;
            // Fails only when the server object is gone, and then the pump
            // stops as well.
            let _ = goal_tx.send((decoded.goal_id, decoded.goal));
        }
        if changed {
            self.publish_status(&goals).await?;
        }
        Ok(replied?)
    }

    async fn handle_get_result(&self, request: ServiceRequest) -> Result<(), ActionCallError> {
        let goal_id = match deserialize_cdr::<GetResultRequest>(&request.payload) {
            Ok(decoded) => decoded.goal_id,
            Err(error) => {
                self.endpoints
                    .get_result
                    .reject(request.id, &format!("invalid request: {error}"))
                    .await?;
                return Ok(());
            }
        };
        let mut goals = self.goals.lock().await;
        let response = match goals.iter_mut().find(|goal| goal.id == goal_id) {
            Some(goal) => match &goal.result {
                Some(response) => response,
                None => {
                    goal.waiters.push(request.id);
                    return Ok(());
                }
            },
            None => &self.unknown_response,
        };
        self.endpoints
            .get_result
            .reply(request.id, response)
            .await?;
        Ok(())
    }

    async fn handle_cancel_goal(&self, request: ServiceRequest) -> Result<(), ActionCallError> {
        let requested = match deserialize_cdr::<CancelGoalWire>(&request.payload) {
            Ok(decoded) => decoded.goal_info,
            Err(error) => {
                self.endpoints
                    .cancel_goal
                    .reject(request.id, &format!("invalid request: {error}"))
                    .await?;
                return Ok(());
            }
        };
        let mut goals = self.goals.lock().await;
        let (goals_canceling, return_code) = cancel(&mut goals, &requested);
        let changed = !goals_canceling.is_empty();
        let response = serialize_cdr(&CancelGoalResponse {
            return_code,
            goals_canceling,
        })?;
        self.endpoints
            .cancel_goal
            .reply(request.id, &response)
            .await?;
        if changed {
            self.publish_status(&goals).await?;
        }
        Ok(())
    }

    async fn reap_abandoned(&self) -> Result<(), ActionCallError> {
        let mut goals = self.goals.lock().await;
        if self.abort_abandoned(&mut goals).await {
            self.publish_status(&goals).await?;
        }
        Ok(())
    }

    /// Aborts goals that ran past `goal_timeout` and answers their result
    /// waiters. Returns whether any goal changed.
    async fn abort_abandoned(&self, goals: &mut [ServerGoal]) -> bool {
        let mut changed = false;
        for goal in goals.iter_mut() {
            if is_terminal(goal.status) || goal.accepted_at.elapsed() < self.options.goal_timeout {
                continue;
            }
            tracing::warn!(
                "aborting action goal {} that was not finished within {:?}",
                goal.id.uuid,
                self.options.goal_timeout
            );
            goal.status = GoalStatusEnum::Aborted;
            goal.result = Some(self.aborted_response.clone());
            changed = true;
            for request in std::mem::take(&mut goal.waiters) {
                if let Err(error) = self
                    .endpoints
                    .get_result
                    .reply(request, &self.aborted_response)
                    .await
                {
                    tracing::warn!("failed to send Zenoh action result: {error}");
                }
            }
        }
        changed
    }

    async fn publish_status(&self, goals: &[ServerGoal]) -> Result<(), ActionCallError> {
        let status = GoalStatusArray {
            status_list: goals
                .iter()
                .map(|goal| GoalStatus {
                    goal_info: GoalInfo {
                        goal_id: goal.id,
                        stamp: goal.stamp,
                    },
                    status: goal.status,
                })
                .collect(),
        };
        self.endpoints
            .status
            .publish(&serialize_cdr(&status)?)
            .await?;
        Ok(())
    }
}

fn is_terminal(status: GoalStatusEnum) -> bool {
    matches!(
        status,
        GoalStatusEnum::Succeeded | GoalStatusEnum::Canceled | GoalStatusEnum::Aborted
    )
}

/// Whether a new goal `id` can be accepted: `None` to reject it, else
/// `Some(index)` of the oldest finished goal to evict first when the server
/// is full. Like `rcl_action`, an id that is already in use is rejected, so
/// the live goal and its result waiters are left alone.
fn admit(goals: &[ServerGoal], id: GoalId, max_goals: usize) -> Option<Option<usize>> {
    if goals.iter().any(|goal| goal.id == id) {
        return None;
    }
    if goals.len() < max_goals {
        return Some(None);
    }
    goals
        .iter()
        .position(|goal| is_terminal(goal.status))
        .map(Some)
}

/// Moves the goals a `CancelGoal` request selects to canceling, the way
/// `rcl_action_process_cancel_request` (`rcl_action/src/rcl_action/action_server.c`)
/// does when every cancel is accepted, as here. A non-zero id with a zero
/// stamp names one goal: the reply is `UnknownGoal` if there is no such goal
/// and `GoalTerminated` if it can't be canceled any more (already canceling or
/// finished). Otherwise a zero id and stamp select every goal, and a non-zero
/// stamp selects the goals accepted at or before it plus the named goal; the
/// reply is `None` even when none of them could be canceled. Only accepted
/// or executing goals can be canceled.
fn cancel(
    goals: &mut [ServerGoal],
    requested: &GoalInfo,
) -> (Vec<GoalInfo>, CancelGoalResponseEnum) {
    let cancelable =
        |status| matches!(status, GoalStatusEnum::Accepted | GoalStatusEnum::Executing);
    let named = (!requested.goal_id.uuid.is_nil()).then_some(requested.goal_id);
    let mut canceling = Vec::new();
    let mut start_canceling = |goal: &mut ServerGoal| {
        goal.status = GoalStatusEnum::Canceling;
        canceling.push(GoalInfo {
            goal_id: goal.id,
            stamp: goal.stamp,
        });
    };
    if let Some(id) = named
        && requested.stamp == Time::ZERO
    {
        let return_code = match goals.iter_mut().find(|goal| goal.id == id) {
            None => CancelGoalResponseEnum::UnknownGoal,
            Some(goal) if !cancelable(goal.status) => CancelGoalResponseEnum::GoalTerminated,
            Some(goal) => {
                start_canceling(goal);
                CancelGoalResponseEnum::None
            }
        };
        return (canceling, return_code);
    }
    for goal in goals.iter_mut() {
        let selected = requested.stamp == Time::ZERO
            || goal.stamp <= requested.stamp
            || Some(goal.id) == named;
        if selected && cancelable(goal.status) {
            start_canceling(goal);
        }
    }
    (canceling, CancelGoalResponseEnum::None)
}

fn unix_timestamp_ns() -> i64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .ok()
        .and_then(|duration| i64::try_from(duration.as_nanos()).ok())
        .unwrap_or(0)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Goal id with all 16 bytes set to `n` (`n == 0` is the nil id).
    fn id(n: u8) -> GoalId {
        let mut payload = vec![0, 1, 0, 0];
        payload.extend([n; 16]);
        deserialize_cdr(&payload).unwrap()
    }

    fn goal(n: u8, stamp: i64, status: GoalStatusEnum) -> ServerGoal {
        ServerGoal {
            id: id(n),
            stamp: Time::from_nanos(stamp),
            accepted_at: Instant::now(),
            status,
            result: None,
            waiters: Vec::new(),
        }
    }

    fn info(n: u8, stamp: i64) -> GoalInfo {
        GoalInfo {
            goal_id: id(n),
            stamp: Time::from_nanos(stamp),
        }
    }

    fn statuses(goals: &[ServerGoal]) -> Vec<GoalStatusEnum> {
        goals.iter().map(|goal| goal.status).collect()
    }

    use GoalStatusEnum::{Aborted, Canceling, Executing, Succeeded};

    #[test]
    fn cancel_with_zero_id_and_stamp_cancels_every_running_goal() {
        let mut goals = vec![
            goal(1, 10, Executing),
            goal(2, 20, Succeeded),
            goal(3, 30, Executing),
        ];
        let (canceling, code) = cancel(&mut goals, &info(0, 0));
        assert!(matches!(code, CancelGoalResponseEnum::None));
        assert_eq!(canceling.len(), 2);
        assert_eq!(statuses(&goals), [Canceling, Succeeded, Canceling]);
    }

    #[test]
    fn cancel_by_stamp_takes_goals_accepted_at_or_before_it() {
        let mut goals = vec![
            goal(1, 10, Executing),
            goal(2, 20, Executing),
            goal(3, 30, Executing),
        ];
        cancel(&mut goals, &info(0, 20));
        assert_eq!(statuses(&goals), [Canceling, Canceling, Executing]);

        // An id plus a stamp: that goal and everything up to the stamp.
        let mut goals = vec![
            goal(1, 10, Executing),
            goal(2, 20, Executing),
            goal(3, 30, Executing),
        ];
        cancel(&mut goals, &info(3, 10));
        assert_eq!(statuses(&goals), [Canceling, Executing, Canceling]);
    }

    /// rcl answers a cancel-all or by-stamp request with `ERROR_NONE` even
    /// when no goal could be canceled; only a by-id request reports why not.
    #[test]
    fn cancel_all_with_nothing_running_is_still_accepted() {
        let mut goals = vec![goal(1, 10, Succeeded)];
        let (canceling, code) = cancel(&mut goals, &info(0, 0));
        assert!(canceling.is_empty());
        assert!(matches!(code, CancelGoalResponseEnum::None));

        // An id plus a stamp is not a by-id request either.
        let (canceling, code) = cancel(&mut goals, &info(1, 5));
        assert!(canceling.is_empty());
        assert!(matches!(code, CancelGoalResponseEnum::None));
    }

    #[test]
    fn cancel_by_id_reports_an_unknown_or_uncancelable_goal() {
        let mut goals = vec![goal(1, 10, Canceling), goal(2, 20, Succeeded)];
        for (id, expected) in [
            (1, CancelGoalResponseEnum::GoalTerminated),
            (2, CancelGoalResponseEnum::GoalTerminated),
            (3, CancelGoalResponseEnum::UnknownGoal),
        ] {
            let (canceling, code) = cancel(&mut goals, &info(id, 0));
            assert!(canceling.is_empty());
            assert_eq!(code, expected, "goal {id}");
        }
    }

    #[test]
    fn admit_rejects_a_goal_id_in_use_and_evicts_the_oldest_finished_goal() {
        let goals = vec![goal(1, 10, Executing)];
        assert_eq!(admit(&goals, id(1), 4), None, "id in use");
        assert_eq!(admit(&goals, id(2), 4), Some(None), "room left");

        // Full: goal 2 is the oldest finished goal, so it makes room.
        let goals = vec![
            goal(1, 10, Executing),
            goal(2, 20, Aborted),
            goal(3, 30, Succeeded),
        ];
        assert_eq!(admit(&goals, id(4), 3), Some(Some(1)));

        // Full of running goals.
        let goals = vec![goal(1, 10, Executing), goal(2, 20, Canceling)];
        assert_eq!(admit(&goals, id(3), 2), None);
    }
}
