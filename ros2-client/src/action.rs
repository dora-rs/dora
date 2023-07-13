use std::{
  collections::{btree_map::Entry, BTreeMap},
  marker::PhantomData,
};

use rustdds::*;
use serde::{Deserialize, Serialize};
pub use action_msgs::{CancelGoalRequest, CancelGoalResponse, GoalId, GoalInfo, GoalStatusEnum};
use builtin_interfaces::Time;
#[allow(unused_imports)]
use log::{debug, error, info, warn};
use futures::{
  pin_mut,
  stream::{FusedStream, Stream, StreamExt},
  Future,
};

use crate::{
  action_msgs, builtin_interfaces,
  message::Message,
  service::{request_id::RmwRequestId, AService, Client, Server},
  unique_identifier_msgs, Publisher, Subscription,
};

// A trait to define an Action type
pub trait ActionTypes {
  type GoalType: Message + Clone; // Used by client to set a goal for the server
  type ResultType: Message + Clone; // Used by server to report result when action ends
  type FeedbackType: Message; // Used by server to report progrss during action excution

  fn goal_type_name(&self) -> &str;
  fn result_type_name(&self) -> &str;
  fn feedback_type_name(&self) -> &str;
}

// This is used to construct an ActionType implementation.
pub struct Action<G, R, F> {
  g: PhantomData<G>,
  r: PhantomData<R>,
  f: PhantomData<F>,
  goal_typename: String,
  result_typename: String,
  feedback_typename: String,
}

impl<G, R, F> Action<G, R, F>
where
  G: Message + Clone,
  R: Message + Clone,
  F: Message,
{
  pub fn new(goal_typename: String, result_typename: String, feedback_typename: String) -> Self {
    Self {
      goal_typename,
      result_typename,
      feedback_typename,
      g: PhantomData,
      r: PhantomData,
      f: PhantomData,
    }
  }
}

impl<G, R, F> ActionTypes for Action<G, R, F>
where
  G: Message + Clone,
  R: Message + Clone,
  F: Message,
{
  type GoalType = G;
  type ResultType = R;
  type FeedbackType = F;

  fn goal_type_name(&self) -> &str {
    &self.goal_typename
  }

  fn result_type_name(&self) -> &str {
    &self.result_typename
  }

  fn feedback_type_name(&self) -> &str {
    &self.feedback_typename
  }
}

//TODO: Make fields private, add constructr and accessors.
pub struct ActionClientQosPolicies {
  pub goal_service: QosPolicies,
  pub result_service: QosPolicies,
  pub cancel_service: QosPolicies,
  pub feedback_subscription: QosPolicies,
  pub status_subscription: QosPolicies,
}

pub struct ActionServerQosPolicies {
  pub goal_service: QosPolicies,
  pub result_service: QosPolicies,
  pub cancel_service: QosPolicies,
  pub feedback_publisher: QosPolicies,
  pub status_publisher: QosPolicies,
}

/// Emulating ROS2 IDL code generator: Goal sending/setting service

#[derive(Clone, Serialize, Deserialize, Debug)]
pub struct SendGoalRequest<G> {
  pub goal_id: GoalId,
  pub goal: G,
}
impl<G: Message> Message for SendGoalRequest<G> {}

#[derive(Clone, Serialize, Deserialize, Debug)]
pub struct SendGoalResponse {
  pub accepted: bool,
  pub stamp: builtin_interfaces::Time,
}
impl Message for SendGoalResponse {}

/// Emulating ROS2 IDL code generator: Result getting service
#[derive(Clone, Serialize, Deserialize, Debug)]
pub struct GetResultRequest {
  pub goal_id: GoalId,
}
impl Message for GetResultRequest {}

#[derive(Clone, Serialize, Deserialize, Debug)]
pub struct GetResultResponse<R> {
  pub status: GoalStatusEnum, // interpretation same as in GoalStatus message?
  pub result: R,
}
impl<R: Message> Message for GetResultResponse<R> {}

/// Emulating ROS2 IDL code generator: Feedback Topic message type
#[derive(Clone, Serialize, Deserialize, Debug)]
pub struct FeedbackMessage<F> {
  pub goal_id: GoalId,
  pub feedback: F,
}
impl<F: Message> Message for FeedbackMessage<F> {}

pub struct ActionClient<A>
where
  A: ActionTypes,
  A::GoalType: Message + Clone,
  A::ResultType: Message + Clone,
  A::FeedbackType: Message,
{
  pub(crate) my_goal_client: Client<AService<SendGoalRequest<A::GoalType>, SendGoalResponse>>,

  pub(crate) my_cancel_client:
    Client<AService<action_msgs::CancelGoalRequest, action_msgs::CancelGoalResponse>>,

  pub(crate) my_result_client: Client<AService<GetResultRequest, GetResultResponse<A::ResultType>>>,

  pub(crate) my_feedback_subscription: Subscription<FeedbackMessage<A::FeedbackType>>,

  pub(crate) my_status_subscription: Subscription<action_msgs::GoalStatusArray>,

  pub(crate) my_action_name: String,
}

impl<A> ActionClient<A>
where
  A: ActionTypes,
  A::GoalType: Message + Clone,
  A::ResultType: Message + Clone,
  A::FeedbackType: Message,
{
  pub fn name(&self) -> &str {
    &self.my_action_name
  }

  pub fn goal_client(
    &mut self,
  ) -> &mut Client<AService<SendGoalRequest<A::GoalType>, SendGoalResponse>> {
    &mut self.my_goal_client
  }
  pub fn cancel_client(
    &mut self,
  ) -> &mut Client<AService<action_msgs::CancelGoalRequest, action_msgs::CancelGoalResponse>> {
    &mut self.my_cancel_client
  }
  pub fn result_client(
    &mut self,
  ) -> &mut Client<AService<GetResultRequest, GetResultResponse<A::ResultType>>> {
    &mut self.my_result_client
  }
  pub fn feedback_subscription(&mut self) -> &mut Subscription<FeedbackMessage<A::FeedbackType>> {
    &mut self.my_feedback_subscription
  }
  pub fn status_subscription(&mut self) -> &mut Subscription<action_msgs::GoalStatusArray> {
    &mut self.my_status_subscription
  }

  /// Returns and id of the Request and id for the Goal.
  /// Request id can be used to recognize correct response from Action Server.
  /// Goal id is later used to communicate Goal status and result.
  pub fn send_goal(&self, goal: A::GoalType) -> dds::Result<(RmwRequestId, GoalId)>
  where
    <A as ActionTypes>::GoalType: 'static,
  {
    let goal_id = unique_identifier_msgs::UUID::new_random();
    self
      .my_goal_client
      .send_request(SendGoalRequest { goal_id, goal })
      .map(|req_id| (req_id, goal_id))
  }

  /// Receive a response for the specified goal request, or None if response is
  /// not yet available
  pub fn receive_goal_response(&self, req_id: RmwRequestId) -> dds::Result<Option<SendGoalResponse>>
  where
    <A as ActionTypes>::GoalType: 'static,
  {
    loop {
      match self.my_goal_client.receive_response() {
        Err(e) => break Err(e),
        Ok(None) => break Ok(None), // not yet
        Ok(Some((incoming_req_id, resp))) if incoming_req_id == req_id =>
        // received the expected answer
        {
          break Ok(Some(resp))
        }
        Ok(Some((incoming_req_id, _resp))) => {
          // got someone else's answer. Try again.
          info!(
            "Goal Response not for us: {:?} != {:?}",
            incoming_req_id, req_id
          );
          continue;
        }
      }
    }
    // We loop here to drain all the answers received so far.
    // The mio .poll() only does not trigger again for the next item, if it has
    // been received already.
  }

  pub async fn async_send_goal(&self, goal: A::GoalType) -> dds::Result<(GoalId, SendGoalResponse)>
  where
    <A as ActionTypes>::GoalType: 'static,
  {
    let goal_id = unique_identifier_msgs::UUID::new_random();
    let send_goal_response = self
      .my_goal_client
      .async_call_service(SendGoalRequest { goal_id, goal })
      .await?;
    Ok((goal_id, send_goal_response))
  }

  // From ROS2 docs:
  // https://docs.ros2.org/foxy/api/action_msgs/srv/CancelGoal.html
  //
  // Cancel one or more goals with the following policy:
  // - If the goal ID is zero and timestamp is zero, cancel all goals.
  // - If the goal ID is zero and timestamp is not zero, cancel all goals accepted
  //   at or before the timestamp.
  // - If the goal ID is not zero and timestamp is zero, cancel the goal with the
  //   given ID regardless of the time it was accepted.
  // - If the goal ID is not zero and timestamp is not zero, cancel the goal with
  //   the given ID and all goals accepted at or before the timestamp.

  fn cancel_goal_raw(&self, goal_id: GoalId, timestamp: Time) -> dds::Result<RmwRequestId> {
    let goal_info = GoalInfo {
      goal_id,
      stamp: timestamp,
    };
    self
      .my_cancel_client
      .send_request(CancelGoalRequest { goal_info })
  }

  pub fn cancel_goal(&self, goal_id: GoalId) -> dds::Result<RmwRequestId> {
    self.cancel_goal_raw(goal_id, Time::ZERO)
  }

  pub fn cancel_all_goals_before(&self, timestamp: Time) -> dds::Result<RmwRequestId> {
    self.cancel_goal_raw(GoalId::ZERO, timestamp)
  }

  pub fn cancel_all_goals(&self) -> dds::Result<RmwRequestId> {
    self.cancel_goal_raw(GoalId::ZERO, Time::ZERO)
  }

  pub fn receive_cancel_response(
    &self,
    cancel_request_id: RmwRequestId,
  ) -> dds::Result<Option<CancelGoalResponse>> {
    loop {
      match self.my_cancel_client.receive_response() {
        Err(e) => break Err(e),
        Ok(None) => break Ok(None), // not yet
        Ok(Some((incoming_req_id, resp))) if incoming_req_id == cancel_request_id => {
          break Ok(Some(resp))
        } // received expected answer
        Ok(Some(_)) => continue,    // got someone else's answer. Try again.
      }
    }
  }

  pub fn async_cancel_goal(
    &self,
    goal_id: GoalId,
    timestamp: Time,
  ) -> impl Future<Output = dds::Result<CancelGoalResponse>> + '_ {
    let goal_info = GoalInfo {
      goal_id,
      stamp: timestamp,
    };
    self
      .my_cancel_client
      .async_call_service(CancelGoalRequest { goal_info })
  }

  pub fn request_result(&self, goal_id: GoalId) -> dds::Result<RmwRequestId>
  where
    <A as ActionTypes>::ResultType: 'static,
  {
    self
      .my_result_client
      .send_request(GetResultRequest { goal_id })
  }

  pub fn receive_result(
    &self,
    result_request_id: RmwRequestId,
  ) -> dds::Result<Option<(GoalStatusEnum, A::ResultType)>>
  where
    <A as ActionTypes>::ResultType: 'static,
  {
    loop {
      match self.my_result_client.receive_response() {
        Err(e) => break Err(e),
        Ok(None) => break Ok(None), // not yet
        Ok(Some((incoming_req_id, GetResultResponse { status, result })))
          if incoming_req_id == result_request_id =>
        {
          break Ok(Some((status, result)))
        } // received expected answer
        Ok(Some(_)) => continue,    // got someone else's answer. Try again.
      }
    }
  }

  /// Asynchronously request goal result.
  /// Result should be requested as soon as a goal is accepted.
  /// Result ia actually received only when Server informs that the goal has
  /// either Succeeded, or has been Canceled or Aborted.
  pub async fn async_request_result(
    &self,
    goal_id: GoalId,
  ) -> dds::Result<(GoalStatusEnum, A::ResultType)>
  where
    <A as ActionTypes>::ResultType: 'static,
  {
    let GetResultResponse { status, result } = self
      .my_result_client
      .async_call_service(GetResultRequest { goal_id })
      .await?;
    Ok((status, result))
  }

  pub fn receive_feedback(&self, goal_id: GoalId) -> dds::Result<Option<A::FeedbackType>>
  where
    <A as ActionTypes>::FeedbackType: 'static,
  {
    loop {
      match self.my_feedback_subscription.take() {
        Err(e) => break Err(e),
        Ok(None) => break Ok(None),
        Ok(Some((fb_msg, _msg_info))) if fb_msg.goal_id == goal_id => {
          break Ok(Some(fb_msg.feedback))
        }
        Ok(Some((fb_msg, _msg_info))) => {
          // feedback on some other goal
          debug!(
            "Feedback on another goal {:?} != {:?}",
            fb_msg.goal_id, goal_id
          )
        }
      }
    }
  }

  /// Receive asynchronous feedback stream of goal progress.
  pub fn feedback_stream(
    &self,
    goal_id: GoalId,
  ) -> impl Stream<Item = dds::Result<A::FeedbackType>> + FusedStream + '_
  where
    <A as ActionTypes>::FeedbackType: 'static,
  {
    let expected_goal_id = goal_id; // rename
    self
      .my_feedback_subscription
      .async_stream()
      .filter_map(move |result| async move {
        match result {
          Err(e) => Some(Err(e)),
          Ok((FeedbackMessage { goal_id, feedback }, _msg_info)) => {
            if goal_id == expected_goal_id {
              Some(Ok(feedback))
            } else {
              debug!("Feedback for some other {:?}.", goal_id);
              None
            }
          }
        }
      })
  }

  /// Note: This does not take GoalId and will therefore report status of all
  /// Goals.
  pub fn receive_status(&self) -> dds::Result<Option<action_msgs::GoalStatusArray>> {
    self
      .my_status_subscription
      .take()
      .map(|r| r.map(|(gsa, _msg_info)| gsa))
  }

  pub async fn async_receive_status(&self) -> dds::Result<action_msgs::GoalStatusArray> {
    let (m, _msg_info) = self.my_status_subscription.async_take().await?;
    Ok(m)
  }

  /// Async Stream of status updates
  /// Action server send updates containing status of all goals, hence an array.
  pub fn all_statuses_stream(
    &self,
  ) -> impl Stream<Item = dds::Result<action_msgs::GoalStatusArray>> + FusedStream + '_ {
    self
      .my_status_subscription
      .async_stream()
      .map(|result| result.map(|(gsa, _mi)| gsa))
  }

  pub fn status_stream(
    &self,
    goal_id: GoalId,
  ) -> impl Stream<Item = dds::Result<action_msgs::GoalStatus>> + FusedStream + '_ {
    self
      .all_statuses_stream()
      .filter_map(move |result| async move {
        match result {
          Err(e) => Some(Err(e)),
          Ok(gsa) => gsa
            .status_list
            .into_iter()
            .find(|gs| gs.goal_info.goal_id == goal_id)
            .map(Ok),
        }
      })
  }
} // impl

// Example topic names and types at DDS level:

// rq/turtle1/rotate_absolute/_action/send_goalRequest :
// turtlesim::action::dds_::RotateAbsolute_SendGoal_Request_ rr/turtle1/
// rotate_absolute/_action/send_goalReply :
// turtlesim::action::dds_::RotateAbsolute_SendGoal_Response_

// rq/turtle1/rotate_absolute/_action/cancel_goalRequest  :
// action_msgs::srv::dds_::CancelGoal_Request_ rr/turtle1/rotate_absolute/
// _action/cancel_goalReply  : action_msgs::srv::dds_::CancelGoal_Response_

// rq/turtle1/rotate_absolute/_action/get_resultRequest :
// turtlesim::action::dds_::RotateAbsolute_GetResult_Request_ rr/turtle1/
// rotate_absolute/_action/get_resultReply :
// turtlesim::action::dds_::RotateAbsolute_GetResult_Response_

// rt/turtle1/rotate_absolute/_action/feedback :
// turtlesim::action::dds_::RotateAbsolute_FeedbackMessage_

// rt/turtle1/rotate_absolute/_action/status :
// action_msgs::msg::dds_::GoalStatusArray_

pub struct ActionServer<A>
where
  A: ActionTypes,
  A::GoalType: Message + Clone,
  A::ResultType: Message + Clone,
  A::FeedbackType: Message,
{
  pub(crate) my_goal_server: Server<AService<SendGoalRequest<A::GoalType>, SendGoalResponse>>,

  pub(crate) my_cancel_server:
    Server<AService<action_msgs::CancelGoalRequest, action_msgs::CancelGoalResponse>>,

  pub(crate) my_result_server: Server<AService<GetResultRequest, GetResultResponse<A::ResultType>>>,

  pub(crate) my_feedback_publisher: Publisher<FeedbackMessage<A::FeedbackType>>,

  pub(crate) my_status_publisher: Publisher<action_msgs::GoalStatusArray>,

  pub(crate) my_action_name: String,
}

impl<A> ActionServer<A>
where
  A: ActionTypes,
  A::GoalType: Message + Clone,
  A::ResultType: Message + Clone,
  A::FeedbackType: Message,
{
  pub fn name(&self) -> &str {
    &self.my_action_name
  }

  pub fn goal_server(
    &mut self,
  ) -> &mut Server<AService<SendGoalRequest<A::GoalType>, SendGoalResponse>> {
    &mut self.my_goal_server
  }
  pub fn cancel_server(
    &mut self,
  ) -> &mut Server<AService<action_msgs::CancelGoalRequest, action_msgs::CancelGoalResponse>> {
    &mut self.my_cancel_server
  }
  pub fn result_server(
    &mut self,
  ) -> &mut Server<AService<GetResultRequest, GetResultResponse<A::ResultType>>> {
    &mut self.my_result_server
  }
  pub fn feedback_publisher(&mut self) -> &mut Publisher<FeedbackMessage<A::FeedbackType>> {
    &mut self.my_feedback_publisher
  }
  pub fn my_status_publisher(&mut self) -> &mut Publisher<action_msgs::GoalStatusArray> {
    &mut self.my_status_publisher
  }

  /// Receive a new goal, if available.
  pub fn receive_goal(&self) -> dds::Result<Option<(RmwRequestId, SendGoalRequest<A::GoalType>)>>
  where
    <A as ActionTypes>::GoalType: 'static,
  {
    self.my_goal_server.receive_request()
  }

  /// Send a response for the specified goal request
  pub fn send_goal_response(&self, req_id: RmwRequestId, resp: SendGoalResponse) -> dds::Result<()>
  where
    <A as ActionTypes>::GoalType: 'static,
  {
    self.my_goal_server.send_response(req_id, resp)
  }

  /// Receive a cancel request, if available.
  pub fn receive_cancel_request(
    &self,
  ) -> dds::Result<Option<(RmwRequestId, action_msgs::CancelGoalRequest)>> {
    self.my_cancel_server.receive_request()
  }

  // Respond to a received cancel request
  pub fn send_cancel_response(
    &self,
    req_id: RmwRequestId,
    resp: action_msgs::CancelGoalResponse,
  ) -> dds::Result<()> {
    self.my_cancel_server.send_response(req_id, resp)
  }

  pub fn receive_result_request(&self) -> dds::Result<Option<(RmwRequestId, GetResultRequest)>>
  where
    <A as ActionTypes>::ResultType: 'static,
  {
    self.my_result_server.receive_request()
  }

  pub fn send_result(
    &self,
    result_request_id: RmwRequestId,
    resp: GetResultResponse<A::ResultType>,
  ) -> dds::Result<()>
  where
    <A as ActionTypes>::ResultType: 'static,
  {
    self.my_result_server.send_response(result_request_id, resp)
  }

  pub fn send_feedback(&self, goal_id: GoalId, feedback: A::FeedbackType) -> dds::Result<()> {
    self
      .my_feedback_publisher
      .publish(FeedbackMessage { goal_id, feedback })
  }

  // Send the status of all known goals.
  pub fn send_goal_statuses(&self, goal_statuses: action_msgs::GoalStatusArray) -> dds::Result<()> {
    self.my_status_publisher.publish(goal_statuses)
  }
} // impl

#[derive(Clone, Copy)]
pub struct NewGoalHandle<G> {
  inner: InnerGoalHandle<G>,
  req_id: RmwRequestId,
}

impl<G> NewGoalHandle<G> {
  pub fn goal_id(&self) -> GoalId {
    self.inner.goal_id
  }
}

#[derive(Clone, Copy)]
pub struct AcceptedGoalHandle<G> {
  inner: InnerGoalHandle<G>,
}

impl<G> AcceptedGoalHandle<G> {
  pub fn goal_id(&self) -> GoalId {
    self.inner.goal_id
  }
}

#[derive(Clone, Copy)]
pub struct ExecutingGoalHandle<G> {
  inner: InnerGoalHandle<G>,
}

impl<G> ExecutingGoalHandle<G> {
  pub fn goal_id(&self) -> GoalId {
    self.inner.goal_id
  }
}

#[derive(Clone, Copy)]
struct InnerGoalHandle<G> {
  goal_id: GoalId,
  phantom: PhantomData<G>,
}

pub struct CancelHandle {
  req_id: RmwRequestId,
  goals: Vec<GoalId>,
}

impl CancelHandle {
  pub fn goals(&self) -> impl Iterator<Item = GoalId> + '_ {
    self.goals.iter().cloned()
  }
  pub fn contains_goal(&self, goal_id: &GoalId) -> bool {
    self.goals.contains(goal_id)
  }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GoalEndStatus {
  Succeeded,
  Aborted,
  Canceled,
}

#[derive(Debug)]
pub enum GoalError {
  NoSuchGoal,
  WrongGoalState,
  DDSError(dds::Error),
}

impl From<dds::Error> for GoalError {
  fn from(e: dds::Error) -> Self {
    GoalError::DDSError(e)
  }
}

#[derive(Debug, Clone)]
struct AsyncGoal<A>
where
  A: ActionTypes,
{
  status: GoalStatusEnum,
  accepted_time: Option<builtin_interfaces::Time>,
  goal: A::GoalType,
}

pub struct AsyncActionServer<A>
where
  A: ActionTypes,
  A::GoalType: Message + Clone,
  A::ResultType: Message + Clone,
  A::FeedbackType: Message,
{
  actionserver: ActionServer<A>,
  goals: BTreeMap<GoalId, AsyncGoal<A>>,
  result_requests: BTreeMap<GoalId, RmwRequestId>,
}

impl<A> AsyncActionServer<A>
where
  A: ActionTypes,
  A::GoalType: Message + Clone,
  A::ResultType: Message + Clone,
  A::FeedbackType: Message,
{
  pub fn new(actionserver: ActionServer<A>) -> Self {
    AsyncActionServer::<A> {
      actionserver,
      goals: BTreeMap::new(),
      result_requests: BTreeMap::new(),
    }
  }

  pub fn get_new_goal(&self, handle: NewGoalHandle<A::GoalType>) -> Option<&A::GoalType> {
    self.goals.get(&handle.inner.goal_id).map(|ag| &ag.goal)
  }

  /// Reveice a new goal from an action client.
  /// Server should immediately either accept or reject the goal.
  pub async fn receive_new_goal(&mut self) -> dds::Result<NewGoalHandle<A::GoalType>>
  where
    <A as ActionTypes>::GoalType: 'static,
  {
    let (req_id, goal_id) = loop {
      let (req_id, goal_request) = self
        .actionserver
        .my_goal_server
        .async_receive_request()
        .await?;
      match self.goals.entry(goal_request.goal_id) {
        e @ Entry::Vacant(_) => {
          e.or_insert(AsyncGoal {
            status: GoalStatusEnum::Unknown,
            goal: goal_request.goal,
            accepted_time: None,
          });
          break (req_id, goal_request.goal_id);
        }
        Entry::Occupied(_) => {
          error!(
            "Received duplicate goal_id {:?} , req_id={:?}",
            goal_request.goal_id, req_id
          );
          continue; // just discard this request
        }
      }
    };
    let inner = InnerGoalHandle {
      goal_id,
      phantom: PhantomData,
    };
    Ok(NewGoalHandle { inner, req_id })
  }

  /// Convert a newly received goal into a accepted goal, i.e. accept it
  /// for execution later. Client will be notified of acceptance.
  /// Note: Once the goal is accepted, the server must eventually call
  /// `.send_result_response()` even if the goal is canceled or aborted.
  pub async fn accept_goal(
    &mut self,
    handle: NewGoalHandle<A::GoalType>,
  ) -> Result<AcceptedGoalHandle<A::GoalType>, GoalError>
  where
    A::GoalType: 'static,
  {
    match self.goals.entry(handle.inner.goal_id) {
      Entry::Vacant(_) => Err(GoalError::NoSuchGoal),
      Entry::Occupied(o) => match o.get() {
        AsyncGoal {
          status: GoalStatusEnum::Unknown,
          ..
        } => {
          let now = builtin_interfaces::Time::now();
          let mut_o = o.into_mut();
          mut_o.status = GoalStatusEnum::Accepted;
          mut_o.accepted_time = Some(now);
          self.publish_statuses().await;
          self.actionserver.my_goal_server.send_response(
            handle.req_id,
            SendGoalResponse {
              accepted: true,
              stamp: now,
            },
          )?;
          Ok(AcceptedGoalHandle {
            inner: handle.inner,
          })
        }
        AsyncGoal {
          status: wrong_status,
          ..
        } => {
          error!(
            "Tried to accept goal {:?} but status was {:?}, expected Unknown.",
            handle.inner.goal_id, wrong_status
          );
          Err(GoalError::WrongGoalState)
        }
      },
    }
  }

  /// Reject a received goal. Client will be notified of rejection.
  /// Server should not process the goal further.
  pub async fn reject_goal(&mut self, handle: NewGoalHandle<A::GoalType>) -> Result<(), GoalError>
  where
    A::GoalType: 'static,
  {
    match self.goals.entry(handle.inner.goal_id) {
      Entry::Vacant(_) => Err(GoalError::NoSuchGoal),
      Entry::Occupied(o) => {
        match o.get() {
          AsyncGoal {
            status: GoalStatusEnum::Unknown,
            ..
          } => {
            self.actionserver.my_goal_server.send_response(
              handle.req_id,
              SendGoalResponse {
                accepted: false,
                stamp: builtin_interfaces::Time::now(),
              },
            )?;
            //o.into_mut().0 = GoalStatusEnum::Rejected; -- there is no such state
            //self.publish_statuses().await; -- this is not reported
            Ok(())
          }
          AsyncGoal {
            status: wrong_status,
            ..
          } => {
            error!(
              "Tried to reject goal {:?} but status was {:?}, expected Unknown.",
              handle.inner.goal_id, wrong_status
            );
            Err(GoalError::WrongGoalState)
          }
        }
      }
    }
  }

  /// Convert an accepted goal into a execting goal, i.e. start the execution.
  /// Executing goal can publish feedback.
  pub async fn start_executing_goal(
    &mut self,
    handle: AcceptedGoalHandle<A::GoalType>,
  ) -> Result<ExecutingGoalHandle<A::GoalType>, GoalError> {
    match self.goals.entry(handle.inner.goal_id) {
      Entry::Vacant(_) => Err(GoalError::NoSuchGoal),
      Entry::Occupied(o) => match o.get() {
        AsyncGoal {
          status: GoalStatusEnum::Accepted,
          ..
        } => {
          o.into_mut().status = GoalStatusEnum::Executing;
          self.publish_statuses().await;
          Ok(ExecutingGoalHandle {
            inner: handle.inner,
          })
        }
        AsyncGoal {
          status: wrong_status,
          ..
        } => {
          error!(
            "Tried to execute goal {:?} but status was {:?}, expected Accepted.",
            handle.inner.goal_id, wrong_status
          );
          Err(GoalError::WrongGoalState)
        }
      },
    }
  }

  /// Publish feedback on how the execution is proceeding.
  pub async fn publish_feedback(
    &mut self,
    handle: ExecutingGoalHandle<A::GoalType>,
    feedback: A::FeedbackType,
  ) -> Result<(), GoalError> {
    match self.goals.entry(handle.inner.goal_id) {
      Entry::Vacant(_) => Err(GoalError::NoSuchGoal),
      Entry::Occupied(o) => match o.get() {
        AsyncGoal {
          status: GoalStatusEnum::Executing,
          ..
        } => {
          self
            .actionserver
            .send_feedback(handle.inner.goal_id, feedback)?;
          Ok(())
        }
        AsyncGoal {
          status: wrong_status,
          ..
        } => {
          error!(
            "Tried publish feedback on goal {:?} but status was {:?}, expected Executing.",
            handle.inner.goal_id, wrong_status
          );
          Err(GoalError::WrongGoalState)
        }
      },
    }
  }

  /// Notify Client that a goal end state was reached and
  /// what was the result of the action.
  /// This async will not resolve until the action client has requested for the
  /// result, but the client should request the result as soon as server
  /// accepts the goal.
  // TODO: It is a bit silly that we have to supply a "result" even though
  // goal got canceled. But we have to send something in the ResultResponse.
  // And where does it say that result is not significant if cancelled or aborted?
  pub async fn send_result_response(
    &mut self,
    handle: ExecutingGoalHandle<A::GoalType>,
    result_status: GoalEndStatus,
    result: A::ResultType,
  ) -> Result<(), GoalError>
  where
    A::ResultType: 'static,
  {
    // We translate from interface type to internal type to ensure that
    // the end status is an end status and not e.g. "Accepted".
    let result_status = match result_status {
      GoalEndStatus::Succeeded => GoalStatusEnum::Succeeded,
      GoalEndStatus::Aborted => GoalStatusEnum::Aborted,
      GoalEndStatus::Canceled => GoalStatusEnum::Canceled,
    };

    // First, we must get a result request.
    // It may already have been read or not.
    // We will read these into a buffer, because there may be requests for
    // other goals' results also.
    let req_id = match self.result_requests.get(&handle.inner.goal_id) {
      Some(req_id) => *req_id,
      None => {
        let res_reqs = self.actionserver.my_result_server.receive_request_stream();
        pin_mut!(res_reqs);
        loop {
          // result request was not yet here. Keep receiving until we get it.
          let (req_id, GetResultRequest { goal_id }) = res_reqs.select_next_some().await?;
          if goal_id == handle.inner.goal_id {
            break req_id;
          } else {
            self.result_requests.insert(goal_id, req_id);
            debug!(
              "Got result request for goal_id={:?} req_id={:?}",
              goal_id, req_id
            );
            // and loop to wait for the next
          }
        }
      }
    };

    match self.goals.entry(handle.inner.goal_id) {
      Entry::Vacant(_) => Err(GoalError::NoSuchGoal),
      Entry::Occupied(o) => {
        match o.get() {
          // Accepted, executing, or canceling goal can be canceled or aborted
          // TODO: Accepted goal cannot succeed, it must be executing before success.
          AsyncGoal {
            status: GoalStatusEnum::Accepted,
            ..
          }
          | AsyncGoal {
            status: GoalStatusEnum::Executing,
            ..
          }
          | AsyncGoal {
            status: GoalStatusEnum::Canceling,
            ..
          } => {
            o.into_mut().status = result_status;
            self.publish_statuses().await;
            self.actionserver.send_result(
              req_id,
              GetResultResponse {
                status: result_status,
                result,
              },
            )?;
            debug!(
              "Send result for goal_id={:?}  req_id={:?}",
              handle.inner.goal_id, req_id
            );
            Ok(())
          }
          AsyncGoal {
            status: wrong_status,
            ..
          } => {
            error!(
              "Tried to finish goal {:?} but status was {:?}.",
              handle.inner.goal_id, wrong_status
            );
            Err(GoalError::WrongGoalState)
          }
        }
      }
    }
  }

  /// Abort goal execution, because action server has determined it
  /// cannot continue execution.
  pub async fn abort_executing_goal(
    &mut self,
    handle: ExecutingGoalHandle<A::GoalType>,
  ) -> Result<(), GoalError> {
    self.abort_goal(handle.inner).await
  }
  pub async fn abort_accepted_goal(
    &mut self,
    handle: AcceptedGoalHandle<A::GoalType>,
  ) -> Result<(), GoalError> {
    self.abort_goal(handle.inner).await
  }

  async fn abort_goal(&mut self, handle: InnerGoalHandle<A::GoalType>) -> Result<(), GoalError> {
    match self.goals.entry(handle.goal_id) {
      Entry::Vacant(_) => Err(GoalError::NoSuchGoal),
      Entry::Occupied(o) => match o.get() {
        AsyncGoal {
          status: GoalStatusEnum::Accepted,
          ..
        }
        | AsyncGoal {
          status: GoalStatusEnum::Executing,
          ..
        } => {
          o.into_mut().status = GoalStatusEnum::Aborted;
          self.publish_statuses().await;
          Ok(())
        }
        AsyncGoal {
          status: wrong_status,
          ..
        } => {
          error!(
            "Tried to abort goal {:?} but status was {:?}, expected Accepted or Executing. ",
            handle.goal_id, wrong_status
          );
          Err(GoalError::WrongGoalState)
        }
      },
    }
  }

  /// Receive a set of cancel requests from the action client.
  /// The server should now respond either by accepting (some of) the
  /// cancel requests or rejecting all of them. The GoalIds that are requested
  /// to be cancelled can be currently at either accepted or executing state.
  pub async fn receive_cancel_request(&self) -> dds::Result<CancelHandle> {
    let (req_id, CancelGoalRequest { goal_info }) = self
      .actionserver
      .my_cancel_server
      .async_receive_request()
      .await?;

    #[allow(clippy::type_complexity)] // How would you refactor this type?
    let goal_filter: Box<dyn FnMut(&(&GoalId, &AsyncGoal<A>)) -> bool> = match goal_info {
      GoalInfo {
        goal_id: GoalId::ZERO,
        stamp: builtin_interfaces::Time::ZERO,
      } => Box::new(|(_, _)| true), // cancel all goals

      GoalInfo {
        goal_id: GoalId::ZERO,
        stamp,
      } => Box::new(move |(_, ag)| ag.accepted_time.map(|at| at < stamp).unwrap_or(false)),

      GoalInfo {
        goal_id,
        stamp: builtin_interfaces::Time::ZERO,
      } => Box::new(move |(g_id, _)| goal_id == **g_id),

      GoalInfo { goal_id, stamp } => Box::new(move |(g_id, ag)| {
        goal_id == **g_id || ag.accepted_time.map(move |at| at < stamp).unwrap_or(false)
      }),
    };

    // TODO:
    // Should check if the specified GoalId was unknown to us
    // or already terminated.
    // In those case outright send a negative response and not return to the
    // application.
    let cancel_handle = CancelHandle {
      req_id,
      goals: self
        .goals
        .iter()
        // only consider goals with status Executing or Accepted for Cancel
        .filter(|(_, async_goal)| {
          async_goal.status == GoalStatusEnum::Executing
            || async_goal.status == GoalStatusEnum::Accepted
        })
        // and then filter those that were specified by the cancel request
        .filter(goal_filter)
        .map(|p| *p.0)
        .collect(),
    };

    Ok(cancel_handle)
  }

  /// Respond to action client's cancel requests.
  /// The iterator of goals should list those GoalIds that will start canceling.
  /// For the other GoalIds, the cancel is not accepted and they do not change
  /// their state.
  pub async fn respond_to_cancel_requests(
    &mut self,
    cancel_handle: &CancelHandle,
    goals_to_cancel: impl Iterator<Item = GoalId>,
  ) -> dds::Result<()> {
    let canceling_goals: Vec<GoalInfo> = goals_to_cancel
      .filter_map(|goal_id| {
        self
          .goals
          .get(&goal_id)
          .and_then(|AsyncGoal { accepted_time, .. }| {
            accepted_time.map(|stamp| GoalInfo { goal_id, stamp })
          })
      })
      .collect();

    for goal_info in &canceling_goals {
      self
        .goals
        .entry(goal_info.goal_id)
        .and_modify(|gg| gg.status = GoalStatusEnum::Canceling);
    }
    self.publish_statuses().await;

    let response = action_msgs::CancelGoalResponse {
      return_code: if canceling_goals.is_empty() {
        action_msgs::CancelGoalResponseEnum::Rejected
      } else {
        action_msgs::CancelGoalResponseEnum::None // i.e. no error
      },
      goals_canceling: canceling_goals,
    };

    self
      .actionserver
      .my_cancel_server
      .async_send_response(cancel_handle.req_id, response)
      .await
  }

  // This function is private, because all status publishing happens automatically
  // via goal startus changes.
  async fn publish_statuses(&self) {
    let goal_status_array = action_msgs::GoalStatusArray {
      status_list: self
        .goals
        .iter()
        .map(
          |(
            goal_id,
            AsyncGoal {
              status,
              accepted_time,
              ..
            },
          )| action_msgs::GoalStatus {
            status: *status,
            goal_info: GoalInfo {
              goal_id: *goal_id,
              stamp: accepted_time.unwrap_or(builtin_interfaces::Time::ZERO),
            },
          },
        )
        .collect(),
    };
    debug!(
      "Reporting statuses for {:?}",
      goal_status_array
        .status_list
        .iter()
        .map(|gs| gs.goal_info.goal_id)
    );
    self
      .actionserver
      .send_goal_statuses(goal_status_array)
      .unwrap_or_else(|e| error!("AsyncActionServer::publish_statuses: {:?}", e));
  }
}
