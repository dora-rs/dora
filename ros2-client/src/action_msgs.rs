use serde::{Deserialize, Serialize};
use serde_repr::{Deserialize_repr, Serialize_repr};

use crate::message::Message;

pub type GoalId = crate::unique_identifier_msgs::UUID;

/// From [GoalInfo](https://docs.ros2.org/foxy/api/action_msgs/msg/GoalInfo.html)
#[derive(Clone, Serialize, Deserialize, Debug)]
pub struct GoalInfo {
  pub goal_id: GoalId,
  pub stamp: crate::builtin_interfaces::Time, // Time when the goal was accepted
}
impl Message for GoalInfo {}

#[derive(Clone, Copy, Serialize_repr, Deserialize_repr, PartialEq, Debug)]
#[repr(i8)]
pub enum GoalStatusEnum {
  Unknown = 0, // Let's use this also for "New"
  Accepted = 1,
  Executing = 2,
  Canceling = 3,
  Succeeded = 4,
  Canceled = 5,
  Aborted = 6,
}

/// From [GoalStatus](https://docs.ros2.org/foxy/api/action_msgs/msg/GoalStatus.html)
#[derive(Clone, Serialize, Deserialize, Debug)]
pub struct GoalStatus {
  pub goal_info: GoalInfo,
  pub status: GoalStatusEnum,
}
impl Message for GoalStatus {}

/// From [GoalStatusArray](https://docs.ros2.org/foxy/api/action_msgs/msg/GoalStatusArray.html)
#[derive(Clone, Serialize, Deserialize, Debug)]
pub struct GoalStatusArray {
  pub status_list: Vec<GoalStatus>,
}
impl Message for GoalStatusArray {}

/// From [CancelGoal](https://docs.ros2.org/foxy/api/action_msgs/srv/CancelGoal.html)
// Cancel one or more goals with the following policy:
//
// - If the goal ID is zero and timestamp is zero, cancel all goals.
// - If the goal ID is zero and timestamp is not zero, cancel all goals accepted at or before the
//   timestamp.
// - If the goal ID is not zero and timestamp is zero, cancel the goal with the given ID regardless
//   of the time it was accepted.
// - If the goal ID is not zero and timestamp is not zero, cancel the goal with the given ID and all
//   goals accepted at or before the timestamp.
#[derive(Clone, Serialize, Deserialize, Debug)]
pub struct CancelGoalRequest {
  pub(crate) goal_info: GoalInfo,
}
impl Message for CancelGoalRequest {}

/// From [CancelGoal](https://docs.ros2.org/foxy/api/action_msgs/srv/CancelGoal.html)
#[derive(Clone, Copy, Serialize_repr, Deserialize_repr, PartialEq, Debug)]
#[repr(i8)]
pub enum CancelGoalResponseEnum {
  // Doc comments here copied from ROS2 message definition.
  /// Indicates the request was accepted without any errors.
  /// One or more goals have transitioned to the CANCELING state.
  /// The goals_canceling list is not empty.
  None = 0,

  /// Indicates the request was rejected.
  /// No goals have transitioned to the CANCELING state. The goals_canceling
  /// list is empty.
  Rejected = 1,

  /// Indicates the requested goal ID does not exist.
  /// No goals have transitioned to the CANCELING state. The goals_canceling
  /// list is empty.
  UnknownGoal = 2,

  /// Indicates the goal is not cancelable because it is already in a terminal
  /// state. No goals have transitioned to the CANCELING state. The
  /// goals_canceling list is empty.
  GoalTerminated = 3,
}

/// From [CancelGoal](https://docs.ros2.org/foxy/api/action_msgs/srv/CancelGoal.html)
#[derive(Clone, Serialize, Deserialize, Debug)]
pub struct CancelGoalResponse {
  pub return_code: CancelGoalResponseEnum,
  pub goals_canceling: Vec<GoalInfo>,
}
impl Message for CancelGoalResponse {}
