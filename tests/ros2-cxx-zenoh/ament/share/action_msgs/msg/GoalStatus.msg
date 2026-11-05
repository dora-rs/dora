# An action goal can be in one of these states after it is accepted by an action
# server.
#
# For more information, see http://design.ros2.org/articles/actions.html

# Indicates status has not been properly set.
int8 STATUS_UNKNOWN   = 0

# The goal has been accepted and is awaiting execution.
int8 STATUS_ACCEPTED  = 1

# The goal is currently being executed by the action server.
int8 STATUS_EXECUTING = 2

# The client has requested that the goal be canceled and the action server has
# accepted the cancel request.
int8 STATUS_CANCELING = 3

# The goal was achieved successfully by the action server.
int8 STATUS_SUCCEEDED = 4

# The goal was canceled after an external request from an action client.
int8 STATUS_CANCELED  = 5

# The goal was terminated by the action server without an external request.
int8 STATUS_ABORTED   = 6

# Goal info (contains ID and timestamp).
GoalInfo goal_info

# Action goal state-machine status.
int8 status
