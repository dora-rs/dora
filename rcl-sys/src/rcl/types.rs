//! API in rcl/types.h

use crate::*;

/// The type that holds an rcl return code.
pub type rcl_ret_t = rmw_ret_t;

/// Success return code.
pub const RCL_RET_OK: u32 = 0;
/// Unspecified error return code.
pub const RCL_RET_ERROR: u32 = 1;
/// Timeout occurred return code.
pub const RCL_RET_TIMEOUT: u32 = 2;
/// Failed to allocate memory return code.
pub const RCL_RET_BAD_ALLOC: u32 = 10;
/// Invalid argument return code.
pub const RCL_RET_INVALID_ARGUMENT: u32 = 11;
/// Unsupported return code.
pub const RCL_RET_UNSUPPORTED: u32 = 3;

// rcl specific ret codes start at 100
/// [rcl_init()] already called return code.
pub const RCL_RET_ALREADY_INIT: u32 = 100;
/// [rcl_init()] not yet called return code.
pub const RCL_RET_NOT_INIT: u32 = 101;
/// Mismatched rmw identifier return code.
pub const RCL_RET_MISMATCHED_RMW_ID: u32 = 102;
/// Topic name does not pass validation.
pub const RCL_RET_TOPIC_NAME_INVALID: u32 = 103;
/// Service name (same as topic name) does not pass validation.
pub const RCL_RET_SERVICE_NAME_INVALID: u32 = 104;
/// Topic name substitution is unknown.
pub const RCL_RET_UNKNOWN_SUBSTITUTION: u32 = 105;
/// [rcl_shutdown()] already called return code.
pub const RCL_RET_ALREADY_SHUTDOWN: u32 = 106;

// rcl node specific ret codes in 2XX
/// Invalid [rcl_node_t] given return code.
pub const RCL_RET_NODE_INVALID: u32 = 200;
/// Invalid node name return code.
pub const RCL_RET_NODE_INVALID_NAME: u32 = 201;
/// Invalid node namespace return code.
pub const RCL_RET_NODE_INVALID_NAMESPACE: u32 = 202;
/// Failed to find node name
pub const RCL_RET_NODE_NAME_NON_EXISTENT: u32 = 203;

// rcl publisher specific ret codes in 3XX
/// Invalid [rcl_publisher_t] given return code.
pub const RCL_RET_PUBLISHER_INVALID: u32 = 300;

// rcl subscription specific ret codes in 4XX
/// Invalid [rcl_subscription_t] given return code.
pub const RCL_RET_SUBSCRIPTION_INVALID: u32 = 400;
/// Failed to take a message from the subscription return code.
pub const RCL_RET_SUBSCRIPTION_TAKE_FAILED: u32 = 401;

// rcl service client specific ret codes in 5XX
/// Invalid [rcl_client_t] given return code.
pub const RCL_RET_CLIENT_INVALID: u32 = 500;
/// Failed to take a response from the client return code.
pub const RCL_RET_CLIENT_TAKE_FAILED: u32 = 501;

// rcl service server specific ret codes in 6XX
/// Invalid [rcl_service_t] given return code.
pub const RCL_RET_SERVICE_INVALID: u32 = 600;
/// Failed to take a request from the service return code.
pub const RCL_RET_SERVICE_TAKE_FAILED: u32 = 601;

// rcl guard condition specific ret codes in 7XX

// rcl timer specific ret codes in 8XX
/// Invalid [rcl_timer_t] given return code.
pub const RCL_RET_TIMER_INVALID: u32 = 800;
/// Given timer was canceled return code.
pub const RCL_RET_TIMER_CANCELED: u32 = 801;

// rcl wait and wait set specific ret codes in 9XX
/// Invalid [rcl_wait_set_t] given return code.
pub const RCL_RET_WAIT_SET_INVALID: u32 = 900;
/// Given rcl_wait_set_t is empty return code.
pub const RCL_RET_WAIT_SET_EMPTY: u32 = 901;
/// Given rcl_wait_set_t is full return code.
pub const RCL_RET_WAIT_SET_FULL: u32 = 902;

// rcl argument parsing specific ret codes in 1XXX
/// Argument is not a valid remap rule
pub const RCL_RET_INVALID_REMAP_RULE: u32 = 1001;
/// Expected one type of lexeme but got another
pub const RCL_RET_WRONG_LEXEME: u32 = 1002;
/// Found invalid ros argument while parsing
pub const RCL_RET_INVALID_ROS_ARGS: u32 = 1003;
/// Argument is not a valid parameter rule
pub const RCL_RET_INVALID_PARAM_RULE: u32 = 1010;
/// Argument is not a valid log level rule
pub const RCL_RET_INVALID_LOG_LEVEL_RULE: u32 = 1020;

// rcl event specific ret codes in 20XX
/// Invalid [rcl_event_t] given return code.
pub const RCL_RET_EVENT_INVALID: u32 = 2000;
/// Failed to take an event from the event handle
pub const RCL_RET_EVENT_TAKE_FAILED: u32 = 2001;

// rcl_lifecycle state register ret codes in 30XX
/// rcl_lifecycle state registered
#[cfg(feature = "galactic+")]
pub const RCL_RET_LIFECYCLE_STATE_REGISTERED: u32 = 3000;
/// rcl_lifecycle state not registered
#[cfg(feature = "galactic+")]
pub const RCL_RET_LIFECYCLE_STATE_NOT_REGISTERED: u32 = 3001;

/// typedef for [rmw_serialized_message_t];
pub type rcl_serialized_message_t = rmw_serialized_message_t;
