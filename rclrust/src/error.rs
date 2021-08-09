use std::fmt;

use anyhow::Result;

use crate::internal::ffi::*;

#[derive(Debug)]
pub struct RclErrorBase {
    message: String,
    file: String,
    line: u64,
}

impl RclErrorBase {
    unsafe fn new(error_state_ptr: *const rcl_sys::rcutils_error_state_t) -> Self {
        let error_state = error_state_ptr.as_ref().unwrap();
        Self {
            message: String::from_c_char(error_state.message.as_ptr()).unwrap(),
            file: String::from_c_char(error_state.file.as_ptr()).unwrap(),
            line: error_state.line_number,
        }
    }
}

impl fmt::Display for RclErrorBase {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "[rclrust]: {}, at {}:{}",
            self.message, self.file, self.line
        )
    }
}

#[derive(Debug, thiserror::Error)]
pub enum RclRustError {
    #[error("Unspecified error.\n{0}")]
    RclError(RclErrorBase),
    #[error("Timeout occurred.\n{0}")]
    RclTimeout(RclErrorBase),
    #[error("Failed to allocate memory.\n{0}")]
    RclBadAlloc(RclErrorBase),
    #[error("Invalid argument.\n{0}")]
    RclInvalidArgument(RclErrorBase),
    #[error("Unsupported.\n{0}")]
    RclUnsupported(RclErrorBase),

    // rcl specific ret codes start at 100
    #[error("rcl_init() already called.\n{0}")]
    RclAlreadyInit(RclErrorBase),
    #[error("rcl_init() not yet called.\n{0}")]
    RclNotInit(RclErrorBase),
    #[error("Mismatched rmw identifier.\n{0}")]
    RclMismatchedRmwId(RclErrorBase),
    #[error("Topic name does not pass validation.\n{0}")]
    RclTopicNameInvalid(RclErrorBase),
    #[error("Service name (same as topic name) does not pass validation.\n{0}")]
    RclServiceNameInvalid(RclErrorBase),
    #[error("Topic name substitution is unknown.\n{0}")]
    RclUnknownSubstitution(RclErrorBase),
    #[error("rcl_shutdown() already called.\n{0}")]
    RclAlreadyShutdown(RclErrorBase),

    // rcl node specific ret codes in 2XX
    #[error("Invalid rcl_node_t given.\n{0}")]
    RclNodeInvalid(RclErrorBase),
    #[error("Invalid node name.\n{0}")]
    RclNodeInvalidName(RclErrorBase),
    #[error("Invalid node namespace.\n{0}")]
    RclNodeInvalidNamespace(RclErrorBase),
    #[error("Failed to find node name.\n{0}")]
    RclNodeNameNonExistent(RclErrorBase),

    // rcl publisher specific ret codes in 3XX
    #[error("Invalid rcl_publisher_t given.\n{0}")]
    RclPublisherInvalid(RclErrorBase),

    // rcl subscription specific ret codes in 4XX
    #[error("Invalid rcl_subscription_t given.\n{0}")]
    RclSubscriptionInvalid(RclErrorBase),
    #[error("Failed to take a message from the subscription.\n{0}")]
    RclSubscriptionTakeFailed(RclErrorBase),

    // rcl service client specific ret codes in 5XX
    #[error("Invalid rcl_client_t given.\n{0}")]
    RclClientInvalid(RclErrorBase),
    #[error("Failed to take a response from the client.\n{0}")]
    RclClientTakeFailed(RclErrorBase),

    // rcl service server specific ret codes in 6XX
    #[error("Invalid rcl_service_t given.\n{0}")]
    RclServiceInvalid(RclErrorBase),
    #[error("Failed to take a request from the service.\n{0}")]
    RclServiceTakeFailed(RclErrorBase),

    // rcl guard condition specific ret codes in 7XX

    // rcl timer specific ret codes in 8XX
    #[error("Invalid rcl_timer_t given.\n{0}")]
    RclTimerInvalid(RclErrorBase),
    #[error("Given timer was canceled.\n{0}")]
    RclTimerCanceled(RclErrorBase),

    // rcl wait and wait set specific ret codes in 9XX
    #[error("Invalid rcl_wait_set_t given.\n{0}")]
    RclWaitSetInvalid(RclErrorBase),
    #[error("Given rcl_wait_set_t is empty.\n{0}")]
    RclWaitSetEmpty(RclErrorBase),
    #[error("Given rcl_wait_set_t is full.\n{0}")]
    RclWaitSetFull(RclErrorBase),

    // rcl argument parsing specific ret codes in 1XXX
    #[error("Argument is not a valid remap rule.\n{0}")]
    RclInvalidRemapRule(RclErrorBase),
    #[error("Expected one type of lexeme but got another.\n{0}")]
    RclWrongLexeme(RclErrorBase),
    #[error("Found invalid ros argument while parsing.\n{0}")]
    RclInvalidROSArgs(RclErrorBase),
    #[error("Argument is not a valid parameter rule.\n{0}")]
    RclInvalidParamRule(RclErrorBase),
    #[error("Argument is not a valid log level rule.\n{0}")]
    RclInvalidLogLevelRule(RclErrorBase),

    // rcl event specific ret codes in 20XX
    #[error("Invalid rcl_event_t given.\n{0}")]
    RclEventInvalid(RclErrorBase),
    #[error("Failed to take an event from the event handle.\n{0}")]
    RclEventTakeFailed(RclErrorBase),

    // rcl_lifecycle state register ret codes in 30XX
    #[error("rcl_lifecycle state registered.\n{0}")]
    RclLifecycleStateRegistered(RclErrorBase),
    #[error("rcl_lifecycle state not registered.\n{0}")]
    RclLifecycleStateNotRegistered(RclErrorBase),

    #[error("Runtime Error: {0}")]
    RuntimeError(&'static str),
    #[error("Service is canceled.")]
    ServiceIsCanceled,
}

pub(crate) fn result_from_rcl_ret(ret: rcl_sys::rcl_ret_t) -> Result<()> {
    if ret as u32 == rcl_sys::RCL_RET_OK {
        return Ok(());
    }
    let error_state = unsafe { rcl_sys::rcutils_get_error_state() };
    if error_state.is_null() {
        return Err(RclRustError::RuntimeError("rcl error state is not set").into());
    }
    let base_error = unsafe { RclErrorBase::new(error_state) };
    unsafe { rcl_sys::rcutils_reset_error() }

    let error = {
        use rcl_sys::*;
        match ret as u32 {
            RCL_RET_TIMEOUT => RclRustError::RclTimeout(base_error),
            RCL_RET_BAD_ALLOC => RclRustError::RclBadAlloc(base_error),
            RCL_RET_INVALID_ARGUMENT => RclRustError::RclInvalidArgument(base_error),
            RCL_RET_UNSUPPORTED => RclRustError::RclUnsupported(base_error),

            RCL_RET_ALREADY_INIT => RclRustError::RclAlreadyInit(base_error),
            RCL_RET_NOT_INIT => RclRustError::RclNotInit(base_error),
            RCL_RET_MISMATCHED_RMW_ID => RclRustError::RclMismatchedRmwId(base_error),
            RCL_RET_TOPIC_NAME_INVALID => RclRustError::RclTopicNameInvalid(base_error),
            RCL_RET_SERVICE_NAME_INVALID => RclRustError::RclServiceNameInvalid(base_error),
            RCL_RET_UNKNOWN_SUBSTITUTION => RclRustError::RclUnknownSubstitution(base_error),
            RCL_RET_ALREADY_SHUTDOWN => RclRustError::RclAlreadyShutdown(base_error),

            RCL_RET_NODE_INVALID => RclRustError::RclNodeInvalid(base_error),
            RCL_RET_NODE_INVALID_NAME => RclRustError::RclNodeInvalidName(base_error),
            RCL_RET_NODE_INVALID_NAMESPACE => RclRustError::RclNodeInvalidNamespace(base_error),
            RCL_RET_NODE_NAME_NON_EXISTENT => RclRustError::RclNodeNameNonExistent(base_error),

            RCL_RET_PUBLISHER_INVALID => RclRustError::RclPublisherInvalid(base_error),

            RCL_RET_SUBSCRIPTION_INVALID => RclRustError::RclSubscriptionInvalid(base_error),
            RCL_RET_SUBSCRIPTION_TAKE_FAILED => RclRustError::RclSubscriptionTakeFailed(base_error),

            RCL_RET_SERVICE_INVALID => RclRustError::RclServiceNameInvalid(base_error),
            RCL_RET_SERVICE_TAKE_FAILED => RclRustError::RclServiceTakeFailed(base_error),

            RCL_RET_TIMER_INVALID => RclRustError::RclTimerInvalid(base_error),
            RCL_RET_TIMER_CANCELED => RclRustError::RclTimerCanceled(base_error),

            RCL_RET_WAIT_SET_INVALID => RclRustError::RclWaitSetInvalid(base_error),
            RCL_RET_WAIT_SET_EMPTY => RclRustError::RclWaitSetEmpty(base_error),
            RCL_RET_WAIT_SET_FULL => RclRustError::RclWaitSetFull(base_error),

            RCL_RET_INVALID_REMAP_RULE => RclRustError::RclInvalidRemapRule(base_error),
            RCL_RET_WRONG_LEXEME => RclRustError::RclWrongLexeme(base_error),
            RCL_RET_INVALID_ROS_ARGS => RclRustError::RclInvalidROSArgs(base_error),
            RCL_RET_INVALID_PARAM_RULE => RclRustError::RclInvalidParamRule(base_error),
            RCL_RET_INVALID_LOG_LEVEL_RULE => RclRustError::RclInvalidLogLevelRule(base_error),

            RCL_RET_EVENT_INVALID => RclRustError::RclEventInvalid(base_error),
            RCL_RET_EVENT_TAKE_FAILED => RclRustError::RclEventTakeFailed(base_error),

            _ => RclRustError::RclError(base_error),
        }
    };
    Err(error.into())
}

pub(crate) trait ToRclRustResult {
    fn to_result(self) -> Result<()>;
}

impl ToRclRustResult for rcl_sys::rcl_ret_t {
    fn to_result(self) -> Result<()> {
        result_from_rcl_ret(self)
    }
}
