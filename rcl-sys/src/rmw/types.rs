//! API in rmw/types.h

use std::os::raw::{c_char, c_void};

use crate::rcutils::rcutils_time_point_value_t;

// 24 bytes is the most memory needed to represent the GID by any current
// implementation. It may need to be increased in the future.
pub const RMW_GID_STORAGE_SIZE: usize = 24;

#[repr(u32)]
/// Endpoint enumeration type
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub enum rmw_endpoint_type_t {
    /// Endpoint type has not yet been set
    RMW_ENDPOINT_INVALID = 0,
    /// Creates and publishes messages to the ROS topic
    RMW_ENDPOINT_PUBLISHER = 1,
    /// Listens for and receives messages from a topic
    RMW_ENDPOINT_SUBSCRIPTION = 2,
}

#[cfg(feature = "galactic+")]
#[repr(u32)]
/// Unique network flow endpoints requirement enumeration
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub enum rmw_unique_network_flow_endpoints_requirement_t {
    /// Unique network flow endpoints not required
    RMW_UNIQUE_NETWORK_FLOW_ENDPOINTS_NOT_REQUIRED = 0,

    /// Unique network flow endpoins strictly required.
    /// Error if not provided by RMW implementation.
    RMW_UNIQUE_NETWORK_FLOW_ENDPOINTS_STRICTLY_REQUIRED = 1,

    /// Unique network flow endpoints optionally required.
    /// No error if not provided RMW implementation.
    RMW_UNIQUE_NETWORK_FLOW_ENDPOINTS_OPTIONALLY_REQUIRED = 2,

    /// Unique network flow endpoints requirement decided by system.
    RMW_UNIQUE_NETWORK_FLOW_ENDPOINTS_SYSTEM_DEFAULT = 3,
}

/// Options that can be used to configure the creation of a publisher in rmw.
#[repr(C)]
#[derive(Debug)]
pub struct rmw_publisher_options_t {
    /// Used to pass rmw implementation specific resources during publisher creation.
    pub rmw_specific_publisher_payload: *mut c_void,
    #[cfg(feature = "galactic+")]
    pub require_unique_network_flow_endpoints: rmw_unique_network_flow_endpoints_requirement_t,
}

/// Options that can be used to configure the creation of a subscription in rmw.
#[repr(C)]
#[derive(Debug)]
pub struct rmw_subscription_options_t {
    /// Used to pass rmw implementation specific resources during subscription creation.
    pub rmw_specific_subscription_payload: *mut c_void,
    /// If true then the middleware should not deliver data from local publishers.
    pub ignore_local_publications: bool,
}

/// Allocation of memory for an rmw publisher
#[repr(C)]
#[derive(Debug)]
pub struct rmw_publisher_allocation_t {
    /// The name of the rmw implementation
    pub implementation_identifier: *const c_char,
    /// Type erased pointer to this allocation
    pub data: *mut c_void,
}

/// Allocation of memory for an rmw subscription
#[repr(C)]
#[derive(Debug)]
pub struct rmw_subscription_allocation_t {
    /// The name of the rmw implementation
    pub implementation_identifier: *const c_char,
    /// Type erased pointer to this allocation
    pub data: *mut c_void,
}

/// An rmw service request identifier
#[repr(C)]
#[derive(Debug)]
pub struct rmw_request_id_t {
    /// The guid of the writer associated with this request
    pub writer_guid: [i8; 16usize],
    /// Sequence number of this service
    pub sequence_number: i64,
}

/// Struct representing a time point for rmw
#[repr(C)]
#[derive(Debug)]
pub struct rmw_time_t {
    /// Seconds since the epoch
    pub sec: u64,
    /// Nanoseconds component of this time point
    pub nsec: u64,
}

pub type rmw_time_point_value_t = rcutils_time_point_value_t;

/// Meta-data for a service-related take.
#[repr(C)]
#[derive(Debug)]
pub struct rmw_service_info_t {
    pub source_timestamp: rmw_time_point_value_t,
    pub received_timestamp: rmw_time_point_value_t,
    pub request_id: rmw_request_id_t,
}

#[repr(u32)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub enum RMWQoSReliabilityPolicy {
    /// Implementation specific default
    SystemDefault = 0,
    /// Guarantee that samples are delivered, may retry multiple times.
    Reliable = 1,
    /// Attempt to deliver samples, but some may be lost if the network is not robust
    BestEffort = 2,
    /// Reliability policy has not yet been set
    Unknown = 3,
}

#[repr(u32)]
/// QoS history enumerations describing how samples endure
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub enum RMWQoSHistoryPolicy {
    /// Implementation default for history policy
    SystemDefault = 0,
    /// Only store up to a maximum number of samples, dropping oldest once max is exceeded
    KeepLast = 1,
    /// Store all samples, subject to resource limits
    KeepAll = 2,
    /// History policy has not yet been set
    Unknown = 3,
}

#[repr(u32)]
/// QoS durability enumerations describing how samples persist
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub enum RMWQoSDurabilityPolicy {
    /// Impplementation specific default
    SystemDefault = 0,
    /// The rmw publisher is responsible for persisting samples for “late-joining” subscribers
    TransientLocal = 1,
    /// Samples are not persistent
    Volatile = 2,
    /// Durability policy has not yet been set
    Unknown = 3,
}

#[repr(u32)]
/// QoS liveliness enumerations that describe a publisher's reporting policy for its alive status.
/// For a subscriber, these are its requirements for its topic's publishers.
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub enum RMWQoSLivelinessPolicy {
    /// Implementation specific default
    SystemDefault = 0,
    /// The signal that establishes a Topic is alive comes from the ROS rmw layer.
    Automatic = 1,
    /// Explicitly asserting node liveliness is required in this case.
    /// This option is deprecated, use RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC if your application
    /// requires to assert liveliness manually.
    #[deprecated]
    ManualByNode = 2,
    /// The signal that establishes a Topic is alive is at the Topic level. Only publishing a message
    /// on the Topic or an explicit signal from the application to assert liveliness on the Topic
    /// will mark the Topic as being alive.
    ManualByTopic = 3,
    /// Liveliness policy has not yet been set
    Unknown = 4,
}

pub const RMWQoSDeadlineDefault: rmw_time_t = rmw_time_t { sec: 0, nsec: 0 };
pub const RMWQoSLifespanDefault: rmw_time_t = rmw_time_t { sec: 0, nsec: 0 };
pub const RMWQoSLivelinessLeaseDurationDefault: rmw_time_t = rmw_time_t { sec: 0, nsec: 0 };

/// ROS MiddleWare quality of service profile.
#[repr(C)]
#[derive(Debug)]
pub struct rmw_qos_profile_t {
    pub history: RMWQoSHistoryPolicy,
    /// Size of the message queue.
    pub depth: usize,
    /// Reliabiilty QoS policy setting
    pub reliability: RMWQoSReliabilityPolicy,
    /// Durability QoS policy setting
    pub durability: RMWQoSDurabilityPolicy,
    /// The period at which messages are expected to be sent/received
    pub deadline: rmw_time_t,
    /// The age at which messages are considered expired and no longer valid
    pub lifespan: rmw_time_t,
    /// Liveliness QoS policy setting
    pub liveliness: RMWQoSLivelinessPolicy,
    /// The time within which the RMW node or publisher must show that it is alive
    pub liveliness_lease_duration: rmw_time_t,
    /// If true, any ROS specific namespacing conventions will be circumvented.
    pub avoid_ros_namespace_conventions: bool,
}

/// ROS graph ID of the topic
#[repr(C)]
#[derive(Debug)]
pub struct rmw_gid_t {
    /// Name of the rmw implementation
    pub implementation_identifier: *const c_char,
    /// Bype data Gid value
    pub data: [u8; 24usize],
}

/// Information describing an rmw message
#[repr(C)]
#[derive(Debug)]
pub struct rmw_message_info_t {
    pub source_timestamp: rmw_time_point_value_t,
    pub received_timestamp: rmw_time_point_value_t,
    pub publisher_gid: rmw_gid_t,
    /// Whether this message is from intra_process communication or not
    pub from_intra_process: bool,
}

extern "C" {
    /// Get zero initialized mesage info.
    pub fn rmw_get_zero_initialized_message_info() -> rmw_message_info_t;
}

/// Default size of the rmw queue when history is set to RMW_QOS_POLICY_HISTORY_KEEP_LAST,
/// 0 indicates it is currently not set
pub const RMW_QOS_POLICY_DEPTH_SYSTEM_DEFAULT: usize = 0;

/// QoS Liveliness Changed information provided by a subscription.
#[repr(C)]
#[derive(Debug)]
pub struct rmw_liveliness_changed_status_t {
    /// The total number of currently active Publishers which publish to the topic associated with
    /// the Subscription.
    /// This count increases when a newly matched Publisher asserts its liveliness for the first time
    /// or when a Publisher previously considered to be not alive reasserts its liveliness.
    /// The count decreases when a Publisher considered alive fails to assert its liveliness and
    /// becomes not alive, whether because it was deleted normally or for some other reason.
    pub alive_count: i32,
    /// The total count of current Publishers which publish to the topic associated with the
    /// Subscription that are no longer asserting their liveliness.
    /// This count increases when a Publisher considered alive fails to assert its liveliness and
    /// becomes not alive for some reason other than the normal deletion of that Publisher.
    /// It decreases when a previously not alive Publisher either reasserts its liveliness or is
    /// deleted normally.
    pub not_alive_count: i32,
    /// The change in the alive_count since the status was last read.
    pub alive_count_change: i32,
    /// The change in the not_alive_count since the status was last read.
    pub not_alive_count_change: i32,
}

/// QoS Requested Deadline Missed information provided by a subscription.
#[repr(C)]
#[derive(Debug)]
pub struct rmw_requested_deadline_missed_status_t {
    /// Lifetime cumulative number of missed deadlines detected for any instance read by the
    /// subscription.
    /// Missed deadlines accumulate; that is, each deadline period the total_count will be incremented
    /// by one for each instance for which data was not received.
    pub total_count: i32,
    /// The incremental number of deadlines detected since the status was read.
    pub total_count_change: i32,
}

/// QoS Liveliness Lost information provided by a publisher.
#[repr(C)]
#[derive(Debug)]
pub struct rmw_liveliness_lost_status_t {
    /// Lifetime cumulative number of times that a previously-alive Publisher became not alive due to
    /// a failure to actively signal its liveliness within its offered liveliness period.
    /// This count does not change when an already not alive Publisher simply remains not alive for
    /// another liveliness period.
    pub total_count: i32,
    /// The change in total_count since the last time the status was last read.
    pub total_count_change: i32,
}

/// QoS Deadline Missed information provided by a publisher.
#[repr(C)]
#[derive(Debug)]
pub struct rmw_offered_deadline_missed_status_t {
    /// Lifetime cumulative number of offered deadline periods elapsed during which a Publisher failed
    /// to provide data.
    /// Missed deadlines accumulate; that is, each deadline period the total_count will be incremented
    /// by one.
    pub total_count: i32,
    /// The change in total_count since the last time the status was last read.
    pub total_count_change: i32,
}
