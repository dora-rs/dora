//! Values shared by all ROS2 transport implementations.

use std::time::Duration;

/// ROS2 reliability behavior.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Reliability {
    /// Samples may be dropped by the transport.
    BestEffort,
    /// Samples use reliable delivery with a bounded blocking time.
    Reliable {
        /// Maximum duration a write may block.
        max_blocking_time: Duration,
    },
}

/// ROS2 durability behavior supported by the bridge.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Durability {
    /// Only samples published while a subscriber is present are available.
    Volatile,
    /// A publisher retains history for late-joining subscribers.
    TransientLocal,
}

/// ROS2 history behavior.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum History {
    /// Retain at most `depth` samples.
    KeepLast {
        /// Positive history depth.
        depth: i32,
    },
    /// Retain every sample up to transport resource limits.
    KeepAll,
}

/// ROS2 liveliness behavior.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Liveliness {
    /// The transport automatically asserts participant liveliness.
    Automatic {
        /// Lease duration, or infinity when absent.
        lease_duration: Option<Duration>,
    },
    /// The participant manually asserts liveliness.
    ManualByParticipant {
        /// Lease duration, or infinity when absent.
        lease_duration: Option<Duration>,
    },
    /// Each publisher manually asserts liveliness.
    ManualByTopic {
        /// Lease duration, or infinity when absent.
        lease_duration: Option<Duration>,
    },
}

/// Backend-neutral ROS2 QoS policies supported by Dora.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Ros2Qos {
    /// Reliability policy.
    pub reliability: Reliability,
    /// Durability policy.
    pub durability: Durability,
    /// History policy.
    pub history: History,
    /// Liveliness policy.
    pub liveliness: Liveliness,
}

impl Default for Ros2Qos {
    fn default() -> Self {
        Self {
            reliability: Reliability::BestEffort,
            durability: Durability::Volatile,
            history: History::KeepLast { depth: 1 },
            liveliness: Liveliness::Automatic {
                lease_duration: None,
            },
        }
    }
}

/// Metadata carried with one ROS2 topic sample.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MessageMetadata {
    /// Publisher-local monotonic sequence number.
    pub sequence_number: i64,
    /// Source timestamp expressed as Unix nanoseconds.
    pub source_timestamp_ns: i64,
    /// ROS2 publisher global identifier.
    pub publisher_gid: [u8; 16],
}

/// Backend-neutral identifier for one ROS2 service request.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct RequestId {
    /// Client-local monotonic sequence number.
    pub sequence_number: i64,
    /// ROS2 client global identifier.
    pub client_gid: [u8; 16],
}

/// Failure returned by a native ROS2 transport adapter.
#[derive(Debug, thiserror::Error)]
pub enum TransportError {
    /// The selected transport cannot report this QoS status event.
    #[error("ROS2 QoS event `{event}` is unsupported by this transport")]
    UnsupportedQosEvent {
        /// Stable ROS2 event name.
        event: &'static str,
    },
    /// The selected backend is not available in this build phase.
    #[error("ROS2 transport `{transport}` is not implemented")]
    UnsupportedTransport {
        /// Stable backend name used in diagnostics.
        transport: &'static str,
    },
    /// The existing DDS context could not be created.
    #[error("failed to create DDS ROS2 context: {message}")]
    DdsContext {
        /// Original error rendered without backend data.
        message: String,
    },
    /// A DDS node could not be created.
    #[error("failed to create DDS ROS2 node: {message}")]
    DdsNode {
        /// Original error rendered without backend data.
        message: String,
    },
    #[error("failed to create Zenoh ROS2 context: {message}")]
    ZenohContext { message: String },
    #[error("failed to create Zenoh ROS2 node: {message}")]
    ZenohNode { message: String },
}
