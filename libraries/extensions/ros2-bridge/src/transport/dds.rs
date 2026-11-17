//! Adapter for the existing `ros2-client`/RustDDS implementation.

use std::time::Duration;

use super::{Durability, History, Liveliness, Reliability, Ros2Qos, TransportError};

/// Pick the DDS service mapping matching the user's ROS2 middleware.
///
/// `RMW_IMPLEMENTATION` is primary and `ROS_DISTRO` is the compatibility
/// fallback. Unknown values preserve the historical enhanced mapping.
pub fn detect_service_mapping() -> ros2_client::ServiceMapping {
    match std::env::var("RMW_IMPLEMENTATION").ok().as_deref() {
        Some("rmw_fastrtps_cpp") => return ros2_client::ServiceMapping::Enhanced,
        Some("rmw_cyclonedds_cpp") => return ros2_client::ServiceMapping::Cyclone,
        Some(other) => {
            tracing::warn!(
                "unknown RMW_IMPLEMENTATION `{other}`, falling back to \
                 ServiceMapping::Enhanced (see dora-rs/dora#449)"
            );
            return ros2_client::ServiceMapping::Enhanced;
        }
        None => {}
    }
    match std::env::var("ROS_DISTRO").ok().as_deref() {
        Some("humble" | "iron" | "jazzy" | "kilted" | "rolling") => {
            ros2_client::ServiceMapping::Enhanced
        }
        Some("galactic") => ros2_client::ServiceMapping::Cyclone,
        Some(other) => {
            tracing::warn!(
                "unknown ROS_DISTRO `{other}`, falling back to \
                 ServiceMapping::Enhanced (see dora-rs/dora#449)"
            );
            ros2_client::ServiceMapping::Enhanced
        }
        None => ros2_client::ServiceMapping::Enhanced,
    }
}

/// Existing DDS ROS2 context.
pub struct Context {
    inner: ros2_client::Context,
}

impl Context {
    /// Open a DDS ROS2 context in the requested ROS domain.
    pub fn new(domain_id: u16) -> Result<Self, TransportError> {
        ros2_client::Context::with_options(ros2_client::ContextOptions::new().domain_id(domain_id))
            .map(|inner| Self { inner })
            .map_err(|error| TransportError::DdsContext {
                message: format!("{error:?}"),
            })
    }

    /// Create an existing DDS ROS2 node.
    pub fn new_node(
        &self,
        node_name: ros2_client::NodeName,
        options: ros2_client::NodeOptions,
    ) -> Result<Node, TransportError> {
        self.inner
            .new_node(node_name, options)
            .map(|inner| Node { inner })
            .map_err(|error| TransportError::DdsNode {
                message: format!("{error:?}"),
            })
    }

    /// Borrow the concrete context for legacy DDS-only call sites.
    pub fn as_inner(&self) -> &ros2_client::Context {
        &self.inner
    }
}

/// Existing DDS ROS2 node.
pub struct Node {
    inner: ros2_client::Node,
}

impl Node {
    /// Consume the adapter and return the legacy DDS node.
    pub fn into_inner(self) -> ros2_client::Node {
        self.inner
    }

    /// Borrow the concrete node for legacy DDS-only call sites.
    pub fn as_inner(&self) -> &ros2_client::Node {
        &self.inner
    }

    /// Mutably borrow the concrete node for legacy DDS-only call sites.
    pub fn as_inner_mut(&mut self) -> &mut ros2_client::Node {
        &mut self.inner
    }
}

/// The longest duration `ros2-client` can put on the wire: it converts a
/// blocking time or lease with `as_nanos() as i64`. Anything this long or
/// longer means "infinite", as `rustdds::Duration` used to saturate it.
const INFINITE: Duration = Duration::from_nanos(i64::MAX as u64);

fn finite(duration: Duration) -> Option<Duration> {
    (duration < INFINITE).then_some(duration)
}

/// A QoS duration given as f64 seconds, as the Python and C++ QoS types take
/// it: infinite or too long means "infinite" (blocking / never expiring), and
/// negative or NaN means zero.
pub fn duration_from_secs_f64(secs: f64) -> Duration {
    Duration::try_from_secs_f64(secs.max(0.0)).unwrap_or(Duration::MAX)
}

/// The DDS durability sent for the TRANSIENT/PERSISTENT kinds that the Python
/// and C++ QoS types expose but ROS 2 lacks: TRANSIENT_LOCAL, the closest kind
/// that still serves late joiners. Warns once per process.
pub fn transient_local_for_unsupported_durability() -> ros2_client::qos::Durability {
    static WARN: std::sync::Once = std::sync::Once::new();
    WARN.call_once(|| {
        tracing::warn!(
            "QoS durability `transient`/`persistent` is not supported on the DDS transport \
             (ROS 2 has no such kind); using `transient_local`"
        );
    });
    ros2_client::qos::Durability::TransientLocal
}

/// Convert backend-neutral policies into the `ros2-client` DDS QoS profile.
///
/// The profile has no MANUAL_BY_PARTICIPANT liveliness (ROS 2 deprecated the
/// equivalent MANUAL_BY_NODE and rclcpp rejects it), so that kind is sent as
/// AUTOMATIC with the same lease, and a warning is logged once.
pub fn to_qos_profile(qos: &Ros2Qos) -> ros2_client::QosProfile {
    use ros2_client::qos;

    let reliability = match qos.reliability {
        Reliability::BestEffort => qos::Reliability::BestEffort,
        // On DDS `when_full` only sets the RTPS `max_blocking_time`, so
        // `Wait(d)` is exactly the previous `max_blocking_time: d`.
        Reliability::Reliable { max_blocking_time } => qos::Reliability::Reliable {
            when_full: finite(max_blocking_time).map_or(qos::WhenFull::Block, qos::WhenFull::Wait),
        },
    };
    let durability = match qos.durability {
        Durability::Volatile => qos::Durability::Volatile,
        Durability::TransientLocal => qos::Durability::TransientLocal,
    };
    let history = match qos.history {
        // A depth below 1 would retain nothing.
        History::KeepLast { depth } => qos::History::KeepLast {
            depth: usize::try_from(depth.max(1)).unwrap_or(1),
        },
        History::KeepAll => qos::History::KeepAll,
    };
    let (liveliness, lease) = match qos.liveliness {
        Liveliness::Automatic { lease_duration } => (qos::Liveliness::Automatic, lease_duration),
        Liveliness::ManualByTopic { lease_duration } => {
            (qos::Liveliness::ManualByTopic, lease_duration)
        }
        Liveliness::ManualByParticipant { lease_duration } => {
            static WARN: std::sync::Once = std::sync::Once::new();
            WARN.call_once(|| {
                tracing::warn!(
                    "QoS liveliness `manual_by_participant` is not supported on the DDS \
                     transport (ROS 2 has no such kind); using `automatic`"
                );
            });
            (qos::Liveliness::Automatic, lease_duration)
        }
    };

    ros2_client::QosProfile {
        reliability,
        durability,
        history,
        deadline: None,
        lifespan: None,
        liveliness,
        liveliness_lease: lease.and_then(finite),
    }
}

/// Convert a `ros2-client` DDS QoS profile back to neutral policies.
pub fn from_qos_profile(profile: &ros2_client::QosProfile) -> Ros2Qos {
    use ros2_client::qos;

    let reliability = match profile.reliability {
        qos::Reliability::BestEffort => Reliability::BestEffort,
        qos::Reliability::Reliable { when_full } => Reliability::Reliable {
            max_blocking_time: match when_full {
                qos::WhenFull::Fail => Duration::ZERO,
                qos::WhenFull::Wait(d) => d,
                qos::WhenFull::Block => INFINITE,
            },
        },
    };
    let durability = match profile.durability {
        qos::Durability::Volatile => Durability::Volatile,
        qos::Durability::TransientLocal => Durability::TransientLocal,
    };
    let history = match profile.history {
        qos::History::KeepLast { depth } => History::KeepLast {
            depth: i32::try_from(depth).unwrap_or(i32::MAX),
        },
        qos::History::KeepAll => History::KeepAll,
    };
    let lease_duration = profile.liveliness_lease;
    let liveliness = match profile.liveliness {
        qos::Liveliness::Automatic => Liveliness::Automatic { lease_duration },
        qos::Liveliness::ManualByTopic => Liveliness::ManualByTopic { lease_duration },
    };

    Ros2Qos {
        reliability,
        durability,
        history,
        liveliness,
    }
}
