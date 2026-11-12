//! Adapter for the existing `ros2-client`/RustDDS implementation.

use std::time::{Duration, Instant};

use super::{Durability, History, Liveliness, Reliability, Ros2Qos, TransportError};

pub struct PendingDdsRequests<T> {
    entries: Vec<(T, Instant)>,
    max_len: usize,
    timeout: Option<Duration>,
}

impl<T> PendingDdsRequests<T> {
    pub fn new(max_len: usize, timeout: Option<Duration>) -> Self {
        Self {
            entries: Vec::new(),
            max_len,
            timeout,
        }
    }

    pub fn push(&mut self, request_id: T) {
        self.evict_expired();
        if self.entries.len() >= self.max_len {
            self.entries.remove(0);
        }
        self.entries.push((request_id, Instant::now()));
    }

    pub fn take_matching(&mut self, mut predicate: impl FnMut(&T) -> bool) -> Option<T> {
        self.evict_expired();
        self.entries
            .iter()
            .position(|(request_id, _)| predicate(request_id))
            .map(|index| self.entries.remove(index).0)
    }

    fn evict_expired(&mut self) {
        let Some(timeout) = self.timeout else {
            return;
        };
        let now = Instant::now();
        self.entries
            .retain(|(_, inserted_at)| now.duration_since(*inserted_at) < timeout);
    }

    #[cfg(test)]
    fn len(&self) -> usize {
        self.entries.len()
    }
}

#[cfg(test)]
mod pending_request_tests {
    use super::PendingDdsRequests;
    use std::thread;

    #[test]
    fn evicts_unreplied_requests_after_timeout() {
        let mut pending = PendingDdsRequests::new(8, Some(std::time::Duration::from_millis(10)));

        pending.push(1);
        thread::sleep(std::time::Duration::from_millis(20));
        pending.push(2);

        assert_eq!(pending.len(), 1);
        assert_eq!(pending.take_matching(|id| *id == 1), None);
        assert_eq!(pending.take_matching(|id| *id == 2), Some(2));
    }
}

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

/// Convert backend-neutral policies into RustDDS policies.
pub fn to_rustdds_qos(qos: &Ros2Qos) -> rustdds::QosPolicies {
    use rustdds::policy;

    let reliability = match qos.reliability {
        Reliability::BestEffort => policy::Reliability::BestEffort,
        Reliability::Reliable { max_blocking_time } => policy::Reliability::Reliable {
            max_blocking_time: max_blocking_time.into(),
        },
    };
    let durability = match qos.durability {
        Durability::Volatile => policy::Durability::Volatile,
        Durability::TransientLocal => policy::Durability::TransientLocal,
    };
    let history = match qos.history {
        History::KeepLast { depth } => policy::History::KeepLast { depth },
        History::KeepAll => policy::History::KeepAll,
    };
    let liveliness = match qos.liveliness {
        Liveliness::Automatic { lease_duration } => policy::Liveliness::Automatic {
            lease_duration: to_dds_duration(lease_duration),
        },
        Liveliness::ManualByParticipant { lease_duration } => {
            policy::Liveliness::ManualByParticipant {
                lease_duration: to_dds_duration(lease_duration),
            }
        }
        Liveliness::ManualByTopic { lease_duration } => policy::Liveliness::ManualByTopic {
            lease_duration: to_dds_duration(lease_duration),
        },
    };

    rustdds::QosPolicyBuilder::new()
        .reliability(reliability)
        .durability(durability)
        .history(history)
        .liveliness(liveliness)
        .build()
}

/// Convert the bridge-supported subset of RustDDS policies to neutral policies.
pub fn from_rustdds_qos(qos: &rustdds::QosPolicies) -> Ros2Qos {
    use rustdds::policy;

    let reliability = match qos.reliability().unwrap_or(policy::Reliability::BestEffort) {
        policy::Reliability::BestEffort => Reliability::BestEffort,
        policy::Reliability::Reliable { max_blocking_time } => Reliability::Reliable {
            max_blocking_time: max_blocking_time.into(),
        },
    };
    let durability = match qos.durability().unwrap_or(policy::Durability::Volatile) {
        policy::Durability::TransientLocal => Durability::TransientLocal,
        policy::Durability::Volatile
        | policy::Durability::Transient
        | policy::Durability::Persistent => Durability::Volatile,
    };
    let history = match qos
        .history()
        .unwrap_or(policy::History::KeepLast { depth: 1 })
    {
        policy::History::KeepLast { depth } => History::KeepLast { depth },
        policy::History::KeepAll => History::KeepAll,
    };
    let liveliness = match qos.liveliness().unwrap_or(policy::Liveliness::Automatic {
        lease_duration: rustdds::Duration::INFINITE,
    }) {
        policy::Liveliness::Automatic { lease_duration } => Liveliness::Automatic {
            lease_duration: from_dds_duration(lease_duration),
        },
        policy::Liveliness::ManualByParticipant { lease_duration } => {
            Liveliness::ManualByParticipant {
                lease_duration: from_dds_duration(lease_duration),
            }
        }
        policy::Liveliness::ManualByTopic { lease_duration } => Liveliness::ManualByTopic {
            lease_duration: from_dds_duration(lease_duration),
        },
    };

    Ros2Qos {
        reliability,
        durability,
        history,
        liveliness,
    }
}

fn to_dds_duration(duration: Option<Duration>) -> rustdds::Duration {
    duration
        .map(Into::into)
        .unwrap_or(rustdds::Duration::INFINITE)
}

fn from_dds_duration(duration: rustdds::Duration) -> Option<Duration> {
    (duration != rustdds::Duration::INFINITE).then(|| duration.into())
}
