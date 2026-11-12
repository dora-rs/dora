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

/// Bounded queue of in-flight DDS requests paired with their send timestamps.
///
/// Used by generated DDS service and action clients whose response pump
/// demultiplexes replies by `RmwRequestId`. Without timeout eviction and a hard
/// capacity cap, unanswered requests (crashed or dropped servers) accumulate
/// without bound.
pub struct PendingDdsRequests<T> {
    entries: std::collections::VecDeque<(T, std::time::Instant)>,
    max_pending: usize,
    timeout: Option<Duration>,
}

impl<T> PendingDdsRequests<T> {
    /// Create a queue holding at most `max_pending` items, optionally expiring
    /// entries older than `timeout`.
    pub fn new(max_pending: usize, timeout: Option<Duration>) -> Self {
        Self {
            entries: std::collections::VecDeque::new(),
            max_pending,
            timeout,
        }
    }

    /// Number of currently tracked pending requests.
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    /// Whether there are no currently tracked pending requests.
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// Drop entries whose age at `now` has reached `timeout`; returns the count removed.
    pub fn prune_expired_at(&mut self, now: std::time::Instant) -> usize {
        let Some(timeout) = self.timeout else {
            return 0;
        };
        let before = self.entries.len();
        self.entries
            .retain(|(_, sent_at)| now.saturating_duration_since(*sent_at) < timeout);
        before - self.entries.len()
    }

    /// Record a new pending request at the current instant, evicting expired
    /// entries first and dropping the oldest if already at capacity.
    pub fn push(&mut self, item: T) {
        self.push_at(item, std::time::Instant::now());
    }

    /// Record a new pending request stamped with `now`.
    pub fn push_at(&mut self, item: T, now: std::time::Instant) {
        self.prune_expired_at(now);
        if self.max_pending == 0 {
            return;
        }
        while self.entries.len() >= self.max_pending {
            self.entries.pop_front();
        }
        self.entries.push_back((item, now));
    }

    /// Prune expired entries at the current instant, then remove and return the
    /// first entry matching `predicate`.
    pub fn take_matching<F>(&mut self, predicate: F) -> Option<T>
    where
        F: FnMut(&T) -> bool,
    {
        self.take_matching_at(std::time::Instant::now(), predicate)
    }

    /// Prune expired entries at `now`, then remove and return the first entry
    /// matching `predicate`.
    pub fn take_matching_at<F>(&mut self, now: std::time::Instant, mut predicate: F) -> Option<T>
    where
        F: FnMut(&T) -> bool,
    {
        self.prune_expired_at(now);
        let pos = self.entries.iter().position(|(item, _)| predicate(item))?;
        self.entries.remove(pos).map(|(item, _)| item)
    }
}

#[cfg(test)]
mod pending_requests_tests {
    use super::PendingDdsRequests;
    use std::time::{Duration, Instant};

    #[test]
    fn evicts_expired_entries_on_push_and_match() {
        let mut pending = PendingDdsRequests::new(64, Some(Duration::from_secs(30)));
        let t0 = Instant::now();
        pending.push_at(1u64, t0);
        pending.push_at(2u64, t0 + Duration::from_secs(20));

        // At t0 + 35s, request 1 has expired while request 2 (15s old) remains valid.
        let t1 = t0 + Duration::from_secs(35);
        assert_eq!(pending.take_matching_at(t1, |id| *id == 1), None);
        assert_eq!(pending.len(), 1);
        assert_eq!(pending.take_matching_at(t1, |id| *id == 2), Some(2));
        assert!(pending.is_empty());

        // Pushing after expiration prunes stale entries before recording the new one.
        pending.push_at(3u64, t1);
        pending.push_at(4u64, t1 + Duration::from_secs(31));
        assert_eq!(pending.len(), 1);
        assert_eq!(
            pending.take_matching_at(t1 + Duration::from_secs(31), |id| *id == 3),
            None
        );
        assert_eq!(
            pending.take_matching_at(t1 + Duration::from_secs(31), |id| *id == 4),
            Some(4)
        );
    }

    #[test]
    fn evicts_oldest_when_at_capacity() {
        let mut pending = PendingDdsRequests::new(3, Some(Duration::from_secs(30)));
        let t0 = Instant::now();
        pending.push_at(10u64, t0);
        pending.push_at(20u64, t0);
        pending.push_at(30u64, t0);
        pending.push_at(40u64, t0);

        assert_eq!(pending.len(), 3);
        assert_eq!(pending.take_matching_at(t0, |id| *id == 10), None);
        assert_eq!(pending.take_matching_at(t0, |id| *id == 20), Some(20));
        assert_eq!(pending.take_matching_at(t0, |id| *id == 30), Some(30));
        assert_eq!(pending.take_matching_at(t0, |id| *id == 40), Some(40));
        assert!(pending.is_empty());
    }

    #[test]
    fn without_timeout_only_enforces_capacity_and_matches_out_of_order() {
        let mut pending = PendingDdsRequests::new(2, None);
        let t0 = Instant::now();
        pending.push_at("a", t0);
        pending.push_at("b", t0 + Duration::from_secs(600));
        assert_eq!(pending.len(), 2);

        // Out-of-order response matches "b" and leaves "a" intact.
        assert_eq!(
            pending.take_matching_at(t0 + Duration::from_secs(600), |id| *id == "b"),
            Some("b")
        );
        assert_eq!(pending.len(), 1);
        assert_eq!(
            pending.take_matching_at(t0 + Duration::from_secs(1200), |id| *id == "a"),
            Some("a")
        );
    }
}
