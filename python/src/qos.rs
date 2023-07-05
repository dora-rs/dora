use ::dora_ros2_bridge::rustdds::{self, policy};
use pyo3::prelude::{pyclass, pymethods};

#[derive(Debug, Clone)]
#[pyclass(get_all, set_all)]
#[non_exhaustive]
pub struct Ros2QosPolicies {
    pub durability: Ros2Durability,
    pub liveliness: Ros2Liveliness,
    pub liveliness_lease_duration: f64,
    pub reliability: Ros2Reliability,
    pub reliability_max_blocking_time: f64,
    pub history: Ros2History,
    pub history_keep_last: i32,
}

#[pymethods]
impl Ros2QosPolicies {
    #[new]
    pub fn new() -> Self {
        Self {
            durability: Ros2Durability::Volatile,
            liveliness: Ros2Liveliness::Automatic,
            liveliness_lease_duration: f64::INFINITY,
            reliability: Ros2Reliability::BestEffort,
            reliability_max_blocking_time: 0.0,
            history: Ros2History::KeepLast,
            history_keep_last: 1,
        }
    }
}

impl From<Ros2QosPolicies> for rustdds::QosPolicies {
    fn from(value: Ros2QosPolicies) -> Self {
        rustdds::QosPolicyBuilder::new()
            .durability(value.durability.into())
            .liveliness(value.liveliness.convert(value.liveliness_lease_duration))
            .reliability(
                value
                    .reliability
                    .convert(value.reliability_max_blocking_time),
            )
            .history(value.history.convert(value.history_keep_last))
            .build()
    }
}

/// DDS 2.2.3.4 DURABILITY
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[pyclass]
pub enum Ros2Durability {
    Volatile,
    TransientLocal,
    Transient,
    Persistent,
}

impl From<Ros2Durability> for policy::Durability {
    fn from(value: Ros2Durability) -> Self {
        match value {
            Ros2Durability::Volatile => policy::Durability::Volatile,
            Ros2Durability::TransientLocal => policy::Durability::TransientLocal,
            Ros2Durability::Transient => policy::Durability::Transient,
            Ros2Durability::Persistent => policy::Durability::Persistent,
        }
    }
}

/// DDS 2.2.3.11 LIVELINESS
#[derive(Copy, Clone, Debug, PartialEq)]
#[pyclass]
pub enum Ros2Liveliness {
    Automatic,
    ManualByParticipant,
    ManualByTopic,
}

impl Ros2Liveliness {
    fn convert(self, lease_duration: f64) -> policy::Liveliness {
        let lease_duration = if lease_duration.is_infinite() {
            rustdds::Duration::DURATION_INFINITE
        } else {
            rustdds::Duration::from_frac_seconds(lease_duration)
        };
        match self {
            Ros2Liveliness::Automatic => policy::Liveliness::Automatic { lease_duration },
            Ros2Liveliness::ManualByParticipant => {
                policy::Liveliness::ManualByParticipant { lease_duration }
            }
            Ros2Liveliness::ManualByTopic => policy::Liveliness::ManualByTopic { lease_duration },
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
#[pyclass]
pub enum Ros2Reliability {
    BestEffort,
    Reliable,
}

impl Ros2Reliability {
    fn convert(self, max_blocking_time: f64) -> policy::Reliability {
        match self {
            Ros2Reliability::BestEffort => policy::Reliability::BestEffort,
            Ros2Reliability::Reliable => policy::Reliability::Reliable {
                max_blocking_time: rustdds::Duration::from_frac_seconds(max_blocking_time),
            },
        }
    }
}

/// DDS 2.2.3.18 HISTORY
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[pyclass]
pub enum Ros2History {
    KeepLast,
    KeepAll,
}

impl Ros2History {
    fn convert(self, keep_last: i32) -> policy::History {
        match self {
            Ros2History::KeepLast => policy::History::KeepLast { depth: keep_last },
            Ros2History::KeepAll => policy::History::KeepAll,
        }
    }
}
