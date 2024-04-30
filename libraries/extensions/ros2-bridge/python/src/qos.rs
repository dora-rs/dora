use ::dora_ros2_bridge::rustdds::{self, policy};
use pyo3::prelude::{pyclass, pymethods};

/// ROS2 QoS Policy
///
/// :type durability: dora.Ros2Durability, optional
/// :type liveliness: dora.Ros2Liveliness, optional
/// :type reliable: bool, optional
/// :type keep_all: bool, optional
/// :type lease_duration: float, optional
/// :type max_blocking_time: float, optional
/// :type keep_last: int, optional
/// :rtype: dora.Ros2QoSPolicies
///
#[derive(Debug, Clone)]
#[pyclass]
#[non_exhaustive]
pub struct Ros2QosPolicies {
    pub durability: Ros2Durability,
    pub liveliness: Ros2Liveliness,
    pub lease_duration: f64,
    pub reliable: bool,
    pub max_blocking_time: f64,
    pub keep_all: bool,
    pub keep_last: i32,
}

#[pymethods]
impl Ros2QosPolicies {
    #[new]
    pub fn new(
        durability: Option<Ros2Durability>,
        liveliness: Option<Ros2Liveliness>,
        reliable: Option<bool>,
        keep_all: Option<bool>,
        lease_duration: Option<f64>,
        max_blocking_time: Option<f64>,
        keep_last: Option<i32>,
    ) -> Self {
        Self {
            durability: durability.unwrap_or(Ros2Durability::Volatile),
            liveliness: liveliness.unwrap_or(Ros2Liveliness::Automatic),
            lease_duration: lease_duration.unwrap_or(f64::INFINITY),
            reliable: reliable.unwrap_or(false),
            max_blocking_time: max_blocking_time.unwrap_or(0.0),
            keep_all: keep_all.unwrap_or(false),
            keep_last: keep_last.unwrap_or(1),
        }
    }
}

impl From<Ros2QosPolicies> for rustdds::QosPolicies {
    fn from(value: Ros2QosPolicies) -> Self {
        rustdds::QosPolicyBuilder::new()
            .durability(value.durability.into())
            .liveliness(value.liveliness.convert(value.lease_duration))
            .reliability(if value.reliable {
                policy::Reliability::Reliable {
                    max_blocking_time: rustdds::Duration::from_frac_seconds(
                        value.max_blocking_time,
                    ),
                }
            } else {
                policy::Reliability::BestEffort
            })
            .history(if value.keep_all {
                policy::History::KeepAll
            } else {
                policy::History::KeepLast {
                    depth: value.keep_last,
                }
            })
            .build()
    }
}

/// DDS 2.2.3.4 DURABILITY
///
/// :rtype: dora.Ros2Durability
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[pyclass]
pub enum Ros2Durability {
    Volatile,
    TransientLocal,
    Transient,
    Persistent,
}

/// :type value: dora.Ros2Durability
/// :rtype: dora.Ros2Durability
impl From<Ros2Durability> for policy::Durability {
    /// :type value: dora.Ros2Durability
    /// :rtype: dora.Ros2Durability
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
/// :rtype: dora.Ros2Liveliness
#[derive(Copy, Clone, Debug, PartialEq)]
#[pyclass]
pub enum Ros2Liveliness {
    Automatic,
    ManualByParticipant,
    ManualByTopic,
}

impl Ros2Liveliness {
    /// :type lease_duration: float
    /// :rtype: dora.Ros2Liveliness
    fn convert(self, lease_duration: f64) -> policy::Liveliness {
        let lease_duration = if lease_duration.is_infinite() {
            rustdds::Duration::INFINITE
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
