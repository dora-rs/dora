use ::dora_ros2_bridge::rustdds::{self, policy};
use ::dora_ros2_bridge::transport::{Durability, History, Liveliness, Reliability, Ros2Qos};
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
/// :rtype: dora.Ros2QosPolicies
///
#[derive(Clone)]
#[pyclass(from_py_object)]
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
    #[pyo3(signature = (durability=None, liveliness=None, reliable=None, keep_all=None, lease_duration=None, max_blocking_time=None, keep_last=None))]
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
        ::dora_ros2_bridge::transport::dds::to_rustdds_qos(&value.into())
    }
}

impl From<Ros2QosPolicies> for Ros2Qos {
    fn from(value: Ros2QosPolicies) -> Self {
        let lease_duration = (!value.lease_duration.is_infinite())
            .then(|| std::time::Duration::from_secs_f64(value.lease_duration));
        Self {
            durability: match value.durability {
                Ros2Durability::TransientLocal => Durability::TransientLocal,
                _ => Durability::Volatile,
            },
            liveliness: match value.liveliness {
                Ros2Liveliness::Automatic => Liveliness::Automatic { lease_duration },
                Ros2Liveliness::ManualByParticipant => {
                    Liveliness::ManualByParticipant { lease_duration }
                }
                Ros2Liveliness::ManualByTopic => Liveliness::ManualByTopic { lease_duration },
            },
            reliability: if value.reliable {
                Reliability::Reliable {
                    max_blocking_time: std::time::Duration::from_secs_f64(value.max_blocking_time),
                }
            } else {
                Reliability::BestEffort
            },
            history: if value.keep_all {
                History::KeepAll
            } else {
                History::KeepLast {
                    depth: value.keep_last,
                }
            },
        }
    }
}

/// DDS 2.2.3.4 DURABILITY
///
/// :rtype: dora.Ros2Durability
#[derive(Copy, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[pyclass(eq, eq_int, from_py_object)]
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
#[derive(Copy, Clone, PartialEq)]
#[pyclass(eq, eq_int, from_py_object)]
pub enum Ros2Liveliness {
    Automatic,
    ManualByParticipant,
    ManualByTopic,
}
