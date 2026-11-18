use ::dora_ros2_bridge::ros2_client;
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

impl From<Ros2QosPolicies> for ros2_client::QosProfile {
    fn from(value: Ros2QosPolicies) -> Self {
        // The backend-neutral `Ros2Qos` collapses `Transient`/`Persistent` to
        // `Volatile`, so build reliability/history/liveliness through it and
        // then set the durability from the Python value directly.
        let durability = value.durability.into();
        ros2_client::QosProfile {
            durability,
            ..::dora_ros2_bridge::transport::dds::to_qos_profile(&value.into())
        }
    }
}

impl From<Ros2QosPolicies> for Ros2Qos {
    fn from(value: Ros2QosPolicies) -> Self {
        // Shared with the C++ bridge: never panics; infinite or too long means
        // "infinite", negative or NaN means zero.
        let to_duration = ::dora_ros2_bridge::transport::dds::duration_from_secs_f64;
        let lease_duration =
            (!value.lease_duration.is_infinite()).then(|| to_duration(value.lease_duration));
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
                    max_blocking_time: to_duration(value.max_blocking_time),
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

impl From<Ros2Durability> for ros2_client::qos::Durability {
    fn from(value: Ros2Durability) -> Self {
        match value {
            Ros2Durability::Volatile => Self::Volatile,
            Ros2Durability::TransientLocal => Self::TransientLocal,
            Ros2Durability::Transient | Ros2Durability::Persistent => {
                ::dora_ros2_bridge::transport::dds::transient_local_for_unsupported_durability()
            }
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

#[cfg(test)]
mod tests {
    use super::*;

    fn policies(
        durability: Ros2Durability,
        lease_duration: f64,
        reliable: bool,
        max_blocking_time: f64,
    ) -> Ros2QosPolicies {
        Ros2QosPolicies {
            durability,
            liveliness: Ros2Liveliness::Automatic,
            lease_duration,
            reliable,
            max_blocking_time,
            keep_all: false,
            keep_last: 1,
        }
    }

    /// The DDS profile carries VOLATILE and TRANSIENT_LOCAL as-is. ROS 2 (and
    /// `ros2-client` >= 0.11) has no TRANSIENT/PERSISTENT, so those are sent as
    /// TRANSIENT_LOCAL, the closest kind that still serves late joiners. They
    /// must not fall back to VOLATILE, as the backend-neutral `Ros2Qos` does.
    #[test]
    fn dds_durability_maps_to_ros2_kinds() {
        use ::dora_ros2_bridge::ros2_client::qos::Durability as D;
        for (variant, expected) in [
            (Ros2Durability::Volatile, D::Volatile),
            (Ros2Durability::TransientLocal, D::TransientLocal),
            (Ros2Durability::Transient, D::TransientLocal),
            (Ros2Durability::Persistent, D::TransientLocal),
        ] {
            let qos: ::dora_ros2_bridge::ros2_client::QosProfile =
                policies(variant, f64::INFINITY, false, 0.0).into();
            assert_eq!(qos.durability, expected);
        }
    }

    /// NaN / negative / overflowing QoS durations must not panic (they used to
    /// hit `Duration::from_secs_f64`, which panics). They are clamped to zero.
    #[test]
    fn invalid_qos_durations_do_not_panic() {
        for bad in [f64::NAN, -1.0, f64::MAX] {
            // `lease_duration` path (finite non-infinite value reaches the conversion)
            let _: Ros2Qos = policies(Ros2Durability::Volatile, bad, false, 0.0).into();
            // `max_blocking_time` path (only reached when `reliable` is true)
            let reliable: Ros2Qos =
                policies(Ros2Durability::Volatile, f64::INFINITY, true, bad).into();
            assert!(matches!(reliable.reliability, Reliability::Reliable { .. }));
        }
    }
}
