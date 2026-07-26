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
        // `Transient`/`Persistent` durability exists only on the DDS path: the
        // backend-neutral `Ros2Qos` collapses both to `Volatile`, so routing DDS
        // QoS through it would silently downgrade them on the wire. Build the
        // shared policies through the neutral path (reliability/history/
        // liveliness), then restore the faithful four-variant DDS durability.
        let durability: policy::Durability = value.durability.into();
        let neutral = ::dora_ros2_bridge::transport::dds::to_rustdds_qos(&value.into());
        let mut builder = rustdds::QosPolicyBuilder::new().durability(durability);
        if let Some(reliability) = neutral.reliability() {
            builder = builder.reliability(reliability);
        }
        if let Some(history) = neutral.history() {
            builder = builder.history(history);
        }
        if let Some(liveliness) = neutral.liveliness() {
            builder = builder.liveliness(liveliness);
        }
        builder.build()
    }
}

impl From<Ros2QosPolicies> for Ros2Qos {
    fn from(value: Ros2QosPolicies) -> Self {
        // `Duration::from_secs_f64` panics on a negative, NaN, or overflowing
        // value; the pre-existing `rustdds` path did not. Convert without
        // panicking, treating an invalid duration as zero.
        let to_duration = |secs: f64| {
            std::time::Duration::try_from_secs_f64(secs).unwrap_or(std::time::Duration::ZERO)
        };
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

    /// The DDS conversion must preserve all four durability variants. Routing it
    /// through the two-variant backend-neutral `Ros2Qos` used to silently
    /// downgrade `Transient`/`Persistent` to `Volatile` (regression from #2790).
    #[test]
    fn dds_durability_preserves_all_variants() {
        for (variant, expected) in [
            (Ros2Durability::Volatile, policy::Durability::Volatile),
            (
                Ros2Durability::TransientLocal,
                policy::Durability::TransientLocal,
            ),
            (Ros2Durability::Transient, policy::Durability::Transient),
            (Ros2Durability::Persistent, policy::Durability::Persistent),
        ] {
            let qos: rustdds::QosPolicies = policies(variant, f64::INFINITY, false, 0.0).into();
            assert_eq!(qos.durability(), Some(expected));
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
