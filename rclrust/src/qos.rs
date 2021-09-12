//! QoS (Quality of Service)

use std::time::Duration;

use crate::{impl_from_trait_for_enum, time::RclDurationT};

/// QoS reliability enumerations
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReliabilityPolicy {
    /// Implementation specific default
    SystemDefault,
    /// Guarantee that samples are delivered, may retry multiple times.
    Reliable,
    /// Attempt to deliver samples, but some may be lost if the network is not robust
    BestEffort,
    /// Reliability policy has not yet been set
    Unknown,
}

impl_from_trait_for_enum! {
    ReliabilityPolicy,
    rcl_sys::rmw_qos_reliability_policy_t,
    SystemDefault := RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
    Reliable := RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    BestEffort := RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    Unknown := RMW_QOS_POLICY_RELIABILITY_UNKNOWN,
}

/// QoS history enumerations describing how samples endure
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HistoryPolicy {
    /// Implementation default for history policy
    SystemDefault,
    /// Only store up to a maximum number of samples, dropping oldest once max is exceeded
    KeepLast,
    /// Store all samples, subject to resource limits
    KeepAll,
    /// History policy has not yet been set
    Unknown,
}

impl_from_trait_for_enum! {
    HistoryPolicy,
    rcl_sys::rmw_qos_history_policy_t,
    SystemDefault := RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT,
    KeepLast := RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    KeepAll := RMW_QOS_POLICY_HISTORY_KEEP_ALL,
    Unknown := RMW_QOS_POLICY_HISTORY_UNKNOWN,
}

/// QoS durability enumerations describing how samples persist
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DurabilityPolicy {
    /// Implementation specific default
    SystemDefault,
    /// The rmw publisher is responsible for persisting samples for “late-joining” subscribers
    TransientLocal,
    /// Samples are not persistent
    Volatile,
    /// Durability policy has not yet been set
    Unknown,
}

impl_from_trait_for_enum! {
    DurabilityPolicy,
    rcl_sys::rmw_qos_durability_policy_t,
    SystemDefault := RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT,
    TransientLocal := RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
    Volatile := RMW_QOS_POLICY_DURABILITY_VOLATILE,
    Unknown := RMW_QOS_POLICY_DURABILITY_UNKNOWN,
}

/// QoS liveliness enumerations that describe a publisher's reporting policy for its alive status.
/// For a subscriber, these are its requirements for its topic's publishers.
#[allow(deprecated)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LivelinessPolicy {
    /// Implementation specific default
    SystemDefault,
    /// The signal that establishes a Topic is alive comes from the ROS rmw layer.
    Automatic,
    /// Explicitly asserting node liveliness is required in this case.
    /// This option is deprecated, use RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC if your application
    /// requires to assert liveliness manually.
    #[deprecated]
    ManualByNode,
    /// The signal that establishes a Topic is alive is at the Topic level. Only publishing a message
    /// on the Topic or an explicit signal from the application to assert liveliness on the Topic
    /// will mark the Topic as being alive.
    ManualByTopic,
    /// Liveliness policy has not yet been set
    Unknown,
}

impl_from_trait_for_enum! {
    LivelinessPolicy,
    rcl_sys::rmw_qos_liveliness_policy_t,
    SystemDefault := RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    Automatic := RMW_QOS_POLICY_LIVELINESS_AUTOMATIC,
    #[allow(deprecated)]
    ManualByNode := RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_NODE,
    ManualByTopic := RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC,
    Unknown := RMW_QOS_POLICY_LIVELINESS_UNKNOWN,
}

/// QoS profile
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct QoSProfile {
    /// History QoS policy setting.
    pub history: HistoryPolicy,
    /// Size of the message queue.
    pub depth: usize,
    /// Reliabiilty QoS policy setting.
    pub reliability: ReliabilityPolicy,
    /// Durability QoS policy setting.
    pub durability: DurabilityPolicy,
    /// The period at which messages are expected to be sent/received.
    pub deadline: Duration,
    /// The age at which messages are considered expired and no longer valid.
    pub lifespan: Duration,
    /// Liveliness QoS policy setting.
    pub liveliness: LivelinessPolicy,
    /// The time within which the RMW node or publisher must show that it is alive.
    pub liveliness_lease_duration: Duration,
    /// If true, any ROS specific namespacing conventions will be circumvented.
    ///
    /// In the case of DDS and topics, for example, this means the typical ROS specific prefix of rt
    /// would not be applied as described here:
    ///
    /// <http://design.ros2.org/articles/topic_and_service_names.html#ros-specific-namespace-prefix>
    ///
    /// This might be useful when trying to directly connect a native DDS topic with a ROS 2 topic.
    pub avoid_ros_namespace_conventions: bool,
}

impl QoSProfile {
    /// Sensor Data QoS class
    ///    - History: Keep last,
    ///    - Depth: 5,
    ///    - Reliability: Best effort,
    ///    - Durability: Volatile,
    ///    - Deadline: Default,
    ///    - Lifespan: Default,
    ///    - Liveliness: System default,
    ///    - Liveliness lease duration: Default,
    ///    - avoid ros namespace conventions: false
    pub const fn sensor_data() -> Self {
        Self {
            history: HistoryPolicy::KeepLast,
            depth: 5,
            reliability: ReliabilityPolicy::BestEffort,
            durability: DurabilityPolicy::Volatile,
            ..Self::common()
        }
    }

    /// Parameters QoS class
    ///    - History: Keep last,
    ///    - Depth: 1000,
    ///    - Reliability: Reliable,
    ///    - Durability: Volatile,
    ///    - Deadline: Default,
    ///    - Lifespan: Default,
    ///    - Liveliness: System default,
    ///    - Liveliness lease duration: Default,
    ///    - Avoid ros namespace conventions: false
    pub const fn parameters() -> Self {
        Self {
            history: HistoryPolicy::KeepLast,
            depth: 1000,
            reliability: ReliabilityPolicy::Reliable,
            durability: DurabilityPolicy::Volatile,
            ..Self::common()
        }
    }

    /// Default QoS class
    ///    - History: Keep last,
    ///    - Depth: 10,
    ///    - Reliability: Reliable,
    ///    - Durability: Volatile,
    ///    - Deadline: Default,
    ///    - Lifespan: Default,
    ///    - Liveliness: System default,
    ///    - Liveliness lease duration: Default,
    ///    - Avoid ros namespace conventions: false
    pub const fn default() -> Self {
        Self {
            history: HistoryPolicy::KeepLast,
            depth: 10,
            reliability: ReliabilityPolicy::Reliable,
            durability: DurabilityPolicy::Volatile,
            ..Self::common()
        }
    }

    /// Services QoS class
    ///    - History: Keep last,
    ///    - Depth: 10,
    ///    - Reliability: Reliable,
    ///    - Durability: Volatile,
    ///    - Deadline: Default,
    ///    - Lifespan: Default,
    ///    - Liveliness: System default,
    ///    - Liveliness lease duration: Default,
    ///    - Avoid ros namespace conventions: false
    pub const fn services_default() -> Self {
        Self {
            history: HistoryPolicy::KeepLast,
            depth: 10,
            reliability: ReliabilityPolicy::Reliable,
            durability: DurabilityPolicy::Volatile,
            ..Self::common()
        }
    }

    /// Parameter events QoS class
    ///    - History: Keep last,
    ///    - Depth: 1000,
    ///    - Reliability: Reliable,
    ///    - Durability: Volatile,
    ///    - Deadline: Default,
    ///    - Lifespan: Default,
    ///    - Liveliness: System default,
    ///    - Liveliness lease duration: Default,
    ///    - Avoid ros namespace conventions: false
    pub const fn parameter_events() -> Self {
        Self {
            history: HistoryPolicy::KeepLast,
            depth: 1000,
            reliability: ReliabilityPolicy::Reliable,
            durability: DurabilityPolicy::Volatile,
            ..Self::common()
        }
    }

    /// System defaults QoS class
    ///    - History: System default,
    ///    - Depth: System default,
    ///    - Reliability: System default,
    ///    - Durability: System default,
    ///    - Deadline: Default,
    ///    - Lifespan: Default,
    ///    - Liveliness: System default,
    ///    - Liveliness lease duration: Default,
    ///    - Avoid ros namespace conventions: false
    pub const fn system_default() -> Self {
        Self::common()
    }

    /// Unknow QoS class
    ///    - History: Unknown,
    ///    - Depth: System default,
    ///    - Reliability: Unknown,
    ///    - Durability: Unknown,
    ///    - Deadline: Default,
    ///    - Lifespan: Default,
    ///    - Liveliness: Unknown,
    ///    - Liveliness lease duration: Default,
    ///    - Avoid ros namespace conventions: false
    pub const fn unknown() -> Self {
        Self {
            history: HistoryPolicy::Unknown,
            reliability: ReliabilityPolicy::Unknown,
            durability: DurabilityPolicy::Unknown,
            liveliness: LivelinessPolicy::Unknown,
            ..Self::common()
        }
    }

    const fn common() -> Self {
        Self {
            history: HistoryPolicy::SystemDefault,
            depth: rcl_sys::RMW_QOS_POLICY_DEPTH_SYSTEM_DEFAULT as usize,
            reliability: ReliabilityPolicy::SystemDefault,
            durability: DurabilityPolicy::SystemDefault,
            deadline: Duration::ZERO,
            lifespan: Duration::ZERO,
            liveliness: LivelinessPolicy::SystemDefault,
            liveliness_lease_duration: Duration::ZERO,
            avoid_ros_namespace_conventions: false,
        }
    }

    /// Set the history policy.
    ///
    /// # Examples
    ///
    /// ```
    /// # use rclrust::qos::{HistoryPolicy, QoSProfile};
    /// #
    /// let qos = QoSProfile::default().history(HistoryPolicy::KeepAll);
    /// ```
    pub const fn history(self, history: HistoryPolicy) -> Self {
        Self { history, ..self }
    }

    /// Set the history to keep last.
    ///
    /// # Examples
    ///
    /// ```
    /// # use rclrust::qos::QoSProfile;
    /// #
    /// let qos = QoSProfile::default().keep_last(10);
    /// ```
    pub const fn keep_last(self, depth: usize) -> Self {
        Self {
            history: HistoryPolicy::KeepLast,
            depth,
            ..self
        }
    }

    /// Set the history to keep all.
    ///
    /// # Examples
    ///
    /// ```
    /// # use rclrust::qos::QoSProfile;
    /// #
    /// let qos = QoSProfile::default().keep_all();
    /// ```
    pub const fn keep_all(self) -> Self {
        Self {
            history: HistoryPolicy::KeepAll,
            depth: 0,
            ..self
        }
    }

    /// Set the reliability setting.
    ///
    /// # Examples
    ///
    /// ```
    /// # use rclrust::qos::{QoSProfile, ReliabilityPolicy};
    /// #
    /// let qos = QoSProfile::default().reliability(ReliabilityPolicy::Reliable);
    /// ```
    pub const fn reliability(self, reliability: ReliabilityPolicy) -> Self {
        Self {
            reliability,
            ..self
        }
    }

    /// Set the reliability setting to reliable.
    ///
    /// # Examples
    ///
    /// ```
    /// # use rclrust::qos::QoSProfile;
    /// #
    /// let qos = QoSProfile::default().reliable();
    /// ```
    pub const fn reliable(self) -> Self {
        self.reliability(ReliabilityPolicy::Reliable)
    }

    /// Set the reliability setting to best effort.
    ///
    /// # Examples
    ///
    /// ```
    /// # use rclrust::qos::QoSProfile;
    /// #
    /// let qos = QoSProfile::default().best_effort();
    /// ```
    pub const fn best_effort(self) -> Self {
        self.reliability(ReliabilityPolicy::BestEffort)
    }

    /// Set the durability setting.
    ///
    /// # Examples
    ///
    /// ```
    /// # use rclrust::qos::{DurabilityPolicy, QoSProfile};
    /// #
    /// let qos = QoSProfile::default().durability(DurabilityPolicy::Volatile);
    /// ```
    pub const fn durability(self, durability: DurabilityPolicy) -> Self {
        Self { durability, ..self }
    }

    /// Set the durability setting to volatile.
    ///
    /// # Examples
    ///
    /// ```
    /// # use rclrust::qos::QoSProfile;
    /// #
    /// let qos = QoSProfile::default().volatile();
    /// ```
    pub const fn volatile(self) -> Self {
        self.durability(DurabilityPolicy::Volatile)
    }

    /// Set the durability setting to transient local.
    ///
    /// # Examples
    ///
    /// ```
    /// # use rclrust::qos::QoSProfile;
    /// #
    /// let qos = QoSProfile::default().transient_local();
    /// ```
    pub const fn transient_local(self) -> Self {
        self.durability(DurabilityPolicy::TransientLocal)
    }

    /// Set the deadline setting.
    ///
    /// # Examples
    ///
    /// ```
    /// # use std::time::Duration;
    /// #
    /// # use rclrust::qos::QoSProfile;
    /// #
    /// let qos = QoSProfile::default().deadline(Duration::from_secs(5));
    /// ```
    pub const fn deadline(self, deadline: Duration) -> Self {
        Self { deadline, ..self }
    }

    /// Set the lifespan setting.
    ///
    /// # Examples
    ///
    /// ```
    /// # use std::time::Duration;
    /// #
    /// # use rclrust::qos::QoSProfile;
    /// #
    /// let qos = QoSProfile::default().lifespan(Duration::from_secs(5));
    /// ```
    pub const fn lifespan(self, lifespan: Duration) -> Self {
        Self { lifespan, ..self }
    }

    /// Set the liveliness setting.
    ///
    /// # Examples
    ///
    /// ```
    /// # use rclrust::qos::{LivelinessPolicy, QoSProfile};
    /// #
    /// let qos = QoSProfile::default().liveliness(LivelinessPolicy::Automatic);
    /// ```
    pub const fn liveliness(self, liveliness: LivelinessPolicy) -> Self {
        Self { liveliness, ..self }
    }

    /// Set the liveliness_lease_duration setting.
    ///
    /// # Examples
    ///
    /// ```
    /// # use std::time::Duration;
    /// #
    /// # use rclrust::qos::QoSProfile;
    /// #
    /// let qos = QoSProfile::default().liveliness_lease_duration(Duration::from_secs(5));
    /// ```
    pub const fn liveliness_lease_duration(self, liveliness_lease_duration: Duration) -> Self {
        Self {
            liveliness_lease_duration,
            ..self
        }
    }

    /// Set the avoid_ros_namespace_conventions setting.
    ///
    /// # Examples
    ///
    /// ```
    /// # use rclrust::qos::QoSProfile;
    /// #
    /// let qos = QoSProfile::default().avoid_ros_namespace_conventions(true);
    /// ```
    pub const fn avoid_ros_namespace_conventions(
        self,
        avoid_ros_namespace_conventions: bool,
    ) -> Self {
        Self {
            avoid_ros_namespace_conventions,
            ..self
        }
    }
}

impl From<&rcl_sys::rmw_qos_profile_t> for QoSProfile {
    fn from(qos: &rcl_sys::rmw_qos_profile_t) -> Self {
        let (history, depth) = match qos.history {
            rcl_sys::rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_ALL => {
                (HistoryPolicy::KeepAll, 0)
            }
            _ => (HistoryPolicy::KeepLast, qos.depth),
        };

        Self {
            depth,
            history,
            reliability: qos.reliability.into(),
            durability: qos.durability.into(),
            liveliness: qos.liveliness.into(),
            deadline: Duration::from_rmw_time_t(&qos.deadline),
            lifespan: Duration::from_rmw_time_t(&qos.lifespan),
            liveliness_lease_duration: Duration::from_rmw_time_t(&qos.liveliness_lease_duration),
            avoid_ros_namespace_conventions: qos.avoid_ros_namespace_conventions,
        }
    }
}

impl From<&QoSProfile> for rcl_sys::rmw_qos_profile_t {
    fn from(qos: &QoSProfile) -> Self {
        Self {
            history: qos.history.into(),
            depth: qos.depth,
            reliability: qos.reliability.into(),
            durability: qos.durability.into(),
            deadline: qos.deadline.to_rmw_time_t(),
            lifespan: qos.lifespan.to_rmw_time_t(),
            liveliness: qos.liveliness.into(),
            liveliness_lease_duration: qos.liveliness_lease_duration.to_rmw_time_t(),
            avoid_ros_namespace_conventions: qos.avoid_ros_namespace_conventions,
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn reliability_policy_equal() {
        let equal_after_convert = |from: ReliabilityPolicy| {
            assert_eq!(
                from,
                ReliabilityPolicy::from(rcl_sys::rmw_qos_reliability_policy_t::from(from))
            );
        };

        equal_after_convert(ReliabilityPolicy::SystemDefault);
        equal_after_convert(ReliabilityPolicy::Reliable);
        equal_after_convert(ReliabilityPolicy::BestEffort);
        equal_after_convert(ReliabilityPolicy::Unknown);
    }

    #[test]
    fn history_policy_equal() {
        let equal_after_convert = |from: HistoryPolicy| {
            assert_eq!(
                from,
                HistoryPolicy::from(rcl_sys::rmw_qos_history_policy_t::from(from))
            );
        };

        equal_after_convert(HistoryPolicy::SystemDefault);
        equal_after_convert(HistoryPolicy::KeepLast);
        equal_after_convert(HistoryPolicy::KeepAll);
        equal_after_convert(HistoryPolicy::Unknown);
    }

    #[test]
    fn durability_policy_equal() {
        let equal_after_convert = |from: DurabilityPolicy| {
            assert_eq!(
                from,
                DurabilityPolicy::from(rcl_sys::rmw_qos_durability_policy_t::from(from))
            );
        };

        equal_after_convert(DurabilityPolicy::SystemDefault);
        equal_after_convert(DurabilityPolicy::TransientLocal);
        equal_after_convert(DurabilityPolicy::Volatile);
        equal_after_convert(DurabilityPolicy::Unknown);
    }

    #[test]
    fn qos_profile_equal() {
        let equal_after_convert = |from: QoSProfile| {
            assert_eq!(
                from,
                QoSProfile::from(&rcl_sys::rmw_qos_profile_t::from(&from))
            );
        };

        equal_after_convert(QoSProfile::default());
    }

    #[test]
    fn qos_profile_histroy() {
        let policy = HistoryPolicy::KeepAll;

        let qos = QoSProfile::unknown();
        assert_ne!(qos.history, policy);

        let qos = qos.history(policy);
        assert_eq!(qos.history, policy);
    }

    #[test]
    fn qos_profile_keep_last() {
        let policy = HistoryPolicy::KeepLast;
        let depth = 100;

        let qos = QoSProfile::unknown();
        assert_ne!(qos.history, policy);
        assert_ne!(qos.depth, depth);

        let qos = qos.keep_last(depth);
        assert_eq!(qos.history, policy);
        assert_eq!(qos.depth, depth);
    }

    #[test]
    fn qos_profile_keep_all() {
        let policy = HistoryPolicy::KeepAll;
        let depth = 0;

        let qos = QoSProfile::default();
        assert_ne!(qos.history, policy);
        assert_ne!(qos.depth, depth);

        let qos = qos.keep_all();
        assert_eq!(qos.history, policy);
        assert_eq!(qos.depth, depth);
    }

    #[test]
    fn qos_profile_reliability() {
        let policy = ReliabilityPolicy::BestEffort;

        let qos = QoSProfile::unknown();
        assert_ne!(qos.reliability, policy);

        let qos = qos.reliability(policy);
        assert_eq!(qos.reliability, policy);
    }

    #[test]
    fn qos_profile_reliable() {
        let policy = ReliabilityPolicy::Reliable;

        let qos = QoSProfile::unknown();
        assert_ne!(qos.reliability, policy);

        let qos = qos.reliable();
        assert_eq!(qos.reliability, policy);
    }

    #[test]
    fn qos_profile_best_effort() {
        let policy = ReliabilityPolicy::BestEffort;

        let qos = QoSProfile::unknown();
        assert_ne!(qos.reliability, policy);

        let qos = qos.best_effort();
        assert_eq!(qos.reliability, policy);
    }

    #[test]
    fn qos_profile_durability() {
        let policy = DurabilityPolicy::TransientLocal;

        let qos = QoSProfile::unknown();
        assert_ne!(qos.durability, policy);

        let qos = qos.durability(policy);
        assert_eq!(qos.durability, policy);
    }

    #[test]
    fn qos_profile_volatile() {
        let policy = DurabilityPolicy::Volatile;

        let qos = QoSProfile::unknown();
        assert_ne!(qos.durability, policy);

        let qos = qos.volatile();
        assert_eq!(qos.durability, policy);
    }

    #[test]
    fn qos_profile_transient_local() {
        let policy = DurabilityPolicy::TransientLocal;

        let qos = QoSProfile::unknown();
        assert_ne!(qos.durability, policy);

        let qos = qos.transient_local();
        assert_eq!(qos.durability, policy);
    }

    #[test]
    fn qos_profile_deadline() {
        let time = Duration::from_secs(5);

        let qos = QoSProfile::unknown();
        assert_ne!(qos.deadline, time);

        let qos = qos.deadline(time);
        assert_eq!(qos.deadline, time);
    }

    #[test]
    fn qos_profile_lifespan() {
        let time = Duration::from_secs(5);

        let qos = QoSProfile::unknown();
        assert_ne!(qos.lifespan, time);

        let qos = qos.lifespan(time);
        assert_eq!(qos.lifespan, time);
    }

    #[test]
    fn qos_profile_liveliness() {
        let policy = LivelinessPolicy::ManualByTopic;

        let qos = QoSProfile::unknown();
        assert_ne!(qos.liveliness, policy);

        let qos = qos.liveliness(policy);
        assert_eq!(qos.liveliness, policy);
    }

    #[test]
    fn qos_profile_liveliness_lease_duration() {
        let time = Duration::from_secs(5);

        let qos = QoSProfile::unknown();
        assert_ne!(qos.liveliness_lease_duration, time);

        let qos = qos.liveliness_lease_duration(time);
        assert_eq!(qos.liveliness_lease_duration, time);
    }

    #[test]
    fn qos_profile_avoid_ros_namespace_conventions() {
        let flag = true;

        let qos = QoSProfile::unknown();
        assert_ne!(qos.avoid_ros_namespace_conventions, flag);

        let qos = qos.avoid_ros_namespace_conventions(flag);
        assert_eq!(qos.avoid_ros_namespace_conventions, flag);
    }
}
