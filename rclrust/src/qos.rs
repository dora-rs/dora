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
    ManualByNode := RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_NODE,
    ManualByTopic := RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC,
    Unknown := RMW_QOS_POLICY_LIVELINESS_UNKNOWN,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PolicyKind {
    Invalid,
    Durability,
    Deadline,
    Liveliness,
    Reliability,
    History,
    Lifespan,
}

impl_from_trait_for_enum! {
    PolicyKind,
    rcl_sys::rmw_qos_policy_kind_t,
    Invalid := RMW_QOS_POLICY_INVALID,
    Durability := RMW_QOS_POLICY_DURABILITY,
    Deadline := RMW_QOS_POLICY_DEADLINE,
    Liveliness := RMW_QOS_POLICY_LIVELINESS,
    Reliability := RMW_QOS_POLICY_RELIABILITY,
    History := RMW_QOS_POLICY_HISTORY,
    Lifespan := RMW_QOS_POLICY_LIFESPAN,
}

impl From<PolicyKind> for String {
    fn from(policy_kind: PolicyKind) -> Self {
        use PolicyKind::*;
        match policy_kind {
            Durability => "DURABILITY_QOS_POLICY".to_string(),
            Deadline => "DEADLINE_QOS_POLICY".to_string(),
            Liveliness => "LIVELINESS_QOS_POLICY".to_string(),
            Reliability => "RELIABILITY_QOS_POLICY".to_string(),
            History => "HISTORY_QOS_POLICY".to_string(),
            Lifespan => "LIFESPAN_QOS_POLICY".to_string(),
            _ => "INVALID_QOS_POLICY".to_string(),
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct QoSProfile {
    history: HistoryPolicy,
    depth: usize,
    reliability: ReliabilityPolicy,
    durability: DurabilityPolicy,
    deadline: Duration,
    lifespan: Duration,
    liveliness: LivelinessPolicy,
    liveliness_lease_duration: Duration,
    avoid_ros_namespace_conventions: bool,
}

impl QoSProfile {
    /// Set the history policy.
    ///
    /// # Examples
    ///
    /// ```
    /// use rclrust::qos::{HistoryPolicy, QoSProfile};
    ///
    /// let qos = QoSProfile::system_default().history(HistoryPolicy::KeepAll);
    /// ```
    pub fn history(&mut self, history: HistoryPolicy) -> &mut Self {
        self.history = history;
        self
    }

    /// Set the history to keep last.
    ///
    /// # Examples
    ///
    /// ```
    /// use rclrust::qos::QoSProfile;
    ///
    /// let qos = QoSProfile::system_default().keep_last(10);
    /// ```
    pub fn keep_last(&mut self, depth: usize) -> &mut Self {
        self.history = HistoryPolicy::KeepLast;
        self.depth = depth;
        self
    }

    /// Set the history to keep all.
    ///
    /// # Examples
    ///
    /// ```
    /// use rclrust::qos::QoSProfile;
    ///
    /// let qos = QoSProfile::system_default().keep_all();
    /// ```
    pub fn keep_all(&mut self) -> &mut Self {
        self.history = HistoryPolicy::KeepAll;
        self.depth = 0;
        self
    }

    /// Set the reliability setting.
    ///
    /// # Examples
    ///
    /// ```
    /// use rclrust::qos::{QoSProfile, ReliabilityPolicy};
    ///
    /// let qos = QoSProfile::system_default().reliability(ReliabilityPolicy::Reliable);
    /// ```
    pub fn reliability(&mut self, reliablity: ReliabilityPolicy) -> &mut Self {
        self.reliability = reliablity;
        self
    }

    /// Set the reliability setting to reliable.
    ///
    /// # Examples
    ///
    /// ```
    /// use rclrust::qos::QoSProfile;
    ///
    /// let qos = QoSProfile::system_default().reliable();
    /// ```
    pub fn reliable(&mut self) -> &mut Self {
        self.reliability(ReliabilityPolicy::Reliable)
    }

    /// Set the reliability setting to best effort.
    ///
    /// # Examples
    ///
    /// ```
    /// use rclrust::qos::QoSProfile;
    ///
    /// let qos = QoSProfile::system_default().best_effort();
    /// ```
    pub fn best_effort(&mut self) -> &mut Self {
        self.reliability(ReliabilityPolicy::BestEffort)
    }

    /// Set the durability setting.
    ///
    /// # Examples
    ///
    /// ```
    /// use rclrust::qos::{DurabilityPolicy, QoSProfile};
    ///
    /// let qos = QoSProfile::system_default().durability(DurabilityPolicy::Volatile);
    /// ```
    pub fn durability(&mut self, durability: DurabilityPolicy) -> &mut Self {
        self.durability = durability;
        self
    }

    /// Set the durability setting to volatile.
    ///
    /// # Examples
    ///
    /// ```
    /// use rclrust::qos::QoSProfile;
    ///
    /// let qos = QoSProfile::system_default().volatile();
    /// ```
    pub fn volatile(&mut self) -> &mut Self {
        self.durability(DurabilityPolicy::Volatile)
    }

    /// Set the durability setting to transient local.
    ///
    /// # Examples
    ///
    /// ```
    /// use rclrust::qos::QoSProfile;
    ///
    /// let qos = QoSProfile::system_default().transient_local();
    /// ```
    pub fn transient_local(&mut self) -> &mut Self {
        self.durability(DurabilityPolicy::TransientLocal)
    }

    /// Set the deadline setting.
    ///
    /// # Examples
    ///
    /// ```
    /// use std::time::Duration;
    ///
    /// use rclrust::qos::QoSProfile;
    ///
    /// let qos = QoSProfile::system_default().deadline(Duration::new(5, 0));
    /// ```
    pub fn deadline(&mut self, deadline: Duration) -> &mut Self {
        self.deadline = deadline;
        self
    }

    /// Set the lifespan setting.
    ///
    /// # Examples
    ///
    /// ```
    /// use std::time::Duration;
    ///
    /// use rclrust::qos::QoSProfile;
    ///
    /// let qos = QoSProfile::system_default().lifespan(Duration::new(5, 0));
    /// ```
    pub fn lifespan(&mut self, lifespan: Duration) -> &mut Self {
        self.lifespan = lifespan;
        self
    }

    /// Set the liveliness setting.
    ///
    /// # Examples
    ///
    /// ```
    /// use rclrust::qos::{LivelinessPolicy, QoSProfile};
    ///
    /// let qos = QoSProfile::system_default().liveliness(LivelinessPolicy::Automatic);
    /// ```
    pub fn liveliness(&mut self, liveliness: LivelinessPolicy) -> &mut Self {
        self.liveliness = liveliness;
        self
    }

    /// Set the liveliness_lease_duration setting.
    ///
    /// # Examples
    ///
    /// ```
    /// use std::time::Duration;
    ///
    /// use rclrust::qos::QoSProfile;
    ///
    /// let qos = QoSProfile::system_default().liveliness_lease_duration(Duration::new(5, 0));
    /// ```
    pub fn liveliness_lease_duration(&mut self, liveliness_lease_duration: Duration) -> &mut Self {
        self.liveliness_lease_duration = liveliness_lease_duration;
        self
    }

    /// Set the avoid_ros_namespace_conventions setting.
    ///
    /// # Examples
    ///
    /// ```
    /// use rclrust::qos::QoSProfile;
    ///
    /// let qos = QoSProfile::system_default().avoid_ros_namespace_conventions(true);
    /// ```
    pub fn avoid_ros_namespace_conventions(
        &mut self,
        avoid_ros_namespace_conventions: bool,
    ) -> &mut Self {
        self.avoid_ros_namespace_conventions = avoid_ros_namespace_conventions;
        self
    }

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
    ///
    /// # Examples
    ///
    /// ```
    /// use rclrust::qos::QoSProfile;
    ///
    /// let qos = QoSProfile::sensor_data();
    /// ```
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
    ///
    /// # Examples
    ///
    /// ```
    /// use rclrust::qos::QoSProfile;
    ///
    /// let qos = QoSProfile::parameters();
    /// ```
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
    ///
    /// # Examples
    ///
    /// ```
    /// use rclrust::qos::QoSProfile;
    ///
    /// let qos = QoSProfile::default();
    /// ```
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
    ///
    /// # Examples
    ///
    /// ```
    /// use rclrust::qos::QoSProfile;
    ///
    /// let qos = QoSProfile::services_default();
    /// ```
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
    ///
    /// # Examples
    ///
    /// ```
    /// use rclrust::qos::QoSProfile;
    ///
    /// let qos = QoSProfile::parameter_events();
    /// ```
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
    ///
    /// # Examples
    ///
    /// ```
    /// use rclrust::qos::QoSProfile;
    ///
    /// let qos = QoSProfile::system_default();
    /// ```
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
    ///
    /// # Examples
    ///
    /// ```
    /// use rclrust::qos::QoSProfile;
    ///
    /// let qos = QoSProfile::unknown();
    /// ```
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
