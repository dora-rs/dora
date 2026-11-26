//! Adapter for the existing `ros2-client`/RustDDS implementation.

use std::{path::Path, sync::Arc, time::Duration};

use super::{
    Durability, History, Liveliness, Reliability, Ros2Qos, TransportError,
    diagnostics::DdsDiagnostics,
    interfaces::{self, INTERFACES_ENV},
};
use crate::rustdds;

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
    diagnostics: Arc<DdsDiagnostics>,
}

impl Context {
    /// Open a DDS ROS2 context in the requested ROS domain, advertising only
    /// the addresses in [`INTERFACES_ENV`] when it is set.
    pub fn new(domain_id: u16) -> Result<Self, TransportError> {
        Self::new_with_security(domain_id, None)
    }

    /// Open a DDS ROS2 context with optional DDS security configuration.
    pub fn new_with_security(
        domain_id: u16,
        security_dir: Option<&Path>,
    ) -> Result<Self, TransportError> {
        let interfaces = match std::env::var(INTERFACES_ENV) {
            Ok(value) => Some(value),
            Err(std::env::VarError::NotPresent) => None,
            Err(std::env::VarError::NotUnicode(_)) => {
                return Err(TransportError::DdsContext {
                    message: format!("{INTERFACES_ENV} is not valid UTF-8"),
                });
            }
        };
        Self::with_interfaces_and_security(domain_id, interfaces.as_deref(), security_dir)
    }

    /// [`Context::new`] with the value of [`INTERFACES_ENV`] passed in: only
    /// those local addresses are advertised and used, or all when `None`.
    #[allow(dead_code)]
    pub(crate) fn with_interfaces(
        domain_id: u16,
        value: Option<&str>,
    ) -> Result<Self, TransportError> {
        Self::with_interfaces_and_security(domain_id, value, None)
    }

    pub(crate) fn with_interfaces_and_security(
        domain_id: u16,
        value: Option<&str>,
        security_dir: Option<&Path>,
    ) -> Result<Self, TransportError> {
        super::diagnostics::report_ros_distro();
        let requested =
            interfaces::parse(value).map_err(|message| TransportError::DdsContext { message })?;
        // What `ros2_client::Context::with_options` does, plus the network
        // filter, which its `ContextOptions` does not expose.
        let mut builder = rustdds::DomainParticipantBuilder::new(domain_id);
        let local = interfaces::local_addrs();
        let unrestricted = if let Some(requested) = &requested {
            interfaces::check(requested, &local)
                .map_err(|message| TransportError::DdsContext { message })?;
            interfaces::report_once(requested, &local);
            builder = builder.with_only_networks(requested.iter().copied());
            None
        } else {
            interfaces::Unrestricted::new(&local)
        };

        let builder = if let Some(sec_dir) = security_dir {
            Self::apply_security(builder, sec_dir)?
        } else {
            builder
        };

        fn context_error(error: impl std::fmt::Debug) -> TransportError {
            TransportError::DdsContext {
                message: format!("{error:?}"),
            }
        }
        let participant = builder.build().map_err(context_error)?;
        let inner =
            ros2_client::Context::from_domain_participant(participant).map_err(context_error)?;
        let diagnostics = Arc::new(DdsDiagnostics::start(
            inner.domain_participant(),
            detect_service_mapping(),
            unrestricted,
        ));
        Ok(Self { inner, diagnostics })
    }

    /// Validate that a DDS security directory exists and contains the required
    /// ROS 2 SROS2 credentials.
    pub fn validate_security_dir(sec_dir: &Path) -> Result<(), TransportError> {
        if !sec_dir.is_dir() {
            return Err(TransportError::DdsContext {
                message: format!(
                    "DDS Security directory does not exist or is not a directory: {}",
                    sec_dir.display()
                ),
            });
        }

        let required_files = [
            "identity_ca.cert.pem",
            "cert.pem",
            "key.pem",
            "permissions_ca.cert.pem",
            "governance.p7s",
            "permissions.p7s",
        ];

        let mut missing = Vec::new();
        for file in required_files {
            if !sec_dir.join(file).is_file() {
                missing.push(file);
            }
        }

        if !missing.is_empty() {
            return Err(TransportError::DdsContext {
                message: format!(
                    "DDS Security credential directory `{}` is missing required file(s): {}",
                    sec_dir.display(),
                    missing.join(", ")
                ),
            });
        }

        Ok(())
    }

    #[cfg(feature = "security")]
    fn apply_security(
        builder: rustdds::DomainParticipantBuilder,
        sec_dir: &Path,
    ) -> Result<rustdds::DomainParticipantBuilder, TransportError> {
        Self::validate_security_dir(sec_dir)?;
        let configs = rustdds::DomainParticipantSecurityConfigFiles::with_ros_default_names(
            sec_dir,
            String::new(),
        );
        Ok(builder.builtin_security(configs))
    }

    #[cfg(not(feature = "security"))]
    fn apply_security(
        _builder: rustdds::DomainParticipantBuilder,
        sec_dir: &Path,
    ) -> Result<rustdds::DomainParticipantBuilder, TransportError> {
        Self::validate_security_dir(sec_dir)?;
        Err(TransportError::DdsContext {
            message: format!(
                "DDS Security was requested with credentials at `{}`, but this build of \
                 dora-ros2-bridge was compiled without the 'security' feature",
                sec_dir.display()
            ),
        })
    }

    /// Create an existing DDS ROS2 node.
    pub fn new_node(
        &self,
        node_name: ros2_client::NodeName,
        options: ros2_client::NodeOptions,
    ) -> Result<Node, TransportError> {
        self.inner
            .new_node(node_name.clone(), options)
            .map(|inner| Node {
                inner,
                name: node_name,
                diagnostics: self.diagnostics.clone(),
            })
            .map_err(|error| TransportError::DdsNode {
                message: format!("{error:?}"),
            })
    }

    /// Borrow the concrete context for legacy DDS-only call sites.
    pub fn as_inner(&self) -> &ros2_client::Context {
        &self.inner
    }

    /// The watcher for QoS and RMW-mapping mismatches with ROS 2 peers, for
    /// endpoints created on the concrete context.
    pub fn diagnostics(&self) -> &Arc<DdsDiagnostics> {
        &self.diagnostics
    }

    /// Consume the adapter and return the concrete context.
    pub fn into_inner(self) -> ros2_client::Context {
        self.inner
    }
}

/// Existing DDS ROS2 node.
pub struct Node {
    inner: ros2_client::Node,
    name: ros2_client::NodeName,
    diagnostics: Arc<DdsDiagnostics>,
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

    /// The node's name, which places its relative service and action names.
    pub fn name(&self) -> &ros2_client::NodeName {
        &self.name
    }

    /// The context's watcher for QoS and RMW-mapping mismatches: register each
    /// endpoint created on the concrete node.
    pub fn diagnostics(&self) -> &DdsDiagnostics {
        &self.diagnostics
    }

    /// Mutably borrow the concrete node for legacy DDS-only call sites.
    pub fn as_inner_mut(&mut self) -> &mut ros2_client::Node {
        &mut self.inner
    }
}

/// Durations this long or longer mean "infinite". An RTPS duration holds its
/// seconds in an `i32`, and RustDDS converts into it with a wrapping cast, so
/// a longer blocking time or lease must not reach it as a finite value.
const INFINITE: Duration = Duration::from_secs(i32::MAX as u64);

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

#[cfg(test)]
mod tests {
    use super::*;
    use std::{
        net::{IpAddr, Ipv4Addr, SocketAddr},
        time::Instant,
    };

    /// A domain no other test in this crate uses.
    const DOMAIN_ID: u16 = 89;

    #[derive(serde::Deserialize)]
    struct Probe {}

    #[test]
    fn an_address_this_host_lacks_fails_naming_the_variable() {
        let Err(TransportError::DdsContext { message }) =
            Context::with_interfaces(DOMAIN_ID, Some("192.0.2.1"))
        else {
            panic!("expected a DdsContext error");
        };
        assert!(message.contains(INTERFACES_ENV), "{message}");
        assert!(message.contains("192.0.2.1"), "{message}");
    }

    #[test]
    fn a_restricted_context_advertises_only_the_given_address() {
        let restricted = Context::with_interfaces(DOMAIN_ID, Some("127.0.0.1")).unwrap();
        let mut node = restricted
            .new_node(
                ros2_client::NodeName::new("/", "restricted").unwrap(),
                ros2_client::NodeOptions::new(),
            )
            .unwrap();
        let topic = node
            .as_inner_mut()
            .create_topic(
                &ros2_client::Name::new("/", "interfaces_probe").unwrap(),
                ros2_client::MessageTypeName::new("std_msgs", "Empty"),
                &ros2_client::QosProfile::subscription_default(),
            )
            .unwrap();
        let _reader = node
            .as_inner_mut()
            .create_subscription::<Probe>(&topic, None)
            .unwrap();

        let observer = Context::with_interfaces(DOMAIN_ID, None).unwrap();
        let deadline = Instant::now() + Duration::from_secs(20);
        let locators = loop {
            let found = observer
                .as_inner()
                .domain_participant()
                .discovered_readers()
                .into_iter()
                .find(|r| {
                    r.subscription_topic_data
                        .topic_name()
                        .ends_with("interfaces_probe")
                });
            if let Some(reader) = found {
                break reader.reader_proxy.unicast_locator_list;
            }
            assert!(
                Instant::now() < deadline,
                "the restricted reader was never discovered"
            );
            std::thread::sleep(Duration::from_millis(100));
        };
        let ips: Vec<IpAddr> = locators
            .into_iter()
            .map(|l| SocketAddr::from(l).ip())
            .collect();
        assert!(!ips.is_empty());
        assert!(
            ips.iter().all(|ip| *ip == IpAddr::V4(Ipv4Addr::LOCALHOST)),
            "{ips:?}"
        );
    }

    #[test]
    fn security_dir_must_exist() {
        let nonexistent = Path::new("/nonexistent/security/dir");
        let err = Context::validate_security_dir(nonexistent).unwrap_err();
        let TransportError::DdsContext { message } = err else {
            panic!("expected DdsContext error, got {err:?}");
        };
        assert!(message.contains("does not exist"), "{message}");
    }

    #[test]
    fn security_dir_missing_files_are_reported() {
        let temp =
            std::env::temp_dir().join(format!("test_dds_sec_missing_{}", std::process::id()));
        std::fs::create_dir_all(&temp).unwrap();
        // create only one file
        std::fs::write(temp.join("cert.pem"), b"dummy").unwrap();

        let err = Context::validate_security_dir(&temp).unwrap_err();
        let TransportError::DdsContext { message } = err else {
            panic!("expected DdsContext error, got {err:?}");
        };
        assert!(message.contains("missing required file(s)"), "{message}");
        assert!(message.contains("identity_ca.cert.pem"), "{message}");
        assert!(message.contains("governance.p7s"), "{message}");

        let _ = std::fs::remove_dir_all(&temp);
    }

    #[test]
    fn security_validation_passes_when_all_files_present() {
        let temp = std::env::temp_dir().join(format!("test_dds_sec_valid_{}", std::process::id()));
        std::fs::create_dir_all(&temp).unwrap();
        for file in [
            "identity_ca.cert.pem",
            "cert.pem",
            "key.pem",
            "permissions_ca.cert.pem",
            "governance.p7s",
            "permissions.p7s",
        ] {
            std::fs::write(temp.join(file), b"dummy").unwrap();
        }

        assert!(Context::validate_security_dir(&temp).is_ok());

        #[cfg(not(feature = "security"))]
        {
            let Err(err) = Context::new_with_security(DOMAIN_ID, Some(&temp)) else {
                panic!("expected new_with_security to fail without the 'security' feature");
            };
            let TransportError::DdsContext { message } = err else {
                panic!("expected DdsContext error, got {err:?}");
            };
            assert!(
                message.contains("without the 'security' feature"),
                "{message}"
            );
        }

        let _ = std::fs::remove_dir_all(&temp);
    }
}
