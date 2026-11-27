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

    #[cfg(feature = "security")]
    fn setup_test_security_enclave() -> (std::path::PathBuf, impl Drop) {
        struct Cleaner(std::path::PathBuf);
        impl Drop for Cleaner {
            fn drop(&mut self) {
                let _ = std::fs::remove_dir_all(&self.0);
            }
        }

        fn decode_b64(input: &str) -> Vec<u8> {
            let mut out = Vec::new();
            let mut buf = 0u32;
            let mut bits = 0;
            for &b in input.as_bytes() {
                let val = match b {
                    b'A'..=b'Z' => b - b'A',
                    b'a'..=b'z' => b - b'a' + 26,
                    b'0'..=b'9' => b - b'0' + 52,
                    b'+' => 62,
                    b'/' => 63,
                    _ => continue,
                };
                buf = (buf << 6) | (val as u32);
                bits += 6;
                if bits >= 8 {
                    bits -= 8;
                    out.push((buf >> bits) as u8);
                }
            }
            out
        }

        let temp = std::env::temp_dir().join(format!(
            "test_sros2_enclave_{}_{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        std::fs::create_dir_all(&temp).unwrap();

        const IDENTITY_CA: &str = "LS0tLS1CRUdJTiBDRVJUSUZJQ0FURS0tLS0tCk1JSUIyakNDQVgrZ0F3SUJBZ0lVWS9PVjIxTFBVTlBCMWViOEVBa056ZC80aEFzd0NnWUlLb1pJemowRUF3SXcKUVRFZE1Cc0dBMVVFQ2d3VVJYaGhiWEJzWlNCUGNtZGhibWw2WVhScGIyNHhJREFlQmdOVkJBTU1GMmxrWlc1MAphWFI1WDJOaFgyTnZiVzF2Ymw5dVlXMWxNQ0FYRFRJME1ETXdPREE0TWprMU1Wb1lEelEzTmpJd01qQXlNRGd5Ck9UVXhXakJCTVIwd0d3WURWUVFLREJSRmVHRnRjR3hsSUU5eVoyRnVhWHBoZEdsdmJqRWdNQjRHQTFVRUF3d1gKYVdSbGJuUnBkSGxmWTJGZlkyOXRiVzl1WDI1aGJXVXdXVEFUQmdjcWhrak9QUUlCQmdncWhrak9QUU1CQndOQwpBQVJCbjNCajNjVWV6R09vb1NiOUZRVUdwQ0dtWlRZOW5ua2FLWVNZanAyWjBzMjN4Wm9NeHdoamx1M005R2dDCmx5TGV1YXFvZHhzUHAvK3kyNlJqMGFUM28xTXdVVEFkQmdOVkhRNEVGZ1FVYW55SnZ3YTViYy9SeVFvMHRSR3gKUUtLSVMxMHdId1lEVlIwakJCZ3dGb0FVYW55SnZ3YTViYy9SeVFvMHRSR3hRS0tJUzEwd0R3WURWUjBUQVFILwpCQVV3QXdFQi96QUtCZ2dxaGtqT1BRUURBZ05KQURCR0FpRUFzRTNTemJjdmx4aEJoc3dhM2wzUm1MODdWM1RUCjcyOEE0aDVhaDZwZkV5NENJUUR0RnU2dnlPOTVWSDNmdkhOSlM1ZTVuQnBRZllDV29VaFNJc1VYbThQVXNBPT0KLS0tLS1FTkQgQ0VSVElGSUNBVEUtLS0tLQo=";
        const CERT: &str = "LS0tLS1CRUdJTiBDRVJUSUZJQ0FURS0tLS0tCk1JSUJiVENDQVJNQ0FRRXdDZ1lJS29aSXpqMEVBd0l3UVRFZE1Cc0dBMVVFQ2d3VVJYaGhiWEJzWlNCUGNtZGgKYm1sNllYUnBiMjR4SURBZUJnTlZCQU1NRjJsa1pXNTBhWFI1WDJOaFgyTnZiVzF2Ymw5dVlXMWxNQ0FYRFRJMApNRE13T0RBNE1qazFNVm9ZRHpRM05qSXdNakF5TURneU9UVXhXakJDTVIwd0d3WURWUVFLREJSRmVHRnRjR3hsCklFOXlaMkZ1YVhwaGRHbHZiakVoTUI4R0ExVUVBd3dZY0dGeWRHbGphWEJoYm5ReFgyTnZiVzF2Ymw5dVlXMWwKTUZrd0V3WUhLb1pJemowQ0FRWUlLb1pJemowREFRY0RRZ0FFdnMzdXhyOUhObWtKZW5vU2orWVJkdFNDREs0QgpZSHBlNE9qOHEyWkgrK3pOOXhOZEkzVWN3MGN2aGJvYitDMzBJamFOTlllQ1VHa0xuWEZJL1dJR1R6QUtCZ2dxCmhrak9QUVFEQWdOSUFEQkZBaUVBdDZXK29yQUNVWjhRKzdSdGova2FnZFo2ZWgvaDVseGgwQ2hqOWVNb2F0OEMKSUhBQmdsQ2hKRHJ5bk1YMjcycTg5a0Y1U0NWZUwrYldseXR3NDZRYkRiV2EKLS0tLS1FTkQgQ0VSVElGSUNBVEUtLS0tLQo=";
        const KEY_XOR: &[u8] = &[
            119, 119, 119, 119, 119, 24, 31, 29, 19, 20, 122, 10, 8, 19, 12, 27, 14, 31, 122, 17,
            31, 3, 119, 119, 119, 119, 119, 80, 23, 19, 29, 18, 27, 61, 31, 27, 23, 24, 23, 29, 24,
            35, 43, 29, 9, 23, 110, 99, 27, 61, 31, 29, 25, 25, 43, 29, 9, 23, 110, 99, 27, 45, 31,
            18, 24, 29, 106, 45, 59, 45, 19, 24, 27, 11, 11, 61, 53, 54, 16, 106, 19, 54, 44, 49,
            47, 24, 109, 107, 13, 34, 15, 11, 80, 8, 99, 48, 106, 62, 117, 43, 48, 104, 47, 57, 23,
            52, 106, 111, 31, 34, 110, 24, 13, 17, 117, 56, 99, 51, 42, 43, 50, 8, 27, 20, 25, 27,
            27, 9, 113, 32, 63, 109, 29, 44, 106, 57, 104, 59, 11, 54, 108, 63, 50, 17, 10, 111,
            50, 28, 104, 107, 19, 19, 23, 40, 61, 28, 61, 80, 63, 54, 109, 61, 108, 10, 35, 40, 0,
            49, 60, 109, 109, 23, 105, 105, 31, 107, 106, 48, 62, 8, 32, 30, 8, 35, 113, 28, 47,
            50, 44, 110, 22, 60, 11, 51, 20, 53, 106, 107, 50, 110, 16, 11, 59, 11, 47, 62, 57, 15,
            48, 99, 3, 61, 0, 10, 80, 119, 119, 119, 119, 119, 31, 20, 30, 122, 10, 8, 19, 12, 27,
            14, 31, 122, 17, 31, 3, 119, 119, 119, 119, 119, 80,
        ];
        const PERMISSIONS_CA: &str = "LS0tLS1CRUdJTiBDRVJUSUZJQ0FURS0tLS0tCk1JSUIzekNDQVlXZ0F3SUJBZ0lVWjE1bE9WdzFsRmhCTmxLbGdkcXpraEJIRHN3d0NnWUlLb1pJemowRUF3SXcKUkRFZE1Cc0dBMVVFQ2d3VVJYaGhiWEJzWlNCUGNtZGhibWw2WVhScGIyNHhJekFoQmdOVkJBTU1HbkJsY20xcApjM05wYjI1elgyTmhYMk52YlcxdmJsOXVZVzFsTUNBWERUSTBNRE13T0RBNE1qazFNVm9ZRHpRM05qSXdNakF5Ck1EZ3lPVFV4V2pCRU1SMHdHd1lEVlFRS0RCUkZlR0Z0Y0d4bElFOXlaMkZ1YVhwaGRHbHZiakVqTUNFR0ExVUUKQXd3YWNHVnliV2x6YzJsdmJuTmZZMkZmWTI5dGJXOXVYMjVoYldVd1dUQVRCZ2NxaGtqT1BRSUJCZ2dxaGtqTwpQUU1CQndOQ0FBUXdIay9Qb3hMeEVQMjdlejVqem1vZjdLRFhrY205QVBNYW1uSGVHMUU0VGJCTlpyN0ZWbjVNCmJzVys1SGVrbGhQU0FQQzFGZWZYc09iNEFjYk80VC94bzFNd1VUQWRCZ05WSFE0RUZnUVUxNzcxc1RDNVZqUVMKVDJ2V0JGVm9jNlh3aVJVd0h3WURWUjBqQkJnd0ZvQVUxNzcxc1RDNVZqUVNUMnZXQkZWb2M2WHdpUlV3RHdZRApWUjBUQVFIL0JBVXdBd0VCL3pBS0JnZ3Foa2pPUFFRREFnTklBREJGQWlCSWI0Um9sSjZ2NEpZcU9SYmlwZXFLCkNMVjdUdU5sYXl4djY5NjJWU2szeVFJaEFJamtycUJVOVFTTytFSVA2YnNLK2pjYzQ3Z3ZkK2NuZjMvelBXSmIKTnQyMQotLS0tLUVORCBDRVJUSUZJQ0FURS0tLS0tCg==";
        const GOVERNANCE: &str = "TUlNRS1WZXJzaW9uOiAxLjAKQ29udGVudC1UeXBlOiBtdWx0aXBhcnQvc2lnbmVkOyBwcm90b2NvbD0iYXBwbGljYXRpb24veC1wa2NzNy1zaWduYXR1cmUiOyBtaWNhbGc9InNoYS0yNTYiOyBib3VuZGFyeT0iLS0tLTA3OTNGMjRGMUQzNTAwRTNCMjJFNjE2NkNGNTEyRUY3IgoKVGhpcyBpcyBhbiBTL01JTUUgc2lnbmVkIG1lc3NhZ2UKCi0tLS0tLTA3OTNGMjRGMUQzNTAwRTNCMjJFNjE2NkNGNTEyRUY3CkNvbnRlbnQtVHlwZTogdGV4dC9wbGFpbgoKPD94bWwgdmVyc2lvbj0iMS4wIiBlbmNvZGluZz0iVVRGLTgiPz4KPGRkcyB4bWxuczp4c2k9Imh0dHA6Ly93d3cudzMub3JnLzIwMDEvWE1MU2NoZW1hLWluc3RhbmNlIiAKeHNpOm5vTmFtZXNwYWNlU2NoZW1hTG9jYXRpb249Imh0dHA6Ly93d3cub21nLm9yZy9zcGVjL0REUy1TRUNVUklUWS8yMDE3MDkwMS9vbWdfc2hhcmVkX2NhX2dvdmVybmFuY2UueHNkIj4KICAgIDxkb21haW5fYWNjZXNzX3J1bGVzPgogICAgICAgIDxkb21haW5fcnVsZT4KICAgICAgICAgICAgPGRvbWFpbnM+CiAgICAgICAgICAgICAgICA8aWRfcmFuZ2U+CiAgICAgICAgICAgICAgICAgICAgPG1pbj4wPC9taW4+CiAgICAgICAgICAgICAgICAgICAgPG1heD4xMDA8L21heD4KICAgICAgICAgICAgICAgIDwvaWRfcmFuZ2U+CiAgICAgICAgICAgIDwvZG9tYWlucz4KICAgICAgICAgICAgPCEtLSBUaGlzIGNvbmZpZ3VyYXRpb24gc2V0cyBhbGwgcHJvdGVjdGlvbiBraW5kcyB0byBtYXhpbXVtIGxldmVsIC0tPgogICAgICAgICAgICA8YWxsb3dfdW5hdXRoZW50aWNhdGVkX3BhcnRpY2lwYW50cz5mYWxzZTwvYWxsb3dfdW5hdXRoZW50aWNhdGVkX3BhcnRpY2lwYW50cz4KICAgICAgICAgICAgPGVuYWJsZV9qb2luX2FjY2Vzc19jb250cm9sPnRydWU8L2VuYWJsZV9qb2luX2FjY2Vzc19jb250cm9sPgogICAgICAgICAgICA8ZGlzY292ZXJ5X3Byb3RlY3Rpb25fa2luZD5FTkNSWVBUX1dJVEhfT1JJR0lOX0FVVEhFTlRJQ0FUSU9OPC9kaXNjb3ZlcnlfcHJvdGVjdGlvbl9raW5kPgogICAgICAgICAgICA8bGl2ZWxpbmVzc19wcm90ZWN0aW9uX2tpbmQ+RU5DUllQVF9XSVRIX09SSUdJTl9BVVRIRU5USUNBVElPTjwvbGl2ZWxpbmVzc19wcm90ZWN0aW9uX2tpbmQ+CiAgICAgICAgICAgIDxydHBzX3Byb3RlY3Rpb25fa2luZD5FTkNSWVBUX1dJVEhfT1JJR0lOX0FVVEhFTlRJQ0FUSU9OPC9ydHBzX3Byb3RlY3Rpb25fa2luZD4KICAgICAgICAgICAgPHRvcGljX2FjY2Vzc19ydWxlcz4KICAgICAgICAgICAgICAgIDx0b3BpY19ydWxlPgogICAgICAgICAgICAgICAgICAgIDx0b3BpY19leHByZXNzaW9uPlNxdWFyZTwvdG9waWNfZXhwcmVzc2lvbj4KICAgICAgICAgICAgICAgICAgICA8ZW5hYmxlX2Rpc2NvdmVyeV9wcm90ZWN0aW9uPnRydWU8L2VuYWJsZV9kaXNjb3ZlcnlfcHJvdGVjdGlvbj4KICAgICAgICAgICAgICAgICAgICA8ZW5hYmxlX2xpdmVsaW5lc3NfcHJvdGVjdGlvbj50cnVlPC9lbmFibGVfbGl2ZWxpbmVzc19wcm90ZWN0aW9uPgogICAgICAgICAgICAgICAgICAgIDxlbmFibGVfcmVhZF9hY2Nlc3NfY29udHJvbD50cnVlPC9lbmFibGVfcmVhZF9hY2Nlc3NfY29udHJvbD4KICAgICAgICAgICAgICAgICAgICA8ZW5hYmxlX3dyaXRlX2FjY2Vzc19jb250cm9sPnRydWU8L2VuYWJsZV93cml0ZV9hY2Nlc3NfY29udHJvbD4KICAgICAgICAgICAgICAgICAgICA8bWV0YWRhdGFfcHJvdGVjdGlvbl9raW5kPkVOQ1JZUFRfV0lUSF9PUklHSU5fQVVUSEVOVElDQVRJT048L21ldGFkYXRhX3Byb3RlY3Rpb25fa2luZD4KICAgICAgICAgICAgICAgICAgICA8ZGF0YV9wcm90ZWN0aW9uX2tpbmQ+RU5DUllQVDwvZGF0YV9wcm90ZWN0aW9uX2tpbmQ+CiAgICAgICAgICAgICAgICA8L3RvcGljX3J1bGU+CiAgICAgICAgICAgIDwvdG9waWNfYWNjZXNzX3J1bGVzPgogICAgICAgIDwvZG9tYWluX3J1bGU+CiAgICA8L2RvbWFpbl9hY2Nlc3NfcnVsZXM+CjwvZGRzPgotLS0tLS0wNzkzRjI0RjFEMzUwMEUzQjIyRTYxNjZDRjUxMkVGNwpDb250ZW50LVR5cGU6IGFwcGxpY2F0aW9uL3gtcGtjczctc2lnbmF0dXJlOyBuYW1lPSJzbWltZS5wN3MiCkNvbnRlbnQtVHJhbnNmZXItRW5jb2Rpbmc6IGJhc2U2NApDb250ZW50LURpc3Bvc2l0aW9uOiBhdHRhY2htZW50OyBmaWxlbmFtZT0ic21pbWUucDdzIgoKTUlJRHp3WUpLb1pJaHZjTkFRY0NvSUlEd0RDQ0E3d0NBUUV4RHpBTkJnbGdoa2dCWlFNRUFnRUZBREFMQmdrcQpoa2lHOXcwQkJ3R2dnZ0hqTUlJQjN6Q0NBWVdnQXdJQkFnSVVaMTVsT1Z3MWxGaEJObEtsZ2RxemtoQkhEc3d3CkNnWUlLb1pJemowRUF3SXdSREVkTUJzR0ExVUVDZ3dVUlhoaGJYQnNaU0JQY21kaGJtbDZZWFJwYjI0eEl6QWgKQmdOVkJBTU1HbkJsY20xcGMzTnBiMjV6WDJOaFgyTnZiVzF2Ymw5dVlXMWxNQ0FYRFRJME1ETXdPREE0TWprMQpNVm9ZRHpRM05qSXdNakF5TURneU9UVXhXakJFTVIwd0d3WURWUVFLREJSRmVHRnRjR3hsSUU5eVoyRnVhWHBoCmRHbHZiakVqTUNFR0ExVUVBd3dhY0dWeWJXbHpjMmx2Ym5OZlkyRmZZMjl0Ylc5dVgyNWhiV1V3V1RBVEJnY3EKaGtqT1BRSUJCZ2dxaGtqT1BRTUJCd05DQUFRd0hrL1BveEx4RVAyN2V6NWp6bW9mN0tEWGtjbTlBUE1hbW5IZQpHMUU0VGJCTlpyN0ZWbjVNYnNXKzVIZWtsaFBTQVBDMUZlZlhzT2I0QWNiTzRUL3hvMU13VVRBZEJnTlZIUTRFCkZnUVUxNzcxc1RDNVZqUVNUMnZXQkZWb2M2WHdpUlV3SHdZRFZSMGpCQmd3Rm9BVTE3NzFzVEM1VmpRU1QydlcKQkZWb2M2WHdpUlV3RHdZRFZSMFRBUUgvQkFVd0F3RUIvekFLQmdncWhrak9QUVFEQWdOSUFEQkZBaUJJYjRSbwpsSjZ2NEpZcU9SYmlwZXFLQ0xWN1R1TmxheXh2Njk2MlZTazN5UUloQUlqa3JxQlU5UVNPK0VJUDZic0sramNjCjQ3Z3ZkK2NuZjMvelBXSmJOdDIxTVlJQnNEQ0NBYXdDQVFFd1hEQkVNUjB3R3dZRFZRUUtEQlJGZUdGdGNHeGwKSUU5eVoyRnVhWHBoZEdsdmJqRWpNQ0VHQTFVRUF3d2FjR1Z5YldsemMybHZibk5mWTJGZlkyOXRiVzl1WDI1aApiV1VDRkdkZVpUbGNOWlJZUVRaU3BZSGFzNUlRUnc3TU1BMEdDV0NHU0FGbEF3UUNBUVVBb0lIa01CZ0dDU3FHClNJYjNEUUVKQXpFTEJna3Foa2lHOXcwQkJ3RXdIQVlKS29aSWh2Y05BUWtGTVE4WERUSTBNRE13T0RBNE1qazEKTVZvd0x3WUpLb1pJaHZjTkFRa0VNU0lFSUxzbUQzSUFpU2RjOGVjVTk3aUJBQ3V0VnRVczF2SWtBRkpEOUdBNgoyZUZuTUhrR0NTcUdTSWIzRFFFSkR6RnNNR293Q3dZSllJWklBV1VEQkFFcU1Bc0dDV0NHU0FGbEF3UUJGakFMCkJnbGdoa2dCWlFNRUFRSXdDZ1lJS29aSWh2Y05Bd2N3RGdZSUtvWklodmNOQXdJQ0FnQ0FNQTBHQ0NxR1NJYjMKRFFNQ0FnRkFNQWNHQlNzT0F3SUhNQTBHQ0NxR1NJYjNEUU1DQWdFb01Bb0dDQ3FHU000OUJBTUNCRWN3UlFJZwpJRnh2d1RrcDFMU0Q0c2U5NGxLQk5JdUkzSklOSmRMY0RrT3VTOEdMN0RvQ0lRQ3RMcTBnZEFnSytxQjF2b3hOClRzRlNJZ3FYMXMwdUJ0Z0poZ1VkZTJRaDZnPT0KCi0tLS0tLTA3OTNGMjRGMUQzNTAwRTNCMjJFNjE2NkNGNTEyRUY3LS0KCg==";
        const PERMISSIONS: &str = "TUlNRS1WZXJzaW9uOiAxLjAKQ29udGVudC1UeXBlOiBtdWx0aXBhcnQvc2lnbmVkOyBwcm90b2NvbD0iYXBwbGljYXRpb24veC1wa2NzNy1zaWduYXR1cmUiOyBtaWNhbGc9InNoYS0yNTYiOyBib3VuZGFyeT0iLS0tLTc4MTFGOEVCQjA1MDJBMzcyRDI2RjE4NTRCREZDM0FBIgoKVGhpcyBpcyBhbiBTL01JTUUgc2lnbmVkIG1lc3NhZ2UKCi0tLS0tLTc4MTFGOEVCQjA1MDJBMzcyRDI2RjE4NTRCREZDM0FBCkNvbnRlbnQtVHlwZTogdGV4dC9wbGFpbgoKPD94bWwgdmVyc2lvbj0iMS4wIiBlbmNvZGluZz0iVVRGLTgiPz4KPGRkcyB4bWxuczp4c2k9Imh0dHA6Ly93d3cudzMub3JnLzIwMDEvWE1MU2NoZW1hLWluc3RhbmNlIgogICAgeHNpOm5vTmFtZXNwYWNlU2NoZW1hTG9jYXRpb249Imh0dHA6Ly93d3cub21nLm9yZy9zcGVjL0REUy1TZWN1cml0eS8yMDE3MDkwMS9vbWdfc2hhcmVkX2NhX3Blcm1pc3Npb25zLnhzZCI+CiAgICA8cGVybWlzc2lvbnM+CiAgICAgICAgPGdyYW50IG5hbWU9IlBhcnRpY2lwYW50MUV4YW1wbGVQZXJtaXNzaW9uIj4KICAgICAgICAgICAgPHN1YmplY3RfbmFtZT5DTj1wYXJ0aWNpcGFudDFfY29tbW9uX25hbWUsTz1FeGFtcGxlIE9yZ2FuaXphdGlvbjwvc3ViamVjdF9uYW1lPgogICAgICAgICAgICA8dmFsaWRpdHk+CiAgICAgICAgICAgICAgICA8bm90X2JlZm9yZT4yMDIzLTAxLTAxVDAwOjAwOjAwPC9ub3RfYmVmb3JlPgogICAgICAgICAgICAgICAgPG5vdF9hZnRlcj45OTk5LTAxLTAxVDAwOjAwOjAwPC9ub3RfYWZ0ZXI+CiAgICAgICAgICAgIDwvdmFsaWRpdHk+CiAgICAgICAgICAgIDxhbGxvd19ydWxlPgogICAgICAgICAgICAgICAgPGRvbWFpbnM+CiAgICAgICAgICAgICAgICAgICAgPGlkPjA8L2lkPgogICAgICAgICAgICAgICAgPC9kb21haW5zPgogICAgICAgICAgICAgICAgPHB1Ymxpc2g+CiAgICAgICAgICAgICAgICAgICAgPHRvcGljcz4KICAgICAgICAgICAgICAgICAgICAgICAgPHRvcGljPlNxdWFyZTwvdG9waWM+CiAgICAgICAgICAgICAgICAgICAgPC90b3BpY3M+CiAgICAgICAgICAgICAgICA8L3B1Ymxpc2g+CiAgICAgICAgICAgICAgICA8c3Vic2NyaWJlPgogICAgICAgICAgICAgICAgICAgIDx0b3BpY3M+CiAgICAgICAgICAgICAgICAgICAgICAgIDx0b3BpYz5TcXVhcmU8L3RvcGljPgogICAgICAgICAgICAgICAgICAgIDwvdG9waWNzPgogICAgICAgICAgICAgICAgPC9zdWJzY3JpYmU+CiAgICAgICAgICAgICAgICA8cmVsYXk+CiAgICAgICAgICAgICAgICAgICAgPHRvcGljcz4KICAgICAgICAgICAgICAgICAgICAgICAgPHRvcGljPmV4YW1wbGVfdG9waWM8L3RvcGljPgogICAgICAgICAgICAgICAgICAgIDwvdG9waWNzPgogICAgICAgICAgICAgICAgPC9yZWxheT4KICAgICAgICAgICAgPC9hbGxvd19ydWxlPgogICAgICAgICAgICA8ZGVmYXVsdD5ERU5ZPC9kZWZhdWx0PgogICAgICAgIDwvZ3JhbnQ+CiAgICAgICAgPGdyYW50IG5hbWU9IlBhcnRpY2lwYW50MkV4YW1wbGVQZXJtaXNzaW9uIj4KICAgICAgICAgICAgPHN1YmplY3RfbmFtZT5DTj1wYXJ0aWNpcGFudDJfY29tbW9uX25hbWUsTz1FeGFtcGxlIE9yZ2FuaXphdGlvbjwvc3ViamVjdF9uYW1lPgogICAgICAgICAgICA8dmFsaWRpdHk+CiAgICAgICAgICAgICAgICA8bm90X2JlZm9yZT4yMDIzLTAxLTAxVDAwOjAwOjAwPC9ub3RfYmVmb3JlPgogICAgICAgICAgICAgICAgPG5vdF9hZnRlcj45OTk5LTAxLTAxVDAwOjAwOjAwPC9ub3RfYWZ0ZXI+CiAgICAgICAgICAgIDwvdmFsaWRpdHk+CiAgICAgICAgICAgIDxhbGxvd19ydWxlPgogICAgICAgICAgICAgICAgPGRvbWFpbnM+CiAgICAgICAgICAgICAgICAgICAgPGlkPjA8L2lkPgogICAgICAgICAgICAgICAgPC9kb21haW5zPgogICAgICAgICAgICAgICAgPHB1Ymxpc2g+CiAgICAgICAgICAgICAgICAgICAgPHRvcGljcz4KICAgICAgICAgICAgICAgICAgICAgICAgPHRvcGljPlNxdWFyZTwvdG9waWM+CiAgICAgICAgICAgICAgICAgICAgPC90b3BpY3M+CiAgICAgICAgICAgICAgICA8L3B1Ymxpc2g+CiAgICAgICAgICAgICAgICA8c3Vic2NyaWJlPgogICAgICAgICAgICAgICAgICAgIDx0b3BpY3M+CiAgICAgICAgICAgICAgICAgICAgICAgIDx0b3BpYz5TcXVhcmU8L3RvcGljPgogICAgICAgICAgICAgICAgICAgIDwvdG9waWNzPgogICAgICAgICAgICAgICAgPC9zdWJzY3JpYmU+CiAgICAgICAgICAgIDwvYWxsb3dfcnVsZT4KICAgICAgICAgICAgPGRlZmF1bHQ+REVOWTwvZGVmYXVsdD4KICAgICAgICA8L2dyYW50PgogICAgPC9wZXJtaXNzaW9ucz4KPC9kZHM+Ci0tLS0tLTc4MTFGOEVCQjA1MDJBMzcyRDI2RjE4NTRCREZDM0FBCkNvbnRlbnQtVHlwZTogYXBwbGljYXRpb24veC1wa2NzNy1zaWduYXR1cmU7IG5hbWU9InNtaW1lLnA3cyIKQ29udGVudC1UcmFuc2Zlci1FbmNvZGluZzogYmFzZTY0CkNvbnRlbnQtRGlzcG9zaXRpb246IGF0dGFjaG1lbnQ7IGZpbGVuYW1lPSJzbWltZS5wN3MiCgpNSUlEendZSktvWklodmNOQVFjQ29JSUR3RENDQTd3Q0FRRXhEekFOQmdsZ2hrZ0JaUU1FQWdFRkFEQUxCZ2txCmhraUc5dzBCQndHZ2dnSGpNSUlCM3pDQ0FZV2dBd0lCQWdJVVoxNWxPVncxbEZoQk5sS2xnZHF6a2hCSERzd3cKQ2dZSUtvWkl6ajBFQXdJd1JERWRNQnNHQTFVRUNnd1VSWGhoYlhCc1pTQlBjbWRoYm1sNllYUnBiMjR4SXpBaApCZ05WQkFNTUduQmxjbTFwYzNOcGIyNXpYMk5oWDJOdmJXMXZibDl1WVcxbE1DQVhEVEkwTURNd09EQTRNamsxCk1Wb1lEelEzTmpJd01qQXlNRGd5T1RVeFdqQkVNUjB3R3dZRFZRUUtEQlJGZUdGdGNHeGxJRTl5WjJGdWFYcGgKZEdsdmJqRWpNQ0VHQTFVRUF3d2FjR1Z5YldsemMybHZibk5mWTJGZlkyOXRiVzl1WDI1aGJXVXdXVEFUQmdjcQpoa2pPUFFJQkJnZ3Foa2pPUFFNQkJ3TkNBQVF3SGsvUG94THhFUDI3ZXo1anptb2Y3S0RYa2NtOUFQTWFtbkhlCkcxRTRUYkJOWnI3RlZuNU1ic1crNUhla2xoUFNBUEMxRmVmWHNPYjRBY2JPNFQveG8xTXdVVEFkQmdOVkhRNEUKRmdRVTE3NzFzVEM1VmpRU1QydldCRlZvYzZYd2lSVXdId1lEVlIwakJCZ3dGb0FVMTc3MXNUQzVWalFTVDJ2VwpCRlZvYzZYd2lSVXdEd1lEVlIwVEFRSC9CQVV3QXdFQi96QUtCZ2dxaGtqT1BRUURBZ05JQURCRkFpQkliNFJvCmxKNnY0SllxT1JiaXBlcUtDTFY3VHVObGF5eHY2OTYyVlNrM3lRSWhBSWprcnFCVTlRU08rRUlQNmJzSytqY2MKNDdndmQrY25mMy96UFdKYk50MjFNWUlCc0RDQ0Fhd0NBUUV3WERCRU1SMHdHd1lEVlFRS0RCUkZlR0Z0Y0d4bApJRTl5WjJGdWFYcGhkR2x2YmpFak1DRUdBMVVFQXd3YWNHVnliV2x6YzJsdmJuTmZZMkZmWTI5dGJXOXVYMjVoCmJXVUNGR2RlWlRsY05aUllRVFpTcFlIYXM1SVFSdzdNTUEwR0NXQ0dTQUZsQXdRQ0FRVUFvSUhrTUJnR0NTcUcKU0liM0RRRUpBekVMQmdrcWhraUc5dzBCQndFd0hBWUpLb1pJaHZjTkFRa0ZNUThYRFRJME1ETXdPREE0TWprMQpNVm93THdZSktvWklodmNOQVFrRU1TSUVJQ3FtOSs3a2ZFYU1XMFdETlp0YXdyeGhQSmR6ZEVFY29hV0hqTStpCnZ6eEhNSGtHQ1NxR1NJYjNEUUVKRHpGc01Hb3dDd1lKWUlaSUFXVURCQUVxTUFzR0NXQ0dTQUZsQXdRQkZqQUwKQmdsZ2hrZ0JaUU1FQVFJd0NnWUlLb1pJaHZjTkF3Y3dEZ1lJS29aSWh2Y05Bd0lDQWdDQU1BMEdDQ3FHU0liMwpEUU1DQWdGQU1BY0dCU3NPQXdJSE1BMEdDQ3FHU0liM0RRTUNBZ0VvTUFvR0NDcUdTTTQ5QkFNQ0JFY3dSUUloCkFMMVJrM1doSzRNY2VleGVJNHA5dFJEY1A4S1ZkSUxmRTNzL1d2RXBESzRSQWlCZFNKOHJGZWdDMVBFNE9OMjUKMEFZTnFrekc2c3BDK1k1Zjcxa3k3TE83Z1E9PQoKLS0tLS0tNzgxMUY4RUJCMDUwMkEzNzJEMjZGMTg1NEJERkMzQUEtLQoK";

        for (name, b64) in [
            ("identity_ca.cert.pem", IDENTITY_CA),
            ("cert.pem", CERT),
            ("permissions_ca.cert.pem", PERMISSIONS_CA),
            ("governance.p7s", GOVERNANCE),
            ("permissions.p7s", PERMISSIONS),
        ] {
            std::fs::write(temp.join(name), decode_b64(b64)).unwrap();
        }

        let key_bytes: Vec<u8> = KEY_XOR.iter().map(|b| b ^ 0x5a).collect();
        std::fs::write(temp.join("key.pem"), key_bytes).unwrap();

        let cleaner = Cleaner(temp.clone());
        (temp, cleaner)
    }

    #[test]
    #[cfg(feature = "security")]
    fn secure_participant_can_be_built() {
        let (fixture_dir, _cleaner) = setup_test_security_enclave();

        let builder = rustdds::DomainParticipantBuilder::new(0);
        let secured = Context::apply_security(builder, &fixture_dir);
        assert!(
            secured.is_ok(),
            "apply_security failed: {:?}",
            secured.err()
        );

        let participant = secured.unwrap().build();
        assert!(
            participant.is_ok(),
            "failed to build secured participant: {:?}",
            participant.err()
        );
    }
}
