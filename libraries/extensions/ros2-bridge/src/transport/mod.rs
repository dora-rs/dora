//! Backend-neutral building blocks for ROS2 transports.

use dora_message::descriptor::Ros2TransportConfig;

pub mod action;
pub mod dds;
pub mod diagnostics;
pub mod interfaces;
pub mod parameters;
mod types;
pub mod zenoh;

pub use types::{
    Durability, History, Liveliness, MessageMetadata, Reliability, RequestId, Ros2Qos,
    TransportError,
};

/// A ROS2 context backed by one native transport.
pub enum Context {
    /// The existing `ros2-client`/RustDDS context.
    Dds(dds::Context),
    #[cfg(feature = "rmw-zenoh")]
    Zenoh(zenoh::Context),
}

impl Context {
    /// Create a context for the requested transport.
    pub fn new(config: &Ros2TransportConfig, domain_id: u16) -> Result<Self, TransportError> {
        match config {
            Ros2TransportConfig::Dds => dds::Context::new(domain_id).map(Self::Dds),
            Ros2TransportConfig::Zenoh { .. } => {
                Err(TransportError::UnsupportedTransport { transport: "zenoh" })
            }
        }
    }

    #[cfg(feature = "rmw-zenoh")]
    pub async fn open(
        config: &Ros2TransportConfig,
        domain_id: u16,
    ) -> Result<Self, TransportError> {
        match config {
            Ros2TransportConfig::Dds => dds::Context::new(domain_id).map(Self::Dds),
            Ros2TransportConfig::Zenoh { config_uri, .. } => {
                zenoh::Context::open(zenoh::ContextOptions {
                    domain_id: usize::from(domain_id),
                    config_uri: config_uri
                        .as_ref()
                        .map(|path| path.to_string_lossy().into_owned()),
                })
                .await
                .map(Self::Zenoh)
                .map_err(|error| TransportError::ZenohContext {
                    message: error.to_string(),
                })
            }
        }
    }

    #[cfg(feature = "rmw-zenoh")]
    pub async fn new_named_node(
        &self,
        namespace: &str,
        name: &str,
        options: ros2_client::NodeOptions,
    ) -> Result<Node, TransportError> {
        match self {
            Self::Dds(context) => context
                .new_node(
                    ros2_client::NodeName::new(namespace, name).map_err(|error| {
                        TransportError::DdsNode {
                            message: error.to_string(),
                        }
                    })?,
                    options,
                )
                .map(Box::new)
                .map(Node::Dds),
            Self::Zenoh(context) => context
                .create_node(&context.zid(), name, "/", namespace, name)
                .await
                .map(Node::Zenoh)
                .map_err(|error| TransportError::ZenohNode {
                    message: error.to_string(),
                }),
        }
    }

    /// Create a node within this context.
    pub fn new_node(
        &self,
        node_name: ros2_client::NodeName,
        options: ros2_client::NodeOptions,
    ) -> Result<Node, TransportError> {
        match self {
            Self::Dds(context) => context
                .new_node(node_name, options)
                .map(Box::new)
                .map(Node::Dds),
            #[cfg(feature = "rmw-zenoh")]
            Self::Zenoh(_) => Err(TransportError::ZenohNode {
                message: "use async new_named_node for Zenoh".into(),
            }),
        }
    }
}

/// A ROS2 node backed by one native transport.
pub enum Node {
    /// The existing `ros2-client`/RustDDS node.
    Dds(Box<dds::Node>),
    #[cfg(feature = "rmw-zenoh")]
    Zenoh(zenoh::Node),
}

/// A publisher backed by either the existing DDS implementation or native Zenoh.
pub enum Publisher<D> {
    /// Existing DDS publisher type supplied by the caller.
    Dds(D),
    /// Native Zenoh publisher.
    #[cfg(feature = "rmw-zenoh")]
    Zenoh(Box<zenoh::pubsub::RawPublisher>),
}

/// A subscription backed by either the existing DDS implementation or native Zenoh.
pub enum Subscription<D, T = Vec<u8>> {
    /// Existing DDS subscription type supplied by the caller.
    Dds(D),
    /// Native Zenoh subscription with a bounded decoded queue.
    #[cfg(feature = "rmw-zenoh")]
    Zenoh(Box<zenoh::pubsub::RawSubscription<T>>),
    /// Retains the decoded value type in DDS-only builds without allocating storage.
    #[doc(hidden)]
    _Value(std::marker::PhantomData<fn() -> T>),
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use dora_message::descriptor::Ros2TransportConfig;

    use super::{Context, Durability, History, Liveliness, Reliability, Ros2Qos, dds};
    use crate::ros2_client;

    #[test]
    fn dds_qos_adapter_preserves_reliable_keep_all() {
        let qos = Ros2Qos {
            reliability: Reliability::Reliable {
                max_blocking_time: Duration::from_millis(250),
            },
            durability: Durability::TransientLocal,
            history: History::KeepAll,
            liveliness: Liveliness::Automatic {
                lease_duration: None,
            },
        };
        let profile = dds::to_qos_profile(&qos);
        assert_eq!(dds::from_qos_profile(&profile), qos);
    }

    #[test]
    fn dds_qos_adapter_preserves_best_effort_keep_last() {
        let qos = Ros2Qos {
            reliability: Reliability::BestEffort,
            durability: Durability::Volatile,
            history: History::KeepLast { depth: 7 },
            liveliness: Liveliness::ManualByTopic {
                lease_duration: Some(Duration::from_secs(3)),
            },
        };
        let profile = dds::to_qos_profile(&qos);
        assert_eq!(dds::from_qos_profile(&profile), qos);
    }

    /// `ros2-client` >= 0.11 has no MANUAL_BY_PARTICIPANT liveliness (ROS 2
    /// deprecated MANUAL_BY_NODE, and rclcpp rejects it), so the DDS path
    /// degrades it to AUTOMATIC and keeps the lease.
    #[test]
    fn dds_qos_adapter_degrades_manual_by_participant_to_automatic() {
        let qos = Ros2Qos {
            reliability: Reliability::BestEffort,
            durability: Durability::Volatile,
            history: History::KeepLast { depth: 1 },
            liveliness: Liveliness::ManualByParticipant {
                lease_duration: Some(Duration::from_secs(2)),
            },
        };
        let profile = dds::to_qos_profile(&qos);
        assert_eq!(
            dds::from_qos_profile(&profile).liveliness,
            Liveliness::Automatic {
                lease_duration: Some(Duration::from_secs(2)),
            }
        );
    }

    /// An RTPS duration holds its seconds in an `i32`, and RustDDS wraps a
    /// longer one instead of saturating it. Such a duration must not reach it
    /// as a finite value: it means "infinite" (block / never expire).
    #[test]
    fn dds_qos_adapter_treats_unrepresentable_durations_as_infinite() {
        use ros2_client::qos;
        let profile_for = |duration| {
            dds::to_qos_profile(&Ros2Qos {
                reliability: Reliability::Reliable {
                    max_blocking_time: duration,
                },
                durability: Durability::Volatile,
                history: History::KeepLast { depth: 1 },
                liveliness: Liveliness::Automatic {
                    lease_duration: Some(duration),
                },
            })
        };
        let first_too_long = Duration::from_secs(i32::MAX as u64);
        for too_long in [
            first_too_long,
            Duration::from_secs(3_000_000_000),
            Duration::from_secs(1_000_000_000_000),
            Duration::MAX,
        ] {
            let profile = profile_for(too_long);
            assert_eq!(
                profile.reliability,
                qos::Reliability::Reliable {
                    when_full: qos::WhenFull::Block
                },
                "{too_long:?}"
            );
            assert_eq!(profile.liveliness_lease, None, "{too_long:?}");
            // ...and a blocking profile survives the round trip.
            assert_eq!(
                dds::to_qos_profile(&dds::from_qos_profile(&profile)).reliability,
                profile.reliability
            );
        }

        let longest = first_too_long - Duration::from_nanos(1);
        let profile = profile_for(longest);
        assert_eq!(
            profile.reliability,
            qos::Reliability::Reliable {
                when_full: qos::WhenFull::Wait(longest)
            }
        );
        assert_eq!(profile.liveliness_lease, Some(longest));
    }

    /// A keep-last depth below 1 retains nothing; clamp it like the YAML path.
    #[test]
    fn dds_qos_adapter_clamps_keep_last_depth_to_one() {
        for depth in [-1, 0] {
            let qos = Ros2Qos {
                reliability: Reliability::BestEffort,
                durability: Durability::Volatile,
                history: History::KeepLast { depth },
                liveliness: Liveliness::Automatic {
                    lease_duration: None,
                },
            };
            assert_eq!(
                dds::to_qos_profile(&qos).history,
                ros2_client::qos::History::KeepLast { depth: 1 }
            );
        }
    }

    #[test]
    fn context_new_dds_uses_requested_domain() {
        let context = Context::new(&Ros2TransportConfig::Dds, 42).unwrap();
        match context {
            Context::Dds(context) => assert_eq!(context.as_inner().domain_id(), 42),
            #[cfg(feature = "rmw-zenoh")]
            Context::Zenoh(_) => panic!("DDS configuration created a Zenoh context"),
        };
    }
}
