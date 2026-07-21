//! Backend-neutral building blocks for ROS2 transports.

use dora_message::descriptor::Ros2TransportConfig;

pub mod action;
pub mod dds;
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
    pub fn new(config: &Ros2TransportConfig) -> Result<Self, TransportError> {
        match config {
            Ros2TransportConfig::Dds => dds::Context::new().map(Self::Dds),
            Ros2TransportConfig::Zenoh { .. } => {
                Err(TransportError::UnsupportedTransport { transport: "zenoh" })
            }
        }
    }

    #[cfg(feature = "rmw-zenoh")]
    pub async fn open(config: &Ros2TransportConfig) -> Result<Self, TransportError> {
        match config {
            Ros2TransportConfig::Dds => dds::Context::new().map(Self::Dds),
            Ros2TransportConfig::Zenoh { config_uri, .. } => {
                zenoh::Context::open(zenoh::ContextOptions {
                    domain_id: std::env::var("ROS_DOMAIN_ID")
                        .ok()
                        .and_then(|value| value.parse().ok())
                        .unwrap_or(0),
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

/// A service client backed by DDS or native Zenoh.
pub enum ServiceClient<D> {
    Dds(D),
    #[cfg(feature = "rmw-zenoh")]
    Zenoh(Box<zenoh::service::RawServiceClient>),
}

/// A service server backed by DDS or native Zenoh.
pub enum ServiceServer<D> {
    Dds(D),
    #[cfg(feature = "rmw-zenoh")]
    Zenoh(Box<zenoh::service::RawServiceServer>),
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use dora_message::descriptor::Ros2TransportConfig;

    use super::{Context, Durability, History, Liveliness, Reliability, Ros2Qos, dds};

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
        let rustdds = dds::to_rustdds_qos(&qos);
        assert_eq!(dds::from_rustdds_qos(&rustdds), qos);
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
        let rustdds = dds::to_rustdds_qos(&qos);
        assert_eq!(dds::from_rustdds_qos(&rustdds), qos);
    }

    #[test]
    fn context_new_dds_creates_dds_variant() {
        let context = Context::new(&Ros2TransportConfig::Dds).unwrap();
        assert!(matches!(context, Context::Dds(_)));
    }
}
