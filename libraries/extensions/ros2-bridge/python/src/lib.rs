use std::{
    borrow::Cow,
    collections::HashMap,
    path::{Path, PathBuf},
    sync::Arc,
};

use ::dora_ros2_bridge::transport::zenoh::{
    compatibility::{TypeDescriptionResolver, resolve_action, resolve_message, resolve_service},
    keyexpr::{DataKey, TopicToken},
    pubsub::{NodePublisher, NodeSubscription, PubSubError},
    qos::ZenohQosMapping,
    service::{NodeServiceClient, NodeServiceServer, wait_for_service},
};
use ::dora_ros2_bridge::{
    ros2_client, rustdds,
    transport::action::{
        GoalSlots, MAX_CONCURRENT_GOALS,
        zenoh::{
            ActionClient as ZenohActionClient, ActionKeys, ActionServer as ZenohActionServer,
            ActionTokens, wait_for_server as wait_for_action_server,
        },
    },
};
use arrow::{
    array::{ArrayData, make_array},
    pyarrow::{FromPyArrow, ToPyArrow},
};
use dora_ros2_bridge_arrow::{
    deserialize_cdr, deserialize_raw_cdr, serialize_cdr, serialize_raw_cdr,
};
use dora_ros2_bridge_msg_gen::types::Message;
use eyre::{Context, ContextCompat, Result, eyre};
use futures::{Stream, StreamExt};
use pyo3::{
    Bound, Py, PyAny, PyResult, Python,
    prelude::{pyclass, pymethods},
    types::{
        PyAnyMethods, PyDict, PyDictMethods, PyList, PyListMethods, PyModule, PyModuleMethods,
    },
};
use std::time::{Duration, Instant};
use typed::{
    BridgeActionType, BridgeMessage, BridgeServiceType, TypeInfo, TypeInfoGuard, TypedValue,
    deserialize::StructDeserializer,
};

pub mod qos;
pub mod typed;

#[derive(Clone)]
#[pyclass(from_py_object)]
pub struct Ros2Transport {
    config: dora_message::descriptor::Ros2TransportConfig,
}

#[pymethods]
impl Ros2Transport {
    #[staticmethod]
    pub fn dds() -> Self {
        Self {
            config: dora_message::descriptor::Ros2TransportConfig::Dds,
        }
    }

    #[staticmethod]
    #[pyo3(signature = (compatibility, config_uri=None))]
    pub fn zenoh(compatibility: &str, config_uri: Option<PathBuf>) -> PyResult<Self> {
        let compatibility = match compatibility {
            "humble" => dora_message::descriptor::RmwZenohCompatibility::Humble,
            "rep2016" => dora_message::descriptor::RmwZenohCompatibility::Rep2016,
            _ => {
                return Err(pyo3::exceptions::PyValueError::new_err(
                    "compatibility must be `humble` or `rep2016`",
                ));
            }
        };
        if config_uri
            .as_ref()
            .is_some_and(|path| path.as_os_str().is_empty())
        {
            return Err(pyo3::exceptions::PyValueError::new_err(
                "Zenoh config_uri must not be empty",
            ));
        }
        Ok(Self {
            config: dora_message::descriptor::Ros2TransportConfig::Zenoh {
                compatibility,
                config_uri,
            },
        })
    }

    #[getter]
    pub fn kind(&self) -> &'static str {
        match self.config {
            dora_message::descriptor::Ros2TransportConfig::Dds => "dds",
            dora_message::descriptor::Ros2TransportConfig::Zenoh { .. } => "zenoh",
        }
    }
}

/// Drop a taken-but-unanswered service request after this long, so a server
/// whose handler skips `send_response` can't leak `pending` entries forever.
/// Matches the standalone bridge daemon.
const SERVICE_RESPONSE_TIMEOUT: Duration = Duration::from_secs(30);
/// Hard cap on outstanding taken-but-unanswered requests. Matches the daemon.
const MAX_PENDING_REQUESTS: usize = 64;

/// ROS2 Context holding all messages definition for receiving and sending messages to ROS2.
///
/// By default, Ros2Context will use env `AMENT_PREFIX_PATH` to search for message definition.
///
/// AMENT_PREFIX_PATH folder structure should be the following:
///
/// - For messages: <namespace>/msg/<name>.msg
/// - For services: <namespace>/srv/<name>.srv
///
/// You can also use `ros_paths` if you don't want to use env variable.
///
/// The DDS domain id is taken from the `domain_id` argument; if omitted, from
/// the `ROS_DOMAIN_ID` environment variable; if neither is set, default to 0.
/// A `ROS_DOMAIN_ID` that is set but not a valid domain id raises an error.
///
/// warning::
///     dora Ros2 bridge functionality is considered **unstable**. It may be changed
///     at any point without it being considered a breaking change.
///
/// ```python
/// context = Ros2Context(domain_id=1)   # explicit domain
/// context = Ros2Context()              # domain from ROS_DOMAIN_ID, else 0
/// ```
///
/// :type ros_paths: typing.List[str], optional
/// :type domain_id: int, optional
///
#[pyclass]
pub struct Ros2Context {
    context: dora_ros2_bridge::transport::Context,
    transport: Ros2Transport,
    messages: Arc<HashMap<String, HashMap<String, Message>>>,
}

/// Resolve the DDS domain: explicit `domain_id` wins, else `ROS_DOMAIN_ID`, else 0.
/// A `ROS_DOMAIN_ID` that is set but unparsable is an error (as rcl treats it), not a
/// silent fall back to domain 0 — which would surface as "no ROS topics visible".
fn resolve_domain_id(
    explicit: Option<u16>,
    env: Result<String, std::env::VarError>,
) -> eyre::Result<u16> {
    match explicit {
        Some(id) => Ok(id),
        None => match env {
            // Empty/whitespace means unset (some setups export `ROS_DOMAIN_ID=`).
            Ok(s) if s.trim().is_empty() => Ok(0),
            Ok(s) => s
                .trim()
                .parse()
                .wrap_err_with(|| format!("invalid ROS_DOMAIN_ID {:?}", s.trim())),
            Err(std::env::VarError::NotPresent) => Ok(0),
            Err(std::env::VarError::NotUnicode(s)) => {
                eyre::bail!(
                    "ROS_DOMAIN_ID is not valid unicode: {}",
                    s.to_string_lossy()
                )
            }
        },
    }
}

#[cfg(test)]
mod domain_id_tests {
    use super::resolve_domain_id;
    use std::env::VarError;

    #[test]
    fn resolves_domain_id() {
        assert_eq!(resolve_domain_id(Some(5), Ok("9".into())).unwrap(), 5); // explicit wins
        assert_eq!(resolve_domain_id(None, Ok(" 7 ".into())).unwrap(), 7); // env, trimmed
        assert_eq!(resolve_domain_id(None, Ok("  ".into())).unwrap(), 0); // empty/ws -> unset
        assert_eq!(
            resolve_domain_id(None, Err(VarError::NotPresent)).unwrap(),
            0
        ); // unset -> 0
        assert!(resolve_domain_id(None, Ok("abc".into())).is_err()); // set-but-invalid -> error
        assert!(resolve_domain_id(None, Ok("70000".into())).is_err()); // out of u16 range -> error
    }
}

#[pymethods]
impl Ros2Context {
    /// Create a new context
    #[new]
    #[pyo3(signature = (ros_paths=None, transport=None, domain_id=None))]
    pub fn new(
        ros_paths: Option<Vec<PathBuf>>,
        transport: Option<Ros2Transport>,
        domain_id: Option<u16>,
    ) -> eyre::Result<Self> {
        Python::attach(|py| -> Result<()> {
            let warnings = py
                .import("warnings")
                .wrap_err("failed to import `warnings` module")?;
            warnings
            .call_method1("warn", ("dora-rs ROS2 Bridge is unstable and may change at any point without it being considered a breaking change",))
            .wrap_err("failed to call `warnings.warn` module")?;
            Ok(())
        })?;
        let ament_prefix_path = std::env::var("AMENT_PREFIX_PATH");
        let empty = String::new();

        let paths: Vec<_> = match &ros_paths {
            Some(paths) => paths.iter().map(|p| p.as_path()).collect(),
            None => {
                let ament_prefix_path_parsed = match &ament_prefix_path {
                    Ok(path) => path,
                    Err(std::env::VarError::NotPresent) => &empty,
                    Err(std::env::VarError::NotUnicode(s)) => {
                        eyre::bail!(
                            "AMENT_PREFIX_PATH is not valid unicode: `{}`",
                            s.to_string_lossy()
                        );
                    }
                };

                ament_prefix_path_parsed.split(':').map(Path::new).collect()
            }
        };

        let packages = dora_ros2_bridge_msg_gen::get_packages(&paths)
            .map_err(|err| eyre!(err))
            .context("failed to parse ROS2 message types")?;

        // Mirror the standalone bridge daemon's `load_messages`: index plain
        // messages plus the per-service and per-action request/response/goal/
        // result/feedback messages, so service and action types resolve too.
        let mut messages: HashMap<String, HashMap<String, Message>> = HashMap::new();
        for package in packages {
            let entry = messages.entry(package.name.clone()).or_default();
            for message in package.messages {
                entry.insert(message.name.clone(), message);
            }
            for service in package.services {
                entry.insert(service.request.name.clone(), service.request);
                entry.insert(service.response.name.clone(), service.response);
            }
            for action in package.actions {
                entry.insert(action.goal.name.clone(), action.goal);
                entry.insert(action.result.name.clone(), action.result);
                entry.insert(action.feedback.name.clone(), action.feedback);
            }
        }

        let domain_id = resolve_domain_id(domain_id, std::env::var("ROS_DOMAIN_ID"))?;
        let transport = transport.unwrap_or_else(Ros2Transport::dds);
        let context = futures::executor::block_on(dora_ros2_bridge::transport::Context::open(
            &transport.config,
            domain_id,
        ))?;
        Ok(Self {
            context,
            transport,
            messages: Arc::new(messages),
        })
    }

    #[getter]
    pub fn transport_kind(&self) -> &'static str {
        self.transport.kind()
    }

    /// Create a new ROS2 node
    ///
    /// ```python
    /// ros2_node = ros2_context.new_node(
    ///     "turtle_teleop",
    ///     "/ros2_demo",
    ///     Ros2NodeOptions(rosout=True),
    /// )
    /// ```
    ///
    /// warning::
    ///     dora Ros2 bridge functionality is considered **unstable**. It may be changed
    ///     at any point without it being considered a breaking change.
    ///
    /// :type name: str
    /// :type namespace: str
    /// :type options: dora.Ros2NodeOptions
    /// :type parameters: dict, optional   # initial ROS2 parameters to declare
    /// :rtype: dora.Ros2Node
    #[pyo3(signature = (name, namespace, options, parameters=None))]
    pub fn new_node(
        &self,
        name: &str,
        namespace: &str,
        options: Ros2NodeOptions,
        parameters: Option<Bound<'_, PyDict>>,
    ) -> eyre::Result<Ros2Node> {
        let rosout_requested = options.rosout;
        let name = ros2_client::NodeName::new(namespace, name)
            .map_err(|err| eyre!("invalid node name: {err}"))?;
        // ros2-client hosts the ROS2 parameter services natively (driven by the
        // spinner below); declare any initial parameters on the node options so
        // `ros2 param` / rclpy clients see them and `set_parameter` can update
        // them at runtime.
        let mut node_options: ros2_client::NodeOptions = options.into();
        // Records each declared parameter's value type so the validator below can
        // enforce type stability on *every* set path (the local `set_parameter`
        // and the native rcl_interfaces services the spinner serves to remote
        // `ros2 param` / rclpy clients), not just the Python one.
        let mut declared_types: HashMap<
            String,
            std::mem::Discriminant<ros2_client::ParameterValue>,
        > = HashMap::new();
        if let Some(parameters) = parameters {
            for (key, value) in parameters.iter() {
                let key: String = key.extract().wrap_err("parameter name must be a str")?;
                // `use_sim_time` is a built-in: ros2-client re-declares it as
                // Boolean(false) after user declarations, silently dropping any
                // value set here. Reject so the footgun is loud; it can still be
                // changed at runtime via `set_parameter`.
                if key == "use_sim_time" {
                    eyre::bail!(
                        "`use_sim_time` is a built-in parameter and cannot be declared here; \
                         set it at runtime with `set_parameter` instead"
                    );
                }
                let value = py_to_parameter_value(&value)
                    .wrap_err_with(|| format!("invalid value for parameter `{key}`"))?;
                declared_types.insert(key.clone(), std::mem::discriminant(&value));
                node_options = node_options.declare_parameter(&key, value);
            }
        }
        // Type-stability validator: reject a set whose value type differs from the
        // declared one. ros2-client invokes this on both the local and the
        // service-served set paths, and calls it without holding the parameter
        // lock, so touching only this private map cannot deadlock.
        let validator_types = declared_types.clone();
        node_options = node_options.parameter_validator(Box::new(
            move |name: &str, value: &ros2_client::ParameterValue| match validator_types.get(name) {
                Some(declared) if *declared != std::mem::discriminant(value) => Err(format!(
                    "parameter `{name}` is type-stable; cannot change its declared type"
                )),
                _ => Ok(()),
            },
        ));
        if matches!(self.context, dora_ros2_bridge::transport::Context::Zenoh(_))
            && !declared_types.is_empty()
        {
            eyre::bail!("ROS2 parameters are not yet supported by the native Zenoh transport");
        }
        if matches!(self.context, dora_ros2_bridge::transport::Context::Zenoh(_))
            && rosout_requested
        {
            eyre::bail!("ROS2 rosout is not yet supported by the native Zenoh transport");
        }
        let mut node = futures::executor::block_on(self.context.new_named_node(
            namespace,
            name.base_name(),
            node_options,
        ))?;

        // Start a spinner so service/discovery status events are processed.
        // One worker thread is enough (the spinner is a single task); this keeps
        // the per-node thread cost at 1 instead of one-per-CPU-core.
        let spinner_pool = if let dora_ros2_bridge::transport::Node::Dds(dds) = &mut node {
            let spinner_pool = futures::executor::ThreadPool::builder()
                .pool_size(1)
                .create()
                .map_err(|e| eyre!("failed to create ros2 spinner pool: {e}"))?;
            let spinner = dds
                .as_inner_mut()
                .spinner()
                .map_err(|e| eyre!("failed to create ros2 spinner: {e:?}"))?;
            spinner_pool.spawn_ok(async move {
                if let Err(err) = spinner.spin().await {
                    eprintln!("ros2 spinner stopped: {err:?}");
                }
            });
            Some(spinner_pool)
        } else {
            None
        };

        Ok(Ros2Node {
            node,
            transport: self.transport.clone(),
            messages: self.messages.clone(),
            _spinner_pool: spinner_pool,
        })
    }
}

/// ROS2 Node
///
/// warnings::
/// - dora Ros2 bridge functionality is considered **unstable**. It may be changed
///   at any point without it being considered a breaking change.
/// - There's a known issue about ROS2 nodes not being discoverable by ROS2
///   See: https://github.com/jhelovuo/ros2-client/issues/4
///
#[pyclass]
pub struct Ros2Node {
    node: dora_ros2_bridge::transport::Node,
    transport: Ros2Transport,
    messages: Arc<HashMap<String, HashMap<String, Message>>>,
    // Keeps the ROS2 spinner task alive for the node's lifetime. ros2-client
    // requires a running spinner for service/discovery status events
    // (`wait_for_service` panics otherwise).
    _spinner_pool: Option<futures::executor::ThreadPool>,
}

impl Ros2Node {
    fn dds_node(&self) -> eyre::Result<&ros2_client::Node> {
        match &self.node {
            dora_ros2_bridge::transport::Node::Dds(node) => Ok(node.as_inner()),
            dora_ros2_bridge::transport::Node::Zenoh(_) => {
                eyre::bail!("this ROS2 node feature is not supported by the native Zenoh transport")
            }
        }
    }
}

#[pymethods]
impl Ros2Node {
    /// Create a ROS2 topic to connect to.
    ///
    /// ```python
    /// turtle_twist_topic = ros2_node.create_topic(
    ///     "/turtle1/cmd_vel", "geometry_msgs/Twist", topic_qos
    /// )
    /// ```
    ///
    /// :type name: str
    /// :type message_type: str
    /// :type qos: dora.Ros2QosPolicies
    /// :rtype: dora.Ros2Topic
    pub fn create_topic(
        &self,
        name: &str,
        message_type: String,
        qos: qos::Ros2QosPolicies,
    ) -> eyre::Result<Ros2Topic> {
        let (namespace_name, message_name) = match (
            message_type.split_once('/'),
            message_type.split_once("::"),
        ) {
            (Some(msg), None) => msg,
            (None, Some(msg)) => msg,
            _ => eyre::bail!(
                "Expected message type in the format `namespace/message` or `namespace::message`, such as `std_msgs/UInt8` but got: {}",
                message_type
            ),
        };

        let neutral_qos: dora_ros2_bridge::transport::Ros2Qos = qos.clone().into();
        let topic = match &self.node {
            dora_ros2_bridge::transport::Node::Dds(node) => {
                let message_type_name =
                    ros2_client::MessageTypeName::new(namespace_name, message_name);
                let topic_name = ros2_client::Name::parse(name)
                    .map_err(|err| eyre!("failed to parse ROS2 topic name: {err}"))?;
                TopicBackend::Dds(node.as_inner().create_topic(
                    &topic_name,
                    message_type_name,
                    &dora_ros2_bridge::transport::dds::to_rustdds_qos(&neutral_qos),
                )?)
            }
            dora_ros2_bridge::transport::Node::Zenoh(_) => {
                let dora_message::descriptor::Ros2TransportConfig::Zenoh { compatibility, .. } =
                    self.transport.config
                else {
                    unreachable!()
                };
                let identity = resolve_message(
                    compatibility,
                    namespace_name,
                    message_name,
                    &TypeDescriptionResolver::from_ament_prefix_path(),
                )?;
                let domain = std::env::var("ROS_DOMAIN_ID")
                    .ok()
                    .and_then(|value| value.parse().ok())
                    .unwrap_or(0);
                let key = DataKey::new(domain, name, &identity)?.as_str().to_owned();
                TopicBackend::Zenoh {
                    key,
                    token: TopicToken {
                        name: name.into(),
                        type_name: identity.dds_name.clone(),
                        type_hash: identity.key_hash_component(),
                        qos: ZenohQosMapping::from_ros_qos(&neutral_qos).to_string(),
                    },
                    qos: neutral_qos,
                }
            }
        };
        let type_info = TypeInfo {
            package_name: namespace_name.to_owned().into(),
            message_name: message_name.to_owned().into(),
            messages: self.messages.clone(),
        };

        Ok(Ros2Topic { topic, type_info })
    }

    /// Create a ROS2 publisher
    ///
    /// ```python
    /// pose_publisher = ros2_node.create_publisher(turtle_pose_topic)
    /// ```
    /// warnings:
    /// - dora Ros2 bridge functionality is considered **unstable**. It may be changed
    ///   at any point without it being considered a breaking change.
    ///
    /// :type topic: dora.Ros2Topic
    /// :type qos: dora.Ros2QosPolicies, optional
    /// :rtype: dora.Ros2Publisher
    #[pyo3(signature = (topic, qos=None))]
    pub fn create_publisher(
        &mut self,
        topic: &Ros2Topic,
        qos: Option<qos::Ros2QosPolicies>,
    ) -> eyre::Result<Ros2Publisher> {
        let publisher = match (&mut self.node, &topic.topic) {
            (dora_ros2_bridge::transport::Node::Dds(node), TopicBackend::Dds(topic)) => {
                PublisherBackend::Dds(
                    node.as_inner_mut()
                        .create_publisher(topic, qos.map(Into::into))?,
                )
            }
            (
                dora_ros2_bridge::transport::Node::Zenoh(node),
                TopicBackend::Zenoh {
                    key,
                    token,
                    qos: topic_qos,
                },
            ) => {
                let qos = qos.map(Into::into).unwrap_or(*topic_qos);
                PublisherBackend::Zenoh(futures::executor::block_on(NodePublisher::declare(
                    node,
                    key,
                    token.clone(),
                    &qos,
                ))?)
            }
            _ => eyre::bail!("topic belongs to a different ROS2 transport"),
        };
        Ok(Ros2Publisher {
            publisher,
            type_info: topic.type_info.clone(),
        })
    }

    /// Create a ROS2 subscription
    ///
    /// ```python
    /// pose_reader = ros2_node.create_subscription(turtle_pose_topic)
    /// ```
    ///
    /// warnings:
    /// - dora Ros2 bridge functionality is considered **unstable**. It may be changed
    ///   at any point without it being considered a breaking change.
    ///
    /// :type topic: dora.Ros2Topic
    /// :type qos: dora.Ros2QosPolicies, optional
    /// :rtype: dora.Ros2Subscription
    #[pyo3(signature = (topic, qos=None))]
    pub fn create_subscription(
        &mut self,
        topic: &Ros2Topic,
        qos: Option<qos::Ros2QosPolicies>,
    ) -> eyre::Result<Ros2Subscription> {
        let subscription = match (&mut self.node, &topic.topic) {
            (dora_ros2_bridge::transport::Node::Dds(node), TopicBackend::Dds(topic)) => {
                SubscriptionBackend::Dds(Some(
                    node.as_inner_mut()
                        .create_subscription(topic, qos.map(Into::into))?,
                ))
            }
            (
                dora_ros2_bridge::transport::Node::Zenoh(node),
                TopicBackend::Zenoh {
                    key,
                    token,
                    qos: topic_qos,
                },
            ) => {
                let qos = qos.map(Into::into).unwrap_or(*topic_qos);
                let info = topic.type_info.clone();
                let decoder = Arc::new(move |bytes: &[u8]| {
                    deserialize_raw_cdr(bytes, info.clone(), 64 * 1024 * 1024)
                        .map_err(|error| PubSubError::Decode(error.to_string()))
                });
                SubscriptionBackend::Zenoh(Some(futures::executor::block_on(
                    NodeSubscription::declare(
                        node,
                        key,
                        token.clone(),
                        &qos,
                        64,
                        64 * 1024 * 1024,
                        decoder,
                    ),
                )?))
            }
            _ => eyre::bail!("topic belongs to a different ROS2 transport"),
        };
        Ok(Ros2Subscription {
            subscription,
            deserializer: StructDeserializer::new(Cow::Owned(topic.type_info.clone())),
        })
    }

    /// Create a ROS2 service client.
    ///
    /// Waits (bounded) for a matching service server to become available
    /// before returning. ROS2 services require **reliable** QoS.
    ///
    /// ```python
    /// client = ros2_node.create_service_client(
    ///     "/add_two_ints", "example_interfaces/AddTwoInts",
    ///     Ros2QosPolicies(reliable=True),
    /// )
    /// response = client.call(pa.array([{"a": 2, "b": 3}]))
    /// ```
    ///
    /// warnings:
    /// - dora Ros2 bridge functionality is considered **unstable**. It may be changed
    ///   at any point without it being considered a breaking change.
    ///
    /// :type service_name: str
    /// :type service_type: str
    /// :type qos: dora.Ros2QosPolicies
    /// :rtype: dora.Ros2ServiceClient
    pub fn create_service_client(
        &mut self,
        py: Python<'_>,
        service_name: &str,
        service_type: String,
        qos: qos::Ros2QosPolicies,
    ) -> eyre::Result<Ros2ServiceClient> {
        let (package, type_name, request_type_info, response_type_info) =
            service_type_infos(&service_type, &self.messages)?;

        let neutral_qos: dora_ros2_bridge::transport::Ros2Qos = qos.into();
        let client = match &mut self.node {
            dora_ros2_bridge::transport::Node::Dds(node) => ServiceClientBackend::Dds(Box::new(
                node.as_inner_mut()
                    .create_client::<BridgeServiceType>(
                        dora_ros2_bridge::detect_service_mapping(),
                        &parse_ros2_name(service_name)?,
                        &ros2_client::ServiceTypeName::new(&package, &type_name),
                        dora_ros2_bridge::transport::dds::to_rustdds_qos(&neutral_qos),
                        dora_ros2_bridge::transport::dds::to_rustdds_qos(&neutral_qos),
                    )
                    .map_err(|e| eyre!("failed to create service client: {e:?}"))?,
            )),
            dora_ros2_bridge::transport::Node::Zenoh(node) => {
                let dora_message::descriptor::Ros2TransportConfig::Zenoh { compatibility, .. } =
                    self.transport.config
                else {
                    unreachable!()
                };
                let identity = resolve_service(
                    compatibility,
                    &package,
                    &type_name,
                    &TypeDescriptionResolver::from_ament_prefix_path(),
                )?;
                let domain = std::env::var("ROS_DOMAIN_ID")
                    .ok()
                    .and_then(|v| v.parse().ok())
                    .unwrap_or(0);
                let key = DataKey::new(domain, service_name, &identity)?;
                let token = TopicToken {
                    name: service_name.into(),
                    type_name: identity.dds_name.clone(),
                    type_hash: identity.key_hash_component(),
                    qos: ZenohQosMapping::from_ros_qos(&neutral_qos).to_string(),
                };
                let client = futures::executor::block_on(NodeServiceClient::declare(
                    node,
                    key.as_str(),
                    token.clone(),
                    MAX_PENDING_REQUESTS,
                ))?;
                py.detach(|| {
                    futures::executor::block_on(wait_for_service(
                        node.graph(),
                        &token.name,
                        &token.type_name,
                        &token.type_hash,
                        &token.qos,
                        Instant::now() + Duration::from_secs(20),
                    ))
                })?;
                ServiceClientBackend::Zenoh(client)
            }
        };

        // Mirror the daemon: wait for the server before returning so the first
        // `call()` does not race service discovery. Release the GIL: this blocks
        // for up to 20s and must not freeze other Python threads.
        if let ServiceClientBackend::Dds(client) = &client {
            let node = self.dds_node()?;
            py.detach(|| {
                futures::executor::block_on(async {
                    for _ in 0..10 {
                        let wait = client.wait_for_service(node);
                        futures::pin_mut!(wait);
                        let timeout = futures_timer::Delay::new(Duration::from_secs(2));
                        match futures::future::select(wait, timeout).await {
                            futures::future::Either::Left(((), _)) => return Ok(()),
                            futures::future::Either::Right(_) => {}
                        }
                    }
                    eyre::bail!("service `{service_name}` not available after 10 retries")
                })
            })?;
        }

        Ok(Ros2ServiceClient {
            client,
            request_type_info,
            response_type_info,
        })
    }

    /// Create a ROS2 service server.
    ///
    /// Drive it by polling [`Ros2ServiceServer::take_request`] and replying with
    /// [`Ros2ServiceServer::send_response`]. ROS2 services require **reliable** QoS.
    ///
    /// ```python
    /// server = ros2_node.create_service_server(
    ///     "/add_two_ints", "example_interfaces/AddTwoInts",
    ///     Ros2QosPolicies(reliable=True),
    /// )
    /// req = server.take_request(timeout_s=0.1)
    /// if req is not None:
    ///     request_id, value = req
    ///     args = value[0].as_py()
    ///     server.send_response(request_id, pa.array([{"sum": args["a"] + args["b"]}]))
    /// ```
    ///
    /// warnings:
    /// - dora Ros2 bridge functionality is considered **unstable**. It may be changed
    ///   at any point without it being considered a breaking change.
    ///
    /// :type service_name: str
    /// :type service_type: str
    /// :type qos: dora.Ros2QosPolicies
    /// :rtype: dora.Ros2ServiceServer
    pub fn create_service_server(
        &mut self,
        service_name: &str,
        service_type: String,
        qos: qos::Ros2QosPolicies,
    ) -> eyre::Result<Ros2ServiceServer> {
        let (package, type_name, request_type_info, response_type_info) =
            service_type_infos(&service_type, &self.messages)?;

        let neutral_qos: dora_ros2_bridge::transport::Ros2Qos = qos.into();
        let server = match &mut self.node {
            dora_ros2_bridge::transport::Node::Dds(node) => ServiceServerBackend::Dds(Box::new(
                node.as_inner_mut()
                    .create_server::<BridgeServiceType>(
                        dora_ros2_bridge::detect_service_mapping(),
                        &parse_ros2_name(service_name)?,
                        &ros2_client::ServiceTypeName::new(&package, &type_name),
                        dora_ros2_bridge::transport::dds::to_rustdds_qos(&neutral_qos),
                        dora_ros2_bridge::transport::dds::to_rustdds_qos(&neutral_qos),
                    )
                    .map_err(|e| eyre!("failed to create service server: {e:?}"))?,
            )),
            dora_ros2_bridge::transport::Node::Zenoh(node) => {
                let dora_message::descriptor::Ros2TransportConfig::Zenoh { compatibility, .. } =
                    self.transport.config
                else {
                    unreachable!()
                };
                let identity = resolve_service(
                    compatibility,
                    &package,
                    &type_name,
                    &TypeDescriptionResolver::from_ament_prefix_path(),
                )?;
                let domain = std::env::var("ROS_DOMAIN_ID")
                    .ok()
                    .and_then(|v| v.parse().ok())
                    .unwrap_or(0);
                let key = DataKey::new(domain, service_name, &identity)?;
                let token = TopicToken {
                    name: service_name.into(),
                    type_name: identity.dds_name.clone(),
                    type_hash: identity.key_hash_component(),
                    qos: ZenohQosMapping::from_ros_qos(&neutral_qos).to_string(),
                };
                ServiceServerBackend::Zenoh(futures::executor::block_on(
                    NodeServiceServer::declare(
                        node,
                        key.as_str(),
                        token,
                        MAX_PENDING_REQUESTS,
                        SERVICE_RESPONSE_TIMEOUT,
                    ),
                )?)
            }
        };

        Ok(Ros2ServiceServer {
            server,
            request_type_info,
            response_type_info,
            pending: HashMap::new(),
            next_id: 0,
        })
    }

    // ---- SPIKE: action constructors (issue #1170 Phase 2) ----
    pub fn create_action_client(
        &mut self,
        action_name: &str,
        action_type: String,
        qos: qos::Ros2QosPolicies,
    ) -> eyre::Result<Ros2ActionClient> {
        let (package, type_name, goal_type_info, result_type_info, feedback_type_info) =
            action_type_infos(&action_type, &self.messages)?;
        let neutral_qos: dora_ros2_bridge::transport::Ros2Qos = qos.into();
        let client = match &mut self.node {
            dora_ros2_bridge::transport::Node::Dds(node) => {
                ActionClientBackend::Dds(Box::new(
                    node.as_inner_mut()
                        .create_action_client::<BridgeActionType>(
                            dora_ros2_bridge::detect_service_mapping(),
                            &parse_ros2_name(action_name)?,
                            &ros2_client::ActionTypeName::new(&package, &type_name),
                            ros2_client::action::ActionClientQosPolicies {
                                goal_service: dora_ros2_bridge::transport::dds::to_rustdds_qos(
                                    &neutral_qos,
                                ),
                                result_service: dora_ros2_bridge::transport::dds::to_rustdds_qos(
                                    &neutral_qos,
                                ),
                                cancel_service: dora_ros2_bridge::transport::dds::to_rustdds_qos(
                                    &neutral_qos,
                                ),
                                feedback_subscription:
                                    dora_ros2_bridge::transport::dds::to_rustdds_qos(&neutral_qos),
                                status_subscription:
                                    dora_ros2_bridge::transport::dds::to_rustdds_qos(&neutral_qos),
                            },
                        )
                        .map_err(|e| eyre!("failed to create action client: {e:?}"))?,
                ))
            }
            dora_ros2_bridge::transport::Node::Zenoh(node) => {
                let dora_message::descriptor::Ros2TransportConfig::Zenoh { compatibility, .. } =
                    self.transport.config
                else {
                    unreachable!()
                };
                let (keys, tokens) = zenoh_action_entities(
                    compatibility,
                    action_name,
                    &package,
                    &type_name,
                    &neutral_qos,
                )?;
                let readiness = tokens.clone();
                let client = futures::executor::block_on(ZenohActionClient::declare(
                    node,
                    &keys,
                    tokens,
                    &neutral_qos,
                    64 * 1024 * 1024,
                ))?;
                futures::executor::block_on(wait_for_action_server(
                    node.graph(),
                    &readiness,
                    Instant::now() + Duration::from_secs(20),
                ))?;
                ActionClientBackend::Zenoh(Box::new(client))
            }
        };
        Ok(Ros2ActionClient {
            client,
            goal_type_info,
            result_type_info,
            feedback_type_info,
            goals: GoalSlots::new(MAX_CONCURRENT_GOALS),
        })
    }

    pub fn create_action_server(
        &mut self,
        action_name: &str,
        action_type: String,
        qos: qos::Ros2QosPolicies,
    ) -> eyre::Result<Ros2ActionServer> {
        let (package, type_name, goal_type_info, result_type_info, feedback_type_info) =
            action_type_infos(&action_type, &self.messages)?;
        let neutral_qos: dora_ros2_bridge::transport::Ros2Qos = qos.into();
        let (server, zenoh_result_requests) =
            match &mut self.node {
                dora_ros2_bridge::transport::Node::Dds(node) => {
                    let server = node
                        .as_inner_mut()
                        .create_action_server::<BridgeActionType>(
                            dora_ros2_bridge::detect_service_mapping(),
                            &parse_ros2_name(action_name)?,
                            &ros2_client::ActionTypeName::new(&package, &type_name),
                            ros2_client::action::ActionServerQosPolicies {
                                goal_service: dora_ros2_bridge::transport::dds::to_rustdds_qos(
                                    &neutral_qos,
                                ),
                                result_service: dora_ros2_bridge::transport::dds::to_rustdds_qos(
                                    &neutral_qos,
                                ),
                                cancel_service: dora_ros2_bridge::transport::dds::to_rustdds_qos(
                                    &neutral_qos,
                                ),
                                feedback_publisher:
                                    dora_ros2_bridge::transport::dds::to_rustdds_qos(&neutral_qos),
                                status_publisher: dora_ros2_bridge::transport::dds::to_rustdds_qos(
                                    &neutral_qos,
                                ),
                            },
                        )
                        .map_err(|e| eyre!("failed to create action server: {e:?}"))?;
                    (
                        ActionServerBackend::Dds(Box::new(
                            ros2_client::action::AsyncActionServer::new(server),
                        )),
                        None,
                    )
                }
                dora_ros2_bridge::transport::Node::Zenoh(node) => {
                    let dora_message::descriptor::Ros2TransportConfig::Zenoh {
                        compatibility, ..
                    } = self.transport.config
                    else {
                        unreachable!()
                    };
                    let (keys, tokens) = zenoh_action_entities(
                        compatibility,
                        action_name,
                        &package,
                        &type_name,
                        &neutral_qos,
                    )?;
                    let server = Arc::new(futures::executor::block_on(
                        ZenohActionServer::declare(node, &keys, tokens, &neutral_qos),
                    )?);
                    let (tx, rx) = flume::bounded(MAX_CONCURRENT_GOALS * 2);
                    let receiver = server.clone();
                    std::thread::spawn(move || {
                        futures::executor::block_on(async move {
                            while let Ok(request) = receiver.get_result.recv().await {
                                if tx.send(request).is_err() {
                                    break;
                                }
                            }
                        })
                    });
                    (ActionServerBackend::Zenoh(server), Some(rx))
                }
            };
        Ok(Ros2ActionServer {
            server,
            goal_type_info,
            result_type_info,
            feedback_type_info,
            executing: GoalSlots::new(MAX_CONCURRENT_GOALS),
            zenoh_result_requests,
            pending_result_requests: HashMap::new(),
            pending_result_seq: 0,
        })
    }
    // ---- END SPIKE ----

    /// Set a ROS2 parameter. The parameter must have been declared (via the
    /// `parameters=` argument of `new_node`); the change is published on
    /// `/parameter_events` and visible to `ros2 param get` and rclpy clients.
    ///
    /// :type name: str
    /// :type value: bool | int | float | str | bytes | list
    pub fn set_parameter(&self, name: &str, value: Bound<'_, PyAny>) -> eyre::Result<()> {
        let value = py_to_parameter_value(&value)
            .wrap_err_with(|| format!("invalid value for parameter `{name}`"))?;
        // Type stability is enforced by the validator registered in `new_node`,
        // which ros2-client also applies to remote `ros2 param set` calls.
        self.dds_node()?
            .set_parameter(name, value)
            .map_err(|e| eyre!("failed to set parameter `{name}`: {e}"))
    }

    /// Get a ROS2 parameter value, or `None` if it is not set.
    ///
    /// :type name: str
    /// :rtype: bool | int | float | str | bytes | list | None
    pub fn get_parameter(&self, py: Python<'_>, name: &str) -> eyre::Result<Py<PyAny>> {
        match self.dds_node()?.get_parameter(name) {
            Some(value) => parameter_value_to_py(py, &value),
            None => Ok(py.None()),
        }
    }

    /// List the names of all declared ROS2 parameters.
    ///
    /// :rtype: list[str]
    pub fn list_parameters(&self) -> Vec<String> {
        self.dds_node()
            .map(|node| node.list_parameters())
            .unwrap_or_default()
    }

    /// Whether a ROS2 parameter with the given name is declared.
    ///
    /// :type name: str
    /// :rtype: bool
    pub fn has_parameter(&self, name: &str) -> bool {
        self.dds_node().is_ok_and(|node| node.has_parameter(name))
    }
}

/// Parse a fully-qualified ROS2 name (e.g. `/ns/sub/base`) into a
/// `ros2_client::Name`. `ros2_client::Name` requires the base name to be a
/// single token (no slashes) while the namespace may be multi-component, so we
/// split at the last `/`: `/demo_params/get_parameters` -> namespace
/// `/demo_params`, base `get_parameters`. A single-segment name (`/base` or
/// `base`) keeps namespace `/`.
fn parse_ros2_name(full: &str) -> eyre::Result<ros2_client::Name> {
    let trimmed = full.trim_end_matches('/');
    let (namespace, base) = match trimmed.rsplit_once('/') {
        Some((ns, base)) => (if ns.is_empty() { "/" } else { ns }, base),
        None => ("/", trimmed),
    };
    ros2_client::Name::new(namespace, base)
        .map_err(|e| eyre!("failed to parse ROS2 name `{full}`: {e}"))
}

/// Convert a Python value into a `ros2_client::ParameterValue`, mirroring the
/// ROS2 ParameterValue variants. `bool` is checked before `int` (Python `bool`
/// is an `int` subclass); arrays must be homogeneous and non-empty (an empty
/// list is ambiguous).
fn py_to_parameter_value(v: &Bound<'_, PyAny>) -> eyre::Result<ros2_client::ParameterValue> {
    use ros2_client::ParameterValue as P;
    if v.is_instance_of::<pyo3::types::PyBytes>() {
        return Ok(P::ByteArray(v.extract::<Vec<u8>>()?));
    }
    if v.is_instance_of::<pyo3::types::PyBool>() {
        return Ok(P::Boolean(v.extract::<bool>()?));
    }
    // A Python `int` always maps to ROS Integer; never silently demote an
    // out-of-i64-range int to Double (ROS2 has no uint64 / bigint param type).
    if v.is_instance_of::<pyo3::types::PyInt>() {
        return v
            .extract::<i64>()
            .map(P::Integer)
            .map_err(|_| eyre!("integer parameter out of i64 range"));
    }
    // Gate float on the concrete `PyFloat` type rather than a permissive
    // `extract::<f64>()`, which would also accept anything with `__float__`
    // (e.g. `numpy.int64`) and silently demote an int-typed value to Double.
    if v.is_instance_of::<pyo3::types::PyFloat>() {
        return Ok(P::Double(v.extract::<f64>()?));
    }
    if v.is_instance_of::<pyo3::types::PyString>() {
        return Ok(P::String(v.extract::<String>()?));
    }
    if let Ok(list) = v.cast::<PyList>() {
        return py_list_to_parameter_value(list);
    }
    eyre::bail!(
        "unsupported parameter value (use bool/int/float/str/bytes or a non-empty homogeneous list)"
    )
}

/// Convert a Python `list` into a ROS2 array `ParameterValue`. The element type
/// is taken from the first item and *every* element must match it: classifying
/// by repeated `extract::<Vec<T>>()` instead would silently coerce a stray
/// `bool` into an integer array (`[True, 2]` -> `[1, 2]`), violating the
/// homogeneous-array contract. Empty lists are rejected (ambiguous type).
fn py_list_to_parameter_value(
    list: &Bound<'_, PyList>,
) -> eyre::Result<ros2_client::ParameterValue> {
    use pyo3::types::{PyBool, PyFloat, PyInt, PyString};
    use ros2_client::ParameterValue as P;
    let Some(first) = list.iter().next() else {
        eyre::bail!(
            "empty list is an ambiguous parameter type (cannot infer the array element type)"
        )
    };
    // `bool` before `int`: Python `bool` is an `int` subclass.
    if first.is_instance_of::<PyBool>() {
        let mut out = Vec::with_capacity(list.len());
        for item in list.iter() {
            if !item.is_instance_of::<PyBool>() {
                eyre::bail!("non-homogeneous list: expected every element to be a bool");
            }
            out.push(item.extract::<bool>()?);
        }
        return Ok(P::BooleanArray(out));
    }
    if first.is_instance_of::<PyInt>() {
        let mut out = Vec::with_capacity(list.len());
        for item in list.iter() {
            // `bool` is an `int` subclass, so exclude it explicitly: `[1, True]`
            // must be rejected, not coerced into `[1, 1]`.
            if item.is_instance_of::<PyBool>() || !item.is_instance_of::<PyInt>() {
                eyre::bail!("non-homogeneous list: expected every element to be an int");
            }
            out.push(
                item.extract::<i64>()
                    .map_err(|_| eyre!("integer list element out of i64 range"))?,
            );
        }
        return Ok(P::IntegerArray(out));
    }
    if first.is_instance_of::<PyFloat>() {
        let mut out = Vec::with_capacity(list.len());
        for item in list.iter() {
            if !item.is_instance_of::<PyFloat>() {
                eyre::bail!("non-homogeneous list: expected every element to be a float");
            }
            out.push(item.extract::<f64>()?);
        }
        return Ok(P::DoubleArray(out));
    }
    if first.is_instance_of::<PyString>() {
        let mut out = Vec::with_capacity(list.len());
        for item in list.iter() {
            if !item.is_instance_of::<PyString>() {
                eyre::bail!("non-homogeneous list: expected every element to be a str");
            }
            out.push(item.extract::<String>()?);
        }
        return Ok(P::StringArray(out));
    }
    eyre::bail!("unsupported list element type (use bool/int/float/str)")
}

/// Convert a `ros2_client::ParameterValue` into a Python object.
fn parameter_value_to_py(
    py: Python<'_>,
    v: &ros2_client::ParameterValue,
) -> eyre::Result<Py<PyAny>> {
    use pyo3::IntoPyObject;
    use ros2_client::ParameterValue as P;
    let obj = match v {
        P::NotSet => py.None(),
        P::Boolean(b) => b
            .into_pyobject(py)
            .map_err(|e| eyre!("{e:?}"))?
            .to_owned()
            .into_any()
            .unbind(),
        P::Integer(i) => i
            .into_pyobject(py)
            .map_err(|e| eyre!("{e:?}"))?
            .into_any()
            .unbind(),
        P::Double(d) => d
            .into_pyobject(py)
            .map_err(|e| eyre!("{e:?}"))?
            .into_any()
            .unbind(),
        P::String(s) => s
            .into_pyobject(py)
            .map_err(|e| eyre!("{e:?}"))?
            .into_any()
            .unbind(),
        P::ByteArray(b) => pyo3::types::PyBytes::new(py, b).into_any().unbind(),
        P::BooleanArray(a) => a
            .clone()
            .into_pyobject(py)
            .map_err(|e| eyre!("{e:?}"))?
            .into_any()
            .unbind(),
        P::IntegerArray(a) => a
            .clone()
            .into_pyobject(py)
            .map_err(|e| eyre!("{e:?}"))?
            .into_any()
            .unbind(),
        P::DoubleArray(a) => a
            .clone()
            .into_pyobject(py)
            .map_err(|e| eyre!("{e:?}"))?
            .into_any()
            .unbind(),
        P::StringArray(a) => a
            .clone()
            .into_pyobject(py)
            .map_err(|e| eyre!("{e:?}"))?
            .into_any()
            .unbind(),
    };
    Ok(obj)
}

/// Parse a `namespace/Service` (or `namespace::Service`) type string into its
/// package + type name and the `_Request`/`_Response` `TypeInfo`s the bridge
/// uses to (de)serialize each side. Shared by the service client and server.
fn service_type_infos(
    service_type: &str,
    messages: &Arc<HashMap<String, HashMap<String, Message>>>,
) -> eyre::Result<(String, String, TypeInfo<'static>, TypeInfo<'static>)> {
    let (package, type_name) = match (service_type.split_once('/'), service_type.split_once("::")) {
        (Some(t), None) => t,
        (None, Some(t)) => t,
        _ => eyre::bail!(
            "Expected service type in the format `namespace/service` or `namespace::service`, such as `example_interfaces/AddTwoInts` but got: {service_type}"
        ),
    };

    let request_type_info = TypeInfo {
        package_name: package.to_owned().into(),
        message_name: format!("{type_name}_Request").into(),
        messages: messages.clone(),
    };
    let response_type_info = TypeInfo {
        package_name: package.to_owned().into(),
        message_name: format!("{type_name}_Response").into(),
        messages: messages.clone(),
    };

    Ok((
        package.to_owned(),
        type_name.to_owned(),
        request_type_info,
        response_type_info,
    ))
}

/// ROS2 Node Options
/// :type rosout: bool, optional
///
#[derive(Clone, Default)]
#[pyclass(from_py_object)]
#[non_exhaustive]
pub struct Ros2NodeOptions {
    pub rosout: bool,
}

#[pymethods]
impl Ros2NodeOptions {
    #[pyo3(signature = (rosout=None))]
    #[new]
    pub fn new(rosout: Option<bool>) -> Self {
        Self {
            rosout: rosout.unwrap_or(false),
        }
    }
}

impl From<Ros2NodeOptions> for ros2_client::NodeOptions {
    fn from(value: Ros2NodeOptions) -> Self {
        ros2_client::NodeOptions::new().enable_rosout(value.rosout)
    }
}

/// ROS2 Topic
/// :type rosout: bool, optional
///
/// warnings:
/// - dora Ros2 bridge functionality is considered **unstable**. It may be changed
///   at any point without it being considered a breaking change.
#[pyclass]
#[non_exhaustive]
pub struct Ros2Topic {
    topic: TopicBackend,
    type_info: TypeInfo<'static>,
}

enum TopicBackend {
    Dds(rustdds::Topic),
    Zenoh {
        key: String,
        token: TopicToken,
        qos: dora_ros2_bridge::transport::Ros2Qos,
    },
}

/// ROS2 Publisher
///
/// warnings:
/// - dora Ros2 bridge functionality is considered **unstable**. It may be changed
///   at any point without it being considered a breaking change.
#[pyclass]
#[non_exhaustive]
pub struct Ros2Publisher {
    publisher: PublisherBackend,
    type_info: TypeInfo<'static>,
}

enum PublisherBackend {
    Dds(ros2_client::Publisher<TypedValue<'static>>),
    Zenoh(NodePublisher),
}

#[pymethods]
impl Ros2Publisher {
    /// Publish a message into ROS2 topic.
    ///
    /// Remember that the data format should respect the structure of the ROS2 message using an arrow Structure.
    ///
    /// ex:
    /// ```python
    /// gripper_command.publish(
    ///     pa.array(
    ///         [
    ///             {
    ///                 "name": "gripper",
    ///                 "cmd": np.float32(5),
    ///             }
    ///         ]
    ///     ),
    /// )
    /// ```
    ///
    /// :type data: pyarrow.Array
    /// :rtype: None
    ///
    pub fn publish(&self, data: Bound<'_, PyAny>) -> eyre::Result<()> {
        let value = pyarrow_to_array_data(&data)?;
        //// add type info to ensure correct serialization (e.g. struct types
        //// and map types need to be serialized differently)
        let typed_value = TypedValue {
            value: &make_array(value),
            type_info: &self.type_info,
        };

        match &self.publisher {
            PublisherBackend::Dds(publisher) => publisher
                .publish(typed_value)
                .map_err(|e| e.forget_data())
                .context("publish failed")?,
            PublisherBackend::Zenoh(publisher) => {
                let cdr = serialize_raw_cdr(&typed_value)?;
                futures::executor::block_on(publisher.publish(&cdr))?;
            }
        }
        Ok(())
    }
}

/// ROS2 Subscription
///
///
/// warnings:
/// - dora Ros2 bridge functionality is considered **unstable**. It may be changed
///   at any point without it being considered a breaking change.
#[pyclass]
#[non_exhaustive]
pub struct Ros2Subscription {
    deserializer: StructDeserializer<'static>,
    subscription: SubscriptionBackend,
}

enum SubscriptionBackend {
    Dds(Option<ros2_client::Subscription<ArrayData>>),
    Zenoh(Option<NodeSubscription<ArrayData>>),
}

#[pymethods]
impl Ros2Subscription {
    pub fn next(&self, py: Python) -> eyre::Result<Option<Py<PyAny>>> {
        let value = match &self.subscription {
            SubscriptionBackend::Dds(subscription) => {
                let message = subscription
                    .as_ref()
                    .context("subscription was already used")?
                    .take_seed(self.deserializer.clone())
                    .context("failed to take next message from subscription")?;
                let Some((value, _)) = message else {
                    return Ok(None);
                };
                value
            }
            SubscriptionBackend::Zenoh(subscription) => {
                let Some((value, _)) = subscription
                    .as_ref()
                    .context("subscription was already used")?
                    .try_recv()?
                else {
                    return Ok(None);
                };
                value
            }
        };

        let message = value.to_pyarrow(py)?.unbind();
        // TODO: add `info`

        Ok(Some(message))
    }
}

impl Ros2Subscription {
    pub fn into_stream(&mut self) -> eyre::Result<Ros2SubscriptionStream> {
        let subscription = match &mut self.subscription {
            SubscriptionBackend::Dds(value) => Ros2SubscriptionStreamBackend::Dds(
                value.take().context("subscription was already used")?,
            ),
            SubscriptionBackend::Zenoh(value) => Ros2SubscriptionStreamBackend::Zenoh(
                value.take().context("subscription was already used")?,
            ),
        };

        Ok(Ros2SubscriptionStream {
            deserializer: self.deserializer.clone(),
            subscription,
        })
    }
}

pub struct Ros2SubscriptionStream {
    deserializer: StructDeserializer<'static>,
    subscription: Ros2SubscriptionStreamBackend,
}

enum Ros2SubscriptionStreamBackend {
    Dds(ros2_client::Subscription<ArrayData>),
    Zenoh(NodeSubscription<ArrayData>),
}

impl Ros2SubscriptionStream {
    pub fn as_stream(&self) -> std::pin::Pin<Box<dyn Stream<Item = eyre::Result<ArrayData>> + '_>> {
        match &self.subscription {
            Ros2SubscriptionStreamBackend::Dds(subscription) => Box::pin(
                subscription
                    .async_stream_seed(self.deserializer.clone())
                    .map(|result| result.map(|(data, _)| data).map_err(Into::into)),
            ),
            Ros2SubscriptionStreamBackend::Zenoh(subscription) => Box::pin(
                futures::stream::unfold(subscription, |subscription| async move {
                    Some((
                        subscription
                            .recv_async()
                            .await
                            .map(|(data, _)| data)
                            .map_err(Into::into),
                        subscription,
                    ))
                }),
            ),
        }
    }
}

impl Stream for Ros2SubscriptionStream {
    type Item = eyre::Result<ArrayData>;

    fn poll_next(
        self: std::pin::Pin<&mut Self>,
        cx: &mut std::task::Context<'_>,
    ) -> std::task::Poll<Option<Self::Item>> {
        let s = self.as_stream();
        futures::pin_mut!(s);
        s.poll_next_unpin(cx)
    }
}

/// ROS2 service client. Create via [`Ros2Node::create_service_client`].
///
/// warnings:
/// - dora Ros2 bridge functionality is considered **unstable**. It may be changed
///   at any point without it being considered a breaking change.
// ---- SPIKE: action types + helper (issue #1170 Phase 2) ----
fn action_type_infos(
    action_type: &str,
    messages: &Arc<HashMap<String, HashMap<String, Message>>>,
) -> eyre::Result<(
    String,
    String,
    TypeInfo<'static>,
    TypeInfo<'static>,
    TypeInfo<'static>,
)> {
    let (package, type_name) = match (action_type.split_once('/'), action_type.split_once("::")) {
        (Some(t), None) => t,
        (None, Some(t)) => t,
        _ => eyre::bail!(
            "Expected action type `namespace/Action` or `namespace::Action`, got: {action_type}"
        ),
    };
    let mk = |suffix: &str| TypeInfo {
        package_name: package.to_owned().into(),
        message_name: format!("{type_name}_{suffix}").into(),
        messages: messages.clone(),
    };
    Ok((
        package.to_owned(),
        type_name.to_owned(),
        mk("Goal"),
        mk("Result"),
        mk("Feedback"),
    ))
}

fn zenoh_action_entities(
    compatibility: dora_message::descriptor::RmwZenohCompatibility,
    action_name: &str,
    package: &str,
    action_type: &str,
    qos: &dora_ros2_bridge::transport::Ros2Qos,
) -> eyre::Result<(ActionKeys, ActionTokens)> {
    let resolver = TypeDescriptionResolver::from_ament_prefix_path();
    let identities = resolve_action(compatibility, package, action_type, &resolver)?;
    let find = |suffix: &str| {
        identities
            .iter()
            .find(|identity| identity.ros_name.ends_with(suffix))
            .cloned()
            .with_context(|| format!("missing action identity {suffix}"))
    };
    let send_goal = find("_SendGoal")?;
    let get_result = find("_GetResult")?;
    let feedback = find("_FeedbackMessage")?;
    let cancel = resolve_service(compatibility, "action_msgs", "CancelGoal", &resolver)?;
    let status = resolve_message(compatibility, "action_msgs", "GoalStatusArray", &resolver)?;
    let endpoints = dora_ros2_bridge::transport::action::ActionEndpoints::new(
        action_name,
        package,
        action_type,
    );
    let domain = std::env::var("ROS_DOMAIN_ID")
        .ok()
        .and_then(|value| value.parse().ok())
        .unwrap_or(0);
    let key = |name: &str, identity| -> eyre::Result<String> {
        Ok(DataKey::new(domain, name, identity)?.as_str().into())
    };
    let qos_string = ZenohQosMapping::from_ros_qos(qos).to_string();
    let status_qos = dora_ros2_bridge::transport::action::zenoh::action_status_qos(qos);
    let token =
        |name: &str,
         identity: &dora_ros2_bridge::transport::zenoh::compatibility::RosTypeIdentity| {
            TopicToken {
                name: name.into(),
                type_name: identity.dds_name.clone(),
                type_hash: identity.key_hash_component(),
                qos: qos_string.clone(),
            }
        };
    Ok((
        ActionKeys {
            send_goal: key(&endpoints.send_goal.name, &send_goal)?,
            get_result: key(&endpoints.get_result.name, &get_result)?,
            cancel_goal: key(&endpoints.cancel_goal.name, &cancel)?,
            feedback: key(&endpoints.feedback.name, &feedback)?,
            status: key(&endpoints.status.name, &status)?,
        },
        ActionTokens {
            send_goal: token(&endpoints.send_goal.name, &send_goal),
            get_result: token(&endpoints.get_result.name, &get_result),
            cancel_goal: token(&endpoints.cancel_goal.name, &cancel),
            feedback: token(&endpoints.feedback.name, &feedback),
            status: TopicToken {
                qos: ZenohQosMapping::from_ros_qos(&status_qos).to_string(),
                ..token(&endpoints.status.name, &status)
            },
        },
    ))
}

#[pyclass]
#[non_exhaustive]
pub struct Ros2ActionClient {
    client: ActionClientBackend,
    goal_type_info: TypeInfo<'static>,
    result_type_info: TypeInfo<'static>,
    feedback_type_info: TypeInfo<'static>,
    goals: GoalSlots<String, ros2_client::action::GoalId>,
}

enum ActionClientBackend {
    Dds(Box<ros2_client::action::ActionClient<BridgeActionType>>),
    Zenoh(Box<ZenohActionClient>),
}

#[pyclass]
#[non_exhaustive]
pub struct Ros2ActionServer {
    server: ActionServerBackend,
    goal_type_info: TypeInfo<'static>,
    result_type_info: TypeInfo<'static>,
    feedback_type_info: TypeInfo<'static>,
    executing: GoalSlots<String, PythonServerGoal>,
    zenoh_result_requests:
        Option<flume::Receiver<dora_ros2_bridge::transport::zenoh::service::ServiceRequest>>,
    pending_result_requests: HashMap<String, (dora_ros2_bridge::transport::RequestId, u64)>,
    /// Monotonic counter used to evict the oldest stashed request on overflow.
    pending_result_seq: u64,
}

enum ActionServerBackend {
    Dds(Box<ros2_client::action::AsyncActionServer<BridgeActionType>>),
    Zenoh(Arc<ZenohActionServer>),
}

enum PythonServerGoal {
    Dds(
        ros2_client::action::ExecutingGoalHandle<BridgeMessage>,
        Instant,
    ),
    Zenoh(ros2_client::action::GoalId, Instant),
}

impl PythonServerGoal {
    fn created(&self) -> Instant {
        match self {
            Self::Dds(_, created) | Self::Zenoh(_, created) => *created,
        }
    }
}
const ACTION_GOAL_TIMEOUT_S: f64 = 30.0;
const ACTION_RESULT_TIMEOUT_S: f64 = 300.0;

#[pymethods]
impl Ros2ActionClient {
    // SPIKE: client block_on borrow proof — async_send_goal borrows &self.client
    // and is raced against Delay inside one block_on.
    #[pyo3(signature = (goal, timeout_s=None))]
    pub fn send_goal(
        &mut self,
        goal: Bound<'_, PyAny>,
        timeout_s: Option<f64>,
    ) -> eyre::Result<Option<String>> {
        if self.goals.len() >= MAX_CONCURRENT_GOALS {
            eyre::bail!("max concurrent goals ({MAX_CONCURRENT_GOALS}) reached");
        }
        let py = goal.py();
        let array_data = pyarrow_to_array_data(&goal)?;
        let timeout = timeout_or(timeout_s, ACTION_GOAL_TIMEOUT_S);
        let goal_type_info = self.goal_type_info.clone();
        let (goal_id, accepted) = py.detach(|| {
            let _guard = TypeInfoGuard::serialize(goal_type_info);
            match &self.client {
                ActionClientBackend::Dds(client) => futures::executor::block_on(async {
                    let send = client.async_send_goal(BridgeMessage(Some(array_data)));
                    futures::pin_mut!(send);
                    let delay = futures_timer::Delay::new(timeout);
                    match futures::future::select(send, delay).await {
                        futures::future::Either::Left((r, _)) => r
                            .map(|(id, response)| (id, response.accepted))
                            .map_err(|e| eyre!("failed to send action goal: {e:?}")),
                        futures::future::Either::Right(_) => {
                            eyre::bail!("action goal send timed out after {timeout:?}")
                        }
                    }
                }),
                ActionClientBackend::Zenoh(client) => {
                    let goal_id = ros2_client::action::GoalId::new_random();
                    let request = serialize_cdr(&ros2_client::action::SendGoalRequest {
                        goal_id,
                        goal: BridgeMessage(Some(array_data)),
                    })?;
                    let response =
                        futures::executor::block_on(client.send_goal.call(request, timeout))?;
                    let response: ros2_client::action::SendGoalResponse =
                        deserialize_cdr(&response)?;
                    Ok((goal_id, response.accepted))
                }
            }
        })?;
        if !accepted {
            return Ok(None);
        }
        let id = goal_id.uuid.to_string();
        self.goals.insert(id.clone(), goal_id)?;
        Ok(Some(id))
    }

    /// Poll up to `timeout_s` (default 1.0) for one feedback message for `goal_id`.
    #[pyo3(signature = (goal_id, timeout_s=None))]
    pub fn take_feedback(
        &self,
        py: Python<'_>,
        goal_id: &str,
        timeout_s: Option<f64>,
    ) -> eyre::Result<Option<Py<PyAny>>> {
        let gid = *self
            .goals
            .get(goal_id)
            .with_context(|| format!("unknown goal_id {goal_id:?}"))?;
        let timeout = timeout_or(timeout_s, 1.0);
        let feedback_type_info = self.feedback_type_info.clone();
        let msg = py.detach(|| {
            let _guard = TypeInfoGuard::deserialize(feedback_type_info);
            match &self.client {
                ActionClientBackend::Dds(client) => futures::executor::block_on(async {
                    // feedback_stream is !Unpin (a FilterMap over async closures);
                    // pin the STREAM so `.next()` (which requires Self: Unpin) works.
                    let s = client.feedback_stream(gid);
                    futures::pin_mut!(s);
                    let delay = futures_timer::Delay::new(timeout);
                    match futures::future::select(s.next(), delay).await {
                        futures::future::Either::Left((Some(r), _)) => r
                            .map(|fb| fb.0)
                            .map_err(|e| eyre!("feedback read error: {e:?}")),
                        futures::future::Either::Left((None, _)) => Ok(None),
                        futures::future::Either::Right(_) => Ok(None),
                    }
                }),
                ActionClientBackend::Zenoh(client) => futures::executor::block_on(async {
                    let recv = client.feedback.recv_async();
                    futures::pin_mut!(recv);
                    let delay = futures_timer::Delay::new(timeout);
                    match futures::future::select(recv, delay).await {
                        futures::future::Either::Left((result, _)) => {
                            let (payload, _) = result?;
                            let message: ros2_client::action::FeedbackMessage<BridgeMessage> =
                                deserialize_cdr(&payload)?;
                            Ok(message.feedback.0.filter(|_| message.goal_id == gid))
                        }
                        futures::future::Either::Right(_) => Ok(None),
                    }
                }),
            }
        })?;
        match msg {
            Some(data) => Ok(Some(data.to_pyarrow(py)?.unbind())),
            None => Ok(None),
        }
    }

    /// Request and block for the terminal result of `goal_id`. Returns
    /// (status_str, result_array), or None on timeout (slot kept for retry).
    #[pyo3(signature = (goal_id, timeout_s=None))]
    pub fn take_result(
        &mut self,
        py: Python<'_>,
        goal_id: &str,
        timeout_s: Option<f64>,
    ) -> eyre::Result<Option<(String, Py<PyAny>)>> {
        let gid = *self
            .goals
            .get(goal_id)
            .with_context(|| format!("unknown goal_id {goal_id:?}"))?;
        let timeout = timeout_or(timeout_s, ACTION_RESULT_TIMEOUT_S);
        let result_type_info = self.result_type_info.clone();
        let out = py.detach(|| {
            let _guard = TypeInfoGuard::deserialize(result_type_info);
            match &self.client {
                ActionClientBackend::Dds(client) => futures::executor::block_on(async {
                    let req = client.async_request_result(gid);
                    futures::pin_mut!(req);
                    let delay = futures_timer::Delay::new(timeout);
                    match futures::future::select(req, delay).await {
                        futures::future::Either::Left((r, _)) => {
                            r.map(Some).map_err(|e| eyre!("result error: {e:?}"))
                        }
                        futures::future::Either::Right(_) => Ok(None),
                    }
                }),
                ActionClientBackend::Zenoh(client) => {
                    let request =
                        serialize_cdr(&ros2_client::action::GetResultRequest { goal_id: gid })?;
                    let response =
                        futures::executor::block_on(client.get_result.call(request, timeout));
                    match response {
                        Ok(payload) => {
                            let response: ros2_client::action::GetResultResponse<BridgeMessage> =
                                deserialize_cdr(&payload)?;
                            Ok(Some((response.status, response.result)))
                        }
                        Err(dora_ros2_bridge::transport::zenoh::service::ServiceError::Timeout) => {
                            Ok(None)
                        }
                        Err(error) => Err(error.into()),
                    }
                }
            }
        })?;
        match out {
            Some((status, msg)) => {
                self.goals.remove(&goal_id.to_owned());
                let status_str = status_enum_to_str(status);
                let data = msg.0.context("action result contained no data")?;
                Ok(Some((status_str, data.to_pyarrow(py)?.unbind())))
            }
            None => Ok(None),
        }
    }

    /// Cancel a goal (`goal_id`) or all goals (`None`). Returns the
    /// CancelGoalResponse return_code as an int. Frees the in-flight slot(s).
    #[pyo3(signature = (goal_id=None, timeout_s=None))]
    pub fn cancel(
        &mut self,
        py: Python<'_>,
        goal_id: Option<&str>,
        timeout_s: Option<f64>,
    ) -> eyre::Result<i8> {
        let timeout = timeout_or(timeout_s, 10.0);
        let gid = match goal_id {
            Some(s) => *self
                .goals
                .get(s)
                .with_context(|| format!("unknown goal_id {s:?}"))?,
            None => ros2_client::action::GoalId::ZERO,
        };
        let stamp = ros2_client::builtin_interfaces::Time::ZERO;
        let code = py.detach(|| match &self.client {
            ActionClientBackend::Dds(client) => futures::executor::block_on(async {
                let fut = client.async_cancel_goal(gid, stamp);
                futures::pin_mut!(fut);
                let delay = futures_timer::Delay::new(timeout);
                match futures::future::select(fut, delay).await {
                    futures::future::Either::Left((r, _)) => r
                        .map(|resp| resp.return_code as i8)
                        .map_err(|e| eyre!("cancel error: {e:?}")),
                    futures::future::Either::Right(_) => eyre::bail!("cancel timed out"),
                }
            }),
            ActionClientBackend::Zenoh(client) => {
                #[derive(serde::Serialize)]
                struct CancelRequest {
                    goal_info: ros2_client::action::GoalInfo,
                }
                let request = serialize_cdr(&CancelRequest {
                    goal_info: ros2_client::action::GoalInfo {
                        goal_id: gid,
                        stamp,
                    },
                })?;
                let response =
                    futures::executor::block_on(client.cancel_goal.call(request, timeout))?;
                let response: ros2_client::action::CancelGoalResponse = deserialize_cdr(&response)?;
                Ok(response.return_code as i8)
            }
        })?;
        match goal_id {
            Some(s) => {
                self.goals.remove(&s.to_owned());
            }
            None => {
                self.goals.clear();
            }
        }
        Ok(code)
    }
}

#[pymethods]
impl Ros2ActionServer {
    #[pyo3(signature = (timeout_s=None))]
    pub fn take_goal(
        &mut self,
        py: Python<'_>,
        timeout_s: Option<f64>,
    ) -> eyre::Result<Option<(String, Py<PyAny>)>> {
        let timeout = timeout_or(timeout_s, 1.0);
        let goal_type_info = self.goal_type_info.clone();
        let taken: Option<(PythonServerGoal, ArrayData)> = py.detach(|| {
            let _guard = TypeInfoGuard::deserialize(goal_type_info);
            match &self.server {
                ActionServerBackend::Dds(server) => futures::executor::block_on(async {
                    let recv = server.receive_new_goal();
                    futures::pin_mut!(recv);
                    let delay = futures_timer::Delay::new(timeout);
                    let handle = match futures::future::select(recv, delay).await {
                        futures::future::Either::Left((result, _)) => {
                            result.map_err(|error| eyre!("receive_new_goal: {error:?}"))?
                        }
                        futures::future::Either::Right(_) => {
                            return Ok::<_, eyre::Report>(None);
                        }
                    };
                    let data = server.get_new_goal(handle.clone());
                    let accepted = server
                        .accept_goal(handle)
                        .await
                        .map_err(|error| eyre!("accept_goal: {error:?}"))?;
                    let executing = server
                        .start_executing_goal(accepted)
                        .await
                        .map_err(|error| eyre!("start_executing_goal: {error:?}"))?;
                    Ok(data
                        .and_then(|message| message.0)
                        .map(|data| (PythonServerGoal::Dds(executing, Instant::now()), data)))
                }),
                ActionServerBackend::Zenoh(server) => futures::executor::block_on(async {
                    let recv = server.send_goal.recv();
                    futures::pin_mut!(recv);
                    let delay = futures_timer::Delay::new(timeout);
                    let request = match futures::future::select(recv, delay).await {
                        futures::future::Either::Left((result, _)) => result?,
                        futures::future::Either::Right(_) => {
                            return Ok::<_, eyre::Report>(None);
                        }
                    };
                    let decoded: ros2_client::action::SendGoalRequest<BridgeMessage> =
                        deserialize_cdr(&request.payload)?;
                    let accepted =
                        self.executing.len() < MAX_CONCURRENT_GOALS && decoded.goal.0.is_some();
                    let response = serialize_cdr(&ros2_client::action::SendGoalResponse {
                        accepted,
                        stamp: ros2_client::builtin_interfaces::Time::ZERO,
                    })?;
                    server.send_goal.reply(request.id, &response).await?;
                    Ok(decoded.goal.0.filter(|_| accepted).map(|data| {
                        (
                            PythonServerGoal::Zenoh(decoded.goal_id, Instant::now()),
                            data,
                        )
                    }))
                }),
            }
        })?;
        let Some((executing, arr)) = taken else {
            return Ok(None);
        };
        let goal_id = match &executing {
            PythonServerGoal::Dds(handle, _) => handle.goal_id().uuid.to_string(),
            PythonServerGoal::Zenoh(id, _) => id.uuid.to_string(),
        };
        while self.executing.len() >= MAX_CONCURRENT_GOALS {
            let Some(oldest) = self
                .executing
                .iter()
                .min_by_key(|(_, goal)| goal.created())
                .map(|(k, _)| k.clone())
            else {
                break;
            };
            self.executing.remove(&oldest);
        }
        self.executing.insert(goal_id.clone(), executing)?;
        publish_python_zenoh_status(&self.server, &self.executing)?;
        Ok(Some((goal_id, arr.to_pyarrow(py)?.unbind())))
    }

    pub fn send_feedback(&self, goal_id: &str, feedback: Bound<'_, PyAny>) -> eyre::Result<()> {
        let py = feedback.py();
        let goal = self
            .executing
            .get(goal_id)
            .with_context(|| format!("unknown/finished goal {goal_id}"))?;
        let array_data = pyarrow_to_array_data(&feedback)?;
        let feedback_type_info = self.feedback_type_info.clone();
        py.detach(|| {
            let _guard = TypeInfoGuard::serialize(feedback_type_info);
            match (&self.server, goal) {
                (ActionServerBackend::Dds(server), PythonServerGoal::Dds(handle, _)) => {
                    futures::executor::block_on(
                        server.publish_feedback(handle.clone(), BridgeMessage(Some(array_data))),
                    )
                    .map_err(|error| eyre!("publish_feedback: {error:?}"))
                }
                (ActionServerBackend::Zenoh(server), PythonServerGoal::Zenoh(id, _)) => {
                    let payload = serialize_cdr(&ros2_client::action::FeedbackMessage {
                        goal_id: *id,
                        feedback: BridgeMessage(Some(array_data)),
                    })?;
                    futures::executor::block_on(server.feedback.publish(&payload))?;
                    Ok(())
                }
                _ => eyre::bail!("action goal belongs to a different transport"),
            }
        })?;
        Ok(())
    }

    /// Send the terminal result and retire the goal. `status`:
    /// succeeded/aborted/canceled/None(=succeeded)/other(=aborted).
    #[pyo3(signature = (goal_id, result, status=None, timeout_s=None))]
    pub fn send_result(
        &mut self,
        goal_id: &str,
        result: Bound<'_, PyAny>,
        status: Option<&str>,
        timeout_s: Option<f64>,
    ) -> eyre::Result<()> {
        let py = result.py();
        let goal = self
            .executing
            .get(goal_id)
            .with_context(|| format!("unknown/finished goal {goal_id}"))?;
        let end = map_status(status);
        let array_data = pyarrow_to_array_data(&result)?;
        let timeout = timeout_or(timeout_s, ACTION_RESULT_TIMEOUT_S);
        let result_type_info = self.result_type_info.clone();
        py.detach(|| {
            let _guard = TypeInfoGuard::serialize(result_type_info);
            match (&self.server, goal) {
                (ActionServerBackend::Dds(server), PythonServerGoal::Dds(handle, _)) => {
                    futures::executor::block_on(async {
                        let send = server.send_result_response(
                            handle.clone(),
                            end,
                            BridgeMessage(Some(array_data)),
                        );
                        futures::pin_mut!(send);
                        let delay = futures_timer::Delay::new(timeout);
                        match futures::future::select(send, delay).await {
                            futures::future::Either::Left((result, _)) => {
                                result.map_err(|error| eyre!("send_result_response: {error:?}"))
                            }
                            futures::future::Either::Right(_) => {
                                eyre::bail!("send_result timed out after {timeout:?}")
                            }
                        }
                    })
                }
                (ActionServerBackend::Zenoh(server), PythonServerGoal::Zenoh(id, _)) => {
                    let status = end_status_to_status(end);
                    let response = serialize_cdr(&ros2_client::action::GetResultResponse {
                        status,
                        result: BridgeMessage(Some(array_data)),
                    })?;
                    let request_id = self
                        .pending_result_requests
                        .remove(goal_id)
                        .map(|(id, _)| id)
                        .or_else(|| {
                            let receiver = self.zenoh_result_requests.as_ref()?;
                            let deadline = Instant::now() + timeout;
                            let mut malformed = 0u64;
                            loop {
                                let remaining = deadline.checked_duration_since(Instant::now())?;
                                let request = receiver.recv_timeout(remaining).ok()?;
                                // A malformed, peer-controlled payload must not abort a
                                // legitimate pending result: skip it and keep draining,
                                // exactly like the valid-but-non-matching branch below.
                                // Log only the first drop per call so a malformed flood
                                // can't spam stderr.
                                let decoded: ros2_client::action::GetResultRequest =
                                    match deserialize_cdr(&request.payload) {
                                        Ok(decoded) => decoded,
                                        Err(error) => {
                                            malformed += 1;
                                            if malformed == 1 {
                                                eprintln!(
                                                    "dropping malformed get-result request(s): {error:?}"
                                                );
                                            }
                                            continue;
                                        }
                                    };
                                let key = decoded.goal_id.uuid.to_string();
                                if decoded.goal_id == *id {
                                    return Some(request.id);
                                }
                                // The key is a peer-controlled goal UUID; a flood of
                                // non-matching get-result queries must not grow this
                                // stash without bound. Cap it and evict on overflow.
                                if self.pending_result_requests.len() >= MAX_PENDING_REQUESTS
                                    && !self.pending_result_requests.contains_key(&key)
                                {
                                    // Evict the OLDEST stash entry (lowest seq), not an
                                    // arbitrary HashMap entry, so a flood can't drop a
                                    // recently-stashed legitimate request.
                                    if let Some(evict) = self
                                        .pending_result_requests
                                        .iter()
                                        .min_by_key(|(_, (_, seq))| *seq)
                                        .map(|(evict_key, _)| evict_key.clone())
                                    {
                                        self.pending_result_requests.remove(&evict);
                                    }
                                }
                                let seq = self.pending_result_seq;
                                self.pending_result_seq = self.pending_result_seq.wrapping_add(1);
                                self.pending_result_requests.insert(key, (request.id, seq));
                            }
                        })
                        .context("get-result request timed out")?;
                    futures::executor::block_on(server.get_result.reply(request_id, &response))?;
                    Ok(())
                }
                _ => eyre::bail!("action goal belongs to a different transport"),
            }
        })?;
        self.executing.remove(goal_id);
        publish_python_zenoh_status(&self.server, &self.executing)?;
        Ok(())
    }

    /// Poll up to `timeout_s` (default 1.0) for a cancel request. Returns the
    /// goal_id strings the client asked to cancel that we still track, or None.
    #[pyo3(signature = (timeout_s=None))]
    pub fn take_cancel(
        &self,
        py: Python<'_>,
        timeout_s: Option<f64>,
    ) -> eyre::Result<Option<Vec<String>>> {
        let timeout = timeout_or(timeout_s, 1.0);
        py.detach(|| match &self.server {
            ActionServerBackend::Dds(server) => futures::executor::block_on(async {
                let recv = server.receive_cancel_request();
                futures::pin_mut!(recv);
                let delay = futures_timer::Delay::new(timeout);
                let cancel_handle = match futures::future::select(recv, delay).await {
                    futures::future::Either::Left((r, _)) => {
                        r.map_err(|e| eyre!("receive_cancel_request: {e:?}"))?
                    }
                    futures::future::Either::Right(_) => return Ok(None),
                };
                let goals: Vec<_> = cancel_handle.goals().collect();
                server
                    .respond_to_cancel_requests(&cancel_handle, goals.iter().copied())
                    .await
                    .map_err(|e| eyre!("respond_to_cancel_requests: {e:?}"))?;
                let mine: Vec<String> = goals
                    .iter()
                    .map(|g| g.uuid.to_string())
                    .filter(|s| self.executing.contains_key(s))
                    .collect();
                Ok(Some(mine))
            }),
            ActionServerBackend::Zenoh(server) => futures::executor::block_on(async {
                let recv = server.cancel_goal.recv();
                futures::pin_mut!(recv);
                let delay = futures_timer::Delay::new(timeout);
                let request = match futures::future::select(recv, delay).await {
                    futures::future::Either::Left((result, _)) => result?,
                    futures::future::Either::Right(_) => return Ok(None),
                };
                #[derive(serde::Deserialize)]
                struct CancelRequest {
                    goal_info: ros2_client::action::GoalInfo,
                }
                let decoded: CancelRequest = deserialize_cdr(&request.payload)?;
                let requested = decoded.goal_info.goal_id.uuid;
                let mine: Vec<String> = self
                    .executing
                    .iter()
                    .filter_map(|(key, goal)| match goal {
                        PythonServerGoal::Zenoh(id, _)
                            if requested.is_nil() || id.uuid == requested =>
                        {
                            Some(key.clone())
                        }
                        _ => None,
                    })
                    .collect();
                let goals_canceling = mine
                    .iter()
                    .filter_map(|key| match self.executing.get(key) {
                        Some(PythonServerGoal::Zenoh(id, _)) => {
                            Some(ros2_client::action::GoalInfo {
                                goal_id: *id,
                                stamp: ros2_client::builtin_interfaces::Time::ZERO,
                            })
                        }
                        _ => None,
                    })
                    .collect();
                let response = ros2_client::action::CancelGoalResponse {
                    return_code: if mine.is_empty() {
                        ros2_client::action_msgs::CancelGoalResponseEnum::UnknownGoal
                    } else {
                        ros2_client::action_msgs::CancelGoalResponseEnum::None
                    },
                    goals_canceling,
                };
                server
                    .cancel_goal
                    .reply(request.id, &serialize_cdr(&response)?)
                    .await?;
                Ok(Some(mine))
            }),
        })
    }
}

fn end_status_to_status(
    status: ros2_client::action::GoalEndStatus,
) -> ros2_client::action::GoalStatusEnum {
    match status {
        ros2_client::action::GoalEndStatus::Succeeded => {
            ros2_client::action::GoalStatusEnum::Succeeded
        }
        ros2_client::action::GoalEndStatus::Canceled => {
            ros2_client::action::GoalStatusEnum::Canceled
        }
        ros2_client::action::GoalEndStatus::Aborted => ros2_client::action::GoalStatusEnum::Aborted,
    }
}

fn publish_python_zenoh_status(
    server: &ActionServerBackend,
    goals: &GoalSlots<String, PythonServerGoal>,
) -> eyre::Result<()> {
    let ActionServerBackend::Zenoh(server) = server else {
        return Ok(());
    };
    let status = ros2_client::action_msgs::GoalStatusArray {
        status_list: goals
            .iter()
            .filter_map(|(_, goal)| match goal {
                PythonServerGoal::Zenoh(id, _) => Some(ros2_client::action_msgs::GoalStatus {
                    goal_info: ros2_client::action::GoalInfo {
                        goal_id: *id,
                        stamp: ros2_client::builtin_interfaces::Time::ZERO,
                    },
                    status: ros2_client::action::GoalStatusEnum::Executing,
                }),
                _ => None,
            })
            .collect(),
    };
    futures::executor::block_on(server.status.publish(&serialize_cdr(&status)?))?;
    Ok(())
}

/// Convert an optional `timeout_s` (seconds) from Python into a `Duration`,
/// falling back to `default_s` for `None` and for any non-finite, negative, or
/// overflowing value. Guards against `Duration::from_secs_f64` panicking on a
/// NaN/inf/negative argument (a process crash from public API input).
fn timeout_or(timeout_s: Option<f64>, default_s: f64) -> Duration {
    let secs = timeout_s.unwrap_or(default_s);
    Duration::try_from_secs_f64(secs).unwrap_or_else(|_| Duration::from_secs_f64(default_s))
}

fn status_enum_to_str(status: ros2_client::action::GoalStatusEnum) -> String {
    use ros2_client::action::GoalStatusEnum as S;
    match status {
        S::Succeeded => "succeeded",
        S::Aborted => "aborted",
        S::Canceled => "canceled",
        S::Accepted => "accepted",
        S::Executing => "executing",
        S::Canceling => "canceling",
        S::Unknown => "unknown",
    }
    .to_string()
}

fn map_status(status: Option<&str>) -> ros2_client::action::GoalEndStatus {
    use ros2_client::action::GoalEndStatus as E;
    match status {
        None | Some("succeeded") => E::Succeeded,
        Some("aborted") => E::Aborted,
        Some("canceled") => E::Canceled,
        Some(_) => E::Aborted,
    }
}
// ---- END SPIKE ----

#[pyclass]
#[non_exhaustive]
pub struct Ros2ServiceClient {
    client: ServiceClientBackend,
    request_type_info: TypeInfo<'static>,
    response_type_info: TypeInfo<'static>,
}

enum ServiceClientBackend {
    Dds(Box<ros2_client::Client<BridgeServiceType>>),
    Zenoh(NodeServiceClient),
}

#[pymethods]
impl Ros2ServiceClient {
    /// Send a request and block until the response arrives.
    ///
    /// The request must match the service's `_Request` structure as an Arrow
    /// struct (e.g. `pa.array([{"a": 2, "b": 3}])`). Returns the `_Response`
    /// as a pyarrow array. Raises on timeout (default 30s).
    ///
    /// :type request: pyarrow.Array
    /// :type timeout_s: float, optional
    /// :rtype: pyarrow.Array
    #[pyo3(signature = (request, timeout_s=None))]
    pub fn call(
        &self,
        request: Bound<'_, PyAny>,
        timeout_s: Option<f64>,
    ) -> eyre::Result<Py<PyAny>> {
        let py = request.py();
        let array_data = pyarrow_to_array_data(&request)?;

        // Serialize the request under the request TypeInfo (guard clears the
        // thread-local on drop).
        if let ServiceClientBackend::Zenoh(client) = &self.client {
            let request_value = make_array(array_data.clone());
            let request = serialize_raw_cdr(&TypedValue {
                value: &request_value,
                type_info: &self.request_type_info,
            })?;
            let timeout = timeout_or(timeout_s, 30.0);
            let response =
                py.detach(|| futures::executor::block_on(client.call(request, timeout)))?;
            let data =
                deserialize_raw_cdr(&response, self.response_type_info.clone(), 64 * 1024 * 1024)?;
            return Ok(data.to_pyarrow(py)?.unbind());
        }
        let ServiceClientBackend::Dds(client) = &self.client else {
            unreachable!()
        };
        let req_id = {
            let _guard = TypeInfoGuard::serialize(self.request_type_info.clone());
            client
                .send_request(BridgeMessage(Some(array_data)))
                .map_err(|e| eyre!("failed to send service request: {e:?}"))?
        };

        // Release the GIL while blocking on the network so other Python threads
        // (e.g. a service server producing this very response) can run. The
        // TypeInfoGuard lives inside the closure, which runs on the same OS
        // thread, so the deserialize thread-local stays correct.
        let timeout = timeout_or(timeout_s, 30.0);
        let response_type_info = self.response_type_info.clone();
        let response = py.detach(|| {
            let _guard = TypeInfoGuard::deserialize(response_type_info);
            futures::executor::block_on(async {
                let recv = client.async_receive_response(req_id);
                futures::pin_mut!(recv);
                let delay = futures_timer::Delay::new(timeout);
                match futures::future::select(recv, delay).await {
                    futures::future::Either::Left((result, _)) => {
                        result.map_err(|e| eyre!("failed to receive service response: {e:?}"))
                    }
                    futures::future::Either::Right(_) => {
                        eyre::bail!("service response timed out after {timeout:?}")
                    }
                }
            })
        })?;

        let data = response.0.context("service response contained no data")?;
        Ok(data.to_pyarrow(py)?.unbind())
    }
}

/// ROS2 service server. Create via [`Ros2Node::create_service_server`].
///
/// warnings:
/// - dora Ros2 bridge functionality is considered **unstable**. It may be changed
///   at any point without it being considered a breaking change.
#[pyclass]
#[non_exhaustive]
pub struct Ros2ServiceServer {
    server: ServiceServerBackend,
    request_type_info: TypeInfo<'static>,
    response_type_info: TypeInfo<'static>,
    // Maps the integer id handed to Python back to the ROS2 request id (plus the
    // time it was taken), so `send_response` can reply to the correct (possibly
    // out-of-order) request. The insertion time bounds the map: stale entries
    // from requests that never got a response are evicted in `take_request`.
    pending: HashMap<u64, (PendingServiceRequest, Instant)>,
    next_id: u64,
}

enum ServiceServerBackend {
    Dds(Box<ros2_client::Server<BridgeServiceType>>),
    Zenoh(NodeServiceServer),
}

enum PendingServiceRequest {
    Dds(ros2_client::service::RmwRequestId),
    Zenoh(dora_ros2_bridge::transport::RequestId),
}

#[pymethods]
impl Ros2ServiceServer {
    /// Wait up to `timeout_s` (default 1s) for the next request.
    ///
    /// Returns `(request_id, request_array)`, or `None` if no request arrived
    /// within the timeout. Pass `request_id` back to `send_response`.
    ///
    /// :type timeout_s: float, optional
    /// :rtype: tuple[int, pyarrow.Array] | None
    #[pyo3(signature = (timeout_s=None))]
    pub fn take_request(
        &mut self,
        py: Python<'_>,
        timeout_s: Option<f64>,
    ) -> eyre::Result<Option<(u64, Py<PyAny>)>> {
        let timeout = timeout_or(timeout_s, 1.0);

        // Deserialize the request under the request TypeInfo (guard clears the
        // thread-local on drop). Mirrors the daemon's `run_service_server`.
        // Release the GIL while waiting so other Python threads can run.
        let request_type_info = self.request_type_info.clone();
        let received: Option<(PendingServiceRequest, ArrayData)> = match &self.server {
            ServiceServerBackend::Dds(server) => py.detach(|| {
                let _guard = TypeInfoGuard::deserialize(request_type_info);
                futures::executor::block_on(async {
                    let recv = server.async_receive_request();
                    futures::pin_mut!(recv);
                    let delay = futures_timer::Delay::new(timeout);
                    match futures::future::select(recv, delay).await {
                        futures::future::Either::Left((result, _)) => result
                            .map(|(id, message)| {
                                message.0.map(|data| (PendingServiceRequest::Dds(id), data))
                            })
                            .map_err(|e| eyre!("failed to receive service request: {e:?}")),
                        futures::future::Either::Right(_) => Ok(None),
                    }
                })
            })?,
            ServiceServerBackend::Zenoh(server) => py.detach(|| {
                futures::executor::block_on(async {
                    let recv = server.recv();
                    futures::pin_mut!(recv);
                    let delay = futures_timer::Delay::new(timeout);
                    match futures::future::select(recv, delay).await {
                        futures::future::Either::Left((result, _)) => {
                            let request = result?;
                            let data = deserialize_raw_cdr(
                                &request.payload,
                                request_type_info,
                                64 * 1024 * 1024,
                            )?;
                            Ok::<_, eyre::Report>(Some((
                                PendingServiceRequest::Zenoh(request.id),
                                data,
                            )))
                        }
                        futures::future::Either::Right(_) => Ok(None),
                    }
                })
            })?,
        };

        let Some((request_id, data)) = received else {
            return Ok(None);
        };

        // Bound `pending`: drop entries whose response was never sent (timed
        // out), then enforce a hard cap by evicting the oldest. Without this a
        // handler that skips `send_response` would leak entries forever.
        let now = Instant::now();
        self.pending
            .retain(|_, (_, taken)| now.duration_since(*taken) < SERVICE_RESPONSE_TIMEOUT);
        while self.pending.len() >= MAX_PENDING_REQUESTS {
            if let Some(oldest) = self
                .pending
                .iter()
                .min_by_key(|(_, (_, taken))| *taken)
                .map(|(id, _)| *id)
            {
                self.pending.remove(&oldest);
            } else {
                break;
            }
        }

        let id = self.next_id;
        self.next_id = self.next_id.wrapping_add(1);
        self.pending.insert(id, (request_id, now));
        Ok(Some((id, data.to_pyarrow(py)?.unbind())))
    }

    /// Send the response for a request previously returned by `take_request`.
    ///
    /// :type request_id: int
    /// :type response: pyarrow.Array
    /// :rtype: None
    pub fn send_response(
        &mut self,
        request_id: u64,
        response: Bound<'_, PyAny>,
    ) -> eyre::Result<()> {
        let py = response.py();
        let (transport_id, _taken) = self.pending.remove(&request_id).with_context(|| {
            format!("unknown request_id {request_id} (already answered or never taken)")
        })?;
        let array_data = pyarrow_to_array_data(&response)?;

        // Release the GIL while sending; bound the send with a timeout so a
        // congested transport can't freeze the Python thread indefinitely.
        let response_type_info = self.response_type_info.clone();
        match (&self.server, transport_id) {
            (ServiceServerBackend::Dds(server), PendingServiceRequest::Dds(rmw_id)) => {
                py.detach(|| {
                    let _guard = TypeInfoGuard::serialize(response_type_info);
                    futures::executor::block_on(async {
                        let send =
                            server.async_send_response(rmw_id, BridgeMessage(Some(array_data)));
                        futures::pin_mut!(send);
                        let delay = futures_timer::Delay::new(SERVICE_RESPONSE_TIMEOUT);
                        match futures::future::select(send, delay).await {
                            futures::future::Either::Left((result, _)) => {
                                result.map_err(|e| eyre!("failed to send service response: {e:?}"))
                            }
                            futures::future::Either::Right(_) => {
                                eyre::bail!("service response send timed out")
                            }
                        }
                    })
                })?
            }
            (ServiceServerBackend::Zenoh(server), PendingServiceRequest::Zenoh(id)) => {
                let response_value = make_array(array_data);
                let payload = serialize_raw_cdr(&TypedValue {
                    value: &response_value,
                    type_info: &response_type_info,
                })?;
                py.detach(|| futures::executor::block_on(server.reply(id, &payload)))?;
            }
            _ => eyre::bail!("service request belongs to a different ROS2 transport"),
        }
        Ok(())
    }
}

/// Convert a pyarrow value (dict, StructScalar, or Array) into Arrow `ArrayData`.
/// Shared by [`Ros2Publisher::publish`] and [`Ros2ServiceClient::call`].
fn pyarrow_to_array_data(data: &Bound<'_, PyAny>) -> eyre::Result<ArrayData> {
    let pyarrow = PyModule::import(data.py(), "pyarrow")?;

    let data = if data.is_instance_of::<PyDict>() {
        // convert to arrow struct scalar
        pyarrow.getattr("scalar")?.call1((data,))?
    } else {
        data.clone()
    };

    let data = if data.is_instance(&pyarrow.getattr("StructScalar")?)? {
        // convert to arrow array
        let list = PyList::new(data.py(), [data]).context("Failed to create Py::List")?;
        pyarrow.getattr("array")?.call1((list,))?
    } else {
        data
    };

    Ok(ArrayData::from_pyarrow_bound(&data)?)
}

pub fn create_dora_ros2_bridge_module(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<Ros2Transport>()?;
    m.add_class::<Ros2Context>()?;
    m.add_class::<Ros2Node>()?;
    m.add_class::<Ros2NodeOptions>()?;
    m.add_class::<Ros2Topic>()?;
    m.add_class::<Ros2Publisher>()?;
    m.add_class::<Ros2Subscription>()?;
    m.add_class::<Ros2ServiceClient>()?;
    m.add_class::<Ros2ServiceServer>()?;
    m.add_class::<Ros2ActionClient>()?;
    m.add_class::<Ros2ActionServer>()?;
    m.add_class::<qos::Ros2QosPolicies>()?;
    m.add_class::<qos::Ros2Durability>()?;
    m.add_class::<qos::Ros2Liveliness>()?;

    Ok(())
}

#[pyo3::pymodule]
fn dora_ros2_bridge_python(m: &Bound<'_, PyModule>) -> PyResult<()> {
    create_dora_ros2_bridge_module(m)
}

#[cfg(test)]
mod transport_tests {
    use super::{Ros2NodeOptions, Ros2Transport};

    #[test]
    fn transport_defaults_and_profiles_are_explicit() {
        assert_eq!(Ros2Transport::dds().kind(), "dds");
        assert_eq!(
            Ros2Transport::zenoh("humble", None).unwrap().kind(),
            "zenoh"
        );
        assert!(Ros2Transport::zenoh("automatic", None).is_err());
    }

    #[test]
    fn node_options_do_not_own_transport_configuration() {
        let options = Ros2NodeOptions::new(Some(true));
        assert!(options.rosout);
        assert_eq!(
            std::mem::size_of::<Ros2NodeOptions>(),
            std::mem::size_of::<bool>()
        );
    }
}
