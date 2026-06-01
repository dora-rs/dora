use std::{
    borrow::Cow,
    collections::HashMap,
    path::{Path, PathBuf},
    sync::Arc,
};

use ::dora_ros2_bridge::{ros2_client, rustdds};
use arrow::{
    array::{ArrayData, make_array},
    pyarrow::{FromPyArrow, ToPyArrow},
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
/// warning::
///     dora Ros2 bridge functionality is considered **unstable**. It may be changed
///     at any point without it being considered a breaking change.
///
/// ```python
/// context = Ros2Context()
/// ```
///
/// :type ros_paths: typing.List[str], optional
///
#[pyclass]
pub struct Ros2Context {
    context: ros2_client::Context,
    messages: Arc<HashMap<String, HashMap<String, Message>>>,
}

#[pymethods]
impl Ros2Context {
    /// Create a new context
    #[new]
    #[pyo3(signature = (ros_paths=None))]
    pub fn new(ros_paths: Option<Vec<PathBuf>>) -> eyre::Result<Self> {
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

        Ok(Self {
            context: ros2_client::Context::new()?,
            messages: Arc::new(messages),
        })
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
        let mut node = self
            .context
            .new_node(name, node_options)
            .map_err(|e| eyre::eyre!("failed to create ROS2 node: {e:?}"))?;

        // Start a spinner so service/discovery status events are processed.
        // One worker thread is enough (the spinner is a single task); this keeps
        // the per-node thread cost at 1 instead of one-per-CPU-core.
        let spinner_pool = futures::executor::ThreadPool::builder()
            .pool_size(1)
            .create()
            .map_err(|e| eyre!("failed to create ros2 spinner pool: {e}"))?;
        let spinner = node
            .spinner()
            .map_err(|e| eyre!("failed to create ros2 spinner: {e:?}"))?;
        spinner_pool.spawn_ok(async move {
            if let Err(err) = spinner.spin().await {
                eprintln!("ros2 spinner stopped: {err:?}");
            }
        });

        Ok(Ros2Node {
            node,
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
    node: ros2_client::Node,
    messages: Arc<HashMap<String, HashMap<String, Message>>>,
    // Keeps the ROS2 spinner task alive for the node's lifetime. ros2-client
    // requires a running spinner for service/discovery status events
    // (`wait_for_service` panics otherwise).
    _spinner_pool: futures::executor::ThreadPool,
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

        let message_type_name = ros2_client::MessageTypeName::new(namespace_name, message_name);
        let topic_name = ros2_client::Name::parse(name)
            .map_err(|err| eyre!("failed to parse ROS2 topic name: {err}"))?;
        let topic = self
            .node
            .create_topic(&topic_name, message_type_name, &qos.into())?;
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
        let publisher = self
            .node
            .create_publisher(&topic.topic, qos.map(Into::into))?;
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
        let subscription = self
            .node
            .create_subscription(&topic.topic, qos.map(Into::into))?;
        Ok(Ros2Subscription {
            subscription: Some(subscription),
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

        let service_qos: rustdds::QosPolicies = qos.into();
        let client = self
            .node
            .create_client::<BridgeServiceType>(
                dora_ros2_bridge::detect_service_mapping(),
                &parse_ros2_name(service_name)?,
                &ros2_client::ServiceTypeName::new(&package, &type_name),
                service_qos.clone(),
                service_qos,
            )
            .map_err(|e| eyre!("failed to create service client: {e:?}"))?;

        // Mirror the daemon: wait for the server before returning so the first
        // `call()` does not race service discovery. Release the GIL: this blocks
        // for up to 20s and must not freeze other Python threads.
        let node = &self.node;
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

        let service_qos: rustdds::QosPolicies = qos.into();
        let server = self
            .node
            .create_server::<BridgeServiceType>(
                dora_ros2_bridge::detect_service_mapping(),
                &parse_ros2_name(service_name)?,
                &ros2_client::ServiceTypeName::new(&package, &type_name),
                service_qos.clone(),
                service_qos,
            )
            .map_err(|e| eyre!("failed to create service server: {e:?}"))?;

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
        let q: rustdds::QosPolicies = qos.into();
        let client = self
            .node
            .create_action_client::<BridgeActionType>(
                dora_ros2_bridge::detect_service_mapping(),
                &parse_ros2_name(action_name)?,
                &ros2_client::ActionTypeName::new(&package, &type_name),
                ros2_client::action::ActionClientQosPolicies {
                    goal_service: q.clone(),
                    result_service: q.clone(),
                    cancel_service: q.clone(),
                    feedback_subscription: q.clone(),
                    status_subscription: q,
                },
            )
            .map_err(|e| eyre!("failed to create action client: {e:?}"))?;
        Ok(Ros2ActionClient {
            client,
            goal_type_info,
            result_type_info,
            feedback_type_info,
            goals: HashMap::new(),
            in_flight: 0,
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
        let q: rustdds::QosPolicies = qos.into();
        let server = self
            .node
            .create_action_server::<BridgeActionType>(
                dora_ros2_bridge::detect_service_mapping(),
                &parse_ros2_name(action_name)?,
                &ros2_client::ActionTypeName::new(&package, &type_name),
                ros2_client::action::ActionServerQosPolicies {
                    goal_service: q.clone(),
                    result_service: q.clone(),
                    cancel_service: q.clone(),
                    feedback_publisher: q.clone(),
                    status_publisher: q,
                },
            )
            .map_err(|e| eyre!("failed to create action server: {e:?}"))?;
        let server = ros2_client::action::AsyncActionServer::new(server);
        Ok(Ros2ActionServer {
            server,
            goal_type_info,
            result_type_info,
            feedback_type_info,
            executing: HashMap::new(),
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
        self.node
            .set_parameter(name, value)
            .map_err(|e| eyre!("failed to set parameter `{name}`: {e}"))
    }

    /// Get a ROS2 parameter value, or `None` if it is not set.
    ///
    /// :type name: str
    /// :rtype: bool | int | float | str | bytes | list | None
    pub fn get_parameter(&self, py: Python<'_>, name: &str) -> eyre::Result<Py<PyAny>> {
        match self.node.get_parameter(name) {
            Some(value) => parameter_value_to_py(py, &value),
            None => Ok(py.None()),
        }
    }

    /// List the names of all declared ROS2 parameters.
    ///
    /// :rtype: list[str]
    pub fn list_parameters(&self) -> Vec<String> {
        self.node.list_parameters()
    }

    /// Whether a ROS2 parameter with the given name is declared.
    ///
    /// :type name: str
    /// :rtype: bool
    pub fn has_parameter(&self, name: &str) -> bool {
        self.node.has_parameter(name)
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
    topic: rustdds::Topic,
    type_info: TypeInfo<'static>,
}

/// ROS2 Publisher
///
/// warnings:
/// - dora Ros2 bridge functionality is considered **unstable**. It may be changed
///   at any point without it being considered a breaking change.
#[pyclass]
#[non_exhaustive]
pub struct Ros2Publisher {
    publisher: ros2_client::Publisher<TypedValue<'static>>,
    type_info: TypeInfo<'static>,
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

        self.publisher
            .publish(typed_value)
            .map_err(|e| e.forget_data())
            .context("publish failed")?;
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
    subscription: Option<ros2_client::Subscription<ArrayData>>,
}

#[pymethods]
impl Ros2Subscription {
    pub fn next(&self, py: Python) -> eyre::Result<Option<Py<PyAny>>> {
        let message = self
            .subscription
            .as_ref()
            .context("subscription was already used")?
            .take_seed(self.deserializer.clone())
            .context("failed to take next message from subscription")?;
        let Some((value, _info)) = message else {
            return Ok(None);
        };

        let message = value.to_pyarrow(py)?.unbind();
        // TODO: add `info`

        Ok(Some(message))
    }
}

impl Ros2Subscription {
    pub fn into_stream(&mut self) -> eyre::Result<Ros2SubscriptionStream> {
        let subscription = self
            .subscription
            .take()
            .context("subscription was already used")?;

        Ok(Ros2SubscriptionStream {
            deserializer: self.deserializer.clone(),
            subscription,
        })
    }
}

pub struct Ros2SubscriptionStream {
    deserializer: StructDeserializer<'static>,
    subscription: ros2_client::Subscription<ArrayData>,
}

impl Ros2SubscriptionStream {
    pub fn as_stream(
        &self,
    ) -> impl Stream<Item = Result<(ArrayData, ros2_client::MessageInfo), rustdds::dds::ReadError>> + '_
    {
        self.subscription
            .async_stream_seed(self.deserializer.clone())
    }
}

impl Stream for Ros2SubscriptionStream {
    type Item = Result<(ArrayData, ros2_client::MessageInfo), rustdds::dds::ReadError>;

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

#[pyclass]
#[non_exhaustive]
pub struct Ros2ActionClient {
    client: ros2_client::action::ActionClient<BridgeActionType>,
    goal_type_info: TypeInfo<'static>,
    result_type_info: TypeInfo<'static>,
    feedback_type_info: TypeInfo<'static>,
    goals: HashMap<String, ros2_client::action::GoalId>,
    in_flight: usize,
}

#[pyclass]
#[non_exhaustive]
pub struct Ros2ActionServer {
    server: ros2_client::action::AsyncActionServer<BridgeActionType>,
    goal_type_info: TypeInfo<'static>,
    result_type_info: TypeInfo<'static>,
    feedback_type_info: TypeInfo<'static>,
    executing: HashMap<
        String,
        (
            ros2_client::action::ExecutingGoalHandle<BridgeMessage>,
            Instant,
        ),
    >,
}
const MAX_CONCURRENT_GOALS: usize = 8;
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
        if self.in_flight >= MAX_CONCURRENT_GOALS {
            eyre::bail!("max concurrent goals ({MAX_CONCURRENT_GOALS}) reached");
        }
        let py = goal.py();
        let array_data = pyarrow_to_array_data(&goal)?;
        let timeout = timeout_or(timeout_s, ACTION_GOAL_TIMEOUT_S);
        let goal_type_info = self.goal_type_info.clone();
        let (goal_id, resp) = py.detach(|| {
            let _guard = TypeInfoGuard::serialize(goal_type_info);
            futures::executor::block_on(async {
                let send = self.client.async_send_goal(BridgeMessage(Some(array_data)));
                futures::pin_mut!(send);
                let delay = futures_timer::Delay::new(timeout);
                match futures::future::select(send, delay).await {
                    futures::future::Either::Left((r, _)) => {
                        r.map_err(|e| eyre!("failed to send action goal: {e:?}"))
                    }
                    futures::future::Either::Right(_) => {
                        eyre::bail!("action goal send timed out after {timeout:?}")
                    }
                }
            })
        })?;
        if !resp.accepted {
            return Ok(None);
        }
        let id = goal_id.uuid.to_string();
        self.goals.insert(id.clone(), goal_id);
        self.in_flight += 1;
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
            futures::executor::block_on(async {
                // feedback_stream is !Unpin (a FilterMap over async closures);
                // pin the STREAM so `.next()` (which requires Self: Unpin) works.
                let s = self.client.feedback_stream(gid);
                futures::pin_mut!(s);
                let delay = futures_timer::Delay::new(timeout);
                match futures::future::select(s.next(), delay).await {
                    futures::future::Either::Left((Some(r), _)) => r
                        .map(|fb| fb.0)
                        .map_err(|e| eyre!("feedback read error: {e:?}")),
                    futures::future::Either::Left((None, _)) => Ok(None),
                    futures::future::Either::Right(_) => Ok(None),
                }
            })
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
            futures::executor::block_on(async {
                let req = self.client.async_request_result(gid);
                futures::pin_mut!(req);
                let delay = futures_timer::Delay::new(timeout);
                match futures::future::select(req, delay).await {
                    futures::future::Either::Left((r, _)) => {
                        r.map(Some).map_err(|e| eyre!("result error: {e:?}"))
                    }
                    futures::future::Either::Right(_) => Ok(None),
                }
            })
        })?;
        match out {
            Some((status, msg)) => {
                self.goals.remove(goal_id);
                self.in_flight = self.in_flight.saturating_sub(1);
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
        let code = py.detach(|| {
            futures::executor::block_on(async {
                let fut = self.client.async_cancel_goal(gid, stamp);
                futures::pin_mut!(fut);
                let delay = futures_timer::Delay::new(timeout);
                match futures::future::select(fut, delay).await {
                    futures::future::Either::Left((r, _)) => r
                        .map(|resp| resp.return_code as i8)
                        .map_err(|e| eyre!("cancel error: {e:?}")),
                    futures::future::Either::Right(_) => eyre::bail!("cancel timed out"),
                }
            })
        })?;
        match goal_id {
            Some(s) => {
                if self.goals.remove(s).is_some() {
                    self.in_flight = self.in_flight.saturating_sub(1);
                }
            }
            None => {
                self.in_flight = self.in_flight.saturating_sub(self.goals.len());
                self.goals.clear();
            }
        }
        Ok(code)
    }
}

#[pymethods]
impl Ros2ActionServer {
    // SPIKE: server block_on borrow proof — receive_new_goal/accept_goal/
    // start_executing_goal all borrow &self.server inside one block_on on a
    // &mut self method. (Trimmed: abort + cap-eviction tail omitted for the spike.)
    #[pyo3(signature = (timeout_s=None))]
    pub fn take_goal(
        &mut self,
        py: Python<'_>,
        timeout_s: Option<f64>,
    ) -> eyre::Result<Option<(String, Py<PyAny>)>> {
        let timeout = timeout_or(timeout_s, 1.0);
        let goal_type_info = self.goal_type_info.clone();
        let taken = py.detach(|| {
            let _guard = TypeInfoGuard::deserialize(goal_type_info);
            futures::executor::block_on(async {
                let recv = self.server.receive_new_goal();
                futures::pin_mut!(recv);
                let delay = futures_timer::Delay::new(timeout);
                let handle = match futures::future::select(recv, delay).await {
                    futures::future::Either::Left((r, _)) => {
                        r.map_err(|e| eyre!("receive_new_goal: {e:?}"))?
                    }
                    futures::future::Either::Right(_) => return Ok(None),
                };
                // handles are Clone (not Copy: BridgeMessage isn't Copy) and the
                // FSM methods consume by value, so clone to read the payload
                // before accepting consumes the NewGoalHandle.
                let data = self.server.get_new_goal(handle.clone());
                let accepted = self
                    .server
                    .accept_goal(handle)
                    .await
                    .map_err(|e| eyre!("accept_goal: {e:?}"))?;
                let executing = self
                    .server
                    .start_executing_goal(accepted)
                    .await
                    .map_err(|e| eyre!("start_executing_goal: {e:?}"))?;
                Ok::<_, eyre::Report>(Some((executing, data)))
            })
        })?;
        let Some((executing, data)) = taken else {
            return Ok(None);
        };
        let goal_id = executing.goal_id().uuid.to_string();
        // No payload: drop the handle (don't track it). The goal dangles on the
        // wire; we can't send a meaningful terminal result without a payload.
        let Some(arr) = data.and_then(|m| m.0) else {
            return Ok(None);
        };
        // Leak guard: bound the local map by dropping the oldest tracked goal
        // when at cap. We do NOT send a wire abort — there is no valid empty
        // result to serialize, and waiting on a client that may never request
        // the result would stall the node. The dropped goal dangles on the wire
        // (best effort); single-goal usage never reaches the cap.
        let now = Instant::now();
        while self.executing.len() >= MAX_CONCURRENT_GOALS {
            let Some(oldest) = self
                .executing
                .iter()
                .min_by_key(|(_, (_, t))| *t)
                .map(|(k, _)| k.clone())
            else {
                break;
            };
            self.executing.remove(&oldest);
        }
        self.executing.insert(goal_id.clone(), (executing, now));
        Ok(Some((goal_id, arr.to_pyarrow(py)?.unbind())))
    }

    // SPIKE: &self block_on borrow — publish_feedback borrows &self.server.
    pub fn send_feedback(&self, goal_id: &str, feedback: Bound<'_, PyAny>) -> eyre::Result<()> {
        let py = feedback.py();
        // ExecutingGoalHandle<BridgeMessage> is Clone (not Copy) and
        // publish_feedback consumes it; clone to keep the goal in the map.
        let handle = self
            .executing
            .get(goal_id)
            .with_context(|| format!("unknown/finished goal {goal_id}"))?
            .0
            .clone();
        let array_data = pyarrow_to_array_data(&feedback)?;
        let feedback_type_info = self.feedback_type_info.clone();
        py.detach(|| {
            let _guard = TypeInfoGuard::serialize(feedback_type_info);
            futures::executor::block_on(
                self.server
                    .publish_feedback(handle, BridgeMessage(Some(array_data))),
            )
            .map_err(|e| eyre!("publish_feedback: {e:?}"))
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
        // Clone the handle but keep the goal tracked: send_result_response does
        // not resolve until the client requests the result, so on a timeout (or
        // serialize/send error) we must leave the goal in `executing` so the
        // caller can retry. We only retire it after the send succeeds.
        let handle = self
            .executing
            .get(goal_id)
            .with_context(|| format!("unknown/finished goal {goal_id}"))?
            .0
            .clone();
        let end = map_status(status);
        let array_data = pyarrow_to_array_data(&result)?;
        let timeout = timeout_or(timeout_s, ACTION_RESULT_TIMEOUT_S);
        let result_type_info = self.result_type_info.clone();
        py.detach(|| {
            let _guard = TypeInfoGuard::serialize(result_type_info);
            futures::executor::block_on(async {
                let send =
                    self.server
                        .send_result_response(handle, end, BridgeMessage(Some(array_data)));
                futures::pin_mut!(send);
                let delay = futures_timer::Delay::new(timeout);
                match futures::future::select(send, delay).await {
                    futures::future::Either::Left((r, _)) => {
                        r.map_err(|e| eyre!("send_result_response: {e:?}"))
                    }
                    futures::future::Either::Right(_) => {
                        eyre::bail!("send_result timed out after {timeout:?}")
                    }
                }
            })
        })?;
        // Success: retire the goal. On any error/timeout above, the `?` returns
        // early and the goal stays in `executing` for a retry.
        self.executing.remove(goal_id);
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
        py.detach(|| {
            futures::executor::block_on(async {
                let recv = self.server.receive_cancel_request();
                futures::pin_mut!(recv);
                let delay = futures_timer::Delay::new(timeout);
                let cancel_handle = match futures::future::select(recv, delay).await {
                    futures::future::Either::Left((r, _)) => {
                        r.map_err(|e| eyre!("receive_cancel_request: {e:?}"))?
                    }
                    futures::future::Either::Right(_) => return Ok(None),
                };
                let goals: Vec<_> = cancel_handle.goals().collect();
                self.server
                    .respond_to_cancel_requests(&cancel_handle, goals.iter().copied())
                    .await
                    .map_err(|e| eyre!("respond_to_cancel_requests: {e:?}"))?;
                let mine: Vec<String> = goals
                    .iter()
                    .map(|g| g.uuid.to_string())
                    .filter(|s| self.executing.contains_key(s))
                    .collect();
                Ok(Some(mine))
            })
        })
    }
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
    client: ros2_client::Client<BridgeServiceType>,
    request_type_info: TypeInfo<'static>,
    response_type_info: TypeInfo<'static>,
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
        let req_id = {
            let _guard = TypeInfoGuard::serialize(self.request_type_info.clone());
            self.client
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
                let recv = self.client.async_receive_response(req_id);
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
    server: ros2_client::Server<BridgeServiceType>,
    request_type_info: TypeInfo<'static>,
    response_type_info: TypeInfo<'static>,
    // Maps the integer id handed to Python back to the ROS2 request id (plus the
    // time it was taken), so `send_response` can reply to the correct (possibly
    // out-of-order) request. The insertion time bounds the map: stale entries
    // from requests that never got a response are evicted in `take_request`.
    pending: HashMap<u64, (ros2_client::service::RmwRequestId, Instant)>,
    next_id: u64,
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
        let received = py.detach(|| {
            let _guard = TypeInfoGuard::deserialize(request_type_info);
            futures::executor::block_on(async {
                let recv = self.server.async_receive_request();
                futures::pin_mut!(recv);
                let delay = futures_timer::Delay::new(timeout);
                match futures::future::select(recv, delay).await {
                    futures::future::Either::Left((result, _)) => result
                        .map(Some)
                        .map_err(|e| eyre!("failed to receive service request: {e:?}")),
                    futures::future::Either::Right(_) => Ok(None),
                }
            })
        })?;

        let Some((rmw_id, message)) = received else {
            return Ok(None);
        };
        let Some(data) = message.0 else {
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
        self.pending.insert(id, (rmw_id, now));
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
        let (rmw_id, _taken) = self.pending.remove(&request_id).with_context(|| {
            format!("unknown request_id {request_id} (already answered or never taken)")
        })?;
        let array_data = pyarrow_to_array_data(&response)?;

        // Release the GIL while sending; bound the send with a timeout so a
        // congested transport can't freeze the Python thread indefinitely.
        let response_type_info = self.response_type_info.clone();
        py.detach(|| {
            let _guard = TypeInfoGuard::serialize(response_type_info);
            futures::executor::block_on(async {
                let send = self
                    .server
                    .async_send_response(rmw_id, BridgeMessage(Some(array_data)));
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
        })?;
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
