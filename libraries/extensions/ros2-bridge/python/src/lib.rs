use std::{
    borrow::Cow,
    collections::HashMap,
    path::{Path, PathBuf},
    sync::Arc,
};

use arrow::{
    array::{ArrayData, make_array},
    pyarrow::{FromPyArrow, ToPyArrow},
};
use ::dora_ros2_bridge::{ros2_client, rustdds};
use dora_ros2_bridge_msg_gen::types::Message;
use eyre::{Context, ContextCompat, Result, eyre};
use futures::{Stream, StreamExt};
use pyo3::{
    Bound, Py, PyAny, PyResult, Python,
    prelude::{pyclass, pymethods},
    types::{PyAnyMethods, PyDict, PyList, PyModule, PyModuleMethods},
};
use std::time::Duration;
use typed::{
    BridgeMessage, BridgeServiceType, TypeInfo, TypeInfoGuard, TypedValue,
    deserialize::StructDeserializer,
};

pub mod qos;
pub mod typed;

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
    /// :rtype: dora.Ros2Node
    pub fn new_node(
        &self,
        name: &str,
        namespace: &str,
        options: Ros2NodeOptions,
    ) -> eyre::Result<Ros2Node> {
        let name = ros2_client::NodeName::new(namespace, name)
            .map_err(|err| eyre!("invalid node name: {err}"))?;
        let mut node = self
            .context
            .new_node(name, options.into())
            .map_err(|e| eyre::eyre!("failed to create ROS2 node: {e:?}"))?;

        // Start a spinner so service/discovery status events are processed.
        let spinner_pool = futures::executor::ThreadPool::new()
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
                detect_service_mapping(),
                &ros2_client::Name::new("/", service_name.trim_start_matches('/'))
                    .map_err(|e| eyre!("failed to parse service name: {e}"))?,
                &ros2_client::ServiceTypeName::new(&package, &type_name),
                service_qos.clone(),
                service_qos,
            )
            .map_err(|e| eyre!("failed to create service client: {e:?}"))?;

        // Mirror the daemon: wait for the server before returning so the first
        // `call()` does not race service discovery.
        let ready = async {
            for _ in 0..10 {
                let wait = client.wait_for_service(&self.node);
                futures::pin_mut!(wait);
                let timeout = futures_timer::Delay::new(Duration::from_secs(2));
                match futures::future::select(wait, timeout).await {
                    futures::future::Either::Left(((), _)) => return Ok(()),
                    futures::future::Either::Right(_) => {}
                }
            }
            eyre::bail!("service `{service_name}` not available after 10 retries")
        };
        futures::executor::block_on(ready)?;

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
                detect_service_mapping(),
                &ros2_client::Name::new("/", service_name.trim_start_matches('/'))
                    .map_err(|e| eyre!("failed to parse service name: {e}"))?,
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

        // NOTE: blocks holding the GIL, matching the rest of this binding
        // (e.g. `Ros2Subscription::next`). Releasing the GIL here is a future
        // refinement (pyo3 0.28 reworked the GIL-release API).
        let timeout = Duration::from_secs_f64(timeout_s.unwrap_or(30.0));
        let response = {
            let _guard = TypeInfoGuard::deserialize(self.response_type_info.clone());
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
            })?
        };

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
    // Maps the integer id handed to Python back to the ROS2 request id, so
    // `send_response` can reply to the correct (possibly out-of-order) request.
    pending: HashMap<u64, ros2_client::service::RmwRequestId>,
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
        let timeout = Duration::from_secs_f64(timeout_s.unwrap_or(1.0));

        // Deserialize the request under the request TypeInfo (guard clears the
        // thread-local on drop). Mirrors the daemon's `run_service_server`.
        let received = {
            let _guard = TypeInfoGuard::deserialize(self.request_type_info.clone());
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
            })?
        };

        let Some((rmw_id, message)) = received else {
            return Ok(None);
        };
        let Some(data) = message.0 else {
            return Ok(None);
        };

        let id = self.next_id;
        self.next_id = self.next_id.wrapping_add(1);
        self.pending.insert(id, rmw_id);
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
        let rmw_id = self.pending.remove(&request_id).with_context(|| {
            format!("unknown request_id {request_id} (already answered or never taken)")
        })?;
        let array_data = pyarrow_to_array_data(&response)?;

        let _guard = TypeInfoGuard::serialize(self.response_type_info.clone());
        futures::executor::block_on(
            self.server
                .async_send_response(rmw_id, BridgeMessage(Some(array_data))),
        )
        .map_err(|e| eyre!("failed to send service response: {e:?}"))?;
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

/// Pick the `ServiceMapping` matching the active ROS2 middleware, from
/// `RMW_IMPLEMENTATION` (primary) then `ROS_DISTRO` (fallback), defaulting to
/// `Enhanced`. Mirrors the standalone bridge daemon (see dora-rs/dora#449).
fn detect_service_mapping() -> ros2_client::ServiceMapping {
    use ros2_client::ServiceMapping::{Cyclone, Enhanced};
    match std::env::var("RMW_IMPLEMENTATION").ok().as_deref() {
        Some("rmw_cyclonedds_cpp") => return Cyclone,
        Some(_) => return Enhanced,
        None => {}
    }
    match std::env::var("ROS_DISTRO").ok().as_deref() {
        Some("galactic") => Cyclone,
        _ => Enhanced,
    }
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
    m.add_class::<qos::Ros2QosPolicies>()?;
    m.add_class::<qos::Ros2Durability>()?;
    m.add_class::<qos::Ros2Liveliness>()?;

    Ok(())
}
