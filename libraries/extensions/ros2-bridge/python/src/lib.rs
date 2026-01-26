use std::{
    borrow::Cow,
    collections::HashMap,
    path::{Path, PathBuf},
    sync::Arc,
    cell::RefCell,
};

use ::dora_ros2_bridge::{ros2_client, rustdds};
use arrow::{
    array::{ArrayData, make_array, ArrayRef},
    pyarrow::{FromPyArrow, ToPyArrow},
};
use dora_ros2_bridge_msg_gen::types::Message;
use eyre::{Context, ContextCompat, Result, eyre};
use futures::{Stream, StreamExt};
use pyo3::{
    Bound, PyAny, PyObject, PyResult, Python,
    prelude::{pyclass, pymethods},
    types::{PyAnyMethods, PyDict, PyList, PyModule, PyModuleMethods},
};
use pyo3_special_method_derive::{Dict, Dir, Repr, Str};
use typed::{TypeInfo, TypedValue, deserialize::StructDeserializer};

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
#[derive(Str, Repr, Dir, Dict)]
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
        Python::with_gil(|py| -> Result<()> {
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

        let mut messages = HashMap::new();
        for message in packages.into_iter().flat_map(|p| {
            p.services
                .into_iter()
                .flat_map(|s| [s.request, s.response])
                .chain(p.messages)
        }) {
            let entry: &mut HashMap<String, Message> =
                messages.entry(message.package.clone()).or_default();
            entry.insert(message.name.clone(), message);
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
        Ok(Ros2Node {
            node: self
                .context
                .new_node(name, options.into())
                .map_err(|e| eyre::eyre!("failed to create ROS2 node: {e:?}"))?,
            messages: self.messages.clone(),
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
#[derive(Str, Repr, Dir, Dict)]
pub struct Ros2Node {
    node: ros2_client::Node,
    messages: Arc<HashMap<String, HashMap<String, Message>>>,
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

    /// Create a ROS2 service client
    ///
    /// :type name: str
    /// :type service_type: str
    /// :type qos: dora.Ros2QosPolicies, optional
    /// :rtype: dora.Ros2Client
    #[pyo3(signature = (name, service_type, qos=None))]
    pub fn create_client(
        &mut self,
        name: &str,
        service_type: String,
        qos: Option<qos::Ros2QosPolicies>,
    ) -> eyre::Result<Ros2Client> {
        let (namespace_name, service_name) = match (
            service_type.split_once('/'),
            service_type.split_once("::"),
        ) {
            (Some(msg), None) => msg,
            (None, Some(msg)) => msg,
            _ => eyre::bail!(
                "Expected service type in the format `namespace/service` or `namespace::service`, such as `example_interfaces/AddTwoInts` but got: {}",
                service_type
            ),
        };

        let request_type_info = TypeInfo {
            package_name: namespace_name.to_owned().into(),
            message_name: format!("{}_Request", service_name).into(),
            messages: self.messages.clone(),
        };
        let response_type_info = TypeInfo {
            package_name: namespace_name.to_owned().into(),
            message_name: format!("{}_Response", service_name).into(),
            messages: self.messages.clone(),
        };

        let client = self
            .node
            .create_client::<TypedService>(
                ros2_client::ServiceMapping::Enhanced,
                &ros2_client::Name::parse(name).map_err(|e| eyre::eyre!(e))?,
                &ros2_client::ServiceTypeName::new(namespace_name, service_name),
                qos.clone().map(Into::into).unwrap_or(rustdds::QosPolicies::builder().build()),
                qos.map(Into::into).unwrap_or(rustdds::QosPolicies::builder().build()),
            )
            .map_err(|e| eyre::eyre!("failed to create updated service client: {e:?}"))?;

        Ok(Ros2Client {
            client,
            request_type_info,
            response_type_info,
        })
    }
}

/// ROS2 Node Options
/// :type rosout: bool, optional
///
#[derive(Clone, Default, Str, Repr, Dir, Dict)]
#[pyclass]
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
#[derive(Str, Repr, Dir, Dict)]
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
#[derive(Str, Repr, Dir, Dict)]
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
        let pyarrow = PyModule::import(data.py(), "pyarrow")?;

        let data = if data.is_instance_of::<PyDict>() {
            // convert to arrow struct scalar
            pyarrow.getattr("scalar")?.call1((data,))?
        } else {
            data
        };

        let data = if data.is_instance(&pyarrow.getattr("StructScalar")?)? {
            // convert to arrow array
            let list = PyList::new(data.py(), [data]).context("Failed to create Py::List")?;
            pyarrow.getattr("array")?.call1((list,))?
        } else {
            data
        };

        let value = arrow::array::ArrayData::from_pyarrow_bound(&data)?;
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
#[derive(Str, Repr, Dir, Dict)]
#[non_exhaustive]
pub struct Ros2Subscription {
    deserializer: StructDeserializer<'static>,
    subscription: Option<ros2_client::Subscription<ArrayData>>,
}

#[pymethods]
impl Ros2Subscription {
    pub fn next(&self, py: Python) -> eyre::Result<Option<PyObject>> {
        let message = self
            .subscription
            .as_ref()
            .context("subscription was already used")?
            .take_seed(self.deserializer.clone())
            .context("failed to take next message from subscription")?;
        let Some((value, _info)) = message else {
            return Ok(None);
        };

        let message = value.to_pyarrow(py)?;
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


thread_local! {
    static CURRENT_TYPE_INFO: RefCell<Option<TypeInfo<'static>>> = RefCell::new(None);
}

#[derive(Debug, Clone)]
pub struct OwnedTypedValue {
    pub value: ArrayRef,
    pub type_info: TypeInfo<'static>,
}

impl serde::Serialize for OwnedTypedValue {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        let tv = TypedValue {
            value: &self.value,
            type_info: &self.type_info,
        };
        tv.serialize(serializer)
    }
}

impl<'de> serde::Deserialize<'de> for OwnedTypedValue {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        let type_info = CURRENT_TYPE_INFO.with(|ti| ti.borrow().clone())
            .ok_or_else(|| serde::de::Error::custom("No TypeInfo set"))?;
        let seed = StructDeserializer::new(Cow::Owned(type_info.clone()));
        use serde::de::DeserializeSeed;
        let array_data = seed.deserialize(deserializer).map_err(serde::de::Error::custom)?;
        Ok(OwnedTypedValue {
            value: make_array(array_data),
            type_info,
        })
    }
}

impl ros2_client::Message for OwnedTypedValue {}

#[derive(Debug)]
struct TypedService;

impl ros2_client::Service for TypedService {
    type Request = OwnedTypedValue;
    type Response = OwnedTypedValue;

    fn request_type_name(&self) -> &str {
        "ignored"
    }

    fn response_type_name(&self) -> &str {
        "ignored"
    }
}

/// ROS2 Service Client
///
/// warnings:
/// - dora Ros2 bridge functionality is considered **unstable**. It may be changed
///   at any point without it being considered a breaking change.
#[pyclass]
#[derive(Str, Repr, Dir, Dict)]
#[non_exhaustive]
pub struct Ros2Client {
    client: ros2_client::Client<TypedService>,
    request_type_info: TypeInfo<'static>,
    response_type_info: TypeInfo<'static>,
}

#[pymethods]
impl Ros2Client {
    pub fn wait_for_service(&self, node: &Ros2Node) -> eyre::Result<()> {
        let service_ready = async {
            for _ in 0..10 {
                let ready = self.client.wait_for_service(&node.node);
                futures::pin_mut!(ready);
                let timeout = dora_ros2_bridge::futures_timer::Delay::new(std::time::Duration::from_secs(2));
                match futures::future::select(ready, timeout).await {
                    futures::future::Either::Left(((), _)) => return Ok(()),
                    futures::future::Either::Right(_) => {
                        // timeout, retry
                    }
                }
            }
            eyre::bail!("service not available");
        };
        futures::executor::block_on(service_ready)
    }

    pub fn call(&self, request: Bound<'_, PyAny>) -> eyre::Result<PyObject> {
        let py = request.py();
        let pyarrow = PyModule::import(py, "pyarrow")?;

        let data = if request.is_instance_of::<PyDict>() {
             pyarrow.getattr("scalar")?.call1((request,))?
        } else {
            request
        };

        let data = if data.is_instance(&pyarrow.getattr("StructScalar")?)? {
            let list = PyList::new(data.py(), [data]).context("Failed to create Py::List")?;
            pyarrow.getattr("array")?.call1((list,))?
        } else {
             data
        };
        
        let value = arrow::array::ArrayData::from_pyarrow_bound(&data)?;
        
        let req = OwnedTypedValue {
            value: make_array(value),
            type_info: self.request_type_info.clone(), 
        };
        
        let client = &self.client;
        let response_type_info = self.response_type_info.clone();
        
        let res = futures::executor::block_on(async {
             let req_id = client.async_send_request(req).await.map_err(|e| eyre::eyre!(e))?;
             
             CURRENT_TYPE_INFO.with(|ti| {
                 *ti.borrow_mut() = Some(response_type_info);
             });
             
             let result = client.async_receive_response(req_id).await.map_err(|e| eyre::eyre!(e));
             
             CURRENT_TYPE_INFO.with(|ti| {
                 *ti.borrow_mut() = None;
             });
             
             result
        });
        
        match res {
             Ok(response) => {
                 let value = response.value.to_data(); 
                 Ok(value.to_pyarrow(py)?)
             }
             Err(e) => Err(e),
        }
    }
}

pub fn create_dora_ros2_bridge_module(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<Ros2Context>()?;
    m.add_class::<Ros2Node>()?;
    m.add_class::<Ros2NodeOptions>()?;
    m.add_class::<Ros2Topic>()?;
    m.add_class::<Ros2Publisher>()?;
    m.add_class::<Ros2Subscription>()?;
    m.add_class::<Ros2Client>()?; // Added
    m.add_class::<qos::Ros2QosPolicies>()?;
    m.add_class::<qos::Ros2Durability>()?;
    m.add_class::<qos::Ros2Liveliness>()?;

    Ok(())
}

