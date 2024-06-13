use std::{
    borrow::Cow,
    collections::HashMap,
    path::{Path, PathBuf},
    sync::Arc,
};

use ::dora_ros2_bridge::{ros2_client, rustdds};
use arrow::{
    array::{make_array, ArrayData},
    pyarrow::{FromPyArrow, ToPyArrow},
};
use dora_ros2_bridge_msg_gen::types::Message;
use eyre::{eyre, Context, ContextCompat, Result};
use futures::{Stream, StreamExt};
use pyo3::{
    prelude::{pyclass, pymethods},
    types::{PyAnyMethods, PyDict, PyList, PyModule, PyModuleMethods},
    Bound, PyAny, PyObject, PyResult, Python,
};
use typed::{deserialize::StructDeserializer, TypeInfo, TypedValue};

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
    pub fn new(ros_paths: Option<Vec<PathBuf>>) -> eyre::Result<Self> {
        Python::with_gil(|py| -> Result<()> {
            let warnings = py
                .import_bound("warnings")
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
        for message in packages.into_iter().flat_map(|p| p.messages.into_iter()) {
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
        let (namespace_name, message_name) =
            match (message_type.split_once('/'), message_type.split_once("::")) {
                (Some(msg), None) => msg,
                (None, Some(msg)) => msg,
                _ => eyre::bail!("Expected message type in the format `namespace/message` or `namespace::message`, such as `std_msgs/UInt8` but got: {}", message_type),
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
}

/// ROS2 Node Options
/// :type rosout: bool, optional
///
#[derive(Debug, Clone, Default)]
#[pyclass]
#[non_exhaustive]
pub struct Ros2NodeOptions {
    pub rosout: bool,
}

#[pymethods]
impl Ros2NodeOptions {
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
        let pyarrow = PyModule::import_bound(data.py(), "pyarrow")?;

        let data = if data.is_instance_of::<PyDict>() {
            // convert to arrow struct scalar
            pyarrow.getattr("scalar")?.call1((data,))?
        } else {
            data
        };

        let data = if data.is_instance(&pyarrow.getattr("StructScalar")?)? {
            // convert to arrow array
            let list = PyList::new_bound(data.py(), [data]);
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

pub fn create_dora_ros2_bridge_module(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<Ros2Context>()?;
    m.add_class::<Ros2Node>()?;
    m.add_class::<Ros2NodeOptions>()?;
    m.add_class::<Ros2Topic>()?;
    m.add_class::<Ros2Publisher>()?;
    m.add_class::<Ros2Subscription>()?;
    m.add_class::<qos::Ros2QosPolicies>()?;
    m.add_class::<qos::Ros2Durability>()?;
    m.add_class::<qos::Ros2Liveliness>()?;

    Ok(())
}
