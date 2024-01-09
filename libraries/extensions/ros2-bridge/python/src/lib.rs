use std::{
    collections::HashMap,
    path::{Path, PathBuf},
    sync::Arc,
};

use ::dora_ros2_bridge::{ros2_client, rustdds};
use arrow::{
    array::ArrayData,
    pyarrow::{FromPyArrow, ToPyArrow},
};
use dora_ros2_bridge_msg_gen::types::Message;
use eyre::{eyre, Context, ContextCompat};
use futures::{Stream, StreamExt};
use pyo3::{
    prelude::{pyclass, pymethods},
    types::{PyDict, PyList, PyModule},
    PyAny, PyObject, PyResult, Python,
};
use typed::{deserialize::TypedDeserializer, for_message, TypeInfo, TypedValue};

pub mod qos;
pub mod typed;

#[pyclass]
pub struct Ros2Context {
    context: ros2_client::Context,
    messages: Arc<HashMap<String, HashMap<String, Message>>>,
}

#[pymethods]
impl Ros2Context {
    #[new]
    pub fn new(ros_paths: Option<Vec<PathBuf>>) -> eyre::Result<Self> {
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
    pub fn new_node(
        &self,
        name: &str,
        namespace: &str,
        options: Ros2NodeOptions,
    ) -> eyre::Result<Ros2Node> {
        let name = ros2_client::NodeName::new(namespace, name)
            .map_err(|err| eyre!("invalid node name: {err}"))?;
        Ok(Ros2Node {
            node: self.context.new_node(name, options.into())?,
            messages: self.messages.clone(),
        })
    }
}

#[pyclass]
pub struct Ros2Node {
    node: ros2_client::Node,
    messages: Arc<HashMap<String, HashMap<String, Message>>>,
}

#[pymethods]
impl Ros2Node {
    pub fn create_topic(
        &self,
        name: &str,
        message_type: String,
        qos: qos::Ros2QosPolicies,
    ) -> eyre::Result<Ros2Topic> {
        let (namespace_name, message_name) = message_type.split_once("::").with_context(|| {
            format!(
                "message type must be of form `package::type`, is `{}`",
                message_type
            )
        })?;
        let message_type_name = ros2_client::MessageTypeName::new(namespace_name, message_name);
        let topic_name = ros2_client::Name::parse(name)
            .map_err(|err| eyre!("failed to parse ROS2 topic name: {err}"))?;
        let topic = self
            .node
            .create_topic(&topic_name, message_type_name, &qos.into())?;
        let type_info =
            for_message(&self.messages, namespace_name, message_name).with_context(|| {
                format!("failed to determine type info for message {namespace_name}/{message_name}")
            })?;

        Ok(Ros2Topic { topic, type_info })
    }

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
            deserializer: TypedDeserializer::new(topic.type_info.clone()),
        })
    }
}

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

#[pyclass]
#[non_exhaustive]
pub struct Ros2Topic {
    topic: rustdds::Topic,
    type_info: TypeInfo,
}

#[pyclass]
#[non_exhaustive]
pub struct Ros2Publisher {
    publisher: ros2_client::Publisher<TypedValue<'static>>,
    type_info: TypeInfo,
}

#[pymethods]
impl Ros2Publisher {
    pub fn publish(&self, data: &PyAny) -> eyre::Result<()> {
        let pyarrow = PyModule::import(data.py(), "pyarrow")?;

        let data = if data.is_instance_of::<PyDict>() {
            // convert to arrow struct scalar
            pyarrow.getattr("scalar")?.call1((data,))?
        } else {
            data
        };

        let data = if data.is_instance(pyarrow.getattr("StructScalar")?)? {
            // convert to arrow array
            let list = PyList::new(data.py(), [data]);
            pyarrow.getattr("array")?.call1((list,))?
        } else {
            data
        };

        let value = arrow::array::ArrayData::from_pyarrow(data)?;
        //// add type info to ensure correct serialization (e.g. struct types
        //// and map types need to be serialized differently)
        let typed_value = TypedValue {
            value: &value,
            type_info: &self.type_info,
        };

        self.publisher
            .publish(typed_value)
            .map_err(|e| e.forget_data())
            .context("publish failed")?;
        Ok(())
    }
}

#[pyclass]
#[non_exhaustive]
pub struct Ros2Subscription {
    deserializer: TypedDeserializer,
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
    deserializer: TypedDeserializer,
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

pub fn create_dora_ros2_bridge_module(m: &PyModule) -> PyResult<()> {
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
