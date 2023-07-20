use std::{
    collections::HashMap,
    ops::Deref,
    path::{Path, PathBuf},
    sync::Arc,
};

use ::dora_ros2_bridge::{ros2_client, rustdds};
use dora_ros2_bridge_msg_gen::types::Message;
use eyre::{eyre, Context, ContextCompat};
use pyo3::{
    prelude::{pyclass, pymethods, pymodule},
    types::PyModule,
    PyAny, PyObject, PyResult, Python,
};
use typed::{
    deserialize::{Ros2Value, TypedDeserializer},
    TypeInfo, TypedValue,
};

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
        Ok(Ros2Node {
            node: self.context.new_node(name, namespace, options.into())?,
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
        let encoded_type_name = format!("{namespace_name}::msg::dds_::{message_name}_");
        let topic = self
            .node
            .create_topic(name, encoded_type_name, &qos.into())?;
        let type_info = TypeInfo::for_message(&self.messages, namespace_name, message_name)
            .context("failed to determine type info for message")?;

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
            subscription,
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
    type_info: TypeInfo,
    publisher: ros2_client::Publisher<TypedValue<'static>>,
}

#[pymethods]
impl Ros2Publisher {
    pub fn publish(&self, data: &PyAny) -> eyre::Result<()> {
        // TODO: add support for arrow types
        let value = pythonize::depythonize(data).context("failed to depythonize data")?;

        // add type info to ensure correct serialization (e.g. struct types
        // and map types need to be serialized differently)
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
    subscription: ros2_client::Subscription<Ros2Value>,
}

#[pymethods]
impl Ros2Subscription {
    pub fn next(&self, py: Python) -> eyre::Result<Option<PyObject>> {
        let message = self
            .subscription
            .take_seed(self.deserializer.clone())
            .context("failed to take next message from subscription")?;
        let Some((value, _info)) = message else {
            return Ok(None)
        };

        let message =
            pythonize::pythonize(py, value.deref()).context("failed to pythonize value")?;

        // TODO: add `info`

        Ok(Some(message))
    }
}

#[pymodule]
fn dora_ros2_bridge(_py: Python, m: &PyModule) -> PyResult<()> {
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
