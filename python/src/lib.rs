use ::dora_ros2_bridge::{ros2_client, rustdds};
use eyre::Context;
use pyo3::{
    prelude::{pyclass, pymethods, pymodule},
    types::PyModule,
    PyAny, PyObject, PyResult, Python,
};

pub mod qos;

#[pyclass]
pub struct Ros2Context {
    context: ros2_client::Context,
}

#[pymethods]
impl Ros2Context {
    #[new]
    pub fn new() -> eyre::Result<Self> {
        Ok(Self {
            context: ros2_client::Context::new()?,
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
        })
    }
}

#[pyclass]
pub struct Ros2Node {
    node: ros2_client::Node,
}

#[pymethods]
impl Ros2Node {
    pub fn create_topic(
        &self,
        name: &str,
        type_name: String,
        qos: qos::Ros2QosPolicies,
    ) -> eyre::Result<Ros2Topic> {
        let topic = self.node.create_topic(name, type_name, &qos.into())?;
        Ok(Ros2Topic { topic })
    }

    pub fn create_publisher(
        &mut self,
        topic: &Ros2Topic,
        qos: Option<qos::Ros2QosPolicies>,
    ) -> eyre::Result<Ros2Publisher> {
        let publisher = self
            .node
            .create_publisher(&topic.topic, qos.map(Into::into))?;
        Ok(Ros2Publisher { publisher })
    }

    pub fn create_subscription(
        &mut self,
        topic: &Ros2Topic,
        qos: Option<qos::Ros2QosPolicies>,
    ) -> eyre::Result<Ros2Subscription> {
        let subscription = self
            .node
            .create_subscription(&topic.topic, qos.map(Into::into))?;
        Ok(Ros2Subscription { subscription })
    }
}

#[derive(Debug, Clone, Default)]
#[pyclass]
#[non_exhaustive]
pub struct Ros2NodeOptions {
    pub enable_rosout: bool,
}

#[pymethods]
impl Ros2NodeOptions {
    #[new]
    pub fn new() -> Self {
        Default::default()
    }
}

impl From<Ros2NodeOptions> for ros2_client::NodeOptions {
    fn from(value: Ros2NodeOptions) -> Self {
        ros2_client::NodeOptions::new().enable_rosout(value.enable_rosout)
    }
}

#[pyclass]
#[non_exhaustive]
pub struct Ros2Topic {
    topic: rustdds::Topic,
}

#[pyclass]
#[non_exhaustive]
pub struct Ros2Publisher {
    publisher: ros2_client::Publisher<serde_yaml::Value>,
}

#[pymethods]
impl Ros2Publisher {
    pub fn publish(&self, data: &PyAny) -> eyre::Result<()> {
        // TODO: add support for arrow types
        let data = pythonize::depythonize(data).context("failed to depythonize data")?;

        self.publisher.publish(data).context("publish failed")?;
        Ok(())
    }
}

#[pyclass]
#[non_exhaustive]
pub struct Ros2Subscription {
    subscription: ros2_client::Subscription<serde_yaml::Value>,
}

#[pymethods]
impl Ros2Subscription {
    pub fn next(&self, py: Python) -> eyre::Result<Option<PyObject>> {
        let message = self
            .subscription
            .take()
            .context("failed to take next message from subscription")?;
        let Some((value, _info)) = message else {
            return Ok(None)
        };

        let message = pythonize::pythonize(py, &value).context("failed to pythonize value")?;

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
    m.add_class::<qos::Ros2History>()?;
    m.add_class::<qos::Ros2Liveliness>()?;
    m.add_class::<qos::Ros2Reliability>()?;
    Ok(())
}
