use std::{
    collections::HashMap,
    ffi::CString,
    sync::{Arc, Mutex},
    thread::{self, JoinHandle},
    time::Duration,
};

use anyhow::{ensure, Context as _, Result};
use futures::channel::mpsc;
use rclrust_msg::{
    _core::{FFIToRust, MessageT, ServiceT},
    rcl_interfaces::msg::ParameterDescriptor,
};

use crate::{
    client::Client,
    clock::ClockType,
    context::{Context, RclContext},
    error::ToRclRustResult,
    executor::{Executor, ExecutorMessage},
    graph::{RclNamesAndTypes, RclStringArray},
    internal::ffi::*,
    log::Logger,
    node_options::NodeOptions,
    parameter::{Parameter, ParameterValue, Parameters},
    publisher::Publisher,
    qos::QoSProfile,
    rclrust_error,
    service::Service,
    subscription::Subscription,
    timer::Timer,
};

#[derive(Debug)]
pub(crate) struct RclNode {
    r#impl: Box<rcl_sys::rcl_node_t>,
    context: Arc<Mutex<RclContext>>,
}

unsafe impl Send for RclNode {}

impl RclNode {
    fn new(
        context: Arc<Mutex<RclContext>>,
        name: &str,
        namespace: Option<&str>,
        options: &NodeOptions,
    ) -> Result<Self> {
        let mut node = Box::new(unsafe { rcl_sys::rcl_get_zero_initialized_node() });
        let name_c_str = CString::new(name)?;
        let namespace_c_str = CString::new(namespace.unwrap_or_default())?;

        unsafe {
            rcl_sys::rcl_node_init(
                &mut *node,
                name_c_str.as_ptr(),
                namespace_c_str.as_ptr(),
                context.lock().unwrap().raw_mut(),
                options.raw(),
            )
            .to_result()
            .with_context(|| "rcl_sys::rcl_node_init in RclNode::new")?;
        }

        Ok(Self {
            r#impl: node,
            context,
        })
    }

    #[inline]
    pub(crate) const fn raw(&self) -> &rcl_sys::rcl_node_t {
        &self.r#impl
    }

    #[inline]
    pub(crate) fn raw_mut(&mut self) -> &mut rcl_sys::rcl_node_t {
        &mut self.r#impl
    }

    fn is_valid(&self) -> bool {
        unsafe { rcl_sys::rcl_node_is_valid(self.raw()) }
    }

    fn name(&self) -> String {
        unsafe {
            let name = rcl_sys::rcl_node_get_name(self.raw());
            String::from_c_char(name).unwrap()
        }
    }

    fn namespace(&self) -> String {
        unsafe {
            let namespace = rcl_sys::rcl_node_get_namespace(self.raw());
            String::from_c_char(namespace).unwrap()
        }
    }

    pub(crate) fn fully_qualified_name(&self) -> String {
        unsafe {
            let name = rcl_sys::rcl_node_get_fully_qualified_name(self.raw());
            String::from_c_char(name).unwrap()
        }
    }

    fn logger_name(&self) -> String {
        unsafe {
            let logger_name = rcl_sys::rcl_node_get_logger_name(self.raw());
            String::from_c_char(logger_name).unwrap()
        }
    }

    pub fn get_options(&self) -> Option<&rcl_sys::rcl_node_options_t> {
        unsafe { rcl_sys::rcl_node_get_options(self.raw()).as_ref() }
    }

    pub fn use_global_arguments(&self) -> Option<bool> {
        self.get_options().map(|opt| opt.use_global_arguments)
    }

    fn get_topic_names_and_types(&self, no_mangle: bool) -> Result<HashMap<String, Vec<String>>> {
        let mut names_and_types = RclNamesAndTypes::new();
        unsafe {
            rcl_sys::rcl_get_topic_names_and_types(
                self.raw(),
                &mut rcl_sys::rcutils_get_default_allocator(),
                no_mangle,
                names_and_types.raw_mut(),
            )
            .to_result()?;

            Ok(names_and_types.to_hash_map())
        }
    }

    fn get_service_names_and_types(&self) -> Result<HashMap<String, Vec<String>>> {
        let mut names_and_types = RclNamesAndTypes::new();
        unsafe {
            rcl_sys::rcl_get_service_names_and_types(
                self.raw(),
                &mut rcl_sys::rcutils_get_default_allocator(),
                names_and_types.raw_mut(),
            )
            .to_result()?;

            Ok(names_and_types.to_hash_map())
        }
    }

    fn get_node_names(&self) -> Result<Vec<String>> {
        Ok(self
            .get_node_names_and_namespace()?
            .into_iter()
            .map(|(name, _)| name)
            .collect())
    }

    fn get_node_names_and_namespace(&self) -> Result<Vec<(String, String)>> {
        let mut node_names = RclStringArray::new();
        let mut node_namespaces = RclStringArray::new();
        unsafe {
            rcl_sys::rcl_get_node_names(
                self.raw(),
                rcl_sys::rcutils_get_default_allocator(),
                node_names.raw_mut(),
                node_namespaces.raw_mut(),
            )
            .to_result()?;

            Ok(node_names.iter().zip(node_namespaces.iter()).collect())
        }
    }
}

impl Drop for RclNode {
    fn drop(&mut self) {
        let result = unsafe {
            let _guard = self.context.lock().unwrap();
            rcl_sys::rcl_node_fini(&mut *self.r#impl).to_result()
        };
        if let Err(e) = result {
            rclrust_error!(
                Logger::new("rclrust"),
                "Failed to clean up rcl node handle: {}",
                e
            )
        }
    }
}

#[derive(Debug)]
pub struct Node {
    pub(crate) handle: Arc<Mutex<RclNode>>,
    pub(crate) context: Arc<Mutex<RclContext>>,
    parameters: Parameters,
    wait_thread: Option<JoinHandle<Result<()>>>,
    tx: mpsc::Sender<ExecutorMessage>,
}

impl Node {
    pub(crate) fn new(
        context: &Context,
        name: &str,
        namespace: Option<&str>,
        options: &NodeOptions,
    ) -> Result<Self> {
        ensure!(context.is_valid(), "given Context is not valid");

        let context = Arc::clone(&context.handle);
        let handle = RclNode::new(Arc::clone(&context), name, namespace, options)?;
        let parameters = Parameters::new(Arc::clone(&context), &handle)?;

        let (tx, rx) = mpsc::channel(10);

        let wait_thread = {
            let context = Arc::clone(&context);

            thread::spawn(move || {
                let mut executor = Executor::new(context, rx);
                loop {
                    executor.spin()?
                }
            })
        };

        Ok(Self {
            handle: Arc::new(Mutex::new(handle)),
            context,
            parameters,
            wait_thread: Some(wait_thread),
            tx,
        })
    }

    pub(crate) fn clone_handle(&self) -> Arc<Mutex<RclNode>> {
        Arc::clone(&self.handle)
    }

    /// # Examples
    ///
    /// ```
    /// let ctx = rclrust::init().unwrap();
    /// let node = ctx.create_node("node1").unwrap();
    /// assert!(node.is_valid())
    /// ```
    pub fn is_valid(&self) -> bool {
        self.handle.lock().unwrap().is_valid()
    }

    /// # Examples
    ///
    /// ```
    /// let ctx = rclrust::init().unwrap();
    /// let node = ctx.create_node("node1").unwrap();
    /// assert_eq!(&node.name(), "node1");
    /// ```
    pub fn name(&self) -> String {
        self.handle.lock().unwrap().name()
    }

    /// # Examples
    ///
    /// ```
    /// let ctx = rclrust::init().unwrap();
    /// let node = ctx.create_node_with_ns("node1", "ns").unwrap();
    /// assert_eq!(&node.namespace(), "/ns");
    ///
    /// let node = ctx.create_node("node1").unwrap();
    /// assert_eq!(&node.namespace(), "/");
    /// ```
    pub fn namespace(&self) -> String {
        self.handle.lock().unwrap().namespace()
    }

    /// # Examples
    ///
    /// ```
    /// let ctx = rclrust::init().unwrap();
    /// let node = ctx.create_node_with_ns("node1", "ns").unwrap();
    /// assert_eq!(&node.fully_qualified_name(), "/ns/node1");
    /// ```
    pub fn fully_qualified_name(&self) -> String {
        self.handle.lock().unwrap().fully_qualified_name()
    }

    /// # Examples
    ///
    /// ```
    /// let ctx = rclrust::init().unwrap();
    /// let node = ctx.create_node_with_ns("node1", "ns").unwrap();
    /// let logger = node.logger();
    /// ```
    pub fn logger(&self) -> Logger {
        Logger::new(&self.logger_name())
    }

    /// # Examples
    ///
    /// ```
    /// let ctx = rclrust::init().unwrap();
    /// let node = ctx.create_node_with_ns("node1", "ns").unwrap();
    /// assert_eq!(&node.logger_name(), "ns.node1");
    /// ```
    pub fn logger_name(&self) -> String {
        self.handle.lock().unwrap().logger_name()
    }

    pub fn declare_parameter(&self, name: &str, default_value: &ParameterValue) -> Result<()> {
        self.declare_parameter_full(name, default_value, ParameterDescriptor::default(), false)
    }

    pub fn declare_parameter_full(
        &self,
        name: &str,
        default_value: &ParameterValue,
        parameter_descriptor: ParameterDescriptor,
        ignore_override: bool,
    ) -> Result<()> {
        self.parameters.declare_parameter(
            name,
            default_value,
            parameter_descriptor,
            ignore_override,
        )
    }

    pub fn has_parameter(&self, name: &str) -> bool {
        self.parameters.has_parameter(name)
    }

    pub fn get_parameter(&self, name: &str) -> Option<Parameter> {
        self.parameters.get_parameter(name)
    }

    pub fn set_parameter(&self, parameter: Parameter) -> Result<()> {
        self.parameters
            .set_parameters_atomically(&[parameter])?
            .to_result()
    }

    pub fn create_publisher<T>(&self, topic_name: &str, qos: &QoSProfile) -> Result<Publisher<T>>
    where
        T: MessageT,
    {
        Publisher::new(self, topic_name, qos)
    }

    pub fn create_subscription<T, F>(
        &mut self,
        topic_name: &str,
        callback: F,
        qos: &QoSProfile,
    ) -> Result<Subscription<T>>
    where
        T: MessageT + 'static,
        F: Fn(Arc<T>) + Send + 'static,
    {
        let subscription = Subscription::new(
            self,
            topic_name,
            move |msg| callback(Arc::new(unsafe { T::from_raw(&msg) })),
            qos,
        )?;
        self.tx
            .try_send(ExecutorMessage::Subscription(Box::new(
                subscription.create_invoker(),
            )))
            .expect("try_send should succeed");
        Ok(subscription)
    }

    pub fn create_raw_subscription<T, F>(
        &mut self,
        topic_name: &str,
        callback: F,
        qos: &QoSProfile,
    ) -> Result<Subscription<T>>
    where
        T: MessageT + 'static,
        F: Fn(Arc<T::Raw>) + Send + 'static,
    {
        let subscription = Subscription::new(self, topic_name, callback, qos)?;
        self.tx
            .try_send(ExecutorMessage::Subscription(Box::new(
                subscription.create_invoker(),
            )))
            .expect("try_send should succeed");
        Ok(subscription)
    }

    pub fn create_timer<F>(
        &mut self,
        period: Duration,
        clock_type: ClockType,
        callback: F,
    ) -> Result<Arc<Timer>>
    where
        F: Fn() + Send + 'static,
    {
        let timer = Timer::new(self, period, clock_type, callback)?;
        self.tx
            .try_send(ExecutorMessage::Timer(timer.create_invoker()))
            .expect("try_send should succeed");
        Ok(timer)
    }

    pub fn create_wall_timer<F>(&mut self, period: Duration, callback: F) -> Result<Arc<Timer>>
    where
        F: Fn() + Send + 'static,
    {
        self.create_timer(period, ClockType::SteadyTime, callback)
    }

    pub fn create_client<Srv>(
        &mut self,
        service_name: &str,
        qos: &QoSProfile,
    ) -> Result<Client<Srv>>
    where
        Srv: ServiceT + 'static,
    {
        let client = Client::<Srv>::new(self, service_name, qos)?;
        self.tx
            .try_send(ExecutorMessage::Client(Box::new(client.create_invoker())))
            .expect("try_send should succeed");
        Ok(client)
    }

    pub fn create_service<Srv, F>(
        &mut self,
        service_name: &str,
        callback: F,
        qos: &QoSProfile,
    ) -> Result<Service<Srv>>
    where
        Srv: ServiceT + 'static,
        F: Fn(Srv::Request) -> Srv::Response + Send + 'static,
    {
        let service = Service::<Srv>::new(
            self,
            service_name,
            move |req_raw| (callback)(unsafe { req_raw.to_rust() }),
            qos,
        )?;
        self.tx
            .try_send(ExecutorMessage::Service(Box::new(service.create_invoker())))
            .expect("try_send should succeed");
        Ok(service)
    }

    pub fn create_raw_service<Srv, F>(
        &mut self,
        service_name: &str,
        callback: F,
        qos: &QoSProfile,
    ) -> Result<Service<Srv>>
    where
        Srv: ServiceT + 'static,
        F: Fn(&<Srv::Request as MessageT>::Raw) -> Srv::Response + Send + 'static,
    {
        let service = Service::new(self, service_name, callback, qos)?;
        self.tx
            .try_send(ExecutorMessage::Service(Box::new(service.create_invoker())))
            .expect("try_send should succeed");
        Ok(service)
    }

    pub fn wait(&mut self) {
        if let Some(handle) = self.wait_thread.take() {
            if let Err(e) = handle.join() {
                rclrust_error!(Logger::new("rclrust"), "{:?}", e);
            }
        }
    }

    pub fn get_topic_names_and_types(
        &self,
        no_mangle: bool,
    ) -> Result<HashMap<String, Vec<String>>> {
        self.handle
            .lock()
            .unwrap()
            .get_topic_names_and_types(no_mangle)
    }

    pub fn get_service_names_and_types(&self) -> Result<HashMap<String, Vec<String>>> {
        self.handle.lock().unwrap().get_service_names_and_types()
    }

    pub fn get_node_names(&self) -> Result<Vec<String>> {
        self.handle.lock().unwrap().get_node_names()
    }

    pub fn get_node_names_and_namespace(&self) -> Result<Vec<(String, String)>> {
        self.handle.lock().unwrap().get_node_names_and_namespace()
    }
}

#[cfg(test)]
mod test {

    use super::*;

    #[test]
    fn node_init() -> Result<()> {
        let ctx = crate::init()?;
        let node = ctx.create_node("test_node")?;
        assert!(node.is_valid());

        Ok(())
    }

    #[test]
    fn node_name_without_namespace() -> Result<()> {
        let ctx = crate::init()?;
        let node = ctx.create_node("test_node")?;
        assert_eq!(node.name(), "test_node");
        assert_eq!(node.namespace(), "/");
        assert_eq!(node.fully_qualified_name(), "/test_node");
        assert_eq!(node.logger_name(), "test_node");

        Ok(())
    }

    #[test]
    fn node_name_with_namespace() -> Result<()> {
        let ctx = crate::init()?;
        let node = ctx.create_node_with_ns("test_node", "ns1")?;
        assert_eq!(node.name(), "test_node");
        assert_eq!(node.namespace(), "/ns1");
        assert_eq!(node.fully_qualified_name(), "/ns1/test_node");
        assert_eq!(node.logger_name(), "ns1.test_node");

        Ok(())
    }

    #[test]
    fn node_logger() -> Result<()> {
        let ctx = crate::init()?;
        let node = ctx.create_node_with_ns("test_node", "ns1")?;
        crate::rclrust_debug!(node.logger(), "logging {}{}{}", 21, "abc", 20.);

        Ok(())
    }

    #[test]
    fn node_declare_parameter() -> Result<()> {
        let ctx = crate::init()?;
        let node = ctx.create_node("test_node")?;
        node.declare_parameter("param", &ParameterValue::integer(42))?;
        assert!(node.has_parameter("param"));
        assert_eq!(
            node.get_parameter("param").unwrap(),
            Parameter {
                name: "param".into(),
                value: ParameterValue::integer(42)
            }
        );

        Ok(())
    }

    #[tokio::test]
    async fn get_topic_names_and_types() -> Result<()> {
        use rclrust_msg::std_msgs::msg::Int32;

        let ctx = crate::init()?;
        let node1 = ctx.create_node("test_node")?;
        let mut node2 = ctx.create_node_with_ns("another_node", "ns")?;

        let _pub = node2.create_publisher::<Int32>("pub", &QoSProfile::default())?;
        let _sub = node2.create_subscription::<Int32, _>("sub", |_| (), &QoSProfile::default())?;

        std::thread::sleep(std::time::Duration::from_millis(50));
        let hash_map = node1.get_topic_names_and_types(false)?;

        assert!(hash_map.contains_key("/ns/pub"));
        assert_eq!(hash_map["/ns/pub"], vec!["std_msgs/msg/Int32".to_string()]);
        assert!(hash_map.contains_key("/ns/sub"));
        assert_eq!(hash_map["/ns/sub"], vec!["std_msgs/msg/Int32".to_string()]);
        assert!(hash_map.contains_key("/rosout"));
        assert_eq!(
            hash_map["/rosout"],
            vec!["rcl_interfaces/msg/Log".to_string()]
        );

        Ok(())
    }

    #[tokio::test]
    async fn get_service_names_and_types() -> Result<()> {
        use rclrust_msg::std_srvs::srv::{Empty, Empty_Response, SetBool};

        let ctx = crate::init()?;
        let node1 = ctx.create_node("test_node")?;
        let mut node2 = ctx.create_node_with_ns("another_node", "ns")?;

        let _service = node2.create_service::<Empty, _>(
            "service",
            |_| Empty_Response::default(),
            &QoSProfile::default(),
        )?;
        let _client = node2.create_client::<SetBool>("client", &QoSProfile::default())?;

        std::thread::sleep(std::time::Duration::from_millis(50));
        let hash_map = node1.get_service_names_and_types()?;

        println!("{:?}", hash_map);

        assert!(hash_map.contains_key("/ns/service"));
        assert_eq!(
            hash_map["/ns/service"],
            vec!["std_srvs/srv/Empty".to_string()]
        );
        assert!(hash_map.contains_key("/ns/client"));
        assert_eq!(
            hash_map["/ns/client"],
            vec!["std_srvs/srv/SetBool".to_string()]
        );

        Ok(())
    }

    #[test]
    fn get_node_names() -> Result<()> {
        let ctx = crate::init()?;
        let node1 = ctx.create_node_with_ns("test_node", "ns")?;
        let _node2 = ctx.create_node("another_node")?;

        assert_eq!(
            node1.get_node_names()?,
            vec!["test_node".to_string(), "another_node".to_string()]
        );

        Ok(())
    }

    #[test]
    fn get_node_names_and_namespace() -> Result<()> {
        let ctx = crate::init()?;
        let node1 = ctx.create_node_with_ns("test_node", "ns")?;
        let _node2 = ctx.create_node("another_node")?;

        assert_eq!(
            node1.get_node_names_and_namespace()?,
            vec![
                ("test_node".to_string(), "/ns".to_string()),
                ("another_node".to_string(), "/".to_string())
            ]
        );

        Ok(())
    }
}
