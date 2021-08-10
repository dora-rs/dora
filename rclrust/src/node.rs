use std::ffi::CString;
use std::sync::{Arc, Mutex, Weak};
use std::time::Duration;

use anyhow::{ensure, Context as _, Result};
use rclrust_msg::_core::{FFIToRust, MessageT, ServiceT};
use rclrust_msg::rcl_interfaces::msg::ParameterDescriptor;

use crate::client::{Client, ClientBase};
use crate::clock::ClockType;
use crate::context::{Context, RclContext};
use crate::error::ToRclRustResult;
use crate::internal::ffi::*;
use crate::log::Logger;
use crate::node_options::NodeOptions;
use crate::parameter::{Parameter, ParameterValue, Parameters};
use crate::publisher::Publisher;
use crate::qos::QoSProfile;
use crate::rclrust_error;
use crate::service::{Service, ServiceBase};
use crate::subscription::{Subscription, SubscriptionBase};
use crate::timer::Timer;
use crate::wait_set::RclWaitSet;

#[derive(Debug)]
pub(crate) struct RclNode(rcl_sys::rcl_node_t);

unsafe impl Send for RclNode {}

impl RclNode {
    fn new(
        context: &mut RclContext,
        name: &str,
        namespace: Option<&str>,
        options: &NodeOptions,
    ) -> Result<Self> {
        let mut node = unsafe { rcl_sys::rcl_get_zero_initialized_node() };
        let name_c_str = CString::new(name)?;
        let namespace_c_str = CString::new(namespace.unwrap_or_default())?;

        unsafe {
            rcl_sys::rcl_node_init(
                &mut node,
                name_c_str.as_ptr(),
                namespace_c_str.as_ptr(),
                context.raw_mut(),
                options.raw(),
            )
            .to_result()
            .with_context(|| "rcl_sys::rcl_node_init in RclNode::new")?;
        }

        Ok(Self(node))
    }

    pub(crate) const fn raw(&self) -> &rcl_sys::rcl_node_t {
        &self.0
    }

    pub(crate) unsafe fn raw_mut(&mut self) -> &mut rcl_sys::rcl_node_t {
        &mut self.0
    }

    fn is_valid(&self) -> bool {
        unsafe { rcl_sys::rcl_node_is_valid(&self.0) }
    }

    fn name(&self) -> String {
        unsafe {
            let name = rcl_sys::rcl_node_get_name(&self.0);
            String::from_c_char(name).unwrap()
        }
    }

    fn namespace(&self) -> String {
        unsafe {
            let namespace = rcl_sys::rcl_node_get_namespace(&self.0);
            String::from_c_char(namespace).unwrap()
        }
    }

    pub(crate) fn fully_qualified_name(&self) -> String {
        unsafe {
            let name = rcl_sys::rcl_node_get_fully_qualified_name(&self.0);
            String::from_c_char(name).unwrap()
        }
    }

    fn logger_name(&self) -> String {
        unsafe {
            let logger_name = rcl_sys::rcl_node_get_logger_name(&self.0);
            String::from_c_char(logger_name).unwrap()
        }
    }

    pub fn get_options(&self) -> Option<&rcl_sys::rcl_node_options_t> {
        unsafe { rcl_sys::rcl_node_get_options(&self.0).as_ref() }
    }

    pub fn use_global_arguments(&self) -> Option<bool> {
        self.get_options().map(|opt| opt.use_global_arguments)
    }

    unsafe fn fini(&mut self, _ctx: &RclContext) -> Result<()> {
        rcl_sys::rcl_node_fini(&mut self.0)
            .to_result()
            .with_context(|| "rcl_sys::rcl_node_fini in RclNode::fini")
    }
}

pub struct Node<'ctx> {
    pub(crate) handle: Arc<Mutex<RclNode>>,
    pub(crate) context: &'ctx Context,
    pub(crate) subscriptions: Mutex<Vec<Weak<dyn SubscriptionBase>>>,
    pub(crate) timers: Mutex<Vec<Weak<Timer>>>,
    pub(crate) clients: Mutex<Vec<Weak<dyn ClientBase>>>,
    pub(crate) services: Mutex<Vec<Weak<dyn ServiceBase>>>,
    parameters: Parameters,
}

impl<'ctx> Node<'ctx> {
    pub(crate) fn new(
        context: &'ctx Context,
        name: &str,
        namespace: Option<&str>,
        options: &NodeOptions,
    ) -> Result<Arc<Self>> {
        ensure!(context.is_valid(), "given Context is not valid");
        let mut context_handle = context.handle.lock().unwrap();

        let handle = { RclNode::new(&mut context_handle, name, namespace, options)? };
        let parameters = Parameters::new(&context_handle, &handle)?;

        Ok(Arc::new(Self {
            handle: Arc::new(Mutex::new(handle)),
            context,
            subscriptions: Default::default(),
            timers: Default::default(),
            clients: Default::default(),
            services: Default::default(),
            parameters,
        }))
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
        &self,
        topic_name: &str,
        callback: F,
        qos: &QoSProfile,
    ) -> Result<Arc<Subscription<T>>>
    where
        T: MessageT + 'static,
        F: Fn(T) + 'static,
    {
        let sub = Subscription::new(
            self,
            topic_name,
            move |msg| callback(unsafe { T::from_raw(msg) }),
            qos,
        )?;
        let weak_sub = Arc::downgrade(&sub) as Weak<dyn SubscriptionBase>;
        self.subscriptions.lock().unwrap().push(weak_sub);
        Ok(sub)
    }

    pub fn create_raw_subscription<T, F>(
        &self,
        topic_name: &str,
        callback: F,
        qos: &QoSProfile,
    ) -> Result<Arc<Subscription<T>>>
    where
        T: MessageT + 'static,
        F: Fn(&T::Raw) + 'static,
    {
        let sub = Subscription::new(self, topic_name, callback, qos)?;
        let weak_sub = Arc::downgrade(&sub) as Weak<dyn SubscriptionBase>;
        self.subscriptions.lock().unwrap().push(weak_sub);
        Ok(sub)
    }

    pub fn create_timer<F>(
        &self,
        period: Duration,
        clock_type: ClockType,
        callback: F,
    ) -> Result<Arc<Timer>>
    where
        F: Fn() + 'static,
    {
        let timer = Timer::new(self, period, clock_type, callback)?;
        let weak_timer = Arc::downgrade(&timer);
        self.timers.lock().unwrap().push(weak_timer);
        Ok(timer)
    }

    pub fn create_wall_timer<F>(&self, period: Duration, callback: F) -> Result<Arc<Timer>>
    where
        F: Fn() + 'static,
    {
        self.create_timer(period, ClockType::SteadyTime, callback)
    }

    pub fn create_client<Srv>(
        &self,
        service_name: &str,
        qos: &QoSProfile,
    ) -> Result<Arc<Client<Srv>>>
    where
        Srv: ServiceT + 'static,
    {
        let client = Client::<Srv>::new(self, service_name, qos)?;
        let weak = Arc::downgrade(&client) as Weak<dyn ClientBase>;
        self.clients.lock().unwrap().push(weak);
        Ok(client)
    }

    pub fn create_service<Srv, F>(
        &self,
        service_name: &str,
        callback: F,
        qos: &QoSProfile,
    ) -> Result<Arc<Service<Srv>>>
    where
        Srv: ServiceT + 'static,
        F: Fn(Srv::Request) -> Srv::Response + 'static,
    {
        let srv = Service::<Srv>::new(
            self,
            service_name,
            move |req_raw| (callback)(unsafe { req_raw.to_rust() }),
            qos,
        )?;
        let weak_srv = Arc::downgrade(&srv) as Weak<dyn ServiceBase>;
        self.services.lock().unwrap().push(weak_srv);
        Ok(srv)
    }

    pub fn create_raw_service<Srv, F>(
        &self,
        service_name: &str,
        callback: F,
        qos: &QoSProfile,
    ) -> Result<Arc<Service<Srv>>>
    where
        Srv: ServiceT + 'static,
        F: Fn(&<Srv::Request as MessageT>::Raw) -> Srv::Response + 'static,
    {
        let srv = Service::new(self, service_name, callback, qos)?;
        let weak_srv = Arc::downgrade(&srv) as Weak<dyn ServiceBase>;
        self.services.lock().unwrap().push(weak_srv);
        Ok(srv)
    }

    pub(crate) fn add_to_wait_set(&self, wait_set: &mut RclWaitSet) -> Result<()> {
        self.subscriptions
            .lock()
            .unwrap()
            .iter()
            .filter_map(|weak| weak.upgrade())
            .try_for_each(|subscription| wait_set.add_subscription(subscription.handle()))?;

        self.timers
            .lock()
            .unwrap()
            .iter()
            .filter_map(|weak| weak.upgrade())
            .try_for_each(|timer| wait_set.add_timer(&timer.handle().lock().unwrap()))?;

        self.clients
            .lock()
            .unwrap()
            .iter()
            .filter_map(|weak| weak.upgrade())
            .try_for_each(|client| wait_set.add_client(client.handle()))?;

        self.services
            .lock()
            .unwrap()
            .iter()
            .filter_map(|weak| weak.upgrade())
            .try_for_each(|service| wait_set.add_service(service.handle()))?;

        Ok(())
    }

    pub(crate) fn call_callbacks(&self) -> Result<()> {
        self.subscriptions
            .lock()
            .unwrap()
            .iter()
            .filter_map(|weak| weak.upgrade())
            .try_for_each(|subscription| subscription.call_callback())?;

        self.timers
            .lock()
            .unwrap()
            .iter()
            .filter_map(|weak| weak.upgrade())
            .try_for_each(|timer| timer.call_callback())?;

        self.clients
            .lock()
            .unwrap()
            .iter()
            .filter_map(|weak| weak.upgrade())
            .try_for_each(|client| client.process_requests())?;

        self.services
            .lock()
            .unwrap()
            .iter()
            .filter_map(|weak| weak.upgrade())
            .try_for_each(|service| service.call_callback())?;

        Ok(())
    }
}

impl Drop for Node<'_> {
    fn drop(&mut self) {
        if let Err(e) = unsafe {
            self.handle
                .lock()
                .unwrap()
                .fini(&self.context.handle.lock().unwrap())
        } {
            rclrust_error!(
                Logger::new("rclrust"),
                "Failed to clean up rcl node handle: {}",
                e
            )
        }
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
}
