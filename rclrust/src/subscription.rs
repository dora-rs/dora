use std::ffi::CString;
use std::os::raw::c_void;
use std::sync::{Arc, Mutex};

use anyhow::{Context, Result};

use crate::error::ToRclRustResult;
use crate::internal::ffi::*;
use crate::log::Logger;
use crate::node::{Node, RclNode};
use crate::qos::QoSProfile;
use crate::rclrust_error;

#[derive(Debug)]
pub(crate) struct RclSubscription(rcl_sys::rcl_subscription_t);

unsafe impl Send for RclSubscription {}

impl RclSubscription {
    fn new<T>(node: &RclNode, topic_name: &str, qos: &QoSProfile) -> Result<Self>
    where
        T: rclrust_msg::_core::MessageT,
    {
        let mut subscription = unsafe { rcl_sys::rcl_get_zero_initialized_subscription() };
        let topic_c_str = CString::new(topic_name)?;
        let mut options = unsafe { rcl_sys::rcl_subscription_get_default_options() };
        options.qos = qos.into();

        unsafe {
            rcl_sys::rcl_subscription_init(
                &mut subscription,
                node.raw(),
                T::type_support() as *const _,
                topic_c_str.as_ptr(),
                &options,
            )
            .to_result()
            .with_context(|| "rcl_sys::rcl_subscription_init in RclSubscription::new")?;
        }

        Ok(Self(subscription))
    }

    pub const fn raw(&self) -> &rcl_sys::rcl_subscription_t {
        &self.0
    }

    unsafe fn fini(&mut self, node: &mut RclNode) -> Result<()> {
        rcl_sys::rcl_subscription_fini(&mut self.0, node.raw_mut())
            .to_result()
            .with_context(|| "rcl_sys::rcl_subscription_init in RclSubscription::fini")
    }

    fn take<T>(&self, message: &mut T::Raw) -> Result<()>
    where
        T: rclrust_msg::_core::MessageT,
    {
        unsafe {
            rcl_sys::rcl_take(
                &self.0,
                message as *const _ as *mut c_void,
                std::ptr::null_mut(),
                std::ptr::null_mut(),
            )
            .to_result()
            .with_context(|| "rcl_sys::rcl_take in RclSubscription::take")?;
        }

        Ok(())
    }

    fn topic_name(&self) -> Option<String> {
        unsafe {
            let topic_name = rcl_sys::rcl_subscription_get_topic_name(&self.0);
            String::from_c_char(topic_name)
        }
    }

    fn is_valid(&self) -> bool {
        unsafe { rcl_sys::rcl_subscription_is_valid(&self.0) }
    }

    fn publisher_count(&self) -> Result<usize> {
        let mut size = 0;
        unsafe {
            rcl_sys::rcl_subscription_get_publisher_count(&self.0, &mut size)
                .to_result()
                .with_context(|| {
                    "rcl_sys::rcl_subscription_get_publisher_count in RclSubscription::publisher_count"
                })?;
        }
        Ok(size)
    }
}

pub(crate) trait SubscriptionBase {
    fn handle(&self) -> &RclSubscription;
    fn call_callback(&self) -> Result<()>;
}

pub struct RawSubscription<T>
where
    T: rclrust_msg::_core::MessageT,
{
    handle: RclSubscription,
    callback: Box<dyn Fn(&T::Raw)>,
    node_handle: Arc<Mutex<RclNode>>,
}

impl<'ctx, T> RawSubscription<T>
where
    T: rclrust_msg::_core::MessageT,
{
    pub(crate) fn new<F>(
        node: &Node<'ctx>,
        topic_name: &str,
        callback: F,
        qos: &QoSProfile,
    ) -> Result<Arc<Self>>
    where
        F: Fn(&T::Raw) + 'static,
    {
        let node_handle = node.clone_handle();
        let handle = RclSubscription::new::<T>(&node_handle.lock().unwrap(), topic_name, qos)?;

        Ok(Arc::new(Self {
            handle,
            callback: Box::new(callback),
            node_handle,
        }))
    }

    pub fn topic_name(&self) -> Option<String> {
        self.handle().topic_name()
    }

    pub fn is_valid(&self) -> bool {
        self.handle().is_valid()
    }

    pub fn publisher_count(&self) -> Result<usize> {
        self.handle().publisher_count()
    }
}

impl<T> SubscriptionBase for RawSubscription<T>
where
    T: rclrust_msg::_core::MessageT,
{
    fn handle(&self) -> &RclSubscription {
        &self.handle
    }

    fn call_callback(&self) -> Result<()> {
        let mut message = T::Raw::default();
        self.handle.take::<T>(&mut message)?;
        (self.callback)(&message);

        Ok(())
    }
}

impl<T> Drop for RawSubscription<T>
where
    T: rclrust_msg::_core::MessageT,
{
    fn drop(&mut self) {
        if let Err(e) = unsafe { self.handle.fini(&mut self.node_handle.lock().unwrap()) } {
            rclrust_error!(
                Logger::new("rclrust"),
                "Failed to clean up rcl node handle: {}",
                e
            )
        }
    }
}

pub struct Subscription<T>
where
    T: rclrust_msg::_core::MessageT,
{
    handle: RclSubscription,
    callback: Box<dyn Fn(T)>,
    node_handle: Arc<Mutex<RclNode>>,
}

impl<'ctx, T> Subscription<T>
where
    T: rclrust_msg::_core::MessageT,
{
    pub(crate) fn new<F>(
        node: &Node<'ctx>,
        topic_name: &str,
        callback: F,
        qos: &QoSProfile,
    ) -> Result<Arc<Self>>
    where
        F: Fn(T) + 'static,
    {
        let node_handle = node.clone_handle();
        let handle = RclSubscription::new::<T>(&node_handle.lock().unwrap(), topic_name, qos)?;

        Ok(Arc::new(Self {
            handle,
            callback: Box::new(callback),
            node_handle,
        }))
    }

    pub fn topic_name(&self) -> Option<String> {
        self.handle().topic_name()
    }

    pub fn is_valid(&self) -> bool {
        self.handle().is_valid()
    }

    pub fn publisher_count(&self) -> Result<usize> {
        self.handle().publisher_count()
    }
}

impl<T> SubscriptionBase for Subscription<T>
where
    T: rclrust_msg::_core::MessageT,
{
    fn handle(&self) -> &RclSubscription {
        &self.handle
    }

    fn call_callback(&self) -> Result<()> {
        let mut message = T::Raw::default();
        self.handle.take::<T>(&mut message)?;
        (self.callback)(unsafe { T::from_raw(&message) });

        Ok(())
    }
}

impl<T> Drop for Subscription<T>
where
    T: rclrust_msg::_core::MessageT,
{
    fn drop(&mut self) {
        if let Err(e) = unsafe { self.handle.fini(&mut self.node_handle.lock().unwrap()) } {
            rclrust_error!(
                Logger::new("rclrust"),
                "Failed to clean up rcl node handle: {}",
                e
            )
        }
    }
}
