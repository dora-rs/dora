use std::{
    ffi::CString,
    os::raw::c_void,
    sync::{Arc, Mutex},
};

use anyhow::{Context, Result};
use rclrust_msg::_core::MessageT;

use crate::{
    error::{RclRustError, ToRclRustResult},
    internal::ffi::*,
    log::Logger,
    node::{Node, RclNode},
    qos::QoSProfile,
    rclrust_error,
};

#[derive(Debug)]
pub(crate) struct RclSubscription(Box<rcl_sys::rcl_subscription_t>);

unsafe impl Send for RclSubscription {}

impl RclSubscription {
    fn new<T>(node: &RclNode, topic_name: &str, qos: &QoSProfile) -> Result<Self>
    where
        T: MessageT,
    {
        let mut subscription =
            Box::new(unsafe { rcl_sys::rcl_get_zero_initialized_subscription() });
        let topic_c_str = CString::new(topic_name)?;
        let mut options = unsafe { rcl_sys::rcl_subscription_get_default_options() };
        options.qos = qos.into();

        unsafe {
            rcl_sys::rcl_subscription_init(
                &mut *subscription,
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
        rcl_sys::rcl_subscription_fini(&mut *self.0, node.raw_mut())
            .to_result()
            .with_context(|| "rcl_sys::rcl_subscription_init in RclSubscription::fini")
    }

    fn take<T>(&self) -> Result<T::Raw>
    where
        T: MessageT,
    {
        let mut message = T::Raw::default();
        unsafe {
            rcl_sys::rcl_take(
                &*self.0,
                &mut message as *mut _ as *mut c_void,
                std::ptr::null_mut(),
                std::ptr::null_mut(),
            )
            .to_result()
            .with_context(|| "rcl_sys::rcl_take in RclSubscription::take")?;
        }

        Ok(message)
    }

    fn topic_name(&self) -> Option<String> {
        unsafe {
            let topic_name = rcl_sys::rcl_subscription_get_topic_name(&*self.0);
            String::from_c_char(topic_name)
        }
    }

    fn is_valid(&self) -> bool {
        unsafe { rcl_sys::rcl_subscription_is_valid(&*self.0) }
    }

    fn publisher_count(&self) -> Result<usize> {
        let mut size = 0;
        unsafe {
            rcl_sys::rcl_subscription_get_publisher_count(&*self.0, &mut size)
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

pub struct Subscription<T>
where
    T: MessageT,
{
    handle: RclSubscription,
    callback: Box<dyn Fn(&T::Raw)>,
    node_handle: Arc<Mutex<RclNode>>,
}

impl<T> Subscription<T>
where
    T: MessageT,
{
    pub(crate) fn new<'ctx, F>(
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
        self.handle.topic_name()
    }

    pub fn is_valid(&self) -> bool {
        self.handle.is_valid()
    }

    pub fn publisher_count(&self) -> Result<usize> {
        self.handle.publisher_count()
    }
}

impl<T> SubscriptionBase for Subscription<T>
where
    T: MessageT,
{
    fn handle(&self) -> &RclSubscription {
        &self.handle
    }

    fn call_callback(&self) -> Result<()> {
        match self.handle.take::<T>() {
            Ok(message) => (self.callback)(&message),
            Err(e) => match e.downcast_ref::<RclRustError>() {
                Some(RclRustError::RclSubscriptionTakeFailed(_)) => {}
                _ => return Err(e),
            },
        }
        Ok(())
    }
}

impl<T> Drop for Subscription<T>
where
    T: MessageT,
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
