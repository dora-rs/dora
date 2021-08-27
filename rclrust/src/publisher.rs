use std::{
    ffi::CString,
    marker::PhantomData,
    os::raw::c_void,
    sync::{Arc, Mutex},
};

use anyhow::{Context, Result};
use rclrust_msg::_core::MessageT;

use crate::{
    error::ToRclRustResult,
    internal::ffi::*,
    log::Logger,
    node::{Node, RclNode},
    qos::QoSProfile,
    rclrust_error,
};

#[derive(Debug)]
pub(crate) struct RclPublisher {
    r#impl: Box<rcl_sys::rcl_publisher_t>,
    node: Arc<Mutex<RclNode>>,
}

unsafe impl Send for RclPublisher {}
unsafe impl Sync for RclPublisher {}

impl RclPublisher {
    fn new<T>(node: Arc<Mutex<RclNode>>, topic_name: &str, qos: &QoSProfile) -> Result<Self>
    where
        T: MessageT,
    {
        let mut publisher = Box::new(unsafe { rcl_sys::rcl_get_zero_initialized_publisher() });
        let topic_c_str = CString::new(topic_name)?;
        let mut options = unsafe { rcl_sys::rcl_publisher_get_default_options() };
        options.qos = qos.into();

        unsafe {
            rcl_sys::rcl_publisher_init(
                &mut *publisher,
                node.lock().unwrap().raw(),
                T::type_support() as *const _,
                topic_c_str.as_ptr(),
                &options,
            )
            .to_result()
            .with_context(|| "rcl_sys::rcl_publisher_init in RclPublisher::new")?;
        }

        Ok(Self {
            r#impl: publisher,
            node,
        })
    }

    #[inline]
    const fn raw(&self) -> &rcl_sys::rcl_publisher_t {
        &self.r#impl
    }

    fn publish<T>(&self, message: &T) -> Result<()>
    where
        T: MessageT,
    {
        unsafe {
            rcl_sys::rcl_publish(
                self.raw(),
                &message.to_raw_ref() as *const _ as *const c_void,
                std::ptr::null_mut(),
            )
            .to_result()
            .with_context(|| "rcl_sys::rcl_publish in RclPublisher::publish")?;
        }

        Ok(())
    }

    fn topic_name(&self) -> String {
        unsafe {
            let name = rcl_sys::rcl_publisher_get_topic_name(self.raw());
            String::from_c_char(name).unwrap()
        }
    }

    fn is_valid(&self) -> bool {
        unsafe { rcl_sys::rcl_publisher_is_valid(self.raw()) }
    }

    fn subscription_count(&self) -> Result<usize> {
        let mut size = 0;
        unsafe {
            rcl_sys::rcl_publisher_get_subscription_count(self.raw(), &mut size)
                .to_result()
                .with_context(|| {
                    "rcl_sys::rcl_publisher_get_subscription_count in RclPublisher::subscription_count"
                })?;
        }
        Ok(size)
    }
}

impl Drop for RclPublisher {
    fn drop(&mut self) {
        if let Err(e) = unsafe {
            rcl_sys::rcl_publisher_fini(&mut *self.r#impl, self.node.lock().unwrap().raw_mut())
                .to_result()
        } {
            rclrust_error!(
                Logger::new("rclrust"),
                "Failed to clean up rcl publisher handle: {}",
                e
            )
        }
    }
}

pub struct Publisher<T>
where
    T: MessageT,
{
    handle: RclPublisher,
    _phantom: PhantomData<T>,
}

impl<T> Publisher<T>
where
    T: MessageT,
{
    pub(crate) fn new(node: &Node, topic_name: &str, qos: &QoSProfile) -> Result<Self> {
        let handle = RclPublisher::new::<T>(node.clone_handle(), topic_name, qos)?;

        Ok(Self {
            handle,
            _phantom: Default::default(),
        })
    }

    pub fn publish(&self, message: &T) -> Result<()> {
        self.handle.publish(message)
    }

    pub fn topic_name(&self) -> String {
        self.handle.topic_name()
    }

    pub fn is_valid(&self) -> bool {
        self.handle.is_valid()
    }

    pub fn subscription_count(&self) -> Result<usize> {
        self.handle.subscription_count()
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn create_publisher() -> Result<()> {
        use rclrust_msg::std_msgs::msg::String;

        let ctx = crate::init()?;
        let node = ctx.create_node("test_node")?;
        let publisher = node.create_publisher::<String>("message", &QoSProfile::default())?;
        assert!(publisher.is_valid());

        Ok(())
    }
}
