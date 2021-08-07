use std::ffi::CString;
use std::marker::PhantomData;
use std::os::raw::c_void;
use std::sync::{Arc, Mutex};

use anyhow::Result;

use crate::error::ToRclRustResult;
use crate::internal::ffi::*;
use crate::log::Logger;
use crate::node::{Node, RclNode};
use crate::qos::QoSProfile;
use crate::rclrust_error;

#[derive(Debug)]
pub(crate) struct RclPublisher(rcl_sys::rcl_publisher_t);

unsafe impl Send for RclPublisher {}

impl RclPublisher {
    fn new<T>(node: &RclNode, topic_name: &str, qos: &QoSProfile) -> Result<Self>
    where
        T: rclrust_msg::_core::MessageT,
    {
        let mut publisher = unsafe { rcl_sys::rcl_get_zero_initialized_publisher() };
        let topic_c_str = CString::new(topic_name)?;
        let mut options = unsafe { rcl_sys::rcl_publisher_get_default_options() };
        options.qos = qos.into();

        unsafe {
            rcl_sys::rcl_publisher_init(
                &mut publisher,
                node.raw(),
                T::type_support() as *const _,
                topic_c_str.as_ptr(),
                &options,
            )
            .to_result()?;
        }

        Ok(Self(publisher))
    }

    unsafe fn fini(&mut self, node: &mut RclNode) -> Result<()> {
        rcl_sys::rcl_publisher_fini(&mut self.0, node.raw_mut()).to_result()
    }

    fn publish<T>(&self, message: &T) -> Result<()>
    where
        T: rclrust_msg::_core::MessageT,
    {
        unsafe {
            rcl_sys::rcl_publish(
                &self.0,
                &message.to_raw_ref() as *const _ as *const c_void,
                std::ptr::null_mut(),
            )
            .to_result()?;
        }

        Ok(())
    }

    fn topic_name(&self) -> Option<String> {
        unsafe {
            let name = rcl_sys::rcl_publisher_get_topic_name(&self.0);
            String::from_c_char(name)
        }
    }

    fn is_valid(&self) -> bool {
        unsafe { rcl_sys::rcl_publisher_is_valid(&self.0) }
    }

    fn subscription_count(&self) -> Result<usize> {
        let mut size = 0;
        unsafe {
            rcl_sys::rcl_publisher_get_subscription_count(&self.0, &mut size).to_result()?;
        }
        Ok(size)
    }
}

pub struct Publisher<T>
where
    T: rclrust_msg::_core::MessageT,
{
    handle: RclPublisher,
    node_handle: Arc<Mutex<RclNode>>,
    _phantom: PhantomData<T>,
}

impl<T> Publisher<T>
where
    T: rclrust_msg::_core::MessageT,
{
    pub(crate) fn new<'a>(node: &'a Node<'a>, topic_name: &str, qos: &QoSProfile) -> Result<Self> {
        let node_handle = node.clone_handle();
        let handle = RclPublisher::new::<T>(&node_handle.lock().unwrap(), topic_name, qos)?;

        Ok(Self {
            handle,
            node_handle,
            _phantom: Default::default(),
        })
    }

    pub fn publish(&self, message: &T) -> Result<()> {
        self.handle.publish(message)
    }

    pub fn topic_name(&self) -> Option<String> {
        self.handle.topic_name()
    }

    pub fn is_valid(&self) -> bool {
        self.handle.is_valid()
    }

    pub fn subscription_count(&self) -> Result<usize> {
        self.handle.subscription_count()
    }
}

impl<T> Drop for Publisher<T>
where
    T: rclrust_msg::_core::MessageT,
{
    fn drop(&mut self) {
        if let Err(e) = unsafe { self.handle.fini(&mut self.node_handle.lock().unwrap()) } {
            rclrust_error!(
                Logger::new("rclrust"),
                "Failed to clean up rcl publisher handle: {}",
                e
            )
        }
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
