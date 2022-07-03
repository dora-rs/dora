//! Wrapper for rcl/publsiher.h
//!
//! <https://docs.ros2.org/foxy/api/rcl/publisher_8h_source.html>
//!
//! - [x] `rcl_get_zero_initialized_publisher`
//! - [x] `rcl_publisher_init`
//! - [x] `rcl_publisher_fini`
//! - [x] `rcl_publisher_get_default_options`
//! - [ ] `rcl_borrow_loaned_message`
//! - [ ] `rcl_return_loaned_message_from_publisher`
//! - [x] `rcl_publish`
//! - [ ] `rcl_publish_serialized_message`
//! - [ ] `rcl_publish_loaned_message`
//! - [ ] `rcl_publisher_assert_liveliness`
//! - [x] `rcl_publisher_get_topic_name`
//! - [ ] `rcl_publisher_get_options`
//! - [ ] `rcl_publisher_get_rmw_handle`
//! - [ ] `rcl_publisher_get_context`
//! - [x] `rcl_publisher_is_valid`
//! - [ ] `rcl_publisher_is_valid_except_context`
//! - [x] `rcl_publisher_get_subscription_count`
//! - [x] `rcl_publisher_get_actual_qos`
//! - [ ] `rcl_publisher_can_loan_messages`

use std::{
    ffi::CString,
    os::raw::c_void,
    sync::{Arc, Mutex},
};

use anyhow::{Context, Result};
use rclrust_msg::_core::MessageT;

use crate::{
    error::ToRclRustResult, internal::ffi::*, log::Logger, node::RclNode, qos::QoSProfile,
    rclrust_error,
};

#[derive(Debug)]
pub(super) struct RclPublisher {
    r#impl: Box<rcl_sys::rcl_publisher_t>,
    node: Arc<Mutex<RclNode>>,
}

unsafe impl Send for RclPublisher {}
unsafe impl Sync for RclPublisher {}

impl RclPublisher {
    pub fn new<T>(node: Arc<Mutex<RclNode>>, topic_name: &str, qos: &QoSProfile) -> Result<Self>
    where
        T: MessageT,
    {
        let mut publisher = Box::new(unsafe { rcl_sys::rcl_get_zero_initialized_publisher() });
        let topic_c_str = CString::new(topic_name)?;
        let mut options = unsafe { rcl_sys::rcl_publisher_get_default_options() };
        options.qos = qos.into();

        unsafe {
            let value = rcl_sys::rcl_publisher_init(
                &mut *publisher,
                node.lock().unwrap().raw(),
                T::type_support() as *const _,
                topic_c_str.as_ptr(),
                &options,
            );
            value
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

    pub fn publish<T>(&self, message: &T) -> Result<()>
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

    pub fn topic_name(&self) -> Option<String> {
        unsafe {
            let name = rcl_sys::rcl_publisher_get_topic_name(self.raw());
            String::from_c_char(name)
        }
    }

    pub fn is_valid(&self) -> bool {
        unsafe { rcl_sys::rcl_publisher_is_valid(self.raw()) }
    }

    pub fn subscription_count(&self) -> Result<usize> {
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

    pub fn actual_qos(&self) -> Option<QoSProfile> {
        unsafe {
            rcl_sys::rcl_publisher_get_actual_qos(self.raw())
                .as_ref()
                .map(|qos| qos.into())
        }
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
