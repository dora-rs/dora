//! Wrapper for rcl/subscription.h
//!
//! <https://docs.ros2.org/foxy/api/rcl/subscription_8h_source.html>
//!
//! - [x] `rcl_get_zero_initialized_subscription`
//! - [x] `rcl_subscription_init`
//! - [x] `rcl_subscription_fini`
//! - [x] `rcl_subscription_get_default_options`
//! - [x] `rcl_take`
//! - [ ] `rcl_take_sequence`
//! - [ ] `rcl_take_serialized_message`
//! - [ ] `rcl_take_loaned_message`
//! - [ ] `rcl_return_loaned_message_from_subscription`
//! - [x] `rcl_subscription_get_topic_name`
//! - [ ] `rcl_subscription_get_options`
//! - [ ] `rcl_subscription_get_rmw_handle`
//! - [x] `rcl_subscription_is_valid`
//! - [x] `rcl_subscription_get_publisher_count`
//! - [ ] `rcl_subscription_get_actual_qos`
//! - [ ] `rcl_subscription_can_loan_messages`

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
pub struct RclSubscription {
    r#impl: Box<rcl_sys::rcl_subscription_t>,
    node: Arc<Mutex<RclNode>>,
}

unsafe impl Send for RclSubscription {}
unsafe impl Sync for RclSubscription {}

impl RclSubscription {
    pub(crate) fn new<T>(
        node: Arc<Mutex<RclNode>>,
        topic_name: &str,
        qos: &QoSProfile,
    ) -> Result<Self>
    where
        T: MessageT,
    {
        let mut subscription =
            Box::new(unsafe { rcl_sys::rcl_get_zero_initialized_subscription() });
        let topic_c_str = CString::new(topic_name)?;
        let mut options = unsafe { rcl_sys::rcl_subscription_get_default_options() };
        options.qos = qos.into();

        unsafe {
            let value = rcl_sys::rcl_subscription_init(
                &mut *subscription,
                node.lock().unwrap().raw(),
                T::type_support() as *const _,
                topic_c_str.as_ptr(),
                &options,
            );
            value
                .to_result()
                .with_context(|| "rcl_sys::rcl_subscription_init in RclSubscription::new")?;
        }

        Ok(Self {
            r#impl: subscription,
            node,
        })
    }

    #[inline]
    pub const fn raw(&self) -> &rcl_sys::rcl_subscription_t {
        &self.r#impl
    }

    pub fn take<T>(&self) -> Result<T::Raw>
    where
        T: MessageT,
    {
        let mut message = Default::default();
        unsafe {
            rcl_sys::rcl_take(
                self.raw(),
                &mut message as *mut _ as *mut c_void,
                std::ptr::null_mut(),
                std::ptr::null_mut(),
            )
            .to_result()
            .with_context(|| "rcl_sys::rcl_take in RclSubscription::take")?;
        }

        Ok(message)
    }

    pub fn topic_name(&self) -> Option<String> {
        unsafe {
            let topic_name = rcl_sys::rcl_subscription_get_topic_name(self.raw());
            String::from_c_char(topic_name)
        }
    }

    pub fn is_valid(&self) -> bool {
        unsafe { rcl_sys::rcl_subscription_is_valid(self.raw()) }
    }

    pub fn publisher_count(&self) -> Result<usize> {
        let mut size = 0;
        unsafe {
            rcl_sys::rcl_subscription_get_publisher_count(self.raw(), &mut size)
                .to_result()
                .with_context(|| {
                    "rcl_sys::rcl_subscription_get_publisher_count in RclSubscription::publisher_count"
                })?;
        }
        Ok(size)
    }

    pub fn actual_qos(&self) -> Option<QoSProfile> {
        unsafe {
            rcl_sys::rcl_subscription_get_actual_qos(self.raw())
                .as_ref()
                .map(|qos| qos.into())
        }
    }
}

impl Drop for RclSubscription {
    fn drop(&mut self) {
        if let Err(e) = unsafe {
            rcl_sys::rcl_subscription_fini(&mut *self.r#impl, self.node.lock().unwrap().raw_mut())
                .to_result()
        } {
            rclrust_error!(
                Logger::new("rclrust"),
                "Failed to clean up rcl node handle: {}",
                e
            )
        }
    }
}
