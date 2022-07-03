//! Wrapper for rcl/service.h
//!
//! <https://docs.ros2.org/foxy/api/rcl/service_8h_source.html>
//!
//! - [x] `rcl_get_zero_initialized_service`
//! - [x] `rcl_service_init`
//! - [x] `rcl_service_fini`
//! - [x] `rcl_service_get_default_options`
//! - [ ] `rcl_take_request_with_info`
//! - [x] `rcl_take_request`
//! - [x] `rcl_send_response`
//! - [x] `rcl_service_get_service_name`
//! - [ ] `rcl_service_get_options`
//! - [ ] `rcl_service_get_rmw_handle`
//! - [x] `rcl_service_is_valid`

use std::{
    ffi::{c_void, CString},
    mem::MaybeUninit,
    sync::{Arc, Mutex},
};

use anyhow::{Context, Result};
use rclrust_msg::_core::{MessageT, ServiceRequestRaw, ServiceT};

use crate::{
    error::ToRclRustResult, internal::ffi::*, log::Logger, node::RclNode, qos::QoSProfile,
    rclrust_error,
};

#[derive(Debug)]
pub struct RclService {
    r#impl: Box<rcl_sys::rcl_service_t>,
    node: Arc<Mutex<RclNode>>,
}

unsafe impl Send for RclService {}
unsafe impl Sync for RclService {}

impl RclService {
    pub(crate) fn new<Srv>(
        node: Arc<Mutex<RclNode>>,
        service_name: &str,
        qos: &QoSProfile,
    ) -> Result<Self>
    where
        Srv: ServiceT,
    {
        let mut service = Box::new(unsafe { rcl_sys::rcl_get_zero_initialized_service() });
        let service_c_str = CString::new(service_name)?;
        let mut options = unsafe { rcl_sys::rcl_service_get_default_options() };
        options.qos = qos.into();

        unsafe {
            let value = rcl_sys::rcl_service_init(
                &mut *service,
                node.lock().unwrap().raw(),
                Srv::type_support() as *const _,
                service_c_str.as_ptr(),
                &options,
            );
            value.to_result()?;
        }

        Ok(Self {
            r#impl: service,
            node,
        })
    }

    #[inline]
    pub const fn raw(&self) -> &rcl_sys::rcl_service_t {
        &self.r#impl
    }

    pub fn take_request<Srv>(&self) -> Result<(rcl_sys::rmw_request_id_t, ServiceRequestRaw<Srv>)>
    where
        Srv: ServiceT,
    {
        let mut request_header = MaybeUninit::uninit();
        let mut request = Default::default();
        unsafe {
            rcl_sys::rcl_take_request(
                self.raw(),
                request_header.as_mut_ptr(),
                &mut request as *mut _ as *mut c_void,
            )
            .to_result()
            .with_context(|| "rcl_sys::rcl_take_request in RclService::take_request")?;
        }

        Ok((unsafe { request_header.assume_init() }, request))
    }

    pub fn send_response<Srv>(
        &self,
        response_header: &mut rcl_sys::rmw_request_id_t,
        response: Srv::Response,
    ) -> Result<()>
    where
        Srv: ServiceT,
    {
        unsafe {
            rcl_sys::rcl_send_response(
                self.raw(),
                response_header,
                &response.to_raw_ref() as *const _ as *mut c_void,
            )
            .to_result()
            .with_context(|| "rcl_sys::rcl_send_response in RclService::send_response")
        }
    }

    pub fn service_name(&self) -> Option<String> {
        unsafe {
            let name = rcl_sys::rcl_service_get_service_name(self.raw());
            String::from_c_char(name)
        }
    }

    pub fn is_valid(&self) -> bool {
        unsafe { rcl_sys::rcl_service_is_valid(self.raw()) }
    }
}

impl Drop for RclService {
    fn drop(&mut self) {
        if let Err(e) = unsafe {
            rcl_sys::rcl_service_fini(&mut *self.r#impl, self.node.lock().unwrap().raw_mut())
                .to_result()
        } {
            rclrust_error!(
                Logger::new("rclrust"),
                "Failed to clean up rcl service handle: {}",
                e
            )
        }
    }
}
