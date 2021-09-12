//! Wrapper for rcl/client.h
//!
//! <https://docs.ros2.org/foxy/api/rcl/client_8h_source.html>
//!
//! - [x] `rcl_get_zero_initialized_client`
//! - [x] `rcl_client_init`
//! - [x] `rcl_client_fini`
//! - [x] `rcl_client_get_default_options`
//! - [x] `rcl_send_request`
//! - [ ] `rcl_take_response_with_info`
//! - [x] `rcl_take_response`
//! - [x] `rcl_client_get_service_name`
//! - [ ] `rcl_client_get_options`
//! - [ ] `rcl_client_get_rmw_handle`
//! - [x] `rcl_client_is_valid`

use std::{
    ffi::CString,
    mem::MaybeUninit,
    os::raw::c_void,
    sync::{Arc, Mutex},
};

use anyhow::{Context as _, Result};
use rclrust_msg::_core::{MessageT, ServiceResponseRaw, ServiceT};

use crate::{
    error::ToRclRustResult, internal::ffi::*, log::Logger, node::RclNode, qos::QoSProfile,
    rclrust_error,
};

#[derive(Debug)]
pub struct RclClient {
    r#impl: Box<rcl_sys::rcl_client_t>,
    node: Arc<Mutex<RclNode>>,
}

unsafe impl Send for RclClient {}
unsafe impl Sync for RclClient {}

impl RclClient {
    pub(crate) fn new<Srv>(
        node: Arc<Mutex<RclNode>>,
        service_name: &str,
        qos: &QoSProfile,
    ) -> Result<Self>
    where
        Srv: ServiceT,
    {
        let mut client = Box::new(unsafe { rcl_sys::rcl_get_zero_initialized_client() });
        let service_c_str = CString::new(service_name)?;
        let mut options = unsafe { rcl_sys::rcl_client_get_default_options() };
        options.qos = qos.into();

        unsafe {
            rcl_sys::rcl_client_init(
                &mut *client,
                node.lock().unwrap().raw(),
                Srv::type_support() as *const _,
                service_c_str.as_ptr(),
                &options,
            )
            .to_result()
            .with_context(|| "rcl_sys::rcl_client_init in RclClient::new")?;
        }

        Ok(Self {
            r#impl: client,
            node,
        })
    }

    #[inline]
    pub const fn raw(&self) -> &rcl_sys::rcl_client_t {
        &self.r#impl
    }

    #[inline]
    pub unsafe fn raw_mut(&mut self) -> &mut rcl_sys::rcl_client_t {
        &mut self.r#impl
    }

    pub fn send_request<Srv>(&self, request: &Srv::Request) -> Result<i64>
    where
        Srv: ServiceT,
    {
        let mut sequence_number = 0;
        unsafe {
            rcl_sys::rcl_send_request(
                self.raw(),
                &request.to_raw_ref() as *const _ as *const c_void,
                &mut sequence_number,
            )
            .to_result()
            .with_context(|| "rcl_sys::rcl_send_request in RclClient::send_request")?;
        }
        Ok(sequence_number)
    }

    pub fn take_response<Srv>(&self) -> Result<(rcl_sys::rmw_request_id_t, ServiceResponseRaw<Srv>)>
    where
        Srv: ServiceT,
    {
        let mut request_header = MaybeUninit::uninit();
        let mut response = Default::default();
        unsafe {
            rcl_sys::rcl_take_response(
                self.raw(),
                request_header.as_mut_ptr(),
                &mut response as *mut _ as *mut c_void,
            )
            .to_result()
            .with_context(|| "rcl_sys::rcl_take_response in RclClient::take_response")?;
        }

        Ok((unsafe { request_header.assume_init() }, response))
    }

    pub fn service_name(&self) -> Option<String> {
        unsafe {
            let name = rcl_sys::rcl_client_get_service_name(self.raw());
            String::from_c_char(name)
        }
    }

    pub fn is_valid(&self) -> bool {
        unsafe { rcl_sys::rcl_client_is_valid(self.raw()) }
    }

    /// graph.h
    pub fn service_is_available(&self) -> Result<bool> {
        let mut is_available = false;
        unsafe {
            rcl_sys::rcl_service_server_is_available(
                self.node.lock().unwrap().raw(),
                self.raw(),
                &mut is_available,
            )
            .to_result()
            .with_context(|| {
                "rcl_sys::rcl_service_server_is_available in RclClient::service_is_available"
            })?;
        }
        Ok(is_available)
    }
}

impl Drop for RclClient {
    fn drop(&mut self) {
        if let Err(e) = unsafe {
            rcl_sys::rcl_client_fini(self.raw_mut(), self.node.lock().unwrap().raw_mut())
                .to_result()
        } {
            rclrust_error!(
                Logger::new("rclrust"),
                "Failed to clean up rcl client handle: {}",
                e
            )
        }
    }
}
