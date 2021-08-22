use std::{
    ffi::{c_void, CString},
    mem::MaybeUninit,
    sync::{Arc, Mutex},
};

use anyhow::{Context, Result};
use rclrust_msg::_core::{MessageT, ServiceT};

use crate::{
    error::{RclRustError, ToRclRustResult},
    internal::ffi::*,
    log::Logger,
    node::{Node, RclNode},
    qos::QoSProfile,
    rclrust_error,
};

pub struct RclService(Box<rcl_sys::rcl_service_t>);

unsafe impl Send for RclService {}

impl RclService {
    fn new<Srv>(node: &RclNode, service_name: &str, qos: &QoSProfile) -> Result<Self>
    where
        Srv: ServiceT,
    {
        let mut service = Box::new(unsafe { rcl_sys::rcl_get_zero_initialized_service() });
        let service_c_str = CString::new(service_name)?;
        let mut options = unsafe { rcl_sys::rcl_service_get_default_options() };
        options.qos = qos.into();

        unsafe {
            rcl_sys::rcl_service_init(
                &mut *service,
                node.raw(),
                Srv::type_support() as *const _,
                service_c_str.as_ptr(),
                &options,
            )
            .to_result()?;
        }

        Ok(Self(service))
    }

    pub const fn raw(&self) -> &rcl_sys::rcl_service_t {
        &self.0
    }

    unsafe fn fini(&mut self, node: &mut RclNode) -> Result<()> {
        rcl_sys::rcl_service_fini(&mut *self.0, node.raw_mut())
            .to_result()
            .with_context(|| "rcl_sys::rcl_service_fini in RclService::fini")
    }

    fn take_request<Srv>(
        &self,
    ) -> Result<(rcl_sys::rmw_request_id_t, <Srv::Request as MessageT>::Raw)>
    where
        Srv: ServiceT,
    {
        let mut request_header = MaybeUninit::uninit();
        let mut request = <Srv::Request as MessageT>::Raw::default();
        unsafe {
            rcl_sys::rcl_take_request(
                &*self.0,
                request_header.as_mut_ptr(),
                &mut request as *mut _ as *mut c_void,
            )
            .to_result()
            .with_context(|| "rcl_sys::rcl_take_request in RclService::take_request")?;
        }

        Ok((unsafe { request_header.assume_init() }, request))
    }

    fn send_response<Srv>(
        &self,
        response_header: &mut rcl_sys::rmw_request_id_t,
        response: Srv::Response,
    ) -> Result<()>
    where
        Srv: ServiceT,
    {
        unsafe {
            rcl_sys::rcl_send_response(
                &*self.0,
                response_header,
                &response.to_raw_ref() as *const _ as *mut c_void,
            )
            .to_result()
            .with_context(|| "rcl_sys::rcl_send_response in RclService::send_response")
        }
    }

    fn service_name(&self) -> String {
        unsafe {
            let name = rcl_sys::rcl_service_get_service_name(&*self.0);
            String::from_c_char(name).unwrap_or_default()
        }
    }

    fn is_valid(&self) -> bool {
        unsafe { rcl_sys::rcl_service_is_valid(&*self.0) }
    }
}

pub(crate) trait ServiceBase {
    fn handle(&self) -> &RclService;
    fn call_callback(&self) -> Result<()>;
}

pub struct Service<Srv>
where
    Srv: ServiceT,
{
    handle: RclService,
    callback: Box<dyn Fn(&<Srv::Request as MessageT>::Raw) -> Srv::Response>,
    node_handle: Arc<Mutex<RclNode>>,
}

impl<Srv> Service<Srv>
where
    Srv: ServiceT,
{
    pub(crate) fn new<'ctx, F>(
        node: &Node<'ctx>,
        service_name: &str,
        callback: F,
        qos: &QoSProfile,
    ) -> Result<Arc<Self>>
    where
        F: Fn(&<Srv::Request as MessageT>::Raw) -> Srv::Response + 'static,
    {
        let node_handle = node.clone_handle();
        let handle = RclService::new::<Srv>(&node_handle.lock().unwrap(), service_name, qos)?;

        Ok(Arc::new(Self {
            handle,
            callback: Box::new(callback),
            node_handle,
        }))
    }

    pub fn service_name(&self) -> String {
        self.handle.service_name()
    }

    pub fn is_valid(&self) -> bool {
        self.handle.is_valid()
    }
}

impl<Srv> ServiceBase for Service<Srv>
where
    Srv: ServiceT,
{
    fn handle(&self) -> &RclService {
        &self.handle
    }

    fn call_callback(&self) -> Result<()> {
        match self.handle.take_request::<Srv>() {
            Ok((mut req_header, req)) => {
                let res = (self.callback)(&req);
                self.handle.send_response::<Srv>(&mut req_header, res)
            }
            Err(e) => match e.downcast_ref::<RclRustError>() {
                Some(RclRustError::RclSubscriptionTakeFailed(_)) => Ok(()),
                _ => Err(e),
            },
        }
    }
}

impl<Srv> Drop for Service<Srv>
where
    Srv: ServiceT,
{
    fn drop(&mut self) {
        if let Err(e) = unsafe { self.handle.fini(&mut self.node_handle.lock().unwrap()) } {
            rclrust_error!(
                Logger::new("rclrust"),
                "Failed to clean up rcl service handle: {}",
                e
            )
        }
    }
}
