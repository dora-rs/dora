use std::collections::HashMap;
use std::ffi::CString;
use std::marker::PhantomData;
use std::mem::MaybeUninit;
use std::os::raw::c_void;
use std::sync::{Arc, Mutex, Weak};
use std::time::Duration;

use anyhow::{anyhow, Context as _, Result};
use futures::channel::oneshot;
use rclrust_msg::_core::{FFIToRust, MessageT, ServiceT};

use crate::context::Context;
use crate::error::{RclRustError, ToRclRustResult};
use crate::internal::ffi::*;
use crate::log::Logger;
use crate::node::{Node, RclNode};
use crate::qos::QoSProfile;
use crate::rclrust_error;
use crate::wait_set::RclWaitSet;

#[derive(Debug)]
pub(crate) struct RclClient(rcl_sys::rcl_client_t);

unsafe impl Send for RclClient {}

impl RclClient {
    pub fn new<Srv>(node: &RclNode, service_name: &str, qos: &QoSProfile) -> Result<Self>
    where
        Srv: ServiceT,
    {
        let mut client = unsafe { rcl_sys::rcl_get_zero_initialized_client() };
        let service_c_str = CString::new(service_name)?;
        let mut options = unsafe { rcl_sys::rcl_client_get_default_options() };
        options.qos = qos.into();

        unsafe {
            rcl_sys::rcl_client_init(
                &mut client,
                node.raw(),
                Srv::type_support() as *const _,
                service_c_str.as_ptr(),
                &options,
            )
            .to_result()
            .with_context(|| "rcl_sys::rcl_client_init in RclClient::new")?;
        }

        Ok(Self(client))
    }

    pub const fn raw(&self) -> &rcl_sys::rcl_client_t {
        &self.0
    }

    unsafe fn fini(&mut self, node: &mut RclNode) -> Result<()> {
        rcl_sys::rcl_client_fini(&mut self.0, node.raw_mut())
            .to_result()
            .with_context(|| "rcl_sys::rcl_client_fini in RclClient::fini")
    }

    fn send_request<Srv>(&self, request: &Srv::Request) -> Result<i64>
    where
        Srv: ServiceT,
    {
        let mut sequence_number = 0;
        unsafe {
            rcl_sys::rcl_send_request(
                &self.0,
                &request.to_raw_ref() as *const _ as *const c_void,
                &mut sequence_number,
            )
            .to_result()
            .with_context(|| "rcl_sys::rcl_send_request in RclClient::send_request")?;
        }
        Ok(sequence_number)
    }

    fn take_response<Srv>(
        &self,
    ) -> Result<(rcl_sys::rmw_request_id_t, <Srv::Response as MessageT>::Raw)>
    where
        Srv: ServiceT,
    {
        let mut request_header = MaybeUninit::uninit();
        let mut response = <Srv::Response as MessageT>::Raw::default();
        unsafe {
            rcl_sys::rcl_take_response(
                &self.0,
                request_header.as_mut_ptr(),
                &mut response as *mut _ as *mut c_void,
            )
            .to_result()
            .with_context(|| "rcl_sys::rcl_take_response in RclClient::take_response")?;
        }

        Ok((unsafe { request_header.assume_init() }, response))
    }

    fn service_name(&self) -> String {
        unsafe {
            let name = rcl_sys::rcl_client_get_service_name(&self.0);
            String::from_c_char(name).unwrap_or_default()
        }
    }

    fn service_is_available(&self, node: &RclNode) -> Result<bool> {
        let mut is_available = false;
        unsafe {
            rcl_sys::rcl_service_server_is_available(node.raw(), &self.0, &mut is_available)
                .to_result()
                .with_context(|| {
                    "rcl_sys::rcl_service_server_is_available in RclClient::service_is_available"
                })?;
        }
        Ok(is_available)
    }

    fn is_valid(&self) -> bool {
        unsafe { rcl_sys::rcl_client_is_valid(&self.0) }
    }
}

pub(crate) trait ClientBase {
    fn handle(&self) -> &RclClient;
    fn process_requests(&self) -> Result<()>;
}

pub struct Client<Srv>
where
    Srv: ServiceT + 'static,
{
    handle: RclClient,
    node_handle: Arc<Mutex<RclNode>>,
    pendings: Mutex<HashMap<i64, oneshot::Sender<Srv::Response>>>,
    _phantom: PhantomData<Srv>,
}

impl<Srv> Client<Srv>
where
    Srv: ServiceT,
{
    pub(crate) fn new<'ctx>(
        node: &Node<'ctx>,
        service_name: &str,
        qos: &QoSProfile,
    ) -> Result<Arc<Self>> {
        let node_handle = node.clone_handle();
        let handle = RclClient::new::<Srv>(&node_handle.lock().unwrap(), service_name, qos)?;

        Ok(Arc::new(Self {
            handle,
            node_handle,
            pendings: Mutex::new(HashMap::new()),
            _phantom: Default::default(),
        }))
    }

    pub fn send_request(
        self: &Arc<Self>,
        request: &Srv::Request,
    ) -> Result<ServiceResponseTask<Srv>> {
        let id = self.handle.send_request::<Srv>(request)?;
        let (sender, receiver) = oneshot::channel::<Srv::Response>();
        self.pendings.lock().unwrap().insert(id, sender);

        Ok(ServiceResponseTask {
            receiver,
            client: Arc::downgrade(self) as Weak<dyn ClientBase>,
        })
    }

    pub fn service_name(&self) -> String {
        self.handle.service_name()
    }

    pub fn service_is_available(&self) -> Result<bool> {
        self.handle
            .service_is_available(&self.node_handle.lock().unwrap())
    }

    pub fn is_valid(&self) -> bool {
        self.handle.is_valid()
    }
}

impl<Srv> ClientBase for Client<Srv>
where
    Srv: ServiceT,
{
    fn handle(&self) -> &RclClient {
        &self.handle
    }

    fn process_requests(&self) -> Result<()> {
        match self.handle.take_response::<Srv>() {
            Ok((req_header, res)) => {
                let mut pendings = self.pendings.lock().unwrap();
                let sender = pendings
                    .remove(&req_header.sequence_number)
                    .ok_or_else(|| anyhow!("fail to find key in Client::process_requests"))?;
                sender.send(unsafe { res.to_rust() }).map_err(|_| {
                    anyhow!("fail to send response via channel in Client::process_requests")
                })?;
                Ok(())
            }
            Err(e) => match e.downcast_ref::<RclRustError>() {
                Some(RclRustError::RclClientTakeFailed(_)) => Ok(()),
                _ => Err(e),
            },
        }
    }
}

impl<Srv> Drop for Client<Srv>
where
    Srv: ServiceT,
{
    fn drop(&mut self) {
        if let Err(e) = unsafe { self.handle.fini(&mut self.node_handle.lock().unwrap()) } {
            rclrust_error!(
                Logger::new("rclrust"),
                "Failed to clean up rcl client handle: {}",
                e
            )
        }
    }
}

pub struct ServiceResponseTask<Srv>
where
    Srv: ServiceT,
{
    receiver: oneshot::Receiver<Srv::Response>,
    client: Weak<dyn ClientBase>,
}

impl<Srv> ServiceResponseTask<Srv>
where
    Srv: ServiceT,
{
    pub fn wait_response(mut self, context: &Context) -> Result<Option<Srv::Response>> {
        while context.is_valid() {
            match self.spin_some(context, Duration::from_nanos(500)) {
                Ok(_) => {}
                Err(e) => match e.downcast_ref::<RclRustError>() {
                    Some(RclRustError::RclTimeout(_)) => {}
                    _ => return Err(e),
                },
            }
            match self.receiver.try_recv() {
                Ok(Some(v)) => return Ok(Some(v)),
                Ok(None) => {}
                Err(_) => return Err(RclRustError::ServiceIsCanceled.into()),
            }
        }

        Ok(None)
    }

    fn spin_some(&self, context: &Context, max_duration: Duration) -> Result<()> {
        if let Some(client) = self.client.upgrade() {
            let mut wait_set =
                RclWaitSet::new(&mut context.handle.lock().unwrap(), 0, 0, 0, 1, 0, 0)?;

            wait_set.clear()?;
            wait_set.add_client(client.handle())?;
            wait_set.wait(max_duration.as_nanos() as i64)?;
            client.process_requests()?;
        }

        Ok(())
    }
}
