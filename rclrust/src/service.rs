use std::{
    ffi::{c_void, CString},
    fmt,
    mem::MaybeUninit,
    sync::{Arc, Mutex},
};

use anyhow::{Context, Result};
use futures::channel::mpsc;
use rclrust_msg::_core::{MessageT, ServiceT};

use crate::{
    error::{RclRustError, ToRclRustResult},
    internal::{
        ffi::*,
        worker::{ReceiveWorker, WorkerMessage},
    },
    log::Logger,
    node::{Node, RclNode},
    qos::QoSProfile,
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
    fn new<Srv>(node: Arc<Mutex<RclNode>>, service_name: &str, qos: &QoSProfile) -> Result<Self>
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
                node.lock().unwrap().raw(),
                Srv::type_support() as *const _,
                service_c_str.as_ptr(),
                &options,
            )
            .to_result()?;
        }

        Ok(Self {
            r#impl: service,
            node,
        })
    }

    pub const fn raw(&self) -> &rcl_sys::rcl_service_t {
        &self.r#impl
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
                &*self.r#impl,
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
                &*self.r#impl,
                response_header,
                &response.to_raw_ref() as *const _ as *mut c_void,
            )
            .to_result()
            .with_context(|| "rcl_sys::rcl_send_response in RclService::send_response")
        }
    }

    fn service_name(&self) -> String {
        unsafe {
            let name = rcl_sys::rcl_service_get_service_name(&*self.r#impl);
            String::from_c_char(name).unwrap()
        }
    }

    fn is_valid(&self) -> bool {
        unsafe { rcl_sys::rcl_service_is_valid(&*self.r#impl) }
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

type ChannelMessage<Srv> = (
    rcl_sys::rmw_request_id_t,
    <<Srv as ServiceT>::Request as MessageT>::Raw,
);

pub struct Service<Srv>
where
    Srv: ServiceT,
{
    handle: Arc<RclService>,
    worker: ReceiveWorker<ChannelMessage<Srv>>,
}

impl<Srv> Service<Srv>
where
    Srv: ServiceT,
{
    pub(crate) fn new<F>(
        node: &Node,
        service_name: &str,
        callback: F,
        qos: &QoSProfile,
    ) -> Result<Self>
    where
        <Srv::Request as MessageT>::Raw: 'static,
        F: Fn(&<Srv::Request as MessageT>::Raw) -> Srv::Response + Send + 'static,
    {
        let handle = Arc::new(RclService::new::<Srv>(
            node.clone_handle(),
            service_name,
            qos,
        )?);

        let callback = {
            let handle = Arc::clone(&handle);

            move |(mut req_header, req)| {
                let res = (callback)(&req);
                handle.send_response::<Srv>(&mut req_header, res).unwrap();
            }
        };

        Ok(Self {
            handle,
            worker: ReceiveWorker::new(callback),
        })
    }

    pub(crate) fn create_invoker(&self) -> ServiceInvoker<Srv> {
        ServiceInvoker {
            handle: self.clone_handle(),
            tx: Some(self.clone_tx()),
        }
    }

    pub(crate) fn clone_handle(&self) -> Arc<RclService> {
        Arc::clone(&self.handle)
    }

    pub(crate) fn clone_tx(&self) -> mpsc::Sender<WorkerMessage<ChannelMessage<Srv>>> {
        self.worker.clone_tx()
    }

    pub fn service_name(&self) -> String {
        self.handle.service_name()
    }

    pub fn is_valid(&self) -> bool {
        self.handle.is_valid()
    }
}

pub(crate) trait ServiceInvokerBase: fmt::Debug {
    fn handle(&self) -> &RclService;
    fn invoke(&mut self) -> Result<()>;
}

pub(crate) struct ServiceInvoker<Srv>
where
    Srv: ServiceT,
{
    handle: Arc<RclService>,
    tx: Option<mpsc::Sender<WorkerMessage<ChannelMessage<Srv>>>>,
}

impl<Srv> ServiceInvoker<Srv>
where
    Srv: ServiceT,
{
    fn stop(&mut self) {
        self.tx.take();
    }
}

impl<Srv> fmt::Debug for ServiceInvoker<Srv>
where
    Srv: ServiceT,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "ServiceInvoker {{{:?}}}", self.handle)
    }
}

impl<Srv> ServiceInvokerBase for ServiceInvoker<Srv>
where
    Srv: ServiceT,
{
    fn handle(&self) -> &RclService {
        &self.handle
    }

    fn invoke(&mut self) -> Result<()> {
        if let Some(ref mut tx) = self.tx {
            match tx.try_send(WorkerMessage::Message(self.handle.take_request::<Srv>()?)) {
                Ok(_) => (),
                Err(e) if e.is_disconnected() => self.stop(),
                Err(_) => {
                    return Err(RclRustError::MessageQueueIsFull {
                        type_: "Service",
                        name: self.handle.service_name(),
                    }
                    .into())
                }
            }
        }

        Ok(())
    }
}
