use std::{
    collections::HashMap,
    ffi::CString,
    fmt,
    mem::MaybeUninit,
    os::raw::c_void,
    sync::{Arc, Mutex},
    thread,
    time::Duration,
};

use anyhow::{Context as _, Result};
use futures::channel::{mpsc, oneshot};
use rclrust_msg::_core::{FFIToRust, MessageT, ServiceResponseRaw, ServiceT};

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
pub(crate) struct RclClient {
    r#impl: Box<rcl_sys::rcl_client_t>,
    node: Arc<Mutex<RclNode>>,
}

unsafe impl Send for RclClient {}
unsafe impl Sync for RclClient {}

impl RclClient {
    pub fn new<Srv>(node: Arc<Mutex<RclNode>>, service_name: &str, qos: &QoSProfile) -> Result<Self>
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
    fn raw_mut(&mut self) -> &mut rcl_sys::rcl_client_t {
        &mut self.r#impl
    }

    fn send_request<Srv>(&self, request: &Srv::Request) -> Result<i64>
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

    fn take_response<Srv>(&self) -> Result<(rcl_sys::rmw_request_id_t, ServiceResponseRaw<Srv>)>
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

    fn service_name(&self) -> String {
        unsafe {
            let name = rcl_sys::rcl_client_get_service_name(self.raw());
            String::from_c_char(name).unwrap()
        }
    }

    fn service_is_available(&self) -> Result<bool> {
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

    fn is_valid(&self) -> bool {
        unsafe { rcl_sys::rcl_client_is_valid(self.raw()) }
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

type ChannelMessage<Srv> = (rcl_sys::rmw_request_id_t, ServiceResponseRaw<Srv>);

pub struct Client<Srv>
where
    Srv: ServiceT + 'static,
{
    handle: Arc<RclClient>,
    worker: ReceiveWorker<ChannelMessage<Srv>>,
    pendings: Arc<Mutex<HashMap<i64, oneshot::Sender<Srv::Response>>>>,
}

impl<Srv> Client<Srv>
where
    Srv: ServiceT,
{
    pub(crate) fn new(node: &Node, service_name: &str, qos: &QoSProfile) -> Result<Self> {
        let handle = Arc::new(RclClient::new::<Srv>(
            node.clone_handle(),
            service_name,
            qos,
        )?);

        let pendings = Arc::new(Mutex::new(
            HashMap::<i64, oneshot::Sender<Srv::Response>>::new(),
        ));

        let callback = {
            let pendings = Arc::clone(&pendings);

            move |(req_header, res): (rcl_sys::rmw_request_id_t, ServiceResponseRaw<Srv>)| {
                pendings
                    .lock()
                    .unwrap()
                    .remove(&req_header.sequence_number)
                    .unwrap_or_else(|| panic!("fail to find key in Client::process_requests"))
                    .send(unsafe { res.to_rust() })
                    .unwrap_or_else(|_| {
                        panic!("fail to send response via channel in Client::process_requests")
                    });
            }
        };

        Ok(Self {
            handle,
            worker: ReceiveWorker::new(callback),
            pendings,
        })
    }

    pub(crate) fn create_invoker(&self) -> ClientInvoker<Srv> {
        ClientInvoker {
            handle: self.clone_handle(),
            tx: Some(self.clone_tx()),
        }
    }

    pub(crate) fn clone_handle(&self) -> Arc<RclClient> {
        Arc::clone(&self.handle)
    }

    pub(crate) fn clone_tx(&self) -> mpsc::Sender<WorkerMessage<ChannelMessage<Srv>>> {
        self.worker.clone_tx()
    }

    pub async fn send_request(&mut self, request: &Srv::Request) -> Result<Srv::Response> {
        let id = self.handle.send_request::<Srv>(request)?;
        let (tx, rx) = oneshot::channel::<Srv::Response>();
        self.pendings.lock().unwrap().insert(id, tx);

        Ok(rx.await?)
    }

    pub fn service_name(&self) -> String {
        self.handle.service_name()
    }

    pub fn wait_service(&self) -> Result<()> {
        while !self.service_is_available()? {
            thread::sleep(Duration::from_millis(1));
        }
        Ok(())
    }

    pub fn service_is_available(&self) -> Result<bool> {
        self.handle.service_is_available()
    }

    pub fn is_valid(&self) -> bool {
        self.handle.is_valid()
    }
}

pub(crate) trait ClientInvokerBase: fmt::Debug {
    fn handle(&self) -> &RclClient;
    fn invoke(&mut self) -> Result<()>;
}

pub(crate) struct ClientInvoker<Srv>
where
    Srv: ServiceT,
{
    handle: Arc<RclClient>,
    tx: Option<mpsc::Sender<WorkerMessage<ChannelMessage<Srv>>>>,
}

impl<Srv> ClientInvoker<Srv>
where
    Srv: ServiceT,
{
    fn stop(&mut self) {
        self.tx.take();
    }
}

impl<Srv> fmt::Debug for ClientInvoker<Srv>
where
    Srv: ServiceT,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "ClientInvoker {{{:?}}}", self.handle)
    }
}

impl<Srv> ClientInvokerBase for ClientInvoker<Srv>
where
    Srv: ServiceT,
{
    fn handle(&self) -> &RclClient {
        &self.handle
    }

    fn invoke(&mut self) -> Result<()> {
        if let Some(ref mut tx) = self.tx {
            match tx.try_send(WorkerMessage::Message(self.handle.take_response::<Srv>()?)) {
                Ok(_) => (),
                Err(e) if e.is_disconnected() => self.stop(),
                Err(_) => {
                    return Err(RclRustError::MessageQueueIsFull {
                        type_: "Client",
                        name: self.handle.service_name(),
                    }
                    .into())
                }
            }
        }

        Ok(())
    }
}
