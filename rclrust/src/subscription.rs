use std::{
    ffi::CString,
    fmt,
    os::raw::c_void,
    sync::{Arc, Mutex},
};

use anyhow::{Context, Result};
use futures::channel::mpsc;
use rclrust_msg::_core::MessageT;

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
pub(crate) struct RclSubscription {
    r#impl: Box<rcl_sys::rcl_subscription_t>,
    node: Arc<Mutex<RclNode>>,
}

unsafe impl Send for RclSubscription {}
unsafe impl Sync for RclSubscription {}

impl RclSubscription {
    fn new<T>(node: Arc<Mutex<RclNode>>, topic_name: &str, qos: &QoSProfile) -> Result<Self>
    where
        T: MessageT,
    {
        let mut subscription =
            Box::new(unsafe { rcl_sys::rcl_get_zero_initialized_subscription() });
        let topic_c_str = CString::new(topic_name)?;
        let mut options = unsafe { rcl_sys::rcl_subscription_get_default_options() };
        options.qos = qos.into();

        unsafe {
            rcl_sys::rcl_subscription_init(
                &mut *subscription,
                node.lock().unwrap().raw(),
                T::type_support() as *const _,
                topic_c_str.as_ptr(),
                &options,
            )
            .to_result()
            .with_context(|| "rcl_sys::rcl_subscription_init in RclSubscription::new")?;
        }

        Ok(Self {
            r#impl: subscription,
            node,
        })
    }

    pub const fn raw(&self) -> &rcl_sys::rcl_subscription_t {
        &self.r#impl
    }

    fn take<T>(&self) -> Result<Arc<T::Raw>>
    where
        T: MessageT,
    {
        let mut message = T::Raw::default();
        unsafe {
            rcl_sys::rcl_take(
                &*self.r#impl,
                &mut message as *mut _ as *mut c_void,
                std::ptr::null_mut(),
                std::ptr::null_mut(),
            )
            .to_result()
            .with_context(|| "rcl_sys::rcl_take in RclSubscription::take")?;
        }

        Ok(Arc::new(message))
    }

    fn topic_name(&self) -> String {
        unsafe {
            let topic_name = rcl_sys::rcl_subscription_get_topic_name(&*self.r#impl);
            String::from_c_char(topic_name).unwrap()
        }
    }

    fn is_valid(&self) -> bool {
        unsafe { rcl_sys::rcl_subscription_is_valid(&*self.r#impl) }
    }

    fn publisher_count(&self) -> Result<usize> {
        let mut size = 0;
        unsafe {
            rcl_sys::rcl_subscription_get_publisher_count(&*self.r#impl, &mut size)
                .to_result()
                .with_context(|| {
                    "rcl_sys::rcl_subscription_get_publisher_count in RclSubscription::publisher_count"
                })?;
        }
        Ok(size)
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

pub struct Subscription<T>
where
    T: MessageT,
{
    handle: Arc<RclSubscription>,
    worker: ReceiveWorker<Arc<T::Raw>>,
}

impl<T> Subscription<T>
where
    T: MessageT,
{
    pub(crate) fn new<F>(
        node: &Node,
        topic_name: &str,
        callback: F,
        qos: &QoSProfile,
    ) -> Result<Self>
    where
        T::Raw: 'static,
        F: Fn(Arc<T::Raw>) + Send + 'static,
    {
        let handle = Arc::new(RclSubscription::new::<T>(
            node.clone_handle(),
            topic_name,
            qos,
        )?);

        Ok(Self {
            handle,
            worker: ReceiveWorker::new(callback),
        })
    }

    pub(crate) fn create_invoker(&self) -> SubscriptionInvoker<T> {
        SubscriptionInvoker {
            handle: self.clone_handle(),
            tx: Some(self.clone_tx()),
        }
    }

    pub(crate) fn clone_handle(&self) -> Arc<RclSubscription> {
        Arc::clone(&self.handle)
    }

    pub(crate) fn clone_tx(&self) -> mpsc::Sender<WorkerMessage<Arc<T::Raw>>> {
        self.worker.clone_tx()
    }

    pub fn topic_name(&self) -> String {
        self.handle.topic_name()
    }

    pub fn is_valid(&self) -> bool {
        self.handle.is_valid()
    }

    pub fn publisher_count(&self) -> Result<usize> {
        self.handle.publisher_count()
    }
}

pub(crate) trait SubscriptionInvokerBase: fmt::Debug {
    fn handle(&self) -> &RclSubscription;
    fn invoke(&mut self) -> Result<()>;
}

pub(crate) struct SubscriptionInvoker<T>
where
    T: MessageT,
{
    handle: Arc<RclSubscription>,
    tx: Option<mpsc::Sender<WorkerMessage<Arc<T::Raw>>>>,
}

impl<T> SubscriptionInvoker<T>
where
    T: MessageT,
{
    fn stop(&mut self) {
        self.tx.take();
    }
}

impl<T> fmt::Debug for SubscriptionInvoker<T>
where
    T: MessageT,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "SubscriptionInvoker {{{:?}}}", self.handle)
    }
}

impl<T> SubscriptionInvokerBase for SubscriptionInvoker<T>
where
    T: MessageT,
    T::Raw: 'static,
{
    fn handle(&self) -> &RclSubscription {
        &self.handle
    }

    fn invoke(&mut self) -> Result<()> {
        if let Some(ref mut tx) = self.tx {
            match tx.try_send(WorkerMessage::Message(self.handle.take::<T>()?)) {
                Ok(_) => (),
                Err(e) if e.is_disconnected() => self.stop(),
                Err(_) => {
                    return Err(RclRustError::MessageQueueIsFull {
                        type_: "Subscription",
                        name: self.handle.topic_name(),
                    }
                    .into())
                }
            }
        }

        Ok(())
    }
}
