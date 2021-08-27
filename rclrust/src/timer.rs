use std::{
    convert::TryInto,
    fmt,
    sync::{Arc, Mutex},
    time::Duration,
};

use anyhow::{Context, Result};
use futures::channel::mpsc;

use crate::{
    clock::{Clock, ClockType},
    context::RclContext,
    error::{RclRustError, ToRclRustResult},
    internal::worker::{ReceiveWorker, WorkerMessage},
    log::Logger,
    node::Node,
    rclrust_error,
};

#[derive(Debug)]
pub struct RclTimer(Box<rcl_sys::rcl_timer_t>);

unsafe impl Send for RclTimer {}

impl RclTimer {
    fn new(clock: &mut Clock, context: &mut RclContext, period: Duration) -> Result<Self> {
        let mut timer = Box::new(unsafe { rcl_sys::rcl_get_zero_initialized_timer() });

        unsafe {
            rcl_sys::rcl_timer_init(
                &mut *timer,
                clock.raw_mut(),
                context.raw_mut(),
                period.as_nanos().try_into().unwrap(),
                None,
                rcl_sys::rcutils_get_default_allocator(),
            )
            .to_result()
            .with_context(|| "rcl_sys::rcl_timer_init in RclTimer::new")?;
        }

        Ok(Self(timer))
    }

    #[inline]
    pub const fn raw(&self) -> &rcl_sys::rcl_timer_t {
        &self.0
    }

    #[allow(dead_code)]
    fn is_ready(&self) -> Result<bool> {
        let mut ready = false;
        unsafe {
            rcl_sys::rcl_timer_is_ready(self.raw(), &mut ready)
                .to_result()
                .with_context(|| "rcl_sys::rcl_timer_is_ready in RclTimer::is_ready")?;
        }
        Ok(ready)
    }

    fn call(&mut self) -> Result<()> {
        unsafe {
            rcl_sys::rcl_timer_call(&mut *self.0)
                .to_result()
                .with_context(|| "rcl_sys::rcl_timer_call in RclTimer::call")
        }
    }
}

impl Drop for RclTimer {
    fn drop(&mut self) {
        if let Err(e) = unsafe { rcl_sys::rcl_timer_fini(&mut *self.0).to_result() } {
            rclrust_error!(
                Logger::new("rclrust"),
                "Failed to clean up rcl timer handle: {}",
                e
            )
        }
    }
}

pub struct Timer {
    handle: Arc<Mutex<RclTimer>>,
    _clock: Box<Clock>,
    worker: ReceiveWorker<()>,
}

impl Timer {
    pub(crate) fn new<F>(
        node: &Node,
        period: Duration,
        clock_type: ClockType,
        callback: F,
    ) -> Result<Arc<Self>>
    where
        F: Fn() + Send + 'static,
    {
        let mut clock = Box::new(Clock::new(clock_type)?);
        let handle = Arc::new(Mutex::new(RclTimer::new(
            &mut clock,
            &mut node.context.lock().unwrap(),
            period,
        )?));
        Ok(Arc::new(Self {
            handle,
            _clock: clock,
            worker: ReceiveWorker::new(move |_| callback()),
        }))
    }

    pub(crate) fn create_invoker(&self) -> TimerInvoker {
        TimerInvoker {
            handle: self.clone_handle(),
            tx: Some(self.clone_tx()),
        }
    }

    pub(crate) fn clone_handle(&self) -> Arc<Mutex<RclTimer>> {
        Arc::clone(&self.handle)
    }

    pub(crate) fn clone_tx(&self) -> mpsc::Sender<WorkerMessage<()>> {
        self.worker.clone_tx()
    }
}

pub(crate) struct TimerInvoker {
    pub handle: Arc<Mutex<RclTimer>>,
    tx: Option<mpsc::Sender<WorkerMessage<()>>>,
}

impl fmt::Debug for TimerInvoker {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "TimerInvoker {{{:?}}}", self.handle)
    }
}

impl TimerInvoker {
    fn stop(&mut self) {
        self.tx.take();
    }

    pub fn invoke(&mut self) -> Result<()> {
        self.handle.lock().unwrap().call()?;
        if let Some(ref mut tx) = self.tx {
            match tx.try_send(WorkerMessage::Message(())) {
                Ok(_) => (),
                Err(e) if e.is_disconnected() => self.stop(),
                Err(_) => {
                    return Err(RclRustError::MessageQueueIsFull {
                        type_: "Timer",
                        name: "<none>".into(),
                    }
                    .into())
                }
            }
        }

        Ok(())
    }
}
