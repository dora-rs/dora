use std::convert::TryInto;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use anyhow::{Context, Result};

use crate::clock::{Clock, ClockType};
use crate::context::RclContext;
use crate::error::ToRclRustResult;
use crate::log::Logger;
use crate::node::Node;
use crate::rclrust_error;

#[derive(Debug)]
pub struct RclTimer(rcl_sys::rcl_timer_t);

unsafe impl Send for RclTimer {}

impl RclTimer {
    fn new(clock: &mut Clock, context: &mut RclContext, period: Duration) -> Result<Self> {
        let mut timer = unsafe { rcl_sys::rcl_get_zero_initialized_timer() };

        unsafe {
            rcl_sys::rcl_timer_init(
                &mut timer,
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

    pub const fn raw(&self) -> &rcl_sys::rcl_timer_t {
        &self.0
    }

    fn is_ready(&self) -> Result<bool> {
        let mut ready = false;
        unsafe {
            rcl_sys::rcl_timer_is_ready(&self.0, &mut ready)
                .to_result()
                .with_context(|| "rcl_sys::rcl_timer_is_ready in RclTimer::is_ready")?;
        }
        Ok(ready)
    }

    fn call(&mut self) -> Result<()> {
        unsafe {
            rcl_sys::rcl_timer_call(&mut self.0)
                .to_result()
                .with_context(|| "rcl_sys::rcl_timer_call in RclTimer::call")
        }
    }
}

impl Drop for RclTimer {
    fn drop(&mut self) {
        if let Err(e) = unsafe { rcl_sys::rcl_timer_fini(&mut self.0).to_result() } {
            rclrust_error!(
                Logger::new("rclrust"),
                "Failed to clean up rcl timer handle: {}",
                e
            )
        }
    }
}

pub struct Timer {
    handle: Mutex<RclTimer>,
    _clock: Box<Clock>,
    callback: Box<dyn Fn()>,
}

impl<'ctx> Timer {
    pub(crate) fn new<F>(
        node: &Node<'ctx>,
        period: Duration,
        clock_type: ClockType,
        callback: F,
    ) -> Result<Arc<Self>>
    where
        F: Fn() + 'static,
    {
        let mut clock = Box::new(Clock::new(clock_type)?);
        let handle = RclTimer::new(&mut clock, &mut node.context.handle.lock().unwrap(), period)?;
        Ok(Arc::new(Self {
            handle: Mutex::new(handle),
            _clock: clock,
            callback: Box::new(callback),
        }))
    }

    pub(crate) const fn handle(&self) -> &Mutex<RclTimer> {
        &self.handle
    }

    pub(crate) fn call_callback(&self) -> Result<()> {
        let mut handle = self.handle.lock().unwrap();

        if handle.is_ready()? {
            handle.call()?;
            drop(handle);
            (self.callback)()
        }
        Ok(())
    }
}
