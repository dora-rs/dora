use anyhow::{Context, Result};

use crate::client::RclClient;
use crate::context::RclContext;
use crate::error::ToRclRustResult;
use crate::log::Logger;
use crate::rclrust_error;
use crate::service::RclService;
use crate::subscription::RclSubscription;
use crate::timer::RclTimer;

#[derive(Debug)]
pub(crate) struct RclWaitSet(rcl_sys::rcl_wait_set_t);

impl RclWaitSet {
    pub fn new(
        context: &mut RclContext,
        n_subscriptions: usize,
        n_guard_conditions: usize,
        n_timers: usize,
        n_clients: usize,
        n_services: usize,
        n_events: usize,
    ) -> Result<Self> {
        let mut wait_set = unsafe { rcl_sys::rcl_get_zero_initialized_wait_set() };

        unsafe {
            rcl_sys::rcl_wait_set_init(
                &mut wait_set,
                n_subscriptions,
                n_guard_conditions,
                n_timers,
                n_clients,
                n_services,
                n_events,
                context.raw_mut(),
                rcl_sys::rcutils_get_default_allocator(),
            )
            .to_result()
            .with_context(|| "rcl_sys::rcl_wait_set_init in RclWaitSet::new")?;
        }

        Ok(Self(wait_set))
    }

    pub fn wait(&mut self, timeout: i64) -> Result<()> {
        unsafe {
            rcl_sys::rcl_wait(&mut self.0, timeout)
                .to_result()
                .with_context(|| "rcl_sys::rcl_wait in RclWaitSet::wait")
        }
    }

    #[allow(unused)]
    fn is_valid(&self) -> bool {
        unsafe { rcl_sys::rcl_wait_set_is_valid(&self.0) }
    }

    pub fn clear(&mut self) -> Result<()> {
        unsafe {
            rcl_sys::rcl_wait_set_clear(&mut self.0)
                .to_result()
                .with_context(|| "rcl_sys::rcl_wait_set_clear in RclWaitSet::clear")
        }
    }

    pub fn add_subscription(&mut self, subscription: &RclSubscription) -> Result<()> {
        unsafe {
            rcl_sys::rcl_wait_set_add_subscription(
                &mut self.0,
                subscription.raw(),
                std::ptr::null_mut(),
            )
            .to_result()
            .with_context(|| {
                "rcl_sys::rcl_wait_set_add_subscription in RclWaitSet::add_subscription"
            })
        }
    }

    pub fn add_timer(&mut self, timer: &RclTimer) -> Result<()> {
        unsafe {
            rcl_sys::rcl_wait_set_add_timer(&mut self.0, timer.raw(), std::ptr::null_mut())
                .to_result()
                .with_context(|| "rcl_sys::rcl_wait_set_add_timer in RclWaitSet::add_timer")
        }
    }

    pub fn add_client(&mut self, client: &RclClient) -> Result<()> {
        unsafe {
            rcl_sys::rcl_wait_set_add_client(&mut self.0, client.raw(), std::ptr::null_mut())
                .to_result()
                .with_context(|| "rcl_sys::rcl_wait_set_add_client in RclWaitSet::add_client")
        }
    }

    pub fn add_service(&mut self, service: &RclService) -> Result<()> {
        unsafe {
            rcl_sys::rcl_wait_set_add_service(&mut self.0, service.raw(), std::ptr::null_mut())
                .to_result()
                .with_context(|| "rcl_sys::rcl_wait_set_add_service in RclWaitSet::add_service")
        }
    }
}

impl Drop for RclWaitSet {
    fn drop(&mut self) {
        if let Err(e) = unsafe { rcl_sys::rcl_wait_set_fini(&mut self.0).to_result() } {
            rclrust_error!(
                Logger::new("rclrust"),
                "rcl_wait_set_fini should succeed. {}",
                e
            );
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_rcl_wait_set_new() -> Result<()> {
        let ctx = crate::init()?;
        let wait_set = RclWaitSet::new(&mut ctx.handle().lock().unwrap(), 1, 1, 1, 1, 1, 1)?;
        assert!(wait_set.is_valid());

        Ok(())
    }

    #[test]
    fn test_rcl_wait_set_clear() -> Result<()> {
        let ctx = crate::init()?;
        let mut wait_set = RclWaitSet::new(&mut ctx.handle().lock().unwrap(), 1, 1, 1, 1, 1, 1)?;
        wait_set.clear()?;

        Ok(())
    }
}
