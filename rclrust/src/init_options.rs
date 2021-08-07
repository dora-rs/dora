use anyhow::Result;

use crate::error::ToRclRustResult;
use crate::log::Logger;
use crate::rclrust_error;

#[derive(Debug)]
pub(crate) struct RclInitOptions(rcl_sys::rcl_init_options_t);

unsafe impl Send for RclInitOptions {}

impl RclInitOptions {
    pub fn new() -> Result<Self> {
        unsafe {
            let mut options = rcl_sys::rcl_get_zero_initialized_init_options();
            let allocator = rcl_sys::rcutils_get_default_allocator();
            rcl_sys::rcl_init_options_init(&mut options, allocator).to_result()?;
            Ok(Self(options))
        }
    }

    pub const fn raw(&self) -> &rcl_sys::rcl_init_options_t {
        &self.0
    }
}

impl Drop for RclInitOptions {
    fn drop(&mut self) {
        if let Err(e) = unsafe { rcl_sys::rcl_init_options_fini(&mut self.0).to_result() } {
            rclrust_error!(
                Logger::new("rclrust"),
                "Failed to clean up rcl init options handle: {}",
                e
            )
        }
    }
}

#[derive(Debug)]
pub struct InitOptions {
    options: RclInitOptions,
    // shutdown_on_sigint: bool,
    // initialize_logging: bool,
}

impl InitOptions {
    pub fn new() -> Result<Self> {
        Ok(Self {
            options: RclInitOptions::new()?,
            // shutdown_on_sigint: true,
            // initialize_logging: true,
        })
    }

    pub(crate) const fn raw(&self) -> &rcl_sys::rcl_init_options_t {
        self.options.raw()
    }

    // / # Examples
    // / ```
    // / use rclrust::InitOptions;
    // /
    // / let init_options = InitOptions::new().unwrap();
    // / assert_eq!(init_options.initialize_logging(), true);
    // / ```
    // pub const fn initialize_logging(&self) -> bool {
    //     self.initialize_logging
    // }

    // / # Examples
    // / ```
    // / use rclrust::InitOptions;
    // /
    // / let init_options = InitOptions::new().unwrap().set_initialize_logging(false);
    // / assert_eq!(init_options.initialize_logging(), false);
    // / ```
    // #[allow(clippy::missing_const_for_fn)]
    // pub fn set_initialize_logging(self, initialize_logging: bool) -> Self {
    //     Self {
    //         initialize_logging,
    //         ..self
    //     }
    // }

    // pub const fn shutdown_on_sigint(&self) -> bool {
    //     self.shutdown_on_sigint
    // }

    // #[allow(clippy::missing_const_for_fn)]
    // pub fn set_shutdown_on_sigint(self, shutdown_on_sigint: bool) -> Self {
    //     Self {
    //         shutdown_on_sigint,
    //         ..self
    //     }
    // }
}
