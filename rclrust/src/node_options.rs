use crate::{error::ToRclRustResult, log::Logger, rclrust_error};

#[derive(Debug)]
pub(crate) struct RclNodeOptions(rcl_sys::rcl_node_options_t);

unsafe impl Send for RclNodeOptions {}

impl RclNodeOptions {
    #[inline]
    pub const fn raw(&self) -> &rcl_sys::rcl_node_options_t {
        &self.0
    }
}

impl Default for RclNodeOptions {
    fn default() -> Self {
        Self(unsafe { rcl_sys::rcl_node_get_default_options() })
    }
}

impl Drop for RclNodeOptions {
    fn drop(&mut self) {
        if let Err(e) = unsafe { rcl_sys::rcl_node_options_fini(&mut self.0).to_result() } {
            rclrust_error!(
                Logger::new("rclrust"),
                "Failed to clean up rcl node init options: {}",
                e
            )
        }
    }
}

#[derive(Debug, Default)]
pub struct NodeOptions {
    options: RclNodeOptions,
}

impl NodeOptions {
    pub fn new() -> Self {
        Default::default()
    }

    pub(crate) const fn raw(&self) -> &rcl_sys::rcl_node_options_t {
        self.options.raw()
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_node_options_new() {
        let _options = NodeOptions::new();
    }
}
