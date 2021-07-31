use crate::error::ToRclRustResult;
use crate::log::Logger;
use crate::rclrust_error;

#[derive(Debug)]
pub(crate) struct RclNodeOptions(rcl_sys::rcl_node_options_t);

unsafe impl Send for RclNodeOptions {}

impl RclNodeOptions {
    pub fn new() -> Self {
        Self(unsafe { rcl_sys::rcl_node_get_default_options() })
    }

    pub const fn raw(&self) -> &rcl_sys::rcl_node_options_t {
        &self.0
    }
}

impl Drop for RclNodeOptions {
    fn drop(&mut self) {
        let ret = unsafe { rcl_sys::rcl_node_options_fini(&mut self.0).to_result() };

        if let Err(e) = ret {
            rclrust_error!(
                Logger::new("rclrust"),
                "Failed to clean up rcl node init options: {}",
                e
            )
        }
    }
}

#[derive(Debug)]
pub struct NodeOptions {
    options: RclNodeOptions,
}

impl NodeOptions {
    pub fn new() -> Self {
        Self {
            options: RclNodeOptions::new(),
        }
    }

    pub(crate) const fn raw(&self) -> &rcl_sys::rcl_node_options_t {
        self.options.raw()
    }
}

impl Default for NodeOptions {
    fn default() -> Self {
        Self::new()
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
