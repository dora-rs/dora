use std::{
    ffi::CString,
    os::raw::c_int,
    sync::{Arc, Mutex},
};

use anyhow::{Context as _, Result};

use crate::{
    error::ToRclRustResult,
    init_options::InitOptions,
    log::{logging_output_handler, Logger, LOGGER_MUTEX},
    node::Node,
    node_options::NodeOptions,
    rclrust_error,
};

#[derive(Debug)]
pub(crate) struct RclContext(Box<rcl_sys::rcl_context_t>);

unsafe impl Send for RclContext {}

impl RclContext {
    fn new(args: Vec<String>, init_options: &InitOptions) -> Result<Self> {
        let mut handle = unsafe { Box::new(rcl_sys::rcl_get_zero_initialized_context()) };
        let args = args
            .into_iter()
            .map(CString::new)
            .collect::<std::result::Result<Vec<_>, _>>()?;
        let c_args: Vec<*const _> = args.iter().map(|s| s.as_ptr()).collect();

        let argv = if args.is_empty() {
            // to avoid invalid argument error
            std::ptr::null()
        } else {
            c_args.as_ptr()
        };

        unsafe {
            rcl_sys::rcl_init(
                c_args.len() as c_int,
                argv,
                init_options.raw(),
                &mut *handle,
            )
            .to_result()
            .with_context(|| "rcl_sys::rcl_init in RclContext::new")?;

            {
                let _guard = LOGGER_MUTEX.lock();
                rcl_sys::rcl_logging_configure_with_output_handler(
                    &handle.global_arguments,
                    &rcl_sys::rcutils_get_default_allocator(),
                    Some(logging_output_handler),
                )
                .to_result()?
            }

            Ok(Self(handle))
        }
    }

    pub fn raw_mut(&mut self) -> &mut rcl_sys::rcl_context_t {
        self.0.as_mut()
    }

    pub(crate) fn is_valid(&mut self) -> bool {
        unsafe { rcl_sys::rcl_context_is_valid(self.0.as_mut()) }
    }

    fn shutdown(&mut self) -> Result<()> {
        if self.is_valid() {
            unsafe {
                rcl_sys::rcl_shutdown(self.0.as_mut())
                    .to_result()
                    .with_context(|| "rcl_sys::rcl_shutdown in RclContext::shutdown")?
            }
        }
        Ok(())
    }

    pub(crate) const fn global_arguments(&self) -> &rcl_sys::rcl_arguments_t {
        &self.0.global_arguments
    }
}

impl Drop for RclContext {
    fn drop(&mut self) {
        if let Err(e) = self.shutdown() {
            rclrust_error!(
                Logger::new("rclrust"),
                "Failed to shutdown rcl context: {}",
                e
            )
        }
        if let Err(e) = unsafe { rcl_sys::rcl_context_fini(self.0.as_mut()).to_result() } {
            rclrust_error!(
                Logger::new("rclrust"),
                "Failed to clean up rcl context handle: {}",
                e
            )
        }
    }
}

#[derive(Debug)]
pub struct Context {
    pub(crate) handle: Arc<Mutex<RclContext>>,
    shutdown_reason: Mutex<Option<String>>,
}

impl Context {
    pub(crate) fn new(args: Vec<String>, init_options: InitOptions) -> Result<Self> {
        let handle = RclContext::new(args, &init_options)?;

        Ok(Self {
            handle: Arc::new(Mutex::new(handle)),
            shutdown_reason: Default::default(),
        })
    }

    pub fn is_valid(&self) -> bool {
        self.handle.lock().unwrap().is_valid()
    }

    pub(crate) fn shutdown(&self, reason: impl Into<String>) -> Result<()> {
        self.handle.lock().unwrap().shutdown()?;
        *self.shutdown_reason.lock().unwrap() = Some(reason.into());

        Ok(())
    }

    /// Create node with empty namespace and default options
    ///
    /// # Examples
    /// ```
    /// let ctx = rclrust::init().unwrap();
    /// let node = ctx.create_node("test_node").unwrap();
    /// assert_eq!(&node.fully_qualified_name(), "/test_node");
    /// ```
    pub fn create_node(&self, name: &str) -> Result<Node> {
        Node::new(self, name, None, &NodeOptions::new())
    }

    /// Create node with empty namespace and specified options
    ///
    /// # Examples
    /// ```
    /// use rclrust::NodeOptions;
    ///
    /// let ctx = rclrust::init().unwrap();
    /// let options = NodeOptions::new();
    /// let node = ctx.create_node_with_options("test_node", &options).unwrap();
    /// assert_eq!(&node.fully_qualified_name(), "/test_node");
    /// ```
    pub fn create_node_with_options(&self, name: &str, options: &NodeOptions) -> Result<Node> {
        Node::new(self, name, None, options)
    }

    /// Create node with namespace and default options
    ///
    /// # Examples
    /// ```
    /// let ctx = rclrust::init().unwrap();
    /// let node = ctx.create_node_with_ns("test_node", "ns").unwrap();
    /// assert_eq!(&node.fully_qualified_name(), "/ns/test_node");
    /// ```
    pub fn create_node_with_ns(&self, name: &str, namespace: &str) -> Result<Node> {
        Node::new(self, name, Some(namespace), &NodeOptions::new())
    }

    /// Create node with empty namespace and specified options
    //
    /// # Examples
    /// ```
    /// use rclrust::NodeOptions;
    ///
    /// let ctx = rclrust::init().unwrap();
    /// let options = NodeOptions::new();
    /// let node = ctx
    ///     .create_node_with_ns_and_options("test_node", "ns", &options)
    ///     .unwrap();
    /// assert_eq!(&node.fully_qualified_name(), "/ns/test_node");
    /// ```
    pub fn create_node_with_ns_and_options(
        &self,
        name: &str,
        namespace: &str,
        options: &NodeOptions,
    ) -> Result<Node> {
        Node::new(self, name, Some(namespace), options)
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn context_init() -> Result<()> {
        let ctx = Context::new(Vec::new(), InitOptions::new()?)?;
        assert!(ctx.is_valid());

        Ok(())
    }

    #[test]
    fn create_node() -> Result<()> {
        let ctx = crate::init()?;

        let node = ctx.create_node("test_node")?;
        assert_eq!(&node.fully_qualified_name(), "/test_node");

        Ok(())
    }

    #[test]
    fn create_node_with_ns() -> Result<()> {
        let ctx = crate::init()?;
        let node = ctx.create_node_with_ns("test_node", "ns")?;
        assert_eq!(&node.fully_qualified_name(), "/ns/test_node");

        Ok(())
    }
}
