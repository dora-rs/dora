use std::ffi::CString;
use std::os::raw::c_int;
use std::ptr::NonNull;
use std::sync::{Arc, Mutex};

use anyhow::Result;

use crate::error::ToRclRustResult;
use crate::init_options::InitOptions;
use crate::log::Logger;
use crate::node::Node;
use crate::node_options::NodeOptions;
use crate::rclrust_error;

#[derive(Debug)]
pub(crate) struct RclContext(NonNull<rcl_sys::rcl_context_t>);

unsafe impl Send for RclContext {}

impl RclContext {
    fn new(args: &[CString], init_options: &InitOptions) -> Result<Self> {
        let c_args: Vec<*const _> = args.iter().map(|s| s.as_ptr()).collect();
        let argv = if args.is_empty() {
            // to avoid invalid argument error
            std::ptr::null()
        } else {
            c_args.as_ptr()
        };

        unsafe {
            let mut handle = rcl_sys::rcl_get_zero_initialized_context();
            rcl_sys::rcl_init(c_args.len() as c_int, argv, init_options.raw(), &mut handle)
                .to_result()?;
            Ok(Self(NonNull::new(Box::into_raw(Box::new(handle))).unwrap()))
        }
    }

    pub unsafe fn as_mut_ptr(&mut self) -> *mut rcl_sys::rcl_context_t {
        self.0.as_mut()
    }

    fn is_valid(&mut self) -> bool {
        unsafe { rcl_sys::rcl_context_is_valid(self.0.as_mut()) }
    }

    fn shutdown(&mut self) -> Result<()> {
        if self.is_valid() {
            unsafe { rcl_sys::rcl_shutdown(self.0.as_mut()).to_result()? }
        }
        Ok(())
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
    handle: Mutex<RclContext>,
    shutdown_reason: Mutex<Option<String>>,
}

impl Context {
    pub(crate) fn new(args: &[CString], init_options: InitOptions) -> Result<Arc<Self>> {
        let handle = RclContext::new(args, &init_options)?;

        Ok(Arc::new(Self {
            handle: Mutex::new(handle),
            shutdown_reason: Mutex::new(None),
        }))
    }

    pub(crate) const fn handle(&self) -> &Mutex<RclContext> {
        &self.handle
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
    pub fn create_node<'a>(&'a self, name: &str) -> Result<Arc<Node<'a>>> {
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
    pub fn create_node_with_options<'a>(
        &'a self,
        name: &str,
        options: &NodeOptions,
    ) -> Result<Arc<Node<'a>>> {
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
    pub fn create_node_with_ns<'a>(&'a self, name: &str, namespace: &str) -> Result<Arc<Node<'a>>> {
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
    /// let node = ctx.create_node_with_ns_and_options("test_node", "ns", &options).unwrap();
    /// assert_eq!(&node.fully_qualified_name(), "/ns/test_node");
    /// ```
    pub fn create_node_with_ns_and_options<'a>(
        &'a self,
        name: &str,
        namespace: &str,
        options: &NodeOptions,
    ) -> Result<Arc<Node<'a>>> {
        Node::new(self, name, Some(namespace), options)
    }
}

impl Drop for Context {
    fn drop(&mut self) {
        let ret = unsafe { rcl_sys::rcl_logging_fini().to_result() };
        if let Err(e) = ret {
            rclrust_error!(
                Logger::new("rclrust"),
                "Failed to tear down the logging setup: {}",
                e
            )
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn context_init() -> Result<()> {
        let ctx = Context::new(&[], InitOptions::new()?)?;
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
