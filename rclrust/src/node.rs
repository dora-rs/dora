use std::ffi::CString;
use std::sync::{Arc, Mutex};

use anyhow::{ensure, Result};

use crate::context::{Context, RclContext};
use crate::error::ToRclRustResult;
use crate::internal::ffi::*;
use crate::log::Logger;
use crate::node_options::NodeOptions;
use crate::rclrust_error;

#[derive(Debug)]
pub(crate) struct RclNode(rcl_sys::rcl_node_t);

unsafe impl Send for RclNode {}

impl RclNode {
    fn new(
        context: &mut RclContext,
        name: &str,
        namespace: Option<&str>,
        options: &NodeOptions,
    ) -> Result<Self> {
        let mut node = unsafe { rcl_sys::rcl_get_zero_initialized_node() };
        let name_c_str = CString::new(name)?;
        let namespace_c_str = CString::new(namespace.unwrap_or_default())?;

        unsafe {
            rcl_sys::rcl_node_init(
                &mut node,
                name_c_str.as_ptr(),
                namespace_c_str.as_ptr(),
                context.as_mut_ptr(),
                options.raw(),
            )
            .to_result()?;
        }

        Ok(Self(node))
    }

    fn valid(&self) -> bool {
        unsafe { rcl_sys::rcl_node_is_valid(&self.0) }
    }

    fn name(&self) -> String {
        unsafe {
            let name = rcl_sys::rcl_node_get_name(&self.0);
            String::from_c_char(name).unwrap()
        }
    }

    fn namespace(&self) -> String {
        unsafe {
            let namespace = rcl_sys::rcl_node_get_namespace(&self.0);
            String::from_c_char(namespace).unwrap()
        }
    }

    fn fully_qualified_name(&self) -> String {
        unsafe {
            let name = rcl_sys::rcl_node_get_fully_qualified_name(&self.0);
            String::from_c_char(name).unwrap()
        }
    }

    fn logger_name(&self) -> String {
        unsafe {
            let logger_name = rcl_sys::rcl_node_get_logger_name(&self.0);
            String::from_c_char(logger_name).unwrap()
        }
    }

    unsafe fn fini(&mut self, ctx: &Mutex<RclContext>) -> Result<()> {
        let _guard = ctx.lock();
        rcl_sys::rcl_node_fini(&mut self.0).to_result()
    }
}

#[derive(Debug)]
pub struct Node<'a> {
    handle: Mutex<RclNode>,
    context_handle: &'a Mutex<RclContext>,
}

impl<'a> Node<'a> {
    pub(crate) fn new(
        context: &'a Context,
        name: &str,
        namespace: Option<&str>,
        options: &NodeOptions,
    ) -> Result<Arc<Self>> {
        ensure!(context.valid(), "given Context is not valid");

        let handle = {
            let mut context_handle_inner = context.handle().lock().unwrap();
            RclNode::new(&mut context_handle_inner, name, namespace, options)?
        };

        Ok(Arc::new(Self {
            handle: Mutex::new(handle),
            context_handle: context.handle(),
        }))
    }

    /// # Examples
    ///
    /// ```
    /// let ctx = rclrust::init().unwrap();
    /// let node = ctx.create_node("node1").unwrap();
    /// assert!(node.valid())
    /// ```
    pub fn valid(&self) -> bool {
        self.handle.lock().unwrap().valid()
    }

    /// # Examples
    ///
    /// ```
    /// let ctx = rclrust::init().unwrap();
    /// let node = ctx.create_node("node1").unwrap();
    /// assert_eq!(&node.name(), "node1");
    /// ```
    pub fn name(&self) -> String {
        self.handle.lock().unwrap().name()
    }

    /// # Examples
    ///
    /// ```
    /// let ctx = rclrust::init().unwrap();
    /// let node = ctx.create_node_with_ns("node1", "ns").unwrap();
    /// assert_eq!(&node.namespace(), "/ns");
    ///
    /// let node = ctx.create_node("node1").unwrap();
    /// assert_eq!(&node.namespace(), "/");
    /// ```
    pub fn namespace(&self) -> String {
        self.handle.lock().unwrap().namespace()
    }

    /// # Examples
    ///
    /// ```
    /// let ctx = rclrust::init().unwrap();
    /// let node = ctx.create_node_with_ns("node1", "ns").unwrap();
    /// assert_eq!(&node.fully_qualified_name(), "/ns/node1");
    /// ```
    pub fn fully_qualified_name(&self) -> String {
        self.handle.lock().unwrap().fully_qualified_name()
    }

    /// # Examples
    ///
    /// ```
    /// let ctx = rclrust::init().unwrap();
    /// let node = ctx.create_node_with_ns("node1", "ns").unwrap();
    /// let logger = node.logger();
    /// ```
    pub fn logger(&self) -> Logger {
        Logger::new(&self.logger_name())
    }

    /// # Examples
    ///
    /// ```
    /// let ctx = rclrust::init().unwrap();
    /// let node = ctx.create_node_with_ns("node1", "ns").unwrap();
    /// assert_eq!(&node.logger_name(), "ns.node1");
    /// ```
    pub fn logger_name(&self) -> String {
        self.handle.lock().unwrap().logger_name()
    }
}

impl<'a> Drop for Node<'a> {
    fn drop(&mut self) {
        let ret = unsafe { self.handle.lock().unwrap().fini(self.context_handle) };
        if let Err(e) = ret {
            rclrust_error!(
                Logger::new("rclrust"),
                "Failed to clean up rcl node handle: {}",
                e
            )
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn node_init() -> Result<()> {
        let ctx = crate::init()?;
        let node = ctx.create_node("test_node")?;
        assert!(node.valid());

        Ok(())
    }

    #[test]
    fn node_name_without_namespace() -> Result<()> {
        let ctx = crate::init()?;
        let node = ctx.create_node("test_node")?;
        assert_eq!(node.name(), "test_node");
        assert_eq!(node.namespace(), "/");
        assert_eq!(node.fully_qualified_name(), "/test_node");
        assert_eq!(node.logger_name(), "test_node");

        Ok(())
    }

    #[test]
    fn node_name_with_namespace() -> Result<()> {
        let ctx = crate::init()?;
        let node = ctx.create_node_with_ns("test_node", "ns1")?;
        assert_eq!(node.name(), "test_node");
        assert_eq!(node.namespace(), "/ns1");
        assert_eq!(node.fully_qualified_name(), "/ns1/test_node");
        assert_eq!(node.logger_name(), "ns1.test_node");

        Ok(())
    }

    #[test]
    fn node_logger() -> Result<()> {
        let ctx = crate::init()?;
        let node = ctx.create_node_with_ns("test_node", "ns1")?;
        crate::rclrust_debug!(node.logger(), "logging {}{}{}", 21, "abc", 20.);

        Ok(())
    }
}
