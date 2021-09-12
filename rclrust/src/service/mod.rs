//! Serve service

use std::sync::Arc;

use anyhow::Result;
use futures::channel::mpsc;
use rclrust_msg::_core::{MessageT, ServiceT};

use crate::{
    internal::worker::{ReceiveWorker, WorkerMessage},
    node::Node,
    qos::QoSProfile,
};

pub mod rcl_wrapper;
pub use rcl_wrapper::RclService;

pub mod invoker;
pub use invoker::{ServiceInvoker, ServiceInvokerBase};

type ChannelMessage<Srv> = (
    rcl_sys::rmw_request_id_t,
    <<Srv as ServiceT>::Request as MessageT>::Raw,
);

/// Service server
pub struct Service<Srv>
where
    Srv: ServiceT,
{
    handle: Arc<RclService>,
    worker: ReceiveWorker<ChannelMessage<Srv>>,
}

impl<Srv> Service<Srv>
where
    Srv: ServiceT,
{
    pub(crate) fn new<F>(
        node: &Node,
        service_name: &str,
        callback: F,
        qos: &QoSProfile,
    ) -> Result<Self>
    where
        <Srv::Request as MessageT>::Raw: 'static,
        F: Fn(&<Srv::Request as MessageT>::Raw) -> Srv::Response + Send + 'static,
    {
        let handle = Arc::new(RclService::new::<Srv>(
            node.clone_handle(),
            service_name,
            qos,
        )?);

        let callback = {
            let handle = Arc::clone(&handle);

            move |(mut req_header, req)| {
                let res = (callback)(&req);
                handle.send_response::<Srv>(&mut req_header, res).unwrap();
            }
        };

        Ok(Self {
            handle,
            worker: ReceiveWorker::new(callback),
        })
    }

    /// Get the service name.
    ///
    /// # Examples
    ///
    /// ```
    /// # use anyhow::Result;
    /// # use rclrust::qos::QoSProfile;
    /// # use rclrust_msg::std_srvs::srv::{Empty, Empty_Response};
    /// #
    /// # #[tokio::main]
    /// # async fn main() -> Result<()> {
    /// # let ctx = rclrust::init()?;
    /// # let mut node = ctx.create_node("node")?;
    /// # let callback = |_req| Empty_Response {};
    /// let subscription =
    ///     node.create_service::<Empty, _>("service", callback, &QoSProfile::default())?;
    /// assert_eq!(subscription.service_name().unwrap(), "/service");
    /// # Ok(())
    /// # }
    /// ```
    pub fn service_name(&self) -> Option<String> {
        self.handle.service_name()
    }

    /// Check if this service is valid or not. Normally, a return value should be `true`.
    ///
    /// # Examples
    ///
    /// ```
    /// # use anyhow::Result;
    /// # use rclrust::qos::QoSProfile;
    /// # use rclrust_msg::std_srvs::srv::{Empty, Empty_Response};
    /// #
    /// # #[tokio::main]
    /// # async fn main() -> Result<()> {
    /// # let ctx = rclrust::init()?;
    /// # let mut node = ctx.create_node("node")?;
    /// # let callback = |_req| Empty_Response {};
    /// let subscription =
    ///     node.create_service::<Empty, _>("service", callback, &QoSProfile::default())?;
    /// assert!(subscription.is_valid());
    /// # Ok(())
    /// # }
    /// ```
    pub fn is_valid(&self) -> bool {
        self.handle.is_valid()
    }

    pub(crate) fn create_invoker(&self) -> ServiceInvoker<Srv> {
        ServiceInvoker::new_from_target(self)
    }

    pub(crate) fn clone_handle(&self) -> Arc<RclService> {
        Arc::clone(&self.handle)
    }

    pub(crate) fn clone_tx(&self) -> mpsc::Sender<WorkerMessage<ChannelMessage<Srv>>> {
        self.worker.clone_tx()
    }
}

#[cfg(test)]
mod test {
    use rclrust_msg::std_srvs::srv::{Empty, Empty_Response};

    use super::*;

    fn random_name() -> String {
        use rand::{distributions::Alphanumeric, thread_rng, Rng};

        thread_rng()
            .sample_iter(&Alphanumeric)
            .map(char::from)
            .filter(|c| c.is_alphabetic())
            .take(20)
            .collect()
    }

    #[tokio::test]
    async fn service_service_name() -> Result<()> {
        let ctx = crate::init()?;
        let mut node = ctx.create_node(&random_name())?;
        let service = node.create_service::<Empty, _>(
            "service",
            |_req| Empty_Response {},
            &QoSProfile::default(),
        )?;
        assert_eq!(service.service_name().unwrap(), "/service");
        Ok(())
    }

    #[tokio::test]
    async fn service_is_valid() -> Result<()> {
        let ctx = crate::init()?;
        let mut node = ctx.create_node(&random_name())?;
        let service = node.create_service::<Empty, _>(
            "service",
            |_req| Empty_Response {},
            &QoSProfile::default(),
        )?;
        assert!(service.is_valid());
        Ok(())
    }
}
