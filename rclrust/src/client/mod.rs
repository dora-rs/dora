//! Service client

use std::{
    collections::HashMap,
    sync::{Arc, Mutex},
    thread,
    time::Duration,
};

use anyhow::Result;
use futures::channel::{mpsc, oneshot};
use rclrust_msg::_core::{FFIToRust, ServiceResponseRaw, ServiceT};

use crate::{
    internal::worker::{ReceiveWorker, WorkerMessage},
    node::Node,
    qos::QoSProfile,
};

pub mod rcl_wrapper;
pub use rcl_wrapper::RclClient;

pub mod invoker;
pub use invoker::{ClientInvoker, ClientInvokerBase};

type ChannelMessage<Srv> = (rcl_sys::rmw_request_id_t, ServiceResponseRaw<Srv>);

/// Service client
pub struct Client<Srv>
where
    Srv: ServiceT + 'static,
{
    handle: Arc<RclClient>,
    worker: ReceiveWorker<ChannelMessage<Srv>>,
    pendings: Arc<Mutex<HashMap<i64, oneshot::Sender<Srv::Response>>>>,
}

impl<Srv> Client<Srv>
where
    Srv: ServiceT,
{
    pub(crate) fn new(node: &Node, service_name: &str, qos: &QoSProfile) -> Result<Self> {
        let handle = Arc::new(RclClient::new::<Srv>(
            node.clone_handle(),
            service_name,
            qos,
        )?);

        let pendings = Arc::new(Mutex::new(
            HashMap::<i64, oneshot::Sender<Srv::Response>>::new(),
        ));

        let callback = {
            let pendings = Arc::clone(&pendings);

            move |(req_header, res): (rcl_sys::rmw_request_id_t, ServiceResponseRaw<Srv>)| {
                pendings
                    .lock()
                    .unwrap()
                    .remove(&req_header.sequence_number)
                    .unwrap_or_else(|| panic!("fail to find key in Client::process_requests"))
                    .send(unsafe { res.to_rust() })
                    .unwrap_or_else(|_| {
                        panic!("fail to send response via channel in Client::process_requests")
                    });
            }
        };

        Ok(Self {
            handle,
            worker: ReceiveWorker::new(callback),
            pendings,
        })
    }

    /// Get the service name.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// # use anyhow::Result;
    /// # use rclrust::qos::QoSProfile;
    /// # use rclrust_msg::std_srvs::srv::{Empty, Empty_Request};
    /// #
    /// # #[tokio::main]
    /// # async fn main() -> Result<()> {
    /// # let ctx = rclrust::init()?;
    /// # let mut node = ctx.create_node("node")?;
    /// let mut client = node.create_client::<Empty>("service", &QoSProfile::default())?;
    /// let res = client.send_request(&Empty_Request {}).await?;
    /// # Ok(())
    /// # }
    /// ```
    pub async fn send_request(&mut self, request: &Srv::Request) -> Result<Srv::Response> {
        let id = self.handle.send_request::<Srv>(request)?;
        let (tx, rx) = oneshot::channel::<Srv::Response>();
        self.pendings.lock().unwrap().insert(id, tx);

        Ok(rx.await?)
    }

    /// Get the service name.
    ///
    /// # Examples
    ///
    /// ```
    /// # use anyhow::Result;
    /// # use rclrust::qos::QoSProfile;
    /// # use rclrust_msg::std_srvs::srv::Empty;
    /// #
    /// # #[tokio::main]
    /// # async fn main() -> Result<()> {
    /// # let ctx = rclrust::init()?;
    /// # let mut node = ctx.create_node("node")?;
    /// let client = node.create_client::<Empty>("service", &QoSProfile::default())?;
    /// assert_eq!(client.service_name().unwrap(), "/service");
    /// # Ok(())
    /// # }
    /// ```
    pub fn service_name(&self) -> Option<String> {
        self.handle.service_name()
    }

    /// Get the service name.
    ///
    /// # Examples
    ///
    /// ```
    /// # use anyhow::Result;
    /// # use rclrust::qos::QoSProfile;
    /// # use rclrust_msg::std_srvs::srv::Empty;
    /// #
    /// # #[tokio::main]
    /// # async fn main() -> Result<()> {
    /// # let ctx = rclrust::init()?;
    /// # let mut node = ctx.create_node("node")?;
    /// let client = node.create_client::<Empty>("service", &QoSProfile::default())?;
    /// assert!(client.is_valid());
    /// # Ok(())
    /// # }
    /// ```
    pub fn is_valid(&self) -> bool {
        self.handle.is_valid()
    }

    /// Get the service name.
    ///
    /// # Examples
    ///
    /// ```
    /// # use anyhow::Result;
    /// # use rclrust::qos::QoSProfile;
    /// # use rclrust_msg::std_srvs::srv::Empty;
    /// #
    /// # #[tokio::main]
    /// # async fn main() -> Result<()> {
    /// # let ctx = rclrust::init()?;
    /// # let mut node = ctx.create_node("node")?;
    /// let client = node.create_client::<Empty>("service", &QoSProfile::default())?;
    /// println!("{}", client.service_is_available()?);
    /// # Ok(())
    /// # }
    /// ```
    pub fn service_is_available(&self) -> Result<bool> {
        self.handle.service_is_available()
    }

    /// Get the service name.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// # use anyhow::Result;
    /// # use rclrust::qos::QoSProfile;
    /// # use rclrust_msg::std_srvs::srv::Empty;
    /// #
    /// # #[tokio::main]
    /// # async fn main() -> Result<()> {
    /// # let ctx = rclrust::init()?;
    /// # let mut node = ctx.create_node("node")?;
    /// let client = node.create_client::<Empty>("service", &QoSProfile::default())?;
    /// client.wait_service()?;
    /// # Ok(())
    /// # }
    /// ```
    pub fn wait_service(&self) -> Result<()> {
        while !self.service_is_available()? {
            thread::sleep(Duration::from_millis(1));
        }
        Ok(())
    }

    pub(crate) fn create_invoker(&self) -> ClientInvoker<Srv> {
        ClientInvoker::new_from_target(self)
    }

    pub(crate) fn clone_handle(&self) -> Arc<RclClient> {
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
    async fn service_req_res() -> Result<()> {
        use rclrust_msg::example_interfaces::srv::{
            AddTwoInts, AddTwoInts_Request, AddTwoInts_Response,
        };

        let ctx = crate::init()?;
        let mut service_node = ctx.create_node(&random_name())?;

        let service_name = random_name();

        let _srv = service_node.create_service::<AddTwoInts, _>(
            &service_name,
            |req| AddTwoInts_Response { sum: req.a + req.b },
            &QoSProfile::default(),
        )?;

        let mut client_node = ctx.create_node(&random_name())?;
        let mut client =
            client_node.create_client::<AddTwoInts>(&service_name, &QoSProfile::default())?;
        client.wait_service()?;
        assert_eq!(
            client
                .send_request(&AddTwoInts_Request { a: 15, b: 27 })
                .await?
                .sum,
            42
        );

        Ok(())
    }

    #[tokio::test]
    async fn client_service_name() -> Result<()> {
        let ctx = crate::init()?;
        let mut node = ctx.create_node(&random_name())?;
        let client = node.create_client::<Empty>("service", &QoSProfile::default())?;
        assert_eq!(client.service_name().unwrap(), "/service");

        Ok(())
    }

    #[tokio::test]
    async fn client_is_valid() -> Result<()> {
        let ctx = crate::init()?;
        let mut node = ctx.create_node(&random_name())?;
        let client = node.create_client::<Empty>("service", &QoSProfile::default())?;
        assert!(client.is_valid());

        Ok(())
    }

    #[tokio::test]
    async fn client_service_is_not_available() -> Result<()> {
        let ctx = crate::init()?;
        let mut node = ctx.create_node(&random_name())?;
        let client = node.create_client::<Empty>("service", &QoSProfile::default())?;
        assert!(!client.service_is_available()?);

        Ok(())
    }

    #[tokio::test]
    async fn client_wait_service() -> Result<()> {
        let ctx = crate::init()?;
        let mut node = ctx.create_node(&random_name())?;
        let client = node.create_client::<Empty>("service", &QoSProfile::default())?;

        let _service = node.create_service::<Empty, _>(
            "service",
            |_req| Empty_Response {},
            &QoSProfile::default(),
        )?;

        client.wait_service()?;
        assert!(client.service_is_available()?);

        Ok(())
    }
}
