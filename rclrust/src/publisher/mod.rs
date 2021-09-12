//! Publish topics

use std::marker::PhantomData;

use anyhow::Result;
use rclrust_msg::_core::MessageT;

use crate::{node::Node, qos::QoSProfile};

mod rcl_wrapper;
use rcl_wrapper::RclPublisher;

/// Publisher struct
pub struct Publisher<T>
where
    T: MessageT,
{
    handle: RclPublisher,
    _phantom: PhantomData<T>,
}

impl<T> Publisher<T>
where
    T: MessageT,
{
    pub(crate) fn new(node: &Node, topic_name: &str, qos: &QoSProfile) -> Result<Self> {
        let handle = RclPublisher::new::<T>(node.clone_handle(), topic_name, qos)?;

        Ok(Self {
            handle,
            _phantom: Default::default(),
        })
    }

    /// Publish a message.
    ///
    /// # Examples
    ///
    /// ```
    /// # use anyhow::Result;
    /// # use rclrust::qos::QoSProfile;
    /// use rclrust_msg::std_msgs::msg::Int32;
    ///
    /// # fn main() -> Result<()> {
    /// # let ctx = rclrust::init()?;
    /// # let node = ctx.create_node("node")?;
    /// let publisher = node.create_publisher::<Int32>("message", &QoSProfile::default())?;
    /// publisher.publish(&Int32 { data: 42 })?;
    /// # Ok(())
    /// # }
    /// ```
    pub fn publish(&self, message: &T) -> Result<()> {
        self.handle.publish(message)
    }

    /// Get the topic name which this publisher publishes on.
    ///
    /// # Examples
    ///
    /// ```
    /// # use anyhow::Result;
    /// # use rclrust::qos::QoSProfile;
    /// # use rclrust_msg::std_msgs::msg::Int32;
    /// #
    /// # fn main() -> Result<()> {
    /// # let ctx = rclrust::init()?;
    /// # let node = ctx.create_node("node")?;
    /// let publisher = node.create_publisher::<Int32>("message", &QoSProfile::default())?;
    /// assert_eq!(&publisher.topic_name().unwrap(), "/message");
    /// # Ok(())
    /// # }
    /// ```
    pub fn topic_name(&self) -> Option<String> {
        self.handle.topic_name()
    }

    /// Check if this publisher is valid or not. Normally, a return value should be `true`.
    ///
    /// # Examples
    ///
    /// ```
    /// # use anyhow::Result;
    /// # use rclrust::qos::QoSProfile;
    /// # use rclrust_msg::std_msgs::msg::Int32;
    /// #
    /// # fn main() -> Result<()> {
    /// # let ctx = rclrust::init()?;
    /// # let node = ctx.create_node("node")?;
    /// let publisher = node.create_publisher::<Int32>("message", &QoSProfile::default())?;
    /// assert!(publisher.is_valid());
    /// # Ok(())
    /// # }
    /// ```
    pub fn is_valid(&self) -> bool {
        self.handle.is_valid()
    }

    /// Get how many subscriber are subscribing the topic which this publisher publishes on.
    ///
    /// # Examples
    ///
    /// ```
    /// # use anyhow::Result;
    /// # use rclrust::qos::QoSProfile;
    /// # use rclrust_msg::std_msgs::msg::Int32;
    /// #
    /// # fn main() -> Result<()> {
    /// # let ctx = rclrust::init()?;
    /// # let node = ctx.create_node("node")?;
    /// let publisher = node.create_publisher::<Int32>("message", &QoSProfile::default())?;
    /// println!("{}", publisher.subscription_count()?);
    /// # Ok(())
    /// # }
    /// ```
    pub fn subscription_count(&self) -> Result<usize> {
        self.handle.subscription_count()
    }

    /// Get the actual QoS settings, after the defaults have been determined.
    ///
    /// # Examples
    ///
    /// ```
    /// # use anyhow::Result;
    /// # use rclrust::qos::QoSProfile;
    /// # use rclrust_msg::std_msgs::msg::Int32;
    /// #
    /// # fn main() -> Result<()> {
    /// # let ctx = rclrust::init()?;
    /// # let node = ctx.create_node("node")?;
    /// let publisher = node.create_publisher::<Int32>("message", &QoSProfile::default())?;
    /// println!("{:?}", publisher.actual_qos().unwrap());
    /// # Ok(())
    /// # }
    /// ```
    pub fn actual_qos(&self) -> Option<QoSProfile> {
        self.handle.actual_qos()
    }
}

#[cfg(test)]
mod test {
    use std::time::Duration;

    use rclrust_msg::std_msgs::msg::Int32;

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
    async fn publisher_publish() -> Result<()> {
        use std::sync::{
            atomic::{AtomicU32, Ordering},
            Arc,
        };

        let ctx = crate::init()?;
        let pub_node = ctx.create_node(&random_name())?;
        let mut sub_node = ctx.create_node(&random_name())?;

        let topic_name = random_name();

        let counter = Arc::new(AtomicU32::new(0));
        let _subscriber = {
            let counter = Arc::clone(&counter);
            sub_node.create_subscription(
                &topic_name,
                move |topic: Arc<Int32>| {
                    counter.fetch_add(1, Ordering::Relaxed);
                    assert_eq!(topic.data, 42);
                    println!("called");
                },
                &QoSProfile::default(),
            )?
        };

        let publisher = pub_node.create_publisher::<Int32>(&topic_name, &QoSProfile::default())?;
        publisher.publish(&Int32 { data: 42 })?;
        tokio::time::sleep(Duration::from_millis(100)).await;

        assert_eq!(counter.load(Ordering::Relaxed), 1);

        Ok(())
    }

    #[test]
    fn publisher_topic_name() -> Result<()> {
        let ctx = crate::init()?;
        let node = ctx.create_node(&random_name())?;
        let publisher = node.create_publisher::<Int32>("message", &QoSProfile::default())?;
        assert_eq!(publisher.topic_name().unwrap(), "/message");

        Ok(())
    }

    #[test]
    fn publisher_is_valid() -> Result<()> {
        let ctx = crate::init()?;
        let node = ctx.create_node(&random_name())?;
        let publisher = node.create_publisher::<Int32>("message", &QoSProfile::default())?;
        assert!(publisher.is_valid());

        Ok(())
    }

    #[tokio::test]
    async fn publisher_subscription_count() -> Result<()> {
        let ctx = crate::init()?;
        let mut node = ctx.create_node(&random_name())?;
        let publisher = node.create_publisher::<Int32>("message", &QoSProfile::default())?;

        assert_eq!(publisher.subscription_count()?, 0);

        let _sub =
            node.create_subscription::<Int32, _>("message", |_| (), &QoSProfile::default())?;

        assert_eq!(publisher.subscription_count()?, 1);

        Ok(())
    }

    #[test]
    fn publisher_actual_qos() -> Result<()> {
        use crate::qos::LivelinessPolicy;

        let ctx = crate::init()?;
        let node = ctx.create_node(&random_name())?;
        let qos = QoSProfile::sensor_data()
            .deadline(Duration::from_millis(5))
            .lifespan(Duration::from_millis(10))
            .liveliness(LivelinessPolicy::Automatic)
            .liveliness_lease_duration(Duration::from_millis(15));
        let publisher = node.create_publisher::<Int32>("message", &qos)?;

        assert_eq!(publisher.actual_qos().unwrap(), qos);

        Ok(())
    }
}
