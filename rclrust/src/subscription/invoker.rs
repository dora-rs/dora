use std::{fmt, sync::Arc};

use anyhow::Result;
use futures::channel::mpsc;
use rclrust_msg::_core::MessageT;

use super::{RclSubscription, Subscription};
use crate::{error::RclRustError, internal::worker::WorkerMessage, rclrust_debug, Logger};

pub trait SubscriptionInvokerBase: fmt::Debug {
    fn handle(&self) -> &RclSubscription;
    fn invoke(&mut self) -> Result<()>;
}

pub struct SubscriptionInvoker<T>
where
    T: MessageT,
{
    handle: Arc<RclSubscription>,
    tx: Option<mpsc::Sender<WorkerMessage<Arc<T::Raw>>>>,
}

impl<T> SubscriptionInvoker<T>
where
    T: MessageT,
{
    pub fn new_from_target(target: &Subscription<T>) -> Self {
        Self {
            handle: target.clone_handle(),
            tx: Some(target.clone_tx()),
        }
    }

    fn stop(&mut self) {
        self.tx.take();
    }
}

impl<T> fmt::Debug for SubscriptionInvoker<T>
where
    T: MessageT,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "SubscriptionInvoker {{{:?}}}", self.handle)
    }
}

impl<T> SubscriptionInvokerBase for SubscriptionInvoker<T>
where
    T: MessageT,
    T::Raw: 'static,
{
    fn handle(&self) -> &RclSubscription {
        &self.handle
    }

    fn invoke(&mut self) -> Result<()> {
        if let Some(ref mut tx) = self.tx {
            let msg = match self.handle.take::<T>() {
                Ok(v) => v,
                Err(e) => {
                    return if let Some(RclRustError::RclSubscriptionTakeFailed(_)) =
                        e.downcast_ref::<RclRustError>()
                    {
                        rclrust_debug!(
                            Logger::new("rclrust"),
                            "`rcl_wait()` indicate that message is ready, however which incorrect. I know this happens when I use Cyclone DDS."
                        );
                        Ok(())
                    } else {
                        Err(e)
                    };
                }
            };

            match tx.try_send(WorkerMessage::Message(Arc::new(msg))) {
                Ok(_) => (),
                Err(e) if e.is_disconnected() => self.stop(),
                Err(_) => {
                    return Err(RclRustError::MessageQueueIsFull {
                        type_: "Subscription",
                        name: self
                            .handle
                            .topic_name()
                            .expect("Subscription should be valid"),
                    }
                    .into())
                }
            }
        }

        Ok(())
    }
}
