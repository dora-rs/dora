use std::{
    sync::{Arc, Mutex},
    thread,
    time::Duration,
};

use anyhow::Result;
use derive_new::new;
use futures::channel::mpsc;

use crate::{
    client::ClientInvokerBase, context::RclContext, error::RclRustError,
    service::ServiceInvokerBase, subscription::SubscriptionInvokerBase, timer::TimerInvoker,
    wait_set::RclWaitSet,
};

#[derive(Debug)]
pub enum ExecutorMessage {
    Subscription(Box<dyn SubscriptionInvokerBase + Send>),
    Timer(TimerInvoker),
    Client(Box<dyn ClientInvokerBase + Send>),
    Service(Box<dyn ServiceInvokerBase + Send>),
    Terminate,
}

#[derive(new)]
pub struct Executor {
    context: Arc<Mutex<RclContext>>,
    rx: mpsc::Receiver<ExecutorMessage>,
    #[new(default)]
    subscriptions: Vec<Box<dyn SubscriptionInvokerBase + Send>>,
    #[new(default)]
    timers: Vec<TimerInvoker>,
    #[new(default)]
    clients: Vec<Box<dyn ClientInvokerBase + Send>>,
    #[new(default)]
    services: Vec<Box<dyn ServiceInvokerBase + Send>>,
}

impl Executor {
    pub fn spin(&mut self) -> Result<()> {
        let max_duration = Duration::from_millis(50);

        while self.context.lock().unwrap().is_valid() {
            loop {
                match self.rx.try_next() {
                    Ok(Some(ExecutorMessage::Subscription(v))) => self.subscriptions.push(v),
                    Ok(Some(ExecutorMessage::Timer(v))) => self.timers.push(v),
                    Ok(Some(ExecutorMessage::Client(v))) => self.clients.push(v),
                    Ok(Some(ExecutorMessage::Service(v))) => self.services.push(v),
                    Ok(Some(ExecutorMessage::Terminate)) => return Ok(()),
                    Ok(None) => return Ok(()),
                    Err(_) => break,
                }
            }

            if let Err(e) = self.spin_some(max_duration) {
                match e.downcast_ref::<RclRustError>() {
                    Some(RclRustError::RclTimeout(_)) => continue,
                    Some(RclRustError::RclWaitSetEmpty(_)) => {
                        thread::sleep(max_duration);
                        continue;
                    }
                    _ => return Err(e),
                }
            }
        }

        Ok(())
    }

    pub fn spin_some(&mut self, max_duration: Duration) -> Result<()> {
        let n_subscriptions = self.subscriptions.len();
        let n_timers = self.timers.len();
        let n_clients = self.clients.len();
        let n_services = self.services.len();

        let mut wait_set = RclWaitSet::new(
            &mut self.context.lock().unwrap(),
            n_subscriptions,
            0,
            n_timers,
            n_clients,
            n_services,
            0,
        )?;

        wait_set.clear()?;

        self.subscriptions
            .iter()
            .try_for_each(|subscription| wait_set.add_subscription(subscription.handle()))?;

        self.timers
            .iter()
            .try_for_each(|timer| wait_set.add_timer(&timer.handle.lock().unwrap()))?;

        self.clients
            .iter()
            .try_for_each(|client| wait_set.add_client(client.handle()))?;

        self.services
            .iter()
            .try_for_each(|service| wait_set.add_service(service.handle()))?;

        wait_set.wait(max_duration.as_nanos() as i64)?;

        self.subscriptions
            .iter_mut()
            .zip(wait_set.subscriptions_ready())
            .filter(|(_, ready)| *ready)
            .try_for_each(|(subscription, _)| subscription.invoke())?;

        self.timers
            .iter_mut()
            .zip(wait_set.timers_ready())
            .filter(|(_, ready)| *ready)
            .try_for_each(|(timer, _)| timer.invoke())?;

        self.clients
            .iter_mut()
            .zip(wait_set.clients_ready())
            .filter(|(_, ready)| *ready)
            .try_for_each(|(client, _)| client.invoke())?;

        self.services
            .iter_mut()
            .zip(wait_set.services_ready())
            .filter(|(_, ready)| *ready)
            .try_for_each(|(service, _)| service.invoke())?;

        Ok(())
    }
}
