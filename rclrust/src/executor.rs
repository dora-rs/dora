use std::sync::{Arc, Weak};
use std::time::Duration;

use anyhow::Result;

use crate::context::Context;
use crate::error::RclRustError;
use crate::node::Node;
use crate::wait_set::RclWaitSet;

pub fn spin<'ctx>(node: &Arc<Node<'ctx>>) -> Result<()> {
    let mut exec = SingleThreadExecutor::new(node.context_ref())?;
    exec.add_node(node);
    exec.spin()?;

    Ok(())
}

pub fn spin_some<'ctx>(node: &Arc<Node<'ctx>>) -> Result<()> {
    let mut exec = SingleThreadExecutor::new(node.context_ref())?;
    exec.add_node(node);
    exec.spin_some(Duration::new(0, 0))?;

    Ok(())
}

pub struct SingleThreadExecutor<'ctx> {
    context: &'ctx Context,
    nodes: Vec<Weak<Node<'ctx>>>,
}

impl<'ctx> SingleThreadExecutor<'ctx> {
    pub fn new(context: &'ctx Context) -> Result<Self> {
        Ok(Self {
            context,
            nodes: Vec::new(),
        })
    }

    pub fn spin(&self) -> Result<()> {
        while self.context.is_valid() {
            if let Err(e) = self.spin_some(Duration::from_nanos(500)) {
                match e.downcast_ref::<RclRustError>() {
                    Some(RclRustError::RclTimeout(_)) => continue,
                    _ => return Err(e),
                }
            }
        }

        Ok(())
    }

    pub fn spin_some(&self, max_duration: Duration) -> Result<()> {
        let (n_subscriptions, _, _, _, _, _) = self.nodes.iter().filter_map(|n| n.upgrade()).fold(
            (0, 0, 0, 0, 0, 0),
            |(subs, guards, timers, clients, services, events), node| {
                (
                    subs + node.subscriptions.lock().unwrap().len(),
                    guards,
                    timers,
                    clients,
                    services,
                    events,
                )
            },
        );

        let mut wait_set = RclWaitSet::new(
            &mut self.context.handle().lock().unwrap(),
            n_subscriptions,
            0,
            0,
            0,
            0,
            0,
        )?;

        wait_set.clear()?;

        for node in self.nodes.iter().filter_map(|n| n.upgrade()) {
            for subscription in node.subscriptions.lock().unwrap().iter() {
                if let Some(subscription) = subscription.upgrade() {
                    wait_set.add_subscription(&subscription.handle())?;
                }
            }
        }

        wait_set.wait(max_duration.as_nanos() as i64)?;

        for node in self.nodes.iter().filter_map(|n| n.upgrade()) {
            for subscription in node.subscriptions.lock().unwrap().iter() {
                if let Some(subscription) = subscription.upgrade() {
                    subscription.call_callback().unwrap();
                }
            }
        }

        Ok(())
    }

    fn add_node(&mut self, node: &Arc<Node<'ctx>>) {
        self.nodes.push(Arc::downgrade(node));
    }
}
