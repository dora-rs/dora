//! Explicit local peer connections for externally started dynamic nodes.
//!
//! Configuration requests run on the daemon's serial event loop. Publish the
//! assigned listener in the plan before replying, so concurrent joiners cannot
//! both miss one another. A dynamic node dials producers *and* consumers: a
//! running static node cannot change its original dial list, but the transport
//! established by the joiner carries traffic in both directions.

use std::{collections::BTreeSet, sync::Arc};

use dora_core::topics::reserve_loopback_zenoh_endpoint;
use dora_message::{
    config::InputMapping, daemon_to_node::NodeConfig, dynamic_node::DynamicNodePeering, id::NodeId,
};
use eyre::{Context, ContextCompat, bail};

use crate::{RunningDataflow, spawn::NodeZenohPeering};

impl RunningDataflow {
    pub(crate) fn dynamic_node_config(
        &mut self,
        node_id: &NodeId,
        daemon_endpoint: Option<&str>,
    ) -> eyre::Result<(NodeConfig, DynamicNodePeering)> {
        let config = self
            .running_nodes
            .get(node_id)
            .with_context(|| format!("no node with ID `{node_id}` in {}", self.id))?
            .node_config
            .clone();
        if !config.dynamic {
            bail!("node with ID `{node_id}` in {} is not dynamic", self.id);
        }
        if self.stop_sent {
            bail!("dataflow {} is stopping", self.id);
        }

        // A fresh listener per request: nothing holds a port while the node is
        // down, so reusing an earlier one could hand a restarted node a port
        // that another process has taken since. Stability is not needed for
        // connectivity, because every (re)joining node dials its neighbours.
        let mut reserved = None;
        for _ in 0..16 {
            let endpoint =
                reserve_loopback_zenoh_endpoint().context("reserve dynamic node zenoh listener")?;
            // Another joiner can have a plan without having bound its
            // listener yet. Do not assign that apparently free port.
            if !self
                .zenoh_peering
                .values()
                .any(|p| p.listen.contains(&endpoint))
            {
                reserved = Some(endpoint);
                break;
            }
        }
        let listen =
            reserved.context("could not reserve a distinct dynamic node zenoh listener")?;

        let mut neighbours = BTreeSet::new();
        for (consumer, running) in &self.running_nodes {
            for input in running.node_config.run_config.inputs.values() {
                if let InputMapping::User(mapping) = &input.mapping {
                    if consumer == node_id {
                        neighbours.insert(mapping.source.clone());
                    }
                    if &mapping.source == node_id {
                        neighbours.insert(consumer.clone());
                    }
                }
            }
        }
        neighbours.remove(node_id);
        let mut connect: BTreeSet<String> =
            daemon_endpoint.into_iter().map(str::to_owned).collect();
        for neighbour in neighbours {
            // Only local nodes have a plan here. Remote dynamic edges retain
            // daemon forwarding; loopback must never be advertised cross-host.
            if self.running_nodes.contains_key(&neighbour)
                && let Some(plan) = self.zenoh_peering.get(&neighbour)
                && let Some(endpoint) = plan.listen.first()
            {
                connect.insert(endpoint.clone());
            }
        }
        let connect: Vec<_> = connect.into_iter().collect();
        Arc::make_mut(&mut self.zenoh_peering).insert(
            node_id.clone(),
            NodeZenohPeering {
                listen: vec![listen.clone()],
                connect: connect.clone(),
                routable: false,
            },
        );
        Ok((config, DynamicNodePeering { listen, connect }))
    }
}
