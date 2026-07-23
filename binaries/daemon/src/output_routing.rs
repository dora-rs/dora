//! Startup-handshake routing: which consumers must ack a producer's startup
//! markers, and which outputs must stay on the daemon path.
//!
//! The daemon computes this from the **actual** placement of the dataflow's
//! nodes at spawn time (`spawn_nodes` — the set this daemon was told to run),
//! not from the descriptor's `deploy` section, which is scheduling *intent*
//! that label-based placement can resolve differently. The result is delivered
//! to each node via [`NodeConfig::output_routing`]
//! (`dora_message::daemon_to_node::NodeConfig`).
//!
//! Policy (see the `OutputRouting` docs for the consumer-side contract):
//! - a consumer under **another daemon** pins the output to the daemon path
//!   (`daemon_only`): the node-to-node zenoh mesh is same-machine only, and
//!   only daemon-path sends feed the inter-daemon forwarder (dora #2738).
//!   This holds for dynamic remote consumers too — they can only ever be
//!   reached through forwarding.
//! - a **local static** consumer becomes a required acker: the producer keeps
//!   the output on the lossless daemon path until this consumer's startup ack
//!   proves the direct zenoh route end-to-end.
//! - **local dynamic** consumers are neither: they join at arbitrary times (or
//!   never), so nothing may wait on them, and their pre-join messages are
//!   inherently out of scope.
//!
//! When cross-machine node-to-node zenoh lands, only this policy changes:
//! remote static consumers become required ackers instead of forcing
//! `daemon_only`, and a fully-acked output goes pure-zenoh across machines.

use std::collections::{BTreeMap, BTreeSet};

use dora_core::{config::InputMapping, descriptor::ResolvedNode};
use dora_message::{
    daemon_to_node::{OutputRouting, RequiredAcker},
    id::{DataId, NodeId},
};

use crate::{CoreNodeKindExt, node_inputs};

/// Computes the per-output routing for every producer in `local_nodes`, from
/// the full resolved node set of the dataflow.
///
/// Every declared output of a local producer gets an entry (a zero-consumer
/// output resolves to the default routing: no pin, no required ackers — the
/// producer may use the direct zenoh path immediately). Consumers of remote
/// producers are ignored here; the remote producer's own daemon sees those
/// consumers as non-local and pins the output on its side.
pub fn compute_output_routing(
    nodes: &BTreeMap<NodeId, ResolvedNode>,
    local_nodes: &BTreeSet<NodeId>,
) -> BTreeMap<NodeId, BTreeMap<DataId, OutputRouting>> {
    let mut routing: BTreeMap<NodeId, BTreeMap<DataId, OutputRouting>> = local_nodes
        .iter()
        .filter_map(|id| nodes.get(id))
        .map(|producer| {
            let outputs = producer
                .kind
                .run_config()
                .outputs
                .into_iter()
                .map(|output_id| (output_id, OutputRouting::default()))
                .collect();
            (producer.id.clone(), outputs)
        })
        .collect();

    for consumer in nodes.values() {
        let consumer_local = local_nodes.contains(&consumer.id);
        let consumer_dynamic = consumer.kind.dynamic();
        for (input_id, input) in node_inputs(consumer) {
            let InputMapping::User(mapping) = input.mapping else {
                continue;
            };
            // Producer not local (or not declared): not this daemon's call.
            let Some(outputs) = routing.get_mut(&mapping.source) else {
                continue;
            };
            // Input referencing an undeclared output: descriptor validation's
            // problem, not routing's.
            let Some(entry) = outputs.get_mut(&mapping.output) else {
                continue;
            };
            if !consumer_local {
                entry.daemon_only = true;
            } else if !consumer_dynamic {
                entry.required_ackers.insert(RequiredAcker {
                    node_id: consumer.id.clone(),
                    input_id,
                });
            }
        }
    }

    routing
}

#[cfg(test)]
mod tests {
    use super::*;
    use dora_core::descriptor::{Descriptor, DescriptorExt};

    fn routing_for(
        yaml: &str,
        local: &[&str],
    ) -> BTreeMap<NodeId, BTreeMap<DataId, OutputRouting>> {
        let descriptor: Descriptor = serde_yaml::from_str(yaml).expect("parse descriptor");
        let nodes = descriptor
            .resolve_aliases_and_set_defaults()
            .expect("resolve descriptor");
        let local_nodes: BTreeSet<NodeId> = local
            .iter()
            .map(|id| NodeId::from(id.to_string()))
            .collect();
        compute_output_routing(&nodes, &local_nodes)
    }

    fn acker(node: &str, input: &str) -> RequiredAcker {
        RequiredAcker {
            node_id: NodeId::from(node.to_string()),
            input_id: DataId::from(input.to_string()),
        }
    }

    fn output<'a>(
        routing: &'a BTreeMap<NodeId, BTreeMap<DataId, OutputRouting>>,
        node: &str,
        output: &str,
    ) -> &'a OutputRouting {
        routing
            .get(&NodeId::from(node.to_string()))
            .unwrap_or_else(|| panic!("no routing for node {node}"))
            .get(&DataId::from(output.to_string()))
            .unwrap_or_else(|| panic!("no routing for output {node}/{output}"))
    }

    const CHAIN: &str = r#"
nodes:
  - id: source
    path: ./source
    outputs:
      - image
      - unconsumed
  - id: static-sink
    path: ./sink
    inputs:
      camera: source/image
  - id: dynamic-sink
    path: dynamic
    inputs:
      camera: source/image
"#;

    #[test]
    fn local_static_consumer_is_a_required_acker() {
        let routing = routing_for(CHAIN, &["source", "static-sink", "dynamic-sink"]);
        let image = output(&routing, "source", "image");
        assert!(!image.daemon_only);
        assert_eq!(
            image.required_ackers,
            BTreeSet::from([acker("static-sink", "camera")])
        );
    }

    #[test]
    fn local_dynamic_consumer_is_neither_acker_nor_pin() {
        // Dynamic nodes join at arbitrary times and nothing may wait on them,
        // and a local dynamic consumer needs no daemon forwarding — so an
        // output consumed *only* by a local dynamic node is immediately
        // eligible for the direct zenoh path.
        let yaml = r#"
nodes:
  - id: source
    path: ./source
    outputs:
      - image
  - id: dynamic-sink
    path: dynamic
    inputs:
      camera: source/image
"#;
        let routing = routing_for(yaml, &["source", "dynamic-sink"]);
        let image = output(&routing, "source", "image");
        assert!(!image.daemon_only);
        assert!(image.required_ackers.is_empty());
    }

    #[test]
    fn zero_consumer_output_gets_default_routing() {
        // Present with no pin and no ackers: the producer may go direct-zenoh
        // immediately (late/debug subscribers can still attach).
        let routing = routing_for(CHAIN, &["source", "static-sink", "dynamic-sink"]);
        let unconsumed = output(&routing, "source", "unconsumed");
        assert_eq!(unconsumed, &OutputRouting::default());
    }

    #[test]
    fn remote_consumers_pin_daemon_only_static_or_dynamic() {
        // Remote *static* consumer pins.
        let routing = routing_for(CHAIN, &["source", "dynamic-sink"]);
        assert!(output(&routing, "source", "image").daemon_only);

        // Remote *dynamic* consumer pins too: a dynamic node on another daemon
        // can only ever be reached through inter-daemon forwarding, which only
        // daemon-path sends feed (#2738).
        let routing = routing_for(CHAIN, &["source", "static-sink"]);
        let image = output(&routing, "source", "image");
        assert!(image.daemon_only);
        // The local acker is still recorded accurately alongside the pin (the
        // producer ignores ackers on pinned outputs today; the cross-machine
        // follow-up flips this policy).
        assert_eq!(
            image.required_ackers,
            BTreeSet::from([acker("static-sink", "camera")])
        );
    }

    #[test]
    fn remote_producers_are_absent_from_the_result() {
        let routing = routing_for(CHAIN, &["static-sink"]);
        assert!(!routing.contains_key(&NodeId::from("source".to_string())));
        // The consumer has no outputs; it still gets an (empty) entry.
        assert_eq!(
            routing
                .get(&NodeId::from("static-sink".to_string()))
                .map(BTreeMap::len),
            Some(0)
        );
    }

    #[test]
    fn self_loop_consumer_acks_itself() {
        let yaml = r#"
nodes:
  - id: looper
    path: ./looper
    outputs:
      - state
    inputs:
      previous: looper/state
"#;
        let routing = routing_for(yaml, &["looper"]);
        assert_eq!(
            output(&routing, "looper", "state").required_ackers,
            BTreeSet::from([acker("looper", "previous")])
        );
    }

    #[test]
    fn runtime_operator_inputs_are_operator_prefixed() {
        // A runtime (operator) consumer's acks arrive under the
        // `operator_id/input_id` key that `runtime_node_inputs` assigns, so the
        // required-acker identity must use the same prefixing.
        let yaml = r#"
nodes:
  - id: source
    path: ./source
    outputs:
      - image
  - id: runtime-consumer
    operators:
      - id: op
        shared-library: ./op
        inputs:
          camera: source/image
"#;
        let routing = routing_for(yaml, &["source", "runtime-consumer"]);
        assert_eq!(
            output(&routing, "source", "image").required_ackers,
            BTreeSet::from([acker("runtime-consumer", "op/camera")])
        );
    }

    #[test]
    fn multiple_local_consumers_are_all_required() {
        let yaml = r#"
nodes:
  - id: source
    path: ./source
    outputs:
      - image
  - id: sink-a
    path: ./sink
    inputs:
      camera: source/image
  - id: sink-b
    path: ./sink
    inputs:
      cam: source/image
      cam2: source/image
"#;
        let routing = routing_for(yaml, &["source", "sink-a", "sink-b"]);
        assert_eq!(
            output(&routing, "source", "image").required_ackers,
            BTreeSet::from([
                acker("sink-a", "camera"),
                acker("sink-b", "cam"),
                acker("sink-b", "cam2"),
            ])
        );
    }
}
