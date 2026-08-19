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
//! - a **static** consumer becomes a required acker, wherever it runs: the
//!   producer keeps the output on the lossless daemon path until this
//!   consumer's startup ack proves the direct zenoh route end-to-end. The
//!   producer gives that proof a bounded startup window and pins the output to
//!   the daemon path for the run if it does not arrive in time, so adding a
//!   required acker that is slow to answer costs the fast path rather than
//!   reordering a live topic (dora-rs/dora#2891).
//! - a **remote static** consumer only qualifies when its producer has an
//!   endpoint that consumer's machine can dial — `routable_producers`, decided
//!   by `spawn::reserve_node_listeners`. Without one there is no route to
//!   prove, so waiting for an ack that cannot come would spend a whole startup
//!   window before falling back to where the output belonged all along:
//!   `daemon_only`.
//! - a consumer under another daemon that is **dynamic** pins the output to the
//!   daemon path unconditionally: it joins at an arbitrary time, so no endpoint
//!   can be planned for it and only forwarding can ever reach it.
//! - **local dynamic** consumers are neither: they join at arbitrary times (or
//!   never), so nothing may wait on them, and their pre-join messages are
//!   inherently out of scope.
//!
//! A remote static consumer that acks takes its edge off the daemon path
//! entirely — producer node to consumer node, one zenoh hop, no daemon on
//! either side of the wire. Note that `daemon_only` remains sticky per
//! *output*, not per consumer: an output with a remote dynamic consumer stays
//! pinned for all of them, because only daemon-path sends feed the inter-daemon
//! forwarder (dora #2738).

use std::collections::{BTreeMap, BTreeSet, HashMap};

use dora_core::{config::InputMapping, descriptor::ResolvedNode};
use dora_message::{
    daemon_to_node::{OutputRouting, RequiredAcker},
    id::{DataId, NodeId},
};

use crate::{CoreNodeKindExt, OutputId, node_inputs};

/// Computes the per-output routing for every producer in `local_nodes`, from
/// the full resolved node set of the dataflow.
///
/// Every declared output of a local producer gets an entry (a zero-consumer
/// output resolves to the default routing: no pin, no required ackers — the
/// producer may use the direct zenoh path immediately). Consumers of remote
/// producers are ignored here; the remote producer's own daemon decides those.
///
/// `routable_producers` are the local nodes that were given an endpoint
/// reachable from other machines, which is what makes a *remote* consumer's
/// direct route possible at all.
pub fn compute_output_routing(
    nodes: &BTreeMap<NodeId, ResolvedNode>,
    local_nodes: &BTreeSet<NodeId>,
    routable_producers: &BTreeSet<NodeId>,
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
            // A liveness deadline on a *remote* input is only fed by the
            // daemon path (see the `daemon_only` reasoning below).
            let watches_liveness = input.input_timeout.is_some();
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
            // A dynamic consumer is never an acker — nothing may wait on a node
            // that may never join — and a *remote* one additionally pins the
            // output, since forwarding is the only way to reach it.
            if consumer_dynamic {
                if !consumer_local {
                    entry.daemon_only = true;
                }
            } else if consumer_local
                || (routable_producers.contains(&mapping.source) && !watches_liveness)
            {
                entry.required_ackers.insert(RequiredAcker {
                    node_id: consumer.id.clone(),
                    input_id,
                });
            } else {
                // Two reasons to pin a remote static consumer's output.
                //
                // No dialable producer endpoint: there is no direct route to
                // prove, so pinning beats spending a startup window waiting for
                // an ack that cannot arrive.
                //
                // Or the consumer declares an `input_timeout`: its deadline is
                // armed unfired and refreshed only when its *own* daemon sees
                // the message — either delivering it (`send_output_to_local_
                // receivers`) or being told about it by a local producer
                // (`note_output_sent_to_local_receivers`, which walks local
                // mappings only). A direct cross-machine send reaches neither,
                // so `last_received` would stay `None`, the deadline would
                // never fire, and `input_timeout` — plus the circuit breaker
                // built on it — would silently stop working on exactly the
                // edges most likely to need it. The fast path is not worth a
                // liveness guarantee the descriptor asked for.
                entry.daemon_only = true;
            }
        }
    }

    routing
}

/// Routing for a node added to a *running* dataflow (`dora node add`),
/// computed from the live dataflow state rather than the (stale) descriptor.
///
/// Receivers exist at add time only when a node of the same id ran before
/// (remove + re-add) — their subscribers and ack publishers are still alive,
/// so the ordinary handshake proves those routes again. An output with **no**
/// current receivers is pinned to the daemon path: its future consumers can
/// only be wired via `dora node connect` (`AddMapping`), and connect-edges
/// deliver solely on the daemon path — a direct-zenoh output would starve
/// them (the consumer has no zenoh subscriber for a source it didn't declare).
pub fn added_node_output_routing(
    node_id: &NodeId,
    outputs: BTreeSet<DataId>,
    mappings: &HashMap<OutputId, BTreeSet<(NodeId, DataId)>>,
    open_external_mappings: &BTreeSet<OutputId>,
    dynamic_nodes: &BTreeSet<NodeId>,
) -> BTreeMap<DataId, OutputRouting> {
    outputs
        .into_iter()
        .map(|output_id| {
            let output = OutputId(node_id.clone(), output_id.clone());
            let mut routing = OutputRouting {
                daemon_only: open_external_mappings.contains(&output),
                ..Default::default()
            };
            match mappings
                .get(&output)
                .filter(|receivers| !receivers.is_empty())
            {
                Some(receivers) => {
                    for (receiver, input_id) in receivers {
                        if !dynamic_nodes.contains(receiver) {
                            routing.required_ackers.insert(RequiredAcker {
                                node_id: receiver.clone(),
                                input_id: input_id.clone(),
                            });
                        }
                    }
                }
                None => {
                    routing.daemon_only = true;
                }
            }
            (output_id, routing)
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use dora_core::descriptor::{Descriptor, DescriptorExt};

    /// Routing where every local producer is dialable from other machines —
    /// the state after a successful endpoint exchange.
    fn routing_for(
        yaml: &str,
        local: &[&str],
    ) -> BTreeMap<NodeId, BTreeMap<DataId, OutputRouting>> {
        routing_with_routable(yaml, local, local)
    }

    fn routing_with_routable(
        yaml: &str,
        local: &[&str],
        routable: &[&str],
    ) -> BTreeMap<NodeId, BTreeMap<DataId, OutputRouting>> {
        let descriptor: Descriptor = serde_yaml::from_str(yaml).expect("parse descriptor");
        let nodes = descriptor
            .resolve_aliases_and_set_defaults()
            .expect("resolve descriptor");
        let local_nodes: BTreeSet<NodeId> = local
            .iter()
            .map(|id| NodeId::from(id.to_string()))
            .collect();
        let routable_producers: BTreeSet<NodeId> = routable
            .iter()
            .map(|id| NodeId::from(id.to_string()))
            .collect();
        compute_output_routing(&nodes, &local_nodes, &routable_producers)
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
    fn a_remote_static_consumer_acks_instead_of_pinning() {
        // `static-sink` runs under another daemon and `source` is dialable from
        // there, so the edge has a direct route to prove — the whole point of
        // the cross-machine node mesh. Until it is proven the producer stays on
        // the daemon path, so this is a fast path won, never a message lost.
        let routing = routing_with_routable(CHAIN, &["source", "dynamic-sink"], &["source"]);
        let image = output(&routing, "source", "image");
        assert!(!image.daemon_only);
        assert_eq!(
            image.required_ackers,
            BTreeSet::from([acker("static-sink", "camera")])
        );
    }

    #[test]
    fn a_remote_static_consumer_pins_when_its_producer_is_undialable() {
        // No routable endpoint for `source` — a single-machine bind, or an
        // endpoint exchange that timed out. There is no route to prove, and
        // waiting for an ack that cannot arrive would burn a whole startup
        // window before landing on the daemon path anyway.
        let routing = routing_with_routable(CHAIN, &["source", "dynamic-sink"], &[]);
        let image = output(&routing, "source", "image");
        assert!(image.daemon_only);
        assert!(image.required_ackers.is_empty());
    }

    /// An `input_timeout` on a remote input is refreshed only when the
    /// consumer's *own* daemon sees the message, and a direct cross-machine
    /// send bypasses it. Taking the fast path would leave the deadline armed
    /// but unfired forever — the descriptor asked for liveness detection, so
    /// the daemon path wins.
    #[test]
    fn a_remote_consumer_watching_liveness_keeps_the_daemon_path() {
        let yaml = r#"
nodes:
  - id: source
    path: ./source
    outputs:
      - image
  - id: watchful-sink
    path: ./sink
    inputs:
      camera:
        source: source/image
        input_timeout: 5
"#;
        let routing = routing_with_routable(yaml, &["source"], &["source"]);
        let image = output(&routing, "source", "image");
        assert!(image.daemon_only);
        assert!(image.required_ackers.is_empty());

        // The same consumer on this machine is unaffected: its daemon is the
        // one delivering (or being notified of) every message either way.
        let routing = routing_with_routable(yaml, &["source", "watchful-sink"], &["source"]);
        let image = output(&routing, "source", "image");
        assert!(!image.daemon_only);
        assert_eq!(
            image.required_ackers,
            BTreeSet::from([acker("watchful-sink", "camera")])
        );
    }

    #[test]
    fn a_remote_dynamic_consumer_still_pins() {
        // A dynamic node on another daemon joins at an arbitrary time, so no
        // endpoint can be planned for it and only inter-daemon forwarding can
        // reach it — which only daemon-path sends feed (#2738).
        let routing = routing_with_routable(CHAIN, &["source", "static-sink"], &["source"]);
        let image = output(&routing, "source", "image");
        assert!(image.daemon_only);
        // The local acker is still recorded accurately alongside the pin.
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
    fn added_node_without_receivers_is_pinned_to_the_daemon_path() {
        // A freshly added id has no receivers; future consumers can only be
        // wired via `dora node connect`, which delivers on the daemon path
        // only — going direct-zenoh would starve them.
        let node = NodeId::from("added".to_string());
        let routing = added_node_output_routing(
            &node,
            BTreeSet::from([DataId::from("out".to_string())]),
            &HashMap::new(),
            &BTreeSet::new(),
            &BTreeSet::new(),
        );
        let out = routing.get(&DataId::from("out".to_string())).unwrap();
        assert!(out.daemon_only);
        assert!(out.required_ackers.is_empty());
    }

    #[test]
    fn readded_node_requires_acks_from_its_existing_static_receivers() {
        // remove + re-add: the old consumers' subscribers and ack publishers
        // are still alive, so the ordinary handshake proves the routes again.
        let node = NodeId::from("sender".to_string());
        let out_id = DataId::from("value".to_string());
        let mappings = HashMap::from([(
            OutputId(node.clone(), out_id.clone()),
            BTreeSet::from([
                (
                    NodeId::from("receiver".to_string()),
                    DataId::from("value".to_string()),
                ),
                (
                    NodeId::from("dyn-sink".to_string()),
                    DataId::from("value".to_string()),
                ),
            ]),
        )]);
        let dynamic_nodes = BTreeSet::from([NodeId::from("dyn-sink".to_string())]);
        let routing = added_node_output_routing(
            &node,
            BTreeSet::from([out_id.clone()]),
            &mappings,
            &BTreeSet::new(),
            &dynamic_nodes,
        );
        let out = routing.get(&out_id).unwrap();
        assert!(!out.daemon_only);
        // The static receiver must ack; the dynamic one must not be required.
        assert_eq!(
            out.required_ackers,
            BTreeSet::from([RequiredAcker {
                node_id: NodeId::from("receiver".to_string()),
                input_id: DataId::from("value".to_string()),
            }])
        );
    }

    #[test]
    fn readded_node_with_remote_receiver_is_pinned() {
        let node = NodeId::from("sender".to_string());
        let out_id = DataId::from("value".to_string());
        let output = OutputId(node.clone(), out_id.clone());
        let mappings = HashMap::from([(
            output.clone(),
            BTreeSet::from([(
                NodeId::from("local-receiver".to_string()),
                DataId::from("value".to_string()),
            )]),
        )]);
        let routing = added_node_output_routing(
            &node,
            BTreeSet::from([out_id.clone()]),
            &mappings,
            &BTreeSet::from([output]),
            &BTreeSet::new(),
        );
        assert!(routing.get(&out_id).unwrap().daemon_only);
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
