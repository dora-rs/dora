use dora_message::{
    config::{Input, InputMapping, NodeRunConfig},
    descriptor::{EnvValue, GitRepoRev, NodeSource},
    id::{DataId, NodeId, OperatorId},
};
use eyre::{Context, OptionExt, Result, bail};
use std::{
    collections::{BTreeMap, HashMap},
    env::consts::EXE_EXTENSION,
    path::{Path, PathBuf},
    process::Command,
};
use tracing::warn;

// reexport for compatibility
pub use dora_message::descriptor::{
    CoreNodeKind, CustomNode, DYNAMIC_SOURCE, Descriptor, Node, OperatorConfig, OperatorDefinition,
    OperatorSource, PythonSource, ResolvedNode, RestartPolicy, RmwZenohCompatibility,
    Ros2BridgeConfig, Ros2Direction, Ros2QosConfig, Ros2TopicConfig, Ros2TransportConfig,
    RuntimeNode, SHELL_SOURCE, SingleOperatorDefinition,
};
pub use validate::ResolvedNodeExt;
pub use visualize::collect_dora_timers;

mod expand;
pub mod validate;
mod visualize;

pub use expand::{
    ExpandedDescriptor, ModuleBoundaries, check_module_file, expand_modules,
    expand_modules_with_boundaries,
};

pub trait DescriptorExt {
    fn resolve_aliases_and_set_defaults(&self) -> eyre::Result<BTreeMap<NodeId, ResolvedNode>>;
    fn visualize_as_mermaid_with_boundaries(
        &self,
        boundaries: &ModuleBoundaries,
    ) -> eyre::Result<String>;
    /// Apply a command-line override to the dataflow's completion policy.
    ///
    /// `None` leaves the descriptor's own `exit_when_nodes_finish`
    /// alone, so a setting written in the YAML stands; `Some(v)`
    /// overrides it in either direction, so `--exit-when-nodes-finish`
    /// can force the policy on and `=false` can force it off for a
    /// descriptor that asks for it (dora-rs/dora#2920).
    ///
    /// Shared rather than spelled out at each call site: `dora run`,
    /// `dora start` and the daemon all have to agree, and three copies
    /// of the same three lines is how one of them ends up not being
    /// updated.
    fn apply_exit_when_nodes_finish(&mut self, over: Option<bool>);

    fn blocking_read(path: &Path) -> eyre::Result<Descriptor>;
    fn parse(buf: Vec<u8>) -> eyre::Result<Descriptor>;
    fn check(&self, working_dir: &Path) -> eyre::Result<()>;
    /// Expand all module references into flat nodes.
    ///
    /// Module nodes are replaced by the inner nodes defined in their module
    /// file. Internal IDs are prefixed with `{module_id}.` and input/output
    /// wiring is rewritten accordingly.
    fn expand(&self, working_dir: &Path) -> eyre::Result<Descriptor>;
    /// Like [`expand`](Self::expand) but also returns module boundary metadata
    /// for visualization.
    fn expand_with_boundaries(
        &self,
        working_dir: &Path,
    ) -> eyre::Result<(Descriptor, ModuleBoundaries)>;
}

pub const SINGLE_OPERATOR_DEFAULT_ID: &str = "op";

/// Prefixes a single-operator node's output id with the operator id
/// (e.g. `result` -> `op/result`) so downstream references resolve to the
/// operator-qualified output.
///
/// Operator ids are *not* validated against the `DataId` character set when a
/// descriptor is parsed (`OperatorId` stores its string verbatim), so build the
/// qualified id fallibly: `DataId::from` panics on characters outside
/// `[a-zA-Z0-9_./-]`, which would abort `dora check`/`graph`/`build` on an
/// otherwise-parseable descriptor. Returning an `Err` surfaces it as a clean
/// descriptor error instead.
fn prefix_output_with_operator_id(op_name: &OperatorId, output: &DataId) -> eyre::Result<DataId> {
    format!("{op_name}/{output}")
        .parse::<DataId>()
        .map_err(|e| {
            eyre::eyre!(
                "operator id `{op_name}` produces an invalid output id `{op_name}/{output}`: {e}"
            )
        })
}

/// Like [`DescriptorExt::resolve_aliases_and_set_defaults`], but resolves
/// `desc` as a node (or nodes) being added to an already-running dataflow
/// whose current node set is `topology_nodes`.
///
/// Whole-descriptor resolution rewrites an input that references a
/// single-`operator:` producer from the bare output name to the
/// operator-qualified one (`result` -> `op/result`). The dynamic-topology
/// `AddNode` path resolves the new node in isolation, so that producer is not
/// present in the descriptor being resolved and the rewrite is skipped —
/// leaving the added node subscribed to an output name nobody publishes
/// (silent data loss, #2877). Supplying the surrounding `topology_nodes` makes
/// those producers visible so the prefixing is applied.
///
/// `topology_nodes` contribute *only* to the single-operator output-prefixing
/// lookup — they are never themselves resolved or emitted, and nothing else on
/// the surrounding descriptor (`env`, `deploy`, …) is consulted. Callers that
/// want the running dataflow's `env` merged in must set it on `desc` (the
/// `AddNode` handler does, see #2919). `desc`'s own nodes take precedence in
/// the lookup, so a node id present in both resolves against the copy being
/// added.
pub fn resolve_aliases_and_set_defaults_in_topology(
    desc: &Descriptor,
    topology_nodes: &[Node],
) -> eyre::Result<BTreeMap<NodeId, ResolvedNode>> {
    let default_op_id = OperatorId::from(SINGLE_OPERATOR_DEFAULT_ID.to_string());

    let single_operator_nodes: HashMap<_, _> = topology_nodes
        .iter()
        .chain(desc.nodes.iter())
        .filter_map(|n| {
            n.operator
                .as_ref()
                .map(|op| (&n.id, op.id.as_ref().unwrap_or(&default_op_id)))
        })
        .collect();

    /// Check node-level fields that are silently dropped during resolution
    /// for a given node kind. Rejects fields that do not belong to the kind
    /// determined by `Node::kind()`.
    fn validate_node_fields_for_kind(node: &Node) -> eyre::Result<()> {
        let kind = node.kind()?;
        let mut conflicts = Vec::new();

        match kind {
            NodeKind::Standard(_) => {
                if node.hub.is_some() {
                    conflicts.push("hub");
                }
            }
            NodeKind::Operator(_) | NodeKind::Runtime(_) => {
                if node.path.is_some() {
                    conflicts.push("path");
                }
                if node.path_sha256.is_some() {
                    conflicts.push("path_sha256");
                }
                if node.git.is_some() {
                    conflicts.push("git");
                }
                if node.hub.is_some() {
                    conflicts.push("hub");
                }
                if node.branch.is_some() {
                    conflicts.push("branch");
                }
                if node.tag.is_some() {
                    conflicts.push("tag");
                }
                if node.rev.is_some() {
                    conflicts.push("rev");
                }
                if node.build.is_some() {
                    conflicts.push("build");
                }
                if node.args.is_some() {
                    conflicts.push("args");
                }
                if node.send_stdout_as.is_some() {
                    conflicts.push("send_stdout_as");
                }
                if node.send_logs_as.is_some() {
                    conflicts.push("send_logs_as");
                }
                if node.min_log_level.is_some() {
                    conflicts.push("min_log_level");
                }
                if node.max_log_size.is_some() {
                    conflicts.push("max_log_size");
                }
                if node.max_rotated_files.is_some() {
                    conflicts.push("max_rotated_files");
                }
                if !node.outputs.is_empty() {
                    conflicts.push("outputs");
                }
                if !node.output_types.is_empty() {
                    conflicts.push("output_types");
                }
                if !node.input_types.is_empty() {
                    conflicts.push("input_types");
                }
                if !node.output_framing.is_empty() {
                    conflicts.push("output_framing");
                }
                if !node.output_metadata.is_empty() {
                    conflicts.push("output_metadata");
                }
                if node.pattern.is_some() {
                    conflicts.push("pattern");
                }
                if !matches!(node.restart_policy, RestartPolicy::Never) {
                    conflicts.push("restart_policy");
                }
                if node.max_restarts != 0 {
                    conflicts.push("max_restarts");
                }
                if node.restart_delay.is_some() {
                    conflicts.push("restart_delay");
                }
                if node.max_restart_delay.is_some() {
                    conflicts.push("max_restart_delay");
                }
                if node.restart_window.is_some() {
                    conflicts.push("restart_window");
                }
                if node.health_check_timeout.is_some() {
                    conflicts.push("health_check_timeout");
                }
                if node.finish_grace_secs.is_some() {
                    conflicts.push("finish_grace_secs");
                }
                if node.shared_memory_pool_size.is_some() {
                    conflicts.push("shared_memory_pool_size");
                }
            }
            NodeKind::Custom(_) => {
                // Source-definition fields are mutually exclusive with
                // `custom.source`. The rest are merged by
                // `merge_node_level_fields_into_custom` and already reset.
                if node.git.is_some() {
                    conflicts.push("git");
                }
                if node.branch.is_some() {
                    conflicts.push("branch");
                }
                if node.tag.is_some() {
                    conflicts.push("tag");
                }
                if node.rev.is_some() {
                    conflicts.push("rev");
                }
                if node.hub.is_some() {
                    conflicts.push("hub");
                }
                // `pattern` and `output_metadata` have no counterpart in
                // `CustomNode` / `NodeRunConfig` — they must stay inside
                // `operator.config` or be used on a Standard node.
                if !node.output_metadata.is_empty() {
                    conflicts.push("output_metadata");
                }
                if node.pattern.is_some() {
                    conflicts.push("pattern");
                }
            }
            NodeKind::Module(_) => {
                eyre::bail!(
                    "module node `{}` must be expanded before resolution — call expand_modules() first",
                    node.id
                );
            }
            NodeKind::Ros2Bridge(_) => {}
        }

        if !conflicts.is_empty() {
            eyre::bail!(
                "node `{}` has fields that are not supported on its node kind: {}\n\
                 hint: these fields are silently dropped during resolution; \
                 move them into the appropriate sub-configuration",
                node.id,
                conflicts.join(", ")
            );
        }
        Ok(())
    }

    /// Merge Node-level fields into `CustomNode` for Custom-kind nodes.
    ///
    /// Standard nodes copy Node-level fields into their resolved `CustomNode`
    /// during resolution (see the `NodeKindMut::Standard` arm). Custom nodes
    /// previously skipped this step — when a user wrote fields like `outputs`
    /// or `restart_policy` outside the `custom:` block they landed in `Node.*`
    /// and were silently dropped (BUG-005).
    ///
    /// This function fills empty sub-structure fields from Node-level values,
    /// with sub-structure values winning on conflict (they're more specific).
    /// After the merge the Node-level fields are reset to their defaults so
    /// downstream validation can still flag truly incompatible fields (`git`,
    /// `hub`, etc.) without false-positives on now-merged fields.
    fn merge_node_level_fields_into_custom(node: &mut Node) {
        if let Some(ref mut custom) = node.custom {
            let rc = &mut custom.run_config;
            // Fields whose Node-level value was ignored because the
            // sub-structure already had a (non-default) value set.
            let mut shadowed = Vec::new();

            // ── run_config fields ──
            if rc.outputs.is_empty() && !node.outputs.is_empty() {
                rc.outputs = std::mem::take(&mut node.outputs);
            } else if !rc.outputs.is_empty() && !node.outputs.is_empty() {
                shadowed.push("outputs");
            }
            if rc.inputs.is_empty() && !node.inputs.is_empty() {
                rc.inputs = std::mem::take(&mut node.inputs);
            } else if !rc.inputs.is_empty() && !node.inputs.is_empty() {
                shadowed.push("inputs");
            }
            if rc.output_types.is_empty() && !node.output_types.is_empty() {
                rc.output_types = std::mem::take(&mut node.output_types);
            } else if !rc.output_types.is_empty() && !node.output_types.is_empty() {
                shadowed.push("output_types");
            }
            if rc.output_framing.is_empty() && !node.output_framing.is_empty() {
                rc.output_framing = std::mem::take(&mut node.output_framing);
            } else if !rc.output_framing.is_empty() && !node.output_framing.is_empty() {
                shadowed.push("output_framing");
            }
            if rc.input_types.is_empty() && !node.input_types.is_empty() {
                rc.input_types = std::mem::take(&mut node.input_types);
            } else if !rc.input_types.is_empty() && !node.input_types.is_empty() {
                shadowed.push("input_types");
            }
            if rc.shared_memory_pool_size.is_none() && node.shared_memory_pool_size.is_some() {
                rc.shared_memory_pool_size = node.shared_memory_pool_size.take();
            } else if rc.shared_memory_pool_size.is_some() && node.shared_memory_pool_size.is_some()
            {
                shadowed.push("shared_memory_pool_size");
            }

            // ── restart fields ──
            //
            // Note: `restart_policy` and `max_restarts` are non-Option types
            // whose defaults (Never / 0) are indistinguishable from
            // "explicitly set to default". If custom already has a non-default
            // value we keep it; if both are set to the same non-default we
            // silently skip the merge (the Node-level value is dropped).
            // This is the same trade-off Standard nodes make when they always
            // copy Node-level fields.
            if matches!(custom.restart_policy, RestartPolicy::Never)
                && !matches!(node.restart_policy, RestartPolicy::Never)
            {
                custom.restart_policy =
                    std::mem::replace(&mut node.restart_policy, RestartPolicy::Never);
            }
            if custom.max_restarts == 0 && node.max_restarts != 0 {
                custom.max_restarts = std::mem::replace(&mut node.max_restarts, 0);
            }
            if custom.restart_delay.is_none() && node.restart_delay.is_some() {
                custom.restart_delay = node.restart_delay.take();
            } else if custom.restart_delay.is_some() && node.restart_delay.is_some() {
                shadowed.push("restart_delay");
            }
            if custom.max_restart_delay.is_none() && node.max_restart_delay.is_some() {
                custom.max_restart_delay = node.max_restart_delay.take();
            } else if custom.max_restart_delay.is_some() && node.max_restart_delay.is_some() {
                shadowed.push("max_restart_delay");
            }
            if custom.restart_window.is_none() && node.restart_window.is_some() {
                custom.restart_window = node.restart_window.take();
            } else if custom.restart_window.is_some() && node.restart_window.is_some() {
                shadowed.push("restart_window");
            }

            // ── runtime fields ──
            if custom.health_check_timeout.is_none() && node.health_check_timeout.is_some() {
                custom.health_check_timeout = node.health_check_timeout.take();
            } else if custom.health_check_timeout.is_some() && node.health_check_timeout.is_some() {
                shadowed.push("health_check_timeout");
            }
            if custom.finish_grace_secs.is_none() && node.finish_grace_secs.is_some() {
                custom.finish_grace_secs = node.finish_grace_secs.take();
            } else if custom.finish_grace_secs.is_some() && node.finish_grace_secs.is_some() {
                shadowed.push("finish_grace_secs");
            }

            // ── build / execution fields ──
            if custom.args.is_none() && node.args.is_some() {
                custom.args = node.args.take();
            } else if custom.args.is_some() && node.args.is_some() {
                shadowed.push("args");
            }
            if custom.build.is_none() && node.build.is_some() {
                custom.build = node.build.take();
            } else if custom.build.is_some() && node.build.is_some() {
                shadowed.push("build");
            }
            if custom.path_sha256.is_none() && node.path_sha256.is_some() {
                custom.path_sha256 = node.path_sha256.take();
            } else if custom.path_sha256.is_some() && node.path_sha256.is_some() {
                shadowed.push("path_sha256");
            }

            // ── logging fields ──
            if custom.send_stdout_as.is_none() && node.send_stdout_as.is_some() {
                custom.send_stdout_as = node.send_stdout_as.take();
            } else if custom.send_stdout_as.is_some() && node.send_stdout_as.is_some() {
                shadowed.push("send_stdout_as");
            }
            if custom.send_logs_as.is_none() && node.send_logs_as.is_some() {
                custom.send_logs_as = node.send_logs_as.take();
            } else if custom.send_logs_as.is_some() && node.send_logs_as.is_some() {
                shadowed.push("send_logs_as");
            }
            if custom.min_log_level.is_none() && node.min_log_level.is_some() {
                custom.min_log_level = node.min_log_level.take();
            } else if custom.min_log_level.is_some() && node.min_log_level.is_some() {
                shadowed.push("min_log_level");
            }
            if custom.max_log_size.is_none() && node.max_log_size.is_some() {
                custom.max_log_size = node.max_log_size.take();
            } else if custom.max_log_size.is_some() && node.max_log_size.is_some() {
                shadowed.push("max_log_size");
            }
            if custom.max_rotated_files.is_none() && node.max_rotated_files.is_some() {
                custom.max_rotated_files = node.max_rotated_files.take();
            } else if custom.max_rotated_files.is_some() && node.max_rotated_files.is_some() {
                shadowed.push("max_rotated_files");
            }

            if !shadowed.is_empty() {
                warn!(
                    "node `{}`: the following fields are set at both the node level \
                     and inside `custom:` — the `custom:` values take precedence: {}",
                    node.id,
                    shadowed.join(", ")
                );
            }
        }
    }

    let mut resolved = BTreeMap::new();
    for mut node in desc.nodes.clone() {
        // Merge Node-level fields into CustomNode. Standard nodes do this
        // during their →Custom conversion; Custom nodes previously skipped
        // it, causing these fields to be silently dropped (BUG-005).
        merge_node_level_fields_into_custom(&mut node);
        // adjust ROS2 bridge input mappings early (before node_kind borrows node)
        if node.ros2.is_some() {
            let mut ros2_conflicts = Vec::new();
            if node.build.is_some() {
                ros2_conflicts.push("build");
            }
            if node.path_sha256.is_some() {
                ros2_conflicts.push("path_sha256");
            }
            if !ros2_conflicts.is_empty() {
                eyre::bail!(
                    "node `{}` has `ros2` together with {}: these fields are not \
                     supported on ROS2 bridge nodes — the bridge binary is pre-built",
                    node.id,
                    ros2_conflicts.join(", ")
                );
            }

            for input in node.inputs.values_mut() {
                if let InputMapping::User(m) = &mut input.mapping
                    && let Some(op_name) = single_operator_nodes.get(&m.source).copied()
                {
                    m.output = prefix_output_with_operator_id(op_name, &m.output)?;
                }
            }
        }

        // adjust input mappings
        validate_node_fields_for_kind(&node)?;

        let mut node_kind = node_kind_mut(&mut node)?;
        let input_mappings: Vec<_> = match &mut node_kind {
            NodeKindMut::Standard { inputs, .. } => inputs.values_mut().collect(),
            NodeKindMut::Runtime(node) => node
                .operators
                .iter_mut()
                .flat_map(|op| op.config.inputs.values_mut())
                .collect(),
            NodeKindMut::Custom(node) => node.run_config.inputs.values_mut().collect(),
            NodeKindMut::Operator(operator) => operator.config.inputs.values_mut().collect(),
            NodeKindMut::Ros2Bridge(_) => vec![],
        };
        for mapping in input_mappings
            .into_iter()
            .filter_map(|i| match &mut i.mapping {
                InputMapping::Timer { .. } | InputMapping::Logs(_) => None,
                InputMapping::User(m) => Some(m),
            })
        {
            if let Some(op_name) = single_operator_nodes.get(&mapping.source).copied() {
                mapping.output = prefix_output_with_operator_id(op_name, &mapping.output)?;
            }
        }

        // resolve nodes
        let kind = match node_kind {
            NodeKindMut::Standard {
                path,
                source,
                inputs: _,
            } => CoreNodeKind::Custom(CustomNode {
                path: path.clone(),
                source,
                path_sha256: node.path_sha256,
                args: node.args,
                build: node.build,
                send_stdout_as: node.send_stdout_as,
                send_logs_as: node.send_logs_as,
                min_log_level: node.min_log_level,
                max_log_size: node.max_log_size,
                max_rotated_files: node.max_rotated_files,
                run_config: NodeRunConfig {
                    inputs: node.inputs,
                    outputs: node.outputs,
                    output_types: node.output_types,
                    output_framing: node.output_framing,
                    input_types: node.input_types,
                    shared_memory_pool_size: node.shared_memory_pool_size,
                },
                envs: None,
                restart_policy: node.restart_policy,
                max_restarts: node.max_restarts,
                restart_delay: node.restart_delay,
                max_restart_delay: node.max_restart_delay,
                restart_window: node.restart_window,
                health_check_timeout: node.health_check_timeout,
                finish_grace_secs: node.finish_grace_secs,
            }),
            NodeKindMut::Custom(node) => CoreNodeKind::Custom(node.clone()),
            NodeKindMut::Runtime(node) => CoreNodeKind::Runtime(node.clone()),
            NodeKindMut::Operator(op) => CoreNodeKind::Runtime(RuntimeNode {
                operators: vec![OperatorDefinition {
                    id: op.id.clone().unwrap_or_else(|| default_op_id.clone()),
                    config: op.config.clone(),
                }],
            }),
            NodeKindMut::Ros2Bridge(config) => {
                let bridge_config_json = serde_json::to_string(&config)
                    .context("failed to serialize ROS2 bridge config")?;

                let mut envs = BTreeMap::new();
                envs.insert(
                    "DORA_ROS2_BRIDGE_CONFIG".to_string(),
                    EnvValue::String(bridge_config_json),
                );

                CoreNodeKind::Custom(CustomNode {
                    path: "dora-ros2-bridge-node".to_string(),
                    source: NodeSource::Local,
                    path_sha256: None,
                    args: node.args,
                    build: None,
                    send_stdout_as: node.send_stdout_as,
                    send_logs_as: node.send_logs_as,
                    min_log_level: node.min_log_level,
                    max_log_size: node.max_log_size,
                    max_rotated_files: node.max_rotated_files,
                    run_config: NodeRunConfig {
                        inputs: node.inputs,
                        outputs: node.outputs,
                        output_types: node.output_types,
                        output_framing: node.output_framing,
                        input_types: node.input_types,
                        shared_memory_pool_size: node.shared_memory_pool_size,
                    },
                    envs: Some(envs),
                    restart_policy: node.restart_policy,
                    max_restarts: node.max_restarts,
                    restart_delay: node.restart_delay,
                    max_restart_delay: node.max_restart_delay,
                    restart_window: node.restart_window,
                    health_check_timeout: node.health_check_timeout,
                    finish_grace_secs: node.finish_grace_secs,
                })
            }
        };

        if resolved.contains_key(&node.id) {
            eyre::bail!(
                "duplicate node ID `{}` — each node must have a unique `id`",
                node.id
            );
        }
        resolved.insert(
            node.id.clone(),
            ResolvedNode {
                id: node.id,
                name: node.name,
                description: node.description,
                // Merge the dataflow-level `env` into the per-node `env`.
                // Per-node keys win on conflict so a node can override a
                // shared default (e.g. global `RUST_LOG=info` with one
                // verbose node setting `RUST_LOG=debug`).
                env: merge_env(desc.env.as_ref(), node.env),
                cpu_affinity: node.cpu_affinity,
                deploy: node.deploy,
                kind,
            },
        );
    }

    Ok(resolved)
}

impl DescriptorExt for Descriptor {
    fn resolve_aliases_and_set_defaults(&self) -> eyre::Result<BTreeMap<NodeId, ResolvedNode>> {
        resolve_aliases_and_set_defaults_in_topology(self, &[])
    }

    fn visualize_as_mermaid_with_boundaries(
        &self,
        boundaries: &ModuleBoundaries,
    ) -> eyre::Result<String> {
        let resolved = self.resolve_aliases_and_set_defaults()?;
        let flowchart = visualize::visualize_nodes_with_boundaries(&resolved, boundaries);
        Ok(flowchart)
    }

    fn apply_exit_when_nodes_finish(&mut self, over: Option<bool>) {
        if let Some(over) = over {
            self.exit_when_nodes_finish = Some(over);
        }
    }

    fn blocking_read(path: &Path) -> eyre::Result<Descriptor> {
        let buf = std::fs::read(path).context("failed to open given file")?;
        Descriptor::parse(buf)
    }

    fn parse(buf: Vec<u8>) -> eyre::Result<Descriptor> {
        serde_yaml::from_slice(&buf).context("failed to parse given descriptor")
    }

    fn check(&self, working_dir: &Path) -> eyre::Result<()> {
        let expanded = self.expand(working_dir)?;
        validate::check_dataflow(&expanded, working_dir)
            .wrap_err("Dataflow could not be validated.")
    }

    fn expand(&self, working_dir: &Path) -> eyre::Result<Descriptor> {
        expand::expand_modules(self, working_dir)
    }

    fn expand_with_boundaries(
        &self,
        working_dir: &Path,
    ) -> eyre::Result<(Descriptor, ModuleBoundaries)> {
        let expanded = expand::expand_modules_with_boundaries(self, working_dir)?;
        Ok((expanded.descriptor, expanded.boundaries))
    }
}

/// Merge dataflow-level `env` into a node's `env`, with per-node keys winning
/// on conflict. Returns `None` when both inputs are empty so the resolved
/// node serializes cleanly when no env vars are set anywhere.
fn merge_env(
    global: Option<&BTreeMap<String, EnvValue>>,
    node: Option<BTreeMap<String, EnvValue>>,
) -> Option<BTreeMap<String, EnvValue>> {
    match (global, node) {
        (None, node) => node,
        (Some(global), None) if global.is_empty() => None,
        (Some(global), None) => Some(global.clone()),
        (Some(global), Some(node)) => {
            let mut merged = global.clone();
            // Per-node entries override global ones on key conflict.
            merged.extend(node);
            Some(merged)
        }
    }
}

pub async fn read_as_descriptor(path: &Path) -> eyre::Result<Descriptor> {
    let buf = tokio::fs::read(path)
        .await
        .context("failed to open given file")?;
    Descriptor::parse(buf)
}

fn node_kind_mut(node: &mut Node) -> eyre::Result<NodeKindMut<'_>> {
    match node.kind()? {
        NodeKind::Module(_) => {
            eyre::bail!(
                "module node `{}` must be expanded before resolution — \
                 call expand_modules() first",
                node.id
            )
        }
        NodeKind::Standard(_) => {
            let source = match (&node.git, &node.branch, &node.tag, &node.rev) {
                (None, None, None, None) => NodeSource::Local,
                (Some(repo), branch, tag, rev) => {
                    let rev = match (branch, tag, rev) {
                        (None, None, None) => None,
                        (Some(branch), None, None) => Some(GitRepoRev::Branch(branch.clone())),
                        (None, Some(tag), None) => Some(GitRepoRev::Tag(tag.clone())),
                        (None, None, Some(rev)) => Some(GitRepoRev::Rev(rev.clone())),
                        other @ (_, _, _) => {
                            eyre::bail!(
                                "only one of `branch`, `tag`, and `rev` are allowed (got {other:?})"
                            )
                        }
                    };
                    NodeSource::GitBranch {
                        repo: repo.clone(),
                        rev,
                    }
                }
                (None, _, _, _) => {
                    eyre::bail!("`git` source required when using branch, tag, or rev")
                }
            };

            Ok(NodeKindMut::Standard {
                path: node.path.as_ref().ok_or_eyre("missing `path` attribute")?,
                source,
                inputs: &mut node.inputs,
            })
        }
        NodeKind::Runtime(_) => node
            .operators
            .as_mut()
            .map(NodeKindMut::Runtime)
            .ok_or_eyre("no operators"),
        NodeKind::Custom(_) => node
            .custom
            .as_mut()
            .map(NodeKindMut::Custom)
            .ok_or_eyre("no custom"),
        NodeKind::Operator(_) => node
            .operator
            .as_mut()
            .map(NodeKindMut::Operator)
            .ok_or_eyre("no operator"),
        NodeKind::Ros2Bridge(_) => node
            .ros2
            .as_ref()
            .map(NodeKindMut::Ros2Bridge)
            .ok_or_eyre("no ros2"),
    }
}

/// Returns `true` if `source` is an `http://` or `https://` URL.
///
/// This is the trust boundary that decides whether a node `path` is fetched as
/// a remote download (and, for hub artifacts, checksum-verified) versus
/// resolved as a local filesystem path. The match is on the literal scheme
/// prefix and is **case-sensitive**: an upper-cased scheme such as `HTTPS://`
/// is treated as a path, not a URL. Schemes other than HTTP(S) (e.g. `ftp://`,
/// `s3://`, `file://`) are likewise not considered URLs here.
///
/// ```
/// use dora_core::descriptor::source_is_url;
///
/// assert!(source_is_url("https://example.com/node"));
/// assert!(source_is_url("http://example.com/node"));
///
/// assert!(!source_is_url("./build/my_node"));
/// assert!(!source_is_url("/usr/bin/my_node"));
/// assert!(!source_is_url("s3://bucket/key"));
/// assert!(!source_is_url("HTTPS://example.com/node")); // case-sensitive
/// ```
pub fn source_is_url(source: &str) -> bool {
    source.starts_with("https://") || source.starts_with("http://")
}

pub fn resolve_path(source: &str, working_dir: &Path) -> Result<PathBuf> {
    let path = Path::new(&source);
    let path = if path.extension().is_none() {
        path.with_extension(EXE_EXTENSION)
    } else {
        path.to_owned()
    };

    // Search path within current working directory.
    let joined = working_dir.join(&path);
    if joined.exists() {
        absolutize_preserving_symlinks(&joined)
    // Otherwise resolve against the `uv`-managed environment first (when `uv`
    // is available), then fall back to the system `$PATH`.
    } else if which::which("uv").is_ok() {
        resolve_path_via_uv(&path)
    } else if let Ok(abs_path) = which::which(&path) {
        Ok(abs_path)
    } else {
        bail!("Could not find source path {}", path.display())
    }
}

/// Resolve a hub node's entrypoint under **confined** rules (spec §11): the
/// executable may only come from the node's own working directory
/// (`<clone>/<subdir>`) or its managed Python environment. There is no
/// ambient-`$PATH` fallback — a typo or missing console script fails loudly
/// instead of silently running a host binary — and the resolved path is
/// canonicalized and checked to stay inside those roots, so a symlink
/// escaping the working dir is rejected rather than followed.
pub fn resolve_path_confined(
    source: &str,
    working_dir: &Path,
    python_env_dir: Option<&Path>,
) -> Result<PathBuf> {
    let path = Path::new(&source);
    let path = if path.extension().is_none() {
        path.with_extension(EXE_EXTENSION)
    } else {
        path.to_owned()
    };

    // a console script installed into the node's managed env
    if let Some(env_dir) = python_env_dir {
        let bin_dir = env_dir.join(if cfg!(windows) { "Scripts" } else { "bin" });
        let candidate = bin_dir.join(&path);
        if candidate.is_file() {
            return confine(&candidate, &bin_dir);
        }
    }

    // a file within the node's working dir (e.g. target/release/<bin>)
    let candidate = working_dir.join(&path);
    if candidate.exists() {
        return confine(&candidate, working_dir);
    }

    bail!(
        "could not find `{}` in the node's working directory `{}`{} — \
         hub nodes resolve only within their own package (no $PATH fallback)",
        path.display(),
        working_dir.display(),
        python_env_dir
            .map(|env| format!(" or its managed environment `{}`", env.display()))
            .unwrap_or_default(),
    )
}

/// Canonicalize `candidate` and require it to stay under `root`.
fn confine(candidate: &Path, root: &Path) -> Result<PathBuf> {
    let resolved = candidate
        .canonicalize()
        .with_context(|| format!("failed to canonicalize `{}`", candidate.display()))?;
    let root = root
        .canonicalize()
        .with_context(|| format!("failed to canonicalize `{}`", root.display()))?;
    if !resolved.starts_with(&root) {
        bail!(
            "entrypoint `{}` resolves outside the node's directory `{}` \
             (symlink escape?) — refusing to run it",
            resolved.display(),
            root.display()
        );
    }
    Ok(resolved)
}

/// Make an existing executable path absolute WITHOUT following symlinks.
///
/// Deliberately not `canonicalize()`: that resolves symlinks, and a
/// virtualenv's `bin/python` is a symlink whose *location* is what
/// CPython uses to discover `pyvenv.cfg`. Resolving it before exec runs
/// the base interpreter with no venv, so imports that work in a shell
/// fail under dora (dora-rs/dora#2918). `path::absolute` only prepends
/// the cwd and drops `.` components; symlinks and `..` are left for the
/// kernel to resolve at exec time, which matches shell behavior.
///
/// Errors if the path does not exist (`exists()` traverses symlinks, so
/// a dangling link counts as missing — same outcome canonicalize gave).
/// Shared by every [`resolve_path`] branch so the no-symlink-resolution
/// contract cannot regress in one branch while the tests exercise
/// another.
fn absolutize_preserving_symlinks(path: &Path) -> Result<PathBuf> {
    if !path.exists() {
        bail!("path {} does not exist", path.display());
    }
    std::path::absolute(path)
        .with_context(|| format!("failed to make path {} absolute", path.display()))
}

/// Resolve `path` against the `uv`-managed environment by running
/// `uv run which <path>`, returning an absolute path.
///
/// Unlike a fire-and-forget spawn, this waits for the child, checks its
/// exit status (so a missing binary surfaces as an error), and verifies
/// the captured location exists — without resolving symlinks, which
/// would reintroduce the venv bypass fixed for the working-dir branch
/// (dora-rs/dora#2918).
fn resolve_path_via_uv(path: &Path) -> Result<PathBuf> {
    let which = if cfg!(windows) { "where" } else { "which" };
    let output = Command::new("uv")
        .arg("run")
        .arg(which)
        .arg(path)
        .output()
        .with_context(|| format!("failed to run `uv run {which}`"))?;
    if !output.status.success() {
        bail!("Could not find source path {} within uv", path.display());
    }
    // `which`/`where` may emit multiple matches; the first line is the
    // resolved binary.
    let stdout = String::from_utf8_lossy(&output.stdout);
    let resolved = stdout
        .lines()
        .map(str::trim)
        .find(|line| !line.is_empty())
        .ok_or_else(|| eyre::eyre!("`uv run {which} {}` produced no output", path.display()))?;
    absolutize_preserving_symlinks(Path::new(resolved))
        .with_context(|| format!("uv-resolved path {resolved} is not usable"))
}

pub trait NodeExt {
    fn kind(&self) -> eyre::Result<NodeKind<'_>>;
}

impl NodeExt for Node {
    fn kind(&self) -> eyre::Result<NodeKind<'_>> {
        if self.hub.is_some() && self.path.is_none() {
            // `hub:` is desugared into a concrete git node by `dora build` /
            // `dora run` / `dora validate` before any kind dispatch — an
            // unresolved reference reaching this point means a flow that
            // skipped resolution
            eyre::bail!(
                "node `{}` uses an unresolved `hub:` reference — run `dora build` \
                 first (`dora start` requires a prior build for hub nodes)",
                self.id
            );
        }
        match (
            &self.path,
            &self.operators,
            &self.custom,
            &self.operator,
            &self.ros2,
            &self.module,
        ) {
            (None, None, None, None, None, None) => {
                eyre::bail!(
                    "node `{}` requires a `path`, `custom`, `operators`, `ros2`, or `module` field",
                    self.id
                )
            }
            (None, None, None, Some(operator), None, None) => Ok(NodeKind::Operator(operator)),
            (None, None, Some(custom), None, None, None) => Ok(NodeKind::Custom(custom)),
            (None, Some(runtime), None, None, None, None) => Ok(NodeKind::Runtime(runtime)),
            (Some(path), None, None, None, None, None) => Ok(NodeKind::Standard(path)),
            (None, None, None, None, Some(ros2), None) => Ok(NodeKind::Ros2Bridge(ros2)),
            (None, None, None, None, None, Some(module)) => Ok(NodeKind::Module(module)),
            _ => {
                eyre::bail!(
                    "node `{}` has multiple exclusive fields set, only one of `path`, `custom`, `operators`, `operator`, `ros2`, and `module` is allowed",
                    self.id
                )
            }
        }
    }
}

#[derive(Debug)]
pub enum NodeKind<'a> {
    Standard(&'a String),
    /// Dora runtime node
    Runtime(&'a RuntimeNode),
    Custom(&'a CustomNode),
    Operator(&'a SingleOperatorDefinition),
    /// ROS2 bridge node
    Ros2Bridge(&'a Ros2BridgeConfig),
    /// Module (sub-dataflow) reference — must be expanded before resolution
    Module(&'a String),
}

#[derive(Debug)]
enum NodeKindMut<'a> {
    Standard {
        path: &'a String,
        source: NodeSource,
        inputs: &'a mut BTreeMap<DataId, Input>,
    },
    /// Dora runtime node
    Runtime(&'a mut RuntimeNode),
    Custom(&'a mut CustomNode),
    Operator(&'a mut SingleOperatorDefinition),
    /// ROS2 bridge node
    Ros2Bridge(&'a Ros2BridgeConfig),
}

#[cfg(test)]
mod tests {
    /// dora-rs/dora#2920: the command-line flag beats the descriptor in
    /// BOTH directions, and its absence beats neither.
    ///
    /// The third state matters: if "flag omitted" meant `false`, a YAML
    /// `exit_when_nodes_finish: true` would be overridden on every
    /// invocation, and since `dora run` and `dora start` are the only
    /// ways to start a dataflow, the descriptor field could never take
    /// effect at all.
    #[test]
    fn exit_when_nodes_finish_override_semantics() {
        use super::DescriptorExt;

        let parse = |yaml: &str| -> Descriptor { serde_yaml::from_str(yaml).expect("parse") };
        let with_setting = "exit_when_nodes_finish: true\nnodes:\n  - id: a\n    path: ./a\n";
        let without = "nodes:\n  - id: a\n    path: ./a\n";

        // Omitted: the descriptor decides, either way.
        let mut d = parse(with_setting);
        d.apply_exit_when_nodes_finish(None);
        assert_eq!(
            d.exit_when_nodes_finish,
            Some(true),
            "omitting the flag must not silently disable a policy the \
             dataflow file asked for"
        );

        let mut d = parse(without);
        d.apply_exit_when_nodes_finish(None);
        assert_eq!(d.exit_when_nodes_finish, None, "and must not invent one");

        // Given: it wins, including against an opposite descriptor value.
        let mut d = parse(with_setting);
        d.apply_exit_when_nodes_finish(Some(false));
        assert_eq!(
            d.exit_when_nodes_finish,
            Some(false),
            "`--exit-when-nodes-finish=false` must be able to turn OFF a \
             policy the dataflow file turned on"
        );

        let mut d = parse(without);
        d.apply_exit_when_nodes_finish(Some(true));
        assert_eq!(d.exit_when_nodes_finish, Some(true));
    }

    use super::*;

    fn env(pairs: &[(&str, &str)]) -> BTreeMap<String, EnvValue> {
        pairs
            .iter()
            .map(|(k, v)| (k.to_string(), EnvValue::String(v.to_string())))
            .collect()
    }

    #[test]
    fn merge_env_returns_none_when_both_absent() {
        assert!(merge_env(None, None).is_none());
    }

    #[test]
    fn merge_env_keeps_per_node_when_no_global() {
        let node_env = env(&[("A", "1")]);
        let merged = merge_env(None, Some(node_env.clone())).unwrap();
        assert_eq!(merged, node_env);
    }

    #[test]
    fn merge_env_keeps_global_when_no_per_node() {
        let global = env(&[("A", "1")]);
        let merged = merge_env(Some(&global), None).unwrap();
        assert_eq!(merged, global);
    }

    #[test]
    fn merge_env_per_node_overrides_global_on_conflict() {
        let global = env(&[("A", "global"), ("B", "global")]);
        let node_env = env(&[("A", "node"), ("C", "node")]);
        let merged = merge_env(Some(&global), Some(node_env)).unwrap();
        assert_eq!(merged.get("A"), Some(&EnvValue::String("node".into())));
        assert_eq!(merged.get("B"), Some(&EnvValue::String("global".into())));
        assert_eq!(merged.get("C"), Some(&EnvValue::String("node".into())));
    }

    fn resolved_input_mapping<'a>(
        resolved: &'a BTreeMap<NodeId, ResolvedNode>,
        node: &str,
        input: &str,
    ) -> &'a InputMapping {
        let node = resolved
            .get(&NodeId::from(node.to_string()))
            .expect("node resolved");
        let inputs = match &node.kind {
            CoreNodeKind::Custom(n) => &n.run_config.inputs,
            CoreNodeKind::Runtime(_) => panic!("expected custom node"),
        };
        &inputs
            .get(&DataId::from(input.to_string()))
            .expect("input present")
            .mapping
    }

    #[test]
    fn add_node_prefixes_single_operator_producer_input_via_topology() {
        // A node added to a running dataflow via `AddNode` is resolved against a
        // single-node descriptor. If its input references an existing
        // single-`operator:` producer, the `op/` output prefix that
        // whole-descriptor resolution applies must still be added — sourced
        // from the surrounding topology — or the node subscribes to an output
        // name nobody publishes and silently receives no data (#2877).
        let topology: Descriptor = serde_yaml::from_str(
            "\
nodes:
  - id: producer
    operator:
      python: producer.py
      outputs:
        - result
",
        )
        .expect("parse topology");

        let added: Descriptor = serde_yaml::from_str(
            "\
nodes:
  - id: consumer
    path: consumer
    inputs:
      reading: producer/result
",
        )
        .expect("parse added node");

        // With the topology the input is prefixed to the operator-qualified
        // output name the runtime actually publishes under (`op/result`).
        let resolved = resolve_aliases_and_set_defaults_in_topology(&added, &topology.nodes)
            .expect("resolve in topology");
        match resolved_input_mapping(&resolved, "consumer", "reading") {
            InputMapping::User(m) => {
                assert_eq!(m.source, NodeId::from("producer".to_string()));
                assert_eq!(m.output, DataId::from("op/result".to_string()));
            }
            other => panic!("expected user mapping, got {other:?}"),
        }

        // Contrast: with no topology the producer is not in scope at all, so
        // there is nothing to key the rewrite off and the name stays bare.
        // That is the correct answer for the inputs given — which is exactly
        // why the `AddNode` path had to stop resolving in isolation.
        let resolved_isolated = added
            .resolve_aliases_and_set_defaults()
            .expect("resolve isolated");
        match resolved_input_mapping(&resolved_isolated, "consumer", "reading") {
            InputMapping::User(m) => {
                assert_eq!(m.output, DataId::from("result".to_string()));
            }
            other => panic!("expected user mapping, got {other:?}"),
        }
    }

    #[test]
    fn topology_lookup_prefers_the_node_being_added_over_a_same_id_topology_entry() {
        // The lookup chains topology nodes *before* `desc`'s own, so a node id
        // present in both resolves against the copy being added rather than a
        // stale topology entry. Unreachable through `AddNode` today (duplicate
        // ids are rejected up front and `RemoveNode` prunes the stored
        // descriptor), but the precedence should not depend on that.
        let topology: Descriptor = serde_yaml::from_str(
            "\
nodes:
  - id: producer
    operator:
      id: stale
      python: producer.py
      outputs:
        - result
",
        )
        .expect("parse topology");

        let added: Descriptor = serde_yaml::from_str(
            "\
nodes:
  - id: producer
    operator:
      id: fresh
      python: producer.py
      outputs:
        - result
  - id: consumer
    path: consumer
    inputs:
      reading: producer/result
",
        )
        .expect("parse added nodes");

        let resolved = resolve_aliases_and_set_defaults_in_topology(&added, &topology.nodes)
            .expect("resolve in topology");
        match resolved_input_mapping(&resolved, "consumer", "reading") {
            InputMapping::User(m) => {
                assert_eq!(m.output, DataId::from("fresh/result".to_string()));
            }
            other => panic!("expected user mapping, got {other:?}"),
        }
    }

    #[test]
    fn descriptor_global_env_parses_from_yaml() {
        // Verify the new top-level `env:` field parses and the resolver
        // hands merged envs to every node with per-node keys winning.
        let yaml = r#"
env:
  RUST_LOG: info
  OTEL_ENDPOINT: http://collector:4317
nodes:
  - id: a
    path: ./a
    env:
      RUST_LOG: debug
  - id: b
    path: ./b
"#;
        let desc: Descriptor = serde_yaml::from_str(yaml).expect("parse");
        let resolved = desc.resolve_aliases_and_set_defaults().expect("resolve");

        let a = resolved.get(&NodeId::from("a".to_string())).unwrap();
        let a_env = a.env.as_ref().expect("node a inherits env");
        // Per-node RUST_LOG=debug wins over global RUST_LOG=info.
        assert_eq!(
            a_env.get("RUST_LOG"),
            Some(&EnvValue::String("debug".into()))
        );
        // Global key not overridden on node a is still visible.
        assert_eq!(
            a_env.get("OTEL_ENDPOINT"),
            Some(&EnvValue::String("http://collector:4317".into()))
        );

        let b = resolved.get(&NodeId::from("b".to_string())).unwrap();
        let b_env = b.env.as_ref().expect("node b inherits global env");
        assert_eq!(
            b_env.get("RUST_LOG"),
            Some(&EnvValue::String("info".into()))
        );
        assert_eq!(
            b_env.get("OTEL_ENDPOINT"),
            Some(&EnvValue::String("http://collector:4317".into()))
        );
    }

    #[test]
    fn invalid_operator_id_prefix_errors_instead_of_panicking() {
        // A single-operator node whose `operator.id` contains a character
        // outside the `DataId` set (here a space) is accepted at parse time
        // (`OperatorId` is unvalidated). When another node references its
        // output, the resolver prefixes the output with the operator id. That
        // used to build the qualified id via `DataId::from`, which panics on
        // invalid characters and aborted `dora check`/`graph`/`build`. It must
        // now surface as a clean `Err` instead.
        let yaml = r#"
nodes:
  - id: producer
    operator:
      id: "bad id"
      python: op.py
      outputs: [result]
  - id: consumer
    path: ./consumer
    inputs:
      x: producer/result
"#;
        let desc: Descriptor = serde_yaml::from_str(yaml).expect("parse");
        let result = desc.resolve_aliases_and_set_defaults();
        assert!(result.is_err(), "expected a clean descriptor error, got Ok");
    }

    #[test]
    fn resolve_path_errors_for_nonexistent_binary() {
        // Regression for #2016: the `uv` fallback previously spawned
        // `uv run which <path>` fire-and-forget and returned the original
        // (relative) path even when the binary did not exist. A missing
        // binary must surface as an `Err`, and any successful resolution
        // must be an absolute path.
        let working_dir = std::env::current_dir().expect("cwd");
        let result = resolve_path("dora_nonexistent_binary_2016_regression", &working_dir);
        assert!(
            result.is_err(),
            "expected Err for a binary that exists nowhere, got {result:?}"
        );
    }

    /// dora-rs/dora#2918: resolving a node path must NOT follow symlinks.
    ///
    /// A virtualenv's `bin/python` is a symlink to the base interpreter,
    /// and CPython's venv discovery hinges on that: it looks for
    /// `pyvenv.cfg` relative to the path it was *invoked* as, not the
    /// symlink's target. Canonicalizing before exec therefore runs the
    /// base interpreter with no venv — imports that work in a shell
    /// (`.venv/bin/python -c "import numpy"`) fail under dora with
    /// `ModuleNotFoundError`.
    #[test]
    #[cfg(unix)]
    fn resolve_path_preserves_symlinks() {
        let tmp = tempfile::tempdir().expect("tempdir");
        let target = tmp.path().join("base-interpreter.bin");
        std::fs::write(&target, b"x").unwrap();
        let link = tmp.path().join("venv-python.bin");
        std::os::unix::fs::symlink(&target, &link).unwrap();

        // relative source resolved against the working dir
        let resolved = resolve_path("venv-python.bin", tmp.path()).unwrap();
        assert!(resolved.is_absolute());
        assert!(
            resolved.ends_with("venv-python.bin"),
            "resolve_path followed the symlink: {} — venv discovery \
             (pyvenv.cfg) is keyed off the symlink location, so execing \
             the target bypasses the venv",
            resolved.display()
        );

        // absolute source (the shape from the issue: `path: /…/.venv/bin/python`)
        let resolved = resolve_path(link.to_str().unwrap(), Path::new("/")).unwrap();
        assert!(
            resolved.ends_with("venv-python.bin"),
            "absolute symlink path was canonicalized: {}",
            resolved.display()
        );

        // `..` components survive too: they are resolved by the kernel at
        // exec time, AFTER any symlinked directories — which is the
        // shell-matching semantic. A lexical "cleanup" that collapses
        // them would resolve differently through symlinked dirs.
        std::fs::create_dir(tmp.path().join("sub")).unwrap();
        let resolved = resolve_path("sub/../venv-python.bin", tmp.path()).unwrap();
        assert!(
            resolved.ends_with("sub/../venv-python.bin"),
            "`..` was normalized away: {}",
            resolved.display()
        );
    }

    /// A dangling symlink is "missing": `exists()` traverses the link, so
    /// resolution falls through to the uv/$PATH branches and ultimately
    /// errors — the same outcome the old `canonicalize()` failure gave.
    /// Pins the branch boundary so a switch to `symlink_metadata()`
    /// (which would treat the dangling link as present and exec a
    /// guaranteed-ENOENT path) doesn't slip in silently.
    #[test]
    #[cfg(unix)]
    fn resolve_path_treats_dangling_symlink_as_missing() {
        let tmp = tempfile::tempdir().expect("tempdir");
        let link = tmp.path().join("dangling-2918-regression.bin");
        std::os::unix::fs::symlink(tmp.path().join("no-such-target"), &link).unwrap();

        let result = resolve_path("dangling-2918-regression.bin", tmp.path());
        assert!(
            result.is_err(),
            "a dangling symlink must not resolve (nothing on uv/$PATH matches \
             this name either), got {result:?}"
        );
    }

    #[test]
    fn resolve_path_confined_has_no_path_fallback() {
        let tmp = tempfile::tempdir().expect("tempdir");
        // `sh` exists on $PATH everywhere on unix — confined resolution must
        // NOT find it (spec §11: a typo or missing console script fails, it
        // never silently runs a host binary)
        let result = resolve_path_confined("sh", tmp.path(), None);
        assert!(result.is_err(), "expected Err, got {result:?}");

        // a real file in the working dir resolves
        let exe = if cfg!(windows) {
            "node.exe"
        } else {
            "node.bin"
        };
        std::fs::write(tmp.path().join(exe), b"x").unwrap();
        let resolved = resolve_path_confined(exe, tmp.path(), None).unwrap();
        assert!(resolved.is_absolute());

        // a managed-env console script resolves through the env's bin dir
        let env_dir = tmp.path().join("env");
        let bin_dir = env_dir.join(if cfg!(windows) { "Scripts" } else { "bin" });
        std::fs::create_dir_all(&bin_dir).unwrap();
        std::fs::write(bin_dir.join(exe), b"x").unwrap();
        let resolved =
            resolve_path_confined(exe, tmp.path().join("empty").as_path(), Some(&env_dir));
        assert!(resolved.is_ok(), "{resolved:?}");
    }

    #[cfg(unix)]
    #[test]
    fn resolve_path_confined_rejects_symlink_escape() {
        let tmp = tempfile::tempdir().expect("tempdir");
        let working_dir = tmp.path().join("work");
        std::fs::create_dir_all(&working_dir).unwrap();
        let outside = tmp.path().join("outside.bin");
        std::fs::write(&outside, b"x").unwrap();
        std::os::unix::fs::symlink(&outside, working_dir.join("escape.bin")).unwrap();
        let result = resolve_path_confined("escape.bin", &working_dir, None);
        assert!(
            result.is_err(),
            "a symlink pointing outside the working dir must be rejected, got {result:?}"
        );
        let msg = format!("{:#}", result.unwrap_err());
        assert!(msg.contains("outside"), "{msg}");
    }

    #[test]
    fn unresolved_hub_node_has_clear_kind_error() {
        let node: Node = serde_yaml::from_str("id: x\nhub: dora-yolo@^0.5\n").unwrap();
        let err = node.kind().unwrap_err();
        assert!(format!("{err}").contains("dora build"), "{err}");
    }

    #[test]
    fn resolve_path_via_uv_errors_for_nonexistent_binary() {
        // Pins the #2016 root cause directly on the `uv` branch. The buggy
        // code spawned `uv run which <path>` fire-and-forget and returned
        // `Ok(<relative path>)` regardless of the child's exit status, so a
        // missing binary was silently accepted. This branch only runs when
        // `uv` is installed (the only environment where the bug manifested),
        // so guard on its presence to keep the test meaningful where it can
        // actually discriminate the fix.
        if which::which("uv").is_err() {
            return;
        }
        let path = Path::new("dora_nonexistent_binary_2016_regression");
        let result = resolve_path_via_uv(path);
        assert!(
            result.is_err(),
            "expected Err from `uv run which` for a missing binary, got {result:?}"
        );
    }

    #[test]
    fn descriptor_without_global_env_preserves_per_node_env() {
        // Regression guard: no top-level `env:` must leave per-node env
        // semantics unchanged.
        let yaml = r#"
nodes:
  - id: a
    path: ./a
    env:
      FOO: bar
  - id: b
    path: ./b
"#;
        let desc: Descriptor = serde_yaml::from_str(yaml).expect("parse");
        let resolved = desc.resolve_aliases_and_set_defaults().expect("resolve");
        let a = resolved.get(&NodeId::from("a".to_string())).unwrap();
        assert_eq!(
            a.env.as_ref().and_then(|e| e.get("FOO")),
            Some(&EnvValue::String("bar".into()))
        );
        let b = resolved.get(&NodeId::from("b".to_string())).unwrap();
        assert!(b.env.is_none(), "node b has no env anywhere");
    }

    #[test]
    fn duplicate_node_id_is_rejected() {
        // Regression for #2393: a plain dataflow with two nodes sharing the
        // same `id` must return an error instead of silently discarding one.
        let yaml = r#"
nodes:
  - id: my-node
    path: ./a
  - id: my-node
    path: ./b
"#;
        let desc: Descriptor = serde_yaml::from_str(yaml).expect("parse");
        let err = desc
            .resolve_aliases_and_set_defaults()
            .expect_err("duplicate node ID must be rejected");
        let msg = format!("{err:#}");
        assert!(
            msg.contains("duplicate node ID") && msg.contains("my-node"),
            "unexpected error message: {msg}"
        );
    }
}
