use adora_message::{
    config::{Input, InputMapping, UserInputMapping},
    descriptor::{Descriptor, EnvValue, Node},
    id::{DataId, NodeId},
};
use eyre::{Context, bail};
use serde::Deserialize;
use std::{
    collections::{BTreeMap, BTreeSet, HashSet},
    path::{Path, PathBuf},
};

/// Check if a path string is absolute on any platform.
/// On Windows, `Path::is_absolute()` returns false for Unix-style `/foo` paths,
/// so we also check for a leading `/` to catch cross-platform absolute paths.
fn is_absolute_any_platform(path: &str) -> bool {
    Path::new(path).is_absolute() || path.starts_with('/')
}

/// Maximum nesting depth for recursive module expansion. Prevents unbounded
/// recursion from deeply nested or circular module graphs that evade the
/// path-based cycle check. 8 levels covers realistic robotics pipelines
/// while keeping memory usage bounded.
const MAX_MODULE_DEPTH: u8 = 8;

/// Maximum module file size (1 MB). Prevents DoS from huge or infinite files.
const MAX_MODULE_FILE_SIZE: u64 = 1_048_576;

/// Reserved node-ID prefix used inside module files to reference module inputs.
/// Usage in module YAML: `_mod/port_name`
const MODULE_INPUT_SOURCE: &str = "_mod";

/// Header section of a module definition file.
#[derive(Debug, Clone, Deserialize)]
struct ModuleHeader {
    name: String,
    #[serde(default)]
    inputs: Vec<DataId>,
    #[serde(default)]
    inputs_optional: Vec<DataId>,
    #[serde(default)]
    outputs: Vec<DataId>,
}

/// A module definition file (`*_module.yml`).
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct ModuleFile {
    module: ModuleHeader,
    nodes: Vec<Node>,
    /// Module-level build command, runs before inner node builds.
    #[serde(default)]
    build: Option<String>,
}

/// Metadata about which nodes came from which module, used for graph
/// visualization with subgraph boundaries.
#[derive(Debug, Clone, Default)]
pub struct ModuleBoundaries {
    /// Maps module_id -> list of expanded node IDs that belong to it.
    pub modules: BTreeMap<String, Vec<String>>,
}

/// Result of expanding modules, including boundary metadata for visualization.
#[derive(Debug, Clone)]
pub struct ExpandedDescriptor {
    pub descriptor: Descriptor,
    pub boundaries: ModuleBoundaries,
}

/// Expand all module references in `descriptor` into flat nodes.
///
/// Module nodes are replaced by the inner nodes from the referenced module
/// file. Internal IDs are prefixed with `{module_id}.` and all input/output
/// wiring is rewritten so the result is a plain flat descriptor.
pub fn expand_modules(descriptor: &Descriptor, base_dir: &Path) -> eyre::Result<Descriptor> {
    Ok(expand_modules_with_boundaries(descriptor, base_dir)?.descriptor)
}

/// Like [`expand_modules`] but also returns module boundary metadata for
/// visualization.
pub fn expand_modules_with_boundaries(
    descriptor: &Descriptor,
    base_dir: &Path,
) -> eyre::Result<ExpandedDescriptor> {
    let has_modules = descriptor.nodes.iter().any(|n| n.module.is_some());
    if !has_modules {
        return Ok(ExpandedDescriptor {
            descriptor: descriptor.clone(),
            boundaries: ModuleBoundaries::default(),
        });
    }

    let canonical_base = base_dir
        .canonicalize()
        .with_context(|| format!("failed to resolve base directory: {}", base_dir.display()))?;
    let mut seen = HashSet::new();
    let mut flat_nodes = Vec::new();
    let mut output_maps: BTreeMap<String, BTreeMap<String, String>> = BTreeMap::new();
    let mut boundaries = ModuleBoundaries::default();

    for node in &descriptor.nodes {
        if node.module.is_some() {
            let (expanded, omap) =
                expand_module_node(node, base_dir, &canonical_base, 0, &mut seen)?;
            let module_id = node.id.to_string();
            output_maps.insert(module_id.clone(), omap);
            let node_ids: Vec<String> = expanded.iter().map(|n| n.id.to_string()).collect();
            boundaries.modules.insert(module_id, node_ids);
            flat_nodes.extend(expanded);
        } else {
            flat_nodes.push(node.clone());
        }
    }

    rewrite_external_refs(&mut flat_nodes, &output_maps)?;

    // Verify expanded node IDs are unique
    let mut id_set = HashSet::new();
    for node in &flat_nodes {
        if !id_set.insert(node.id.to_string()) {
            bail!(
                "duplicate node ID `{}` after module expansion — check for \
                 conflicting node names across modules and top-level nodes",
                node.id
            );
        }
    }

    Ok(ExpandedDescriptor {
        descriptor: Descriptor {
            nodes: flat_nodes,
            communication: descriptor.communication.clone(),
            deploy: descriptor.deploy.clone(),
            debug: descriptor.debug.clone(),
            health_check_interval: descriptor.health_check_interval,
            strict_types: descriptor.strict_types,
            type_rules: descriptor.type_rules.clone(),
        },
        boundaries,
    })
}

/// Validate a module file in isolation without expanding it into a dataflow.
///
/// Checks:
/// - Module header is well-formed (name, inputs, outputs)
/// - All inner nodes are parseable
/// - All `_mod/X` references point to declared inputs or optional inputs
/// - All declared outputs are produced by some inner node (or nested module)
/// - No circular references within the module
pub fn check_module_file(module_path: &Path) -> eyre::Result<()> {
    let canonical = module_path
        .canonicalize()
        .with_context(|| format!("module file not found: {}", module_path.display()))?;
    let module_file = load_module_file(&canonical)?;
    let module_dir = canonical
        .parent()
        .expect("module file must have a parent directory");

    // Collect all declared + optional input names
    let all_input_names: BTreeSet<String> = module_file
        .module
        .inputs
        .iter()
        .chain(module_file.module.inputs_optional.iter())
        .map(|d| d.to_string())
        .collect();

    // Check _mod/ references point to declared inputs
    for node in &module_file.nodes {
        for (input_id, input) in &node.inputs {
            if let InputMapping::User(m) = &input.mapping {
                if m.source.to_string() == MODULE_INPUT_SOURCE {
                    let port = m.output.to_string();
                    if !all_input_names.contains(&port) {
                        bail!(
                            "module `{}`: node `{}` input `{}` references \
                             `_mod/{}` but `{}` is not declared in module \
                             inputs or inputs_optional",
                            module_file.module.name,
                            node.id,
                            input_id,
                            port,
                            port,
                        );
                    }
                }
            }
        }
    }

    // Check outputs: each declared output should be produced by some inner node.
    // For nested module children, recursively load their declared outputs.
    let mut inner_outputs: BTreeSet<String> = module_file
        .nodes
        .iter()
        .filter(|n| n.module.is_none())
        .flat_map(|n| n.outputs.iter().map(|o| o.to_string()))
        .collect();

    // Check nested module files exist and collect their declared outputs
    for node in &module_file.nodes {
        if let Some(ref mod_path) = node.module {
            if is_absolute_any_platform(mod_path) {
                bail!(
                    "module `{}`: nested module path `{}` must be relative (node `{}`)",
                    module_file.module.name,
                    mod_path,
                    node.id,
                );
            }
            let nested = module_dir.join(mod_path);
            let nested_canonical = nested.canonicalize().with_context(|| {
                format!(
                    "module `{}`: nested module `{}` referenced by node `{}` not found",
                    module_file.module.name, mod_path, node.id,
                )
            })?;
            let base_canonical = module_dir.canonicalize()?;
            if !nested_canonical.starts_with(&base_canonical) {
                bail!(
                    "module `{}`: nested module path `{}` escapes the module directory",
                    module_file.module.name,
                    mod_path,
                );
            }
            // Load nested module to collect its declared outputs
            let nested_module = load_module_file(&nested_canonical)?;
            for output in &nested_module.module.outputs {
                inner_outputs.insert(output.to_string());
            }
        }
    }

    for declared_output in &module_file.module.outputs {
        let output_str = declared_output.to_string();
        if !inner_outputs.contains(&output_str) {
            bail!(
                "module `{}` declares output `{}` but no inner node produces it",
                module_file.module.name,
                declared_output,
            );
        }
    }

    Ok(())
}

/// Expand a single module node into its constituent flat nodes.
///
/// Returns `(expanded_nodes, output_map)` where output_map maps each declared
/// module output name to the prefixed inner node ID that produces it.
fn expand_module_node(
    node: &Node,
    base_dir: &Path,
    canonical_base: &Path,
    depth: u8,
    seen: &mut HashSet<PathBuf>,
) -> eyre::Result<(Vec<Node>, BTreeMap<String, String>)> {
    if depth >= MAX_MODULE_DEPTH {
        bail!(
            "module nesting exceeds depth limit of {MAX_MODULE_DEPTH} \
             (node `{}`)",
            node.id
        );
    }

    let module_path_str = node
        .module
        .as_ref()
        .expect("expand_module_node called on non-module node");

    // Security: reject absolute paths and path traversal
    if is_absolute_any_platform(module_path_str) {
        bail!(
            "module path `{}` must be relative (node `{}`)",
            module_path_str,
            node.id
        );
    }

    let module_path = base_dir.join(module_path_str);
    let canonical = module_path
        .canonicalize()
        .with_context(|| format!("module file not found: {}", module_path.display()))?;

    if !canonical.starts_with(canonical_base) {
        bail!(
            "module path `{}` escapes the project directory (node `{}`)",
            module_path_str,
            node.id
        );
    }

    if !seen.insert(canonical.clone()) {
        bail!(
            "circular module reference detected: {} (node `{}`)\n\
             hint: check that module files do not reference each other \
             in a cycle",
            module_path.display(),
            node.id
        );
    }

    let module_file = load_module_file(&canonical)?;
    let module_id = node.id.to_string();
    let module_dir = canonical
        .parent()
        .expect("module file must have a parent directory");

    // Validate: all required module inputs are provided by the node's inputs
    for declared_input in &module_file.module.inputs {
        if !node.inputs.contains_key(declared_input) {
            bail!(
                "module `{}` declares required input `{}` but node `{}` \
                 does not provide it\n\
                 hint: add `{}: <source_node>/<output>` to the node's inputs",
                module_file.module.name,
                declared_input,
                node.id,
                declared_input,
            );
        }
    }
    // Optional inputs: no error if missing — inner nodes referencing them
    // will simply have their input removed during rewrite.

    // Build optional input set once for fast lookup
    let optional_inputs: BTreeSet<String> = module_file
        .module
        .inputs_optional
        .iter()
        .map(|d| d.to_string())
        .collect();

    // Validate and collect params
    for key in node.params.keys() {
        if !key.chars().all(|c| c.is_ascii_alphanumeric() || c == '_') || key.is_empty() {
            bail!(
                "invalid param key `{}` in node `{}`: must be non-empty and \
                 contain only [A-Za-z0-9_]",
                key,
                node.id
            );
        }
    }
    let params = &node.params;

    // Build set of inner node IDs for cross-reference rewriting
    let inner_node_ids: BTreeSet<String> =
        module_file.nodes.iter().map(|n| n.id.to_string()).collect();

    // Phase 1: prefix IDs, rewrite inputs, resolve paths, substitute params
    let mut prefixed_nodes = Vec::new();
    for mut inner_node in module_file.nodes {
        let prefixed_id: NodeId = format!("{module_id}.{}", inner_node.id).into();
        inner_node.id = prefixed_id;

        // Rewrite inputs (None = optional input not provided, skip it)
        let mut new_inputs = BTreeMap::new();
        for (input_id, input) in &inner_node.inputs {
            match rewrite_module_input(
                input,
                &module_id,
                &node.inputs,
                &inner_node_ids,
                &optional_inputs,
            )? {
                Some(new_input) => {
                    new_inputs.insert(input_id.clone(), new_input);
                }
                None => continue, // optional input not provided
            }
        }
        inner_node.inputs = new_inputs;

        // Resolve relative paths: make inner node paths relative to base_dir
        if let Some(ref path) = inner_node.path {
            if !path.contains("://") && !Path::new(path).is_absolute() {
                let resolved = module_dir.join(path);
                let relative = resolved.strip_prefix(canonical_base).map_err(|_| {
                    eyre::eyre!(
                        "module node `{}` path `{}` resolves outside the project \
                         directory (resolved to `{}`)",
                        inner_node.id,
                        path,
                        resolved.display()
                    )
                })?;
                inner_node.path = Some(relative.to_string_lossy().into_owned());
            }
        }

        // Propagate deploy from module node to inner nodes
        if inner_node.deploy.is_none() {
            inner_node.deploy = node.deploy.clone();
        }

        // Substitute params in env values
        if !params.is_empty() {
            substitute_params_in_node(&mut inner_node, params);
        }

        // Prepend module-level build command to inner node builds
        if let Some(ref module_build) = module_file.build {
            let inner_build = inner_node.build.take();
            inner_node.build = Some(match inner_build {
                Some(existing) => format!("{module_build}\n{existing}"),
                None => module_build.clone(),
            });
        }

        prefixed_nodes.push(inner_node);
    }

    // Phase 2: recursively expand nested modules (before building output map)
    // Collect nested output maps so sibling nodes can reference nested module
    // outputs correctly via rewrite_external_refs.
    let mut nested_output_maps: BTreeMap<String, BTreeMap<String, String>> = BTreeMap::new();
    let mut final_nodes = Vec::new();
    for inner_node in prefixed_nodes {
        if inner_node.module.is_some() {
            let nested_id = inner_node.id.to_string();
            let (nested, nested_omap) =
                expand_module_node(&inner_node, module_dir, canonical_base, depth + 1, seen)?;
            nested_output_maps.insert(nested_id, nested_omap);
            final_nodes.extend(nested);
        } else {
            final_nodes.push(inner_node);
        }
    }

    // Rewrite sibling references that point to nested module outputs
    if !nested_output_maps.is_empty() {
        rewrite_external_refs(&mut final_nodes, &nested_output_maps)?;
    }

    // Phase 3: build output map from fully-expanded flat nodes
    let mut output_map: BTreeMap<String, String> = BTreeMap::new();
    for declared_output in &module_file.module.outputs {
        let producer = final_nodes
            .iter()
            .find(|n| n.outputs.contains(declared_output))
            .ok_or_else(|| {
                eyre::eyre!(
                    "module `{}` declares output `{}` but no inner node produces it",
                    module_file.module.name,
                    declared_output,
                )
            })?;
        output_map.insert(declared_output.to_string(), producer.id.to_string());
    }

    // Remove from seen so the same module file can be used in different
    // branches (multiple instances)
    seen.remove(&canonical);

    Ok((final_nodes, output_map))
}

/// Substitute `${_param.name}` references in a node's args and inject params
/// into the node's env map as `EnvValue::String` entries.
fn substitute_params_in_node(node: &mut Node, params: &BTreeMap<String, String>) {
    // Substitute in args
    if let Some(ref mut args) = node.args {
        *args = substitute_params_in_str(args, params);
    }

    // Inject params into env map (caller params override inner defaults)
    let env = node.env.get_or_insert_with(BTreeMap::new);
    for (key, value) in params {
        env.insert(
            format!("PARAM_{}", key.to_uppercase()),
            EnvValue::String(value.clone()),
        );
    }
}

fn substitute_params_in_str(s: &str, params: &BTreeMap<String, String>) -> String {
    let mut result = s.to_string();
    for (key, value) in params {
        let pattern = format!("${{_param.{key}}}");
        result = result.replace(&pattern, value);
    }
    result
}

/// Rewrite a single input mapping inside a module's inner node.
///
/// - `_mod/X` references are resolved to the actual source from the module
///   node's inputs map. Returns `None` if the port is optional and not provided.
/// - Internal cross-references (`sibling_node/output`) are prefixed with the
///   module ID.
/// - Timer inputs pass through unchanged.
fn rewrite_module_input(
    input: &Input,
    module_id: &str,
    module_inputs: &BTreeMap<DataId, Input>,
    inner_node_ids: &BTreeSet<String>,
    optional_inputs: &BTreeSet<String>,
) -> eyre::Result<Option<Input>> {
    match &input.mapping {
        InputMapping::Timer { .. } | InputMapping::Logs(_) => Ok(Some(input.clone())),
        InputMapping::User(user_mapping) => {
            let source_str = user_mapping.source.to_string();

            if source_str == MODULE_INPUT_SOURCE {
                // `_mod/port_name` -> resolve from module node's inputs
                let port_name = user_mapping.output.to_string();
                match module_inputs.get(&*port_name) {
                    Some(bound_input) => {
                        // Preserve queue_size/timeout from the inner node if set,
                        // otherwise fall back to the module node's binding
                        Ok(Some(Input {
                            mapping: bound_input.mapping.clone(),
                            queue_size: input.queue_size.or(bound_input.queue_size),
                            input_timeout: input.input_timeout.or(bound_input.input_timeout),
                            queue_policy: input.queue_policy.or(bound_input.queue_policy),
                        }))
                    }
                    None if optional_inputs.contains(&port_name) => Ok(None),
                    None => bail!(
                        "module input reference `_mod/{}` not found in module node inputs",
                        port_name,
                    ),
                }
            } else if inner_node_ids.contains(&source_str) {
                // Internal cross-reference: prefix with module_id
                Ok(Some(Input {
                    mapping: InputMapping::User(UserInputMapping {
                        source: format!("{module_id}.{source_str}").into(),
                        output: user_mapping.output.clone(),
                    }),
                    queue_size: input.queue_size,
                    input_timeout: input.input_timeout,
                    queue_policy: input.queue_policy,
                }))
            } else {
                // External reference — pass through unchanged
                Ok(Some(input.clone()))
            }
        }
    }
}

/// Rewrite inputs across all nodes that reference module outputs.
///
/// If node X has input `nav_stack/cmd_vel` and `nav_stack` was a module whose
/// output `cmd_vel` is produced by inner node `controller`, rewrite to
/// `nav_stack.controller/cmd_vel`.
fn rewrite_external_refs(
    nodes: &mut [Node],
    output_maps: &BTreeMap<String, BTreeMap<String, String>>,
) -> eyre::Result<()> {
    if output_maps.is_empty() {
        return Ok(());
    }

    for node in nodes.iter_mut() {
        rewrite_inputs_map(&mut node.inputs, output_maps, &node.id)?;

        if let Some(ref mut operators) = node.operators {
            for op in &mut operators.operators {
                rewrite_inputs_map(&mut op.config.inputs, output_maps, &node.id)?;
            }
        }
        if let Some(ref mut operator) = node.operator {
            rewrite_inputs_map(&mut operator.config.inputs, output_maps, &node.id)?;
        }
        if let Some(ref mut custom) = node.custom {
            rewrite_inputs_map(&mut custom.run_config.inputs, output_maps, &node.id)?;
        }
    }
    Ok(())
}

fn rewrite_inputs_map(
    inputs: &mut BTreeMap<DataId, Input>,
    output_maps: &BTreeMap<String, BTreeMap<String, String>>,
    node_id: &NodeId,
) -> eyre::Result<()> {
    let mut new_inputs = BTreeMap::new();
    for (input_id, input) in inputs.iter() {
        let new_input = match &input.mapping {
            InputMapping::User(user_mapping) => {
                let source_str = user_mapping.source.to_string();
                if let Some(omap) = output_maps.get(&source_str) {
                    let output_str = user_mapping.output.to_string();
                    if let Some(prefixed_node) = omap.get(&output_str) {
                        Input {
                            mapping: InputMapping::User(UserInputMapping {
                                source: prefixed_node.clone().into(),
                                output: user_mapping.output.clone(),
                            }),
                            queue_size: input.queue_size,
                            input_timeout: input.input_timeout,
                            queue_policy: input.queue_policy,
                        }
                    } else {
                        bail!(
                            "node `{}` references `{}/{}` but module `{}` \
                             does not declare output `{}`",
                            node_id,
                            source_str,
                            output_str,
                            source_str,
                            output_str,
                        );
                    }
                } else {
                    input.clone()
                }
            }
            InputMapping::Timer { .. } | InputMapping::Logs(_) => input.clone(),
        };
        new_inputs.insert(input_id.clone(), new_input);
    }
    *inputs = new_inputs;
    Ok(())
}

fn load_module_file(path: &Path) -> eyre::Result<ModuleFile> {
    let metadata = std::fs::metadata(path)
        .with_context(|| format!("failed to stat module file: {}", path.display()))?;
    if metadata.len() > MAX_MODULE_FILE_SIZE {
        bail!(
            "module file too large ({} bytes, limit {}): {}",
            metadata.len(),
            MAX_MODULE_FILE_SIZE,
            path.display()
        );
    }
    let buf = std::fs::read(path)
        .with_context(|| format!("failed to read module file: {}", path.display()))?;
    serde_yaml::from_slice(&buf)
        .with_context(|| format!("failed to parse module file: {}", path.display()))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;
    use tempfile::TempDir;

    fn write_file(dir: &Path, name: &str, content: &str) -> PathBuf {
        let path = dir.join(name);
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent).unwrap();
        }
        let mut f = std::fs::File::create(&path).unwrap();
        f.write_all(content.as_bytes()).unwrap();
        path
    }

    fn parse_descriptor(yaml: &str) -> Descriptor {
        serde_yaml::from_str(yaml).unwrap()
    }

    #[test]
    fn expand_flat_passthrough() {
        let tmp = TempDir::new().unwrap();
        let base = tmp.path();

        write_file(
            base,
            "echo_module.yml",
            r#"
module:
  name: echo
  inputs: [data_in]
  outputs: [data_out]

nodes:
  - id: passthrough
    path: echo.py
    inputs:
      incoming: _mod/data_in
    outputs:
      - data_out
"#,
        );

        let desc = parse_descriptor(
            r#"
nodes:
  - id: source
    path: source.py
    outputs:
      - number

  - id: my_echo
    module: echo_module.yml
    inputs:
      data_in: source/number

  - id: sink
    path: sink.py
    inputs:
      result: my_echo/data_out
"#,
        );

        let expanded = expand_modules(&desc, base).unwrap();

        assert_eq!(expanded.nodes.len(), 3);
        let names: Vec<_> = expanded.nodes.iter().map(|n| n.id.to_string()).collect();
        assert!(names.contains(&"my_echo.passthrough".to_string()));
        assert!(!names.contains(&"my_echo".to_string()));

        let passthrough = expanded
            .nodes
            .iter()
            .find(|n| n.id.to_string() == "my_echo.passthrough")
            .unwrap();
        let incoming = &passthrough.inputs[&DataId::from("incoming".to_string())];
        match &incoming.mapping {
            InputMapping::User(m) => {
                assert_eq!(m.source.to_string(), "source");
                assert_eq!(m.output.to_string(), "number");
            }
            _ => panic!("expected user mapping"),
        }

        let sink = expanded
            .nodes
            .iter()
            .find(|n| n.id.to_string() == "sink")
            .unwrap();
        let result = &sink.inputs[&DataId::from("result".to_string())];
        match &result.mapping {
            InputMapping::User(m) => {
                assert_eq!(m.source.to_string(), "my_echo.passthrough");
                assert_eq!(m.output.to_string(), "data_out");
            }
            _ => panic!("expected user mapping"),
        }
    }

    #[test]
    fn expand_internal_cross_ref() {
        let tmp = TempDir::new().unwrap();
        let base = tmp.path();

        write_file(
            base,
            "pipeline_module.yml",
            r#"
module:
  name: pipeline
  inputs: [data_in]
  outputs: [data_out]

nodes:
  - id: stage_a
    path: a.py
    inputs:
      raw: _mod/data_in
    outputs:
      - intermediate

  - id: stage_b
    path: b.py
    inputs:
      intermediate: stage_a/intermediate
    outputs:
      - data_out
"#,
        );

        let desc = parse_descriptor(
            r#"
nodes:
  - id: src
    path: src.py
    outputs: [val]
  - id: pipe
    module: pipeline_module.yml
    inputs:
      data_in: src/val
  - id: dst
    path: dst.py
    inputs:
      result: pipe/data_out
"#,
        );

        let expanded = expand_modules(&desc, base).unwrap();

        let stage_b = expanded
            .nodes
            .iter()
            .find(|n| n.id.to_string() == "pipe.stage_b")
            .unwrap();
        let inter = &stage_b.inputs[&DataId::from("intermediate".to_string())];
        match &inter.mapping {
            InputMapping::User(m) => {
                assert_eq!(m.source.to_string(), "pipe.stage_a");
                assert_eq!(m.output.to_string(), "intermediate");
            }
            _ => panic!("expected user mapping"),
        }
    }

    #[test]
    fn expand_depth_limit() {
        let tmp = TempDir::new().unwrap();
        let base = tmp.path();

        for i in 0..=MAX_MODULE_DEPTH {
            let next = if i < MAX_MODULE_DEPTH {
                format!(
                    "  - id: inner\n    module: level{}_module.yml\n    inputs:\n      x: _mod/x",
                    i + 1
                )
            } else {
                "  - id: inner\n    path: leaf.py\n    inputs:\n      x: _mod/x\n    outputs:\n      - y".to_string()
            };

            write_file(
                base,
                &format!("level{i}_module.yml"),
                &format!(
                    r#"
module:
  name: level{i}
  inputs: [x]
  outputs: [y]

nodes:
{next}
"#
                ),
            );
        }

        let desc = parse_descriptor(
            r#"
nodes:
  - id: root
    module: level0_module.yml
    inputs:
      x: somewhere/val
"#,
        );

        let result = expand_modules(&desc, base);
        assert!(result.is_err());
        assert!(
            result
                .unwrap_err()
                .to_string()
                .contains("nesting exceeds depth limit")
        );
    }

    #[test]
    fn expand_circular_reference() {
        let tmp = TempDir::new().unwrap();
        let base = tmp.path();

        write_file(
            base,
            "self_module.yml",
            r#"
module:
  name: self_ref
  inputs: [x]
  outputs: [y]

nodes:
  - id: recurse
    module: self_module.yml
    inputs:
      x: _mod/x
"#,
        );

        let desc = parse_descriptor(
            r#"
nodes:
  - id: top
    module: self_module.yml
    inputs:
      x: somewhere/val
"#,
        );

        let result = expand_modules(&desc, base);
        assert!(result.is_err());
        let err_msg = result.unwrap_err().to_string();
        assert!(err_msg.contains("circular module reference"));
        // Feature 8: better error message with hint
        assert!(err_msg.contains("hint"));
    }

    #[test]
    fn expand_missing_module_file() {
        let tmp = TempDir::new().unwrap();
        let base = tmp.path();

        let desc = parse_descriptor(
            r#"
nodes:
  - id: broken
    module: nonexistent_module.yml
    inputs:
      x: somewhere/val
"#,
        );

        let result = expand_modules(&desc, base);
        assert!(result.is_err());
        assert!(
            result
                .unwrap_err()
                .to_string()
                .contains("nonexistent_module.yml")
        );
    }

    #[test]
    fn expand_undefined_input_port() {
        let tmp = TempDir::new().unwrap();
        let base = tmp.path();

        write_file(
            base,
            "needs_input_module.yml",
            r#"
module:
  name: needs_input
  inputs: [required_port]
  outputs: [out]

nodes:
  - id: inner
    path: inner.py
    inputs:
      x: _mod/required_port
    outputs:
      - out
"#,
        );

        let desc = parse_descriptor(
            r#"
nodes:
  - id: mod_node
    module: needs_input_module.yml
    inputs:
      wrong_name: somewhere/val
"#,
        );

        let result = expand_modules(&desc, base);
        assert!(result.is_err());
        let err_msg = result.unwrap_err().to_string();
        assert!(err_msg.contains("required_port"));
        // Feature 8: hint in error
        assert!(err_msg.contains("hint"));
    }

    #[test]
    fn expand_undefined_output_port() {
        let tmp = TempDir::new().unwrap();
        let base = tmp.path();

        write_file(
            base,
            "bad_output_module.yml",
            r#"
module:
  name: bad_output
  inputs: []
  outputs: [nonexistent]

nodes:
  - id: inner
    path: inner.py
    outputs:
      - something_else
"#,
        );

        let desc = parse_descriptor(
            r#"
nodes:
  - id: mod_node
    module: bad_output_module.yml
"#,
        );

        let result = expand_modules(&desc, base);
        assert!(result.is_err());
        assert!(result.unwrap_err().to_string().contains("nonexistent"));
    }

    #[test]
    fn expand_no_modules_passthrough() {
        let desc = parse_descriptor(
            r#"
nodes:
  - id: a
    path: a.py
    outputs: [x]
  - id: b
    path: b.py
    inputs:
      x: a/x
"#,
        );

        let tmp = TempDir::new().unwrap();
        let expanded = expand_modules(&desc, tmp.path()).unwrap();
        assert_eq!(expanded.nodes.len(), 2);
        assert_eq!(expanded.nodes[0].id.to_string(), "a");
        assert_eq!(expanded.nodes[1].id.to_string(), "b");
    }

    #[test]
    fn expand_multiple_instances() {
        let tmp = TempDir::new().unwrap();
        let base = tmp.path();

        write_file(
            base,
            "filter_module.yml",
            r#"
module:
  name: filter
  inputs: [raw]
  outputs: [filtered]

nodes:
  - id: proc
    path: filter.py
    inputs:
      data: _mod/raw
    outputs:
      - filtered
"#,
        );

        let desc = parse_descriptor(
            r#"
nodes:
  - id: cam1
    path: cam.py
    outputs: [frame]
  - id: cam2
    path: cam.py
    outputs: [frame]
  - id: filter1
    module: filter_module.yml
    inputs:
      raw: cam1/frame
  - id: filter2
    module: filter_module.yml
    inputs:
      raw: cam2/frame
  - id: merger
    path: merge.py
    inputs:
      a: filter1/filtered
      b: filter2/filtered
"#,
        );

        let expanded = expand_modules(&desc, base).unwrap();

        let names: Vec<_> = expanded.nodes.iter().map(|n| n.id.to_string()).collect();
        assert!(names.contains(&"filter1.proc".to_string()));
        assert!(names.contains(&"filter2.proc".to_string()));
        assert_eq!(expanded.nodes.len(), 5);

        let merger = expanded
            .nodes
            .iter()
            .find(|n| n.id.to_string() == "merger")
            .unwrap();
        let a_input = &merger.inputs[&DataId::from("a".to_string())];
        match &a_input.mapping {
            InputMapping::User(m) => assert_eq!(m.source.to_string(), "filter1.proc"),
            _ => panic!("expected user mapping"),
        }
        let b_input = &merger.inputs[&DataId::from("b".to_string())];
        match &b_input.mapping {
            InputMapping::User(m) => assert_eq!(m.source.to_string(), "filter2.proc"),
            _ => panic!("expected user mapping"),
        }
    }

    #[test]
    fn expand_nested_modules() {
        let tmp = TempDir::new().unwrap();
        let base = tmp.path();

        write_file(
            base,
            "inner_module.yml",
            r#"
module:
  name: inner
  inputs: [x]
  outputs: [y]

nodes:
  - id: leaf
    path: leaf.py
    inputs:
      x: _mod/x
    outputs:
      - y
"#,
        );

        write_file(
            base,
            "outer_module.yml",
            r#"
module:
  name: outer
  inputs: [a]
  outputs: [y]

nodes:
  - id: nested
    module: inner_module.yml
    inputs:
      x: _mod/a
"#,
        );

        let desc = parse_descriptor(
            r#"
nodes:
  - id: src
    path: src.py
    outputs: [val]
  - id: wrapper
    module: outer_module.yml
    inputs:
      a: src/val
"#,
        );

        let expanded = expand_modules(&desc, base).unwrap();
        let names: Vec<_> = expanded.nodes.iter().map(|n| n.id.to_string()).collect();
        assert!(names.contains(&"wrapper.nested.leaf".to_string()));
    }

    // ---- Feature 3: optional inputs ----

    #[test]
    fn expand_optional_input_provided() {
        let tmp = TempDir::new().unwrap();
        let base = tmp.path();

        write_file(
            base,
            "opt_module.yml",
            r#"
module:
  name: opt
  inputs: [required]
  inputs_optional: [config]
  outputs: [out]

nodes:
  - id: worker
    path: worker.py
    inputs:
      data: _mod/required
      cfg: _mod/config
    outputs:
      - out
"#,
        );

        // Provide both required and optional
        let desc = parse_descriptor(
            r#"
nodes:
  - id: src
    path: src.py
    outputs: [data, cfg]
  - id: m
    module: opt_module.yml
    inputs:
      required: src/data
      config: src/cfg
"#,
        );

        let expanded = expand_modules(&desc, base).unwrap();
        let worker = expanded
            .nodes
            .iter()
            .find(|n| n.id.to_string() == "m.worker")
            .unwrap();
        // Both inputs should be present
        assert_eq!(worker.inputs.len(), 2);
    }

    #[test]
    fn expand_optional_input_omitted() {
        let tmp = TempDir::new().unwrap();
        let base = tmp.path();

        write_file(
            base,
            "opt_module.yml",
            r#"
module:
  name: opt
  inputs: [required]
  inputs_optional: [config]
  outputs: [out]

nodes:
  - id: worker
    path: worker.py
    inputs:
      data: _mod/required
      cfg: _mod/config
    outputs:
      - out
"#,
        );

        // Only provide required, not optional
        let desc = parse_descriptor(
            r#"
nodes:
  - id: src
    path: src.py
    outputs: [data]
  - id: m
    module: opt_module.yml
    inputs:
      required: src/data
"#,
        );

        let expanded = expand_modules(&desc, base).unwrap();
        let worker = expanded
            .nodes
            .iter()
            .find(|n| n.id.to_string() == "m.worker")
            .unwrap();
        // Only the required input should be present; optional was dropped
        assert_eq!(worker.inputs.len(), 1);
        assert!(
            worker
                .inputs
                .contains_key(&DataId::from("data".to_string()))
        );
    }

    // ---- Feature 4: params substitution ----

    #[test]
    fn expand_params_in_env() {
        let tmp = TempDir::new().unwrap();
        let base = tmp.path();

        write_file(
            base,
            "param_module.yml",
            r#"
module:
  name: parameterized
  inputs: [data]
  outputs: [out]

nodes:
  - id: proc
    path: proc.py
    inputs:
      data: _mod/data
    outputs:
      - out
"#,
        );

        let desc = parse_descriptor(
            r#"
nodes:
  - id: src
    path: src.py
    outputs: [val]
  - id: m
    module: param_module.yml
    inputs:
      data: src/val
    params:
      speed: "1.5"
      mode: turbo
"#,
        );

        let expanded = expand_modules(&desc, base).unwrap();
        let proc = expanded
            .nodes
            .iter()
            .find(|n| n.id.to_string() == "m.proc")
            .unwrap();
        let env = proc.env.as_ref().unwrap();
        // Params are injected as PARAM_<UPPERCASE_KEY>
        assert_eq!(env["PARAM_SPEED"], EnvValue::String("1.5".to_string()));
        assert_eq!(env["PARAM_MODE"], EnvValue::String("turbo".to_string()));
    }

    #[test]
    fn expand_params_in_args() {
        let tmp = TempDir::new().unwrap();
        let base = tmp.path();

        write_file(
            base,
            "args_module.yml",
            r#"
module:
  name: with_args
  inputs: [data]
  outputs: [out]

nodes:
  - id: proc
    path: proc.py
    inputs:
      data: _mod/data
    outputs:
      - out
    args: --speed ${_param.speed} --verbose
"#,
        );

        let desc = parse_descriptor(
            r#"
nodes:
  - id: src
    path: src.py
    outputs: [val]
  - id: m
    module: args_module.yml
    inputs:
      data: src/val
    params:
      speed: "2.0"
"#,
        );

        let expanded = expand_modules(&desc, base).unwrap();
        let proc = expanded
            .nodes
            .iter()
            .find(|n| n.id.to_string() == "m.proc")
            .unwrap();
        assert_eq!(proc.args.as_deref(), Some("--speed 2.0 --verbose"));
    }

    // ---- Feature 5: module-level build ----

    #[test]
    fn expand_module_build_prepended() {
        let tmp = TempDir::new().unwrap();
        let base = tmp.path();

        write_file(
            base,
            "build_module.yml",
            r#"
module:
  name: buildable
  inputs: [data]
  outputs: [out]

build: pip install -r requirements.txt

nodes:
  - id: proc
    path: proc.py
    inputs:
      data: _mod/data
    outputs:
      - out
    build: python setup.py build
"#,
        );

        let desc = parse_descriptor(
            r#"
nodes:
  - id: src
    path: src.py
    outputs: [val]
  - id: m
    module: build_module.yml
    inputs:
      data: src/val
"#,
        );

        let expanded = expand_modules(&desc, base).unwrap();
        let proc = expanded
            .nodes
            .iter()
            .find(|n| n.id.to_string() == "m.proc")
            .unwrap();
        let build = proc.build.as_deref().unwrap();
        assert!(build.starts_with("pip install -r requirements.txt"));
        assert!(build.contains("python setup.py build"));
    }

    #[test]
    fn expand_module_build_no_inner_build() {
        let tmp = TempDir::new().unwrap();
        let base = tmp.path();

        write_file(
            base,
            "build_only_module.yml",
            r#"
module:
  name: build_only
  inputs: [data]
  outputs: [out]

build: make all

nodes:
  - id: proc
    path: proc.py
    inputs:
      data: _mod/data
    outputs:
      - out
"#,
        );

        let desc = parse_descriptor(
            r#"
nodes:
  - id: src
    path: src.py
    outputs: [val]
  - id: m
    module: build_only_module.yml
    inputs:
      data: src/val
"#,
        );

        let expanded = expand_modules(&desc, base).unwrap();
        let proc = expanded
            .nodes
            .iter()
            .find(|n| n.id.to_string() == "m.proc")
            .unwrap();
        assert_eq!(proc.build.as_deref(), Some("make all"));
    }

    // ---- Feature 1: boundaries metadata ----

    #[test]
    fn expand_returns_boundaries() {
        let tmp = TempDir::new().unwrap();
        let base = tmp.path();

        write_file(
            base,
            "simple_module.yml",
            r#"
module:
  name: simple
  inputs: [x]
  outputs: [y]

nodes:
  - id: a
    path: a.py
    inputs:
      x: _mod/x
    outputs: [mid]
  - id: b
    path: b.py
    inputs:
      mid: a/mid
    outputs: [y]
"#,
        );

        let desc = parse_descriptor(
            r#"
nodes:
  - id: src
    path: src.py
    outputs: [val]
  - id: mod1
    module: simple_module.yml
    inputs:
      x: src/val
"#,
        );

        let result = expand_modules_with_boundaries(&desc, base).unwrap();
        assert!(result.boundaries.modules.contains_key("mod1"));
        let members = &result.boundaries.modules["mod1"];
        assert!(members.contains(&"mod1.a".to_string()));
        assert!(members.contains(&"mod1.b".to_string()));
    }

    // ---- Feature 9: check_module_file ----

    #[test]
    fn check_module_file_valid() {
        let tmp = TempDir::new().unwrap();
        let path = write_file(
            tmp.path(),
            "valid_module.yml",
            r#"
module:
  name: valid
  inputs: [x]
  outputs: [y]

nodes:
  - id: proc
    path: proc.py
    inputs:
      x: _mod/x
    outputs:
      - y
"#,
        );

        check_module_file(&path).unwrap();
    }

    #[test]
    fn check_module_file_bad_mod_ref() {
        let tmp = TempDir::new().unwrap();
        let path = write_file(
            tmp.path(),
            "bad_ref_module.yml",
            r#"
module:
  name: bad_ref
  inputs: [x]
  outputs: [y]

nodes:
  - id: proc
    path: proc.py
    inputs:
      x: _mod/nonexistent
    outputs:
      - y
"#,
        );

        let result = check_module_file(&path);
        assert!(result.is_err());
        assert!(result.unwrap_err().to_string().contains("nonexistent"));
    }

    #[test]
    fn check_module_file_bad_output() {
        let tmp = TempDir::new().unwrap();
        let path = write_file(
            tmp.path(),
            "bad_out_module.yml",
            r#"
module:
  name: bad_out
  inputs: []
  outputs: [missing]

nodes:
  - id: proc
    path: proc.py
    outputs:
      - other
"#,
        );

        let result = check_module_file(&path);
        assert!(result.is_err());
        assert!(result.unwrap_err().to_string().contains("missing"));
    }

    // ---- Security tests ----

    #[test]
    fn reject_absolute_module_path() {
        let desc = parse_descriptor(
            r#"
nodes:
  - id: evil
    module: /etc/passwd
"#,
        );
        let tmp = TempDir::new().unwrap();
        let result = expand_modules(&desc, tmp.path());
        assert!(result.is_err());
        let msg = result.unwrap_err().to_string();
        assert!(msg.contains("must be relative"), "got: {msg}");
    }

    #[test]
    fn reject_path_traversal_module() {
        let tmp = TempDir::new().unwrap();
        let base = tmp.path();

        // Create a module outside the base dir
        let parent = base.parent().unwrap();
        write_file(parent, "escape_module.yml", "module:\n  name: x\nnodes: []");

        let desc = parse_descriptor(
            r#"
nodes:
  - id: evil
    module: ../escape_module.yml
"#,
        );
        let result = expand_modules(&desc, base);
        assert!(result.is_err());
        let msg = result.unwrap_err().to_string();
        assert!(msg.contains("escapes"), "got: {msg}");
    }

    #[test]
    fn reject_invalid_param_key() {
        let tmp = TempDir::new().unwrap();
        let base = tmp.path();

        write_file(
            base,
            "param_mod.yml",
            r#"
module:
  name: p
  inputs: [x]
  outputs: [y]

nodes:
  - id: n
    path: n.py
    inputs:
      x: _mod/x
    outputs: [y]
"#,
        );

        let desc = parse_descriptor(
            r#"
nodes:
  - id: src
    path: src.py
    outputs: [v]
  - id: m
    module: param_mod.yml
    inputs:
      x: src/v
    params:
      "bad}key": value
"#,
        );
        let result = expand_modules(&desc, base);
        assert!(result.is_err());
        let msg = result.unwrap_err().to_string();
        assert!(msg.contains("invalid param key"), "got: {msg}");
    }

    #[test]
    fn reject_duplicate_node_ids() {
        let tmp = TempDir::new().unwrap();
        let base = tmp.path();

        write_file(
            base,
            "dup_module.yml",
            r#"
module:
  name: dup
  inputs: []
  outputs: [y]

nodes:
  - id: inner
    path: inner.py
    outputs: [y]
"#,
        );

        // Top-level node named "m.inner" collides with module expansion
        let desc = parse_descriptor(
            r#"
nodes:
  - id: m.inner
    path: other.py
    outputs: [z]
  - id: m
    module: dup_module.yml
"#,
        );
        let result = expand_modules(&desc, base);
        assert!(result.is_err());
        let msg = result.unwrap_err().to_string();
        assert!(msg.contains("duplicate node ID"), "got: {msg}");
    }

    #[test]
    fn param_override_precedence() {
        let tmp = TempDir::new().unwrap();
        let base = tmp.path();

        write_file(
            base,
            "override_mod.yml",
            r#"
module:
  name: override
  inputs: [x]
  outputs: [y]

nodes:
  - id: proc
    path: proc.py
    inputs:
      x: _mod/x
    outputs: [y]
    env:
      PARAM_SPEED: "default_value"
"#,
        );

        let desc = parse_descriptor(
            r#"
nodes:
  - id: src
    path: src.py
    outputs: [v]
  - id: m
    module: override_mod.yml
    inputs:
      x: src/v
    params:
      speed: "caller_value"
"#,
        );

        let expanded = expand_modules(&desc, base).unwrap();
        let proc = expanded
            .nodes
            .iter()
            .find(|n| n.id.to_string() == "m.proc")
            .unwrap();
        let env = proc.env.as_ref().unwrap();
        // Caller params should override inner default
        assert_eq!(
            env["PARAM_SPEED"],
            EnvValue::String("caller_value".to_string())
        );
    }
}
