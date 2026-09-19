use dora_message::{
    config::{Input, InputMapping, UserInputMapping, format_duration},
    descriptor::{CoreNodeKind, OperatorDefinition},
    id::{DataId, NodeId},
};

use super::{CustomNode, ModuleBoundaries, ResolvedNode, RuntimeNode, SINGLE_OPERATOR_DEFAULT_ID};
use std::{
    collections::{BTreeMap, BTreeSet, HashMap, HashSet},
    fmt::Write as _,
    time::Duration,
};

pub fn visualize_nodes_with_boundaries(
    nodes: &BTreeMap<NodeId, ResolvedNode>,
    boundaries: &ModuleBoundaries,
) -> String {
    let mut flowchart = "flowchart TB\n".to_owned();
    let mut all_nodes = HashMap::new();

    // Track which nodes are inside a module subgraph
    let mut module_nodes: HashSet<&str> = HashSet::new();
    for members in boundaries.modules.values() {
        for m in members {
            module_nodes.insert(m.as_str());
        }
    }

    // Render module subgraphs first
    for (module_id, members) in &boundaries.modules {
        // Sanitize ID for Mermaid (dots are invalid in subgraph IDs)
        let safe_id = module_id.replace('.', "_");
        writeln!(flowchart, "subgraph {safe_id} [{module_id}]").unwrap();
        for node in nodes.values() {
            let node_id_str: &str = node.id.as_ref();
            if members.iter().any(|m| m == node_id_str) {
                visualize_node(node, &mut flowchart);
                all_nodes.insert(&node.id, node);
            }
        }
        flowchart.push_str("end\n");
    }

    // Render non-module nodes
    for node in nodes.values() {
        let node_id_str: &str = node.id.as_ref();
        if !module_nodes.contains(node_id_str) {
            visualize_node(node, &mut flowchart);
            all_nodes.insert(&node.id, node);
        }
    }

    let dora_timers = collect_dora_timers(nodes);
    if !dora_timers.is_empty() {
        writeln!(flowchart, "subgraph ___dora___ [dora]").unwrap();
        writeln!(flowchart, "  subgraph ___timer_timer___ [timer]").unwrap();
        for interval in dora_timers {
            let duration = format_duration(interval);
            writeln!(flowchart, "    dora/timer/{duration}[\\{duration}/]").unwrap();
        }
        flowchart.push_str("  end\n");
        flowchart.push_str("end\n");
    }

    for node in nodes.values() {
        visualize_node_inputs(node, &mut flowchart, &all_nodes)
    }

    flowchart
}

pub fn collect_dora_timers(nodes: &BTreeMap<NodeId, ResolvedNode>) -> BTreeSet<Duration> {
    let mut dora_timers = BTreeSet::new();
    for node in nodes.values() {
        match &node.kind {
            CoreNodeKind::Runtime(node) => {
                for operator in &node.operators {
                    collect_dora_nodes(operator.config.inputs.values(), &mut dora_timers);
                }
            }
            CoreNodeKind::Custom(node) => {
                collect_dora_nodes(node.run_config.inputs.values(), &mut dora_timers);
            }
        }
    }
    dora_timers
}

fn collect_dora_nodes(
    values: std::collections::btree_map::Values<DataId, Input>,
    dora_timers: &mut BTreeSet<Duration>,
) {
    for input in values {
        match &input.mapping {
            InputMapping::User(_) | InputMapping::Logs(_) => {}
            InputMapping::Timer { interval } => {
                dora_timers.insert(*interval);
            }
        }
    }
}

/// Escape Mermaid-significant characters in free-form label text (e.g. a node
/// `description`) so it stays well-formed inside a quoted Mermaid label.
///
/// Node and data IDs are charset-validated to be Mermaid-safe, but a
/// `description` is unrestricted, so a `"` (or `<`, `>`, `#`, `&`) would
/// otherwise corrupt the generated diagram. Mermaid accepts HTML entity codes
/// written with a leading `#` in place of `&`, so we emit those. `#` is escaped
/// first so the entities we introduce are not re-processed.
///
/// A raw newline is escaped last: the label is written into a single-line
/// Mermaid statement, so a newline (e.g. from a YAML block-scalar
/// `description`) would split the statement across lines and make the whole
/// document invalid. It becomes a `<br/>` line break — inserted *after* the
/// `<`/`>` escaping above so this markup is not itself escaped. A `\r` is
/// dropped first so a CRLF source does not leave a stray carriage return
/// (nor produce a doubled break).
fn escape_mermaid_label(text: &str) -> String {
    text.replace('#', "#35;")
        .replace('&', "#amp;")
        .replace('"', "#quot;")
        .replace('<', "#lt;")
        .replace('>', "#gt;")
        .replace('\r', "")
        .replace('\n', "<br/>")
}

fn visualize_node(node: &ResolvedNode, flowchart: &mut String) {
    let node_id = &node.id;
    let description = if let Some(desc) = &node.description {
        format!("<hr/>*{}*", escape_mermaid_label(desc))
    } else {
        "".to_string()
    };

    match &node.kind {
        CoreNodeKind::Custom(node) => visualize_custom_node(node_id, description, node, flowchart),
        CoreNodeKind::Runtime(RuntimeNode { operators, .. }) => {
            visualize_runtime_node(node_id, description, operators, flowchart)
        }
    }
}

fn visualize_custom_node(
    node_id: &NodeId,
    description: String,
    node: &CustomNode,
    flowchart: &mut String,
) {
    if node.run_config.inputs.is_empty() {
        // source node
        writeln!(flowchart, "  {node_id}[\\\"**{node_id}**{description}\"/]").unwrap();
    } else if node.run_config.outputs.is_empty() {
        // sink node
        writeln!(flowchart, "  {node_id}[/\"**{node_id}**{description}\"\\]").unwrap();
    } else {
        // normal node
        writeln!(flowchart, "  {node_id}[\"**{node_id}**{description}\"]").unwrap();
    }
}

fn visualize_runtime_node(
    node_id: &NodeId,
    description: String,
    operators: &[OperatorDefinition],
    flowchart: &mut String,
) {
    if operators.len() == 1 && operators[0].id.as_ref() == SINGLE_OPERATOR_DEFAULT_ID {
        let operator = &operators[0];
        // The node's mermaid id must match how edges reference this operator,
        // which `visualize_node_inputs` builds as `{node_id}/{operator.id}`.
        // Use the operator id here rather than a second literal `op` so the two
        // can never drift apart if `SINGLE_OPERATOR_DEFAULT_ID` changes.
        let operator_id = &operator.id;
        // Only quote the label (and append the description) when there is a
        // description, so a node without one renders exactly as before. The
        // description already carries its own `<hr/>` separator and is escaped
        // for the quoted Mermaid label by `visualize_node`.
        let label = if description.is_empty() {
            node_id.to_string()
        } else {
            format!("\"{node_id}{description}\"")
        };
        // single operator node
        if operator.config.inputs.is_empty() {
            // source node
            writeln!(flowchart, "  {node_id}/{operator_id}[\\{label}/]").unwrap();
        } else if operator.config.outputs.is_empty() {
            // sink node
            writeln!(flowchart, "  {node_id}/{operator_id}[/{label}\\]").unwrap();
        } else {
            // normal node
            writeln!(flowchart, "  {node_id}/{operator_id}[{label}]").unwrap();
        }
    } else {
        // Sanitize the id for Mermaid the same way the module-subgraph path
        // above does (dots are invalid in subgraph IDs) and keep the original
        // id as the readable label. Node ids legally contain `.`, and any
        // runtime node inside a module is prefixed with `{module_id}.` during
        // expansion, so an unsanitized `subgraph {node_id}` here emits an
        // invalid Mermaid document for those nodes. The contained operator
        // nodes keep their raw `{node_id}/{operator_id}` ids (dots are valid in
        // node ids, only subgraph ids), so edges still line up.
        let safe_id = node_id.as_ref().replace('.', "_");
        // Quote the subgraph title only when a description must be appended, so
        // a node without one keeps its previous unquoted title (same shape as
        // the single-operator `label` above).
        let title = if description.is_empty() {
            node_id.to_string()
        } else {
            format!("\"{node_id}{description}\"")
        };
        writeln!(flowchart, "subgraph {safe_id} [{title}]").unwrap();
        for operator in operators {
            let operator_id = &operator.id;
            if operator.config.inputs.is_empty() {
                // source operator
                writeln!(flowchart, "  {node_id}/{operator_id}[\\{operator_id}/]").unwrap();
            } else if operator.config.outputs.is_empty() {
                // sink operator
                writeln!(flowchart, "  {node_id}/{operator_id}[/{operator_id}\\]").unwrap();
            } else {
                // normal operator
                writeln!(flowchart, "  {node_id}/{operator_id}[{operator_id}]").unwrap();
            }
        }
        flowchart.push_str("end\n");
    }
}

fn visualize_node_inputs(
    node: &ResolvedNode,
    flowchart: &mut String,
    nodes: &HashMap<&NodeId, &ResolvedNode>,
) {
    let node_id = &node.id;
    match &node.kind {
        CoreNodeKind::Custom(node) => {
            visualize_inputs(node_id.as_ref(), &node.run_config.inputs, flowchart, nodes)
        }
        CoreNodeKind::Runtime(RuntimeNode { operators, .. }) => {
            for operator in operators {
                visualize_inputs(
                    &format!("{node_id}/{}", operator.id),
                    &operator.config.inputs,
                    flowchart,
                    nodes,
                )
            }
        }
    }
}

fn visualize_inputs(
    target: &str,
    inputs: &BTreeMap<DataId, Input>,
    flowchart: &mut String,
    nodes: &HashMap<&NodeId, &ResolvedNode>,
) {
    for (input_id, input) in inputs {
        match &input.mapping {
            mapping @ (InputMapping::Timer { .. } | InputMapping::Logs(_)) => {
                writeln!(flowchart, "  {mapping} -- {input_id} --> {target}").unwrap();
            }
            InputMapping::User(mapping) => {
                visualize_user_mapping(mapping, target, nodes, input_id, flowchart)
            }
        }
    }
}

fn visualize_user_mapping(
    mapping: &UserInputMapping,
    target: &str,
    nodes: &HashMap<&NodeId, &ResolvedNode>,
    input_id: &DataId,
    flowchart: &mut String,
) {
    let UserInputMapping { source, output } = mapping;
    let mut source_found = false;
    if let Some(source_node) = nodes.get(source) {
        match &source_node.kind {
            CoreNodeKind::Custom(custom_node) => {
                if custom_node.run_config.outputs.contains(output) {
                    let type_label = custom_node
                        .run_config
                        .output_types
                        .get(output)
                        .map(|t| format_type_label(t))
                        .unwrap_or_default();
                    let data = if output == input_id {
                        format!("{output}{type_label}")
                    } else {
                        format!("{output} as {input_id}{type_label}")
                    };
                    writeln!(flowchart, "  {source} -- {data} --> {target}").unwrap();
                    source_found = true;
                }
            }
            CoreNodeKind::Runtime(RuntimeNode { operators, .. }) => {
                let (operator_id, output) = output.split_once('/').unwrap_or(("", output));
                if let Some(operator) = operators.iter().find(|o| o.id.as_ref() == operator_id)
                    && operator.config.outputs.contains(output)
                {
                    let type_label = operator
                        .config
                        .output_types
                        .get(&DataId::from(output.to_owned()))
                        .map(|t| format_type_label(t))
                        .unwrap_or_default();
                    let data = if output == input_id.as_str() {
                        format!("{output}{type_label}")
                    } else {
                        format!("{output} as {input_id}{type_label}")
                    };
                    writeln!(flowchart, "  {source}/{operator_id} -- {data} --> {target}").unwrap();
                    source_found = true;
                }
            }
        }
    }
    if !source_found {
        writeln!(flowchart, "  missing>missing] -- {input_id} --> {target}").unwrap();
    }
}

/// Format a type URN as a short label for graph edges.
/// Extracts the type name from the URN (e.g. `std/media/v1/Image` -> ` [Image]`).
fn format_type_label(urn: &str) -> String {
    let short = crate::types::urn_short_name(urn);
    format!(" [{short}]")
}

#[cfg(test)]
mod tests {
    use super::{escape_mermaid_label, visualize_nodes_with_boundaries};
    use crate::descriptor::{Descriptor, DescriptorExt, ModuleBoundaries};

    /// End-to-end guard through `visualize_nodes_with_boundaries`: a `"` in a
    /// node `description` must reach the emitted Mermaid label as the entity
    /// `#quot;`, never as a bare `"` that would prematurely close the quoted
    /// label. Reverting the `escape_mermaid_label` call in `visualize_node`
    /// to interpolate the raw `desc` fails this test (the unit tests above,
    /// which call the helper directly, would not catch that regression).
    #[test]
    fn description_with_double_quote_is_escaped_in_output() {
        let yaml = r#"
nodes:
  - id: camera
    path: ./camera
    description: 'Captures "raw" frames'
    outputs:
      - image
"#;
        let desc: Descriptor = serde_yaml::from_str(yaml).expect("parse");
        let resolved = desc.resolve_aliases_and_set_defaults().expect("resolve");
        let flowchart = visualize_nodes_with_boundaries(&resolved, &ModuleBoundaries::default());

        assert!(
            flowchart.contains("Captures #quot;raw#quot; frames"),
            "description `\"` must be escaped to `#quot;`; got:\n{flowchart}"
        );
        assert!(
            !flowchart.contains(r#""raw""#),
            "a bare quoted substring corrupts the Mermaid label; got:\n{flowchart}"
        );
    }

    /// A YAML block-scalar `description` carries raw newlines. The label is
    /// written into a single-line Mermaid statement, so a raw `\n` would split
    /// the statement and invalidate the document; it must become a `<br/>`
    /// break and the node's statement must stay on one line.
    #[test]
    fn multiline_description_becomes_single_line_with_br() {
        let yaml = "
nodes:
  - id: camera
    path: ./camera
    description: |
      Captures frames
      from the front camera
    outputs:
      - image
";
        let desc: Descriptor = serde_yaml::from_str(yaml).expect("parse");
        let resolved = desc.resolve_aliases_and_set_defaults().expect("resolve");
        let flowchart = visualize_nodes_with_boundaries(&resolved, &ModuleBoundaries::default());

        assert!(
            flowchart.contains("Captures frames<br/>from the front camera"),
            "a newline in the description must become a `<br/>`; got:\n{flowchart}"
        );
        // Every node statement must be a single line: no statement line may
        // contain the mid-description text without also closing the label.
        let split_line = flowchart
            .lines()
            .any(|l| l.trim() == "from the front camera");
        assert!(
            !split_line,
            "the description must not split the Mermaid statement across lines; got:\n{flowchart}"
        );
    }

    /// A single-`operator:` node is a `Runtime` node, and its `description`
    /// must reach the Mermaid label the same way a custom node's does — the
    /// `_description` parameter used to be discarded, silently dropping the
    /// field for operator nodes. Reverting the fix (dropping the description in
    /// `visualize_runtime_node`) fails this test.
    #[test]
    fn single_operator_node_description_reaches_the_label() {
        let yaml = r#"
nodes:
  - id: detector
    description: 'Finds "objects"'
    operator:
      python: detector.py
      inputs:
        image: camera/image
      outputs:
        - bbox
  - id: camera
    path: ./camera
    outputs:
      - image
"#;
        let desc: Descriptor = serde_yaml::from_str(yaml).expect("parse");
        let resolved = desc.resolve_aliases_and_set_defaults().expect("resolve");
        let flowchart = visualize_nodes_with_boundaries(&resolved, &ModuleBoundaries::default());

        // The single operator defaults to id `op`, and the label carries the
        // escaped description inside a quoted, `<hr/>`-separated Mermaid label.
        assert!(
            flowchart.contains(r#"detector/op["detector<hr/>*Finds #quot;objects#quot;*"]"#),
            "the operator node's description must reach its Mermaid label; got:\n{flowchart}"
        );
    }

    /// A runtime node without a description keeps its previous unquoted label,
    /// so the fix does not change existing `dora graph` output for the common
    /// no-description case.
    #[test]
    fn operator_node_without_description_stays_unquoted() {
        let yaml = r#"
nodes:
  - id: detector
    operator:
      python: detector.py
      inputs:
        image: camera/image
      outputs:
        - bbox
  - id: camera
    path: ./camera
    outputs:
      - image
"#;
        let desc: Descriptor = serde_yaml::from_str(yaml).expect("parse");
        let resolved = desc.resolve_aliases_and_set_defaults().expect("resolve");
        let flowchart = visualize_nodes_with_boundaries(&resolved, &ModuleBoundaries::default());

        assert!(
            flowchart.contains("detector/op[detector]"),
            "a description-less operator node must render unquoted as before; got:\n{flowchart}"
        );
    }

    /// A multi-`operators:` node renders as a subgraph; its node-level
    /// `description` must appear in the subgraph title (it was dropped before).
    #[test]
    fn multi_operator_subgraph_title_carries_the_description() {
        let yaml = r#"
nodes:
  - id: pipeline
    description: Two stage
    operators:
      - id: stage1
        python: s1.py
        outputs:
          - mid
      - id: stage2
        python: s2.py
        inputs:
          mid: pipeline/stage1/mid
        outputs:
          - out
"#;
        let desc: Descriptor = serde_yaml::from_str(yaml).expect("parse");
        let resolved = desc.resolve_aliases_and_set_defaults().expect("resolve");
        let flowchart = visualize_nodes_with_boundaries(&resolved, &ModuleBoundaries::default());

        assert!(
            flowchart.contains(r#"subgraph pipeline ["pipeline<hr/>*Two stage*"]"#),
            "the subgraph title must carry the node description; got:\n{flowchart}"
        );
    }

    #[test]
    fn escape_mermaid_label_passes_plain_text_through() {
        assert_eq!(
            escape_mermaid_label("Captures raw frames"),
            "Captures raw frames"
        );
    }

    #[test]
    fn escape_mermaid_label_escapes_double_quote() {
        // A `"` would otherwise prematurely close the quoted Mermaid label.
        assert_eq!(
            escape_mermaid_label(r#"Captures "raw" frames"#),
            "Captures #quot;raw#quot; frames"
        );
    }

    #[test]
    fn escape_mermaid_label_escapes_all_significant_chars() {
        assert_eq!(
            escape_mermaid_label(r#"a & b < c > d " e # f"#),
            "a #amp; b #lt; c #gt; d #quot; e #35; f"
        );
    }

    #[test]
    fn escape_mermaid_label_does_not_double_escape_entities() {
        // The `#` from an introduced entity must not be re-escaped into `#35;`.
        assert_eq!(escape_mermaid_label(r#"""#), "#quot;");
        assert!(!escape_mermaid_label(r#"""#).contains("#35;"));
    }
}
