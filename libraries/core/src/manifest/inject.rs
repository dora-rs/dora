//! Manifest → descriptor contract injection (spec §6.2, P1.4).
//!
//! A `dora-node.yml` sitting next to a plain `path:` node is picked up at
//! validate/build time: the manifest's typed ports surface as the node's
//! `input_types`/`output_types` descriptor annotations (the dataflow author's
//! own annotations win), its custom types are registered, and the dataflow's
//! wiring is checked against the declared ports. Local development thereby
//! gets the same contract validation as hub consumption (UC11).
//!
//! Scope (P1.4): injection runs in `dora build` and `dora validate` only —
//! compose-time checking. The injected annotations do not reach the runtime
//! first-message type check or `dora graph`, which read the descriptor
//! through paths that don't inject (deliberate for now; see plan §6.2).

use std::path::{Component, Path, PathBuf};

use dora_message::{descriptor::Descriptor, id::NodeId};

use super::{
    MANIFEST_FILENAME, NodeManifest,
    validate::{check_shipped_type_urn, sanitize},
};
use crate::{descriptor::source_is_url, types::TypeRegistry};

/// Outcome of scanning a dataflow for adjacent node manifests.
#[derive(Debug, Default)]
pub struct InjectionResult {
    /// Human-readable notes about applied injections.
    pub notes: Vec<String>,
    /// Problems found; callers treat these like type warnings
    /// (printed, and fatal under `strict_types`).
    pub warnings: Vec<String>,
}

/// Scan a dataflow for `path:` nodes with an adjacent `dora-node.yml` and
/// inject the manifest contracts into the descriptor (spec §6.2).
///
/// Custom types shipped by matched manifests are registered into `registry`
/// so the subsequent type check can resolve them.
pub fn inject_adjacent_manifests(
    dataflow: &mut Descriptor,
    working_dir: &Path,
    registry: &mut TypeRegistry,
) -> InjectionResult {
    let mut result = InjectionResult::default();

    // Pass 1: locate each node's manifest.
    let mut matched = Vec::new();
    for (idx, node) in dataflow.nodes.iter().enumerate() {
        // Only plain local-path nodes: URL paths are downloaded artifacts,
        // git sources are not cloned yet at this stage, and runtime/operator/
        // ros2 nodes have no node-level executable.
        let Some(path) = node.path.as_deref() else {
            continue;
        };
        if source_is_url(path)
            || node.git.is_some()
            || node.operators.is_some()
            || node.operator.is_some()
            || node.ros2.is_some()
        {
            continue;
        }
        if let Some(found) = find_manifest_for(path, working_dir, &node.id, &mut result) {
            matched.push((idx, found));
        }
    }

    // Register shipped types before validating ports, so a manifest can
    // reference a type shipped by another node regardless of node order.
    //
    // Two gates must pass before a type reaches the *shared* registry, where a
    // sibling node's port check can rely on it:
    //  - namespace ownership — an invalid manifest (e.g. an `acme` node
    //    shipping a `beta/…` URN) must not grant a definition another node
    //    relies on; ownership is intrinsic, so it is safe to check up front.
    //  - body-resolvability — every field type must resolve, so a sibling that
    //    ships `acme/foo/v1/X` with an unresolvable field can't make a
    //    *referencing* node's port check pass against a type that can never be
    //    materialized into an Arrow schema.
    // Field bodies may reference *other* shipped types, so resolvability is
    // checked against a scratch registry. Admission is a *fixpoint*: a
    // candidate enters `scratch` only once its `arrow:` discriminant is known
    // and every field type resolves against what is already admitted. A type
    // referencing a body-invalid sibling is therefore never admitted, and a
    // third type referencing *it* can't be admitted either — the gate is
    // transitively tight (a plain "insert every candidate first" pass would
    // let `field_type_resolves` see a body-invalid sibling by key presence and
    // wrongly admit its referent). Already-present URNs are left untouched:
    // `types/` definitions and same-manifest re-ships win.
    let mut scratch = registry.clone();
    let mut candidates = Vec::new();
    for (_, (_, manifest)) in &matched {
        for (urn, def) in &manifest.types {
            if check_shipped_type_urn(urn, &manifest.namespace).is_none() {
                candidates.push((urn, def));
            }
        }
    }
    loop {
        let mut admitted = false;
        for &(urn, def) in &candidates {
            if scratch.resolve(urn).is_some() {
                continue;
            }
            // body-valid = a known `arrow:` discriminant AND every field type
            // resolves; matches `NodeManifest::validate`.
            let body_valid = crate::types::is_known_arrow_type(&def.arrow)
                && def
                    .fields
                    .iter()
                    .all(|f| scratch.field_type_resolves(&f.r#type));
            if body_valid {
                scratch.insert_type(urn.clone(), def.clone());
                admitted = true;
            }
        }
        if !admitted {
            break;
        }
    }
    // Register the survivors into the shared registry, skipping URNs already
    // present so preserved definitions win. Register the *admitted* definition
    // from `scratch`, not the candidate's own `def`: `candidates` is not
    // deduplicated by URN, so when two same-namespace manifests ship the same
    // URN, `scratch.resolve(urn).is_some()` only proves that *some* body-valid
    // definition was admitted — the first-listed candidate's `def` may be the
    // body-invalid duplicate, which must not reach the shared registry (#2599).
    for &(urn, _def) in &candidates {
        if registry.resolve(urn).is_none()
            && let Some(admitted) = scratch.resolve(urn)
        {
            let _ = registry.add_user_type(urn, admitted.clone());
        }
    }

    // Pass 2: validate each manifest against the full registry and inject.
    for (idx, (manifest_path, manifest)) in matched {
        apply_manifest_contracts(
            &mut dataflow.nodes[idx],
            &manifest,
            &manifest_path.display().to_string(),
            registry,
            &mut result,
        );
    }
    result
}

/// Find the manifest governing `path` by walking up from the executable's
/// directory. A manifest applies iff its `entrypoint` names the same file —
/// `<manifest dir>/<entrypoint> == <working_dir>/<path>` — which makes the
/// association exact for build outputs like `target/release/<bin>` where the
/// manifest sits at the node's source root, not next to the binary.
fn find_manifest_for(
    path: &str,
    working_dir: &Path,
    node_id: &NodeId,
    result: &mut InjectionResult,
) -> Option<(PathBuf, NodeManifest)> {
    // A bare `path:` (no separator) is still a local file the daemon resolves
    // as `working_dir/<path>` (a console script or a local executable next to
    // `dora-node.yml`), so it is not skipped: `containment_roots` roots it at
    // the working dir and the walk below matches an adjacent manifest whose
    // entrypoint names it. A path that escapes the working dir returns `None`.
    let Some((full, working_dir)) = containment_roots(path, working_dir) else {
        // absolute or `..`-escaping paths point outside the dataflow tree —
        // never walk unrelated directories looking for a manifest. (Env-var
        // paths like `$HOME/bin/node` also land here or miss below: the
        // daemon expands them at spawn time, injection does not.)
        return None;
    };
    for dir in full.parent()?.ancestors() {
        if !dir.starts_with(&working_dir) {
            break;
        }
        let candidate = dir.join(MANIFEST_FILENAME);
        if candidate.is_file() {
            let manifest = match NodeManifest::read(&candidate) {
                Ok(manifest) => manifest,
                Err(err) => {
                    result.warnings.push(sanitize(&format!(
                        "node \"{node_id}\": failed to read {}: {err:#}",
                        candidate.display()
                    )));
                    return None;
                }
            };
            // manifests are written once and consumed on any platform —
            // treat `\` in the entrypoint as a separator everywhere
            let entrypoint = manifest.entrypoint.replace('\\', "/");
            if normalize(&dir.join(entrypoint)) == full {
                return Some((candidate, manifest));
            }
            // A bare `path:` (console script) commonly shares its working dir
            // with an unrelated node's manifest, so a mismatch there is not a
            // mistake — warn only when the path has a directory component that
            // strongly associates it with the manifest it sits under.
            if path.contains('/') || path.contains('\\') {
                result.warnings.push(sanitize(&format!(
                    "node \"{node_id}\": {} has entrypoint `{}`, which does not name \
                     the node's path `{path}` — contracts not injected",
                    candidate.display(),
                    manifest.entrypoint,
                )));
            }
            return None;
        }
    }
    None
}

/// Compute the normalized executable path and working-dir root used for the
/// manifest walk, or `None` if the node path escapes the working directory.
///
/// The working dir is absolutized first: a relative working dir like `.`
/// (the default when the dataflow sits in the invocation directory) would
/// otherwise normalize to an empty prefix, which trivially "contains" every
/// path — including absolute ones — and lets leading `..` components be
/// silently dropped instead of detected as escapes.
fn containment_roots(path: &str, working_dir: &Path) -> Option<(PathBuf, PathBuf)> {
    let working_dir = if working_dir.is_absolute() {
        normalize(working_dir)
    } else {
        let cwd = std::env::current_dir().ok()?;
        normalize(&cwd.join(working_dir))
    };
    let full = normalize(&working_dir.join(path));
    if full.starts_with(&working_dir) {
        Some((full, working_dir))
    } else {
        None
    }
}

/// Lexically normalize a path (collapse `.` and resolve `..`) without
/// touching the filesystem — the executable may not be built yet.
fn normalize(path: &Path) -> PathBuf {
    let mut out = PathBuf::new();
    for component in path.components() {
        match component {
            Component::CurDir => {}
            Component::ParentDir => {
                out.pop();
            }
            other => out.push(other),
        }
    }
    out
}

/// Apply a manifest's contracts to a descriptor node: validate the
/// manifest, check the dataflow's wiring against the declared ports, and
/// inject port types (the dataflow author's own annotations win).
/// `source_label` names where the manifest came from in messages (a file
/// path for adjacent manifests, a `namespace/name@version` for hub packages).
pub fn apply_manifest_contracts(
    node: &mut dora_message::descriptor::Node,
    manifest: &NodeManifest,
    source_label: &str,
    registry: &TypeRegistry,
    result: &mut InjectionResult,
) {
    let issues = manifest.validate(registry);
    if !issues.is_empty() {
        result.warnings.push(sanitize(&format!(
            "node \"{}\": {} has {} problem(s) — contracts not injected (first: {})",
            node.id,
            source_label,
            issues.len(),
            issues[0]
        )));
        return;
    }

    // the dataflow's wiring must stay within the declared ports (spec §10.1)
    for input in node.inputs.keys() {
        if !manifest.inputs.contains_key(input.as_str()) {
            result.warnings.push(sanitize(&format!(
                "node \"{}\": input `{input}` is not declared in {}",
                node.id, source_label
            )));
        }
    }
    for output in &node.outputs {
        if !manifest.outputs.contains_key(output.as_str()) {
            result.warnings.push(sanitize(&format!(
                "node \"{}\": output `{output}` is not declared in {}",
                node.id, source_label
            )));
        }
    }
    for (name, def) in &manifest.inputs {
        if def.is_required() && !node.inputs.keys().any(|k| k.as_str() == name) {
            result.warnings.push(sanitize(&format!(
                "node \"{}\": required input `{name}` is not wired",
                node.id
            )));
        }
    }

    // inject contract types; the dataflow author's own annotations win
    let mut injected = 0;
    let port_sets = [
        (&manifest.inputs, &mut node.input_types),
        (&manifest.outputs, &mut node.output_types),
    ];
    for (ports, annotations) in port_sets {
        for (name, def) in ports {
            let Some(urn) = &def.r#type else {
                continue;
            };
            // manifest validation guarantees port names parse as DataId
            let Ok(id) = name.parse() else {
                continue;
            };
            if let std::collections::btree_map::Entry::Vacant(entry) = annotations.entry(id) {
                entry.insert(urn.clone());
                injected += 1;
            }
        }
    }
    if injected > 0 {
        result.notes.push(sanitize(&format!(
            "node \"{}\": injected {injected} contract type(s) from {}",
            node.id, source_label
        )));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const MANIFEST: &str = r#"
apiVersion: 1
name: dora-yolo
namespace: dora-rs
runtime: rust
entrypoint: target/release/dora-yolo
inputs:
  image:
    type: std/media/v1/Image
outputs:
  bbox:
    type: std/vision/v1/BoundingBox
"#;

    fn dataflow(yaml: &str) -> Descriptor {
        serde_yaml::from_str(yaml).unwrap()
    }

    fn write_manifest(dir: &Path, content: &str) {
        std::fs::create_dir_all(dir).unwrap();
        std::fs::write(dir.join(MANIFEST_FILENAME), content).unwrap();
    }

    #[test]
    fn injects_types_from_adjacent_manifest() {
        let tmp = tempfile::tempdir().unwrap();
        write_manifest(&tmp.path().join("yolo"), MANIFEST);
        let mut df = dataflow(
            r#"
nodes:
  - id: camera
    path: cam
    outputs: [image]
  - id: detector
    path: yolo/target/release/dora-yolo
    inputs:
      image: camera/image
    outputs: [bbox]
"#,
        );
        let mut registry = TypeRegistry::new();
        let result = inject_adjacent_manifests(&mut df, tmp.path(), &mut registry);
        assert_eq!(result.warnings, Vec::<String>::new());
        assert_eq!(result.notes.len(), 1);
        let detector = &df.nodes[1];
        assert_eq!(
            detector
                .input_types
                .get(&"image".parse::<dora_message::id::DataId>().unwrap()),
            Some(&"std/media/v1/Image".to_string())
        );
        assert_eq!(
            detector
                .output_types
                .get(&"bbox".parse::<dora_message::id::DataId>().unwrap()),
            Some(&"std/vision/v1/BoundingBox".to_string())
        );
    }

    #[test]
    fn author_annotations_win() {
        let tmp = tempfile::tempdir().unwrap();
        write_manifest(&tmp.path().join("yolo"), MANIFEST);
        let mut df = dataflow(
            r#"
nodes:
  - id: detector
    path: yolo/target/release/dora-yolo
    inputs:
      image: other/image
    input_types:
      image: std/core/v1/Bytes
    outputs: [bbox]
"#,
        );
        let mut registry = TypeRegistry::new();
        inject_adjacent_manifests(&mut df, tmp.path(), &mut registry);
        assert_eq!(
            df.nodes[0]
                .input_types
                .get(&"image".parse::<dora_message::id::DataId>().unwrap()),
            Some(&"std/core/v1/Bytes".to_string()),
            "explicit dataflow annotation must not be overwritten"
        );
    }

    #[test]
    fn entrypoint_mismatch_warns_and_skips() {
        let tmp = tempfile::tempdir().unwrap();
        write_manifest(&tmp.path().join("yolo"), MANIFEST);
        let mut df = dataflow(
            r#"
nodes:
  - id: detector
    path: yolo/other-binary
    inputs:
      image: camera/image
"#,
        );
        let mut registry = TypeRegistry::new();
        let result = inject_adjacent_manifests(&mut df, tmp.path(), &mut registry);
        assert_eq!(result.warnings.len(), 1);
        assert!(result.warnings[0].contains("does not name"), "{result:?}");
        assert!(df.nodes[0].input_types.is_empty());
    }

    #[test]
    fn undeclared_wiring_and_missing_required_input_warn() {
        let tmp = tempfile::tempdir().unwrap();
        write_manifest(&tmp.path().join("yolo"), MANIFEST);
        let mut df = dataflow(
            r#"
nodes:
  - id: detector
    path: yolo/target/release/dora-yolo
    inputs:
      depth: camera/depth
    outputs: [points]
"#,
        );
        let mut registry = TypeRegistry::new();
        let result = inject_adjacent_manifests(&mut df, tmp.path(), &mut registry);
        let all = result.warnings.join("\n");
        assert!(all.contains("input `depth` is not declared"), "{all}");
        assert!(all.contains("output `points` is not declared"), "{all}");
        assert!(all.contains("required input `image` is not wired"), "{all}");
    }

    #[test]
    fn custom_types_are_registered() {
        let tmp = tempfile::tempdir().unwrap();
        write_manifest(
            &tmp.path().join("lidar"),
            r#"
apiVersion: 1
name: lidar
namespace: acme
runtime: rust
entrypoint: target/release/lidar
outputs:
  cloud:
    type: acme/lidar/v1/PointCloud
types:
  acme/lidar/v1/PointCloud:
    arrow: Struct
    fields:
      - name: x
        type: Float32
"#,
        );
        let mut df = dataflow(
            r#"
nodes:
  - id: lidar
    path: lidar/target/release/lidar
    outputs: [cloud]
"#,
        );
        let mut registry = TypeRegistry::new();
        let result = inject_adjacent_manifests(&mut df, tmp.path(), &mut registry);
        assert_eq!(result.warnings, Vec::<String>::new());
        assert!(registry.resolve("acme/lidar/v1/PointCloud").is_some());
        assert_eq!(
            df.nodes[0]
                .output_types
                .get(&"cloud".parse::<dora_message::id::DataId>().unwrap()),
            Some(&"acme/lidar/v1/PointCloud".to_string())
        );
    }

    #[test]
    fn invalid_manifest_warns_and_skips() {
        let tmp = tempfile::tempdir().unwrap();
        write_manifest(
            &tmp.path().join("bad"),
            r#"
apiVersion: 1
name: bad
namespace: dora-rs
runtime: rust
entrypoint: run
env:
  LD_PRELOAD:
    default: x
"#,
        );
        let mut df = dataflow(
            r#"
nodes:
  - id: bad
    path: bad/run
"#,
        );
        let mut registry = TypeRegistry::new();
        let result = inject_adjacent_manifests(&mut df, tmp.path(), &mut registry);
        assert_eq!(result.warnings.len(), 1);
        assert!(
            result.warnings[0].contains("contracts not injected"),
            "{result:?}"
        );
    }

    #[test]
    fn bare_command_and_url_and_git_nodes_are_skipped() {
        let tmp = tempfile::tempdir().unwrap();
        write_manifest(tmp.path(), MANIFEST);
        let mut df = dataflow(
            r#"
nodes:
  - id: console-script
    path: dora-yolo
  - id: url
    path: https://example.com/node
  - id: git-node
    path: target/release/dora-yolo
    git: https://github.com/x/y
"#,
        );
        let mut registry = TypeRegistry::new();
        let result = inject_adjacent_manifests(&mut df, tmp.path(), &mut registry);
        assert_eq!(result.notes, Vec::<String>::new());
        assert_eq!(result.warnings, Vec::<String>::new());
    }

    #[test]
    fn paths_outside_working_dir_are_skipped() {
        let tmp = tempfile::tempdir().unwrap();
        // a manifest exists at the working dir root, but escaping paths must
        // never be associated with anything
        write_manifest(tmp.path(), MANIFEST);
        let mut df = dataflow(
            r#"
nodes:
  - id: absolute
    path: /opt/builds/some-node
  - id: escaping
    path: ../outside/target/release/dora-yolo
"#,
        );
        let mut registry = TypeRegistry::new();
        let result = inject_adjacent_manifests(&mut df, tmp.path(), &mut registry);
        assert_eq!(result.notes, Vec::<String>::new());
        assert_eq!(result.warnings, Vec::<String>::new());
        assert!(df.nodes.iter().all(|n| n.input_types.is_empty()));
    }

    #[test]
    fn relative_working_dir_does_not_collapse_containment() {
        // `dora build dataflow.yml` run inside the dataflow's directory uses
        // working_dir = "." — which must NOT normalize to an empty prefix
        // that trivially contains absolute paths or eats leading `..`
        assert!(
            containment_roots("/opt/builds/some-node", Path::new(".")).is_none(),
            "absolute path must not be contained by a relative working dir"
        );
        assert!(
            containment_roots("../outside/target/release/bin", Path::new(".")).is_none(),
            "`..`-escaping path must be detected, not remapped"
        );
        let (full, wd) = containment_roots("yolo/target/release/bin", Path::new(".")).unwrap();
        assert!(wd.is_absolute());
        assert!(full.starts_with(&wd));
    }

    #[test]
    fn cross_manifest_types_resolve_regardless_of_node_order() {
        let tmp = tempfile::tempdir().unwrap();
        // `consumer` (listed first) references a type shipped by `lidar`
        // (listed second) — pass-1 registration makes this order-independent
        write_manifest(
            &tmp.path().join("consumer"),
            r#"
apiVersion: 1
name: consumer
namespace: beta
runtime: rust
entrypoint: target/release/consumer
inputs:
  cloud:
    type: acme/lidar/v1/PointCloud
    required: false
"#,
        );
        write_manifest(
            &tmp.path().join("lidar"),
            r#"
apiVersion: 1
name: lidar
namespace: acme
runtime: rust
entrypoint: target/release/lidar
outputs:
  cloud:
    type: acme/lidar/v1/PointCloud
types:
  acme/lidar/v1/PointCloud:
    arrow: Struct
    fields:
      - name: x
        type: Float32
"#,
        );
        let mut df = dataflow(
            r#"
nodes:
  - id: consumer
    path: consumer/target/release/consumer
    inputs:
      cloud: lidar/cloud
  - id: lidar
    path: lidar/target/release/lidar
    outputs: [cloud]
"#,
        );
        let mut registry = TypeRegistry::new();
        let result = inject_adjacent_manifests(&mut df, tmp.path(), &mut registry);
        assert_eq!(result.warnings, Vec::<String>::new());
        assert_eq!(result.notes.len(), 2, "{result:?}");
    }

    #[test]
    fn invalid_sibling_cannot_contaminate_the_registry() {
        let tmp = tempfile::tempdir().unwrap();
        // `bad` (namespace acme) ships a type under the *beta* namespace —
        // a violation. It must not be registered, so the unrelated `victim`
        // node referencing it must NOT validate against a definition `bad`
        // was never entitled to ship.
        write_manifest(
            &tmp.path().join("bad"),
            r#"
apiVersion: 1
name: bad
namespace: acme
runtime: rust
entrypoint: target/release/bad
types:
  beta/foo/v1/X:
    arrow: Struct
    fields:
      - name: x
        type: Float32
"#,
        );
        write_manifest(
            &tmp.path().join("victim"),
            r#"
apiVersion: 1
name: victim
namespace: gamma
runtime: rust
entrypoint: target/release/victim
outputs:
  out:
    type: beta/foo/v1/X
"#,
        );
        let mut df = dataflow(
            r#"
nodes:
  - id: bad
    path: bad/target/release/bad
  - id: victim
    path: victim/target/release/victim
    outputs: [out]
"#,
        );
        let mut registry = TypeRegistry::new();
        let result = inject_adjacent_manifests(&mut df, tmp.path(), &mut registry);
        // the out-of-namespace type was never registered
        assert!(registry.resolve("beta/foo/v1/X").is_none());
        // victim's port type does not resolve → its manifest fails validation
        // and contributes no injection
        let all = result.warnings.join("\n");
        assert!(
            all.contains("victim") && all.contains("contracts not injected"),
            "{result:?}"
        );
        assert!(df.nodes[1].output_types.is_empty());
    }

    #[test]
    fn unresolvable_sibling_body_cannot_contaminate_the_registry() {
        let tmp = tempfile::tempdir().unwrap();
        // `bad` owns its namespace but ships a type whose field type does not
        // resolve — it must not register, or a referencing node's port check
        // would pass against a type that can never be materialized.
        write_manifest(
            &tmp.path().join("bad"),
            r#"
apiVersion: 1
name: bad
namespace: acme
runtime: rust
entrypoint: target/release/bad
types:
  acme/foo/v1/X:
    arrow: Struct
    fields:
      - name: x
        type: NotARealType
"#,
        );
        write_manifest(
            &tmp.path().join("victim"),
            r#"
apiVersion: 1
name: victim
namespace: gamma
runtime: rust
entrypoint: target/release/victim
outputs:
  out:
    type: acme/foo/v1/X
"#,
        );
        let mut df = dataflow(
            r#"
nodes:
  - id: bad
    path: bad/target/release/bad
  - id: victim
    path: victim/target/release/victim
    outputs: [out]
"#,
        );
        let mut registry = TypeRegistry::new();
        let _ = inject_adjacent_manifests(&mut df, tmp.path(), &mut registry);
        // the type with an unresolvable body was never registered, so victim's
        // port check correctly fails to resolve it
        assert!(registry.resolve("acme/foo/v1/X").is_none());
        assert!(df.nodes[1].output_types.is_empty());
    }

    #[test]
    fn unknown_arrow_sibling_cannot_contaminate_the_registry() {
        let tmp = tempfile::tempdir().unwrap();
        // valid fields but a bogus `arrow:` — still unmaterializable, so it
        // must not register where a sibling could resolve it
        write_manifest(
            &tmp.path().join("bad"),
            "apiVersion: 1\nname: bad\nnamespace: acme\nruntime: rust\n\
             entrypoint: target/release/bad\ntypes:\n  acme/foo/v1/X:\n    arrow: NotAnArrowType\n",
        );
        write_manifest(
            &tmp.path().join("victim"),
            "apiVersion: 1\nname: victim\nnamespace: gamma\nruntime: rust\n\
             entrypoint: target/release/victim\noutputs:\n  out:\n    type: acme/foo/v1/X\n",
        );
        let mut df = dataflow(
            "nodes:\n  - id: bad\n    path: bad/target/release/bad\n  \
             - id: victim\n    path: victim/target/release/victim\n    outputs: [out]\n",
        );
        let mut registry = TypeRegistry::new();
        let _ = inject_adjacent_manifests(&mut df, tmp.path(), &mut registry);
        assert!(registry.resolve("acme/foo/v1/X").is_none());
        assert!(df.nodes[1].output_types.is_empty());
    }

    #[test]
    fn type_referencing_a_body_invalid_sibling_is_not_registered() {
        let tmp = tempfile::tempdir().unwrap();
        // `bad` ships two own-namespace types: `Y` is body-invalid (its field
        // type doesn't resolve), and `X` references `Y`. `X`'s body is only
        // "resolvable" if `Y` is treated as present — but `Y` can never
        // materialize, so neither may reach the shared registry. The fixpoint
        // admission must reject `X` transitively, not just `Y`.
        write_manifest(
            &tmp.path().join("bad"),
            r#"
apiVersion: 1
name: bad
namespace: acme
runtime: rust
entrypoint: target/release/bad
types:
  acme/foo/v1/Y:
    arrow: Struct
    fields:
      - name: y
        type: NotARealType
  acme/foo/v1/X:
    arrow: Struct
    fields:
      - name: inner
        type: acme/foo/v1/Y
"#,
        );
        write_manifest(
            &tmp.path().join("victim"),
            r#"
apiVersion: 1
name: victim
namespace: gamma
runtime: rust
entrypoint: target/release/victim
outputs:
  out:
    type: acme/foo/v1/X
"#,
        );
        let mut df = dataflow(
            r#"
nodes:
  - id: bad
    path: bad/target/release/bad
  - id: victim
    path: victim/target/release/victim
    outputs: [out]
"#,
        );
        let mut registry = TypeRegistry::new();
        let _ = inject_adjacent_manifests(&mut df, tmp.path(), &mut registry);
        assert!(registry.resolve("acme/foo/v1/Y").is_none());
        assert!(registry.resolve("acme/foo/v1/X").is_none());
        assert!(df.nodes[1].output_types.is_empty());
    }

    #[test]
    fn duplicate_urn_registers_the_body_valid_definition() {
        // Two same-namespace manifests ship the same URN, the body-invalid one
        // listed first. The fixpoint admits only the body-valid definition, so
        // the shared registry must hold *that* one — not the first-listed
        // (invalid) candidate's body. Regression for #2599.
        let tmp = tempfile::tempdir().unwrap();
        // Listed first: body-invalid `X` (its field type doesn't resolve).
        write_manifest(
            &tmp.path().join("bad"),
            r#"
apiVersion: 1
name: bad
namespace: acme
runtime: rust
entrypoint: target/release/bad
types:
  acme/foo/v1/X:
    arrow: Struct
    fields:
      - name: broken
        type: NotARealType
"#,
        );
        // Listed second: body-valid `X`, same URN.
        write_manifest(
            &tmp.path().join("good"),
            r#"
apiVersion: 1
name: good
namespace: acme
runtime: rust
entrypoint: target/release/good
types:
  acme/foo/v1/X:
    arrow: Struct
    fields:
      - name: x
        type: Float32
"#,
        );
        write_manifest(
            &tmp.path().join("victim"),
            r#"
apiVersion: 1
name: victim
namespace: gamma
runtime: rust
entrypoint: target/release/victim
outputs:
  out:
    type: acme/foo/v1/X
"#,
        );
        let mut df = dataflow(
            r#"
nodes:
  - id: bad
    path: bad/target/release/bad
  - id: good
    path: good/target/release/good
  - id: victim
    path: victim/target/release/victim
    outputs: [out]
"#,
        );
        let mut registry = TypeRegistry::new();
        let _ = inject_adjacent_manifests(&mut df, tmp.path(), &mut registry);
        // The registry holds the body-valid definition (field `x`), not the
        // first-listed invalid one (field `broken: NotARealType`).
        let resolved = registry
            .resolve("acme/foo/v1/X")
            .expect("the body-valid duplicate must register");
        assert!(
            resolved.fields.iter().any(|f| f.name == "x"),
            "registry must hold the body-valid definition, got {:?}",
            resolved.fields
        );
        assert!(
            !resolved.fields.iter().any(|f| f.r#type == "NotARealType"),
            "the body-invalid duplicate must not reach the registry, got {:?}",
            resolved.fields
        );
        // The referencing node's output type materializes against the valid body.
        assert!(!df.nodes[2].output_types.is_empty());
    }

    #[test]
    fn bare_path_with_adjacent_manifest_is_injected() {
        // a console-script / bare local-exe `path:` next to its manifest must
        // still get its contracts injected (the entrypoint names the bare path)
        let tmp = tempfile::tempdir().unwrap();
        write_manifest(
            tmp.path(),
            "apiVersion: 1\nname: dora-yolo\nnamespace: acme\nruntime: python\n\
             entrypoint: dora-yolo\noutputs:\n  bbox:\n    type: std/vision/v1/BoundingBox\n",
        );
        let mut df =
            dataflow("nodes:\n  - id: detector\n    path: dora-yolo\n    outputs: [bbox]\n");
        let mut registry = TypeRegistry::new();
        inject_adjacent_manifests(&mut df, tmp.path(), &mut registry);
        assert_eq!(
            df.nodes[0]
                .output_types
                .get(&"bbox".parse::<dora_message::id::DataId>().unwrap()),
            Some(&"std/vision/v1/BoundingBox".to_string())
        );
    }

    #[test]
    fn backslash_entrypoint_matches_cross_platform() {
        let tmp = tempfile::tempdir().unwrap();
        write_manifest(
            &tmp.path().join("yolo"),
            r#"
apiVersion: 1
name: dora-yolo
namespace: dora-rs
runtime: rust
entrypoint: target\release\dora-yolo
outputs:
  bbox:
    type: std/vision/v1/BoundingBox
"#,
        );
        let mut df = dataflow(
            r#"
nodes:
  - id: detector
    path: yolo/target/release/dora-yolo
    outputs: [bbox]
"#,
        );
        let mut registry = TypeRegistry::new();
        let result = inject_adjacent_manifests(&mut df, tmp.path(), &mut registry);
        assert_eq!(result.notes.len(), 1, "{result:?}");
    }

    #[test]
    fn dot_segments_in_path_still_match() {
        let tmp = tempfile::tempdir().unwrap();
        write_manifest(&tmp.path().join("yolo"), MANIFEST);
        let mut df = dataflow(
            r#"
nodes:
  - id: detector
    path: ./yolo/target/release/dora-yolo
    inputs:
      image: camera/image
    outputs: [bbox]
"#,
        );
        let mut registry = TypeRegistry::new();
        let result = inject_adjacent_manifests(&mut df, tmp.path(), &mut registry);
        assert_eq!(result.notes.len(), 1, "{result:?}");
    }
}
