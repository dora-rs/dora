use std::{
    collections::BTreeMap,
    path::{Path, PathBuf},
};

use dora_core::build::BuildInfo;
use dora_message::{
    BuildId, SessionId,
    common::GitSource,
    descriptor::{CoreNodeKind, NodeSource, OperatorSource, ResolvedNode},
    id::NodeId,
};
use eyre::{Context, ContextCompat};

/// Schema tag included in the build-inputs fingerprint canonical form. Bump
/// when the canonicalization shape changes so old fingerprints invalidate
/// automatically.
///
/// Versions:
///   - v1: Custom-node build/source/env/cwd only (initial #1944 implementation)
///   - v2: + Runtime-node operator build/source iteration (review of #1947)
const FINGERPRINT_SCHEMA_VERSION: u32 = 2;

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct DataflowSession {
    pub build_id: Option<BuildId>,
    pub session_id: SessionId,
    pub git_sources: BTreeMap<NodeId, GitSource>,
    pub local_build: Option<BuildInfo>,
    /// FNV-1a 64-bit hex digest of the resolved descriptor's build-inputs.
    /// `None` on sessions written before this field was introduced (treated
    /// as "unknown" — `invalidate_if_build_inputs_changed` clears and rewrites).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub build_fingerprint: Option<String>,
}

impl Default for DataflowSession {
    fn default() -> Self {
        Self {
            build_id: None,
            session_id: SessionId::generate(),
            git_sources: Default::default(),
            local_build: Default::default(),
            build_fingerprint: None,
        }
    }
}

impl DataflowSession {
    pub fn read_session(dataflow_path: &Path) -> eyre::Result<Self> {
        let session_file = session_file_path(dataflow_path)?;
        if session_file.exists() {
            if let Ok(parsed) = deserialize(&session_file) {
                return Ok(parsed);
            } else {
                tracing::warn!(
                    "failed to read dataflow session file, regenerating (you might need to run `dora build` again)"
                );
            }
        }

        let default_session = DataflowSession::default();
        default_session.write_out_for_dataflow(dataflow_path)?;
        Ok(default_session)
    }

    pub fn write_out_for_dataflow(&self, dataflow_path: &Path) -> eyre::Result<()> {
        let session_file = session_file_path(dataflow_path)?;
        let filename = session_file
            .file_name()
            .context("session file has no file name")?
            .to_str()
            .context("session file name is no utf8")?;
        if let Some(parent) = session_file.parent() {
            std::fs::create_dir_all(parent).context("failed to create out dir")?;
        }
        std::fs::write(&session_file, self.serialize()?)
            .context("failed to write dataflow session file")?;
        let gitignore = session_file.with_file_name(".gitignore");
        if gitignore.exists() {
            let existing =
                std::fs::read_to_string(&gitignore).context("failed to read gitignore")?;
            if !existing
                .lines()
                .any(|l| l.split_once('/') == Some(("", filename)))
            {
                let new = existing + &format!("\n/{filename}\n");
                std::fs::write(gitignore, new).context("failed to update gitignore")?;
            }
        } else {
            std::fs::write(gitignore, format!("/{filename}\n"))
                .context("failed to write gitignore")?;
        }
        Ok(())
    }

    fn serialize(&self) -> eyre::Result<String> {
        serde_yaml::to_string(&self).context("failed to serialize dataflow session file")
    }

    /// Compute the build-inputs fingerprint over the resolved descriptor.
    ///
    /// Inputs (anything that affects what `dora build` produces):
    /// - per-node `build` command (Custom-kind nodes)
    /// - per-node `source` (Local vs GitBranch with repo + branch/tag/rev)
    /// - per-node `env` (sorted map, Display'd values — covers global env after
    ///   resolve_aliases_and_set_defaults merges it)
    /// - per-node `deploy.working_dir` (changes shift the cwd cargo/python runs in)
    /// - per-operator `build` command and `source` (Runtime-kind nodes — Python
    ///   script path, conda env, SharedLibrary path, Wasm path)
    ///
    /// Deliberately ignored (don't affect the build):
    /// - `path` — where the artifact is read from at start time, not built to
    /// - `deploy.machine` — affects placement at start time, not build inputs
    ///
    /// Discussed in #1444 + extended in #1947 review.
    pub fn fingerprint_build_inputs(resolved_nodes: &BTreeMap<NodeId, ResolvedNode>) -> String {
        let mut canonical = String::new();
        // Schema tag in the canonical form so format bumps force regeneration
        // even if descriptor inputs are unchanged. Same approach used by
        // `BuildLockfile::fingerprint_descriptor_git_sources`.
        canonical.push_str(&format!(
            "dora-session-build-inputs-v{FINGERPRINT_SCHEMA_VERSION}\n"
        ));

        // BTreeMap iteration is already sorted by NodeId — deterministic.
        for (node_id, node) in resolved_nodes {
            canonical.push_str("node:");
            canonical.push_str(node_id.as_ref());
            canonical.push('\n');

            // Build command (None and Some("") both contribute distinctly:
            // "build:none" vs "build:" — preserves the "unset vs empty"
            // distinction in case a user toggles between them).
            let build_field = node_build_command(&node.kind);
            match build_field {
                Some(cmd) => {
                    canonical.push_str("build:");
                    canonical.push_str(cmd);
                    canonical.push('\n');
                }
                None => canonical.push_str("build:none\n"),
            }

            // Source (Local vs git repo+rev).
            let source = node_source(&node.kind);
            match source {
                Some(NodeSource::Local) => canonical.push_str("source:local\n"),
                Some(NodeSource::GitBranch { repo, rev }) => {
                    canonical.push_str("source:git\nrepo:");
                    canonical.push_str(repo);
                    canonical.push('\n');
                    let (kind, value) = match rev {
                        Some(dora_message::descriptor::GitRepoRev::Branch(v)) => {
                            ("branch", v.as_str())
                        }
                        Some(dora_message::descriptor::GitRepoRev::Tag(v)) => ("tag", v.as_str()),
                        Some(dora_message::descriptor::GitRepoRev::Rev(v)) => ("rev", v.as_str()),
                        None => ("head", ""),
                    };
                    canonical.push_str("rev:");
                    canonical.push_str(kind);
                    canonical.push(':');
                    canonical.push_str(value);
                    canonical.push('\n');
                }
                None => canonical.push_str("source:none\n"),
            }

            // Env (BTreeMap iteration sorted; covers globals after merge).
            // `EnvValue` has a `Display` impl that produces the canonical text
            // form ("true", "42", "1.5", or the raw string).
            if let Some(env) = &node.env {
                canonical.push_str("env:\n");
                for (k, v) in env {
                    canonical.push_str("  ");
                    canonical.push_str(k);
                    canonical.push('=');
                    canonical.push_str(&v.to_string());
                    canonical.push('\n');
                }
            } else {
                canonical.push_str("env:none\n");
            }

            // Working directory (per-node deploy override).
            if let Some(deploy) = &node.deploy {
                if let Some(cwd) = &deploy.working_dir {
                    canonical.push_str("cwd:");
                    canonical.push_str(&cwd.to_string_lossy());
                    canonical.push('\n');
                } else {
                    canonical.push_str("cwd:none\n");
                }
            } else {
                canonical.push_str("cwd:none\n");
            }

            // Operator-level build commands and sources (Runtime node only).
            // Each operator can carry its own `build:` and `source` (Python /
            // SharedLibrary / Wasm). The schema-v1 fingerprint missed these
            // entirely, so changing an operator's build silently reused the
            // cached `build_id` — see #1947 self-review.
            //
            // `OperatorDefinition.id` is sorted within the Vec because
            // `resolve_aliases_and_set_defaults` preserves user-declared order
            // rather than sorting. Sort by id here so reordering operators in
            // YAML doesn't churn the fingerprint.
            if let CoreNodeKind::Runtime(runtime) = &node.kind {
                let mut by_id: Vec<_> = runtime.operators.iter().collect();
                by_id.sort_by_key(|op| op.id.as_ref());
                if by_id.is_empty() {
                    canonical.push_str("operators:none\n");
                } else {
                    canonical.push_str("operators:\n");
                    for op in by_id {
                        canonical.push_str("  op:");
                        canonical.push_str(op.id.as_ref());
                        canonical.push('\n');
                        match op.config.build.as_deref() {
                            Some(cmd) => {
                                canonical.push_str("    build:");
                                canonical.push_str(cmd);
                                canonical.push('\n');
                            }
                            None => canonical.push_str("    build:none\n"),
                        }
                        match &op.config.source {
                            OperatorSource::SharedLibrary(path) => {
                                canonical.push_str("    source:shared-library\n    path:");
                                canonical.push_str(path);
                                canonical.push('\n');
                            }
                            OperatorSource::Python(py) => {
                                canonical.push_str("    source:python\n    script:");
                                canonical.push_str(&py.source);
                                canonical.push('\n');
                                if let Some(conda) = &py.conda_env {
                                    canonical.push_str("    conda_env:");
                                    canonical.push_str(conda);
                                    canonical.push('\n');
                                } else {
                                    canonical.push_str("    conda_env:none\n");
                                }
                            }
                            OperatorSource::Wasm(path) => {
                                canonical.push_str("    source:wasm\n    path:");
                                canonical.push_str(path);
                                canonical.push('\n');
                            }
                        }
                    }
                }
            }
        }

        fnv1a_64_hex(canonical.as_bytes())
    }

    /// Compare current descriptor's build-inputs fingerprint against the
    /// stored one. On mismatch (including "never recorded"), clear stale
    /// build metadata (`build_id`, `local_build`, `git_sources`) and store
    /// the new fingerprint. Returns `true` if invalidation occurred.
    ///
    /// Call this on `dora start` / `dora run` / `dora daemon
    /// --run-dataflow` before consuming `build_id`, and on `dora build`
    /// after resolving the descriptor (so the fingerprint reflects the
    /// build that just ran).
    pub fn invalidate_if_build_inputs_changed(
        &mut self,
        resolved_nodes: &BTreeMap<NodeId, ResolvedNode>,
    ) -> bool {
        let current = Self::fingerprint_build_inputs(resolved_nodes);
        if self.build_fingerprint.as_deref() == Some(&current) {
            return false;
        }
        let had_build_id = self.build_id.is_some();
        self.build_id = None;
        self.local_build = None;
        self.git_sources.clear();
        self.build_fingerprint = Some(current);
        if had_build_id {
            tracing::info!(
                "dataflow build inputs changed since last session — discarding cached \
                 build_id; run `dora build` to rebuild"
            );
        }
        true
    }
}

/// Extract the `build` command from a `ResolvedNode`'s kind. Returns `None`
/// when the field is unset, and `Some("")` to distinguish an empty-string
/// build from a missing one (a user could plausibly toggle between them).
///
/// Runtime-kind nodes (operator runtime) return `None` here; their
/// operator-level builds are folded into the canonical form separately
/// by `fingerprint_build_inputs`.
fn node_build_command(kind: &CoreNodeKind) -> Option<&str> {
    match kind {
        CoreNodeKind::Custom(custom) => custom.build.as_deref(),
        CoreNodeKind::Runtime(_) => None,
    }
}

/// Extract the `source` from a `ResolvedNode`'s kind.
fn node_source(kind: &CoreNodeKind) -> Option<&NodeSource> {
    match kind {
        CoreNodeKind::Custom(custom) => Some(&custom.source),
        CoreNodeKind::Runtime(_) => None,
    }
}

/// FNV-1a 64-bit, lowercase hex (16 chars). Matches the digest shape used
/// by `BuildLockfile::fingerprint_descriptor_git_sources` so the two
/// fingerprints are visually distinguishable as the same family.
fn fnv1a_64_hex(bytes: &[u8]) -> String {
    const FNV_OFFSET_BASIS: u64 = 0xcbf29ce484222325;
    const FNV_PRIME: u64 = 0x100000001b3;
    let mut hash = FNV_OFFSET_BASIS;
    for byte in bytes {
        hash ^= *byte as u64;
        hash = hash.wrapping_mul(FNV_PRIME);
    }
    format!("{hash:016x}")
}

fn deserialize(session_file: &Path) -> eyre::Result<DataflowSession> {
    std::fs::read_to_string(session_file)
        .context("failed to read DataflowSession file")
        .and_then(|s| {
            serde_yaml::from_str(&s).context("failed to deserialize DataflowSession file")
        })
}

fn session_file_path(dataflow_path: &Path) -> eyre::Result<PathBuf> {
    let file_stem = dataflow_path
        .file_stem()
        .wrap_err("dataflow path has no file stem")?
        .to_str()
        .wrap_err("dataflow file stem is not valid utf-8")?;
    let session_file = dataflow_path
        .with_file_name("out")
        .join(format!("{file_stem}.dora-session.yaml"));
    Ok(session_file)
}

#[cfg(test)]
mod tests {
    use super::*;
    use dora_core::descriptor::DescriptorExt;
    use dora_message::descriptor::Descriptor;

    /// Parse YAML into a resolved-nodes map. Tests stay readable by editing
    /// the YAML rather than wrestling with `CustomNode`'s 20-field struct
    /// literal (and they survive descriptor schema additions).
    fn resolved(yaml: &str) -> BTreeMap<NodeId, ResolvedNode> {
        let desc: Descriptor = serde_yaml::from_str(yaml).expect("yaml parses as Descriptor");
        desc.resolve_aliases_and_set_defaults()
            .expect("descriptor resolves cleanly")
    }

    const BASELINE: &str = "\
nodes:
  - id: a
    path: ./a
    build: cargo build
";

    #[test]
    fn fingerprint_is_stable_for_same_inputs() {
        let nodes = resolved(BASELINE);
        let fp1 = DataflowSession::fingerprint_build_inputs(&nodes);
        let fp2 = DataflowSession::fingerprint_build_inputs(&nodes);
        assert_eq!(fp1, fp2, "same inputs must produce same fingerprint");
    }

    #[test]
    fn fingerprint_changes_when_build_command_changes() {
        let before = resolved(BASELINE);
        let after = resolved(
            "\
nodes:
  - id: a
    path: ./a
    build: cargo build --release
",
        );
        assert_ne!(
            DataflowSession::fingerprint_build_inputs(&before),
            DataflowSession::fingerprint_build_inputs(&after),
            "build command change must change fingerprint",
        );
    }

    #[test]
    fn fingerprint_changes_when_source_changes_local_to_git() {
        let local = resolved(BASELINE);
        let git = resolved(
            "\
nodes:
  - id: a
    path: ./a
    git: https://github.com/dora-rs/dora.git
    build: cargo build
",
        );
        assert_ne!(
            DataflowSession::fingerprint_build_inputs(&local),
            DataflowSession::fingerprint_build_inputs(&git),
            "switching local->git source must change fingerprint",
        );
    }

    #[test]
    fn fingerprint_changes_when_git_rev_changes() {
        let rev_a = resolved(
            "\
nodes:
  - id: a
    path: ./a
    git: https://github.com/dora-rs/dora.git
    rev: aaaaaaa
    build: cargo build
",
        );
        let rev_b = resolved(
            "\
nodes:
  - id: a
    path: ./a
    git: https://github.com/dora-rs/dora.git
    rev: bbbbbbb
    build: cargo build
",
        );
        assert_ne!(
            DataflowSession::fingerprint_build_inputs(&rev_a),
            DataflowSession::fingerprint_build_inputs(&rev_b),
            "different git revs must produce different fingerprints",
        );
    }

    #[test]
    fn fingerprint_changes_when_env_changes() {
        let without_env = resolved(BASELINE);
        let with_env = resolved(
            "\
nodes:
  - id: a
    path: ./a
    build: cargo build
    env:
      RUST_LOG: info
",
        );
        assert_ne!(
            DataflowSession::fingerprint_build_inputs(&without_env),
            DataflowSession::fingerprint_build_inputs(&with_env),
            "adding env must change fingerprint",
        );
    }

    #[test]
    fn fingerprint_unchanged_when_only_path_changes() {
        // Path is where the artifact is *read from* at start time, not where
        // it's built to. Per the #1444 policy, path-only changes must not
        // invalidate cached build metadata.
        let map_a = resolved(
            "\
nodes:
  - id: a
    path: ./target/debug/a
    build: cargo build
",
        );
        let map_b = resolved(
            "\
nodes:
  - id: a
    path: ./target/release/a
    build: cargo build
",
        );
        assert_eq!(
            DataflowSession::fingerprint_build_inputs(&map_a),
            DataflowSession::fingerprint_build_inputs(&map_b),
            "path-only change MUST NOT change fingerprint (per #1444 policy)",
        );
    }

    #[test]
    fn invalidate_clears_build_metadata_on_mismatch() {
        let nodes = resolved(BASELINE);
        let mut session = DataflowSession {
            build_id: Some(BuildId::generate()),
            git_sources: BTreeMap::from([(
                NodeId::from("a".to_string()),
                GitSource {
                    repo: "x".to_string(),
                    commit_hash: "y".to_string(),
                },
            )]),
            ..Default::default()
        };

        let invalidated = session.invalidate_if_build_inputs_changed(&nodes);
        assert!(
            invalidated,
            "first call against an un-fingerprinted session must invalidate"
        );
        assert!(session.build_id.is_none(), "build_id must be cleared");
        assert!(session.local_build.is_none(), "local_build must be cleared");
        assert!(
            session.git_sources.is_empty(),
            "git_sources must be cleared"
        );
        assert!(
            session.build_fingerprint.is_some(),
            "new fingerprint must be stored"
        );
    }

    #[test]
    fn invalidate_is_noop_when_fingerprint_matches() {
        let nodes = resolved(BASELINE);
        let mut session = DataflowSession::default();
        // Prime the fingerprint as if a previous `dora build` had stored it.
        session.invalidate_if_build_inputs_changed(&nodes);
        // Set a build_id to mimic post-build state.
        let post_build_id = Some(BuildId::generate());
        session.build_id = post_build_id;

        let invalidated = session.invalidate_if_build_inputs_changed(&nodes);
        assert!(!invalidated, "matching fingerprint must NOT invalidate");
        assert_eq!(
            session.build_id, post_build_id,
            "build_id must survive matching call"
        );
    }

    #[test]
    fn session_roundtrip_with_new_field() {
        let session = DataflowSession {
            build_fingerprint: Some("abcdef0123456789".to_string()),
            ..Default::default()
        };
        let yaml = serde_yaml::to_string(&session).unwrap();
        let parsed: DataflowSession = serde_yaml::from_str(&yaml).unwrap();
        assert_eq!(
            parsed.build_fingerprint.as_deref(),
            Some("abcdef0123456789")
        );
    }

    #[test]
    fn session_roundtrip_backward_compatible_without_field() {
        // Sessions written before this field existed must still deserialize.
        let legacy_yaml = "build_id: null\n\
             session_id: 00000000-0000-0000-0000-000000000000\n\
             git_sources: {}\n\
             local_build: null\n";
        let parsed: DataflowSession =
            serde_yaml::from_str(legacy_yaml).expect("legacy session must deserialize");
        assert!(parsed.build_fingerprint.is_none());
    }

    // ---------------------------------------------------------------------
    // Coverage added during /pr-review of #1947. The initial 10 tests
    // exercised only single-node Custom-kind dataflows, leaving the policy
    // partially unverified for operators, multi-node ordering, env
    // mutation, and deploy.machine ignore. These tests close those gaps.
    // ---------------------------------------------------------------------

    /// `deploy.machine` is a placement input, not a build input — per the
    /// #1444 narrower policy, changing it MUST NOT invalidate the session.
    /// The original test suite only verified path-only changes; this
    /// closes the symmetric gap.
    #[test]
    fn fingerprint_unchanged_when_only_deploy_machine_changes() {
        let on_m1 = resolved(
            "\
nodes:
  - id: a
    path: ./a
    build: cargo build
    _unstable_deploy:
      machine: machine-1
",
        );
        let on_m2 = resolved(
            "\
nodes:
  - id: a
    path: ./a
    build: cargo build
    _unstable_deploy:
      machine: machine-2
",
        );
        assert_eq!(
            DataflowSession::fingerprint_build_inputs(&on_m1),
            DataflowSession::fingerprint_build_inputs(&on_m2),
            "deploy.machine-only change MUST NOT change fingerprint (per #1444 policy)",
        );
    }

    /// `deploy.working_dir` is a build input — the cargo/python invocation
    /// runs from there. Per policy, changing it MUST invalidate.
    #[test]
    fn fingerprint_changes_when_deploy_working_dir_changes() {
        let cwd_a = resolved(
            "\
nodes:
  - id: a
    path: ./a
    build: cargo build
    _unstable_deploy:
      working_dir: /tmp/a
",
        );
        let cwd_b = resolved(
            "\
nodes:
  - id: a
    path: ./a
    build: cargo build
    _unstable_deploy:
      working_dir: /tmp/b
",
        );
        assert_ne!(
            DataflowSession::fingerprint_build_inputs(&cwd_a),
            DataflowSession::fingerprint_build_inputs(&cwd_b),
            "deploy.working_dir change must change fingerprint",
        );
    }

    /// Env removal and value-mutation are both forms of env change. The
    /// original test suite only covered env *addition*; this verifies the
    /// fingerprint detects the other two shapes.
    #[test]
    fn fingerprint_changes_when_env_value_mutates() {
        let info = resolved(
            "\
nodes:
  - id: a
    path: ./a
    build: cargo build
    env:
      RUST_LOG: info
",
        );
        let debug = resolved(
            "\
nodes:
  - id: a
    path: ./a
    build: cargo build
    env:
      RUST_LOG: debug
",
        );
        assert_ne!(
            DataflowSession::fingerprint_build_inputs(&info),
            DataflowSession::fingerprint_build_inputs(&debug),
            "mutating an env value must change fingerprint",
        );
    }

    #[test]
    fn fingerprint_changes_when_env_key_removed() {
        let two_keys = resolved(
            "\
nodes:
  - id: a
    path: ./a
    build: cargo build
    env:
      RUST_LOG: info
      FEATURE: x
",
        );
        let one_key = resolved(
            "\
nodes:
  - id: a
    path: ./a
    build: cargo build
    env:
      RUST_LOG: info
",
        );
        assert_ne!(
            DataflowSession::fingerprint_build_inputs(&two_keys),
            DataflowSession::fingerprint_build_inputs(&one_key),
            "removing an env key must change fingerprint",
        );
    }

    /// Multi-node ordering: BTreeMap iteration is sorted by NodeId, so
    /// reordering nodes in YAML must not change the fingerprint. The
    /// original test suite never exercised more than one node, so this
    /// pins the determinism guarantee.
    #[test]
    fn fingerprint_unchanged_when_only_node_yaml_order_changes() {
        let ab = resolved(
            "\
nodes:
  - id: a
    path: ./a
    build: cargo build
  - id: b
    path: ./b
    build: make
",
        );
        let ba = resolved(
            "\
nodes:
  - id: b
    path: ./b
    build: make
  - id: a
    path: ./a
    build: cargo build
",
        );
        assert_eq!(
            DataflowSession::fingerprint_build_inputs(&ab),
            DataflowSession::fingerprint_build_inputs(&ba),
            "node YAML ordering must not affect fingerprint (BTreeMap sorts by id)",
        );
    }

    /// Operator-level build commands DO affect what gets built and MUST
    /// invalidate. This is the P1 from /pr-review of #1947 — the v1
    /// fingerprint silently ignored Runtime nodes, so changing an
    /// operator's `pip install` build silently kept the cached `build_id`.
    #[test]
    fn fingerprint_changes_when_operator_build_changes() {
        let pandas_v2 = resolved(
            "\
nodes:
  - id: foo
    operators:
      - id: op
        python: ./op.py
        build: pip install pandas==2.0
        inputs: {}
        outputs: []
",
        );
        let pandas_v2_1 = resolved(
            "\
nodes:
  - id: foo
    operators:
      - id: op
        python: ./op.py
        build: pip install pandas==2.1
        inputs: {}
        outputs: []
",
        );
        assert_ne!(
            DataflowSession::fingerprint_build_inputs(&pandas_v2),
            DataflowSession::fingerprint_build_inputs(&pandas_v2_1),
            "operator-level build change must change fingerprint",
        );
    }

    #[test]
    fn fingerprint_changes_when_operator_source_changes() {
        let py_a = resolved(
            "\
nodes:
  - id: foo
    operators:
      - id: op
        python: ./op_a.py
        inputs: {}
        outputs: []
",
        );
        let py_b = resolved(
            "\
nodes:
  - id: foo
    operators:
      - id: op
        python: ./op_b.py
        inputs: {}
        outputs: []
",
        );
        assert_ne!(
            DataflowSession::fingerprint_build_inputs(&py_a),
            DataflowSession::fingerprint_build_inputs(&py_b),
            "operator-level Python source change must change fingerprint",
        );
    }

    #[test]
    fn fingerprint_unchanged_when_only_operator_yaml_order_changes() {
        // Operators within a node are stored in a Vec (not a BTreeMap), so
        // the fingerprint code sorts them by `id` before canonicalizing.
        // Verify reordering in YAML doesn't churn the fingerprint.
        let ab = resolved(
            "\
nodes:
  - id: foo
    operators:
      - id: a
        python: ./a.py
        inputs: {}
        outputs: []
      - id: b
        python: ./b.py
        inputs: {}
        outputs: []
",
        );
        let ba = resolved(
            "\
nodes:
  - id: foo
    operators:
      - id: b
        python: ./b.py
        inputs: {}
        outputs: []
      - id: a
        python: ./a.py
        inputs: {}
        outputs: []
",
        );
        assert_eq!(
            DataflowSession::fingerprint_build_inputs(&ab),
            DataflowSession::fingerprint_build_inputs(&ba),
            "operator YAML ordering within a node must not affect fingerprint",
        );
    }
}
