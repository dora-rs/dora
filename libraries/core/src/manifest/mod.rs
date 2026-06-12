//! Node manifest (`dora-node.yml`) — the typed, dora-specific description of
//! a node: contracts, entry point, env/config surface.
//!
//! See `docs/plan-node-hub.md` §5 for the format specification. The manifest
//! lives next to the node's native package manifest (`pyproject.toml` /
//! `Cargo.toml`) and is copied verbatim into a hub index entry at publish
//! time, so discovery never needs to fetch source.

pub mod inject;
pub mod validate;

use std::{collections::BTreeMap, path::Path};

use eyre::Context;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

use crate::types::TypeDef;

/// Conventional file name of a node manifest.
pub const MANIFEST_FILENAME: &str = "dora-node.yml";

/// The manifest schema version this library reads and writes.
pub const MANIFEST_API_VERSION: u64 = 1;

/// A node manifest (`dora-node.yml`).
///
/// Holds only what native package manifests *cannot*: dora contracts and dora
/// wiring. Name, version, license, and dependencies are read from
/// `pyproject.toml`/`Cargo.toml` at publish time where possible.
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct NodeManifest {
    /// Manifest schema version. Currently always `1`.
    #[serde(rename = "apiVersion")]
    pub api_version: u64,

    /// Package name. Defaults to `[project].name` / `[package].name` of the
    /// native manifest at publish time; required for standalone validation.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,

    /// Index namespace the package publishes under (a GitHub org or user).
    pub namespace: String,

    /// One-line description, shown by `dora hub search`/`info`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,

    /// Categories from the fixed list (spec §7.6); drive search facets.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub categories: Vec<Category>,

    /// Free-form search keywords.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub keywords: Vec<String>,

    /// Language runtime of the node.
    pub runtime: Runtime,

    /// What to run, relative to the node's working dir.
    ///
    /// Python: a console script on the managed-env PATH (e.g. `dora-yolo`).
    /// Rust/C/C++: the build output, e.g. `target/release/dora-yolo`.
    /// Must be a relative path without `..` components.
    pub entrypoint: String,

    /// Build command run in the node's working dir when the package is
    /// installed (e.g. `pip install .`, `cargo build --release`). Arbitrary
    /// code by design, like any `build:` — `--locked` binds it to the
    /// reviewed, commit-pinned source (spec §11).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub build: Option<String>,

    /// Optional platform allowlist, e.g. `[linux-x86_64, macos-aarch64]`.
    /// Empty means all platforms.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub platforms: Vec<String>,

    /// Supported dora version range (semver requirement, e.g. `>=0.4`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub dora: Option<String>,

    /// Typed input ports. May be empty (source nodes).
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub inputs: BTreeMap<String, PortDef>,

    /// Typed output ports. May be empty (sink nodes).
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub outputs: BTreeMap<String, PortDef>,

    /// Documented + typed configuration surface (environment variables).
    /// Security-sensitive names (`PATH`, `PYTHONPATH`, `LD_*`, `DYLD_*`) are
    /// rejected at validation.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub env: BTreeMap<String, EnvVarDef>,

    /// Custom type definitions shipped with the node, keyed by full URN
    /// (e.g. `acme/lidar/v1/PointCloud`). Must live under the package's
    /// namespace; `std/` is rejected.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub types: BTreeMap<String, TypeDef>,

    /// Informational hardware/system requirements; surfaced by `dora hub
    /// info` and at build start, never auto-installed.
    #[serde(default, skip_serializing_if = "Requirements::is_empty")]
    pub requirements: Requirements,

    /// Example dataflow snippet shown by `dora hub info` (YAML node list).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub example: Option<String>,
}

/// Language runtime of a node.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "lowercase")]
pub enum Runtime {
    Python,
    Rust,
    C,
    Cpp,
}

/// Fixed category list (spec §7.6). Extended by index PR + discussion.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "kebab-case")]
pub enum Category {
    Sensor,
    Actuator,
    Robot,
    Transform,
    Filter,
    MlInference,
    Llm,
    Speech,
    Communication,
    Recorder,
    Visualization,
    Simulator,
    Debug,
}

/// A typed input or output port.
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct PortDef {
    /// Type URN (e.g. `std/media/v1/Image`). Omitted = untyped; validation
    /// skips the port.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub r#type: Option<String>,

    /// Whether a dataflow must wire this input (defaults to `true`; only
    /// meaningful on inputs — validation rejects it on outputs).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub required: Option<bool>,

    /// Human-readable description of the port.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
}

impl PortDef {
    /// Whether a dataflow must wire this input (defaults to `true`).
    pub fn is_required(&self) -> bool {
        self.required.unwrap_or(true)
    }
}

/// A documented environment variable.
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct EnvVarDef {
    /// Value type: `string` (default), `int`, `float`, or `bool`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub r#type: Option<EnvVarType>,

    /// Default value used when the variable is not set in the dataflow.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub default: Option<EnvDefault>,

    /// Human-readable description.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
}

/// A literal env default value.
///
/// Deliberately *not* the descriptor's `EnvValue`: that type expands `$VAR`
/// references against the local environment at deserialization time, which is
/// right for a dataflow being deployed but wrong for a manifest — manifests
/// are published verbatim into an index, so defaults must stay literal and
/// parse identically on every machine.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(untagged)]
pub enum EnvDefault {
    Bool(bool),
    Integer(i64),
    Float(f64),
    String(String),
}

/// Declared value type of an environment variable.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "lowercase")]
pub enum EnvVarType {
    String,
    Int,
    Float,
    Bool,
}

impl EnvVarType {
    /// The lowercase name as written in manifests.
    pub fn as_str(self) -> &'static str {
        match self {
            EnvVarType::String => "string",
            EnvVarType::Int => "int",
            EnvVarType::Float => "float",
            EnvVarType::Bool => "bool",
        }
    }
}

/// Informational hardware/system requirements (spec §5): documentation
/// surfaced by `dora hub info`, never auto-installed.
#[derive(Debug, Clone, Default, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Requirements {
    /// Hardware requirements, e.g. `[cuda]`. v1 actively probes only `cuda`.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub hardware: Vec<String>,

    /// System package requirements, keyed by package manager or platform.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub system: BTreeMap<String, String>,
}

impl Requirements {
    fn is_empty(&self) -> bool {
        self.hardware.is_empty() && self.system.is_empty()
    }
}

/// Upper bound on manifest size. Manifests are small by construction; the cap
/// bounds memory for the YAML parse of untrusted index content.
pub const MAX_MANIFEST_SIZE: usize = 1024 * 1024;

impl NodeManifest {
    /// Parse a manifest from YAML.
    pub fn parse(yaml: &str) -> eyre::Result<Self> {
        if yaml.len() > MAX_MANIFEST_SIZE {
            eyre::bail!(
                "node manifest too large ({} bytes, max {MAX_MANIFEST_SIZE})",
                yaml.len()
            );
        }
        serde_yaml::from_str(yaml).context("failed to parse node manifest")
    }

    /// Read and parse a manifest file.
    pub fn read(path: &Path) -> eyre::Result<Self> {
        use std::io::Read as _;
        // bound the read itself (not just the parse) — a size check on
        // metadata alone would miss FIFOs/devices and growing files
        let file = std::fs::File::open(path)
            .with_context(|| format!("failed to read node manifest at `{}`", path.display()))?;
        let mut content = String::new();
        file.take(MAX_MANIFEST_SIZE as u64 + 1)
            .read_to_string(&mut content)
            .with_context(|| format!("failed to read node manifest at `{}`", path.display()))?;
        Self::parse(&content)
            .with_context(|| format!("invalid node manifest at `{}`", path.display()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The full example manifest from spec §5.
    const SPEC_EXAMPLE: &str = r#"
apiVersion: 1
name: dora-yolo
namespace: dora-rs
description: YOLO object detection on camera frames
categories: [ml-inference]
keywords: [vision, detection, yolo]

runtime: python
entrypoint: dora-yolo
platforms: []
dora: ">=0.4"

inputs:
  image:
    type: std/media/v1/Image
    required: true
    description: BGR frame to run detection on
outputs:
  bbox:
    type: std/vision/v1/BBox2D
    description: detected bounding boxes

env:
  MODEL:
    default: yolov8n.pt
    description: model weights file or hub id
  CONFIDENCE:
    type: float
    default: 0.4

requirements:
  hardware: []
  system: {}

example: |
  - id: detector
    hub: dora-yolo@^0.5
    inputs:
      image: camera/image
    outputs:
      - bbox
"#;

    #[test]
    fn parses_spec_example() {
        let m = NodeManifest::parse(SPEC_EXAMPLE).unwrap();
        assert_eq!(m.api_version, 1);
        assert_eq!(m.name.as_deref(), Some("dora-yolo"));
        assert_eq!(m.namespace, "dora-rs");
        assert_eq!(m.runtime, Runtime::Python);
        assert_eq!(m.entrypoint, "dora-yolo");
        assert_eq!(m.categories, vec![Category::MlInference]);
        assert_eq!(
            m.inputs["image"].r#type.as_deref(),
            Some("std/media/v1/Image")
        );
        assert!(m.inputs["image"].is_required());
        assert_eq!(
            m.outputs["bbox"].r#type.as_deref(),
            Some("std/vision/v1/BBox2D")
        );
        assert_eq!(m.env["CONFIDENCE"].r#type, Some(EnvVarType::Float));
        assert!(m.example.is_some());
    }

    #[test]
    fn ports_may_be_empty() {
        // sinks have no outputs; sources no inputs (spec D9)
        let m = NodeManifest::parse(
            r#"
apiVersion: 1
name: dora-recorder
namespace: dora-rs
runtime: rust
entrypoint: target/release/dora-recorder
inputs:
  data:
    type: std/core/v1/Bytes
"#,
        )
        .unwrap();
        assert!(m.outputs.is_empty());
        assert_eq!(m.inputs.len(), 1);
    }

    #[test]
    fn unknown_fields_rejected() {
        let err = NodeManifest::parse(
            r#"
apiVersion: 1
name: x
namespace: y
runtime: python
entrypoint: x
no_such_field: true
"#,
        )
        .unwrap_err();
        assert!(format!("{err:#}").contains("no_such_field"), "{err:#}");
    }

    #[test]
    fn env_defaults_stay_literal() {
        // EnvDefault must NOT expand `$VAR` against the local environment —
        // manifests are published verbatim and must parse identically on
        // every machine (the descriptor's EnvValue would expand here)
        let m = NodeManifest::parse(
            r#"
apiVersion: 1
name: x
namespace: y
runtime: python
entrypoint: x
env:
  MODEL_PATH:
    default: $HOME/weights.pt
"#,
        )
        .unwrap();
        assert_eq!(
            m.env["MODEL_PATH"].default,
            Some(EnvDefault::String("$HOME/weights.pt".into()))
        );
    }

    #[test]
    fn checked_in_schema_is_current() {
        let schema = schemars::schema_for!(NodeManifest);
        let expected = serde_json::to_value(&schema).unwrap();
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("dora-node-schema.json");
        let on_disk: serde_json::Value =
            serde_json::from_str(&std::fs::read_to_string(&path).unwrap()).unwrap();
        assert_eq!(
            on_disk, expected,
            "dora-node-schema.json is stale — run `cargo run -p dora-core --bin generate_schema`"
        );
    }

    #[test]
    fn rejects_oversized_manifest() {
        let huge = format!("apiVersion: 1\n# {}", "x".repeat(MAX_MANIFEST_SIZE));
        let err = NodeManifest::parse(&huge).unwrap_err();
        assert!(format!("{err:#}").contains("too large"), "{err:#}");
    }

    #[test]
    fn roundtrips_through_serde() {
        let m = NodeManifest::parse(SPEC_EXAMPLE).unwrap();
        let yaml = serde_yaml::to_string(&m).unwrap();
        let again = NodeManifest::parse(&yaml).unwrap();
        assert_eq!(again.name, m.name);
        assert_eq!(again.inputs.len(), m.inputs.len());
        assert_eq!(again.env.len(), m.env.len());
    }
}
