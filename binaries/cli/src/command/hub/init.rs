//! `dora hub init` — scaffold a `dora-node.yml` next to the node's native
//! package manifest (spec §5, P1.2). Name, runtime, and entrypoint are
//! pre-filled from `pyproject.toml` / `Cargo.toml` where possible; contracts
//! are left as commented examples for the author to fill in.

use std::path::{Path, PathBuf};

use dora_core::manifest::{MANIFEST_FILENAME, NodeManifest};
use eyre::{Context, bail};

use crate::command::Executable;

/// Scaffold a dora-node.yml manifest for a node
#[derive(Debug, clap::Args)]
pub struct Init {
    /// Directory containing the node (defaults to the current directory)
    #[clap(value_name = "PATH", default_value = ".")]
    path: PathBuf,
}

impl Executable for Init {
    fn execute(self) -> eyre::Result<()> {
        let manifest_path = self.path.join(MANIFEST_FILENAME);
        if manifest_path.exists() {
            bail!(
                "`{}` already exists — refusing to overwrite",
                manifest_path.display()
            );
        }
        if !self.path.is_dir() {
            bail!("`{}` is not a directory", self.path.display());
        }

        let detected = detect_node(&self.path);
        let namespace = detect_namespace(&self.path);
        let content = render_manifest(&detected, &namespace);

        // self-check: the scaffold must parse as a valid manifest so the
        // template can never drift from the schema
        NodeManifest::parse(&content).context("internal error: generated manifest is invalid")?;

        std::fs::write(&manifest_path, &content)
            .with_context(|| format!("failed to write `{}`", manifest_path.display()))?;

        println!("Wrote {}.", manifest_path.display());
        println!(
            "Next steps:\n  \
             1. fill in the typed inputs/outputs (see `types/std/` for available type URNs)\n  \
             2. check it: dora validate --node-manifest {}",
            manifest_path.display()
        );
        Ok(())
    }
}

struct DetectedNode {
    name: String,
    runtime: &'static str,
    entrypoint: String,
    description: Option<String>,
    /// Where the defaults came from, mentioned in the scaffold comments.
    source: Option<&'static str>,
}

/// Pre-fill name/runtime/entrypoint from the native package manifest.
fn detect_node(dir: &Path) -> DetectedNode {
    if let Some(detected) = detect_python(dir) {
        return detected;
    }
    if let Some(detected) = detect_rust(dir) {
        return detected;
    }
    let name = dir
        .canonicalize()
        .ok()
        .and_then(|p| p.file_name().map(|n| n.to_string_lossy().into_owned()))
        .map(|n| normalize_name(&n))
        .filter(|n| !n.is_empty())
        .unwrap_or_else(|| "my-node".into());
    DetectedNode {
        entrypoint: name.clone(),
        name,
        runtime: "python",
        description: None,
        source: None,
    }
}

fn detect_python(dir: &Path) -> Option<DetectedNode> {
    let raw = std::fs::read_to_string(dir.join("pyproject.toml")).ok()?;
    let parsed: toml::Table = raw.parse().ok()?;
    // PEP 621 `[project]`, with a fallback for Poetry-style metadata
    let project = parsed.get("project").or_else(|| {
        parsed
            .get("tool")
            .and_then(|t| t.get("poetry"))
            .filter(|p| p.get("name").is_some())
    })?;
    let name = normalize_name(project.get("name")?.as_str()?);
    if name.is_empty() {
        return None;
    }
    // prefer the console script named like the package; else the first one
    let entrypoint = project
        .get("scripts")
        .and_then(|s| s.as_table())
        .and_then(|t| {
            t.keys()
                .find(|k| **k == name)
                .or_else(|| t.keys().next())
                .cloned()
        })
        .unwrap_or_else(|| name.clone());
    Some(DetectedNode {
        name,
        runtime: "python",
        entrypoint,
        description: project
            .get("description")
            .and_then(|d| d.as_str())
            .map(String::from),
        source: Some("pyproject.toml"),
    })
}

fn detect_rust(dir: &Path) -> Option<DetectedNode> {
    let raw = std::fs::read_to_string(dir.join("Cargo.toml")).ok()?;
    let parsed: toml::Table = raw.parse().ok()?;
    let package = parsed.get("package")?;
    let name = normalize_name(package.get("name")?.as_str()?);
    if name.is_empty() {
        return None;
    }
    Some(DetectedNode {
        entrypoint: format!("target/release/{name}"),
        name,
        runtime: "rust",
        description: package
            .get("description")
            .and_then(|d| d.as_str())
            .map(String::from),
        source: Some("Cargo.toml"),
    })
}

/// Best-effort namespace default: the GitHub org/user of the `origin` remote.
fn detect_namespace(dir: &Path) -> String {
    let output = std::process::Command::new("git")
        .args(["remote", "get-url", "origin"])
        .current_dir(dir)
        .output();
    if let Ok(output) = output
        && output.status.success()
        && let Ok(url) = String::from_utf8(output.stdout)
    {
        // first path segment of the remote URL, forge-agnostic:
        // https://<host>/<org>/<repo>.git or git@<host>:<org>/<repo>.git
        let url = url.trim();
        let path = if let Some(rest) = url.split_once("://").map(|(_, r)| r) {
            rest.split_once('/').map(|(_, p)| p)
        } else {
            url.split_once(':').map(|(_, p)| p)
        };
        let org: String = path
            .and_then(|p| p.split('/').next())
            .unwrap_or("")
            .trim()
            .to_ascii_lowercase()
            .chars()
            .filter(|c| c.is_ascii_alphanumeric() || *c == '-')
            .collect();
        if !org.is_empty() {
            return org;
        }
    }
    "your-namespace".into()
}

/// Lowercase and replace `_` so the name passes manifest validation
/// (PEP 503-style normalization). May return an empty string when nothing
/// usable remains — callers fall back to another source.
fn normalize_name(name: &str) -> String {
    name.trim()
        .to_ascii_lowercase()
        .replace('_', "-")
        .chars()
        .filter(|c| c.is_ascii_alphanumeric() || "-.".contains(*c))
        .collect::<String>()
        .trim_matches(['-', '.'])
        .to_string()
}

fn render_manifest(node: &DetectedNode, namespace: &str) -> String {
    let DetectedNode {
        name,
        runtime,
        entrypoint,
        description,
        source,
    } = node;
    let detected_note = match source {
        Some(source) => format!(" # detected from {source}"),
        None => " # TODO: verify".into(),
    };
    // double-quote: native-manifest values can contain `:`, quotes, etc.
    let yaml_quote = |s: &str| {
        format!(
            "\"{}\"",
            s.replace('\\', "\\\\")
                .replace('"', "\\\"")
                .replace('\n', " ")
        )
    };
    let description = yaml_quote(
        description
            .as_deref()
            .unwrap_or("TODO: one-line description of what this node does"),
    );
    let entrypoint = yaml_quote(entrypoint);
    // keep this template in sync with the NodeManifest schema; execute()
    // parse-checks it before writing
    format!(
        r#"# dora-node.yml — node manifest (see `dora validate --node-manifest`)
# Spec: https://github.com/dora-rs/dora/blob/main/docs/plan-node-hub.md
apiVersion: 1
name: "{name}"{detected_note}
namespace: "{namespace}" # the GitHub org/user you publish under
description: {description}
# categories: [sensor] # one or more of: sensor, actuator, robot, transform,
#                      # filter, ml-inference, llm, speech, communication,
#                      # recorder, visualization, simulator, debug
# keywords: []

runtime: {runtime}
entrypoint: {entrypoint}{detected_note}

# Typed contracts: declare your ports so dataflows are checked at compose
# time. Type URNs come from the standard library (types/std/) or your own
# `types:` definitions. Sources may have zero inputs; sinks zero outputs.
# (When uncommenting the examples, also remove the `{{}}` placeholder.)
inputs: {{}}
#  image:
#    type: std/media/v1/Image
#    description: BGR frame to process
outputs: {{}}
#  bbox:
#    type: std/vision/v1/BoundingBox
#    description: detected bounding boxes

# Documented configuration surface:
# env:
#   MODEL:
#     default: model.safetensors
#     description: model weights to load
"#
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn scaffold_parses_for_all_detection_paths() {
        for node in [
            DetectedNode {
                name: "dora-yolo".into(),
                runtime: "python",
                entrypoint: "dora-yolo".into(),
                description: Some("YOLO object detection".into()),
                source: Some("pyproject.toml"),
            },
            DetectedNode {
                name: "lidar".into(),
                runtime: "rust",
                entrypoint: "target/release/lidar".into(),
                description: None,
                source: Some("Cargo.toml"),
            },
            DetectedNode {
                name: "my-node".into(),
                runtime: "python",
                entrypoint: "my-node".into(),
                description: None,
                source: None,
            },
        ] {
            let content = render_manifest(&node, "acme");
            let manifest = NodeManifest::parse(&content)
                .unwrap_or_else(|e| panic!("scaffold must parse: {e:#}\n{content}"));
            assert_eq!(manifest.name.as_deref(), Some(node.name.as_str()));
            assert_eq!(manifest.namespace, "acme");
        }
    }

    #[test]
    fn detects_python_project() {
        let tmp = tempfile::tempdir().unwrap();
        std::fs::write(
            tmp.path().join("pyproject.toml"),
            r#"
[project]
name = "Dora_Yolo"
description = "object detection"

[project.scripts]
dora-yolo = "dora_yolo.main:main"
"#,
        )
        .unwrap();
        let node = detect_node(tmp.path());
        assert_eq!(node.name, "dora-yolo");
        assert_eq!(node.runtime, "python");
        assert_eq!(node.entrypoint, "dora-yolo");
        assert_eq!(node.description.as_deref(), Some("object detection"));
    }

    #[test]
    fn detects_rust_project() {
        let tmp = tempfile::tempdir().unwrap();
        std::fs::write(
            tmp.path().join("Cargo.toml"),
            "[package]\nname = \"lidar_driver\"\n",
        )
        .unwrap();
        let node = detect_node(tmp.path());
        assert_eq!(node.name, "lidar-driver");
        assert_eq!(node.runtime, "rust");
        assert_eq!(node.entrypoint, "target/release/lidar-driver");
    }

    #[test]
    fn normalizes_names() {
        assert_eq!(normalize_name("Dora_Yolo"), "dora-yolo");
        assert_eq!(normalize_name("  Node (v2)! "), "nodev2");
        assert_eq!(normalize_name("_foo_"), "foo");
        assert_eq!(normalize_name("(c++)"), "c");
        assert_eq!(normalize_name("(++)"), "");
    }

    #[test]
    fn detects_poetry_project() {
        let tmp = tempfile::tempdir().unwrap();
        std::fs::write(
            tmp.path().join("pyproject.toml"),
            "[tool.poetry]\nname = \"dora-lidar\"\ndescription = \"driver\"\n",
        )
        .unwrap();
        let node = detect_node(tmp.path());
        assert_eq!(node.name, "dora-lidar");
        assert_eq!(node.runtime, "python");
    }

    #[test]
    fn prefers_script_matching_package_name() {
        let tmp = tempfile::tempdir().unwrap();
        std::fs::write(
            tmp.path().join("pyproject.toml"),
            r#"
[project]
name = "my-node"

[project.scripts]
alpha-tool = "x:a"
my-node = "x:main"
"#,
        )
        .unwrap();
        assert_eq!(detect_node(tmp.path()).entrypoint, "my-node");
    }
}
