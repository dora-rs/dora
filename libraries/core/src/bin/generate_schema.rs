use std::{
    env, fs,
    path::{Path, PathBuf},
};

use dora_core::manifest::NodeManifest;
use dora_message::descriptor::Descriptor;
use schemars::schema_for;

/// The workspace root, i.e. where the second published copy of the descriptor
/// schema lives.
///
/// `CARGO_MANIFEST_DIR` is `libraries/core`, so the root is two levels up. That
/// is checked rather than assumed: this binary ships inside the published
/// `dora-core` crate, where `../..` is a registry directory that must not be
/// written to. Returns `None` when the parent is not the dora workspace.
fn workspace_root(manifest_dir: &Path) -> Option<PathBuf> {
    let root = manifest_dir.parent()?.parent()?;
    let cargo_toml = fs::read_to_string(root.join("Cargo.toml")).ok()?;
    cargo_toml
        .contains("[workspace]")
        .then(|| root.to_path_buf())
}

fn main() {
    let schema = schema_for!(Descriptor);
    let raw_schema =
        serde_json::to_string_pretty(&schema).expect("Could not serialize schema to json");

    // Add additional properties to True, as #[derive(transparent)] of enums are not well handled.
    //
    // 'OneOf' such as Custom Nodes, Operators and Single Operators overwrite property values of the initial struct `Nodes`.`
    // which make the original properties such as `id` and `name` not validated by IDE extensions.
    let raw_schema = raw_schema.replace(
        "\"additionalProperties\": false",
        "\"additionalProperties\": true",
    );

    // Remove `serde(from=` nested field as they are not handled properly by `schemars`
    let raw_schema = raw_schema.replace(
        "\"python\": {
              \"$ref\": \"#/definitions/PythonSource\"
            }",
        "",
    );
    let raw_schema = raw_schema.replace(
        "{
            \"$ref\": \"#/definitions/Input\"
          }",
        "true",
    );

    // Get the Cargo root manifest directory
    let manifest_dir = env::var("CARGO_MANIFEST_DIR").expect("CARGO_MANIFEST_DIR is not set");
    let manifest_dir = Path::new(&manifest_dir);

    // Create the path for the new file next to Cargo.toml
    let new_file_path = manifest_dir.join("dora-schema.json");

    // write to file
    fs::write(new_file_path, &raw_schema).expect("Could not write schema to file");

    // The repo root holds a second copy, and it is the one the guide hands out
    // as a `$schema` URL, so it has to be regenerated in the same breath (#3088).
    if let Some(root) = workspace_root(manifest_dir) {
        fs::write(root.join("dora-schema.json"), &raw_schema)
            .expect("Could not write schema to workspace root");
    }

    // Node manifest schema (dora-node.yml, see docs/plan-node-hub.md §5)
    let node_schema = schema_for!(NodeManifest);
    let raw_node_schema = serde_json::to_string_pretty(&node_schema)
        .expect("Could not serialize node manifest schema to json");
    let node_schema_path = manifest_dir.join("dora-node-schema.json");
    fs::write(node_schema_path, raw_node_schema)
        .expect("Could not write node manifest schema to file");
}

#[cfg(test)]
mod tests {
    use super::workspace_root;
    use std::{fs, path::Path};

    /// The descriptor schema is published at two URLs: the repo-root copy that
    /// `docs/yaml-spec.md` and `guide/src/concepts/dataflow-yaml.md` paste into
    /// users' `# yaml-language-server: $schema=` lines, and the
    /// `libraries/core` copy that `libraries/core/README.md` points at. Both are
    /// live, so they must not drift (#3088 — the root copy silently stopped
    /// tracking the descriptor after #2512).
    #[test]
    fn root_schema_matches_crate_copy() {
        let manifest_dir = Path::new(env!("CARGO_MANIFEST_DIR"));
        // No repo above the packaged crate — nothing to keep in sync there.
        let Ok(root_copy) = fs::read_to_string(manifest_dir.join("../../dora-schema.json")) else {
            return;
        };

        // The root copy is only rewritten when `main` recognizes the workspace,
        // and a failed detection is silent, so pin it here too.
        assert!(
            workspace_root(manifest_dir).is_some(),
            "the repo-root `dora-schema.json` exists, but `workspace_root` did \
             not recognize the workspace, so `generate_schema` would skip it"
        );

        let crate_copy = fs::read_to_string(manifest_dir.join("dora-schema.json"))
            .expect("libraries/core/dora-schema.json is missing");

        assert!(
            root_copy == crate_copy,
            "the repo-root `dora-schema.json` drifted from \
             `libraries/core/dora-schema.json`; run \
             `cargo run -p dora-core --bin generate_schema` to rewrite both"
        );
    }

    /// Guards the registry case: `<crate>/../..` there is somebody else's
    /// directory, and a stray `dora-schema.json` must not be written into it.
    #[test]
    fn workspace_root_rejects_a_plain_package_parent() {
        let dir = tempfile::tempdir().unwrap();
        let manifest_dir = dir.path().join("libraries/core");
        fs::create_dir_all(&manifest_dir).unwrap();
        fs::write(
            dir.path().join("Cargo.toml"),
            "[package]\nname = \"other\"\n",
        )
        .unwrap();

        assert_eq!(workspace_root(&manifest_dir), None);

        fs::write(dir.path().join("Cargo.toml"), "[workspace]\nmembers = []\n").unwrap();
        assert_eq!(workspace_root(&manifest_dir).as_deref(), Some(dir.path()));
    }
}
