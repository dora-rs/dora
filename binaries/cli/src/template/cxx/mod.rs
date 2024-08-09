use eyre::{bail, Context, ContextCompat};
use std::{
    fs,
    path::{Path, PathBuf},
};

use super::Kind;

const NODE: &str = include_str!("node-template.cc");
const TALKER: &str = include_str!("talker-template.cc");
const LISTENER: &str = include_str!("listener-template.cc");

pub fn create(args: super::CreateArgs, use_path_deps: bool) -> eyre::Result<()> {
    let super::CreateArgs {
        kind,
        lang: _,
        name,
        path,
    } = args;

    match kind {
        Kind::CustomNode => create_custom_node(name, path, NODE),
        Kind::Dataflow => create_dataflow(name, path, use_path_deps),
    }
}

fn create_dataflow(
    name: String,
    path: Option<PathBuf>,
    use_path_deps: bool,
) -> Result<(), eyre::ErrReport> {
    const DATAFLOW_YML: &str = include_str!("dataflow-template.yml");

    if name.contains('/') {
        bail!("dataflow name must not contain `/` separators");
    }
    if !name.is_ascii() {
        bail!("dataflow name must be ASCII");
    }

    // create directories
    let root = path.as_deref().unwrap_or_else(|| Path::new(&name));
    fs::create_dir(root)
        .with_context(|| format!("failed to create directory `{}`", root.display()))?;

    let dataflow_yml = DATAFLOW_YML.replace("___name___", &name);
    let dataflow_yml_path = root.join("dataflow.yml");
    fs::write(&dataflow_yml_path, dataflow_yml)
        .with_context(|| format!("failed to write `{}`", dataflow_yml_path.display()))?;

    create_custom_node("talker_1".into(), Some(root.join("talker_1")), TALKER)?;
    create_custom_node("talker_2".into(), Some(root.join("talker_2")), TALKER)?;
    create_custom_node("listener_1".into(), Some(root.join("listener_1")), LISTENER)?;
    create_cmakefile(root.to_path_buf(), use_path_deps)?;

    println!(
        "Created new C++ dataflow at `{name}` at {}",
        Path::new(".").join(root).display()
    );

    Ok(())
}

fn create_cmakefile(root: PathBuf, use_path_deps: bool) -> Result<(), eyre::ErrReport> {
    const CMAKEFILE: &str = include_str!("cmake-template.txt");

    let cmake_file = if use_path_deps {
        let manifest_dir = Path::new(env!("CARGO_MANIFEST_DIR"));
        let workspace_dir = manifest_dir
            .parent()
            .context("Could not get manifest parent folder")?
            .parent()
            .context("Could not get manifest grandparent folder")?;
        CMAKEFILE.replace("__DORA_PATH__", workspace_dir.to_str().unwrap())
    } else {
        CMAKEFILE.replace("__DORA_PATH__", "")
    };

    let cmake_path = root.join("CMakeLists.txt");
    fs::write(&cmake_path, cmake_file)
        .with_context(|| format!("failed to write `{}`", cmake_path.display()))?;

    println!("Created new CMakeLists.txt at {}", cmake_path.display());
    Ok(())
}

fn create_custom_node(
    name: String,
    path: Option<PathBuf>,
    template_scripts: &str,
) -> Result<(), eyre::ErrReport> {
    if name.contains('/') {
        bail!("node name must not contain `/` separators");
    }
    if !name.is_ascii() {
        bail!("node name must be ASCII");
    }

    // create directories
    let root = path.as_deref().unwrap_or_else(|| Path::new(&name));
    fs::create_dir(root)
        .with_context(|| format!("failed to create directory `{}`", root.display()))?;

    let node_path = root.join("node.cc");
    fs::write(&node_path, template_scripts)
        .with_context(|| format!("failed to write `{}`", node_path.display()))?;

    // TODO: Makefile?

    println!(
        "Created new C++ custom node `{name}` at {}",
        Path::new(".").join(root).display()
    );

    Ok(())
}
