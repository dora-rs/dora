use eyre::{bail, Context};
use std::{
    fs,
    path::{Path, PathBuf},
};

use super::Kind;

const NODE_PY: &str = include_str!("node/node-template.py");
const TALKER_PY: &str = include_str!("talker/talker-template.py");
const LISTENER_PY: &str = include_str!("listener/listener-template.py");

pub fn create(args: super::CreateArgs) -> eyre::Result<()> {
    let super::CreateArgs {
        kind,
        lang: _,
        name,
        path,
    } = args;

    match kind {
        Kind::CustomNode => create_custom_node(name, path, NODE_PY),
        Kind::Dataflow => create_dataflow(name, path),
    }
}

fn create_custom_node(
    name: String,
    path: Option<PathBuf>,
    template_scripts: &str,
) -> Result<(), eyre::ErrReport> {
    // create directories
    let root = path.as_deref().unwrap_or_else(|| Path::new(&name));
    fs::create_dir(root)
        .with_context(|| format!("failed to create directory `{}`", root.display()))?;

    let node_path = root.join(format!("{name}.py"));
    fs::write(&node_path, template_scripts)
        .with_context(|| format!("failed to write `{}`", node_path.display()))?;

    println!(
        "Created new Python node `{name}` at {}",
        Path::new(".").join(root).display()
    );

    Ok(())
}

fn create_dataflow(name: String, path: Option<PathBuf>) -> Result<(), eyre::ErrReport> {
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

    create_custom_node("talker_1".into(), Some(root.join("talker_1")), TALKER_PY)?;
    create_custom_node("talker_2".into(), Some(root.join("talker_2")), TALKER_PY)?;
    create_custom_node(
        "listener_1".into(),
        Some(root.join("listener_1")),
        LISTENER_PY,
    )?;

    println!(
        "Created new yaml dataflow `{name}` at {}",
        Path::new(".").join(root).display()
    );

    Ok(())
}
