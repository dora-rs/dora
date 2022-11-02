use eyre::Context;
use std::{
    fs,
    path::{Path, PathBuf},
};

pub fn create(args: crate::CommandNew) -> eyre::Result<()> {
    let crate::CommandNew {
        kind,
        lang: _,
        name,
        path,
    } = args;

    match kind {
        crate::Kind::Operator => create_operator(name, path),
        crate::Kind::CustomNode => create_custom_node(name, path),
    }
}

fn create_operator(name: String, path: Option<PathBuf>) -> Result<(), eyre::ErrReport> {
    const OPERATOR_PY: &str = include_str!("operator/operator-template.py");

    // create directories
    let root = path.as_deref().unwrap_or_else(|| Path::new(&name));
    fs::create_dir(&root)
        .with_context(|| format!("failed to create directory `{}`", root.display()))?;

    let operator_path = root.join("operator.py");
    fs::write(&operator_path, OPERATOR_PY)
        .with_context(|| format!("failed to write `{}`", operator_path.display()))?;

    println!(
        "Created new Python operator `{name}` at {}",
        Path::new(".").join(root).display()
    );

    Ok(())
}
fn create_custom_node(name: String, path: Option<PathBuf>) -> Result<(), eyre::ErrReport> {
    const NODE_PY: &str = include_str!("node/node-template.py");

    // create directories
    let root = path.as_deref().unwrap_or_else(|| Path::new(&name));
    fs::create_dir(&root)
        .with_context(|| format!("failed to create directory `{}`", root.display()))?;

    let node_path = root.join("node.py");
    fs::write(&node_path, NODE_PY)
        .with_context(|| format!("failed to write `{}`", node_path.display()))?;

    println!(
        "Created new Python node `{name}` at {}",
        Path::new(".").join(root).display()
    );

    Ok(())
}
