use eyre::{bail, Context};
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
        crate::Kind::Dataflow => create_dataflow(name, path),
    }
}

fn create_dataflow(name: String, path: Option<PathBuf>) -> Result<(), eyre::ErrReport> {
    const DATAFLOW_YML: &str = include_str!("dataflow-template.yml");
    const WORKSPACE_CARGO_TOML: &str = include_str!("Cargo-template.toml");

    if name.contains('/') {
        bail!("dataflow name must not contain `/` separators");
    }
    if !name.is_ascii() {
        bail!("dataflow name must be ASCII");
    }

    // create directories
    let root = path.as_deref().unwrap_or_else(|| Path::new(&name));
    fs::create_dir(&root)
        .with_context(|| format!("failed to create directory `{}`", root.display()))?;

    let dataflow_yml = DATAFLOW_YML.replace("___name___", &name);
    let dataflow_yml_path = root.join("dataflow.yml");
    fs::write(&dataflow_yml_path, &dataflow_yml)
        .with_context(|| format!("failed to write `{}`", dataflow_yml_path.display()))?;
    let cargo_toml = WORKSPACE_CARGO_TOML.replace("___name___", &name);
    let cargo_toml_path = root.join("Cargo.toml");
    fs::write(&cargo_toml_path, &cargo_toml)
        .with_context(|| format!("failed to write `{}`", cargo_toml_path.display()))?;

    create_operator("op-1".into(), Some(root.join("op-1")))?;
    create_operator("op-2".into(), Some(root.join("op-2")))?;
    create_custom_node("node-1".into(), Some(root.join("node-1")))?;

    println!(
        "Created new Rust dataflow at `{name}` at {}",
        Path::new(".").join(root).display()
    );

    Ok(())
}

fn create_operator(name: String, path: Option<PathBuf>) -> Result<(), eyre::ErrReport> {
    const CARGO_TOML: &str = include_str!("operator/Cargo-template.toml");
    const LIB_RS: &str = include_str!("operator/lib-template.rs");

    if name.contains('/') {
        bail!("operator name must not contain `/` separators");
    }
    if !name.is_ascii() {
        bail!("operator name must be ASCII");
    }

    // create directories
    let root = path.as_deref().unwrap_or_else(|| Path::new(&name));
    fs::create_dir(&root)
        .with_context(|| format!("failed to create directory `{}`", root.display()))?;
    let src = root.join("src");
    fs::create_dir(&src)
        .with_context(|| format!("failed to create directory `{}`", src.display()))?;

    let cargo_toml = CARGO_TOML.replace("___name___", &name);
    let cargo_toml_path = root.join("Cargo.toml");
    fs::write(&cargo_toml_path, &cargo_toml)
        .with_context(|| format!("failed to write `{}`", cargo_toml_path.display()))?;

    let lib_rs_path = src.join("lib.rs");
    fs::write(&lib_rs_path, LIB_RS)
        .with_context(|| format!("failed to write `{}`", lib_rs_path.display()))?;

    println!(
        "Created new Rust operator `{name}` at {}",
        Path::new(".").join(root).display()
    );

    Ok(())
}

fn create_custom_node(name: String, path: Option<PathBuf>) -> Result<(), eyre::ErrReport> {
    const CARGO_TOML: &str = include_str!("node/Cargo-template.toml");
    const MAIN_RS: &str = include_str!("node/main-template.rs");

    if name.contains('/') {
        bail!("node name must not contain `/` separators");
    }
    if !name.is_ascii() {
        bail!("node name must be ASCII");
    }

    // create directories
    let root = path.as_deref().unwrap_or_else(|| Path::new(&name));
    fs::create_dir(&root)
        .with_context(|| format!("failed to create directory `{}`", root.display()))?;
    let src = root.join("src");
    fs::create_dir(&src)
        .with_context(|| format!("failed to create directory `{}`", src.display()))?;

    let cargo_toml = CARGO_TOML.replace("___name___", &name);
    let cargo_toml_path = root.join("Cargo.toml");
    fs::write(&cargo_toml_path, &cargo_toml)
        .with_context(|| format!("failed to write `{}`", cargo_toml_path.display()))?;

    let main_rs_path = src.join("main.rs");
    fs::write(&main_rs_path, MAIN_RS)
        .with_context(|| format!("failed to write `{}`", main_rs_path.display()))?;

    println!(
        "Created new Rust custom node `{name}` at {}",
        Path::new(".").join(root).display()
    );

    Ok(())
}
