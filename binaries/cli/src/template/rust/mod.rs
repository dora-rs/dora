use eyre::{bail, Context};
use std::{
    fs,
    path::{Path, PathBuf},
};

const MAIN_RS: &str = include_str!("node/main-template.rs");
const TALKER_RS: &str = include_str!("talker/main-template.rs");
const LISTENER_RS: &str = include_str!("listener/main-template.rs");

const VERSION: &str = env!("CARGO_PKG_VERSION");
pub fn create(args: crate::CommandNew, use_path_deps: bool) -> eyre::Result<()> {
    let crate::CommandNew {
        kind,
        lang: _,
        name,
        path,
    } = args;

    match kind {
        crate::Kind::Operator => {
            bail!("Operators are going to be depreciated, please don't use it")
        }
        crate::Kind::CustomNode => create_custom_node(name, path, use_path_deps, MAIN_RS),
        crate::Kind::Dataflow => create_dataflow(name, path, use_path_deps),
    }
}

fn create_dataflow(
    name: String,
    path: Option<PathBuf>,
    use_path_deps: bool,
) -> Result<(), eyre::ErrReport> {
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
    fs::create_dir(root)
        .with_context(|| format!("failed to create directory `{}`", root.display()))?;

    let dataflow_yml = DATAFLOW_YML.replace("___name___", &name);
    let dataflow_yml_path = root.join("dataflow.yml");
    fs::write(&dataflow_yml_path, dataflow_yml)
        .with_context(|| format!("failed to write `{}`", dataflow_yml_path.display()))?;
    let cargo_toml = WORKSPACE_CARGO_TOML.replace("___name___", &name);
    let cargo_toml_path = root.join("Cargo.toml");
    fs::write(&cargo_toml_path, cargo_toml)
        .with_context(|| format!("failed to write `{}`", cargo_toml_path.display()))?;

    create_custom_node(
        "talker_1".into(),
        Some(root.join("talker_1")),
        use_path_deps,
        TALKER_RS,
    )?;
    create_custom_node(
        "talker_2".into(),
        Some(root.join("talker_2")),
        use_path_deps,
        TALKER_RS,
    )?;
    create_custom_node(
        "listener_1".into(),
        Some(root.join("listener_1")),
        use_path_deps,
        LISTENER_RS,
    )?;

    println!(
        "Created new Rust dataflow at `{name}` at {}",
        Path::new(".").join(root).display()
    );

    Ok(())
}

#[deprecated(since = "0.3.4")]
#[allow(unused)]
fn create_operator(
    name: String,
    path: Option<PathBuf>,
    use_path_deps: bool,
) -> Result<(), eyre::ErrReport> {
    const CARGO_TOML: &str = include_str!("operator/Cargo-template.toml");
    const LIB_RS: &str = include_str!("operator/lib-template.rs");

    if name.contains('/') {
        bail!("operator name must not contain `/` separators");
    }
    if name.contains('-') {
        bail!(
            "operator name must not contain `-` separators as 
        it get replaced by `_` as a static library."
        );
    }
    if !name.is_ascii() {
        bail!("operator name must be ASCII");
    }

    // create directories
    let root = path.as_deref().unwrap_or_else(|| Path::new(&name));
    fs::create_dir(root)
        .with_context(|| format!("failed to create directory `{}`", root.display()))?;
    let src = root.join("src");
    fs::create_dir(&src)
        .with_context(|| format!("failed to create directory `{}`", src.display()))?;

    let dep = if use_path_deps {
        r#"dora-operator-api = { path = "../../apis/rust/operator" }"#.to_string()
    } else {
        format!(r#"dora-operator-api = "{VERSION}""#)
    };
    let cargo_toml = CARGO_TOML
        .replace("___name___", &name)
        .replace("dora-operator-api = {}", &dep);

    let cargo_toml_path = root.join("Cargo.toml");
    fs::write(&cargo_toml_path, cargo_toml)
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

fn create_custom_node(
    name: String,
    path: Option<PathBuf>,
    use_path_deps: bool,
    template_scripts: &str,
) -> Result<(), eyre::ErrReport> {
    const CARGO_TOML: &str = include_str!("node/Cargo-template.toml");

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
    let src = root.join("src");
    fs::create_dir(&src)
        .with_context(|| format!("failed to create directory `{}`", src.display()))?;

    let dep = if use_path_deps {
        r#"dora-node-api = { path = "../../apis/rust/node" }"#.to_string()
    } else {
        format!(r#"dora-node-api = "{VERSION}""#)
    };
    let cargo_toml = CARGO_TOML
        .replace("___name___", &name)
        .replace("dora-node-api = {}", &dep);
    let cargo_toml_path = root.join("Cargo.toml");
    fs::write(&cargo_toml_path, cargo_toml)
        .with_context(|| format!("failed to write `{}`", cargo_toml_path.display()))?;

    let main_rs_path = src.join("main.rs");
    fs::write(&main_rs_path, template_scripts)
        .with_context(|| format!("failed to write `{}`", main_rs_path.display()))?;

    println!(
        "Created new Rust custom node `{name}` at {}",
        Path::new(".").join(root).display()
    );

    Ok(())
}
