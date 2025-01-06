use eyre::{bail, Context};
use std::{
    fs,
    path::{Path, PathBuf},
};

const MAIN_PY: &str = include_str!("node-name/node_name/main.py");
const _MAIN_PY: &str = include_str!("node-name/node_name/__main__.py");
const _INIT_PY: &str = include_str!("node-name/node_name/__init__.py");
const _TEST_PY: &str = include_str!("node-name/tests/test_node_name.py");
const PYPROJECT_TOML: &str = include_str!("node-name/pyproject.toml");
const README_MD: &str = include_str!("node-name/README.md");

const TALKER_PY: &str = include_str!("talker/talker-template.py");
const LISTENER_PY: &str = include_str!("listener/listener-template.py");

pub fn create(args: crate::CommandNew) -> eyre::Result<()> {
    let crate::CommandNew {
        kind,
        lang: _,
        name,
        path,
    } = args;

    match kind {
        crate::Kind::CustomNode => create_custom_node(name, path, MAIN_PY),
        crate::Kind::Dataflow => create_dataflow(name, path),
    }
}

fn replace_space(file: &str, name: &str) -> String {
    let file = file.replace("node-name", &name.replace(" ", "-"));
    file.replace("node_name", &name.replace(" ", "_"))
}
fn create_custom_node(
    name: String,
    path: Option<PathBuf>,
    main: &str,
) -> Result<(), eyre::ErrReport> {
    // create directories
    let root = path.unwrap_or_else(|| PathBuf::from(name.replace(" ", "-")));
    let module_path = root.join(name.replace(" ", "_"));
    fs::create_dir(&root)
        .with_context(|| format!("failed to create directory `{}`", &root.display()))?;

    fs::create_dir(&module_path)
        .with_context(|| format!("failed to create directory `{}`", &root.display()))?;

    fs::create_dir(&root.join("tests"))
        .with_context(|| format!("failed to create directory `{}`", &root.display()))?;

    // PYPROJECT.toml
    let node_path = root.join("pyproject.toml");
    let pyproject = replace_space(PYPROJECT_TOML, &name);
    fs::write(&node_path, pyproject)
        .with_context(|| format!("failed to write `{}`", node_path.display()))?;

    // README.md
    let node_path = root.join("README.md");
    fs::write(&node_path, README_MD.replace("name", &name))
        .with_context(|| format!("failed to write `{}`", node_path.display()))?;

    // main.py
    let node_path = module_path.join("main.py");
    fs::write(&node_path, main)
        .with_context(|| format!("failed to write `{}`", node_path.display()))?;

    // __main__.py
    let node_path = module_path.join("__main__.py");
    fs::write(&node_path, _MAIN_PY)
        .with_context(|| format!("failed to write `{}`", node_path.display()))?;

    // __init__.py
    let node_path = module_path.join("__init__.py");
    fs::write(&node_path, _INIT_PY)
        .with_context(|| format!("failed to write `{}`", node_path.display()))?;

    // tests/tests_node_name.py
    let node_path = root
        .join("tests")
        .join(format!("test_{}.py", name.replace(" ", "_")));
    let file = replace_space(_TEST_PY, &name);
    fs::write(&node_path, file)
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

    create_custom_node("talker 1".into(), Some(root.join("talker-1")), TALKER_PY)?;
    create_custom_node("talker 2".into(), Some(root.join("talker-2")), TALKER_PY)?;
    create_custom_node(
        "listener 1".into(),
        Some(root.join("listener-1")),
        LISTENER_PY,
    )?;

    println!(
        "Created new yaml dataflow `{name}` at {}",
        Path::new(".").join(root).display()
    );

    Ok(())
}
