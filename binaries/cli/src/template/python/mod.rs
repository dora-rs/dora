use eyre::{Context, bail};
use std::{
    fs,
    path::{Path, PathBuf},
};

const MAIN_PY: &str = include_str!("__node-name__/__node_name__/main.py");
const _MAIN_PY: &str = include_str!("__node-name__/__node_name__/__main__.py");
const _INIT_PY: &str = include_str!("__node-name__/__node_name__/__init__.py");
const _TEST_PY: &str = include_str!("__node-name__/tests/test___node_name__.py");
const PYPROJECT_TOML: &str = include_str!("__node-name__/pyproject.toml");
const README_MD: &str = include_str!("__node-name__/README.md");

const TALKER_PY: &str = include_str!("talker/talker-template.py");
const LISTENER_PY: &str = include_str!("listener/listener-template.py");
const WORKSPACE_PYPROJECT: &str = include_str!("workspace-pyproject-template.toml");

pub fn create(args: crate::CommandNew, use_path_deps: bool) -> eyre::Result<()> {
    let crate::CommandNew {
        kind,
        lang: _,
        name,
        path,
    } = args;

    match kind {
        crate::Kind::Node => create_custom_node(name, path, MAIN_PY),
        crate::Kind::Dataflow => create_dataflow(name, path, use_path_deps),
    }
}

fn replace_space(file: &str, name: &str) -> String {
    let mut file = file.replace("__node-name__", &name.replace(" ", "-"));
    file = file.replace("__node_name__", &name.replace("-", "_").replace(" ", "_"));
    file.replace("Node Name", name)
}
fn create_custom_node(
    name: String,
    path: Option<PathBuf>,
    main: &str,
) -> Result<(), eyre::ErrReport> {
    // create directories
    let root = path.unwrap_or_else(|| PathBuf::from(name.replace(" ", "-")));
    let module_path = root.join(name.replace(" ", "_").replace("-", "_"));
    fs::create_dir(&root)
        .with_context(|| format!("failed to create root directory `{}`", &root.display()))?;

    fs::create_dir(&module_path)
        .with_context(|| format!("failed to create module directory `{}`", &root.display()))?;

    fs::create_dir(root.join("tests"))
        .with_context(|| format!("failed to create tests directory `{}`", &root.display()))?;

    // PYPROJECT.toml
    let node_path = root.join("pyproject.toml");
    let pyproject = replace_space(PYPROJECT_TOML, &name);
    fs::write(&node_path, pyproject)
        .with_context(|| format!("failed to write `{}`", node_path.display()))?;

    // README.md
    let node_path = root.join("README.md");
    fs::write(&node_path, README_MD.replace("Node Name", &name))
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

    // tests/tests___node_name__.py
    let node_path = root.join("tests").join(format!(
        "test_{}.py",
        name.replace(" ", "_").replace("-", "_")
    ));
    let file = replace_space(_TEST_PY, &name);
    fs::write(&node_path, file)
        .with_context(|| format!("failed to write `{}`", node_path.display()))?;

    println!(
        "Created new Python node `{name}` at {}",
        Path::new(".").join(&root).display()
    );
    println!("   cd {}", Path::new(".").join(&root).display());
    println!("   uv pip install -e . # Install",);
    println!("   uv run ruff check . --fix # Format");
    println!("   uv run ruff check . # Lint",);
    println!("   uv run pytest . # Test");

    Ok(())
}

fn create_dataflow(
    name: String,
    path: Option<PathBuf>,
    use_path_deps: bool,
) -> Result<(), eyre::ErrReport> {
    const DATAFLOW_YML: &str = include_str!("dataflow-template.yml");
    const WORKSPACE_README: &str = include_str!("README.md");

    if name.contains('/') {
        bail!("dataflow name must not contain `/` separators");
    }
    if !name.is_ascii() {
        bail!("dataflow name must be ASCII");
    }

    // create directories
    let root = path.as_deref().unwrap_or_else(|| Path::new(&name));
    fs::create_dir(root)
        .with_context(|| format!("failed to create module directory `{}`", root.display()))?;

    let dataflow_yml = DATAFLOW_YML.replace("___name___", &name);
    let dataflow_yml_path = root.join("dataflow.yml");
    fs::write(&dataflow_yml_path, dataflow_yml)
        .with_context(|| format!("failed to write `{}`", dataflow_yml_path.display()))?;

    let readme_path = root.join("README.md");
    fs::write(&readme_path, WORKSPACE_README)
        .with_context(|| format!("failed to write `{}`", readme_path.display()))?;

    let pyproject_path = root.join("pyproject.toml");
    // `___DORA_RS_PATH_SOURCE___` is the placeholder inside the
    // existing `[tool.uv.sources]` block — replace it with the
    // path-pinned line in CI mode, drop it otherwise. We cannot append
    // a second `[tool.uv.sources]` header at the end of the file
    // (duplicate section = TOML parse error), so this stays an in-place
    // substitution rather than a `push_str`.
    let dora_rs_source = if use_path_deps {
        "dora-rs = { path = \"../apis/python/node\", editable = true }"
    } else {
        ""
    };
    let pyproject = WORKSPACE_PYPROJECT
        .replace("___name___", &name)
        .replace("___DORA_RS_PATH_SOURCE___", dora_rs_source);
    fs::write(&pyproject_path, pyproject)
        .with_context(|| format!("failed to write `{}`", pyproject_path.display()))?;

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
