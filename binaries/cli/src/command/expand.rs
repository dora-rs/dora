use super::Executable;
use dora_core::descriptor::{Descriptor, DescriptorExt, check_module_file, without_path_fallback};
use eyre::Context;
use std::path::{Path, PathBuf};

#[derive(Debug, clap::Args)]
/// Expand module references in a dataflow and print the flat result
pub struct Expand {
    /// Path to the dataflow descriptor file (or module file with --module)
    #[clap(value_name = "PATH", value_hint = clap::ValueHint::FilePath)]
    dataflow: PathBuf,
    /// Validate a standalone module file instead of a full dataflow
    #[clap(long, action)]
    module: bool,
}

impl Executable for Expand {
    fn execute(self) -> eyre::Result<()> {
        if self.module {
            check_module_file(&self.dataflow)?;
            println!("Module file is valid: {}", self.dataflow.display());
            return Ok(());
        }

        let yaml = expanded_yaml(&self.dataflow)?;
        print!("{yaml}");
        // serde_yaml includes a trailing newline; if not, ensure one for
        // shell pipeline composability
        if !yaml.ends_with('\n') {
            println!();
        }
        Ok(())
    }
}

fn expanded_yaml(dataflow: &Path) -> eyre::Result<String> {
    let working_dir = dataflow
        .parent()
        .filter(|p| !p.as_os_str().is_empty())
        .unwrap_or_else(|| Path::new("."));
    let descriptor = Descriptor::blocking_read(dataflow)
        .with_context(|| format!("failed to read dataflow at `{}`", dataflow.display()))?
        .expand(working_dir)
        .context("failed to expand modules")?;
    // `path_fallback` is internal, and a dataflow file that sets it is
    // rejected; leave it out so the output can be run like the input.
    serde_yaml::to_string(&without_path_fallback(&descriptor))
        .context("failed to serialize expanded descriptor")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn module_dir_reaches_the_same_file_from_different_dataflows() {
        let dir = tempfile::tempdir().unwrap();
        let nested = dir.path().join("flows");
        let module = nested.join("worker module");
        std::fs::create_dir_all(&module).unwrap();
        std::fs::write(module.join("config.txt"), "worker config").unwrap();
        let worker = module.join("worker.yml");
        std::fs::write(
            &worker,
            r#"module:
  name: worker
nodes:
  - id: worker
    path: dynamic
    build: 'prepare "$MODULE_DIR/config.txt"'
    args: '"$MODULE_DIR/config.txt" $MODULE_DIR_SUFFIX'
    env:
      CONFIG: $MODULE_DIR/config.txt
      LITERAL: $$MODULE_DIR
      ADJACENT: $PATH$MODULE_DIR/config.txt
  - id: version
    path: dynamic
    args: 1.10
"#,
        )
        .unwrap();

        for (base, prefix) in [
            (dir.path(), "flows/worker module"),
            (&nested, "worker module"),
        ] {
            let dataflow = base.join("dataflow.yml");
            std::fs::write(
                &dataflow,
                format!("nodes:\n  - id: m\n    module: {prefix}/worker.yml\n"),
            )
            .unwrap();
            let flat = Descriptor::blocking_read(&dataflow)
                .unwrap()
                .expand(base)
                .unwrap();
            let node = &flat.nodes[0];
            let file = format!("{prefix}/config.txt");
            assert_eq!(
                node.args.as_ref().unwrap(),
                &format!("\"{file}\" $MODULE_DIR_SUFFIX")
            );
            assert_eq!(node.build.as_ref().unwrap(), &format!("prepare \"{file}\""));
            let env = node.env.as_ref().unwrap();
            assert_eq!(env["CONFIG"].to_string(), file);
            assert_eq!(
                std::fs::read_to_string(base.join(file)).unwrap(),
                "worker config"
            );
            assert_eq!(env["LITERAL"].to_string(), "$MODULE_DIR");
            assert_eq!(
                env["ADJACENT"].to_string(),
                format!("{}{prefix}/config.txt", std::env::var("PATH").unwrap())
            );
            assert_eq!(flat.nodes[1].args.as_deref(), Some("1.10"));
        }
        check_module_file(&worker).unwrap();
    }

    #[cfg(unix)]
    #[test]
    fn module_dir_through_symlink_reaches_the_modules_sibling() {
        let dir = tempfile::tempdir().unwrap();
        let modules = dir.path().join("modules");
        let worker = dir.path().join("nodes/worker");
        let dataflow_dir = dir.path().join("dataflows/a");
        for path in [&modules, &worker, &dataflow_dir] {
            std::fs::create_dir_all(path).unwrap();
        }
        std::fs::write(worker.join("script.py"), "worker script").unwrap();
        std::fs::write(
            modules.join("worker.yml"),
            "module:\n  name: worker\nnodes:\n  - id: worker\n    path: python\n    args: $MODULE_DIR/../nodes/worker/script.py\n",
        )
        .unwrap();
        std::os::unix::fs::symlink("../../modules", dataflow_dir.join("modules")).unwrap();
        let dataflow = dataflow_dir.join("dataflow.yml");
        std::fs::write(
            &dataflow,
            "nodes:\n  - id: m\n    module: modules/worker.yml\n",
        )
        .unwrap();
        let flat: Descriptor = serde_yaml::from_str(&expanded_yaml(&dataflow).unwrap()).unwrap();
        let script = flat.nodes[0].args.as_ref().unwrap();
        assert_eq!(
            std::fs::read_to_string(dataflow_dir.join(script)).unwrap(),
            "worker script"
        );
    }

    #[test]
    fn nested_module_dir_belongs_to_the_declaring_file() {
        let dir = tempfile::tempdir().unwrap();
        std::fs::create_dir(dir.path().join("child")).unwrap();
        std::fs::write(
            dir.path().join("outer.yml"),
            r#"module:
  name: outer
build: prepare $MODULE_DIR/outer
nodes:
  - id: inner
    module: child/inner.yml
    env:
      PARENT: $MODULE_DIR/parent
"#,
        )
        .unwrap();
        std::fs::write(
            dir.path().join("child/inner.yml"),
            r#"module:
  name: inner
nodes:
  - id: worker
    path: dynamic
    build: prepare ${MODULE_DIR}/inner
    args: $MODULE_DIR/worker $PARAM_CONFIG
"#,
        )
        .unwrap();
        let dataflow = dir.path().join("dataflow.yml");
        std::fs::write(
            &dataflow,
            "nodes:\n  - id: outer\n    module: outer.yml\n    params:\n      config: user/config\n",
        )
        .unwrap();
        let flat: Descriptor = serde_yaml::from_str(&expanded_yaml(&dataflow).unwrap()).unwrap();
        let node = &flat.nodes[0];
        assert_eq!(
            node.build.as_deref(),
            Some("prepare ./outer\nprepare child/inner")
        );
        assert_eq!(node.args.as_deref(), Some("child/worker user/config"));
        assert_eq!(node.env.as_ref().unwrap()["PARENT"].to_string(), "./parent");
    }

    /// The output is a dataflow file the user can run, as on main, so it
    /// leaves out the internal `path_fallback` that a dataflow file must not
    /// set.
    #[test]
    fn expanded_output_can_be_read_back() {
        let dir = tempfile::tempdir().unwrap();
        let modules = dir.path().join("modules");
        std::fs::create_dir(&modules).unwrap();
        let tool = format!("tool{}", std::env::consts::EXE_SUFFIX);
        std::fs::write(modules.join(tool), "").unwrap();
        std::fs::write(
            modules.join("m.yml"),
            "module:\n  name: m\nnodes:\n  - id: tool\n    path: tool\n",
        )
        .unwrap();
        let dataflow = dir.path().join("dataflow.yml");
        std::fs::write(&dataflow, "nodes:\n  - id: m\n    module: modules/m.yml\n").unwrap();

        let yaml = expanded_yaml(&dataflow).unwrap();
        let expanded = dir.path().join("expanded.yml");
        std::fs::write(&expanded, &yaml).unwrap();
        let read_back = Descriptor::blocking_read(&expanded).unwrap();
        assert_eq!(read_back.nodes.len(), 1);
        assert_eq!(read_back.nodes[0].id.to_string(), "m.tool");
        assert_eq!(
            read_back.nodes[0].path.as_deref().map(Path::new),
            Some(Path::new("modules").join("tool").as_path())
        );
        read_back.check(dir.path()).unwrap();
    }
}
