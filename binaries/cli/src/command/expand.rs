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
