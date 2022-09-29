use std::{
    fs,
    path::{Path, PathBuf},
};

use dora_core::descriptor::Descriptor;
use eyre::Context;

pub fn visualize_as_mermaid(dataflow: PathBuf) -> eyre::Result<String> {
    let descriptor = read_descriptor(&dataflow)
        .with_context(|| format!("failed to read dataflow at `{}`", dataflow.display()))?;
    let visualized = descriptor
        .visualize_as_mermaid()
        .context("failed to visualize descriptor")?;

    Ok(visualized)
}

pub fn read_descriptor(file: &Path) -> eyre::Result<Descriptor> {
    let descriptor_file = fs::read(file).context("failed to open given file")?;
    let descriptor: Descriptor =
        serde_yaml::from_slice(&descriptor_file).context("failed to parse given descriptor")?;
    Ok(descriptor)
}
