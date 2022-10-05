use std::{fs, path::Path};

use dora_core::descriptor::Descriptor;
use eyre::Context;

const MERMAID_TEMPLATE: &str = include_str!("mermaid-template.html");

pub fn visualize_as_html(dataflow: &Path) -> eyre::Result<String> {
    let mermaid = visualize_as_mermaid(dataflow)?;
    Ok(MERMAID_TEMPLATE.replacen("____insert____", &mermaid, 1))
}

pub fn visualize_as_mermaid(dataflow: &Path) -> eyre::Result<String> {
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
