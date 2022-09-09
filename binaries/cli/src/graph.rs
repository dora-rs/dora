use std::{
    fs,
    path::{Path, PathBuf},
};

use dora_core::descriptor::Descriptor;
use eyre::Context;

pub fn run(dataflow: PathBuf) -> eyre::Result<()> {
    let descriptor = read_descriptor(&dataflow)
        .with_context(|| format!("failed to read dataflow at `{}`", dataflow.display()))?;
    let visualized = descriptor
        .visualize_as_mermaid()
        .context("failed to visualize descriptor")?;
    println!("{visualized}");
    println!(
        "Paste the above output on https://mermaid.live/ or in a \
        ```mermaid code block on GitHub to display it."
    );

    Ok(())
}

fn read_descriptor(file: &Path) -> eyre::Result<Descriptor> {
    let descriptor_file = fs::read(file).context("failed to open given file")?;
    let descriptor: Descriptor =
        serde_yaml::from_slice(&descriptor_file).context("failed to parse given descriptor")?;
    Ok(descriptor)
}
