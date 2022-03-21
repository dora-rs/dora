use dora_rs::descriptor::Descriptor;
use eyre::Context;
use std::{fs::File, path::PathBuf};
use structopt::StructOpt;

#[derive(Debug, Clone, StructOpt)]
struct Args {
    file: PathBuf,
}

fn main() -> eyre::Result<()> {
    let args = Args::from_args();
    let descriptor_file = File::open(&args.file).context("failed to open given file")?;

    let descriptor: Descriptor =
        serde_yaml::from_reader(descriptor_file).context("failed to parse given descriptor")?;

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
