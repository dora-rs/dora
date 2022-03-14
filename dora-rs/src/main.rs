use dora_rs::descriptor::Descriptor;
use eyre::Context;
use std::{fs::File, path::PathBuf};
use structopt::StructOpt;

#[derive(Debug, Clone, StructOpt)]
struct Args {
    command: String,
    file: PathBuf,
}

fn main() -> eyre::Result<()> {
    let args = Args::from_args();
    let descriptor_file = File::open(&args.file).context("failed to open given file")?;

    let descriptor: Descriptor =
        serde_yaml::from_reader(descriptor_file).context("failed to parse given descriptor")?;

    match args.command.as_str() {
        "graph" => {
            let visualized = descriptor
                .visualize_as_mermaid()
                .context("failed to visualize descriptor")?;
            println!("{visualized}");
            println!(
                "Paste the above output on https://mermaid.live/ or in a \
        ```mermaid code block on GitHub to display it."
            );
        }
        "commands" => {
            let commands = descriptor
                .print_commands()
                .context("Failed to generate commands.")?;
            println!("{commands}");
        }
        _ => {
            unimplemented!()
        }
    };

    Ok(())
}
