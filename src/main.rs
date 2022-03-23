use dora_rs::descriptor::Descriptor;
use eyre::Context;
use std::{fs::File, path::PathBuf};
use structopt::StructOpt;

#[derive(Debug, Clone, StructOpt)]
#[structopt(about = "Dora control")]
enum Command {
    #[structopt(about = "Print Graph")]
    Graph { file: PathBuf },
    #[structopt(about = "Run Python server")]
    StartPython(dora_rs::python::server::PythonCommand),
}

fn main() -> eyre::Result<()> {
    env_logger::init();

    let command = Command::from_args();
    match command {
        Command::Graph { file } => {
            let descriptor_file = File::open(&file).context("failed to open given file")?;

            let descriptor: Descriptor = serde_yaml::from_reader(descriptor_file)
                .context("failed to parse given descriptor")?;
            let visualized = descriptor
                .visualize_as_mermaid()
                .context("failed to visualize descriptor")?;
            println!("{visualized}");
            println!(
                "Paste the above output on https://mermaid.live/ or in a \
        ```mermaid code block on GitHub to display it."
            );
        }
        Command::StartPython(command) => {
            dora_rs::python::server::run(command).context("python server failed")?;
        }
    }

    Ok(())
}
