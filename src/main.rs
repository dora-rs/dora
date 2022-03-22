use dora_rs::{descriptor::Descriptor, server::start_server};
use eyre::{Context, ContextCompat, Result};
use pyo3::{prelude::*, prepare_freethreaded_python};
use std::{fs::File, path::PathBuf};
use structopt::StructOpt;

#[derive(Debug, Clone, StructOpt)]
#[structopt(about = "Dora control")]
enum Command {
    #[structopt(about = "Print Graph")]
    Graph { file: PathBuf },
    #[structopt(about = "Run Python server")]
    StartPython { server: String },
}

fn main() -> Result<()> {
    let command = Command::from_args();
    match command {
        Command::Graph { file } => {
            let descriptor_file = File::open(&file)
                .context("failed to open given file")
                .unwrap();

            let descriptor: Descriptor = serde_yaml::from_reader(descriptor_file)
                .context("failed to parse given descriptor")
                .unwrap();
            let visualized = descriptor
                .visualize_as_mermaid()
                .context("failed to visualize descriptor")
                .unwrap();
            println!("{visualized}");
            println!(
                "Paste the above output on https://mermaid.live/ or in a \
        ```mermaid code block on GitHub to display it."
            );
        }
        Command::StartPython { server } => {
            let mut server = server.split(":");
            let file = server.next().context("Server string is empty.").unwrap();
            let app = server.next().context("No app found").unwrap();
            let rt = tokio::runtime::Runtime::new().unwrap();
            prepare_freethreaded_python();
            rt.block_on(async {
                let _result = start_server(file, app).await;
            });
        }
    }

    Ok(())
}
