use dora_rs::descriptor::Descriptor;
use eyre::{eyre, Context};
use futures::{stream::FuturesUnordered, StreamExt};
use futures_concurrency::Merge;
use std::path::{Path, PathBuf};
use structopt::StructOpt;
use tokio::io::{AsyncBufReadExt, BufReader};
use tokio_stream::wrappers::LinesStream;

#[derive(Debug, Clone, StructOpt)]
#[structopt(about = "Dora control")]
enum Command {
    #[structopt(about = "Print Graph")]
    Visualize { file: PathBuf },
    #[structopt(about = "Run Python server")]
    StartPython(dora_rs::python::server::PythonCommand),
    #[structopt(about = "Run dataflow pipeline")]
    Run { file: PathBuf },
}

#[tokio::main]
async fn main() -> eyre::Result<()> {
    env_logger::init();

    let command = Command::from_args();
    match command {
        Command::Visualize { file } => {
            let descriptor = read_descriptor(&file).await?;
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
        Command::Run { file } => {
            let descriptor = read_descriptor(&file).await?;

            let mut outputs = Vec::new();
            let tasks = FuturesUnordered::new();

            for source in &descriptor.sources {
                let mut command = tokio::process::Command::new(&source.run);
                command.stdout(std::process::Stdio::piped());

                let mut child = command
                    .spawn()
                    .with_context(|| format!("failed to spawn source {}", source.id))?;
                let stdout = child
                    .stdout
                    .take()
                    .ok_or_else(|| eyre!("failed to take stdout handle of source"))?;
                let reader = LinesStream::new(BufReader::new(stdout).lines());

                let source_id = source.id.clone();
                let result = tokio::spawn(async move {
                    let status = child.wait().await.context("child process failed")?;
                    if status.success() {
                        Ok(())
                    } else if let Some(code) = status.code() {
                        Err(eyre!("Source {source_id} failed with exit code: {code}"))
                    } else {
                        Err(eyre!("Source {source_id} failed (unknown exit code)"))
                    }
                });

                outputs.push(reader.map(|l| (source.output.clone(), l)));
                tasks.push(result);
            }

            // print all output for now (the eventual goal is to pass it to operators)
            let mut merged = outputs.merge();
            while let Some((name, line)) = merged.next().await {
                let output =
                    line.with_context(|| format!("failed to get next line of output {name}"))?;
                println!("Output {name}: {output}");
            }
        }
    }

    Ok(())
}

async fn read_descriptor(file: &Path) -> Result<Descriptor, eyre::Error> {
    let descriptor_file = tokio::fs::read(file)
        .await
        .context("failed to open given file")?;
    let descriptor: Descriptor =
        serde_yaml::from_slice(&descriptor_file).context("failed to parse given descriptor")?;
    Ok(descriptor)
}
