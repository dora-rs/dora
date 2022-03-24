use dora_rs::descriptor::Descriptor;
use eyre::{eyre, Context};
use futures::{stream::FuturesUnordered, StreamExt};
use std::{
    collections::BTreeMap,
    path::{Path, PathBuf},
};
use structopt::StructOpt;
use tokio_stream::wrappers::{ReceiverStream, TcpListenerStream};
use tokio_util::codec::{Framed, LengthDelimitedCodec};

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
        Command::Run { file } => run_dataflow(file).await?,
    }

    Ok(())
}

async fn run_dataflow(file: PathBuf) -> eyre::Result<()> {
    let descriptor = read_descriptor(&file).await?;

    let socket = tokio::net::TcpListener::bind("127.0.0.1:0")
        .await
        .wrap_err("failed to create TCP listener")?;
    let socket_addr = socket
        .local_addr()
        .wrap_err("failed to get socket address")?;

    let (outputs_tx, outputs) = tokio::sync::mpsc::channel(10);
    tokio::spawn(async move {
        let mut incoming = TcpListenerStream::new(socket);
        while let Some(connection) = incoming.next().await.transpose()? {
            let outputs_tx = outputs_tx.clone();
            tokio::spawn(async move {
                let mut framed = Framed::new(connection, LengthDelimitedCodec::new());
                while let Some(frame) = framed.next().await.transpose()? {
                    let deserialized: BTreeMap<String, Vec<u8>> = bincode::deserialize(&frame)
                        .wrap_err("failed to deserialize output message")?;
                    outputs_tx.send(deserialized).await?;
                }
                Result::<_, eyre::Error>::Ok(())
            });
        }
        Result::<_, eyre::Error>::Ok(())
    });

    let tasks = FuturesUnordered::new();
    for source in &descriptor.sources {
        let mut command = tokio::process::Command::new(&source.run);
        command.env("SERVER_SOCKET_ADDR", socket_addr.to_string());

        let mut child = command
            .spawn()
            .with_context(|| format!("failed to spawn source {}", source.id))?;

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
        tasks.push(result);
    }

    let mut outputs_stream = ReceiverStream::new(outputs);
    while let Some(output) = outputs_stream.next().await {
        println!("Output: {output:?}");
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
