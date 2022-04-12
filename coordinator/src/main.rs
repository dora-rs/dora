use descriptor::Descriptor;
use eyre::{eyre, WrapErr};
use futures::{stream::FuturesUnordered, StreamExt};
use std::path::{Path, PathBuf};

mod descriptor;

#[derive(Debug, Clone, clap::Parser)]
#[clap(about = "Dora coordinator")]
enum Command {
    #[clap(about = "Print Graph")]
    Visualize { file: PathBuf },
    #[clap(about = "Run dataflow pipeline")]
    Run { file: PathBuf },
}

#[tokio::main]
async fn main() -> eyre::Result<()> {
    let command = clap::Parser::parse();
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
        Command::Run { file } => run_dataflow(file.clone())
            .await
            .wrap_err_with(|| format!("failed to run dataflow at {}", file.display()))?,
    }

    Ok(())
}

async fn run_dataflow(file: PathBuf) -> eyre::Result<()> {
    let Descriptor {
        mut communication,
        operators,
    } = read_descriptor(&file)
        .await
        .wrap_err_with(|| format!("failed to read dataflow descriptor at {}", file.display()))?;

    // add uuid as prefix to ensure isolation
    communication.zenoh_prefix += &format!("/{}", uuid::Uuid::new_v4());

    let mut tasks = FuturesUnordered::new();
    for operator in operators {
        let operator_id = operator.config.id.clone();
        let result = spawn_operator(operator, &communication)
            .wrap_err_with(|| format!("failed to spawn operator {operator_id}"))?;
        tasks.push(result);
    }

    while let Some(task_result) = tasks.next().await {
        task_result
            .wrap_err("failed to join async task")?
            .wrap_err("operator failed")?;
    }

    Ok(())
}

fn spawn_operator(
    operator: descriptor::Operator,
    communication: &dora_api::config::CommunicationConfig,
) -> eyre::Result<tokio::task::JoinHandle<eyre::Result<(), eyre::Error>>> {
    let mut args = operator.run.split_ascii_whitespace();
    let cmd = args
        .next()
        .ok_or_else(|| eyre!("`run` field must not be empty"))?;
    let mut command = tokio::process::Command::new(cmd);
    command.args(args);
    command.env(
        "OPERATOR_CONFIG",
        serde_yaml::to_string(&operator.config).wrap_err("failed to serialize operator config")?,
    );
    command.env(
        "COMMUNICATION_CONFIG",
        serde_yaml::to_string(communication)
            .wrap_err("failed to serialize communication config")?,
    );
    let mut child = command
        .spawn()
        .wrap_err_with(|| format!("failed to run command `{}`", &operator.run))?;
    let operator_id = operator.config.id.clone();
    let result = tokio::spawn(async move {
        let status = child.wait().await.context("child process failed")?;
        if status.success() {
            println!("operator {operator_id} finished");
            Ok(())
        } else if let Some(code) = status.code() {
            Err(eyre!(
                "operator {operator_id} failed with exit code: {code}"
            ))
        } else {
            Err(eyre!("operator {operator_id} failed (unknown exit code)"))
        }
    });
    Ok(result)
}

async fn read_descriptor(file: &Path) -> Result<Descriptor, eyre::Error> {
    let descriptor_file = tokio::fs::read(file)
        .await
        .context("failed to open given file")?;
    let descriptor: Descriptor =
        serde_yaml::from_slice(&descriptor_file).context("failed to parse given descriptor")?;
    Ok(descriptor)
}
