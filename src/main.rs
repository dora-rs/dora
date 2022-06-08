use eyre::Context;
use structopt::StructOpt;

#[derive(Debug, Clone, StructOpt)]
#[structopt(about = "Dora control")]
enum Command {
    #[structopt(about = "Run Python server")]
    StartPython(dora_rs::python::server::PythonCommand),
}

#[tokio::main]
async fn main() -> eyre::Result<()> {
    env_logger::init();

    let command = Command::from_args();
    match command {
        Command::StartPython(command) => {
            dora_rs::python::server::run(command).context("python server failed")?;
        }
    }

    Ok(())
}
