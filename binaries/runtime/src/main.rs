use clap::Parser;

/// Show author, version and about.
#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {}

fn main() -> Result<(), eyre::Report> {
    Args::parse();
    dora_runtime::main()
}
