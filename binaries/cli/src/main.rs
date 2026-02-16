use clap::Parser;
use adora_cli::Args;

fn main() {
    let args = Args::parse();
    adora_cli::lib_main(args);
}
