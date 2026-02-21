use adora_cli::Args;
use clap::Parser;

fn main() {
    let args = Args::parse();
    adora_cli::lib_main(args);
}
