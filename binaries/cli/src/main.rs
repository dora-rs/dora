use clap::Parser;
use dora_cli::Args;

fn main() -> () {
    let args = Args::parse();
    dora_cli::lib_main(args);
}
