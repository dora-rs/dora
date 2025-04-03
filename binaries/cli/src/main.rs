use clap::Parser;
use dora_cli::Args;
use dora_daemon::DoraCommand;

fn main() {
    let args = Args::parse();
    let dora_command = DoraCommand {
        executable: std::env::current_exe().expect("failed to get current executable"),
        args: Vec::new(),
    };
    dora_cli::lib_main(args, dora_command);
}
