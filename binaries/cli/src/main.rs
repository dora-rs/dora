use clap::Parser;

fn main() {
    let mut args: Vec<String> = std::env::args().collect();

    if args.len() == 2 && matches!(args[1].as_str(), "-h" | "--help") {
        dora_cli::print_legacy_help_with_hint();
        return;
    }

    let next_positional_index = args
        .iter()
        .enumerate()
        .skip(1)
        .find(|(_, arg)| !arg.starts_with('-'))
        .map(|(idx, arg)| (idx, arg.as_str()));

    let next_positional = next_positional_index.map(|(_, arg)| arg);

    if matches!(next_positional, Some("help")) && args.len() == 2 {
        dora_cli::print_legacy_help_with_hint();
        return;
    }

    let should_use_new_cli = matches!(next_positional, Some("tui") | Some("dashboard"));

    if should_use_new_cli {
        match dora_cli::cli::Cli::try_parse_from(&args) {
            Ok(cli) => {
                dora_cli::run_new_cli(cli);
                return;
            }
            Err(e) => {
                e.exit();
            }
        }
    }

    if let Some((idx, _)) = next_positional_index {
        match args[idx].as_str() {
            "ps" => args[idx] = "list".to_string(),
            _ => {}
        }
    }

    dora_cli::lib_main(dora_cli::Args::parse_from(args));
}
