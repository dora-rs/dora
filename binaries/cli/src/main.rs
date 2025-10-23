use clap::Parser;

fn main() {
    // Use new hybrid CLI - demonstrates Issue #1 implementation
    match dora_cli::cli::Cli::try_parse() {
        Ok(cli) => {
            // Use new hybrid CLI with three-tier structure and global flags
            dora_cli::hybrid_main(cli);
        }
        Err(e) => {
            // Show parsing error and fall back to legacy for now
            eprintln!("Note: Using legacy CLI (new hybrid CLI parsing failed)");
            eprintln!("Error: {}", e);
            eprintln!();
            
            // Fallback to legacy CLI for backward compatibility
            let args = dora_cli::Args::parse();
            dora_cli::lib_main(args);
        }
    }
}
