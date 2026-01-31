use dora_node_api::DoraNode;
use eyre::Result;

fn main() -> Result<()> {
    let (_node, _events) = DoraNode::init_from_env()?;

    println!("Starting...");

    // Simulate some successful operations
    for i in 0..3 {
        println!("Processing message {}", i);
        std::thread::sleep(std::time::Duration::from_secs(1));
    }

    // Simulate an error condition - exit with non-zero exit code
    // The daemon will automatically propagate this error to downstream nodes
    println!("Encountered an error! Exiting with error code...");
    eprintln!("Error: Simulated error: exiting with exit code 1");
    std::process::exit(1);
}
