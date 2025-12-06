use dora_node_api::{DoraNode, dora_core::config::DataId};
use eyre::Result;

fn main() -> Result<()> {
    let (mut node, _events) = DoraNode::init_from_env()?;
    let output_id = DataId::from("data".to_string());

    println!("[Producer] Starting...");

    // Simulate some successful operations
    for i in 0..3 {
        println!("[Producer] Sending message {}", i);
        std::thread::sleep(std::time::Duration::from_secs(1));
    }

    // Simulate an error condition
    println!("[Producer] Encountered an error! Sending error event instead of crashing...");
    node.send_error(
        output_id.clone(),
        "Simulated processing error: invalid data format"
    )?;

    // Continue running (demonstrating graceful degradation)
    println!("[Producer] Continuing after error...");
    std::thread::sleep(std::time::Duration::from_secs(2));

    println!("[Producer] Exiting gracefully");
    Ok(())
}

